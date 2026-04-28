#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""YOLOE 动态目标跟踪 API。

该文件只负责模型加载、动态 prompt 设置和单帧 tracking 推理。
图像来源、label 来源、ROS 可视化发布等测试逻辑放在 test.py。
"""

from __future__ import annotations

import argparse
import base64
import sys
import threading
import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import torch
import uvicorn
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel, Field


YOLOE_ROOT = Path(__file__).resolve().parents[1]
if str(YOLOE_ROOT) not in sys.path:
    sys.path.insert(0, str(YOLOE_ROOT))

from ultralytics import YOLOE  # noqa: E402
from ultralytics.models.yolo.yoloe.predict_vp import YOLOEVPSegPredictor  # noqa: E402


def _now() -> float:
    return time.time()


def _decode_image_base64(image_base64: str) -> np.ndarray:
    """将 base64 编码图像解码为 RGB OpenCV ndarray。"""
    raw = base64.b64decode(image_base64)
    np_buf = np.frombuffer(raw, dtype=np.uint8)
    image_bgr = cv2.imdecode(np_buf, cv2.IMREAD_COLOR)
    if image_bgr is None:
        raise ValueError("cv2.imdecode returned None")
    return cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)


def _clip_bbox(bbox: list[float], image: np.ndarray) -> list[int] | None:
    """将 xyxy bbox 裁剪到图像范围内。"""
    h, w = image.shape[:2]
    x1, y1, x2, y2 = [float(v) for v in bbox[:4]]
    x1 = min(max(x1, 0.0), float(w - 1))
    y1 = min(max(y1, 0.0), float(h - 1))
    x2 = min(max(x2, 0.0), float(w - 1))
    y2 = min(max(y2, 0.0), float(h - 1))
    if x2 <= x1 or y2 <= y1:
        return None
    return [int(round(x1)), int(round(y1)), int(round(x2)), int(round(y2))]


def _bbox_iou(box_a: list[int] | list[float] | None, box_b: list[int] | list[float] | None) -> float:
    """计算两个 xyxy bbox 的 IoU，用于首帧绑定和丢失后重绑定。"""
    if box_a is None or box_b is None:
        return 0.0
    ax1, ay1, ax2, ay2 = [float(v) for v in box_a]
    bx1, by1, bx2, by2 = [float(v) for v in box_b]
    inter_x1 = max(ax1, bx1)
    inter_y1 = max(ay1, by1)
    inter_x2 = min(ax2, bx2)
    inter_y2 = min(ay2, by2)
    inter_w = max(0.0, inter_x2 - inter_x1)
    inter_h = max(0.0, inter_y2 - inter_y1)
    inter = inter_w * inter_h
    if inter <= 0.0:
        return 0.0
    area_a = max(1.0, (ax2 - ax1) * (ay2 - ay1))
    area_b = max(1.0, (bx2 - bx1) * (by2 - by1))
    return float(inter / max(1.0, area_a + area_b - inter))


class TrackRequest(BaseModel):
    """单帧 tracking 请求。

    label 是外部系统本次要跟踪的文本目标；API 内部会在 label 或 prompt 变化时自动切换 YOLOE 类别空间并重置 tracker。
    """

    image_base64: str = Field(..., description="JPEG/PNG 图像的 base64 字符串")
    label: str = Field(..., min_length=1, description="YOLOE 文本提示，例如 person 或 red box")
    stamp: float | None = Field(None, description="外部图像时间戳")
    init_bbox: list[float] | None = Field(None, description="可选 xyxy 初始框，多同类目标时建议提供")
    reset: bool = Field(False, description="强制重置当前 tracker")
    tracker: str = Field("botsort", description="botsort 或 bytetrack")
    prompt_mode: str = Field("text", description="text / visual / hybrid")
    update_visual_prompt: bool = Field(False, description="是否使用当前 init_bbox 更新 visual prompt embedding")
    visual_prompt_reset_tracker: bool = Field(True, description="visual prompt 更新后是否重置底层 tracker")
    conf: float | None = Field(None, ge=0.0, le=1.0)
    iou: float | None = Field(None, ge=0.0, le=1.0)
    imgsz: int | None = Field(None, ge=32)


class ResetRequest(BaseModel):
    """重置 tracker 状态。"""

    reason: str = ""


class YoloeTrackEngine:
    """YOLOE 单目标跟踪引擎。"""

    def __init__(
        self,
        *,
        model_path: str,
        tracker_dir: str,
        device: str,
        conf: float,
        iou: float,
        imgsz: int,
        rebind_iou_threshold: float,
        rebind_score_threshold: float,
    ) -> None:
        self.model_path = str(model_path)
        self.tracker_dir = Path(tracker_dir)
        self.device = str(device)
        self.default_conf = float(conf)
        self.default_iou = float(iou)
        self.default_imgsz = int(imgsz)
        self.rebind_iou_threshold = float(rebind_iou_threshold)
        self.rebind_score_threshold = float(rebind_score_threshold)

        self.lock = threading.Lock()
        self.model = self._load_model()

        self.current_label = ""
        self.current_tracker = "botsort"
        self.current_prompt_mode = "text"
        self.current_prompt_source = "text"
        self.target_id: int | None = None
        self.last_bbox: list[int] | None = None
        self.last_score = 0.0
        self.state = "idle"
        self.reason = "not_started"
        self.frame_seq = 0
        self.latest_result = self._make_result(ok=False, reason=self.reason)

    def _load_model(self):
        if self.device.startswith("cuda") and not torch.cuda.is_available():
            raise RuntimeError("CUDA is requested but torch.cuda.is_available() is False")
        model = YOLOE(self.model_path)
        model.to(self.device)
        model.eval()
        return model

    def _tracker_cfg_path(self, tracker: str) -> str:
        tracker = str(tracker or "botsort").strip().lower()
        if tracker not in {"botsort", "bytetrack"}:
            raise ValueError(f"unsupported tracker: {tracker}")
        cfg = self.tracker_dir / f"{tracker}.yaml"
        if not cfg.exists():
            raise FileNotFoundError(f"tracker config not found: {cfg}")
        return str(cfg)

    def _reset_ultralytics_trackers(self) -> None:
        predictor = getattr(self.model, "predictor", None)
        if predictor is None or not hasattr(predictor, "trackers"):
            return
        for tracker in predictor.trackers or []:
            if hasattr(tracker, "reset"):
                tracker.reset()

    def reset(self, reason: str = "") -> dict[str, Any]:
        with self.lock:
            self._reset_ultralytics_trackers()
            self.current_label = ""
            self.current_prompt_mode = "text"
            self.current_prompt_source = "text"
            self.target_id = None
            self.last_bbox = None
            self.last_score = 0.0
            self.state = "idle"
            self.reason = reason or "reset"
            self.latest_result = self._make_result(ok=False, reason=self.reason)
            return dict(self.latest_result)

    def status(self) -> dict[str, Any]:
        with self.lock:
            return {
                "state": self.state,
                "label": self.current_label,
                "tracker": self.current_tracker,
                "prompt_mode": self.current_prompt_mode,
                "prompt_source": self.current_prompt_source,
                "track_id": self.target_id,
                "last_bbox": self.last_bbox,
                "last_score": self.last_score,
                "reason": self.reason,
                "frame_seq": self.frame_seq,
            }

    def latest(self) -> dict[str, Any]:
        with self.lock:
            return dict(self.latest_result)

    def track(self, req: TrackRequest) -> dict[str, Any]:
        image_rgb = _decode_image_base64(req.image_base64)
        stamp = float(req.stamp if req.stamp is not None else _now())
        label = req.label.strip()
        if not label:
            raise ValueError("label is required")

        with self.lock:
            tracker = req.tracker.strip().lower()
            prompt_mode = self._normalize_prompt_mode(req.prompt_mode)
            label_changed = label != self.current_label
            tracker_changed = tracker != self.current_tracker
            prompt_changed = prompt_mode != self.current_prompt_mode
            init_bbox = _clip_bbox(req.init_bbox, image_rgb) if req.init_bbox is not None else None
            if req.reset or label_changed or tracker_changed or prompt_changed:
                self._set_text_prompt(label)
                self.current_label = label
                self.current_tracker = tracker
                self.current_prompt_mode = prompt_mode
                self.current_prompt_source = "text"
                self.target_id = None
                self.last_bbox = None
                self.last_score = 0.0
                self.state = "acquiring"
                self.reason = "reset" if req.reset else "new_target"
                self._reset_ultralytics_trackers()

            self.frame_seq += 1
            frame_seq = self.frame_seq
            conf = float(req.conf if req.conf is not None else self.default_conf)
            iou = float(req.iou if req.iou is not None else self.default_iou)
            imgsz = int(req.imgsz if req.imgsz is not None else self.default_imgsz)
            tracker_cfg = self._tracker_cfg_path(self.current_tracker)

            if prompt_mode in {"visual", "hybrid"} and req.update_visual_prompt:
                if init_bbox is None:
                    if prompt_mode == "visual":
                        return self._mark_lost(stamp=stamp, frame_seq=frame_seq, reason="visual_prompt_requires_bbox")
                else:
                    self._set_visual_prompt(image_rgb, label, init_bbox, imgsz=imgsz)
                    self.current_prompt_mode = prompt_mode
                    self.current_prompt_source = "visual"
                    if req.visual_prompt_reset_tracker:
                        self._reset_ultralytics_trackers()
                        self.target_id = None

            results = self.model.track(
                source=image_rgb,
                persist=True,
                tracker=tracker_cfg,
                conf=conf,
                iou=iou,
                imgsz=imgsz,
                device=self.device,
                verbose=False,
            )

            result = results[0] if results else None
            candidates = self._extract_candidates(result, image_rgb)
            if not candidates:
                return self._mark_lost(stamp=stamp, frame_seq=frame_seq, reason="no_candidates")

            selected = self._select_target(candidates, init_bbox=init_bbox)
            if selected is None:
                return self._mark_lost(stamp=stamp, frame_seq=frame_seq, reason="target_missing")

            self.target_id = int(selected["track_id"])
            self.last_bbox = list(selected["bbox"])
            self.last_score = float(selected["score"])
            self.state = "active"
            self.reason = ""
            self.latest_result = self._make_result(
                ok=True,
                stamp=stamp,
                frame_seq=frame_seq,
                bbox=self.last_bbox,
                track_id=self.target_id,
                score=self.last_score,
                cls=selected["cls"],
                has_mask=selected["has_mask"],
                reason="",
            )
            return dict(self.latest_result)

    def _normalize_prompt_mode(self, prompt_mode: str) -> str:
        """规整 prompt 模式，避免请求侧大小写或空字符串影响运行。"""
        mode = str(prompt_mode or "text").strip().lower()
        if mode not in {"text", "visual", "hybrid"}:
            raise ValueError(f"unsupported prompt_mode: {prompt_mode}")
        return mode

    def _set_text_prompt(self, label: str) -> None:
        """使用文本 prompt 设置单类检测空间。"""
        self.model.set_classes([label], self.model.get_text_pe([label]))
        self.model.predictor = None

    def _set_visual_prompt(self, image_rgb: np.ndarray, label: str, bbox: list[int], *, imgsz: int) -> None:
        """使用慢模型 bbox 生成 visual prompt embedding，并切换为单目标视觉 prompt 检测空间。"""
        prompts = {
            "bboxes": np.asarray([bbox], dtype=np.float32),
            "cls": np.asarray([0], dtype=np.int64),
        }
        self.model.predict(
            [image_rgb],
            prompts=prompts,
            predictor=YOLOEVPSegPredictor,
            return_vpe=True,
            imgsz=int(imgsz),
            device=self.device,
            verbose=False,
        )
        vpe = self.model.predictor.vpe
        self.model.set_classes([label], vpe)
        self.model.predictor = None

    def _extract_candidates(self, result, image_rgb: np.ndarray) -> list[dict[str, Any]]:
        if result is None or result.boxes is None or result.boxes.id is None:
            return []
        boxes = result.boxes
        ids = boxes.id.int().cpu().tolist()
        xyxy = boxes.xyxy.cpu().tolist()
        confs = boxes.conf.cpu().tolist() if boxes.conf is not None else [1.0] * len(ids)
        classes = boxes.cls.int().cpu().tolist() if boxes.cls is not None else [0] * len(ids)
        has_masks = result.masks is not None and getattr(result.masks, "data", None) is not None

        candidates = []
        for track_id, bbox, score, cls in zip(ids, xyxy, confs, classes):
            clipped = _clip_bbox(bbox, image_rgb)
            if clipped is None:
                continue
            candidates.append(
                {
                    "track_id": int(track_id),
                    "bbox": clipped,
                    "score": float(score),
                    "cls": int(cls),
                    "has_mask": bool(has_masks),
                }
            )
        return candidates

    def _select_target(self, candidates: list[dict[str, Any]], init_bbox: list[float] | None) -> dict[str, Any] | None:
        # 上层传入 init_bbox 时，说明 VLM/慢模型正在重新确认目标；
        # 此时优先用 bbox 在当前 YOLOE 轨迹中重新绑定 id，而不是盲目沿用旧 id。
        if init_bbox is None and self.target_id is not None:
            for candidate in candidates:
                if int(candidate["track_id"]) == int(self.target_id):
                    return candidate

        hint_bbox = init_bbox or self.last_bbox
        if hint_bbox is None:
            return max(candidates, key=lambda item: item["score"])

        best = None
        best_score = -1.0
        best_iou = -1.0
        for candidate in candidates:
            overlap = _bbox_iou(candidate["bbox"], hint_bbox)
            score = 0.75 * overlap + 0.25 * float(candidate["score"])
            if score > best_score:
                best = candidate
                best_score = score
                best_iou = overlap

        if best is None:
            return None
        if best_iou >= self.rebind_iou_threshold:
            return best
        if self.target_id is None and float(best["score"]) >= self.rebind_score_threshold:
            return best
        if self.target_id is not None:
            for candidate in candidates:
                if int(candidate["track_id"]) == int(self.target_id):
                    return candidate
        return None

    def _mark_lost(self, *, stamp: float, frame_seq: int, reason: str) -> dict[str, Any]:
        self.state = "lost" if self.target_id is not None else "acquiring"
        self.reason = reason
        self.latest_result = self._make_result(
            ok=False,
            stamp=stamp,
            frame_seq=frame_seq,
            bbox=self.last_bbox,
            track_id=self.target_id,
            score=self.last_score,
            reason=reason,
        )
        return dict(self.latest_result)

    def _make_result(
        self,
        *,
        ok: bool,
        stamp: float = 0.0,
        frame_seq: int | None = None,
        bbox: list[int] | None = None,
        track_id: int | None = None,
        score: float = 0.0,
        cls: int | None = None,
        has_mask: bool = False,
        reason: str = "",
    ) -> dict[str, Any]:
        return {
            "ok": bool(ok),
            "state": self.state,
            "label": self.current_label,
            "tracker": self.current_tracker,
            "prompt_mode": self.current_prompt_mode,
            "prompt_source": self.current_prompt_source,
            "track_id": track_id,
            "bbox": bbox,
            "score": float(score),
            "cls": cls,
            "has_mask": bool(has_mask),
            "stamp": float(stamp),
            "frame_seq": frame_seq,
            "source": f"yoloe:{self.current_tracker}",
            "reason": str(reason or ""),
            "wall_time": _now(),
        }


def create_app(engine: YoloeTrackEngine) -> FastAPI:
    app = FastAPI(title="YOLOE Track Engine")

    @app.post("/track")
    def track(req: TrackRequest):
        try:
            return engine.track(req)
        except Exception as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc

    @app.post("/reset")
    def reset(req: ResetRequest | None = None):
        return engine.reset(reason="" if req is None else req.reason)

    @app.get("/status")
    def status():
        return engine.status()

    @app.get("/latest")
    def latest():
        return engine.latest()

    return app


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="YOLOE track engine API")
    parser.add_argument("--model", default=str(YOLOE_ROOT / "yoloe-v8m-seg.pt"))
    parser.add_argument("--tracker-dir", default=str(YOLOE_ROOT / "ultralytics" / "cfg" / "trackers"))
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--conf", type=float, default=0.1)
    parser.add_argument("--iou", type=float, default=0.5)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--rebind-iou-threshold", type=float, default=0.05)
    parser.add_argument("--rebind-score-threshold", type=float, default=0.15)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2250)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    engine = YoloeTrackEngine(
        model_path=args.model,
        tracker_dir=args.tracker_dir,
        device=args.device,
        conf=args.conf,
        iou=args.iou,
        imgsz=args.imgsz,
        rebind_iou_threshold=args.rebind_iou_threshold,
        rebind_score_threshold=args.rebind_score_threshold,
    )
    app = create_app(engine)
    uvicorn.run(app, host=args.host, port=args.port, reload=False, workers=1)


if __name__ == "__main__":
    main()

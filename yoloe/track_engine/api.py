#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""YOLOE 单目标跟踪 API（基于官方 model.track() 方案）。

核心变更（v2）：
- 使用官方 model.track(source=frame, persist=True) 替代手写的 _select_target / _extract_candidates
- 跟踪器内部自带 Kalman 预测 + 两阶段匹配 + 运动补偿，无需外围再实现一套
- VLM init_bbox 仅用于首帧匹配目标 track_id，后续帧由跟踪器自动维持
- 切换 label 时 set_classes + predictor=None 强制干净重启
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

# cuDNN 加速保持关闭，手动修改数据推理精度
torch.backends.cudnn.enabled = False
torch.backends.cuda.matmul.allow_tf32 = True

from ultralytics import YOLOE  # noqa: E402


def _now() -> float:
    return time.time()


def _ms(seconds: float) -> float:
    return round(float(seconds) * 1000.0, 3)


def _decode_image_base64(image_base64: str) -> np.ndarray:
    """将 base64 编码图像解码为 OpenCV BGR ndarray。"""
    raw = base64.b64decode(image_base64)
    np_buf = np.frombuffer(raw, dtype=np.uint8)
    image_bgr = cv2.imdecode(np_buf, cv2.IMREAD_COLOR)
    if image_bgr is None:
        raise ValueError("cv2.imdecode returned None")
    return image_bgr


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


def _bbox_iou(box_a, box_b) -> float:
    """计算两个 xyxy bbox 的 IoU。"""
    if box_a is None or box_b is None:
        return 0.0
    ax1, ay1, ax2, ay2 = [float(v) for v in box_a[:4]]
    bx1, by1, bx2, by2 = [float(v) for v in box_b[:4]]
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


# ──────────────────────────────────────────────
# Request / Response
# ──────────────────────────────────────────────

class TrackRequest(BaseModel):
    """单帧 tracking 请求。

    init_bbox 仅用于首帧（reset=true 时）匹配目标 track_id：
    从 YOLOE + 跟踪器输出中找到与 VLM bbox IoU 最高的轨迹作为目标，
    之后该目标由跟踪器自行维持，不再需要 init_bbox。
    当目标丢失需要重新捕获时，再次发送 init_bbox 即可重绑。

    （以下参数保留以兼容旧客户端，但不再生效：strict_identity, allow_rebind, lost_rebind,
     prompt_mode, update_visual_prompt, visual_prompt_reset_tracker）
    """

    image_base64: str = Field(..., description="JPEG/PNG 图像的 base64 字符串")
    label: str = Field(..., min_length=1, description="YOLOE 文本提示，例如 red box")
    stamp: float | None = Field(None, description="外部图像时间戳")
    init_bbox: list[float] | None = Field(None, description="VLM 给出的初始 xyxy 框，用于匹配目标 track_id")
    reset: bool = Field(False, description="强制重置当前 tracking 会话并重新匹配 init_bbox")
    tracker: str = Field("deepocsort", description="tracker 类型：deepocsort / botsort / bytetrack / ocsort")
    conf: float | None = Field(None, ge=0.0, le=1.0)
    iou: float | None = Field(None, ge=0.0, le=1.0)
    imgsz: int | None = Field(None, ge=32)

    # ── 以下参数保留以兼容旧客户端，不再生效 ──
    strict_identity: bool = Field(True, description="[已废弃] 跟踪器自行维持 ID")
    allow_rebind: bool = Field(False, description="[已废弃] 由 init_bbox 触发重绑")
    lost_rebind: bool = Field(False, description="[已废弃] 由 init_bbox 触发重绑")
    prompt_mode: str = Field("text", description="[已废弃] YOLOE 目前仅支持 text 模式")
    update_visual_prompt: bool = Field(False, description="[已废弃]")
    visual_prompt_reset_tracker: bool = Field(True, description="[已废弃]")


class ResetRequest(BaseModel):
    reason: str = ""


# ──────────────────────────────────────────────
# Engine
# ──────────────────────────────────────────────

class YoloeTrackEngine:
    """YOLOE 单目标跟踪引擎（基于官方 model.track()）。"""

    def __init__(
        self,
        *,
        model_path: str,
        tracker_dir: str,
        device: str,
        conf: float,
        iou: float,
        imgsz: int,
        init_bbox_match_iou: float = 0.1,
    ) -> None:
        self.model_path = str(model_path)
        self.tracker_dir = Path(tracker_dir)
        self.device = str(device)
        self.default_conf = float(conf)
        self.default_iou = float(iou)
        self.default_imgsz = int(imgsz)
        self.init_bbox_match_iou = float(init_bbox_match_iou)

        self.lock = threading.Lock()
        self.model = self._load_model()

        # tracking 状态
        self.current_label: str = ""
        self.current_tracker: str = "botsort"
        self.target_track_id: int | None = None
        self.last_bbox: list[int] | None = None
        self.last_score: float = 0.0
        self.state: str = "idle"
        self.frame_seq: int = 0
        self.latest_result: dict[str, Any] = {}

    # ── 模型加载 ──

    def _load_model(self):
        if self.device.startswith("cuda") and not torch.cuda.is_available():
            raise RuntimeError("CUDA is requested but torch.cuda.is_available() is False")
        model = YOLOE(self.model_path)
        model.to(self.device)
        model.eval()
        return model

    def _tracker_cfg_path(self, tracker: str) -> str:
        from ultralytics.trackers.track import TRACKER_MAP  # noqa: E402

        tracker = str(tracker).strip().lower() or "botsort"
        if tracker not in TRACKER_MAP:
            raise ValueError(f"unsupported tracker: {tracker}")
        cfg = self.tracker_dir / f"{tracker}.yaml"
        if not cfg.exists():
            raise FileNotFoundError(f"tracker config not found: {cfg}")
        return str(cfg)

    # ── 核心 track ──

    def track(self, req: TrackRequest) -> dict[str, Any]:
        total_t0 = time.perf_counter()
        timings: dict[str, float] = {}

        t0 = time.perf_counter()
        image_bgr = _decode_image_base64(req.image_base64)
        timings["decode_ms"] = _ms(time.perf_counter() - t0)

        label = req.label.strip()
        if not label:
            raise ValueError("label is required")
        stamp = float(req.stamp if req.stamp is not None else _now())

        lock_t0 = time.perf_counter()
        with self.lock:
            timings["lock_wait_ms"] = _ms(time.perf_counter() - lock_t0)

            tracker_name = req.tracker.strip().lower() or "botsort"
            tracker_cfg = self._tracker_cfg_path(tracker_name)
            conf = float(req.conf if req.conf is not None else self.default_conf)
            iou_val = float(req.iou if req.iou is not None else self.default_iou)
            imgsz = int(req.imgsz if req.imgsz is not None else self.default_imgsz)

            label_changed = (label != self.current_label) or (tracker_name != self.current_tracker)

            # ── 重置：新 label 或显式 reset ──
            if req.reset or label_changed:
                timings.update(self._reset_for_label(label, tracker_cfg))

            self.current_label = label
            self.current_tracker = tracker_name
            self.frame_seq += 1

            # ═══════════════════════════════════════════
            # 核心：使用官方 model.track() 单帧跟踪
            # ═══════════════════════════════════════════
            infer_t0 = time.perf_counter()
            try:
                results = self.model.track(
                    source=image_bgr,
                    persist=True,
                    conf=conf,
                    iou=iou_val,
                    imgsz=imgsz,
                    tracker=tracker_cfg,
                    device=self.device,
                    verbose=False,
                )
            except Exception:
                # track() 首次调用的内部注册失败时，回退到不带 tracker 的 predict，
                # 下一帧会自动注册
                results = self.model.predict(
                    source=image_bgr,
                    conf=conf,
                    iou=iou_val,
                    imgsz=imgsz,
                    device=self.device,
                    verbose=False,
                )
            infer_ms = _ms(time.perf_counter() - infer_t0)
            timings["model_track_ms"] = infer_ms

            result = results[0] if results else None
            boxes = result.boxes if result is not None else None
            speed = getattr(result, "speed", None) or {}

            # 提取 tracker 内部耗时信息
            timings["yoloe_preprocess_ms"] = float(speed.get("preprocess", 0.0))
            timings["yoloe_inference_ms"] = float(speed.get("inference", 0.0))
            timings["yoloe_postprocess_ms"] = float(speed.get("postprocess", 0.0))
            timings["total_ms"] = _ms(time.perf_counter() - total_t0)

            # ── 无检测 ──
            if boxes is None or len(boxes) == 0:
                timings["candidate_count"] = 0.0
                self.latest_result = self._lost(stamp, "no_detections", timings)
                self._log_frame(label, "track", timings)
                return dict(self.latest_result)

            det_count = len(boxes)
            timings["candidate_count"] = float(det_count)

            # ── VLM init_bbox → 匹配目标 track_id ──
            if req.init_bbox is not None:
                clipped = _clip_bbox(req.init_bbox, image_bgr)
                if clipped is not None:
                    self.target_track_id = self._identify_target(boxes, clipped)
                    if self.target_track_id is not None:
                        self.state = "active"
                        print(
                            f"[YOLOE_TRACK] init_bbox matched track_id={self.target_track_id} "
                            f"label={label!r}",
                            flush=True,
                        )
                    else:
                        self.latest_result = self._lost(
                            stamp, "init_bbox_no_match", timings
                        )
                        self._log_frame(label, "track", timings)
                        return dict(self.latest_result)
                else:
                    self.latest_result = self._lost(
                        stamp, "init_bbox_out_of_bounds", timings
                    )
                    self._log_frame(label, "track", timings)
                    return dict(self.latest_result)

            # ── 按 target_track_id 筛选 ──
            if self.target_track_id is None:
                # 尚未通过 init_bbox 确认目标，返回无目标
                self.latest_result = self._lost(stamp, "awaiting_init_bbox", timings)
                self._log_frame(label, "track", timings)
                return dict(self.latest_result)

            target = self._find_target_by_id(boxes)
            if target is None:
                self.latest_result = self._lost(stamp, "target_missing", timings)
                self._log_frame(label, "track", timings)
                return dict(self.latest_result)

            # ── 目标已锁定 ──
            self.last_bbox = target["bbox"]
            self.last_score = target["score"]
            self.state = "active"

            self.latest_result = self._make_response(
                ok=True,
                stamp=stamp,
                bbox=self.last_bbox,
                track_id=self.target_track_id,
                score=self.last_score,
                reason="",
                timings=timings,
            )
            self._log_frame(label, "track", timings)
            return dict(self.latest_result)

    # ── 重置 ──

    def _reset_for_label(self, label: str, tracker_cfg: str) -> dict[str, float]:
        """为新 label 重置 YOLOE 类空间和跟踪器状态。"""
        timings: dict[str, float] = {}
        t0 = time.perf_counter()
        self.model.set_classes([label])
        timings["set_classes_ms"] = _ms(time.perf_counter() - t0)

        # 清除 predictor，下次 track() 自动重建（包括 tracker 注册）
        t0 = time.perf_counter()
        self.model.predictor = None
        timings["predictor_reset_ms"] = _ms(time.perf_counter() - t0)

        self.target_track_id = None
        self.last_bbox = None
        self.last_score = 0.0
        self.state = "acquiring"
        self.frame_seq = 0

        print(
            f"[YOLOE_TRACK] reset label={label!r} tracker={Path(tracker_cfg).stem}",
            flush=True,
        )
        return timings

    # ── 目标识别 ──

    def _identify_target(self, boxes, init_bbox: list[int]) -> int | None:
        """从跟踪器输出中找到与 VLM init_bbox IoU 最高的 track_id。"""
        if boxes.id is None:
            return None
        track_ids = boxes.id.int().cpu().tolist()
        best_iou = 0.0
        best_id = None
        for i, tid in enumerate(track_ids):
            bbox = boxes.xyxy[i].cpu().tolist()
            iou = _bbox_iou(bbox, init_bbox)
            if iou > best_iou:
                best_iou = iou
                best_id = int(tid)
        if best_id is not None and best_iou >= self.init_bbox_match_iou:
            return best_id
        return None

    def _find_target_by_id(self, boxes) -> dict[str, Any] | None:
        """在跟踪器输出中查找 target_track_id。"""
        if self.target_track_id is None or boxes.id is None:
            return None
        track_ids = boxes.id.int().cpu().tolist()
        try:
            idx = track_ids.index(self.target_track_id)
        except ValueError:
            return None
        bbox = boxes.xyxy[idx].cpu().tolist()
        confs = boxes.conf
        score = float(confs[idx]) if confs is not None and len(confs) > idx else 1.0
        return {"bbox": [int(round(v)) for v in bbox[:4]], "score": score}

    # ── 响应构造 ──

    def _make_response(
        self,
        *,
        ok: bool,
        stamp: float = 0.0,
        bbox: list[int] | None = None,
        track_id: int | None = None,
        score: float = 0.0,
        reason: str = "",
        timings: dict[str, float] | None = None,
    ) -> dict[str, Any]:
        return {
            "ok": bool(ok),
            "state": self.state,
            "label": self.current_label,
            "tracker": self.current_tracker,
            "track_id": track_id,
            "bbox": bbox,
            "score": float(score),
            "stamp": float(stamp),
            "frame_seq": int(self.frame_seq),
            "source": f"yoloe:{self.current_tracker}",
            "reason": str(reason or ""),
            "timings": dict(timings or {}),
            "wall_time": _now(),
        }

    def _lost(
        self,
        stamp: float,
        reason: str,
        timings: dict[str, float] | None = None,
    ) -> dict[str, Any]:
        self.state = "lost"
        return self._make_response(
            ok=False,
            stamp=stamp,
            bbox=self.last_bbox,
            track_id=self.target_track_id,
            score=self.last_score,
            reason=reason,
            timings=timings,
        )

    def _log_frame(self, label: str, mode: str, timings: dict[str, float]) -> None:
        print(
            "[YOLOE_TRACK]",
            f"seq={self.frame_seq}",
            f"mode={mode}",
            f"label={label!r}",
            f"state={self.state}",
            f"track_id={self.target_track_id}",
            f"candidates={int(timings.get('candidate_count', 0.0))}",
            f"track_ms={timings.get('model_track_ms', 0.0)}",
            f"total_ms={timings.get('total_ms', 0.0)}",
            flush=True,
        )

    # ── API 管理 ──

    def reset(self, reason: str = "") -> dict[str, Any]:
        with self.lock:
            self.model.predictor = None
            self.current_label = ""
            self.target_track_id = None
            self.last_bbox = None
            self.last_score = 0.0
            self.state = "idle"
            self.frame_seq = 0
            self.latest_result = self._make_response(
                ok=False, reason=reason or "reset"
            )
            print(f"[YOLOE_TRACK] manual reset reason={reason!r}", flush=True)
            return dict(self.latest_result)

    def status(self) -> dict[str, Any]:
        with self.lock:
            return {
                "state": self.state,
                "label": self.current_label,
                "tracker": self.current_tracker,
                "track_id": self.target_track_id,
                "last_bbox": self.last_bbox,
                "last_score": self.last_score,
                "frame_seq": self.frame_seq,
            }

    def latest(self) -> dict[str, Any]:
        with self.lock:
            return dict(self.latest_result)


# ──────────────────────────────────────────────
# FastAPI app
# ──────────────────────────────────────────────

def create_app(engine: YoloeTrackEngine) -> FastAPI:
    app = FastAPI(title="YOLOE Track Engine v2")

    @app.post("/track")
    def track(req: TrackRequest):
        try:
            return engine.track(req)
        except Exception as exc:
            print(f"YOLOE track API request failed: {exc}", flush=True)
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


# ──────────────────────────────────────────────
# CLI
# ──────────────────────────────────────────────

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="YOLOE Track Engine v2 (official model.track)")
    parser.add_argument("--model", default=str(YOLOE_ROOT / "yoloe-v8m-seg.pt"))
    parser.add_argument("--tracker-dir", default=str(YOLOE_ROOT / "ultralytics" / "cfg" / "trackers"))
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--conf", type=float, default=0.1)
    parser.add_argument("--iou", type=float, default=0.5)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--init-bbox-match-iou", type=float, default=0.1,
                        help="VLM init_bbox 与 tracker 输出匹配的最小 IoU 阈值")
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
        init_bbox_match_iou=args.init_bbox_match_iou,
    )
    app = create_app(engine)
    uvicorn.run(app, host=args.host, port=args.port, reload=False, workers=1)


if __name__ == "__main__":
    main()

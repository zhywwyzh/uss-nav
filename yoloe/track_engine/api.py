#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""YOLOE 动态目标跟踪 API。

该文件只负责模型加载、动态 prompt 设置和单帧 tracking 推理。
图像来源、label 来源、ROS 可视化发布等测试逻辑放在 test.py。
"""

from __future__ import annotations

import argparse
import base64
import gc
import sys
import threading
import time
from functools import partial
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
from ultralytics.trackers.track import on_predict_start  # noqa: E402


def _now() -> float:
    return time.time()


def _ms(seconds: float) -> float:
    """将秒转换为毫秒，保留三位小数便于 JSON 观察。"""
    return round(float(seconds) * 1000.0, 3)


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

    def _is_ultralytics_tracker_callback(self, callback) -> bool:
        """判断 callback 是否为 model.track() 自动注册的 tracker 回调。"""
        func = getattr(callback, "func", callback)
        is_timed_tracker_callback = (
            getattr(func, "__self__", None) is self
            and getattr(func, "__name__", "") == "_on_predict_postprocess_end_with_timing"
        )
        return (
            (
                getattr(func, "__module__", "") == "ultralytics.trackers.track"
                and getattr(func, "__name__", "") in {"on_predict_start", "on_predict_postprocess_end"}
            )
            or is_timed_tracker_callback
        )

    def _clear_ultralytics_tracker_callbacks(self) -> int:
        """移除旧 tracker 回调，避免后续 predict() 仍被旧 tracking 状态污染。"""
        callbacks = getattr(self.model, "callbacks", None)
        if not callbacks:
            return 0

        removed = 0
        for event in ("on_predict_start", "on_predict_postprocess_end"):
            old_items = list(callbacks.get(event, []))
            new_items = [cb for cb in old_items if not self._is_ultralytics_tracker_callback(cb)]
            callbacks[event] = new_items
            removed += len(old_items) - len(new_items)
        return removed

    def _reset_predictor_and_tracker_state(self) -> int:
        """清理 predictor、tracker 实例和 tracker callbacks，确保新 prompt 首帧是纯检测。"""
        if self.model is None:
            return 0
        self._reset_ultralytics_trackers()
        removed_callbacks = self._clear_ultralytics_tracker_callbacks()
        self.model.predictor = None
        return removed_callbacks

    def _cuda_sync(self) -> None:
        """等待 CUDA 队列完成，避免释放和重载模型时仍有异步任务占用显存。"""
        if self.device.startswith("cuda") and torch.cuda.is_available():
            torch.cuda.synchronize()

    def _reload_model_for_prompt_switch(self, label: str) -> dict[str, float]:
        """为新 label 重载整个 YOLOE 模型，彻底丢弃旧 prompt 和 tracker 运行状态。"""
        timings: dict[str, float] = {}
        reload_t0 = time.perf_counter()
        print(f"[YOLOE_RELOAD] start label={label!r}", flush=True)

        t0 = time.perf_counter()
        self._reset_predictor_and_tracker_state()
        timings["reload_clear_state_ms"] = _ms(time.perf_counter() - t0)

        t0 = time.perf_counter()
        old_model = self.model
        self.model = None
        del old_model
        gc.collect()
        timings["reload_gc_ms"] = _ms(time.perf_counter() - t0)

        if self.device.startswith("cuda"):
            t0 = time.perf_counter()
            self._cuda_sync()
            torch.cuda.empty_cache()
            self._cuda_sync()
            timings["reload_cuda_release_ms"] = _ms(time.perf_counter() - t0)

        t0 = time.perf_counter()
        self.model = self._load_model()
        timings["reload_load_model_ms"] = _ms(time.perf_counter() - t0)

        t0 = time.perf_counter()
        self._set_text_prompt(label, force=True)
        timings["reload_set_text_prompt_ms"] = _ms(time.perf_counter() - t0)
        timings["reload_model_ms"] = _ms(time.perf_counter() - reload_t0)
        print(
            "[YOLOE_RELOAD] done",
            f"label={label!r}",
            f"total_ms={timings['reload_model_ms']}",
            flush=True,
        )
        return timings

    def reset(self, reason: str = "") -> dict[str, Any]:
        with self.lock:
            self._reset_predictor_and_tracker_state()

            self.current_label = ""
            self.current_prompt_mode = "text"
            self.current_prompt_source = "text"

            # 关键：清掉你自己维护的文本 prompt 缓存
            self._current_text_prompt = ""

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

    def _register_timed_tracker_callbacks(self, *, persist: bool, timings: dict[str, float]) -> None:
        """注册带计时的 tracker callback，用于拆分 YOLOE 推理和 tracker 更新耗时。"""
        self._clear_ultralytics_tracker_callbacks()
        self.model.add_callback("on_predict_start", partial(on_predict_start, persist=persist))
        self.model.add_callback(
            "on_predict_postprocess_end",
            partial(self._on_predict_postprocess_end_with_timing, persist=persist, timings=timings),
        )

    def _on_predict_postprocess_end_with_timing(
        self,
        predictor: object,
        persist: bool = False,
        timings: dict[str, float] | None = None,
    ) -> None:
        """执行 Ultralytics tracker 后处理，并记录 tracker callback 的耗时。"""
        callback_t0 = time.perf_counter()
        timings = timings if timings is not None else {}
        path, im0s = predictor.batch[:2]

        is_obb = predictor.args.task == "obb"
        is_stream = predictor.dataset.mode == "stream"
        tracker_update_seconds = 0.0
        for i in range(len(im0s)):
            tracker = predictor.trackers[i if is_stream else 0]
            vid_path = predictor.save_dir / Path(path[i]).name
            if not persist and predictor.vid_path[i if is_stream else 0] != vid_path:
                tracker.reset()
                predictor.vid_path[i if is_stream else 0] = vid_path

            det = (predictor.results[i].obb if is_obb else predictor.results[i].boxes).cpu().numpy()
            if len(det) == 0:
                continue

            # 这里是 BoT-SORT / ByteTrack 的核心更新入口，单独计时便于和 YOLOE 推理区分。
            update_t0 = time.perf_counter()
            tracks = tracker.update(det, im0s[i])
            tracker_update_seconds += time.perf_counter() - update_t0
            if len(tracks) == 0:
                continue
            idx = tracks[:, -1].astype(int)
            predictor.results[i] = predictor.results[i][idx]

            update_args = {"obb" if is_obb else "boxes": torch.as_tensor(tracks[:, :-1])}
            predictor.results[i].update(**update_args)

        timings["tracker_update_ms"] = _ms(tracker_update_seconds)
        timings["tracker_callback_ms"] = _ms(time.perf_counter() - callback_t0)

    def _track_with_timing(
        self,
        *,
        source: np.ndarray,
        tracker: str,
        conf: float,
        iou: float,
        imgsz: int,
        timings: dict[str, float],
    ):
        """调用 Ultralytics predict(track mode)，并用自定义 callback 统计 tracker 耗时。"""
        self._register_timed_tracker_callbacks(persist=True, timings=timings)
        return self.model.predict(
            source=source,
            tracker=tracker,
            conf=conf,
            iou=iou,
            imgsz=imgsz,
            device=self.device,
            verbose=False,
            batch=1,
            mode="track",
        )

    def _record_yoloe_speed(self, result, timings: dict[str, float]) -> None:
        """从 Ultralytics Result.speed 中提取 YOLOE 预处理、推理和后处理耗时。"""
        speed = getattr(result, "speed", None) or {}
        if not speed:
            return
        timings["yoloe_preprocess_ms"] = round(float(speed.get("preprocess", 0.0)), 3)
        timings["yoloe_inference_ms"] = round(float(speed.get("inference", 0.0)), 3)
        timings["yoloe_postprocess_ms"] = round(float(speed.get("postprocess", 0.0)), 3)

    def _log_timing(
        self,
        *,
        frame_seq: int,
        label: str,
        mode: str,
        timings: dict[str, float],
    ) -> None:
        """打印单帧关键耗时，便于观察 YOLOE 推理和 tracking 更新开销。"""
        print(
            "[YOLOE_TIMING]",
            f"frame_seq={frame_seq}",
            f"label={label!r}",
            f"mode={mode}",
            f"yoloe_preprocess_ms={timings.get('yoloe_preprocess_ms', 0.0)}",
            f"yoloe_inference_ms={timings.get('yoloe_inference_ms', 0.0)}",
            f"yoloe_postprocess_ms={timings.get('yoloe_postprocess_ms', 0.0)}",
            f"tracker_update_ms={timings.get('tracker_update_ms', 0.0)}",
            f"tracker_callback_ms={timings.get('tracker_callback_ms', 0.0)}",
            f"model_predict_ms={timings.get('model_predict_ms', 0.0)}",
            f"model_track_ms={timings.get('model_track_ms', 0.0)}",
            f"total_ms={timings.get('total_ms', 0.0)}",
            f"candidate_count={int(timings.get('candidate_count', 0.0))}",
            flush=True,
        )

    def track(self, req: TrackRequest) -> dict[str, Any]:
        total_t0 = time.perf_counter()
        timings: dict[str, float] = {}
        t0 = time.perf_counter()
        image_rgb = _decode_image_base64(req.image_base64)
        timings["decode_ms"] = _ms(time.perf_counter() - t0)
        stamp = float(req.stamp if req.stamp is not None else _now())
        label = req.label.strip()
        if not label:
            raise ValueError("label is required")

        lock_t0 = time.perf_counter()
        with self.lock:
            timings["lock_wait_ms"] = _ms(time.perf_counter() - lock_t0)
            tracker = req.tracker.strip().lower()
            prompt_mode = self._normalize_prompt_mode(req.prompt_mode)
            label_changed = label != self.current_label
            tracker_changed = tracker != self.current_tracker
            prompt_changed = prompt_mode != self.current_prompt_mode
            print(
                "[YOLOE_TRACK]",
                "req_label=", repr(label),
                "current_label=", repr(self.current_label),
                "cached_text_prompt=", repr(getattr(self, "_current_text_prompt", None)),
                "label_changed=", label_changed,
                "tracker_changed=", tracker_changed,
                "prompt_changed=", prompt_changed,
                flush=True,
            )
            init_bbox = _clip_bbox(req.init_bbox, image_rgb) if req.init_bbox is not None else None
            new_target_started = False

            if req.reset or label_changed or tracker_changed or prompt_changed:
                t0 = time.perf_counter()

                if req.reset or label_changed:
                    timings.update(self._reload_model_for_prompt_switch(label))
                else:
                    removed_callbacks = self._reset_predictor_and_tracker_state()
                    timings["clear_tracker_callbacks"] = float(removed_callbacks)
                    self._set_text_prompt(label, force=True)
                    timings["set_text_prompt_ms"] = _ms(time.perf_counter() - t0)

                self.current_label = label
                self.current_tracker = tracker
                self.current_prompt_mode = prompt_mode
                self.current_prompt_source = "text"

                self.target_id = None
                self.last_bbox = None
                self.last_score = 0.0

                self.state = "acquiring"
                self.reason = "reset" if req.reset else "new_target"

                # 关键：标记这一帧是新目标首帧
                new_target_started = True

            self.frame_seq += 1
            frame_seq = self.frame_seq
            conf = float(req.conf if req.conf is not None else self.default_conf)
            iou = float(req.iou if req.iou is not None else self.default_iou)
            imgsz = int(req.imgsz if req.imgsz is not None else self.default_imgsz)
            tracker_cfg = self._tracker_cfg_path(self.current_tracker)

            if prompt_mode in {"visual", "hybrid"} and req.update_visual_prompt:
                if init_bbox is None:
                    if prompt_mode == "visual":
                        timings["total_ms"] = _ms(time.perf_counter() - total_t0)
                        return self._mark_lost(
                            stamp=stamp,
                            frame_seq=frame_seq,
                            reason="visual_prompt_requires_bbox",
                            timings=timings,
                        )
                else:
                    t0 = time.perf_counter()
                    self._set_visual_prompt(image_rgb, label, init_bbox, imgsz=imgsz)
                    timings["set_visual_prompt_ms"] = _ms(time.perf_counter() - t0)
                    self.current_prompt_mode = prompt_mode
                    self.current_prompt_source = "visual"
                    if req.visual_prompt_reset_tracker:
                        removed_callbacks = self._reset_predictor_and_tracker_state()
                        timings["clear_visual_tracker_callbacks"] = float(removed_callbacks)
                        self.target_id = None

            t0 = time.perf_counter()
            timing_mode = "track"

            if new_target_started:
                # 关键：新 label 的第一帧只做检测，不做 tracking。
                # label/reset 切换时已重载模型，避免 predict() 仍执行旧 BoT-SORT / ByteTrack 状态。
                timing_mode = "predict"
                results = self.model.predict(
                    source=image_rgb,
                    conf=conf,
                    iou=iou,
                    imgsz=imgsz,
                    device=self.device,
                    verbose=False,
                )
                timings["model_predict_ms"] = _ms(time.perf_counter() - t0)
            else:
                results = self._track_with_timing(
                    source=image_rgb,
                    tracker=tracker_cfg,
                    conf=conf,
                    iou=iou,
                    imgsz=imgsz,
                    timings=timings,
                )
                timings["model_track_ms"] = _ms(time.perf_counter() - t0)

            result = results[0] if results else None
            self._record_yoloe_speed(result, timings)
            t0 = time.perf_counter()
            candidates = self._extract_candidates(result, image_rgb)
            timings["extract_candidates_ms"] = _ms(time.perf_counter() - t0)
            timings["candidate_count"] = float(len(candidates))
            if not candidates:
                timings["total_ms"] = _ms(time.perf_counter() - total_t0)
                self._log_timing(frame_seq=frame_seq, label=label, mode=timing_mode, timings=timings)
                return self._mark_lost(
                    stamp=stamp,
                    frame_seq=frame_seq,
                    reason="no_candidates",
                    timings=timings,
                )

            t0 = time.perf_counter()
            selected = self._select_target(candidates, init_bbox=init_bbox)
            timings["select_target_ms"] = _ms(time.perf_counter() - t0)
            if selected is None:
                timings["total_ms"] = _ms(time.perf_counter() - total_t0)
                self._log_timing(frame_seq=frame_seq, label=label, mode=timing_mode, timings=timings)
                return self._mark_lost(
                    stamp=stamp,
                    frame_seq=frame_seq,
                    reason="target_missing",
                    timings=timings,
                )

            selected_track_id = int(selected["track_id"])

            # 如果这一帧是 predict() 首帧，track_id 是临时负数，不要把它当成真实 tracker id
            if selected_track_id < 0:
                self.target_id = None
            else:
                self.target_id = selected_track_id

            self.last_bbox = list(selected["bbox"])
            self.last_score = float(selected["score"])
            self.state = "active"
            self.reason = ""
            timings["total_ms"] = _ms(time.perf_counter() - total_t0)
            self._log_timing(frame_seq=frame_seq, label=label, mode=timing_mode, timings=timings)
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
                timings=timings,
            )
            return dict(self.latest_result)

    def _normalize_prompt_mode(self, prompt_mode: str) -> str:
        """规整 prompt 模式，避免请求侧大小写或空字符串影响运行。"""
        mode = str(prompt_mode or "text").strip().lower()
        if mode not in {"text", "visual", "hybrid"}:
            raise ValueError(f"unsupported prompt_mode: {prompt_mode}")
        return mode

    def _set_text_prompt(self, label: str, *, force: bool = False) -> None:
        """使用文本 prompt 设置单类检测空间。"""
        label = label.strip()

        if not force and getattr(self, "_current_text_prompt", None) == label:
            return

        # 关键：切换文本 prompt 前，先丢弃旧 predictor
        # 避免 predictor 内部还拿着上一次的 classes / trackers / prompt 状态
        self._reset_predictor_and_tracker_state()

        text_pe = self.model.get_text_pe([label])
        self.model.set_classes([label], text_pe)

        # 关键：set_classes 之后再次丢弃 predictor
        # 确保下一次 model.track() 会用新的类别空间重新构造 predictor
        self._reset_predictor_and_tracker_state()

        self._current_text_prompt = label

    def _set_visual_prompt(self, image_rgb: np.ndarray, label: str, bbox: list[int], *, imgsz: int) -> None:
        """使用慢模型 bbox 生成 visual prompt embedding，并切换为单目标视觉 prompt 检测空间。"""
        prompts = {
            "bboxes": np.asarray([bbox], dtype=np.float32),
            "cls": np.asarray([0], dtype=np.int64),
        }
        # model.track() 会把 predictor 置为普通 SegmentationPredictor；
        # visual prompt 必须强制重建为 YOLOEVPSegPredictor，才能拿到 vpe。
        self._reset_predictor_and_tracker_state()
        self.model.predict(
            [image_rgb],
            prompts=prompts,
            predictor=YOLOEVPSegPredictor,
            return_vpe=True,
            imgsz=int(imgsz),
            device=self.device,
            verbose=False,
        )
        if not hasattr(self.model.predictor, "vpe"):
            raise RuntimeError(f"visual prompt predictor did not produce vpe: {type(self.model.predictor).__name__}")
        vpe = self.model.predictor.vpe
        self.model.set_classes([label], vpe)
        self._reset_predictor_and_tracker_state()
        self._current_text_prompt = ""

    def _extract_candidates(self, result, image_rgb: np.ndarray) -> list[dict[str, Any]]:
        if result is None or result.boxes is None:
            return []

        boxes = result.boxes
        xyxy = boxes.xyxy.cpu().tolist()

        if not xyxy:
            return []

        # track() 有 boxes.id；predict() 没有 boxes.id
        # predict() 首帧时我们临时给候选分配负数 id，后续 track 会重新生成真实 id
        if boxes.id is not None:
            ids = boxes.id.int().cpu().tolist()
        else:
            ids = [-(i + 1) for i in range(len(xyxy))]

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

    def _mark_lost(
        self,
        *,
        stamp: float,
        frame_seq: int,
        reason: str,
        timings: dict[str, float] | None = None,
    ) -> dict[str, Any]:
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
            timings=timings,
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
        timings: dict[str, float] | None = None,
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
            "timings": dict(timings or {}),
            "wall_time": _now(),
        }


def create_app(engine: YoloeTrackEngine) -> FastAPI:
    app = FastAPI(title="YOLOE Track Engine")

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

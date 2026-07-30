#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""YOLOE TensorRT 固定词表目标跟踪 API。

该服务保留 api.py 的 /track /reset /status /latest HTTP 接口，但不再支持运行时
text prompt / visual prompt。服务启动时先用 pt 模型和固定 classes 表导出 engine，
随后只加载 TensorRT engine 做高频检测与 track。
"""

from __future__ import annotations

import argparse
import base64
import gc
import os
import sys
import threading
import time
from dataclasses import dataclass
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

from ultralytics import YOLO  # noqa: E402
from ultralytics.trackers.track import TRACKER_MAP, on_predict_start  # noqa: E402
from ultralytics.utils import IterableSimpleNamespace, yaml_load  # noqa: E402
from ultralytics.utils.checks import check_yaml  # noqa: E402


def _now() -> float:
    return time.time()


def _ms(seconds: float) -> float:
    """将秒转换为毫秒，便于 API 侧直接观察分段耗时。"""
    return round(float(seconds) * 1000.0, 3)


def _uses_cuda_device(device: str) -> bool:
    """判断请求设备是否为 CUDA GPU。"""
    return str(device or "").strip().lower().startswith("cuda")


def _decode_image_base64(image_base64: str) -> np.ndarray:
    """将 base64 编码图像解码为 RGB ndarray。"""
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
    """计算两个 xyxy bbox 的 IoU，用于目标绑定和丢失后重绑定。"""
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


def _normalize_label(label: str) -> str:
    """统一 label 查表格式；输出仍保留 classes 文件中的原始类别名。"""
    return " ".join(str(label or "").strip().lower().split())


def _parse_imgsz(value: str | int | list[int] | tuple[int, ...]) -> int | tuple[int, int]:
    """解析 imgsz 参数，支持 640 或 480,640 两种形式。"""
    if isinstance(value, int):
        return int(value)
    if isinstance(value, (list, tuple)):
        vals = [int(v) for v in value]
    else:
        vals = [int(v.strip()) for v in str(value).replace("x", ",").split(",") if v.strip()]
    if len(vals) == 1:
        return int(vals[0])
    if len(vals) == 2:
        return (int(vals[0]), int(vals[1]))
    raise ValueError(f"invalid imgsz: {value!r}")


class TrackRequest(BaseModel):
    """单帧 tracking 请求。

    TensorRT 固定词表模式下，label 必须已经存在于启动时传入的 classes 表中。
    prompt_mode / update_visual_prompt 字段仅为兼容旧客户端保留，不参与推理。
    """

    image_base64: str = Field(..., description="JPEG/PNG 图像的 base64 字符串")
    label: str = Field(..., min_length=1, description="固定词表中的目标类别名")
    stamp: float | None = Field(None, description="外部图像时间戳")
    init_bbox: list[float] | None = Field(None, description="可选 xyxy 初始框，多同类目标时建议提供")
    reset: bool = Field(False, description="强制重置当前 tracker")
    strict_identity: bool = Field(True, description="已有 target_id 后是否禁止普通帧自动切换到其他 id")
    allow_rebind: bool = Field(False, description="是否允许本次请求按 bbox/last_bbox 重新绑定新 id")
    lost_rebind: bool = Field(False, description="是否为 lost 后显式重捕获请求")
    tracker: str = Field("deepocsort", description="deepocsort / botsort / bytetrack / ocsort / tracktrack / fasttrack")
    prompt_mode: str = Field("fixed_vocab", description="兼容字段；TensorRT API 固定为 fixed_vocab")
    update_visual_prompt: bool = Field(False, description="兼容字段；TensorRT API 忽略")
    visual_prompt_reset_tracker: bool = Field(True, description="兼容字段；TensorRT API 忽略")
    conf: float | None = Field(None, ge=0.0, le=1.0)
    iou: float | None = Field(None, ge=0.0, le=1.0)
    imgsz: int | None = Field(None, ge=32, description="兼容字段；固定形状 engine 默认忽略请求覆盖")


class ResetRequest(BaseModel):
    """重置 tracker 状态。"""

    reason: str = ""


@dataclass
class PipelineFrame:
    """服务端错帧流水线中的待检测帧。"""

    seq: int
    generation: int
    stamp: float
    image_rgb: np.ndarray
    label: str
    class_id: int
    tracker: str
    conf: float
    iou: float
    strict_identity: bool
    submit_wall: float


@dataclass
class PipelineDetection:
    """YOLOE 检测完成后交给 tracker 的中间结果。"""

    frame: PipelineFrame
    boxes: Any
    model_predict_ms: float
    detect_total_ms: float


class YoloeTensorRtTrackEngine:
    """YOLOE TensorRT 固定词表单目标跟踪引擎。"""

    def __init__(
        self,
        *,
        pt_model_path: str,
        engine_path: str,
        classes_path: str,
        tracker_dir: str,
        device: str,
        conf: float,
        iou: float,
        imgsz: int | tuple[int, int],
        engine_imgsz: int | tuple[int, int],
        rebuild_engine: bool,
        rebind_iou_threshold: float,
        rebind_score_threshold: float,
        pipeline_track: bool,
    ) -> None:
        self.pt_model_path = Path(pt_model_path)
        self.engine_path = Path(engine_path)
        self.classes_path = Path(classes_path)
        self.tracker_dir = Path(tracker_dir)
        self.device = str(device)
        self.default_conf = float(conf)
        self.default_iou = float(iou)
        self.default_imgsz = imgsz
        self.engine_imgsz = engine_imgsz
        self.rebind_iou_threshold = float(rebind_iou_threshold)
        self.rebind_score_threshold = float(rebind_score_threshold)
        self.pipeline_track = bool(pipeline_track)

        self.class_names = self._load_classes(self.classes_path)
        self.label_to_class_id = self._build_label_index(self.class_names)

        self.lock = threading.RLock()
        self.model_lock = threading.Lock()
        self._ensure_engine(rebuild=bool(rebuild_engine))
        self.model = self._load_engine()
        self._warmup_engine()

        self.current_label = ""
        self.current_class_id: int | None = None
        self.current_tracker = "deepocsort"
        self.current_prompt_mode = "fixed_vocab"
        self.current_prompt_source = "fixed_vocab"
        self.target_id: int | None = None
        self.last_bbox: list[int] | None = None
        self.last_score = 0.0
        self.lost_since = 0.0
        self.last_seen_stamp = 0.0
        self.state = "idle"
        self.reason = "not_started"
        self.frame_seq = 0
        self.latest_result = self._make_result(ok=False, reason=self.reason)
        self._pipeline_cond = threading.Condition()
        self._pipeline_det_cond = threading.Condition()
        self._pipeline_pending_frame: PipelineFrame | None = None
        self._pipeline_pending_detection: PipelineDetection | None = None
        self._pipeline_stop = False
        self._pipeline_generation = 0
        self._pipeline_tracker = None
        self._pipeline_tracker_name = ""
        self._pipeline_tracker_lock = threading.Lock()
        self._pipeline_detector_thread = None
        self._pipeline_tracker_thread = None
        if self.pipeline_track:
            self._pipeline_detector_thread = threading.Thread(
                target=self._pipeline_detector_loop,
                name="yoloe-trt-detector",
                daemon=True,
            )
            self._pipeline_tracker_thread = threading.Thread(
                target=self._pipeline_tracker_loop,
                name="yoloe-trt-tracker",
                daemon=True,
            )
            self._pipeline_detector_thread.start()
            self._pipeline_tracker_thread.start()

    def _load_classes(self, classes_path: Path) -> list[str]:
        """读取固定词表；词表顺序必须与导出 engine 时完全一致。"""
        if not classes_path.exists():
            raise FileNotFoundError(f"classes file not found: {classes_path}")
        names = []
        for line in classes_path.read_text(encoding="utf-8").splitlines():
            name = line.strip()
            if name:
                names.append(name)
        if not names:
            raise ValueError(f"classes file is empty: {classes_path}")
        return names

    def _build_label_index(self, names: list[str]) -> dict[str, int]:
        """建立 label 到类别 id 的映射，重复归一化名称会导致类别歧义。"""
        index: dict[str, int] = {}
        for class_id, name in enumerate(names):
            key = _normalize_label(name)
            if key in index:
                raise ValueError(f"duplicate class label after normalization: {name!r}")
            index[key] = int(class_id)
        return index

    def _resolve_label(self, label: str) -> tuple[str, int]:
        """将请求 label 解析为固定词表中的原始名称和 class id。"""
        key = _normalize_label(label)
        if not key:
            raise ValueError("label is required")
        if key not in self.label_to_class_id:
            raise ValueError(f"label_not_in_fixed_vocab: {label}")
        class_id = int(self.label_to_class_id[key])
        return self.class_names[class_id], class_id

    def _ensure_engine(self, *, rebuild: bool) -> None:
        """根据固定词表导出 TensorRT engine；已有 engine 默认复用。"""
        if self.engine_path.exists() and not rebuild:
            return
        if self.device.startswith("cuda") and not torch.cuda.is_available():
            raise RuntimeError("CUDA is requested but torch.cuda.is_available() is False")
        if not self.pt_model_path.exists():
            raise FileNotFoundError(f"pt model not found: {self.pt_model_path}")

        self.engine_path.parent.mkdir(parents=True, exist_ok=True)
        print(
            "[YOLOE_TRT_EXPORT] start",
            f"pt={self.pt_model_path}",
            f"engine={self.engine_path}",
            f"classes={len(self.class_names)}",
            f"imgsz={self.engine_imgsz}",
            flush=True,
        )
        model = YOLO(str(self.pt_model_path)).cuda()
        model.set_classes(self.class_names, model.get_text_pe(self.class_names))
        exported_path = Path(
            model.export(
                format="engine",
                imgsz=self.engine_imgsz,
                dynamic=False,
                half=True,
                simplify=True,
            )
        )
        del model
        gc.collect()
        if torch.cuda.is_available():
            torch.cuda.empty_cache()

        if exported_path.resolve() != self.engine_path.resolve():
            os.replace(str(exported_path), str(self.engine_path))
        print("[YOLOE_TRT_EXPORT] done", f"engine={self.engine_path}", flush=True)

    def _load_engine(self):
        """加载 TensorRT engine。"""
        if not self.engine_path.exists():
            raise FileNotFoundError(f"engine not found: {self.engine_path}")
        print(
            "[YOLOE_TRT_LOAD]",
            f"engine={self.engine_path}",
            f"device={self.device}",
            f"gpu={_uses_cuda_device(self.device)}",
            flush=True,
        )
        return YOLO(str(self.engine_path))

    def _warmup_engine(self) -> None:
        """在主线程预热 TensorRT engine，避免首个 HTTP worker 线程触发 CUDA/TensorRT 懒初始化。"""
        if isinstance(self.default_imgsz, tuple):
            height, width = int(self.default_imgsz[0]), int(self.default_imgsz[1])
        else:
            height = width = int(self.default_imgsz)
        height = max(32, height)
        width = max(32, width)
        warmup_image = np.zeros((height, width, 3), dtype=np.uint8)
        print(
            "[YOLOE_TRT_WARMUP] start",
            f"shape={height}x{width}",
            f"device={self.device}",
            f"gpu={_uses_cuda_device(self.device)}",
            flush=True,
        )
        with torch.no_grad():
            with self.model_lock:
                self.model.predict(
                    source=warmup_image,
                    conf=self.default_conf,
                    iou=self.default_iou,
                    imgsz=self.default_imgsz,
                    device=self.device,
                    verbose=False,
                )
        print("[YOLOE_TRT_WARMUP] done", flush=True)

    def _tracker_cfg_path(self, tracker: str) -> str:
        tracker = str(tracker or "deepocsort").strip().lower()
        if tracker not in TRACKER_MAP:
            raise ValueError(f"unsupported tracker: {tracker}")
        cfg = self.tracker_dir / f"{tracker}.yaml"
        if not cfg.exists():
            raise FileNotFoundError(f"tracker config not found: {cfg}")
        return str(cfg)

    def _build_pipeline_tracker(self, tracker: str):
        """创建独立于 Ultralytics predictor callback 的 tracker 实例。"""
        cfg_path = self._tracker_cfg_path(tracker)
        cfg = IterableSimpleNamespace(**yaml_load(check_yaml(cfg_path)))
        if cfg.tracker_type not in TRACKER_MAP:
            raise ValueError(f"unsupported tracker type: {cfg.tracker_type}")
        return TRACKER_MAP[cfg.tracker_type](args=cfg, frame_rate=30)

    def _clear_pipeline_queues_locked(self) -> None:
        """清空服务端流水线排队帧，并让正在处理的旧帧失效。"""
        self._pipeline_generation += 1
        with self._pipeline_cond:
            self._pipeline_pending_frame = None
            self._pipeline_cond.notify_all()
        with self._pipeline_det_cond:
            self._pipeline_pending_detection = None
            self._pipeline_det_cond.notify_all()

    def _drop_pipeline_tracker_locked(self) -> None:
        """彻底丢弃 pipeline tracker；仅用于 tracker 类型切换等必须重建的场景。"""
        self._pipeline_tracker = None
        self._pipeline_tracker_name = ""

    def _reset_pipeline_tracker_state_locked(self, timings: dict[str, float] | None = None) -> None:
        """重置 tracker 轨迹/GMC 状态，但保留 ReID encoder，避免 TensorRT ReID 反复初始化。"""
        tracker = self._pipeline_tracker
        if tracker is None:
            if timings is not None:
                timings["pipeline_tracker_reset"] = 0.0
            return
        reset_t0 = time.perf_counter()
        with self._pipeline_tracker_lock:
            if hasattr(tracker, "reset"):
                tracker.reset()
        if timings is not None:
            timings["pipeline_tracker_reset"] = 1.0
            timings["pipeline_tracker_reset_ms"] = _ms(time.perf_counter() - reset_t0)

    def _reset_pipeline_locked(
        self,
        *,
        reset_tracker_state: bool = True,
        drop_tracker: bool = False,
        timings: dict[str, float] | None = None,
    ) -> None:
        """清空错帧队列；按需重置轨迹状态或丢弃 tracker 对象。"""
        self._clear_pipeline_queues_locked()
        if drop_tracker:
            with self._pipeline_tracker_lock:
                self._drop_pipeline_tracker_locked()
            if timings is not None:
                timings["pipeline_tracker_dropped"] = 1.0
            return
        if reset_tracker_state:
            self._reset_pipeline_tracker_state_locked(timings=timings)

    def _submit_pipeline_frame(
        self,
        *,
        image_rgb: np.ndarray,
        label: str,
        class_id: int,
        tracker: str,
        stamp: float,
        conf: float,
        iou: float,
        strict_identity: bool,
    ) -> dict[str, Any]:
        """提交普通连续 update 帧，并立即返回最近完成的 tracking 结果。"""
        with self.lock:
            self.frame_seq += 1
            frame = PipelineFrame(
                seq=int(self.frame_seq),
                generation=int(self._pipeline_generation),
                stamp=float(stamp),
                image_rgb=image_rgb,
                label=str(label),
                class_id=int(class_id),
                tracker=str(tracker),
                conf=float(conf),
                iou=float(iou),
                strict_identity=bool(strict_identity),
                submit_wall=time.perf_counter(),
            )
        with self._pipeline_cond:
            self._pipeline_pending_frame = frame
            self._pipeline_cond.notify()
        result = self.latest()
        timings = dict(result.get("timings") or {})
        timings["pipeline_submit_ms"] = 0.0
        timings["pipeline_return_latest"] = 1.0
        result["timings"] = timings
        return result

    def _pipeline_detector_loop(self) -> None:
        """检测线程：只保留最新帧，执行 YOLOE TensorRT predict。"""
        while True:
            with self._pipeline_cond:
                while self._pipeline_pending_frame is None and not self._pipeline_stop:
                    self._pipeline_cond.wait()
                if self._pipeline_stop:
                    return
                frame = self._pipeline_pending_frame
                self._pipeline_pending_frame = None

            detect_t0 = time.perf_counter()
            try:
                predict_t0 = time.perf_counter()
                with torch.no_grad():
                    with self.model_lock:
                        results = self.model.predict(
                            source=frame.image_rgb,
                            conf=frame.conf,
                            iou=frame.iou,
                            imgsz=self.default_imgsz,
                            device=self.device,
                            verbose=False,
                        )
                model_predict_ms = _ms(time.perf_counter() - predict_t0)
                result = results[0] if results else None
                boxes = None if result is None or result.boxes is None else result.boxes.cpu().numpy()
                detection = PipelineDetection(
                    frame=frame,
                    boxes=boxes,
                    model_predict_ms=model_predict_ms,
                    detect_total_ms=_ms(time.perf_counter() - detect_t0),
                )
            except Exception as exc:
                self._mark_pipeline_error(frame, f"pipeline_detect_error:{exc}")
                continue

            with self.lock:
                # reset / rebind 后，正在检测线程中的旧帧可能已经失效，避免旧结果覆盖新目标状态。
                if frame.generation != self._pipeline_generation:
                    continue
            with self._pipeline_det_cond:
                self._pipeline_pending_detection = detection
                self._pipeline_det_cond.notify()

    def _pipeline_tracker_loop(self) -> None:
        """Tracker 线程：按检测完成顺序更新 BoT-SORT/ByteTrack 状态。"""
        while True:
            with self._pipeline_det_cond:
                while self._pipeline_pending_detection is None and not self._pipeline_stop:
                    self._pipeline_det_cond.wait()
                if self._pipeline_stop:
                    return
                detection = self._pipeline_pending_detection
                self._pipeline_pending_detection = None
            self._process_pipeline_detection(detection)

    def _mark_pipeline_error(self, frame: PipelineFrame, reason: str) -> None:
        timings = {
            "pipeline": 1.0,
            "total_ms": _ms(time.perf_counter() - frame.submit_wall),
        }
        with self.lock:
            if frame.generation != self._pipeline_generation:
                return
            self._mark_lost(
                stamp=frame.stamp,
                frame_seq=frame.seq,
                reason=reason,
                timings=timings,
            )

    def _process_pipeline_detection(self, detection: PipelineDetection) -> None:
        frame = detection.frame
        total_t0 = frame.submit_wall
        timings: dict[str, float] = {
            "pipeline": 1.0,
            "model_predict_ms": float(detection.model_predict_ms),
            "pipeline_detect_total_ms": float(detection.detect_total_ms),
            "yoloe_trt_gpu": float(1 if _uses_cuda_device(self.device) else 0),
            "yoloe_trt_device": self.device,
            "yoloe_trt_engine": str(self.engine_path),
        }
        try:
            with self.lock:
                if frame.generation != self._pipeline_generation:
                    return
            with self._pipeline_tracker_lock:
                if frame.generation != self._pipeline_generation:
                    return
                build_t0 = time.perf_counter()
                tracker_reused = self._pipeline_tracker is not None and self._pipeline_tracker_name == frame.tracker
                if not tracker_reused:
                    self._pipeline_tracker = self._build_pipeline_tracker(frame.tracker)
                    self._pipeline_tracker_name = frame.tracker
                    timings["pipeline_tracker_build_ms"] = _ms(time.perf_counter() - build_t0)
                timings["pipeline_tracker_reused"] = float(1 if tracker_reused else 0)
                tracker = self._pipeline_tracker

                boxes = detection.boxes
                # 只保留当前目标类别的 bbox，避免无关类别干扰 tracker 关联
                if boxes is not None and len(boxes) > 0:
                    boxes = boxes[boxes[:, 5] == frame.class_id]
                if boxes is None or len(boxes) == 0:
                    tracks = []
                    timings["tracker_input_det_count"] = 0.0
                    timings["tracker_output_track_count"] = 0.0
                    timings["tracker_update_ms"] = 0.0
                else:
                    tracker_t0 = time.perf_counter()
                    tracks = tracker.update(boxes, frame.image_rgb)
                    timings["tracker_update_ms"] = _ms(time.perf_counter() - tracker_t0)
                    reid_stats = dict(getattr(tracker, "last_reid_stats", {}) or {})
                    timings["tracker_reid_enabled"] = float(1 if reid_stats.get("enabled") else 0)
                    timings["tracker_reid_feature_count"] = float(reid_stats.get("feature_count", 0) or 0)
                    timings["tracker_reid_feature_dim"] = float(reid_stats.get("feature_dim", 0) or 0)
                    timings["tracker_reid_inference_ms"] = float(reid_stats.get("inference_ms", 0.0) or 0.0)
                    timings["tracker_reid_backend"] = str(reid_stats.get("backend", "none"))
                    timings["tracker_reid_gpu"] = float(1 if reid_stats.get("gpu") else 0)
                    timings["tracker_reid_gpu_idx"] = float(reid_stats.get("gpu_idx", -1))
                    timings["tracker_input_det_count"] = float(len(boxes))
                    timings["tracker_output_track_count"] = float(len(tracks))
                    timings["tracker_dropped_count"] = float(max(0, len(boxes) - len(tracks)))

            if boxes is None or len(boxes) == 0:
                timings["total_ms"] = _ms(time.perf_counter() - total_t0)
                with self.lock:
                    if frame.generation != self._pipeline_generation:
                        return
                    self._mark_lost(
                        stamp=frame.stamp,
                        frame_seq=frame.seq,
                        reason="no_target_class_candidates",
                        timings=timings,
                    )
                return

            candidates = self._extract_candidates_from_tracks(tracks, frame.image_rgb)
            candidates = [item for item in candidates if int(item["cls"]) == int(frame.class_id)]
            timings["candidate_count"] = float(len(candidates))
            timings["all_candidate_count"] = float(len(tracks))
            with self.lock:
                if frame.generation != self._pipeline_generation:
                    return
                if not candidates:
                    timings["total_ms"] = _ms(time.perf_counter() - total_t0)
                    self._mark_lost(
                        stamp=frame.stamp,
                        frame_seq=frame.seq,
                        reason="no_target_class_candidates",
                        timings=timings,
                    )
                    return

                selected = self._select_target(
                    candidates,
                    init_bbox=None,
                    strict_identity=frame.strict_identity,
                    allow_rebind=False,
                )
                if selected is None:
                    timings["total_ms"] = _ms(time.perf_counter() - total_t0)
                    self._mark_lost(
                        stamp=frame.stamp,
                        frame_seq=frame.seq,
                        reason="target_missing",
                        timings=timings,
                    )
                    return

                selected_track_id = int(selected["track_id"])
                self.current_label = frame.label
                self.current_class_id = frame.class_id
                self.current_tracker = frame.tracker
                self.target_id = None if selected_track_id < 0 else selected_track_id
                self.last_bbox = list(selected["bbox"])
                self.last_score = float(selected["score"])
                self.lost_since = 0.0
                self.last_seen_stamp = frame.stamp
                self.state = "active"
                self.reason = ""
                timings["model_track_ms"] = float(detection.model_predict_ms) + float(timings["tracker_update_ms"])
                timings["total_ms"] = _ms(time.perf_counter() - total_t0)
                self.latest_result = self._make_result(
                    ok=True,
                    stamp=frame.stamp,
                    frame_seq=frame.seq,
                    bbox=self.last_bbox,
                    track_id=self.target_id,
                    score=self.last_score,
                    cls=selected["cls"],
                    has_mask=False,
                    reason="",
                    timings=timings,
                )
        except Exception as exc:
            self._mark_pipeline_error(frame, f"pipeline_tracker_error:{exc}")

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
        """移除旧 tracker 回调，避免后续 predict() 被旧 tracking 状态污染。"""
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
        """清理 tracker 状态，但保留 TensorRT predictor/execution context。

        TensorRT engine 的 predictor 初始化成本和 native 状态较重，频繁置空 predictor
        容易在 FastAPI 服务中触发底层 CUDA/TensorRT 资源重复释放。固定词表模式下
        label 切换不需要重建 predictor，只需要重置 tracker 和移除 tracker callbacks。
        """
        if self.model is None:
            return 0
        self._reset_ultralytics_trackers()
        removed_callbacks = self._clear_ultralytics_tracker_callbacks()
        return removed_callbacks

    def _register_timed_tracker_callbacks(self, *, persist: bool, timings: dict[str, float]) -> None:
        """注册带计时的 tracker callback，用于拆分 TensorRT YOLOE 和 tracker/ReID 耗时。"""
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
        """执行 Ultralytics tracker 后处理，并记录 BoT-SORT/ReID 统计。"""
        callback_t0 = time.perf_counter()
        timings = timings if timings is not None else {}
        path, im0s = predictor.batch[:2]

        is_obb = predictor.args.task == "obb"
        is_stream = predictor.dataset.mode == "stream"
        tracker_update_seconds = 0.0
        total_tracker_input_dets = 0
        total_tracker_output_tracks = 0
        last_reid_stats: dict[str, Any] = {}
        for i in range(len(im0s)):
            tracker = predictor.trackers[i if is_stream else 0]
            vid_path = predictor.save_dir / Path(path[i]).name
            if not persist and predictor.vid_path[i if is_stream else 0] != vid_path:
                tracker.reset()
                predictor.vid_path[i if is_stream else 0] = vid_path

            det = (predictor.results[i].obb if is_obb else predictor.results[i].boxes).cpu().numpy()
            # 只保留当前目标类别的 bbox，避免无关类别干扰 tracker 关联
            det = det[det.cls == self.current_class_id]
            total_tracker_input_dets += int(len(det))
            if len(det) == 0:
                continue

            update_t0 = time.perf_counter()
            tracks = tracker.update(det, im0s[i])
            tracker_update_seconds += time.perf_counter() - update_t0
            total_tracker_output_tracks += int(len(tracks))
            last_reid_stats = dict(getattr(tracker, "last_reid_stats", {}) or {})
            if len(tracks) == 0:
                continue
            idx = tracks[:, -1].astype(int)
            predictor.results[i] = predictor.results[i][idx]

            update_args = {"obb" if is_obb else "boxes": torch.as_tensor(tracks[:, :-1])}
            predictor.results[i].update(**update_args)

        timings["tracker_update_ms"] = _ms(tracker_update_seconds)
        timings["tracker_callback_ms"] = _ms(time.perf_counter() - callback_t0)
        timings["tracker_input_det_count"] = float(total_tracker_input_dets)
        timings["tracker_output_track_count"] = float(total_tracker_output_tracks)
        timings["tracker_dropped_count"] = float(max(0, total_tracker_input_dets - total_tracker_output_tracks))
        timings["tracker_reid_enabled"] = float(1 if last_reid_stats.get("enabled") else 0)
        timings["tracker_reid_feature_count"] = float(last_reid_stats.get("feature_count", 0) or 0)
        timings["tracker_reid_feature_dim"] = float(last_reid_stats.get("feature_dim", 0) or 0)
        timings["tracker_reid_inference_ms"] = float(last_reid_stats.get("inference_ms", 0.0) or 0.0)
        timings["tracker_reid_backend"] = str(last_reid_stats.get("backend", "none"))
        timings["tracker_reid_gpu"] = float(1 if last_reid_stats.get("gpu") else 0)
        timings["tracker_reid_gpu_idx"] = float(last_reid_stats.get("gpu_idx", -1))

    def _track_with_timing(
        self,
        *,
        source: np.ndarray,
        tracker: str,
        conf: float,
        iou: float,
        imgsz: int | tuple[int, int],
        timings: dict[str, float],
    ):
        """调用 Ultralytics track mode，并把 tracker/ReID 统计写入 timings。"""
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

    def reset(self, reason: str = "") -> dict[str, Any]:
        with self.lock:
            self._reset_pipeline_locked()
            self._reset_predictor_and_tracker_state()
            self.current_label = ""
            self.current_class_id = None
            self.current_prompt_mode = "fixed_vocab"
            self.current_prompt_source = "fixed_vocab"
            self.target_id = None
            self.last_bbox = None
            self.last_score = 0.0
            self.lost_since = 0.0
            self.last_seen_stamp = 0.0
            self.state = "idle"
            self.reason = reason or "reset"
            self.latest_result = self._make_result(ok=False, reason=self.reason)
            return dict(self.latest_result)

    def status(self) -> dict[str, Any]:
        with self.lock:
            return {
                "state": self.state,
                "label": self.current_label,
                "class_id": self.current_class_id,
                "tracker": self.current_tracker,
                "prompt_mode": self.current_prompt_mode,
                "prompt_source": self.current_prompt_source,
                "track_id": self.target_id,
                "last_bbox": self.last_bbox,
                "last_score": self.last_score,
                "lost_since": self.lost_since,
                "last_seen_stamp": self.last_seen_stamp,
                "reason": self.reason,
                "frame_seq": self.frame_seq,
                "engine": str(self.engine_path),
                "classes": str(self.classes_path),
                "class_count": len(self.class_names),
                "imgsz": self.default_imgsz,
                "engine_imgsz": self.engine_imgsz,
                "device": self.device,
                "gpu": _uses_cuda_device(self.device),
                "pipeline_track": bool(self.pipeline_track),
            }

    def latest(self) -> dict[str, Any]:
        with self.lock:
            return dict(self.latest_result)

    def track(self, req: TrackRequest) -> dict[str, Any]:
        total_t0 = time.perf_counter()
        timings: dict[str, float] = {}
        t0 = time.perf_counter()
        image_rgb = _decode_image_base64(req.image_base64)
        timings["decode_ms"] = _ms(time.perf_counter() - t0)
        stamp = float(req.stamp if req.stamp is not None else _now())
        label, class_id = self._resolve_label(req.label)

        tracker = req.tracker.strip().lower()
        conf = float(req.conf if req.conf is not None else self.default_conf)
        iou = float(req.iou if req.iou is not None else self.default_iou)
        if (
            self.pipeline_track
            and not req.reset
            and req.init_bbox is None
            and not req.allow_rebind
            and not req.lost_rebind
            and class_id == self.current_class_id
            and tracker == self.current_tracker
        ):
            return self._submit_pipeline_frame(
                image_rgb=image_rgb,
                label=label,
                class_id=class_id,
                tracker=tracker,
                stamp=stamp,
                conf=conf,
                iou=iou,
                strict_identity=bool(req.strict_identity),
            )

        lock_t0 = time.perf_counter()
        with self.lock:
            timings["lock_wait_ms"] = _ms(time.perf_counter() - lock_t0)
            timings["yoloe_trt_gpu"] = float(1 if _uses_cuda_device(self.device) else 0)
            timings["yoloe_trt_device"] = self.device
            timings["yoloe_trt_engine"] = str(self.engine_path)
            label_changed = class_id != self.current_class_id
            tracker_changed = tracker != self.current_tracker
            init_bbox = _clip_bbox(req.init_bbox, image_rgb) if req.init_bbox is not None else None
            pipeline_rebind = bool(self.pipeline_track and (init_bbox is not None or req.allow_rebind or req.lost_rebind))
            new_target_started = False

            if pipeline_rebind:
                self._reset_pipeline_locked(timings=timings)
            if req.reset or label_changed or tracker_changed:
                t0 = time.perf_counter()
                self._reset_pipeline_locked(drop_tracker=tracker_changed, timings=timings)
                removed_callbacks = self._reset_predictor_and_tracker_state()
                timings["clear_tracker_callbacks"] = float(removed_callbacks)
                timings["clear_state_ms"] = _ms(time.perf_counter() - t0)

                self.current_label = label
                self.current_class_id = class_id
                self.current_tracker = tracker
                self.current_prompt_mode = "fixed_vocab"
                self.current_prompt_source = "fixed_vocab"
                self.target_id = None
                self.last_bbox = None
                self.last_score = 0.0
                self.lost_since = 0.0
                self.last_seen_stamp = 0.0
                self.state = "acquiring"
                self.reason = "reset" if req.reset else "new_target"
                new_target_started = True

            self.frame_seq += 1
            frame_seq = self.frame_seq
            imgsz = self.default_imgsz
            if req.imgsz is not None and req.imgsz != self.default_imgsz:
                # TensorRT engine 是固定形状导出，保持服务端启动时的 imgsz，避免请求覆盖导致 shape 不匹配。
                timings["ignored_request_imgsz"] = float(req.imgsz)
            tracker_cfg = self._tracker_cfg_path(self.current_tracker)

            t0 = time.perf_counter()
            if new_target_started or pipeline_rebind:
                with self.model_lock:
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
                with self.model_lock:
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
            t0 = time.perf_counter()
            all_candidates = self._extract_candidates(result, image_rgb)
            candidates = [item for item in all_candidates if int(item["cls"]) == int(class_id)]
            timings["extract_candidates_ms"] = _ms(time.perf_counter() - t0)
            timings["candidate_count"] = float(len(candidates))
            timings["all_candidate_count"] = float(len(all_candidates))
            if not candidates:
                timings["total_ms"] = _ms(time.perf_counter() - total_t0)
                return self._mark_lost(
                    stamp=stamp,
                    frame_seq=frame_seq,
                    reason="no_target_class_candidates",
                    timings=timings,
                )

            t0 = time.perf_counter()
            selected = self._select_target(
                candidates,
                init_bbox=init_bbox,
                strict_identity=bool(req.strict_identity),
                allow_rebind=bool(req.allow_rebind or req.lost_rebind or init_bbox is not None),
            )
            timings["select_target_ms"] = _ms(time.perf_counter() - t0)
            if selected is None:
                timings["total_ms"] = _ms(time.perf_counter() - total_t0)
                return self._mark_lost(
                    stamp=stamp,
                    frame_seq=frame_seq,
                    reason="target_missing",
                    timings=timings,
                )

            selected_track_id = int(selected["track_id"])
            self.target_id = None if selected_track_id < 0 else selected_track_id
            self.last_bbox = list(selected["bbox"])
            self.last_score = float(selected["score"])
            self.lost_since = 0.0
            self.last_seen_stamp = stamp
            self.state = "active"
            self.reason = ""
            timings["total_ms"] = _ms(time.perf_counter() - total_t0)
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

    def _extract_candidates(self, result, image_rgb: np.ndarray) -> list[dict[str, Any]]:
        if result is None or result.boxes is None:
            return []

        boxes = result.boxes
        xyxy = boxes.xyxy.cpu().tolist()
        if not xyxy:
            return []

        # predict() 首帧没有真实 tracker id，使用负数临时 id，后续 track() 再绑定真实 id。
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

    def _extract_candidates_from_tracks(self, tracks, image_rgb: np.ndarray) -> list[dict[str, Any]]:
        """将 tracker.update 返回的 ndarray 转为统一候选列表。"""
        if tracks is None or len(tracks) == 0:
            return []
        candidates = []
        for row in np.asarray(tracks):
            if len(row) < 7:
                continue
            bbox = row[:4].tolist()
            clipped = _clip_bbox(bbox, image_rgb)
            if clipped is None:
                continue
            candidates.append(
                {
                    "track_id": int(row[4]),
                    "bbox": clipped,
                    "score": float(row[5]),
                    "cls": int(row[6]),
                    "has_mask": False,
                }
            )
        return candidates

    def _select_target(
        self,
        candidates: list[dict[str, Any]],
        init_bbox: list[float] | None,
        *,
        strict_identity: bool,
        allow_rebind: bool,
    ) -> dict[str, Any] | None:
        """从同类候选中选择当前目标，优先保持 track id，其次按 bbox 重绑定。"""
        if init_bbox is None and self.target_id is not None:
            for candidate in candidates:
                if int(candidate["track_id"]) == int(self.target_id):
                    return candidate
            if strict_identity and not allow_rebind:
                return None

        hint_bbox = init_bbox or self.last_bbox
        if hint_bbox is None:
            return max(candidates, key=lambda item: item["score"])

        if self.target_id is not None and not allow_rebind:
            return None

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
        if self.lost_since <= 0.0:
            self.lost_since = float(stamp)
        self.reason = reason
        self.latest_result = self._make_result(
            ok=False,
            stamp=stamp,
            frame_seq=frame_seq,
            bbox=self.last_bbox,
            track_id=self.target_id,
            score=self.last_score,
            cls=self.current_class_id,
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
            "class_id": self.current_class_id,
            "tracker": self.current_tracker,
            "prompt_mode": self.current_prompt_mode,
            "prompt_source": self.current_prompt_source,
            "track_id": track_id,
            "bbox": bbox,
            "score": float(score),
            "cls": cls,
            "has_mask": bool(has_mask),
            "lost_since": float(self.lost_since),
            "last_seen_stamp": float(self.last_seen_stamp),
            "stamp": float(stamp),
            "frame_seq": frame_seq,
            "source": f"yoloe_trt:{self.current_tracker}",
            "reason": str(reason or ""),
            "timings": dict(timings or {}),
            "wall_time": _now(),
        }


def create_app(engine: YoloeTensorRtTrackEngine) -> FastAPI:
    app = FastAPI(title="YOLOE TensorRT Fixed-Vocab Track Engine")

    @app.post("/track")
    async def track(req: TrackRequest):
        try:
            return engine.track(req)
        except Exception as exc:
            print(f"YOLOE TensorRT track API request failed: {exc}", flush=True)
            raise HTTPException(status_code=400, detail=str(exc)) from exc

    @app.post("/reset")
    async def reset(req: ResetRequest | None = None):
        return engine.reset(reason="" if req is None else req.reason)

    @app.get("/status")
    async def status():
        return engine.status()

    @app.get("/latest")
    async def latest():
        return engine.latest()

    return app


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="YOLOE TensorRT fixed-vocab tracking API")
    parser.add_argument("--pt-model", default=str(YOLOE_ROOT / "prompt" / "yoloe_pretrain" / "yoloe-11m-seg.pt"))
    parser.add_argument("--engine", default=str(YOLOE_ROOT / "yoloe-v8m-seg-test.onnx"))
    parser.add_argument("--classes", default=str(YOLOE_ROOT / "prompt" / "prompt2.txt"))
    parser.add_argument("--tracker-dir", default=str(YOLOE_ROOT / "ultralytics" / "cfg" / "trackers"))
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--conf", type=float, default=0.1)
    parser.add_argument("--iou", type=float, default=0.5)
    parser.add_argument("--imgsz", default="480,640", help="TensorRT 推理尺寸；固定形状 engine 应与导出尺寸一致")
    parser.add_argument("--engine-imgsz", default="480,640", help="engine 导出尺寸，例如 480,640")
    parser.add_argument("--rebuild-engine", action="store_true", help="忽略已有 engine，重新从 pt 和 classes 导出")
    parser.add_argument("--rebind-iou-threshold", type=float, default=0.05)
    parser.add_argument("--rebind-score-threshold", type=float, default=0.15)
    parser.add_argument("--pipeline-track", action="store_true", help="普通连续 update 使用服务端 YOLOE/tracker 错帧流水线")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2250)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    engine = YoloeTensorRtTrackEngine(
        pt_model_path=args.pt_model,
        engine_path=args.engine,
        classes_path=args.classes,
        tracker_dir=args.tracker_dir,
        device=args.device,
        conf=args.conf,
        iou=args.iou,
        imgsz=_parse_imgsz(args.imgsz),
        engine_imgsz=_parse_imgsz(args.engine_imgsz),
        rebuild_engine=args.rebuild_engine,
        rebind_iou_threshold=args.rebind_iou_threshold,
        rebind_score_threshold=args.rebind_score_threshold,
        pipeline_track=args.pipeline_track,
    )
    app = create_app(engine)
    uvicorn.run(app, host=args.host, port=args.port, reload=False, workers=1)


if __name__ == "__main__":
    main()

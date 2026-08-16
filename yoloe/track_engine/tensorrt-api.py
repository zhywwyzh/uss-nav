#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""YOLOE TensorRT 固定词表目标跟踪 API（v2：基于官方 model.track() 方案）。

与 api.py（动态 text prompt）的区别：
- 使用预导出的 TensorRT engine，词表在启动时固定
- 不支持运行时 set_classes()，label 必须在固定词表中
- 其余逻辑与 api.py v2 一致：model.track(persist=True) + VLM bbox→track_id 匹配
"""

from __future__ import annotations

import argparse
import base64
import gc
import os
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

# 结构化时序日志（依赖 track_engine 包，须在 YOLOE_ROOT 入 sys.path 之后导入）
from track_engine.tracking_logger import TrackingLogger, cuda_memory_snapshot, gc_snapshot  # noqa: E402

torch.backends.cudnn.enabled = False
torch.backends.cuda.matmul.allow_tf32 = True

from ultralytics import YOLO  # noqa: E402
from ultralytics.trackers.track import TRACKER_MAP, on_predict_start, on_predict_postprocess_end  # noqa: E402
from functools import partial  # noqa: E402


def _now() -> float:
    return time.time()


def _ms(seconds: float) -> float:
    return round(float(seconds) * 1000.0, 3)


def _uses_cuda_device(device: str) -> bool:
    return str(device or "").strip().lower().startswith("cuda")


def _decode_image_base64(image_base64: str) -> np.ndarray:
    raw = base64.b64decode(image_base64)
    np_buf = np.frombuffer(raw, dtype=np.uint8)
    image_bgr = cv2.imdecode(np_buf, cv2.IMREAD_COLOR)
    if image_bgr is None:
        raise ValueError("cv2.imdecode returned None")
    return image_bgr


def _clip_bbox(bbox: list[float], image: np.ndarray) -> list[int] | None:
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


def _normalize_label(label: str) -> str:
    return " ".join(str(label or "").strip().lower().split())


# ──────────────────────────────────────────────
# Request / Response
# ──────────────────────────────────────────────

class TrackRequest(BaseModel):
    """单帧 tracking 请求（TensorRT 固定词表版）。

    init_bbox 仅用于首帧匹配目标 track_id。
    """

    image_base64: str = Field(..., description="JPEG/PNG 图像的 base64 字符串")
    label: str = Field(..., min_length=1, description="固定词表中的目标类别名")
    stamp: float | None = Field(None, description="外部图像时间戳")
    init_bbox: list[float] | None = Field(None, description="VLM 给出的初始 xyxy 框，用于匹配目标 track_id")
    reset: bool = Field(False, description="强制重置当前 tracking 会话并重新匹配 init_bbox")
    tracker: str = Field("botsort", description="tracker 类型")
    conf: float | None = Field(None, ge=0.0, le=1.0)
    iou: float | None = Field(None, ge=0.0, le=1.0)
    imgsz: int | None = Field(None, description="兼容字段；固定形状 engine 忽略")
    send_seq: int = Field(0, description="客户端发送序列号，服务端回传以验证一一对应")

    # ── 以下参数保留以兼容旧客户端，不再生效 ──
    strict_identity: bool = Field(True, description="[已废弃]")
    allow_rebind: bool = Field(False, description="[已废弃]")
    lost_rebind: bool = Field(False, description="[已废弃]")
    prompt_mode: str = Field("fixed_vocab", description="[已废弃]")
    update_visual_prompt: bool = Field(False, description="[已废弃]")
    visual_prompt_reset_tracker: bool = Field(True, description="[已废弃]")


class ResetRequest(BaseModel):
    reason: str = ""


# ──────────────────────────────────────────────
# Engine
# ──────────────────────────────────────────────

class YoloeTensorRtTrackEngine:
    """YOLOE TensorRT 固定词表单目标跟踪引擎（v2）。"""

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
        init_bbox_match_iou: float = 0.1,
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
        self.init_bbox_match_iou = float(init_bbox_match_iou)

        # 固定词表
        self.class_names = self._load_classes(self.classes_path)
        self.label_to_class_id = self._build_label_index(self.class_names)

        self.lock = threading.Lock()
        self.model_lock = threading.Lock()
        self._ensure_engine(rebuild=bool(rebuild_engine))
        self.model = self._load_engine()
        self._warmup_engine()

        # tracking 状态
        self.current_label: str = ""
        self.current_class_id: int | None = None
        self.current_tracker: str = "deepocsort"
        self.target_track_id: int | None = None
        self.last_bbox: list[int] | None = None
        self.last_score: float = 0.0
        self.state: str = "idle"
        self.frame_seq: int = 0
        self.latest_result: dict[str, Any] = {}

        # 手动 tracker 实例（跨帧维持状态，只跟踪 target class 的检测框）
        self._tracker_instance_name: str = ""
        self._filtered_class_id: int | None = None
        self._tracking_timings: dict[str, float] = {}

        # ── 结构化日志 ──
        self._track_logger = TrackingLogger.get("tracker_server")

    # ── 回调式追踪（复现官方 model.track() 流程）──

    def _clear_tracker_callbacks(self) -> int:
        """移除旧 tracker 回调，避免旧的过滤逻辑污染新调用。"""
        callbacks = getattr(self.model, "callbacks", None)
        if not callbacks:
            return 0
        removed = 0
        for event in ("on_predict_start", "on_predict_postprocess_end"):
            old_items = list(callbacks.get(event, []))
            new_items = [
                cb for cb in old_items
                if "ultralytics.trackers.track" not in str(getattr(getattr(cb, "func", cb), "__module__", ""))
                and getattr(getattr(cb, "func", cb), "__name__", "") not in (
                    "_filter_boxes_for_target_class", "on_predict_start", "on_predict_postprocess_end"
                )
            ]
            callbacks[event] = new_items
            removed += len(old_items) - len(new_items)
        return removed

    def _filter_boxes_for_target_class(self, predictor: "object", class_id: int) -> None:
        """on_predict_postprocess_end 回调：过滤掉非目标类别的检测框。

        在官方 tracker 回调之前运行，确保 tracker 只看到目标类别的框。
        """
        for i in range(len(predictor.results)):
            boxes = predictor.results[i].boxes
            if boxes is None or len(boxes) == 0:
                continue
            if boxes.cls is None:
                continue
            mask = boxes.cls.int().cpu().numpy() == class_id
            if not mask.any():
                predictor.results[i] = predictor.results[i][0:0]
                continue
            idx = mask.nonzero()[0]
            predictor.results[i] = predictor.results[i][idx]

    def _register_label_filter(self, class_id: int, tracker_cfg: str) -> None:
        """注册回复现官方 model.track(): 在 tracker 回调之前插入 class_id 过滤。

        model.track() 内部会调用 register_tracker() 注册两个回调：
        on_predict_start → on_predict_postprocess_end（tracker 在此处理全部检测框）。
        我们在 on_predict_postprocess_end 的前面插入自己的回调，
        过滤掉非目标类别的框后再交由官方 tracker 处理。
        """
        self._clear_tracker_callbacks()
        self._filtered_class_id = int(class_id)

        # 注册过滤回调（先于 tracker 回调执行）
        self.model.add_callback(
            "on_predict_postprocess_end",
            partial(
                self._filter_boxes_for_target_class,
                class_id=class_id,
            ),
        )

    # ── 词表 ──

    def _load_classes(self, classes_path: Path) -> list[str]:
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
        index: dict[str, int] = {}
        for class_id, name in enumerate(names):
            key = _normalize_label(name)
            if key in index:
                raise ValueError(f"duplicate class label: {name!r}")
            index[key] = int(class_id)
        return index

    def _resolve_label(self, label: str) -> tuple[str, int]:
        key = _normalize_label(label)
        if not key:
            raise ValueError("label is required")
        if key not in self.label_to_class_id:
            raise ValueError(f"label_not_in_fixed_vocab: {label}")
        class_id = int(self.label_to_class_id[key])
        return self.class_names[class_id], class_id

    # ── 模型加载 ──

    def _ensure_engine(self, *, rebuild: bool) -> None:
        if self.engine_path.exists() and not rebuild:
            return
        if self.device.startswith("cuda") and not torch.cuda.is_available():
            raise RuntimeError("CUDA requested but not available")
        print(f"[YOLOE_TRT] exporting engine to {self.engine_path}")
        model = YOLO(str(self.pt_model_path))
        model.to(self.device)
        if self.default_imgsz != self.engine_imgsz:
            print(f"[YOLOE_TRT] WARNING: engine_imgsz={self.engine_imgsz} != imgsz={self.default_imgsz}")
        model.export(
            format="engine",
            imgsz=self.engine_imgsz,
            device=self.device,
            half=True,
        )
        exported = Path(str(self.pt_model_path).replace(".pt", ".engine"))
        if exported.exists():
            if self.engine_path != exported:
                import shutil
                shutil.copy2(str(exported), str(self.engine_path))
        del model
        gc.collect()
        if _uses_cuda_device(self.device):
            torch.cuda.synchronize()
            torch.cuda.empty_cache()

    def _load_engine(self):
        if self.device.startswith("cuda") and not torch.cuda.is_available():
            raise RuntimeError("CUDA requested but not available")
        # TensorRT engine 已经编译到特定设备，不需要 .to(device) / .eval()
        model = YOLO(str(self.engine_path))
        return model

    def _warmup_engine(self) -> None:
        h, w = (640, 640)
        if isinstance(self.engine_imgsz, int):
            h = w = int(self.engine_imgsz)
        elif isinstance(self.engine_imgsz, (tuple, list)) and len(self.engine_imgsz) >= 2:
            h, w = int(self.engine_imgsz[0]), int(self.engine_imgsz[1])
        dummy = np.zeros((h, w, 3), dtype=np.uint8)
        for _ in range(3):
            with torch.no_grad():
                with self.model_lock:
                    self.model.predict(source=dummy, imgsz=self.engine_imgsz, device=self.device, verbose=False)

    def _tracker_cfg_path(self, tracker: str) -> str:
        tracker = str(tracker).strip().lower() or "deepocsort"
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
        send_seq = int(req.send_seq if req.send_seq is not None else 0)

        t0 = time.perf_counter()
        image_bgr = _decode_image_base64(req.image_base64)
        timings["decode_ms"] = _ms(time.perf_counter() - t0)

        stamp = float(req.stamp if req.stamp is not None else _now())
        label, class_id = self._resolve_label(req.label)
        tracker_name = req.tracker.strip().lower() or "botsort"
        tracker_cfg = self._tracker_cfg_path(tracker_name)
        conf = float(req.conf if req.conf is not None else self.default_conf)
        iou_val = float(req.iou if req.iou is not None else self.default_iou)
        imgsz = self.default_imgsz

        lock_t0 = time.perf_counter()
        with self.lock:
            timings["lock_wait_ms"] = _ms(time.perf_counter() - lock_t0)
            timings["yoloe_trt_gpu"] = float(1 if _uses_cuda_device(self.device) else 0)
            timings["yoloe_trt_device"] = self.device
            timings["yoloe_trt_engine"] = str(self.engine_path)

            label_changed = (class_id != self.current_class_id) or (tracker_name != self.current_tracker)


            # ── 轨道数量上限裁剪：每帧保持 lost_stracks 最近 15 条，防止 O(n²) 退化 ──
            reset_t0 = time.perf_counter()
            pred = self.model.predictor
            if pred is not None and hasattr(pred, "trackers") and pred.trackers:
                for t in pred.trackers:
                    # 只裁剪 lost_stracks（活跃跟踪目标保留不动）
                    if hasattr(t, "lost_stracks") and len(t.lost_stracks) > 15:
                        t.lost_stracks = t.lost_stracks[-15:]
                    if hasattr(t, "removed_stracks"):
                        t.removed_stracks = []
            timings["trt_periodic_reset_ms"] = _ms(time.perf_counter() - reset_t0)
            if req.reset or label_changed:
                # 注册/更新 label 过滤回调（先于 tracker 回调执行）
                self._register_label_filter(class_id, tracker_cfg)
                # 重置 predictor，强制 model.track() 重建 tracker
                self.model.predictor = None
                self.current_label = label
                self.current_class_id = class_id
                self.current_tracker = tracker_name
                self.target_track_id = None
                self.last_bbox = None
                self.last_score = 0.0
                self.state = "acquiring"
                self.frame_seq = 0
                timings["trt_reset"] = 1.0
                print(
                    f"[YOLOE_TRT] reset label={label!r} class_id={class_id} tracker={tracker_name}",
                    flush=True,
                )

            self.frame_seq += 1

            # ═══════════════════════════════════════════════════
            # 复现官方 model.track(): model.track(source, persist=True, tracker=cfg)
            # 过滤回调 _filter_boxes_for_target_class 已在 on_predict_postprocess_end
            # 中先于官方 tracker 回调运行，只保留目标 class_id 的检查框。
            # ═══════════════════════════════════════════════════
            infer_t0 = time.perf_counter()
            try:
                with self.model_lock:
                    results = self.model.track(
                        source=image_bgr,
                        persist=True,
                        conf=conf,
                        iou=iou_val,
                        imgsz=imgsz,
                        tracker=tracker_cfg,
                        verbose=False,
                    )
            except Exception:
                with self.model_lock:
                    results = self.model.predict(
                        source=image_bgr,
                        conf=conf,
                        iou=iou_val,
                        imgsz=imgsz,
                        verbose=False,
                    )
            infer_ms = _ms(time.perf_counter() - infer_t0)
            timings["model_track_ms"] = infer_ms
            # ── 从 predictor 回读 tracker 内部各阶段耗时 ──
            try:
                pred = self.model.predictor
                timings["track_on_predict_start_ms"] = float(getattr(pred, "_track_timing_on_predict_start_ms", 0.0))
                timings["track_compute_extras_ms"] = float(getattr(pred, "_track_timing_compute_extras_ms", 0.0))
                timings["track_det_copy_ms"] = float(getattr(pred, "_track_timing_det_copy_ms", 0.0))
                timings["track_tracker_update_ms"] = float(getattr(pred, "_track_timing_tracker_update_ms", 0.0))
                timings["track_response_update_ms"] = float(getattr(pred, "_track_timing_response_update_ms", 0.0))
                timings["track_postprocess_end_ms"] = float(getattr(pred, "_track_timing_on_predict_postprocess_end_ms", 0.0))
            except Exception:
                pass
            # ── model.track() 返回后开始计时：class 过滤 + 后续处理 ──
            filter_t0 = time.perf_counter()

            result = results[0] if results else None
            boxes = result.boxes if result is not None else None
            speed = getattr(result, "speed", None) or {}
            timings["yoloe_preprocess_ms"] = float(speed.get("preprocess", 0.0))
            timings["yoloe_inference_ms"] = float(speed.get("inference", 0.0))
            timings["yoloe_postprocess_ms"] = float(speed.get("postprocess", 0.0))
            timings["total_ms"] = _ms(time.perf_counter() - total_t0)

            all_count = len(boxes) if boxes is not None else 0
            timings["all_predicted_count"] = float(all_count)

            # ── 无检测 ──
            if boxes is None or len(boxes) == 0:
                timings["candidate_count"] = 0.0
                self.latest_result = self._lost(stamp, "no_detections", timings)
                self._log_frame(label, timings)
                self._track_logger.log(self._build_log_record(send_seq, timings))
                return dict(self.latest_result)

            # ── 按 class_id 过滤（tracker 已处理过滤后的框，这里只需确认目标类别存在）──
            if boxes.cls is not None and len(boxes.cls) > 0:
                cls_arr = boxes.cls.int().cpu().numpy()
                target_indices = (cls_arr == class_id).nonzero()[0]
            else:
                target_indices = []

            timings["filter_class_ms"] = _ms(time.perf_counter() - filter_t0)
            timings["candidate_count"] = float(len(target_indices))
            timings["all_candidate_count"] = float(all_count)

            if len(target_indices) == 0:
                self.latest_result = self._lost(stamp, f"no_class_{label}", timings)
                self._log_frame(label, timings)
                self._track_logger.log(self._build_log_record(send_seq, timings))
                return dict(self.latest_result)

            identify_ms = 0.0
            # ── VLM init_bbox → 匹配目标 track_id ──
            if req.init_bbox is not None:
                clipped = _clip_bbox(req.init_bbox, image_bgr)
                if clipped is not None:
                    id_t0 = time.perf_counter()
                    matched_id = self._identify_target(boxes, target_indices, clipped)
                    identify_ms += _ms(time.perf_counter() - id_t0)
                    timings["identify_ms"] = identify_ms
                    if matched_id is not None:
                        self.target_track_id = matched_id
                        self.state = "active"
                        print(
                            f"[YOLOE_TRT] init_bbox matched track_id={self.target_track_id} "
                            f"label={label!r}",
                            flush=True,
                        )
                    else:
                        self.latest_result = self._lost(stamp, "init_bbox_no_match", timings)
                        self._log_frame(label, timings)
                        self._track_logger.log(self._build_log_record(send_seq, timings))
                        return dict(self.latest_result)
                else:
                    self.latest_result = self._lost(stamp, "init_bbox_out_of_bounds", timings)
                    self._log_frame(label, timings)
                    self._track_logger.log(self._build_log_record(send_seq, timings))
                    return dict(self.latest_result)

            timings["identify_ms"] = identify_ms
            # ── 按 target_track_id 筛选 ──
            if self.target_track_id is None:
                self.latest_result = self._lost(stamp, "awaiting_init_bbox", timings)
                self._log_frame(label, timings)
                self._track_logger.log(self._build_log_record(send_seq, timings))
                return dict(self.latest_result)

            id_t0 = time.perf_counter()
            target = self._find_target_by_id(boxes, target_indices)
            identify_ms += _ms(time.perf_counter() - id_t0)
            timings["identify_ms"] = identify_ms
            if target is None:
                self.latest_result = self._lost(stamp, "target_missing", timings)
                self._log_frame(label, timings)
                self._track_logger.log(self._build_log_record(send_seq, timings))
                return dict(self.latest_result)

            # ── 目标已锁定 ──
            self.last_bbox = target["bbox"]
            self.last_score = target["score"]
            self.state = "active"

            rb_t0 = time.perf_counter()
            self.latest_result = self._make_response(
                ok=True,
                stamp=stamp,
                bbox=self.last_bbox,
                track_id=self.target_track_id,
                score=self.last_score,
                cls=class_id,
                reason="",
                timings=timings,
            )
            timings["response_build_ms"] = _ms(time.perf_counter() - rb_t0)
            # 响应中的 timings 也补上 response_build_ms，与日志保持一致
            self.latest_result["timings"] = dict(timings)
            self._log_frame(label, timings)
            self._track_logger.log(self._build_log_record(send_seq, timings))
            return dict(self.latest_result)

    def _build_log_record(self, send_seq: int, timings: dict[str, float]) -> dict[str, Any]:
        """构造结构化 JSON 日志记录：send_seq/frame_seq/state/label + 全量 timings + CUDA/GC 快照。"""
        record: dict[str, Any] = {
            "send_seq": int(send_seq),
            "frame_seq": int(self.frame_seq),
            "state": self.state,
            "label": self.current_label,
            "reason": str(self.latest_result.get("reason", "")),
            "timings": dict(timings),
            "cuda_memory": cuda_memory_snapshot(),
        }
        # GC 快照开销略高，每 50 帧采集一次
        if self.frame_seq % 50 == 0:
            record["gc_state"] = gc_snapshot()
        return record

    # ── 目标识别 ──

    def _identify_target(self, boxes, indices, init_bbox: list[int]) -> int | None:
        """从 tracker 输出中找到与 VLM init_bbox IoU 最高的 track_id。"""
        if boxes.id is None or len(indices) == 0:
            return None
        track_ids = boxes.id.int().cpu().tolist()
        best_iou, best_id = 0.0, None
        for i in indices:
            bbox = boxes.xyxy[i].cpu().tolist()
            iou_val = _bbox_iou(bbox, init_bbox)
            if iou_val > best_iou:
                best_iou, best_id = iou_val, int(track_ids[i])
        return best_id if best_id is not None and best_iou >= self.init_bbox_match_iou else None

    def _find_target_by_id(self, boxes, indices) -> dict | None:
        """在目标类别中按 track_id 查找。"""
        if self.target_track_id is None or boxes.id is None:
            return None
        track_ids = boxes.id.int().cpu().tolist()
        for i in indices:
            if int(track_ids[i]) == int(self.target_track_id):
                bbox = boxes.xyxy[i].cpu().tolist()
                confs = boxes.conf
                score = float(confs[i]) if confs is not None and len(confs) > i else 1.0
                return {"bbox": [int(round(v)) for v in bbox[:4]], "score": score}
        return None

    # ── 响应构造 ──

    def _make_response(
        self,
        *,
        ok: bool,
        stamp: float = 0.0,
        bbox: list[int] | None = None,
        track_id: int | None = None,
        score: float = 0.0,
        cls: int | None = None,
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
            "cls": cls,
            "stamp": float(stamp),
            "frame_seq": int(self.frame_seq),
            "source": f"yoloe_trt:{self.current_tracker}",
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

    def _log_frame(self, label: str, timings: dict[str, float]) -> None:
        # 每 10 帧详细打印，含 CUDA 内存
        if self.frame_seq % 10 == 0:
            try:
                import torch
                _a = torch.cuda.memory_allocated(0) // 1024 // 1024
                _r = torch.cuda.memory_reserved(0) // 1024 // 1024
                _mem = f"gpu_alloc={_a}MB gpu_reserved={_r}MB"
            except Exception:
                _mem = "gpu=N/A"
            d = timings.get("decode_ms", 0)
            tr = timings.get("model_track_ms", 0)
            p = timings.get("yoloe_inference_ms", 0)
            to = timings.get("total_ms", 0)
            po = to - tr - d if to > tr + d else 0
            ad = int(timings.get("all_predicted_count", 0))
            ca = int(timings.get("candidate_count", 0))
            print(
                f"[YOLOE_TRT_PERF] frame={self.frame_seq} "
                f"decode={d:.1f} track={tr:.1f} yolo={p:.1f} post={po:.1f} total={to:.1f} "
                f"det={ad} cand={ca} state={self.state} {_mem}",
                flush=True,
            )

    # ── API 管理 ──

    def reset(self, reason: str = "") -> dict[str, Any]:
        with self.lock:
            self.model.predictor = None
            gc.collect()
            if _uses_cuda_device(self.device):
                torch.cuda.synchronize()
                torch.cuda.empty_cache()
            self.current_label = ""
            self.current_class_id = None
            self.target_track_id = None
            self.last_bbox = None
            self.last_score = 0.0
            self.state = "idle"
            self.frame_seq = 0
            self.latest_result = self._make_response(ok=False, reason=reason or "reset")
            print(f"[YOLOE_TRT] manual reset reason={reason!r}", flush=True)
            return dict(self.latest_result)

    def status(self) -> dict[str, Any]:
        with self.lock:
            return {
                "state": self.state,
                "label": self.current_label,
                "class_id": self.current_class_id,
                "tracker": self.current_tracker,
                "track_id": self.target_track_id,
                "last_bbox": self.last_bbox,
                "last_score": self.last_score,
                "frame_seq": self.frame_seq,
                "engine": str(self.engine_path),
            }

    def latest(self) -> dict[str, Any]:
        with self.lock:
            return dict(self.latest_result)


# ──────────────────────────────────────────────
# FastAPI app
# ──────────────────────────────────────────────

def create_app(engine: YoloeTensorRtTrackEngine) -> FastAPI:
    app = FastAPI(title="YOLOE TensorRT Track Engine v2")

    @app.post("/track")
    def track(req: TrackRequest):
        recv_ts = time.perf_counter()
        try:
            call_ts = time.perf_counter()
            result = engine.track(req)
            # recv_ms：从收到请求到开始调用 track() 的处理/排队耗时
            result["recv_ms"] = _ms(call_ts - recv_ts)
            # send_seq 一一对应验证：结果中若携带 send_seq 则回传，否则回传请求值
            result["echo_send_seq"] = result.get("send_seq", req.send_seq)
            return result
        except Exception as exc:
            import traceback
            traceback.print_exc()
            print(f"YOLOE TRT track API request failed: {exc}", flush=True)
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

def _parse_imgsz(value: str) -> int | tuple[int, int]:
    """解析 imgsz，支持 640 或 480,640 两种形式。"""
    if isinstance(value, int):
        return value
    vals = [int(v.strip()) for v in str(value).replace("x", ",").split(",") if v.strip()]
    if len(vals) == 1:
        return vals[0]
    if len(vals) == 2:
        return (vals[0], vals[1])
    raise ValueError(f"invalid imgsz: {value!r}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="YOLOE TensorRT Track Engine v2 (official model.track)")
    parser.add_argument("--pt-model", default=str(YOLOE_ROOT / "yoloe-v8m-seg.pt"),
                        help="PyTorch 模型路径（用于导出 engine）")
    parser.add_argument("--engine", default=str(YOLOE_ROOT / "pretrain/yoloe-26n-seg.engine"),
                        help="TensorRT engine 路径")
    parser.add_argument("--classes", default=str(YOLOE_ROOT / "prompt/prompt.txt"),
                        help="固定词表文件路径")
    parser.add_argument("--tracker-dir", default=str(YOLOE_ROOT / "ultralytics/cfg/trackers"))
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--conf", type=float, default=0.1)
    parser.add_argument("--iou", type=float, default=0.5)
    parser.add_argument("--imgsz", type=_parse_imgsz, default=640)
    parser.add_argument("--engine-imgsz", type=_parse_imgsz, default=640,
                        help="导出 engine 时使用的输入尺寸（须与 engine 一致）")
    parser.add_argument("--rebuild-engine", action="store_true",
                        help="强制重新导出 TensorRT engine")
    parser.add_argument("--init-bbox-match-iou", type=float, default=0.1,
                        help="VLM init_bbox 与 tracker 输出匹配的最小 IoU 阈值")
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
        imgsz=args.imgsz,
        engine_imgsz=args.engine_imgsz,
        rebuild_engine=args.rebuild_engine,
        init_bbox_match_iou=args.init_bbox_match_iou,
    )
    app = create_app(engine)
    uvicorn.run(app, host=args.host, port=args.port, reload=False, workers=1)


if __name__ == "__main__":
    main()

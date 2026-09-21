#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""YOLOE TensorRT 固定词表目标跟踪 API（v3：显式会话协议 + 显式管线）。

与 api.py（动态 text prompt）的区别：
- 使用预导出的 TensorRT engine，词表在启动时固定
- 不支持运行时 set_classes()，label 必须在固定词表中
- 显式会话协议（/session/start、/track、/session/rebind、/session/close）：
  会话身份 + 帧序号/时钟单调校验 + operation_id/request_id 幂等 + 身份版本
- 显式管线：detect（进程级 predictor 复用）→ 目标类别过滤 → tracker.update
  （会话级 tracker，每接受帧恰好一次）→ 身份筛选；不使用模型回调
"""

from __future__ import annotations

import argparse
import base64
import gc
import hashlib
import os
import sys
import threading
import time
import uuid
from collections import OrderedDict, deque
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import torch
import uvicorn
from fastapi import FastAPI, Request
from pydantic import BaseModel, Field


YOLOE_ROOT = Path(__file__).resolve().parents[1]
if str(YOLOE_ROOT) not in sys.path:
    sys.path.insert(0, str(YOLOE_ROOT))

# 结构化时序日志（依赖 track_engine 包，须在 YOLOE_ROOT 入 sys.path 之后导入）
from track_engine.tracking_logger import TrackingLogger, cuda_memory_snapshot, gc_snapshot  # noqa: E402

torch.backends.cudnn.enabled = False
torch.backends.cuda.matmul.allow_tf32 = True

from ultralytics import YOLO  # noqa: E402
from ultralytics.trackers.track import TRACKER_MAP  # noqa: E402
from ultralytics.utils import YAML, IterableSimpleNamespace  # noqa: E402


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

class ProtocolRequest(BaseModel):
    class Config:
        # 不静默接受旧版 reset/init_bbox 等字段，浮点时间和坐标必须有限。
        extra = "forbid"
        allow_inf_nan = False


class SessionStartRequest(ProtocolRequest):
    """创建 tracking 会话（幂等键：operation_id）。

    首帧必须携带与图像同帧的 init_bbox 用于匹配目标 track_id。
    """

    operation_id: str = Field(..., min_length=1, description="start 幂等操作 ID")
    server_instance_id: str = Field(..., description="从 status 同步的服务实例 ID")
    expected_epoch: int = Field(..., ge=0, description="从 status 同步的会话世代，防止迟到 start 复活")
    label: str = Field(..., min_length=1, description="固定词表中的目标类别名")
    tracker: str = Field("botsort", description="tracker 类型")
    conf: float | None = Field(None, ge=0.0, le=1.0)
    iou: float | None = Field(None, ge=0.0, le=1.0)
    image_base64: str = Field(..., description="首帧图像 base64")
    init_bbox: list[float] = Field(..., min_items=4, max_items=4, description="首帧同帧 VLM bbox（必填）")
    stamp: float = Field(..., gt=0, description="首帧图像时间戳")
    frame_seq: int = Field(1, ge=1, description="客户端帧序号，首帧必须为 1")


class SessionTrackRequest(ProtocolRequest):
    """普通跟踪帧：不允许附带 init_bbox / reset / label（协议禁止隐式重绑与建会话）。"""

    server_instance_id: str = Field(..., description="服务实例 ID")
    session_id: str = Field(..., description="会话 ID")
    request_id: str = Field(..., min_length=1, description="/track 去重键（重复请求返回缓存，绝不二次更新）")
    frame_seq: int = Field(..., ge=1, description="客户端帧序号（会话内严格递增）")
    stamp: float = Field(..., gt=0, description="图像时间戳（同一时钟域内严格递增）")
    image_base64: str = Field(..., description="当前帧图像 base64")
    conf: float | None = Field(None, ge=0.0, le=1.0)
    iou: float | None = Field(None, ge=0.0, le=1.0)
    send_seq: int = Field(0, description="兼容字段：客户端调试序列号，仅回显")


class SessionRebindRequest(ProtocolRequest):
    """会话内身份重绑（幂等键：operation_id）。

    同帧证据定位：按 VLM 候选框对应图像的 stamp 在服务端帧历史缓存中定位该帧，
    在该帧候选框中匹配 bbox，并验证候选身份到当前帧仍连续可信；唯一可信匹配才
    原子提交（identity_revision 递增）；歧义/无匹配/历史过期拒绝且不改动当前身份。
    """

    server_instance_id: str = Field(..., description="服务实例 ID")
    session_id: str = Field(..., description="会话 ID")
    operation_id: str = Field(..., min_length=1, description="rebind 幂等操作 ID")
    identity_revision: int = Field(..., ge=0, description="客户端当前已提交身份版本")
    bbox: list[float] = Field(..., min_items=4, max_items=4, description="候选目标框 xyxy")
    stamp: float = Field(..., gt=0, description="VLM 候选框对应的精确图像时间戳")
    frame_seq: int = Field(..., ge=1, description="候选框来源帧引用（与 stamp 一起精确校验）")


class SessionCloseRequest(ProtocolRequest):
    """关闭指定会话（幂等键：operation_id）。旧会话的 close 不影响新会话。"""

    server_instance_id: str = Field(..., description="服务实例 ID")
    session_id: str = Field(..., description="要关闭的会话 ID")
    operation_id: str = Field(..., min_length=1, description="close 幂等操作 ID")
    reason: str = Field("", description="关闭原因")
    pending_start_operation_id: str = ""
    expected_epoch: int | None = None




# ──────────────────────────────────────────────
# Engine
# ──────────────────────────────────────────────

class YoloeTensorRtTrackEngine:
    """YOLOE TensorRT 固定词表单目标跟踪引擎（v3）。"""

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
        frame_history_cap: int = 64,
        frame_history_ttl: float = 8.0,
        rebind_unique_margin: float = 0.1,
        op_cache_ttl: float = 120.0,
        op_cache_max: int = 128,
        max_queue_age: float = 5.0,
        identity_confirm_frames: int = 2,
        log_every: int = 10,
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

        self.lock = threading.RLock()
        self.model_lock = threading.Lock()
        self._ensure_engine(rebuild=bool(rebuild_engine))
        self.model = self._load_engine()
        self._warmup_engine()
        # 显式管线不使用模型回调：清除可能残留的 tracker 回调，
        # 保证 predict() 只做检测、tracker.update 只由服务端显式调用
        self._clear_tracker_callbacks()

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
        # 推理/tracker 更新异常后置位：状态原子性无法确认，拒绝普通帧直到显式 reset
        self.needs_reinitialize: bool = False

        # ── 显式会话协议状态（全部状态修改在 self.lock 内进行）──
        # 服务实例 ID：进程启动时随机生成，客户端据此检测服务重启并重新同步
        self.server_instance_id: str = uuid.uuid4().hex
        # 会话 ID：关闭后保留用于归属错误回显；session_active 表示会话是否存活
        self.session_id: str | None = None
        self.session_active: bool = False
        # 身份版本：rebind 原子提交成功后递增；客户端据此拒绝旧版本在途结果
        self.identity_revision: int = 0
        self.epoch = 0
        self.start_operation_id = ""
        self.max_queue_age = max(0.1, float(max_queue_age))
        self.identity_confirm_frames = max(1, int(identity_confirm_frames))
        self.log_every = max(1, int(log_every))
        self.rebind_unique_margin = max(0.0, float(rebind_unique_margin))
        self._continuity = {}
        self._segment_seq = 0
        self._target_segment = None
        # 帧顺序状态：会话内只接受严格递增 frame_seq 与单调 stamp
        self.last_frame_seq: int = 0
        self.last_stamp: float = 0.0
        # 最近接受帧的目标类别检测快照（track_id/bbox/score），供 rebind 匹配
        self._frame_tracks_snapshot: dict[str, Any] = {}
        # 帧历史缓存：按 (stamp, frame_seq, entries) 存储每帧目标类别检测，
        # 供 rebind 做同帧证据定位与历史身份解析；条数 + 墙钟 TTL 双上限有界
        self._frame_history: deque = deque(maxlen=max(1, int(frame_history_cap)))
        self._frame_history_ttl = max(0.1, float(frame_history_ttl))
        # 幂等缓存：operation_id（start/close/rebind）与 request_id（track）→ 缓存结果
        # 有界（_op_cache_max 条）+ TTL（_op_cache_ttl 秒）淘汰；同 ID 不同指纹拒绝
        self._op_cache: OrderedDict[str, dict[str, Any]] = OrderedDict()
        self._track_cache: OrderedDict[str, dict[str, Any]] = OrderedDict()
        self._op_cache_ttl = max(1.0, float(op_cache_ttl))
        self._op_cache_max = max(1, int(op_cache_max))

        # 手动 tracker 实例（跨帧维持状态，只跟踪 target class 的检测框）
        # 会话级 tracker：随会话重建（新实例不携带旧 Kalman/GMC/外观平滑/ID 计数状态）；
        # 检测 predictor 与 ReID encoder 为进程级资源，跨会话复用（方案 §6.2）
        self._pipeline_tracker: Any = None
        self._pipeline_tracker_cfg: Any = None

        # ── 结构化日志 ──
        self._track_logger = TrackingLogger.get("tracker_server")

    # ── 回调清理与 tracker 配置（显式管线，不注册模型回调）──

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

    def _load_tracker_cfg(self, tracker_name: str) -> Any:
        """加载 tracker 配置并处理 ReID auto 回退（等价官方 on_predict_start 逻辑）。

        TRT engine 后端无法注册 forward hook 提取检测头特征，
        with_reid 且 model=auto 时回退外部 ReID 模型（yolo26n-cls.pt）；
        encoder 实例由 reid.build_encoder 的进程级缓存复用。
        """
        cfg = IterableSimpleNamespace(**YAML.load(self._tracker_cfg_path(tracker_name)))
        cfg.device = self.device  # ReID encoder 运行在服务设备上
        if (
            cfg.tracker_type in {"botsort", "tracktrack", "deepocsort"}
            and cfg.with_reid
            and cfg.model == "auto"
        ):
            cfg.model = "yolo26n-cls.pt"
        return cfg

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
        if tracker not in {"botsort", "bytetrack", "deepocsort", "ocsort"}:
            raise ValueError(f"unsupported tracker: {tracker}")
        cfg = self.tracker_dir / f"{tracker}.yaml"
        if not cfg.exists():
            raise FileNotFoundError(f"tracker config not found: {cfg}")
        return str(cfg)

    # ── 会话协议：校验、幂等与错误构造 ──

    def _protocol_error(
        self,
        code: str,
        *,
        detail: str = "",
        session_id: str | None = None,
        frame_seq: int | None = None,
    ) -> dict[str, Any]:
        """构造协议层错误响应（HTTP 200 + error 码，客户端统一解析，不触发 HTTP 异常）。"""
        resp = {
            "ok": False,
            "error": str(code),
            "reason": str(detail or code),
            "server_instance_id": self.server_instance_id,
            "session_id": session_id if session_id is not None else self.session_id,
            "identity_revision": int(self.identity_revision),
            "frame_seq": int(frame_seq if frame_seq is not None else self.last_frame_seq),
            "wall_time": _now(),
        }
        print(
            f"[YOLOE_TRT] protocol reject: code={code} detail={detail!r} session={resp['session_id']}",
            flush=True,
        )
        return resp

    def _check_instance(self, server_instance_id: str) -> str | None:
        """实例校验：服务重启后旧客户端请求必须经 /status 重新同步。返回 None 表示通过。"""
        if not server_instance_id or str(server_instance_id) != self.server_instance_id:
            return "instance_mismatch"
        return None

    def _check_session(self, server_instance_id: str, session_id: str) -> tuple[str | None, dict[str, Any] | None]:
        """实例 + 会话校验（锁内）。返回 (错误码, 错误响应)；通过时 (None, None)。"""
        inst_err = self._check_instance(server_instance_id)
        if inst_err:
            return inst_err, self._protocol_error(inst_err, session_id=session_id)
        if str(session_id) != str(self.session_id or ""):
            # 旧会话请求（含晚到的旧 update/close）：不影响当前会话，明确拒绝
            return "session_mismatch", self._protocol_error("session_mismatch", session_id=session_id)
        if not self.session_active:
            # 会话已关闭（close / 时钟回退 / 异常标记）
            code = "needs_reinitialize" if self.needs_reinitialize else "session_closed"
            return code, self._protocol_error(code, session_id=session_id)
        return None, None

    def _check_frame_order(self, frame_seq: int, stamp: float) -> tuple[str, str] | None:
        """帧顺序校验（锁内）。通过时推进 last_frame_seq/last_stamp 并返回 None，否则返回 (错误码, detail)。

        采集时钟回退直接终止会话（session_active=False），客户端需重新建会话。
        """
        frame_seq = int(frame_seq)
        if frame_seq < self.last_frame_seq:
            return ("out_of_order", f"frame_seq={frame_seq} < last={self.last_frame_seq}")
        if frame_seq == self.last_frame_seq:
            return ("stale_frame", f"frame_seq={frame_seq} == last={self.last_frame_seq}")
        if self.last_stamp > 0.0 and stamp > 0.0 and float(stamp) < self.last_stamp - 1e-6:
            self.session_active = False
            return ("clock_regression", f"stamp={stamp:.6f} < last={self.last_stamp:.6f}")
        if self.last_stamp > 0.0 and stamp <= self.last_stamp:
            return ("stale_frame", "image stamp must strictly increase")
        self.last_frame_seq = frame_seq
        if stamp > 0.0:
            self.last_stamp = float(stamp)
        return None

    def _op_cache_get(self, cache: OrderedDict, key: str) -> dict[str, Any] | None:
        """读取未过期的幂等缓存结果；过期条目就地淘汰。"""
        entry = cache.get(key)
        if entry is None:
            return None
        if (time.monotonic() - float(entry.get("wall", 0.0))) > self._op_cache_ttl:
            cache.pop(key, None)
            return None
        return entry

    def _op_cache_put(self, cache: OrderedDict, key: str, result: dict[str, Any], fingerprint) -> None:
        """写入幂等缓存并按容量淘汰（最旧优先），保证缓存有界。"""
        cache[key] = {"result": result, "wall": time.monotonic(), "fp": fingerprint}
        while len(cache) > self._op_cache_max:
            cache.popitem(last=False)

    @staticmethod
    def _fingerprint(req):
        """指纹涵盖接口类型和完整请求，避免同 ID 换图像/bbox 后仍命中缓存。"""
        return hashlib.sha256((type(req).__name__ + req.json()).encode()).hexdigest()

    def _push_frame_history(self, snapshot):
        """每一张接受帧都记录（包括空检测），身份中断后重新分配连续性段。"""
        now = time.monotonic()
        current = {}
        for entry in snapshot["entries"]:
            tid = entry["track_id"]
            previous = self._continuity.get(tid)
            if previous is None:
                self._segment_seq += 1
            segment, count = (previous[0], previous[1] + 1) if previous else (self._segment_seq, 1)
            entry["segment"] = segment
            entry["confirmed_frames"] = count
            current[tid] = (segment, count)
        self._continuity = current
        snapshot["monotonic"] = now
        self._frame_history.append(snapshot)
        self._expire_history(now)

    def _expire_history(self, now):
        while self._frame_history and now - self._frame_history[0]["monotonic"] > self._frame_history_ttl:
            self._frame_history.popleft()

    def _find_history(self, frame_seq, stamp):
        self._expire_history(time.monotonic())
        return next((snap for snap in self._frame_history
                     if snap["frame_seq"] == frame_seq and snap["stamp"] == stamp), None)

    # ── 会话协议操作（HTTP 入口串行校验与执行）──

    def _close_session_locked(self) -> None:
        """锁内关闭当前会话：释放 tracker 状态，保留检测和 ReID 资源。

        session_id 保留用于后续请求归属回显（session_closed/session_mismatch）。
        """
        self.session_active = False
        self.needs_reinitialize = False
        self.state = "idle"
        self.target_track_id = None
        self.last_bbox = None
        self.last_score = 0.0
        self.current_label = ""
        self.current_class_id = None
        self._frame_tracks_snapshot = {}
        self._frame_history.clear()
        self._continuity.clear()
        self._target_segment = None
        self.epoch += 1
        # 会话级 tracker 随会话释放；检测 predictor（TRT engine）保留复用，
        # 正常停止不丢弃检测资源，已移除绕过会话身份的无条件 /reset。
        self._pipeline_tracker = None
        self._pipeline_tracker_cfg = None

    def session_start(self, req: SessionStartRequest) -> dict[str, Any]:
        """创建会话并执行首帧：同帧 init_bbox 匹配目标身份。幂等（operation_id）。"""
        timings: dict[str, float] = {}
        total_t0 = time.perf_counter()
        t0 = time.perf_counter()
        image_bgr = _decode_image_base64(req.image_base64)
        timings["decode_ms"] = _ms(time.perf_counter() - t0)
        # HTTP 入口已持有事务锁；解析词表和 tracker 配置。
        label, class_id = self._resolve_label(req.label)
        tracker_name = str(req.tracker or "botsort").strip().lower() or "botsort"
        self._tracker_cfg_path(tracker_name)  # 仅校验配置存在，cfg 在锁内加载
        conf = float(req.conf if req.conf is not None else self.default_conf)
        iou_val = float(req.iou if req.iou is not None else self.default_iou)
        stamp = float(req.stamp if req.stamp is not None else _now())
        fingerprint = self._fingerprint(req)

        lock_t0 = time.perf_counter()
        with self.lock:
            timings["lock_wait_ms"] = _ms(time.perf_counter() - lock_t0)
            # ① 幂等：同 operation_id 重放返回缓存结果；同 ID 不同内容拒绝
            cached = self._op_cache_get(self._op_cache, req.operation_id)
            if cached is not None:
                if cached.get("fp") != fingerprint:
                    return self._protocol_error(
                        "operation_conflict", detail="same operation_id with different content"
                    )
                return dict(cached["result"])
            # ② 实例校验：客户端已知实例不匹配（服务重启）→ 拒绝，需重新同步
            if req.server_instance_id and str(req.server_instance_id) != self.server_instance_id:
                resp = self._protocol_error("instance_mismatch")
                self._op_cache_put(self._op_cache, req.operation_id, resp, fingerprint)
                return resp
            if req.expected_epoch != self.epoch:
                return self._protocol_error("epoch_mismatch")
            # ③ 会话占用检查：健康活动会话拒绝隐式覆盖；needs_reinitialize 死会话允许重建
            if self.session_active:
                resp = self._protocol_error(
                    "session_active",
                    detail="close the active session before starting a new one",
                    session_id=self.session_id,
                )
                self._op_cache_put(self._op_cache, req.operation_id, resp, fingerprint)
                return resp
            # ④ 创建会话：只重建 tracker/GMC/身份/历史缓存；检测 predictor 与
            # ReID encoder 为进程级资源跨会话复用（方案 §6.2），
            # 不以 predictor=None 作为正常停止/重启手段
            self.session_id = uuid.uuid4().hex
            self.start_operation_id = req.operation_id
            self.epoch += 1
            self._frame_history.clear()
            self._continuity.clear()
            self._segment_seq = 0
            self._target_segment = None
            self.session_active = True
            self.identity_revision = 0
            self.last_frame_seq = 0
            self.last_stamp = 0.0
            self.needs_reinitialize = False
            self.session_conf, self.session_iou = conf, iou_val
            tracker_cfg_ns = self._load_tracker_cfg(tracker_name)
            self._pipeline_tracker = TRACKER_MAP[tracker_cfg_ns.tracker_type](args=tracker_cfg_ns)
            self._pipeline_tracker_cfg = tracker_cfg_ns
            self.current_label = label
            self.current_class_id = class_id
            self.current_tracker = tracker_name
            self.target_track_id = None
            self.last_bbox = None
            self.last_score = 0.0
            self.state = "acquiring"
            self._frame_tracks_snapshot = {}
            # ⑤ 首帧顺序校验
            order_err = self._check_frame_order(int(req.frame_seq), stamp)
            if order_err is not None:
                code, detail = order_err
                self.session_active = False
                resp = self._protocol_error(
                    code, detail=detail, session_id=self.session_id, frame_seq=int(req.frame_seq)
                )
                self._op_cache_put(self._op_cache, req.operation_id, resp, fingerprint)
                return resp
            print(
                f"[YOLOE_TRT] session start sid={self.session_id[:8]} label={label!r} "
                f"class_id={class_id} tracker={tracker_name} op={req.operation_id}",
                flush=True,
            )
            # ⑥ 首帧推理 + init_bbox 身份匹配
            try:
                result = self._run_frame(
                    image_bgr,
                    stamp=stamp,
                    conf=conf,
                    iou_val=iou_val,
                    timings=timings,
                    total_t0=total_t0,
                    init_bbox=req.init_bbox,
                    log_seq=int(req.frame_seq),
                )
            except Exception as exc:
                # 推理/首帧匹配异常：状态原子性无法确认，关闭会话并标记待重初始化
                import traceback
                traceback.print_exc()
                self.needs_reinitialize = True
                self.session_active = False
                self.state = "needs_reinitialize"
                resp = self._protocol_error("needs_reinitialize", detail=str(exc), session_id=self.session_id)
                self._op_cache_put(self._op_cache, req.operation_id, resp, fingerprint)
                return resp
            if not result.get("ok", False):
                # 首帧未匹配到目标（init_bbox_no_match 等）：会话无意义，服务端直接关闭
                self.session_active = False
                self.state = "idle"
                print(
                    f"[YOLOE_TRT] session start failed ({result.get('reason')}), "
                    f"closed sid={self.session_id[:8]}",
                    flush=True,
                )
            self._op_cache_put(self._op_cache, req.operation_id, result, fingerprint)
            return dict(result)

    def session_track(self, req: SessionTrackRequest) -> dict[str, Any]:
        """普通跟踪帧：实例/会话/顺序校验 → request_id 去重 → 推理 → 目标筛选。"""
        timings: dict[str, float] = {}
        total_t0 = time.perf_counter()
        t0 = time.perf_counter()
        image_bgr = _decode_image_base64(req.image_base64)
        timings["decode_ms"] = _ms(time.perf_counter() - t0)
        conf = float(req.conf if req.conf is not None else self.default_conf)
        iou_val = float(req.iou if req.iou is not None else self.default_iou)
        stamp = float(req.stamp if req.stamp is not None else _now())

        lock_t0 = time.perf_counter()
        with self.lock:
            timings["lock_wait_ms"] = _ms(time.perf_counter() - lock_t0)
            # ① 实例 + 会话校验
            err, resp = self._check_session(req.server_instance_id, req.session_id)
            if err:
                return resp
            # ② request_id 去重：重复请求返回缓存结果，绝不二次更新 tracker
            cached = self._op_cache_get(self._track_cache, req.request_id)
            if cached is not None:
                if cached.get("fp") != self._fingerprint(req):
                    return self._protocol_error(
                        "operation_conflict",
                        detail="request_id reused with different frame_seq",
                        session_id=req.session_id,
                        frame_seq=int(req.frame_seq),
                    )
                return dict(cached["result"])
            # ③ 帧顺序校验（重复/乱序拒绝；时钟回退同时终止会话）
            order_err = self._check_frame_order(int(req.frame_seq), stamp)
            if order_err is not None:
                code, detail = order_err
                resp = self._protocol_error(
                    code, detail=detail, session_id=req.session_id, frame_seq=int(req.frame_seq)
                )
                if code != "clock_regression":
                    self._op_cache_put(self._track_cache, req.request_id, resp, self._fingerprint(req))
                return resp
            # ④ 推理 + 目标筛选（普通帧禁止 init_bbox，协议层已删除隐式重绑路径）
            try:
                result = self._run_frame(
                    image_bgr,
                    stamp=stamp,
                    conf=conf,
                    iou_val=iou_val,
                    timings=timings,
                    total_t0=total_t0,
                    init_bbox=None,
                    log_seq=int(req.send_seq or 0),
                )
            except Exception as exc:
                # 状态原子性无法确认：关闭会话 + 标记待重初始化（禁止隐式重试）
                import traceback
                traceback.print_exc()
                self.needs_reinitialize = True
                self.session_active = False
                self.state = "needs_reinitialize"
                return self._protocol_error("needs_reinitialize", detail=str(exc), session_id=req.session_id)
            self._op_cache_put(self._track_cache, req.request_id, result, self._fingerprint(req))
            return dict(result)

    def session_rebind(self, req: SessionRebindRequest) -> dict[str, Any]:
        """会话内身份重绑（历史身份解析）：同帧证据定位 → 候选匹配 → 连续性验证 → 原子提交。

        ① 按 VLM 图像 stamp 在帧历史缓存定位同帧证据，无证据 → history_expired；
        ② 在该历史帧候选框中匹配 bbox（IoU 门槛 + 第一/第二差距，歧义拒绝）；
        ③ 连续性验证：候选 ID 的连续性段一致，且当前连续观测数达到门槛；
        ④ 原子提交 target_track_id 并递增 identity_revision；
        ⑤ 响应只返回身份验证信息（bbox=None），历史结果不作为当前控制观测。
        """
        with self.lock:
            fingerprint = self._fingerprint(req)
            cached = self._op_cache_get(self._op_cache, req.operation_id)
            if cached is not None:
                if cached.get("fp") != fingerprint:
                    return self._protocol_error(
                        "operation_conflict", detail="same operation_id with different content",
                        session_id=req.session_id,
                    )
                return dict(cached["result"])
            err, resp = self._check_session(req.server_instance_id, req.session_id)
            if err:
                return resp
            # 身份版本一致性：两端版本不同步说明存在未确认的提交/回滚，拒绝重绑
            if int(req.identity_revision) != int(self.identity_revision):
                resp = self._protocol_error(
                    "identity_revision_mismatch",
                    detail=f"client={req.identity_revision} server={self.identity_revision}",
                    session_id=req.session_id,
                )
                self._op_cache_put(self._op_cache, req.operation_id, resp, fingerprint)
                return resp
            # 必须同时匹配服务端接受的 frame_seq 和 stamp，不保留最近帧兜底。
            hist = self._find_history(req.frame_seq, req.stamp)
            if hist is None:
                resp = self._protocol_error("history_expired", session_id=req.session_id)
                self._op_cache_put(self._op_cache, req.operation_id, resp, fingerprint)
                return resp
            # ② 在历史帧候选框中匹配 VLM bbox
            entries = list(hist.get("entries") or [])
            best_iou, best_entry, second_iou = 0.0, None, 0.0
            for entry in entries:
                iou_val = _bbox_iou(entry["bbox"], list(req.bbox))
                if iou_val > best_iou:
                    second_iou, best_iou = best_iou, iou_val
                    best_entry = entry
                elif iou_val > second_iou:
                    second_iou = iou_val
            unique_margin = self.rebind_unique_margin  # 歧义保护间隔（CLI 可配）
            ambiguous = second_iou >= self.init_bbox_match_iou and (best_iou - second_iou) < unique_margin
            if best_entry is None or best_iou < self.init_bbox_match_iou or ambiguous:
                resp = self._protocol_error(
                    "identity_unverified",
                    detail=f"best_iou={best_iou:.3f} second_iou={second_iou:.3f} candidates={len(entries)}",
                    session_id=req.session_id,
                )
                self._op_cache_put(self._op_cache, req.operation_id, resp, fingerprint)
                return resp
            # 同一数字 ID 丢失后恢复不能证明同一身份，必须连续性段也一致。
            current = self._continuity.get(int(best_entry["track_id"]))
            if (current is None or current[0] != best_entry["segment"]
                    or current[1] < self.identity_confirm_frames):
                resp = self._protocol_error("identity_unverified", session_id=req.session_id)
                self._op_cache_put(self._op_cache, req.operation_id, resp, fingerprint)
                return resp
            # ④ 原子提交：更新目标身份并递增版本（失败路径不进入这里，当前身份保持不变）
            self.target_track_id = int(best_entry["track_id"])
            self._target_segment = best_entry["segment"]
            self.identity_revision += 1
            self.state = "active"
            # ⑤ 身份专用响应：不携带 bbox（历史结果不作为当前控制观测，
            #    客户端等待下一张正常新帧输出）
            result = self._make_response(
                ok=True,
                stamp=float(hist.get("stamp") or 0.0),
                bbox=None,
                track_id=self.target_track_id,
                score=0.0,
                cls=self.current_class_id,
                reason="rebind_committed",
                timings={},
            )
            result["identity_revision"] = int(self.identity_revision)
            result["verified_frame_seq"] = int(hist.get("frame_seq") or 0)
            result["verified_stamp"] = float(hist.get("stamp") or 0.0)
            print(
                f"[YOLOE_TRT] rebind committed sid={str(req.session_id)[:8]} "
                f"track_id={self.target_track_id} revision={self.identity_revision} "
                f"verified_frame={result['verified_frame_seq']} iou={best_iou:.3f}",
                flush=True,
            )
            self._op_cache_put(self._op_cache, req.operation_id, result, fingerprint)
            return dict(result)

    def session_close(self, req: SessionCloseRequest) -> dict[str, Any]:
        """关闭指定会话（幂等）。旧会话的 close 不影响新会话。"""
        with self.lock:
            fingerprint = self._fingerprint(req)
            cached = self._op_cache_get(self._op_cache, req.operation_id)
            if cached is not None:
                if cached.get("fp") != fingerprint:
                    return self._protocol_error(
                        "operation_conflict", detail="same operation_id with different content",
                        session_id=req.session_id,
                    )
                return dict(cached["result"])
            base = {
                "ok": True,
                "server_instance_id": self.server_instance_id,
                "session_id": str(req.session_id),
                "identity_revision": int(self.identity_revision),
                "reason": str(req.reason or ""),
                "wall_time": _now(),
            }
            inst_err = self._check_instance(req.server_instance_id)
            if inst_err:
                # 服务已重启：目标会话必然不存在，视为已关闭（幂等安全）
                base["already_closed"] = True
                base["reason"] = f"instance_restarted:{req.reason}"
            elif (req.pending_start_operation_id and
                  ((self.start_operation_id == req.pending_start_operation_id)
                   or (not self.session_active and req.expected_epoch == self.epoch))):
                # start 响应丢失时，关闭已创建会话或提升 epoch 拒绝尚未执行的迟到 start。
                self._close_session_locked()
            elif str(req.session_id) != str(self.session_id or ""):
                base["noop"] = "stale_session"
            else:
                self._close_session_locked()
            base["epoch"] = self.epoch
            self._op_cache_put(self._op_cache, req.operation_id, base, fingerprint)
            return dict(base)

    # ── 帧执行核心 ──

    def _run_frame(
        self,
        image_bgr: np.ndarray,
        *,
        stamp: float,
        conf: float,
        iou_val: float,
        timings: dict[str, float],
        total_t0: float,
        init_bbox: list[float] | None = None,
        log_seq: int = 0,
    ) -> dict[str, Any]:
        """锁内执行一帧：检测 → 类别过滤 → tracker.update → 身份匹配/目标筛选 → 响应。

        显式管线（方案 §6.2）：复用进程级检测 predictor 与 ReID encoder，
        每个接受帧恰好调用一次 tracker.update；init_bbox 非 None 仅为会话
        首帧（同帧 VLM bbox 匹配目标身份）。异常向上抛出，由会话操作统一处理。
        """
        label = self.current_label
        class_id = self.current_class_id
        timings["yoloe_trt_gpu"] = float(1 if _uses_cuda_device(self.device) else 0)
        timings["yoloe_trt_device"] = self.device
        timings["yoloe_trt_engine"] = str(self.engine_path)

        # tracker 池诊断（只读）：生命周期统一由 tracker 内部管理，服务端不做破坏性裁剪
        diag_t0 = time.perf_counter()
        if self._pipeline_tracker is not None:
            diag = self._pipeline_tracker.get_diagnostics()
            timings["tracked_count"] = float(diag["tracked"])
            timings["lost_count"] = float(diag["lost"])
            timings["removed_count"] = float(diag["removed"])
            timings["max_lost_age_frames"] = float(diag["max_lost_age_frames"])
        timings["tracker_diag_ms"] = _ms(time.perf_counter() - diag_t0)

        # 日志/响应使用客户端帧序号（会话内严格递增）
        self.frame_seq = int(self.last_frame_seq)

        # ═══════════════════════════════════════════════════
        # 显式管线：detect（复用进程级 predictor，无 tracker 回调）→
        # 目标类别过滤 → tracker.update（每接受帧恰好一次）→ 身份筛选。
        # 数据流与官方 on_predict_postprocess_end 保持一致
        # （det = boxes.cpu().numpy()，img = orig_img，空检测同样推进一帧）。
        # ═══════════════════════════════════════════════════
        infer_t0 = time.perf_counter()
        with self.model_lock:
            results = self.model.predict(
                source=image_bgr,
                conf=conf,
                iou=iou_val,
                imgsz=self.default_imgsz,
                device=self.device,
                verbose=False,
            )
        timings["detect_infer_ms"] = _ms(time.perf_counter() - infer_t0)

        filter_t0 = time.perf_counter()
        result = results[0] if results else None
        boxes = result.boxes if result is not None else None
        # 目标类别过滤（原 _filter_boxes_for_target_class 回调的显式等价实现；
        # result[idx] 同步切片 boxes/masks，保持分割输出一致）
        if boxes is not None and len(boxes) > 0 and boxes.cls is not None:
            cls_mask = boxes.cls.int().cpu().numpy() == int(class_id)
            if not cls_mask.any():
                result = result[0:0]
            else:
                result = result[cls_mask.nonzero()[0]]
        # tracker.update：会话级 tracker 实例，更新耗时显式计时
        update_t0 = time.perf_counter()
        det = result.boxes.cpu().numpy()
        self._pipeline_tracker.protected_track_id = self.target_track_id
        tracks = self._pipeline_tracker.update(det, result.orig_img, stamp=stamp)
        diagnostics = self._pipeline_tracker.get_diagnostics()
        timings.update(diagnostics)
        timings.update(self._pipeline_tracker.reid_timings)
        timings.update(tracked_count=diagnostics["tracked"], lost_count=diagnostics["lost"],
                       removed_count=diagnostics["removed"])
        timings["track_tracker_update_ms"] = _ms(time.perf_counter() - update_t0)
        timings["filter_class_ms"] = _ms(time.perf_counter() - filter_t0)
        if len(tracks):
            # 按 tracker 输出行回填：最后一列为检测索引，同步重排检测框
            track_idx = tracks[:, -1].astype(int)
            result = result[track_idx]
            result.update(boxes=torch.as_tensor(tracks[:, :-1], device=result.boxes.data.device))
        timings["model_track_ms"] = _ms(time.perf_counter() - infer_t0)

        boxes = result.boxes
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
            self._frame_tracks_snapshot = {
                "frame_seq": int(self.last_frame_seq),
                "stamp": float(stamp),
                "entries": [],
            }
            self._push_frame_history(self._frame_tracks_snapshot)
            self.latest_result = self._lost(stamp, "no_detections", timings)
            self._log_frame(label, timings)
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

        # ── 目标类别检测快照：供 rebind 的精确同帧身份查询──
        if boxes.id is not None:
            track_ids = boxes.id.int().cpu().tolist()
            self._frame_tracks_snapshot = {
                "frame_seq": int(self.last_frame_seq),
                "stamp": float(stamp),
                "entries": [
                    {
                        "track_id": int(track_ids[i]),
                        "bbox": [int(round(v)) for v in boxes.xyxy[i].cpu().tolist()[:4]],
                        "score": float(boxes.conf[i]) if boxes.conf is not None and len(boxes.conf) > i else 1.0,
                    }
                    for i in target_indices
                ],
            }
        else:
            self._frame_tracks_snapshot = {"frame_seq": int(self.last_frame_seq), "stamp": float(stamp), "entries": []}

        self._push_frame_history(self._frame_tracks_snapshot)
        if len(target_indices) == 0:
            self.latest_result = self._lost(stamp, f"no_class_{label}", timings)
            self._log_frame(label, timings)
            return dict(self.latest_result)

        identify_ms = 0.0
        # ── 首帧 init_bbox → 匹配目标 track_id（普通帧由协议禁止携带）──
        if init_bbox is not None:
            clipped = _clip_bbox(init_bbox, image_bgr)
            if clipped is not None:
                id_t0 = time.perf_counter()
                matched_id = self._identify_target(boxes, target_indices, clipped)
                identify_ms += _ms(time.perf_counter() - id_t0)
                timings["identify_ms"] = identify_ms
                if matched_id is not None:
                    self.target_track_id = matched_id
                    self._target_segment = self._continuity[matched_id][0]
                    self.state = "active"
                    print(
                        f"[YOLOE_TRT] init_bbox matched track_id={self.target_track_id} "
                        f"label={label!r}",
                        flush=True,
                    )
                else:
                    self.latest_result = self._lost(stamp, "init_bbox_no_match", timings)
                    self._log_frame(label, timings)
                    return dict(self.latest_result)
            else:
                self.latest_result = self._lost(stamp, "init_bbox_out_of_bounds", timings)
                self._log_frame(label, timings)
                return dict(self.latest_result)

        timings["identify_ms"] = identify_ms
        # ── 按 target_track_id 筛选 ──
        if self.target_track_id is None:
            self.latest_result = self._lost(stamp, "awaiting_init_bbox", timings)
            self._log_frame(label, timings)
            return dict(self.latest_result)

        id_t0 = time.perf_counter()
        target = self._find_target_by_id(boxes, target_indices)
        identify_ms += _ms(time.perf_counter() - id_t0)
        timings["identify_ms"] = identify_ms
        if target is None:
            self.latest_result = self._lost(stamp, "target_missing", timings)
            self._log_frame(label, timings)
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
        # 返回前刷新端到端总耗时（含响应构造），并回填到响应 timings 保持与日志一致
        timings["total_ms"] = _ms(time.perf_counter() - total_t0)
        self.latest_result["timings"] = dict(timings)
        self._log_frame(label, timings)
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
        continuity = self._continuity.get(self.target_track_id)
        if continuity is None or continuity[0] != self._target_segment:
            return None
        if self.frame_seq > 1 and continuity[1] < self.identity_confirm_frames:
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
            # 会话身份字段：客户端据此做会话归属与身份版本校验
            "server_instance_id": self.server_instance_id,
            "session_id": self.session_id,
            "identity_revision": int(self.identity_revision),
            "protocol_version": 3,
            "epoch": self.epoch,
            "start_operation_id": self.start_operation_id,
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

    def status(self) -> dict[str, Any]:
        with self.lock:
            resp = {
                "server_instance_id": self.server_instance_id,
                "state": self.state,
                "session_id": self.session_id,
                "session_active": self.session_active,
                "protocol_version": 3,
                "epoch": self.epoch,
                "start_operation_id": self.start_operation_id,
                "operation_ttl": self._op_cache_ttl,
                "identity_revision": int(self.identity_revision),
                "last_frame_seq": int(self.last_frame_seq),
                "needs_reinitialize": self.needs_reinitialize,
                "label": self.current_label,
                "class_id": self.current_class_id,
                "tracker": self.current_tracker,
                "track_id": self.target_track_id,
                "last_bbox": self.last_bbox,
                "last_score": self.last_score,
                "frame_seq": int(self.frame_seq),
                "engine": str(self.engine_path),
                "device": self.device,
                "conf": self.default_conf,
                "iou": self.default_iou,
            }
            tracker_name = self.current_tracker or "deepocsort"
        # tracker 实际生效配置（ReID/GMC 等）在锁外读取 YAML
        try:
            resp["tracker_cfg"] = YAML.load(self._tracker_cfg_path(tracker_name))
        except Exception as exc:
            resp["tracker_cfg_error"] = str(exc)
        return resp

    def latest(self) -> dict[str, Any]:
        with self.lock:
            return dict(self.latest_result)


# ──────────────────────────────────────────────
# FastAPI app
# ──────────────────────────────────────────────

def create_app(engine: YoloeTensorRtTrackEngine) -> FastAPI:
    app = FastAPI(title="YOLOE TensorRT Track Engine v3")

    @app.middleware("http")
    async def capture_entry(request, call_next):
        request.state.received = time.perf_counter()
        return await call_next(request)

    def dispatch(handler, req, request):
        entered = time.perf_counter()
        with engine.lock:
            if time.perf_counter() - request.state.received > engine.max_queue_age:
                return engine._protocol_error("stale_request")
            # 幂等缓存前也检查实例；旧实例的 close 允许安全地返回已关闭。
            if handler != engine.session_close and req.server_instance_id != engine.server_instance_id:
                return engine._protocol_error("instance_mismatch")
            lock_wait_ms = _ms(time.perf_counter() - entered)
            result = dict(handler(req))
            result["operation_id" if hasattr(req, "operation_id") else "request_id"] = (
                getattr(req, "operation_id", None) or req.request_id)
            result["recv_ms"] = _ms(entered - request.state.received)
            result["timings"] = dict(result.get("timings") or {})
            result["timings"]["lock_wait_ms"] = lock_wait_ms
            result["timings"]["total_ms"] = _ms(time.perf_counter() - request.state.received)
            if engine.frame_seq % engine.log_every == 0 or not result.get("ok"):
                log_record = dict(result, cuda_memory=cuda_memory_snapshot())
                log_record["rss_mb"] = int(Path("/proc/self/statm").read_text().split()[1]) * os.sysconf("SC_PAGE_SIZE") / 1048576
                engine._track_logger.log(log_record)
            return result

    @app.post("/session/start")
    def start(req: SessionStartRequest, request: Request):
        return dispatch(engine.session_start, req, request)

    @app.post("/track")
    def track(req: SessionTrackRequest, request: Request):
        return dispatch(engine.session_track, req, request)

    @app.post("/session/rebind")
    def rebind(req: SessionRebindRequest, request: Request):
        return dispatch(engine.session_rebind, req, request)

    @app.post("/session/close")
    def close(req: SessionCloseRequest, request: Request):
        return dispatch(engine.session_close, req, request)

    @app.get("/status")
    def status():
        return engine.status()

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
    parser = argparse.ArgumentParser(description="YOLOE TensorRT Track Engine v3 (explicit sessions)")
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
    parser.add_argument("--frame-history-cap", type=int, default=64,
                        help="rebind 同帧证据的帧历史缓存条数上限")
    parser.add_argument("--frame-history-ttl", type=float, default=8.0,
                        help="帧历史缓存墙钟 TTL（秒），超龄条目不再作为同帧证据")
    parser.add_argument("--rebind-unique-margin", type=float, default=0.1,
                        help="rebind 歧义保护间隔：最佳与次优 IoU 差距小于该值且次优达标时拒绝")
    parser.add_argument("--op-cache-ttl", type=float, default=120.0,
                        help="operation_id/request_id 幂等缓存 TTL（秒）")
    parser.add_argument("--op-cache-max", type=int, default=128,
                        help="幂等缓存容量上限（条，最旧优先淘汰）")
    parser.add_argument("--max-queue-age", type=float, default=5.0)
    parser.add_argument("--identity-confirm-frames", type=int, default=2)
    parser.add_argument("--log-every", type=int, default=10)
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
        frame_history_cap=args.frame_history_cap,
        frame_history_ttl=args.frame_history_ttl,
        rebind_unique_margin=args.rebind_unique_margin,
        op_cache_ttl=args.op_cache_ttl,
        op_cache_max=args.op_cache_max,
        max_queue_age=args.max_queue_age, identity_confirm_frames=args.identity_confirm_frames,
        log_every=args.log_every,
    )
    # 启动横幅：打印实际生效的资源与配置，便于与 YAML/脚本参数核对
    try:
        cpu_set = ",".join(str(c) for c in sorted(os.sched_getaffinity(0)))
    except Exception:
        cpu_set = "unknown"
    print(
        f"[YOLOE_TRT] server starting: protocol=3 instance={engine.server_instance_id[:8]} "
        f"engine={engine.engine_path} imgsz={engine.engine_imgsz} device={engine.device} "
        f"tracker_dir={engine.tracker_dir} cpus=[{cpu_set}] "
        f"frame_history(cap={args.frame_history_cap},ttl={args.frame_history_ttl}s) "
        f"rebind_margin={args.rebind_unique_margin} "
        f"op_cache(ttl={args.op_cache_ttl}s,max={args.op_cache_max})",
        flush=True,
    )
    app = create_app(engine)
    uvicorn.run(app, host=args.host, port=args.port, reload=False, workers=1)


if __name__ == "__main__":
    main()

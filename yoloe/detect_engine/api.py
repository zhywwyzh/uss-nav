#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""YOLO 通用多类别目标检测 API。

该文件只负责模型加载与单帧检测推理，不做 tracking、不支持文本 prompt、不输出 track_id。
与 track_engine 的区别：/detect 一次返回图中全部目标（bbox + conf + cls + name）。

YOLO26 是 end2end（NMS-free）模型：head 只做 top-k 选择，同一目标的近似重复框可能存活，
因此服务端在 model.predict 之后额外执行一轮经典 IoU 后 NMS（见 nms_indices），
行为与原 yolo_infer.py 脚本保持一致。
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

from ultralytics import YOLO  # noqa: E402
from ultralytics.utils.metrics import box_iou  # noqa: E402

# 注意：这里刻意不设置 torch.backends.cudnn.enabled / matmul.allow_tf32。
# track_engine 的这两行是 YOLOE/TRT 特调；本检测服务按 yolo_infer.py 的原始口径运行，不加任何后端开关。


def _now() -> float:
    return time.time()


def _ms(seconds: float) -> float:
    """将秒转换为毫秒，保留三位小数便于 JSON 观察。"""
    return round(float(seconds) * 1000.0, 3)


def _decode_image_base64(image_base64: str) -> np.ndarray:
    """将 base64 编码图像解码为 BGR OpenCV ndarray。

    Ultralytics 的 numpy 输入口径是 OpenCV BGR（见 LoadPilAndNumpy._single_check），
    predictor.preprocess 内部会无条件做一次 BGR->RGB。这里必须保持 cv2.imdecode 的原始
    BGR，不能提前转 RGB，否则通道会被翻两次，模型实际吃到 BGR，与训练口径相反。
    """
    raw = base64.b64decode(image_base64)
    np_buf = np.frombuffer(raw, dtype=np.uint8)
    image_bgr = cv2.imdecode(np_buf, cv2.IMREAD_COLOR)
    if image_bgr is None:
        raise ValueError("cv2.imdecode returned None")
    return image_bgr


def nms_indices(preds: torch.Tensor, iou_thres: float = 0.5, agnostic: bool = False) -> list[int]:
    """Greedy NMS on (N, 6) tensor [x1, y1, x2, y2, conf, cls]; returns kept indices.

    YOLO26 is an end2end (NMS-free) model: its head only does top-k selection, so
    near-duplicate boxes for the same object can survive. This pass re-applies
    classic IoU-based NMS on the final predictions.
    """
    keep: list[int] = []
    if preds is None or len(preds) == 0:
        return keep
    order = torch.argsort(preds[:, 4], descending=True)
    suppressed = torch.zeros(len(preds), dtype=torch.bool, device=preds.device)
    for i in order.tolist():
        if suppressed[i]:
            continue
        keep.append(i)
        ious = box_iou(preds[i : i + 1, :4], preds[:, :4]).squeeze(0)
        if agnostic:
            suppressed |= ious > iou_thres
        else:  # suppress only same-class overlaps (class-aware NMS)
            same_cls = preds[i, 5] == preds[:, 5]
            suppressed |= (ious > iou_thres) & same_cls
    return keep


class DetectRequest(BaseModel):
    """单帧多类别检测请求。"""

    image_base64: str = Field(..., description="JPEG/PNG 图像的 base64 字符串")
    stamp: float | None = Field(None, description="外部图像时间戳，原样回显")
    conf: float | None = Field(None, ge=0.0, le=1.0, description="每请求置信度覆盖，默认取启动参数")
    iou: float | None = Field(None, ge=0.0, le=1.0, description="后处理 NMS IoU 阈值覆盖，默认 0.5；越小抑制越多")
    agnostic_nms: bool | None = Field(None, description="NMS 是否类别无关，默认 false")
    imgsz: int | None = Field(None, ge=32, description="推理尺寸覆盖，默认 1960，必须与训练尺寸一致")


class YoloDetectEngine:
    """YOLO 多类别检测引擎。

    单 uvicorn worker + 引擎级锁串行化推理：model.predict 会修改 model.predictor
    内部状态，并发请求不能共享（与 track_engine 一致）。
    """

    def __init__(
        self,
        *,
        model_path: str,
        device: str,
        conf: float,
        iou: float,
        imgsz: int,
        agnostic_nms: bool,
    ) -> None:
        self.model_path = str(model_path)
        self.device = str(device)
        self.default_conf = float(conf)
        self.default_iou = float(iou)  # 仅用于后 NMS，不传给 model.predict
        self.default_imgsz = int(imgsz)
        self.default_agnostic_nms = bool(agnostic_nms)

        self.lock = threading.Lock()
        self.model = self._load_model()
        self.names: dict[int, str] = self._extract_names()
        self.frame_seq = 0

    def _load_model(self):
        if not Path(self.model_path).exists():
            raise FileNotFoundError(
                f"model not found: {self.model_path}；请将 best.pt 放到 "
                f"yoloe/detect_engine/models/ 下，或通过 --model 指定路径"
            )
        if self.device.startswith("cuda") and not torch.cuda.is_available():
            raise RuntimeError("CUDA is requested but torch.cuda.is_available() is False")
        model = YOLO(self.model_path)
        model.to(self.device)
        model.eval()
        return model

    def _extract_names(self) -> dict[int, str]:
        """从 model.names 提取类别 id → 名称映射；部分模型首次 predict 后才填充，请求时用 result.names 兜底。"""
        raw = getattr(self.model, "names", None)
        names: dict[int, str] = {}
        if isinstance(raw, dict):
            for k, v in raw.items():
                names[int(k)] = str(v)
        elif isinstance(raw, (list, tuple)):
            names = {i: str(v) for i, v in enumerate(raw)}
        return names

    def _cls_name(self, cls_id: int, result_names: Any) -> str:
        if not self.names and isinstance(result_names, dict):
            return str(result_names.get(cls_id, cls_id))
        if not self.names and isinstance(result_names, (list, tuple)):
            return str(result_names[cls_id]) if cls_id < len(result_names) else str(cls_id)
        return self.names.get(cls_id, str(cls_id))

    def _record_speed(self, result, timings: dict[str, float]) -> None:
        """从 Ultralytics Result.speed 中提取预处理、推理和后处理耗时。"""
        speed = getattr(result, "speed", None) or {}
        if not speed:
            return
        timings["preprocess_ms"] = round(float(speed.get("preprocess", 0.0)), 3)
        timings["inference_ms"] = round(float(speed.get("inference", 0.0)), 3)
        timings["postprocess_ms"] = round(float(speed.get("postprocess", 0.0)), 3)

    def _log_timing(
        self,
        *,
        frame_seq: int,
        n_before: int,
        n_after: int,
        timings: dict[str, float],
    ) -> None:
        """打印单帧关键耗时，便于观察 YOLO 推理和后 NMS 开销。"""
        print(
            "[YOLO_DETECT_TIMING]",
            f"frame_seq={frame_seq}",
            f"n_before={n_before}",
            f"n_after={n_after}",
            f"decode_ms={timings.get('decode_ms', 0.0)}",
            f"lock_wait_ms={timings.get('lock_wait_ms', 0.0)}",
            f"model_predict_ms={timings.get('model_predict_ms', 0.0)}",
            f"preprocess_ms={timings.get('preprocess_ms', 0.0)}",
            f"inference_ms={timings.get('inference_ms', 0.0)}",
            f"postprocess_ms={timings.get('postprocess_ms', 0.0)}",
            f"nms_ms={timings.get('nms_ms', 0.0)}",
            f"serialize_ms={timings.get('serialize_ms', 0.0)}",
            f"total_ms={timings.get('total_ms', 0.0)}",
            flush=True,
        )

    def detect(self, req: DetectRequest) -> dict[str, Any]:
        total_t0 = time.perf_counter()
        timings: dict[str, float] = {}
        stamp = float(req.stamp if req.stamp is not None else _now())

        t0 = time.perf_counter()
        image_bgr = _decode_image_base64(req.image_base64)
        timings["decode_ms"] = _ms(time.perf_counter() - t0)

        conf = float(req.conf if req.conf is not None else self.default_conf)
        iou_thres = float(req.iou if req.iou is not None else self.default_iou)
        agnostic = bool(req.agnostic_nms if req.agnostic_nms is not None else self.default_agnostic_nms)
        imgsz = int(req.imgsz if req.imgsz is not None else self.default_imgsz)

        lock_t0 = time.perf_counter()
        with self.lock:
            timings["lock_wait_ms"] = _ms(time.perf_counter() - lock_t0)
            self.frame_seq += 1
            frame_seq = self.frame_seq

            # 注意：刻意不传 iou 给 predict —— YOLO26 end2end head 本身无 NMS，
            # iou 只驱动下方的手工后 NMS，保持与 yolo_infer.py 一致。
            t0 = time.perf_counter()
            results = self.model.predict(
                source=image_bgr,
                conf=conf,
                imgsz=imgsz,
                device=self.device,
                save=False,
                verbose=False,
            )
            timings["model_predict_ms"] = _ms(time.perf_counter() - t0)
            result = results[0] if results else None
            if result is not None:
                self._record_speed(result, timings)

            # 后 NMS（YOLO26 end2end head 跳过了 NMS，重复框可能存活）
            boxes = getattr(result, "boxes", None)
            n_before = len(boxes) if boxes is not None else 0
            n_after = n_before
            if boxes is not None and len(boxes):
                t0 = time.perf_counter()
                preds = torch.cat([boxes.xyxy, boxes.conf[:, None], boxes.cls[:, None]], dim=1)
                keep = nms_indices(preds, iou_thres, agnostic)
                if len(keep) < len(boxes):
                    result.boxes = boxes[keep]
                n_after = len(result.boxes)
                timings["nms_ms"] = _ms(time.perf_counter() - t0)

            t0 = time.perf_counter()
            detections: list[dict[str, Any]] = []
            if boxes is not None and len(boxes):
                result_names = getattr(result, "names", None)
                xyxy = boxes.xyxy.cpu().tolist()
                confs = boxes.conf.cpu().tolist()
                clss = boxes.cls.int().cpu().tolist()
                for (x1, y1, x2, y2), c, cls_id in zip(xyxy, confs, clss):
                    detections.append(
                        {
                            "bbox": [float(x1), float(y1), float(x2), float(y2)],
                            "conf": float(c),
                            "cls": int(cls_id),
                            "name": self._cls_name(int(cls_id), result_names),
                        }
                    )
            timings["serialize_ms"] = _ms(time.perf_counter() - t0)

        timings["total_ms"] = _ms(time.perf_counter() - total_t0)
        self._log_timing(frame_seq=frame_seq, n_before=n_before, n_after=n_after, timings=timings)
        return {
            "ok": True,
            "stamp": stamp,
            "detections": detections,
            "n_before": n_before,
            "n_after": n_after,
            "timings": timings,
            "wall_time": _now(),
            "source": "yolo:detect",
        }

    def status(self) -> dict[str, Any]:
        return {
            "ok": True,
            "model": self.model_path,
            "device": self.device,
            "defaults": {
                "conf": self.default_conf,
                "iou": self.default_iou,
                "imgsz": self.default_imgsz,
                "agnostic_nms": self.default_agnostic_nms,
            },
            "n_classes": len(self.names),
            "names": dict(self.names) if len(self.names) <= 64 else {f"...({len(self.names)} classes)": ""},
            "frame_seq": self.frame_seq,
            "wall_time": _now(),
        }

    def healthy(self) -> dict[str, Any]:
        return {"ok": True, "model": self.model_path, "device": self.device, "wall_time": _now()}


def create_app(engine: YoloDetectEngine) -> FastAPI:
    app = FastAPI(title="YOLO Detect Engine")

    @app.post("/detect")
    def detect(req: DetectRequest):
        try:
            return engine.detect(req)
        except Exception as exc:
            print(f"YOLO detect API request failed: {exc}", flush=True)
            raise HTTPException(status_code=400, detail=str(exc)) from exc

    @app.get("/status")
    def status():
        return engine.status()

    @app.get("/healthy")
    def healthy():
        return engine.healthy()

    return app


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="YOLO detect engine API")
    parser.add_argument("--model", default=str(YOLOE_ROOT / "detect_engine" / "models" / "best.pt"))
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--conf", type=float, default=0.1)
    parser.add_argument("--iou", type=float, default=0.5, help="后处理 NMS IoU 阈值（越小抑制越多）")
    parser.add_argument("--agnostic-nms", action="store_true", help="NMS 类别无关（一个目标被检成 2 类时开启）")
    parser.add_argument("--imgsz", type=int, default=1960, help="推理尺寸，必须与训练尺寸一致")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2251)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    engine = YoloDetectEngine(
        model_path=args.model,
        device=args.device,
        conf=args.conf,
        iou=args.iou,
        imgsz=args.imgsz,
        agnostic_nms=args.agnostic_nms,
    )
    app = create_app(engine)
    uvicorn.run(app, host=args.host, port=args.port, reload=False, workers=1)


if __name__ == "__main__":
    main()

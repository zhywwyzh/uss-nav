# Ultralytics 🚀 AGPL-3.0 License - https://ultralytics.com/license
"""Shared ReID encoder used by BoT-SORT, Deep OC-SORT, and TrackTrack.

* `.pt` YOLO checkpoints — loaded via `YOLO()`; embeddings are pulled from the second-to-last
layer through the predictor's `embed=[...]` argument (works with classification and ReID
backbones).
* Any other extension (`.torchscript`, `.onnx`, `.engine`, `.openvino`, …) — loaded via
`AutoBackend`; the model is expected to output the embedding tensor directly.
"""

from __future__ import annotations

import numpy as np
import torch

from ultralytics.nn.autobackend import AutoBackend
from ultralytics.utils.ops import xywh2xyxy
from ultralytics.utils.plotting import save_one_box

REID_ASSETS = frozenset(f"yolo26{k}-reid.onnx" for k in "nsmlx")


class ReID:
    """ReID encoder. Routes `.pt` to the YOLO predictor path; everything else to `AutoBackend`."""

    def __init__(self, model: str, imgsz: int = 224, device: str | torch.device | None = None, fp16: bool = False):
        """Initialize encoder for re-identification.

        Args:
            model (str): Path to a ReID model. `.pt` runs through the YOLO predictor (embed-layer extraction); other
                extensions go through `AutoBackend`.
            imgsz (int): Square input size used for crop preprocessing on the AutoBackend path. Overridden by the
                model's own static input size when one is detected.
            device (str | torch.device | None): Inference device; defaults to CUDA if available.
            fp16 (bool): Use half precision when the backend supports it.
        """
        self.imgsz = imgsz
        self.batch_size = None
        self.device = (
            torch.device(device) if device is not None else torch.device("cuda" if torch.cuda.is_available() else "cpu")
        )
        self.is_pt = str(model).endswith(".pt")

        if self.is_pt:
            from ultralytics import YOLO

            self.model = YOLO(model)
            # Initialize predictor with embed=[idx] so subsequent calls return embeddings.
            self.model(embed=[len(self.model.model.model) - 2], device=self.device, verbose=False, save=False)
            self.fp16 = False
        else:
            from pathlib import Path

            if Path(str(model)).name in REID_ASSETS:
                from ultralytics.utils.downloads import attempt_download_asset

                model = attempt_download_asset(str(model))
            self.model = AutoBackend(str(model), device=self.device, fp16=fp16, verbose=False)
            self.fp16 = self.model.fp16

            # Get model's input size for a fixed batch and crop size or detect dynamic batch and crop sizes.
            session = getattr(self.model, "session", None)
            shape = session.get_inputs()[0].shape if session is not None else ()
            if len(shape) == 4:
                if isinstance(shape[0], int) and shape[0] > 0:
                    self.batch_size = shape[0]
                if isinstance(shape[2], int) and shape[2] > 0:
                    self.imgsz = shape[2]

    @staticmethod
    def _crop_detections(img: np.ndarray, dets: np.ndarray) -> list[np.ndarray]:
        """Crop detection regions from image, converting xywh to xyxy first.

        Args:
            img (np.ndarray): BGR image.
            dets (np.ndarray): Detections in xywh format (first 4 columns used).

        Returns:
            (list[np.ndarray]): Cropped image patches.
        """
        return [save_one_box(det, img, save=False) for det in xywh2xyxy(torch.from_numpy(dets[:, :4]))]

    def _crops_to_tensor(self, crops: list[np.ndarray]) -> torch.Tensor:
        """Stack a list of valid image crops into a normalized BCHW float tensor at self.imgsz."""
        batch = torch.empty(len(crops), 3, self.imgsz, self.imgsz, dtype=torch.float32)
        for i, c in enumerate(crops):
            t = torch.from_numpy(np.ascontiguousarray(c[..., ::-1])).permute(2, 0, 1).unsqueeze(0).float() / 255.0
            batch[i] = torch.nn.functional.interpolate(
                t, size=(self.imgsz, self.imgsz), mode="bilinear", align_corners=False
            )[0]
        batch = batch.to(self.device)
        return batch.half() if self.fp16 else batch

    @torch.no_grad()
    def __call__(self, img: np.ndarray, dets: np.ndarray) -> list[np.ndarray | None]:
        """Extract embeddings for detected objects."""
        crops = self._crop_detections(img, dets)
        valid = [bool(c.size) for c in crops]
        valid_crops = [crop for crop, keep in zip(crops, valid) if keep]
        if not valid_crops:
            return [None] * len(crops)

        if self.is_pt:
            feats = self.model.predictor(valid_crops)
            if len(feats) != len(valid_crops) and feats[0].shape[0] == len(valid_crops):
                feats = feats[0]  # batched prediction with non-PyTorch backend
            valid_feats = [f.cpu().numpy() for f in feats]
        else:
            batch = self._crops_to_tensor(valid_crops)
            bs, n = self.batch_size, batch.shape[0]
            if bs is None or n == bs:
                feats = self.model(batch)
            else:  # fixed-batch model (e.g. static ONNX): run in chunks of bs, padding the last partial chunk
                outs = []
                for s in range(0, n, bs):
                    chunk = batch[s : s + bs]
                    if chunk.shape[0] < bs:
                        chunk = torch.cat([chunk, chunk[-1:].expand(bs - chunk.shape[0], *chunk.shape[1:])], 0)
                    outs.append(self.model(chunk))
                feats = torch.cat(outs, 0)[:n]
            valid_feats = [f.cpu().numpy() for f in feats]

        valid_feats = iter(valid_feats)
        return [next(valid_feats) if keep else None for keep in valid]


def build_encoder(with_reid: bool, model: str | None, device: str | torch.device | None = None):
    """Return a ReID encoder, the native-features pass-through, or None.

    Args:
        with_reid (bool): Whether ReID is enabled at all.
        model (str | None): `"auto"` returns a callable that converts pre-extracted backbone features to numpy arrays;
            any other value loads a `ReID` model from that path. Ignored when `with_reid` is False.
        device (str | torch.device | None): Inference device for the ReID model; defaults to CUDA if available.

    Returns:
        (Callable | None): A `(img, dets) -> list[np.ndarray | None]` encoder, or None when ReID is disabled.
    """
    if not with_reid:
        return None
    if model == "auto":

        def _auto_encoder(feats, _dets):
            if isinstance(feats, np.ndarray):
                return [f for f in feats]
            return [f.cpu().numpy() for f in feats]

        return _auto_encoder
    return ReID(model, device=device)


def smooth_feature(
    feat: np.ndarray, smooth: np.ndarray | None, alpha: float
) -> tuple[np.ndarray | None, np.ndarray | None]:
    """L2-normalize `feat` and blend it into `smooth` via exponential moving average.

    Args:
        feat (np.ndarray): New (un-normalized) appearance feature.
        smooth (np.ndarray | None): Current smoothed feature, or None on the first update.
        alpha (float): EMA weight on the existing `smooth` (``1.0`` keeps it unchanged).

    Returns:
        curr (np.ndarray | None): The normalized current feature, or None when `feat` is zero-norm (carries no
            appearance information, so the caller should leave its features unchanged).
        smooth (np.ndarray | None): The updated, renormalized smoothed feature.
    """
    norm = np.linalg.norm(feat)
    if norm < 1e-12:  # zero-norm feature has no appearance info; signal the caller to keep its current features
        return None, smooth
    feat = feat / norm
    if smooth is None:
        return feat, feat.copy()
    smooth = alpha * smooth + (1 - alpha) * feat
    return feat, smooth / np.linalg.norm(smooth)

# ====================================================================
# 以下为 YOLOE 项目自定义 FastReID / TensorRT ReID encoder（保留）

# ====================================================================
# 以下为 YOLOE 项目自定义 FastReID / TensorRT ReID encoder（保留）
# ====================================================================

from pathlib import Path
import sys
import time

import cv2
import torch.nn.functional as F

YOLOE_ROOT = Path(__file__).resolve().parents[3]
def _resolve_yoloe_path(path_value: str | Path, *, field_name: str) -> Path:
    """解析 ReID 配置路径，支持绝对路径和相对 yoloe 根目录的路径。"""
    if path_value is None or str(path_value).strip() == "":
        raise ValueError(f"{field_name} is required when BoT-SORT with_reid=True")

    path = Path(str(path_value)).expanduser()
    if not path.is_absolute():
        path = YOLOE_ROOT / path
    path = path.resolve()
    if not path.exists():
        raise FileNotFoundError(f"{field_name} not found: {path}")
    return path


class FastReIDEncoder:
    """FastReID 外观特征提取器，用于 BoT-SORT 的 ReID 分支。"""

    def __init__(
        self,
        config_path: str | Path,
        weights_path: str | Path,
        device: str = "cuda:0",
        root_path: str | Path = "fast-reid",
    ):
        self.root_path = _resolve_yoloe_path(root_path, field_name="fast_reid_root")
        self.config_path = _resolve_yoloe_path(config_path, field_name="fast_reid_config")
        self.weights_path = _resolve_yoloe_path(weights_path, field_name="fast_reid_weights")
        self.device = torch.device(device if torch.cuda.is_available() or not str(device).startswith("cuda") else "cpu")

        get_cfg, build_model, checkpointer_cls = self._import_fastreid(self.root_path)

        cfg = get_cfg()
        cfg.merge_from_file(str(self.config_path))
        cfg.MODEL.WEIGHTS = str(self.weights_path)
        cfg.MODEL.DEVICE = str(self.device)
        if hasattr(cfg.MODEL, "BACKBONE") and hasattr(cfg.MODEL.BACKBONE, "PRETRAIN"):
            cfg.MODEL.BACKBONE.PRETRAIN = False
        cfg.freeze()

        self.cfg = cfg
        self.model = build_model(cfg)
        checkpointer_cls(self.model).load(str(self.weights_path))
        self.model.to(self.device)
        self.model.eval()
        self.input_size = tuple(int(v) for v in cfg.INPUT.SIZE_TEST)
        self.backend = "fastreid"
        self.last_stats = {
            "backend": self.backend,
            "gpu": str(self.device).startswith("cuda"),
            "gpu_idx": int(str(self.device).split(":", 1)[1]) if str(self.device).startswith("cuda:") else -1,
            "feature_count": 0,
            "feature_dim": 0,
            "inference_ms": 0.0,
        }

    @staticmethod
    def _import_fastreid(root_path: Path):
        """兼容 pip fastreid 和原始 BoT-SORT fast_reid 子目录两种导入结构。"""
        root_text = str(root_path)
        if root_text not in sys.path:
            # fast-reid 源码仓库位于 yoloe/fast-reid，运行时需加入 sys.path 才能 import fastreid。
            sys.path.insert(0, root_text)

        try:
            from fastreid.config import get_cfg
            from fastreid.modeling import build_model
            from fastreid.utils.checkpoint import Checkpointer

            return get_cfg, build_model, Checkpointer
        except ImportError:
            pass

        try:
            from fast_reid.fastreid.config import get_cfg
            from fast_reid.fastreid.modeling.meta_arch import build_model
            from fast_reid.fastreid.utils.checkpoint import Checkpointer

            return get_cfg, build_model, Checkpointer
        except ImportError as exc:
            raise ImportError(
                "FastReID is required when BoT-SORT with_reid=True. "
                "Install fastreid in the YOLOE runtime environment or set with_reid=False."
            ) from exc

    def inference(self, img: np.ndarray, dets: np.ndarray) -> np.ndarray:
        """对检测框裁剪图像并返回 L2 归一化后的 ReID embedding。"""
        infer_t0 = time.perf_counter()
        if img is None:
            raise ValueError("FastReID inference requires the original image")
        if len(dets) == 0:
            self._record_stats(np.empty((0, 0), dtype=np.float32), infer_t0)
            return np.empty((0, 0), dtype=np.float32)

        crops = [self._crop_detection(img, det) for det in dets]
        valid_crops = [crop for crop in crops if crop is not None]
        if len(valid_crops) != len(crops):
            raise ValueError("FastReID received invalid detection boxes for image cropping")

        batch = torch.stack([self._preprocess(crop) for crop in valid_crops], dim=0).to(self.device)
        with torch.no_grad():
            features = self.model({"images": batch})
            if isinstance(features, dict):
                features = features.get("features", next(iter(features.values())))
            features = F.normalize(features, dim=1)
        result = features.detach().cpu().numpy().astype(np.float32)
        self._record_stats(result, infer_t0)
        return result

    def _record_stats(self, features: np.ndarray, start_time: float) -> None:
        """记录最近一次 ReID 推理统计，供 API 日志读取。"""
        self.last_stats = {
            "backend": self.backend,
            "gpu": str(self.device).startswith("cuda"),
            "gpu_idx": int(str(self.device).split(":", 1)[1]) if str(self.device).startswith("cuda:") else -1,
            "feature_count": int(features.shape[0]) if features.ndim >= 1 else 0,
            "feature_dim": int(features.shape[1]) if features.ndim >= 2 else 0,
            "inference_ms": round((time.perf_counter() - start_time) * 1000.0, 3),
        }

    def _preprocess(self, crop: np.ndarray) -> torch.Tensor:
        """按 FastReID 测试输入尺寸 resize，并转为 CHW tensor。"""
        crop_rgb = crop
        crop_rgb = cv2.resize(crop_rgb, tuple(self.input_size[::-1]), interpolation=cv2.INTER_LINEAR)
        return torch.as_tensor(crop_rgb.astype("float32").transpose(2, 0, 1)).float()

    @staticmethod
    def _crop_detection(img: np.ndarray, det: np.ndarray) -> np.ndarray | None:
        """将 Ultralytics 的 xywh(+angle)+idx 检测结果转为图像裁剪区域。"""
        if img is None or img.size == 0:
            return None
        det_xywh = np.asarray(det[:4], dtype=np.float32)
        if det_xywh.shape[0] < 4 or not np.isfinite(det_xywh).all():
            return None
        height, width = img.shape[:2]
        cx, cy, box_w, box_h = [float(v) for v in det_xywh]
        if box_w <= 1.0 or box_h <= 1.0:
            return None
        x1 = max(0, min(width - 1, int(round(cx - box_w / 2.0))))
        y1 = max(0, min(height - 1, int(round(cy - box_h / 2.0))))
        x2 = max(0, min(width, int(round(cx + box_w / 2.0))))
        y2 = max(0, min(height, int(round(cy + box_h / 2.0))))
        if x2 <= x1 or y2 <= y1:
            return None
        return img[y1:y2, x1:x2]


class TensorRTReIDEncoder:
    """TensorRT ReID 外观特征提取器，用于 BoT-SORT 的低延迟 ReID 分支。"""

    def __init__(
        self,
        engine_path: str | Path,
        *,
        batch_size: int = 16,
        input_size: tuple[int, int] = (256, 128),
        device: str = "cuda:0",
    ):
        self.engine_path = _resolve_yoloe_path(engine_path, field_name="fast_reid_engine")
        self.batch_size = max(1, int(batch_size))
        self.input_size = (int(input_size[0]), int(input_size[1]))
        self.gpu_idx = self._parse_gpu_idx(device)

        try:
            import pycuda.driver as cuda
            import tensorrt as trt
        except ImportError as exc:
            raise ImportError(
                "TensorRT ReID backend requires pycuda and tensorrt. "
                "Install them in the YOLOE runtime environment or set reid_backend=fastreid."
            ) from exc

        self.cuda = cuda
        self.trt = trt
        self.trt_logger = trt.Logger(trt.Logger.ERROR)

        self.cuda.init()
        self.device_ctx = self.cuda.Device(self.gpu_idx).retain_primary_context()

        self.device_ctx.push()
        try:
            self.engine = self._load_engine()
            self.context = self.engine.create_execution_context()
            self.trt10 = self._is_trt10()
            self.inputs, self.outputs, self.bindings, self.stream = self._allocate_buffers()
        finally:
            self.device_ctx.pop()

        self.input_shape = tuple(int(v) for v in self.inputs[0]["shape"])
        self.engine_batch_size = int(self.input_shape[0])
        self.batch_size = min(self.batch_size, self.engine_batch_size)

        self.backend = "tensorrt"
        self.valid_indices = []
        self.last_stats = {
            "backend": self.backend,
            "feature_count": 0,
            "feature_dim": 0,
            "inference_ms": 0.0,
        }

        print(
            "[ReID][TensorRT] initialized",
            f"engine={self.engine_path}",
            f"trt_version={self.trt.__version__}",
            f"trt10={self.trt10}",
            f"gpu=True",
            f"gpu_idx={self.gpu_idx}",
            "cuda_context=primary",
            f"requested_batch={batch_size}",
            f"runtime_batch={self.batch_size}",
            f"engine_batch={self.engine_batch_size}",
            f"input_shape={self.input_shape}",
            f"output_shape={tuple(int(v) for v in self.outputs[-1]['shape'])}",
            flush=True,
        )

    @staticmethod
    def _parse_gpu_idx(device: str) -> int:
        device = str(device or "cuda:0").strip().lower()
        if device.startswith("cuda:") and device.split(":", 1)[1].isdigit():
            return int(device.split(":", 1)[1])
        return 0

    def _load_engine(self):
        with open(self.engine_path, "rb") as f, self.trt.Runtime(self.trt_logger) as runtime:
            engine = runtime.deserialize_cuda_engine(f.read())
        if engine is None:
            raise RuntimeError(f"failed to deserialize TensorRT ReID engine: {self.engine_path}")
        return engine

    def _is_trt10(self) -> bool:
        """
        TensorRT 10 使用 tensor API：
          engine.num_io_tensors
          engine.get_tensor_name()
          engine.get_tensor_mode()
          context.set_tensor_address()
          context.execute_async_v3()

        TensorRT 8 使用 binding API：
          engine.num_bindings
          engine.get_binding_name()
          engine.binding_is_input()
          context.execute_async_v2()
        """
        return hasattr(self.engine, "num_io_tensors")

    def _engine_io(self) -> list[dict]:
        """返回 engine 的输入输出信息，兼容 TensorRT 8 和 TensorRT 10。"""
        items = []

        if self._is_trt10():
            for i in range(self.engine.num_io_tensors):
                name = self.engine.get_tensor_name(i)
                is_input = self.engine.get_tensor_mode(name) == self.trt.TensorIOMode.INPUT
                shape = tuple(int(v) for v in self.engine.get_tensor_shape(name))
                dtype = self.trt.nptype(self.engine.get_tensor_dtype(name))
                items.append(
                    {
                        "index": i,
                        "name": name,
                        "is_input": bool(is_input),
                        "shape": shape,
                        "dtype": dtype,
                    }
                )
        else:
            for i in range(self.engine.num_bindings):
                name = self.engine.get_binding_name(i)
                is_input = bool(self.engine.binding_is_input(i))
                shape = tuple(int(v) for v in self.engine.get_binding_shape(i))
                dtype = self.trt.nptype(self.engine.get_binding_dtype(i))
                items.append(
                    {
                        "index": i,
                        "name": name,
                        "is_input": is_input,
                        "shape": shape,
                        "dtype": dtype,
                    }
                )

        if not items:
            raise RuntimeError("TensorRT ReID engine has no IO tensors/bindings")

        return items

    def _fix_dynamic_input_shape(self, shape: tuple[int, ...]) -> tuple[int, ...]:
        """
        修复动态输入 shape。
        FastReID TensorRT engine 通常是 BCHW：
          B, 3, H, W
        """
        if len(shape) != 4:
            raise RuntimeError(f"Unsupported TensorRT ReID input shape: {shape}")

        fixed = []
        for i, dim in enumerate(shape):
            if dim >= 0:
                fixed.append(int(dim))
                continue

            if i == 0:
                fixed.append(int(self.batch_size))
            elif i == 1:
                fixed.append(3)
            elif i == 2:
                fixed.append(int(self.input_size[0]))
            elif i == 3:
                fixed.append(int(self.input_size[1]))
            else:
                raise RuntimeError(f"Unsupported dynamic TensorRT ReID input shape: {shape}")

        return tuple(fixed)

    def _get_context_shape(self, item: dict) -> tuple[int, ...]:
        """读取 context 中当前 tensor/binding shape。"""
        if self._is_trt10():
            return tuple(int(v) for v in self.context.get_tensor_shape(item["name"]))
        return tuple(int(v) for v in self.context.get_binding_shape(item["index"]))

    def _set_input_shape_if_dynamic(self, item: dict) -> tuple[int, ...]:
        """如果输入是动态 shape，则设置运行时 shape，并返回最终 shape。"""
        shape = tuple(int(v) for v in item["shape"])

        if any(dim < 0 for dim in shape):
            fixed_shape = self._fix_dynamic_input_shape(shape)
            if self._is_trt10():
                ok = self.context.set_input_shape(item["name"], fixed_shape)
                if ok is False:
                    raise RuntimeError(
                        f"TensorRT ReID set_input_shape failed: name={item['name']}, shape={fixed_shape}"
                    )
            else:
                self.context.set_binding_shape(item["index"], fixed_shape)

        final_shape = self._get_context_shape(item)

        if any(dim < 0 for dim in final_shape):
            raise RuntimeError(
                f"TensorRT ReID input has unresolved dynamic shape: "
                f"name={item['name']}, original={shape}, final={final_shape}"
            )

        return final_shape

    def _allocate_buffers(self):
        """
        分配 TensorRT 输入输出显存/页锁定内存。

        兼容：
        - TensorRT 8: binding API + execute_async_v2
        - TensorRT 10: tensor API + set_tensor_address + execute_async_v3
        """
        inputs, outputs, bindings = [], [], []
        stream = self.cuda.Stream()

        io_items = self._engine_io()

        # 1. 先设置输入 shape。输出 shape 需要在输入 shape 确定之后读取。
        for item in io_items:
            if not item["is_input"]:
                continue
            item["shape"] = self._set_input_shape_if_dynamic(item)

        # 2. 再读取输出 shape，并分配 buffer。
        for item in io_items:
            if item["is_input"]:
                shape = tuple(int(v) for v in item["shape"])
            else:
                shape = self._get_context_shape(item)
                item["shape"] = shape

            if any(dim < 0 for dim in shape):
                raise RuntimeError(
                    f"TensorRT ReID tensor has unresolved dynamic shape: "
                    f"name={item['name']}, shape={shape}"
                )

            size = int(self.trt.volume(shape))
            if size <= 0:
                raise RuntimeError(
                    f"TensorRT ReID tensor has invalid size: "
                    f"name={item['name']}, shape={shape}, size={size}"
                )

            host_mem = self.cuda.pagelocked_empty(size, item["dtype"])
            device_mem = self.cuda.mem_alloc(host_mem.nbytes)

            bindings.append(int(device_mem))

            mem_item = {
                "host": host_mem,
                "device": device_mem,
                "shape": shape,
                "dtype": item["dtype"],
                "name": item["name"],
                "index": item["index"],
                "is_input": item["is_input"],
            }

            if item["is_input"]:
                inputs.append(mem_item)
            else:
                outputs.append(mem_item)

            print(
                "[ReID][TensorRT][IO]",
                f"name={item['name']}",
                f"is_input={item['is_input']}",
                f"dtype={item['dtype']}",
                f"shape={shape}",
                f"size={size}",
                flush=True,
            )

        if len(inputs) != 1 or len(outputs) < 1:
            raise RuntimeError(
                f"TensorRT ReID engine must have one input and at least one output, "
                f"got inputs={len(inputs)}, outputs={len(outputs)}"
            )

        return inputs, outputs, bindings, stream

    def inference(self, img: np.ndarray, dets: np.ndarray) -> np.ndarray:
        """对检测框裁剪图像并返回 L2 归一化后的 TensorRT ReID embedding。"""
        infer_t0 = time.perf_counter()
        if img is None:
            raise ValueError("TensorRT ReID inference requires the original image")
        if len(dets) == 0:
            self.valid_indices = []
            self._record_stats(np.empty((0, 0), dtype=np.float32), infer_t0)
            return np.empty((0, 0), dtype=np.float32)

        valid_crops = []
        self.valid_indices = []
        for index, det in enumerate(dets):
            crop = FastReIDEncoder._crop_detection(img, det)
            if crop is None:
                continue
            valid_crops.append(crop)
            self.valid_indices.append(index)

        if not valid_crops:
            self._record_stats(np.empty((0, 0), dtype=np.float32), infer_t0)
            print(
                "[ReID][TensorRT] inference",
                "gpu=True",
                f"gpu_idx={self.gpu_idx}",
                f"dets={len(dets)}",
                "valid=0",
                "features=(0, 0)",
                f"ms={self.last_stats['inference_ms']}",
                flush=True,
            )
            return np.empty((0, 0), dtype=np.float32)

        chunks = []
        for start in range(0, len(valid_crops), self.batch_size):
            batch_crops = valid_crops[start : start + self.batch_size]
            chunks.append(self._infer_batch(batch_crops))

        features = np.concatenate(chunks, axis=0)
        result = self._l2_normalize(features.astype(np.float32))
        self._record_stats(result, infer_t0)

        print(
            "[ReID][TensorRT] inference",
            "gpu=True",
            f"gpu_idx={self.gpu_idx}",
            f"dets={len(dets)}",
            f"valid={len(valid_crops)}",
            f"features={tuple(int(v) for v in result.shape)}",
            f"ms={self.last_stats['inference_ms']}",
            flush=True,
        )
        return result

    def _record_stats(self, features: np.ndarray, start_time: float) -> None:
        """记录最近一次 TensorRT ReID 推理统计，供 API 日志读取。"""
        self.last_stats = {
            "backend": self.backend,
            "feature_count": int(features.shape[0]) if features.ndim >= 1 else 0,
            "feature_dim": int(features.shape[1]) if features.ndim >= 2 else 0,
            "inference_ms": round((time.perf_counter() - start_time) * 1000.0, 3),
        }

    def _infer_batch(self, crops: list[np.ndarray]) -> np.ndarray:
        batch = np.stack([self._preprocess(crop) for crop in crops], axis=0)
        valid_bsz = batch.shape[0]

        if valid_bsz > self.engine_batch_size:
            raise RuntimeError(
                f"TensorRT ReID batch too large: valid_bsz={valid_bsz}, engine_batch_size={self.engine_batch_size}"
            )

        if valid_bsz < self.engine_batch_size:
            padding = np.zeros(
                (self.engine_batch_size - valid_bsz, 3, self.input_size[0], self.input_size[1]),
                dtype=np.float32,
            )
            batch = np.concatenate([batch, padding], axis=0)

        batch = np.ascontiguousarray(batch.astype(self.inputs[0]["dtype"]))

        expected_input_size = int(np.prod(self.inputs[0]["shape"]))
        if batch.size != expected_input_size:
            raise RuntimeError(
                f"TensorRT ReID input size mismatch: "
                f"batch.shape={batch.shape}, batch.size={batch.size}, "
                f"engine_input_shape={self.inputs[0]['shape']}, expected_size={expected_input_size}"
            )

        self.device_ctx.push()
        try:
            np.copyto(self.inputs[0]["host"], batch.ravel())

            self.cuda.memcpy_htod_async(
                self.inputs[0]["device"],
                self.inputs[0]["host"],
                self.stream,
            )

            if self._is_trt10():
                # TensorRT 10: 使用 tensor name 绑定 device address。
                for item in self.inputs + self.outputs:
                    ok = self.context.set_tensor_address(item["name"], int(item["device"]))
                    if ok is False:
                        raise RuntimeError(
                            f"TensorRT ReID set_tensor_address failed: name={item['name']}"
                        )

                ok = self.context.execute_async_v3(stream_handle=self.stream.handle)
            else:
                # TensorRT 8: 使用 binding list。
                ok = self.context.execute_async_v2(
                    bindings=self.bindings,
                    stream_handle=self.stream.handle,
                )

            if ok is False:
                raise RuntimeError("TensorRT ReID execution failed")

            for output in self.outputs:
                self.cuda.memcpy_dtoh_async(output["host"], output["device"], self.stream)

            self.stream.synchronize()

        finally:
            self.device_ctx.pop()

        output = self.outputs[-1]
        features = output["host"].reshape(self.engine_batch_size, -1)[:valid_bsz]
        return features.copy()

    def _preprocess(self, crop: np.ndarray) -> np.ndarray:
        """按 TensorRT engine 输入尺寸 resize，并转为 CHW float32。"""
        height, width = self.input_size
        crop_rgb = cv2.resize(crop, (width, height), interpolation=cv2.INTER_CUBIC)
        return crop_rgb.astype("float32").transpose(2, 0, 1)

    @staticmethod
    def _l2_normalize(features: np.ndarray) -> np.ndarray:
        norm = np.linalg.norm(features, ord=2, axis=1, keepdims=True)
        return features / (norm + np.finfo(np.float32).eps)

    def __del__(self):
        context = getattr(self, "device_ctx", None)
        if context is not None:
            try:
                context.detach()
            except Exception:
                pass


def build_reid_encoder(args):
    """根据 botsort.yaml 构造 ReID encoder。"""
    backend = str(getattr(args, "reid_backend", "fastreid") or "fastreid").strip().lower()
    print(f"[ReID] build backend={backend} with_reid={getattr(args, 'with_reid', None)}", flush=True)

    if backend in {"fastreid", "pytorch"}:
        return FastReIDEncoder(
            config_path=args.fast_reid_config,
            weights_path=args.fast_reid_weights,
            device=getattr(args, "reid_device", "cuda:0"),
            root_path=getattr(args, "fast_reid_root", "fast-reid"),
        )

    if backend in {"tensorrt", "trt"}:
        return TensorRTReIDEncoder(
            engine_path=args.fast_reid_engine,
            batch_size=getattr(args, "reid_batch_size", 16),
            input_size=tuple(getattr(args, "reid_input_size", [256, 128])),
            device=getattr(args, "reid_device", "cuda:0"),
        )


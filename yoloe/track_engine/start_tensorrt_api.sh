#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
YOLOE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

cd "${YOLOE_ROOT}"

# 默认 tracker: deepocsort (ORU+OCR+ReID+GMC). TensorRT 仅提供检测框,
# ReID 回退到 model=auto 的内建特征; 若需外观区分, 使用 standard 版 (start_yoloe_tracking_api.sh)
# HTTP 请求中可通过 "tracker" 字段覆盖: deepocsort/botsort/bytetrack/ocsort/tracktrack/fasttrack
python track_engine/tensorrt-api.py \
  --pt-model "${YOLOE_ROOT}/yoloe-v8m-seg.pt" \
  --engine "${YOLOE_ROOT}/yoloe-v8m-seg.engine" \
  --classes "${YOLOE_ROOT}/prompt/prompt.txt" \
  --tracker-dir "${YOLOE_ROOT}/ultralytics/cfg/trackers" \
  --host "127.0.0.1" \
  --port 2250 \
  --conf 0.1 \
  --iou 0.5 \
  --imgsz 480,640 \
  --engine-imgsz 480,640 \
  --pipeline-track \
  "$@"

#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
YOLOE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

cd "${YOLOE_ROOT}"

python3 track_engine/tensorrt-api.py \
  --pt-model "${YOLOE_ROOT}/prompt/yoloe_pretrain/yoloe-11m-seg.pt" \
  --engine "${YOLOE_ROOT}/prompt/yoloe_pretrain/yoloe-11m-seg-fixed.engine" \
  --classes "${YOLOE_ROOT}/prompt/prompt.txt" \
  --tracker-dir "${YOLOE_ROOT}/ultralytics/cfg/trackers" \
  --host "127.0.0.1" \
  --port 2250 \
  --conf 0.1 \
  --iou 0.5 \
  --imgsz 480,640 \
  --engine-imgsz 480,640 \
  "$@"

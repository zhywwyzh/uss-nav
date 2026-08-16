#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
YOLOE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

cd "${YOLOE_ROOT}"

# v2: 使用官方 model.track(persist=True) + VLM bbox→track_id 匹配
# 默认 tracker: botsort (自带 GMC + 低分救援，适合无人机)
# HTTP 请求中可通过 "tracker" 字段覆盖: botsort/bytetrack/deepocsort/ocsort
# CPU7: 检测器单核
# python track_engine/tensorrt-api.py \
taskset -c 7 python track_engine/tensorrt-api.py \
  --pt-model "${YOLOE_ROOT}/pretrain/yoloe-26n-seg.pt" \
  --engine "${YOLOE_ROOT}/pretrain/yoloe-26n-seg.engine" \
  --classes "${YOLOE_ROOT}/prompt/prompt.txt" \
  --tracker-dir "${YOLOE_ROOT}/ultralytics/cfg/trackers" \
  --host "127.0.0.1" \
  --port 2250 \
  --conf 0.1 \
  --iou 0.5 \
  --imgsz 480,640 \
  --engine-imgsz 480,640 \
  --init-bbox-match-iou "${YOLOE_TRT_INIT_BBOX_IOU:-0.1}" \
  "$@"

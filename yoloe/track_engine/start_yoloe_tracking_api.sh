#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
YOLOE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

# 默认 tracker: deepocsort (ORU+OCR+ReID+GMC, 适合机器人跟拍的遮挡/人群场景)
# HTTP 请求中可通过 "tracker" 字段覆盖: deepocsort/botsort/bytetrack/ocsort/tracktrack/fasttrack
# 配置: ultralytics/cfg/trackers/deepocsort.yaml
python3 "${SCRIPT_DIR}/api.py" \
  --model "${YOLOE_DIR}/yoloe-v8m-seg.pt" \
  --host "${YOLOE_TRACK_HOST:-127.0.0.1}" \
  --port "${YOLOE_TRACK_PORT:-2250}" \
  --device "${YOLOE_TRACK_DEVICE:-cuda:0}" \
  "$@"

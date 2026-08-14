#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
YOLOE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

# 默认 tracker: deepocsort (ORU+OCR+ReID+GMC, 适合机器人跟拍的遮挡/人群场景)
# HTTP 请求中可通过 "tracker" 字段覆盖: deepocsort/botsort/bytetrack/ocsort/tracktrack/fasttrack
# 配置: ultralytics/cfg/trackers/deepocsort.yaml
# 注意：--init-bbox-match-iou 控制 VLM bbox 与 tracker 输出匹配的最小 IoU
#       设为 0.1 表示：只要 VLM bbox 与 tracker 中某个目标有 10% 重叠即可匹配
#       YOLOE_TRACK_INIT_BBOX_IOU 环境变量可覆盖
python3 "${SCRIPT_DIR}/api.py" \
  --model "${YOLOE_DIR}/yoloe-v8m-seg.pt" \
  --host "${YOLOE_TRACK_HOST:-127.0.0.1}" \
  --port "${YOLOE_TRACK_PORT:-2250}" \
  --device "${YOLOE_TRACK_DEVICE:-cuda:0}" \
  --init-bbox-match-iou "${YOLOE_TRACK_INIT_BBOX_IOU:-0.1}" \
  "$@"

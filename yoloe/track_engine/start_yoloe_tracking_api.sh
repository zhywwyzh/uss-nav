#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
YOLOE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

python3 "${SCRIPT_DIR}/api.py" \
  --model "${YOLOE_DIR}/yoloe-v8m-seg.pt" \
  --host "${YOLOE_TRACK_HOST:-127.0.0.1}" \
  --port "${YOLOE_TRACK_PORT:-2250}" \
  --device "${YOLOE_TRACK_DEVICE:-cuda:0}" \
  "$@"

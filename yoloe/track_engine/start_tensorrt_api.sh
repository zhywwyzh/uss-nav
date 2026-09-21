#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
YOLOE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

cd "${YOLOE_ROOT}"

# v3: 显式会话协议 + 显式管线（detect → 类别过滤 → tracker.update → 身份筛选）
# tracker 类型由客户端 /session/start 请求指定（run_main 默认 deepocsort），
# 实际生效的 tracker/ReID/GMC 配置见启动横幅与会话 start 日志
#
# CPU 亲和性：默认绑定 CPU7（整个进程及继承亲和性的线程）；设 YOLOE_TRT_CPUS 为
# 空串则不绑核（用于资源对照实验），或设为逗号分隔的核集合（如 "6,7"）
CPUS="${YOLOE_TRT_CPUS-7}"
RUN_PREFIX=()
if [[ -n "${CPUS}" ]]; then
  RUN_PREFIX=(taskset -c "${CPUS}")
fi

"${RUN_PREFIX[@]}" python track_engine/tensorrt-api.py \
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

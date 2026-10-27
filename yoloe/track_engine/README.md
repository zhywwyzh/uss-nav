# YOLOE Track Engine

本目录提供一个独立的 YOLOE 文本目标跟踪服务，不修改原有 `predict_realtime_cam_sim.py`。

## 功能

- `api.py` 只负责启动 YOLOE 模型、接收外部传入的 `image + label`、运行 tracking 并返回结果。
- `test.py` 是 ROS 测试客户端，负责订阅相机图像、调用 API、绘制跟踪结果并发布 `/nav_mission_image`。
- `fps_test.py` 是 ROS 性能测试客户端，使用相同图像订阅和发布方式，额外统计输入 FPS、API 处理 FPS、HTTP 往返耗时和服务端分段耗时。
- API 使用 `YOLOE.set_classes([label], model.get_text_pe([label]))` 动态切换文本目标。
- API 使用 Ultralytics `model.track(..., persist=True)` 输出 `bbox + track_id`。

## 启动 API

```bash
cd /home/gwq/workspace/cloud/VLA-Diff/ros_ws/uss-nav/yoloe
bash track_engine/start_yoloe_tracking_api.sh
```

可通过环境变量覆盖常用参数：

```bash
YOLOE_TRACK_PORT=2250 \
YOLOE_TRACK_DEVICE=cuda:0 \
bash track_engine/start_yoloe_tracking_api.sh
```

## TensorRT 固定词表 API

`start_tensorrt_api.sh` 启动 `tensorrt-api.py`，使用启动时固定的 classes 文件导出/加载 TensorRT engine。该模式下 `/track` 的 `label` 必须存在于 classes 文件中，适合把上层语义目标先收敛到固定类别，例如 `person`。

```bash
bash track_engine/start_tensorrt_api.sh
```

脚本默认传入 `--pipeline-track`。普通连续 `/track` 请求会只提交最新帧并立即返回最近完成的 tracking 结果；服务端 detector 线程执行 YOLOE TensorRT `predict`，tracker 线程按检测完成顺序执行 BoT-SORT/ByteTrack/GMC/ReID 后处理。这样 `frame_k+1` 的 YOLOE 推理可以和 `frame_k` 的 tracker 后处理错帧重叠，降低 `/track` 同步等待时间。`reset`、`init_bbox`、`allow_rebind` 和 `lost_rebind` 请求仍同步执行，并会清空流水线状态，保证初始化和重捕获不会被旧帧覆盖。身份安全优先时，rebind 会重置 BoT-SORT 轨迹和 GMC 状态，但保留已有 ReID TensorRT encoder，避免每次慢刷新都重新初始化 ReID engine。

## BoT-SORT ReID

默认 `ultralytics/cfg/trackers/botsort.yaml` 中 `with_reid: True`，BoT-SORT 会在 YOLOE API 进程内加载 FastReID 模型，不需要单独启动 ReID 服务。当前默认使用 `fast-reid` 仓库内的 Market1501 R50 配置和权重：

```text
fast-reid/
├── configs/Market1501/bagtricks_R50.yml
└── weights/market_bot_R50.pth
```

`fast_reid_root`、`fast_reid_config` 和 `fast_reid_weights` 支持绝对路径，也支持相对 `ros_ws/uss-nav/yoloe` 的路径。若希望恢复原始 BoT-SORT 行为，将 `with_reid` 改为 `False`，此时不会导入或加载 FastReID。

如果希望 ReID 分支也使用 TensorRT，将 `reid_backend` 改为 `tensorrt`，并确保 `fast_reid_engine` 指向已导出的 ReID engine：

```yaml
with_reid: True
reid_backend: tensorrt
fast_reid_engine: fast-reid/outputs/trt_model/market_bot_R50.engine
reid_batch_size: 16
reid_input_size: [256, 128]
```

`fast-reid/tools/deploy/export_market_bot_r50_trt.sh` 会按同一 batch size 导出 ONNX 和 TensorRT engine。运行时 TensorRT ReID 会过滤越界、非有限值或尺寸过小的检测框，避免单个无效 crop 使 `/track` 返回 400；如果一帧中没有有效 ReID crop，该帧不会初始化新的 BoT-SORT detection。

## API

`api.py` 提供一个单帧 tracking 接口：

```bash
curl -X POST http://127.0.0.1:2250/track \
  -H 'Content-Type: application/json' \
  -d '{
    "label":"person",
    "image_base64":"...",
    "tracker":"botsort"
  }'
```

当 `label` 改变或 `reset=true` 时，API 会直接重载整个 YOLOE 模型，再重新设置 YOLOE 文本类别。该策略会牺牲切换瞬间的速度，但可以彻底丢弃旧 predictor、tracker、prompt embedding 和 Ultralytics `model.track()` 注册的 tracking callbacks。仅 `tracker` 或 `prompt_mode` 改变时，API 会清理 tracker 状态并重建 predictor。重载期间 `/track` 请求会同步等待，调用方 timeout 需要覆盖模型重载、文本 embedding 和 visual prompt 生成耗时。

如果上层 VLM 已经给出初始框，建议传入 `init_bbox`，多同类目标场景会更稳定。

查看最新结果：

```bash
curl http://127.0.0.1:2250/latest
```

重置状态：

```bash
curl -X POST http://127.0.0.1:2250/reset \
  -H 'Content-Type: application/json' \
  -d '{"reason":"manual"}'
```

## ROS 测试客户端

`test.py` 从 ROS 压缩图像话题获取图像，把图像和 label 发送给 API，然后根据返回的 bbox/id 绘制图像并发布：

```text
/nav_mission_image
```

示例：

```bash
python3 track_engine/test.py \
  --label person \
  --image-topic /camera1/color/image/compressed \
  --output-topic /nav_mission_image \
  --api-url http://127.0.0.1:2250
```

## ROS 帧率测试客户端

`fps_test.py` 用于定位 YOLOE-track 速率瓶颈。它会订阅同一个压缩图像话题，调用 `/track`，把带框和耗时信息的图像发布到 `/nav_mission_image`，并每秒打印一次统计日志。

```bash
python3 track_engine/fps_test.py \
  --label person \
  --image-topic /camera1/color/image/compressed \
  --output-topic /nav_mission_image \
  --api-url http://127.0.0.1:2250 \
  --prompt-mode text \
  --rate 30
```

日志中关键字段含义：

- `input`：ROS 图像输入 FPS。
- `api`：客户端实际完成 `/track` 请求的 FPS。
- `rtt_avg`：客户端 HTTP 往返平均耗时。
- `server_avg`：API 服务端总处理平均耗时。
- `model_avg`：YOLOE `model.track()` 平均耗时。
- `last_decode`：服务端 base64/JPEG 解码耗时。

如果 `api` 约 1Hz，优先看 `model_avg` 和 `server_avg`。若 `model_avg` 本身接近 1000ms，瓶颈在 YOLOE 推理或 tracker；若 `server_avg` 远大于 `model_avg`，需要看 prompt 更新、解码或锁等待。

## 输出格式

`/latest` 或 `/track` 返回示例：

```json
{
  "ok": true,
  "state": "active",
  "label": "person",
  "track_id": 3,
  "bbox": [100, 80, 220, 310],
  "score": 0.81,
  "cls": 0,
  "has_mask": true,
  "stamp": 123.45,
  "frame_seq": 58,
  "source": "yoloe:botsort",
  "reason": "",
  "timings": {
    "decode_ms": 2.1,
    "lock_wait_ms": 0.0,
    "model_track_ms": 45.8,
    "extract_candidates_ms": 0.4,
    "select_target_ms": 0.1,
    "candidate_count": 2.0,
    "total_ms": 49.2
  }
}
```

## 注意

- `api.py` 优先满足动态 label 和 tracking，不使用 TensorRT engine；固定词表 TensorRT 链路请使用 `tensorrt-api.py` 或 `start_tensorrt_api.sh`。
- `label` 不需要是 COCO 类别，但 YOLOE 对过长或过抽象描述的检测稳定性取决于模型自身能力。
- 如果从 `tree` 切换到 `black chair` 等新目标，切换首帧的 `timings` 中应出现 `reload_model_ms`；后续帧 `label_changed=False` 只表示服务端当前状态已经切到新 label。若切换时客户端超时或看似卡住，优先把调用方 timeout 提高到 30 秒以上，并查看 API 控制台的 `[YOLOE_RELOAD] start/done` 日志。
- 默认只返回 bbox/id，不把 mask 编码进 API JSON，避免高频接口过重。
- 实际系统中可以直接从其他模块拿到 image 和 label 后调用 `/track`，不需要使用 `test.py`。

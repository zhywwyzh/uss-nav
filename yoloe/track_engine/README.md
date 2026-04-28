# YOLOE Track Engine

本目录提供一个独立的 YOLOE 文本目标跟踪服务，不修改原有 `predict_realtime_cam_sim.py`。

## 功能

- `api.py` 只负责启动 YOLOE 模型、接收外部传入的 `image + label`、运行 tracking 并返回结果。
- `test.py` 是 ROS 测试客户端，负责订阅相机图像、调用 API、绘制跟踪结果并发布 `/nav_mission_image`。
- API 使用 `YOLOE.set_classes([label], model.get_text_pe([label]))` 动态切换文本目标。
- API 使用 Ultralytics `model.track(..., persist=True)` 输出 `bbox + track_id`。

## 启动 API

```bash
cd /home/gwq/workspace/VLA_Diff/ros_ws/uss-nav/yoloe
bash track_engine/start_yoloe_tracking_api.sh
```

可通过环境变量覆盖常用参数：

```bash
YOLOE_TRACK_PORT=2250 \
YOLOE_TRACK_DEVICE=cuda:0 \
bash track_engine/start_yoloe_tracking_api.sh
```

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

当 `label` 改变、`tracker` 改变或 `reset=true` 时，API 会自动重置 tracker 并重新设置 YOLOE 文本类别。

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
  "reason": ""
}
```

## 注意

- 当前实现优先满足动态 label 和 tracking，不使用 TensorRT engine。
- `label` 不需要是 COCO 类别，但 YOLOE 对过长或过抽象描述的检测稳定性取决于模型自身能力。
- 默认只返回 bbox/id，不把 mask 编码进 API JSON，避免高频接口过重。
- 实际系统中可以直接从其他模块拿到 image 和 label 后调用 `/track`，不需要使用 `test.py`。

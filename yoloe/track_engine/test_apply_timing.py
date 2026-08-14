#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""独立测试 _apply_tracking_output 各步骤耗时。
在 orin-nx-noetic-gpu 容器内运行:
  source /opt/ros/noetic/setup.bash
  cd /gwq/copaw_agent_skill/VLA_Diff/run_main
  python3 /gwq/copaw_agent_skill/uss-nav/yoloe/track_engine/test_apply_timing.py
"""

from __future__ import annotations

import json
import sys
import time
from typing import Any

import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String

# ── 辅助工具 ──
_t0 = 0.0


def tick(label: str = "") -> None:
    global _t0
    _t0 = time.perf_counter()
    if label:
        print(f"  [{label}]", end="", flush=True)


def tock(label: str = "") -> float:
    global _t0
    elapsed = (time.perf_counter() - _t0) * 1000.0
    if label:
        print(f" {elapsed:.1f}ms")
    return elapsed


def publish_bbox_debug_image(bbox, image, pub):
    """模拟 agent_run 的画框 + JPEG 编码 + 发布。"""
    try:
        h, w = image.shape[:2]
        x1, y1, x2, y2 = [max(0, min(int(v), d-1)) for v, d in zip(bbox[:4], [w, h, w, h])]
        display = image.copy()
        cv2.rectangle(display, (x1, y1), (x2, y2), (0, 255, 0), 2)
        ok, buf = cv2.imencode(".jpg", display, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
        if ok:
            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format = "jpeg"
            msg.data = buf.tobytes()
            pub.publish(msg)
    except Exception as exc:
        print(f"  bbox_debug_image error: {exc}")


def publish_tracking_state(payload: dict, pub):
    """模拟 tracking 状态 JSON 发布。"""
    msg = String(data=json.dumps(payload, ensure_ascii=False))
    pub.publish(msg)


def compute_object_pose_from_bbox(bbox, depth_image, camera_info, image):
    """从 2D bbox + 深度图计算 3D 物体位姿（简化版）。"""
    if depth_image is None or camera_info is None:
        return None

    x1, y1, x2, y2 = [int(v) for v in bbox[:4]]
    h, w = depth_image.shape[:2]
    x1 = max(0, min(x1, w - 1))
    y1 = max(0, min(y1, h - 1))
    x2 = max(0, min(x2, w - 1))
    y2 = max(0, min(y2, h - 1))

    roi = depth_image[y1:y2, x1:x2]
    if roi.size == 0:
        return None

    # 取区域中值深度
    if len(roi.shape) == 3:
        roi = roi[:, :, 0]
    valid = roi[roi > 0]
    if len(valid) == 0:
        return None
    depth_val = float(np.median(valid)) / 1000.0  # mm → m

    # 相机内参
    if isinstance(camera_info, dict):
        fx, fy = camera_info.get("fx", 500), camera_info.get("fy", 500)
        cx, cy = camera_info.get("cx", w / 2), camera_info.get("cy", h / 2)
    else:
        fx = fy = 500.0
        cx, cy = w / 2, h / 2

    # 2D 中心 → 3D
    center_x = (x1 + x2) / 2
    center_y = (y1 + y2) / 2
    x3d = (center_x - cx) * depth_val / fx
    y3d = (center_y - cy) * depth_val / fy
    z3d = depth_val

    return [float(x3d), float(y3d), float(z3d)]


# ── 主测试 ──

def main():
    rospy.init_node("test_apply_timing", anonymous=True)
    print("=" * 65)
    print("  _apply_tracking_output 各步骤耗时测试")
    print("=" * 65)

    # 初始化发布器
    nav_pub = rospy.Publisher("/nav_mission_image", CompressedImage, queue_size=2)
    state_pub = rospy.Publisher("/planning/tracking_result", String, queue_size=2)
    rospy.sleep(0.5)

    # 测试 bbox（模拟椅子检测）
    test_bbox = [37, 108, 202, 477]

    # 尝试获取图像帧
    image = None
    depth = None

    # 1. 尝试从已有 topic 订阅一帧
    try:
        print("\n[1] 等待 RGB 图像...")
        msg = rospy.wait_for_message("/camera/color/image_raw/compressed", CompressedImage, timeout=5.0)
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        print(f"    获取到 RGB: {image.shape}")
    except Exception as exc:
        print(f"    无法获取 RGB ({exc}), 使用合成图像")
        image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)

    # 2. 尝试获取深度图
    try:
        print("[2] 等待深度图像...")
        dmsg = rospy.wait_for_message("/camera/aligned_depth_to_color/image_raw", Image, timeout=5.0)
        depth = np.frombuffer(dmsg.data, dtype=np.uint16).reshape(dmsg.height, dmsg.width)
        print(f"    获取到深度: {depth.shape}")
    except Exception as exc:
        print(f"    无法获取深度 ({exc}), 跳过 3D 投影测试")
        depth = None

    print(f"\n  测试图像: {image.shape[1]}x{image.shape[0]}, 深度: {'有' if depth is not None else '无'}")
    print(f"  测试 bbox: {test_bbox}")
    print()

    # ═══════════════════════════════════════════════════
    # 开始分步计时
    # ═══════════════════════════════════════════════════

    ITERATIONS = 10
    results: dict[str, list[float]] = {
        "publish_debug_image": [],
        "compute_3d_pose": [],
        "publish_tracking_state": [],
    }

    for i in range(ITERATIONS):
        print(f"--- 第 {i+1}/{ITERATIONS} 次 ---")

        # Step A: 画框 + JPEG 编码 + ROS publish
        tick("A.publish_bbox_debug")
        publish_bbox_debug_image(test_bbox, image, nav_pub)
        t_a = tock()
        results["publish_debug_image"].append(t_a)

        # Step B: 3D 投影
        tick("B.compute_3d_pose   ")
        pose = None
        if depth is not None:
            # 模拟真实的深度投影链路
            b_t0 = time.perf_counter()
            # 2D bbox → 取深度区域
            x1, y1, x2, y2 = [int(v) for v in test_bbox[:4]]
            h, w_img = image.shape[:2]
            x1, y1 = max(0, min(x1, w_img - 1)), max(0, min(y1, h - 1))
            x2, y2 = max(0, min(x2, w_img - 1)), max(0, min(y2, h - 1))
            # 深度图可能和 RGB 尺寸不同，需要缩放
            if depth.shape[0] != h or depth.shape[1] != w_img:
                d_h, d_w = depth.shape[:2]
                sx1 = int(x1 * d_w / w_img)
                sy1 = int(y1 * d_h / h)
                sx2 = int(x2 * d_w / w_img)
                sy2 = int(y2 * d_h / h)
            else:
                sx1, sy1, sx2, sy2 = x1, y1, x2, y2
            roi = depth[sy1:sy2, sx1:sx2]
            if roi.size > 0:
                valid = roi[roi > 0]
                if len(valid) > 0:
                    depth_val = float(np.median(valid.astype(np.float64))) / 1000.0
                    # 简单针孔投影
                    fx, fy = 500.0, 500.0
                    cx, cy = w_img / 2, h / 2
                    cx_b = (x1 + x2) / 2
                    cy_b = (y1 + y2) / 2
                    pose = [
                        float((cx_b - cx) * depth_val / fx),
                        float((cy_b - cy) * depth_val / fy),
                        float(depth_val),
                    ]
            b_elapsed = (time.perf_counter() - b_t0) * 1000.0
        else:
            b_elapsed = 0.0
        print(f" {b_elapsed:.1f}ms → pose={pose}")
        results["compute_3d_pose"].append(b_elapsed)

        # Step C: JSON 序列化 + ROS String publish
        tick("C.publish_state      ")
        payload = {
            "active": True,
            "track_id": 1,
            "bbox_xyxy": test_bbox,
            "score": 0.85,
            "object_pose": pose,
            "source": "test",
            "stamp": time.time(),
        }
        publish_tracking_state(payload, state_pub)
        t_c = tock()
        results["publish_tracking_state"].append(t_c)

        rospy.sleep(0.1)

    # ═══════════════════════════════════════════════════
    # 汇总
    # ═══════════════════════════════════════════════════
    import statistics

    print()
    print("=" * 65)
    print("  汇总 (各步骤 " + str(ITERATIONS) + " 次平均)")
    print("=" * 65)
    print(f"{'步骤':<30} {'avg':>7} {'min':>7} {'max':>7} {'p50':>7}")
    print("-" * 65)

    total = 0
    for name, vals in results.items():
        if not vals:
            continue
        avg = statistics.mean(vals)
        mn = min(vals)
        mx = max(vals)
        p50 = statistics.median(vals)
        total += avg
        pct = avg / max(total, 0.001) * 100
        bar = "█" * max(1, int(pct / 3)) if pct > 1 else ""
        print(f"{name:<30} {avg:6.1f}ms {mn:6.1f}ms {mx:6.1f}ms {p50:6.1f}ms {pct:.0f}% {bar}")

    print("-" * 65)
    print(f"{'★ apply 总耗时 (模拟)':<30} {total:6.1f}ms")
    print()

    # 额外：测试裸 JPEG 编码开销
    print("── 附加测试：裸 JPEG 编码 ──")
    jpeg_times = []
    for _ in range(20):
        t0 = time.perf_counter()
        ok, buf = cv2.imencode(".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
        jpeg_times.append((time.perf_counter() - t0) * 1000.0)
    print(f"  JPEG 编码: avg={statistics.mean(jpeg_times):.1f}ms  "
          f"p50={statistics.median(jpeg_times):.1f}ms  "
          f"size={'OK' if ok else 'FAIL'}")

    print("\n  完成。")


if __name__ == "__main__":
    main()

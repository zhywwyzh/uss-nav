#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""YOLOE tracking API 的 ROS 测试客户端。

该脚本负责：
1. 从 ROS 压缩图像话题读取图像。
2. 将图像和测试 label 发送给 api.py。
3. 根据 API 返回的 bbox/track_id 绘制可视化图。
4. 将可视化图发布到 /nav_mission_image。
"""

from __future__ import annotations

import argparse
import base64
import threading
import time
from dataclasses import dataclass
from typing import Any

import cv2
import numpy as np
import requests
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, Image


@dataclass
class LatestImage:
    """测试客户端只保留最新图像，避免 API 处理慢时积压历史帧。"""

    seq: int
    stamp: float
    bgr: np.ndarray
    encoded: bytes


class YoloeTrackTestClient:
    """ROS 图像到 YOLOE tracking API 的测试桥接。"""

    def __init__(
        self,
        *,
        api_url: str,
        label: str,
        image_topic: str,
        output_topic: str,
        tracker: str,
        conf: float,
        iou: float,
        imgsz: int,
        rate_hz: float,
        timeout: float,
        init_bbox: list[float] | None,
    ) -> None:
        self.api_url = api_url.rstrip("/")
        self.label = label
        self.image_topic = image_topic
        self.output_topic = output_topic
        self.tracker = tracker
        self.conf = float(conf)
        self.iou = float(iou)
        self.imgsz = int(imgsz)
        self.rate_hz = float(rate_hz)
        self.timeout = float(timeout)
        self.init_bbox = init_bbox

        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.latest_image: LatestImage | None = None
        self.seq = 0
        self.last_processed_seq = 0
        self.last_result: dict[str, Any] | None = None

        self.image_pub = rospy.Publisher(self.output_topic, Image, queue_size=2)
        self.image_sub = rospy.Subscriber(
            self.image_topic,
            CompressedImage,
            self._image_callback,
            queue_size=1,
            buff_size=2**24,
        )

    def _image_callback(self, msg: CompressedImage) -> None:
        try:
            image_bgr = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as exc:
            rospy.logwarn("track test image decode failed: %s", exc)
            return

        encoded = bytes(msg.data)
        with self.lock:
            self.seq += 1
            self.latest_image = LatestImage(
                seq=self.seq,
                stamp=msg.header.stamp.to_sec(),
                bgr=image_bgr,
                encoded=encoded,
            )

    def spin(self) -> None:
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            packet = self._take_latest_unprocessed()
            if packet is None:
                rate.sleep()
                continue

            result = self._send_to_api(packet)
            vis = self._draw_result(packet.bgr.copy(), result)
            self._publish_image(vis)
            rate.sleep()

    def _take_latest_unprocessed(self) -> LatestImage | None:
        with self.lock:
            if self.latest_image is None or self.latest_image.seq <= self.last_processed_seq:
                return None
            packet = self.latest_image
            self.last_processed_seq = packet.seq
            return packet

    def _send_to_api(self, packet: LatestImage) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "image_base64": base64.b64encode(packet.encoded).decode("ascii"),
            "label": self.label,
            "stamp": packet.stamp,
            "tracker": self.tracker,
            "conf": self.conf,
            "iou": self.iou,
            "imgsz": self.imgsz,
        }
        if self.init_bbox is not None:
            payload["init_bbox"] = self.init_bbox

        try:
            response = requests.post(f"{self.api_url}/track", json=payload, timeout=self.timeout)
            response.raise_for_status()
            result = response.json()
            self.last_result = result
            return result
        except Exception as exc:
            rospy.logwarn("YOLOE tracking API request failed: %s", exc)
            return {
                "ok": False,
                "state": "api_error",
                "label": self.label,
                "track_id": None,
                "bbox": None,
                "score": 0.0,
                "reason": str(exc),
            }

    def _draw_result(self, image_bgr: np.ndarray, result: dict[str, Any]) -> np.ndarray:
        ok = bool(result.get("ok", False))
        bbox = result.get("bbox")
        label = str(result.get("label") or self.label)
        state = str(result.get("state") or "")
        reason = str(result.get("reason") or "")
        score = float(result.get("score") or 0.0)
        track_id = result.get("track_id")

        color = (0, 220, 0) if ok else (0, 170, 255)
        if bbox is not None and len(bbox) >= 4:
            x1, y1, x2, y2 = [int(v) for v in bbox[:4]]
            cv2.rectangle(image_bgr, (x1, y1), (x2, y2), color, 2)
            text = f"{label} id={track_id} {score:.2f}" if track_id is not None else f"{label} {score:.2f}"
            cv2.putText(
                image_bgr,
                text,
                (x1, max(20, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                color,
                2,
                cv2.LINE_AA,
            )

        status = f"{state}: {reason}" if reason else state
        cv2.putText(
            image_bgr,
            status,
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            color,
            2,
            cv2.LINE_AA,
        )
        return image_bgr

    def _publish_image(self, image_bgr: np.ndarray) -> None:
        try:
            msg = self.bridge.cv2_to_imgmsg(image_bgr, encoding="bgr8")
            msg.header.stamp = rospy.Time.now()
            self.image_pub.publish(msg)
        except CvBridgeError as exc:
            rospy.logwarn("track test image publish failed: %s", exc)


def _parse_bbox(text: str) -> list[float] | None:
    if not text:
        return None
    parts = [p.strip() for p in text.split(",")]
    if len(parts) != 4:
        raise ValueError("--init-bbox must be x1,y1,x2,y2")
    return [float(v) for v in parts]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="ROS test client for YOLOE tracking API")
    parser.add_argument("--api-url", default="http://127.0.0.1:2250")
    parser.add_argument("--label", default="red car")
    parser.add_argument("--image-topic", default="/camera1/color/image/compressed")
    parser.add_argument("--output-topic", default="/nav_mission_image")
    parser.add_argument("--tracker", default="botsort", choices=["botsort", "bytetrack"])
    parser.add_argument("--conf", type=float, default=0.1)
    parser.add_argument("--iou", type=float, default=0.5)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--rate", type=float, default=10.0)
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--init-bbox", default="", help="可选 x1,y1,x2,y2")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rospy.init_node("yoloe_track_test_client", anonymous=True)
    client = YoloeTrackTestClient(
        api_url=args.api_url,
        label=args.label,
        image_topic=args.image_topic,
        output_topic=args.output_topic,
        tracker=args.tracker,
        conf=args.conf,
        iou=args.iou,
        imgsz=args.imgsz,
        rate_hz=args.rate,
        timeout=args.timeout,
        init_bbox=_parse_bbox(args.init_bbox),
    )
    # 留出短暂时间让 publisher/subscriber 完成注册。
    time.sleep(0.2)
    client.spin()


if __name__ == "__main__":
    main()

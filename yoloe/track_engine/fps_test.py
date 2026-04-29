#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""YOLOE tracking API 的 ROS 帧率测试客户端。

该脚本和 test.py 使用相同的图像订阅与可视化发布方式，但重点统计：
1. 相机输入 FPS。
2. 实际请求 API 的处理 FPS。
3. 客户端 HTTP 往返耗时。
4. API 返回的服务端分段耗时。
"""

from __future__ import annotations

import argparse
import base64
import threading
import time
from collections import deque
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
    """只保留最新图像，避免 API 慢时积压历史帧影响测速。"""

    seq: int
    stamp: float
    wall_time: float
    bgr: np.ndarray
    encoded: bytes


class RollingStats:
    """固定时间窗统计 FPS 与延迟均值。"""

    def __init__(self, window_sec: float) -> None:
        self.window_sec = max(0.5, float(window_sec))
        self.events = deque()
        self.values = deque()

    def add_event(self, now: float | None = None) -> None:
        now = time.time() if now is None else float(now)
        self.events.append(now)
        self._trim(now)

    def add_value(self, value: float, now: float | None = None) -> None:
        now = time.time() if now is None else float(now)
        self.values.append((now, float(value)))
        self._trim(now)

    def fps(self) -> float:
        self._trim(time.time())
        if len(self.events) < 2:
            return 0.0
        span = max(1e-6, float(self.events[-1]) - float(self.events[0]))
        return float(len(self.events) - 1) / span

    def mean(self) -> float:
        self._trim(time.time())
        if not self.values:
            return 0.0
        return float(sum(v for _, v in self.values) / len(self.values))

    def _trim(self, now: float) -> None:
        min_t = float(now) - self.window_sec
        while self.events and float(self.events[0]) < min_t:
            self.events.popleft()
        while self.values and float(self.values[0][0]) < min_t:
            self.values.popleft()


class YoloeTrackFpsClient:
    """订阅 ROS 图像并持续请求 YOLOE tracking API，统计端到端性能。"""

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
        prompt_mode: str,
        update_visual_prompt: bool,
        visual_prompt_reset_tracker: bool,
        stats_window: float,
        log_interval: float,
    ) -> None:
        self.api_url = api_url.rstrip("/")
        self.label = str(label)
        self.image_topic = str(image_topic)
        self.output_topic = str(output_topic)
        self.tracker = str(tracker)
        self.conf = float(conf)
        self.iou = float(iou)
        self.imgsz = int(imgsz)
        self.rate_hz = float(rate_hz)
        self.timeout = float(timeout)
        self.init_bbox = init_bbox
        self.prompt_mode = str(prompt_mode)
        self.update_visual_prompt = bool(update_visual_prompt)
        self.visual_prompt_reset_tracker = bool(visual_prompt_reset_tracker)
        self.log_interval = max(0.5, float(log_interval))

        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.latest_image: LatestImage | None = None
        self.seq = 0
        self.last_processed_seq = 0
        self.request_count = 0
        self.last_result: dict[str, Any] | None = None
        self.last_log_time = 0.0

        self.input_stats = RollingStats(stats_window)
        self.process_stats = RollingStats(stats_window)
        self.rtt_stats = RollingStats(stats_window)
        self.server_total_stats = RollingStats(stats_window)
        self.server_model_stats = RollingStats(stats_window)

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
            rospy.logwarn("fps test image decode failed: %s", exc)
            return

        now = time.time()
        self.input_stats.add_event(now)
        with self.lock:
            self.seq += 1
            self.latest_image = LatestImage(
                seq=self.seq,
                stamp=msg.header.stamp.to_sec(),
                wall_time=now,
                bgr=image_bgr,
                encoded=bytes(msg.data),
            )

    def spin(self) -> None:
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            packet = self._take_latest_unprocessed()
            if packet is None:
                rate.sleep()
                continue

            result, rtt_ms = self._send_to_api(packet)
            self._update_stats(result, rtt_ms)
            vis = self._draw_result(packet.bgr.copy(), result, rtt_ms)
            self._publish_image(vis)
            self._log_stats_if_due(result)
            rate.sleep()

    def _take_latest_unprocessed(self) -> LatestImage | None:
        with self.lock:
            if self.latest_image is None or self.latest_image.seq <= self.last_processed_seq:
                return None
            packet = self.latest_image
            self.last_processed_seq = packet.seq
            return packet

    def _send_to_api(self, packet: LatestImage) -> tuple[dict[str, Any], float]:
        self.request_count += 1
        payload: dict[str, Any] = {
            "image_base64": base64.b64encode(packet.encoded).decode("ascii"),
            "label": self.label,
            "stamp": packet.stamp,
            "tracker": self.tracker,
            "prompt_mode": self.prompt_mode,
            "update_visual_prompt": bool(self.update_visual_prompt),
            "visual_prompt_reset_tracker": bool(self.visual_prompt_reset_tracker),
            "conf": self.conf,
            "iou": self.iou,
            "imgsz": self.imgsz,
        }
        if self.init_bbox is not None:
            payload["init_bbox"] = self.init_bbox
        if self.request_count == 1:
            payload["reset"] = True

        t0 = time.perf_counter()
        try:
            response = requests.post(f"{self.api_url}/track", json=payload, timeout=self.timeout)
            rtt_ms = (time.perf_counter() - t0) * 1000.0
            if not response.ok:
                detail = self._format_error_response(response)
                raise RuntimeError(f"HTTP {response.status_code}: {detail}")
            result = response.json()
            self.last_result = result
            return result, rtt_ms
        except Exception as exc:
            rtt_ms = (time.perf_counter() - t0) * 1000.0
            rospy.logwarn("YOLOE tracking API fps request failed: %s", exc)
            return {
                "ok": False,
                "state": "api_error",
                "label": self.label,
                "track_id": None,
                "bbox": None,
                "score": 0.0,
                "reason": str(exc),
                "timings": {},
            }, rtt_ms

    def _format_error_response(self, response: requests.Response) -> str:
        """提取 FastAPI 返回的 detail，避免只看到 400 Bad Request。"""
        try:
            body = response.json()
            detail = body.get("detail", body)
        except Exception:
            detail = response.text
        return str(detail)[:500]

    def _update_stats(self, result: dict[str, Any], rtt_ms: float) -> None:
        now = time.time()
        self.process_stats.add_event(now)
        self.rtt_stats.add_value(rtt_ms, now)
        timings = result.get("timings") or {}
        self.server_total_stats.add_value(float(timings.get("total_ms") or 0.0), now)
        self.server_model_stats.add_value(float(timings.get("model_track_ms") or 0.0), now)

    def _draw_result(self, image_bgr: np.ndarray, result: dict[str, Any], rtt_ms: float) -> np.ndarray:
        ok = bool(result.get("ok", False))
        bbox = result.get("bbox")
        label = str(result.get("label") or self.label)
        state = str(result.get("state") or "")
        reason = str(result.get("reason") or "")
        score = float(result.get("score") or 0.0)
        track_id = result.get("track_id")
        timings = result.get("timings") or {}

        color = (0, 220, 0) if ok else (0, 170, 255)
        if bbox is not None and len(bbox) >= 4:
            x1, y1, x2, y2 = [int(v) for v in bbox[:4]]
            cv2.rectangle(image_bgr, (x1, y1), (x2, y2), color, 2)
            text = f"{label} id={track_id} {score:.2f}" if track_id is not None else f"{label} {score:.2f}"
            cv2.putText(image_bgr, text, (x1, max(20, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        lines = [
            f"in={self.input_stats.fps():.1f}Hz api={self.process_stats.fps():.1f}Hz rtt={rtt_ms:.1f}ms",
            f"srv_total={float(timings.get('total_ms') or 0.0):.1f}ms model={float(timings.get('model_track_ms') or 0.0):.1f}ms",
            f"decode={float(timings.get('decode_ms') or 0.0):.1f}ms candidates={int(timings.get('candidate_count') or 0)}",
            f"{state}: {reason}" if reason else state,
        ]
        y = 26
        for line in lines:
            cv2.putText(image_bgr, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.62, color, 2, cv2.LINE_AA)
            y += 26
        return image_bgr

    def _publish_image(self, image_bgr: np.ndarray) -> None:
        try:
            msg = self.bridge.cv2_to_imgmsg(image_bgr, encoding="bgr8")
            msg.header.stamp = rospy.Time.now()
            self.image_pub.publish(msg)
        except CvBridgeError as exc:
            rospy.logwarn("fps test image publish failed: %s", exc)

    def _log_stats_if_due(self, result: dict[str, Any]) -> None:
        now = time.time()
        if now - self.last_log_time < self.log_interval:
            return
        self.last_log_time = now
        timings = result.get("timings") or {}
        rospy.loginfo(
            "[YOLOE-FPS] input=%.2fHz api=%.2fHz rtt_avg=%.1fms server_avg=%.1fms model_avg=%.1fms "
            "last_total=%.1fms last_model=%.1fms last_decode=%.1fms ok=%s track_id=%s bbox=%s score=%.3f state=%s reason=%s",
            self.input_stats.fps(),
            self.process_stats.fps(),
            self.rtt_stats.mean(),
            self.server_total_stats.mean(),
            self.server_model_stats.mean(),
            float(timings.get("total_ms") or 0.0),
            float(timings.get("model_track_ms") or 0.0),
            float(timings.get("decode_ms") or 0.0),
            str(bool(result.get("ok", False))),
            str(result.get("track_id")),
            str(result.get("bbox")),
            float(result.get("score") or 0.0),
            str(result.get("state") or ""),
            str(result.get("reason") or ""),
        )


def _parse_bbox(text: str) -> list[float] | None:
    if not text:
        return None
    parts = [p.strip() for p in text.split(",")]
    if len(parts) != 4:
        raise ValueError("--init-bbox must be x1,y1,x2,y2")
    return [float(v) for v in parts]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="ROS FPS test client for YOLOE tracking API")
    parser.add_argument("--api-url", default="http://127.0.0.1:2250")
    parser.add_argument("--label", default="red car")
    parser.add_argument("--image-topic", default="/camera1/color/image/compressed")
    parser.add_argument("--output-topic", default="/nav_mission_image")
    parser.add_argument("--tracker", default="botsort", choices=["botsort", "bytetrack"])
    parser.add_argument("--prompt-mode", default="text", choices=["text", "visual", "hybrid"])
    parser.add_argument("--update-visual-prompt", action="store_true")
    parser.add_argument("--visual-prompt-reset-tracker", action="store_true")
    parser.add_argument("--conf", type=float, default=0.1)
    parser.add_argument("--iou", type=float, default=0.5)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--rate", type=float, default=30.0, help="客户端轮询最新图像的最高频率")
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--stats-window", type=float, default=5.0)
    parser.add_argument("--log-interval", type=float, default=1.0)
    parser.add_argument("--init-bbox", default="", help="可选 x1,y1,x2,y2，用于首帧绑定目标")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rospy.init_node("yoloe_track_fps_test", anonymous=True)
    client = YoloeTrackFpsClient(
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
        prompt_mode=args.prompt_mode,
        update_visual_prompt=args.update_visual_prompt,
        visual_prompt_reset_tracker=args.visual_prompt_reset_tracker,
        stats_window=args.stats_window,
        log_interval=args.log_interval,
    )
    time.sleep(0.2)
    client.spin()


if __name__ == "__main__":
    main()

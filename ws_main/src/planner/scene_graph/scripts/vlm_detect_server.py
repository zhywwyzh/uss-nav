#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""VLM 检测节点：在 EXPLORATION 找物任务中替代 YOLOE，逐帧用多模态大模型检测目标。

输入:  RGB + Depth + Odom (message_filters 时间同步，保证每组数据同一时刻)
输出:  EncodeMask → /yoloe/encodemask (与 YOLOE 完全相同的话题和格式)

target 来源: /bridge/Instruct (agent 下发的统一指令)
  - TURN_OBJECT_NAV (1): command="sofa" → 设为 target，开始逐帧 VLM 查询
  - TURN_REGULAR_EXPLORATION (3): 纯探索 → 清空 target
"""

import base64
import os
import threading
import time

import cv2
import numpy as np
import rospy
import message_filters
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry
from openai import OpenAI
from sensor_msgs.msg import CompressedImage, Image
from quadrotor_msgs.msg import Instruction

try:
    from scene_graph.msg import EncodeMask, WordVector
except ImportError:
    rospy.logwarn("[VLM_DETECT] scene_graph msgs not found, trying std_msgs")
    from std_msgs.msg import EncodeMask, WordVector


# 与 ego-planner 对齐: param_states.py INSTRUCTION_TYPE
TURN_OBJECT_NAV = 1
TURN_OBJECT_ID_NAV = 2
TURN_REGULAR_EXPLORATION = 3


class VlmDetectServer:

    def __init__(self):
        # ---- 输入话题 (与 YOLOE 相同) ----
        rgb_topic   = rospy.get_param("~rgb_topic",   "/camera1/color_sync/image/compressed")
        depth_topic = rospy.get_param("~depth_topic", "/camera1/depth_sync/image/compressed")
        odom_topic  = rospy.get_param("~odom_topic",  "/unity_odom_sync")

        # ---- 输出话题 (与 YOLOE 相同) ----
        encodemask_topic = rospy.get_param("~encodemask_topic", "/yoloe/encodemask")
        self.result_pub = rospy.Publisher(encodemask_topic, EncodeMask, queue_size=5)

        # ---- VLM 配置 ----
        vlm_base_url = rospy.get_param("~vlm_base_url", "http://127.0.0.1:2230/v1")
        vlm_api_key_env = rospy.get_param("~vlm_api_key_env", "DOUBAO_API_KEY")
        vlm_api_key = os.environ.get(vlm_api_key_env, "EMPTY")
        self.vlm_temperature = rospy.get_param("~vlm_temperature", 0.8)
        self.resize_width = rospy.get_param("~resize_width", 640)

        # ---- 节流: 两次 VLM 查询最小间隔 ----
        self.query_interval_s = rospy.get_param("~query_interval_s", 2.0)
        self.last_query_time = 0.0
        self.inflight = False

        # ---- 当前目标 (从 /bridge/Instruct 获取) ----
        self.target_object = ""
        rospy.Subscriber("/bridge/Instruct", Instruction,
                         self._instruction_callback, queue_size=2)

        # ---- 初始化 ----
        os.environ.pop("HTTP_PROXY", None)
        os.environ.pop("HTTPS_PROXY", None)
        os.environ.pop("http_proxy", None)
        os.environ.pop("https_proxy", None)
        self.client = OpenAI(api_key=vlm_api_key, base_url=vlm_base_url)
        self.cv_bridge = CvBridge()

        self._is_rgb_compressed   = "compressed" in rgb_topic
        self._is_depth_compressed = "compressed" in depth_topic

        rospy.loginfo("[VLM_DETECT] init: base_url=%s rgb=%s depth=%s odom=%s encodemask=%s "
                      "interval=%.1fs",
                      vlm_base_url, rgb_topic, depth_topic, odom_topic,
                      encodemask_topic, self.query_interval_s)

    # ---- target 获取 ----

    def _instruction_callback(self, msg: Instruction):
        itype = int(getattr(msg, "instruction_type", -1))
        cmd = str(getattr(msg, "command", "") or "").strip()

        if itype in (TURN_OBJECT_NAV, TURN_OBJECT_ID_NAV):
            if cmd and cmd != self.target_object:
                self.target_object = cmd
                rospy.loginfo("[VLM_DETECT] target: %r", cmd)
        elif itype == TURN_REGULAR_EXPLORATION and self.target_object:
            rospy.loginfo("[VLM_DETECT] clear target (was %r)", self.target_object)
            self.target_object = ""

    # ---- 时间同步入口 ----

    def start(self):
        if self._is_rgb_compressed:
            rgb_sub = message_filters.Subscriber(
                rospy.get_param("~rgb_topic", "/camera1/color_sync/image/compressed"),
                CompressedImage)
        else:
            rgb_sub = message_filters.Subscriber(
                rospy.get_param("~rgb_topic", "/camera1/color_sync/image/compressed"),
                Image)

        if self._is_depth_compressed:
            depth_sub = message_filters.Subscriber(
                rospy.get_param("~depth_topic", "/camera1/depth_sync/image/compressed"),
                CompressedImage)
        else:
            depth_sub = message_filters.Subscriber(
                rospy.get_param("~depth_topic", "/camera1/depth_sync/image/compressed"),
                Image)

        odom_sub = message_filters.Subscriber(
            rospy.get_param("~odom_topic", "/unity_odom_sync"), Odometry)

        slop = rospy.get_param("~time_slop", 0.05)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub, odom_sub], queue_size=50, slop=slop)
        self.ts.registerCallback(self.synced_callback)
        rospy.loginfo("[VLM_DETECT] started, slop=%.3f", slop)

    def synced_callback(self, rgb_msg, depth_msg, odom_msg):
        if not self.target_object or self.inflight:
            return
        now = rospy.Time.now().to_sec()
        if now - self.last_query_time < self.query_interval_s:
            return
        self.last_query_time = now
        self.inflight = True

        # ---- 解码本组数据 ----
        try:
            if self._is_rgb_compressed:
                cv_rgb = self.cv_bridge.compressed_imgmsg_to_cv2(rgb_msg, "bgr8")
            else:
                cv_rgb = self.cv_bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            cv_rgb = cv2.cvtColor(cv_rgb, cv2.COLOR_BGR2RGB)
        except CvBridgeError as e:
            rospy.logerr("[VLM_DETECT] rgb decode: %s", e)
            self.inflight = False
            return

        try:
            if self._is_depth_compressed:
                depth_compressed = depth_msg
            else:
                cv_depth = self.cv_bridge.imgmsg_to_cv2(depth_msg, "passthrough")
                depth_compressed = self.cv_bridge.cv2_to_compressed_imgmsg(
                    cv_depth, dst_format="png")
        except CvBridgeError as e:
            rospy.logerr("[VLM_DETECT] depth decode: %s", e)
            self.inflight = False
            return

        h, w = cv_rgb.shape[:2]
        if w != self.resize_width:
            cv_rgb = cv2.resize(cv_rgb, (self.resize_width, int(h * self.resize_width / w)))

        p = odom_msg.pose.pose.position
        q = odom_msg.pose.pose.orientation
        odom_arr = np.array([p.x, p.y, p.z, q.x, q.y, q.z, q.w], dtype=np.float64)

        view = {
            "rgb": cv_rgb,
            "rgb_original": cv_rgb,
            "rgb_msg": rgb_msg,
            "depth_compressed": depth_compressed,
            "odom": odom_arr,
            "header": rgb_msg.header,
        }
        threading.Thread(target=self._vlm_query, args=(view,), daemon=True).start()

    # ---- VLM 两步查询 ----

    def _vlm_query(self, view: dict):
        try:
            visible = self._check_visible(view["rgb"], self.target_object)
            if not visible:
                return
            bbox = self._get_bbox(view["rgb"], self.target_object)
            if bbox is None or len(bbox) < 4:
                return
            self._publish_encodemask(view, bbox)
            rospy.loginfo("[VLM_DETECT] %r found bbox=%s", self.target_object, bbox)
        except Exception as e:
            rospy.logwarn("[VLM_DETECT] query error: %s", e)
        finally:
            self.inflight = False

    def _check_visible(self, rgb: np.ndarray, target: str) -> bool:
        _, buf = cv2.imencode(".jpg", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        img_b64 = base64.b64encode(buf).decode("utf-8")

        prompt = (
            f"<image>\n"
            f"Determine whether {target} is visible in this image. "
            f'Reply ONLY: {{"vis_first": true}} or {{"vis_first": false}}.'
        )
        resp = self.client.chat.completions.create(
            model=self._model_name(),
            messages=[{"role": "user", "content": [
                {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{img_b64}"}},
                {"type": "text", "text": prompt},
            ]}],
            temperature=self.vlm_temperature, max_tokens=128)
        content = self._strip_think(resp.choices[0].message.content)
        data, _ = self._try_parse_json(content)
        return bool(data.get("vis_first", False)) if isinstance(data, dict) else False

    def _get_bbox(self, rgb: np.ndarray, target: str):
        _, buf = cv2.imencode(".jpg", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        img_b64 = base64.b64encode(buf).decode("utf-8")

        prompt = (
            f"<image>\n"
            f"Return the bounding box of {target}. "
            f'Format: {{"bbox_2d": [x1, y1, x2, y2]}}.'
        )
        resp = self.client.chat.completions.create(
            model=self._model_name(),
            messages=[{"role": "user", "content": [
                {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{img_b64}"}},
                {"type": "text", "text": prompt},
            ]}],
            temperature=self.vlm_temperature, max_tokens=256)
        content = self._strip_think(resp.choices[0].message.content)

        import re as _re
        m = _re.search(r"\{[^{}]*\}", content)
        data, _ = self._try_parse_json(m.group() if m else content)
        bbox = data.get("bbox_2d") if isinstance(data, dict) else None
        if bbox is None or bbox == "none":
            return None

        x1, y1, x2, y2 = [int(v) for v in bbox[:4]]
        h, w = rgb.shape[:2]
        scale_x = w / 1000.0
        scale_y = h / 1000.0
        return [int(x1 * scale_x), int(y1 * scale_y), int(x2 * scale_x), int(y2 * scale_y)]

    # ---- 发布 EncodeMask ----

    def _publish_encodemask(self, view: dict, bbox: list):
        msg = EncodeMask()
        msg.header.stamp = view["header"].stamp
        msg.header.frame_id = "world"
        msg.current_odom = self._odom_msg(view["odom"])
        msg.current_depth = view["depth_compressed"]

        rgb_bgr = cv2.cvtColor(view["rgb_original"], cv2.COLOR_RGB2BGR)
        msg.current_rgb = self.cv_bridge.cv2_to_compressed_imgmsg(rgb_bgr, dst_format="jpeg")

        h, w = view["rgb_original"].shape[:2]
        mask = np.zeros((h, w), dtype=np.uint8)
        x1, y1, x2, y2 = [max(0, int(v)) for v in bbox]
        mask[y1:min(y2, h), x1:min(x2, w)] = 255
        mc = self.cv_bridge.cv2_to_compressed_imgmsg(mask, dst_format="png;params=[16,0]")
        mc.format = "png"
        msg.masks.append(mc)
        msg.labels.append(self.target_object)
        msg.confs.append(1.0)
        wv = WordVector()
        wv.word_vector = [0.0] * 512
        msg.word_vectors.append(wv)
        self.result_pub.publish(msg)

    # ---- 工具 ----

    def _model_name(self) -> str:
        try:
            return self.client.models.list().data[0].id
        except Exception:
            return "default"

    @staticmethod
    def _strip_think(text: str) -> str:
        import re as _re
        return _re.sub(r"<think>.*?</think>", "", text, flags=_re.DOTALL).strip()

    @staticmethod
    def _try_parse_json(text: str):
        import json as _json
        try:
            return _json.loads(text), 1
        except Exception:
            return {}, 0

    @staticmethod
    def _odom_msg(arr: np.ndarray) -> Odometry:
        m = Odometry()
        m.header.frame_id = "world"
        m.pose.pose.position.x = float(arr[0])
        m.pose.pose.position.y = float(arr[1])
        m.pose.pose.position.z = float(arr[2])
        m.pose.pose.orientation.x = float(arr[3])
        m.pose.pose.orientation.y = float(arr[4])
        m.pose.pose.orientation.z = float(arr[5])
        m.pose.pose.orientation.w = float(arr[6])
        return m


def main():
    rospy.init_node("vlm_detect_server", anonymous=True)
    try:
        node = VlmDetectServer()
        node.start()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
/yoloe/plot 转发节点（地面站侧）

背景: swarm bridge 将远端 /yoloe/plot (sensor_msgs/CompressedImage) 转发到本机后,
本机 rviz 的 Image 显示以 raw transport 订阅时期望 sensor_msgs/Image 类型,
与 CompressedImage 类型不匹配, 连接被 bridge 丢弃, rviz 显示 "No image received"。

本节点订阅本机 /yoloe/plot (CompressedImage), 解码后重新发布为标准的
raw sensor_msgs/Image 话题, rviz 直接订阅输出话题 (Transport Hint = raw) 即可正常显示。

话题通过私有参数/重映射配置:
  ~input   (默认 /yoloe/plot)      输入压缩图话题
  ~output  (默认 /yoloe/plot_std)  输出原始图像话题
"""
import io

import numpy as np
import rospy
from PIL import Image as PilImage
from sensor_msgs.msg import CompressedImage, Image


class YoloePlotRelay(object):
    """将 CompressedImage 解码后转发为 raw sensor_msgs/Image。"""

    def __init__(self):
        # 输入/输出话题通过 remap 配置, 便于 launch 文件灵活指定
        in_topic = rospy.get_param('~input', '/yoloe/plot')
        out_topic = rospy.get_param('~output', '/yoloe/plot_std')
        self.pub = rospy.Publisher(out_topic, Image, queue_size=2)
        self.sub = rospy.Subscriber(in_topic, CompressedImage, self._on_image, queue_size=2)
        rospy.loginfo('yoloe_plot_relay: %s -> %s' % (in_topic, out_topic))

    def _on_image(self, msg):
        try:
            # 解码 jpeg 压缩图并统一转为 RGB
            pil_img = PilImage.open(io.BytesIO(msg.data)).convert('RGB')
            arr = np.asarray(pil_img)

            out = Image()
            out.header = msg.header
            out.height, out.width = arr.shape[0], arr.shape[1]
            out.encoding = 'rgb8'
            out.is_bigendian = 0
            out.step = arr.shape[1] * 3
            out.data = arr.tobytes()
            self.pub.publish(out)
        except Exception as e:  # 解码失败只记录日志, 不影响主流程
            rospy.logwarn_throttle(5.0, 'decode %s failed: %s' % (self.sub.resolved_name, e))


if __name__ == '__main__':
    rospy.init_node('yoloe_plot_relay')
    relay = YoloePlotRelay()
    rospy.spin()

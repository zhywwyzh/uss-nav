#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
from nav_msgs.msg import Odometry

class OdomTfBroadcaster:
    def __init__(self):
        # 初始化节点
        rospy.init_node('odom_to_tf_broadcaster', anonymous=True)

        # 获取参数，方便在launch文件中修改
        # 默认订阅 '/odom' 话题
        self.odom_topic = rospy.get_param('~odom_topic', '/ekf_quat/ekf_odom')

        # 某些情况下你可能想强制指定 frame_id，可以在这里修改，
        # 否则默认使用 Odometry 消息自带的 frame_id
        self.parent_frame_id = rospy.get_param('~parent_frame', 'world')
        self.child_frame_id = rospy.get_param('~child_frame', 'body')

        # 初始化 TF 广播器
        self.tf_broadcaster = tf.TransformBroadcaster()

        # 订阅 Odometry 话题
        self.sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback)

        rospy.loginfo(f"Started Odom to TF broadcaster. Listening to: {self.odom_topic}")

    def odom_callback(self, msg):
        """
        回调函数：接收 Odometry 消息并发布 TF
        """
        # 1. 获取位置 (Translation)
        pos = msg.pose.pose.position
        translation = (pos.x, pos.y, pos.z)

        # 2. 获取姿态 (Rotation/Quaternion)
        ori = msg.pose.pose.orientation
        rotation = (ori.x, ori.y, ori.z, ori.w)

        # 3. 确定 Frame ID
        # 如果参数没指定，就用消息里自带的 frame_id (通常是 'odom' 和 'base_link')
        parent_frame = self.parent_frame_id if self.parent_frame_id else msg.header.frame_id
        child_frame = self.child_frame_id if self.child_frame_id else msg.child_frame_id

        # 4. 发布 TF 变换
        # 参数顺序: (平移, 旋转, 时间戳, 子坐标系, 父坐标系)
        self.tf_broadcaster.sendTransform(
            translation,
            rotation,
            msg.header.stamp, # 使用消息的时间戳以保持同步
            child_frame,      # 例如: base_link
            parent_frame      # 例如: odom 或 map
        )

if __name__ == '__main__':
    try:
        node = OdomTfBroadcaster()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
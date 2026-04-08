#! /usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

def filter_point_cloud(input_cloud, output_cloud, max_height):
    for p in pc2.read_points(input_cloud, field_names=("x", "y", "z"), skip_nans=True):
        if p[2] <= max_height:
            output_cloud.append([p[0], p[1], p[2]])

def point_cloud_callback(msg):
    filtered_cloud = []
    filter_point_cloud(msg, filtered_cloud, 4.0)  # 指定高度阈值为2.0m

    filtered_msg = pc2.create_cloud_xyz32(msg.header, filtered_cloud)
    filtered_msg.header.frame_id = "world"
    pub.publish(filtered_msg)

rospy.init_node("point_cloud_filter")
sub = rospy.Subscriber("/map_generator/global_cloud", PointCloud2, point_cloud_callback)
pub = rospy.Publisher("/map_generator/global_cloud/fixed", PointCloud2, queue_size=10)

rospy.spin()

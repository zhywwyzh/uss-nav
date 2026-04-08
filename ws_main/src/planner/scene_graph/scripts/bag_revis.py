#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import copy
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import ColorRGBA

class SceneGraphLayerAdjuster:
    def __init__(self):
        rospy.init_node('scenegraph_layer_adjuster', anonymous=True)
  
        # 默认层级偏移量 (Key: namespace, Value: Z-axis offset)
        # 你可以在运行时通过 rosparam 动态覆盖这些值
        self.default_offsets = {
            "top": 4.0 + 10.0,
            "area": 4.0 + 10.0,  
            "object": 6.5 + 10.0,   
            "hgrid": -2.0 + 10.0, 
            "path": -2.0 + 10.0,
            "area_box": 6.5 + 10.0,
            "topo": 0.0,
            "ftr": -2.0 + 10.0
        }
    

        # --- 初始化 Publisher 和 Subscriber ---
        # self.scenegraph_sub_ = rospy.Subscriber("/scene_graph/vis", MarkerArray, self.sceneGraphCallback, queue_size=10)
        # self.scenegraph_pub_ = rospy.Publisher("/scene_graph_revis/vis", MarkerArray, queue_size=10)

        # self.path_sub_ = rospy.Subscriber("/scene_graph/traj_with_vel", Marker, self.pathCallback, queue_size=10)
        # self.path_pub_ = rospy.Publisher("/scene_graph_revis/traj_with_vel", Marker, queue_size=10)

        # self.object_pt_cloud_sub_ = rospy.Subscriber("/object_pointcloud", PointCloud2, self.objectPtCloudCallback, queue_size=10)
        # self.object_pt_cloud_pub_ = rospy.Publisher("/object_pointcloud_revis", PointCloud2, queue_size=10)

        # self.area_info_sub_ = rospy.Subscriber("/skeleton/cluster_vis", MarkerArray, self.areaInfoCallback, queue_size=10)
        # self.area_info_pub_ = rospy.Publisher("/skeleton/cluster_vis_revis", MarkerArray, queue_size=10)

        self.topo_sub_ = rospy.Subscriber("/drone_0_ego_planner_node/skeleton_vis", MarkerArray, self.topoCallback, queue_size=10)
        self.topo_pub_ = rospy.Publisher("/drone_0_ego_planner_node/skeleton_vis_revis", MarkerArray, queue_size=10)

        # self.ftr_info_sub_ = rospy.Subscriber("/drone_0_ego_planner_node/frontier_info", MarkerArray, self.ftrInfoCallback, queue_size=10)
        # self.ftr_info_pub_ = rospy.Publisher("/drone_0_ego_planner_node/frontier_info_revis", MarkerArray, queue_size=10)

        # self.hgird_sub_ = rospy.Subscriber("/drone_0_ego_planner_node/hgrid_info", MarkerArray, self.hgridCallback, queue_size=10)
        # self.hgird_pub_ = rospy.Publisher("/drone_0_ego_planner_node/hgrid_info_revis", MarkerArray, queue_size=10)

    def sceneGraphCallback(self, msg):
        # --- 复制原始消息 ---
        msg_copy = copy.deepcopy(msg)

        msg_output = MarkerArray()

        # --- 遍历 MarkerArray ---
        for i in range(len(msg_copy.markers)):
            marker = msg_copy.markers[i]

            if ("top_level" in marker.ns):
                marker.pose.position.z += self.default_offsets["top"]
                msg_output.markers.append(marker)

            elif("room_level" in marker.ns):
                for j in range(len(marker.points)):
                    marker.points[j].z += self.default_offsets["area"]
                msg_output.markers.append(marker)

            elif("room_label_text" in marker.ns):
                marker.pose.position.z += self.default_offsets["area"]    
                marker.scale.x = 0.8
                marker.scale.y = 0.8
                marker.scale.z = 0.8
                msg_output.markers.append(marker)

            elif("room_top_edge" in marker.ns):
                for j in range(len(marker.points)):
                    if j%2 == 0:
                        marker.points[j].z += self.default_offsets["top"]
                    else:
                        marker.points[j].z += self.default_offsets["area"]
                marker.scale.x = 0.08
                marker.scale.y = 0.08
                marker.scale.z = 0.08
                msg_output.markers.append(marker)

            elif("obj_level" in marker.ns):
                for j in range(len(marker.points)):
                    if j%2 == 0:
                        marker.points[j].z += self.default_offsets["object"]
                    else:
                        marker.points[j].z += self.default_offsets["area"]
                marker.scale.x = 0.05
                marker.scale.y = 0.05
                marker.scale.z = 0.05
                msg_output.markers.append(marker)
        
        self.scenegraph_pub_.publish(msg_output)
    
    def pathCallback(self, msg:Marker):
        msg.scale.x = 0.08

        for i in range(len(msg.points)):
            msg.points[i].z += self.default_offsets["path"]
        self.path_pub_.publish(msg)

    def objectPtCloudCallback(self, msg: PointCloud2):
        # 1. 定义读取的字段
        # 注意：ROS中颜色通常是一个 packed float，字段名通常是 'rgb' 或 'rgba'
        read_fields = ("x", "y", "z", "rgb")
        
        # 2. 读取点云 (生成器)
        cloud_gen = pc2.read_points(msg, field_names=read_fields, skip_nans=True)
        
        new_points = []

        # 3. 遍历并修改 Z，同时保留 RGB
        for p in cloud_gen:
            x, y, z, rgb = p  # 解包数据
            
            # 修改 Z
            new_z = z + self.default_offsets["object"]
            
            # 存入列表
            new_points.append([x, y, new_z, rgb])

        # 4. 准备 Header 和 Fields 用于打包
        # 我们需要告诉 create_cloud 数据的格式
        # 也可以直接复用 msg.fields，但为了保险，这里显式定义常用格式
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            # RGB 通常也是作为一个 32位 float (或 int32) 存储
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        # 5. 创建新消息
        # 注意：这里使用 create_cloud 而不是 create_cloud_xyz32
        new_msg = pc2.create_cloud(msg.header, fields, new_points)
        self.object_pt_cloud_pub_.publish(new_msg)

    def areaInfoCallback(self, msg: MarkerArray):
        for i in range(len(msg.markers)):
            marker = msg.markers[i]
            if "box" in marker.ns:
                for j in range(len(marker.points)):
                    marker.points[j].z += self.default_offsets["area_box"]
                    
            elif "sphere" in marker.ns:
                for j in range(len(marker.points)):
                    marker.points[j].z += self.default_offsets["topo"]
        
        self.area_info_pub_.publish(msg)

    def topoCallback(self, msg: MarkerArray):
        for i in range(len(msg.markers)):
            # if "edge" in msg.markers[i].ns:
                # if "edges" in msg.markers[i].ns:
                #     msg.markers[i].color.r = 51.0 / 255.0
                #     msg.markers[i].color.g = 244 / 255.0
                #     msg.markers[i].color.b = 243 / 255.0
                #     msg.markers[i].scale.x = 0.05
                #     msg.markers[i].scale.y = 0.05
                #     msg.markers[i].scale.z = 0.05

                # for j in range(len(msg.markers[i].points)):
                #     msg.markers[i].points[j].z += self.default_offsets["topo"]

            if "facet" in msg.markers[i].ns:
                msg.markers[i].color.a = 0.5
                msg.markers[i].color.r = 51.0 / 255.0
                msg.markers[i].color.g = 244.0 / 255.0
                msg.markers[i].color.b = 243.0 / 255.0
                msg.markers[i].scale.x = 0.03
                msg.markers[i].scale.y = 0.03
                msg.markers[i].scale.z = 0.03
            
        self.topo_pub_.publish(msg)
    
    def ftrInfoCallback(self, msg: MarkerArray):
        for i in range(len(msg.markers)):
            if "cell" in msg.markers[i].ns:
                msg.markers[i].color.a = 0.7
                for j in range(len(msg.markers[i].points)):
                    msg.markers[i].points[j].z += self.default_offsets["ftr"]
        self.ftr_info_pub_.publish(msg)

    def hgridCallback(self, msg: MarkerArray):
        for i in range(len(msg.markers)):
            if "mesh" in msg.markers[i].ns:
               for j in range(len(msg.markers[i].points)):
                    msg.markers[i].points[j].z += self.default_offsets["hgrid"]

            if "state" in msg.markers[i].ns:
                msg.markers[i].pose.position.z += self.default_offsets["hgrid"]

        self.hgird_pub_.publish(msg)


if __name__ == '__main__':
    try:
        node = SceneGraphLayerAdjuster()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
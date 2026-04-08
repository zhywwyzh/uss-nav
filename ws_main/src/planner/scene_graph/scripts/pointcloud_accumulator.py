#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
点云累积和降采样脚本
功能：
1. 订阅指定topic的点云
2. 将点云进行累积
3. 按指定分辨率进行体素化降采样
4. 发布累积后的点云到新的topic (原topic名称 + '_dsp_all')
"""

"""
rosrun scene_graph pointcloud_accumulator.py \
  _input_topic:=/your/pointcloud/topic \
  _voxel_size:=0.05 \
  _publish_rate:=1.0
"""


import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from threading import Lock


class PointCloudAccumulator:
    def __init__(self, input_topic, voxel_size=0.1, x_min=-float('inf'), x_max=float('inf'),
                 y_min=-float('inf'), y_max=float('inf'), z_min=0.0, z_max=float('inf')):
        """
        初始化点云累积器
        
        Args:
            input_topic (str): 输入的点云topic名称
            voxel_size (float): 体素化分辨率（降采样单元大小）
            x_min (float): x轴最小阈值，小于此值的点将被过滤
            x_max (float): x轴最大阈值，大于此值的点将被过滤
            y_min (float): y轴最小阈值，小于此值的点将被过滤
            y_max (float): y轴最大阈值，大于此值的点将被过滤
            z_min (float): z轴最小阈值，小于此值的点将被过滤
            z_max (float): z轴最大阈值，大于此值的点将被过滤
        """
        self.input_topic = input_topic
        self.voxel_size = voxel_size
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max
        self.z_min = z_min
        self.z_max = z_max
        
        # 输出topic名称（原topic + '_dsp_all'）
        self.output_topic = input_topic + '_dsp_all'
        
        # 累积的点云数据
        self.accumulated_points = []
        self.accumulated_colors = []
        self.frame_id = 'map'
        
        # 线程锁，保证线程安全
        self.lock = Lock()
        
        # 创建subscriber和publisher
        self.subscriber = rospy.Subscriber(
            input_topic,
            PointCloud2,
            self.pointcloud_callback,
            queue_size=2
        )
        
        self.publisher = rospy.Publisher(
            self.output_topic,
            PointCloud2,
            queue_size=2
        )
        
        rospy.loginfo(f"点云累积器已启动")
        rospy.loginfo(f"输入topic: {self.input_topic}")
        rospy.loginfo(f"输出topic: {self.output_topic}")
        rospy.loginfo(f"降采样体素大小: {self.voxel_size} m")
        rospy.loginfo(f"x轴范围: [{self.x_min}, {self.x_max}] m")
        rospy.loginfo(f"y轴范围: [{self.y_min}, {self.y_max}] m")
        rospy.loginfo(f"z轴范围: [{self.z_min}, {self.z_max}] m")
    
    def pointcloud_callback(self, msg):
        """
        点云回调函数，接收新的点云并累积
        每次累积后都进行降采样，下一次在降采样点云的基础上继续累积
        
        Args:
            msg (PointCloud2): 接收到的点云消息
        """
        try:
            # 保存frame_id用于输出
            self.frame_id = msg.header.frame_id
            
            # 将PointCloud2消息转换为点坐标数组
            points = list(pc2.read_points(
                msg,
                skip_nans=True,
                field_names=("x", "y", "z")
            ))
            
            if len(points) == 0:
                rospy.logwarn("接收到空的点云")
                return
            
            # 检查是否有RGB或其他颜色信息
            has_color = False
            color_field = None
            if msg.fields:
                field_names = [f.name for f in msg.fields]
                if 'rgb' in field_names:
                    color_field = 'rgb'
                    has_color = True
                elif 'rgba' in field_names:
                    color_field = 'rgba'
                    has_color = True
            
            # 如果有颜色信息，获取颜色数据
            if has_color:
                points_with_color = list(pc2.read_points(
                    msg,
                    skip_nans=True,
                    field_names=("x", "y", "z", color_field)
                ))
                colors = np.array([p[3] for p in points_with_color], dtype=np.uint32)
                points = np.array([(p[0], p[1], p[2]) for p in points_with_color])
            else:
                colors = None
                points = np.array(points)
            
            with self.lock:
                # 累积点云
                if len(self.accumulated_points) == 0:
                    self.accumulated_points = points
                    if colors is not None:
                        self.accumulated_colors = colors
                else:
                    self.accumulated_points = np.vstack([
                        self.accumulated_points,
                        points
                    ])
                    if colors is not None and len(self.accumulated_colors) > 0:
                        self.accumulated_colors = np.hstack([
                            self.accumulated_colors,
                            colors
                        ])
                
                # 在降采样之前，先进行xyz轴范围滤波
                filtered_points, filtered_colors = self.filter_by_bounds(
                    self.accumulated_points,
                    self.accumulated_colors if len(self.accumulated_colors) > 0 else None
                )
                
                # 每次累积后立即进行降采样，以降采样后的点云作为下一次的累积基础
                downsampled_points, color_indices = self.voxel_downsample(
                    filtered_points,
                    self.voxel_size
                )
                
                rospy.loginfo(f"累积后点数: {len(self.accumulated_points)}, 滤波后点数: {len(filtered_points)}, 降采样后点数: {len(downsampled_points)}")
                
                # 更新累积的点云为降采样后的点云
                self.accumulated_points = downsampled_points
                
                # 如果有颜色，也更新为降采样后的颜色
                if filtered_colors is not None and len(filtered_colors) > 0:
                    self.accumulated_colors = filtered_colors[color_indices]
                else:
                    self.accumulated_colors = []
        
        except Exception as e:
            rospy.logerr(f"处理点云时出错: {e}")
    
    def filter_by_bounds(self, points, colors=None):
        """
        按xyz轴范围滤除点云
        
        Args:
            points (np.ndarray): 输入点云 (N, 3)
            colors (np.ndarray): 对应的颜色数据 (N,)，可选
            
        Returns:
            tuple: (滤波后的点, 滤波后的颜色) 或 (滤波后的点, None)
        """
        if len(points) == 0:
            return points, colors
        
        # 创建三轴滤波掩码
        mask_x = (points[:, 0] >= self.x_min) & (points[:, 0] <= self.x_max)
        mask_y = (points[:, 1] >= self.y_min) & (points[:, 1] <= self.y_max)
        mask_z = (points[:, 2] >= self.z_min) & (points[:, 2] <= self.z_max)
        mask = mask_x & mask_y & mask_z
        
        # 应用掩码
        filtered_points = points[mask]
        filtered_colors = None
        
        if colors is not None and len(colors) > 0:
            filtered_colors = colors[mask]
        
        if len(filtered_points) < len(points):
            rospy.loginfo(f"范围滤波: {len(points)} -> {len(filtered_points)} 点 "
                         f"(x: [{self.x_min}, {self.x_max}], y: [{self.y_min}, {self.y_max}], z: [{self.z_min}, {self.z_max}])")
        
        return filtered_points, filtered_colors
    
    def filter_by_z(self, points, colors=None):
        """
        按z轴阈值范围滤除点云（向后兼容）
        
        Args:
            points (np.ndarray): 输入点云 (N, 3)
            colors (np.ndarray): 对应的颜色数据 (N,)，可选
            
        Returns:
            tuple: (滤波后的点, 滤波后的颜色) 或 (滤波后的点, None)
        """
        return self.filter_by_bounds(points, colors)
    
    def voxel_downsample(self, points, voxel_size):
        """
        体素化降采样
        
        Args:
            points (np.ndarray): 输入点云 (N, 3)
            voxel_size (float): 体素大小
            
        Returns:
            np.ndarray: 降采样后的点云
        """
        if len(points) == 0:
            return points
        
        # 计算点所在的体素索引
        voxel_indices = np.floor(points / voxel_size).astype(np.int32)
        
        # 创建一个字典来存储每个体素内的点
        voxel_dict = {}
        for i, voxel_idx in enumerate(voxel_indices):
            # 将数组转换为元组作为字典的键
            key = tuple(voxel_idx)
            if key not in voxel_dict:
                voxel_dict[key] = []
            voxel_dict[key].append(i)
        
        # 对每个体素内的点求平均值
        downsampled_points = []
        downsampled_indices = []
        
        for indices in voxel_dict.values():
            # 计算体素内点的平均值
            downsampled_points.append(points[indices].mean(axis=0))
            downsampled_indices.append(indices[0])  # 保存第一个点的索引用于获取颜色
        
        return np.array(downsampled_points), np.array(downsampled_indices)
    
    def publish_accumulated_pointcloud(self):
        """
        发布累积的点云（已经在每次累积时进行过降采样）
        """
        with self.lock:
            if len(self.accumulated_points) == 0:
                rospy.logwarn("没有累积的点云数据")
                return
            
            rospy.loginfo(f"发布点数: {len(self.accumulated_points)}")
            
            # 创建PointCloud2消息
            header = self.create_header()
            
            # 如果有颜色信息，创建带颜色的点云
            if len(self.accumulated_colors) > 0:
                colors = self.accumulated_colors
                fields = [
                    pc2.PointField('x', 0, pc2.PointField.FLOAT32, 1),
                    pc2.PointField('y', 4, pc2.PointField.FLOAT32, 1),
                    pc2.PointField('z', 8, pc2.PointField.FLOAT32, 1),
                    pc2.PointField('rgb', 12, pc2.PointField.UINT32, 1),
                ]
                
                points_with_color = np.zeros(
                    len(self.accumulated_points),
                    dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32), ('rgb', np.uint32)]
                )
                points_with_color['x'] = self.accumulated_points[:, 0]
                points_with_color['y'] = self.accumulated_points[:, 1]
                points_with_color['z'] = self.accumulated_points[:, 2]
                points_with_color['rgb'] = colors
                
                msg = pc2.create_cloud(header, fields, points_with_color)
            else:
                # 创建不带颜色的点云
                fields = [
                    pc2.PointField('x', 0, pc2.PointField.FLOAT32, 1),
                    pc2.PointField('y', 4, pc2.PointField.FLOAT32, 1),
                    pc2.PointField('z', 8, pc2.PointField.FLOAT32, 1),
                ]
                msg = pc2.create_cloud_xyz32(header, self.accumulated_points)
            
            self.publisher.publish(msg)
            rospy.loginfo(f"已发布累积点云到 {self.output_topic}")
    
    def create_header(self):
        """
        创建点云消息头
        """
        from std_msgs.msg import Header
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = self.frame_id
        return header
    
    def reset_accumulated_pointcloud(self):
        """
        重置累积的点云
        """
        with self.lock:
            self.accumulated_points = []
            self.accumulated_colors = []
            rospy.loginfo("已重置累积的点云")
    
    def get_stats(self):
        """
        获取统计信息
        """
        with self.lock:
            return {
                'accumulated_points': len(self.accumulated_points),
                'accumulated_colors': len(self.accumulated_colors),
            }


def main():
    """
    主函数
    """
    rospy.init_node('pointcloud_accumulator', anonymous=False)
    
    # 从参数服务器获取配置参数
    input_topic = rospy.get_param('~input_topic', '/camera/depth/points')
    voxel_size = rospy.get_param('~voxel_size', 0.1)
    
    # x轴范围参数
    x_min = rospy.get_param('~x_min', -float('inf'))
    x_max = rospy.get_param('~x_max', float('inf'))
    
    # y轴范围参数
    y_min = rospy.get_param('~y_min', -float('inf'))
    y_max = rospy.get_param('~y_max', float('inf'))
    
    # z轴范围参数
    z_min = rospy.get_param('~z_min', 0.0)
    z_max = rospy.get_param('~z_max', float('inf'))
    
    publish_rate = rospy.get_param('~publish_rate', 1.0)  # Hz
    
    # 创建累积器
    accumulator = PointCloudAccumulator(input_topic, voxel_size, x_min, x_max, y_min, y_max, z_min, z_max)
    
    # 设置发布频率
    rate = rospy.Rate(publish_rate)
    
    try:
        while not rospy.is_shutdown():
            # 定期发布累积的点云
            accumulator.publish_accumulated_pointcloud()
            
            # 打印统计信息
            stats = accumulator.get_stats()
            rospy.loginfo(f"统计 - 累积点数: {stats['accumulated_points']}")
            
            rate.sleep()
    
    except KeyboardInterrupt:
        rospy.loginfo("节点被中断")
    except Exception as e:
        rospy.logerr(f"发生错误: {e}")


if __name__ == '__main__':
    main()

// Created by gwq on 9/23/25.
//

#ifndef TRAJ_VISUALIZER_H
#define TRAJ_VISUALIZER_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <Eigen/Dense> // 1. 包含Eigen头文件
#include <string>
#include <vector>

class TrajectoryVisualizer {
public:
    // 构造函数现在也初始化了新成员
    TrajectoryVisualizer(ros::NodeHandle& nh) : nh_(nh) {
        // 从参数服务器获取参数
        nh_.param<std::string>("/traj_vis/map_frame_id", map_frame_id_, "world");
        nh_.param<double>("/traj_vis/min_speed", min_speed_, 0.0);
        nh_.param<double>("/traj_vis/max_speed", max_speed_, 0.65); // m/s
        nh_.param<double>("/traj_vis/distance_threshold", distance_threshold_, 0.1); // 最小距离阈值

        // 初始化发布者和订阅者
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/scene_graph/traj_with_vel", 10);

        // 初始化Marker消息
        initMarker();
    }

    /**
     * @brief 核心功能接口：向轨迹中添加一个点 (已修改)
     * @param pos 新的位置点 (Eigen::Vector3d)
     * @param vel 当前的速度（用于着色）
     */
    void addPoint(const Eigen::Vector3d& pos, double vel) {
        // 如果是第一个点，或者与上一个点的距离超过阈值
        if (calculateDistance(pos, last_point_) > distance_threshold_) {

            // 4.1. 将 Eigen::Vector3d 转换为 geometry_msgs::Point
            geometry_msgs::Point new_ros_point;
            new_ros_point.x = pos.x();
            new_ros_point.y = pos.y();
            new_ros_point.z = pos.z();

            // 4.2. 添加位置点
            trajectory_marker_.points.push_back(new_ros_point);

            // 4.3. 根据速度计算颜色并添加
            std_msgs::ColorRGBA color = speedToColor(vel);
            trajectory_marker_.colors.push_back(color);

            // 4.4. 更新并发布Marker
            trajectory_marker_.header.stamp = ros::Time::now();
            marker_pub_.publish(trajectory_marker_);

            // 4.5. 更新状态
            last_point_     = pos; // 更新为Eigen类型
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher marker_pub_;
    visualization_msgs::Marker trajectory_marker_;

    // 配置参数
    std::string odom_topic_;
    std::string map_frame_id_;
    double min_speed_;
    double max_speed_;
    double distance_threshold_;

    // 状态变量
    Eigen::Vector3d last_point_; // 2. 修改成员变量类型

    // 3. 修改距离计算函数以使用Eigen (更高效)
    double calculateDistance(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2) {
        return (p1 - p2).norm();
    }

    void initMarker() {
        // ... (这部分函数与之前完全相同，无需修改)
        trajectory_marker_.header.frame_id = map_frame_id_;
        trajectory_marker_.header.stamp = ros::Time::now();
        trajectory_marker_.ns = "trajectory";
        trajectory_marker_.id = 0;
        trajectory_marker_.type = visualization_msgs::Marker::LINE_STRIP;
        trajectory_marker_.action = visualization_msgs::Marker::ADD;
        trajectory_marker_.pose.orientation.w = 1.0;
        trajectory_marker_.scale.x            = 0.05;
    }

    std_msgs::ColorRGBA speedToColor(double speed) {
        double normalized_speed = (speed - min_speed_) / (max_speed_ - min_speed_);
        normalized_speed = std::max(0.0, std::min(1.0, normalized_speed));
        double hue = (1.0 - normalized_speed) * 240.0 / 360.0;
        return hsvToRgb(hue, 1.0, 1.0);
    }

    std_msgs::ColorRGBA hsvToRgb(double h, double s, double v) {
        std_msgs::ColorRGBA rgb_color;
        rgb_color.a = 0.8;
        int i = floor(h * 6);
        double f = h * 6 - i;
        double p = v * (1 - s);
        double q = v * (1 - f * s);
        double t = v * (1 - (1 - f) * s);

        switch (i % 6) {
            case 0: rgb_color.r = v, rgb_color.g = t, rgb_color.b = p; break;
            case 1: rgb_color.r = q, rgb_color.g = v, rgb_color.b = p; break;
            case 2: rgb_color.r = p, rgb_color.g = v, rgb_color.b = t; break;
            case 3: rgb_color.r = p, rgb_color.g = q, rgb_color.b = v; break;
            case 4: rgb_color.r = t, rgb_color.g = p, rgb_color.b = v; break;
            case 5: rgb_color.r = v, rgb_color.g = p, rgb_color.b = q; break;
        }
        return rgb_color;
    }
};

#endif //TRAJ_VISUALIZER_H
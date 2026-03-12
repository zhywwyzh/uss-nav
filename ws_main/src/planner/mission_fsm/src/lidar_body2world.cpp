#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <Eigen/Eigen>

#include <tf/tf.h>

class PointCloudTransformer {
public:
    PointCloudTransformer() {
        // 初始化ROS节点
        nh_.reset(new ros::NodeHandle("~"));
        
        // 获取ROS参数
        nh_->param<std::string>("odom_topic", odom_topic_, "/unity_odom_sync");
        nh_->param<std::string>("cloud_topic", cloud_topic_, "/livox/lidar_sync");
        nh_->param<std::string>("output_frame", output_frame_, "world");
        nh_->param<double>("filter_distance", filter_distance_, 0.2);
        nh_->param<double>("approximate_sync_threshold", sync_threshold_, 0.01);
        nh_->param<int>("queue_size", queue_size_, 10);
        nh_->param<bool>("debug", debug_, false);

        // 创建订阅者和发布者
        odom_sub_  = std::make_unique <message_filters::Subscriber<nav_msgs::Odometry>>
                    (*nh_, odom_topic_, queue_size_, ros::TransportHints().tcpNoDelay());
        cloud_sub_ = std::make_unique <message_filters::Subscriber<sensor_msgs::PointCloud2>>
                    (*nh_, cloud_topic_, 10, ros::TransportHints().tcpNoDelay());
        cloud_pub_ = nh_->advertise<sensor_msgs::PointCloud2>("/livox/lidar_world", 2);
        
        // 创建近似时间同步策略
        typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> SyncPolicy;
        sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(queue_size_), *odom_sub_, *cloud_sub_);
        sync_->registerCallback(boost::bind(&PointCloudTransformer::combinedCallback, this, _1, _2));
        
        ROS_INFO("PointCloudTransformer node initialized with time synchronization.");
        ROS_INFO("Filter distance: %.2f meters", filter_distance_);
        ROS_INFO("Sync threshold: %.2f seconds", sync_threshold_);
    }
    ~PointCloudTransformer() = default;

private:
    std::unique_ptr<ros::NodeHandle> nh_;
    std::unique_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub_;
    std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> cloud_sub_;
    ros::Publisher cloud_pub_;
    
    std::string odom_topic_;
    std::string cloud_topic_;
    std::string output_frame_;
    double filter_distance_;
    bool debug_;
    int queue_size_;
    double sync_threshold_;
    
    std::unique_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime
                     <nav_msgs::Odometry, sensor_msgs::PointCloud2>>> sync_;

    void combinedCallback(const nav_msgs::Odometry::ConstPtr& odom_msg, 
                          const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {


        try {
            ros::Time t1 = ros::Time::now();
            
            // 检查时间戳差异
            ros::Duration time_diff = odom_msg->header.stamp - cloud_msg->header.stamp;
            if (std::fabs(time_diff.toSec()) > sync_threshold_) {
                ROS_WARN("Time difference between odom and pointcloud exceeds threshold: %.3f seconds", 
                         std::fabs(time_diff.toSec()));
            }
            
            // 从odometry消息中提取位置和姿态
            const auto& pos = odom_msg->pose.pose.position;
            const auto& ori = odom_msg->pose.pose.orientation;
            Eigen::Vector3f odom_pos(pos.x, pos.y, pos.z);
            
            // 使用Eigen构建变换矩阵
            Eigen::Translation3f translation(pos.x, pos.y, pos.z);
            Eigen::Quaternionf quaternion(ori.w, ori.x, ori.y, ori.z);
            Eigen::Matrix3f rotation = quaternion.toRotationMatrix();
            Eigen::Affine3f transform_matrix = translation * rotation;
            
            // 将PointCloud2消息转换为PCL点云
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(*cloud_msg, *cloud);
            pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            
            size_t original_size = cloud->size();
            size_t filtered_count = 0;

            double odom_yaw = tf::getYaw(odom_msg->pose.pose.orientation);
            for (const auto& point : *cloud) {
                // 转换为Eigen向量
                Eigen::Vector3f eigen_point(point.x, point.y, point.z);
                // Eigen::Vector3f global_point = transform_matrix * eigen_point;
                Eigen::Vector3f global_point = eigen_point;
                if ((global_point - odom_pos).norm() <= filter_distance_) {
                    filtered_count++;
                }else {
                    Eigen::Vector3f transformed_point = transform_matrix * eigen_point;
                    // transformed_cloud->push_back(pcl::PointXYZ(
                    //     transformed_point.x(),
                    //     transformed_point.y(),
                    //     transformed_point.z()
                    // ));
                    // 判断点云是否在当前odom正前方一定的yaw角范围内
                    double dx = point.x - odom_pos.x();
                    double dy = point.y - odom_pos.y();
                    double point_yaw = std::atan2(dy, dx);
                    double yaw_diff = std::fabs(point_yaw - odom_yaw);
                    yaw_diff        = std::min(yaw_diff, 2.0 * M_PI - yaw_diff);
                    if (fabs(yaw_diff) <= 0.8)
                        transformed_cloud->push_back(point);
                }
            }
            
            // 打印过滤统计信息
            if (original_size > 0 && debug_) {
                double filter_percentage = (static_cast<double>(filtered_count) / original_size) * 100.0;
                ROS_DEBUG("Filtered %.2f%% of points (removed %zu points within %.2f meters)", 
                         filter_percentage, filtered_count, filter_distance_);
            }
            
            // 将变换后的PCL点云转回ROS消息
            sensor_msgs::PointCloud2 transformed_cloud_msg;
            pcl::toROSMsg(*transformed_cloud, transformed_cloud_msg);
            
            // 设置正确的坐标系
            transformed_cloud_msg.header.stamp    = cloud_msg->header.stamp;
            transformed_cloud_msg.header.frame_id = output_frame_;
            
            // 发布变换后的点云
            cloud_pub_.publish(transformed_cloud_msg);
            ROS_INFO_STREAM_THROTTLE(2.0, "Time Spent: " << (ros::Time::now() - t1).toSec() * 1e3 << " ms.");
            
        } catch (const std::exception& e) {
            ROS_ERROR("Exception processing point cloud: %s", e.what());
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_transformer");
    
    PointCloudTransformer pt_transformer;
    
    ros::spin();
    
    return 0;
}
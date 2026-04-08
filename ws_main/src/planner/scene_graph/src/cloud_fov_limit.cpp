#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <deque>
#include <mutex>
#include <algorithm> // 必须包含，用于 std::lower_bound

class FovWorldFilterNode {
public:
    FovWorldFilterNode() {
        nh_.reset(new ros::NodeHandle("~"));

        // 参数配置
        nh_->param<std::string>("odom_topic", odom_topic_, "/ekf_quat/ekf_odom");
        nh_->param<std::string>("cloud_topic", cloud_topic_, "/cloud_registered");
        nh_->param<std::string>("output_frame", output_frame_, "world");
        
        // 修复：使用浮点数除法，避免整数除法导致结果为0
        nh_->param<double>("fov_limit_angle", fov_limit_angle_, 45.0 / 180.0 * M_PI);

        // 时间同步容差
        nh_->param<double>("time_sync_threshold", time_sync_threshold_, 0.05); // 稍微放宽一点阈值

        // 距离限制参数
        nh_->param<double>("max_distance", max_distance_, 50.0); // 最大距离限制，单位：米

        odom_sub_ = nh_->subscribe(odom_topic_, 100, &FovWorldFilterNode::odomCallback, this);
        cloud_sub_ = nh_->subscribe(cloud_topic_, 2, &FovWorldFilterNode::cloudCallback, this);
        cloud_pub_ = nh_->advertise<sensor_msgs::PointCloud2>("/cloud_fov_limited", 2);

        ROS_INFO("FovWorldFilterNode initialized. Method: Optimized Binary Search.");
    }

private:
    ros::NodeHandlePtr nh_;
    ros::Subscriber odom_sub_;
    ros::Subscriber cloud_sub_;
    ros::Publisher cloud_pub_;

    std::string odom_topic_;
    std::string cloud_topic_;
    std::string output_frame_;
    double fov_limit_angle_;
    double time_sync_threshold_;
    double max_distance_;

    std::deque<nav_msgs::Odometry> odom_queue_;
    std::mutex odom_mutex_;

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        odom_queue_.push_back(*msg);
        
        // 保持队列长度适中，避免内存溢出，但也要足够长以覆盖延迟
        if (odom_queue_.size() > 5000) odom_queue_.pop_front();
    }

    /**
     * @brief 使用二分查找寻找时间戳最近的里程计数据
     * 性能：O(log N)
     */
    bool findClosestOdom(const ros::Time& cloud_time, nav_msgs::Odometry& result_odom) {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        
        if (odom_queue_.empty()) return false;

        // 1. 快速边界检查：如果查询时间太早或太晚，直接返回失败，避免无意义搜索
        if (cloud_time < odom_queue_.front().header.stamp - ros::Duration(time_sync_threshold_) ||
            cloud_time > odom_queue_.back().header.stamp + ros::Duration(time_sync_threshold_)) {
            return false;
        }

        // 2. 二分查找 (std::lower_bound)
        // 寻找第一个 header.stamp >= cloud_time 的迭代器
        auto lower = std::lower_bound(odom_queue_.begin(), odom_queue_.end(), cloud_time,
            [](const nav_msgs::Odometry& msg, const ros::Time& t) {
                return msg.header.stamp < t;
            });

        // 3. 寻找最近邻
        // lower 指向的是 >= time 的元素。我们需要比较 lower 和 prev(lower) 哪个更近
        auto best_it = odom_queue_.end();
        double min_diff = std::numeric_limits<double>::max();

        // 检查 lower 指向的元素 (右侧邻居)
        if (lower != odom_queue_.end()) {
            double diff = std::fabs((lower->header.stamp - cloud_time).toSec());
            if (diff < min_diff) {
                min_diff = diff;
                best_it = lower;
            }
        }

        // 检查 lower 前一个元素 (左侧邻居)
        if (lower != odom_queue_.begin()) {
            auto prev = std::prev(lower);
            double diff = std::fabs((prev->header.stamp - cloud_time).toSec());
            if (diff < min_diff) {
                min_diff = diff;
                best_it = prev;
            }
        }

        // 4. 验证阈值
        if (best_it != odom_queue_.end() && min_diff <= time_sync_threshold_) {
            result_odom = *best_it;
            return true;
        }

        return false;
    }

    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
        // 1. 寻找匹配的 Odometry (高性能版)
        nav_msgs::Odometry best_odom;
        if (!findClosestOdom(cloud_msg->header.stamp, best_odom)) {
            // 使用 THROTTLE 避免刷屏，提示调整 queue size 或 threshold
            ROS_WARN_THROTTLE(1.0, "Odom sync failed. Cloud Time: %.3f. Queue Size: %lu", 
                              cloud_msg->header.stamp.toSec(), odom_queue_.size());
            return;
        }

        // 2. 构建变换矩阵 T_world_to_body
        // 使用 float (Affine3f) 提高后续大量点云计算的速度
        const auto& pos = best_odom.pose.pose.position;
        const auto& ori = best_odom.pose.pose.orientation;
        
        Eigen::Translation3f translation(pos.x, pos.y, pos.z);
        Eigen::Quaternionf quaternion(ori.w, ori.x, ori.y, ori.z);
        
        // Odom 是 Body->World，我们需要逆变换 World->Body
        Eigen::Affine3f t_world_to_body = (translation * quaternion.toRotationMatrix()).inverse();

        // 3. 转换输入点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*cloud_msg, *input_cloud);

        pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        // 预分配内存，避免 push_back 时的多次 reallocation
        output_cloud->points.reserve(input_cloud->size());

        // 缓存 limit，避免循环内转换
        float limit_angle_f = static_cast<float>(fov_limit_angle_);
        float max_distance_f = static_cast<float>(max_distance_);

        // 4. 遍历并过滤 (计算密集区域)
        for (const auto& world_point : input_cloud->points) {
            // P_body = T_world_to_body * P_world
            // 手动展开矩阵乘法比 Eigen 的通用乘法略快，但这里直接用 Eigen 方便阅读，编译器开启 -O3 后差异不大
            Eigen::Vector3f pt_world_vec(world_point.x, world_point.y, world_point.z);
            Eigen::Vector3f pt_body_vec = t_world_to_body * pt_world_vec;

            // --- 性能优化技巧 ---
            // 假设 FOV 是前方扇形且小于 180度 (如 +/- 80度)
            // 如果 x < 0，点在身后，直接跳过，无需计算耗时的 atan2
            if (pt_body_vec.x() < 0.001f) continue; 

            // 使用 atan2f (float版本) 替代 atan2 (double版本)
            float angle = std::atan2(pt_body_vec.y(), pt_body_vec.x());

            // 同时检查 FOV 角度 和 距离限制
            if (std::fabs(angle) <= limit_angle_f) {
                // 计算点到无人机的距离（在体坐标系中）
                float distance = pt_body_vec.norm();
                
                // 如果距离在限制范围内，保留原始的世界系点
                if (distance <= max_distance_f) {
                    output_cloud->points.push_back(world_point);
                }
            }
        } 

        // 5. 发布
        if (!output_cloud->empty()) {
            sensor_msgs::PointCloud2 output_msg;
            pcl::toROSMsg(*output_cloud, output_msg);
            
            output_msg.header = cloud_msg->header; 
            output_msg.header.frame_id = output_frame_;

            cloud_pub_.publish(output_msg);
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "fov_world_filter_node");
    FovWorldFilterNode node;
    ros::spin();
    return 0;
}
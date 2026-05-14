#include <algorithm>
#include <cmath>
#include <vector>

#include <Eigen/Core>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <quadrotor_msgs/OccMap3d.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

namespace {

class WsMainMapAdapter {
 public:
  explicit WsMainMapAdapter(ros::NodeHandle& nh) : nh_(nh) {
    double local_x = 20.0;
    double local_y = 20.0;
    double local_z = 5.0;
    nh_.param("resolution", resolution_, 0.15);
    nh_.param("local_x", local_x, 20.0);
    nh_.param("local_y", local_y, 20.0);
    nh_.param("local_z", local_z, 5.0);
    nh_.param("inflate_size", inflate_size_, 0.15);
    nh_.param("inflate_num", inflate_num_, 3);
    nh_.param("unknown_as_free", unknown_as_free_, true);
    nh_.param("publish_esdf_placeholder", publish_esdf_placeholder_, false);
    nh_.param("use_mask", use_mask_, true);
    nh_.param("target_mask_x", target_mask_x_, 0.5);
    nh_.param("target_mask_y", target_mask_y_, 0.5);
    nh_.param("target_mask_z", target_mask_z_, 1.0);
    nh_.param("target_timeout", target_timeout_, 0.5);

    local_size_ = Eigen::Vector3d(local_x, local_y, local_z);
    size_x_ = mapCellsToPowerOfTwo(local_size_.x());
    size_y_ = mapCellsToPowerOfTwo(local_size_.y());
    size_z_ = mapCellsToPowerOfTwo(local_size_.z());

    gridmap_pub_ = nh_.advertise<quadrotor_msgs::OccMap3d>("gridmap_inflate", 1);
    odom_sub_ = nh_.subscribe("odom", 20, &WsMainMapAdapter::odomCallback, this,
                              ros::TransportHints().tcpNoDelay());
    cloud_sub_ = nh_.subscribe("occupancy", 1, &WsMainMapAdapter::cloudCallback, this,
                               ros::TransportHints().tcpNoDelay());
    if (use_mask_) {
      target_sub_ = nh_.subscribe("target", 1, &WsMainMapAdapter::targetCallback, this,
                                  ros::TransportHints().tcpNoDelay());
    }
  }

 private:
  int mapCellsToPowerOfTwo(double length) const {
    const double raw_cells = length / resolution_;
    const int exp = static_cast<int>(std::floor(std::log2(raw_cells)));
    return 1 << std::max(exp, 0);
  }

  Eigen::Vector3i pos2idx(const Eigen::Vector3d& pos) const {
    return (pos / resolution_).array().floor().cast<int>();
  }

  bool isInLocalGrid(const Eigen::Vector3i& id, int offset_x, int offset_y, int offset_z,
                     int& local_x, int& local_y, int& local_z) const {
    local_x = id.x() - offset_x;
    local_y = id.y() - offset_y;
    local_z = id.z() - offset_z;
    return local_x >= 0 && local_x < size_x_ &&
           local_y >= 0 && local_y < size_y_ &&
           local_z >= 0 && local_z < size_z_;
  }

  int linearIndex(int x, int y, int z) const {
    return (z * size_y_ + y) * size_x_ + x;
  }

  void odomCallback(const nav_msgs::OdometryConstPtr& msg) {
    odom_pos_.x() = msg->pose.pose.position.x;
    odom_pos_.y() = msg->pose.pose.position.y;
    odom_pos_.z() = msg->pose.pose.position.z;
    has_odom_ = true;
  }

  void targetCallback(const nav_msgs::OdometryConstPtr& msg) {
    target_pos_.x() = msg->pose.pose.position.x;
    target_pos_.y() = msg->pose.pose.position.y;
    target_pos_.z() = msg->pose.pose.position.z;
    last_target_stamp_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    has_target_ = true;
  }

  void clearTargetMask(std::vector<int8_t>& data,
                       int offset_x, int offset_y, int offset_z) const {
    if (!use_mask_ || !has_target_) {
      return;
    }
    if (target_timeout_ > 0.0 &&
        (ros::Time::now() - last_target_stamp_).toSec() > target_timeout_) {
      return;
    }

    const Eigen::Vector3d mask(target_mask_x_, target_mask_y_, target_mask_z_);
    const Eigen::Vector3i min_id = pos2idx(target_pos_ - mask);
    const Eigen::Vector3i max_id = pos2idx(target_pos_ + mask);

    int local_x, local_y, local_z;
    for (int x = min_id.x(); x <= max_id.x(); ++x) {
      for (int y = min_id.y(); y <= max_id.y(); ++y) {
        for (int z = min_id.z(); z <= max_id.z(); ++z) {
          const Eigen::Vector3i id(x, y, z);
          if (!isInLocalGrid(id, offset_x, offset_y, offset_z, local_x, local_y, local_z)) {
            continue;
          }
          data[linearIndex(local_x, local_y, local_z)] = -1;
        }
      }
    }
  }

  void markOccupied(std::vector<int8_t>& data,
                    const Eigen::Vector3d& pos,
                    int offset_x,
                    int offset_y,
                    int offset_z) const {
    int local_x, local_y, local_z;
    const Eigen::Vector3i id = pos2idx(pos);
    if (!isInLocalGrid(id, offset_x, offset_y, offset_z, local_x, local_y, local_z)) {
      return;
    }
    data[linearIndex(local_x, local_y, local_z)] = 1;
  }

  void markInflatedPoint(std::vector<int8_t>& data,
                         const Eigen::Vector3d& pos,
                         int offset_x,
                         int offset_y,
                         int offset_z) const {
    markOccupied(data, pos, offset_x, offset_y, offset_z);
    if (inflate_size_ <= 0.0 || inflate_num_ <= 0) {
      return;
    }

    // 按真实距离生成膨胀采样点，再重新映射到局部栅格，避免直接按栅格索引固定偏移。
    for (int ix = -inflate_num_; ix <= inflate_num_; ++ix) {
      for (int iy = -inflate_num_; iy <= inflate_num_; ++iy) {
        for (int iz = -inflate_num_; iz <= inflate_num_; ++iz) {
          const Eigen::Vector3d offset(ix * inflate_size_,
                                       iy * inflate_size_,
                                       iz * inflate_size_);
          markOccupied(data, pos + offset, offset_x, offset_y, offset_z);
        }
      }
    }
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    if (!has_odom_) {
      ROS_WARN_THROTTLE(1.0, "[ws_main_map_adapter] waiting for odom");
      return;
    }

    const Eigen::Vector3i center_idx = pos2idx(odom_pos_);
    const int offset_x = center_idx.x() - size_x_ / 2;
    const int offset_y = center_idx.y() - size_y_ / 2;
    const int offset_z = center_idx.z() - size_z_ / 2;

    quadrotor_msgs::OccMap3d grid_msg;
    grid_msg.header.stamp = msg->header.stamp;
    grid_msg.header.frame_id = msg->header.frame_id.empty() ? "world" : msg->header.frame_id;
    grid_msg.resolution = resolution_;
    grid_msg.inflate_size = inflate_num_;
    grid_msg.size_x = size_x_;
    grid_msg.size_y = size_y_;
    grid_msg.size_z = size_z_;
    grid_msg.offset_x = offset_x;
    grid_msg.offset_y = offset_y;
    grid_msg.offset_z = offset_z;

    const int8_t default_value = unknown_as_free_ ? -1 : 0;
    std::vector<int8_t> raw_data(size_x_ * size_y_ * size_z_, default_value);

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);

    for (const auto& point : cloud.points) {
      Eigen::Vector3d pos(point.x, point.y, point.z);
      if (pos.array().isNaN().any()) {
        continue;
      }
      markInflatedPoint(raw_data, pos, offset_x, offset_y, offset_z);
    }

    clearTargetMask(raw_data, offset_x, offset_y, offset_z);
    grid_msg.data = raw_data;
    if (publish_esdf_placeholder_) {
      grid_msg.esdf.assign(size_x_ * size_y_ * size_z_, 0.0);
    }

    gridmap_pub_.publish(grid_msg);
  }

  ros::NodeHandle nh_;
  ros::Subscriber cloud_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber target_sub_;
  ros::Publisher gridmap_pub_;

  double resolution_;
  Eigen::Vector3d local_size_;
  double inflate_size_;
  int inflate_num_;
  bool unknown_as_free_;
  bool publish_esdf_placeholder_;
  bool use_mask_;
  double target_mask_x_;
  double target_mask_y_;
  double target_mask_z_;
  double target_timeout_;

  int size_x_;
  int size_y_;
  int size_z_;
  bool has_odom_ = false;
  bool has_target_ = false;
  Eigen::Vector3d odom_pos_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d target_pos_ = Eigen::Vector3d::Zero();
  ros::Time last_target_stamp_;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "ws_main_map_adapter");
  ros::NodeHandle nh("~");
  WsMainMapAdapter adapter(nh);
  ros::spin();
  return 0;
}

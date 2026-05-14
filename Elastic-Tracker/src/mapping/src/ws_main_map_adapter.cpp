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
    nh_.param("inflate_size", inflate_size_, 0);
    nh_.param("unknown_as_free", unknown_as_free_, true);
    nh_.param("publish_esdf_placeholder", publish_esdf_placeholder_, false);

    local_size_ = Eigen::Vector3d(local_x, local_y, local_z);
    size_x_ = mapCellsToPowerOfTwo(local_size_.x());
    size_y_ = mapCellsToPowerOfTwo(local_size_.y());
    size_z_ = mapCellsToPowerOfTwo(local_size_.z());

    gridmap_pub_ = nh_.advertise<quadrotor_msgs::OccMap3d>("gridmap_inflate", 1);
    odom_sub_ = nh_.subscribe("odom", 20, &WsMainMapAdapter::odomCallback, this,
                              ros::TransportHints().tcpNoDelay());
    cloud_sub_ = nh_.subscribe("occupancy_inflate", 1, &WsMainMapAdapter::cloudCallback, this,
                               ros::TransportHints().tcpNoDelay());
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
    grid_msg.inflate_size = inflate_size_;
    grid_msg.size_x = size_x_;
    grid_msg.size_y = size_y_;
    grid_msg.size_z = size_z_;
    grid_msg.offset_x = offset_x;
    grid_msg.offset_y = offset_y;
    grid_msg.offset_z = offset_z;

    const int8_t default_value = unknown_as_free_ ? -1 : 0;
    grid_msg.data.assign(size_x_ * size_y_ * size_z_, default_value);
    if (publish_esdf_placeholder_) {
      grid_msg.esdf.assign(size_x_ * size_y_ * size_z_, 0.0);
    }

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);

    int local_x, local_y, local_z;
    for (const auto& point : cloud.points) {
      Eigen::Vector3d pos(point.x, point.y, point.z);
      if (pos.array().isNaN().any()) {
        continue;
      }
      const Eigen::Vector3i id = pos2idx(pos);
      if (!isInLocalGrid(id, offset_x, offset_y, offset_z, local_x, local_y, local_z)) {
        continue;
      }
      grid_msg.data[linearIndex(local_x, local_y, local_z)] = 1;
    }

    gridmap_pub_.publish(grid_msg);
  }

  ros::NodeHandle nh_;
  ros::Subscriber cloud_sub_;
  ros::Subscriber odom_sub_;
  ros::Publisher gridmap_pub_;

  double resolution_;
  Eigen::Vector3d local_size_;
  int inflate_size_;
  bool unknown_as_free_;
  bool publish_esdf_placeholder_;

  int size_x_;
  int size_y_;
  int size_z_;
  bool has_odom_ = false;
  Eigen::Vector3d odom_pos_ = Eigen::Vector3d::Zero();
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "ws_main_map_adapter");
  ros::NodeHandle nh("~");
  WsMainMapAdapter adapter(nh);
  ros::spin();
  return 0;
}

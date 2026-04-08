#ifndef _VISUALIZATION_HPP_
#define _VISUALIZATION_HPP_

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/MarkerArray.h>
#include <unordered_map>
#include<iostream>
#include <sstream>
#include <string>

using namespace std;

namespace visualization {
  using PublisherMap = std::unordered_map<std::string, ros::Publisher>;
  enum Color { red, green, blue };
  
class Visualization {
  
 private:
  ros::NodeHandle nh_;
  PublisherMap publisher_map_;
  
  void setMarkerColor(
      visualization_msgs::Marker &marker, 
      Color color = blue, 
      double a = 1) {
    marker.color.a = a;
    switch (color) {
      case red:
        marker.color.r = 1;
        marker.color.g = 0;
        marker.color.b = 0;
        break;
      case green:
        marker.color.r = 0;
        marker.color.g = 1;
        marker.color.b = 0;
        break;
      case blue:
        marker.color.r = 0;
        marker.color.g = 0;
        marker.color.b = 1;
        break;
    }
  }

  void setMarkerScale(visualization_msgs::Marker &marker, 
                      const double& x, 
                      const double& y, 
                      const double& z) {
    marker.scale.x = x;
    marker.scale.y = y;
    marker.scale.z = z;
  }

  void setMarkerPose(visualization_msgs::Marker &marker, 
                     const double& x, 
                     const double& y, 
                     const double& z) {
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.w = 1;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
  }

 public:

  Visualization(ros::NodeHandle &nh) : nh_(nh) {}

  // CENTER: std::vector<Vector3d>
  template <class CENTER, class TOPIC>
  void visualize_a_ball(const CENTER& c, 
                        const double& r, 
                        const TOPIC& topic, 
                        const Color color = blue, 
                        const double a = 1) {
    auto got = publisher_map_.find(topic);
    if (got == publisher_map_.end()) {
      ros::Publisher pub = nh_.advertise<visualization_msgs::Marker>(topic, 10);
      publisher_map_[topic] = pub;
    }
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    setMarkerColor(marker, color, a);
    setMarkerScale(marker, 2*r, 2*r, 2*r);
    setMarkerPose(marker, c[0], c[1], c[2]);
    marker.header.stamp = ros::Time::now();
    publisher_map_[topic].publish(marker);
  }


  // point_cloud: PointCloud<pcl::PointXYZ>
  void visualize_real_pointcloud(const pcl::PointCloud<pcl::PointXYZ> pc, std::string topic, std::string frame_id = "world") {
    auto got = publisher_map_.find(topic);
    if (got == publisher_map_.end()) {
      ros::Publisher pub = nh_.advertise<sensor_msgs::PointCloud2>(topic, 10);
      publisher_map_[topic] = pub;
    }
    pcl::PointCloud<pcl::PointXYZ> point_cloud;
    // point_cloud.reserve( pc.size() );
    for (const auto& pt : pc.points) {
      // if(abs(pt.x)<100 && abs(pt.y)<100 &&abs(pt.z)<100)
        point_cloud.points.emplace_back(pt.x, pt.y, pt.z);
    }
    std::cout<<"pc size: "<<point_cloud.size()<<std::endl;
    sensor_msgs::PointCloud2 point_cloud_msg;
    pcl::toROSMsg(point_cloud, point_cloud_msg);
    point_cloud_msg.header.frame_id = frame_id;
    point_cloud_msg.header.stamp = ros::Time::now();
    publisher_map_[topic].publish(point_cloud_msg);
  }

  // pc: std::vector<Vector3d>
  template <class PC, class TOPIC>
  void visualize_pointcloud(const PC& pc, const TOPIC& topic, std::string frame_id = "world") {
    auto got = publisher_map_.find(topic);
    if (got == publisher_map_.end()) {
      ros::Publisher pub = nh_.advertise<sensor_msgs::PointCloud2>(topic, 10);
      publisher_map_[topic] = pub;
    }
    pcl::PointCloud<pcl::PointXYZ> point_cloud;
    sensor_msgs::PointCloud2 point_cloud_msg;
    point_cloud.reserve( pc.size() );
    for (const auto& pt : pc) {
      point_cloud.points.emplace_back(pt[0], pt[1], pt[2]);
    }
    pcl::toROSMsg(point_cloud, point_cloud_msg);
    point_cloud_msg.header.frame_id = frame_id;
    point_cloud_msg.header.stamp = ros::Time::now();
    publisher_map_[topic].publish(point_cloud_msg);
  }

  // pc: std::vector<Vector4d>
  template <class PCI, class TOPIC>
  void visualize_pointcloud_intensity(const PCI& pc, const TOPIC& topic) {
    auto got = publisher_map_.find(topic);
    if (got == publisher_map_.end()) {
      ros::Publisher pub = nh_.advertise<sensor_msgs::PointCloud2>(topic, 10);
      publisher_map_[topic] = pub;
    }
    pcl::PointCloud<pcl::PointXYZI> point_cloud;
    sensor_msgs::PointCloud2 point_cloud_msg;
    point_cloud.reserve( pc.size() );
    for (const auto& pt : pc) {
      pcl::PointXYZI pti;
      pti.x = pt[0];
      pti.y = pt[1];
      pti.z = pt[2];
      pti.intensity = pt[3];
      point_cloud.points.push_back(pti);
    }
    pcl::toROSMsg(point_cloud, point_cloud_msg);
    point_cloud_msg.header.frame_id = "world";
    point_cloud_msg.header.stamp = ros::Time::now();
    publisher_map_[topic].publish(point_cloud_msg);
  }

  // path: std::vector<Vector3d>
  template <class PATH, class TOPIC>
  void visualize_path(const PATH& path, const TOPIC& topic) {
    auto got = publisher_map_.find(topic);
    if (got == publisher_map_.end()) {
      ros::Publisher pub = nh_.advertise<nav_msgs::Path>(topic, 10);
      publisher_map_[topic] = pub;
    }
    nav_msgs::Path path_msg;
    geometry_msgs::PoseStamped tmpPose;
    tmpPose.header.frame_id = "world";
    for (const auto& pt : path) {
      tmpPose.pose.position.x = pt[0];
      tmpPose.pose.position.y = pt[1];
      tmpPose.pose.position.z = pt[2];
      path_msg.poses.push_back(tmpPose);
    }
    path_msg.header.frame_id = "world";
    path_msg.header.stamp = ros::Time::now();
    publisher_map_[topic].publish(path_msg);
  }

  template <class BALLS, class TOPIC>
  void visualize_balls(const BALLS& balls, 
                       const TOPIC& topic, 
                       const Color color = blue, 
                       const double a = 1) {
    auto got = publisher_map_.find(topic);
    if (got == publisher_map_.end()) {
      ros::Publisher pub = nh_.advertise<visualization_msgs::MarkerArray>(topic, 10);
      publisher_map_[topic] = pub;
    }
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = 0;
    setMarkerColor(marker, color, a);
    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.reserve(balls.size()+1);
    marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(marker);
    marker.action = visualization_msgs::Marker::ADD;
    for (const auto& ball : balls) {
      setMarkerPose(marker, ball[0], ball[1], ball[2]);
      auto d = 2 * ball.r;
      setMarkerScale(marker, d, d, d);
      marker_array.markers.push_back(marker);
      marker.id ++;
    }
    publisher_map_[topic].publish(marker_array);
  }


  template <class TEXT, class TOPIC>
  void visualize_texts(const vector<string> prefix, const TEXT& texts, 
                       const double scale, 
                       const TOPIC& topic, 
                       const Color color = red, 
                       const double a = 1) {
    auto got = publisher_map_.find(topic);
    if (got == publisher_map_.end()) {
      ros::Publisher pub = nh_.advertise<visualization_msgs::MarkerArray>(topic, 10);
      publisher_map_[topic] = pub;
    }
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = 0;
    setMarkerColor(marker, color, a);
    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.reserve(texts.size()+1);
    marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(marker);
    marker.action = visualization_msgs::Marker::ADD;
    int i = 0;
    for (const auto& text : texts) {
      setMarkerPose(marker, text[0], text[1], text[2]);
      setMarkerScale(marker, scale, scale, scale);
      // std::ostringstream str;
      // str.precision(2);//覆盖默认精度
      // str.setf(std::ios::fixed);//保留小数位
      // str<<text[3];
      marker.text = prefix[i++];
      marker_array.markers.push_back(marker);
      marker.id ++;
    }
    if(publisher_map_[topic].getNumSubscribers() > 0)
      publisher_map_[topic].publish(marker_array);
  }

  template <class PAIRLINE, class TOPIC>
  // eg for PAIRLINE: std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
  void visualize_pairline(const PAIRLINE& pairline, const TOPIC& topic, const double scale = 0.02, const Color color = red) {
    auto got = publisher_map_.find(topic);
    if (got == publisher_map_.end()) {
      ros::Publisher pub = nh_.advertise<visualization_msgs::Marker>(topic, 10);
      publisher_map_[topic] = pub;
    }
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    setMarkerPose(marker, 0, 0, 0);
    setMarkerColor(marker, color, 0.5);
    setMarkerScale(marker, scale, scale, scale);
    marker.points.resize(2*pairline.size());
    for (size_t i=0; i<pairline.size(); ++i) {
      marker.points[2*i+0].x = pairline[i].first [0];
      marker.points[2*i+0].y = pairline[i].first [1];
      marker.points[2*i+0].z = pairline[i].first [2];
      marker.points[2*i+1].x = pairline[i].second[0];
      marker.points[2*i+1].y = pairline[i].second[1];
      marker.points[2*i+1].z = pairline[i].second[2];
    }
    publisher_map_[topic].publish(marker);
  }

  template <class ARROWS, class TOPIC>
  // ARROWS: pair<Vector3d, Vector3d>
  void visualize_arrows(const ARROWS& arrows, const TOPIC& topic, const Color color = red) {
    auto got = publisher_map_.find(topic);
    if (got == publisher_map_.end()) {
      ros::Publisher pub = nh_.advertise<visualization_msgs::MarkerArray>(topic, 10);
      publisher_map_[topic] = pub;
    }
    visualization_msgs::Marker clear_previous_msg;
    clear_previous_msg.action = visualization_msgs::Marker::DELETEALL;
    visualization_msgs::Marker arrow_msg;
    arrow_msg.type = visualization_msgs::Marker::ARROW;
    arrow_msg.action = visualization_msgs::Marker::ADD;
    arrow_msg.header.frame_id = "world";
    arrow_msg.id = 0;
    arrow_msg.points.resize(2);
    setMarkerPose(arrow_msg, 0, 0, 0);
    setMarkerScale(arrow_msg, 0.07, 0.15, 0);
    setMarkerColor(arrow_msg, color);
    visualization_msgs::MarkerArray arrow_list_msg;
    arrow_list_msg.markers.reserve(1+arrows.size());
    arrow_list_msg.markers.push_back(clear_previous_msg);
    for (const auto& arrow : arrows) {
      arrow_msg.points[0].x = arrow.first [0];
      arrow_msg.points[0].y = arrow.first [1];
      arrow_msg.points[0].z = arrow.first [2];
      arrow_msg.points[1].x = arrow.second[0];
      arrow_msg.points[1].y = arrow.second[1];
      arrow_msg.points[1].z = arrow.second[2];
      arrow_list_msg.markers.push_back(arrow_msg);
      arrow_msg.id += 1;
    }

    publisher_map_[topic].publish(arrow_list_msg);
  }

};

} // namespace visualization

# endif

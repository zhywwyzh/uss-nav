/**
 * Copyright (c) 2017, California Institute of Technology.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of the California Institute of
 * Technology.
 */

#include "apriltag_ros/continuous_detector.h"

#include <pluginlib/class_list_macros.hpp>

#include <nav_msgs/Odometry.h>

PLUGINLIB_EXPORT_CLASS(apriltag_ros::ContinuousDetector, nodelet::Nodelet);

namespace apriltag_ros
{
void ContinuousDetector::onInit ()
{
  ros::NodeHandle& nh = getNodeHandle();
  ros::NodeHandle& pnh = getPrivateNodeHandle();

  tag_detector_ = std::shared_ptr<TagDetector>(new TagDetector(pnh));
  draw_tag_detections_image_ = getAprilTagOption<bool>(pnh, 
      "publish_tag_detections_image", false);
  it_ = std::shared_ptr<image_transport::ImageTransport>(
      new image_transport::ImageTransport(nh));

  std::string transport_hint;
  pnh.param<std::string>("transport_hint", transport_hint, "raw");

  int queue_size;
  pnh.param<int>("queue_size", queue_size, 1);
  camera_image_subscriber_ =
      it_->subscribeCamera("image_rect", queue_size,
                          &ContinuousDetector::imageCallback, this,
                          image_transport::TransportHints(transport_hint));
  tag_detections_publisher_ =
      nh.advertise<AprilTagDetectionArray>("tag_detections", 1);

  tag_position_detections_publisher_ = nh.advertise<nav_msgs::Odometry>("/tag_detections_position", 1);
  
  if (draw_tag_detections_image_)
  {
    tag_detections_image_publisher_ = it_->advertise("tag_detections_image", 1);
  }

  refresh_params_service_ =
      pnh.advertiseService("refresh_tag_params", 
                          &ContinuousDetector::refreshParamsCallback, this);

  // detect_timer_ = nh.createTimer(ros::Duration(0.01), &ContinuousDetector::detect_image, this);
  // get_image();
}


void ContinuousDetector::detect_image(const ros::TimerEvent& event)
{
  if (tag_detections_publisher_.getNumSubscribers() == 0 &&
      tag_detections_image_publisher_.getNumSubscribers() == 0 &&
      !tag_detector_->get_publish_tf())
  {
    // ROS_INFO_STREAM("No subscribers and no tf publishing, skip processing.");
    return;
  }

  if(frame.empty()){
    return;
  }
  double time0 = ros::Time::now().toSec();
  cv::Mat image_cv;
  frame.copyTo(image_cv);
  AprilTagDetectionArray tag_detector_0;

  cv::imshow("image_raw", frame);
  cv::waitKey('1');

  double time1 = ros::Time::now().toSec();

  tag_detector_0 = tag_detector_->detectTags_image(image_cv);
  double time2 = ros::Time::now().toSec();

  tag_detections_publisher_.publish(
      tag_detector_0);

  if(!tag_detector_0.detections.empty()){
    nav_msgs::Odometry tag_odom;
    tag_odom.header.stamp = ros::Time::now();
    tag_odom.pose.pose.position.x = tag_detector_0.detections[0].pose.pose.pose.position.x;
    tag_odom.pose.pose.position.y = tag_detector_0.detections[0].pose.pose.pose.position.y;
    tag_odom.pose.pose.position.z = tag_detector_0.detections[0].pose.pose.pose.position.z;
    tag_position_detections_publisher_.publish(tag_odom);
  }

  
  if (draw_tag_detections_image_)
  {
    tag_detector_->drawDetections_image(image_cv);
    cv::namedWindow("image", cv::WINDOW_NORMAL);  // 创建可调整大小的窗口
    cv::resizeWindow("image", 640, 480);  // 设置窗口尺寸
    cv::imshow("image", image_cv);
    cv::waitKey('1');
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_cv).toImageMsg();
    tag_detections_image_publisher_.publish(msg);
  }
  double time3 = ros::Time::now().toSec();
  // std::cout << "detect_time:" << time2 - time1 ;
  // std::cout << "    pub_time:" << time3 - time2 ;
  // std::cout << "    read_time:" << time1 - time0 << std::endl;
}

void ContinuousDetector::get_image()
{
  cv::VideoCapture capture("nvarguscamerasrc ! video/x-raw(memory:NVMM), width=960, height=720,  \
                        format=(string)NV12, framerate=(fraction)60/1 ! nvvidconv flip-method=0 ! \
                        video/x-raw, width=960, height=720, format=(string)BGRx ! videoconvert ! \ 
                        video/x-raw, format=(string)BGR ! appsink", cv::CAP_GSTREAMER);
  
  if (!capture.isOpened()) {
      std::cerr << "Error opening the camera" << std::endl;
      return;
  }
  try 
  {
    while (ros::ok()) {
      capture >> frame;
      // std::cout << "152:" << ros::Time::now() <<std::endl;
      if (frame.empty()) {
          std::cerr << "Error reading frame" << std::endl;
          break;
      }
    }
  }
  catch(const std::exception& e) 
  {
          std::cerr << "An exception occurred: " << e.what() << std::endl;
  }
  // cv::destroyAllWindows();
  capture.release();
}

void ContinuousDetector::refreshTagParameters()
{
  // Resetting the tag detector will cause a new param server lookup
  // So if the parameters have changed (by someone/something), 
  // they will be updated dynamically
  std::scoped_lock<std::mutex> lock(detection_mutex_);
  ros::NodeHandle& pnh = getPrivateNodeHandle();
  tag_detector_.reset(new TagDetector(pnh));
}

bool ContinuousDetector::refreshParamsCallback(std_srvs::Empty::Request& req,
                                               std_srvs::Empty::Response& res)
{
  refreshTagParameters();
  return true;
}

void ContinuousDetector::imageCallback (
    const sensor_msgs::ImageConstPtr& image_rect,
    const sensor_msgs::CameraInfoConstPtr& camera_info)
{
  double time1 = ros::Time::now().toSec();
  std::scoped_lock<std::mutex> lock(detection_mutex_);
  // Lazy updates:
  // When there are no subscribers _and_ when tf is not published,
  // skip detection.
  if (tag_detections_publisher_.getNumSubscribers() == 0 &&
      tag_detections_image_publisher_.getNumSubscribers() == 0 &&
      !tag_detector_->get_publish_tf())
  {
    // ROS_INFO_STREAM("No subscribers and no tf publishing, skip processing.");
    return;
  }

  // Convert ROS's sensor_msgs::Image to cv_bridge::CvImagePtr in order to run
  // AprilTag 2 on the iamge
  try
  {
    cv_image_ = cv_bridge::toCvCopy(image_rect, image_rect->encoding);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Publish detected tags in the image by AprilTag 2
  AprilTagDetectionArray tag_detector_0;
  tag_detector_0 = tag_detector_->detectTags(cv_image_,camera_info);

  double time2 = ros::Time::now().toSec();

  tag_detections_publisher_.publish(
      tag_detector_0);

  if(!tag_detector_0.detections.empty()){
    nav_msgs::Odometry tag_odom;
    tag_odom.header.stamp = ros::Time::now();
    tag_odom.pose.pose.position.x = tag_detector_0.detections[0].pose.pose.pose.position.x;
    tag_odom.pose.pose.position.y = tag_detector_0.detections[0].pose.pose.pose.position.y;
    tag_odom.pose.pose.position.z = tag_detector_0.detections[0].pose.pose.pose.position.z;
    tag_position_detections_publisher_.publish(tag_odom);
  }
  

  // Publish the camera image overlaid by outlines of the detected tags and
  // their payload values
  if (draw_tag_detections_image_)
  {
    tag_detector_->drawDetections(cv_image_);
    tag_detections_image_publisher_.publish(cv_image_->toImageMsg());
  }

  double time3 = ros::Time::now().toSec();
  // std::cout << "detect_time:" << time2 - time1;
  // std::cout << "   pub_time:" << time3 - time2;
  // std::cout << "   total_time:" << time3 - time1 << std::endl;
}

} // namespace apriltag_ros

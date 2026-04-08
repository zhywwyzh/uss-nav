/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <tf/transform_listener.h>

#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include "rviz/display_context.h"
#include "rviz/properties/string_property.h"

#include <quadrotor_msgs/GoalSet.h>

#include "goal_tool.h"

namespace rviz
{

Goal3DTool::Goal3DTool()
{
  shortcut_key_ = 'g';

  topic_property_ = new StringProperty("Topic", "goal",
                                       "The topic on which to publish navigation goals.",
                                       getPropertyContainer(), SLOT(updateTopic()), this);
  // topic_property_droneID_ = new StringProperty("Topic", "goal_with_id",
  //                                              "The topic on which to publish navigation goals.",
  //                                              getPropertyContainer(), SLOT(updateTopic()), this);
}

void Goal3DTool::onInitialize()
{
  Pose3DTool::onInitialize();
  setName("3D Nav Goal");
  updateTopic();
}

void Goal3DTool::updateTopic()
{
  pub_goal_ = nh_.advertise<geometry_msgs::PoseStamped>(topic_property_->getStdString(), 1);
  pub_droneID_goal_ = nh_.advertise<quadrotor_msgs::GoalSet>("/goal_with_id_from_station", 1);
  chosen_drone_sub_=  nh_.subscribe("/chosen_single_drone_start_trigger", 100,&Goal3DTool::chosen_drone_sub_cb, this);
}
void Goal3DTool::chosen_drone_sub_cb(const std_msgs::Float64ConstPtr &msg)
{
  std::cout<<"\033[32m [Goal3DTool] chosen_drone_sub_cb: "<<msg->data<< "\033[0m"<<std::endl;
  currentflagZ=msg->data;
  return;
}
void Goal3DTool::onPoseSet(double x, double y, double z, double theta)
{
  ROS_WARN("3D Goal Set");
  std::string fixed_frame = context_->getFixedFrame().toStdString();
  tf::Quaternion quat;
  quat.setRPY(0.0, 0.0, theta);
  tf::Stamped<tf::Pose> p = tf::Stamped<tf::Pose>(tf::Pose(quat, tf::Point(x, y, z)), ros::Time::now(), fixed_frame);
  geometry_msgs::PoseStamped goal;
  tf::poseStampedTFToMsg(p, goal);
  ROS_WARN("Setting goal: Frame:%s, Position(%.3f, %.3f, %.3f), Orientation(%.3f, %.3f, %.3f, %.3f) = Angle: %.3f\n", fixed_frame.c_str(),
           goal.pose.position.x, goal.pose.position.y, goal.pose.position.z,
           goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w, theta);
  pub_goal_.publish(goal);

  quadrotor_msgs::GoalSet goal_with_id;
  goal_with_id.to_drone_ids.push_back(1);
  ROS_WARN_STREAM("goal_with_id.drone_id: "<< goal_with_id.to_drone_ids[0]); //通过single_goal_chosen来指定ID drone
  geometry_msgs::Point pt;
  pt.x = x; pt.y = y; pt.z = z;
  goal_with_id.goal.push_back(pt);
  goal_with_id.yaw.push_back(0.0);
  goal_with_id.look_forward = true;
  goal_with_id.goal_to_follower = false;
  pub_droneID_goal_.publish(goal_with_id);
}

} // end namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz::Goal3DTool, rviz::Tool)

//
// Created by gwq on 9/21/24.
//
#include <ros/ros.h>
#include <plan_manage/ego_replan_fsm.h>
#include <visualization_msgs/Marker.h>
#include <map_interface/map_interface.hpp>
#include <rc_fsm.h>
// msgs
#include <quadrotor_msgs/EgoGoalSet.h>
#include <quadrotor_msgs/GoalSet.h>

using ego_planner::EGOReplanFSM;
using ego_planner::MapInterface;

ros::Publisher ego_goal_pub_;
ros::Subscriber goal_set_sub_;

// void goalSetCallback(const quadrotor_msgs::GoalSet::ConstPtr& msg);
// void pubLocalGoal(Eigen::Vector3d local_aim_pose, double yaw_aim, bool look_forward);

int main(int argc, char **argv){
  // Set CPU affinity for thread: ego_planner [core 0]
  // int core_id = 0;
  // cpu_set_t cpuset;
  // CPU_ZERO(&cpuset);
  // CPU_SET(core_id, &cpuset);
  // if (sched_setaffinity(0, sizeof(cpu_set_t), &cpuset) == -1) {
  //   std::cerr << "Failed to set CPU affinity for thread: planner "<< std::endl;
  // }

  ros::init(argc, argv, "ego_planner_node");
  ros::NodeHandle nh("~");
  // goal_set_sub_ = nh.subscribe("/goal_with_id_from_station", 2, &goalSetCallback,
  //                                             ros::TransportHints().tcpNoDelay(true));
  // ego_goal_pub_ = nh.advertise<quadrotor_msgs::EgoGoalSet>("local_goal", 10);


  INFO_MSG_GREEN(">>>>>>>>>>>> EGO INIT >>>>>>>>>>>>");
  EGOReplanFSM ego_planner_fsm_;
  ego_planner_fsm_.init(nh);
  INFO_MSG_GREEN("<<<<<<<<<< EGO INIT DONE <<<<<<<<<");


  INFO_MSG_GREEN(">>>>>>>>>>>> MAP INTERFACE INIT >>>>>>>>>>>>");
  MapInterface::Ptr map_;
  map_.reset(new MapInterface(nh, ego_planner_fsm_.getMapPtr()));
  INFO_MSG_GREEN("<<<<<<<<<<<< MAP_INTERFACE INIT <<<<<<<<<<<<");

  std::shared_ptr<RCCtrlFSM> rc_fsm_;
  rc_fsm_.reset(new RCCtrlFSM(nh, map_));

  ros::spin();
  return 0;
}

// void goalSetCallback(const quadrotor_msgs::GoalSet::ConstPtr& msg){
//   INFO_MSG_BLUE("[MAIN]: ===== Received new goal set from station =====");
//   pubLocalGoal(Eigen::Vector3d(msg->goal[0].x, msg->goal[0].y, msg->goal[0].z), msg->yaw[0], true);
// }
//
// void pubLocalGoal(Eigen::Vector3d local_aim_pose, double yaw_aim, bool look_forward){
//   quadrotor_msgs::EgoGoalSet ego_goal_msg;
//   ego_goal_msg.drone_id = 0;
//   ego_goal_msg.goal[0]  = local_aim_pose.x();
//   ego_goal_msg.goal[1]  = local_aim_pose.y();
//   ego_goal_msg.goal[2]  = 1.6;
//   ego_goal_msg.yaw              = yaw_aim;
//   ego_goal_msg.look_forward     = true;
//   ego_goal_msg.goal_to_follower = false;
//   ego_goal_pub_.publish(ego_goal_msg);
// }


#ifndef _FAST_EXPLORATION_FSM_H_
#define _FAST_EXPLORATION_FSM_H_

#include <Eigen/Eigen>

#include <Eigen/src/Core/Matrix.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/BatteryState.h>

#include <quadrotor_msgs/Instruction.h>
#include <quadrotor_msgs/InstructionResMsg.h>
#include <quadrotor_msgs/EgoPlannerResult.h>
#include <quadrotor_msgs/MultiPoseGraph.h>
#include <quadrotor_msgs/HgridMsg.h>
#include <quadrotor_msgs/FrontierMsg.h>
#include <quadrotor_msgs/PerceptionMsg.h>
#include <quadrotor_msgs/EgoGoalSet.h>
#include <quadrotor_msgs/GoalSet.h>
#include <quadrotor_msgs/DetectOut.h>
#include <quadrotor_msgs/TrackCommand.h>

#include <algorithm>
#include <iostream>
#include <vector>
#include <memory>
#include <string>
#include <thread>
#include <deque>
#include <mutex>
#include <exploration_manager/frontier_manager.h>
#include <exploration_manager/mission_data.h>
#include <scene_graph/object_factory.h>
#include <scene_graph/scene_graph.h>
#include <scene_graph/traj_visualizer.h>
#include <std_msgs/Bool.h>

using Eigen::Vector3d;
using std::vector;
using std::shared_ptr;
using std::unique_ptr;
using std::string;

namespace ego_planner {
class Tabv;
class FrontierManager;
class PlanningVisualization;
struct FSMParam;
struct FSMData;
class EkfEstimator;
class PerceptionDataMsgFactory;


class FastExplorationFSM {
private:
  /* planning utils */
  MapInterface::Ptr                                 map_;
  shared_ptr<FrontierManager>                       expl_manager_;
  shared_ptr<SceneGraph>                            scene_graph_;
  shared_ptr<TrajectoryVisualizer>                  traj_visualizer_;
  shared_ptr<PlanningVisualization>                 visualization_;
  shared_ptr<FSMParam>                              fp_;
  shared_ptr<FSMData>                               fd_;
  shared_ptr<MissionData>                           md_;
  double                                            scale_;

  std::mutex mtx_; 

  bool classic_;

  /* ROS utils */
  ros::NodeHandle node_;
  ros::Timer exec_timer_, frontier_timer_;
  ros::Subscriber trigger_sub_, odom_sub_, goal_from_station_sub_, egoplanner_goal_sub_, ego_exec_finish_sub_;
  ros::Subscriber track_command_sub_, target_sub_;
  ros::Subscriber instruction_sub_, ego_plan_res_sub_, battery_sub_, perception_data_sub_;
  ros::Publisher ego_goal_pub_, perception_data_pub_, instruction_resp_pub_;
  ros::Publisher vis_marker_pub_, vis_path_pub_;

  // LLM related
  MISSION_FSM_STATE stash_state_{MISSION_FSM_STATE::UNKONWN};
  unsigned int cur_prompt_id_{0};
  bool has_made_area_decision_{false}, need_rotate_yaw_{false};    // only used for llm plan
  int expl_area_id_{-1};
  double think_duration_limit_;
  double think_start_time_;

 private:
  /* helper functions */
  int callExplorationPlanner(Eigen::Vector3d& aim_pose, Eigen::Vector3d& aim_vel, double& aim_yaw,
                             vector<Eigen::Vector3d>& path_res);
  int callExplorationLLMPlanner(Eigen::Vector3d& aim_pose, Eigen::Vector3d& aim_vel, double& aim_yaw,
                                vector<Eigen::Vector3d>& path_res);
  int callTrackPlanner(Eigen::Vector3d& aim_pose, Eigen::Vector3d& aim_vel, double& aim_yaw,
                       vector<Eigen::Vector3d>& path_res);
  
  void transitState(MISSION_FSM_STATE new_state, string pos_call);
  void stashCurStateAndTransit(MISSION_FSM_STATE new_state, string who_called);
  bool getSceneGraphInitSeed(Eigen::Vector3d& init_seed, std::string* reason = nullptr) const;

  /* ROS functions */
  void FSMCallback(const ros::TimerEvent& e);
  void frontierCallback(const ros::TimerEvent& e);
  void triggerCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void egoPlannerGoalCallback(const quadrotor_msgs::GoalSet::ConstPtr& msg);
  void egoExecFinishCallback(const std_msgs::Bool::ConstPtr& msg);
  void trackCommandCallback(const quadrotor_msgs::TrackCommand::ConstPtr& msg);
  void targetCallbackReal(const quadrotor_msgs::DetectOut::ConstPtr& msg);
  void handleGoalInstruction(const std::vector<geometry_msgs::Point>& goals, const std::vector<float>& yaws,
                             bool look_forward, const std::string& source);
  void handleTrackingTarget(const std::vector<geometry_msgs::Point>& global_poses, const std::string& source);

  void instructionCallback(const quadrotor_msgs::InstructionConstPtr& msg);
  void batteryCallBack(const sensor_msgs::BatteryState msg);
  void odometryCallback(const nav_msgs::OdometryConstPtr& msg);
  void egoPlanResCallback(const quadrotor_msgs::EgoPlannerResultConstPtr& msg);
  bool getAndPublishNextAim(vector<Eigen::Vector3d>& path_res,
                              const bool look_forward = true, const double aim_yaw = 0.0);
  void pubLocalGoal(const Eigen::Vector3d local_goal, const double yaw = 0.0, const bool look_forward = true, bool yaw_low_speed = false);
  void stopMotion();

  void handelThingkingProcess();
  void planLLMExplore();
  void planRegularExplore();
  void approachRegularExplore();
  void planTrack();
  void approachTrack();
  void handleYawChange();                  // scan the area (Fov expand) and update the map
  void goTargetObject();
  void findTerminateTarget();
  void execDFDemo();

  double adjustTerminateHeightFindingObject(ObjectNode::Ptr target_obj, Eigen::Vector3d init_pos, bool final_point=false);
  double adjustTerminateHeightNormal(const Eigen::Vector3d& next_aim_raw);

  double yawhandle_yaw_raw;
  double yawhandle_yaw_target_left ;
  double yawhandle_yaw_target_right ;
  bool   yawhandle_left_published, yawhandle_right_published, yawhandle_back_published;
  bool   yawhandle_left_ok, yawhandle_right_ok, yawhandle_back_ok;

  void hardResetExploreArea(bool clear_posegraph);

  void displayMissionState();
  void displayPath();
  void visualize(const ros::TimerEvent& e);

  // TOOLS
  void geoPt2Vec3d(const geometry_msgs::Point &p_in, Eigen::Vector3d &p_out);
  void vec3d2GeoPt(const Eigen::Vector3d &p_in, geometry_msgs::Point &p_out);
  geometry_msgs::Point vec3d2GeoPt(const Eigen::Vector3d &p_in);
  Eigen::Vector3d geoPt2Vec3d(const geometry_msgs::Point &p_in);

public:
  FastExplorationFSM(/* args */) {
  }
  ~FastExplorationFSM() {
      scene_graph_->object_factory_->stopThisModule();
  }

  void init(ros::NodeHandle& nh, const MapInterface::Ptr& map);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace ego planner

#endif

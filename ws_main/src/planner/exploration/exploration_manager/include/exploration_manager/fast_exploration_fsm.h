#ifndef _FAST_EXPLORATION_FSM_H_
#define _FAST_EXPLORATION_FSM_H_

#include <Eigen/Eigen>

#include <Eigen/src/Core/Matrix.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/CompressedImage.h>

#include <quadrotor_msgs/Instruction.h>
#include <quadrotor_msgs/InstructionResMsg.h>
#include <quadrotor_msgs/EgoPlannerResult.h>
#include <quadrotor_msgs/MultiPoseGraph.h>
#include <quadrotor_msgs/HgridMsg.h>
#include <quadrotor_msgs/FrontierMsg.h>
#include <quadrotor_msgs/PerceptionMsg.h>
#include <quadrotor_msgs/EgoGoalSet.h>
#include <quadrotor_msgs/EgoStateTrigger.h>
#include <quadrotor_msgs/GoalSet.h>
#include <quadrotor_msgs/DetectOut.h>
#include <quadrotor_msgs/TrackCommand.h>
#include <quadrotor_msgs/VLASwarmBBox.h>
#include <quadrotor_msgs/VLASwarmTarget.h>

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
#include <exploration_manager/vla_swarm_map.h>
#include <scene_graph/object_factory.h>
#include <scene_graph/counting_scene_graph.h>
#include <scene_graph/scene_graph.h>
#include <scene_graph/traj_visualizer.h>
#include <scene_graph/VLASwarmObservation.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>

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
  CountingSceneGraph::Ptr                           counting_scene_graph_;
  shared_ptr<TrajectoryVisualizer>                  traj_visualizer_;
  VLASwarmMap::Ptr                                  vla_swarm_map_;
  shared_ptr<PlanningVisualization>                 visualization_;
  shared_ptr<FSMParam>                              fp_;
  shared_ptr<FSMData>                               fd_;
  shared_ptr<MissionData>                           md_;
  double                                            scale_;

  std::mutex mtx_; 

  bool classic_;

  /* ROS utils */
  ros::NodeHandle node_;
  ros::Timer exec_timer_, frontier_timer_, vla_swarm_map_timer_;
  ros::Subscriber trigger_sub_, odom_sub_, goal_from_station_sub_, egoplanner_goal_sub_, ego_exec_finish_sub_;
  ros::Subscriber track_command_sub_, target_sub_, elastic_tracking_finish_sub_;
  ros::Subscriber instruction_sub_, ego_plan_res_sub_, battery_sub_, perception_data_sub_, emergency_stop_sub_;
  ros::Subscriber vla_swarm_target_sub_, vla_swarm_camera_sub_;
  ros::Subscriber vla_swarm_ego_state_trigger_sub_;
  ros::Subscriber object_id_nav_replan_sub_;    // 订阅 /object_id_nav_replan
  ros::Publisher ego_goal_pub_, goal_from_station_pub_, perception_data_pub_, instruction_resp_pub_;
  ros::Publisher vis_marker_pub_, vis_path_pub_;
  ros::Publisher fsm_state_pub_;
  ros::Publisher tracking_finish_pub_;
  ros::Publisher tracking_target_odom_pub_;
  ros::Publisher tracking_target_3d_pub_;
  ros::Publisher planner_cmd_mux_mode_pub_;
  ros::Publisher elastic_tracker_trigger_pub_;
  ros::Publisher elastic_tracker_stop_pub_;
  ros::Publisher exploration_result_pub_;
  ros::Publisher vla_swarm_result_pub_;
  ros::Publisher vla_swarm_bbox_pub_;
  ros::Publisher vla_swarm_observation_pub_;

  // LLM related
  MISSION_FSM_STATE stash_state_{MISSION_FSM_STATE::UNKONWN};
  unsigned int cur_prompt_id_{0};
  bool has_made_area_decision_{false}, need_rotate_yaw_{false};    // only used for llm plan
  bool enable_yaw_scan_{false};                                   // 是否执行+45°/-45°/回正扫描
  bool enable_scene_graph_update_after_load_{true};               // 载入预存地图后是否继续增量更新场景图
  // 仅由source_task_id=EXPLORATION/COUNTING开启的360度全景旋转状态
  bool need_panorama_{false};
  bool panorama_command_active_{false};
  uint8_t active_instruction_task_id_{0};
  uint32_t active_instruction_session_id_{0};
  // 探索任务计时：收到 TURN_REGULAR_EXPLORATION 启动，publishExplorationResult 结算
  ros::Time exploration_start_time_;
  bool exploration_timer_active_{false};
  std::string exploration_timing_dir_;  // 探索耗时日志目录（默认 logs/exploration_timing）
  double panorama_last_odom_yaw_{0.0};
  double panorama_start_yaw_{0.0};
  double panorama_unwrapped_yaw_{0.0};
  double panorama_accumulated_yaw_{0.0};
  double panorama_command_target_yaw_{0.0};
  Eigen::Vector3d panorama_hold_pos_{Eigen::Vector3d::Zero()};
  double panorama_max_step_{2.0943951023931953};
  double panorama_extend_angle_{0.6981317007977318};
  bool wait_fresh_map_after_reset_{false};
  uint64_t map_reset_update_seq_{0};
  int expl_area_id_{-1};
  double think_duration_limit_;
  double think_start_time_;

  // VLA_Swarm 独立任务上下文。
  bool vla_swarm_enabled_{false};
  bool vla_swarm_active_{false};
  bool vla_swarm_result_published_{false};
  bool vla_swarm_success_{false};
  uint32_t vla_swarm_session_id_{0};
  std::string vla_swarm_command_;
  std::string vla_swarm_finish_reason_;
  std::string vla_swarm_finish_detail_;
  std::string vla_swarm_result_topic_{"/planning/vla_swarm_result"};
  std::string vla_swarm_bbox_topic_{"/vla_swarm/bbox"};
  std::string vla_swarm_target_topic_{"/vla_swarm/target"};
  std::string vla_swarm_camera_topic_;
  std::string vla_swarm_observation_topic_{"/vla_swarm/observation"};
  bool vla_swarm_prompt_pending_{false};
  bool vla_swarm_place_checked_{false};
  int vla_swarm_explore_area_id_{-1};
  unsigned int vla_swarm_prompt_id_{0};
  uint8_t vla_swarm_prompt_type_{0};
  uint32_t vla_swarm_observation_batch_id_{0};
  uint32_t vla_swarm_target_request_id_{0};
  ros::Time vla_swarm_prompt_start_time_;
  ros::Time vla_swarm_target_start_time_;
  ros::Time vla_swarm_observation_stamp_;
  sensor_msgs::CompressedImageConstPtr vla_swarm_latest_camera_image_;
  ros::Time vla_swarm_latest_camera_receive_time_;
  std::mutex vla_swarm_camera_mutex_;
  std::vector<double> vla_swarm_scan_yaw_offsets_;
  size_t vla_swarm_scan_index_{0};
  double vla_swarm_scan_base_yaw_{0.0};
  double vla_swarm_scan_target_yaw_{0.0};
  Eigen::Vector3d vla_swarm_scan_hold_position_{Eigen::Vector3d::Zero()};
  ros::Time vla_swarm_scan_command_time_;
  ros::Time vla_swarm_scan_yaw_reached_time_;
  bool vla_swarm_scan_initialized_{false};
  bool vla_swarm_scan_command_published_{false};
  bool vla_swarm_target_pending_{false};
  bool vla_swarm_target_received_{false};
  bool vla_swarm_target_success_{false};
  uint8_t vla_swarm_target_observation_index_{0};
  uint8_t vla_swarm_target_source_{0};
  Eigen::Vector3d vla_swarm_target_position_{Eigen::Vector3d::Zero()};
  std::string vla_swarm_target_error_;
  std::vector<Eigen::Vector3d> vla_swarm_path_;
  ros::Time vla_swarm_waypoint_publish_time_;
  bool vla_swarm_path_reaches_task_target_{false};
  bool vla_swarm_waypoint_published_{false};
  bool vla_swarm_waypoint_is_final_{false};
  bool vla_swarm_plan_feedback_received_{false};
  bool vla_swarm_plan_feedback_success_{false};
  int vla_swarm_waypoint_retry_count_{0};
  double vla_swarm_prompt_timeout_{20.0};
  double vla_swarm_target_timeout_{10.0};
  double vla_swarm_ego_plan_timeout_{5.0};
  double vla_swarm_ego_exec_timeout_{30.0};
  int vla_swarm_max_plan_retries_{2};
  int vla_swarm_max_target_retries_{2};
  double vla_swarm_waypoint_distance_{2.0};
  double vla_swarm_goal_tolerance_{0.5};
  double vla_swarm_flight_height_{1.0};
  double vla_swarm_map_update_period_{1.0};
  double vla_swarm_scan_yaw_tolerance_{0.08};
  double vla_swarm_scan_settle_time_{0.4};
  double vla_swarm_scan_timeout_{8.0};
  double vla_swarm_scan_yaw_step_deg_{90.0};
  bool vla_swarm_ego_stable_{true};
  int vla_swarm_exploration_round_{0};
  int vla_swarm_max_exploration_rounds_{6};
  // AA 阶段：全局评估与跨轮记忆，参照原始 VLA_Swarm 的 AA→A→B→C→TASK_OVER 链路
  bool vla_swarm_aa_done_{false};
  nlohmann::json vla_swarm_key_action_history_;
  std::map<int, std::string> vla_swarm_room_descriptions_;
  bool vla_swarm_enable_room_description_{false};

 private:
  /* helper functions */
  int callExplorationPlanner(Eigen::Vector3d& aim_pose, Eigen::Vector3d& aim_vel, double& aim_yaw,
                             vector<Eigen::Vector3d>& path_res);
  int callExplorationLLMPlanner(Eigen::Vector3d& aim_pose, Eigen::Vector3d& aim_vel, double& aim_yaw,
                                vector<Eigen::Vector3d>& path_res);
  int callTrackPlanner(Eigen::Vector3d& aim_pose, Eigen::Vector3d& aim_vel, double& aim_yaw,
                       vector<Eigen::Vector3d>& path_res);
  void resetTrackingFinishCandidate();
  bool updateTrackingFinishCandidate(double dis_2_aim, double angle_2_aim);
  void publishTrackingFinish();
  bool useElasticTrackerBackend() const;
  void publishPlannerCmdMuxMode(const std::string& mode, const std::string& source);
  void switchPlannerCmdMuxToEgo(const std::string& source);
  void switchPlannerCmdMuxToElastic(const std::string& source);
  void publishElasticTrackerTrigger(const ros::Time& stamp = ros::Time(),
                                    const std::string& frame_id = "world");
  void stopElasticTracker(const std::string& source);
  void publishTrackingTargetOdom(const Eigen::Vector3d& target_pos,
                                 const ros::Time& stamp = ros::Time(),
                                 const std::string& frame_id = "world");
  void applyExplorationRegionFromInstruction(const quadrotor_msgs::InstructionConstPtr& msg);
  void publishExplorationResult(bool success, const std::string& reason,
                                const std::string& message = "");
  // 探索任务计时：收到 TURN_REGULAR_EXPLORATION 时启动，publishExplorationResult 时结算并写入本地文件
  void logExplorationTiming(bool success, const std::string& reason, const std::string& message);
  bool isVlaSwarmState(MISSION_FSM_STATE state) const;
  void resetVlaSwarmContext();
  void startVlaSwarmTask(const quadrotor_msgs::InstructionConstPtr& msg);
  void cancelVlaSwarmTask(const std::string& reason, const std::string& detail);
  void publishVlaSwarmResult(bool success, const std::string& reason,
                             const std::string& detail = "");
  bool startVlaSwarmTargetRequest(const nlohmann::json& payload);
  bool prepareVlaSwarmPath(const Eigen::Vector3d& requested_goal,
                           bool reaches_task_target, int door_id = -1);
  bool publishNextVlaSwarmWaypoint();
  void retryVlaSwarmWaypoint(const std::string& failure_reason);
  void handleVlaSwarmPlanLocal();
  void handleVlaSwarmWaitLLM();
  void handleVlaSwarmWaitTarget();
  void handleVlaSwarmApproach();
  void handleVlaSwarmYaw();
  void handleVlaSwarmRecovery();
  void handleVlaSwarmFinish();
  
  void transitState(MISSION_FSM_STATE new_state, string pos_call);
  void stashCurStateAndTransit(MISSION_FSM_STATE new_state, string who_called);
  void triggerObjectIdNavReplan(const std::string& reason);  // object-id-nav replan

  // === 探索脱困通用 helper (建议 D/E: 抽取自 goTargetObject 的卡死检测, 供 approachRegularExplore 复用) ===
  // 背景: 旧探索路径无卡死强制推进机制, 与 goTargetObject 的 tier1/tier2 严重不对称.
  bool detectExploreStuck();        // 统一卡死检测: vel+yaw_rate+计时, 返回是否处于卡死状态
  void resetExploreStuckState();    // 重置探索卡死状态(正常推进/重规划时调用, 含失败计数清零)
  void resetExploreEgoState();      // 重置 ego 反馈相关状态(删 frontier/重规划时调用, 建议E)
  bool getSceneGraphInitSeed(Eigen::Vector3d& init_seed, std::string* reason = nullptr) const;

  /* ROS functions */
  void FSMCallback(const ros::TimerEvent& e);
  void frontierCallback(const ros::TimerEvent& e);
  void vlaSwarmMapCallback(const ros::TimerEvent& e);
  void triggerCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void egoPlannerGoalCallback(const quadrotor_msgs::GoalSet::ConstPtr& msg);
  void egoExecFinishCallback(const std_msgs::Bool::ConstPtr& msg);
  void trackCommandCallback(const quadrotor_msgs::TrackCommand::ConstPtr& msg);
  void elasticTrackingFinishCallback(const std_msgs::Bool::ConstPtr& msg);
  void targetCallbackReal(const quadrotor_msgs::DetectOut::ConstPtr& msg);
  void vlaSwarmTargetCallback(
      const quadrotor_msgs::VLASwarmTarget::ConstPtr& msg);
  void vlaSwarmCameraCallback(
      const sensor_msgs::CompressedImageConstPtr& msg);
  void vlaSwarmEgoStateTriggerCallback(
      const quadrotor_msgs::EgoStateTrigger::ConstPtr& msg);
  void objectIdNavReplanCallback(const std_msgs::Bool::ConstPtr& msg);
  void handleGoalInstruction(const std::vector<geometry_msgs::Point>& goals, const std::vector<float>& yaws,
                             bool look_forward, const std::string& source);
  void handleTrackingTarget(const std::vector<geometry_msgs::Point>& global_poses,
                            const std::string& source,
                            const ros::Time& stamp = ros::Time(),
                            const std::string& frame_id = "world");

  void instructionCallback(const quadrotor_msgs::InstructionConstPtr& msg);
  void emergencyStopCallback(const std_msgs::Empty::ConstPtr& msg);
  void batteryCallBack(const sensor_msgs::BatteryState msg);
  void odometryCallback(const nav_msgs::OdometryConstPtr& msg);
  void egoPlanResCallback(const quadrotor_msgs::EgoPlannerResultConstPtr& msg);
  bool getAndPublishNextAim(vector<Eigen::Vector3d>& path_res,
                              const bool look_forward = true, const double aim_yaw = 0.0);
  void pubLocalGoal(
      const Eigen::Vector3d local_goal, const double yaw = 0.0, const bool look_forward = true,
      uint8_t yaw_mode = quadrotor_msgs::EgoGoalSet::YAW_MODE_NORMAL,
      uint8_t yaw_path_mode = quadrotor_msgs::EgoGoalSet::YAW_PATH_SHORTEST);
  void stopMotion();

  void handelThingkingProcess();
  void planLLMExplore();
  void planRegularExplore();
  void approachRegularExplore();
  void planTrack();
  void approachTrack();
  void handleYawChange();                  // scan the area (Fov expand) and update the map
  void startPanoramaRotation();            // EXPLORATION/COUNTING启动阶段360°全景旋转
  void handlePanoramaYaw();                // 全景旋转状态处理
  bool waitForFreshMapAfterReset();         // 清图后等待第一帧新地图，再开始全景旋转
  void goTargetObject();
  void goTargetWithWaypoint();
  void findTerminateTarget();
  void execDFDemo();

  double adjustTerminateHeightFindingObject(ObjectNode::Ptr target_obj, Eigen::Vector3d init_pos, bool final_point=false);
  double adjustTerminateHeightNormal(const Eigen::Vector3d& next_aim_raw);

  double yawhandle_yaw_raw;
  double yawhandle_yaw_target_left ;
  double yawhandle_yaw_target_right ;
  bool   yawhandle_left_published, yawhandle_right_published, yawhandle_back_published;
  bool   yawhandle_left_ok, yawhandle_right_ok, yawhandle_back_ok;

  void hardResetExploreArea(bool clear_occupancy, bool clear_posegraph);

  void displayMissionState();
  void displayPath();
  void displayLocalAim();   // 当前导航点橙色marker可视化
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

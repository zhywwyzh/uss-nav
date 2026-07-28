#ifndef _EXPL_DATA_H_
#define _EXPL_DATA_H_

#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <active_perception/ftr_data_structure.h>
#include <sys/types.h>
#include <string>
#include <vector>
#include <quadrotor_msgs/PerceptionMsg.h>
#include <quadrotor_msgs/Instruction.h>
#include <scene_graph/scene_graph.h>
// #include "poly_traj_utils.hpp"


using std::vector;
using Eigen::Vector3d;

namespace ego_planner {

enum TARGET_TYPE
{
  MANUAL_TARGET  = 1,
  EXPLORE_TARGET = 2,
  PRESET_TARGET  = 3,
  REFENCE_PATH   = 4
};


struct FSMData
{
  // FSM data
  bool                    trigger_, have_odom_, static_state_;
  ros::Time               last_pub_time_;
  ros::Time               warmup_start_time_;

  // odometry state
  Eigen::Vector3d         odom_pos_, odom_vel_;
  Eigen::Quaterniond      odom_orient_;
  double                  odom_yaw_;
  double                  odom_yaw_rate_;            // 角速度 rad/s(里程计回调填充)

  // start state
  Eigen::Vector3d         start_pt_, start_vel_, start_acc_, start_yaw_;

  // home info
  Eigen::Vector3d         home_pos_;
  bool                    has_home_path_ = false;

  // plan res
  vector<Eigen::Vector3d> path_res_;                 // path to nxt frontier
  int                     path_inx_;                 // now point index to be executed
  Eigen::Vector3d         aim_pos_, aim_vel_;        //
  Eigen::Vector3d         local_aim_pos_;
  double                  aim_yaw_;
  bool                    has_rotated_;
  bool                    is_lookforward_;
  Eigen::Vector3d         track_pos_;
  bool                    track_trigger_;
  bool                    track_init_;
  bool                    track_finish_candidate_active_{false};
  bool                    track_finish_sent_{false};
  ros::Time               track_finish_candidate_start_time_;
  Eigen::Vector3d         track_finish_last_pos_;
  double                  track_finish_last_yaw_{0.0};
  double                  track_finish_move_acc_{0.0};
  double                  track_finish_yaw_acc_{0.0};
  bool                    directly_connect_to_goal;  // [gwq] fsm 中关于是否直接连接到目标点的flag （包括从机跟随和Instruct主动控制情况）
  bool                    instruct_directly_to_goal;  // [gwq] Instruct 强制使用ego规划目标点 (相较于前者优先级更高)

  //ego-plan res & topo utils
  Eigen::Vector3d         ego_local_goal_;
  int                     ego_plan_times_;
  bool                    ego_plan_status_;
  bool                    ego_modify_status_;
  int                     goal_replan_times_;
  bool                    ego_exec_finished_;
  double                  target_yaw_;                  // only used for turn yaw slowly

  // perception service
  std::unordered_map<int, bool>                              perception_data_get_response_;
  unordered_map<unsigned int, quadrotor_msgs::PerceptionMsg> map_merge_database_;
  quadrotor_msgs::PerceptionMsg                              local_perception_data_;

  // input info
  Eigen::Vector3d next_given_goal_;
  Eigen::Vector3d waypoint_target_;
  double          waypoint_target_yaw_;

  //scene graph
  std::string target_cmd_, prior_knowledge_;
  int object_target_id_;
  u_int8_t go_object_process_phase{0};
  bool     go_object_in_prior_guide_{false}; // true=当前是先验引导(cloud未构建), 到达后回phase0重试
  u_int8_t go_object_prior_guide_count_{0};  // 先验引导重试计数(最多2次)
  u_int8_t go_waypoint_process_phase{0};
  // 卡死强制推进
  double  stuck_begin_time_{-1.0};       // 进入卡死计时起点(秒), -1表示未进入
  int     stuck_force_advance_count_{0}; // 连续强制推进计数
  bool    stuck_force_advance_triggered_{false}; // 本轮是否已触发(防重复)
  // object-id-nav replan 运行时状态
  quadrotor_msgs::Instruction stored_object_id_nav_instruction_; // 缓存的最新 TURN_OBJECT_ID_NAV 消息
  bool has_stored_object_id_nav_instruction_{false};             // 是否有缓存消息
  double object_id_nav_replan_stuck_begin_time_{-1.0};           // 卡死计时起点(秒), -1=未卡死
  bool object_id_nav_replan_topic_triggered_{false};             // 话题触发标记
  int  object_id_nav_replan_stuck_count_{0};                    // 连续replan触发计数
  bool new_topo_need_predict_immediately_{false};
  bool regular_explore_{false};
  bool find_terminate_target_mode_{false};
  u_int8_t llm_plan_explore_counter_{0};

  // DF Demo
  u_int8_t df_demo_phase_{0};
  u_int8_t explore_count_{0};
  int      df_demo_target_id_{-100};
  bool     df_demo_mode_{false};

  // 探索持续重规划：frontier 变化感知
  size_t   frontier_last_count_{0};   // 上次 frontier 数量
  bool     frontier_changed_{false};  // frontier 列表是否发生变化
};

struct FSMParam
{
  double                  replan_dis_thresh_;
  double                  replan_thresh2_;
  double                  replan_thresh3_;
  double                  replan_time_;  // second
  double                  arrive_dis_thr_;
  double                  battery_thr_;
  bool                    flag_realworld_exp_;
  bool                    enable_area_prediction_{false};
  bool                    auto_init_scene_graph_{true};
  double                  auto_init_delay_sec_{2.0};
  double                  scene_graph_init_forward_dist_{1.8};
  double                  frontier_update_dt_{0.5};  // frontier后台刷新周期，单位秒
  double                  track_finish_hold_time_{3.0};
  double                  track_finish_move_thresh_{0.2};
  double                  track_finish_yaw_thresh_{0.2};
  std::string             tracking_backend_{"ego"};
  std::string             tracking_target_odom_topic_{"/target_ekf_odom"};
  std::string             tracking_target_3d_topic_{"/target_3d_pos"};
  std::string             planner_cmd_mux_mode_topic_{"/planner_mux/mode"};
  std::string             planner_cmd_mux_ego_mode_{"ego"};
  std::string             planner_cmd_mux_elastic_mode_{"elastic"};
  std::string             elastic_tracker_trigger_topic_{"/triger"};
  std::string             elastic_tracker_finish_topic_{"/elastic_tracker/tracking_finish"};
  std::string             elastic_tracker_stop_topic_{"/elastic_tracker/stop"};
  // 卡死强制推进参数
  bool   stuck_force_advance_enable_{true};
  double stuck_force_advance_vel_thresh_{0.1};
  double stuck_force_advance_yaw_rate_thresh_{0.1};
  double stuck_force_advance_duration_{3.0};
  int    stuck_force_advance_max_consecutive_{2};
  // object-id-nav replan 参数
  bool   object_id_nav_replan_enable_{false};
  int    object_id_nav_replan_mode_{0};        // 0=both, 1=stuck only, 2=topic only
  double object_id_nav_replan_stuck_vel_thresh_{0.1};
  double object_id_nav_replan_stuck_yaw_rate_thresh_{0.1};
  double object_id_nav_replan_stuck_duration_{3.0};
  int    object_id_nav_replan_stuck_max_consecutive_{0}; // 最大连续触发次数, 0=不限制
  double object_id_nav_replan_mode2_stuck_fallback_delay_{10.0}; // mode2卡死后等待进入topo-block的延迟(s)
  // object-id-nav 导航语义参数
  bool   object_id_nav_require_final_yaw_{true};          // 导航到物体后是否需要旋转面向它
  bool   object_id_nav_use_thinking_{true};               // 是否使用THINKING引导探索(true=LLM引导, false=纯frontier+直接匹配)
  int    object_id_nav_prior_guide_max_retries_{2};       // 先验引导最大重试次数(cloud未构建时)
  // B3 生命周期: frontier 被选为目标但未能消除的累计次数阈值
  int    max_observation_attempts_{3};
};

struct ExplorationData {
  Frontier                          frontier_to_goal, frontier_to_explore_;
  vector<vector<Vector3d>>          frontiers_;
  vector<Frontier>                  frontiers_with_info_;
  vector<vector<Vector3d>>          dead_frontiers_;
  vector<pair<Vector3d, Vector3d>>  frontier_boxes_;
  // MultiPoseGraph::Ptr               posegraph_m_;
  // PoseGraph                         posegraph_used_by_blacklist_cal_;
  std::unordered_map<int, int>      topo_blacklist_;
  bool                              flag_first_plangoal_;
  vector<Vector3d>                  points_;
  vector<Vector3d>                  averages_;
  vector<Vector3d>                  views_;
  vector<double>                    yaws_;
  vector<Vector3d>                  global_tour_;
  map<int, vector<Vector3d>>        global_tour_map_;
  bool                              force_plangoal_by_frontier_;

  vector<Frontier>                  last_frontiers_with_info_;
  vector<int>                       last_indices_;
  bool                              is_gohome = false;
  bool                              is_stick_to_last = false;

  vector<int>                       refined_ids_;
  vector<vector<Vector3d>>          n_points_;
  vector<Vector3d>                  unrefined_points_;
  vector<Vector3d>                  refined_points_;
  vector<Vector3d>                  refined_views_;  // points + dir(yaw)
  vector<Vector3d>                  refined_views1_, refined_views2_;
  vector<Vector3d>                  refined_tour_;

  vector<Vector3d>                  path_next_goal_; // only for visualizaiton
  vector<int>                       last_grid_ids_;

  // viewpoint planning
  // vector<Vector4d> views_;
  vector<Vector3d>                  views_vis1_, views_vis2_;
  vector<Vector3d>                  centers_, scales_;
  typedef std::shared_ptr<ExplorationData> Ptr;
};

struct ExplorationParam
{
  // params
  bool         refine_local_;
  int          refined_num_;
  double       refined_radius_;
  int          top_view_num_;
  double       max_decay_;
  std::string  tsp_dir_;  // resource dir of tsp solver
  double       relax_time_;
  double       radius_close_;
  double       radius_far_;
  int          frontier_tsp_mode_{0};  // 0: 当前帧更新盒；1: FUEL式累计更新盒并在TSP前同步刷新
  double       track_dist_;
  double       track_dist_thr_;
  double       track_replan_dist_;
  double       track_turn_yaw_dist_;
  double       track_fly_yaw_thr_;
  double       track_yaw_thr_;
  double       track_detect_error_;

  typedef std::shared_ptr<ExplorationParam> Ptr;
};

}  // namespace ego_planner

#endif

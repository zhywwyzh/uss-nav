#ifndef _EXPL_DATA_H_
#define _EXPL_DATA_H_

#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <active_perception/ftr_data_structure.h>
#include <sys/types.h>
#include <vector>
#include <quadrotor_msgs/PerceptionMsg.h>
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
  u_int8_t go_waypoint_process_phase{0};
  bool new_topo_need_predict_immediately_{false};
  bool regular_explore_{false};
  bool find_terminate_target_mode_{false};
  u_int8_t llm_plan_explore_counter_{0};

  // DF Demo
  u_int8_t df_demo_phase_{0};
  u_int8_t explore_count_{0};
  int      df_demo_target_id_{-100};
  bool     df_demo_mode_{false};
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
  double       track_dist_;
  double       track_dist_thr_;
  double       track_replan_dist_;
  double       track_turn_yaw_dist_;
  double       track_yaw_thr_;
  double       track_detect_error_;

  typedef std::shared_ptr<ExplorationParam> Ptr;
};

}  // namespace ego_planner

#endif

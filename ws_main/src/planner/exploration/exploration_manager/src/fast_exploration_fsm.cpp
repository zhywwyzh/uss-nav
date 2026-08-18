#include <Eigen/Eigen>
#include <Eigen/src/Core/Matrix.h>
#include <array>
#include <chrono>
#include <cmath>
#include <ctime>           // 探索耗时日志：strftime/localtime
#include <fstream>         // 探索耗时日志：ofstream
#include <iomanip>         // 探索耗时日志：setprecision
#include <limits>
#include <sstream>
#include <stdexcept>
#include <exploration_manager/mission_data.h>
#include <map_interface/map_interface.hpp>
#include <ostream>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/package.h>            // 性能日志：ros::package::getPath 获取默认日志目录
#include <ros/time.h>
#include <plan_env/perf_logger.h>   // 性能日志插桩
#include <scene_graph/PromptMsg.h>
#include <scene_graph/data_structure.h>
#include <scene_graph/scene_graph.h>
#include <scene_graph/skeleton_generation.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <string>
#include <traj_utils/planning_visualization.h>
#include <exploration_manager/fast_exploration_fsm.h>
#include <exploration_manager/expl_data.h>
#include <plan_env/grid_map.h>
#include <memory>
#include <unistd.h>
#include <visualization_msgs/MarkerArray.h>

#define CALL_EVERY_N_TIMES(func, n)         \
    do {                                    \
        static int counter = 0;             \
        ++counter;                          \
        if (counter >= (n)) {               \
            func();                         \
            counter = 0;                    \
        }                                   \
    } while (0)
using Eigen::Vector4d;

namespace ego_planner {
namespace {
double normalizeAngle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

std::string jsonEscape(const std::string& value) {
  std::string out;
  out.reserve(value.size());
  for (char c : value) {
    if (c == '\\' || c == '"') out.push_back('\\');
    out.push_back(c);
  }
  return out;
}
}  // namespace

void FastExplorationFSM::init(ros::NodeHandle& nh, const MapInterface::Ptr& map) 
{
  md_ = std::make_shared<MissionData>();
  fp_ = std::make_shared<FSMParam>();
  fd_ = std::make_shared<FSMData>();
  node_ = nh;
  /*  Fsm param  */
  nh.param("fsm/drone_id", md_->drone_id_, 0);
  md_->is_initialized_=false;//swarm info callback

  fd_->target_cmd_ = "None";
  fd_->prior_knowledge_ = "Toilet is derictly connected to living room";
  fd_->target_cmd_ = nh.param<std::string>("fsm/target_cmd",    "None");

  nh.param("fsm/thresh_replan1",             fp_->replan_dis_thresh_, -1.0);
  nh.param("fsm/thresh_replan2",             fp_->replan_thresh2_, -1.0);
  nh.param("fsm/thresh_replan3",             fp_->replan_thresh3_, -1.0);
  nh.param("fsm/replan_time",                fp_->replan_time_, -1.0);
  nh.param("fsm/arrive_dis_thr",             fp_->arrive_dis_thr_, 0.1);
  nh.param("fsm/battery_thr",                fp_->battery_thr_,  19.0);
  nh.param("fsm/realworld_experiment",       fp_->flag_realworld_exp_, true);
  nh.param("fsm/enable_area_prediction",     fp_->enable_area_prediction_, true);
  nh.param("fsm/auto_init_scene_graph",      fp_->auto_init_scene_graph_, true);
  nh.param("fsm/auto_init_delay_sec",        fp_->auto_init_delay_sec_, 2.0);
  nh.param("fsm/scene_graph_init_forward_dist", fp_->scene_graph_init_forward_dist_, 1.8);
  nh.param("fsm/frontier_update_dt",          fp_->frontier_update_dt_, 0.5);
  if (fp_->frontier_update_dt_ < 0.05) {
    ROS_WARN("[ExploreFSM] fsm/frontier_update_dt is too small, clamp to 0.05s.");
    fp_->frontier_update_dt_ = 0.05;
  }
  nh.param("fsm/enable_yaw_scan", enable_yaw_scan_, false);
  nh.param("fsm/enable_scene_graph_update_after_load", enable_scene_graph_update_after_load_, true);
  double panorama_max_step_deg = 120.0;
  double panorama_extend_angle_deg = 40.0;
  nh.param("fsm/panorama_max_step", panorama_max_step_deg, 120.0);
  nh.param("fsm/panorama_extend_angle", panorama_extend_angle_deg, 40.0);
  panorama_max_step_ = std::max(1.0, panorama_max_step_deg) * M_PI / 180.0;
  panorama_extend_angle_ = std::max(0.0, panorama_extend_angle_deg) * M_PI / 180.0;
  if (panorama_extend_angle_ >= panorama_max_step_)
  {
    ROS_WARN("[Panorama] fsm/panorama_extend_angle must be smaller than panorama_max_step, clamp it.");
    panorama_extend_angle_ = 0.5 * panorama_max_step_;
  }
  nh.param("tracking/finish_hold_time",       fp_->track_finish_hold_time_, 3.0);
  nh.param("tracking/finish_move_thresh",     fp_->track_finish_move_thresh_, 0.2);
  nh.param("tracking/finish_yaw_thresh",      fp_->track_finish_yaw_thresh_, 0.2);
  nh.param("tracking/backend",                 fp_->tracking_backend_, std::string("ego"));
  nh.param("tracking/target_odom_topic",       fp_->tracking_target_odom_topic_, std::string("/target_ekf_odom"));
  nh.param("tracking/target_3d_topic",         fp_->tracking_target_3d_topic_, std::string("/target_3d_pos"));
  nh.param("planner_cmd_mux/mode_topic",       fp_->planner_cmd_mux_mode_topic_, std::string("/planner_mux/mode"));
  nh.param("planner_cmd_mux/ego_mode",         fp_->planner_cmd_mux_ego_mode_, std::string("ego"));
  nh.param("planner_cmd_mux/elastic_mode",     fp_->planner_cmd_mux_elastic_mode_, std::string("elastic"));
  nh.param("elastic_tracker/trigger_topic",    fp_->elastic_tracker_trigger_topic_, std::string("/triger"));
  nh.param("elastic_tracker/finish_topic",     fp_->elastic_tracker_finish_topic_, std::string("/elastic_tracker/tracking_finish"));
  nh.param("elastic_tracker/stop_topic",       fp_->elastic_tracker_stop_topic_, std::string("/elastic_tracker/stop"));
  // 卡死强制推进参数
  nh.param("topo_block/stuck_force_advance_enable",          fp_->stuck_force_advance_enable_, true);
  nh.param("topo_block/stuck_force_advance_vel_thresh",      fp_->stuck_force_advance_vel_thresh_, 0.1);
  nh.param("topo_block/stuck_force_advance_yaw_rate_thresh", fp_->stuck_force_advance_yaw_rate_thresh_, 0.1);
  nh.param("topo_block/stuck_force_advance_duration",        fp_->stuck_force_advance_duration_, 3.0);
  nh.param("topo_block/stuck_force_advance_max_consecutive", fp_->stuck_force_advance_max_consecutive_, 2);
  nh.param("frontier/max_observation_attempts", fp_->max_observation_attempts_, 3);

  // === 探索脱困参数 (建议 A/B/C/D/E) ===
  // 默认值与 FSMParam 成员初值一致, 通过 rosparam explore_stuck/* 覆盖
  nh.param("explore_stuck/local_aim_fail_max",            fp_->explore_local_aim_fail_max_, 5);
  nh.param("explore_stuck/force_advance_enable",          fp_->explore_stuck_force_advance_enable_, true);
  nh.param("explore_stuck/force_advance_duration",        fp_->explore_stuck_force_advance_duration_, 3.0);
  nh.param("explore_stuck/force_advance_max_consecutive", fp_->explore_stuck_force_advance_max_consecutive_, 2);
  nh.param("explore_stuck/local_stuck_vel_thresh",        fp_->explore_local_stuck_vel_thresh_, 0.1);
  nh.param("explore_stuck/local_stuck_duration",          fp_->explore_local_stuck_duration_, 8.0);
  // object-id-nav replan 参数
  nh.param("object_id_nav_replan/enable",               fp_->object_id_nav_replan_enable_, false);
  nh.param("object_id_nav_replan/mode",                 fp_->object_id_nav_replan_mode_, 0);
  nh.param("object_id_nav_replan/stuck_vel_thresh",     fp_->object_id_nav_replan_stuck_vel_thresh_, 0.1);
  nh.param("object_id_nav_replan/stuck_yaw_rate_thresh",fp_->object_id_nav_replan_stuck_yaw_rate_thresh_, 0.1);
  nh.param("object_id_nav_replan/stuck_duration",       fp_->object_id_nav_replan_stuck_duration_, 3.0);
  nh.param("object_id_nav_replan/stuck_max_consecutive", fp_->object_id_nav_replan_stuck_max_consecutive_, 0);
  nh.param("object_id_nav_replan/mode2_stuck_fallback_delay", fp_->object_id_nav_replan_mode2_stuck_fallback_delay_, 10.0);
  nh.param("object_id_nav/require_final_yaw",             fp_->object_id_nav_require_final_yaw_, true);
  nh.param("object_id_nav/use_thinking",                 fp_->object_id_nav_use_thinking_, true);
  nh.param("object_id_nav/prior_guide_max_retries",      fp_->object_id_nav_prior_guide_max_retries_, 2);
  nh.param("vla_swarm/enable",                  vla_swarm_enabled_, false);
  nh.param("vla_swarm/result_topic",            vla_swarm_result_topic_, std::string("/planning/vla_swarm_result"));
  nh.param("vla_swarm/bbox_topic",              vla_swarm_bbox_topic_, std::string("/vla_swarm/bbox"));
  nh.param("vla_swarm/target_topic",            vla_swarm_target_topic_, std::string("/vla_swarm/target"));
  nh.param(
      "vla_swarm/camera_topic", vla_swarm_camera_topic_,
      std::string("/drone_") + std::to_string(md_->drone_id_) +
          "/camera/color/image/compressed");
  nh.param(
      "vla_swarm/observation_topic", vla_swarm_observation_topic_,
      std::string("/vla_swarm/observation"));
  nh.param("vla_swarm/prompt_timeout",          vla_swarm_prompt_timeout_, 20.0);
  nh.param("vla_swarm/target_timeout",          vla_swarm_target_timeout_, 10.0);
  nh.param("vla_swarm/ego_plan_timeout",        vla_swarm_ego_plan_timeout_, 5.0);
  nh.param("vla_swarm/ego_exec_timeout",        vla_swarm_ego_exec_timeout_, 30.0);
  nh.param("vla_swarm/max_plan_retries",        vla_swarm_max_plan_retries_, 2);
  nh.param("vla_swarm/max_target_retries",      vla_swarm_max_target_retries_, 2);
  nh.param("vla_swarm/max_exploration_rounds",  vla_swarm_max_exploration_rounds_, 6);
  nh.param("vla_swarm/enable_room_description", vla_swarm_enable_room_description_, false);
  nh.param("vla_swarm/waypoint_distance",       vla_swarm_waypoint_distance_, 2.0);
  nh.param("vla_swarm/goal_tolerance",          vla_swarm_goal_tolerance_, 0.5);
  nh.param("vla_swarm/flight_height",           vla_swarm_flight_height_, 1.0);
  nh.param("vla_swarm/map_update_period",        vla_swarm_map_update_period_, 1.0);
  nh.param(
      "vla_swarm/scan_yaw_tolerance", vla_swarm_scan_yaw_tolerance_, 0.08);
  nh.param(
      "vla_swarm/scan_settle_time", vla_swarm_scan_settle_time_, 0.4);
  nh.param(
      "vla_swarm/scan_timeout", vla_swarm_scan_timeout_, 8.0);
  // 优先使用显式角度列表；若未设置则根据扫描步长自动生成。
  std::vector<double> scan_yaw_offsets_deg;
  nh.getParam("vla_swarm/scan_yaw_offsets_deg", scan_yaw_offsets_deg);
  nh.param("vla_swarm/scan_yaw_step_deg", vla_swarm_scan_yaw_step_deg_, 90.0);
  vla_swarm_scan_yaw_step_deg_ = std::max(1.0, std::min(180.0, vla_swarm_scan_yaw_step_deg_));

  if (scan_yaw_offsets_deg.empty()) {
    // 先直行方向(0)，左转一次(-step)，然后持续右转累加直至覆盖近360°，最后归零。
    scan_yaw_offsets_deg.push_back(0.0);
    scan_yaw_offsets_deg.push_back(-vla_swarm_scan_yaw_step_deg_);
    double angle = vla_swarm_scan_yaw_step_deg_;
    while (angle < 360.0 - 1e-6) {
      scan_yaw_offsets_deg.push_back(angle);
      angle += vla_swarm_scan_yaw_step_deg_;
    }
    scan_yaw_offsets_deg.push_back(0.0);
  }

  vla_swarm_scan_yaw_offsets_.clear();
  for (const double offset_deg : scan_yaw_offsets_deg) {
    vla_swarm_scan_yaw_offsets_.push_back(offset_deg * M_PI / 180.0);
  }
  if (vla_swarm_scan_yaw_offsets_.empty()) {
    vla_swarm_scan_yaw_offsets_.push_back(0.0);
  } else if (vla_swarm_scan_yaw_offsets_.size() > 24) {
    ROS_WARN(
        "[VLA_SWARM] scan_yaw_offsets_deg supports at most 24 observations; "
        "extra entries are ignored.");
    vla_swarm_scan_yaw_offsets_.resize(24);
  }
  vla_swarm_waypoint_distance_ = std::max(0.2, vla_swarm_waypoint_distance_);
  vla_swarm_goal_tolerance_ = std::max(0.1, vla_swarm_goal_tolerance_);
  vla_swarm_map_update_period_ = std::max(0.2, vla_swarm_map_update_period_);
  vla_swarm_scan_yaw_tolerance_ =
      std::max(0.01, vla_swarm_scan_yaw_tolerance_);
  vla_swarm_scan_settle_time_ =
      std::max(0.0, vla_swarm_scan_settle_time_);
  vla_swarm_scan_timeout_ = std::max(1.0, vla_swarm_scan_timeout_);

  std::cout << "\n***** Target Cmd : " << fd_->target_cmd_ << "\n" << std::endl;
  std::cout << "ALL Main FSM Params loaded successfully ..." << std::endl;

  // 性能日志初始化：每次启动新建带时间戳的日志文件，仅记录性能插桩输出
  // perf_log/enable 控制开关；perf_log/dir 指定日志目录，默认放到仓库根 logs/perf
  {
    bool perf_log_enable = true;
    std::string perf_log_dir;
    nh.param("perf_log/enable", perf_log_enable, true);
    nh.param("perf_log/dir", perf_log_dir, std::string(""));
    if (perf_log_dir.empty()) {
      // 默认目录：plan_env 包目录往上三级到仓库根，再进入 logs/perf
      perf_log_dir = ros::package::getPath("plan_env") + "/../../../logs/perf";
    }
    PERF_INIT(perf_log_dir, perf_log_enable);
    PERF_LOG("PERF_INIT", "dir=" + perf_log_dir + " enable=" + (perf_log_enable ? "1" : "0"));
  }

  // 探索耗时日志目录初始化：复用 perf_log 的目录解析逻辑，子目录改为 logs/exploration_timing
  // 文件固定为 exploration_timing.log（追加模式），每次探索任务完成时追加一条记录
  {
    nh.param("exploration_timing/dir", exploration_timing_dir_, std::string(""));
    if (exploration_timing_dir_.empty()) {
      exploration_timing_dir_ = ros::package::getPath("plan_env") + "/../../../logs/exploration_timing";
    }
    // 确保目录存在（mkdir -p，与 perf_logger 一致）
    std::string cmd = "mkdir -p " + exploration_timing_dir_;
    system(cmd.c_str());
  }


  fd_->home_pos_ << 0.0, 0.0, 1.0; // TODO
  fd_->ego_exec_finished_ = true;

  /* Initialize main modules */
  map_ = map;
  visualization_      = std::make_shared<PlanningVisualization>(nh);
  scene_graph_        = std::make_shared<SceneGraph>(nh, map_);
  vla_swarm_map_      = std::make_shared<VLASwarmMap>(nh, map_);
  counting_scene_graph_ = std::make_shared<CountingSceneGraph>(nh);
  expl_manager_       = std::make_shared<FrontierManager>(nh, map, scene_graph_);
  traj_visualizer_    = std::make_shared<TrajectoryVisualizer>(nh);

  scene_graph_->setTargetAndPriorKnowledge(fd_->target_cmd_, fd_->prior_knowledge_);

  md_->mission_state_ = MISSION_FSM_STATE::INIT;
  md_->is_leader_     = false;
  md_->is_follower_   = false;

  md_->state_str_[MISSION_FSM_STATE::INIT]              = "INIT";
  md_->state_str_[MISSION_FSM_STATE::WARM_UP]           = "WARM_UP";
  md_->state_str_[MISSION_FSM_STATE::WAIT_TRIGGER]      = "WAIT_TRIGGER";
  md_->state_str_[MISSION_FSM_STATE::PLAN_EXPLORE]      = "PLAN_REGULAR_EXPLORE";
  md_->state_str_[MISSION_FSM_STATE::LLM_PLAN_EXPLORE]  = "PLAN_LLM_EXPLORE";
  md_->state_str_[MISSION_FSM_STATE::PLAN_TRACK]        = "PLAN_TRACK";
  md_->state_str_[MISSION_FSM_STATE::APPROACH_TRACK]    = "APPROACH_TRACK";
  md_->state_str_[MISSION_FSM_STATE::THINKING]          = "THINKING";
  md_->state_str_[MISSION_FSM_STATE::YAW_HANDLE]        = "YAW_HANDLE";
  md_->state_str_[MISSION_FSM_STATE::APPROACH_EXPLORE]  = "APPROACH_EXPLORE";
  md_->state_str_[MISSION_FSM_STATE::STOP]              = "STOP";
  md_->state_str_[MISSION_FSM_STATE::UNKONWN]           = "UNKNOWN";
  md_->state_str_[MISSION_FSM_STATE::GO_TARGET_OBJECT]  = "GO_TARGET_OBJECT";
  md_->state_str_[MISSION_FSM_STATE::GO_TARGET_WITH_WAYPOINT] = "GO_TARGET_WITH_WAYPOINT";
  md_->state_str_[MISSION_FSM_STATE::FIND_TERMINATE_TARGET] = "FIND_TERMINATE_TARGET";
  md_->state_str_[MISSION_FSM_STATE::FINISH]            = "FINISH";
  md_->state_str_[MISSION_FSM_STATE::DF_DEMO]           = "DF_DEMO";
  md_->state_str_[MISSION_FSM_STATE::VLA_SWARM_PLAN_LOCAL] = "VLA_SWARM_PLAN_LOCAL";
  md_->state_str_[MISSION_FSM_STATE::VLA_SWARM_WAIT_LLM] = "VLA_SWARM_WAIT_LLM";
  md_->state_str_[MISSION_FSM_STATE::VLA_SWARM_WAIT_TARGET] = "VLA_SWARM_WAIT_TARGET";
  md_->state_str_[MISSION_FSM_STATE::VLA_SWARM_APPROACH] = "VLA_SWARM_APPROACH";
  md_->state_str_[MISSION_FSM_STATE::VLA_SWARM_YAW_HANDLE] = "VLA_SWARM_YAW_HANDLE";
  md_->state_str_[MISSION_FSM_STATE::VLA_SWARM_RECOVERY] = "VLA_SWARM_RECOVERY";
  md_->state_str_[MISSION_FSM_STATE::VLA_SWARM_FINISH] = "VLA_SWARM_FINISH";

  /* Initialize FSM data */
  fd_->have_odom_    = false;
  fd_->odom_pos_.setZero();
  fd_->odom_vel_.setZero();
  fd_->odom_yaw_ = 0.0;
  fd_->static_state_ = true;
  fd_->trigger_      = false;
  fd_->track_trigger_ = false;
  fd_->track_init_ = false;
  fd_->track_pos_.setZero();
  resetTrackingFinishCandidate();
  fd_->track_finish_sent_ = false;
  fd_->waypoint_target_.setZero();
  fd_->waypoint_target_yaw_ = 0.0;
  fd_->goal_replan_times_ = 0;
  fd_->warmup_start_time_ = ros::Time(0);

  /* Ros sub, pub and timer */
  exec_timer_      = nh.createTimer(ros::Duration(0.1), &FastExplorationFSM::FSMCallback, this);
  frontier_timer_  = nh.createTimer(ros::Duration(fp_->frontier_update_dt_), &FastExplorationFSM::frontierCallback, this);
  vla_swarm_map_timer_ = nh.createTimer(
      ros::Duration(vla_swarm_map_update_period_),
      &FastExplorationFSM::vlaSwarmMapCallback, this);

  // vis_timer_       = nh.createTimer(ros::Duration(0.2), &FastExplorationFSM::visualize, this); // [gwq] has thread problem! Don't turn on!

  instruction_sub_ = nh.subscribe("/bridge/Instruct", 10, &FastExplorationFSM::instructionCallback, this, ros::TransportHints().tcpNoDelay());
  odom_sub_        = nh.subscribe("odom_world",  1, &FastExplorationFSM::odometryCallback, this, ros::TransportHints().tcpNoDelay());
  battery_sub_     = nh.subscribe("/mavros/battery", 10, &FastExplorationFSM::batteryCallBack, this, ros::TransportHints().tcpNoDelay());
  ego_plan_res_sub_= nh.subscribe("/planning/ego_plan_result", 100, &FastExplorationFSM::egoPlanResCallback, this, ros::TransportHints().tcpNoDelay());
  trigger_sub_     = nh.subscribe("/move_base_simple/goal", 2, &FastExplorationFSM::triggerCallback, this, ros::TransportHints().tcpNoDelay());
  egoplanner_goal_sub_ = nh.subscribe("/goal_with_id_from_station", 2, &FastExplorationFSM::egoPlannerGoalCallback, this, ros::TransportHints().tcpNoDelay());
  ego_exec_finish_sub_ = nh.subscribe("exec_finish_trigger", 10, &FastExplorationFSM::egoExecFinishCallback, this, ros::TransportHints().tcpNoDelay());
  track_command_sub_ = nh.subscribe("/planning/track_command", 2, &FastExplorationFSM::trackCommandCallback, this,
                                    ros::TransportHints().tcpNoDelay());
  target_sub_ = nh.subscribe("/tracking_target", 2, &FastExplorationFSM::targetCallbackReal, this,
                             ros::TransportHints().tcpNoDelay());
  elastic_tracking_finish_sub_ = nh.subscribe(fp_->elastic_tracker_finish_topic_, 10,
                                              &FastExplorationFSM::elasticTrackingFinishCallback, this,
                                              ros::TransportHints().tcpNoDelay());
  emergency_stop_sub_ = nh.subscribe("/command/emergency_stop", 10,
                                     &FastExplorationFSM::emergencyStopCallback, this,
                                     ros::TransportHints().tcpNoDelay());
  vla_swarm_target_sub_ = nh.subscribe(
      vla_swarm_target_topic_, 10,
      &FastExplorationFSM::vlaSwarmTargetCallback, this,
      ros::TransportHints().tcpNoDelay());
  vla_swarm_camera_sub_ = nh.subscribe(
      vla_swarm_camera_topic_, 2,
      &FastExplorationFSM::vlaSwarmCameraCallback, this,
      ros::TransportHints().tcpNoDelay());
  vla_swarm_ego_state_trigger_sub_ = nh.subscribe(
      "/planning/ego_state_trigger", 10,
      &FastExplorationFSM::vlaSwarmEgoStateTriggerCallback, this,
      ros::TransportHints().tcpNoDelay());
  object_id_nav_replan_sub_ = nh.subscribe("/object_id_nav_replan", 10,
      &FastExplorationFSM::objectIdNavReplanCallback, this,
      ros::TransportHints().tcpNoDelay());

  ego_goal_pub_         = nh.advertise<quadrotor_msgs::EgoGoalSet>("local_goal", 10);
  goal_from_station_pub_ = nh.advertise<quadrotor_msgs::GoalSet>("/goal_with_id_from_station", 10);
  vis_marker_pub_       = nh.advertise<visualization_msgs::Marker>("planning/fsm_vis", 10);
  vis_path_pub_         = nh.advertise<visualization_msgs::MarkerArray>("planning/fsm_path", 10);
  perception_data_pub_  = nh.advertise<quadrotor_msgs::PerceptionMsg>("/perception_data_to_bridge", 10);
  instruction_resp_pub_ = nh.advertise<quadrotor_msgs::InstructionResMsg>("/Instruct_res", 10);

  fsm_state_pub_        = nh.advertise<std_msgs::String>("/planner/fsm_state", 10);
  tracking_finish_pub_  = nh.advertise<std_msgs::Bool>("/tracking_finish", 10);
  tracking_target_odom_pub_ = nh.advertise<nav_msgs::Odometry>(fp_->tracking_target_odom_topic_, 10);
  tracking_target_3d_pub_ = nh.advertise<geometry_msgs::PointStamped>(fp_->tracking_target_3d_topic_, 10);
  planner_cmd_mux_mode_pub_ = nh.advertise<std_msgs::String>(fp_->planner_cmd_mux_mode_topic_, 10, true);
  elastic_tracker_trigger_pub_ = nh.advertise<geometry_msgs::PoseStamped>(fp_->elastic_tracker_trigger_topic_, 10);
  elastic_tracker_stop_pub_ = nh.advertise<std_msgs::Empty>(fp_->elastic_tracker_stop_topic_, 10);
  exploration_result_pub_ = nh.advertise<std_msgs::String>("/planning/exploration_result", 10);
  vla_swarm_result_pub_ = nh.advertise<std_msgs::String>(vla_swarm_result_topic_, 10);
  vla_swarm_bbox_pub_ =
      nh.advertise<quadrotor_msgs::VLASwarmBBox>(vla_swarm_bbox_topic_, 10);
  vla_swarm_observation_pub_ =
      nh.advertise<scene_graph::VLASwarmObservation>(
          vla_swarm_observation_topic_, 10);

  switchPlannerCmdMuxToEgo("fsm_init");
}

void FastExplorationFSM::applyExplorationRegionFromInstruction(const quadrotor_msgs::InstructionConstPtr& msg)
{
  std::vector<Eigen::Vector3d> polygon;
  if (msg->has_exploration_region && msg->exploration_region.size() >= 3) {
    polygon.reserve(msg->exploration_region.size());
    for (const auto& point : msg->exploration_region) {
      polygon.emplace_back(point.x, point.y, point.z);
    }
    expl_manager_->setExplorationRegion(polygon, true);
    return;
  }
  expl_manager_->setExplorationRegion(polygon, false);
}

void FastExplorationFSM::publishExplorationResult(bool success, const std::string& reason,
                                                  const std::string& message)
{
  // Counting 专用对象图必须先冻结并发布，确保上层收到 exploration finished 时
  // 对应 session 的 JSON 已经进入 ROS 发布队列。
  if (counting_scene_graph_ != nullptr && counting_scene_graph_->active()) {
    counting_scene_graph_->finishSessionAndPublish();
  }

  // 探索任务计时结算：若计时处于激活状态，计算总耗时并写入本地文件
  if (exploration_timer_active_) {
    logExplorationTiming(success, reason, message);
    exploration_timer_active_ = false;
  }

  std_msgs::String msg;
  std::ostringstream ss;
  ss << "{"
     << "\"finished\":true,"
     << "\"success\":" << (success ? "true" : "false") << ","
     << "\"reason\":\"" << jsonEscape(reason) << "\","
     << "\"message\":\"" << jsonEscape(message) << "\","
     << "\"instruction_type\":" << static_cast<int>(md_->instruction_) << ","
     << "\"task_session_id\":" << active_instruction_session_id_ << ","
     << "\"command\":\"" << jsonEscape(fd_->target_cmd_) << "\","
     << "\"has_region\":" << (expl_manager_->hasExplorationRegion() ? "true" : "false") << ","
     << "\"state\":\"" << md_->state_str_[md_->mission_state_] << "\""
     << "}";
  msg.data = ss.str();
  exploration_result_pub_.publish(msg);
}

void FastExplorationFSM::logExplorationTiming(bool success, const std::string& reason,
                                              const std::string& message)
{
  // 计算探索总耗时（秒）
  ros::Time end_time = ros::Time::now();
  double elapsed_sec = (end_time - exploration_start_time_).toSec();

  // 写入本地文件（追加模式），文件固定为 exploration_timing.log
  // 每条记录一行，包含时间戳、成功/失败、原因、耗时、session_id、是否有区域
  std::string file_path = exploration_timing_dir_ + "/exploration_timing.log";
  std::ofstream ofs(file_path, std::ios::out | std::ios::app);
  if (!ofs.is_open()) {
    ROS_WARN("[ExplorationTiming] failed to open %s for writing", file_path.c_str());
    return;
  }

  // 写入 header（仅文件首次创建/为空时）
  ofs.seekp(0, std::ios::end);
  if (ofs.tellp() == 0) {
    ofs << "# exploration_timing.log: 每次探索任务的总耗时记录（追加模式）" << std::endl;
    ofs << "# 格式: [结束时间] success=1/0 reason=xxx elapsed_sec=xxx session_id=xxx has_region=0/1 start_time=xxx end_time=xxx" << std::endl;
  }

  // 格式化时间戳
  std::time_t raw_time = static_cast<std::time_t>(end_time.toSec());
  std::tm* tm_info = std::localtime(&raw_time);
  char time_buf[32];
  std::strftime(time_buf, sizeof(time_buf), "%Y-%m-%d %H:%M:%S", tm_info);

  ofs << "[" << time_buf << "] "
      << "success=" << (success ? 1 : 0) << " "
      << "reason=\"" << reason << "\" "
      << "elapsed_sec=" << std::fixed << std::setprecision(3) << elapsed_sec << " "
      << "session_id=" << active_instruction_session_id_ << " "
      << "has_region=" << (expl_manager_->hasExplorationRegion() ? 1 : 0) << " "
      << "start_time=" << std::fixed << std::setprecision(3) << exploration_start_time_.toSec() << " "
      << "end_time=" << std::fixed << std::setprecision(3) << end_time.toSec() << " "
      << "message=\"" << message << "\""
      << std::endl;
  ofs.flush();
  ofs.close();

  ROS_INFO("[ExplorationTiming] elapsed=%.3fs success=%d reason=%s logged to %s",
           elapsed_sec, success ? 1 : 0, reason.c_str(), file_path.c_str());
}

bool FastExplorationFSM::isVlaSwarmState(MISSION_FSM_STATE state) const
{
  return state == MISSION_FSM_STATE::VLA_SWARM_PLAN_LOCAL ||
         state == MISSION_FSM_STATE::VLA_SWARM_WAIT_LLM ||
         state == MISSION_FSM_STATE::VLA_SWARM_WAIT_TARGET ||
         state == MISSION_FSM_STATE::VLA_SWARM_APPROACH ||
         state == MISSION_FSM_STATE::VLA_SWARM_YAW_HANDLE ||
         state == MISSION_FSM_STATE::VLA_SWARM_RECOVERY ||
         state == MISSION_FSM_STATE::VLA_SWARM_FINISH;
}

void FastExplorationFSM::resetVlaSwarmContext()
{
  if (vla_swarm_prompt_pending_ && scene_graph_ != nullptr) {
    scene_graph_->clearPromptData(vla_swarm_prompt_id_);
  }
  vla_swarm_active_ = false;
  vla_swarm_result_published_ = false;
  vla_swarm_success_ = false;
  vla_swarm_session_id_ = 0;
  vla_swarm_command_.clear();
  vla_swarm_finish_reason_.clear();
  vla_swarm_finish_detail_.clear();
  vla_swarm_prompt_pending_ = false;
  vla_swarm_place_checked_ = false;
  vla_swarm_explore_area_id_ = -1;
  vla_swarm_prompt_id_ = 0;
  vla_swarm_prompt_type_ = 0;
  vla_swarm_observation_batch_id_ = 0;
  vla_swarm_target_request_id_ = 0;
  vla_swarm_prompt_start_time_ = ros::Time();
  vla_swarm_target_start_time_ = ros::Time();
  vla_swarm_observation_stamp_ = ros::Time();
  vla_swarm_scan_index_ = 0;
  vla_swarm_scan_base_yaw_ = 0.0;
  vla_swarm_scan_target_yaw_ = 0.0;
  vla_swarm_scan_hold_position_.setZero();
  vla_swarm_scan_command_time_ = ros::Time();
  vla_swarm_scan_yaw_reached_time_ = ros::Time();
  vla_swarm_scan_initialized_ = false;
  vla_swarm_scan_command_published_ = false;
  vla_swarm_target_pending_ = false;
  vla_swarm_target_received_ = false;
  vla_swarm_target_success_ = false;
  vla_swarm_target_observation_index_ = 0;
  vla_swarm_target_source_ = quadrotor_msgs::VLASwarmTarget::SOURCE_UNKNOWN;
  vla_swarm_target_position_.setZero();
  vla_swarm_target_error_.clear();
  vla_swarm_path_.clear();
  vla_swarm_waypoint_publish_time_ = ros::Time();
  vla_swarm_path_reaches_task_target_ = false;
  vla_swarm_waypoint_published_ = false;
  vla_swarm_waypoint_is_final_ = false;
  vla_swarm_plan_feedback_received_ = false;
  vla_swarm_plan_feedback_success_ = false;
  vla_swarm_waypoint_retry_count_ = 0;
  vla_swarm_ego_stable_ = false;
  vla_swarm_exploration_round_ = 0;
  vla_swarm_aa_done_ = false;
  vla_swarm_key_action_history_.clear();
  // room_descriptions_ 不在此清理 —— 同一 session 内跨轮复用
}

void FastExplorationFSM::publishVlaSwarmResult(bool success, const std::string& reason,
                                               const std::string& detail)
{
  if (!vla_swarm_active_ || vla_swarm_result_published_) {
    return;
  }

  std_msgs::String msg;
  std::ostringstream ss;
  ss << "{"
     << "\"task_session_id\":" << vla_swarm_session_id_ << ","
     << "\"finished\":true,"
     << "\"success\":" << (success ? "true" : "false") << ","
     << "\"reason\":\"" << jsonEscape(reason) << "\","
     << "\"detail\":\"" << jsonEscape(detail) << "\","
     << "\"command\":\"" << jsonEscape(vla_swarm_command_) << "\","
     << "\"state\":\"" << md_->state_str_[md_->mission_state_] << "\""
     << "}";
  msg.data = ss.str();
  vla_swarm_result_pub_.publish(msg);
  vla_swarm_result_published_ = true;
}

void FastExplorationFSM::startVlaSwarmTask(const quadrotor_msgs::InstructionConstPtr& msg)
{
  resetVlaSwarmContext();
  vla_swarm_room_descriptions_.clear();
  vla_swarm_active_ = true;
  vla_swarm_session_id_ = msg->task_session_id;
  vla_swarm_observation_batch_id_ = 1;
  vla_swarm_command_ = msg->command;
  active_instruction_task_id_ = msg->source_task_id;
  active_instruction_session_id_ = msg->task_session_id;
  fd_->target_cmd_ = msg->command;

  switchPlannerCmdMuxToEgo("startVlaSwarmTask");
  stopElasticTracker("startVlaSwarmTask");
  expl_manager_->setExplorationRegion(std::vector<Eigen::Vector3d>(), false);
  stopMotion();
  transitState(MISSION_FSM_STATE::VLA_SWARM_PLAN_LOCAL, "startVlaSwarmTask");
}

bool FastExplorationFSM::startVlaSwarmTargetRequest(
    const nlohmann::json& payload)
{
  if (!payload.contains("bounding_box") ||
      !payload["bounding_box"].is_array() ||
      payload["bounding_box"].size() != 4) {
    vla_swarm_finish_detail_ =
        "Visual target prompt requires bounding_box=[x0,y0,x1,y1]";
    return false;
  }

  int observation_index = 0;
  switch (vla_swarm_prompt_type_) {
    case scene_graph::PromptMsg::PROMPT_TYPE_LOCAL_PLAN_PREDICTION_A1:
    case scene_graph::PromptMsg::PROMPT_TYPE_LOCAL_PLAN_PREDICTION_B1:
      observation_index = 1;
      break;
    case scene_graph::PromptMsg::PROMPT_TYPE_LOCAL_PLAN_PREDICTION_A2:
    case scene_graph::PromptMsg::PROMPT_TYPE_LOCAL_PLAN_PREDICTION_B2:
      observation_index = 2;
      break;
    case scene_graph::PromptMsg::PROMPT_TYPE_LOCAL_PLAN_PREDICTION_A3:
    case scene_graph::PromptMsg::PROMPT_TYPE_LOCAL_PLAN_PREDICTION_B3:
      observation_index = 3;
      break;
    default:
      if (payload.contains("observation_index") &&
          payload["observation_index"].is_number_integer()) {
        observation_index = payload["observation_index"].get<int>();
      }
      break;
  }
  if (observation_index < 0 || observation_index > 3) {
    vla_swarm_finish_detail_ = "observation_index must be in [0,3]";
    return false;
  }

  quadrotor_msgs::VLASwarmBBox request;
  request.header.stamp =
      vla_swarm_observation_stamp_.isZero()
          ? vla_swarm_prompt_start_time_
          : vla_swarm_observation_stamp_;
  request.header.frame_id = "world";
  request.task_session_id = vla_swarm_session_id_;
  request.observation_batch_id = vla_swarm_observation_batch_id_;
  request.request_id = ++vla_swarm_target_request_id_;
  request.observation_index = static_cast<uint8_t>(observation_index);
  try {
    for (size_t index = 0; index < 4; ++index) {
      request.bbox_xyxy[index] =
          payload["bounding_box"][index].get<int>();
    }
  } catch (const std::exception& error) {
    vla_swarm_finish_detail_ = error.what();
    return false;
  }
  if (request.bbox_xyxy[2] <= request.bbox_xyxy[0] ||
      request.bbox_xyxy[3] <= request.bbox_xyxy[1]) {
    vla_swarm_finish_detail_ = "bounding_box has non-positive width or height";
    return false;
  }

  vla_swarm_target_observation_index_ = request.observation_index;
  vla_swarm_target_pending_ = true;
  vla_swarm_target_received_ = false;
  vla_swarm_target_success_ = false;
  vla_swarm_target_error_.clear();
  vla_swarm_target_start_time_ = ros::Time::now();
  vla_swarm_bbox_pub_.publish(request);
  transitState(
      MISSION_FSM_STATE::VLA_SWARM_WAIT_TARGET,
      "VLA_Swarm bbox target request sent");
  return true;
}

bool FastExplorationFSM::prepareVlaSwarmPath(
    const Eigen::Vector3d& requested_goal, bool reaches_task_target,
    int door_id)
{
  if (!requested_goal.allFinite()) {
    vla_swarm_finish_reason_ = "invalid_navigation_goal";
    vla_swarm_finish_detail_ = "Navigation goal contains non-finite values";
    return false;
  }

  Eigen::Vector3d navigation_goal = requested_goal;
  std::vector<Eigen::Vector3d> raw_path;
  if (door_id >= 0) {
    navigation_goal.z() = vla_swarm_flight_height_;
    if (vla_swarm_map_ == nullptr ||
        !vla_swarm_map_->planDoorPath(
            fd_->odom_pos_, door_id, navigation_goal.z(),
            vla_swarm_waypoint_distance_, raw_path)) {
      vla_swarm_finish_reason_ = "small_map_path_unreachable";
      vla_swarm_finish_detail_ =
          "SmallMap A* cannot reach door id=" + std::to_string(door_id);
      return false;
    }

    // SmallMap 负责生成二维门路径，现有三维 A* 负责确认终点确实可由飞行空间到达。
    std::vector<Eigen::Vector3d> verification_path;
    if (!map_->searchPath(
            fd_->odom_pos_, navigation_goal, verification_path, 0.2)) {
      vla_swarm_finish_reason_ = "door_path_unreachable_3d";
      vla_swarm_finish_detail_ =
          "3D occupancy map rejects door id=" + std::to_string(door_id);
      return false;
    }
  } else {
    // 视觉或场景图目标可能位于物体占据栅格内，沿机器人方向寻找最近可执行停靠点。
    Eigen::Vector3d direction_to_robot = Eigen::Vector3d::UnitX();
    if ((fd_->odom_pos_ - requested_goal).norm() > 1e-6) {
      direction_to_robot =
          (fd_->odom_pos_ - requested_goal).normalized();
    }
    bool goal_is_free = false;
    for (double offset = 0.0; offset <= 2.0; offset += 0.2) {
      Eigen::Vector3d candidate =
          requested_goal + direction_to_robot * offset;
      if (map_->getInflateOccupancy(candidate) != MapInterface::OCCUPIED &&
          map_->getOccupancy(candidate) != MapInterface::UNKNOWN) {
        navigation_goal = candidate;
        goal_is_free = true;
        break;
      }
    }
    if (!goal_is_free ||
        !map_->searchPath(
            fd_->odom_pos_, navigation_goal, raw_path, 0.2)) {
      vla_swarm_finish_reason_ = "target_path_unreachable";
      vla_swarm_finish_detail_ =
          "No collision-free 3D path exists for the selected target";
      return false;
    }
  }

  if (raw_path.size() < 2) {
    vla_swarm_finish_reason_ = "path_generation_failed";
    vla_swarm_finish_detail_ =
        "Path generator returned fewer than two points";
    return false;
  }

  // 对三维 A* 的密集输出再次按配置距离采样；SmallMap 路径已采样，但也通过
  // 同一逻辑确保不同来源的 waypoint 间距一致。
  std::vector<Eigen::Vector3d> sampled_path;
  sampled_path.push_back(raw_path.front());
  double distance_since_last_sample = 0.0;
  for (size_t index = 1; index < raw_path.size(); ++index) {
    Eigen::Vector3d segment_start = raw_path[index - 1];
    const Eigen::Vector3d segment_end = raw_path[index];
    Eigen::Vector3d segment = segment_end - segment_start;
    double segment_length = segment.norm();
    while (segment_length > 1e-6 &&
           distance_since_last_sample + segment_length >=
               vla_swarm_waypoint_distance_) {
      const double step =
          vla_swarm_waypoint_distance_ - distance_since_last_sample;
      segment_start += segment * (step / segment_length);
      sampled_path.push_back(segment_start);
      segment = segment_end - segment_start;
      segment_length = segment.norm();
      distance_since_last_sample = 0.0;
    }
    distance_since_last_sample += segment_length;
  }
  if ((sampled_path.back() - raw_path.back()).norm() > 1e-3) {
    sampled_path.push_back(raw_path.back());
  }

  vla_swarm_path_ = std::move(sampled_path);
  vla_swarm_path_reaches_task_target_ = reaches_task_target;
  vla_swarm_waypoint_published_ = false;
  vla_swarm_waypoint_is_final_ = false;
  vla_swarm_plan_feedback_received_ = false;
  vla_swarm_plan_feedback_success_ = false;
  vla_swarm_waypoint_retry_count_ = 0;
  fd_->path_inx_ = 0;
  vla_swarm_finish_reason_.clear();
  vla_swarm_finish_detail_.clear();
  transitState(
      MISSION_FSM_STATE::VLA_SWARM_APPROACH,
      door_id >= 0 ? "VLA_Swarm door path ready"
                   : "VLA_Swarm target path ready");
  return true;
}

bool FastExplorationFSM::publishNextVlaSwarmWaypoint()
{
  if (vla_swarm_path_.size() < 2) {
    return false;
  }

  vla_swarm_plan_feedback_received_ = false;
  vla_swarm_plan_feedback_success_ = false;
  vla_swarm_waypoint_retry_count_ = 0;
  if (!getAndPublishNextAim(vla_swarm_path_, true, fd_->odom_yaw_)) {
    return false;
  }
  vla_swarm_waypoint_published_ = true;
  vla_swarm_waypoint_is_final_ =
      vla_swarm_path_.size() <= 2 ||
      fd_->path_inx_ >= static_cast<int>(vla_swarm_path_.size()) - 1;
  vla_swarm_waypoint_publish_time_ = ros::Time::now();
  return true;
}

void FastExplorationFSM::retryVlaSwarmWaypoint(
    const std::string& failure_reason)
{
  if (vla_swarm_waypoint_retry_count_ >=
      std::max(0, vla_swarm_max_plan_retries_)) {
    vla_swarm_finish_reason_ = failure_reason;
    vla_swarm_finish_detail_ =
        "EGO failed waypoint after " +
        std::to_string(vla_swarm_waypoint_retry_count_) + " retries";
    transitState(
        MISSION_FSM_STATE::VLA_SWARM_RECOVERY,
        "VLA_Swarm EGO retry exhausted");
    return;
  }

  ++vla_swarm_waypoint_retry_count_;
  vla_swarm_plan_feedback_received_ = false;
  vla_swarm_plan_feedback_success_ = false;
  pubLocalGoal(fd_->local_aim_pos_, fd_->odom_yaw_, true);
  vla_swarm_waypoint_publish_time_ = ros::Time::now();
  ROS_WARN_STREAM(
      "[VLA_SWARM] Retry waypoint " << vla_swarm_waypoint_retry_count_
      << "/" << vla_swarm_max_plan_retries_
      << ", reason=" << failure_reason);
}

void FastExplorationFSM::cancelVlaSwarmTask(const std::string& reason, const std::string& detail)
{
  if (!vla_swarm_active_) {
    return;
  }
  stopMotion();
  vla_swarm_success_ = false;
  vla_swarm_finish_reason_ = reason;
  vla_swarm_finish_detail_ = detail;
  transitState(MISSION_FSM_STATE::VLA_SWARM_FINISH, "cancelVlaSwarmTask");
  handleVlaSwarmFinish();
}

void FastExplorationFSM::handleVlaSwarmPlanLocal()
{
  if (!vla_swarm_active_) {
    transitState(MISSION_FSM_STATE::WAIT_TRIGGER, "VLA_Swarm inactive");
    return;
  }

  if (vla_swarm_prompt_pending_) {
    transitState(MISSION_FSM_STATE::VLA_SWARM_WAIT_LLM, "VLA_Swarm prompt already pending");
    return;
  }

  // AA 阶段：全局评估，参照原始 VLA_Swarm 的 AA→A→B→C→TASK_OVER 链路。
  // 在每轮 PLACE 之前询问 LLM 是否继续探索、有无新发现。
  if (!vla_swarm_aa_done_) {
    nlohmann::json aa_context;
    aa_context["exploration_round"] = vla_swarm_exploration_round_;
    aa_context["key_action_history"] = vla_swarm_key_action_history_;
    aa_context["room_descriptions"] = vla_swarm_room_descriptions_;

    vla_swarm_prompt_type_ =
        scene_graph::PromptMsg::PROMPT_TYPE_LOCAL_PLAN_PREDICTION_AA;
    vla_swarm_prompt_id_ = scene_graph_->getCurPromptIdAndPlusOne();
    std::string prompt;
    if (!scene_graph_->vlaSwarmPromptGen(
            vla_swarm_prompt_type_, vla_swarm_command_,
            vla_swarm_session_id_, vla_swarm_observation_batch_id_,
            aa_context, prompt)) {
      vla_swarm_finish_reason_ = "aa_prompt_generation_failed";
      vla_swarm_finish_detail_ = "Failed to generate AA prompt";
      transitState(MISSION_FSM_STATE::VLA_SWARM_RECOVERY,
                   "VLA_Swarm AA prompt generation failed");
      return;
    }
    const int aa_timeout =
        std::max(1, static_cast<int>(std::ceil(vla_swarm_prompt_timeout_)));
    scene_graph_->sendPrompt(vla_swarm_prompt_id_, vla_swarm_prompt_type_,
                             prompt, std::chrono::seconds(aa_timeout), 1);
    vla_swarm_prompt_pending_ = true;
    vla_swarm_prompt_start_time_ = ros::Time::now();
    vla_swarm_aa_done_ = true;
    transitState(MISSION_FSM_STATE::VLA_SWARM_WAIT_LLM,
                 "VLA_Swarm AA prompt sent");
    return;
  }

  if (vla_swarm_map_ == nullptr ||
      (!vla_swarm_map_->ready() && !vla_swarm_map_->update(fd_->odom_pos_))) {
    vla_swarm_finish_reason_ = "small_map_not_ready";
    vla_swarm_finish_detail_ = "SmallMap cannot be generated from the current occupancy map";
    transitState(MISSION_FSM_STATE::VLA_SWARM_RECOVERY, "VLA_Swarm SmallMap unavailable");
    return;
  }

  nlohmann::json semantic_context =
      vla_swarm_map_->promptContext(*scene_graph_, fd_->odom_pos_);
  // 注入跨轮记忆与房间描述，传递给 PLACE/LOCAL_PLAN prompt。
  semantic_context["key_action_history"] = vla_swarm_key_action_history_;
  semantic_context["exploration_round"] = vla_swarm_exploration_round_;
  if (!vla_swarm_room_descriptions_.empty()) {
    semantic_context["room_descriptions"] = vla_swarm_room_descriptions_;
  }

  vla_swarm_prompt_type_ =
      vla_swarm_place_checked_
          ? scene_graph::PromptMsg::PROMPT_TYPE_LOCAL_PLAN_PREDICTION
          : scene_graph::PromptMsg::PROMPT_TYPE_PLACE_PREDICTION;
  if (vla_swarm_place_checked_ &&
      semantic_context["candidate_ids"].empty()) {
    vla_swarm_finish_reason_ = "no_exploration_candidate";
    vla_swarm_finish_detail_ = "SmallMap contains no valid door or frontier candidate";
    transitState(MISSION_FSM_STATE::VLA_SWARM_RECOVERY, "VLA_Swarm has no map candidate");
    return;
  }

  vla_swarm_prompt_id_ = scene_graph_->getCurPromptIdAndPlusOne();
  std::string prompt;
  if (!scene_graph_->vlaSwarmPromptGen(
          vla_swarm_prompt_type_,
          vla_swarm_command_,
          vla_swarm_session_id_,
          vla_swarm_observation_batch_id_,
          semantic_context,
          prompt)) {
    vla_swarm_finish_reason_ = "prompt_generation_failed";
    vla_swarm_finish_detail_ = "Failed to generate VLA_Swarm prompt from SmallMap context";
    transitState(MISSION_FSM_STATE::VLA_SWARM_RECOVERY, "VLA_Swarm prompt generation failed");
    return;
  }

  const int timeout_seconds = std::max(1, static_cast<int>(std::ceil(vla_swarm_prompt_timeout_)));
  const int max_retries = std::max(1, vla_swarm_max_plan_retries_);
  scene_graph_->sendPrompt(
      vla_swarm_prompt_id_,
      vla_swarm_prompt_type_,
      prompt,
      std::chrono::seconds(timeout_seconds),
      max_retries);
  vla_swarm_prompt_pending_ = true;
  vla_swarm_prompt_start_time_ = ros::Time::now();
  vla_swarm_observation_stamp_ = vla_swarm_prompt_start_time_;
  transitState(
      MISSION_FSM_STATE::VLA_SWARM_WAIT_LLM,
      vla_swarm_place_checked_
          ? "VLA_Swarm LOCAL_PLAN prompt sent"
          : "VLA_Swarm PLACE prompt sent");
}

void FastExplorationFSM::handleVlaSwarmWaitLLM()
{
  if (!vla_swarm_active_ || !vla_swarm_prompt_pending_) {
    vla_swarm_finish_reason_ = "prompt_state_invalid";
    vla_swarm_finish_detail_ = "VLA_SWARM_WAIT_LLM has no active prompt";
    transitState(MISSION_FSM_STATE::VLA_SWARM_RECOVERY, "VLA_Swarm missing active prompt");
    return;
  }

  if (!scene_graph_->hasPromptAnswer(vla_swarm_prompt_id_)) {
    const double retry_window =
        std::max(1.0, vla_swarm_prompt_timeout_) * std::max(1, vla_swarm_max_plan_retries_) + 1.0;
    if ((ros::Time::now() - vla_swarm_prompt_start_time_).toSec() <= retry_window) {
      return;
    }
    scene_graph_->clearPromptData(vla_swarm_prompt_id_);
    vla_swarm_prompt_pending_ = false;
    vla_swarm_finish_reason_ = "prompt_timeout";
    vla_swarm_finish_detail_ = "VLA_Swarm prompt did not return within the configured retry window";
    transitState(MISSION_FSM_STATE::VLA_SWARM_RECOVERY, "VLA_Swarm prompt timeout");
    return;
  }

  const VLASwarmPromptResult result =
      scene_graph_->parseVlaSwarmPromptResult(vla_swarm_prompt_id_, vla_swarm_prompt_type_);
  scene_graph_->clearPromptData(vla_swarm_prompt_id_);
  vla_swarm_prompt_pending_ = false;

  if (!result.valid) {
    vla_swarm_finish_reason_ = result.error.empty() ? "invalid_prompt_result" : result.error;
    vla_swarm_finish_detail_ = result.detail;
    transitState(MISSION_FSM_STATE::VLA_SWARM_RECOVERY, "VLA_Swarm invalid prompt JSON");
    return;
  }
  if (!result.success) {
    if (result.error == "observation_not_ready") {
      // 图像快照尚未被处理端接收时，留在当前方向重新固化一帧。
      vla_swarm_scan_command_published_ = false;
      transitState(
          MISSION_FSM_STATE::VLA_SWARM_YAW_HANDLE,
          "VLA_Swarm observation cache not ready");
      return;
    }
    vla_swarm_finish_reason_ = result.error.empty() ? "prompt_error" : result.error;
    vla_swarm_finish_detail_ = result.detail;
    transitState(MISSION_FSM_STATE::VLA_SWARM_RECOVERY, "VLA_Swarm prompt processor error");
    return;
  }

  auto parseIntegerField = [&](const char* field_name, int& value) {
    if (!result.payload.contains(field_name)) {
      vla_swarm_finish_detail_ =
          std::string("Prompt answer requires field: ") + field_name;
      return false;
    }
    try {
      const auto& field = result.payload[field_name];
      if (field.is_number_integer()) {
        value = field.get<int>();
        return true;
      }
      if (field.is_string()) {
        const std::string raw_value = field.get<std::string>();
        size_t consumed = 0;
        value = std::stoi(raw_value, &consumed);
        if (consumed == raw_value.size()) {
          return true;
        }
      }
    } catch (const std::exception& e) {
      vla_swarm_finish_detail_ = e.what();
      return false;
    }
    vla_swarm_finish_detail_ =
        std::string(field_name) + " must be an integer or integer string";
    return false;
  };

  // AA 阶段响应路由：LLM 基于全局历史判断是否继续探索。
  if (vla_swarm_prompt_type_ ==
      scene_graph::PromptMsg::PROMPT_TYPE_LOCAL_PLAN_PREDICTION_AA) {
    bool aa_found = false;
    if (result.payload.contains("found") &&
        result.payload["found"].is_boolean()) {
      aa_found = result.payload["found"].get<bool>();
    }
    std::string aa_action = "continue";
    if (result.payload.contains("action") &&
        result.payload["action"].is_string()) {
      aa_action = result.payload["action"].get<std::string>();
    }

    if (aa_action == "stop") {
      vla_swarm_success_ = false;
      vla_swarm_finish_reason_ = "task_over_by_llm";
      vla_swarm_finish_detail_ = "AA stage: LLM decided exploration is complete";
      transitState(MISSION_FSM_STATE::VLA_SWARM_RECOVERY,
                   "VLA_Swarm AA says stop");
      return;
    }

    if (aa_found) {
      nlohmann::json entry;
      entry["round"] = vla_swarm_exploration_round_;
      entry["action"] = "AA: found potential target, continuing exploration";
      vla_swarm_key_action_history_.push_back(entry);
    }

    transitState(MISSION_FSM_STATE::VLA_SWARM_PLAN_LOCAL,
                 "VLA_Swarm AA complete, continue to PLACE");
    return;
  }

  if (vla_swarm_prompt_type_ ==
      scene_graph::PromptMsg::PROMPT_TYPE_PLACE_PREDICTION) {
    int action = 0;
    if (!parseIntegerField("action", action) ||
        (action != -1 && action != 1 && action != 2)) {
      vla_swarm_finish_reason_ = "invalid_prompt_schema";
      if (vla_swarm_finish_detail_.empty()) {
        vla_swarm_finish_detail_ = "PLACE action must be -1, 1 or 2";
      }
      transitState(MISSION_FSM_STATE::VLA_SWARM_RECOVERY, "VLA_Swarm invalid PLACE result");
      return;
    }

    if (action == -1) {
      // 当前语义没有直接目标时，转入 SmallMap 视觉局部规划。
      vla_swarm_place_checked_ = true;
      transitState(MISSION_FSM_STATE::VLA_SWARM_PLAN_LOCAL, "VLA_Swarm PLACE continues to LOCAL_PLAN");
      return;
    }

    int selected_id = -1;
    if (!parseIntegerField("id", selected_id)) {
      vla_swarm_finish_reason_ = "invalid_prompt_schema";
      transitState(MISSION_FSM_STATE::VLA_SWARM_RECOVERY, "VLA_Swarm invalid PLACE id");
      return;
    }

    Eigen::Vector3d selected_goal = Eigen::Vector3d::Zero();
    bool reaches_task_target = false;
    bool selected_goal_found = false;
    if (action == 1) {
      for (const auto& room : vla_swarm_map_->rooms()) {
        if (room.id == selected_id) {
          selected_goal =
              Eigen::Vector3d(
                  room.center.x(), room.center.y(),
                  vla_swarm_flight_height_);
          selected_goal_found = true;
          break;
        }
      }
    } else if (scene_graph_->object_factory_ != nullptr) {
      const auto object_iterator =
          scene_graph_->object_factory_->object_map_.find(selected_id);
      if (object_iterator !=
              scene_graph_->object_factory_->object_map_.end() &&
          object_iterator->second != nullptr) {
        selected_goal = object_iterator->second->pos;
        reaches_task_target = true;
        selected_goal_found = true;
      }
    }
    if (!selected_goal_found) {
      vla_swarm_finish_reason_ = "place_target_not_found";
      vla_swarm_finish_detail_ =
          "PLACE selected unknown id=" + std::to_string(selected_id);
      transitState(
          MISSION_FSM_STATE::VLA_SWARM_RECOVERY,
          "VLA_Swarm PLACE target missing");
      return;
    }
    if (!prepareVlaSwarmPath(
            selected_goal, reaches_task_target)) {
      transitState(
          MISSION_FSM_STATE::VLA_SWARM_RECOVERY,
          "VLA_Swarm PLACE target path failed");
    }
    return;
  }

  if (vla_swarm_prompt_type_ ==
      scene_graph::PromptMsg::PROMPT_TYPE_LOCAL_PLAN_PREDICTION) {
    int explore_area_id = -1;
    if (!parseIntegerField("explore_area_id", explore_area_id)) {
      vla_swarm_finish_reason_ = "invalid_prompt_schema";
      transitState(MISSION_FSM_STATE::VLA_SWARM_RECOVERY, "VLA_Swarm invalid LOCAL_PLAN result");
      return;
    }

    bool candidate_exists = false;
    for (const auto& door : vla_swarm_map_->doors()) {
      if (door.id == explore_area_id) {
        candidate_exists = true;
        break;
      }
    }
    if (!candidate_exists) {
      vla_swarm_finish_reason_ = "invalid_exploration_candidate";
      vla_swarm_finish_detail_ =
          "LOCAL_PLAN explore_area_id is not present in the current SmallMap";
      transitState(MISSION_FSM_STATE::VLA_SWARM_RECOVERY, "VLA_Swarm candidate not found");
      return;
    }

    vla_swarm_explore_area_id_ = explore_area_id;
    VLASwarmDoor selected_door;
    if (!vla_swarm_map_->findDoor(
            vla_swarm_explore_area_id_, selected_door)) {
      vla_swarm_finish_reason_ = "invalid_exploration_candidate";
      vla_swarm_finish_detail_ =
          "Selected door disappeared before path generation";
      transitState(
          MISSION_FSM_STATE::VLA_SWARM_RECOVERY,
          "VLA_Swarm selected door expired");
      return;
    }
    const Eigen::Vector3d door_goal(
        selected_door.position.x(), selected_door.position.y(),
        vla_swarm_flight_height_);
    if (!prepareVlaSwarmPath(
            door_goal, false, vla_swarm_explore_area_id_)) {
      transitState(
          MISSION_FSM_STATE::VLA_SWARM_RECOVERY,
          "VLA_Swarm door path failed");
    }
    return;
  }

  const bool is_visual_target_prompt =
      vla_swarm_prompt_type_ ==
          scene_graph::PromptMsg::PROMPT_TYPE_LOCAL_PLAN_PREDICTION_A ||
      vla_swarm_prompt_type_ ==
          scene_graph::PromptMsg::PROMPT_TYPE_LOCAL_PLAN_PREDICTION_A1 ||
      vla_swarm_prompt_type_ ==
          scene_graph::PromptMsg::PROMPT_TYPE_LOCAL_PLAN_PREDICTION_A2 ||
      vla_swarm_prompt_type_ ==
          scene_graph::PromptMsg::PROMPT_TYPE_LOCAL_PLAN_PREDICTION_A3 ||
      vla_swarm_prompt_type_ ==
          scene_graph::PromptMsg::PROMPT_TYPE_LOCAL_PLAN_PREDICTION_B ||
      vla_swarm_prompt_type_ ==
          scene_graph::PromptMsg::PROMPT_TYPE_LOCAL_PLAN_PREDICTION_B1 ||
      vla_swarm_prompt_type_ ==
          scene_graph::PromptMsg::PROMPT_TYPE_LOCAL_PLAN_PREDICTION_B2 ||
      vla_swarm_prompt_type_ ==
          scene_graph::PromptMsg::PROMPT_TYPE_LOCAL_PLAN_PREDICTION_B3;
  if (is_visual_target_prompt) {
    bool found = result.payload.contains("bounding_box");
    if (result.payload.contains("found") &&
        result.payload["found"].is_boolean()) {
      found = result.payload["found"].get<bool>();
    }
    if (!found) {
      ++vla_swarm_scan_index_;
      vla_swarm_scan_command_published_ = false;
      vla_swarm_scan_yaw_reached_time_ = ros::Time();
      transitState(
          MISSION_FSM_STATE::VLA_SWARM_YAW_HANDLE,
          "VLA_Swarm target absent in current observation");
      return;
    }
    if (!startVlaSwarmTargetRequest(result.payload)) {
      vla_swarm_finish_reason_ = "invalid_bbox_result";
      transitState(
          MISSION_FSM_STATE::VLA_SWARM_RECOVERY,
          "VLA_Swarm invalid visual target result");
    }
    return;
  }

  vla_swarm_finish_reason_ = "invalid_prompt_type";
  vla_swarm_finish_detail_ = "Unexpected VLA_Swarm prompt type in WAIT_LLM";
  transitState(MISSION_FSM_STATE::VLA_SWARM_RECOVERY, "VLA_Swarm unexpected prompt type");
}

void FastExplorationFSM::handleVlaSwarmWaitTarget()
{
  if (!vla_swarm_active_ || !vla_swarm_target_pending_) {
    vla_swarm_finish_reason_ = "target_state_invalid";
    vla_swarm_finish_detail_ = "VLA_SWARM_WAIT_TARGET has no active bbox request";
    transitState(
        MISSION_FSM_STATE::VLA_SWARM_RECOVERY,
        "VLA_Swarm target request missing");
    return;
  }

  if (!vla_swarm_target_received_) {
    if ((ros::Time::now() - vla_swarm_target_start_time_).toSec() <=
        std::max(1.0, vla_swarm_target_timeout_)) {
      return;
    }
    vla_swarm_target_pending_ = false;
    vla_swarm_finish_reason_ = "target_timeout";
    vla_swarm_finish_detail_ =
        "Neither LiDAR nor MoGe returned a target within target_timeout";
    transitState(
        MISSION_FSM_STATE::VLA_SWARM_RECOVERY,
        "VLA_Swarm target localization timeout");
    return;
  }

  vla_swarm_target_pending_ = false;
  if (!vla_swarm_target_success_) {
    vla_swarm_finish_reason_ = "target_localization_failed";
    vla_swarm_finish_detail_ = vla_swarm_target_error_;
    transitState(
        MISSION_FSM_STATE::VLA_SWARM_RECOVERY,
        "VLA_Swarm target localization failed");
    return;
  }

  if (!prepareVlaSwarmPath(
          vla_swarm_target_position_, true)) {
    transitState(
        MISSION_FSM_STATE::VLA_SWARM_RECOVERY,
        "VLA_Swarm localized target path failed");
  }
}

void FastExplorationFSM::handleVlaSwarmApproach()
{
  if (!vla_swarm_active_ || vla_swarm_path_.size() < 2) {
    vla_swarm_finish_reason_ = "approach_state_invalid";
    vla_swarm_finish_detail_ =
        "VLA_SWARM_APPROACH has no active path";
    transitState(
        MISSION_FSM_STATE::VLA_SWARM_RECOVERY,
        "VLA_Swarm approach path missing");
    return;
  }

  if (!vla_swarm_waypoint_published_) {
    // 等待 ego_state_trigger 确保无人机已停止运动后再发布导航指令。
    if (!vla_swarm_ego_stable_) {
      return;
    }
    vla_swarm_ego_stable_ = false;
    if (!publishNextVlaSwarmWaypoint()) {
      vla_swarm_finish_reason_ = "waypoint_generation_failed";
      vla_swarm_finish_detail_ =
          "No valid local waypoint can be selected from the path";
      transitState(
          MISSION_FSM_STATE::VLA_SWARM_RECOVERY,
          "VLA_Swarm waypoint unavailable");
    }
    return;
  }

  const double elapsed =
      (ros::Time::now() - vla_swarm_waypoint_publish_time_).toSec();
  if (!vla_swarm_plan_feedback_received_) {
    if (elapsed > std::max(0.5, vla_swarm_ego_plan_timeout_)) {
      retryVlaSwarmWaypoint("ego_plan_timeout");
    }
    return;
  }
  if (!vla_swarm_plan_feedback_success_) {
    retryVlaSwarmWaypoint("ego_plan_failed");
    return;
  }

  const double distance_to_waypoint =
      (fd_->odom_pos_ - fd_->local_aim_pos_).norm();
  const bool waypoint_reached =
      distance_to_waypoint <= vla_swarm_goal_tolerance_ ||
      (fd_->ego_exec_finished_ &&
       distance_to_waypoint <=
           std::max(0.75, 2.0 * vla_swarm_goal_tolerance_));
  if (!waypoint_reached) {
    if (elapsed > std::max(
                      vla_swarm_ego_plan_timeout_ + 0.5,
                      vla_swarm_ego_exec_timeout_)) {
      retryVlaSwarmWaypoint("ego_execution_timeout");
    }
    return;
  }

  if (!vla_swarm_waypoint_is_final_) {
    vla_swarm_waypoint_published_ = false;
    if (!publishNextVlaSwarmWaypoint()) {
      vla_swarm_finish_reason_ = "waypoint_advance_failed";
      vla_swarm_finish_detail_ =
          "Path ended before the final waypoint was reached";
      transitState(
          MISSION_FSM_STATE::VLA_SWARM_RECOVERY,
          "VLA_Swarm waypoint advance failed");
    }
    return;
  }

  if (vla_swarm_path_reaches_task_target_) {
    vla_swarm_success_ = true;
    vla_swarm_finish_reason_ = "target_reached";
    vla_swarm_finish_detail_ =
        "EGO completed the VLA_Swarm target path";
    transitState(
        MISSION_FSM_STATE::VLA_SWARM_FINISH,
        "VLA_Swarm target reached");
    return;
  }

  // 到达房间或门后，优先沿路径末端方向观察。对门路径而言，该方向通常指向门后新空间。
  vla_swarm_scan_base_yaw_ = fd_->odom_yaw_;
  if (vla_swarm_path_.size() >= 2) {
    const Eigen::Vector3d terminal_direction =
        vla_swarm_path_.back() -
        vla_swarm_path_[vla_swarm_path_.size() - 2];
    if (terminal_direction.head<2>().norm() > 1e-3) {
      vla_swarm_scan_base_yaw_ =
          std::atan2(terminal_direction.y(), terminal_direction.x());
    }
  }

  // 记录本轮到达的门/区域，作为 AA 阶段的跨轮记忆输入。
  {
    nlohmann::json entry;
    entry["round"] = vla_swarm_exploration_round_ + 1;
    entry["action"] =
        "Arrived at door " + std::to_string(vla_swarm_explore_area_id_);
    vla_swarm_key_action_history_.push_back(entry);
  }

  // 到达房间或门仅表示完成一轮局部探索，刷新批次并执行正面优先的分时观察。
  ++vla_swarm_observation_batch_id_;
  vla_swarm_place_checked_ = false;
  vla_swarm_path_.clear();
  vla_swarm_waypoint_published_ = false;
  vla_swarm_scan_initialized_ = false;
  transitState(
      MISSION_FSM_STATE::VLA_SWARM_YAW_HANDLE,
      "VLA_Swarm exploration waypoint reached, start visual scan");
}

void FastExplorationFSM::handleVlaSwarmYaw()
{
  if (!vla_swarm_active_ || vla_swarm_scan_yaw_offsets_.empty()) {
    vla_swarm_finish_reason_ = "yaw_state_invalid";
    vla_swarm_finish_detail_ =
        "VLA_SWARM_YAW_HANDLE has no active task or scan direction";
    transitState(
        MISSION_FSM_STATE::VLA_SWARM_RECOVERY,
        "VLA_Swarm yaw state invalid");
    return;
  }

  if (!vla_swarm_scan_initialized_) {
    vla_swarm_scan_index_ = 0;
    vla_swarm_scan_command_published_ = false;
    vla_swarm_scan_yaw_reached_time_ = ros::Time();
    vla_swarm_scan_initialized_ = true;
  }

  if (vla_swarm_scan_index_ >= vla_swarm_scan_yaw_offsets_.size()) {
    // 当前门前所有方向均已扫描且未发现目标。
    vla_swarm_scan_initialized_ = false;
    vla_swarm_explore_area_id_ = -1;
    ++vla_swarm_exploration_round_;

    // 达到最大探索轮次时终止，交由上层 agent_run 决定是否重试。
    if (vla_swarm_exploration_round_ > vla_swarm_max_exploration_rounds_) {
      vla_swarm_success_ = false;
      vla_swarm_finish_reason_ = "max_exploration_rounds";
      vla_swarm_finish_detail_ =
          "Exceeded maximum exploration rounds (" +
          std::to_string(vla_swarm_max_exploration_rounds_) +
          ") without finding target";
      transitState(
          MISSION_FSM_STATE::VLA_SWARM_RECOVERY,
          "VLA_Swarm max exploration rounds reached");
      return;
    }

    // 累积跨轮记忆，参照原始 VLA_Swarm 的 key_action_history 机制。
    {
      nlohmann::json entry;
      entry["round"] = vla_swarm_exploration_round_;
      entry["action"] =
          "Scanned " + std::to_string(vla_swarm_scan_yaw_offsets_.size()) +
          " directions at door " + std::to_string(vla_swarm_explore_area_id_) +
          ", no target found";
      vla_swarm_key_action_history_.push_back(entry);
    }
    // 重置 AA 标记，下一轮重新评估全局状态。
    vla_swarm_aa_done_ = false;

    transitState(
        MISSION_FSM_STATE::VLA_SWARM_PLAN_LOCAL,
        "VLA_Swarm visual scan completed without target");
    return;
  }

  if (!vla_swarm_scan_command_published_) {
    // 等待 ego_state_trigger 确保无人机已停止运动后再发布旋转指令。
    if (!vla_swarm_ego_stable_) {
      return;
    }
    vla_swarm_ego_stable_ = false;
    // 在确定无人机停稳后记录悬停位置，避免捕获到惯性滑行中的偏移坐标。
    vla_swarm_scan_hold_position_ = fd_->odom_pos_;
    vla_swarm_scan_target_yaw_ = normalizeAngle(
        vla_swarm_scan_base_yaw_ +
        vla_swarm_scan_yaw_offsets_[vla_swarm_scan_index_]);
    pubLocalGoal(
        vla_swarm_scan_hold_position_, vla_swarm_scan_target_yaw_, false,
        quadrotor_msgs::EgoGoalSet::YAW_MODE_LOW_SPEED);
    vla_swarm_scan_command_time_ = ros::Time::now();
    vla_swarm_scan_yaw_reached_time_ = ros::Time();
    vla_swarm_scan_command_published_ = true;
    ROS_INFO_STREAM(
        "[VLA_SWARM] Scan observation=" << vla_swarm_scan_index_
        << ", target_yaw=" << vla_swarm_scan_target_yaw_);
    return;
  }

  if ((ros::Time::now() - vla_swarm_scan_command_time_).toSec() >
      vla_swarm_scan_timeout_) {
    vla_swarm_finish_reason_ = "yaw_scan_timeout";
    vla_swarm_finish_detail_ =
        "Yaw did not reach the requested observation direction";
    transitState(
        MISSION_FSM_STATE::VLA_SWARM_RECOVERY,
        "VLA_Swarm yaw scan timeout");
    return;
  }

  const double yaw_error =
      std::abs(normalizeAngle(vla_swarm_scan_target_yaw_ - fd_->odom_yaw_));
  if (yaw_error > vla_swarm_scan_yaw_tolerance_) {
    vla_swarm_scan_yaw_reached_time_ = ros::Time();
    return;
  }
  if (vla_swarm_scan_yaw_reached_time_.isZero()) {
    vla_swarm_scan_yaw_reached_time_ = ros::Time::now();
    return;
  }
  if ((ros::Time::now() - vla_swarm_scan_yaw_reached_time_).toSec() <
      vla_swarm_scan_settle_time_) {
    return;
  }

  sensor_msgs::CompressedImageConstPtr camera_image;
  ros::Time camera_receive_time;
  {
    std::lock_guard<std::mutex> lock(vla_swarm_camera_mutex_);
    camera_image = vla_swarm_latest_camera_image_;
    camera_receive_time = vla_swarm_latest_camera_receive_time_;
  }
  if (camera_image == nullptr) {
    return;
  }
  const ros::Time image_stamp =
      camera_image->header.stamp.isZero()
          ? camera_receive_time
          : camera_image->header.stamp;
  if (image_stamp <= vla_swarm_scan_yaw_reached_time_) {
    return;
  }

  scene_graph::VLASwarmObservation observation;
  observation.header.stamp = image_stamp;
  observation.header.frame_id =
      camera_image->header.frame_id.empty()
          ? std::string("camera")
          : camera_image->header.frame_id;
  observation.task_session_id = vla_swarm_session_id_;
  observation.observation_batch_id = vla_swarm_observation_batch_id_;
  observation.observation_index =
      static_cast<uint8_t>(vla_swarm_scan_index_);
  observation.body_yaw = fd_->odom_yaw_;
  observation.odom_pose.position.x = fd_->odom_pos_.x();
  observation.odom_pose.position.y = fd_->odom_pos_.y();
  observation.odom_pose.position.z = fd_->odom_pos_.z();
  observation.odom_pose.orientation.w = fd_->odom_orient_.w();
  observation.odom_pose.orientation.x = fd_->odom_orient_.x();
  observation.odom_pose.orientation.y = fd_->odom_orient_.y();
  observation.odom_pose.orientation.z = fd_->odom_orient_.z();
  observation.image = *camera_image;
  observation.image.header.stamp = image_stamp;
  vla_swarm_observation_pub_.publish(observation);
  vla_swarm_observation_stamp_ = observation.header.stamp;

  static const std::array<uint8_t, 4> prompt_types{{
      scene_graph::PromptMsg::PROMPT_TYPE_LOCAL_PLAN_PREDICTION_A,
      scene_graph::PromptMsg::PROMPT_TYPE_LOCAL_PLAN_PREDICTION_A1,
      scene_graph::PromptMsg::PROMPT_TYPE_LOCAL_PLAN_PREDICTION_A2,
      scene_graph::PromptMsg::PROMPT_TYPE_LOCAL_PLAN_PREDICTION_A3,
  }};
  if (vla_swarm_scan_index_ >= vla_swarm_scan_yaw_offsets_.size()) {
    vla_swarm_finish_reason_ = "yaw_scan_configuration_invalid";
    vla_swarm_finish_detail_ =
        "Scan index exceeds configured observation count";
    transitState(
        MISSION_FSM_STATE::VLA_SWARM_RECOVERY,
        "VLA_Swarm scan configuration invalid");
    return;
  }

  vla_swarm_prompt_type_ =
      prompt_types[vla_swarm_scan_index_ % prompt_types.size()];
  vla_swarm_prompt_id_ = scene_graph_->getCurPromptIdAndPlusOne();
  nlohmann::json visual_context;
  visual_context["observation_index"] = vla_swarm_scan_index_;
  visual_context["preferred_door_id"] = vla_swarm_explore_area_id_;
  std::string prompt;
  if (!scene_graph_->vlaSwarmPromptGen(
          vla_swarm_prompt_type_, vla_swarm_command_,
          vla_swarm_session_id_, vla_swarm_observation_batch_id_,
          visual_context, prompt)) {
    vla_swarm_finish_reason_ = "prompt_generation_failed";
    vla_swarm_finish_detail_ =
        "Failed to generate visual observation prompt";
    transitState(
        MISSION_FSM_STATE::VLA_SWARM_RECOVERY,
        "VLA_Swarm visual prompt generation failed");
    return;
  }

  const int timeout_seconds =
      std::max(1, static_cast<int>(std::ceil(vla_swarm_prompt_timeout_)));
  scene_graph_->sendPrompt(
      vla_swarm_prompt_id_, vla_swarm_prompt_type_, prompt,
      std::chrono::seconds(timeout_seconds),
      std::max(1, vla_swarm_max_plan_retries_));
  vla_swarm_prompt_pending_ = true;
  vla_swarm_prompt_start_time_ = ros::Time::now();
  transitState(
      MISSION_FSM_STATE::VLA_SWARM_WAIT_LLM,
      "VLA_Swarm visual observation prompt sent");
}

void FastExplorationFSM::handleVlaSwarmRecovery()
{
  if (vla_swarm_finish_reason_.empty()) {
    vla_swarm_finish_reason_ = "internal_error";
    vla_swarm_finish_detail_ = "VLA_Swarm recovery has no failure reason";
  }
  stopMotion();
  transitState(MISSION_FSM_STATE::VLA_SWARM_FINISH, "VLA_Swarm recovery");
}

void FastExplorationFSM::handleVlaSwarmFinish()
{
  if (!vla_swarm_active_) {
    transitState(MISSION_FSM_STATE::WAIT_TRIGGER, "VLA_Swarm finish inactive");
    return;
  }

  publishVlaSwarmResult(vla_swarm_success_, vla_swarm_finish_reason_, vla_swarm_finish_detail_);
  vla_swarm_active_ = false;
  transitState(MISSION_FSM_STATE::WAIT_TRIGGER, "VLA_Swarm finish");
}

void FastExplorationFSM::emergencyStopCallback(const std_msgs::Empty::ConstPtr&)
{
  if (!vla_swarm_active_) {
    return;
  }
  cancelVlaSwarmTask("task_cancelled", "received /command/emergency_stop");
}

void FastExplorationFSM::triggerCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  fd_->trigger_ = true;
}

void FastExplorationFSM::handleGoalInstruction(const std::vector<geometry_msgs::Point>& goals,
                                               const std::vector<float>& yaws,
                                               bool look_forward,
                                               const std::string& source) {
  switchPlannerCmdMuxToEgo(source + ":goal");
  stopElasticTracker(source + ":goal");

  if (goals.empty()) {
    ROS_WARN_STREAM("[GOAL] Ignore empty goal instruction from " << source);
    return;
  }

  const auto& first_goal = goals.front();
  const double goal_z = std::isfinite(first_goal.z) ? static_cast<double>(first_goal.z) : 1.0;
  const double yaw = yaws.empty() ? fd_->odom_yaw_ : static_cast<double>(yaws.front());

  {
    std::unique_lock<std::mutex> lck(mtx_);
    fd_->track_trigger_ = false;
    fd_->track_init_ = false;
    resetTrackingFinishCandidate();
    fd_->track_finish_sent_ = false;
    if (!useElasticTrackerBackend()) {
      map_->setTarget(fd_->track_pos_, false);
    }
    if (md_->mission_state_ == MISSION_FSM_STATE::PLAN_TRACK ||
        md_->mission_state_ == MISSION_FSM_STATE::APPROACH_TRACK) {
      stopMotion();
    }
    transitState(MISSION_FSM_STATE::WAIT_TRIGGER, source);
  }

  pubLocalGoal(
      Eigen::Vector3d(first_goal.x, first_goal.y, goal_z),
      yaw,
      look_forward,
      quadrotor_msgs::EgoGoalSet::YAW_MODE_NORMAL);
}

void FastExplorationFSM::handleTrackingTarget(const std::vector<geometry_msgs::Point>& global_poses,
                                              const std::string& source,
                                              const ros::Time& stamp,
                                              const std::string& frame_id) {
  expl_manager_->vis_ptr_->visualize_a_ball(fd_->track_pos_, 0.35, "track_pos", visualization::Color::blue);
  if (global_poses.empty()) return;

  const auto& first_pose = global_poses.front();
  if (first_pose.x == -1 && first_pose.y == -1 && first_pose.z == -1) return;

  std::unique_lock<std::mutex> lck(mtx_);
  if (!fd_->track_trigger_) {
    ROS_WARN_STREAM_THROTTLE(2.0, "Wait for track command, ignore tracking target.");
    return;
  }

  double min_dist = std::numeric_limits<double>::max();
  int min_index = -1;
  for (int i = 0; i < static_cast<int>(global_poses.size()); ++i) {
    const auto& pose = global_poses[i];
    if (pose.x == -1 && pose.y == -1 && pose.z == -1) continue;

    const Eigen::Vector3d candidate = geoPt2Vec3d(pose);
    const double dist = (candidate - fd_->track_pos_).norm();
    if (dist < min_dist) {
      min_dist = dist;
      min_index = i;
    }
  }

  if (min_index < 0) return;

  const Eigen::Vector3d candidate = geoPt2Vec3d(global_poses[min_index]);
  const bool was_track_init = fd_->track_init_;
  if (!fd_->track_init_ || min_dist < expl_manager_->ep_->track_detect_error_) {
    fd_->track_pos_ = candidate;
    fd_->track_init_ = true;
    if (!was_track_init) {
      resetTrackingFinishCandidate();
    }
    ROS_INFO_STREAM_THROTTLE(0.5, "[TRACK] Update target: " << fd_->track_pos_.transpose());
  } else {
    ROS_WARN_STREAM_THROTTLE(1.0, "[TRACK] Ignore target jump, candidate: " << candidate.transpose()
                             << " previous: " << fd_->track_pos_.transpose());
    return;
  }

  if (useElasticTrackerBackend()) {
    publishTrackingTargetOdom(fd_->track_pos_, stamp, frame_id);
    switchPlannerCmdMuxToElastic("tracking_target_update");
    return;
  }

  if (map_->isInited()) {
    map_->setTarget(fd_->track_pos_, fd_->track_init_);
  }

  if (md_->mission_state_ != MISSION_FSM_STATE::PLAN_TRACK &&
      md_->mission_state_ != MISSION_FSM_STATE::APPROACH_TRACK) {
    transitState(MISSION_FSM_STATE::PLAN_TRACK, source);
  }
}

void FastExplorationFSM::egoPlannerGoalCallback(const quadrotor_msgs::GoalSet::ConstPtr &msg) {
  handleGoalInstruction(msg->goal, msg->yaw, msg->look_forward, "goalFromStation");
}

void FastExplorationFSM::egoExecFinishCallback(const std_msgs::Bool::ConstPtr &msg) {
  fd_->ego_exec_finished_ = msg->data;
  INFO_MSG_GREEN("--------- [FSM] EGO-Planner Execution Finished -----------");
}

void FastExplorationFSM::trackCommandCallback(const quadrotor_msgs::TrackCommand::ConstPtr& msg) {
  if (msg->robot_id != md_->drone_id_) return;

  std::unique_lock<std::mutex> lck(mtx_);
  if (!msg->enable)
  {
    fd_->track_trigger_ = false;
    fd_->track_init_ = false;
    resetTrackingFinishCandidate();
    fd_->track_finish_sent_ = false;
    switchPlannerCmdMuxToEgo("trackCommand:disable");
    stopElasticTracker("trackCommand:disable");
    if (!useElasticTrackerBackend()) {
      map_->setTarget(fd_->track_pos_, false);
    }
    if (md_->mission_state_ == MISSION_FSM_STATE::PLAN_TRACK ||
        md_->mission_state_ == MISSION_FSM_STATE::APPROACH_TRACK)
    {
      stopMotion();
      transitState(MISSION_FSM_STATE::WAIT_TRIGGER, "trackCommand:disable");
    }
    return;
  }

  const bool was_track_trigger = fd_->track_trigger_;
  if (useElasticTrackerBackend() && fd_->track_finish_sent_ && !was_track_trigger) {
    ROS_INFO_STREAM_THROTTLE(0.5, "[TRACK] ignore residual Elastic-Tracker enable after finish; waiting for disable/reset.");
    return;
  }
  fd_->track_trigger_ = true;
  if (!was_track_trigger) {
    resetTrackingFinishCandidate();
    fd_->track_finish_sent_ = false;
  }
  if (msg->has_target_position)
  {
    fd_->track_pos_ = geoPt2Vec3d(msg->target_position);
  }
  if (useElasticTrackerBackend()) {
    if (msg->has_target_position) {
      fd_->track_init_ = true;
      publishTrackingTargetOdom(fd_->track_pos_, msg->header.stamp, msg->header.frame_id);
    }
    // Elastic-Tracker 的 /triger 表示新会话开始；持续重发会清空 traj_server 缓存并重置 hover 完成计时。
    if (!was_track_trigger) {
      publishElasticTrackerTrigger(msg->header.stamp, msg->header.frame_id);
    }
    switchPlannerCmdMuxToElastic("trackCommand:elastic_tracker_enable");
    if (md_->mission_state_ != MISSION_FSM_STATE::WAIT_TRIGGER) {
      stopMotion();
      transitState(MISSION_FSM_STATE::WAIT_TRIGGER, "trackCommand:elastic_tracker_enable");
    }
    return;
  }

  switchPlannerCmdMuxToEgo("trackCommand:ego_tracking_enable");
  map_->setTarget(fd_->track_pos_, false);
  transitState(MISSION_FSM_STATE::PLAN_TRACK, "trackCommand:enable");
}

void FastExplorationFSM::elasticTrackingFinishCallback(const std_msgs::Bool::ConstPtr& msg) {
  if (!msg->data || !useElasticTrackerBackend()) {
    return;
  }

  std::unique_lock<std::mutex> lck(mtx_);
  if (!fd_->track_trigger_) {
    ROS_WARN_STREAM_THROTTLE(1.0, "[TRACK] Ignore Elastic-Tracker finish without active tracking session.");
    return;
  }

  fd_->track_trigger_ = false;
  fd_->track_init_ = false;
  resetTrackingFinishCandidate();
  stopMotion();
  switchPlannerCmdMuxToEgo("elasticTrackingFinish");
  publishTrackingFinish();
  if (md_->mission_state_ != MISSION_FSM_STATE::INIT &&
      md_->mission_state_ != MISSION_FSM_STATE::WARM_UP) {
    transitState(MISSION_FSM_STATE::WAIT_TRIGGER, "elasticTrackingFinish");
  }
}

void FastExplorationFSM::targetCallbackReal(const quadrotor_msgs::DetectOut::ConstPtr& msg)
{
  handleTrackingTarget(msg->global_poses, "trackTargetUpdate", msg->header.stamp, msg->header.frame_id);
}

void FastExplorationFSM::vlaSwarmTargetCallback(
    const quadrotor_msgs::VLASwarmTarget::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(mtx_);
  if (!vla_swarm_active_ || !vla_swarm_target_pending_) {
    return;
  }
  if (msg->task_session_id != vla_swarm_session_id_ ||
      msg->observation_batch_id != vla_swarm_observation_batch_id_ ||
      msg->request_id != vla_swarm_target_request_id_ ||
      msg->observation_index != vla_swarm_target_observation_index_) {
    ROS_WARN_STREAM_THROTTLE(
        1.0,
        "[VLA_SWARM] Ignore stale target result: session="
            << msg->task_session_id
            << ", batch=" << msg->observation_batch_id
            << ", request=" << msg->request_id
            << ", observation=" << static_cast<int>(msg->observation_index));
    return;
  }

  vla_swarm_target_received_ = true;
  vla_swarm_target_success_ = msg->success;
  vla_swarm_target_source_ = msg->source;
  vla_swarm_target_error_ = msg->error;
  if (msg->success) {
    vla_swarm_target_position_ = Eigen::Vector3d(
        msg->pose.position.x,
        msg->pose.position.y,
        msg->pose.position.z);
  }
}

void FastExplorationFSM::vlaSwarmCameraCallback(
    const sensor_msgs::CompressedImageConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(vla_swarm_camera_mutex_);
  vla_swarm_latest_camera_image_ = msg;
  vla_swarm_latest_camera_receive_time_ = ros::Time::now();
}

void FastExplorationFSM::vlaSwarmEgoStateTriggerCallback(
    const quadrotor_msgs::EgoStateTrigger::ConstPtr& msg)
{
  vla_swarm_ego_stable_ = msg->data;
}

bool FastExplorationFSM::getSceneGraphInitSeed(Eigen::Vector3d& init_seed, std::string* reason) const {
  init_seed = fd_->odom_pos_;
  init_seed.x() += fp_->scene_graph_init_forward_dist_ * std::cos(fd_->odom_yaw_);
  init_seed.y() += fp_->scene_graph_init_forward_dist_ * std::sin(fd_->odom_yaw_);

  if (!map_->isInGlobalMap(init_seed)) {
    if (reason != nullptr) *reason = "seed out of global map";
    return false;
  }

  if (!map_->isInLocalMap(init_seed)) {
    if (reason != nullptr) *reason = "seed out of local map buffer";
    return false;
  }

  if (map_->getInflateOccupancy(init_seed) == MapInterface::OCCUPIED) {
    if (reason != nullptr) *reason = "seed in occupied region";
    return false;
  }

  return true;
}

void FastExplorationFSM::handelThingkingProcess() {
  double t_cur = ros::Time::now().toSec() - think_start_time_;
  if (stash_state_ == MISSION_FSM_STATE::THINKING && md_->mission_state_ != MISSION_FSM_STATE::WAIT_TRIGGER && !fd_->df_demo_mode_) {
    transitState(LLM_PLAN_EXPLORE, "** fsm RECOVER !!!!!!!");
    stash_state_ = LLM_PLAN_EXPLORE;
  }

  if (t_cur > think_duration_limit_) {
    ROS_ERROR("thinking overtime ! skip this step ...");
    transitState(stash_state_, "thinking overtime ! skip this step ...");
    return ;
  }

  expl_manager_->visualize(fd_->odom_pos_);
  unsigned cur_prompt_id = scene_graph_->wait_recv_id_;

  if (scene_graph_->llm_ans_str_poll_.find(cur_prompt_id) != scene_graph_->llm_ans_str_poll_.end()) {
    INFO_MSG_GREEN("LLM ANS \n" << scene_graph_->llm_ans_str_poll_[cur_prompt_id]);

    if (scene_graph_->llm_prompts_[cur_prompt_id].prompt_type == scene_graph::PromptMsg::PROMPT_TYPE_ROOM_PREDICTION) {
      scene_graph_->handleRoomPredictionResult(cur_prompt_id);

    }else if (scene_graph_->llm_prompts_[cur_prompt_id].prompt_type == scene_graph::PromptMsg::PROMPT_TYPE_EXPL_PREDICTION) {
      expl_area_id_ = scene_graph_->handelExplorationResult(cur_prompt_id);
      if (expl_area_id_ == -100) {
        transitState(MISSION_FSM_STATE::FIND_TERMINATE_TARGET, "Have found correct area !");
        return ;
      }
      if (scene_graph_->skeleton_gen_->area_handler_->area_map_.find(expl_area_id_) != scene_graph_->skeleton_gen_->area_handler_->area_map_.end()) {
        has_made_area_decision_ = (expl_area_id_ != -1);
      }

    }else if(scene_graph_->llm_prompts_[cur_prompt_id].prompt_type == scene_graph::PromptMsg::PROMPT_TYPE_TERMINATE_OBJ_ID){
      fd_->object_target_id_ = scene_graph_->handelTerminateObjIdResult(cur_prompt_id);
      if (fd_->object_target_id_ >= 0) {
        transitState(MISSION_FSM_STATE::GO_TARGET_OBJECT, "Recv Terminate Object ID !");
        return ;
      }else {
        publishExplorationResult(false, "target_not_found", "terminate object id result is invalid");
        transitState(MISSION_FSM_STATE::FINISH, "Recv Terminate Object ID Failed !");
        return ;
      }

    }else if(scene_graph_->llm_prompts_[cur_prompt_id].prompt_type == scene_graph::PromptMsg::PROMPT_TYPE_DF_DEMO){
      fd_->df_demo_target_id_ = scene_graph_->handelDFDemoResult(cur_prompt_id);
      transitState(MISSION_FSM_STATE::DF_DEMO, "Recv DF Demo Result !");
      return ;
    }

    transitState(stash_state_, "THINKING :D");
  }
}

void FastExplorationFSM::planLLMExplore() {
  ROS_INFO("\033[1;31m\n\n ============== [Plan LLM Explore] ==============\033[0m");

  if (waitForFreshMapAfterReset())
    return;

  // 全景旋转优先：task_id=2/8 启动阶段先360°旋转一圈再探索
  if (need_panorama_) {
    handlePanoramaYaw();
    return;
  }

  if (need_rotate_yaw_ && !enable_yaw_scan_)
    need_rotate_yaw_ = false;

  if (need_rotate_yaw_) {
    auto yawLimit = [] (double yaw) {
      while (yaw >= M_PI) yaw -= 2 * M_PI;
      while (yaw < -M_PI) yaw += 2 * M_PI;
      return yaw;
    };
    yawhandle_yaw_raw          = yawLimit(fd_->odom_yaw_);
    yawhandle_yaw_target_left  = yawLimit(yawhandle_yaw_raw + 45 / 180.0 * M_PI);
    yawhandle_yaw_target_right = yawLimit(yawhandle_yaw_raw - 45 / 180.0 * M_PI);
    yawhandle_left_ok = yawhandle_right_ok = yawhandle_back_ok = false;
    yawhandle_left_published = yawhandle_right_published = yawhandle_back_published = false;
    stashCurStateAndTransit(MISSION_FSM_STATE::YAW_HANDLE, "rotate yaw before LLM explore");
    INFO_MSG_YELLOW("[FSM] Plan LLM Explore : No Yaw Handle ... start yaw handle process");
    return ;
  }

  scene_graph_->mountCurPoly(fd_->odom_pos_, fd_->odom_yaw_);

  if (fp_->object_id_nav_use_thinking_ && scene_graph_->needAreaPrediction()) {
    expl_manager_->frontier_finder_->updateSceneGraphWithFtr();
    INFO_MSG_YELLOW("[FSM] Plan LLM Explore : Area Need Prediction ... start area predict process");
    std::string llm_prompt_str;
    scene_graph_->newAreaPredictionPromptGen(llm_prompt_str);
    cur_prompt_id_ = scene_graph_->getCurPromptId();
    scene_graph_->sendPrompt(scene_graph_->getCurPromptIdAndPlusOne(),
                             scene_graph::PromptMsg::PROMPT_TYPE_ROOM_PREDICTION,
                             llm_prompt_str, std::chrono::seconds(20), 1);
    stashCurStateAndTransit(MISSION_FSM_STATE::THINKING, "frontierCallback");
    think_start_time_     = ros::Time::now().toSec();
    think_duration_limit_ = 20.0 * 1.0;

    has_made_area_decision_ = false;
    return ;
  }

  if (!has_made_area_decision_) {
    INFO_MSG_GREEN("[FSM]: check if current area need more exploration ...");
    if (scene_graph_->cur_poly_ != nullptr) {
      // step1 current area need more exploration
      auto it = scene_graph_->skeleton_gen_->area_handler_->area_map_.find(scene_graph_->cur_poly_->area_id_);

      if (it != scene_graph_->skeleton_gen_->area_handler_->area_map_.end() && it->second->room_label_ == "Unknown" && it->second->num_ftrs_ > 0) {
        INFO_MSG_YELLOW("\n ********************************************************************************** ");
        INFO_MSG_YELLOW(" [Plan LLM Expl]: Current Area Need More Exploration, Go to Current Area [" << it->second->id_ << "] ");
        INFO_MSG_YELLOW(" **********************************************************************************\n");
        expl_area_id_ = it->second->id_;
        has_made_area_decision_ = true;

      } else {
        // step2 check nbr area need more exploration
        if (!it->second->nbr_area_.empty()) {
          for (auto nbr_area_id : it->second->nbr_area_) {
            auto nbr_area = scene_graph_->skeleton_gen_->area_handler_->area_map_.find(nbr_area_id.first);
            if(nbr_area != scene_graph_->skeleton_gen_->area_handler_->area_map_.end() && nbr_area->second->room_label_ == "Unknown" && nbr_area->second->num_ftrs_ > 0) {
              INFO_MSG_YELLOW("[Plan LLM Expl]: Found Nbr Area with Unknown Room Label, Go to Nbr Area [" << nbr_area->second->id_ << "] ");
              expl_area_id_ = nbr_area->second->id_;
              has_made_area_decision_ = true;
              break;
            }
          }
        }
      }
    }
  }

  if(fd_->llm_plan_explore_counter_ == 1 && scene_graph_->cur_poly_ != nullptr){
    has_made_area_decision_ = true;
    auto it = scene_graph_->skeleton_gen_->area_handler_->area_map_.find(scene_graph_->cur_poly_->area_id_);
    if (it != scene_graph_->skeleton_gen_->area_handler_->area_map_.end()) {
      expl_area_id_ = it->second->id_;
    }
    INFO_MSG_YELLOW("*** [FSM] Plan LLM Explore : Regular explore ...");
  }

  if (!has_made_area_decision_) {
    // THINKING模式: 通过LLM选择探索区域; 非THINKING模式则跳过, 直接fall-through到常规探索
    if (fp_->object_id_nav_use_thinking_) {
      INFO_MSG_YELLOW("[FSM] Plan LLM Explore : No Area Decision Made ... start llm-exploration process");

      // cur_poly_ 为空时（骨架尚未生成或已被 reset），不能解引用 area_id_。
      // 此时直接发送 LLM prompt 让大模型根据场景图文本信息选择探索区域。
      if (scene_graph_->cur_poly_ != nullptr) {
        scene_graph_->history_visited_area_ids_.push_back(scene_graph_->cur_poly_->area_id_);
      }

      std::string prompt;
      scene_graph_->chooseAreaToGoPromptGen(prompt);
      scene_graph_->sendPrompt(scene_graph_->getCurPromptIdAndPlusOne(),
                               scene_graph::PromptMsg::PROMPT_TYPE_EXPL_PREDICTION,
                               prompt, std::chrono::seconds(10), 1);
      stashCurStateAndTransit(MISSION_FSM_STATE::THINKING, "llm explore plan!");
      think_start_time_     = ros::Time::now().toSec();
      think_duration_limit_ = 10.0 * 1.0;
      return ;
    } else {
      // 非THINKING模式: 使用当前area继续, planLLMExploration内Fix2兜底全局fallback
      has_made_area_decision_ = true;
      if (expl_area_id_ < 0 && scene_graph_->cur_poly_ != nullptr) {
        expl_area_id_ = scene_graph_->cur_poly_->area_id_;
      }
    }
  }

  fd_->path_res_.clear();

  // 全景旋转期间 VLM 可能已检测到目标物体。
  // 若 target_cmd_ 非空且场景图中存在匹配的 object，直接切入终止目标导航，
  // 不再走 frontier 探索路径。
  if (!fd_->target_cmd_.empty() && fd_->target_cmd_ != "None") {
    int found_obj_id = -1;
    for (const auto& obj_pair : scene_graph_->object_factory_->object_map_) {
      if (obj_pair.second->label == fd_->target_cmd_) {
        found_obj_id = obj_pair.second->id;
        break;
      }
    }
    if (found_obj_id >= 0) {
      INFO_MSG_GREEN("[FSM] Target object '" << fd_->target_cmd_
                     << "' (id=" << found_obj_id << ") found in scene graph, navigate to it.");
      fd_->object_target_id_ = found_obj_id;
      transitState(MISSION_FSM_STATE::FIND_TERMINATE_TARGET, "LLM target found in scene graph");
      return;
    }
  }

  int res = callExplorationLLMPlanner(fd_->aim_pos_, fd_->aim_vel_, fd_->aim_yaw_, fd_->path_res_);

  fd_->llm_plan_explore_counter_ ++;
  if(fd_->llm_plan_explore_counter_ >= 2){
    has_made_area_decision_ = false;
    fd_->llm_plan_explore_counter_ = 0;
  }

  if (res == NO_FRONTIER) {
    has_made_area_decision_ = false;
    // 非THINKING模式: 无frontier时先检查是否有匹配目标物体, 有则直接导航过去
    if (!fp_->object_id_nav_use_thinking_ && !fd_->target_cmd_.empty() && fd_->target_cmd_ != "None") {
      int found_id = -1;
      int best_cnt = 0;
      for (const auto& obj_pair : scene_graph_->object_factory_->object_map_) {
        const auto& obj = obj_pair.second;
        if (obj->label == fd_->target_cmd_ && (int)obj->detection_count > best_cnt) {
          found_id = obj->id;
          best_cnt = obj->detection_count;
        }
      }
      if (found_id >= 0) {
        fd_->object_target_id_ = found_id;
        INFO_MSG_GREEN("[FSM] No frontier, direct match target: '" << fd_->target_cmd_
                       << "' id=" << found_id << " detections=" << best_cnt);
        transitState(MISSION_FSM_STATE::FIND_TERMINATE_TARGET, "no frontier, direct object match");
        return;
      }
    }
    publishExplorationResult(false, "target_not_found", "target area has no frontier");
    transitState(FINISH, "LLM Plan No Frontier");
    return;

  } else if (res == FAIL) {
    has_made_area_decision_ = false;
    planRegularExplore();
    INFO_MSG_RED(" !!!!!!!!!!!!!!!!!!!! LLM Plan Explore Failed !!!!!!!!!!!!!!!!!!!!!!");
    INFO_MSG_RED("                     PLan Regular Explore Once ");
    return ;

  }else {
    has_made_area_decision_ = false;
    double dis_2_aim       = (fd_->aim_pos_ - fd_->odom_pos_).norm();
    double dis_2_aim_2d    = (fd_->aim_pos_ - fd_->odom_pos_).head(2).norm();
    bool look_forward      = dis_2_aim >= expl_manager_->ep_->radius_close_;
    fd_->path_inx_ = 0;

    getAndPublishNextAim(fd_->path_res_, look_forward, fd_->aim_yaw_);
    fd_->is_lookforward_ = look_forward;
    fd_->has_rotated_ = !look_forward;
    fd_->last_pub_time_ = ros::Time::now();
    // 建议B/D: 进入 APPROACH_EXPLORE 前初始化 last_progress_time_ 并清零卡死状态
    fd_->last_progress_time_ = ros::Time::now();
    resetExploreStuckState();
    INFO_MSG_GREEN("[EXP-FSM] [look_forward = " << look_forward << "] aim: " << fd_->aim_pos_.transpose() << ", local_aim: " << fd_->local_aim_pos_.transpose());
    transitState(APPROACH_EXPLORE, "LLM Plan Success");
  }
}

void FastExplorationFSM::planRegularExplore() {
  ROS_INFO("\033[1;31mPlan Regular Explore!\033[0m");  //红

  if (waitForFreshMapAfterReset())
    return;

  // 全景旋转优先：task_id=2/8 启动阶段先360°旋转一圈再探索
  if (need_panorama_) {
    handlePanoramaYaw();
    return;
  }

  if (need_rotate_yaw_ && !enable_yaw_scan_)
    need_rotate_yaw_ = false;

  if (need_rotate_yaw_ && fd_->df_demo_mode_) {
    auto yawLimit = [] (double yaw) {
      while (yaw >= M_PI) yaw -= 2 * M_PI;
      while (yaw < -M_PI) yaw += 2 * M_PI;
      return yaw;
    };
    yawhandle_yaw_raw          = yawLimit(fd_->odom_yaw_);
    yawhandle_yaw_target_left  = yawLimit(yawhandle_yaw_raw + 45.0 / 180.0 * M_PI);
    yawhandle_yaw_target_right = yawLimit(yawhandle_yaw_raw - 45.0 / 180.0 * M_PI);
    yawhandle_left_ok = yawhandle_right_ok = yawhandle_back_ok = false;
    yawhandle_left_published = yawhandle_right_published = yawhandle_back_published = false;
    stashCurStateAndTransit(MISSION_FSM_STATE::YAW_HANDLE, "rotate yaw before LLM explore");
    INFO_MSG_YELLOW("[FSM] Plan Regular Explore : No Yaw Handle ... start yaw handle process");
    need_rotate_yaw_ = false;
    return ;
  }

  // 探索过程中检查VLM是否已检测到目标物体, 若匹配则直接导航过去
  if (!fd_->target_cmd_.empty() && fd_->target_cmd_ != "None") {
    int found_obj_id = -1;
    for (const auto& obj_pair : scene_graph_->object_factory_->object_map_) {
      if (obj_pair.second->label == fd_->target_cmd_) {
        found_obj_id = obj_pair.second->id;
        break;
      }
    }
    if (found_obj_id >= 0) {
      INFO_MSG_GREEN("[FSM] Target object '" << fd_->target_cmd_
                     << "' (id=" << found_obj_id << ") found in scene graph, navigate to it.");
      fd_->object_target_id_ = found_obj_id;
      transitState(MISSION_FSM_STATE::FIND_TERMINATE_TARGET, "regular explore: target found in scene graph");
      return;
    }
  }

  fd_->path_res_.clear();
  int res = callExplorationPlanner(fd_->aim_pos_, fd_->aim_vel_, fd_->aim_yaw_, fd_->path_res_);

  if (fd_->df_demo_mode_) 
    need_rotate_yaw_ = enable_yaw_scan_;

  if (res == SUCCEED)
  {
    double dis_2_aim       = (fd_->aim_pos_ - fd_->odom_pos_).norm();
    double dis_2_aim_2d    = (fd_->aim_pos_ - fd_->odom_pos_).head(2).norm();
    bool look_forward = dis_2_aim >= expl_manager_->ep_->radius_close_;
    fd_->path_inx_ = 0;

    getAndPublishNextAim(fd_->path_res_, look_forward, fd_->aim_yaw_);
    fd_->is_lookforward_ = look_forward;
    fd_->has_rotated_ = !look_forward;
    INFO_MSG_GREEN("[EXP-FSM] [look_forward = " << look_forward << "] aim: " << fd_->aim_pos_.transpose() << ", local_aim: " << fd_->local_aim_pos_.transpose());
    fd_->last_pub_time_ = ros::Time::now();
    // 建议B/D: 进入 APPROACH_EXPLORE 前初始化 last_progress_time_ 并清零卡死状态
    fd_->last_progress_time_ = ros::Time::now();
    resetExploreStuckState();

    transitState(APPROACH_EXPLORE, "FSM");
    need_rotate_yaw_ = enable_yaw_scan_;
    // ROS_ERROR_STREAM("[Plan_Traj] start_pos: " << fd_->start_pt_.transpose());
    // ROS_ERROR_STREAM("[Plan_Traj] odom_pos: " << fd_->odom_pos_.transpose());
    // ROS_ERROR_STREAM("[Plan_Traj] start_vel: " << fd_->start_vel_.transpose());
    // ROS_ERROR_STREAM("[Plan_Traj] odom_vel: " << fd_->odom_vel_.transpose());
  }
  else if (res == NO_FRONTIER)
  {
    // 无frontier时最后检查一次VLM是否检测到匹配目标物体
    if (!fd_->target_cmd_.empty() && fd_->target_cmd_ != "None") {
      int found_id = -1;
      int best_cnt = 0;
      for (const auto& obj_pair : scene_graph_->object_factory_->object_map_) {
        const auto& obj = obj_pair.second;
        if (obj->label == fd_->target_cmd_ && (int)obj->detection_count > best_cnt) {
          found_id = obj->id;
          best_cnt = obj->detection_count;
        }
      }
      if (found_id >= 0) {
        fd_->object_target_id_ = found_id;
        INFO_MSG_GREEN("[FSM] No frontier, direct match target: '" << fd_->target_cmd_
                       << "' id=" << found_id << " detections=" << best_cnt);
        transitState(MISSION_FSM_STATE::FIND_TERMINATE_TARGET, "regular explore: no frontier, direct object match");
        return;
      }
    }
    const std::string reason = expl_manager_->hasExplorationRegion() ? "region_explored" : "global_explored";
    publishExplorationResult(true, reason, "no frontier remains");
    transitState(FINISH, "FSM");
    // clearVisMarker();
  }
  else if (res == FAIL)
  {
    // Still in PLAN_TRAJ state, keep replanning
    ROS_ERROR("\n\n ======================== [FSM] Plan fail! ==========================");
    INFO_MSG_RED("================================================================================");
    INFO_MSG_RED("[EXPL-FSM] : Can't reach frontier, delete this frontier and add it to blacklist!");
    INFO_MSG_RED("================================================================================");
    expl_manager_->frontier_finder_->addFtrBlacklist(expl_manager_->ed_->frontier_to_explore_.average_);
    INFO_MSG_RED("add to blacklist: " << expl_manager_->ed_->frontier_to_explore_.average_.transpose());
    expl_manager_->forceDeleteFrontier(expl_manager_->ed_->frontier_to_explore_);
  }
  expl_manager_->visualize(fd_->odom_pos_);
}

void FastExplorationFSM::approachRegularExplore() {

  double dis_2_aim       = (fd_->aim_pos_       - fd_->odom_pos_).norm();
  double dis_2_aim_2d    = (fd_->aim_pos_       - fd_->odom_pos_).head(2).norm();
  double dis_2_local_aim = (fd_->local_aim_pos_ - fd_->odom_pos_).norm();
  double dis_yaw         = fd_->aim_yaw_ - fd_->odom_yaw_;

  // 建议B: t_cur 改用独立的 last_progress_time_ (仅真正推进时刷新)
  // 旧逻辑用 last_pub_time_, 被 [7] 分支无条件刷新 → t_cur 永远 <15s → [5] 卡死恢复被饿死
  double t_cur = (ros::Time::now() - fd_->last_progress_time_).toSec();
  std::string ego_plan_status_str_   = fd_->ego_plan_status_ ? "True" : "False";
  std::string ego_modify_status_str_ = fd_->ego_modify_status_ ? "True" : "False";

  ROS_INFO_STREAM_THROTTLE(0.5, "\033[1;33mApproach EXPLORE...\033[0m \n"
                                "   * Dis to Aim: " << dis_2_aim_2d << "\n"
                                "   * Dis to LocalAim: " << dis_2_local_aim << "\n"
                                "   * Dis to yaw: " << dis_yaw
                                << "\n   * local_aim_fail: " << fd_->local_aim_fail_count_
                                << "\n   * explore_stuck_cnt: " << fd_->explore_stuck_advance_count_
                                << "\n   * t_cur(progress): " << t_cur);  // 黄
  ROS_INFO_STREAM_THROTTLE(0.5, "[EXPL-FSM] : ego local goal -> (" << fd_->ego_local_goal_.transpose() << ")");
  ROS_INFO_STREAM_THROTTLE(0.5, "[EXPL-FSM] : ego plan times: " << fd_->ego_plan_times_
                                                                << "  ego plan statue: " << ego_plan_status_str_
                                                                << "  ego modify status: " << ego_modify_status_str_);

  // ! bad frontier delete
  bool bad_frontier = false;
  Eigen::Vector3d cur_viewpoint = fd_->path_res_.back();
  if (fd_->ego_plan_times_ > 40) {
    INFO_MSG_RED("[EXPL-FSM] : replan time out, delete this frontier and add it to blacklist, replan!");
    bad_frontier = true;
  }

  if (fd_->ego_modify_status_ && fd_->ego_exec_finished_
      && (fd_->odom_pos_ - fd_->ego_local_goal_).norm() < 0.1
      && map_->isInLocalMap(cur_viewpoint) && t_cur > 8.0) {
    INFO_MSG_RED("[EXPL-FSM] : ego modify status, delete this frontier and add it to blacklist, replan!");
    bad_frontier = true;
  }

  // 建议E: 纯本地卡死判据(不依赖 ego 反馈, 打破 ego 断链时的死锁)
  // 条件: 卡死超阈值 + 距 local_aim 近 + getLocalAim 连续失败 → 该 frontier 不可达
  // 背景: ego_plan_result 断链时上面的判据全部失效, 需要纯本地信号兜底
  if (t_cur > fp_->explore_local_stuck_duration_ &&
      fd_->odom_vel_.norm() < fp_->explore_local_stuck_vel_thresh_ &&
      dis_2_local_aim < expl_manager_->ep_->radius_close_ &&
      fd_->local_aim_fail_count_ >= fp_->explore_local_aim_fail_max_) {
    INFO_MSG_RED("[EXPL-FSM] : local stuck (no ego feedback), delete this frontier and add to blacklist!");
    bad_frontier = true;
  }

  // replan judgement
  MISSION_FSM_STATE replan_target_state =
        fd_->regular_explore_ ? MISSION_FSM_STATE::PLAN_EXPLORE : MISSION_FSM_STATE::LLM_PLAN_EXPLORE;

  if (fd_->df_demo_mode_)
    replan_target_state = MISSION_FSM_STATE::DF_DEMO;

  if (bad_frontier){
    INFO_MSG_RED("===========================================================================================");
    INFO_MSG_RED("[EXPL-FSM] : Can't reach frontier, delete this frontier and add it to blacklist, replan!");
    INFO_MSG_RED("===========================================================================================");
    expl_manager_->frontier_finder_->addFtrBlacklist(expl_manager_->ed_->frontier_to_explore_.average_);
    INFO_MSG_RED("Ftr [" << expl_manager_->ed_->frontier_to_explore_.id_ << "] pos : "<< expl_manager_->ed_->frontier_to_explore_.average_.transpose());
    expl_manager_->forceDeleteFrontier(expl_manager_->ed_->frontier_to_explore_);
    // 建议E: 删 frontier 后重置 ego 反馈状态, 防止残留值下次误触发 bad_frontier 或 [6]
    resetExploreEgoState();
    resetExploreStuckState();
    fd_->last_progress_time_ = ros::Time::now();   // 建议B: 重规划起点
    transitState(replan_target_state, "FSM");
    return;   // 建议E: 显式 return, 避免继续走到下方分支
  }

  // ====== replan judgement (优先级从高到低) ======

  // [1] 飞行途中目标 frontier 已被传感器覆盖 → 立即重规划
  if (t_cur > fp_->replan_thresh2_ &&
      expl_manager_->frontier_finder_->isAnyFrontierCovered(fd_->odom_pos_)) {
    ROS_WARN("\n-------------> Replan: frontier covered <-------------\n");
    resetExploreStuckState();
    fd_->last_progress_time_ = ros::Time::now();
    transitState(replan_target_state, "frontier_covered");
    return;
  }

  // [2] 飞行途中 frontier 列表发生变化 → 立即重规划
  if (fd_->frontier_changed_ && t_cur > fp_->replan_thresh2_) {
    ROS_WARN("\n-------------> Replan: frontiers changed <-------------\n");
    fd_->frontier_changed_ = false;
    resetExploreStuckState();
    fd_->last_progress_time_ = ros::Time::now();
    transitState(replan_target_state, "frontier_changed");
    return;
  }

  // [3] 预到达重规划：距最终目标足够近 → 不等完全到达，直接重规划
  bool near_final_waypoint = (fd_->path_inx_ >= (int)fd_->path_res_.size() - 2 ||
                              fd_->path_res_.size() <= 2);
  if (near_final_waypoint &&
      dis_2_aim_2d < expl_manager_->ep_->radius_close_ * 2.0 &&
      t_cur > fp_->replan_thresh2_) {
    ROS_WARN("\n-------------> Replan: pre-arrival (%.1fm to aim) <-------------\n", dis_2_aim_2d);
    resetExploreStuckState();
    fd_->last_progress_time_ = ros::Time::now();
    transitState(replan_target_state, "pre_arrival");
    return;
  }

  // [4] 到达视点（兜底）：位置+Yaw都满足 → 累计 observation 并重规划
  if (dis_2_aim_2d < fp_->replan_dis_thresh_ && fabs(fd_->odom_yaw_ - fd_->aim_yaw_) < 10.0 / 180.0f * M_PI) {
    ROS_WARN("\n-------------> Replan: [Reach Both Pos&Yaw Aim] <-------------\n");
    // B3 生命周期: 到达视点但 frontier 仍在活跃列表中 → 累计尝试次数
    expl_manager_->frontier_finder_->incrementObservationAttempts(expl_manager_->ed_->frontier_to_explore_, fp_->max_observation_attempts_);
    ros::Duration(0.5).sleep();
    ROS_INFO_STREAM("t_cur: " << t_cur);
    resetExploreStuckState();
    fd_->last_progress_time_ = ros::Time::now();
    transitState(replan_target_state, "FSM");
    return;
  }

  // === 建议D: 探索路径 tier2 卡死强制推进 (复用自 goTargetObject, 插在 [5] 之前) ===
  // tier1(清blocked+重规划)已由 [5] 整体重规划覆盖, 此处仅做 tier2: 逐点 path_inx++ 跳过不可见点
  // 目的: 在 [5] 长时间卡死恢复之外, 增加"绕过当前 isVisible 失败的路径点"的细粒度恢复能力
  // 安全性: path_inx++ 后 getAndPublishNextAim 内部仍用严格 isVisible(OCCUPIED+UNKNOWN 双阻断)找下一可见点,
  //         不会绕过视线检查发布不可见目标点; 若下一点也不可见 → getLocalAim 返回 false → 走建议A失败计数
  if (fp_->explore_stuck_force_advance_enable_ && detectExploreStuck()) {
    double stuck_sec = (fd_->explore_stuck_begin_time_ >= 0.0)
                       ? ros::Time::now().toSec() - fd_->explore_stuck_begin_time_
                       : -1.0;
    if (stuck_sec > fp_->explore_stuck_force_advance_duration_ &&
        !fd_->explore_stuck_triggered_ &&
        fd_->explore_stuck_advance_count_ < fp_->explore_stuck_force_advance_max_consecutive_) {

      // Tier2: 逐点强制推进 path_inx (跳过当前 isVisible 失败的路径点)
      if (fd_->path_inx_ < (int)fd_->path_res_.size() - 1) {
        fd_->path_inx_++;
        fd_->explore_stuck_advance_count_++;
        fd_->explore_stuck_triggered_ = true;
        fd_->explore_stuck_begin_time_ = -1.0;
        bool ok = getAndPublishNextAim(fd_->path_res_, true);
        fd_->last_progress_time_ = ros::Time::now();  // 建议B: 强制推进算一次"推进"刷新计时
        ROS_WARN("[EXP-FSM] Stuck tier2: force advance path_inx=%d, count=%d, ok=%d",
                 fd_->path_inx_, fd_->explore_stuck_advance_count_, ok ? 1 : 0);
        return;
      } else {
        // 已是末点无法再推进 → 走 [5] 整体重规划
        ROS_WARN("[EXP-FSM] Stuck at final waypoint, fallback to periodic replan");
      }
    }
  }

  // [5] 兜底卡死恢复：长时间静止 → 强制重规划 (t_cur 基于 last_progress_time_, 建议B)
  if (t_cur > fp_->replan_thresh3_ && fd_->odom_vel_.norm() <= 0.1) {
    ROS_WARN("\n-------------> Replan: periodic stuck recovery (t_cur=%.1fs) <-------------\n", t_cur);
    resetExploreStuckState();
    fd_->last_progress_time_ = ros::Time::now();
    transitState(replan_target_state, "FSM");
    return;
  }

  // [6] Yaw 旋转（兜底：预到达未触发时才走到这里，避免死锁）
  // fallback: ego 未完成但位置极近+速度极低时也允许旋转
  if ((fd_->path_inx_ == fd_->path_res_.size() - 1 || fd_->path_res_.size() == 2) &&
      dis_2_aim_2d < expl_manager_->ep_->radius_close_ && !fd_->has_rotated_ &&
      (fd_->ego_exec_finished_ ||
       (dis_2_aim_2d < 0.5 && fd_->odom_vel_.norm() <= 0.3))) {
    INFO_MSG_CYAN("\n[Approach EXPLORE] Close to Aim Position, Rotate Yaw to Aim Yaw!\n");
    pubLocalGoal(fd_->aim_pos_, fd_->aim_yaw_, false,
                 quadrotor_msgs::EgoGoalSet::YAW_MODE_LOW_SPEED);
    fd_->has_rotated_ = true;
    fd_->last_progress_time_ = ros::Time::now();  // 建议B: 发布了新目标
    INFO_MSG_GREEN("[EXP-FSM] [Rotate Yaw] aim: " << fd_->aim_pos_.transpose() << ", local_aim: " << fd_->local_aim_pos_.transpose());
    return;
  }

  // [7] Waypoint 推进
  if (fd_->path_res_.size() > 2 && dis_2_local_aim < 2.0){
    // 建议C: 内部守卫解耦 ego 反馈 —— 增加"本地卡死+末点"的纯本地 fallback
    // 旧守卫要求 ego_exec_finished_ && ego_modify_status_, ego 断链时永不触发
    if (fd_->path_inx_ >= fd_->path_res_.size() - 1 &&
        ((fd_->ego_exec_finished_ && fd_->ego_modify_status_) ||     // 原条件(ego 反馈正常)
         (t_cur > fp_->explore_local_stuck_duration_ &&              // 建议C: ego 断链时的纯本地 fallback
          fd_->odom_vel_.norm() < fp_->explore_local_stuck_vel_thresh_))) {
      INFO_MSG_RED("\n[Approach EXPLORE] Force Replan, because local goal can't reach!\n");
      resetExploreStuckState();
      fd_->last_progress_time_ = ros::Time::now();
      transitState(MISSION_FSM_STATE::PLAN_EXPLORE, "can't reach local goal");
      return ;
    }
    // 建议A: 检查 getAndPublishNextAim 返回值, 失败时不刷新 last_progress_time_ 让 t_cur 累计
    bool published = getAndPublishNextAim(fd_->path_res_, true);
    if (published) {
      // 真正发布了新 local_goal → 刷新计时器, 重置失败计数与卡死状态
      fd_->last_progress_time_ = ros::Time::now();   // 建议B: 仅成功时刷新
      fd_->local_aim_fail_count_ = 0;                // 建议A: 成功清零
      resetExploreStuckState();
      ROS_INFO_STREAM_THROTTLE(1.0, "[EXP-FSM] [PubNxtLocalAim] aim: " << fd_->aim_pos_.transpose()
                              << ", local_aim: " << fd_->local_aim_pos_.transpose());
    } else {
      // 建议A: 发布失败 → 递增失败计数; 连续失败超阈值 → 强制重规划
      fd_->local_aim_fail_count_++;
      ROS_WARN("[EXP-FSM] getAndPublishNextAim FAILED (%d/%d), not refreshing timer",
               fd_->local_aim_fail_count_, fp_->explore_local_aim_fail_max_);
      if (fd_->local_aim_fail_count_ >= fp_->explore_local_aim_fail_max_) {
        ROS_WARN("[EXP-FSM] local_aim_fail_count >= max, force replan (local aim unreachable)");
        resetExploreStuckState();
        // 建议B: 不刷新 last_progress_time_, 让 [5] 也能在下一帧兜底
        transitState(replan_target_state, "local aim unreachable");
        return;
      }
      // 失败但未超阈值: 不刷新 last_progress_time_, t_cur 继续累计, 让 [5]/tier2 接管
    }
  }
}

void FastExplorationFSM::resetTrackingFinishCandidate() {
  fd_->track_finish_candidate_active_ = false;
  fd_->track_finish_candidate_start_time_ = ros::Time(0);
  fd_->track_finish_last_pos_ = fd_->odom_pos_;
  fd_->track_finish_last_yaw_ = fd_->odom_yaw_;
  fd_->track_finish_move_acc_ = 0.0;
  fd_->track_finish_yaw_acc_ = 0.0;
}

bool FastExplorationFSM::updateTrackingFinishCandidate(double dis_2_aim, double angle_2_aim) {
  const bool near_aim = dis_2_aim < fp_->arrive_dis_thr_;
  const bool yaw_ok = angle_2_aim < expl_manager_->ep_->track_yaw_thr_;
  if (!near_aim || !yaw_ok || fd_->track_finish_sent_) {
    resetTrackingFinishCandidate();
    return false;
  }

  const ros::Time now = ros::Time::now();
  if (!fd_->track_finish_candidate_active_) {
    fd_->track_finish_candidate_active_ = true;
    fd_->track_finish_candidate_start_time_ = now;
    fd_->track_finish_last_pos_ = fd_->odom_pos_;
    fd_->track_finish_last_yaw_ = fd_->odom_yaw_;
    fd_->track_finish_move_acc_ = 0.0;
    fd_->track_finish_yaw_acc_ = 0.0;
    ROS_INFO_STREAM("[TRACK] finish candidate start, pos=" << fd_->odom_pos_.transpose()
                    << ", yaw=" << fd_->odom_yaw_);
    return false;
  }

  fd_->track_finish_move_acc_ += (fd_->odom_pos_ - fd_->track_finish_last_pos_).norm();
  fd_->track_finish_yaw_acc_ += std::fabs(normalizeAngle(fd_->odom_yaw_ - fd_->track_finish_last_yaw_));
  fd_->track_finish_last_pos_ = fd_->odom_pos_;
  fd_->track_finish_last_yaw_ = fd_->odom_yaw_;

  if (fd_->track_finish_move_acc_ > fp_->track_finish_move_thresh_ ||
      fd_->track_finish_yaw_acc_ > fp_->track_finish_yaw_thresh_) {
    ROS_INFO_STREAM("[TRACK] finish candidate reset: move_acc=" << fd_->track_finish_move_acc_
                    << ", yaw_acc=" << fd_->track_finish_yaw_acc_);
    resetTrackingFinishCandidate();
    return false;
  }

  const double hold_time = (now - fd_->track_finish_candidate_start_time_).toSec();
  if (hold_time < fp_->track_finish_hold_time_) {
    ROS_INFO_STREAM_THROTTLE(0.5, "[TRACK] finish candidate holding: time=" << hold_time
                             << "/" << fp_->track_finish_hold_time_
                             << ", move_acc=" << fd_->track_finish_move_acc_
                             << ", yaw_acc=" << fd_->track_finish_yaw_acc_);
    return false;
  }

  return true;
}

void FastExplorationFSM::publishTrackingFinish() {
  if (fd_->track_finish_sent_) {
    return;
  }
  std_msgs::Bool msg;
  msg.data = true;
  tracking_finish_pub_.publish(msg);
  fd_->track_finish_sent_ = true;
  resetTrackingFinishCandidate();
  ROS_INFO("[TRACK] publish /tracking_finish=true");
}

bool FastExplorationFSM::useElasticTrackerBackend() const {
  return fp_->tracking_backend_ == "elastic_tracker" ||
         fp_->tracking_backend_ == "tracker";
}

void FastExplorationFSM::publishPlannerCmdMuxMode(const std::string& mode,
                                                  const std::string& source) {
  std_msgs::String msg;
  msg.data = mode;
  planner_cmd_mux_mode_pub_.publish(msg);
  ROS_INFO_STREAM_THROTTLE(0.5, "[planner_cmd_mux] request mode=" << mode
                           << " from " << source);
}

void FastExplorationFSM::switchPlannerCmdMuxToEgo(const std::string& source) {
  publishPlannerCmdMuxMode(fp_->planner_cmd_mux_ego_mode_, source);
}

void FastExplorationFSM::switchPlannerCmdMuxToElastic(const std::string& source) {
  publishPlannerCmdMuxMode(fp_->planner_cmd_mux_elastic_mode_, source);
}

void FastExplorationFSM::publishElasticTrackerTrigger(const ros::Time& stamp,
                                                      const std::string& frame_id) {
  geometry_msgs::PoseStamped trigger;
  trigger.header.stamp = stamp.isZero() ? ros::Time::now() : stamp;
  trigger.header.frame_id = frame_id.empty() ? "world" : frame_id;
  trigger.pose.position.x = 0.0;
  trigger.pose.position.y = 0.0;
  trigger.pose.position.z = 0.0;
  trigger.pose.orientation.w = 1.0;
  elastic_tracker_trigger_pub_.publish(trigger);
  ROS_INFO_STREAM_THROTTLE(0.5, "[TRACK] publish Elastic-Tracker trigger -> "
                           << fp_->elastic_tracker_trigger_topic_);
}

void FastExplorationFSM::stopElasticTracker(const std::string& source) {
  if (!useElasticTrackerBackend()) {
    return;
  }
  std_msgs::Empty msg;
  elastic_tracker_stop_pub_.publish(msg);
  ROS_INFO_STREAM("[TRACK] publish Elastic-Tracker stop -> "
                  << fp_->elastic_tracker_stop_topic_ << " from " << source);
}

void FastExplorationFSM::publishTrackingTargetOdom(const Eigen::Vector3d& target_pos,
                                                   const ros::Time& stamp,
                                                   const std::string& frame_id) {
  nav_msgs::Odometry target_odom;
  target_odom.header.stamp = stamp.isZero() ? ros::Time::now() : stamp;
  target_odom.header.frame_id = frame_id.empty() ? "world" : frame_id;
  target_odom.child_frame_id = "target";
  target_odom.pose.pose.position.x = target_pos.x();
  target_odom.pose.pose.position.y = target_pos.y();
  target_odom.pose.pose.position.z = target_pos.z();
  target_odom.pose.pose.orientation.w = 1.0;
  target_odom.twist.twist.linear.x = 0.0;
  target_odom.twist.twist.linear.y = 0.0;
  target_odom.twist.twist.linear.z = 0.0;
  tracking_target_odom_pub_.publish(target_odom);

  // 同时发布 PointStamped 到 target_ekf 供 KF 速度估计
  geometry_msgs::PointStamped target_3d;
  target_3d.header.stamp = target_odom.header.stamp;
  target_3d.header.frame_id = target_odom.header.frame_id;
  target_3d.point.x = target_pos.x();
  target_3d.point.y = target_pos.y();
  target_3d.point.z = target_pos.z();
  tracking_target_3d_pub_.publish(target_3d);

  ROS_INFO_STREAM_THROTTLE(0.5, "[TRACK] publish target pos for Elastic-Tracker: "
                           << target_pos.transpose() << " -> "
                           << fp_->tracking_target_odom_topic_
                           << " (3D -> " << fp_->tracking_target_3d_topic_ << ")");
}

void FastExplorationFSM::planTrack() {
  ROS_INFO("\033[1;31mPlan TRACK!\033[0m");

  if (!fd_->track_trigger_) {
    resetTrackingFinishCandidate();
    transitState(MISSION_FSM_STATE::WAIT_TRIGGER, "planTrack:no trigger");
    return;
  }

  if (!fd_->track_init_) {
    resetTrackingFinishCandidate();
    ROS_WARN_THROTTLE(1.0, "[TRACK] Wait tracking target initialization.");
    return;
  }

  const Eigen::Vector3d target_vec = fd_->track_pos_ - fd_->odom_pos_;
  if (target_vec.norm() < 1e-3) {
    resetTrackingFinishCandidate();
    ROS_WARN_THROTTLE(1.0, "[TRACK] Target too close to current position, skip planning.");
    return;
  }

  fd_->path_res_.clear();
  fd_->aim_pos_ = fd_->track_pos_ - target_vec.normalized() * expl_manager_->ep_->track_dist_;
  fd_->aim_pos_.z() = 1.0;
  fd_->aim_yaw_ = atan2(target_vec.y(), target_vec.x());

  ROS_INFO_STREAM_THROTTLE(0.5, "[TRACK] track pos: " << fd_->track_pos_.transpose()
                           << " aim pos: " << fd_->aim_pos_.transpose()
                           << " odom pos: " << fd_->odom_pos_.transpose()
                           << " aim yaw: " << fd_->aim_yaw_);

  const double pos_err = (fd_->aim_pos_ - fd_->odom_pos_).norm();
  const double yaw_err = std::fabs(normalizeAngle(fd_->aim_yaw_ - fd_->odom_yaw_));
  if (pos_err < expl_manager_->ep_->track_replan_dist_ &&
      yaw_err < expl_manager_->ep_->track_yaw_thr_) {
    if (updateTrackingFinishCandidate(pos_err, yaw_err)) {
      publishTrackingFinish();
      return;
    }
    ROS_WARN_THROTTLE(1.0, "[TRACK] Already close to tracking aim, skip planning.");
    fd_->local_aim_pos_ = fd_->aim_pos_;
    fd_->has_rotated_ = true;
    fd_->last_pub_time_ = ros::Time::now();
    transitState(MISSION_FSM_STATE::APPROACH_TRACK, "planTrack:already_close_hold");
    return;
  }

  const int res = callTrackPlanner(fd_->aim_pos_, fd_->aim_vel_, fd_->aim_yaw_, fd_->path_res_);
  if (res != SUCCEED) {
    resetTrackingFinishCandidate();
    ROS_WARN_THROTTLE(1.0, "[TRACK] Tracking target not directly reachable yet.");
    return;
  }

  fd_->path_inx_ = 0;
  fd_->local_aim_pos_ = fd_->aim_pos_;

  const double dis_2_aim_2d = (fd_->aim_pos_ - fd_->odom_pos_).head(2).norm();
  // 远距离 yaw 偏差过大时提前按目标方向边飞边转；近距离 yaw-lock 保持快速转向以尽快找回目标视野。
  const bool near_yaw_lock = dis_2_aim_2d < expl_manager_->ep_->track_turn_yaw_dist_;
  const bool far_yaw_align = !near_yaw_lock && yaw_err > expl_manager_->ep_->track_fly_yaw_thr_;
  const bool look_forward = !(near_yaw_lock || far_yaw_align);
  const bool yaw_low_speed = far_yaw_align;

  pubLocalGoal(fd_->path_res_.back(), fd_->aim_yaw_, look_forward,
               yaw_low_speed ? quadrotor_msgs::EgoGoalSet::YAW_MODE_LOW_SPEED
                             : quadrotor_msgs::EgoGoalSet::YAW_MODE_NORMAL);
  resetTrackingFinishCandidate();
  INFO_MSG_GREEN("[TRACK] [look_forward = " << look_forward
                 << ", yaw_low_speed = " << yaw_low_speed
                 << ", yaw_err = " << yaw_err << "] aim: "
                 << fd_->path_res_.back().transpose() << ", yaw: " << fd_->aim_yaw_);

  fd_->has_rotated_ = near_yaw_lock;
  fd_->last_pub_time_ = ros::Time::now();
  transitState(MISSION_FSM_STATE::APPROACH_TRACK, "planTrack");
}

void FastExplorationFSM::approachTrack() {
  ROS_INFO_STREAM_THROTTLE(0.5, "\033[1;33mApproach TRACK...\033[0m");

  if (!fd_->track_trigger_) {
    resetTrackingFinishCandidate();
    transitState(MISSION_FSM_STATE::WAIT_TRIGGER, "approachTrack:disable");
    return;
  }

  if (!fd_->track_init_) {
    resetTrackingFinishCandidate();
    transitState(MISSION_FSM_STATE::PLAN_TRACK, "approachTrack:wait target");
    return;
  }

  const double dis_2_aim = (fd_->aim_pos_ - fd_->odom_pos_).norm();
  const double dis_2_aim_2d = (fd_->aim_pos_ - fd_->odom_pos_).head(2).norm();
  const double dis_2_local_aim = (fd_->local_aim_pos_ - fd_->odom_pos_).norm();
  const double angle_2_aim = std::fabs(normalizeAngle(fd_->aim_yaw_ - fd_->odom_yaw_));
  const double t_cur = (ros::Time::now() - fd_->last_pub_time_).toSec();

  ROS_INFO_STREAM_THROTTLE(0.5, "[TRACK] Dis to aim: " << dis_2_aim_2d
                           << " local aim: " << dis_2_local_aim
                           << " yaw err: " << angle_2_aim
                           << " t_cur: " << t_cur);

  if (updateTrackingFinishCandidate(dis_2_aim, angle_2_aim)) {
    publishTrackingFinish();
    return;
  }

  if (t_cur > fp_->replan_thresh3_) {
    resetTrackingFinishCandidate();
    transitState(MISSION_FSM_STATE::PLAN_TRACK, "approachTrack:periodic");
    return;
  }

  const Eigen::Vector3d target_vec = fd_->track_pos_ - fd_->odom_pos_;
  if (target_vec.norm() < 1e-3) {
    resetTrackingFinishCandidate();
    transitState(MISSION_FSM_STATE::PLAN_TRACK, "approachTrack:target too close");
    return;
  }

  Eigen::Vector3d aim_pos_new = fd_->track_pos_ - target_vec.normalized() * expl_manager_->ep_->track_dist_;
  aim_pos_new.z() = 1.0;
  if ((fd_->aim_pos_ - aim_pos_new).norm() > expl_manager_->ep_->track_replan_dist_) {
    INFO_MSG_GREEN("[TRACK] aim_pos_old: " << fd_->aim_pos_.transpose()
                   << " aim_pos_new: " << aim_pos_new.transpose());
    resetTrackingFinishCandidate();
    transitState(MISSION_FSM_STATE::PLAN_TRACK, "approachTrack:moved");
    return;
  }

  const double current_dir = atan2(target_vec.y(), target_vec.x());
  if (!fd_->has_rotated_ && dis_2_aim_2d < expl_manager_->ep_->track_turn_yaw_dist_) {
    fd_->has_rotated_ = true;
    fd_->aim_yaw_ = current_dir;
    resetTrackingFinishCandidate();
    pubLocalGoal(fd_->aim_pos_, fd_->aim_yaw_, false,
                 quadrotor_msgs::EgoGoalSet::YAW_MODE_NORMAL);
    INFO_MSG_GREEN("[TRACK] Switch to yaw-lock, aim: " << fd_->aim_pos_.transpose()
                   << ", yaw: " << fd_->aim_yaw_);
    return;
  }

  if (fd_->has_rotated_ &&
      std::fabs(normalizeAngle(current_dir - fd_->aim_yaw_)) > expl_manager_->ep_->track_yaw_thr_) {
    fd_->aim_yaw_ = current_dir;
    resetTrackingFinishCandidate();
    pubLocalGoal(fd_->aim_pos_, fd_->aim_yaw_, false,
                 quadrotor_msgs::EgoGoalSet::YAW_MODE_NORMAL);
    INFO_MSG_GREEN("[TRACK] Update yaw-lock, aim: " << fd_->aim_pos_.transpose()
                   << ", yaw: " << fd_->aim_yaw_);
  }
}

void FastExplorationFSM::startPanoramaRotation() {
  // 全景旋转固定沿yaw正方向执行，odometry增量负责记录真实累计转角。
  panorama_last_odom_yaw_ = fd_->odom_yaw_;
  panorama_start_yaw_ = fd_->odom_yaw_;
  panorama_unwrapped_yaw_ = fd_->odom_yaw_;
  panorama_accumulated_yaw_ = 0.0;
  panorama_command_target_yaw_ = panorama_unwrapped_yaw_;
  panorama_hold_pos_ = fd_->odom_pos_;
  panorama_command_active_ = false;
  need_panorama_ = true;
  need_rotate_yaw_ = false;
  ROS_WARN("[FSM] Start panorama 360° rotation, start_yaw=%.2f°",
           panorama_unwrapped_yaw_ * 180.0 / M_PI);
}

bool FastExplorationFSM::waitForFreshMapAfterReset() {
  if (!wait_fresh_map_after_reset_)
    return false;

  const uint64_t current_update_seq = map_->getOccupancyUpdateSeq();
  if (current_update_seq <= map_reset_update_seq_) {
    ROS_WARN_THROTTLE(1.0, "[FSM] Waiting for the first occupancy update after map reset.");
    return true;
  }

  wait_fresh_map_after_reset_ = false;
  ROS_WARN("[FSM] Fresh occupancy received after map reset, start panorama rotation.");
  startPanoramaRotation();
  return false;
}

void FastExplorationFSM::handlePanoramaYaw() {
  constexpr double TOTAL_ANGLE = 2.0 * M_PI;
  constexpr double FINISH_TOLERANCE = 2.0 * M_PI / 180.0;
  constexpr double TARGET_SETTLE_TOLERANCE = 1.0 * M_PI / 180.0;

  const double remaining = std::max(0.0, TOTAL_ANGLE - panorama_accumulated_yaw_);
  const double final_target_error =
      fabs((panorama_start_yaw_ + TOTAL_ANGLE) - panorama_unwrapped_yaw_);
  if (remaining <= FINISH_TOLERANCE && final_target_error <= TARGET_SETTLE_TOLERANCE) {
    ROS_WARN("[FSM] Panorama 360° done, accumulated=%.2f°",
             panorama_accumulated_yaw_ * 180.0 / M_PI);
    panorama_command_active_ = false;
    need_panorama_ = false;
    // 全景期间 stopMotion 等正常 goal 的 trajectory 链路被 WAIT_YAW 打断，
    // ego FSM 从未到达 WAIT_TARGET，exec_finish_trigger 未发布导致 ego_exec_finished_ 为 false。
    // 全景结束后显式置 true，确保 planLLMExplore 能被 FSM 回调继续调用。
    fd_->ego_exec_finished_ = true;
    return;
  }

  if (panorama_command_active_)
  {
    const double angle_to_command = panorama_command_target_yaw_ - panorama_unwrapped_yaw_;
    const double commanded_angle = panorama_command_target_yaw_ - panorama_start_yaw_;
    const bool final_target_published = commanded_angle >= TOTAL_ANGLE - 1e-6;
    if (final_target_published || angle_to_command > panorama_extend_angle_)
      return;
  }

  // 每次最多向前延长配置角度，最终连续目标严格封顶为起始yaw+360°。
  const double commanded_angle =
      std::max(0.0, panorama_command_target_yaw_ - panorama_start_yaw_);
  const double next_commanded_angle =
      std::min(TOTAL_ANGLE, commanded_angle + panorama_max_step_);
  panorama_command_target_yaw_ = panorama_start_yaw_ + next_commanded_angle;
  ROS_INFO("[Panorama] publish target=%.1f°, accumulated=%.1f°, remaining=%.1f°",
           panorama_command_target_yaw_ * 180.0 / M_PI,
           panorama_accumulated_yaw_ * 180.0 / M_PI,
           remaining * 180.0 / M_PI);
  pubLocalGoal(panorama_hold_pos_, panorama_command_target_yaw_, false,
               quadrotor_msgs::EgoGoalSet::YAW_MODE_PANORAMA,
               quadrotor_msgs::EgoGoalSet::YAW_PATH_KEEP_DIRECTION);
  panorama_command_active_ = true;
}

void FastExplorationFSM::handleYawChange() {
  // 依次转向原始朝向+45°、-45°，最终回到原始朝向。
  if (!enable_yaw_scan_) {
    need_rotate_yaw_ = false;
    transitState(stash_state_, "Yaw Scan Disabled");
    return;
  }

  if (!fd_->ego_exec_finished_) {
    ROS_WARN_STREAM_THROTTLE(1.0, "[Handleyaw] : ego exec not finished, skip yaw handle ...");
    return ;
  }

  if (!yawhandle_left_ok) {
    if (abs(fd_->odom_yaw_ - yawhandle_yaw_target_left) < 0.05) yawhandle_left_ok = true;
    if (!yawhandle_left_published) {
      INFO_MSG("[HandleYaw] | Turn Left ...");
      yawhandle_left_published = true;
      pubLocalGoal(fd_->odom_pos_, yawhandle_yaw_target_left, false,
                   quadrotor_msgs::EgoGoalSet::YAW_MODE_LOW_SPEED);
    }
    return ;
  }

  if (!yawhandle_right_ok) {
    if (abs(fd_->odom_yaw_ - yawhandle_yaw_target_right) < 0.05) yawhandle_right_ok = true;
    if (!yawhandle_right_published) {
      ros::Duration(0.5).sleep();
      INFO_MSG("[HandleYaw] | Turn Right ...");
      yawhandle_right_published = true;
      pubLocalGoal(fd_->odom_pos_, yawhandle_yaw_target_right, false,
                   quadrotor_msgs::EgoGoalSet::YAW_MODE_LOW_SPEED);
    }
    return ;
  }

  if (!yawhandle_back_ok) {
    if (abs(fd_->odom_yaw_ - yawhandle_yaw_raw) < 0.05) yawhandle_back_ok = true;
    if (!yawhandle_back_published) {
      ros::Duration(0.5).sleep();
      INFO_MSG("[HandleYaw] | Back to raw ...");
      yawhandle_back_published = true;
      pubLocalGoal(fd_->odom_pos_, yawhandle_yaw_raw, false,
                   quadrotor_msgs::EgoGoalSet::YAW_MODE_LOW_SPEED);
    }
    return ;
  }

  yawhandle_left_published  = yawhandle_right_published = yawhandle_back_published = false;
  yawhandle_left_ok         = yawhandle_right_ok        = yawhandle_back_ok        = false;
  need_rotate_yaw_          = false;
  transitState(stash_state_, "Yaw Handle Done");
}

void FastExplorationFSM::FSMCallback(const ros::TimerEvent& e)
{
  auto perf_t0 = PERF_NOW();  // 性能日志：FSM 主循环计时起点
  // ROS_INFO_STREAM_THROTTLE(10.0, "** [EXP-FSM]: state: " << md_->state_str_[md_->mission_state_]);
  CALL_EVERY_N_TIMES(displayMissionState, 5);

  std_msgs::String fsm_state_str;
  if (md_->mission_state_ == FINISH || md_->mission_state_ == WAIT_TRIGGER) fsm_state_str.data = "FINISH";
  else fsm_state_str.data = "EXECING";
  fsm_state_pub_.publish(fsm_state_str);

  traj_visualizer_->addPoint(fd_->odom_pos_, fd_->odom_vel_.norm());

  switch (md_->mission_state_) 
  {
    case INIT: 
    {
      // Wait for odometry ready
      if (!fd_->have_odom_) {
        ROS_WARN_THROTTLE(2.0, "no odom.");
        return;
      }
      if (!map_->isInited()){
        ROS_WARN_THROTTLE(2.0, "no map.");
        return;
      } 
      // Go to wait trigger when odom is ok
      fd_->warmup_start_time_ = ros::Time::now();
      transitState(MISSION_FSM_STATE::WARM_UP, "FSM");
      break;
    }

    // warm up 10sec -> init skeleton -> start explore
    case WARM_UP:
    {
      if (fp_->auto_init_scene_graph_ && !fd_->trigger_) {
        const double warmup_elapsed = (ros::Time::now() - fd_->warmup_start_time_).toSec();
        if (warmup_elapsed >= fp_->auto_init_delay_sec_) {
          fd_->trigger_ = true;
        } else {
          ROS_INFO_THROTTLE(2.0, "Wait auto init delay before skeleton expand ... ");
          return;
        }
      }

      if (!fd_->trigger_) {
        ROS_INFO_THROTTLE(10.0, "Wait Trigger For Skeleton Expand ... ");
        return;
      }
      bool new_topo = false;
      fd_->trigger_ = false;
      Eigen::Vector3d init_seed;
      std::string init_block_reason;
      if (!getSceneGraphInitSeed(init_seed, &init_block_reason)) {
        ROS_WARN_STREAM_THROTTLE(2.0, "[EXP-FSM] Scene graph init seed is not ready: " << init_block_reason);
        return;
      }

      ROS_INFO_STREAM("[EXP-FSM] Init scene graph from forward seed: " << init_seed.transpose()
                      << " (yaw=" << fd_->odom_yaw_ << ")");
      scene_graph_->initSceneGraph(init_seed, fd_->odom_yaw_);
      scene_graph_->history_visited_area_ids_.push_back(0);

      if (scene_graph_->skeleton_gen_->ready()) {
        scene_graph_->object_factory_->runThisModule();
      }else {
        transitState(MISSION_FSM_STATE::INIT, "FSM-WARMUP");
        fd_->trigger_ = false;
        return;
      }

      transitState(MISSION_FSM_STATE::WAIT_TRIGGER, "FSM");
      expl_manager_->visualize(fd_->odom_pos_);
      break;
    }

    case WAIT_TRIGGER: 
    {
      // Do nothing but wait for trigger
      if (!fd_->trigger_) {
        ROS_WARN_THROTTLE(10.0, "[EXP-FSM] Wait Trigger For Plan Explore ... ");
        return ;
      }
      fd_->trigger_ = false;
      transitState(MISSION_FSM_STATE::LLM_PLAN_EXPLORE, "FSM");
      expl_manager_->visualize(fd_->odom_pos_);
      break;
    }

    case GO_TARGET_OBJECT: {
      goTargetObject();
      break;
    }

    case GO_TARGET_WITH_WAYPOINT: {
      goTargetWithWaypoint();
      break;
    }

    case FINISH: {
      ROS_INFO_STREAM_THROTTLE(10.0, "\033[1;31mFinish!\033[0m");  //红
      break;
    }

    case YAW_HANDLE: {
      handleYawChange();
      break;
    }

    case THINKING: {
      ROS_INFO_STREAM_THROTTLE(0.5, "\033[1;31mTHINKING!\033[0m");  //red
      handelThingkingProcess();
      break;
    }

    case LLM_PLAN_EXPLORE: {
      if (need_panorama_ || fd_->ego_exec_finished_)
        planLLMExplore();
      break;
    }

    case PLAN_EXPLORE: {
      planRegularExplore();
      break;
    }

    case PLAN_TRACK: {
      planTrack();
      break;
    }

    case APPROACH_EXPLORE: {
      approachRegularExplore();
      break;
    }

    case APPROACH_TRACK: {
      approachTrack();
      break;
    }

    case FIND_TERMINATE_TARGET:{
      findTerminateTarget();
      break;
    }

    case DF_DEMO: {
      execDFDemo();
      break;
    }

    case VLA_SWARM_PLAN_LOCAL: {
      handleVlaSwarmPlanLocal();
      break;
    }

    case VLA_SWARM_WAIT_LLM: {
      handleVlaSwarmWaitLLM();
      break;
    }

    case VLA_SWARM_WAIT_TARGET: {
      handleVlaSwarmWaitTarget();
      break;
    }

    case VLA_SWARM_APPROACH: {
      handleVlaSwarmApproach();
      break;
    }

    case VLA_SWARM_YAW_HANDLE: {
      handleVlaSwarmYaw();
      break;
    }

    case VLA_SWARM_RECOVERY: {
      handleVlaSwarmRecovery();
      break;
    }

    case VLA_SWARM_FINISH: {
      handleVlaSwarmFinish();
      break;
    }

    default:{
      break;
    }

  }

  // 性能日志：记录 FSM 主循环总耗时和当前状态
  // （提前 return 的状态如 INIT/WARM_UP 不记录，因为不涉及规划）
  PERF_LOG_ELAPSED_EX("FSM_CB", perf_t0, "state=" + md_->state_str_[md_->mission_state_]);
}

void FastExplorationFSM::goTargetObject() {
  if (fd_->go_object_process_phase == 0) {
    scene_graph_->mountCurPoly(fd_->odom_pos_, fd_->odom_yaw_);
    if (scene_graph_->getPathToObjectWithId(fd_->object_target_id_, fd_->path_res_, fd_->aim_pos_, fd_->aim_yaw_)) {
      INFO_MSG_GREEN("[Targ Obj] | find path to object success, size: " << fd_->path_res_.size());

      fd_->has_rotated_     = false;
      fd_->stuck_begin_time_ = -1.0;                  // 新路径生成时重置卡死计时
      fd_->stuck_force_advance_count_ = 0;             // 新路径生成时重置强制推进计数
      fd_->stuck_force_advance_triggered_ = false;
      fd_->last_pub_time_   = ros::Time::now();
      fd_->go_object_in_prior_guide_ = false;          // 精确导航
      INFO_MSG_CYAN("[Targ Obj] | PubNxtLocalAim, aim: " << fd_->local_aim_pos_ << ", global aim: " << fd_->aim_pos_);
      getAndPublishNextAim(fd_->path_res_, true, fd_->aim_yaw_);

      displayPath();
      fd_->go_object_process_phase ++;
    }else {
      fd_->go_object_process_phase = 0;
      if(fd_->find_terminate_target_mode_) {
        // === 先验引导: cloud未构建(polyhedron_father==null)时, 用obj近似位置找最近poly做粗导航 ===
        auto obj_map = scene_graph_->object_factory_->object_map_;
        auto obj_it = obj_map.find(fd_->object_target_id_);
        if (obj_it != obj_map.end()) {
          ObjectNode::Ptr obj = obj_it->second;
          if (obj->edge.polyhedron_father == nullptr &&
              fd_->go_object_prior_guide_count_ < fp_->object_id_nav_prior_guide_max_retries_) {
            PolyHedronPtr nearest = scene_graph_->skeleton_gen_->getFrontierTopo(obj->pos);
            if (nearest != nullptr && scene_graph_->cur_poly_ != nullptr &&
                nearest != scene_graph_->cur_poly_) {
              double dis = scene_graph_->skeleton_gen_->astarSearch(
                  scene_graph_->cur_poly_, nearest, fd_->path_res_);
              if (!fd_->path_res_.empty() && dis < 99998.0) {
                fd_->aim_pos_ = nearest->center_;
                Eigen::Vector3d dxy = nearest->center_ - obj->pos;
                double aim_direction = atan2(dxy(1), dxy(0)) + M_PI;
                if (aim_direction > M_PI)  aim_direction -= 2 * M_PI;
                if (aim_direction < -M_PI) aim_direction += 2 * M_PI;
                fd_->aim_yaw_ = aim_direction;
                fd_->has_rotated_ = false;
                fd_->stuck_begin_time_ = -1.0;
                fd_->stuck_force_advance_count_ = 0;
                fd_->stuck_force_advance_triggered_ = false;
                fd_->last_pub_time_ = ros::Time::now();
                fd_->go_object_in_prior_guide_ = true;   // 标记为先验引导
                fd_->go_object_prior_guide_count_++;
                getAndPublishNextAim(fd_->path_res_, true, fd_->aim_yaw_);
                displayPath();
                fd_->go_object_process_phase++;
                INFO_MSG_GREEN("[Targ Obj] prior guidance #" << (int)fd_->go_object_prior_guide_count_
                               << ": obj at " << obj->pos.transpose()
                               << " -> nearest poly area " << nearest->area_id_);
                return;
              }
            }
          }
        }
        publishExplorationResult(false, "target_path_failed", "failed to plan path to target object");
        transitState(FINISH, "** FIND TERMINATE TARGET PATH FAILED **");
      }
      else transitState(WAIT_TRIGGER, "** FIND OBJECT PATH FAILED **");
    }
  }

  if (fd_->go_object_process_phase == 1) {
    double dis_2_aim_2d    = (fd_->aim_pos_       - fd_->odom_pos_).head(2).norm();
    double dis_2_local_aim = (fd_->local_aim_pos_ - fd_->odom_pos_).norm();
    double dis_yaw         = abs(fd_->aim_yaw_ - fd_->odom_yaw_);
    double t_cur = (ros::Time::now() - fd_->last_pub_time_).toSec();
    std::string ego_plan_status_str_   = fd_->ego_plan_status_ ? "True" : "False";
    std::string ego_modify_status_str_ = fd_->ego_modify_status_ ? "True" : "False";
    ROS_INFO_STREAM_THROTTLE(0.5, "\033[1;33mApproach Object...\033[0m \n"
                                  "   * Dis to Aim: " << dis_2_aim_2d << "\n"
                                  "   * Dis to LocalAim: " << dis_2_local_aim << "\n"
                                  "   * Dis to yaw: " << dis_yaw);  // 黄
    ROS_INFO_STREAM_THROTTLE(0.5, "[Targ Obj] : ego local goal -> (" << fd_->ego_local_goal_.transpose() << ")");
    ROS_INFO_STREAM_THROTTLE(0.5, "[Targ Obj] : ego plan times: " << fd_->ego_plan_times_
                                                                  << "  ego plan statue: " << ego_plan_status_str_
                                                                  << "  ego modify status: " << ego_modify_status_str_);

    displayLocalAim();  // 橙色marker标记当前导航点

    bool pos_finish = (dis_2_aim_2d < fp_->replan_dis_thresh_);
    bool yaw_finish = !fp_->object_id_nav_require_final_yaw_ ||
                      (fabs(fd_->odom_yaw_ - fd_->aim_yaw_) / M_PI * 180.0 < 5.0);
    if (pos_finish && yaw_finish) {
      ROS_WARN("-------------> Finish: [Reach Aim] <-------------");
      ROS_INFO_STREAM("t_cur: " << t_cur);
      fd_->go_object_process_phase = 0;
      if (fd_->find_terminate_target_mode_) {
        if (fd_->go_object_in_prior_guide_) {
          // 先验引导到达: 已靠近obj, cloud大概率已构建, 回phase0走精确导航
          INFO_MSG_GREEN("[Targ Obj] prior guidance #" << (int)fd_->go_object_prior_guide_count_
                         << " arrived, retry getPathToObjectWithId");
          return;  // phase已归零, 下次tick重试 (prior guidance或正常路径)
        }
        fd_->go_object_prior_guide_count_ = 0;
        publishExplorationResult(true, "target_found", "reached target object");
        transitState(FINISH, "Find Terminate Target Finish");
      }
      else transitState(WAIT_TRIGGER, "Go Target Object Finish");
      return;
    }

    // crash recovery 1: 末尾 yaw 旋转不到位时重新下发旋转指令
    if (fp_->object_id_nav_require_final_yaw_ &&
        fd_->ego_exec_finished_ && fd_->ego_modify_status_
         && (dis_2_local_aim > 1.0 || dis_yaw > 10.0f / 180.0f * M_PI)
         && fd_->path_inx_ == fd_->path_res_.size() - 1){
      fd_->last_pub_time_ = ros::Time::now();
      ROS_WARN("-------------> RePublish LocalGoal: crash recovery, forcely rotate yaw<----------------");
      pubLocalGoal(fd_->odom_pos_, fd_->aim_yaw_, false,
                   quadrotor_msgs::EgoGoalSet::YAW_MODE_NORMAL);
      // INFO_MSG_GREEN("[Targ Obj] [PubNxtLocalAim] aim: " << fd_->aim_pos_.transpose() << ", local_aim: " << fd_->local_aim_pos_.transpose());
    }

    // Replan after some time
    if (t_cur > fp_->replan_thresh3_ && fd_->odom_vel_.norm() <= 0.1) {
      ROS_WARN("-------------> Replan: periodic call <-------------");
      ROS_WARN("t_cur: %f s", t_cur);
      fd_->go_object_process_phase = 0;
      transitState(WAIT_TRIGGER, "Go Target Object Replan");
      return;
    }

    // ========== 先replan后topo-block: replan耗尽或mode2超时后fallthrough到topo-block ==========
    bool fallthrough_to_topo = false;
    if (fp_->object_id_nav_replan_enable_) {

      // 统一卡死检测(所有mode共用)
      double vel_norm = fd_->odom_vel_.norm();
      double yaw_rate = fabs(fd_->odom_yaw_rate_);
      bool is_stuck = (vel_norm < fp_->object_id_nav_replan_stuck_vel_thresh_ &&
                       yaw_rate < fp_->object_id_nav_replan_stuck_yaw_rate_thresh_);
      if (is_stuck) {
        if (fd_->object_id_nav_replan_stuck_begin_time_ < 0.0) {
          fd_->object_id_nav_replan_stuck_begin_time_ = ros::Time::now().toSec();
        }
      } else {
        fd_->object_id_nav_replan_stuck_begin_time_ = -1.0;  // 有运动, 重置
      }
      double stuck_sec = (fd_->object_id_nav_replan_stuck_begin_time_ >= 0.0)
                         ? ros::Time::now().toSec() - fd_->object_id_nav_replan_stuck_begin_time_
                         : -1.0;

      // ---- Mode 0/1: 卡死自动replan ----
      if (fp_->object_id_nav_replan_mode_ == 0 || fp_->object_id_nav_replan_mode_ == 1) {
        if (stuck_sec > fp_->object_id_nav_replan_stuck_duration_) {
          int max_cnt = fp_->object_id_nav_replan_stuck_max_consecutive_;
          if (max_cnt > 0 && fd_->object_id_nav_replan_stuck_count_ >= max_cnt) {
            ROS_WARN("[ObjIdNavReplan] Stuck replan max reached (count=%d, max=%d), fallback to topo-block",
                     fd_->object_id_nav_replan_stuck_count_, max_cnt);
            fallthrough_to_topo = true;
          } else {
            ROS_WARN("[ObjIdNavReplan] Stuck detected (%.1fs), triggering replan", stuck_sec);
            triggerObjectIdNavReplan("stuck_detected");
            return;
          }
        }
      }

      // ---- Mode 0/2: 话题手动replan ----
      if ((fp_->object_id_nav_replan_mode_ == 0 || fp_->object_id_nav_replan_mode_ == 2) &&
          fd_->object_id_nav_replan_topic_triggered_) {
        if (!fd_->has_stored_object_id_nav_instruction_) {
          ROS_WARN("[ObjIdNavReplan] Topic trigger ignored: no stored instruction");
          fd_->object_id_nav_replan_topic_triggered_ = false;
        } else {
          ROS_WARN("[ObjIdNavReplan] Topic trigger received, triggering replan");
          triggerObjectIdNavReplan("topic_triggered");
          return;
        }
      }

      // ---- Mode 2: 卡死超时fallback到topo-block ----
      if (fp_->object_id_nav_replan_mode_ == 2 &&
          stuck_sec > fp_->object_id_nav_replan_mode2_stuck_fallback_delay_) {
        ROS_WARN("[ObjIdNavReplan] Mode2 stuck %.1fs > fallback delay %.1fs, fallback to topo-block",
                 stuck_sec, fp_->object_id_nav_replan_mode2_stuck_fallback_delay_);
        fallthrough_to_topo = true;
      }

    }

    // ---- topo-block 卡死强制推进(兜底逻辑) ----
    // 仅当 replan 未启用 或 replan 已耗尽(fallthrough) 时执行
    // 分层策略: tier1=强制重规划topo路径(清除blocked), tier2=逐点强制推进path_inx++
    bool run_topo = !fp_->object_id_nav_replan_enable_ || fallthrough_to_topo;
    if (run_topo && fp_->stuck_force_advance_enable_) {
      double vel_norm = fd_->odom_vel_.norm();
      double yaw_rate = fabs(fd_->odom_yaw_rate_);
      if (vel_norm < fp_->stuck_force_advance_vel_thresh_ &&
          yaw_rate < fp_->stuck_force_advance_yaw_rate_thresh_) {
        if (fd_->stuck_begin_time_ < 0.0) {
          fd_->stuck_begin_time_ = ros::Time::now().toSec();
        }
        double stuck_duration = ros::Time::now().toSec() - fd_->stuck_begin_time_;
        if (stuck_duration > fp_->stuck_force_advance_duration_ &&
            !fd_->stuck_force_advance_triggered_ &&
            fd_->stuck_force_advance_count_ < fp_->stuck_force_advance_max_consecutive_) {

          // Tier1: 首次卡死 → 清除blocked标记后内联重规划topo路径(类似强制重启任务)
          if (fd_->stuck_force_advance_count_ == 0) {
            ROS_WARN("[Targ Obj] Stuck tier1: force topo replan (clear blocked + regenerate path)");
            scene_graph_->clearAllBlocked();
            scene_graph_->mountCurPoly(fd_->odom_pos_, fd_->odom_yaw_);
            if (scene_graph_->getPathToObjectWithId(fd_->object_target_id_,
                    fd_->path_res_, fd_->aim_pos_, fd_->aim_yaw_)) {
              INFO_MSG_GREEN("[Targ Obj] Stuck tier1: new path found, size: " << fd_->path_res_.size());
              fd_->path_inx_ = 0;
              getAndPublishNextAim(fd_->path_res_, true, fd_->aim_yaw_);
              displayPath();
              fd_->stuck_force_advance_count_++;
              fd_->stuck_force_advance_triggered_ = true;
              fd_->stuck_begin_time_ = -1.0;
              fd_->last_pub_time_ = ros::Time::now();
            } else {
              // tier1 重规划失败 → 跳过tier1直接进入tier2逻辑
              ROS_WARN("[Targ Obj] Stuck tier1 failed (no path), fallback to tier2");
              fd_->stuck_force_advance_count_ = 1;  // 直接标记为已消耗tier1配额
              // 不设置triggered, 让下一轮立即进入tier2判定
            }
          } else {
            // Tier2: 二次卡死 → 逐点强制推进(当前逻辑)
            if (fd_->path_inx_ < (int)fd_->path_res_.size() - 1) {
              fd_->path_inx_++;
              fd_->stuck_force_advance_count_++;
              fd_->stuck_force_advance_triggered_ = true;
              fd_->stuck_begin_time_ = -1.0;
              getAndPublishNextAim(fd_->path_res_, true, fd_->aim_yaw_);
              fd_->last_pub_time_ = ros::Time::now();
              ROS_WARN("[Targ Obj] Stuck tier2: force advance path_inx=%d, count=%d",
                       fd_->path_inx_, fd_->stuck_force_advance_count_);
            } else {
              // 已是最后一个点无法再推进 → 走全局重规划
              ROS_WARN("[Targ Obj] Stuck at final waypoint, forced replan");
              fd_->go_object_process_phase = 0;
              transitState(WAIT_TRIGGER, "stuck at final waypoint, forced replan");
              return;
            }
          }
        }
      } else {
        fd_->stuck_begin_time_ = -1.0;             // 有运动, 重置计时器
        fd_->stuck_force_advance_triggered_ = false; // 运动表示脱离原卡死状态, 允许下次再触发
      }
    }

    // Close to aim, rotate yaw (仅 require_final_yaw_=true 时执行)
    if (fp_->object_id_nav_require_final_yaw_ &&
        (fd_->path_inx_ >= fd_->path_res_.size() - 1 || fd_->path_res_.size() == 2) &&
        dis_2_aim_2d < expl_manager_->ep_->radius_close_ && !fd_->has_rotated_ && fd_->ego_exec_finished_){
      INFO_MSG_GREEN("[TARG Obj] [Rotate Yaw] yaw: " << fd_->odom_yaw_ << ", target yaw: " << fd_->aim_yaw_
          << ", err : " << (fd_->odom_yaw_ - fd_->aim_yaw_) / 3.14 * 180.0f << "deg");

      auto cur_obj = scene_graph_->object_factory_->object_map_[fd_->object_target_id_];
      Eigen::Vector3d target_pos = fd_->path_res_.back();
      target_pos[2] =  adjustTerminateHeightFindingObject(cur_obj, fd_->aim_pos_, true);
      pubLocalGoal(target_pos, fd_->aim_yaw_, false,
                   quadrotor_msgs::EgoGoalSet::YAW_MODE_LOW_SPEED);
      fd_->has_rotated_ = true;
      return;
    }

    // Local goal
    if (fd_->path_res_.size() > 2 && dis_2_local_aim < 1.5){
      if (fd_->path_inx_ == fd_->path_res_.size() - 1 && dis_2_local_aim < 1.0 
          && fd_->ego_exec_finished_ && fd_->ego_modify_status_) {  
        INFO_MSG_YELLOW("[TARG Obj] Force Replan, because local goal can't reach!");
        fd_->go_object_process_phase = 0;
        transitState(MISSION_FSM_STATE::WAIT_TRIGGER, "can't reach local goal");
        return ;
      }
      getAndPublishNextAim(fd_->path_res_, true, fd_->aim_yaw_);
      fd_->stuck_force_advance_count_ = 0;       // 正常推进时重置卡死强制推进计数
      fd_->stuck_force_advance_triggered_ = false;
      fd_->object_id_nav_replan_stuck_begin_time_ = -1.0;  // 正常推进时重置新replan卡死计时
      fd_->object_id_nav_replan_stuck_count_ = 0;           // 正常推进时重置replan计数
      fd_->last_pub_time_ = ros::Time::now();
      // INFO_MSG_GREEN("[TARG Obj] [PubNxtLocalAim] aim: " << fd_->aim_pos_.transpose() << ", local_aim: " << fd_->local_aim_pos_.transpose());
    }
  }
}


void FastExplorationFSM::goTargetWithWaypoint() {
  if (fd_->go_waypoint_process_phase == 0) {
    scene_graph_->mountCurPoly(fd_->odom_pos_, fd_->odom_yaw_);

    if (scene_graph_->getCurPoly() == nullptr) {
      fd_->go_waypoint_process_phase = 0;
      transitState(WAIT_TRIGGER, "** FIND WAYPOINT TOPO FAILED: CUR POLY NULL **");
      return;
    }

    // === fly-origin 辅助返航: 优先使用已持久化绑定的 polyhedron 作为 topo A* 终点 ===
    // fly-origin 是一种虚拟 object, 在 topo 图上持久化绑定 "返航起点" 所在的稳定 polyhedron,
    // 避免每次返航都依赖脆弱的实时 mountCurTopoPoint 吸附 (origin 落在未建图区域会失败)
    PolyHedronPtr target_poly = nullptr;
    bool fly_origin_found = false;
    bool need_register_fly_origin = false;

    ObjectNode::Ptr fly_origin = scene_graph_->findFlyOrigin();
    if (fly_origin != nullptr && fly_origin->edge.polyhedron_father != nullptr) {
      // 已有 fly-origin: 复用其绑定的 polyhedron 作为 A* 终点
      target_poly = fly_origin->edge.polyhedron_father;
      fly_origin_found = true;
      INFO_MSG_GREEN("[Targ Wpt] | reuse fly-origin bound polyhedron, center: "
                     << target_poly->center_.transpose());
    } else {
      // 无 fly-origin 或绑定已失效: 走原 mountCurTopoPoint 吸附逻辑, 成功后挂载 fly-origin
      target_poly = scene_graph_->skeleton_gen_->mountCurTopoPoint(fd_->waypoint_target_, true);
      need_register_fly_origin = (target_poly != nullptr);
    }

    // 发布 fly-origin 状态日志: found=true 表示复用已存在挂载, false 表示本次新挂载 (或吸附失败)
    scene_graph_->publishFlyOriginStatus(fly_origin_found);

    if (target_poly == nullptr) {
      fd_->go_waypoint_process_phase = 0;
      transitState(WAIT_TRIGGER, "** FIND WAYPOINT TOPO FAILED: TARGET POLY NULL **");
      return;
    }

    fd_->path_res_.clear();
    const double topo_dis =
        scene_graph_->skeleton_gen_->astarSearch(scene_graph_->getCurPoly(), target_poly, fd_->path_res_);
    if (topo_dis >= 99999.0 || fd_->path_res_.empty()) {
      fd_->go_waypoint_process_phase = 0;
      transitState(WAIT_TRIGGER, "** FIND WAYPOINT TOPO PATH FAILED **");
      return;
    }

    if ((fd_->path_res_.back() - fd_->waypoint_target_).norm() > 1e-3) {
      fd_->path_res_.push_back(fd_->waypoint_target_);
    }

    // 吸附成功后立即挂载 fly-origin (决策 1: 返航开始时即挂载)
    // 即使后续 Phase 1 失败, 下次返航仍可复用此绑定; 失效检测由下次查询时的 polyhedron 有效性兜底
    if (need_register_fly_origin) {
      scene_graph_->registerFlyOriginAtPoly(fd_->waypoint_target_, target_poly);
    }

    fd_->aim_pos_ = fd_->waypoint_target_;
    fd_->aim_yaw_ = fd_->waypoint_target_yaw_;

    INFO_MSG_GREEN("[Targ Wpt] | find path to waypoint success, size: " << fd_->path_res_.size());
    getAndPublishNextAim(fd_->path_res_, true, fd_->aim_yaw_);
    fd_->path_inx_      = 0;
    fd_->has_rotated_   = false;
    fd_->last_pub_time_ = ros::Time::now();
    INFO_MSG("[Targ Wpt] | PubNxtLocalAim, aim: " << fd_->local_aim_pos_ << ", global aim: " << fd_->aim_pos_);

    displayPath();
    fd_->go_waypoint_process_phase++;
  }

  if (fd_->go_waypoint_process_phase == 1) {
    double dis_2_aim_2d    = (fd_->aim_pos_       - fd_->odom_pos_).head(2).norm();
    double dis_2_local_aim = (fd_->local_aim_pos_ - fd_->odom_pos_).norm();
    double dis_yaw         = abs(fd_->aim_yaw_ - fd_->odom_yaw_);
    double t_cur = (ros::Time::now() - fd_->last_pub_time_).toSec();
    std::string ego_plan_status_str_   = fd_->ego_plan_status_ ? "True" : "False";
    std::string ego_modify_status_str_ = fd_->ego_modify_status_ ? "True" : "False";
    ROS_INFO_STREAM_THROTTLE(0.5, "\033[1;33mApproach Waypoint...\033[0m \n"
                                  "   * Dis to Aim: " << dis_2_aim_2d << "\n"
                                  "   * Dis to LocalAim: " << dis_2_local_aim << "\n"
                                  "   * Dis to yaw: " << dis_yaw);
    ROS_INFO_STREAM_THROTTLE(0.5, "[Targ Wpt] : ego local goal -> (" << fd_->ego_local_goal_.transpose() << ")");
    ROS_INFO_STREAM_THROTTLE(0.5, "[Targ Wpt] : ego plan times: " << fd_->ego_plan_times_
                                                                  << "  ego plan statue: " << ego_plan_status_str_
                                                                  << "  ego modify status: " << ego_modify_status_str_);

    if (dis_2_aim_2d < fp_->replan_dis_thresh_ && fabs(fd_->odom_yaw_ - fd_->aim_yaw_) / 3.14 * 180.0f < 5.0) {
      ROS_WARN("-------------> Finish: [Reach Both Pos&Yaw Aim] <-------------");
      ROS_INFO_STREAM("t_cur: " << t_cur);
      fd_->go_waypoint_process_phase = 0;
      transitState(WAIT_TRIGGER, "Go Target Waypoint Finish");
      return;
    }

    if (fd_->ego_exec_finished_ && fd_->ego_modify_status_
         && (dis_2_local_aim > 1.0 || dis_yaw > 10.0f / 180.0f * M_PI)
         && fd_->path_inx_ == fd_->path_res_.size() - 1) {
      fd_->last_pub_time_ = ros::Time::now();
      ROS_WARN("-------------> RePublish LocalGoal: crash recovery, forcely rotate yaw<----------------");
      pubLocalGoal(fd_->odom_pos_, fd_->aim_yaw_, false,
                   quadrotor_msgs::EgoGoalSet::YAW_MODE_NORMAL);
    }

    if (t_cur > fp_->replan_thresh3_ && fd_->odom_vel_.norm() <= 0.1) {
      ROS_WARN("-------------> Replan: periodic call <-------------");
      ROS_WARN("t_cur: %f s", t_cur);
      fd_->go_waypoint_process_phase = 0;
      transitState(WAIT_TRIGGER, "Go Target Waypoint Replan");
      return;
    }

    if ((fd_->path_inx_ >= fd_->path_res_.size() - 1 || fd_->path_res_.size() == 2) &&
        dis_2_aim_2d < expl_manager_->ep_->radius_close_ && !fd_->has_rotated_ && fd_->ego_exec_finished_) {
      INFO_MSG_GREEN("[TARG Wpt] [Rotate Yaw] yaw: " << fd_->odom_yaw_ << ", target yaw: " << fd_->aim_yaw_
          << ", err : " << (fd_->odom_yaw_ - fd_->aim_yaw_) / 3.14 * 180.0f << "deg");

      pubLocalGoal(fd_->aim_pos_, fd_->aim_yaw_, false,
                   quadrotor_msgs::EgoGoalSet::YAW_MODE_LOW_SPEED);
      fd_->has_rotated_ = true;
      return;
    }

    if (fd_->path_res_.size() > 2 && dis_2_local_aim < 2.0) {
      if (fd_->path_inx_ == fd_->path_res_.size() - 1 && dis_2_local_aim < 1.0
          && fd_->ego_exec_finished_ && fd_->ego_modify_status_) {
        INFO_MSG_YELLOW("[TARG Wpt] Force Replan, because local goal can't reach!");
        fd_->go_waypoint_process_phase = 0;
        transitState(MISSION_FSM_STATE::WAIT_TRIGGER, "can't reach local goal");
        return;
      }
      getAndPublishNextAim(fd_->path_res_, true, fd_->aim_yaw_);
      fd_->last_pub_time_ = ros::Time::now();
    }
  }
}

void FastExplorationFSM::execDFDemo() {
  if(fd_->df_demo_phase_ == 0){

    if (fd_->df_demo_target_id_ >= 0){
      fd_->explore_count_ = 0;
      fd_->df_demo_phase_ = 1;
      fd_->df_demo_mode_  = false;
      fd_->object_target_id_ = fd_->df_demo_target_id_;
      transitState(MISSION_FSM_STATE::GO_TARGET_OBJECT, "DF Demo[2] go target obj");
      return;
    }

    if (fd_->explore_count_ == 0 && fd_->df_demo_target_id_ == -100){
      std::string prompt;
      scene_graph_->DFDemoPromptGen(prompt);
      scene_graph_->sendPrompt(scene_graph_->getCurPromptIdAndPlusOne(),
                               scene_graph::PromptMsg::PROMPT_TYPE_DF_DEMO,
                               prompt, std::chrono::seconds(10), 1);
      think_start_time_ = ros::Time::now().toSec();
      think_duration_limit_ = 10.0 * 1.0;
      transitState(MISSION_FSM_STATE::THINKING, "DF Demo[1] llm call");
      return ;
    }

    if(fd_->explore_count_ > 2){
      fd_->explore_count_     = 0;
      fd_->df_demo_target_id_ = -100;
      return ;
    }
    
    if (fd_->df_demo_target_id_ == -1){

      int cur_area_id = scene_graph_->cur_poly_->area_id_;
      if (scene_graph_->skeleton_gen_->area_handler_->areas_need_predict_[cur_area_id]){
        INFO_MSG_CYAN("[DF Demo] | current area need llm predict, reset explore count");
        fd_->explore_count_ = 0;
        fd_->df_demo_target_id_ = -100;
        return ;
      } 

      transitState(MISSION_FSM_STATE::PLAN_EXPLORE, "DF Demo[1] plan explore");
      INFO_MSG_CYAN("Do Regular Explore iter [" << fd_->explore_count_ << "]");
      fd_->explore_count_ ++;
      return ;
    }
  }
}

void FastExplorationFSM::findTerminateTarget(){
  fd_->go_object_process_phase    = 0;
  fd_->go_object_in_prior_guide_   = false;  // 新任务重置先验引导标记
  fd_->go_object_prior_guide_count_ = 0;    // 新任务重置先验引导计数
  fd_->find_terminate_target_mode_ = true;

  // 非THINKING模式: 直接从VLM检测结果中按label匹配目标物体
  if (!fp_->object_id_nav_use_thinking_) {
    std::string target_label = fd_->target_cmd_;
    int best_id = -1;
    int best_count = 0;
    for (const auto& obj_pair : scene_graph_->object_factory_->object_map_) {
      const auto& obj = obj_pair.second;
      if (obj->label == target_label && (int)obj->detection_count > best_count) {
        best_id = obj->id;
        best_count = obj->detection_count;
      }
    }
    if (best_id >= 0) {
      fd_->object_target_id_ = best_id;
      INFO_MSG_GREEN("[FSM] Direct match (no thinking): target='" << target_label
                     << "' id=" << best_id << " detections=" << best_count);
      transitState(MISSION_FSM_STATE::GO_TARGET_OBJECT, "Direct object match (no thinking)");
    } else {
      publishExplorationResult(false, "target_not_found",
                               "no object matching '" + target_label + "' in detections");
      transitState(MISSION_FSM_STATE::FINISH, "No matching object (no thinking)");
    }
    return;
  }

  // THINKING模式: 通过LLM识别目标物体
  std::string prompt;
  scene_graph_->chooseTerminateObjIdPromptGen(prompt);
  scene_graph_->sendPrompt(scene_graph_->getCurPromptIdAndPlusOne(),
                           scene_graph::PromptMsg::PROMPT_TYPE_TERMINATE_OBJ_ID,
                           prompt, std::chrono::seconds(10), 1);
  stashCurStateAndTransit(MISSION_FSM_STATE::THINKING, "llm terminate obj plan !");
  think_start_time_ = ros::Time::now().toSec();
  think_duration_limit_ = 10.0 * 1.0;
  return ;
}

double FastExplorationFSM::adjustTerminateHeightFindingObject(ObjectNode::Ptr target_obj, Eigen::Vector3d init_pos, bool final_point){
  // 根据物体的高度，飞机观测物体的xy坐标以及理想观测角度来确定终止高度，并通过安全性检查对高度进行上下微调
  double obj_height             = target_obj->pos.z();
  double observe_angle          = 0.0f / 180.0f * M_PI; // radians
  double observe_xy_distance    = (target_obj->edge.polyhedron_father->center_.head<2>() - target_obj->pos.head<2>()).norm();
  double ideal_terminate_height = obj_height + tan(observe_angle) * observe_xy_distance; // 2.0m away in xy plane
  double ideal_poly_height      = init_pos.z();

  double adjusted_height = ideal_terminate_height;
  // 安全性检查与调整, 在当前位置不安全时尝试向上调整高度
  double height_step = 0.2; // meters
  int max_adjust_steps = 5; // 最大调整步数

  for (int i = 0; i < max_adjust_steps; ++i) {
    Eigen::Vector3d check_pos = init_pos;
    check_pos.z() = adjusted_height;
    if (map_->isInLocalMap(check_pos) &&
        map_->isVisible(fd_->odom_pos_, check_pos) &&
        map_->getInflateOccupancy(check_pos) == MapInterface::FREE) {
      Eigen::Vector3d check_floor = check_pos;
      check_floor.z() -= 0.5; 
      if(map_->getInflateOccupancy(check_floor) == MapInterface::FREE){
        INFO_MSG_CYAN("[FSM] Adjust Terminate Height Finding Object: from " << ideal_terminate_height 
          << " to " << adjusted_height << " m");
        return adjusted_height;
      }      
    }
    adjusted_height += height_step; // 向上调整高度
  }

  // 如果无法找到安全高度，返回理想高度
  if(final_point){
    INFO_MSG_RED("[FSM] Cna't find safe height for terminate point, use poly height: " << ideal_poly_height << " m");
    return target_obj->edge.polyhedron_father->center_.z();
  }else{
    return ideal_poly_height;
  }
}

double FastExplorationFSM::adjustTerminateHeightNormal(const Eigen::Vector3d& next_aim_raw){
  double ideal_terminate_height = fd_->odom_pos_.z();
  double adjusted_height        = ideal_terminate_height;
  double height_step            = 0.2; 
  int max_adjust_steps          = 5; 
  // TODO [gwq] height adjust for normal waypoint not finished
  return ideal_terminate_height;
}

/**
* @brief 从全局路径中智能地选取并发布一个最优的局部目标点，以供下层运动规划器执行。
* * 该函数的核心优化策略有两点：
* 1. 捷径优化：通过回溯检查路径，寻找当前位置可以直接无障碍到达的最远路径点，以跳过不必要的中间点。
* 2. 前瞻距离保证：如果未找到捷径，则顺序选取下一个路径点，并确保该点与机器人当前位置有足够的安全距离，以保证运动规划的平滑性。
* * @param[in]  path_res     机器人需要跟随的全局路径点向量。
* @param[in]  look_forward 一个布尔标志，指示机器人是否应朝向最终目标点的姿态。在长路径导航中，此参数会被内部逻辑覆盖为true，强制朝向局部目标点。
* @param[in]  aim_yaw      当look_forward为false时，指定的最终目标偏航角。
* @return     bool         如果成功找到并发布了一个有效的局部目标点，则返回true；如果路径已执行完毕，则返回false。
*/
// get local aim from the path, path_inx++, if exceed path size, do nothing
bool FastExplorationFSM::getAndPublishNextAim(vector<Eigen::Vector3d>& path_res,
                                              const bool look_forward, const double aim_yaw) {
  auto getLocalAim = [&](vector<Eigen::Vector3d>& path_res, int& path_inx, Eigen::Vector3d& local_goal) -> bool
  {
    for(int i = path_res.size()-1; i > path_inx; i--)
    {
      if (map_->isInLocalMap(path_res[i]) &&
          map_->isVisible(fd_->odom_pos_, path_res[i]))
      {
        Eigen::Vector3d cand = path_res[i];
        // inflate 守卫: 候选点落入膨胀层时, 先尝试 C0 投影到最近 inflate-free 且可达点;
        // 投影趋向下一个路径点(避免回飞); 投影失败才判为真障碍, 去抖标记不可达并跳过(严格不进膨胀层)
        if (map_->getInflateOccupancy(cand) == MapInterface::OCCUPIED)
        {
          // 前向参考: 路径上更靠近目标的下一个点(末点则取自身, 退化为无方向)
          Eigen::Vector3d toward = (i + 1 < (int)path_res.size()) ? path_res[i + 1] : path_res.back();
          Eigen::Vector3d repaired;
          if (scene_graph_->projectToInflateFree(cand, toward, repaired) &&
              map_->isVisible(fd_->odom_pos_, repaired))
          {
            cand = repaired;
            // 模式2: 修复点到toward不可直线可见时, 尝试球交会生成中间点
            if (scene_graph_->getRepairVisMode() == 2 && !map_->isVisible(repaired, toward))
            {
              Eigen::Vector3d mid;
              if (scene_graph_->findIntersectionMidpoint(repaired, toward,
                      scene_graph_->getRepairVisSphereRadius(), mid))
              {
                // 插入中间点到路径中 repaired 和 toward 之间
                path_res.insert(path_res.begin() + i + 1, mid);
                INFO_MSG_GREEN("[EXP-FSM] :[getAndPubNextAim] mode2 insert intersection mid pt");
                { visualization_msgs::Marker mp;
                  mp.header.frame_id = "world"; mp.header.stamp = ros::Time::now();
                  mp.ns = "intersection_mid"; mp.id = 0;
                  mp.type = visualization_msgs::Marker::SPHERE;
                  mp.action = visualization_msgs::Marker::ADD;
                  mp.scale.x = mp.scale.y = mp.scale.z = 0.4;
                  mp.color.r = 0.0f; mp.color.g = 0.5f; mp.color.b = 1.0f; mp.color.a = 0.9f;
                  mp.pose.position.x = mid(0); mp.pose.position.y = mid(1); mp.pose.position.z = mid(2);
                  mp.pose.orientation.w = 1.0;
                  vis_marker_pub_.publish(mp); }
              }
              else
              {
                // 球交会失败 → 标记不可达, 跳过此点
                scene_graph_->markPolyhedronBlocked(path_res[i]);
                continue;
              }
            }
            // 发布红色方块标记修复点
            { visualization_msgs::Marker rp;
              rp.header.frame_id = "world"; rp.header.stamp = ros::Time::now();
              rp.ns = "repair_point"; rp.id = 0;
              rp.type = visualization_msgs::Marker::CUBE;
              rp.action = visualization_msgs::Marker::ADD;
              rp.scale.x = rp.scale.y = rp.scale.z = 0.3;
              rp.color.r = 1.0f; rp.color.g = 0.0f; rp.color.b = 0.0f; rp.color.a = 1.0f;
              rp.pose.position.x = repaired(0); rp.pose.position.y = repaired(1); rp.pose.position.z = repaired(2);
              rp.pose.orientation.w = 1.0;
              vis_marker_pub_.publish(rp); }
            // 插入模式: 将修复点编入拓扑图(旧节点永久丢弃, 新节点连接可见邻居后永久可用)
            scene_graph_->insertReplacementNode(path_res[i], repaired);
          }
          else
          {
            scene_graph_->markPolyhedronBlocked(path_res[i]);
            continue;
          }
        }
        path_inx = i;
        local_goal = cand;
        INFO_MSG_GREEN("[EXP-FSM] :[getAndPubNextAim] direct aim to local_goal");
        return true;
      }
    }
    // path_inx 记录了当前路径执行的进度
    path_inx++;
    int idx = path_inx;
    if (path_inx >= path_res.size())
    {
      path_inx -- ;
      ROS_WARN_THROTTLE(1.0, "[EXP-FSM] :[getAndPubNextAim] Path exec finished");
      return false;
    }
    else
    {
      local_goal = path_res[idx];
      // 查找是否有已经接近的点，并向前搜索，保证ego planner获得的点足够远
      while ((fd_->odom_pos_ - local_goal).norm() < 0.1)
      {
        path_inx++;
        if (path_inx >= path_res.size())
        {
          return false;
        }
        idx = path_inx;
        local_goal = path_res[idx];
      }
      return true;
    }
  };

  // Choose local goal
  INFO_MSG("path_res size: " << path_res.size() << ", path_inx_: " << fd_->path_inx_);
  if(path_res.size() <= 2)   // directly aim to the svp
  {   
    fd_->local_aim_pos_ = path_res.back();
    fd_->aim_pos_       = path_res.back();
    if(md_->mission_state_ == MISSION_FSM_STATE::GO_TARGET_OBJECT && 
        scene_graph_->object_factory_->object_map_.find(fd_->object_target_id_) != scene_graph_->object_factory_->object_map_.end()){
    
      auto cur_obj = scene_graph_->object_factory_->object_map_[fd_->object_target_id_];
      fd_->local_aim_pos_[2] =  adjustTerminateHeightFindingObject(cur_obj, fd_->local_aim_pos_, true);
      fd_->aim_pos_[2] = fd_->local_aim_pos_[2];
    }
    pubLocalGoal(fd_->local_aim_pos_, aim_yaw, look_forward);
    cout << "[EXP-FM][getAndPubNextAim][look_forward = "<< look_forward << "] Pub aim:" << path_res.back().transpose() << ", yaw: " << aim_yaw << endl;
    return true;
  }
  else
  {
    if (getLocalAim(path_res, fd_->path_inx_, fd_->local_aim_pos_))
    {
      if(md_->mission_state_ == MISSION_FSM_STATE::GO_TARGET_OBJECT && 
        scene_graph_->object_factory_->object_map_.find(fd_->object_target_id_) != scene_graph_->object_factory_->object_map_.end()){
    
        auto cur_obj = scene_graph_->object_factory_->object_map_[fd_->object_target_id_];
        if(fd_->path_inx_ == path_res.size() -1){
          fd_->local_aim_pos_[2] =  adjustTerminateHeightFindingObject(cur_obj, fd_->local_aim_pos_, true);
          fd_->aim_pos_[2] = fd_->local_aim_pos_[2];
        }
      }
      pubLocalGoal(fd_->local_aim_pos_, aim_yaw, true);
      cout << "[EXP-FM][getAndPubNextAim][look_forward = 1]" << " Pub local aim: " << fd_->local_aim_pos_.transpose() << endl;
      return true;
    }
    return false;
  }
}

void FastExplorationFSM::pubLocalGoal(const Eigen::Vector3d local_goal, const double yaw,
                                      const bool look_forward, const uint8_t yaw_mode,
                                      const uint8_t yaw_path_mode)
{
  // yaw-only全景命令不占用EGO位置轨迹完成标志，目标续接由odometry角度驱动。
  if (yaw_mode != quadrotor_msgs::EgoGoalSet::YAW_MODE_PANORAMA)
    fd_->ego_exec_finished_ = false;

  quadrotor_msgs::EgoGoalSet msg;
  msg.drone_id = md_->drone_id_;
  msg.source_task_id = active_instruction_task_id_;
  msg.goal[0] = static_cast<float>(local_goal.x());
  msg.goal[1] = static_cast<float>(local_goal.y());
  msg.goal[2] = static_cast<float>(local_goal.z());
  msg.look_forward = look_forward;
  msg.yaw = yaw;
  msg.yaw_low_speed = yaw_mode == quadrotor_msgs::EgoGoalSet::YAW_MODE_LOW_SPEED;
  msg.yaw_mode = yaw_mode;
  msg.yaw_path_mode = yaw_path_mode;
  ego_goal_pub_.publish(msg);
}

// return aim_pose aim_vel, aim_yaw and path_res
int FastExplorationFSM::callExplorationPlanner(Eigen::Vector3d& aim_pose, Eigen::Vector3d& aim_vel, double& aim_yaw, vector<Eigen::Vector3d>& path_res)
{
  auto perf_t0 = PERF_NOW();  // 性能日志：主规划入口计时起点
  // mode 1在TSP前同步消费累计雷达更新盒，完成新frontier生成。
  expl_manager_->updateFrontiersForPlanning(fd_->odom_pos_, fd_->odom_yaw_);

  map_->Lock();
  ros::Time time_r = ros::Time::now() + ros::Duration(fp_->replan_time_);

  int res = expl_manager_->planExploreTSP(fd_->odom_pos_, fd_->odom_vel_, fd_->odom_yaw_,
                                          aim_pose, aim_vel, aim_yaw, path_res);

  map_->Unlock();

  // 性能日志：记录主规划总耗时、返回结果、路径点数和当前里程计位置
  std::string res_str;
  switch (res) {
    case SUCCEED:     res_str = "SUCCEED"; break;
    case NO_FRONTIER: res_str = "NO_FRONTIER"; break;
    case FAIL:        res_str = "FAIL"; break;
    default:          res_str = "UNKNOWN(" + std::to_string(res) + ")"; break;
  }
  PERF_LOG_ELAPSED_EX("CALL_PLAN", perf_t0,
    "res=" + res_str + " path_size=" + std::to_string(path_res.size()) +
    " odom=(" + std::to_string(fd_->odom_pos_.x()) + "," +
                 std::to_string(fd_->odom_pos_.y()) + "," +
                 std::to_string(fd_->odom_pos_.z()) + ")");
  return res;
}

int FastExplorationFSM::callExplorationLLMPlanner(Eigen::Vector3d &aim_pose, Eigen::Vector3d &aim_vel, double &aim_yaw, vector<Eigen::Vector3d> &path_res) {
  int res = expl_manager_->planLLMExploration(expl_area_id_, fd_->odom_pos_, fd_->odom_vel_, fd_->odom_yaw_,
                                              scene_graph_->getCurPoly(), aim_pose, aim_yaw, aim_vel, path_res);
  return res;
}

int FastExplorationFSM::callTrackPlanner(Eigen::Vector3d& aim_pose, Eigen::Vector3d& aim_vel,
                                         double& aim_yaw, vector<Eigen::Vector3d>& path_res)
{
  (void)aim_vel;
  (void)aim_yaw;
  map_->Lock();
  int res = expl_manager_->planTrackGoal(fd_->odom_pos_, fd_->odom_vel_, aim_pose, path_res);
  map_->Unlock();
  return res;
}

void FastExplorationFSM::vlaSwarmMapCallback(const ros::TimerEvent&)
{
  if (!vla_swarm_enabled_ || !fd_->have_odom_ || vla_swarm_map_ == nullptr) {
    return;
  }

  // SmallMap 独立于单次任务持续更新，使 PLACE 和 LOCAL_PLAN 使用同一帧地图语义。
  if (!vla_swarm_map_->update(fd_->odom_pos_)) {
    ROS_WARN_THROTTLE(5.0, "[VLA_SWARM] SmallMap is waiting for an initialized occupancy map.");
  }
}

void FastExplorationFSM::frontierCallback(const ros::TimerEvent& e) {
  static int delay = 0;
  if (!scene_graph_->skeleton_gen_->ready()) {
    ROS_WARN_THROTTLE(2.0, "[ExploreFSM] | skeleton has not been generated, skip once!");
    return ;
  }

  if (++delay < 5) return;
  if (md_->mission_state_ == INIT) return;
  if (waitForFreshMapAfterReset()) return;

  ros::Time t1 = ros::Time::now();
  auto ft = expl_manager_->frontier_finder_;
  auto ed = expl_manager_->ed_;

  bool new_topo = false;
  scene_graph_->updateSceneGraph(fd_->odom_pos_, fd_->odom_yaw_, new_topo);

  if (new_topo && fp_->enable_area_prediction_ && fd_->new_topo_need_predict_immediately_) {
    // 全景旋转期间不中断 yaw 指令：当前 360° 扫描尚未完成，
    // 中途发布非全景 goal 会覆盖全景 yaw 目标，导致 handlePanoramaYaw 无法恢复。
    // 新拓扑的 LLM 预测推迟到全景结束后的 planLLMExplore 中自然触发。
    if (need_panorama_) {
      new_topo = false;
    } else if (fp_->object_id_nav_use_thinking_) {
      // THINKING模式: 发送LLM预测新区域类型, 暂停运动避免边界区域误判
      std::string llm_prompt_str;
      scene_graph_->newAreaPredictionPromptGen(llm_prompt_str);
      cur_prompt_id_ = scene_graph_->getCurPromptId();
      scene_graph_->sendPrompt(scene_graph_->getCurPromptIdAndPlusOne(),
                               scene_graph::PromptMsg::PROMPT_TYPE_ROOM_PREDICTION,
                               llm_prompt_str, std::chrono::seconds(10), 1);
      if (scene_graph_->skeleton_gen_->cur_iter_first_poly_ != nullptr) {
        double aim_direction = fd_->odom_yaw_;
        if (aim_direction > M_PI)
          aim_direction -= 2 * M_PI;
        if (aim_direction < -M_PI)
          aim_direction += 2 * M_PI;
        Eigen::Vector3d aim_pos = scene_graph_->skeleton_gen_->cur_iter_first_poly_->center_;
        pubLocalGoal(aim_pos, aim_direction, false,
                     quadrotor_msgs::EgoGoalSet::YAW_MODE_LOW_SPEED);
        INFO_MSG_GREEN("Get New skeleton info, stop motion & predict new area");
      }
      if (md_->mission_state_ == MISSION_FSM_STATE::WAIT_TRIGGER)
        return;
      transitState(MISSION_FSM_STATE::LLM_PLAN_EXPLORE, "ftr_callback -> New Topo Found -> Replan");
      stashCurStateAndTransit(MISSION_FSM_STATE::THINKING, "frontierCallback -> New Topo Found -> Predict!");
      think_start_time_        = ros::Time::now().toSec();
      think_duration_limit_    = 10.0;
      has_made_area_decision_  = false;
      need_rotate_yaw_         = enable_yaw_scan_;
    }
    // 非THINKING模式: 跳过new-topo中断, 当前探索路径正常replan即可处理
  }

  expl_manager_->setCurrentTopoNode(scene_graph_->skeleton_gen_->mountCurTopoPoint(fd_->odom_pos_, fd_->odom_yaw_));
  scene_graph_->mountCurPoly(fd_->odom_pos_, fd_->odom_yaw_);
  if (new_topo) {
    ft->reCalculateAllFtrTopo(fd_->odom_pos_);
  }
  ft->searchFrontiers(fd_->odom_pos_);
  ft->computeFrontiersToVisit(fd_->odom_pos_);
  ft->updateFrontierCostMatrix();
  ft->updateSceneGraphWithFtr();

  //! Update HGrid
  expl_manager_->updateHgrid();

  // 检测 frontier 变化（新增/消失），供 approachRegularExplore 持续重规划使用
  size_t cur_ftr_count = ft->frontiers_.size();
  if (fd_->frontier_last_count_ != 0 && cur_ftr_count != fd_->frontier_last_count_) {
    fd_->frontier_changed_ = true;
  }
  fd_->frontier_last_count_ = cur_ftr_count;

  // int area_id = scene_graph_->getAreaFromPoly(scene_graph_->getCurPoly());
  // expl_manager_->planLLMExploration(area_id, fd_->odom_pos_, fd_->odom_vel_, fd_->odom_yaw_,
  //                                   scene_graph_->getCurPoly(), fd_->aim_pos_, fd_->aim_yaw_, fd_->aim_vel_, fd_->path_res_);

  // ft->vp_handler_->updateVPWellObserved(fd_->odom_pos_, fd_->odom_yaw_, 2.0);
  // ft->vp_handler_->updateCostMatrix(scene_graph_->cur_areas_[area_id]);
  // Eigen::MatrixXd cost_mat;
  // std::vector<Viewpoint::Ptr> vp_list;
  // ft->vp_handler_->calCurrentAreaCostMatrix(scene_graph_->cur_areas_[area_id], fd_->odom_pos_, fd_->odom_yaw_, fd_->odom_vel_, cost_mat);
  // expl_manager_->findVPGlobalTour(ft->vp_handler_->vps_candidate_, cost_mat, fd_->odom_pos_, fd_->odom_yaw_, fd_->odom_vel_);

  ros::Time t2 = ros::Time::now();
  expl_manager_->visualize(fd_->odom_pos_);
  scene_graph_->visualizeSceneGraph();
  ros::Time t3 = ros::Time::now();

  double vis_time        = (t3 - t2).toSec() * 1e3;
  double perception_time = (t2 - t1).toSec() * 1e3;
  if (perception_time > 20.0 || vis_time > 10.0 )
    ROS_WARN_STREAM("[FtrCallback] : Update time: " << perception_time << " + Vis spend time : " << vis_time << "ms");
}

void FastExplorationFSM::odometryCallback(const nav_msgs::OdometryConstPtr& msg) {
  fd_->odom_pos_(0) = msg->pose.pose.position.x;
  fd_->odom_pos_(1) = msg->pose.pose.position.y;
  fd_->odom_pos_(2) = msg->pose.pose.position.z;

  fd_->odom_vel_(0) = msg->twist.twist.linear.x;
  fd_->odom_vel_(1) = msg->twist.twist.linear.y;
  fd_->odom_vel_(2) = msg->twist.twist.linear.z;

  fd_->odom_yaw_rate_ = msg->twist.twist.angular.z;

  fd_->odom_orient_.w() = msg->pose.pose.orientation.w;
  fd_->odom_orient_.x() = msg->pose.pose.orientation.x;
  fd_->odom_orient_.y() = msg->pose.pose.orientation.y;
  fd_->odom_orient_.z() = msg->pose.pose.orientation.z;

  Eigen::Vector3d rot_x = fd_->odom_orient_.toRotationMatrix().block<3, 1>(0, 0);
  fd_->odom_yaw_ = atan2(rot_x(1), rot_x(0));

  if (need_panorama_)
  {
    double yaw_delta = fd_->odom_yaw_ - panorama_last_odom_yaw_;
    while (yaw_delta > M_PI) yaw_delta -= 2 * M_PI;
    while (yaw_delta < -M_PI) yaw_delta += 2 * M_PI;

    panorama_unwrapped_yaw_ += yaw_delta;
    // 累计量表示相对起始朝向沿正方向的净变化，反向扰动会增加后续剩余角。
    panorama_accumulated_yaw_ = std::max(0.0, panorama_accumulated_yaw_ + yaw_delta);
    panorama_last_odom_yaw_ = fd_->odom_yaw_;
  }

  fd_->have_odom_ = true;
}

void FastExplorationFSM::egoPlanResCallback(const quadrotor_msgs::EgoPlannerResultConstPtr &msg) {
  fd_->ego_local_goal_.x() = msg->planner_goal.x;
  fd_->ego_local_goal_.y() = msg->planner_goal.y;
  fd_->ego_local_goal_.z() = msg->planner_goal.z;
  fd_->ego_plan_times_     = msg->plan_times;
  fd_->ego_plan_status_    = msg->plan_status;
  fd_->ego_modify_status_  = msg->modify_status;

  // EGO 结果消息没有会话字段，只在 VLA 当前局部目标坐标匹配时消费，
  // 避免前一任务或 stopMotion 的迟到回调推进本次路径。
  if (vla_swarm_active_ &&
      md_->mission_state_ == MISSION_FSM_STATE::VLA_SWARM_APPROACH &&
      vla_swarm_waypoint_published_ &&
      (fd_->ego_local_goal_ - fd_->local_aim_pos_).norm() <= 0.25) {
    vla_swarm_plan_feedback_received_ = true;
    vla_swarm_plan_feedback_success_ = msg->plan_status;
    if (msg->plan_status) {
      fd_->ego_exec_finished_ = false;
    }
  }
}

void FastExplorationFSM::instructionCallback(const quadrotor_msgs::InstructionConstPtr& msg)
{
  if (msg->robot_id != md_->drone_id_) return;
  // check recv time frequncy
  static bool ic_first_recv_flag = true;
  static ros::Time ic_last_recv_time;
  const bool bypass_freq_limit =
      msg->instruction_type == quadrotor_msgs::Instruction::TURN_GOAL ||
      msg->instruction_type == quadrotor_msgs::Instruction::TURN_WAYPOINT_NAV ||
      msg->instruction_type == quadrotor_msgs::Instruction::TURN_TRACKING ||
      msg->instruction_type == quadrotor_msgs::Instruction::TURN_OBJECT_NAV ||
      msg->instruction_type == quadrotor_msgs::Instruction::TURN_REGULAR_EXPLORATION ||
      msg->instruction_type == quadrotor_msgs::Instruction::TURN_VLA_SWARM ||
      msg->instruction_type == quadrotor_msgs::Instruction::REQUEST_ALL_AREA_AND_OBJS;
  if (ic_first_recv_flag){
    ic_first_recv_flag = false;
    ic_last_recv_time = ros::Time::now();
  }else if (!bypass_freq_limit && !ic_first_recv_flag &&
            (ros::Time::now() - ic_last_recv_time).toSec() < 0.8){
    ic_last_recv_time = ros::Time::now();
    std::cout << "[InstructionCallback] : recv too frequent, skip once! instruction_type="
              << static_cast<int>(msg->instruction_type)
              << ", command=" << msg->command << std::endl;
    return;
  }else
    ic_last_recv_time = ros::Time::now();

  if (vla_swarm_active_) {
    const bool same_vla_swarm_session =
        msg->instruction_type == quadrotor_msgs::Instruction::TURN_VLA_SWARM &&
        msg->source_task_id == quadrotor_msgs::Instruction::SOURCE_TASK_VLA_SWARM &&
        msg->task_session_id == vla_swarm_session_id_;
    if (same_vla_swarm_session) {
      ROS_WARN_STREAM("[VLA_SWARM] Ignore duplicated Instruction for active session="
                      << vla_swarm_session_id_);
      return;
    }
    cancelVlaSwarmTask("replaced_by_new_task", "received a new Instruction");
  }

  md_->instruction_ = msg->instruction_type;
  if (counting_scene_graph_ != nullptr && counting_scene_graph_->active()) {
    counting_scene_graph_->cancelSession();
  }
  active_instruction_task_id_ = msg->source_task_id;
  active_instruction_session_id_ = msg->task_session_id;
  const bool source_requires_panorama =
      msg->source_task_id == quadrotor_msgs::Instruction::SOURCE_TASK_EXPLORATION ||
      msg->source_task_id == quadrotor_msgs::Instruction::SOURCE_TASK_COUNTING;
  // 每条新Instruction先终止旧调度状态，只有下方匹配的task来源和探索指令可重新开启。
  need_panorama_ = false;
  panorama_command_active_ = false;
  wait_fresh_map_after_reset_ = false;
  fd_->instruct_directly_to_goal = false; // [gwq] 防止从turn_ego_plan状态切出的时候其他状态依旧使用强制ego规划
  if (msg->instruction_type != quadrotor_msgs::Instruction::TURN_TRACKING) {
    switchPlannerCmdMuxToEgo("instructionCallback:non_tracking");
    stopElasticTracker("instructionCallback:non_tracking");
    std::unique_lock<std::mutex> lck(mtx_);
    fd_->track_trigger_ = false;
    fd_->track_init_ = false;
    resetTrackingFinishCandidate();
    fd_->track_finish_sent_ = false;
    if (!useElasticTrackerBackend()) {
      map_->setTarget(fd_->track_pos_, false);
    }
  }

  // load map !
  if (msg->instruction_type == quadrotor_msgs::Instruction::TURN_LOAD_SCENE_GRAPH){
    // stopMotion();
    const bool load_ok = scene_graph_->loadMap(msg->map_folder);
    if (load_ok) {
      fd_->path_res_.clear();
      fd_->path_inx_ = 0;
      fd_->trigger_ = false;
      fd_->regular_explore_ = false;
      fd_->df_demo_mode_ = false;
      fd_->find_terminate_target_mode_ = false;
      fd_->new_topo_need_predict_immediately_ = false;
      fd_->llm_plan_explore_counter_ = 0;
      has_made_area_decision_ = false;
      need_rotate_yaw_ = false;
      expl_area_id_ = -1;
      hardResetExploreArea(false, false);
      scene_graph_->object_factory_->runThisModule();
      scene_graph_->refreshLoadedMapVisualization();
      // 根据配置决定是否冻结场景图增量更新
      if (!enable_scene_graph_update_after_load_) {
        scene_graph_->freezeUpdate();
        INFO_MSG_GREEN("[InstructionCallback] Scene graph update FROZEN after load.");
      }
      transitState(MISSION_FSM_STATE::WAIT_TRIGGER, "instructionCallback(load scene graph)");
      INFO_MSG_GREEN("[InstructionCallback] Load scene graph snapshot succeeded.");
    } else {
      INFO_MSG_RED("[InstructionCallback] Load scene graph snapshot failed.");
    }
    INFO_MSG("\n\n");
    return ;
  }else if(msg->instruction_type == quadrotor_msgs::Instruction::REQUEST_ALL_AREA_AND_OBJS){
    INFO_MSG_CYAN("[InstructionCallback] Request all area and object info, and publish to CoPaw!");
    string scene_graph_json_str;
    scene_graph_->DFDemoPromptGen(scene_graph_json_str);
    scene_graph_->sendSceneGraphJson(scene_graph_json_str);
  }

  if (md_->mission_state_ == INIT || md_->mission_state_ == WARM_UP) return;
  vector<int> target_drone_ids, source_drone_ids;

  switch (msg->instruction_type) 
  {
    case quadrotor_msgs::Instruction::TURN_VLA_SWARM:
      resetVlaSwarmContext();
      vla_swarm_active_ = true;
      vla_swarm_session_id_ = msg->task_session_id;
      vla_swarm_command_ = msg->command;
      if (!vla_swarm_enabled_) {
        vla_swarm_finish_reason_ = "disabled";
        vla_swarm_finish_detail_ = "vla_swarm/enable is false";
        transitState(MISSION_FSM_STATE::VLA_SWARM_FINISH, "instructionCallback:vla_swarm_disabled");
      } else if (msg->source_task_id != quadrotor_msgs::Instruction::SOURCE_TASK_VLA_SWARM) {
        vla_swarm_finish_reason_ = "invalid_source_task";
        vla_swarm_finish_detail_ = "TURN_VLA_SWARM requires SOURCE_TASK_VLA_SWARM";
        transitState(MISSION_FSM_STATE::VLA_SWARM_FINISH, "instructionCallback:vla_swarm_invalid_source");
      } else if (msg->task_session_id == 0) {
        vla_swarm_finish_reason_ = "invalid_session";
        vla_swarm_finish_detail_ = "TURN_VLA_SWARM requires non-zero task_session_id";
        transitState(MISSION_FSM_STATE::VLA_SWARM_FINISH, "instructionCallback:vla_swarm_invalid_session");
      } else if (msg->command.empty()) {
        vla_swarm_finish_reason_ = "invalid_command";
        vla_swarm_finish_detail_ = "TURN_VLA_SWARM requires a non-empty command";
        transitState(MISSION_FSM_STATE::VLA_SWARM_FINISH, "instructionCallback:vla_swarm_invalid_command");
      } else {
        startVlaSwarmTask(msg);
      }
      break;

    case quadrotor_msgs::Instruction::TURN_OBJECT_ID_NAV:
      // 存储最新消息（先清空, 再存储）
      fd_->stored_object_id_nav_instruction_ = *msg;
      fd_->has_stored_object_id_nav_instruction_ = true;
      fd_->object_id_nav_replan_stuck_begin_time_ = -1.0;  // 新任务重置卡死计时
      fd_->object_id_nav_replan_topic_triggered_ = false;
      fd_->object_id_nav_replan_stuck_count_ = 0;           // 新任务重置replan计数

      expl_manager_->setExplorationRegion(std::vector<Eigen::Vector3d>(), false);
      fd_->object_target_id_ = msg->target_obj_id;
      fd_->path_inx_        = 0;
      fd_->go_object_process_phase = 0;
      fd_->go_object_in_prior_guide_ = false;
      fd_->go_object_prior_guide_count_ = 0;
      fd_->find_terminate_target_mode_ = false;
      scene_graph_->clearAllBlocked();     // 新导航任务: 清除上次遗留的不可达标记, 不跨任务持久
      transitState(MISSION_FSM_STATE::GO_TARGET_OBJECT, "instructionCallback");
      break;

    case quadrotor_msgs::Instruction::TURN_WAYPOINT_NAV: {
      expl_manager_->setExplorationRegion(std::vector<Eigen::Vector3d>(), false);
      if (msg->nav_waypoint.empty()) {
        INFO_MSG_RED("[InstructionCallback]: TURN_WAYPOINT_NAV has empty nav_waypoint, switch to WAIT_TRIGGER");
        transitState(MISSION_FSM_STATE::WAIT_TRIGGER, "instructionCallback:empty_waypoint");
        break;
      }

      const auto& nav_waypoint = msg->nav_waypoint.front();
      const double waypoint_z = std::isfinite(nav_waypoint.z) ? static_cast<double>(nav_waypoint.z) : 1.0;
      fd_->waypoint_target_ = Eigen::Vector3d(nav_waypoint.x, nav_waypoint.y, waypoint_z);
      fd_->waypoint_target_yaw_ =
          msg->nav_yaw.empty() ? fd_->odom_yaw_ : static_cast<double>(msg->nav_yaw.front());
      fd_->go_waypoint_process_phase = 0;
      fd_->find_terminate_target_mode_ = false;
      transitState(MISSION_FSM_STATE::GO_TARGET_WITH_WAYPOINT, "instructionCallback");
      break;
    }

    case quadrotor_msgs::Instruction::TURN_OBJECT_NAV:
      applyExplorationRegionFromInstruction(msg);
      if (msg->source_task_id == quadrotor_msgs::Instruction::SOURCE_TASK_COUNTING &&
          msg->task_session_id > 0) {
        counting_scene_graph_->startSession(msg->task_session_id, fd_->odom_pos_);
      }
      if (source_requires_panorama && msg->clear_local_map) {
        stopMotion();
        hardResetExploreArea(true, false);
        map_reset_update_seq_ = map_->getOccupancyUpdateSeq();
        wait_fresh_map_after_reset_ = true;
      } else if (source_requires_panorama) {
        startPanoramaRotation();
      }
      fd_->find_terminate_target_mode_ = false;
      fd_->new_topo_need_predict_immediately_ = true;
      fd_->df_demo_mode_ = false;
      fd_->target_cmd_ = msg->command;
      scene_graph_->setTargetAndPriorKnowledge(fd_->target_cmd_, fd_->prior_knowledge_);
      // THINKING关闭时, LLM不参与area选择, 直接走PLAN_EXPLORE(全局TSP);
      // THINKING开启时, 走LLM_PLAN_EXPLORE让LLM选area
      if (fp_->object_id_nav_use_thinking_) {
        fd_->regular_explore_ = false;
        transitState(MISSION_FSM_STATE::LLM_PLAN_EXPLORE, "instructionCallback");
      } else {
        fd_->regular_explore_ = true;
        transitState(MISSION_FSM_STATE::PLAN_EXPLORE, "instructionCallback");
      }
      break;

    case quadrotor_msgs::Instruction::TURN_REGULAR_EXPLORATION:
      applyExplorationRegionFromInstruction(msg);
      fd_->regular_explore_ = true;
      // 启动探索任务计时（在 publishExplorationResult 时结算）
      exploration_start_time_ = ros::Time::now();
      exploration_timer_active_ = true;
      if (msg->source_task_id == quadrotor_msgs::Instruction::SOURCE_TASK_COUNTING &&
          msg->task_session_id > 0) {
        counting_scene_graph_->startSession(msg->task_session_id, fd_->odom_pos_);
      }
      if (source_requires_panorama && msg->clear_local_map) {
        stopMotion();
        hardResetExploreArea(true, false);
        map_reset_update_seq_ = map_->getOccupancyUpdateSeq();
        wait_fresh_map_after_reset_ = true;
      } else if (source_requires_panorama) {
        startPanoramaRotation();
      }
      fd_->df_demo_mode_    = false;
      fd_->find_terminate_target_mode_ = false;
      transitState(MISSION_FSM_STATE::PLAN_EXPLORE, "instructionCallback");
      break;
    
    case quadrotor_msgs::Instruction::TURN_DF_DEMO:
      expl_manager_->setExplorationRegion(std::vector<Eigen::Vector3d>(), false);
      fd_->df_demo_mode_ = true;
      fd_->df_demo_phase_ = 0;
      fd_->explore_count_ = 0;
      fd_->df_demo_target_id_ = -100;
      for (auto& area_iter : scene_graph_->skeleton_gen_->area_handler_->areas_need_predict_)
        area_iter.second = true;
      fd_->target_cmd_ = msg->command;
      scene_graph_->setTargetAndPriorKnowledge(fd_->target_cmd_, fd_->prior_knowledge_);
      transitState(MISSION_FSM_STATE::DF_DEMO, "instructionCallback");
      break;

    case quadrotor_msgs::Instruction::TURN_GOAL:
      handleGoalInstruction(msg->goal, msg->yaw, msg->look_forward, "instructionCallback:goal");
      break;

    case quadrotor_msgs::Instruction::TURN_TRACKING:
      if (!msg->enable)
      {
        std::unique_lock<std::mutex> lck(mtx_);
        fd_->track_trigger_ = false;
        fd_->track_init_ = false;
        resetTrackingFinishCandidate();
        fd_->track_finish_sent_ = false;
        switchPlannerCmdMuxToEgo("instructionCallback:tracking_disable");
        stopElasticTracker("instructionCallback:tracking_disable");
        if (!useElasticTrackerBackend()) {
          map_->setTarget(fd_->track_pos_, false);
        }
        if (md_->mission_state_ == MISSION_FSM_STATE::PLAN_TRACK ||
            md_->mission_state_ == MISSION_FSM_STATE::APPROACH_TRACK)
        {
          stopMotion();
          transitState(MISSION_FSM_STATE::WAIT_TRIGGER, "instructionCallback:tracking_disable");
        }
        break;
      }

      {
        std::unique_lock<std::mutex> lck(mtx_);
        const bool was_track_trigger = fd_->track_trigger_;
        if (useElasticTrackerBackend() && fd_->track_finish_sent_ && !was_track_trigger) {
          ROS_INFO_STREAM_THROTTLE(0.5, "[TRACK] ignore residual Elastic-Tracker instruction enable after finish; waiting for disable/reset.");
          break;
        }
        fd_->track_trigger_ = true;
        if (!was_track_trigger) {
          resetTrackingFinishCandidate();
          fd_->track_finish_sent_ = false;
        }
        if (msg->has_target_position)
        {
          fd_->track_pos_ = geoPt2Vec3d(msg->target_position);
        }
        if (useElasticTrackerBackend()) {
          if (msg->has_target_position) {
            fd_->track_init_ = true;
            publishTrackingTargetOdom(fd_->track_pos_, msg->header.stamp, msg->header.frame_id);
          }
          // Elastic-Tracker 的 /triger 只用于打开新 session，后续目标更新只走 target odom。
          if (!was_track_trigger) {
            publishElasticTrackerTrigger(msg->header.stamp, msg->header.frame_id);
          }
          switchPlannerCmdMuxToElastic("instructionCallback:elastic_tracker_enable");
          if (md_->mission_state_ != MISSION_FSM_STATE::WAIT_TRIGGER) {
            stopMotion();
            transitState(MISSION_FSM_STATE::WAIT_TRIGGER, "instructionCallback:elastic_tracker_enable");
          }
        } else {
          switchPlannerCmdMuxToEgo("instructionCallback:ego_tracking_enable");
          map_->setTarget(fd_->track_pos_, false);
          if (md_->mission_state_ != MISSION_FSM_STATE::PLAN_TRACK &&
              md_->mission_state_ != MISSION_FSM_STATE::APPROACH_TRACK)
          {
            transitState(MISSION_FSM_STATE::PLAN_TRACK, "instructionCallback:tracking_enable");
          }
        }
      }

      if (!msg->global_poses.empty())
      {
        handleTrackingTarget(msg->global_poses, "instructionCallback:tracking_target",
                             msg->header.stamp, msg->header.frame_id);
      }
      break;
    
    case quadrotor_msgs::Instruction::TURN_SAVE_SCENE_GRAPH: {
      const bool save_ok = scene_graph_->saveMap(msg->map_folder);
      if (save_ok) {
        INFO_MSG_GREEN("[InstructionCallback] Save scene graph snapshot succeeded.");
      } else {
        INFO_MSG_RED("[InstructionCallback] Save scene graph snapshot failed.");
      }
      break;
    }

    default:
      transitState(MISSION_FSM_STATE::WAIT_TRIGGER, "instructionCallback");
      INFO_MSG_RED("[InstructionCallback]: No Valid Instruction! please check, switch to WAIT_TRIGGER"); 
      break;
  }
}

// === 探索脱困通用 helper (建议 D/E) ===
// 这三个 helper 抽取自 goTargetObject 的卡死检测逻辑, 供 approachRegularExplore 复用,
// 消除探索路径与目标导航路径的卡死处理不对称问题.

// 统一卡死检测: 速度+角速度均低于阈值 → 进入卡死计时; 有运动 → 重置计时与触发标记
// 返回当前是否处于卡死状态(是否"可触发"由调用方结合 stuck_sec 判断)
// 注意: 复用 fp_->stuck_force_advance_yaw_rate_thresh_ 作为 yaw_rate 阈值, 与 object nav 一致
bool FastExplorationFSM::detectExploreStuck() {
  double vel_norm = fd_->odom_vel_.norm();
  double yaw_rate = fabs(fd_->odom_yaw_rate_);
  bool is_stuck = (vel_norm < fp_->explore_local_stuck_vel_thresh_ &&
                   yaw_rate < fp_->stuck_force_advance_yaw_rate_thresh_);
  if (is_stuck) {
    if (fd_->explore_stuck_begin_time_ < 0.0)
      fd_->explore_stuck_begin_time_ = ros::Time::now().toSec();
  } else {
    fd_->explore_stuck_begin_time_ = -1.0;            // 有运动, 重置计时
    fd_->explore_stuck_triggered_  = false;           // 脱离卡死, 允许下次再触发
  }
  return is_stuck;
}

// 重置探索卡死状态: 正常推进 path_inx 或整体重规划时调用
// 包含失败计数清零(建议A), 保证新路径从头累计
void FastExplorationFSM::resetExploreStuckState() {
  fd_->explore_stuck_begin_time_    = -1.0;
  fd_->explore_stuck_triggered_     = false;
  fd_->explore_stuck_advance_count_ = 0;
  fd_->local_aim_fail_count_        = 0;
}

// 重置 ego 反馈相关状态(建议E): 删 frontier 或重规划时调用
// 防止 ego_plan_times_/ego_modify_status_ 等残留值下次误触发 bad_frontier 或 [6]
void FastExplorationFSM::resetExploreEgoState() {
  fd_->ego_plan_times_    = 0;
  fd_->ego_modify_status_ = false;
  fd_->ego_exec_finished_ = false;
  fd_->ego_plan_status_   = false;
  fd_->has_rotated_       = false;
}

void FastExplorationFSM::triggerObjectIdNavReplan(const std::string& reason) {
  if (!fd_->has_stored_object_id_nav_instruction_) {
    ROS_WARN("[ObjIdNavReplan] No stored instruction, cannot replan");
    transitState(MISSION_FSM_STATE::WAIT_TRIGGER, "replan_no_stored_instruction");
    return;
  }

  const auto& msg = fd_->stored_object_id_nav_instruction_;
  fd_->object_id_nav_replan_stuck_count_++;
  ROS_WARN("[ObjIdNavReplan] Replanning (target_obj_id=%d, reason=%s, count=%d)",
           msg.target_obj_id, reason.c_str(), fd_->object_id_nav_replan_stuck_count_);

  // 重置卡死状态
  fd_->object_id_nav_replan_stuck_begin_time_ = -1.0;
  fd_->object_id_nav_replan_topic_triggered_ = false;

  // 直接执行重初始化逻辑（与 instructionCallback 中 TURN_OBJECT_ID_NAV 一致）
  expl_manager_->setExplorationRegion(std::vector<Eigen::Vector3d>(), false);
  fd_->object_target_id_ = msg.target_obj_id;
  fd_->path_inx_ = 0;
  fd_->go_object_process_phase = 0;
  fd_->go_object_in_prior_guide_ = false;
  fd_->go_object_prior_guide_count_ = 0;
  fd_->find_terminate_target_mode_ = false;
  scene_graph_->clearAllBlocked();
  transitState(MISSION_FSM_STATE::GO_TARGET_OBJECT, "object_id_nav_replan:" + reason);
}

void FastExplorationFSM::objectIdNavReplanCallback(const std_msgs::Bool::ConstPtr& msg) {
  if (!fp_->object_id_nav_replan_enable_) {
    return;  // 功能未启用, 忽略
  }
  if (msg->data) {
    ROS_INFO("[ObjIdNavReplan] Received /object_id_nav_replan = true");
    fd_->object_id_nav_replan_topic_triggered_ = true;
  }
}

void FastExplorationFSM::batteryCallBack(const sensor_msgs::BatteryState msg) {
  static int trigger_time = 0;
  ROS_INFO_STREAM_THROTTLE(2.0, "[FSM] voltage: " << msg.voltage);
  if (msg.voltage < fp_->battery_thr_) 
  {
    // transitMode(MISSION_MODE::HOME, "batteryCallBack");
    // transitState(MISSION_FSM_STATE::GOHOME, "batteryCallBack");
    ROS_ERROR_THROTTLE(1.0, "\n========================\n***** Battery Low *****\n========================\n");
    trigger_time++;
  }
  return;
}

void FastExplorationFSM::stashCurStateAndTransit(MISSION_FSM_STATE new_state, string who_called) {
  stash_state_ = md_->mission_state_;
  transitState(new_state, who_called);
}

void FastExplorationFSM::transitState(MISSION_FSM_STATE new_state, string pos_call) 
{
  MISSION_FSM_STATE pre_s = md_->mission_state_;
  md_->mission_state_ = new_state;

  ROS_INFO_STREAM("\033[1;36m" << "[" << pos_call << "]: from " << md_->state_str_[pre_s]
                      << " to " << md_->state_str_[md_->mission_state_] << "\033[0m"); // 青色
}

void FastExplorationFSM::displayPath() {
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "global_path";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = marker.scale.y = marker.scale.z = 0.3;
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0f;
  for (int i = 0; i < fd_->path_res_.size(); i++) {
    geometry_msgs::Point p;
    p.x = fd_->path_res_[i](0);
    p.y = fd_->path_res_[i](1);
    p.z = fd_->path_res_[i](2);
    marker.points.push_back(p);
  }
  marker_array.markers.push_back(marker);
  vis_path_pub_.publish(marker_array);
}

void FastExplorationFSM::displayLocalAim() {
  // 橙色SPHERE标记当前local_aim导航点, 尺寸大于路径marker(0.5m > 0.3m)
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "local_aim";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = marker.scale.y = marker.scale.z = 0.5;
  marker.color.r = 1.0f;   // 橙色
  marker.color.g = 0.5f;
  marker.color.b = 0.0f;
  marker.color.a = 0.9f;
  marker.pose.position.x = fd_->local_aim_pos_(0);
  marker.pose.position.y = fd_->local_aim_pos_(1);
  marker.pose.position.z = fd_->local_aim_pos_(2);
  marker.pose.orientation.w = 1.0;
  vis_marker_pub_.publish(marker);
}

void FastExplorationFSM::displayMissionState()
{
  std::string text;
  text = "[S] ";
  switch (md_->mission_state_) {
    case INIT: text += "Init"; break;
    case PLAN_EXPLORE: text += "PExplore"; break;
    case LLM_PLAN_EXPLORE: text += "LLMExplore"; break;
    case PLAN_TRACK: text += "PTrack"; break;
    case WAIT_TRIGGER: text += "WTrigger"; break;
    case WARM_UP : text += "WarmUp"; break;
    case THINKING: text += "Thinking"; break;
    case YAW_HANDLE: text += "YawHandle"; break;
    case FINISH: text += "Finish"; break;
    case APPROACH_EXPLORE: text+="ApproExplore"; break;
    case APPROACH_TRACK: text+="ApproTrack"; break;
    case GO_TARGET_OBJECT: text+="Go-Obj"; break;
    case GO_TARGET_WITH_WAYPOINT: text+="Go-Wpt"; break;
    case DF_DEMO: text+="DFDemo"; break;
    case VLA_SWARM_PLAN_LOCAL: text+="VSwarm-Plan"; break;
    case VLA_SWARM_WAIT_LLM: text+="VSwarm-LLM"; break;
    case VLA_SWARM_WAIT_TARGET: text+="VSwarm-Target"; break;
    case VLA_SWARM_APPROACH: text+="VSwarm-Approach"; break;
    case VLA_SWARM_YAW_HANDLE: text+="VSwarm-Yaw"; break;
    case VLA_SWARM_RECOVERY: text+="VSwarm-Recovery"; break;
    case VLA_SWARM_FINISH: text+="VSwarm-Finish"; break;
    default: text += "Unknown"; break;
  }

  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "mission_status";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING; 
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.z = 0.5;
  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0f;
  marker.text = text;

  marker.pose.position.x = fd_->odom_pos_(0) + 0.5;
  marker.pose.position.y = fd_->odom_pos_(1) + 0.5;
  marker.pose.position.z = fd_->odom_pos_(2) + 1.0;

  vis_marker_pub_.publish(marker);
}

void FastExplorationFSM::visualize(const ros::TimerEvent& e)
{
  displayMissionState();
  expl_manager_->visHgrid(fd_->odom_pos_);
}

void FastExplorationFSM::stopMotion()
{
  pubLocalGoal(fd_->odom_pos_, fd_->odom_yaw_, true);
}

void FastExplorationFSM::hardResetExploreArea(bool clear_occupancy, bool clear_posegraph) {
  (void)clear_posegraph;
  if (clear_occupancy)
    map_->resetOccupancyToUnknown();

  // 重置探索边界及所有由地图派生的探索状态，保留scene graph和topo map。
  map_->resetGlobalBox();
  Eigen::Vector3d global_box_min, global_box_max;
  map_->getGlobalBox(global_box_min, global_box_max);
  expl_manager_->frontier_finder_->frontierForceDeleteAll();
  expl_manager_->hgrid_->init(global_box_min, global_box_max);
  expl_manager_->ed_->frontiers_.clear();
  expl_manager_->ed_->frontiers_with_info_.clear();
  expl_manager_->ed_->dead_frontiers_.clear();
  expl_manager_->ed_->frontier_boxes_.clear();
  expl_manager_->ed_->global_tour_.clear();
  expl_manager_->ed_->global_tour_map_.clear();
  expl_manager_->ed_->path_next_goal_.clear();
  expl_manager_->ed_->last_grid_ids_.clear();
  expl_manager_->ed_->last_frontiers_with_info_.clear();
  expl_manager_->ed_->last_indices_.clear();
  expl_manager_->ed_->refined_ids_.clear();
  expl_manager_->ed_->n_points_.clear();
  expl_manager_->ed_->unrefined_points_.clear();
  expl_manager_->ed_->refined_points_.clear();
  expl_manager_->ed_->refined_views_.clear();
  expl_manager_->ed_->refined_views1_.clear();
  expl_manager_->ed_->refined_views2_.clear();
  expl_manager_->ed_->refined_tour_.clear();

  fd_->path_res_.clear();
  fd_->path_inx_ = 0;
  fd_->llm_plan_explore_counter_ = 0;
  fd_->explore_count_ = 0;
  has_made_area_decision_ = false;
  expl_area_id_ = -1;

  // 清除当前挂载的 polyhedron 引用，强制 planLLMExplore 在下一轮重新 mount。
  // 不清理 skeleton / area_handler 的数据（保留 VLM 检测物体与 polyhedron 的关联）。
  scene_graph_->cur_poly_ = nullptr;

  INFO_MSG_GREEN("=================================");
  INFO_MSG_GREEN("[FSM] : Explore Area Reset Done .");
  INFO_MSG_GREEN("=================================");
}

inline void FastExplorationFSM::geoPt2Vec3d(const geometry_msgs::Point &p_in, Eigen::Vector3d &p_out) {
  p_out.x() = p_in.x; p_out.y() = p_in.y; p_out.z() = p_in.z;
}
inline void FastExplorationFSM::vec3d2GeoPt(const Eigen::Vector3d &p_in, geometry_msgs::Point &p_out) {
  p_out.x = p_in.x(); p_out.y = p_in.y(); p_out.z = p_in.z();
}
inline geometry_msgs::Point FastExplorationFSM::vec3d2GeoPt(const Eigen::Vector3d &p_in) {
  geometry_msgs::Point p_out;
  p_out.x = p_in.x(); p_out.y = p_in.y(); p_out.z = p_in.z();
  return p_out;
}
inline Eigen::Vector3d FastExplorationFSM::geoPt2Vec3d(const geometry_msgs::Point &p_in) {
  Eigen::Vector3d p_out;
  p_out.x() = p_in.x; p_out.y() = p_in.y; p_out.z() = p_in.z;
  return p_out;
}
}  // namespace fast_planner

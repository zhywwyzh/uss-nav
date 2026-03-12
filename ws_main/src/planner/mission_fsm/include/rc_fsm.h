//
// Created by gwq on 11/26/24.
//

#ifndef SRC_RC_FSM_H
#define SRC_RC_FSM_H

#include "ros/ros.h"
#include "Eigen/Eigen"
#include "map_interface/map_interface.hpp"
#include "scene_graph/scene_graph.h"
#include "tf/transform_datatypes.h"
#include "mutex"

#include "mavros_msgs/RCIn.h"
#include "mavros_msgs/State.h"
#include "nav_msgs/Odometry.h"
#include "quadrotor_msgs/GoalSet.h"

#include "quadrotor_msgs/PositionCommand.h"
#include "quadrotor_msgs/TakeoffLand.h"
#include "quadrotor_msgs/EgoGoalSet.h"
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"
using ego_planner::MapInterface;

struct Vector3iHash {
    std::size_t operator()(const Eigen::Vector3i& vec) const {
      std::hash<int> hasher;
      size_t seed = 0;
      seed ^= hasher(vec.x()) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
      seed ^= hasher(vec.y()) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
      seed ^= hasher(vec.z()) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
      return seed;
    }
};

enum RCCtrlFSMState {
  INIT             = 0,
  WAIT_FOR_RC_FLAG = 1,
  EXECUTE_RC_CTRL  = 2
};

class RC_Data_t
{
public:

    double ch_[4];
    double ch10_, last_ch10_;

    mavros_msgs::RCIn msg_;
    mavros_msgs::State state_;
    bool is_armed_;
    ros::Time rcv_stamp_;

    bool activate_rc_ctrl_mode_;
    bool need_takeoff_land_cmd_pub_;
    bool have_effective_rc_cmd_;
    int  ch10_trigger_count_;
    int  ch10_value_change_count_;
    ros::Time ch10_trigger_last_time_;

    static constexpr double GEAR_SHIFT_VALUE = 0.75;
    static constexpr double API_MODE_THRESHOLD_VALUE = 0.75;
    static constexpr double REBOOT_THRESHOLD_VALUE = 0.5;
    static constexpr double DEAD_ZONE = 0.25;

    RC_Data_t();
    void check_validity();
    bool check_centered();
    void process_ch10_trigger();
    void check_ch10_trigger_once();
    void feed(mavros_msgs::RCInConstPtr pMsg);
    void feedMavrosState(mavros_msgs::StateConstPtr pMsg);
};

class Odom_Data_t
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d p_;
    Eigen::Vector3d v_;
    Eigen::Quaterniond q_;
    Eigen::Vector3d w_;
    double yaw_;

    nav_msgs::Odometry msg_;
    ros::Time rcv_stamp_;
    bool recv_new_msg_;

    Odom_Data_t();
    void feed(nav_msgs::OdometryConstPtr pMsg);
};

struct RCCtrlFSMData{
    ros::Time           now_time_, draw_safety_check_last_time_, draw_vel_arrow_last_time_, draw_text_last_time_;
    RCCtrlFSMState      fsm_state_, last_fsm_state_;
    RC_Data_t           rc_data_;
    Odom_Data_t         odom_data_;

    Eigen::Vector3d     pose_check_;
    Eigen::Vector3d     expected_pos_;
    Eigen::Vector3d     expected_speed_;
    double              expected_yaw_rate_;
    double              expected_yaw_;
    ros::Time           last_cmd_pub_time_;

    bool                is_odom_ready_{false};
    bool                is_rc_signal_ready_{false};
    bool                is_in_collision_traj_{false};
};

struct RCCtrlFSMParams{
  double max_speed_;               // m/s
  double max_acc_;                 // m/s^2
  double max_yaw_rate_;            // rad/s
  double rc_timeout_;              // s
  double odom_timeout_;            // s
  std::string odom_sub_topic_, rc_sub_topic_, px4ctrl_cmd_topic_;
  double      fsm_exec_freq_;

  bool        rc_reverse_pitch_;
  bool        rc_reverse_roll_;
  bool        rc_reverse_yaw_;
  bool        rc_reverse_thrust_;
};

class RCCtrlFSM {
public:
  typedef std::shared_ptr<RCCtrlFSM> Ptr;
  RCCtrlFSM(ros::NodeHandle& node, MapInterface::Ptr &map_interface);
  ~RCCtrlFSM();
private:
  std::map<int, std::string>        state_name_map_;
  ros::Subscriber                   rc_sub_, odom_sub_, px4_state_sub_, goal_from_station_sub_;
  ros::Publisher                    px4ctrl_cmd_pub_, rc_ctrl_marker_pub_,
                                    px4ctrl_takeoff_land_pub_, ego_goal_set_pub_, ego_goal_yaw_preset_pub_;
  ros::Timer                        fsm_exec_timer_, fsm_vis_timer_, skeleton_mount_point_timer_;
  std::shared_ptr<RCCtrlFSMData>    data_;
  std::shared_ptr<RCCtrlFSMParams>  params_;
  MapInterface::Ptr                 map_interface_;
  SceneGraph::Ptr                   scene_graph_;
  std::mutex                        mutex_;
  std::unique_ptr<std::thread>      rc_cmd_thread_;

  // callbacks
  void fsmExecTimerCallback(const ros::TimerEvent& event);
  void fsmVisTimerCallback(const ros::TimerEvent& event);
  void skeletonMountPointTimerCallback(const ros::TimerEvent& event);
  void goalFromStationCallback(const quadrotor_msgs::GoalSet::ConstPtr& msg);

  // safety check
  bool isRCReady(const ros::Time& now_time);
  bool isOdomReady(const ros::Time& now_time);
  bool checkDataRecvSafety();
  bool checkMovementSafety(double safe_distance, const Eigen::Vector3d & expected_vel);

  // process
  void fsmProcess();
  void rcCmdProcess();
  void setHoverWithRC();
  void changeFSMState(const std::string log_info, RCCtrlFSMState new_state);
  void holdPosition(const Eigen::Vector3d &pos, const double &yaw, bool use_ego);
  bool findAvailablePointInRange(Eigen::Vector3d &pos_refine, const double &range, const double & step_size);

  // tools
  template<typename T>
  void readParam(ros::NodeHandle &node, std::string param_name, T& param_val, T default_val);
  void drawVelArrow();
  void drawSaftyCheckPoint(Eigen::Vector3d &check_start, Eigen::Vector3d &check_end);
  void drawText();
};

#endif //SRC_RC_FSM_H

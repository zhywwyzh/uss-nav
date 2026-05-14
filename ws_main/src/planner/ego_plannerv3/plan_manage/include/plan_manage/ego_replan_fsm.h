#ifndef _REBO_REPLAN_FSM_H_
#define _REBO_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Twist.h>

#include <optimizer/poly_traj_optimizer.h>
#include <plan_env/grid_map.h>
#include <geometry_msgs/PoseStamped.h>
#include <quadrotor_msgs/EgoGoalSet.h>
#include <quadrotor_msgs/EgoPlannerResult.h>
#include <quadrotor_msgs/EgoStateTrigger.h>
#include <traj_utils/DataDisp.h>
#include <plan_manage/planner_manager.h>
#include <traj_utils/planning_visualization.h>
#include <traj_utils/PolyTraj.h>
#include <traj_utils/MINCOTraj.h>
#include <traj_utils/YawCmd.h>
#include <plan_manage/traj_server.h>

using std::vector;

namespace ego_planner
{

  class EGOReplanFSM {
  public:
    EGOReplanFSM() {}
    ~EGOReplanFSM();

    void init(ros::NodeHandle &nh);
    inline MapManager::Ptr getMapPtr() { return planner_manager_->map_; };

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  private:
    /* ---------- flag ---------- */
    enum FSM_EXEC_STATE
    {
      INIT,
      WAIT_TARGET,
      HANDLE_YAW,
      GEN_NEW_TRAJ,
      REPLAN_TRAJ,
      EXEC_TRAJ,
      EMERGENCY_STOP,
      SEQUENTIAL_START,
      CRASH_RECOVER,
      WAIT_YAW
    };
    enum TARGET_TYPE
    {
      MANUAL_TARGET = 1,
      EXPLORE_TARGET = 2,
      PRESET_TARGET = 3,
      REFENCE_PATH = 4
    };
    struct PlanRetStatistic
    {
      PLAN_RET ret{PLAN_RET::SUCCESS};
      int times{0};
      int keep_failure_times{0};
      ros::Time start_time{ros::Time(0.0)};
      std::queue<std::pair<ros::Time, PLAN_RET>> failure_histroy;
      double succ_calc_time{0.0};

      void setRet(const PLAN_RET r, const double time = -1.0);
      std::string show(bool print = true);
    } plan_ret_stat_;
    struct YAW_CMD
    {
      double des_yaw;
      bool yaw_reach;
      ros::Time cmd_time;
    } yaw_cmd_;

    /* planning utils */
    EGOPlannerManager::Ptr planner_manager_;
    PlanningVisualization::Ptr visualization_;
    traj_utils::DataDisp data_disp_;
    TrajServer traj_server_;

    /* parameters */
    int target_type_; // 1 mannual select, 2 hard code
    double no_replan_thresh_ /*, replan_thresh_*/;
    double waypoints_[50][3];
    int waypoint_num_, wpt_id_;
    // double planning_horizen_;
    double emergency_time_;
    double ego_state_trigger_pos_thresh_;
    double ego_state_trigger_vel_thresh_;
    double ego_state_trigger_acc_thresh_;
    double ego_state_trigger_yaw_rate_thresh_;
    double ego_state_trigger_hold_time_;
    bool flag_realworld_experiment_;
    bool enable_fail_safe_;
    bool enable_ground_height_measurement_;
    bool flag_escape_emergency_;
    bool flag_wait_crash_rec_;
    ros::Time crash_rec_start_time_;
    ros::Time last_density_eval_time_{ros::Time(0)};

    bool have_trigger_, have_target_, have_odom_, cur_traj_to_cur_target_, have_recv_pre_agent_, touch_goal_, mandatory_stop_;
    bool if_handle_yaw_{false};
    bool has_been_modified_;
    bool pending_goal_finish_trigger_;
    ros::Time goal_finish_stable_start_time_;
    FSM_EXEC_STATE exec_state_;

    Eigen::Vector3d start_pt_, start_vel_, start_acc_, start_jerk_; // start state
    Eigen::Vector3d glb_start_pt_, final_goal_;                     // goal state
    // Eigen::Vector3d local_target_pt_, local_target_vel_; // local target state
    Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_, odom_omega_;
    Eigen::Vector3d last_odom_vel_;
    bool odom_acc_ready_{false};
    bool traj_server_yaw_synced_{false};
    ros::Time last_odom_stamp_{ros::Time(0)};
    double odom_yaw_; // odometry state
    Eigen::Quaterniond odom_q_;
    Eigen::Vector3d odom_euler_;
    std::vector<Eigen::Vector3d> wps_;
    quadrotor_msgs::EgoPlannerResult ego_plan_result_;

    // handle yaw
    Eigen::Vector3d target_pos_;
    double target_yaw_;
    bool   target_look_forward_, target_yaw_low_speed_;
    void handleYaw();
    double aim_direction_; // rad
    bool yaw_init_finished_{false};

    /* ROS utils */
    ros::NodeHandle node_;
    ros::Timer exec_timer_, safety_timer_;
    ros::Subscriber waypoint_sub_, waypoint_sub_yaw_preset_sub_, odom_sub_, if_handle_yaw_sub_,
                    trigger_sub_, broadcast_ploytraj_sub_, mandatory_stop_sub_;
    ros::Publisher data_disp_pub_, broadcast_ploytraj_pub_, ground_height_pub_, state_pub_, exec_finish_trigger_pub_, ego_state_trigger_pub_;
    ros::Publisher ego_plan_state_pub_;

    /* state machine functions */
    void execFSMCallback(const ros::TimerEvent &e);
    void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
    void printFSMExecState() const;
    void planningReturnsChk();
    void evaluateEnvironmentDensity();

    /* safety */
    void checkCollision();
    bool callEmergencyStop(Eigen::Vector3d stop_pos);
    bool callCrashRecovery();

    /* local planning */
    PLAN_RET callReboundReplan(bool flag_use_last_optimal, bool flag_random_init, vector<DensityEvalRayData> *pathes);
    bool planFromGlobalTraj(const int trial_times = 1);
    bool planFromLocalTraj(const int trial_times = 1);
    bool getTrajPVAJ(const string data_source);
    void execTraj();

    /* global trajectory */
    void waypointCallback(const geometry_msgs::PoseStampedPtr &msg);
    void aimCallback(const quadrotor_msgs::EgoGoalSetPtr &msg);
    void aimCallbackYawPreset(const quadrotor_msgs::EgoGoalSetPtr &msg);
    void execAim();
    void readGivenWpsAndPlan();
    bool planNextWaypoint(const Eigen::Vector3d next_wp, const double next_yaw = 0.0, const bool look_forward = true, bool yaw_low_speed = false);
    bool mondifyInCollisionFinalGoal();

    /* input-output */
    void mandatoryStopCallback(const std_msgs::Empty &msg);
    void ifHandleYawCallback(const std_msgs::BoolConstPtr &msg);
    void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
    void triggerCallback(const geometry_msgs::PoseStampedPtr &msg);
    void RecvBroadcastMINCOTrajCallback(const traj_utils::MINCOTrajConstPtr &msg);
    void polyTraj2ROSMsg(traj_utils::PolyTraj *poly_msg, traj_utils::MINCOTraj *MINCO_msg);

    /* utils */
    void initEgoPlanResult();
    void updateEgoPlanResult(const Eigen::Vector3d goal, PLAN_RET status);

    /* ground height measurement */
    bool measureGroundHeight() const;
    bool measureGroundHeight2();
  };

} // namespace ego_planner

#endif

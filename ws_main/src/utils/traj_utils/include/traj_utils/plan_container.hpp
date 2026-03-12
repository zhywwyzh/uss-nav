#ifndef _PLAN_CONTAINER_H_
#define _PLAN_CONTAINER_H_

#include <Eigen/Eigen>
#include <vector>
#include <ros/ros.h>

#include <optimizer/poly_traj_utils.hpp>

using std::vector;

namespace ego_planner
{

  typedef std::vector<std::vector<std::pair<double, Eigen::Vector3d>>> PtsChk_t;

  struct EnterUnknownRegionInfo
  {
    bool enable{false};
    int cps_id;
    int K;
    Eigen::Vector3d fixed_pos;
  };

  struct GlobalTrajData
  {
    poly_traj::Trajectory traj;
    double global_start_time; // world time
    double duration;

    /* Global traj time.
       The corresponding global trajectory time of the current local target.
       Used in local target selection process */
    double glb_t_of_lc_tgt;
    /* Global traj time.
       The corresponding global trajectory time of the last local target.
       Used in initial-path-from-last-optimal-trajectory generation process */
    double last_glb_t_of_lc_tgt;
  };

  struct LocalTrajData
  {
    poly_traj::Trajectory traj;
    int drone_id; // A negative value indicates no received trajectories.
    int traj_id;
    double duration;
    double start_time{0.0}; // world time
    double end_time;        // world time
    double last_opt_cp_time;
    Eigen::Vector3d start_pos;
    double des_clearance;
    EnterUnknownRegionInfo uk_info;
  };

  typedef std::vector<LocalTrajData> SwarmTrajData;

  class TrajContainer
  {
  public:
    LocalTrajData local_traj;
    SwarmTrajData swarm_traj;

    TrajContainer()
    {
      local_traj.traj_id = 0;
    }
    ~TrajContainer() {}

    void setLocalTraj(const poly_traj::Trajectory &trajectory, const double last_opt_cp_time_in, const double &world_time,
                      const int drone_id = -1, const EnterUnknownRegionInfo *ent_uk_in = NULL)
    {
      local_traj.drone_id = drone_id;
      local_traj.traj_id++;
      local_traj.duration = trajectory.getTotalDuration();
      local_traj.start_pos = trajectory.getJuncPos(0);
      local_traj.start_time = world_time;
      local_traj.traj = trajectory;
      local_traj.last_opt_cp_time = last_opt_cp_time_in;
      if (ent_uk_in != NULL)
        local_traj.uk_info = *ent_uk_in;
      else
        local_traj.uk_info.enable = false;
    }
  };

  struct PlanParameters
  {
    /* planning algorithm parameters */
    double max_vel_{1.0}, max_acc_{2.0};                               // physical limits in planning
    double max_vel_user_ = max_vel_, max_acc_user_ = max_acc_;         // physical limits given by user
    double max_vel_prevplan_ = max_vel_, max_acc_prevplan_ = max_acc_; // physical limits of the previous planning
    bool desvel_changed_toomuch_{false};
    double polyTraj_piece_length; // distance between adjacient B-spline control points
    double planning_horizen_;
    bool use_multitopology_trajs;
    bool touch_goal;
    bool emergency_{false}; // ignore acc, jerk, snap limits to make the planning easier
    int drone_id;           // single drone: drone_id <= -1, swarm: drone_id >= 0
    enum MODE
    {
      SLOW,
      FAST
    } speed_mode{SLOW};

    /* processing time */
    double time_search_ = 0.0;
    double time_optimize_ = 0.0;
    double time_adjust_ = 0.0;
  };

  struct DensityEvalRayData
  {
    Eigen::Vector3d start_p{Eigen::Vector3d::Zero()};
    Eigen::Vector3d mid_p{Eigen::Vector3d::Zero()}; // mid_p may be further than end_p as end_p records the collision point
    Eigen::Vector3d end_p{Eigen::Vector3d::Zero()};
    double safe_l{0.0};
    double safe_margin{-std::numeric_limits<double>::max()};
    bool safe{false};
    double score{-std::numeric_limits<double>::max()};
    double norm_devi{std::numeric_limits<double>::max()};
    bool full_speed{false};
    double preferred_speed{0.0};
    double time{0};

    bool operator<(DensityEvalRayData b)
    {
      return this->preferred_speed < b.preferred_speed;
    }

    bool operator>(DensityEvalRayData b)
    {
      return this->preferred_speed > b.preferred_speed;
    }
  };

  class TicToc
  {
  public:
    ros::Time t0;

    TicToc() { t0 = ros::Time::now(); }
    void tic() { t0 = ros::Time::now(); }
    double toc(bool print = true)
    {
      double t_passed = (ros::Time::now() - t0).toSec() * 1000;
      if (print)
        ROS_INFO("passed time = %f ms", t_passed);
      return t_passed;
    }
  };

} // namespace ego_planner

#endif
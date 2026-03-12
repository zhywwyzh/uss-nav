#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <stdlib.h>

#include <optimizer/poly_traj_optimizer.h>
#include <traj_utils/DataDisp.h>
#include <plan_env/grid_map.h>
#include <traj_utils/plan_container.hpp>
#include <ros/ros.h>
#include <traj_utils/planning_visualization.h>
#include <optimizer/poly_traj_utils.hpp>

namespace ego_planner
{

  // Fast Planner Manager
  // Key algorithms of mapping and planning are called
  enum PLAN_RET
  {
    SUCCESS = 0,
    LOCAL_TGT_FAIL,
    INIT_FAIL,
    DEFAULT_FAIL
  };

  class EGOPlannerManager
  {
    // SECTION stable
  public:
    EGOPlannerManager();
    ~EGOPlannerManager();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PlanParameters pp_;
    MapManager::Ptr map_;
    TrajContainer traj_;
    std::list<DensityEvalRayData> his_dendat_; // historical density data

    /* main planning interface */
    void initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis = NULL);
    bool computeInitState(
        const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel,
        const Eigen::Vector3d &start_acc, const Eigen::Vector3d &glb_start_pt,
        const Eigen::Vector3d &final_goal, const bool flag_use_last_optimial,
        const bool flag_random_init, vector<DensityEvalRayData> *pathes,
        poly_traj::MinJerkOpt &initMJO, bool &touch_goal);
    double computeInitDuration(
        const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel,
        const Eigen::Vector3d &local_target_pt, const Eigen::Vector3d &local_target_vel);
    bool computePlanningParams(const double vel);
    bool computePlanningHorizon(const double vel);
    bool computeMINCOParams(const double planning_horizon, const double vel);
    PLAN_RET reboundReplan(
        const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel,
        const Eigen::Vector3d &start_acc, const Eigen::Vector3d &start_jerk,
        const Eigen::Vector3d &glb_start_pt, const Eigen::Vector3d &final_goal,
        const bool flag_use_last_optimial, const bool flag_random_init,
        vector<DensityEvalRayData> *pathes, bool &touch_goal);
    bool densityEval(const Eigen::Vector3d start_pt, const Eigen::Vector3d end_pt,
                     DensityEvalRayData *best_ray = NULL, vector<DensityEvalRayData> *all_rays = NULL) const;
    bool DetVelByDensity(DensityEvalRayData &best_ray);
    bool EmergencyStop(Eigen::Vector3d stop_pos);
    bool checkCollision(int drone_id);
    bool OnePieceTrajGen(Eigen::Vector3d start_pos, Eigen::Vector3d end_pos);
    poly_traj::Trajectory OnePieceTrajGen(
        Eigen::Vector3d start_pos, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
        Eigen::Vector3d end_pos, Eigen::Vector3d end_vel, Eigen::Vector3d end_acc, double duration);
    Eigen::Vector3d GenRandomMidPt(const Eigen::Vector3d start_pt, const Eigen::Vector3d end_pt);
    bool getNearbySafePt(const Eigen::Vector3d unsafe_pt, const int max_grid, Eigen::Vector3d &safe_pt);
    bool setLocalTrajFromOpt(const poly_traj::MinJerkOpt &opt, const bool touch_goal, const bool set_uk_info = false);
    inline double getSwarmClearance(void) { return poly_traj_opt_->get_swarm_clearance_(); }
    inline int getCpsNumPrePiece(void) { return poly_traj_opt_->get_cps_num_prePiece_(); }
    inline int getContinousFailureCount(void) { return continous_failures_count_; }

  private:
    PlanningVisualization::Ptr visualization_;

    PolyTrajOptimizer::Ptr poly_traj_opt_;

    int continous_failures_count_{0}, success_cnt_{0}, failure_cnt_{0};
    double sum_success_time_{0.0};

  public:
    typedef unique_ptr<EGOPlannerManager> Ptr;

    // !SECTION
  };
} // namespace ego_planner

#endif
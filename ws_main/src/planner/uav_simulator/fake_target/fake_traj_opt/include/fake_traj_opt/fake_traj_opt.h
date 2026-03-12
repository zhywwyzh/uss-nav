#pragma once
#include <ros/ros.h>
#include <visualization/visualization.hpp>
#include "minco.hpp"
#include <math.h>
// #define DEBUG

#ifdef DEBUG
#define DEBUG_MSG(str) do { std::cout << str << std::endl; } while( false )
#else
#define DEBUG_MSG(str) do { } while ( false )
#endif
namespace traj_opt {

class TrajOpt {
 public:
  ros::NodeHandle nh_;
  // # pieces and # key points
  int N_, K_, dim_t_, dim_p_;
  // weight for time regularization term
  double rhoT_;
  // collision avoiding and dynamics paramters
  double vmax_, amax_;
  double rhoP_, rhoV_, rhoA_;
  double rhoA_t_, rhoV_t_, rhoTracking_t_;
  double rhoA_l_, rhoV_l_, rhoTracking_l_;
  double rhoTracking_, rhosVisibility_;
  double clearance_d_, tolerance_d_, theta_clearance_;
  double tolerance_d_l_, tolerance_d_t_;
  double track_angle_expect_;
  double rhoLanding_v_;
  double landing_a_xy_limit_, landing_a_z_limit_;
  // corridor
  std::vector<Eigen::MatrixXd> cfgVs_;
  std::vector<Eigen::MatrixXd> cfgHs_;
  // Minimum Jerk Optimizer
  minco::MinJerkOpt jerkOpt_;
  // weight for each vertex
  Eigen::VectorXd p_;
  // duration of each piece of the trajectory
  Eigen::VectorXd t_;
  double* x_;
  double sum_T_;

  std::vector<Eigen::Vector3d> tracking_ps_;
  Eigen::Vector3d target_v_;
  std::vector<Eigen::Vector3d> tracking_visible_ps_;
  std::vector<double> tracking_thetas_;
  double tracking_dur_;
  double tracking_dist_;
  double tracking_dt_;
  double landing_dur_;

  std::shared_ptr<visualization::Visualization> visPtr_;
  bool is_debug = false;

  // polyH utils
  bool extractVs(const std::vector<Eigen::MatrixXd>& hPs,
                 std::vector<Eigen::MatrixXd>& vPs) const;

 public:
  TrajOpt(ros::NodeHandle& nh);
  ~TrajOpt() {}

  void setBoundConds(const Eigen::MatrixXd& iniState, const Eigen::MatrixXd& finState);
  int optimize(const double& delta = 1e-4);
  bool generate_traj(const Eigen::MatrixXd& iniState,
                     const Eigen::MatrixXd& finState,
                     const std::vector<Eigen::Vector3d>& target_predcit,
                     const std::vector<Eigen::Vector3d>& visible_ps,
                     const std::vector<double>& thetas,
                     const std::vector<Eigen::MatrixXd>& hPolys,
                     Trajectory& traj);
  bool generate_traj(const Eigen::MatrixXd& iniState,
                     const Eigen::MatrixXd& finState,
                     const std::vector<Eigen::Vector3d>& target_predcit,
                     const Eigen::Vector3d& target_v,
                     const std::vector<Eigen::MatrixXd>& hPolys,
                     Trajectory& traj);
  bool generate_traj(const Eigen::MatrixXd& iniState,
                     const Eigen::MatrixXd& finState,
                     const std::vector<Eigen::MatrixXd>& hPolys,
                     Trajectory& traj);

  void addTimeIntPenalty(double& cost);
  void addTimeCost(double& cost);
  bool grad_cost_p_corridor(const Eigen::Vector3d& p,
                            const Eigen::MatrixXd& hPoly,
                            Eigen::Vector3d& gradp,
                            double& costp);
  bool grad_cost_p_tracking(const Eigen::Vector3d& p,
                            const Eigen::Vector3d& target_p,
                            Eigen::Vector3d& gradp,
                            double& costp);
  bool grad_cost_p_landing(const Eigen::Vector3d& p,
                           const Eigen::Vector3d& target_p,
                           Eigen::Vector3d& gradp,
                           double& costp);
  bool grad_cost_v_landing(const Eigen::Vector3d& p,
                           const Eigen::Vector3d& v,
                           const Eigen::Vector3d& target_p,
                           const Eigen::Vector3d& target_v,
                           Eigen::Vector3d& gradv,
                           double& costv);
  bool grad_cost_visibility(const Eigen::Vector3d& p,
                            const Eigen::Vector3d& center,
                            const Eigen::Vector3d& vis_p,
                            const double& theta,
                            Eigen::Vector3d& gradp,
                            double& costp);
  bool grad_cost_v(const Eigen::Vector3d& v,
                   Eigen::Vector3d& gradv,
                   double& costv);
  bool grad_cost_a(const Eigen::Vector3d& a,
                   Eigen::Vector3d& grada,
                   double& costa);
  void set_track_angle(const double& angle);
  void set_debug();
};

inline void TrajOpt::set_debug(){
  is_debug = true;
}

inline void TrajOpt::set_track_angle(const double& angle){
    track_angle_expect_ = angle;
}

}  // namespace traj_opt
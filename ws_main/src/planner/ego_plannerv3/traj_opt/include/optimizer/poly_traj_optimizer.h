#ifndef _POLY_TRAJ_OPTIMIZER_H_
#define _POLY_TRAJ_OPTIMIZER_H_

#include <Eigen/Eigen>
#include <path_searching/dyn_a_star.h>
#include <plan_env/grid_map.h>
#include <ros/ros.h>
#include "optimizer/lbfgs.hpp"
#include <traj_utils/plan_container.hpp>
#include "poly_traj_utils.hpp"
#include <fstream>

namespace ego_planner
{

#define UNKNOWN_COUND_THRES 20

  class ConstraintPoints
  {
  public:
    int cp_size; // deformation points
    Eigen::MatrixXd points;
    std::vector<std::vector<Eigen::Vector3d>> base_point; // The point at the statrt of the direction vector (collision point)
    std::vector<std::vector<Eigen::Vector3d>> direction;  // Direction vector, must be normalized.
    std::vector<bool> flag_got_pvpair;                    // A flag that used in many places. Initialize it everytime before using it.
    std::vector<double> vel_limit;
    std::vector<double> acc_limit;
    std::vector<double> t;
    std::vector<std::pair<std::pair<Eigen::Vector3d, double>, Eigen::Vector3d>> curve_fitting;
    bool dyn_limit_valid;
    EnterUnknownRegionInfo ent_uk;

    void resize_cp(const int size_set)
    {
      cp_size = size_set;
      dyn_limit_valid = false;

      base_point.clear();
      direction.clear();
      flag_got_pvpair.clear();
      vel_limit.clear();
      acc_limit.clear();
      t.clear();
      curve_fitting.clear();

      points.resize(3, size_set);
      base_point.resize(cp_size);
      direction.resize(cp_size);
      flag_got_pvpair.resize(cp_size);
      vel_limit.resize(cp_size);
      acc_limit.resize(cp_size);
      t.resize(cp_size);
      curve_fitting.resize(cp_size);

      ent_uk.enable = false;
    }

    void segment(ConstraintPoints &buf, const int start, const int end)
    {
      if (start < 0 || end >= cp_size || points.rows() != 3)
      {
        ROS_ERROR("Wrong segment index! start=%d, end=%d", start, end);
        return;
      }

      buf.resize_cp(end - start + 1);
      buf.points = points.block(0, start, 3, end - start + 1);
      buf.cp_size = end - start + 1;
      for (int i = start; i <= end; i++)
      {
        buf.base_point[i - start] = base_point[i];
        buf.direction[i - start] = direction[i];
      }
    }

    static inline int one_thirds_id(const int cps_nums, const bool use_all)
    {
      return use_all ? cps_nums - 1 : (cps_nums - 2) / 3 + 1;
    }

    static inline int two_thirds_id(const int cps_nums, const bool use_all)
    {
      return use_all ? cps_nums - 1 : cps_nums - 1 - (cps_nums - 2) / 3;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };

  struct OptFsm
  {
    enum ACTION
    {
      ROUGH_REBOUND,
      FINE_REBOUND,
      SUCCESS_RET,
      FAILED_RET,
      ADJUST_SPEED,
      ADJUST_WEI
    };

    ACTION act = ACTION::FAILED_RET;

    std::string show(void)
    {
      switch (act)
      {
      case ROUGH_REBOUND:
        return std::string("ROUGH_REBOUND");
      case FINE_REBOUND:
        return std::string("FINE_REBOUND");
      case SUCCESS_RET:
        return std::string("SUCCESS_RET");
      case FAILED_RET:
        return std::string("FAILED_RET");
      case ADJUST_SPEED:
        return std::string("ADJUST_SPEED");
      case ADJUST_WEI:
        return std::string("ADJUST_WEI");
      default:
        return std::string("UNKNOWN");
      }
    }
  };

  class PolyTrajOptimizer
  {

  private:
    MapManager::Ptr map_;
    dyn_a_star::AStar::Ptr a_star_;
    poly_traj::MinJerkOpt jerkOpt_;
    SwarmTrajData *swarm_trajs_{NULL}; // Can not use shared_ptr and no need to free
    ConstraintPoints cps_;
    std::vector<std::pair<Eigen::Vector3d, double>> restrict_plane_;
    PlanParameters pp_cpy_;

    int drone_id_;
    int cps_num_prePiece_, cps_num_prePiece_Long_; // number of distinctive constraint points each piece
    int variable_num_;                             // optimization variables
    int piece_num_;                                // poly traj piece numbers
    int iter_num_, total_iter_num_;                // iteration of the solver
    std::vector<double> min_ellip_dist2_;          // min trajectory distance in swarm
    bool touch_goal_;
    struct MultitopologyData_t
    {
      bool use_multitopology_trajs{false};
      bool initial_obstacles_avoided{false};
    } multitopology_data_;
    double maxv_measure_, maxa_measure_, maxj_measure_;
    Eigen::Vector3d uk_vel_measure_, uk_pos_measure_;
    ros::Time opt_start_time_;

    enum FORCE_STOP_OPTIMIZE_TYPE
    {
      DONT_STOP,
      STOP_FOR_REBOUND,
      STOP_FOR_ERROR
    } force_stop_type_;

    /* optimization parameters */
    double wei_obs_, wei_obs_soft_; // obstacle weight
    double wei_trust_region_;
    double wei_curve_fitting_;
    double wei_plane_;
    double wei_swarm_, wei_swarm_mod_;                                             // swarm weight
    double wei_feas_, wei_feas_mod_;                                               // feasibility weight
    double wei_sqrvar_;                                                            // squared variance weight
    double wei_time_;                                                              // time weight
    double obs_clearance_, obs_clearance4_, obs_clearance_soft_, swarm_clearance_; // safe distance
    double max_vel_, max_acc_, max_jer_, max_sna_;                                 // dynamic limits
    Eigen::Vector3d start_jerk_;

    double t_now_;

  public:
    PolyTrajOptimizer() {}
    ~PolyTrajOptimizer() {}

    enum CHK_RET
    {
      OBS_FREE,
      ERR,
      FINISH,
      TIME_LIM
    };

    /* set variables */
    void setParam(ros::NodeHandle &nh);
    void setEnvironment(const MapManager::Ptr map);
    void setControlPoints(const Eigen::MatrixXd &points);
    void setSwarmTrajs(SwarmTrajData *swarm_trajs_ptr);
    void setDroneId(const int drone_id);
    void setIfTouchGoal(const bool touch_goal);
    void setConstraintPoints(ConstraintPoints cps);
    void setUseMultitopologyTrajs(bool use_multitopology_trajs);
    void setMaxVelAcc(double max_vel, double max_acc);
    void setCPsNumPerPiece(const int N);
    void setPlanParametersCopy(const PlanParameters &pp_cpy);

    /* helper functions */
    inline const ConstraintPoints &getControlPoints(void) { return cps_; }
    inline const poly_traj::MinJerkOpt &getMinJerkOpt(void) { return jerkOpt_; }
    inline int get_cps_num_prePiece_(void) { return cps_num_prePiece_; }
    inline double get_swarm_clearance_(void) { return swarm_clearance_; }

    /* main planning API */
    bool optimizeTrajectory(const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
                            const Eigen::MatrixXd &initInnerPts, const Eigen::VectorXd &initT,
                            double &final_cost);

    bool optimizeTrajectoryShapeOnly(const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
                                     const Eigen::MatrixXd &initInnerPts, const Eigen::VectorXd &initT,
                                     const int CPsNumPerPiece, double &final_cost);

    bool optimizeTrajectoryTimeOnly(const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
                                    const Eigen::MatrixXd &initInnerPts, const Eigen::VectorXd &initT,
                                    double &final_cost);

    bool computePointsToCheck(poly_traj::Trajectory &traj, int id_end, PtsChk_t &pts_check);

    bool normalityCheck();

    // std::vector<std::pair<int, int>> finelyCheckConstraintPointsOnly(Eigen::MatrixXd &init_points);

    /* check collision and set {p,v} pairs to constraint points */
    CHK_RET finelyCheckAndSetConstraintPoints(std::vector<std::pair<int, int>> &segments,
                                              vector<vector<Eigen::Vector3d>> &a_star_pathes,
                                              const poly_traj::MinJerkOpt &pt_data,
                                              const int cps_num_prePiece,
                                              const bool flag_first_init /*= true*/);

    bool roughlyCheckConstraintPoints(void);

    Eigen::Vector3d tryExtendAndChkP(Eigen::Vector3d p, Eigen::Vector3d v, Eigen::Vector3d q);

    bool allowRebound(void);

    void computeVelLim(const Eigen::MatrixXd &iniState, Eigen::MatrixXd finState);

    void prepareFittedCurve();

    /* multi-topo support */
    std::vector<ConstraintPoints> distinctiveTrajs(vector<std::pair<int, int>> segments);

  private:
    /* callbacks by the L-BFGS optimizer */
    static double costFunctionCallback(void *func_data, const double *x, double *grad, const int n);
    static double ShapeOnlyCostFunctionCallback(void *func_data, const double *x, double *grad, const int n);

    static int earlyExitCallback(void *func_data, const double *x, const double *g,
                                 const double fx, const double xnorm, const double gnorm,
                                 const double step, int n, int k, int ls);

    static double stepSizeBound(void *func_data, const double *xp, const double *d, const int n);

    /* mappings between real world time and unconstrained virtual time */
    template <typename EIGENVEC>
    void RealT2VirtualT(const Eigen::VectorXd &RT, EIGENVEC &VT);

    template <typename EIGENVEC>
    void VirtualT2RealT(const EIGENVEC &VT, Eigen::VectorXd &RT);

    template <typename EIGENVEC, typename EIGENVECGD>
    void VirtualTGradCost(const Eigen::VectorXd &RT, const EIGENVEC &VT,
                          const Eigen::VectorXd &gdRT, EIGENVECGD &gdVT,
                          double &costT);

    /* gradient and cost evaluation functions */
    template <typename EIGENVEC>
    void initAndGetSmoothnessGradCost2PT(EIGENVEC &gdT, double &cost);

    void initAndGetSmoothnessGradCost2P(double &cost);

    template <typename EIGENVEC>
    void addPVAJGradCost2CT(EIGENVEC &gdT, Eigen::VectorXd &costs, const int &K);

    void addPGradCost2C(Eigen::VectorXd &costs, const int &N, const int &K);

    bool obstacleGradCostP(const int i_dp,
                           const Eigen::Vector3d &p,
                           Eigen::Vector3d &gradp,
                           double &costp);

    bool ESDFGradCostP(const int i_dp,
                       const Eigen::Vector3d &p,
                       Eigen::Vector3d &gradp,
                       double &costp);

    bool restrictplaneGradCostP(const int i_piece,
                                const Eigen::Vector3d &p,
                                Eigen::Vector3d &gradp,
                                double &costp);

    bool cpsDistGradCostP(const int cps_id,
                          const Eigen::Vector3d &p,
                          Eigen::Vector3d &gradp,
                          double &costp);

    bool FixUnknwonPosGradCostP(const int cps_id,
                                const Eigen::Vector3d &p,
                                const Eigen::Vector3d &v,
                                Eigen::Vector3d &gradp,
                                double &costp,
                                Eigen::Vector3d &gradv,
                                double &costv);

    bool CurveFittingGradCostP(const int cps_id,
                               const Eigen::Vector3d &p,
                               Eigen::Vector3d &gradp,
                               double &costp);

    bool swarmGradCostP(const int i_dp,
                        const double t,
                        const Eigen::Vector3d &p,
                        const Eigen::Vector3d &v,
                        Eigen::Vector3d &gradp,
                        double &gradt,
                        double &grad_prev_t,
                        double &costp);

    bool feasibilityGradCostV(const int i_dp,
                              const Eigen::Vector3d &v,
                              Eigen::Vector3d &gradv,
                              double &costv);

    bool feasibilityGradCostA(const int i_dp,
                              const Eigen::Vector3d &a,
                              Eigen::Vector3d &grada,
                              double &costa);

    bool feasibilityGradCostJ(const int i_dp,
                              const Eigen::Vector3d &j,
                              Eigen::Vector3d &gradj,
                              double &costj);

    bool feasibilityGradCostS(const int i_dp,
                              const Eigen::Vector3d &s,
                              Eigen::Vector3d &grads,
                              double &costs);

    void distanceSqrVarianceWithGradCost2p(const Eigen::MatrixXd &ps,
                                           Eigen::MatrixXd &gdp,
                                           double &var);

    void lengthVarianceWithGradCost2p(const Eigen::MatrixXd &ps,
                                      const int n,
                                      Eigen::MatrixXd &gdp,
                                      double &var);

  public:
    typedef unique_ptr<PolyTrajOptimizer> Ptr;
  };

} // namespace ego_planner
#endif
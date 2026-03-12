#include "optimizer/poly_traj_optimizer.h"

using namespace std;

#define VERBOSE_OUTPUT true
#define PRINTF_COND(STR, ...) \
  if (VERBOSE_OUTPUT)         \
  printf(STR, __VA_ARGS__)

namespace ego_planner
{
  /* main planning API */
  bool PolyTrajOptimizer::optimizeTrajectory(
      const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
      const Eigen::MatrixXd &initInnerPts, const Eigen::VectorXd &initT,
      double &final_cost)
  {
    if (initInnerPts.cols() != (initT.size() - 1))
    {
      ROS_ERROR("initInnerPts.cols() != (initT.size()-1)");
      return false;
    }
    // for (int i = 1; i < initT.size(); ++i)
    //   if (abs(initT(i) - initT(0)) > 1e-10)
    //   {
    //     ROS_ERROR("only support identical time interval now!");
    //     return false;
    //   }

    ROS_INFO("max_vel_=%f, max_acc_=%f", max_vel_, max_acc_);

    // Preparision 1: Some mise params
    ros::Time t0 = ros::Time::now(), t1, t2;
    opt_start_time_ = ros::Time::now();
    int restart_nums = 0, rebound_times = 0;
    bool /*flag_force_return, flag_still_unsafe, flag_success, */ flag_swarm_too_close, flag_dyn_infeas;
    // bool criterion1 = true, criterion2 = true, criterion3 = false;
    multitopology_data_.initial_obstacles_avoided = false;
    wei_swarm_mod_ = wei_swarm_;
    wei_feas_mod_ = wei_feas_;
    constexpr int MAX_RESTART_TIMES = 3;
    constexpr int MAX_REBOUND_TIMES = 20;
    cps_.ent_uk.enable = false;
    cps_.dyn_limit_valid = false;
    OptFsm state;
    total_iter_num_ = 0;

    // Preparision 2: Trajectory related params
    t_now_ = ros::Time::now().toSec();
    start_jerk_ = iniState.col(3);
    piece_num_ = initT.size();
    jerkOpt_.reset(iniState.leftCols(3), finState, piece_num_);
    // variable_num_ = 3 * (piece_num_ - 1) + 1; // identical t
    // double x_init[variable_num_];
    // memcpy(x_init, initInnerPts.data(), initInnerPts.size() * sizeof(x_init[0]));
    // Eigen::Map<Eigen::VectorXd> Vt(x_init + initInnerPts.size(), 1);
    // RealT2VirtualT(initT.block<1, 1>(0, 0), Vt);
    variable_num_ = 4 * (piece_num_ - 1) + 1;
    double x_init[variable_num_];
    memcpy(x_init, initInnerPts.data(), initInnerPts.size() * sizeof(x_init[0]));
    Eigen::Map<Eigen::VectorXd> Vt(x_init + initInnerPts.size(), initT.size());
    RealT2VirtualT(initT, Vt);
    min_ellip_dist2_.resize(swarm_trajs_->size());

    /*************restrict_plane_**************/
    Eigen::Vector3d v_s2e = finState.col(0) - iniState.col(0);
    restrict_plane_.clear();
    for (int i = 0; i < (piece_num_ - 1); ++i)
    {
      restrict_plane_.push_back(std::make_pair(v_s2e, -v_s2e.dot(initInnerPts.col(i))));
    }
    /*************restrict_plane_**************/

    // Preparision 3: LBFGS related params
    lbfgs::lbfgs_parameter_t lbfgs_params;
    lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
    lbfgs_params.max_linesearch = 10;
    lbfgs_params.mem_size = 16;
    lbfgs_params.max_iterations = 100;
    lbfgs_params.past = 3;
    // lbfgs_params.delta = 1.0e-3;
    lbfgs_params.g_epsilon = 1.0e-2;
    // adjust params based on experiments
    const double path_dist = (iniState.col(0) - finState.col(0)).norm();
    if (path_dist < 1.0)
      lbfgs_params.delta = 1.0e-5;
    if (path_dist < 2.0)
      lbfgs_params.delta = 1.0e-4;
    else
      lbfgs_params.delta = 1.0e-3;

    do
    {
      /* ---------- prepare ---------- */
      iter_num_ = 0;
      force_stop_type_ = DONT_STOP;
      flag_swarm_too_close = false;

      /* ---------- optimize ---------- */
      t1 = ros::Time::now();
      int result = lbfgs::lbfgs_optimize(
          variable_num_,
          x_init,
          &final_cost,
          PolyTrajOptimizer::costFunctionCallback,
          PolyTrajOptimizer::stepSizeBound,
          PolyTrajOptimizer::earlyExitCallback,
          this,
          &lbfgs_params);

      t2 = ros::Time::now();
      double time_ms = (t2 - t1).toSec() * 1000;
      double total_time_ms = (t2 - t0).toSec() * 1000;

      /* ---------- get result and check collision ---------- */

      if (normalityCheck())
      {
        if (result == lbfgs::LBFGS_CONVERGENCE ||
            result == lbfgs::LBFGSERR_MAXIMUMITERATION ||
            result == lbfgs::LBFGS_ALREADY_MINIMIZED ||
            result == lbfgs::LBFGS_STOP ||
            result == lbfgs::LBFGSERR_MAXIMUMLINESEARCH)
        {
          if (result != lbfgs::LBFGS_CONVERGENCE &&
              result != lbfgs::LBFGS_ALREADY_MINIMIZED &&
              result != lbfgs::LBFGS_STOP)
          {
            /* NOTE! */
            // lbfgs::LBFGSERR_MAXIMUMLINESEARCH is always caused by small return conditions:
            ROS_WARN_COND(VERBOSE_OUTPUT, "Accepted error. Return = %d, %s", result, lbfgs::lbfgs_strerror(result));
          }

          // flag_force_return = false;

          /* double check: fine collision check */
          std::vector<std::pair<int, int>> segments_nouse;
          vector<vector<Eigen::Vector3d>> Astar_nouse;
          // check swarm distance
          for (size_t i = 0; i < swarm_trajs_->size(); ++i)
            flag_swarm_too_close |= min_ellip_dist2_[i] < pow((swarm_clearance_ + swarm_trajs_->at(i).des_clearance) * 0.95, 2);
          // check dynamical feasibility
          // flag_dyn_infeas = (maxv_measure_ - max_vel_ > 0.2 && sqrt(maxv_measure_) / max_vel_ > 1.1) ||
          //                   (maxa_measure_ - max_acc_ > 0.6 && sqrt(maxa_measure_) / max_acc_ > 1.1); // this check seems useless
          flag_dyn_infeas = false;
          // PRINTF_COND("v:%5.3f : %5.3f, a:%5.3f : %5.3f, j:%5.3f : %5.3f\n", sqrt(maxv_measure_), max_vel_, sqrt(maxa_measure_), max_acc_, sqrt(maxj_measure_), max_jer_);
          // check collision to obstacles
          if (!flag_swarm_too_close && !flag_dyn_infeas)
          {
            PolyTrajOptimizer::CHK_RET ret =
                finelyCheckAndSetConstraintPoints(segments_nouse, Astar_nouse, jerkOpt_, cps_num_prePiece_, false);
            if (ret == CHK_RET::OBS_FREE)
            {
              // flag_success = true;
              PRINTF_COND("\033[32miter=%d/%d,time(ms)=%5.3f,total_t(ms)=%5.3f,cost=%5.3f,action=%s\n\033[0m",
                          iter_num_, total_iter_num_, time_ms, total_time_ms, final_cost, state.show().c_str());

              if (state.act != OptFsm::ADJUST_SPEED) // fix the positon and velocity when entering the unknown region
              {
                restart_nums = 0; // clear
                computeVelLim(iniState, finState);
                prepareFittedCurve();
                // lbfgs_params.delta = 1.0e-5;
                // lbfgs_params.g_epsilon = 1.0e-5;
                state.act = OptFsm::ADJUST_SPEED;
              }
              else
              {
                if (cps_.ent_uk.enable && cps_.dyn_limit_valid &&
                    ((uk_pos_measure_ - cps_.ent_uk.fixed_pos).norm() > 0.3 || uk_vel_measure_.norm() > 0.6))
                {
                  ROS_WARN_COND(VERBOSE_OUTPUT, "The position (%f %f %f) <-> (%f %f %f) difference (%f) or velocity (%f) when enter the unknown regin is still too large!",
                                cps_.ent_uk.fixed_pos(0), cps_.ent_uk.fixed_pos(1), cps_.ent_uk.fixed_pos(2),
                                uk_pos_measure_(0), uk_pos_measure_(1), uk_pos_measure_(2),
                                (uk_pos_measure_ - cps_.ent_uk.fixed_pos).norm(), uk_vel_measure_.norm());
                }
                cps_.dyn_limit_valid = false;
                state.act = OptFsm::SUCCESS_RET;
              }
            }
            else if (ret == CHK_RET::TIME_LIM) // A total fail
            {
              PRINTF_COND("\033[32miter=%d/%d,time(ms)=%5.3f, action=%s->FAILED_RET, TIME_LIM exceeded\n\033[0m",
                          iter_num_, total_iter_num_, time_ms, state.show().c_str());
              state.act = OptFsm::FAILED_RET;
            }
            else
            {
              // A not-blank return value means collision to obstales
              restart_nums++;
              if (restart_nums < MAX_RESTART_TIMES)
              {
                PRINTF_COND("\033[32miter=%d/%d,time(ms)=%5.3f, action=%s->FINE_REBOUND, fine check collided, keep optimizing\n\033[0m",
                            iter_num_, total_iter_num_, time_ms, state.show().c_str());
                state.act = OptFsm::FINE_REBOUND;
              }
              else
              {
                PRINTF_COND("\033[32miter=%d/%d,time(ms)=%5.3f, action=%s->FAILED_RET, fine check collided, reached MAX_RESTART_TIMES\n\033[0m",
                            iter_num_, total_iter_num_, time_ms, state.show().c_str());
                state.act = OptFsm::FAILED_RET;
              }
            }
          }
          else
          {
            restart_nums++;
            if (restart_nums < MAX_RESTART_TIMES)
            {
              if (flag_swarm_too_close)
              {
                PRINTF_COND("Swarm clearance not satisfied, keep optimizing. iter=%d/%d,time(ms)=%5.3f, action=%s, wei_swarm_mod_=%f -> %f\n",
                            iter_num_, total_iter_num_, time_ms, state.show().c_str(), wei_swarm_mod_, wei_swarm_mod_ * 10);
                wei_swarm_mod_ *= 10;
              }
              if (flag_dyn_infeas)
              {
                PRINTF_COND("Dynamical infeasible, keep optimizing. iter=%d/%d,time(ms)=%5.3f, action=%s, wei_feas_mod_=%f -> %f\n",
                            iter_num_, total_iter_num_, time_ms, state.show().c_str(), wei_feas_mod_, wei_feas_mod_ * 10);
                PRINTF_COND("v:%5.3f : %5.3f, a:%5.3f : %5.3f, j:%5.3f : %5.3f\n", sqrt(maxv_measure_), max_vel_, sqrt(maxa_measure_), max_acc_, sqrt(maxj_measure_), max_jer_);
                wei_feas_mod_ *= 10;
              }
              state.act = OptFsm::ADJUST_WEI;
            }
            else
            {
              PRINTF_COND("\033[32miter=%d/%d,time(ms)=%5.3f, action=%s->FAILED_RET, swarm clearance or feasibility not satisfied, reached MAX_RESTART_TIMES\n\033[0m",
                          iter_num_, total_iter_num_, time_ms, state.show().c_str());
              state.act = OptFsm::FAILED_RET;
            }
          }
        }
        else if (result == lbfgs::LBFGSERR_CANCELED)
        {
          // flag_force_return = true;
          rebound_times++;
          if (force_stop_type_ == STOP_FOR_REBOUND && rebound_times <= MAX_REBOUND_TIMES)
          {
            PRINTF_COND("iter=%d/%d, time(ms)=%f, action=%s->ROUGH_REBOUND, rebound\n", iter_num_, total_iter_num_, time_ms, state.show().c_str());
            state.act = OptFsm::ROUGH_REBOUND;
          }
          else if (rebound_times > MAX_REBOUND_TIMES)
          {
            PRINTF_COND("iter=%d, time(ms)=%f, rebound to MAX_REBOUND_TIMES\n", iter_num_, time_ms);
            state.act = OptFsm::FAILED_RET;
          }
          else // force_stop_type_ == STOP_FOR_ERROR
          {
            PRINTF_COND("iter=%d, time(ms)=%f, force_stop_type_ == STOP_FOR_ERROR\n", iter_num_, time_ms);
            state.act = OptFsm::FAILED_RET;
          }
        }
        else
        {
          PRINTF_COND("iter=%d/%d, time(ms)=%f, action=%s->FAILED_RET, error\n", iter_num_, total_iter_num_, time_ms, state.show().c_str());
          ROS_WARN_COND(VERBOSE_OUTPUT, "Solver error. Return = %d, %s. Skip this planning.", result, lbfgs::lbfgs_strerror(result));
          state.act = OptFsm::FAILED_RET;
        }
      }
      else
      {
        state.act = OptFsm::FAILED_RET;
        ROS_WARN_COND(VERBOSE_OUTPUT, "Solver diverged. Skip this planning.");
      }

      if (VERBOSE_OUTPUT)
        cout << "Opt T=" << jerkOpt_.getTraj().getDurations().transpose() << endl;

    } while (state.act != OptFsm::SUCCESS_RET && state.act != OptFsm::FAILED_RET);

    return state.act == OptFsm::SUCCESS_RET;
  }

  bool PolyTrajOptimizer::optimizeTrajectoryShapeOnly(
      const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
      const Eigen::MatrixXd &initInnerPts, const Eigen::VectorXd &initT,
      const int CPsNumPerPiece, double &final_cost)
  {
    if (initInnerPts.cols() != (initT.size() - 1))
    {
      ROS_ERROR("initInnerPts.cols() != (initT.size()-1)");
      return false;
    }
    // for (int i = 1; i < initT.size(); ++i)
    //   if (abs(initT(i) - initT(0)) > 1e-10)
    //   {
    //     ROS_ERROR("only support identical time interval now!");
    //     return false;
    //   }

    // Preparision 1: Some mise params
    ros::Time t1, t2;
    int rebound_times = 0;
    multitopology_data_.initial_obstacles_avoided = false;
    constexpr int MAX_REBOUND_TIMES = 20;
    OptFsm state;

    // Preparision 2: Trajectory related params
    t_now_ = ros::Time::now().toSec();
    piece_num_ = initT.size();
    cps_num_prePiece_Long_ = CPsNumPerPiece;
    jerkOpt_.reset(iniState.leftCols(3), finState, piece_num_);
    jerkOpt_.generate(initInnerPts, initT);
    variable_num_ = 3 * (piece_num_ - 1);
    double x_init[variable_num_];
    memcpy(x_init, initInnerPts.data(), initInnerPts.size() * sizeof(x_init[0]));

    // Preparision 3: LBFGS related params
    lbfgs::lbfgs_parameter_t lbfgs_params;
    lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
    lbfgs_params.mem_size = 4;
    lbfgs_params.max_iterations = 50;
    lbfgs_params.min_step = 1e-32;
    // lbfgs_params.abs_curv_cond = 0;
    lbfgs_params.past = 3;
    lbfgs_params.delta = 1.0e-2;
    lbfgs_params.g_epsilon = 0.01;
    do
    {
      /* ---------- prepare ---------- */
      iter_num_ = 0;
      force_stop_type_ = DONT_STOP;

      /* ---------- optimize ---------- */
      t1 = ros::Time::now();
      int result = lbfgs::lbfgs_optimize(
          variable_num_,
          x_init,
          &final_cost,
          PolyTrajOptimizer::ShapeOnlyCostFunctionCallback,
          PolyTrajOptimizer::stepSizeBound,
          PolyTrajOptimizer::earlyExitCallback,
          this,
          &lbfgs_params);

      t2 = ros::Time::now();
      double time_ms = (t2 - t1).toSec() * 1000;

      /* ---------- get result and check collision ---------- */
      if (result == lbfgs::LBFGS_CONVERGENCE ||
          result == lbfgs::LBFGSERR_MAXIMUMITERATION ||
          result == lbfgs::LBFGS_ALREADY_MINIMIZED ||
          result == lbfgs::LBFGS_STOP)
      {
        PRINTF_COND("iter=%d, time(ms)=%f, success\n", iter_num_, time_ms);
        state.act = OptFsm::SUCCESS_RET;
      }
      else if (result == lbfgs::LBFGSERR_CANCELED)
      {
        // flag_force_return = true;
        rebound_times++;
        if (force_stop_type_ == STOP_FOR_REBOUND && rebound_times <= MAX_REBOUND_TIMES)
        {
          PRINTF_COND("iter=%d, time(ms)=%f, rebound\n", iter_num_, time_ms);
          state.act = OptFsm::ROUGH_REBOUND;
        }
        else if (rebound_times > MAX_REBOUND_TIMES)
        {
          PRINTF_COND("iter=%d, time(ms)=%f, rebound to MAX_REBOUND_TIMES\n", iter_num_, time_ms);
          state.act = OptFsm::FAILED_RET;
        }
        else // force_stop_type_ == STOP_FOR_ERROR
        {
          PRINTF_COND("iter=%d, time(ms)=%f, force_stop_type_ == STOP_FOR_ERROR\n", iter_num_, time_ms);
          state.act = OptFsm::FAILED_RET;
        }
      }
      else
      {
        PRINTF_COND("iter=%d, time(ms)=%f, error\n", iter_num_, time_ms);
        ROS_WARN_COND(VERBOSE_OUTPUT, "Solver error. Return = %d, %s. Skip this planning.", result, lbfgs::lbfgs_strerror(result));
        state.act = OptFsm::FAILED_RET;
      }

    } while (state.act != OptFsm::SUCCESS_RET && state.act != OptFsm::FAILED_RET);

    return state.act == OptFsm::SUCCESS_RET;
  }

  /* callbacks by the L-BFGS optimizer */
  double PolyTrajOptimizer::costFunctionCallback(void *func_data, const double *x, double *grad, const int n)
  {
    PolyTrajOptimizer *opt = reinterpret_cast<PolyTrajOptimizer *>(func_data);

    fill(opt->min_ellip_dist2_.begin(), opt->min_ellip_dist2_.end(), std::numeric_limits<double>::max());
    opt->maxv_measure_ = opt->maxa_measure_ = opt->maxj_measure_ = 0.0;

    Eigen::Map<const Eigen::MatrixXd> P(x, 3, opt->piece_num_ - 1);
    // const Eigen::VectorXd t = Eigen::VectorXd::Constant(opt->piece_num_, x[n - 1]); // identical t
    // Eigen::Map<Eigen::MatrixXd> gradP(grad, 3, opt->piece_num_ - 1);
    // Eigen::VectorXd gradt(opt->piece_num_);
    Eigen::Map<const Eigen::VectorXd> t(x + (3 * (opt->piece_num_ - 1)), opt->piece_num_);
    Eigen::Map<Eigen::MatrixXd> gradP(grad, 3, opt->piece_num_ - 1);
    Eigen::Map<Eigen::VectorXd> gradt(grad + (3 * (opt->piece_num_ - 1)), opt->piece_num_);
    Eigen::VectorXd T(opt->piece_num_);

    Eigen::VectorXd gradT(opt->piece_num_);
    double smoo_cost = 0, time_cost = 0;
    Eigen::VectorXd obs_swarm_feas_qvar_plane_unknonmap_crvfit_costs(7);

    opt->VirtualT2RealT(t, T); // Unbounded virtual time to real time

    opt->jerkOpt_.generate(P, T); // Generate trajectory from {P,T}

    opt->initAndGetSmoothnessGradCost2PT(gradT, smoo_cost); // Smoothness cost

    opt->addPVAJGradCost2CT(gradT, obs_swarm_feas_qvar_plane_unknonmap_crvfit_costs, opt->cps_num_prePiece_); // Time int cost

    if (opt->allowRebound())
    {
      opt->roughlyCheckConstraintPoints(); // Trajectory rebound
    }

    opt->jerkOpt_.getGrad2TP(gradT, gradP); // Gradient prepagation

    opt->VirtualTGradCost(T, t, gradT, gradt, time_cost); // Real time back to virtual time
    // grad[n - 1] = gradt.sum();  // identical t
    // if (!opt->cps_.dyn_limit_valid)
    //   gradt.setZero();
    if (opt->cps_.dyn_limit_valid)
      gradP.setZero();

    opt->iter_num_ += 1;
    opt->total_iter_num_ += 1;
    return smoo_cost + obs_swarm_feas_qvar_plane_unknonmap_crvfit_costs.sum() + time_cost;
  }

  /* callbacks by the L-BFGS optimizer */
  double PolyTrajOptimizer::ShapeOnlyCostFunctionCallback(void *func_data, const double *x, double *grad, const int n)
  {
    PolyTrajOptimizer *opt = reinterpret_cast<PolyTrajOptimizer *>(func_data);

    Eigen::Map<const Eigen::MatrixXd> P(x, 3, opt->piece_num_ - 1);
    Eigen::Map<Eigen::MatrixXd> gradP(grad, 3, opt->piece_num_ - 1);
    Eigen::VectorXd T = opt->jerkOpt_.get_T1();

    double smoo_cost = 0;
    Eigen::VectorXd obs_swarm_feas_qvar_plane_unknonmap_costs(6);

    opt->jerkOpt_.generate(P, T); // Generate trajectory from {P,T}

    opt->initAndGetSmoothnessGradCost2P(smoo_cost); // Smoothness cost

    opt->addPGradCost2C(obs_swarm_feas_qvar_plane_unknonmap_costs, T.size(), opt->cps_num_prePiece_Long_); // Time int cost

    if (opt->allowRebound())
    {
      opt->roughlyCheckConstraintPoints(); // Trajectory rebound
    }

    opt->jerkOpt_.getGrad2P(gradP); // Gradient prepagation

    opt->iter_num_ += 1;
    opt->total_iter_num_ += 1;
    return smoo_cost + obs_swarm_feas_qvar_plane_unknonmap_costs.sum();
  }

  int PolyTrajOptimizer::earlyExitCallback(void *func_data, const double *x, const double *g, const double fx, const double xnorm, const double gnorm, const double step, int n, int k, int ls)
  {
    PolyTrajOptimizer *opt = reinterpret_cast<PolyTrajOptimizer *>(func_data);

    ros::Time t_now = ros::Time::now();
    if ((t_now - opt->opt_start_time_).toSec() > 0.05) // 50ms time lim
    {
      ROS_ERROR_COND(VERBOSE_OUTPUT, "Solver exceeded the 50ms time limit. iter=%d/%d, total_t(ms)=%5.3f, terminate\n\033[0m",
                     opt->iter_num_, opt->total_iter_num_, (t_now - opt->opt_start_time_).toSec() * 1000);
      opt->force_stop_type_ = STOP_FOR_ERROR;
    }

    return (opt->force_stop_type_ == STOP_FOR_ERROR || opt->force_stop_type_ == STOP_FOR_REBOUND);
  }

  double PolyTrajOptimizer::stepSizeBound(void *func_data, const double *xp, const double *d, const int n)
  {
    PolyTrajOptimizer *opt = reinterpret_cast<PolyTrajOptimizer *>(func_data);

    Eigen::Map<const Eigen::VectorXd> step_pos(d, 3 * (opt->piece_num_ - 1));
    const double step_lim = opt->map_->cur_->getResolution();
    return step_lim / step_pos.cwiseAbs().maxCoeff();
  }

  bool PolyTrajOptimizer::computePointsToCheck(
      poly_traj::Trajectory &traj,
      int id_cps_end, PtsChk_t &pts_check)
  {
    TicToc t0;

    pts_check.clear();
    pts_check.resize(id_cps_end);
    const double RES_2 = map_->cur_->getResolution() / 2;
    Eigen::VectorXd duras = traj.getDurations();
    double t = 0.0, t_cps_next = duras(0) * (1.0 / cps_num_prePiece_), v = traj.getVel(0.0).norm();
    pts_check[0].emplace_back(std::pair<double, Eigen::Vector3d>(0.0, traj.getPos(0.0)));
    int cps_id = 0;
    while (true) // three time resolutions: check pt dura, cp dura, piece dura
    {
      t += RES_2 / max(v, 0.1);
      if (t >= t_cps_next)
      {
        if (++cps_id < id_cps_end)
        {
          t_cps_next += duras(cps_id / cps_num_prePiece_) * (1.0 / cps_num_prePiece_);
          if (t >= t_cps_next)
            t = t_cps_next;
        }
        else
          break;
      }

      pts_check[cps_id].emplace_back(std::pair<double, Eigen::Vector3d>(t, traj.getPos(t)));
      v = traj.getVel(t).norm();
    }

    return true;
  }

  bool PolyTrajOptimizer::normalityCheck()
  {
    // Shape check
    constexpr double RATIO_LIM = 3;
    double piece_len = 0;
    int K = cps_num_prePiece_;
    for (int cps_id = 1; cps_id < cps_.points.cols(); ++cps_id)
    {
      piece_len += (cps_.points.col(cps_id) - cps_.points.col(cps_id - 1)).norm();
      if (cps_id % K == 0)
      {
        double piece_dist = (cps_.points.col(cps_id) - cps_.points.col(cps_id - K)).norm();
        if (piece_len > RATIO_LIM * piece_dist)
        {
          ROS_ERROR("Abnormal trajectory shape. piece_dist=%f, piece_len=%f", piece_dist, piece_len);
          return false;
        }
        piece_len = 0;
      }
    }

    return true;
  }

  /* check collision and set {p,v} pairs to constrain points */
  PolyTrajOptimizer::CHK_RET PolyTrajOptimizer::finelyCheckAndSetConstraintPoints(
      std::vector<std::pair<int, int>> &segments,
      vector<vector<Eigen::Vector3d>> &a_star_pathes,
      const poly_traj::MinJerkOpt &pt_data,
      const int cps_num_prePiece,
      const bool flag_first_init /*= true*/)
  {
    TicToc t0;

    Eigen::MatrixXd init_points = pt_data.getInitConstraintPoints(cps_num_prePiece);
    poly_traj::Trajectory traj = pt_data.getTraj();
    double reso = map_->cur_->getResolution();

    if (flag_first_init)
    {
      cps_.resize_cp(init_points.cols());
      cps_.points = init_points;
    }

    /*** Segment the initial trajectory according to obstacles ***/
    vector<std::pair<int, int>> segment_ids;
    constexpr int ENOUGH_INTERVAL = 2;
    int in_id = -1, out_id = -1;
    int same_occ_state_times = ENOUGH_INTERVAL + 1;
    bool occ, last_occ = false, ever_occ = false;
    bool flag_got_start = false, flag_got_end = false, flag_got_end_maybe = false;
    int p_end = ConstraintPoints::two_thirds_id(init_points.cols(), touch_goal_ || pp_cpy_.speed_mode == PlanParameters::MODE::FAST); // only check closed 2/3 points.

    PtsChk_t pts_check;
    if (!computePointsToCheck(traj, p_end, pts_check))
    {
      ROS_ERROR("!computePointsToCheck(traj, p_end, pts_check) return!");
      return CHK_RET::ERR;
    }

    if ( t0.toc(false) > 50 )
    {
      ROS_ERROR("A t0.toc(false)=%f", t0.toc(false));
    }

    for (int p_id = 0; p_id < p_end; ++p_id)
    {
      for (size_t j = 0; j < pts_check[p_id].size(); ++j)
      {
        occ = (map_->getOcc(pts_check[p_id][j].second) > 0);
        if (occ)
          ever_occ = true;

        if (occ && !last_occ)
        {
          if (same_occ_state_times > ENOUGH_INTERVAL || p_id == 0)
          {
            if (p_id - 1 >= 0 && map_->getOcc(cps_.points.col(p_id)) > 0 && !(map_->getOcc(cps_.points.col(p_id - 1)) > 0))
              in_id = p_id - 1;
            else
              in_id = p_id;
            flag_got_start = true;
          }
          same_occ_state_times = 0;
          flag_got_end_maybe = false; // terminate in advance
        }
        else if (!occ && last_occ)
        {
          out_id = p_id + 1;
          flag_got_end_maybe = true;
          same_occ_state_times = 0;
        }
        else
        {
          ++same_occ_state_times;
        }

        if (flag_got_end_maybe && (same_occ_state_times > ENOUGH_INTERVAL || (p_id == p_end - 1)))
        {
          flag_got_end_maybe = false;
          flag_got_end = true;
        }

        last_occ = occ;

        if (flag_got_start && flag_got_end)
        {
          flag_got_start = false;
          flag_got_end = false;
          if (in_id < 0 || out_id < 0)
          {
            ROS_ERROR("Should not happen! in_id=%d, out_id=%d", in_id, out_id);
            return CHK_RET::ERR;
          }
          segment_ids.push_back(std::pair<int, int>(in_id, out_id));
        }
      }

      // when checking the last point, it detects collisons but got no closed segment
      // this part must be placed inside the outer for loop
      if (p_id == p_end - 1 && ever_occ && segment_ids.size() == 0)
      {
        int new_p_id = p_end;
        for (; new_p_id < init_points.cols(); ++new_p_id)
        {
          if (map_->getOcc(init_points.col(new_p_id)) <= 0)
          {
            p_end = new_p_id;
            if (!computePointsToCheck(traj, p_end, pts_check))
            {
              ROS_ERROR("!computePointsToCheck(traj, p_end, pts_check), 2. return!");
              return CHK_RET::ERR;
            }
            break;
          }
        }
        if (new_p_id == init_points.cols())
        {
          ROS_ERROR("All targets are in obstacle, return!");
          return CHK_RET::ERR;
        }
      }
    }

    if ( t0.toc(false) > 50 )
    {
      ROS_ERROR("B t0.toc(false)=%f", t0.toc(false));
    }

    /* Collision free and return in advance */
    if (segment_ids.size() == 0)
    {
      return CHK_RET::OBS_FREE;
    }

    /*** a star search ***/
    // vector<vector<Eigen::Vector3d>> a_star_pathes;
    for (size_t seg_id = 0; seg_id < segment_ids.size(); ++seg_id)
    {
      // Search from back to head
      Eigen::Vector3d in(init_points.col(segment_ids[seg_id].second)), out(init_points.col(segment_ids[seg_id].first));
      if (map_->getOcc(in) > 0)
      {
        if (segment_ids[seg_id].second < (int)pts_check.size())
        {
          size_t k = 0;
          for (; k < pts_check[segment_ids[seg_id].second].size(); ++k)
          {
            Eigen::Vector3d in_tmp = pts_check[segment_ids[seg_id].second][k].second;
            if ((in_tmp - in).norm() > 1.0) // debug only
            {
              ROS_ERROR("0x01 in_tmp=%f, %f, %f, in=%f, %f, %f", in_tmp(0), in_tmp(1), in_tmp(2), in(0), in(1), in(2));
            }
            if (!(map_->getOcc(in_tmp) > 0))
            {
              ROS_INFO_COND(VERBOSE_OUTPUT, "(fine) Move A* satrt from in(%f %f %f) to in_tmp(%f %f %f)", in(0), in(1), in(2), in_tmp(0), in_tmp(1), in_tmp(2));
              in = in_tmp;
              break;
            }
          }
          if (k >= pts_check[segment_ids[seg_id].second].size())
            ROS_ERROR_COND(VERBOSE_OUTPUT, "(fine) No A* start is collision-free. in(%f %f %f)", in(0), in(1), in(2));
        }
        else
        {
          ROS_ERROR_COND(VERBOSE_OUTPUT, "(fine) Can't adjust the first point. segment_ids[%d].second=%d, pts_check.size()=%d, getInflateOccupancy=%d", (int)seg_id,
                         segment_ids[seg_id].second, (int)pts_check.size(), map_->getOcc(init_points.col(segment_ids[seg_id].second)));
        }
      }
      if (map_->getOcc(out) > 0)
      {
        if (segment_ids[seg_id].first > 0)
        {
          int k = (int)pts_check[segment_ids[seg_id].first - 1].size() - 1;
          for (; k >= 0; --k)
          {
            Eigen::Vector3d out_tmp = pts_check[segment_ids[seg_id].first - 1][k].second;
            if (!(map_->getOcc(out_tmp) > 0))
            {
              ROS_INFO_COND(VERBOSE_OUTPUT, "(fine) Move A* end from out(%f %f %f) to out_tmp(%f %f %f)", out(0), out(1), out(2), out_tmp(0), out_tmp(1), out_tmp(2));
              out = out_tmp;
              break;
            }
          }
          if (k < 0)
            ROS_ERROR_COND(VERBOSE_OUTPUT, "(fine) No A* end is collision-free. out(%f %f %f)", out(0), out(1), out(2));
        }
        else
        {
          ROS_ERROR_COND(VERBOSE_OUTPUT, "(fine) Can't adjust the first point[%.3f, %.3f, %.3f]. segment_ids[%d].first=%d, getOcc=%d",
                         out(0), out(1), out(2), (int)seg_id, segment_ids[seg_id].first, map_->getOcc(init_points.col(segment_ids[seg_id].first)));
        }
      }
      dyn_a_star::ASTAR_RET ret = a_star_->AstarSearch(reso, in, out);
      if (ret == dyn_a_star::ASTAR_RET::SUCCESS)
      {
        a_star_pathes.push_back(a_star_->getPath());
      }
      else if (ret == dyn_a_star::ASTAR_RET::NO_PATH && seg_id + 1 < segment_ids.size()) // connect the next segment
      {
        segment_ids[seg_id].second = segment_ids[seg_id + 1].second;
        segment_ids.erase(segment_ids.begin() + seg_id + 1);
        --seg_id;
      }
      else if (ret == dyn_a_star::ASTAR_RET::TIME_LIM)
      {
        // No need more trials
        return CHK_RET::TIME_LIM;
      }
      else
      {
        ROS_WARN_COND(VERBOSE_OUTPUT, "A-star error, force return! in=%f, %f, %f, out=%f, %f, %f", in(0), in(1), in(2), out(0), out(1), out(2));
        return CHK_RET::ERR;
      }
    }

    if ( t0.toc(false) > 50 )
    {
      ROS_ERROR("C t0.toc(false)=%f", t0.toc(false));
    }

    /*** calculate bounds ***/
    int id_low_bound, id_up_bound;
    vector<std::pair<int, int>> bounds(segment_ids.size());
    for (size_t seg_id = 0; seg_id < segment_ids.size(); seg_id++)
    {

      if (seg_id == 0) // first segment
      {
        id_low_bound = 1;
        if (segment_ids.size() > 1)
        {
          id_up_bound = (int)(((segment_ids[0].second + segment_ids[1].first) - 1.0f) / 2); // id_up_bound : -1.0f fix()
        }
        else
        {
          id_up_bound = init_points.cols() - 2;
        }
      }
      else if (seg_id == segment_ids.size() - 1) // last segment, seg_id != 0 here
      {
        id_low_bound = (int)(((segment_ids[seg_id].first + segment_ids[seg_id - 1].second) + 1.0f) / 2); // id_low_bound : +1.0f ceil()
        id_up_bound = init_points.cols() - 2;
      }
      else
      {
        id_low_bound = (int)(((segment_ids[seg_id].first + segment_ids[seg_id - 1].second) + 1.0f) / 2); // id_low_bound : +1.0f ceil()
        id_up_bound = (int)(((segment_ids[seg_id].second + segment_ids[seg_id + 1].first) - 1.0f) / 2);  // id_up_bound : -1.0f fix()
      }

      bounds[seg_id] = std::pair<int, int>(id_low_bound, id_up_bound);
    }

    /*** Adjust segment length ***/
    vector<std::pair<int, int>> adjusted_segment_ids(segment_ids.size());
    constexpr double MINIMUM_PERCENT = 0.0; // Each segment is guaranteed to have sufficient points to generate sufficient force
    int minimum_points = round(init_points.cols() * MINIMUM_PERCENT), num_points;
    for (size_t seg_id = 0; seg_id < segment_ids.size(); seg_id++)
    {
      /*** Adjust segment length ***/
      num_points = segment_ids[seg_id].second - segment_ids[seg_id].first + 1;
      if (num_points < minimum_points)
      {
        double add_points_each_side = (int)(((minimum_points - num_points) + 1.0f) / 2);

        adjusted_segment_ids[seg_id].first = segment_ids[seg_id].first - add_points_each_side >= bounds[seg_id].first
                                                 ? segment_ids[seg_id].first - add_points_each_side
                                                 : bounds[seg_id].first;

        adjusted_segment_ids[seg_id].second = segment_ids[seg_id].second + add_points_each_side <= bounds[seg_id].second
                                                  ? segment_ids[seg_id].second + add_points_each_side
                                                  : bounds[seg_id].second;
      }
      else
      {
        adjusted_segment_ids[seg_id].first = segment_ids[seg_id].first;
        adjusted_segment_ids[seg_id].second = segment_ids[seg_id].second;
      }
    }

    for (size_t seg_id = 1; seg_id < adjusted_segment_ids.size(); seg_id++) // Avoid overlap
    {
      if (adjusted_segment_ids[seg_id - 1].second >= adjusted_segment_ids[seg_id].first)
      {
        double middle = (double)(adjusted_segment_ids[seg_id - 1].second + adjusted_segment_ids[seg_id].first) / 2.0;
        adjusted_segment_ids[seg_id - 1].second = static_cast<int>(middle - 0.1);
        adjusted_segment_ids[seg_id].first = static_cast<int>(middle + 1.1);
      }
    }

    if ( t0.toc(false) > 50 )
    {
      ROS_ERROR("D t0.toc(false)=%f", t0.toc(false));
    }

    // Used for return
    vector<std::pair<int, int>> final_segment_ids;

    /*** Assign data to each segment ***/
    for (size_t seg_id = 0; seg_id < segment_ids.size(); seg_id++)
    {
      // step 1
      for (int p_id = adjusted_segment_ids[seg_id].first; p_id <= adjusted_segment_ids[seg_id].second; ++p_id)
        cps_.flag_got_pvpair[p_id] = false;

      // step 2
      vector<int> got_intersection_id;
      if (segment_ids[seg_id].second - segment_ids[seg_id].first >= 2)
      {
        for (int p_id = segment_ids[seg_id].first + 1; p_id < segment_ids[seg_id].second; ++p_id)
        {
          got_intersection_id.push_back(-1);
          Eigen::Vector3d ctrl_pts_law(init_points.col(p_id + 1) - init_points.col(p_id - 1)), intersection_point;
          int Astar_id = a_star_pathes[seg_id].size() / 2, last_Astar_id; // Let "Astar_id = id_of_the_most_far_away_Astar_point" will be better, but it needs more computation
          double val = (a_star_pathes[seg_id][Astar_id] - init_points.col(p_id)).dot(ctrl_pts_law), init_val = val;
          while (true)
          {

            last_Astar_id = Astar_id;

            if (val >= 0)
            {
              ++Astar_id; // Previous Astar search from back to head
              if (Astar_id >= (int)a_star_pathes[seg_id].size())
              {
                break;
              }
            }
            else
            {
              --Astar_id;
              if (Astar_id < 0)
              {
                break;
              }
            }

            val = (a_star_pathes[seg_id][Astar_id] - init_points.col(p_id)).dot(ctrl_pts_law);

            if (val * init_val <= 0 && (abs(val) > 0 || abs(init_val) > 0)) // val = init_val = 0.0 is not allowed
            {
              intersection_point =
                  a_star_pathes[seg_id][Astar_id] +
                  ((a_star_pathes[seg_id][Astar_id] - a_star_pathes[seg_id][last_Astar_id]) *
                   (ctrl_pts_law.dot(init_points.col(p_id) - a_star_pathes[seg_id][Astar_id]) / ctrl_pts_law.dot(a_star_pathes[seg_id][Astar_id] - a_star_pathes[seg_id][last_Astar_id])) // = t
                  );

              // cout << "seg_id=" << seg_id << " Astar_id=" << Astar_id << " last_Astar_id=" << last_Astar_id << " p_id=" << p_id << endl;
              // cout << "a_star_pathes[seg_id][Astar_id]=" << a_star_pathes[seg_id][Astar_id].transpose() << " a_star_pathes[seg_id][last_Astar_id]=" << a_star_pathes[seg_id][last_Astar_id].transpose() << endl;
              // cout << "ctrl_pts_law=" << ctrl_pts_law.transpose() << " init_points.col(p_id)=" << init_points.col(p_id).transpose() << endl;

              got_intersection_id.back() = p_id;
              break;
            }
          }

          if (got_intersection_id.back() >= 0)
          {
            double length = (intersection_point - init_points.col(p_id)).norm();
            if (length > reso * 0.5) // 0.5*resolution at least
            {
              cps_.flag_got_pvpair[p_id] = true;
              for (double a = length; a >= 0.0; a -= reso)
              {
                bool occ = (map_->getOcc((a / length) * intersection_point + (1 - a / length) * init_points.col(p_id)) > 0);

                if (occ || a < reso)
                {
                  if (occ)
                    a += reso;
                  Eigen::Vector3d p = (a / length) * intersection_point + (1 - a / length) * init_points.col(p_id);
                  Eigen::Vector3d v = (intersection_point - init_points.col(p_id)).normalized();
                  p = tryExtendAndChkP(p, v, cps_.points.col(p_id));
                  cps_.base_point[p_id].push_back(p);
                  cps_.direction[p_id].push_back(v);

                  if ((p - cps_.points.col(p_id)).norm() > 3.0)
                  {
                    ROS_ERROR("here A");
                    cout << "p=" << p.transpose() << " cps_.points.col(p_id)=" << cps_.points.col(p_id).transpose() << endl;
                    cout << "intersection_point=" << intersection_point.transpose() << " init_points.col(p_id)=" << init_points.col(p_id).transpose() << " length=" << length << endl;
                  }
                  break;
                }
              }
            }
            else
            {
              // ROS_INFO_COND(VERBOSE_OUTPUT, "(finely) intersection_point (%f %f %f) is too close to cps_.points.col(%d)[%f %f %f]", 
              //               intersection_point(0), intersection_point(1), intersection_point(2), p_id,
              //               cps_.points.col(p_id)(0), cps_.points.col(p_id)(1), cps_.points.col(p_id)(2));
              got_intersection_id.back() = -1;
            }
          }

          if (got_intersection_id.back() < 0)
          {
            bool found = true;
            Eigen::Vector3d occ_pt = cps_.points.col(p_id);
            if (!(map_->getOcc(occ_pt) > 0))
            {
              for (int expand = 1;; expand++)
              {
                if (expand - 1 >= 0 && expand - 1 < (int)pts_check[p_id].size())
                {
                  Eigen::Vector3d fwd_chk_pt = pts_check[p_id][expand - 1].second;
                  if (map_->getOcc(fwd_chk_pt) > 0)
                  {
                    occ_pt = fwd_chk_pt;
                    found = true;
                    break;
                  }
                }
                if (expand <= (int)pts_check[p_id - 1].size() && expand >= 1)
                {
                  Eigen::Vector3d bwd_chk_pt = pts_check[p_id - 1][pts_check[p_id - 1].size() - expand].second;
                  if (map_->getOcc(bwd_chk_pt) > 0)
                  {
                    occ_pt = bwd_chk_pt;
                    found = true;
                    break;
                  }
                }
                if (expand - 1 >= (int)pts_check[p_id].size() && (int)(pts_check[p_id - 1].size()) - expand < 0)
                {
                  found = false;
                  break;
                }
              }
              if (!found)
              {
                ROS_ERROR("No collided traj around cps_.points.col(%d)", p_id);
              }
              else
              {
                ROS_WARN_COND(VERBOSE_OUTPUT, "Found pt(%f %f %f) around p_id=%d", occ_pt(0), occ_pt(1), occ_pt(2), p_id);
              }
            }

            if (found) // only in-obstacle points are considered
            {
              Eigen::Vector3d occ_pt = init_points.col(p_id);
              Eigen::Vector3d sum = Eigen::Vector3d::Zero();
              const int WIDTH = 1;
              for (int x = -WIDTH; x <= WIDTH; x++)
                for (int y = -WIDTH; y <= WIDTH; y++)
                  for (int z = -WIDTH; z <= WIDTH; z++)
                  {
                    int occ = (map_->getOcc(occ_pt + Eigen::Vector3d(reso * x, reso * y, reso * z)) > 0);
                    if (occ)
                      sum -= Eigen::Vector3d(x, y, z);
                    else
                      sum += Eigen::Vector3d(x, y, z);
                  }
              if (sum.squaredNorm() > 1e-5)
              {
                Eigen::Vector3d avg = sum / 25.0 * reso;
                cps_.flag_got_pvpair[p_id] = true;
                Eigen::Vector3d p = occ_pt + avg;
                Eigen::Vector3d v = (avg).normalized();
                p = tryExtendAndChkP(p, v, cps_.points.col(p_id));
                cps_.base_point[p_id].push_back(p);
                cps_.direction[p_id].push_back(v);
                got_intersection_id.back() = p_id;
                ROS_WARN_COND(VERBOSE_OUTPUT, "Resolved. F3. p_id=%d", p_id);

                if ((p - cps_.points.col(p_id)).norm() > 3.0)
                {
                  ROS_ERROR("here B");
                  cout << "p=" << p.transpose() << " cps_.points.col(p_id)=" << cps_.points.col(p_id).transpose() << endl;
                }
              }
              else
              {
                ROS_ERROR_COND(VERBOSE_OUTPUT, "(fine) No freespace or A* around. Really tough, abort! occ_pt=%f %f %f", occ_pt(0), occ_pt(1), occ_pt(2));
              }
            }
            else
              ROS_ERROR("(fine) map_->getOcc(%f %f %f) = false, p_id=%d", occ_pt(0), occ_pt(1), occ_pt(2), p_id);
          }
        }
      }
      else /* The segment length is too short. Here the control points may outside the A* path, leading to opposite gradient direction. So I have to take special care of it */
      {
        got_intersection_id.push_back(-1);
        Eigen::Vector3d ctrl_pts_law(init_points.col(segment_ids[seg_id].second) - init_points.col(segment_ids[seg_id].first)), intersection_point;
        Eigen::Vector3d middle_point = (init_points.col(segment_ids[seg_id].second) + init_points.col(segment_ids[seg_id].first)) / 2;
        if (!(map_->getOcc(middle_point) > 0))
        {
          double expend = reso / 2;
          double expend_end = (init_points.col(segment_ids[seg_id].second) - init_points.col(segment_ids[seg_id].first)).norm() / 2;
          for (; expend < expend_end; expend += reso / 2)
          {
            Eigen::Vector3d test_p = middle_point + expend * ctrl_pts_law.normalized();
            if (map_->getOcc(test_p) > 0)
            {
              middle_point = test_p;
              break;
            }
            test_p = middle_point - expend * ctrl_pts_law.normalized();
            if (map_->getOcc(test_p) > 0)
            {
              middle_point = test_p;
              break;
            }
          }
          if (expend >= expend_end)
          {
            ROS_ERROR("Can't find any in-collision point.");
            cout << "init_points.col(segment_ids[seg_id].second)=" << init_points.col(segment_ids[seg_id].second).transpose()
                 << " init_points.col(segment_ids[seg_id].first)=" << init_points.col(segment_ids[seg_id].first).transpose() << endl;
          }
        }
        int Astar_id = a_star_pathes[seg_id].size() / 2, last_Astar_id; // Let "Astar_id = id_of_the_most_far_away_Astar_point" will be better, but it needs more computation
        double val = (a_star_pathes[seg_id][Astar_id] - middle_point).dot(ctrl_pts_law), init_val = val;
        while (true)
        {

          last_Astar_id = Astar_id;

          if (val >= 0)
          {
            ++Astar_id; // Previous Astar search from back to head
            if (Astar_id >= (int)a_star_pathes[seg_id].size())
            {
              break;
            }
          }
          else
          {
            --Astar_id;
            if (Astar_id < 0)
            {
              break;
            }
          }

          val = (a_star_pathes[seg_id][Astar_id] - middle_point).dot(ctrl_pts_law);

          if (val * init_val <= 0 && (abs(val) > 0 || abs(init_val) > 0)) // val = init_val = 0.0 is not allowed
          {
            intersection_point =
                a_star_pathes[seg_id][Astar_id] +
                ((a_star_pathes[seg_id][Astar_id] - a_star_pathes[seg_id][last_Astar_id]) *
                 (ctrl_pts_law.dot(middle_point - a_star_pathes[seg_id][Astar_id]) / ctrl_pts_law.dot(a_star_pathes[seg_id][Astar_id] - a_star_pathes[seg_id][last_Astar_id])) // = t
                );

            if ((intersection_point - middle_point).norm() > 0.5 * reso) // 10 cm.
            {
              cps_.flag_got_pvpair[segment_ids[seg_id].first] = true;
              // Eigen::Vector3d p = init_points.col(segment_ids[seg_id].first);
              Eigen::Vector3d p = intersection_point;
              Eigen::Vector3d v = (intersection_point - middle_point).normalized();
              p = tryExtendAndChkP(p, v, cps_.points.col(segment_ids[seg_id].first));
              cps_.base_point[segment_ids[seg_id].first].push_back(p);
              cps_.direction[segment_ids[seg_id].first].push_back(v);

              got_intersection_id.back() = segment_ids[seg_id].first;

              if ((p - cps_.points.col(segment_ids[seg_id].first)).norm() > 3.0)
              {
                ROS_ERROR("here C");
                cout << "p=" << p.transpose() << " cps_.points.col(p_id)=" << cps_.points.col(segment_ids[seg_id].first).transpose() << endl;
              }
            }
            break;
          }
        }

        if (got_intersection_id.back() < 0) // really tough, final trial!
        {
          Eigen::Vector3d occ_pt = middle_point;
          int p_id = segment_ids[seg_id].first;
          for (size_t k = 0; k < pts_check[p_id].size(); ++k)
          {
            if (map_->getOcc(pts_check[p_id][k].second) > 0)
            {
              occ_pt = pts_check[p_id][k].second;
              break;
            }
          }

          Eigen::Vector3d sum = Eigen::Vector3d::Zero();
          const int WIDTH = 1;
          for (int x = -WIDTH; x <= WIDTH; x++)
            for (int y = -WIDTH; y <= WIDTH; y++)
              for (int z = -WIDTH; z <= WIDTH; z++)
              {
                int occ = (map_->getOcc(occ_pt + Eigen::Vector3d(reso * x, reso * y, reso * z)) > 0);
                if (occ)
                  sum -= Eigen::Vector3d(x, y, z);
                else
                  sum += Eigen::Vector3d(x, y, z);
              }
          if (sum.squaredNorm() > 1e-5)
          {
            Eigen::Vector3d avg = sum / 25.0 * reso;
            cps_.flag_got_pvpair[segment_ids[seg_id].first] = true;
            Eigen::Vector3d p = occ_pt + avg;
            Eigen::Vector3d v = (avg).normalized();
            p = tryExtendAndChkP(p, v, cps_.points.col(segment_ids[seg_id].first));
            cps_.base_point[segment_ids[seg_id].first].push_back(p);
            cps_.direction[segment_ids[seg_id].first].push_back(v);
            got_intersection_id.back() = segment_ids[seg_id].first;
            ROS_WARN_COND(VERBOSE_OUTPUT, "Resolved. F2, got_intersection_id.back()=%d", got_intersection_id.back());

            if ((p - cps_.points.col(p_id)).norm() > 3.0)
            {
              ROS_ERROR("here D");
              cout << "p=" << p.transpose() << " cps_.points.col(p_id)=" << cps_.points.col(p_id).transpose() << endl;
            }
          }
          else
          {
            ROS_ERROR_COND(VERBOSE_OUTPUT, "(fine2) No freespace or A* around. Really tough, abort! occ_pt=%f %f %f", occ_pt(0), occ_pt(1), occ_pt(2));
          }
        }
      }

      // step 3
      int start_id = -1;
      for (auto id : got_intersection_id)
        start_id = max(start_id, id);
      if (start_id >= 0)
      {
        // for (int p_id = -1; p_id <= 1; p_id += 2)
        // {
        //   int near_id = got_intersection_id + p_id;
        //   if (!cps_.flag_got_pvpair[near_id] && !map_->getOcc(cps_.points.col(near_id)))
        //   {
        //     cps_.base_point[near_id].push_back(cps_.base_point[got_intersection_id].back());
        //     cps_.direction[near_id].push_back(cps_.direction[got_intersection_id].back());
        //     cps_.flag_got_pvpair[near_id] = true;
        //   }
        // }

        for (int p_id = start_id + 1; p_id <= segment_ids[seg_id].second; ++p_id)
          if (!cps_.flag_got_pvpair[p_id])
          {
            cps_.base_point[p_id].push_back(cps_.base_point[p_id - 1].back());
            cps_.direction[p_id].push_back(cps_.direction[p_id - 1].back());
          }

        for (int p_id = start_id - 1; p_id >= segment_ids[seg_id].first; --p_id)
          if (!cps_.flag_got_pvpair[p_id])
          {
            cps_.base_point[p_id].push_back(cps_.base_point[p_id + 1].back());
            cps_.direction[p_id].push_back(cps_.direction[p_id + 1].back());
          }

        final_segment_ids.push_back(adjusted_segment_ids[seg_id]);
      }
      else
      {
        // Just ignore, it does not matter ^_^.
        ROS_ERROR("(fine)Failed to generate direction! segment_id=%d [%d -> %d]", (int)seg_id, segment_ids[seg_id].first, segment_ids[seg_id].second);
      }
    }

    if ( t0.toc(false) > 50 )
    {
      ROS_ERROR("E t0.toc(false)=%f", t0.toc(false));
    }

    segments = final_segment_ids;
    return CHK_RET::FINISH;
  }

  bool PolyTrajOptimizer::roughlyCheckConstraintPoints(void)
  {

    /*** Check and segment the initial trajectory according to obstacles ***/
    int in_id, out_id;
    vector<std::pair<int, int>> segment_ids;
    bool flag_new_obs_valid = false;
    int p_end = ConstraintPoints::two_thirds_id(cps_.points.cols(), touch_goal_); // only check closed 2/3 points.
    for (int p_id = 1; p_id <= p_end; ++p_id)
    {

      bool occ = (map_->getOcc(cps_.points.col(p_id)) > 0);
      // ROS_INFO("(debug) it=%d, p=%f %f %f, occ=%d", dbgopt_.back().iter_num, cps_.points.col(p_id)(0), cps_.points.col(p_id)(1), cps_.points.col(p_id)(2), occ);

      /*** check if the new collision will be valid ***/
      if (occ)
      {
        for (size_t k = 0; k < cps_.direction[p_id].size(); ++k)
        {
          if ((cps_.points.col(p_id) - cps_.base_point[p_id][k]).dot(cps_.direction[p_id][k]) < 1 * map_->cur_->getResolution()) // current point is outside all the collision_points.
          {
            occ = false;
            break;
          }
        }
      }

      if (occ)
      {
        flag_new_obs_valid = true;

        int bwd_p_id;
        for (bwd_p_id = p_id - 1; bwd_p_id >= 0; --bwd_p_id)
        {
          occ = (map_->getOcc(cps_.points.col(bwd_p_id)) > 0);
          if (!occ)
          {
            in_id = bwd_p_id;
            break;
          }
        }
        if (bwd_p_id < 0) // fail to get the obs free point
        {
          ROS_ERROR("The drone is in obstacle. It means a crash in real-world.");
          in_id = 0;
        }

        int fwd_p_id;
        for (fwd_p_id = p_id + 1; fwd_p_id < cps_.cp_size; ++fwd_p_id)
        {
          occ = (map_->getOcc(cps_.points.col(fwd_p_id)) > 0);

          if (!occ)
          {
            out_id = fwd_p_id;
            break;
          }
        }
        if (fwd_p_id >= cps_.cp_size) // fail to get the obs free point
        {
          ROS_WARN("Local target in collision, skip this planning.");

          force_stop_type_ = STOP_FOR_ERROR;
          return false;
        }

        p_id = fwd_p_id + 1;

        segment_ids.push_back(std::pair<int, int>(in_id, out_id));
        // ROS_INFO("(debug) it=%d, in_id=%d, out_id=%d", dbgopt_.back().iter_num, in_id, out_id);
      }
    }

    if (flag_new_obs_valid)
    {
      vector<vector<Eigen::Vector3d>> a_star_pathes;
      for (size_t seg_id = 0; seg_id < segment_ids.size(); ++seg_id)
      {
        /*** a star search ***/
        Eigen::Vector3d in(cps_.points.col(segment_ids[seg_id].second)), out(cps_.points.col(segment_ids[seg_id].first));
        if (map_->getOcc(in) > 0)
        {
          if (segment_ids[seg_id].second + 1 < cps_.points.cols() - 1)
          {
            double k = 1.0;
            for (; k > 0.0; k -= 0.2) // five trials will be enough
            {
              Eigen::Vector3d in_tmp = k * cps_.points.col(segment_ids[seg_id].second) + (1.0 - k) * cps_.points.col(segment_ids[seg_id].second + 1);
              if ((in_tmp - in).norm() > 1.0) // debug only
              {
                ROS_ERROR("0x02 in_tmp=%f, %f, %f, in=%f, %f, %f", in_tmp(0), in_tmp(1), in_tmp(2), in(0), in(1), in(2));
              }
              if (!(map_->getOcc(in_tmp) > 0))
              {
                ROS_INFO_COND(VERBOSE_OUTPUT, "(rough) Move A* satrt from in(%f %f %f) to in_tmp(%f %f %f)", in(0), in(1), in(2), in_tmp(0), in_tmp(1), in_tmp(2));
                in = in_tmp;
                break;
              }
            }
            if (k <= 0.0)
              ROS_ERROR_COND(VERBOSE_OUTPUT, "(rough) No A* start is collision-free. in(%f %f %f)", in(0), in(1), in(2));
          }
          else
          {
            ROS_ERROR_COND(VERBOSE_OUTPUT, "(fine) Can't adjust the second point. segment_ids[%d].second=%d, getInflateOccupancy=%d", (int)seg_id,
                           segment_ids[seg_id].second, map_->getOcc(cps_.points.col(segment_ids[seg_id].second)));
          }
        }
        if (map_->getOcc(out) > 0)
        {
          if (segment_ids[seg_id].first > 0)
          {
            double k = 1.0;
            for (; k > 0.0; k -= 0.2)
            {
              Eigen::Vector3d out_tmp = k * cps_.points.col(segment_ids[seg_id].first) + (1.0 - k) * cps_.points.col(segment_ids[seg_id].first - 1);
              if ((out_tmp - out).norm() > 1.0)
              {
                ROS_ERROR_COND(VERBOSE_OUTPUT, "out_tmp=%f, %f, %f, out=%f, %f, %f", out_tmp(0), out_tmp(1), out_tmp(2), out(0), out(1), out(2));
              }
              if (!(map_->getOcc(out_tmp) > 0))
              {
                ROS_INFO_COND(VERBOSE_OUTPUT, "(rough) Move A* end from out(%f %f %f) to out_tmp(%f %f %f)", out(0), out(1), out(2), out_tmp(0), out_tmp(1), out_tmp(2));
                out = out_tmp;
                break;
              }
            }
            if (k <= 0.0)
              ROS_ERROR_COND(VERBOSE_OUTPUT, "(rough) No A* end is collision-free. out(%f %f %f)", out(0), out(1), out(2));
          }
          else
          {
            ROS_ERROR_COND(VERBOSE_OUTPUT, "(fine) Can't adjust the first point. segment_ids[%d].second=%d, getInflateOccupancy=%d", (int)seg_id,
                           segment_ids[seg_id].second, map_->getOcc(cps_.points.col(segment_ids[seg_id].first)));
          }
        }
        dyn_a_star::ASTAR_RET ret = a_star_->AstarSearch(/*(in-out).norm()/10+0.05*/ map_->cur_->getResolution(), in, out);
        if (ret == dyn_a_star::ASTAR_RET::SUCCESS)
        {
          a_star_pathes.push_back(a_star_->getPath());
        }
        else if (ret == dyn_a_star::ASTAR_RET::NO_PATH && seg_id + 1 < segment_ids.size()) // connect the next segment
        {
          ROS_WARN("(rough) Concat seg for A*NO_PATH. seg id: bwd[%d, %d], fwd[%d, %d]", 
                   segment_ids[seg_id].first, segment_ids[seg_id].second, segment_ids[seg_id + 1].first, segment_ids[seg_id + 1].second);
          segment_ids[seg_id].second = segment_ids[seg_id + 1].second;
          segment_ids.erase(segment_ids.begin() + seg_id + 1);
          --seg_id;
          // dbgopt_.back().trigger_type = 11; // debug
        }
        else
        {
          // dbgopt_.back().trigger_type = 10; // debug
          ROS_ERROR_COND(VERBOSE_OUTPUT, "A-star error");
          segment_ids.erase(segment_ids.begin() + seg_id);
          --seg_id;
        }
      }

      // /************************debug*****************/
      // while (dbgopt_.back().iter_num > (int)dbgopt_.back().Astar.size())
      // {
      //   vector<vector<Eigen::Vector3d>> blank;
      //   dbgopt_.back().Astar.push_back(blank);
      // }
      // dbgopt_.back().Astar[dbgopt_.back().iter_num - 1] = a_star_pathes;
      // /************************debug*****************/

      for (size_t seg_id = 1; seg_id < segment_ids.size(); seg_id++) // Avoid overlap
      {
        if (segment_ids[seg_id - 1].second >= segment_ids[seg_id].first)
        {
          double middle = (double)(segment_ids[seg_id - 1].second + segment_ids[seg_id].first) / 2.0;
          segment_ids[seg_id - 1].second = static_cast<int>(middle - 0.1);
          segment_ids[seg_id].first = static_cast<int>(middle + 1.1);
        }
      }

      /*** Assign parameters to each segment ***/
      for (size_t seg_id = 0; seg_id < segment_ids.size(); ++seg_id)
      {
        // step 1
        for (int p_id = segment_ids[seg_id].first; p_id <= segment_ids[seg_id].second; ++p_id)
          cps_.flag_got_pvpair[p_id] = false;

        // step 2
        vector<int> got_intersection_id;
        for (int p_id = segment_ids[seg_id].first + 1; p_id < segment_ids[seg_id].second; ++p_id)
        {
          got_intersection_id.push_back(-1);
          Eigen::Vector3d ctrl_pts_law(cps_.points.col(p_id + 1) - cps_.points.col(p_id - 1)), intersection_point;
          int Astar_id = a_star_pathes[seg_id].size() / 2, last_Astar_id; // Let "Astar_id = id_of_the_most_far_away_Astar_point" will be better, but it needs more computation
          double val = (a_star_pathes[seg_id][Astar_id] - cps_.points.col(p_id)).dot(ctrl_pts_law), init_val = val;
          while (true)
          {

            last_Astar_id = Astar_id;

            if (val >= 0)
            {
              ++Astar_id; // Previous Astar search from back to head
              if (Astar_id >= (int)a_star_pathes[seg_id].size())
              {
                break;
              }
            }
            else
            {
              --Astar_id;
              if (Astar_id < 0)
              {
                break;
              }
            }

            val = (a_star_pathes[seg_id][Astar_id] - cps_.points.col(p_id)).dot(ctrl_pts_law);

            if (val * init_val <= 0 && (abs(val) > 0 || abs(init_val) > 0)) // val = init_val = 0.0 is not allowed
            {
              intersection_point =
                  a_star_pathes[seg_id][Astar_id] +
                  ((a_star_pathes[seg_id][Astar_id] - a_star_pathes[seg_id][last_Astar_id]) *
                   (ctrl_pts_law.dot(cps_.points.col(p_id) - a_star_pathes[seg_id][Astar_id]) / ctrl_pts_law.dot(a_star_pathes[seg_id][Astar_id] - a_star_pathes[seg_id][last_Astar_id])) // = t
                  );

              got_intersection_id.back() = p_id;
              break;
            }
          }

          if (got_intersection_id.back() >= 0)
          {
            double length = (intersection_point - cps_.points.col(p_id)).norm();
            const double reso = map_->cur_->getResolution();
            if (length > reso * 0.5) // 0.5*resolution at least
            {
              cps_.flag_got_pvpair[p_id] = true;
              for (double a = length; a >= 0.0; a -= reso)
              {
                bool occ = (map_->getOcc((a / length) * intersection_point + (1 - a / length) * cps_.points.col(p_id)) > 0);

                if (occ || a < reso)
                {
                  if (occ)
                    a += reso;
                  Eigen::Vector3d p = (a / length) * intersection_point + (1 - a / length) * cps_.points.col(p_id);
                  Eigen::Vector3d v = (intersection_point - cps_.points.col(p_id)).normalized();
                  p = tryExtendAndChkP(p, v, cps_.points.col(p_id));
                  cps_.base_point[p_id].push_back(p);
                  cps_.direction[p_id].push_back(v);

                  if ((p - cps_.points.col(p_id)).norm() > 3.0)
                  {
                    ROS_ERROR("here E");
                    cout << "p=" << p.transpose() << " cps_.points.col(p_id)=" << cps_.points.col(p_id).transpose() << endl;
                  }
                  break;
                }
              }
            }
            else
            {
              // ROS_INFO_COND(VERBOSE_OUTPUT, "(roughly) intersection_point (%f %f %f) is too close to cps_.points.col(%d)[%f %f %f]", 
              //               intersection_point(0), intersection_point(1), intersection_point(2), p_id,
              //               cps_.points.col(p_id)(0), cps_.points.col(p_id)(1), cps_.points.col(p_id)(2));
              got_intersection_id.back() = -1;
            }
          }

          if (got_intersection_id.back() < 0) // really tough, final trial!
          {
            Eigen::Vector3d occ_pt = cps_.points.col(p_id);
            if (map_->getOcc(occ_pt) > 0)
            {
              Eigen::Vector3d sum = Eigen::Vector3d::Zero();
              const double res = map_->cur_->getResolution();
              const int WIDTH = 1;
              for (int x = -WIDTH; x <= WIDTH; x++)
                for (int y = -WIDTH; y <= WIDTH; y++)
                  for (int z = -WIDTH; z <= WIDTH; z++)
                  {
                    int occ = (map_->getOcc(occ_pt + Eigen::Vector3d(res * x, res * y, res * z)) > 0);
                    if (occ)
                      sum -= Eigen::Vector3d(x, y, z);
                    else
                      sum += Eigen::Vector3d(x, y, z);
                  }
              if (sum.squaredNorm() > 1e-5)
              {
                Eigen::Vector3d avg = sum / 25.0 * res;
                cps_.flag_got_pvpair[p_id] = true;
                Eigen::Vector3d p = occ_pt + avg;
                Eigen::Vector3d v = (avg).normalized();
                p = tryExtendAndChkP(p, v, cps_.points.col(p_id));
                cps_.base_point[p_id].push_back(p);
                cps_.direction[p_id].push_back(v);
                got_intersection_id.back() = p_id;

                if ((p - cps_.points.col(p_id)).norm() > 3.0)
                {
                  ROS_ERROR("here F");
                  cout << "p=" << p.transpose() << " cps_.points.col(p_id)=" << cps_.points.col(p_id).transpose() << endl;
                }

                // ROS_WARN_COND(VERBOSE_OUTPUT, "Resolved. R. p_id=%d", p_id);
              }
              else
              {
                ROS_ERROR_COND(VERBOSE_OUTPUT, "(roughly) No freespace or A* around. Really tough, abort! occ_pt=%f %f %f", 
                               occ_pt(0), occ_pt(1), occ_pt(2));
                // dbgopt_.back().trigger_type = 1; // debug
              }
            }
            else{
              // ROS_ERROR("(roughly) map_->getOcc(%f %f %f) = false, p_id=%d", 
              //           occ_pt(0), occ_pt(1), occ_pt(2), p_id);
            }
          }
        }

        // step 3
        int start_id = -1;
        for (auto id : got_intersection_id)
          start_id = max(start_id, id);
        if (start_id >= 0)
        {
          for (int p_id = start_id + 1; p_id <= segment_ids[seg_id].second; ++p_id)
            if (!cps_.flag_got_pvpair[p_id])
            {
              cps_.base_point[p_id].push_back(cps_.base_point[p_id - 1].back());
              cps_.direction[p_id].push_back(cps_.direction[p_id - 1].back());
            }

          for (int p_id = start_id - 1; p_id >= segment_ids[seg_id].first; --p_id)
            if (!cps_.flag_got_pvpair[p_id])
            {
              cps_.base_point[p_id].push_back(cps_.base_point[p_id + 1].back());
              cps_.direction[p_id].push_back(cps_.direction[p_id + 1].back());
            }
        }
        else
          ROS_WARN_COND(VERBOSE_OUTPUT, "Failed to generate direction. It doesn't matter.");
      }

      force_stop_type_ = STOP_FOR_REBOUND;
      return true;
    }

    return false;
  }

  Eigen::Vector3d PolyTrajOptimizer::tryExtendAndChkP(Eigen::Vector3d p, Eigen::Vector3d v, Eigen::Vector3d q)
  {
    const int STOP_GRID = 0; // debug
    double dist;
    double reso = map_->big_->getResolution();

    if ((p - q).norm() > 3.0) // debug
    {
      // dbgopt_.back().trigger_type = 456;

      ROS_ERROR("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
      ROS_ERROR("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
      ROS_ERROR("p=(%f %f %f), q=(%f %f %f), abnormal", p(0), p(1), p(2), q(0), q(1), q(2));
      ROS_ERROR("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
      ROS_ERROR("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    }

    dist = map_->big_->getDistancePessi(p);
    for (int i = 1; i <= STOP_GRID; ++i)
    {
      Eigen::Vector3d p_next = p + reso * v;
      double dist_next = map_->big_->getDistancePessi(p_next);
      if (dist_next - dist > 0.8 * reso)
      {
        p = p_next;
        dist = dist_next;
      }
      else
        break;
    }

    return p;
  }

  bool PolyTrajOptimizer::allowRebound(void) // zxzxzx
  {
    // criterion 1
    if (iter_num_ < 3)
      return false;

    // criterion 2
    double min_product = 1;
    for (int i = 3; i <= cps_.points.cols() - 4; ++i) // ignore head and tail
    {
      double product = ((cps_.points.col(i) - cps_.points.col(i - 1)).normalized()).dot((cps_.points.col(i + 1) - cps_.points.col(i)).normalized());
      if (product < min_product)
      {
        min_product = product;
      }
    }
    if (min_product < 0.7) // 45 degree
      return false;

    // criterion 3
    if (multitopology_data_.use_multitopology_trajs)
    {
      if (!multitopology_data_.initial_obstacles_avoided)
      {
        bool avoided = true;
        for (int i = 1; i < cps_.points.cols() - 1; ++i)
        {
          if (cps_.base_point[i].size() > 0)
          {
            // Only adopts "0" since finelyCheckAndSetConstraintPoints() after one optimization can add more base_points.
            if ((cps_.points.col(i) - cps_.base_point[i][0]).dot(cps_.direction[i][0]) < 0)
            {
              avoided = false;
              break;
            }
          }
        }

        multitopology_data_.initial_obstacles_avoided = avoided;
      }

      if (!multitopology_data_.initial_obstacles_avoided)
      {
        return false;
      }
    }

    // all the criterion passed
    return true;
  }

  void PolyTrajOptimizer::prepareFittedCurve()
  {
    for (int cpid = 0; cpid < cps_.cp_size; ++cpid)
    {
      Eigen::Vector3d fitted_p = cps_.points.col(cpid);
      double scale = 1 / (cps_.points.col(cpid == cps_.cp_size - 1 ? cpid : cpid + 1) - cps_.points.col(cpid == 0 ? cpid : cpid - 1)).norm();
      Eigen::Vector3d dir = (cps_.points.col(cpid == cps_.cp_size - 1 ? cpid : cpid + 1) - cps_.points.col(cpid == 0 ? cpid : cpid - 1)).normalized();
      cps_.curve_fitting[cpid] = std::make_pair(std::make_pair(fitted_p, scale), dir);
    }
  }

  void PolyTrajOptimizer::computeVelLim(const Eigen::MatrixXd &iniState, Eigen::MatrixXd finState)
  {
    constexpr double MIN_VEL = 0.1, DACC = 3.0 /* about 17-degree drone tilt */, SIDE_ACC_MAX = 1.5;
    const double DV = pp_cpy_.speed_mode == PlanParameters::MODE::SLOW ? 0.15 : 0.3;
    std::fill(cps_.vel_limit.begin(), cps_.vel_limit.end(), max_vel_);
    std::fill(cps_.acc_limit.begin(), cps_.acc_limit.end(), max_acc_);
    auto traj = jerkOpt_.getTraj();

    // cout << "max_vel_=" << max_vel_ << endl;

    // speed limit to goal
    if (touch_goal_)
    {
      for (int cpid = 0; cpid < cps_.points.cols(); ++cpid)
      {
        cps_.vel_limit[cpid] = max(MIN_VEL, min(sqrt(2 * DACC * (cps_.points.col(cpid) - cps_.points.rightCols(1)).norm()), max_vel_));
        cps_.acc_limit[cpid] = min(cps_.vel_limit[cpid] * 3.0, max_acc_);
      }
    }

    // cout << "cps_.vel_limit[cpid]=";
    // for (int cpid = 0; cpid < cps_.points.cols(); ++cpid)
    // {
    //   cout << "  " << cps_.vel_limit[cpid];
    // }
    // cout << endl;

    // speed limit to obstacle
    for (int cpid = 0; cpid < cps_.points.cols(); ++cpid)
    {
      double dist_min = 9999.0;
      double wallside_a_max = 0.0;
      for (size_t pvid = 0; pvid < cps_.base_point[cpid].size(); ++pvid)
      {
        dist_min = min(dist_min, (cps_.points.col(cpid) - cps_.base_point[cpid][pvid]).dot(cps_.direction[cpid][pvid]));
        Eigen::Vector3d a = traj.getAcc(cps_.t[cpid]);
        wallside_a_max = max(wallside_a_max, abs(a.dot(cps_.direction[cpid][pvid].normalized())));
      }

      if (cps_.base_point[cpid].size() > 0)
      {
        Eigen::Vector3d v = traj.getVel(cps_.t[cpid]);
        Eigen::Vector3d a = traj.getAcc(cps_.t[cpid]);
        double side_a = max(1.0, sqrt(a.squaredNorm() - pow(a.dot(v.normalized()), 2)));
        cps_.vel_limit[cpid] = min(sqrt(SIDE_ACC_MAX / side_a) * v.norm(), max_vel_);
        // cout << "cpid=" << cpid << " wallside_a_max=" << wallside_a_max << " p=" << cps_.points.col(cpid).transpose() << " v=" << v.transpose() << " a=" << a.transpose() << " side_a=" << side_a << endl;
      }
    }

    // for (int cpid = 0; cpid < cps_.points.cols(); ++cpid)
    // {
    //   // double dist_min = 9999.0;
    //   // double wallside_a_max = 0.0;
    //   // for (size_t pvid = 0; pvid < cps_.base_point[cpid].size(); ++pvid)
    //   // {
    //   //   dist_min = min(dist_min, (cps_.points.col(cpid) - cps_.base_point[cpid][pvid]).dot(cps_.direction[cpid][pvid]));
    //   //   Eigen::Vector3d a = traj.getAcc(cps_.t[cpid]);
    //   //   wallside_a_max = max(wallside_a_max, abs(a.dot(cps_.direction[cpid][pvid].normalized())));
    //   // }

    //   // if (cps_.base_point[cpid].size() > 0)
    //   // {
    //   Eigen::Vector3d v = traj.getVel(cps_.t[cpid]);
    //   Eigen::Vector3d a = traj.getAcc(cps_.t[cpid]);
    //   double side_a = max(1.0, sqrt(a.squaredNorm() - pow(a.dot(v.normalized()), 2)));
    //   double occ_dist;
    //   if (!map_Big_->evaluateESDFWithGrad(cps_.points.col(cpid), occ_dist, NULL))
    //     continue;
    //   double side_acc = min(SIDE_ACC_MIN, SIDE_ACC_RATIO * occ_dist);
    //   cps_.vel_limit[cpid] = max(sqrt(side_acc / side_a) * v.norm(), max_vel_);
    //   cout << "  " << side_acc;
    //   // cout << "cpid=" << cpid << " wallside_a_max=" << wallside_a_max << " p=" << cps_.points.col(cpid).transpose() << " v=" << v.transpose() << " a=" << a.transpose() << " side_a=" << side_a << endl;
    //   // }
    // }

    // smoothing the speed changing
    double max_dv = DV;
    for (int cpid = 0; cpid < cps_.points.cols() - 1; ++cpid)
    {
      int tmpid = cpid;
      while (cps_.vel_limit[tmpid] - cps_.vel_limit[tmpid + 1] > max_dv)
      {
        cps_.vel_limit[tmpid] = cps_.vel_limit[tmpid + 1] + max_dv - 1e-5;
        tmpid = max(0, tmpid - 1);
      }
      tmpid = cpid;
      while (cps_.vel_limit[tmpid + 1] - cps_.vel_limit[tmpid] > max_dv)
      {
        cps_.vel_limit[tmpid + 1] = cps_.vel_limit[tmpid] + max_dv - 1e-5;
        tmpid = min((int)cps_.vel_limit.size(), tmpid + 1);
      }
    }

    // unknown region (put it behind the speed smoothing is better)
    for (int cpid = 0; cpid < cps_.points.cols(); ++cpid)
    {
      Eigen::Vector3d pt = cps_.points.col(cpid);
      if (map_->getOcc(pt) == GRID_MAP_UNKNOWN_FLAG)
      {
        double reso = map_->cur_->getResolution();
        int unknown_count = 0;
        for (double xi = -reso; xi <= reso; xi += reso)
          for (double yi = -reso; yi <= reso; yi += reso)
            for (double zi = -reso; zi <= reso; zi += reso)
              if (map_->getOcc(pt + Eigen::Vector3d(xi, yi, zi)) == GRID_MAP_UNKNOWN_FLAG)
                unknown_count++;
        if (unknown_count >= UNKNOWN_COUND_THRES)
        {
          cps_.ent_uk.cps_id = cpid;
          cps_.ent_uk.K = cps_num_prePiece_;
          cps_.ent_uk.fixed_pos = pt;
          cps_.vel_limit[cpid] = min(MIN_VEL, cps_.vel_limit[cpid]);
          cps_.ent_uk.enable = true;
          break;
        }
      }
    }

    for (int cpid = 0; cpid < cps_.points.cols(); ++cpid)
    {
      cps_.acc_limit[cpid] = min(cps_.vel_limit[cpid] * 2.0, max_acc_);
    }

    // adjust the final velocity
    if (touch_goal_)
    {
      finState.col(1) = Eigen::Vector3d::Zero();
      jerkOpt_.reset(iniState.leftCols(3), finState, piece_num_);
    }
    else if (finState.col(1).squaredNorm() > 0.01) // 0.1 m/s
    {
      finState.col(1) = finState.col(1).normalized() * cps_.vel_limit.back();
      jerkOpt_.reset(iniState.leftCols(3), finState, piece_num_);
    }
    // else
    //   ROS_WARN("finState.col(1).norm()=%f < 0.01", finState.col(1).norm());

    // cout << "cps_.vel_limit[cpid]=";
    // for (int cpid = 0; cpid < cps_.points.cols(); ++cpid)
    // {
    //   cout << "  " << cps_.vel_limit[cpid];
    // }
    // cout << endl;

    // debug
    // if (cps_.vel_limit[0] >= 4.99)
    //   dbgopt_.back().trigger_type = 885;

    // set flag
    cps_.dyn_limit_valid = true;
  }

  /* multi-topo support */
  std::vector<ConstraintPoints> PolyTrajOptimizer::distinctiveTrajs(vector<std::pair<int, int>> segments)
  {
    if (segments.size() == 0) // will be invoked again later.
    {
      std::vector<ConstraintPoints> oneSeg;
      oneSeg.push_back(cps_);
      return oneSeg;
    }

    constexpr int MAX_TRAJS = 4;
    constexpr int VARIS = 2;
    int seg_upbound = std::min((int)segments.size(), static_cast<int>(floor(log(MAX_TRAJS) / log(VARIS))));
    std::vector<ConstraintPoints> control_pts_buf;
    control_pts_buf.reserve(MAX_TRAJS);
    const double RESOLUTION = map_->cur_->getResolution();
    const double CTRL_PT_DIST = (cps_.points.col(0) - cps_.points.col(cps_.cp_size - 1)).norm() / (cps_.cp_size - 1);

    // Step 1. Find the opposite vectors and base points for every segment.
    std::vector<std::pair<ConstraintPoints, ConstraintPoints>> RichInfoSegs;
    for (int i = 0; i < seg_upbound; i++)
    {
      std::pair<ConstraintPoints, ConstraintPoints> RichInfoOneSeg;
      ConstraintPoints RichInfoOneSeg_temp;
      cps_.segment(RichInfoOneSeg_temp, segments[i].first, segments[i].second);
      RichInfoOneSeg.first = RichInfoOneSeg_temp;
      RichInfoOneSeg.second = RichInfoOneSeg_temp;
      RichInfoSegs.push_back(RichInfoOneSeg);
    }

    for (int i = 0; i < seg_upbound; i++)
    {

      // 1.1 Find the start occupied point id and the last occupied point id
      if (RichInfoSegs[i].first.cp_size > 1)
      {
        int occ_start_id = -1, occ_end_id = -1;
        Eigen::Vector3d occ_start_pt, occ_end_pt;
        for (int j = 0; j < RichInfoSegs[i].first.cp_size - 1; j++)
        {
          double step_size = RESOLUTION / (RichInfoSegs[i].first.points.col(j) - RichInfoSegs[i].first.points.col(j + 1)).norm() / 2;
          for (double a = 1; a > 0; a -= step_size)
          {
            Eigen::Vector3d pt(a * RichInfoSegs[i].first.points.col(j) + (1 - a) * RichInfoSegs[i].first.points.col(j + 1));
            if (map_->getOcc(pt) > 0)
            {
              occ_start_id = j;
              occ_start_pt = pt;
              goto exit_multi_loop1;
            }
          }
        }
      exit_multi_loop1:;
        for (int j = RichInfoSegs[i].first.cp_size - 1; j >= 1; j--)
        {
          ;
          double step_size = RESOLUTION / (RichInfoSegs[i].first.points.col(j) - RichInfoSegs[i].first.points.col(j - 1)).norm();
          for (double a = 1; a > 0; a -= step_size)
          {
            Eigen::Vector3d pt(a * RichInfoSegs[i].first.points.col(j) + (1 - a) * RichInfoSegs[i].first.points.col(j - 1));
            if (map_->getOcc(pt) > 0)
            {
              occ_end_id = j;
              occ_end_pt = pt;
              goto exit_multi_loop2;
            }
          }
        }
      exit_multi_loop2:;

        // double check
        if (occ_start_id == -1 || occ_end_id == -1)
        {
          // It means that the first or the last control points of one segment are in obstacles, which is not allowed.
          // ROS_WARN("What? occ_start_id=%d, occ_end_id=%d", occ_start_id, occ_end_id);

          segments.erase(segments.begin() + i);
          RichInfoSegs.erase(RichInfoSegs.begin() + i);
          seg_upbound--;
          i--;

          continue;
        }

        // 1.2 Reverse the vector and find new base points from occ_start_id to occ_end_id.
        for (int j = occ_start_id; j <= occ_end_id; j++)
        {
          Eigen::Vector3d base_pt_reverse, base_vec_reverse;
          if (RichInfoSegs[i].first.base_point[j].size() != 1)
          {
            cout << "RichInfoSegs[" << i << "].first.base_point[" << j << "].size()=" << RichInfoSegs[i].first.base_point[j].size() << endl;
            ROS_ERROR("Wrong number of base_points!!! Should not be happen!. 0x1");

            cout << setprecision(5);
            cout << "cps_" << endl;
            cout << " clearance=" << obs_clearance_ << " cps.size=" << cps_.cp_size << endl;
            for (int temp_i = 0; temp_i < cps_.cp_size; temp_i++)
            {
              if (cps_.base_point[temp_i].size() > 1 && cps_.base_point[temp_i].size() < 1000)
              {
                ROS_ERROR("Should not happen!!!");
                cout << "######" << cps_.points.col(temp_i).transpose() << endl;
                for (size_t temp_j = 0; temp_j < cps_.base_point[temp_i].size(); temp_j++)
                  cout << "      " << cps_.base_point[temp_i][temp_j].transpose() << " @ " << cps_.direction[temp_i][temp_j].transpose() << endl;
              }
            }

            std::vector<ConstraintPoints> blank;
            return blank;
          }

          base_vec_reverse = -RichInfoSegs[i].first.direction[j][0];

          // The start and the end case must get taken special care of.
          if (j == occ_start_id)
          {
            base_pt_reverse = occ_start_pt;
          }
          else if (j == occ_end_id)
          {
            base_pt_reverse = occ_end_pt;
          }
          else
          {
            base_pt_reverse = RichInfoSegs[i].first.points.col(j) + base_vec_reverse * (RichInfoSegs[i].first.base_point[j][0] - RichInfoSegs[i].first.points.col(j)).norm();
          }

          if (map_->getOcc(base_pt_reverse) > 0) // Search outward.
          {
            double l_upbound = 5 * CTRL_PT_DIST; // "5" is the threshold.
            double l = RESOLUTION;
            for (; l <= l_upbound; l += RESOLUTION)
            {
              Eigen::Vector3d base_pt_temp = base_pt_reverse + l * base_vec_reverse;
              if (!(map_->getOcc(base_pt_temp) > 0))
              {
                RichInfoSegs[i].second.base_point[j][0] = base_pt_temp;
                RichInfoSegs[i].second.direction[j][0] = base_vec_reverse;
                break;
              }
            }
            if (l > l_upbound)
            {
              ROS_WARN_COND(VERBOSE_OUTPUT, "Can't find the new base points at the opposite within the threshold. i=%d, j=%d", i, j);

              segments.erase(segments.begin() + i);
              RichInfoSegs.erase(RichInfoSegs.begin() + i);
              seg_upbound--;
              i--;

              goto exit_multi_loop3; // break "for (int j = 0; j < RichInfoSegs[i].first.size; j++)"
            }
          }
          else if ((base_pt_reverse - RichInfoSegs[i].first.points.col(j)).norm() >= RESOLUTION) // Unnecessary to search.
          {
            RichInfoSegs[i].second.base_point[j][0] = base_pt_reverse;
            RichInfoSegs[i].second.direction[j][0] = base_vec_reverse;
          }
          else
          {
            ROS_WARN_COND(VERBOSE_OUTPUT, "base_point and control point are too close!");
            if (VERBOSE_OUTPUT)
              cout << "base_point=" << RichInfoSegs[i].first.base_point[j][0].transpose() << " control point=" << RichInfoSegs[i].first.points.col(j).transpose() << endl;

            segments.erase(segments.begin() + i);
            RichInfoSegs.erase(RichInfoSegs.begin() + i);
            seg_upbound--;
            i--;

            goto exit_multi_loop3; // break "for (int j = 0; j < RichInfoSegs[i].first.size; j++)"
          }
        }

        // 1.3 Assign the base points to control points within [0, occ_start_id) and (occ_end_id, RichInfoSegs[i].first.size()-1].
        if (RichInfoSegs[i].second.cp_size)
        {
          for (int j = occ_start_id - 1; j >= 0; j--)
          {
            RichInfoSegs[i].second.base_point[j][0] = RichInfoSegs[i].second.base_point[occ_start_id][0];
            RichInfoSegs[i].second.direction[j][0] = RichInfoSegs[i].second.direction[occ_start_id][0];
          }
          for (int j = occ_end_id + 1; j < RichInfoSegs[i].second.cp_size; j++)
          {
            RichInfoSegs[i].second.base_point[j][0] = RichInfoSegs[i].second.base_point[occ_end_id][0];
            RichInfoSegs[i].second.direction[j][0] = RichInfoSegs[i].second.direction[occ_end_id][0];
          }
        }

      exit_multi_loop3:;
      }
      else
      {
        if (RichInfoSegs[i].first.direction.size() != 1 || RichInfoSegs[i].first.direction[0].size() != 1)
        {
          if (RichInfoSegs[i].first.direction.size() != 1)
            cout << "RichInfoSegs[" << i << "].first.direction.size()=" << RichInfoSegs[i].first.direction.size() << endl;
          else
            cout << "RichInfoSegs[" << i << "].first.base_point[0].size()=" << RichInfoSegs[i].first.base_point[0].size() << endl;
          ROS_ERROR("Wrong number of base_points!!! Should not be happen!. 0x2");

          cout << setprecision(5);
          cout << "cps_" << endl;
          cout << " clearance=" << obs_clearance_ << " cps.size=" << cps_.cp_size << endl;
          for (int temp_i = 0; temp_i < cps_.cp_size; temp_i++)
          {
            if (cps_.base_point[temp_i].size() > 1 && cps_.base_point[temp_i].size() < 1000)
            {
              ROS_ERROR("Should not happen!!!");
              cout << "######" << cps_.points.col(temp_i).transpose() << endl;
              for (size_t temp_j = 0; temp_j < cps_.base_point[temp_i].size(); temp_j++)
                cout << "      " << cps_.base_point[temp_i][temp_j].transpose() << " @ " << cps_.direction[temp_i][temp_j].transpose() << endl;
            }
          }

          std::vector<ConstraintPoints> blank;
          return blank;
        }

        Eigen::Vector3d base_vec_reverse = -RichInfoSegs[i].first.direction[0][0]; // debug "direction" may be empty!
        Eigen::Vector3d base_pt_reverse = RichInfoSegs[i].first.points.col(0) + base_vec_reverse * (RichInfoSegs[i].first.base_point[0][0] - RichInfoSegs[i].first.points.col(0)).norm();

        if (map_->getOcc(base_pt_reverse) > 0) // Search outward.
        {
          double l_upbound = 5 * CTRL_PT_DIST; // "5" is the threshold.
          double l = RESOLUTION;
          for (; l <= l_upbound; l += RESOLUTION)
          {
            Eigen::Vector3d base_pt_temp = base_pt_reverse + l * base_vec_reverse;
            if (!(map_->getOcc(base_pt_temp) > 0))
            {
              RichInfoSegs[i].second.base_point[0][0] = base_pt_temp;
              RichInfoSegs[i].second.direction[0][0] = base_vec_reverse;
              break;
            }
          }
          if (l > l_upbound)
          {
            ROS_WARN_COND(VERBOSE_OUTPUT, "Can't find the new base points at the opposite within the threshold, 2. i=%d", i);

            segments.erase(segments.begin() + i);
            RichInfoSegs.erase(RichInfoSegs.begin() + i);
            seg_upbound--;
            i--;
          }
        }
        else if ((base_pt_reverse - RichInfoSegs[i].first.points.col(0)).norm() >= RESOLUTION) // Unnecessary to search.
        {
          RichInfoSegs[i].second.base_point[0][0] = base_pt_reverse;
          RichInfoSegs[i].second.direction[0][0] = base_vec_reverse;
        }
        else
        {
          ROS_WARN_COND(VERBOSE_OUTPUT, "base_point and control point are too close!, 2");
          if (VERBOSE_OUTPUT)
            cout << "base_point=" << RichInfoSegs[i].first.base_point[0][0].transpose() << " control point=" << RichInfoSegs[i].first.points.col(0).transpose() << endl;

          segments.erase(segments.begin() + i);
          RichInfoSegs.erase(RichInfoSegs.begin() + i);
          seg_upbound--;
          i--;
        }
      }
    }

    // Step 2. Assemble each segment to make up the new control point sequence.
    if (seg_upbound == 0) // After the erase operation above, segment legth will decrease to 0 again.
    {
      std::vector<ConstraintPoints> oneSeg;
      oneSeg.push_back(cps_);
      return oneSeg;
    }

    std::vector<int> selection(seg_upbound);
    std::fill(selection.begin(), selection.end(), 0);
    selection[0] = -1; // init
    int max_traj_nums = static_cast<int>(pow(VARIS, seg_upbound));
    for (int i = 0; i < max_traj_nums; i++)
    {
      // 2.1 Calculate the selection table.
      int digit_id = 0;
      selection[digit_id]++;
      while (digit_id < seg_upbound && selection[digit_id] >= VARIS)
      {
        selection[digit_id] = 0;
        digit_id++;
        if (digit_id >= seg_upbound)
        {
          ROS_ERROR("Should not happen!!! digit_id=%d, seg_upbound=%d", digit_id, seg_upbound);
        }
        selection[digit_id]++;
      }

      // 2.2 Assign params according to the selection table.
      ConstraintPoints cpsOneSample;
      cpsOneSample.resize_cp(cps_.cp_size);
      int cp_id = 0, seg_id = 0, cp_of_seg_id = 0;
      while (/*seg_id < RichInfoSegs.size() ||*/ cp_id < cps_.cp_size)
      {

        if (seg_id >= seg_upbound || cp_id < segments[seg_id].first || cp_id > segments[seg_id].second)
        {
          cpsOneSample.points.col(cp_id) = cps_.points.col(cp_id);
          cpsOneSample.base_point[cp_id] = cps_.base_point[cp_id];
          cpsOneSample.direction[cp_id] = cps_.direction[cp_id];
        }
        else if (cp_id >= segments[seg_id].first && cp_id <= segments[seg_id].second)
        {
          if (!selection[seg_id]) // zx-todo
          {
            cpsOneSample.points.col(cp_id) = RichInfoSegs[seg_id].first.points.col(cp_of_seg_id);
            cpsOneSample.base_point[cp_id] = RichInfoSegs[seg_id].first.base_point[cp_of_seg_id];
            cpsOneSample.direction[cp_id] = RichInfoSegs[seg_id].first.direction[cp_of_seg_id];
            cp_of_seg_id++;
          }
          else
          {
            if (RichInfoSegs[seg_id].second.cp_size)
            {
              cpsOneSample.points.col(cp_id) = RichInfoSegs[seg_id].second.points.col(cp_of_seg_id);
              cpsOneSample.base_point[cp_id] = RichInfoSegs[seg_id].second.base_point[cp_of_seg_id];
              cpsOneSample.direction[cp_id] = RichInfoSegs[seg_id].second.direction[cp_of_seg_id];
              cp_of_seg_id++;
            }
            else
            {
              // Abandon this trajectory.
              goto abandon_this_trajectory;
            }
          }

          if (cp_id == segments[seg_id].second)
          {
            cp_of_seg_id = 0;
            seg_id++;
          }
        }
        else
        {
          ROS_ERROR("Shold not happen!!!!, cp_id=%d, seg_id=%d, segments.front().first=%d, segments.back().second=%d, segments[seg_id].first=%d, segments[seg_id].second=%d",
                    cp_id, seg_id, segments.front().first, segments.back().second, segments[seg_id].first, segments[seg_id].second);
        }

        cp_id++;
      }

      control_pts_buf.push_back(cpsOneSample);

    abandon_this_trajectory:;
    }

    return control_pts_buf;
  }

  /* mappings between real world time and unconstrained virtual time */
  template <typename EIGENVEC>
  void PolyTrajOptimizer::RealT2VirtualT(const Eigen::VectorXd &RT, EIGENVEC &VT)
  {
    for (int i = 0; i < RT.size(); ++i)
    {
      VT(i) = RT(i) > 1.0 ? (sqrt(2.0 * RT(i) - 1.0) - 1.0)
                          : (1.0 - sqrt(2.0 / RT(i) - 1.0));
    }
  }

  template <typename EIGENVEC>
  void PolyTrajOptimizer::VirtualT2RealT(const EIGENVEC &VT, Eigen::VectorXd &RT)
  {
    for (int i = 0; i < VT.size(); ++i)
    {
      RT(i) = VT(i) > 0.0 ? ((0.5 * VT(i) + 1.0) * VT(i) + 1.0)
                          : 1.0 / ((0.5 * VT(i) - 1.0) * VT(i) + 1.0);
    }
  }

  template <typename EIGENVEC, typename EIGENVECGD>
  void PolyTrajOptimizer::VirtualTGradCost(
      const Eigen::VectorXd &RT, const EIGENVEC &VT,
      const Eigen::VectorXd &gdRT, EIGENVECGD &gdVT,
      double &costT)
  {
    for (int i = 0; i < VT.size(); ++i)
    {
      double gdVT2Rt;
      if (VT(i) > 0)
      {
        gdVT2Rt = VT(i) + 1.0;
      }
      else
      {
        double denSqrt = (0.5 * VT(i) - 1.0) * VT(i) + 1.0;
        gdVT2Rt = (1.0 - VT(i)) / (denSqrt * denSqrt);
      }

      gdVT(i) = (gdRT(i) + wei_time_) * gdVT2Rt;
    }

    costT = RT.sum() * wei_time_;
  }

  /* gradient and cost evaluation functions */
  template <typename EIGENVEC>
  void PolyTrajOptimizer::initAndGetSmoothnessGradCost2PT(EIGENVEC &gdT, double &cost)
  {
    jerkOpt_.initGradCost(gdT, cost);
  }

  void PolyTrajOptimizer::initAndGetSmoothnessGradCost2P(double &cost)
  {
    jerkOpt_.initGradCostPOnly(cost);
  }

  template <typename EIGENVEC>
  void PolyTrajOptimizer::addPVAJGradCost2CT(EIGENVEC &gdT, Eigen::VectorXd &cost_rec, const int &K)
  {
    //
    int N = gdT.size();
    Eigen::Vector3d pos, vel, acc, jer, sna, dsna;
    Eigen::Vector3d gradp, gradv, grada, gradj, grads;
    double costp, costv, costa, costj, costs;
    Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3, beta4, beta5;
    double s1, s2, s3, s4, s5;
    double step, alpha;
    Eigen::Matrix<double, 6, 3> gradViolaPc, gradViolaVc, gradViolaAc, gradViolaJc, gradViolaSc;
    double gradViolaPt, gradViolaVt, gradViolaAt, gradViolaJt, gradViolaSt;
    double omg;
    int i_dp = 0;
    cost_rec.setZero();

    // int innerLoop;
    double t = 0;
    for (int i = 0; i < N; ++i)
    {

      const Eigen::Matrix<double, 6, 3> &c = jerkOpt_.get_b().block<6, 3>(i * 6, 0);
      double Ti = jerkOpt_.get_T1()(i);
      step = Ti / K;
      s1 = 0.0;
      // innerLoop = K;

      for (int j = 0; j <= K; ++j)
      {
        s2 = s1 * s1;
        s3 = s2 * s1;
        s4 = s2 * s2;
        s5 = s4 * s1;
        beta0 << 1.0, s1, s2, s3, s4, s5;
        beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
        beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3;
        beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2;
        beta4 << 0.0, 0.0, 0.0, 0.0, 24.0, 120.0 * s1;
        beta5 << 0.0, 0.0, 0.0, 0.0, 0.0, 120.0;
        alpha = 1.0 / K * j;
        pos = c.transpose() * beta0;
        vel = c.transpose() * beta1;
        acc = c.transpose() * beta2;
        jer = c.transpose() * beta3;
        sna = c.transpose() * beta4;
        dsna = c.transpose() * beta5;

        omg = (j == 0 || j == K) ? 0.5 : 1.0;

        cps_.points.col(i_dp) = pos;
        cps_.t[i_dp] = t + step * j;

        // collision
        if (obstacleGradCostP(i_dp, pos, gradp, costp))
        {
          gradViolaPc = beta0 * gradp.transpose();
          gradViolaPt = alpha * gradp.transpose() * vel;
          jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaPc;
          gdT(i) += omg * (costp / K + step * gradViolaPt);
          cost_rec(0) += omg * step * costp;
        }

        // restrict plane
        if (i < N - 1 && j == K && restrictplaneGradCostP(i, pos, gradp, costp))
        {
          gradViolaPc = beta0 * gradp.transpose();
          gradViolaPt = alpha * gradp.transpose() * vel;
          jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += Ti * gradViolaPc;
          gdT(i) += costp + Ti * gradViolaPt;
          cost_rec(4) += Ti * costp;

          // /******debug. Record gradient progress******/
          // plane_gdC.block<6, 3>(i * 6, 0) += Ti * gradViolaPc;
          // plane_gdT(i) += costp + Ti * gradViolaPt;
          // ofstream fd("/home/zx/workspace/EGO-Planner-v2-bigsuperZZZX/EGO-Planner-v2/log/plane.txt", ios::app);
          // fd << setprecision(7);
          // fd << endl
          //    << "iter_num_=" << dbgopt_.back().iter_num << " i=" << i << " pos=" << pos.transpose() << endl
          //    << " gdC=" << endl
          //    << (Ti * gradViolaPc).transpose() << endl
          //    << "gdT(" << i << ")=" << costp + Ti * gradViolaPt << " cost_rec=" << Ti * costp << endl;
          // fd.close();
          // /******debug. Record gradient progress******/
        }

        // enter unknown region
        if (FixUnknwonPosGradCostP(i_dp, pos, vel, gradp, costp, gradv, costv)) // only fix one point
        {
          gradViolaPc = beta0 * gradp.transpose();
          gradViolaPt = alpha * (gradp.transpose().dot(vel));
          jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += gradViolaPc;
          gdT(i) += gradViolaPt;
          cost_rec(5) += costp;

          gradViolaVc = beta1 * gradv.transpose();
          gradViolaVt = alpha * gradv.transpose() * acc;
          jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += gradViolaVc;
          gdT(i) += gradViolaVt;
          cost_rec(5) += costv;

          // /******debug. Record gradient progress******/
          // unknown_gdC.block<6, 3>(i * 6, 0) += gradViolaPc;
          // unknown_gdT(i) += gradViolaPt;
          // ofstream fd("/home/zx/workspace/EGO-Planner-v2-bigsuperZZZX/EGO-Planner-v2/log/Unknwon.txt", ios::app);
          // fd << setprecision(7);
          // fd << endl
          //    << "iter_num_=" << dbgopt_.back().iter_num << " i_dp=" << i_dp << " pos=" << pos.transpose() << endl
          //    << " gdC=" << endl
          //    << (gradViolaPc).transpose() << endl
          //    << "gdT(" << i << ")=" << gradViolaPt << " cost_rec=" << costp << endl;
          // fd.close();
          // /******debug. Record gradient progress******/

          // /**** USEFUL DEBUG TEMPLATE, DO NOT DELETE! debug. Calculate gradient correctness****/
          // const double SCALE = 100000.0;
          // const double STEP = 0.001;
          // Eigen::Vector3d gradp0, gradp1, gradp2, gradp3;
          // double costp0 = 0.0, costp1 = 0.0, costp2 = 0.0, costp3 = 0.0;
          // gradp0 = gradp1 = gradp2 = gradp3 = Eigen::Vector3d::Zero();
          // FixUnknwonPosGradCostP(i_dp, pos, vel, gradp0, costp0);
          // FixUnknwonPosGradCostP(i_dp, pos + Eigen::Vector3d(STEP, 0, 0), vel, gradp1, costp1);
          // FixUnknwonPosGradCostP(i_dp, pos + Eigen::Vector3d(0, STEP, 0), vel, gradp2, costp2);
          // FixUnknwonPosGradCostP(i_dp, pos + Eigen::Vector3d(0, 0, STEP), vel, gradp3, costp3);
          // double ana1 = (gradp1(0) + gradp0(0)) * 0.5, diff1 = (costp1 - costp0) / STEP;
          // double ana2 = (gradp2(1) + gradp0(1)) * 0.5, diff2 = (costp2 - costp0) / STEP;
          // double ana3 = (gradp3(2) + gradp0(2)) * 0.5, diff3 = (costp3 - costp0) / STEP;
          // if (abs((ana1 - diff1) / diff1) > 0.1)
          //   printf("\033[31mFixUnknwonPosGradCostP1: grad=%9.3f:%9.3f i_dp=%2d \tgradana=%9.3f:%9.3f cost=%9.3f:%9.3f\033[0m\n", ana1 * SCALE, diff1 * SCALE, i_dp, gradp1(0) * SCALE, gradp0(0) * SCALE, costp1, costp0);
          // if (abs((ana2 - diff2) / diff2) > 0.1)
          //   printf("\033[31mFixUnknwonPosGradCostP2: grad=%9.3f:%9.3f i_dp=%2d \tgradana=%9.3f:%9.3f cost=%9.3f:%9.3f\033[0m\n", ana2 * SCALE, diff2 * SCALE, i_dp, gradp2(1) * SCALE, gradp0(1) * SCALE, costp2, costp0);
          // if (abs((ana3 - diff3) / diff3) > 0.1)
          //   printf("\033[31mFixUnknwonPosGradCostP3: grad=%9.3f:%9.3f i_dp=%2d \tgradana=%9.3f:%9.3f cost=%9.3f:%9.3f\033[0m\n", ana3 * SCALE, diff3 * SCALE, i_dp, gradp3(2) * SCALE, gradp0(2) * SCALE, costp3, costp0);
          // /**** USEFUL DEBUG TEMPLATE, DO NOT DELETE! debug. Calculate gradient correctness****/
        }

        // curve fitting when optimizing time only
        if (cps_.dyn_limit_valid && CurveFittingGradCostP(i_dp, pos, gradp, costp)) // only fix one point
        {
          gradViolaPc = beta0 * gradp.transpose();
          gradViolaPt = alpha * (gradp.transpose().dot(vel));
          jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += gradViolaPc;
          gdT(i) += gradViolaPt;
          cost_rec(6) += costp;

          // /******debug. Record gradient progress******/
          // unknown_gdC.block<6, 3>(i * 6, 0) += gradViolaPc;
          // unknown_gdT(i) += gradViolaPt;
          // ofstream fd("/home/zx/workspace/EGO-Planner-v2-bigsuperZZZX/EGO-Planner-v2/log/Unknwon.txt", ios::app);
          // fd << setprecision(7);
          // fd << endl
          //    << "iter_num_=" << dbgopt_.back().iter_num << " i_dp=" << i_dp << " pos=" << pos.transpose() << endl
          //    << " gdC=" << endl
          //    << (gradViolaPc).transpose() << endl
          //    << "gdT(" << i << ")=" << gradViolaPt << " cost_rec=" << costp << endl;
          // fd.close();
          // /******debug. Record gradient progress******/
        }

        // swarm
        double gradt, grad_prev_t;
        if (swarmGradCostP(i_dp, t + step * j, pos, vel, gradp, gradt, grad_prev_t, costp))
        {
          gradViolaPc = beta0 * gradp.transpose();
          gradViolaPt = alpha * gradt;
          jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaPc;
          gdT(i) += omg * (costp / K + step * gradViolaPt);
          if (i > 0)
          {
            gdT.head(i).array() += omg * step * grad_prev_t;
          }
          cost_rec(1) += omg * step * costp;
        }

        // feasibility
        if (feasibilityGradCostV(i_dp, vel, gradv, costv))
        {
          gradViolaVc = beta1 * gradv.transpose();
          gradViolaVt = alpha * gradv.transpose() * acc;
          jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaVc;
          gdT(i) += omg * (costv / K + step * gradViolaVt);
          cost_rec(2) += omg * step * costv;
          // /******debug. Record gradient progress******/
          // vel_gdC.block<6, 3>(i * 6, 0) += omg * step * gradViolaVc;
          // vel_gdT(i) += omg * (costv / K + step * gradViolaVt);
          // ofstream fd("/home/zx/workspace/EGO-Planner-v2-bigsuperZZZX/EGO-Planner-v2/log/vel.txt", ios::app);
          // fd << setprecision(7);
          // fd << endl
          //    << "Vel: iter_num_=" << dbgopt_.back().iter_num << " Vel: i_dp=" << i_dp << " Vel=" << vel.transpose() << " norm=" << vel.norm() << endl
          //    << " gdC=" << endl
          //    << (omg * step * gradViolaVc).transpose() << endl
          //    << "gdT(" << i << ")=" << omg * (costv / K + step * gradViolaVt) << " cost_rec=" << omg * step * costv << endl;
          // fd.close();
          // /******debug. Record gradient progress******/

          // /**** USEFUL DEBUG TEMPLATE, DO NOT DELETE! debug. Calculate gradient correctness****/
          // const double SCALE = 100000.0;
          // const double STEP = 0.001;
          // Eigen::Vector3d gradp0, gradp1, gradp2, gradp3;
          // double costp0 = 0.0, costp1 = 0.0, costp2 = 0.0, costp3 = 0.0;
          // gradp0 = gradp1 = gradp2 = gradp3 = Eigen::Vector3d::Zero();
          // feasibilityGradCostV(i_dp, vel, gradp0, costp0);
          // feasibilityGradCostV(i_dp, vel + Eigen::Vector3d(STEP, 0, 0), gradp1, costp1);
          // feasibilityGradCostV(i_dp, vel + Eigen::Vector3d(0, STEP, 0), gradp2, costp2);
          // feasibilityGradCostV(i_dp, vel + Eigen::Vector3d(0, 0, STEP), gradp3, costp3);
          // double ana1 = (gradp1(0) + gradp0(0)) * 0.5, diff1 = (costp1 - costp0) / STEP;
          // double ana2 = (gradp2(1) + gradp0(1)) * 0.5, diff2 = (costp2 - costp0) / STEP;
          // double ana3 = (gradp3(2) + gradp0(2)) * 0.5, diff3 = (costp3 - costp0) / STEP;
          // if (abs((ana1 - diff1) / diff1) > 0.1)
          //   printf("\033[31mfeasibilityGradCostV1: grad=%9.3f:%9.3f i_dp=%2d \tgradana=%9.3f:%9.3f cost=%9.3f:%9.3f\033[0m\n", ana1 * SCALE, diff1 * SCALE, i_dp, gradp1(0) * SCALE, gradp0(0) * SCALE, costp1, costp0);
          // if (abs((ana2 - diff2) / diff2) > 0.1)
          //   printf("\033[31mfeasibilityGradCostV2: grad=%9.3f:%9.3f i_dp=%2d \tgradana=%9.3f:%9.3f cost=%9.3f:%9.3f\033[0m\n", ana2 * SCALE, diff2 * SCALE, i_dp, gradp2(1) * SCALE, gradp0(1) * SCALE, costp2, costp0);
          // if (abs((ana3 - diff3) / diff3) > 0.1)
          //   printf("\033[31mfeasibilityGradCostV3: grad=%9.3f:%9.3f i_dp=%2d \tgradana=%9.3f:%9.3f cost=%9.3f:%9.3f\033[0m\n", ana3 * SCALE, diff3 * SCALE, i_dp, gradp3(2) * SCALE, gradp0(2) * SCALE, costp3, costp0);
          // /**** USEFUL DEBUG TEMPLATE, DO NOT DELETE! debug. Calculate gradient correctness****/
        }

        if (feasibilityGradCostA(i_dp, acc, grada, costa))
        {
          gradViolaAc = beta2 * grada.transpose();
          gradViolaAt = alpha * grada.transpose() * jer;
          jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaAc;
          gdT(i) += omg * (costa / K + step * gradViolaAt);
          cost_rec(2) += omg * step * costa;

          // /******debug. Record gradient progress******/
          // acc_gdC.block<6, 3>(i * 6, 0) += omg * step * gradViolaAc;
          // acc_gdT(i) += omg * (costa / K + step * gradViolaAt);
          // ofstream fd("/home/zx/workspace/EGO-Planner-v2-bigsuperZZZX/EGO-Planner-v2/log/acc.txt", ios::app);
          // fd << setprecision(7);
          // fd << endl
          //    << "Acc: iter_num_=" << dbgopt_.back().iter_num << " Acc: i_dp=" << i_dp << " acc=" << acc.transpose() << " norm=" << acc.norm() << endl
          //    << " gdC=" << endl
          //    << (omg * step * gradViolaAc).transpose() << endl
          //    << "gdT(" << i << ")=" << omg * (costa / K + step * gradViolaAt) << " cost_rec=" << omg * step * costa << endl;
          // fd.close();
          // /******debug. Record gradient progress******/

          // /**** USEFUL DEBUG TEMPLATE, DO NOT DELETE! debug. Calculate gradient correctness****/
          // const double SCALE = 100000.0;
          // const double STEP = 0.001;
          // Eigen::Vector3d gradp0, gradp1, gradp2, gradp3;
          // double costp0 = 0.0, costp1 = 0.0, costp2 = 0.0, costp3 = 0.0;
          // gradp0 = gradp1 = gradp2 = gradp3 = Eigen::Vector3d::Zero();
          // feasibilityGradCostA(i_dp, acc, gradp0, costp0);
          // feasibilityGradCostA(i_dp, acc + Eigen::Vector3d(STEP, 0, 0), gradp1, costp1);
          // feasibilityGradCostA(i_dp, acc + Eigen::Vector3d(0, STEP, 0), gradp2, costp2);
          // feasibilityGradCostA(i_dp, acc + Eigen::Vector3d(0, 0, STEP), gradp3, costp3);
          // double ana1 = (gradp1(0) + gradp0(0)) * 0.5, diff1 = (costp1 - costp0) / STEP;
          // double ana2 = (gradp2(1) + gradp0(1)) * 0.5, diff2 = (costp2 - costp0) / STEP;
          // double ana3 = (gradp3(2) + gradp0(2)) * 0.5, diff3 = (costp3 - costp0) / STEP;
          // if (abs((ana1 - diff1) / diff1) > 0.1)
          //   printf("\033[31mfeasibilityGradCostA1: grad=%9.3f:%9.3f i_dp=%2d \tgradana=%9.3f:%9.3f cost=%9.3f:%9.3f\033[0m\n", ana1 * SCALE, diff1 * SCALE, i_dp, gradp1(0) * SCALE, gradp0(0) * SCALE, costp1, costp0);
          // if (abs((ana2 - diff2) / diff2) > 0.1)
          //   printf("\033[31mfeasibilityGradCostA2: grad=%9.3f:%9.3f i_dp=%2d \tgradana=%9.3f:%9.3f cost=%9.3f:%9.3f\033[0m\n", ana2 * SCALE, diff2 * SCALE, i_dp, gradp2(1) * SCALE, gradp0(1) * SCALE, costp2, costp0);
          // if (abs((ana3 - diff3) / diff3) > 0.1)
          //   printf("\033[31mfeasibilityGradCostA3: grad=%9.3f:%9.3f i_dp=%2d \tgradana=%9.3f:%9.3f cost=%9.3f:%9.3f\033[0m\n", ana3 * SCALE, diff3 * SCALE, i_dp, gradp3(2) * SCALE, gradp0(2) * SCALE, costp3, costp0);
          // /**** USEFUL DEBUG TEMPLATE, DO NOT DELETE! debug. Calculate gradient correctness****/
        }

        if (feasibilityGradCostJ(i_dp, jer, gradj, costj))
        {
          gradViolaJc = beta3 * gradj.transpose();
          gradViolaJt = alpha * gradj.transpose() * sna;
          /**** choise 1 *****/
          const int PowVal = (N >= 3 && i >= 2) ? 3 : 0;
          double TiPow = pow(Ti, PowVal);
          jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaJc * TiPow;
          gdT(i) += omg / K * TiPow * ((PowVal + 1) * costj + Ti * gradViolaJt);
          cost_rec(2) += omg * step * costj * TiPow;
          /**** choise 2 *****/
          // jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaJc;
          // gdT(i) += omg * (costa / K + step * gradViolaJt);
          // cost_rec(2) += omg * step * costj;

          /******debug. Record gradient progress******/
          // if (i_dp == 0)
          // {
          // /**** USEFUL DEBUG TEMPLATE, DO NOT DELETE! debug. Calculate gradient correctness****/
          // const double SCALE = 100000.0;
          // const double STEP = 0.001;
          // Eigen::Vector3d gradp0, gradp1, gradp2, gradp3;
          // double costp0 = 0.0, costp1 = 0.0, costp2 = 0.0, costp3 = 0.0;
          // gradp0 = gradp1 = gradp2 = gradp3 = Eigen::Vector3d::Zero();
          // feasibilityGradCostJ(i_dp, jer, gradp0, costp0);
          // feasibilityGradCostJ(i_dp, jer + Eigen::Vector3d(STEP, 0, 0), gradp1, costp1);
          // feasibilityGradCostJ(i_dp, jer + Eigen::Vector3d(0, STEP, 0), gradp2, costp2);
          // feasibilityGradCostJ(i_dp, jer + Eigen::Vector3d(0, 0, STEP), gradp3, costp3);
          // double ana1 = (gradp1(0) + gradp0(0)) * 0.5, diff1 = (costp1 - costp0) / STEP;
          // double ana2 = (gradp2(1) + gradp0(1)) * 0.5, diff2 = (costp2 - costp0) / STEP;
          // double ana3 = (gradp3(2) + gradp0(2)) * 0.5, diff3 = (costp3 - costp0) / STEP;
          // if (abs((ana1 - diff1) / diff1) > 0.1)
          //   printf("\033[31mfeasibilityGradCostJ1: grad=%9.3f:%9.3f i_dp=%2d \tgradana=%9.3f:%9.3f cost=%9.3f:%9.3f\033[0m\n", ana1 * SCALE, diff1 * SCALE, i_dp, gradp1(0) * SCALE, gradp0(0) * SCALE, costp1, costp0);
          // if (abs((ana2 - diff2) / diff2) > 0.1)
          //   printf("\033[31mfeasibilityGradCostJ2: grad=%9.3f:%9.3f i_dp=%2d \tgradana=%9.3f:%9.3f cost=%9.3f:%9.3f\033[0m\n", ana2 * SCALE, diff2 * SCALE, i_dp, gradp2(1) * SCALE, gradp0(1) * SCALE, costp2, costp0);
          // if (abs((ana3 - diff3) / diff3) > 0.1)
          //   printf("\033[31mfeasibilityGradCostJ3: grad=%9.3f:%9.3f i_dp=%2d \tgradana=%9.3f:%9.3f cost=%9.3f:%9.3f\033[0m\n", ana3 * SCALE, diff3 * SCALE, i_dp, gradp3(2) * SCALE, gradp0(2) * SCALE, costp3, costp0);
          // /**** USEFUL DEBUG TEMPLATE, DO NOT DELETE! debug. Calculate gradient correctness****/

          // jerk_gdC.block<6, 3>(i * 6, 0) += omg * step * gradViolaJc * Ti12;
          // jerk_gdT(i) += omg / K * Ti12 * (13 * costj + Ti * gradViolaJt);
          // ofstream fd("/home/zx/workspace/EGO-Planner-v2-bigsuperZZZX/EGO-Planner-v2/log/jerk.txt", ios::app);
          // fd << setprecision(7);
          // fd << endl
          //    << "Jerk: iter_num_=" << dbgopt_.back().iter_num << " Jerk: i_dp=" << i_dp << " jer=" << jer.transpose() << " norm=" << jer.norm() << " Ti12=" << Ti12 << endl
          //    << " gdC=" << endl
          //    << (omg * step * gradViolaJc * Ti12).transpose() << endl
          //    << "gdT(" << i << ")=" << omg / K * Ti12 * (13 * costj + Ti * gradViolaJt) << " cost_rec=" << omg * step * costj * Ti12 << endl;
          // fd.close();
          // jerk_gdC.block<6, 3>(i * 6, 0) += omg * step * gradViolaJc;
          // jerk_gdT(i) += omg * (costa / K + step * gradViolaJt);
          // ofstream fd("/home/zx/workspace/EGO-Planner-v2-bigsuperZZZX/EGO-Planner-v2/log/jerk.txt", ios::app);
          // fd << setprecision(7);
          // fd << endl
          //    << "Jerk: iter_num_=" << dbgopt_.back().iter_num << " Jerk: i_dp=" << i_dp << " jer=" << jer.transpose() << " norm=" << jer.norm() << endl
          //    << " gdC=" << endl
          //    << (omg * step * gradViolaJc).transpose() << endl
          //    << "gdT(" << i << ")=" << omg * (costa / K + step * gradViolaJt) << " cost_rec=" << omg * step * costj << endl;
          // fd.close();
          // }
          /******debug. Record gradient progress******/
        }

        if (feasibilityGradCostS(i_dp, sna, grads, costs))
        {
          gradViolaSc = beta4 * grads.transpose();
          gradViolaSt = alpha * grads.transpose() * dsna;
          /**** choise 1 *****/
          const int PowVal = (N >= 3 && i >= 2) ? 3 : 0;
          double TiPow = pow(Ti, PowVal);
          jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaSc * TiPow;
          gdT(i) += omg / K * TiPow * ((PowVal + 1) * costs + Ti * gradViolaSt);
          cost_rec(2) += omg * step * costs * TiPow;
        }

        s1 += step;
        if (j != K || (j == K && i == N - 1))
        {
          ++i_dp;
        }
      }

      t += jerkOpt_.get_T1()(i);
    }

    // /******debug. Record gradient progress******/
    // ofstream fd1("/home/zx/workspace/EGO-Planner-v2-bigsuperZZZX/EGO-Planner-v2/log/jerk_gdCT.txt", ios::app);
    // fd1 << setprecision(7);
    // fd1 << endl
    //     << "iter_num_=" << dbgopt_.back().iter_num << endl
    //     << "jerk_gdC=" << endl
    //     << jerk_gdC << endl
    //     << "jerk_gdT=" << endl
    //     << jerk_gdT << endl;
    // fd1.close();
    // ofstream fd2("/home/zx/workspace/EGO-Planner-v2-bigsuperZZZX/EGO-Planner-v2/log/vel_gdCT.txt", ios::app);
    // fd2 << setprecision(7);
    // fd2 << endl
    //     << "iter_num_=" << dbgopt_.back().iter_num << endl
    //     << "vel_gdC=" << endl
    //     << vel_gdC << endl
    //     << "vel_gdT=" << endl
    //     << vel_gdT << endl;
    // fd2.close();
    // ofstream fd3("/home/zx/workspace/EGO-Planner-v2-bigsuperZZZX/EGO-Planner-v2/log/acc_gdCT.txt", ios::app);
    // fd3 << setprecision(7);
    // fd3 << endl
    //     << "iter_num_=" << dbgopt_.back().iter_num << endl
    //     << "acc_gdC=" << endl
    //     << acc_gdC << endl
    //     << "acc_gdT=" << endl
    //     << acc_gdT << endl;
    // fd3.close();
    // ofstream fd4("/home/zx/workspace/EGO-Planner-v2-bigsuperZZZX/EGO-Planner-v2/log/obs_gdCT.txt", ios::app);
    // fd4 << setprecision(7);
    // fd4 << endl
    //     << "iter_num_=" << dbgopt_.back().iter_num << endl
    //     << "obs_gdC=" << endl
    //     << obs_gdC << endl
    //     << "obs_gdT=" << endl
    //     << obs_gdT << endl;
    // fd4.close();
    // ofstream fd5("/home/zx/workspace/EGO-Planner-v2-bigsuperZZZX/EGO-Planner-v2/log/plane_gdCT.txt", ios::app);
    // fd5 << setprecision(7);
    // fd5 << endl
    //     << "iter_num_=" << dbgopt_.back().iter_num << endl
    //     << "plane_gdC=" << endl
    //     << plane_gdC << endl
    //     << "plane_gdT=" << endl
    //     << plane_gdT << endl;
    // fd5.close();
    // ofstream fd6("/home/zx/workspace/EGO-Planner-v2-bigsuperZZZX/EGO-Planner-v2/log/unknown_gdCT.txt", ios::app);
    // fd6 << setprecision(7);
    // fd6 << endl
    //     << "iter_num_=" << dbgopt_.back().iter_num << endl
    //     << "unknown_gdC=" << endl
    //     << unknown_gdC << endl
    //     << "unknown_gdT=" << endl
    //     << unknown_gdT << endl;
    // fd6.close();
    // ofstream fd7("/home/zx/workspace/EGO-Planner-v2-bigsuperZZZX/EGO-Planner-v2/log/all_gdCT.txt", ios::app);
    // fd7 << setprecision(7);
    // fd7 << endl
    //     << "iter_num_=" << dbgopt_.back().iter_num << endl
    //     << "all_gdC=" << endl
    //     << jerk_gdC + vel_gdC + acc_gdC + obs_gdC + plane_gdC + unknown_gdC << endl
    //     << "all_gdT=" << endl
    //     << jerk_gdT + vel_gdT + acc_gdT + obs_gdT + plane_gdT + unknown_gdT << endl;
    // fd7.close();
    // /******debug. Record gradient progress******/
  }

  void PolyTrajOptimizer::addPGradCost2C(Eigen::VectorXd &costs, const int &N, const int &K)
  {
    int i_dp = 0;
    double costp;
    Eigen::Vector3d gradp;
    costs.setZero();

    // int innerLoop;
    double s1;
    double t = 0;
    for (int i = 0; i < N; ++i)
    {

      const Eigen::Matrix<double, 6, 3> &c = jerkOpt_.get_b().block<6, 3>(i * 6, 0);
      double Ti = jerkOpt_.get_T1()(i);
      // double Ti12 = pow(Ti, 12);
      double step = Ti / K;
      s1 = 0.0;
      // innerLoop = K;

      for (int j = 0; j <= K; ++j)
      {
        double s2 = s1 * s1;
        double s3 = s2 * s1;
        double s4 = s2 * s2;
        double s5 = s4 * s1;
        Eigen::Matrix<double, 6, 1> beta0;
        beta0 << 1.0, s1, s2, s3, s4, s5;
        Eigen::Vector3d pos = c.transpose() * beta0;

        double omg = (j == 0 || j == K) ? 0.5 : 1.0;

        cps_.points.col(i_dp) = pos;
        cps_.t[i_dp] = t + step * j;

        // collision
        if (obstacleGradCostP(i_dp, pos, gradp, costp))
        {
          Eigen::Matrix<double, 6, 3> gradViolaPc = beta0 * gradp.transpose();
          jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaPc;
          costs(0) += omg * step * costp;
        }

        if (ESDFGradCostP(i_dp, pos, gradp, costp))
        {
          Eigen::Matrix<double, 6, 3> gradViolaPc = beta0 * gradp.transpose();
          jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaPc;
          costs(0) += omg * step * costp;
        }

        s1 += step;
        if (j != K || (j == K && i == N - 1))
        {
          ++i_dp;
        }
      }

      t += jerkOpt_.get_T1()(i);
    }
  }

  bool PolyTrajOptimizer::obstacleGradCostP(const int i_dp,
                                            const Eigen::Vector3d &p,
                                            Eigen::Vector3d &gradp,
                                            double &costp)
  {
    if (i_dp == 0 || i_dp > ConstraintPoints::two_thirds_id(cps_.points.cols(), touch_goal_ || pp_cpy_.speed_mode == PlanParameters::MODE::FAST)) // only apply to first 2/3
      return false;

    bool ret = false;
    const double TRUST_REGION = 0.6; // two times the normal distance between two cps

    gradp.setZero();
    costp = 0;

    // Obatacle cost
    for (size_t j = 0; j < cps_.direction[i_dp].size(); ++j)
    {
      Eigen::Vector3d ray = (p - cps_.base_point[i_dp][j]);
      double dist = ray.dot(cps_.direction[i_dp][j]);
      double dist_err = (obs_clearance_ - dist);
      // double dist_err_soft = obs_clearance_soft_ - dist;
      Eigen::Vector3d dist_grad = cps_.direction[i_dp][j];
      double Eu_dist_err2 = ray.squaredNorm() - TRUST_REGION * TRUST_REGION;

      if (dist_err > 0)
      {
        ret = true;
        double dist_err3 = dist_err * dist_err * dist_err;
        double dist_err4 = dist_err3 * dist_err;
        costp += wei_obs_ * dist_err4;
        gradp += -wei_obs_ * 4.0 * dist_err3 * dist_grad;
      }

      // if (dist_err_soft > 0)
      // {
      //   ret = true;
      //   double r = 0.05;
      //   double rsqr = r * r;
      //   double term = sqrt(1.0 + dist_err_soft * dist_err_soft / rsqr);
      //   costp += wei_obs_soft_ * rsqr * (term - 1.0);
      //   gradp += -wei_obs_soft_ * dist_err_soft / term * dist_grad;
      // }

      if (Eu_dist_err2 > 0)
      {
        ret = true;
        costp += wei_trust_region_ * Eu_dist_err2 * Eu_dist_err2;
        gradp += wei_trust_region_ * 4.0 * Eu_dist_err2 * ray;
      }
    }

    // // esdf
    // double dist;
    // Eigen::Vector3d dist_grad;
    // map_->cur_->evaluateESDFWithGrad(p, dist, dist_grad);
    // if (dist_grad.norm() > 1e-4)
    //   dist_grad.normalize();

    // if (dist < obs_clearance_soft_)
    // {
    //   costp += wei_obs_soft_ * pow(dist - obs_clearance_soft_, 2);
    //   gradp += wei_obs_soft_ * 2.0 * (dist - obs_clearance_soft_) * dist_grad;
    // }

    return ret;
  }

  bool PolyTrajOptimizer::ESDFGradCostP(const int i_dp,
                                        const Eigen::Vector3d &p,
                                        Eigen::Vector3d &gradp,
                                        double &costp)
  {
    if (i_dp == 0 || i_dp > ConstraintPoints::two_thirds_id(cps_.points.cols(), touch_goal_ || pp_cpy_.speed_mode == PlanParameters::MODE::FAST)) // only apply to first 2/3
      return false;

    bool ret = false;

    // esdf
    double dist;
    double grad[3];
    if (map_->big_->evaluateESDFWithGrad(p, dist, grad))
    {
      Eigen::Map<Eigen::Vector3d> dist_grad(grad, 3);
      // if (dist_grad.norm() > 1e-4)
      //   dist_grad.normalize();

      if (dist < obs_clearance_soft_)
      {
        costp = wei_obs_soft_ * pow(dist - obs_clearance_soft_, 2);
        gradp = wei_obs_soft_ * 2.0 * (dist - obs_clearance_soft_) * dist_grad;
        // cout << "i_dp=" << i_dp << " derr=" << dist - obs_clearance_soft_ << " costp=" << costp << " gradp=" << gradp.transpose() << endl;
        ret = true;
      }
    }

    return ret;
  }

  bool PolyTrajOptimizer::restrictplaneGradCostP(const int i_piece,
                                                 const Eigen::Vector3d &p,
                                                 Eigen::Vector3d &gradp,
                                                 double &costp)
  {
    if ((p - jerkOpt_.getTraj().getJuncPos(i_piece + 1)).norm() > 1e-5)
    {
      ROS_ERROR("return! (p - jerkOpt_.getTraj().getJuncPos(i_piece)).norm() > 1e-5");
      cout << "p=" << p.transpose() << " jerkOpt_.getTraj().getJuncPos(i_piece)=" << jerkOpt_.getTraj().getJuncPos(i_piece).transpose() << endl;
      return false;
    }

    gradp.setZero();
    costp = 0;

    // cout << "restrict_plane_[i_piece].first" << restrict_plane_[i_piece].first << endl;

    double numerator = restrict_plane_[i_piece].first.dot(p) + restrict_plane_[i_piece].second;
    double denominator2 = restrict_plane_[i_piece].first.squaredNorm();
    double deviation2 = numerator * numerator / denominator2;
    costp = wei_plane_ * deviation2;
    gradp = wei_plane_ * 2 * numerator / denominator2 * restrict_plane_[i_piece].first;

    return true;
  }

  bool cpsDistGradCostP(const int cps_id,
                        const Eigen::Vector3d &p,
                        Eigen::Vector3d &gradp,
                        double &costp)
  {
    gradp.setZero();
    costp = 0;
    return false;
  }

  bool PolyTrajOptimizer::FixUnknwonPosGradCostP(const int cps_id,
                                                 const Eigen::Vector3d &p,
                                                 const Eigen::Vector3d &v,
                                                 Eigen::Vector3d &gradp,
                                                 double &costp,
                                                 Eigen::Vector3d &gradv,
                                                 double &costv)
  {
    // Unknown region penlaty should be add on the entire trajectory
    // if (cps_id > ConstraintPoints::two_thirds_id(cps_.points.cols(), touch_goal_))
    //   return false;

    if (!cps_.ent_uk.enable || cps_id != cps_.ent_uk.cps_id)
    {
      return false;
    }

    uk_pos_measure_ = p;
    uk_vel_measure_ = v;

    // share wei_trust_region_ (*10 because here is more important)
    costp = 10 * wei_trust_region_ * ((p - cps_.ent_uk.fixed_pos).squaredNorm());
    gradp = 10 * 2 * wei_trust_region_ * (p - cps_.ent_uk.fixed_pos);

    // velocity will be penalized in feasibilityGradCostV() again
    double vpen = v.squaredNorm() - cps_.vel_limit[cps_id] * cps_.vel_limit[cps_id];
    if (vpen > 0)
    {
      costv = 10 * wei_trust_region_ * vpen * vpen;
      gradv = 10 * wei_trust_region_ * 4 * vpen * v;
    }

    return true;
  }

  bool PolyTrajOptimizer::CurveFittingGradCostP(const int cps_id,
                                                const Eigen::Vector3d &p,
                                                Eigen::Vector3d &gradp,
                                                double &costp)
  {
    if (cps_id > ConstraintPoints::two_thirds_id(cps_.points.cols(), touch_goal_))
      return false;

    gradp.setZero();
    costp = 0;

    // const double wei_curve_fitting_ = 100000;

    // def: f = |x*v|^2/a^2 + |x×v|^2/b^2
    const double a2inv = 1 / 100.0, b2inv = 1 / 1.0;
    double scale = cps_.curve_fitting[cps_id].first.second;
    Eigen::Vector3d x = (p - cps_.curve_fitting[cps_id].first.first) * scale;
    Eigen::Vector3d v = cps_.curve_fitting[cps_id].second;

    double xdotv = x.dot(v);
    Eigen::Vector3d xcrossv = x.cross(v);
    double f = xdotv * xdotv * a2inv + xcrossv.squaredNorm() * b2inv;
    Eigen::Matrix3d m;
    m << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;

    costp += wei_curve_fitting_ * f * f;
    gradp += wei_curve_fitting_ * 2 * f * (2 * xdotv * a2inv * v + 2 * b2inv * m * xcrossv) * scale;

    return true;
  }

  bool PolyTrajOptimizer::swarmGradCostP(const int i_dp,
                                         const double t,
                                         const Eigen::Vector3d &p,
                                         const Eigen::Vector3d &v,
                                         Eigen::Vector3d &gradp,
                                         double &gradt,
                                         double &grad_prev_t,
                                         double &costp)
  {
    if (i_dp <= 0 || i_dp > ConstraintPoints::two_thirds_id(cps_.points.cols(), touch_goal_ || pp_cpy_.speed_mode == PlanParameters::MODE::FAST)) // only apply to first 2/3
      return false;

    bool ret = false;

    gradp.setZero();
    gradt = 0;
    grad_prev_t = 0;
    costp = 0;

    constexpr double a = 2.0, b = 1.0, inv_a2 = 1 / a / a, inv_b2 = 1 / b / b;

    for (size_t id = 0; id < swarm_trajs_->size(); id++)
    {
      if ((swarm_trajs_->at(id).drone_id < 0) || swarm_trajs_->at(id).drone_id == drone_id_)
      {
        continue;
      }

      double traj_i_satrt_time = swarm_trajs_->at(id).start_time;
      double pt_time = (t_now_ - traj_i_satrt_time) + t;                                      // never assign a high-precision golbal time to a double directly!
      const double CLEARANCE = (swarm_clearance_ + swarm_trajs_->at(id).des_clearance) * 1.1; // 1.1 is to compensate slight constraint violation
      const double CLEARANCE2 = CLEARANCE * CLEARANCE;

      Eigen::Vector3d swarm_p, swarm_v;
      if (pt_time < swarm_trajs_->at(id).duration)
      {
        swarm_p = swarm_trajs_->at(id).traj.getPos(pt_time);
        swarm_v = swarm_trajs_->at(id).traj.getVel(pt_time);
      }
      else
      {
        double exceed_time = pt_time - swarm_trajs_->at(id).duration;
        swarm_v = swarm_trajs_->at(id).traj.getVel(swarm_trajs_->at(id).duration);
        swarm_p = swarm_trajs_->at(id).traj.getPos(swarm_trajs_->at(id).duration) +
                  exceed_time * swarm_v;
      }
      Eigen::Vector3d dist_vec = p - swarm_p;
      double ellip_dist2 = dist_vec(2) * dist_vec(2) * inv_a2 + (dist_vec(0) * dist_vec(0) + dist_vec(1) * dist_vec(1)) * inv_b2;
      double dist2_err = CLEARANCE2 - ellip_dist2;
      double dist2_err2 = dist2_err * dist2_err;
      double dist2_err3 = dist2_err2 * dist2_err;

      if (dist2_err3 > 0)
      {
        ret = true;

        costp += wei_swarm_mod_ * dist2_err3;

        Eigen::Vector3d dJ_dP = wei_swarm_mod_ * 3 * dist2_err2 * (-2) * Eigen::Vector3d(inv_b2 * dist_vec(0), inv_b2 * dist_vec(1), inv_a2 * dist_vec(2));
        gradp += dJ_dP;
        gradt += dJ_dP.dot(v - swarm_v);
        grad_prev_t += dJ_dP.dot(-swarm_v);
      }

      if (min_ellip_dist2_[id] > ellip_dist2)
      {
        min_ellip_dist2_[id] = ellip_dist2;
      }
    }

    return ret;
  }

  bool PolyTrajOptimizer::feasibilityGradCostV(const int i_dp,
                                               const Eigen::Vector3d &v,
                                               Eigen::Vector3d &gradv,
                                               double &costv)
  {
    if (i_dp > ConstraintPoints::two_thirds_id(cps_.points.cols(), touch_goal_))
      return false;

    constexpr double VZ_UP_LIM = 3.0, VZ_DOWN_LIM = -2.0; // m/s
    bool ret = false;

    double v_squnorm = v.squaredNorm();
    // if (i_dp > cps_num_prePiece_ && v_squnorm > maxv_measure_) // we ignore the first piece
    if (v_squnorm > maxv_measure_)
      maxv_measure_ = v_squnorm;

    double v_lim2 = (cps_.dyn_limit_valid ? cps_.vel_limit[i_dp] * cps_.vel_limit[i_dp] : max_vel_ * max_vel_);
    if (v_lim2 <= 1.0)
    {
      double vpen = v_squnorm - v_lim2;
      if (vpen > 0)
      {
        costv = wei_feas_mod_ * vpen * vpen;
        gradv = wei_feas_mod_ * 4 * vpen * v;
        ret = true;
      }
    }
    else
    {
      double vpen = v_squnorm / v_lim2 - 1.0;
      if (vpen > 0)
      {
        costv = wei_feas_mod_ * vpen * vpen;
        gradv = wei_feas_mod_ * 4 * vpen / v_lim2 * v;
        ret = true;
      }
    }

    if (v.z() > VZ_UP_LIM)
    {
      double vzpen = v.z() - VZ_UP_LIM;
      costv += wei_feas_mod_ * vzpen * vzpen * vzpen;
      gradv(2) += wei_feas_mod_ * 3 * vzpen * vzpen;
      ret = true;
    }
    if (v.z() < VZ_DOWN_LIM)
    {
      double vzpen = VZ_DOWN_LIM - v.z();
      costv += wei_feas_mod_ * vzpen * vzpen * vzpen;
      gradv(2) += -wei_feas_mod_ * 3 * vzpen * vzpen;
      ret = true;
    }

    return ret;
  }

  bool PolyTrajOptimizer::feasibilityGradCostA(const int i_dp,
                                               const Eigen::Vector3d &a,
                                               Eigen::Vector3d &grada,
                                               double &costa)
  {
    if (pp_cpy_.emergency_)
      return false;

    if (i_dp > ConstraintPoints::two_thirds_id(cps_.points.cols(), touch_goal_))
      return false;

    double a_squnorm = a.squaredNorm();
    // if (i_dp > cps_num_prePiece_ && a_squnorm > maxa_measure_) // we ignore the first piece
    if (a_squnorm > maxa_measure_)
      maxa_measure_ = a_squnorm;

    double a_lim2 = (cps_.dyn_limit_valid ? cps_.acc_limit[i_dp] * cps_.acc_limit[i_dp] : max_acc_ * max_acc_);
    if (a_lim2 <= 1.0)
    {
      double apen = a_squnorm - a_lim2;
      if (apen > 0)
      {
        costa = wei_feas_mod_ * apen * apen;
        grada = wei_feas_mod_ * 4 * apen * a;
        return true;
      }
    }
    else
    {
      double apen = a_squnorm / a_lim2 - 1.0;
      if (apen > 0)
      {
        costa = wei_feas_mod_ * apen * apen;
        grada = wei_feas_mod_ * 4 * apen / a_lim2 * a;
        return true;
      }
    }

    return false;
  }

  bool PolyTrajOptimizer::feasibilityGradCostJ(const int i_dp,
                                               const Eigen::Vector3d &j,
                                               Eigen::Vector3d &gradj,
                                               double &costj)
  {
    if (pp_cpy_.emergency_)
      return false;
    if (i_dp > ConstraintPoints::two_thirds_id(cps_.points.cols(), touch_goal_))
      return false;

    if (i_dp == 0)
    {
      if (start_jerk_.norm() > max_jer_)
        start_jerk_.normalized() * max_jer_;

      double jpen = (j - start_jerk_).squaredNorm();
      costj = wei_feas_mod_ * jpen;
      gradj = wei_feas_mod_ * 2.0 * (j - start_jerk_);
      return true;
    }
    else
    {
      double j_squnorm = j.squaredNorm();
      // if (i_dp > cps_num_prePiece_ && j_squnorm > maxj_measure_) // we ignore the first piece
      if (j_squnorm > maxj_measure_)
        maxj_measure_ = j_squnorm;
      double j_lim2 = max_jer_ * max_jer_;
      if (j_lim2 <= 1.0)
      {
        double jpen = j_squnorm - j_lim2;
        if (jpen > 0)
        {
          costj = wei_feas_mod_ * jpen * jpen * jpen;
          gradj = wei_feas_mod_ * 6.0 * jpen * jpen * j;
          return true;
        }
      }
      else
      {
        double jpen = j_squnorm / j_lim2 - 1.0;
        if (jpen > 0)
        {
          costj = wei_feas_mod_ * jpen * jpen * jpen;
          gradj = wei_feas_mod_ * 6 * jpen * jpen / j_lim2 * j;
          return true;
        }
      }
    }

    return false;
  }

  bool PolyTrajOptimizer::feasibilityGradCostS(const int i_dp,
                                               const Eigen::Vector3d &s,
                                               Eigen::Vector3d &grads,
                                               double &costs)
  {
    if (pp_cpy_.emergency_)
      return false;
    if (i_dp > cps_num_prePiece_ || // we only constrain the first piece
        i_dp > ConstraintPoints::one_thirds_id(cps_.points.cols(), touch_goal_))
      return false;

    double s_squnorm = s.squaredNorm();
    double s_lim2 = max_sna_ * max_sna_;
    double spen = s_squnorm / s_lim2 - 1.0;
    if (spen > 0)
    {
      costs = wei_feas_mod_ * spen * spen;
      grads = wei_feas_mod_ * 4 * spen / s_lim2 * s;
      return true;
    }

    return false;
  }

  void PolyTrajOptimizer::distanceSqrVarianceWithGradCost2p(const Eigen::MatrixXd &ps,
                                                            Eigen::MatrixXd &gdp,
                                                            double &var)
  {
    int N = ps.cols() - 1;
    Eigen::MatrixXd dps = ps.rightCols(N) - ps.leftCols(N);
    Eigen::VectorXd dsqrs = dps.colwise().squaredNorm().transpose();
    // double dsqrsum = dsqrs.sum();
    double dquarsum = dsqrs.squaredNorm();
    // double dsqrmean = dsqrsum / N;
    double dquarmean = dquarsum / N;
    var = wei_sqrvar_ * (dquarmean);
    gdp.resize(3, N + 1);
    gdp.setZero();
    for (int i = 0; i <= N; i++)
    {
      if (i != 0)
      {
        gdp.col(i) += wei_sqrvar_ * (4.0 * (dsqrs(i - 1)) / N * dps.col(i - 1));
      }
      if (i != N)
      {
        gdp.col(i) += wei_sqrvar_ * (-4.0 * (dsqrs(i)) / N * dps.col(i));
      }
    }
    return;
  }

  void PolyTrajOptimizer::lengthVarianceWithGradCost2p(const Eigen::MatrixXd &ps,
                                                       const int n,
                                                       Eigen::MatrixXd &gdp,
                                                       double &var)
  {
    int N = ps.cols() - 1;
    int M = N / n;
    Eigen::MatrixXd dps = ps.rightCols(N) - ps.leftCols(N);
    Eigen::VectorXd ds = dps.colwise().norm().transpose();
    Eigen::VectorXd ls(M), lsqrs(M);
    for (int i = 0; i < M; i++)
    {
      ls(i) = ds.segment(i * n, n).sum();
      lsqrs(i) = ls(i) * ls(i);
    }
    double lm = ls.mean();
    double lsqrm = lsqrs.mean();
    var = wei_sqrvar_ * (lsqrm - lm * lm) + 250.0 * M * lm;
    Eigen::VectorXd gdls = wei_sqrvar_ * 2.0 / M * (ls.array() - lm) + 250.0;
    Eigen::MatrixXd gdds = dps.colwise().normalized();
    gdp.resize(3, N + 1);
    gdp.setZero();
    for (int i = 0; i < M; i++)
    {
      gdp.block(0, i * n, 3, n) -= gdls(i) * gdds.block(0, i * n, 3, n);
      gdp.block(0, i * n + 1, 3, n) += gdls(i) * gdds.block(0, i * n, 3, n);
    }
    return;
  }

  /* helper functions */
  void PolyTrajOptimizer::setParam(ros::NodeHandle &nh)
  {
    // nh.param("optimization/constraint_points_perPiece", cps_num_prePiece_, -1);
    nh.param("optimization/weight_obstacle", wei_obs_, -1.0);
    nh.param("optimization/weight_obstacle_soft", wei_obs_soft_, -1.0);
    nh.param("optimization/weight_trust_region", wei_trust_region_, -1.0);
    nh.param("optimization/weight_curve_fitting", wei_curve_fitting_, -1.0);
    nh.param("optimization/weight_plane", wei_plane_, -1.0);
    nh.param("optimization/weight_swarm", wei_swarm_, -1.0);
    nh.param("optimization/weight_feasibility", wei_feas_, -1.0);
    nh.param("optimization/weight_sqrvariance", wei_sqrvar_, -1.0);
    nh.param("optimization/weight_time", wei_time_, -1.0);
    nh.param("optimization/obstacle_clearance", obs_clearance_, -1.0);
    nh.param("optimization/obstacle_clearance_soft", obs_clearance_soft_, -1.0);
    nh.param("optimization/swarm_clearance", swarm_clearance_, -1.0);
    nh.param("optimization/max_jer", max_jer_, -1.0);
    nh.param("optimization/max_sna", max_sna_, -1.0);

    obs_clearance4_ = pow(obs_clearance_, 4);
  }

  void PolyTrajOptimizer::setEnvironment(const MapManager::Ptr map)
  {
    map_ = map;

    a_star_.reset(new dyn_a_star::AStar);
    a_star_->initAstar(map_, Eigen::Vector3i(100, 100, 100));
  }

  void PolyTrajOptimizer::setControlPoints(const Eigen::MatrixXd &points)
  {
    cps_.points = points;
  }

  void PolyTrajOptimizer::setSwarmTrajs(SwarmTrajData *swarm_trajs_ptr) { swarm_trajs_ = swarm_trajs_ptr; }

  void PolyTrajOptimizer::setDroneId(const int drone_id) { drone_id_ = drone_id; }

  void PolyTrajOptimizer::setIfTouchGoal(const bool touch_goal) { touch_goal_ = touch_goal; }

  void PolyTrajOptimizer::setConstraintPoints(ConstraintPoints cps) { cps_ = cps; }

  void PolyTrajOptimizer::setUseMultitopologyTrajs(bool use_multitopology_trajs) { multitopology_data_.use_multitopology_trajs = use_multitopology_trajs; }

  void PolyTrajOptimizer::setMaxVelAcc(double max_vel, double max_acc) { max_vel_ = max_vel, max_acc_ = max_acc; }

  void PolyTrajOptimizer::setCPsNumPerPiece(const int N) { cps_num_prePiece_ = N, cps_num_prePiece_Long_ = N; }

  void PolyTrajOptimizer::setPlanParametersCopy(const PlanParameters &pp_cpy) { pp_cpy_ = pp_cpy; }

} // namespace ego_planner
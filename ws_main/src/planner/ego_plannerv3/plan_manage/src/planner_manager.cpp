// #include <fstream>
#include <plan_manage/planner_manager.h>
#include <thread>

namespace ego_planner
{

  // SECTION interfaces for setup and query

  EGOPlannerManager::EGOPlannerManager() {}

  EGOPlannerManager::~EGOPlannerManager() { std::cout << "des manager" << std::endl; }

  void EGOPlannerManager::initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis)
  {
    /* read algorithm parameters */

    nh.param("manager/max_vel", pp_.max_vel_user_, -1.0);
    nh.param("manager/max_acc", pp_.max_acc_user_, -1.0);
    // nh.param("manager/polyTraj_piece_length", pp_.polyTraj_piece_length, -1.0);
    // nh.param("manager/planning_horizon", pp_.planning_horizen_, 5.0);
    nh.param("manager/use_multitopology_trajs", pp_.use_multitopology_trajs, false);
    nh.param("manager/drone_id", pp_.drone_id, -1);

    // grid_map_.reset(new GridMap);
    // grid_map_->initMap(nh);
    // map_Big_.reset(new GridMap);
    // map_Big_->initMap(nh);
    // map_Big_->paramAdjust(0.25, 18.0, 18.0, 4.0, true, false, true, "Big");
    map_.reset(new MapManager);
    map_->initMapManager(nh);

    poly_traj_opt_.reset(new PolyTrajOptimizer);
    poly_traj_opt_->setParam(nh);
    poly_traj_opt_->setEnvironment(map_);

    visualization_ = vis;

    poly_traj_opt_->setSwarmTrajs(&traj_.swarm_traj);
    poly_traj_opt_->setDroneId(pp_.drone_id);

    his_dendat_.clear();

    srand(ros::Time::now().nsec);
  }

  PLAN_RET EGOPlannerManager::reboundReplan(
      const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel,
      const Eigen::Vector3d &start_acc, const Eigen::Vector3d &start_jerk,
      const Eigen::Vector3d &glb_start_pt, const Eigen::Vector3d &final_goal,
      const bool flag_use_last_optimial, const bool flag_random_init,
      vector<DensityEvalRayData> *pathes, bool &touch_goal)
  {
    ros::Time t_start = ros::Time::now();
    ros::Duration t_init, t_opt;

    static int count = 0;
    cout << "\033[47;30m\n[" << t_start << "] Drone " << pp_.drone_id << " Replan " << count++ << "\033[0m" << endl;
    // cout.precision(3);
    // cout << "start: " << start_pt.transpose() << " || " << start_vel.transpose() << " final_goal:" << final_goal.transpose() << endl;
    // cout << "init type: " << flag_use_last_optimial << " " << flag_random_init << " touch goal: " << touch_goal << endl;
    cout << "flag_use_last_optimial=" << flag_use_last_optimial << " flag_random_init=" << flag_random_init
         << " pathes->size()=" << (pathes ? (int)pathes->size() : -1) << " touch_goal=" << touch_goal
         << " continous_failures_count_=" << continous_failures_count_ << endl;
    if ((start_pt - final_goal).norm() < 0.01) // Do not return!
      ROS_WARN("The given goal is close to me.");

    /*** STEP 1: INIT ***/
    // densityEval(start_pt, local_target_pt);
    // double traj_dura = computeInitDuration(start_pt, start_vel, local_target_pt, local_target_vel);
    // computePlanningHorizon(pp_.max_vel_);
    // computeMINCOParams(pp_.planning_horizen_, pp_.max_vel_);

    poly_traj::MinJerkOpt initMJO;
    if (!computeInitState(start_pt, start_vel, start_acc, glb_start_pt, final_goal,
                          flag_use_last_optimial, flag_random_init, pathes, initMJO, touch_goal))
    {
      continous_failures_count_++;
      failure_cnt_++;
      return PLAN_RET::INIT_FAIL;
    }

    poly_traj_opt_->setIfTouchGoal(touch_goal);
    poly_traj_opt_->setMaxVelAcc(pp_.max_vel_, pp_.max_acc_);
    poly_traj_opt_->setPlanParametersCopy(pp_);

    vector<std::pair<int, int>> segments;
    vector<vector<Eigen::Vector3d>> AstarPathes;
    PolyTrajOptimizer::CHK_RET ret =
        poly_traj_opt_->finelyCheckAndSetConstraintPoints(segments, AstarPathes, initMJO, poly_traj_opt_->get_cps_num_prePiece_(), true);
    if (ret == PolyTrajOptimizer::CHK_RET::ERR || ret == PolyTrajOptimizer::CHK_RET::TIME_LIM)
    {
      continous_failures_count_++;
      failure_cnt_++;
      return PLAN_RET::INIT_FAIL;
    }

    t_init = ros::Time::now() - t_start;

    Eigen::MatrixXd cstr_pts = initMJO.getInitConstraintPoints(poly_traj_opt_->get_cps_num_prePiece_());
    // visualization
    std::vector<Eigen::Vector3d> point_set;
    for (int i = 0; i < cstr_pts.cols(); ++i)
      point_set.push_back(cstr_pts.col(i));
    visualization_->displayInitPathList(point_set, 0.2, 0);

    t_start = ros::Time::now();

    /*** STEP 2: OPTIMIZE ***/
    bool flag_success = false;
    vector<vector<Eigen::Vector3d>> vis_trajs;
    poly_traj::MinJerkOpt best_MJO;

    if (pp_.use_multitopology_trajs)
    {
      std::vector<ConstraintPoints> trajs = poly_traj_opt_->distinctiveTrajs(segments);
      Eigen::VectorXi success = Eigen::VectorXi::Zero(trajs.size());
      int selected_id = 0;
      poly_traj::Trajectory initTraj = initMJO.getTraj();
      int PN = initTraj.getPieceNum();
      Eigen::MatrixXd all_pos = initTraj.getPositions();
      Eigen::MatrixXd innerPts = all_pos.block(0, 1, 3, PN - 1);
      Eigen::Matrix<double, 3, 4> headState;
      Eigen::Matrix<double, 3, 3> tailState;
      headState << initTraj.getJuncPos(0), initTraj.getJuncVel(0), initTraj.getJuncAcc(0), start_jerk;
      tailState << initTraj.getJuncPos(PN), initTraj.getJuncVel(PN), initTraj.getJuncAcc(PN);
      double final_cost, min_cost = std::numeric_limits<double>::max();

      for (int i = trajs.size() - 1; i >= 0; i--)
      {
        poly_traj_opt_->setConstraintPoints(trajs[i]);
        poly_traj_opt_->setUseMultitopologyTrajs(true);
        if (poly_traj_opt_->optimizeTrajectory(headState, tailState,
                                               innerPts, initTraj.getDurations(), final_cost))
        {
          success[i] = true;

          if (final_cost < min_cost)
          {
            selected_id = i;
            min_cost = final_cost;
            best_MJO = poly_traj_opt_->getMinJerkOpt();
            flag_success = true;
          }

          // visualization
          Eigen::MatrixXd ctrl_pts_temp = poly_traj_opt_->getMinJerkOpt().getInitConstraintPoints(poly_traj_opt_->get_cps_num_prePiece_());
          std::vector<Eigen::Vector3d> point_set;
          for (int j = 0; j < ctrl_pts_temp.cols(); j++)
          {
            point_set.push_back(ctrl_pts_temp.col(j));
          }
          vis_trajs.push_back(point_set);
        }
      }

      t_opt = ros::Time::now() - t_start;

      if (trajs.size() > 1)
      {
        cout << "\033[1;33m"
             << "multi-trajs=" << trajs.size() << ",\033[1;0m"
             << " Success:fail=" << success.sum() << ":" << success.size() - success.sum()
             << " selected:" << selected_id << endl;
      }

      visualization_->displayMultiOptimalPathList(vis_trajs, 0.1); // This visuallization will take up several milliseconds.
    }
    else
    {
      poly_traj::Trajectory initTraj = initMJO.getTraj();
      int PN = initTraj.getPieceNum();
      Eigen::MatrixXd all_pos = initTraj.getPositions();
      Eigen::MatrixXd innerPts = all_pos.block(0, 1, 3, PN - 1);
      Eigen::Matrix<double, 3, 4> headState;
      Eigen::Matrix<double, 3, 3> tailState;
      headState << initTraj.getJuncPos(0), initTraj.getJuncVel(0), initTraj.getJuncAcc(0), start_jerk;
      tailState << initTraj.getJuncPos(PN), initTraj.getJuncVel(PN), initTraj.getJuncAcc(PN);
      double final_cost;
      flag_success = poly_traj_opt_->optimizeTrajectory(headState, tailState,
                                                        innerPts, initTraj.getDurations(), final_cost);

      // flag_success = poly_traj_opt_->optimizeTrajectoryShapeOnly(headState, tailState,
      //                                                            innerPts, initTraj.getDurations(), final_cost);
      best_MJO = poly_traj_opt_->getMinJerkOpt();

      t_opt = ros::Time::now() - t_start;
    }

    /*** STEP 3: Store and display results ***/
    if (flag_success)
    {
      pp_.emergency_ = false;
      continous_failures_count_ = 0;
      success_cnt_++;
      sum_success_time_ += (t_init + t_opt).toSec();
      printf("Success(%.2f%%)=Yes. Time:\033[42m%.3fms,\033[0m init:%.3fms, optimize:%.3fms, avg=%.3fms\n",
             (double)success_cnt_ / (success_cnt_ + failure_cnt_) * 100,
             (t_init + t_opt).toSec() * 1000, t_init.toSec() * 1000, t_opt.toSec() * 1000, sum_success_time_ / success_cnt_ * 1000);
      // cout << "total time:\033[42m" << (t_init + t_opt).toSec()
      //      << "\033[0m,init:" << t_init.toSec()
      //      << ",optimize:" << t_opt.toSec()
      //      << ",avg_time=" << sum_success_time_ / success_cnt_ << endl;

      setLocalTrajFromOpt(best_MJO, touch_goal, true);
      cstr_pts = best_MJO.getInitConstraintPoints(poly_traj_opt_->get_cps_num_prePiece_());
      visualization_->displayOptimalList(cstr_pts, 0);

      // auto traj = best_MJO.getTraj();
      // cout << "================= vaj ==================" << endl;
      // double step = traj.getDurations().sum() / 50;
      // for (double t = 0; t < traj.getDurations().sum(); t += step)
      // {
      //   cout << traj.getVel(t).norm() << " " << traj.getAcc(t).norm() << " " << traj.getJer(t).norm() << endl;
      // }

      // double xxx = 0;
      // TicToc t0;
      // for (double t = 0; t < traj.getDurations().sum(); t += step)
      // {
      //   xxx += traj.getVel(t).norm();
      // }
      // t0.toc();
      // cout << "Xxx=" << xxx <<  endl;
    }
    else
    {
      continous_failures_count_++;
      // if (continous_failures_count_ == 1)
      //   failure_cnt_++; // We only record the first failure after a success.
      failure_cnt_++;

      printf("Success(%.2f%%)=No. Time:\033[41m%.3fms\033[0m\n",
             (double)success_cnt_ / (success_cnt_ + failure_cnt_) * 100, (t_init + t_opt).toSec() * 1000);
      cstr_pts = poly_traj_opt_->getMinJerkOpt().getInitConstraintPoints(poly_traj_opt_->get_cps_num_prePiece_());
      visualization_->displayFailedList(cstr_pts, 0);
    }

    if (flag_success)
      return PLAN_RET::SUCCESS;
    else
      return PLAN_RET::DEFAULT_FAIL;
  }

  double EGOPlannerManager::computeInitDuration(
      const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel,
      const Eigen::Vector3d &local_target_pt, const Eigen::Vector3d &local_target_vel)
  {
    double dist = (local_target_pt - start_pt).norm();
    double full_acc_dec_dist = abs((pp_.max_vel_ * pp_.max_vel_ - start_vel.squaredNorm()) / (2 * pp_.max_acc_)) + abs((pp_.max_vel_ * pp_.max_vel_ - local_target_vel.squaredNorm()) / (2 * pp_.max_acc_));
    double total_time;

    if (dist >= full_acc_dec_dist)
    {
      double uniform_motion_time = (dist - full_acc_dec_dist) / pp_.max_vel_;
      double acc_dec_time = abs((pp_.max_vel_ - start_vel.norm()) / pp_.max_acc_) + abs((pp_.max_vel_ - local_target_vel.norm()) / pp_.max_acc_);
      total_time = uniform_motion_time + acc_dec_time;
    }
    else
    {
      total_time = abs((pp_.max_vel_ - start_vel.norm()) / pp_.max_acc_) + abs((pp_.max_vel_ - local_target_vel.norm()) / pp_.max_acc_); // over estimate the time required
      // if (total_time < dist / pp_.max_vel_)
      // {
      //   ROS_ERROR("total_time < dist / pp_.max_vel_");
      //   cout << "total_time=" << total_time << " dist=" << dist << " pp_.max_vel_=" << pp_.max_vel_ << " pp_.max_acc_=" << pp_.max_acc_ << endl;
      // }
    }
    if (total_time < 0.1)
      ROS_WARN("total_time=%f is too small!", total_time);

    total_time *= 1.2;

    return max(min(total_time * 2, traj_.local_traj.duration), total_time);
  }

  bool EGOPlannerManager::computeInitState(
      const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
      const Eigen::Vector3d &glb_start_pt, const Eigen::Vector3d &final_goal,
      const bool flag_use_last_optimial, const bool flag_random_init, vector<DensityEvalRayData> *pathes,
      poly_traj::MinJerkOpt &initMJO, bool &touch_goal)
  {

    const int TrialTimesLim = 3;
    const int MinPieceNum = 3; // One of the Piece (i.e. an innerPts) will be given manually within the first segment, so MinPieceNum always minus 1
    touch_goal = false;
    vector<Eigen::Vector3d> trajPtVec;

    if (flag_use_last_optimial) /*** case 1: initialize from previous optimal trajectory ***/
    {
      double passed_t_on_lctraj = ros::Time::now().toSec() - traj_.local_traj.start_time;
      double t_to_lc_end = traj_.local_traj.duration - passed_t_on_lctraj;
      if (t_to_lc_end < 0)
      {
        ROS_ERROR("passed_t_on_lctraj=%f, t_to_lc_end=%f, exit and wait for another call.", passed_t_on_lctraj, t_to_lc_end);
        return false;
      }

      int trialtimes = 0;
      double piece_len = min(pp_.polyTraj_piece_length, pp_.planning_horizen_ / (MinPieceNum - 1));
      while (true)
      {
        double t_step = traj_.local_traj.duration / 200, total_len = 0;
        if (unlikely(t_step < 1e-6))
        {
          ROS_ERROR("Error 566! t_step=%f, traj_.local_traj.duration=%f", t_step, traj_.local_traj.duration);
          return false;
        }
        Eigen::Vector3d prev_chk_pos = start_pt;
        trajPtVec.clear();
        trajPtVec.push_back(start_pt);
        for (double t = passed_t_on_lctraj + t_step; total_len < pp_.planning_horizen_; t += t_step)
        {
          if (t >= traj_.local_traj.duration)
            break;

          Eigen::Vector3d pos = traj_.local_traj.traj.getPos(t);
          if ((pos - trajPtVec.back()).norm() >= piece_len && (prev_chk_pos - trajPtVec.back()).norm() < piece_len)
          {
            total_len += (pos - trajPtVec.back()).norm();
            trajPtVec.push_back(pos);
          }
          prev_chk_pos = pos;
        }

        while (total_len < pp_.planning_horizen_)
        {
          const double MergeRatio = 0.333;
          Eigen::Vector3d lc_dir = (final_goal - trajPtVec.back()).normalized();
          if ((final_goal - trajPtVec.back()).norm() > piece_len * (1 + MergeRatio)) // to make sure final_goal have enough distance to trajPtVec.back()
          {
            Eigen::Vector3d pos = trajPtVec.back() + lc_dir * piece_len;
            total_len += (pos - trajPtVec.back()).norm();
            trajPtVec.push_back(pos);
          }
          else
          {
            touch_goal = true;
            Eigen::Vector3d pos = final_goal;
            total_len += (pos - trajPtVec.back()).norm();
            trajPtVec.push_back(pos);
            break;
          }
        }

        if (trajPtVec.size() >= (MinPieceNum - 1) - 1)
          break;
        else
          piece_len /= 1.5;

        if (++trialtimes > TrialTimesLim)
        {
          ROS_ERROR("last optimal: trajPtVec.size()=%ld", trajPtVec.size());
          return false;
        }
      }
    }
    else /*** case 2: path initialization ***/
    {
      // step1: get the rough path with only some nodes
      vector<Eigen::Vector3d> best_path;
      best_path.push_back(start_pt);
      if ((pathes && !pathes->empty()) && !flag_random_init)
      {
        // for (auto path : *pathes)
        //   cout << "score=" << path.score << " mid_p=" << path.mid_p.transpose() << " end_p=" << path.end_p.transpose() << " safe_margin=" << path.safe_margin << " norm_devi=" << path.norm_devi << " safe_l=" << path.safe_l << endl;
        // cout << "score=";
        // for (auto path : *pathes)
        //   cout << " " << path.score;
        // cout << endl;

        // get the best ray
        int best_path_id = 0, path_id = 0;
        double best_score = -std::numeric_limits<double>::max();
        while (path_id < (int)pathes->size())
        {
          if (pathes->at(path_id).score > best_score)
          {
            best_path_id = path_id;
            best_score = pathes->at(path_id).score;
          }
          path_id++;
        }
        DensityEvalRayData best = pathes->at(best_path_id);
        pathes->erase(pathes->begin() + best_path_id);
        if ((best.start_p - start_pt).norm() > 2.0) // start_pt will be used in best_path because best.start_p may become slightly outdated as drone moves
        {
          ROS_ERROR("best_path outdated! [best.start_p(%f %f %f) - start_pt(%f %f %f)].norm() > 2.0m",
                    best.start_p(0), best.start_p(1), best.start_p(2), start_pt(0), start_pt(1), start_pt(2));

          return false;
        }

        // get the best path segment
        if (best.safe_l > 1e-5)
        {
          if (best.safe_l <= (best.mid_p - best.start_p).norm())
          {
            best_path.push_back(best.start_p + (best.mid_p - best.start_p).normalized() * best.safe_l);
          }
          else
          {
            best_path.push_back(best.mid_p);
            best_path.push_back(best.end_p); // end_p is the farthest safe point
          }
        }
        else
        {
          /*The drone gets too close to obstacles. I have no choise but to add randomness.*/
          ROS_WARN_THROTTLE(1.0, "[computeInitState]map_Big_->getDistance(best.start_p)=%f", map_->big_->getDistance(best.start_p));
          best_path.push_back(GenRandomMidPt(start_pt, final_goal));
        }

        if (best_path.size() >= 2)
        {
          Eigen::Vector3d glb_dir = (final_goal - start_pt).normalized();
          double glb_dist = (final_goal - start_pt).norm();
          Eigen::Vector3d glb_2_3_pt = start_pt + (glb_dist * 2.0 / 3.0) * glb_dir;
          for (int chk_id = 1; chk_id < (int)best_path.size(); ++chk_id)
          {
            if (((best_path[chk_id] - best_path[0]).dot(glb_dir)) > (glb_dist * 2.0 / 3.0)) // best_path[0] == start_pt
            {
              Eigen::Vector3d intersection_point =
                  best_path[chk_id - 1] +
                  ((best_path[chk_id] - best_path[chk_id - 1]) *
                   (glb_dir.dot(glb_2_3_pt - best_path[chk_id - 1]) / glb_dir.dot(best_path[chk_id] - best_path[chk_id - 1])));
              cout << "start_pt=" << start_pt.transpose()
                   << " final_goal=" << final_goal.transpose()
                   << " chk_id=" << chk_id
                   << " best_path[chk_id]=" << best_path[chk_id].transpose()
                   << " best_path[chk_id - 1]=" << best_path[chk_id - 1].transpose()
                   << " intersection_point=" << intersection_point.transpose() << endl;
              best_path[chk_id] = (intersection_point);
              best_path.erase(best_path.begin() + chk_id + 1, best_path.end());
              break;
            }
          }
        }
      }
      else
      {
        /* pathes->empty() means the drone gets too close to the final goal.*/
        best_path.push_back(GenRandomMidPt(start_pt, final_goal));
      }
      best_path.push_back(final_goal);

      // step2: uniformly sample trajPtVec on the rough path
      int trialtimes = 0;
      double piece_len = min(pp_.polyTraj_piece_length,
                             min(pp_.planning_horizen_, (best_path.front() - best_path.back()).norm()) /
                                 (MinPieceNum - 1));
      while (true)
      {
        double total_len = 0;
        // Eigen::Vector3d prev_chk_pos = start_pt;
        trajPtVec.clear();
        trajPtVec.push_back(best_path[0]);

        for (int path_id = 0; path_id < (int)best_path.size() - 1; ++path_id)
        {
          Eigen::Vector3d seg_start = trajPtVec.back();
          Eigen::Vector3d seg_dir = (best_path[path_id + 1] - seg_start).normalized();
          double seg_len = (best_path[path_id + 1] - seg_start).norm();
          bool found_local_end = false;
          for (double cur_len = piece_len; cur_len < seg_len; cur_len += piece_len)
          {
            Eigen::Vector3d pos = seg_start + seg_dir * cur_len;
            total_len += piece_len;
            trajPtVec.push_back(pos);
            if (total_len + piece_len > pp_.planning_horizen_)
            {
              found_local_end = true;
              break;
            }
          }

          if (!found_local_end) // path_id always < best_path.size() - 1
          {
            if (path_id + 2 < (int)best_path.size()) // there are unused best_path segemnts left
            {
              Eigen::Vector3d next_dir = (best_path[path_id + 2] - best_path[path_id + 1]).normalized();
              double dist_p2line = ((trajPtVec.back() - best_path[path_id + 1]).cross(next_dir)).norm() / next_dir.norm();
              Eigen::Vector3d foot_p = best_path[path_id + 1] + (trajPtVec.back() - best_path[path_id + 1]).dot(next_dir) * next_dir;
              Eigen::Vector3d next_pt = foot_p + sqrt(piece_len * piece_len - dist_p2line * dist_p2line) * next_dir;
              if ((next_pt - best_path[path_id + 1]).norm() <= (best_path[path_id + 2] - best_path[path_id + 1]).norm())
              {
                total_len += piece_len;
                trajPtVec.push_back(next_pt);
              }
              else
              {
                total_len += (best_path[path_id + 2] - trajPtVec.back()).norm();
                trajPtVec.push_back(best_path[path_id + 2]);
                if (path_id + 2 == (int)best_path.size() - 1) // the last segemt. (why needs this check: length of segment path_id+1 to path_id+2 may be shorter than piece_len)
                {
                  touch_goal = true;
                  break;
                }
              }
            }
            else
            {
              if ((best_path[path_id + 1] - trajPtVec.back()).norm() < 0.001) // 1mm (sometimes "next_pt" generates extermrly close points to best_path[path_id + 1])
              {
                total_len -= (trajPtVec[trajPtVec.size() - 1] - trajPtVec[trajPtVec.size() - 2]).norm();
                trajPtVec.pop_back();
              }
              total_len += (best_path[path_id + 1] - trajPtVec.back()).norm();
              trajPtVec.push_back(best_path[path_id + 1]);
              touch_goal = true;
              break;
              // the for loop will break
            }
          }
          else
            break;
        }

        if (trajPtVec.size() >= (MinPieceNum - 1) - 1)
          break;
        else
          piece_len /= 1.5;

        if (++trialtimes > TrialTimesLim)
        {
          ROS_ERROR("last optimal: trajPtVec.size()=%ld", trajPtVec.size());
          return false;
        }
      }
      // {
      //   ROS_ERROR("pathes->empty()=true");
      //   return false;
      // }
    }

    // step1: to make sure the last point is collision-free
    trajPtVec.insert(trajPtVec.begin() + 1, (2.0 * trajPtVec[0] + trajPtVec[1]) / 3.0); // the trajectory always requires more freedom at first in dense envs.
    if (map_->getOcc(trajPtVec.front()) > 0)
    {
      Eigen::Vector3d safe_pt;
      if (getNearbySafePt(trajPtVec.front(), 2, safe_pt))
      {
        ROS_WARN("Adjust start point by %fm", (safe_pt - trajPtVec.front()).norm());
        trajPtVec.front() = safe_pt;
      }
      else
        ROS_WARN("Can't find any safe point near the init path head.");
    }
    if (map_->getOcc(trajPtVec.back()) > 0)
    {
      Eigen::Vector3d safe_pt;
      if (getNearbySafePt(trajPtVec.back(), 4, safe_pt))
      {
        trajPtVec.back() = safe_pt;
      }
      else
        ROS_WARN("Can't find any safe point near the init path end.");
    }

    // step2: compute inner points
    int piece_num = trajPtVec.size() - 1;
    Eigen::Matrix3d headState, tailState;
    Eigen::MatrixXd innerPs(3, piece_num - 1);
    for (int i = 0; i < piece_num - 1; ++i)
      innerPs.col(i) = trajPtVec[i + 1];

    // step3: compute time allocation as accurate as possible
    double traj_dura = computeInitDuration(trajPtVec.front(), start_vel, trajPtVec.back(), Eigen::Vector3d::Zero());
    Eigen::Vector3d tailVel = touch_goal ? Eigen::Vector3d::Zero() : Eigen::Vector3d((final_goal - trajPtVec.back()).normalized() * (0.7 * pp_.max_vel_));
    auto traj = OnePieceTrajGen(trajPtVec.front(), start_vel, start_acc, trajPtVec.back(), tailVel, Eigen::Vector3d::Zero(), traj_dura);
    double total_len = 0, cur_len = 0;
    for (size_t i = 1; i < trajPtVec.size(); i++)
      total_len += (trajPtVec[i] - trajPtVec[i - 1]).norm();
    Eigen::VectorXd len_percent(piece_num);
    for (size_t i = 1; i < trajPtVec.size(); i++)
    {
      cur_len += (trajPtVec[i] - trajPtVec[i - 1]).norm();
      len_percent(i - 1) = cur_len / total_len;
    }
    double t_step = traj_dura / 100;
    total_len = 0;
    Eigen::Vector3d last_p = traj.getPos(0);
    for (double t = t_step; t <= traj_dura; t += t_step)
    {
      Eigen::Vector3d p = traj.getPos(t);
      total_len += (p - last_p).norm();
      last_p = p;
    }
    int piece_id = 0;
    last_p = traj.getPos(0);
    cur_len = 0;
    double last_percent = 0;
    Eigen::VectorXd piece_dur_vec_tmp(piece_num);
    for (double t = t_step;; t += t_step)
    {
      Eigen::Vector3d p = traj.getPos(t);
      cur_len += (p - last_p).norm();
      double percent = cur_len / total_len;
      while (percent >= len_percent(piece_id))
      {
        piece_dur_vec_tmp(piece_id) = ((percent - len_percent(piece_id)) * (t - t_step) + (len_percent(piece_id) - last_percent) * t) / (percent - last_percent);
        if (++piece_id >= len_percent.size())
          goto out_loop1;
      }
      last_p = p;
      last_percent = percent;
    }
  out_loop1:;
    Eigen::VectorXd piece_dur_vec = piece_dur_vec_tmp;
    for (int i = 1; i < piece_dur_vec.size(); ++i)
      piece_dur_vec(i) = piece_dur_vec_tmp(i) - piece_dur_vec_tmp(i - 1);

    // step4: generate init traj
    headState << trajPtVec.front(), start_vel, start_acc;
    tailState << trajPtVec.back(), tailVel, Eigen::Vector3d::Zero();
    initMJO.reset(headState, tailState, piece_num);
    initMJO.generate(innerPs, piece_dur_vec);

    printf("start pv: [%.2f, %.2f, %.2f] [%.2f, %.2f, %.2f]\n", start_pt(0), start_pt(1), start_pt(2), start_vel(0), start_vel(1), start_vel(2));
    printf("end   pv: [%.2f, %.2f, %.2f] [%.2f, %.2f, %.2f]\n", tailState(0, 0), tailState(1, 0), tailState(2, 0), tailState(0, 1), tailState(1, 1), tailState(2, 1));

    return true;
  }

  Eigen::Vector3d EGOPlannerManager::GenRandomMidPt(const Eigen::Vector3d start_pt, const Eigen::Vector3d end_pt)
  {
    Eigen::Vector3d glb_dir = (end_pt - start_pt).normalized();
    Eigen::Vector3d horizen_dir = ((start_pt - end_pt).cross(Eigen::Vector3d(0, 0, 1))).normalized();
    Eigen::Vector3d vertical_dir = ((start_pt - end_pt).cross(horizen_dir)).normalized();
    double path_total_len = min(pp_.planning_horizen_, (end_pt - start_pt).norm());
    Eigen::Vector3d random_mid_pt =
        (start_pt + glb_dir * path_total_len / 2) +
        (((double)rand()) / RAND_MAX - 0.5) *
            path_total_len * horizen_dir * 0.8 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989) +
        (((double)rand()) / RAND_MAX - 0.5) *
            path_total_len * vertical_dir * 0.4 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989);

    return random_mid_pt;
  }

  bool EGOPlannerManager::getNearbySafePt(const Eigen::Vector3d unsafe_pt, const int max_grid, Eigen::Vector3d &safe_pt)
  {
    TicToc t0;
    const double reso = map_->getReso();
    for (int expand = 1; expand <= max_grid; ++expand)
    {
      for (int x = -expand; x <= expand; x++)
        for (int y = -expand; y <= expand; y++)
          for (int z = -expand; z <= expand; z++)
          {
            Eigen::Vector3d test_pt = unsafe_pt + Eigen::Vector3d(x, y, z) * reso;
            if (map_->getOcc(test_pt) <= 0)
            {
              safe_pt = test_pt;
              return true;
            }
          }
    }

    t0.toc();

    return false;
  }

  bool EGOPlannerManager::computePlanningParams(const double vel)
  {
    /* planning_horizen_ */
    const double PlanningTime = 3.0; // s
    const double HorizonMin = 6.0;   // m
    pp_.planning_horizen_ = max(PlanningTime * vel, HorizonMin);

    // cout << "pp_.max_vel_=" << pp_.max_vel_ << " pp_.planning_horizen_=" << pp_.planning_horizen_ << endl;

    /* polyTraj_piece_length and CPsNumPerPiece */
    if (pp_.planning_horizen_ <= 9)
    {
      pp_.polyTraj_piece_length = pp_.planning_horizen_ / 6;
      poly_traj_opt_->setCPsNumPerPiece(5);
    }
    else if (pp_.planning_horizen_ <= 15)
    {
      pp_.polyTraj_piece_length = pp_.planning_horizen_ / 6;
      poly_traj_opt_->setCPsNumPerPiece(7);
    }
    else if (pp_.planning_horizen_ <= 22)
    {
      pp_.polyTraj_piece_length = pp_.planning_horizen_ / 6;
      poly_traj_opt_->setCPsNumPerPiece(9);
    }
    else if (pp_.planning_horizen_ <= 31)
    {
      pp_.polyTraj_piece_length = pp_.planning_horizen_ / 6;
      poly_traj_opt_->setCPsNumPerPiece(10);
    }
    else
    {
      ROS_ERROR("planning_horizon=%f is too long!!!", pp_.planning_horizen_);
      pp_.polyTraj_piece_length = pp_.planning_horizen_ / 6;
      poly_traj_opt_->setCPsNumPerPiece(10);
    }

    /* set the map to use */
    if (pp_.planning_horizen_ * 2 / 3 < map_->sml_->getLocalMapRange().maxCoeff())
    {
      pp_.speed_mode = PlanParameters::MODE::SLOW;
      map_->setMapUse(MapManager::MAP_USE::SMALL);
    }
    else
    {
      pp_.speed_mode = PlanParameters::MODE::FAST;
      map_->setMapUse(MapManager::MAP_USE::LARGE);
    }

    return true;
  }

  bool EGOPlannerManager::computePlanningHorizon(const double vel)
  {
    const double PlanningTime = 3.0; // s
    const double HorizonMin = 6.0;   // m
    pp_.planning_horizen_ = max(PlanningTime * vel, HorizonMin);

    cout << "pp_.max_vel_=" << pp_.max_vel_ << " pp_.planning_horizen_=" << pp_.planning_horizen_ << endl;
    return true;
  }

  bool EGOPlannerManager::computeMINCOParams(const double planning_horizon, const double vel)
  {
    // const int PrefPieceNum = 6; // Preferred piece number (5 waypoints)
    // const int MaxPieceNum = 10;
    // const double PrefCPsDist = 0.25; // Preferred constraint points spacing
    // const double MaxCPsDist = 0.5;
    // const double MinPieceLen = 1.0; // m

    if (planning_horizon <= 9)
    {
      pp_.polyTraj_piece_length = planning_horizon / 6;
      poly_traj_opt_->setCPsNumPerPiece(5);
    }
    else if (planning_horizon <= 15)
    {
      pp_.polyTraj_piece_length = planning_horizon / 7;
      poly_traj_opt_->setCPsNumPerPiece(7);
    }
    else if (planning_horizon <= 22)
    {
      pp_.polyTraj_piece_length = planning_horizon / 8;
      poly_traj_opt_->setCPsNumPerPiece(8);
    }
    else if (planning_horizon <= 31)
    {
      pp_.polyTraj_piece_length = planning_horizon / 10;
      poly_traj_opt_->setCPsNumPerPiece(8);
    }
    else
    {
      ROS_ERROR("planning_horizon=%f is too long!!!", planning_horizon);
      pp_.polyTraj_piece_length = planning_horizon / 10;
      poly_traj_opt_->setCPsNumPerPiece(8);
    }

    return true;
  }

  bool EGOPlannerManager::densityEval(const Eigen::Vector3d start_pt, const Eigen::Vector3d final_goal,
                                      DensityEvalRayData *best_ray /*= NULL*/, vector<DensityEvalRayData> *all_rays /*= NULL*/) const
  {

    if ((start_pt - final_goal).norm() < 1.0)
    {
      // ROS_INFO("No need to evaluate density when ((start_pt - final_goal).norm() < 1.0).");
      return false;
    }

    // ros::Time t0 = ros::Time::now();

    const double HORIZON = 30.0;
    double horizon = min((start_pt - final_goal).norm(), HORIZON);
    const double MajorAxis = 4.0;
    const double MinorAxis = 1.5;
    const double MidPlaneSTEP = 0.5;
    const double MidPlanePosRatio = 1.0 / 3.0;
    const double SafeMarginCutoff = 3.0;

    vector<DensityEvalRayData> rays;
    const auto dir = (final_goal - start_pt).normalized();
    Eigen::Vector3d u_dir = dir.cross(Eigen::Vector3d(0, 0, 1.0)).normalized();
    Eigen::Vector3d v_dir = u_dir.cross(dir).normalized();
    Eigen::Vector3d mid_p_cen = start_pt + horizon * MidPlanePosRatio * dir;
    for (double u = -MajorAxis; u < MajorAxis; u += MidPlaneSTEP)
      for (double v = -MinorAxis; v < MinorAxis; v += MidPlaneSTEP)
      {
        double devi2 = (u / MajorAxis) * (u / MajorAxis) + (v / MinorAxis) * (v / MinorAxis);
        if (devi2 <= 1.001)
        {
          DensityEvalRayData rd;
          rd.start_p = start_pt;
          rd.mid_p = mid_p_cen + u_dir * u + v_dir * v;
          rd.norm_devi = sqrt(devi2);
          rays.push_back(rd);
        }
      }

    const double reso = map_->big_->getResolution();
    for (auto &ray : rays)
    {
      ray.safe_margin = 1000; // can not be std::numeric_limits<double>::max() !!!
      auto mpt_dir = (ray.mid_p - start_pt).normalized();
      const double phase1_l = (ray.mid_p - start_pt).norm();
      const double phase2_l = phase1_l + horizon * MidPlanePosRatio;
      double l = 0.0;
      do
      {
        Eigen::Vector3d pt;
        if (l < phase1_l)
          pt = start_pt + l * mpt_dir;
        else if (l < phase2_l)
          pt = ray.mid_p + (l - phase1_l) * dir;
        else
        {
          ray.safe_l = phase2_l;
          ray.end_p = ray.mid_p + (phase2_l - phase1_l) * dir;
          ray.safe_margin = min(ray.safe_margin, map_->big_->getDistance(ray.end_p));
          break;
        }
        double dist = map_->big_->getDistance(pt); // if use getDistancePersii, then safe_margin out of the map will be zero.
        // cout << "dist=" << dist << endl;
        if (dist > GRID_MAP_UNKNOWN_ESDF_FLAG_COMP) // goes out of the map, no need to check more in optimistic map.
        {
          // cout << "phase2_l=" << phase2_l << endl;
          ray.safe_l = phase2_l;
          ray.end_p = ray.mid_p + (phase2_l - phase1_l) * dir;
          break;
        }
        // cout << "l=" << l << endl;
        ray.safe_l = l;
        ray.end_p = pt;
        if (dist > 1e-5) // otherwise safe_margin will always < 0
          ray.safe_margin = min(ray.safe_margin, dist);
        else
          break;

        l += max(dist, reso);
      } while (true);

      ray.safe = l >= phase2_l;
      ray.score = HORIZON * ray.safe +                                  /*HORIZON is the weight here*/
                  (3.0) * (HORIZON - (final_goal - ray.end_p).norm()) + /*HORIZON is to make it positive*/
                  (10.0) * min(ray.safe_margin, SafeMarginCutoff) +
                  (8.0) * (1 - ray.norm_devi); /* 1 is to make it positive */

      // cout << "score=" << ray.score << " A=" << (HORIZON - (final_goal - ray.end_p).norm()) << " B=" << ray.safe_margin << " C=" << (1 - ray.norm_devi) << endl;

      if (best_ray && best_ray->score < ray.score)
        *best_ray = ray;
    }

    if (best_ray)
    {

      // cout << "A=" << (HORIZON - (final_goal - best_ray->end_p).norm()) << " B=" << best_ray->safe_margin << " C=" << (1 - best_ray->norm_devi) << endl;

      // if (best_ray->safe_l < 1e-5) // debug
      // {
      //   ROS_ERROR("best_ray->safe_l < 1e-5");
      //   while (ros::ok())
      //     ;
      // }

      if (best_ray->score < -1e100) // not assgned
      {
        ROS_ERROR("no valid ray detected!");
        return false;
      }

      best_ray->start_p = start_pt;

      // step3: determine the real minimum clearance
      const double CHECK_RADIUS = 0.5;
      const double trust_region_l = min(max(3.0 * pp_.max_vel_, 4.0), HORIZON); // params: 3.0s and 4.0m
      // cout << "trust_region_l=" << trust_region_l << endl;
      auto mpt_dir = (best_ray->mid_p - start_pt).normalized();
      const double phase1_l = (best_ray->mid_p - start_pt).norm();
      const auto phase1_dir = (best_ray->mid_p - start_pt).normalized();
      if (phase1_dir.squaredNorm() < 0.5) // debug
        ROS_ERROR("phase1_dir.squaredNorm()=%f, best_ray->mid_p=(%f, %f, %f), start_pt=(%f, %f, %f)",
                  phase1_dir.squaredNorm(), best_ray->mid_p(0), best_ray->mid_p(1), best_ray->mid_p(2), start_pt(0), start_pt(1), start_pt(2));
      const double phase2_l = phase1_l + horizon * MidPlanePosRatio;
      const auto phase2_dir = dir;
      if (phase2_dir.squaredNorm() < 0.5) // debug
        ROS_ERROR("phase2_dir.squaredNorm()=%f, final_goal=(%f, %f, %f), start_pt=(%f, %f, %f)",
                  phase2_dir.squaredNorm(), final_goal(0), final_goal(1), final_goal(2), start_pt(0), start_pt(1), start_pt(2));
      double l = 0.0;
      typedef std::pair<std::pair<Eigen::Vector3d, Eigen::Vector3d>, double> PT;
      struct LessDist
      {
        bool operator()(const PT p1, const PT p2) { return p1.second > p2.second; }
      };
      std::priority_queue<PT, std::vector<PT>, LessDist> sample_pts;
      vector<Eigen::Vector3d> init_of_init; // visualizaiton
      do
      {
        Eigen::Vector3d pt;
        Eigen::Vector3d dir_rec;
        if (l < phase1_l)
        {
          pt = start_pt + l * mpt_dir;
          dir_rec = phase1_dir;
        }
        else if (l < phase2_l)
        {
          pt = best_ray->mid_p + (l - phase1_l) * dir;
          dir_rec = phase2_dir;
        }
        else
          break;
        double dist = map_->big_->getDistance(pt);
        if (dist <= 1e-5)
        {
          break;
        }
        sample_pts.push(std::make_pair(std::make_pair(pt, dir_rec), dist));
        init_of_init.push_back(pt); // visualizaiton

        if (l > trust_region_l) // The check is placed here because we accept one sample point exceeding trust_region_l
          break;

        l += dist > GRID_MAP_UNKNOWN_ESDF_FLAG_COMP ? reso : max(dist, reso);
      } while (true);

      // visualizaiton
      // init_of_init.push_back(best_ray->start_p);
      // init_of_init.push_back(best_ray->mid_p);
      // init_of_init.push_back(best_ray->end_p);
      visualization_->displayInitOfInitPathList(init_of_init, 0.1, 0);

      // cout << "l=" << l << " phase1_l=" << phase1_l << " phase2_l=" << phase2_l << " sample_pts.size()=" << sample_pts.size() << endl;

      if (sample_pts.empty())
      {
        ROS_WARN_THROTTLE(1.0, "map_Big_->getDistance(%f, %f, %f) = %f, stuck in obstacles with the first check.",
                  start_pt(0), start_pt(1), start_pt(2), map_->big_->getDistance(start_pt));
        return false;
      }

      bool found_real_min = false;
      do
      {
        int count = 0;
        const auto ptdata = sample_pts.top();
        double dist_tmp, g[3], dist_last = -std::numeric_limits<double>::max();
        Eigen::Map<Eigen::Vector3d> grad(g, 3);
        auto pt = ptdata.first.first, pt_last = pt;
        bool found_tmp_min = false;
        do
        {
          if (map_->big_->evaluateESDFWithGrad(pt, dist_tmp, g))
          {
            if (dist_tmp > dist_last)
            {
              dist_last = dist_tmp;
              pt_last = pt;
              // compute the next step
              Eigen::Vector3d check_dir = ptdata.first.second.cross(grad.normalized().cross(ptdata.first.second)).normalized();
              Eigen::Vector3d g_proj = grad.dot(check_dir) * check_dir;
              if (g_proj.norm() > 1e-3 && count++ < 10)
              {
                double raw_step_len = g_proj.norm();
                pt += raw_step_len < reso * 0.5 ? g_proj : g_proj.normalized() * reso * 0.5;
                if ((pt - ptdata.first.first).norm() > CHECK_RADIUS)
                  found_tmp_min = true;
              }
              else
                found_tmp_min = true;
            }
            else
              found_tmp_min = true;
          }
          else
          {
            found_real_min = true;
            break;
          }

          if (found_tmp_min)
          {
            sample_pts.pop();
            if (dist_last < -10 || dist_last > 100 || pt_last.squaredNorm() > 100000 || abs(ptdata.first.second.squaredNorm() - 1.0) > 1e-3) // debug
              ROS_ERROR("Abnormal values: dist_last=%f, pt_last.squaredNorm()=%f, ptdata.first.second.squaredNorm()=%f", dist_last, pt_last.squaredNorm(), ptdata.first.second.squaredNorm());
            auto ptnew = make_pair(make_pair(pt_last, ptdata.first.second), dist_last);
            sample_pts.push(ptnew);
            if (sample_pts.top().second >= ptnew.second) // no improvement
              found_real_min = true;
          }

        } while (!found_tmp_min);

      } while (!found_real_min);

      best_ray->safe_margin = sample_pts.top().second;
      best_ray->time = ros::Time::now().toSec();

      // cout << "best_ray->safe_margin=" << best_ray->safe_margin << " start_p=" << best_ray->start_p.transpose() << " end_p=" << best_ray->end_p.transpose() << " safe_l=" << best_ray->safe_l << endl;
    }

    if (all_rays)
    {
      all_rays->reserve(rays.size());
      for (auto ray : rays)
        all_rays->push_back(ray);
    }

    // ROS_WARN("preferred_speed=%f", best_ray.preferred_speed);

    // while (!sample_pts.empty())
    // {
    //   cout << "dist=" << sample_pts.top().second << endl;
    //   sample_pts.pop();
    // }

    // ros::Time t1 = ros::Time::now();
    // cout << "time=" << (t1 - t0).toSec() * 1000 << endl;

    // for (auto ray : rays)
    // {
    //   cout << "safe=" << ray.safe << " ray.l=" << ray.safe_l << " safe_margin=" << ray.safe_margin << " score=" << ray.score << " ray=" << ray.mid_p.transpose() << endl;
    // }
    // cout << "beast_ray: safe=" << best_ray.safe << " ray.l=" << best_ray.safe_l << " safe_margin=" << best_ray.safe_margin << " score=" << best_ray.score << " ray=" << best_ray.mid_p.transpose() << endl;

    return true;
  }

  bool EGOPlannerManager::DetVelByDensity(DensityEvalRayData &best_ray)
  {
    TicToc t0;

    const double SAFE_MARGIN = 1.5; // m
    const double SAFE_TIME = 2.0;   // s
    // const double LowHighVelThres = 3.0; // m/s
    const double MaxVel = 10.0; // m/s
    const double MinVel = 1.0;  // m/s
    const int HisDataNum = 20;
    const double UsedPercentage = 0.6;
    const double ChangeTooMuchThres = 0.4; // 40%
    const double DesVelScale = 0.7;

    bool full_speed = false;
    double preferred_speed = 0.0;
    if (best_ray.safe && best_ray.safe_margin > SAFE_MARGIN) // really sparse, fly under maximum speed
    {
      full_speed = true;
      preferred_speed = min(pp_.max_vel_user_, best_ray.safe_l / SAFE_TIME);
    }
    else
    {
      preferred_speed = DesVelScale * ((MaxVel - MinVel) / SAFE_MARGIN * best_ray.safe_margin + MinVel);
      // cout << "best_ray.safe_margin=" << best_ray.safe_margin << " preferred_speed=" << best_ray.preferred_speed << endl;
      preferred_speed = min(preferred_speed, MaxVel);
      preferred_speed = min(preferred_speed, best_ray.safe_l / SAFE_TIME);
    }

    best_ray.full_speed = full_speed;
    best_ray.preferred_speed = preferred_speed;

    his_dendat_.push_back(best_ray);
    his_dendat_.sort();
    while (his_dendat_.size() > HisDataNum)
    {
      double t_oldest = std::numeric_limits<double>::max();
      auto t_oldest_it = his_dendat_.begin();
      for (auto it = his_dendat_.begin(); it != his_dendat_.end(); ++it)
      {
        if (it->time < t_oldest)
        {
          t_oldest = it->time;
          t_oldest_it = it;
        }
      }
      his_dendat_.erase(t_oldest_it);
    }

    int id = 0, start_id = (int)floor(his_dendat_.size() * UsedPercentage / 2), end_id = his_dendat_.size() - start_id, count = 0;
    double speed_sum = 0.0;
    for (auto it = his_dendat_.begin(); id < end_id; ++it, ++id)
    {
      if (id < start_id)
        continue;
      speed_sum += it->preferred_speed;
      count++;
    }
    double avg_preferred_speed = speed_sum / count;
    // ROS_WARN("avg_preferred_speed=%f, count=%d", avg_preferred_speed, count);

    pp_.max_vel_ = min(avg_preferred_speed, pp_.max_vel_user_);
    pp_.max_acc_ = min(2 * avg_preferred_speed, pp_.max_acc_user_);
    if (abs((pp_.max_vel_ - pp_.max_vel_prevplan_) / pp_.max_vel_prevplan_) > ChangeTooMuchThres)
    {
      pp_.max_vel_prevplan_ = pp_.max_vel_;
      pp_.max_acc_prevplan_ = pp_.max_acc_;
      pp_.desvel_changed_toomuch_ = true; // this flag will trigger a replan
    }
    else
      pp_.desvel_changed_toomuch_ = false;

    // t0.toc();

    // cout << "safe_margin=" << best_ray.safe_margin << " safe_l=" << best_ray.safe_l << " preferred_speed=" << best_ray.preferred_speed << " pp_.max_vel_=" << pp_.max_vel_ << endl;
    // for (auto it : his_dendat_)
    //   cout << "it.preferred_speed=" << it.preferred_speed << endl;
    // cout << "=================================" << endl;

    return true;
  }

  bool EGOPlannerManager::setLocalTrajFromOpt(const poly_traj::MinJerkOpt &opt, const bool touch_goal, const bool set_uk_info /* = false */)
  {
    poly_traj::Trajectory traj = opt.getTraj();
    Eigen::VectorXd duras = traj.getDurations();
    int K = getCpsNumPrePiece();
    int end_id = ConstraintPoints::two_thirds_id(duras.size() * K + 1, touch_goal); // do not change to (touch_goal || pp_.speed_mode == PlanParameters::MODE::FAST) instead because end_time is not not used for collision check only.
    int piece_id = end_id / K;
    double end_time = duras.block(0, 0, piece_id, 1).sum() +
                      (piece_id < duras.size() ? duras(piece_id) / K * (end_id % K) : 0.0);
    traj_.setLocalTraj(traj, end_time, ros::Time::now().toSec(), pp_.drone_id, (set_uk_info ? &(poly_traj_opt_->getControlPoints().ent_uk) : NULL));

    return true;
  }

  bool EGOPlannerManager::OnePieceTrajGen(Eigen::Vector3d start_pos, Eigen::Vector3d end_pos)
  {
    auto ZERO = Eigen::Vector3d::Zero();
    Eigen::Matrix<double, 3, 3> headState, tailState;
    headState << start_pos, ZERO, ZERO;
    tailState << end_pos, ZERO, ZERO;
    poly_traj::MinJerkOpt OnePieceMJO;
    OnePieceMJO.reset(headState, tailState, 1);
    Eigen::Matrix3Xd blank;
    Eigen::VectorXd ts(1);
    ts << 1.0;
    OnePieceMJO.generate(blank, ts);

    traj_.setLocalTraj(OnePieceMJO.getTraj(), 1.0, ros::Time::now().toSec(), pp_.drone_id);
    return true;
  }

  poly_traj::Trajectory EGOPlannerManager::OnePieceTrajGen(
      Eigen::Vector3d start_pos, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
      Eigen::Vector3d end_pos, Eigen::Vector3d end_vel, Eigen::Vector3d end_acc, double duration)
  {
    Eigen::Matrix<double, 3, 3> headState, tailState;
    headState << start_pos, start_vel, start_acc;
    tailState << end_pos, end_vel, end_acc;
    poly_traj::MinJerkOpt OnePieceMJO;
    OnePieceMJO.reset(headState, tailState, 1);
    Eigen::Matrix3Xd blank;
    Eigen::VectorXd ts(1);
    ts << duration;
    OnePieceMJO.generate(blank, ts);

    return OnePieceMJO.getTraj();
  }

  bool EGOPlannerManager::EmergencyStop(Eigen::Vector3d stop_pos)
  {
    auto ZERO = Eigen::Vector3d::Zero();
    Eigen::Matrix<double, 3, 3> headState, tailState;
    headState << stop_pos, ZERO, ZERO;
    tailState = headState;
    poly_traj::MinJerkOpt stopMJO;
    stopMJO.reset(headState, tailState, 2);
    stopMJO.generate(stop_pos, Eigen::Vector2d(1.0, 1.0));

    setLocalTrajFromOpt(stopMJO, false, false);

    return true;
  }

  bool EGOPlannerManager::checkCollision(int drone_id)
  {
    if (traj_.local_traj.start_time < 1e9) // It means my first planning has not started
      return false;
    if (traj_.swarm_traj[drone_id].drone_id != drone_id) // The trajectory is invalid
      return false;

    double my_traj_start_time = traj_.local_traj.start_time;
    double other_traj_start_time = traj_.swarm_traj[drone_id].start_time;

    double t_start = max(my_traj_start_time, other_traj_start_time);
    double t_end = min(my_traj_start_time + traj_.local_traj.duration * 2 / 3,
                       other_traj_start_time + traj_.swarm_traj[drone_id].duration);

    for (double t = t_start; t < t_end; t += 0.03)
    {
      if ((traj_.local_traj.traj.getPos(t - my_traj_start_time) -
           traj_.swarm_traj[drone_id].traj.getPos(t - other_traj_start_time))
              .norm() < (getSwarmClearance() + traj_.swarm_traj[drone_id].des_clearance))
      {
        return true;
      }
    }

    return false;
  }

} // namespace ego_planner

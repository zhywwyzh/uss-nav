#include <exploration_manager/frontier_manager.h>
#include <fstream>
#include <memory>
#include <lkh_tsp_solver/lkh_interface.h>


namespace ego_planner {

FrontierManager::FrontierManager(ros::NodeHandle& nh, const MapInterface::Ptr& map, SceneGraph::Ptr scene_graph)
{
  map_   = map;
  scene_graph_ = scene_graph;
  frontier_finder_ = std::make_shared<FrontierFinder>(map, nh, scene_graph);

  Eigen::Vector3d map_box_max, map_box_min;
  map_->getGlobalBox(map_box_min, map_box_max);

  hgrid_ = std::make_shared<HGrid>(map_, nh);
  hgrid_->init(map_box_min, map_box_max);

  frontier_finder_->setHgrid(hgrid_);

  ed_ = make_shared<ExplorationData>();
  // ed_->posegraph_m_.reset(new MultiPoseGraph);
  // ed_->posegraph_m_->main_inx = 0;
  ep_ = std::make_shared<ExplorationParam>();

  ViewNode::a_star_       = std::make_shared<dyn_a_star::AStar>();
  ViewNode::a_star_->initAstar(map->getRawPtr(), Eigen::Vector3i(100, 100, 100));
  ViewNode::map_          = map;
  // ViewNode::posegraph_m_  = ed_->posegraph_m_;
  ViewNode::sensor_range_ = frontier_finder_->percep_utils_->getSensorMaxDist();

  visualization_ = std::make_shared<PlanningVisualization>(nh);
  vis_ptr_       = std::make_shared<visualization::Visualization>(nh);

  nh.param("exploration/refine_local", ep_->refine_local_, true);
  nh.param("exploration/refined_num", ep_->refined_num_, -1);
  nh.param("exploration/refined_radius", ep_->refined_radius_, -1.0);
  nh.param("exploration/top_view_num", ep_->top_view_num_, -1);
  nh.param("exploration/max_decay", ep_->max_decay_, -1.0);
  nh.param("exploration/tsp_dir", ep_->tsp_dir_, string("null"));
  nh.param("exploration/relax_time", ep_->relax_time_, 1.0);

  nh.param("exploration/vm", ViewNode::vm_, -1.0);
  nh.param("exploration/am", ViewNode::am_, -1.0);
  nh.param("exploration/yd", ViewNode::yd_, -1.0);
  nh.param("exploration/ydd", ViewNode::ydd_, -1.0);
  nh.param("exploration/w_dir", ViewNode::w_dir_, -1.0);

  nh.param("exploration/radius_close_goal", ep_->radius_close_, -1.0);
  nh.param("exploration/radius_far_goal", ep_->radius_far_, -1.0);
  nh.param("tracking/track_dist", ep_->track_dist_, 0.5);
  nh.param("tracking/dist_far", ep_->track_dist_thr_, 5.0);
  nh.param("tracking/replan_dist", ep_->track_replan_dist_, 0.5);
  nh.param("tracking/turn_yaw_dist", ep_->track_turn_yaw_dist_, 0.5);
  nh.param("tracking/yaw_threshold", ep_->track_yaw_thr_, 0.5);
  nh.param("tracking/detect_error", ep_->track_detect_error_, 1.5);

  // Initialize TSP par file
  ofstream par_file(ep_->tsp_dir_ + "/single.par");
  par_file << "PROBLEM_FILE = " << ep_->tsp_dir_ << "/single.tsp\n";
  par_file << "GAIN23 = NO\n";
  par_file << "OUTPUT_TOUR_FILE =" << ep_->tsp_dir_ << "/single.txt\n";
  par_file << "RUNS = 1\n";
}


int FrontierManager::planExploreRapid(const Vector3d& pos, const Vector3d& vel, 
                                      Vector3d& aim_pos, Vector3d& aim_vel, vector<Eigen::Vector3d>& path_res)
{
  //! Update Frontiers
  // setCurrentTopoNode(scene_graph_->skeleton_gen_->mountCurTopoPoint(pos, 0.0f));
  // frontier_finder_->searchFrontiers(pos);
  // scene_graph_->expandSkeleton(pos, 0.0);
  // // frontier_finder_->addPoseGraphInfo(pos);
  // // frontier_finder_->updateMultiPosegraph();
  // frontier_finder_->computeFrontiersToVisit(pos);
  // frontier_finder_->updateFrontierCostMatrix();
  //
  // //! Update HGrid
  // updateHgrid();

  //! Choose Ftr
  frontier_finder_->getFrontiersWithInfo(ed_->frontiers_with_info_);
  if (ed_->frontiers_with_info_.empty())
  {
    ROS_ERROR_STREAM("[Ftr_manager] Ftr List empty!");
    return NO_FRONTIER;
  }

  double score;
  double min_score = 99999;
  Frontier best_ftr;
  static Eigen::Vector3d drone_aim_vel_last = aim_vel;
  for (auto& ftr : ed_->frontiers_with_info_)
  {
    Eigen::Vector3d vp_pos      = ftr.viewpoints_.front().pos_;
    Eigen::Vector3d drone_2_vp  = vp_pos - pos;
    double cur_dis = drone_2_vp.norm();

    double d_theta = acos(vel.normalized().dot(drone_2_vp.normalized())) * 180.0 / M_PI; //0~180
    double d_theta_from_last = acos(drone_aim_vel_last.normalized().dot(drone_2_vp.normalized())) * 180.0 / M_PI; //0~180

    if(cur_dis < frontier_finder_->percep_utils_->getSensorMaxDist()){
        // score = cur_dis;
        // score = cur_dis + (1.0/180.0) * d_theta / (cur_dis*cur_dis) + (1.0/1) * d_theta_from_last / (cur_dis*cur_dis);
        // score = cur_dis + (1.0/30.0) * d_theta_from_last + (1.0/30.0) * d_theta; 
        // score = cur_dis + (1.0/1) * d_theta_from_last / (cur_dis*cur_dis);
        score = cur_dis + (1.0/1) * d_theta / (36.0);

    }else{
        // score = cur_dis + (1.0/1) * d_theta_from_last / (cur_dis*cur_dis);
        score = cur_dis + (1.0/1) * d_theta / (36.0);
        // score = cur_dis;// + (1.0/30.0) * d_theta_from_last;
    }
    ROS_ASSERT(score > 0 && score < 1000);
    if(score < min_score){
        best_ftr = ftr; 
        min_score = score;
    }
  }

  if(min_score < 99999){
    aim_pos = best_ftr.viewpoints_.front().pos_;
    path_res.clear();
    path_res.push_back(pos);
    path_res.push_back(aim_pos);
    aim_vel.setZero();

    // getPathtoGoal(best_sv, drone_p_cur, path_res);
    // fc_info_ptr_->vis_ptr->visualize_path(path_res, "graph_path");
    // res_aimpos = best_sv.getPoseForPlan();

    // res_aimvel = calResVel(res_aimpos, drone_p_cur);

    // cout<<"vmax: "<<v_max<<endl;
    // cout<<"res_aimpos: "<<res_aimpos.transpose()<<endl;
    // cout<<"res_aimvel: "<<res_aimvel.transpose()<<endl;
    // drone_aim_vel_last = aim_vel;
    return SUCCEED;
  }
  return FAIL;
}


int FrontierManager::planExploreTSP(const Vector3d& pos, const Vector3d& vel, const double& yaw,
                                    Vector3d& aim_pos, Vector3d& aim_vel, double& aim_yaw, vector<Eigen::Vector3d>& path_res)
{
  //! Update Frontiers
  // scene_graph_->expandSkeleton(pos, yaw);
  // setCurrentTopoNode(scene_graph_->skeleton_gen_->mountCurTopoPoint(pos, 0.0f));
  // frontier_finder_->searchFrontiers(pos);
  // frontier_finder_->computeFrontiersToVisit(pos);
  // frontier_finder_->updateFrontierCostMatrix();
  //
  // //! Update HGrid
  // updateHgrid();

  //! Choose Ftr
  frontier_finder_->getFrontiersWithInfo(ed_->frontiers_with_info_);

  if (ed_->frontiers_with_info_.empty())
  {
    ROS_ERROR_STREAM("[Ftr_manager] Ftr List empty!");
    return NO_FRONTIER;
  }

  //! If more than one ftr
  ed_->path_next_goal_.clear();
  Frontier next_ftr;
  vector<Eigen::Vector3d> path_2_ftr;
  bool found_path = false;
  ROS_INFO_STREAM("frontiers_ size: " << ed_->frontiers_with_info_.size());

  if (ed_->frontiers_with_info_.size() > 1)
  {
    // Find the global tour passing through all viewpoints
    // Create TSP and solve by LKH
    // Optimal tour is returned as indices of frontier
    vector<int> indices;
    Eigen::MatrixXd cost_mat;
    findGlobalTour(pos, vel, yaw, indices, cost_mat); 

    // ed_->refined_ids_.clear();
    // int knum = min(int(indices.size()), ep_->refined_num_);
    // for (int i = 0; i < knum; ++i)
    // {
    //   auto tmp_ftr = ed_->frontiers_with_info_[indices[i]];
    //   ed_->refined_ids_.push_back(indices[i]);
    //   if ((tmp_ftr.average_ - pos).norm() > ep_->refined_radius_ && ed_->refined_ids_.size() >= 2) break;
    // }
    //
    // // Get top N viewpoints for the next K frontiers
    // ed_->n_points_.clear();
    // ed_->refined_points_.clear();
    // ed_->refined_views_.clear();
    // vector<vector<double>> n_yaws;
    // vector<double> refined_yaws;
    // frontier_finder_->getViewpointsInfo( pos, ed_->refined_ids_, ep_->top_view_num_,
    //                                      ep_->max_decay_, ed_->n_points_, n_yaws);
    // refineLocalTour(pos, vel, yaw, ed_->n_points_, n_yaws, ed_->refined_points_, refined_yaws);

    // Get path to next goal
    next_ftr = ed_->frontiers_with_info_[indices[0]];
    // aim_pos = ed_->refined_points_.front();
    // aim_yaw = refined_yaws.front();
    aim_pos = ed_->frontiers_with_info_[indices[0]].viewpoints_.front().pos_;
    aim_yaw = ed_->frontiers_with_info_[indices[0]].viewpoints_.front().yaw_;
    found_path = planNextFtr(pos, next_ftr, aim_pos, path_2_ftr, false);
    if (!found_path)
    {
      ed_->frontier_to_explore_ = next_ftr;
      return FAIL;
    }

    // Get marker for view visualization
    // for (size_t i = 0; i < ed_->refined_points_.size(); ++i) {
    //   Vector3d view =
    //       ed_->refined_points_[i] + 2.0 * Vector3d(cos(refined_yaws[i]), sin(refined_yaws[i]), 0);
    //   ed_->refined_views_.push_back(view);
    // }
    // ed_->refined_views1_.clear();
    // ed_->refined_views2_.clear();
    // for (size_t i = 0; i < ed_->refined_points_.size(); ++i) {
    //   vector<Vector3d> v1, v2;
    //   frontier_finder_->percep_utils_->setPose(ed_->refined_points_[i], refined_yaws[i]);
    //   frontier_finder_->percep_utils_->getFOV(v1, v2);
    //   ed_->refined_views1_.insert(ed_->refined_views1_.end(), v1.begin(), v1.end());
    //   ed_->refined_views2_.insert(ed_->refined_views2_.end(), v2.begin(), v2.end());
    // }
  }
  else
  {
    // Get path to next goal
    next_ftr = ed_->frontiers_with_info_.front();
    aim_pos = next_ftr.viewpoints_.front().pos_;
    aim_yaw = next_ftr.viewpoints_.front().yaw_;
    found_path = planNextFtr(pos, next_ftr, aim_pos, path_2_ftr, false);
    if (!found_path)
    {
      ed_->frontier_to_explore_ = next_ftr;
      return FAIL;
    }
  }

  aim_vel.setZero();
  path_res = path_2_ftr;
  ed_->frontier_to_explore_ = next_ftr;

  INFO_MSG_GREEN("\n\n *** >>> path_next_goal_[" << path_res.size() << "]:");
  for(auto p:path_res) std::cout << p.transpose() << std::endl;

  ed_->path_next_goal_ = path_2_ftr;
  return SUCCEED;

}

int FrontierManager::planTrackGoal(const Vector3d& pos, const Vector3d& vel,
                                   const Vector3d& far_goal, vector<Eigen::Vector3d>& path_res)
{
  INFO_MSG_GREEN("=================== planTrackGoal =====================");
  (void)vel;

  const Eigen::Vector3d aim_pos = far_goal;
  ed_->path_next_goal_.clear();
  path_res.clear();

  const double dis_2_aim = (pos - aim_pos).norm();
  if (dis_2_aim > ep_->track_dist_thr_)
  {
    INFO_MSG_RED("[TrackPlanner] Target is too far for direct tracking.");
    return FAIL;
  }

  local_aim_type_ = PLAN_TO_WHAT::PATH_TO_GOAL;
  if (!map_->isVisible(pos, aim_pos, 0.0))
  {
    INFO_MSG_RED("[TrackPlanner] Line-of-sight to tracking aim is blocked.");
    return FAIL;
  }

  path_res.push_back(pos);
  path_res.push_back(aim_pos);
  ed_->path_next_goal_ = path_res;
  INFO_MSG("[TrackPlanner] direct visible goal.");
  return SUCCEED;
}

/**
 * @brief 智能地规划一条到下一个前沿目标点的路径。
 * * 该函数采用分层策略：首先尝试最高效的直接连接；如果失败，则在局部地图内使用A*算法进行可达性验证；
 * 如果目标仍不可达或太远，则回退到最低效但最可靠的全局拓扑路径规划。
 * 最终，函数还会对生成的路径进行“捷径”优化，以提高执行效率。
 * * @param[in]  pos                  机器人当前的位置。
 * @param[in]  next_ftr             目标前沿点的完整对象，主要用于获取其拓扑信息。
 * @param[in]  aim_pos              本次规划需要到达的具体目标视点位置。
 * @param[out] path_res             用于存储最终生成的、经过优化的路径点集。
 * @param[in]  force_direct_egoplan 一个布尔标志，若为true，则强制生成一条直线路径，忽略所有中间逻辑。
 * @return     bool                 如果成功规划出一条有效路径，则返回true；否则返回false。
 */
bool FrontierManager::planNextFtr(const Vector3d& pos, const Frontier& next_ftr, Vector3d& aim_pos, vector<Eigen::Vector3d>& path_res, bool force_direct_egoplan)
{
  INFO_MSG_GREEN("================== planNextFtr =====================");
  path_res.clear();
  // PoseGraph posegraph_ = ed_->posegraph_m_->getPosegraph(0);
  // posegraph_.renewKdtree();
  double dis_2_aim = (pos - aim_pos).norm();
  INFO_MSG_GREEN("[PlanNxtFtr] mount cur topo node ...");
  PolyHedronPtr cur_topo_node = scene_graph_->skeleton_gen_->mountCurTopoPoint(pos, false);
  INFO_MSG_GREEN("pos: " << pos.transpose() << ", aim_pos: " << aim_pos.transpose() << ", nxt_ftr[ " << next_ftr.id_ << " ]" << next_ftr.viewpoints_.front().pos_.transpose());

  bool can_direct_arrive = false;
  bool a_star_success = true;
  double a_star_dist;
  if (map_->isVisible(pos, aim_pos, 0.0) &&
      dis_2_aim < ep_->radius_far_)
  {
    can_direct_arrive = true;
    path_res.push_back(pos);
    path_res.push_back(aim_pos);
    INFO_MSG("direct visible goal.");
  }
  else if (map_->isInLocalMap(aim_pos))
  {
    vector<Eigen::Vector3d> path_2_ftr;
    a_star_success = ViewNode::searchPath(pos, aim_pos, path_2_ftr, a_star_dist);
    INFO_MSG("line length to goal: " << dis_2_aim << ", a* path length: " << a_star_dist << " | " << 1.3 * dis_2_aim + map_->getResolution() * 2.0);

    if (dis_2_aim < ep_->radius_far_)
    {
      if (!a_star_success || a_star_dist > 1.3 * dis_2_aim + map_->getResolution() * 2.0) //TODO magic number 1.3
      {
        can_direct_arrive = false;
        INFO_MSG("mid hard goal, turn to topo path search.");
      }
      else
      {
        can_direct_arrive = true;
        path_res.push_back(pos);
        path_res.push_back(aim_pos);
        INFO_MSG("mid easy goal.");
      }
    }
  }

  // 如果ftr topo连接到当前节点，判断其连接是否合理e
  if (map_->isInLocalMap(aim_pos) && next_ftr.topo_father_ == cur_topo_node)
  {
    if (!a_star_success)
    {
      INFO_MSG_RED("Aim in local map, topo connect to current pos, but a* fail.");
      return false;
    } 
    if (a_star_dist > 1.8 * dis_2_aim)
    {
      INFO_MSG_RED("Aim in local map, topo connect to current pos, but a* len too long.");
      // force_direct_egoplan = true;
      return false;
    }
  }

  if (!can_direct_arrive)
  {
    INFO_MSG_GREEN("---- > [FtrManager] far goal < ----");
    if (force_direct_egoplan)
    {
      path_res.push_back(pos);
      path_res.push_back(aim_pos);
      return true;
    }

    vector<Eigen::Vector3d> path_2_ftr;
    double dis_2_ftr = -1.0;

    // 找当前位置最近的topo节点
    if (cur_topo_node != nullptr){
      frontier_finder_->getPathWithTopo(cur_topo_node, pos,
                                    next_ftr.topo_father_, next_ftr.viewpoints_.front().pos_,
                                    path_2_ftr, dis_2_ftr);
    }
    else{
      ROS_ERROR_STREAM("[Ftr_manager] No topo node near current position!");
      path_2_ftr.push_back(pos);
      path_2_ftr.push_back(aim_pos);
      dis_2_ftr = (pos - aim_pos).norm();
    }

    // 简化topo路径, 取第一个在Local地图中看不见的点作为起点
    int start_inx = 0;
    for(int i = path_2_ftr.size()-1; i > 1; i--)
    {
      if (map_->isInLocalMap(path_2_ftr[i]) &&
          map_->isVisible(pos, path_2_ftr[i]))
      {
        start_inx = i;
        break;
      }
    }
    path_res.push_back(pos);
    for (size_t i = start_inx; i < path_2_ftr.size(); ++i)
    {
      path_res.push_back(path_2_ftr[i]);
    }

    INFO_MSG_GREEN("path_2_ftr[" << path_2_ftr.size() << "]:");
    for(auto p:path_2_ftr) std::cout << p.transpose() << std::endl;
  }
  return true;
}

// void FrontierManager::updateTopoBlacklist(Eigen::Vector3d goal, Eigen::Vector3d cur,
//                                           double range, std::unordered_map<int, int> & blacklist) {
//   // init some judgement data
//   bool blacklist_changed = false;
//   double dist_2_goal = (goal - cur).norm();
//
//   // black list logic
//   if(dist_2_goal < ep_->radius_far_ / 2.0) {
//     // init topo class
//     // update posegraph
//     frontier_finder_->getPoseGraph(ed_->posegraph_m_);
//     ed_->posegraph_used_by_blacklist_cal_ = ed_->posegraph_m_->getPosegraph(0);
//     ed_->posegraph_used_by_blacklist_cal_.renewKdtree();
//
//     map<int, double> nearby_v_set;
//     pcl::PointXYZI start_pos;
//     start_pos.x = static_cast<float>(cur.x());
//     start_pos.y = static_cast<float>(cur.y());
//     start_pos.z = static_cast<float>(cur.z());
//     ed_->posegraph_used_by_blacklist_cal_.getPotenConnectSetB(start_pos, nearby_v_set, range, 0.0);
//
//     // init astar search
//     vector <Vector3d> path_tmp;
//     double dis_res_tmp;
//     // iterate all close topo point, if topo point is close enough and goal is in
//     // radius_far_ at the same time; then judge if topo point can directly reach goal
//     for (auto &point: nearby_v_set) {
//       pcl::PointXYZI topo_p = ed_->posegraph_used_by_blacklist_cal_.getCor(point.first);
//       Eigen::Vector3d topo_pos(topo_p.x, topo_p.y, topo_p.z);  // get pose of topo point
//       if (!ViewNode::searchPath(topo_pos, goal, path_tmp, dis_res_tmp)) {
//         blacklist[point.first] = 1; // add to black list
//         blacklist_changed = true;
//       }
//     }
//   }
//   // refresh visualization
//   if(blacklist_changed)
//     visBlacklist();
// }

// Eigen::Vector3d FrontierManager::getBlacklistTopoPos(int idx) {
//   pcl::PointXYZI topo_p = ed_->posegraph_used_by_blacklist_cal_.getCor(idx);
//   Eigen::Vector3d topo_pos(topo_p.x, topo_p.y, topo_p.z);
//   return topo_pos;
// }

void FrontierManager::forceDeleteFrontier(Frontier ftr)
{
  frontier_finder_->frontierForceDelete(ftr);
}

void FrontierManager::updateHgrid()
{
  // frontier_finder_->getFrontierAvgs(ed_->averages_);
  frontier_finder_->getFrontiers(ed_->frontiers_);
  
  vector<int> first_ids, second_ids;
  hgrid_->inputFrontiers(ed_->frontiers_);

  int drone_id = 1;
  bool reallocated = false;
  vector<int> grid_ids;
  hgrid_->updateGridData(
    drone_id, grid_ids, reallocated, ed_->last_grid_ids_, first_ids, second_ids);

  // ROS_WARN_STREAM("[grid_ids]:");
  // for (auto id:grid_ids){
  //   ROS_WARN_STREAM("id: " << id);
  // }

  ed_->last_grid_ids_ = grid_ids;
}

int FrontierManager::planLLMExploration(const int &area_id, const Eigen::Vector3d &cur_pos,
                                        const Eigen::Vector3d cur_vel, const double &cur_yaw,
                                        const PolyHedronPtr &cur_poly, Eigen::Vector3d &aim_pos,
                                        double &aim_yaw, Eigen::Vector3d &aim_vel, std::vector<Eigen::Vector3d> &path_res) {

  ros::Time t1 = ros::Time::now();
  std::vector<Frontier> ftrs_in_area;
  // frontiers_是一个列表，遍历它
  for (auto& ftr: frontier_finder_->frontiers_) {
    if (ftr.topo_father_ != nullptr) {
      if (ftr.topo_father_->area_id_ == area_id)
        ftrs_in_area.push_back(ftr);
    }
  }
  INFO_MSG_YELLOW("[LLM-Plan] : find " << ftrs_in_area.size() << " frontiers in area " << area_id);
  for (const auto& ftr : ftrs_in_area)
    INFO_MSG_YELLOW("Candidate frontier : " << ftr.id_ << ", pos " << ftr.average_.transpose());

  Frontier nxt_ftr;
  bool     found_path_2_nxt_ftr = false;
  vector<Eigen::Vector3d> path_2_nxt_ftr;

  if (!ftrs_in_area.empty()) {
    std::vector<int> indices;
    findGlobalTour_SomeFtrs(ftrs_in_area, cur_pos, cur_vel, cur_yaw, cur_poly, indices);
    // local goal refine
    // step1. find all ftrs **needed to refined** -> calculate refinded_ids_
    ed_->refined_ids_.clear();
    int knum = min(static_cast<int>(indices.size()), ep_->refined_num_);
    for (int i = 0; i< knum; ++i) {
      auto tmp_ftr = ftrs_in_area[indices[i]];
      ed_->refined_ids_.push_back(indices[i]);
      if ((tmp_ftr.average_ - cur_pos).norm() > ep_->refined_radius_ && ed_->refined_ids_.size() >= 2)
        break;
    }
    ed_->n_points_.clear(); ed_->refined_points_.clear(); ed_->refined_views_.clear();
    vector<vector<double>> n_yaws;
    vector<double> refined_yaws;
    // step2. get candidate viewpoint list
    for (int i = 0; i<ed_->refined_ids_.size(); ++i) {
      auto& f = ftrs_in_area.at(ed_->refined_ids_[i]);
      vector<Eigen::Vector3d> pts; vector<double> yaws;
      int visib_thresh = f.viewpoints_.front().visib_num_ * ep_->max_decay_;
      for (const auto& vp : f.viewpoints_) {
        if (static_cast<int>(pts.size()) >= ep_->top_view_num_ || vp.visib_num_ < visib_thresh) break;   // 视点过多或者有效观测不足
        if ((vp.pos_ - cur_pos).norm() < frontier_finder_->min_candidate_dist_) continue;
        pts.push_back(vp.pos_); yaws.push_back(vp.yaw_);
      }
      if (pts.empty()) {
        for (const auto& vp : f.viewpoints_) {
          if (pts.size() >= ep_->top_view_num_ || vp.visib_num_ < visib_thresh) break;
          pts.push_back(vp.pos_); yaws.push_back(vp.yaw_);
        }
      }
      ed_->n_points_.push_back(pts);
      n_yaws.push_back(yaws);
    }
    // step3. get refined local tour
    ros::Time t2 = ros::Time::now();
    refineLocalTour(cur_pos, cur_vel, cur_yaw, ed_->n_points_, n_yaws, ed_->refined_points_, refined_yaws);
    INFO_MSG_GREEN("[RefineLocalTour]: time spent : " << (ros::Time::now() - t2).toSec() * 1e3 << "ms");
    // step4. get nxt target
    nxt_ftr = ftrs_in_area[indices[0]];
    aim_pos = ed_->refined_points_.front();
    aim_yaw = refined_yaws.front();
    found_path_2_nxt_ftr = planNextFtr(cur_pos, nxt_ftr, aim_pos, path_2_nxt_ftr, false);
    if (!found_path_2_nxt_ftr){
      ed_->frontier_to_explore_ = nxt_ftr;
      return FAIL;
    }
    // step5.  Get marker for view visualization
    for (size_t i = 0; i < ed_->refined_points_.size(); ++i) {
      Vector3d view =
          ed_->refined_points_[i] + 2.0 * Vector3d(cos(refined_yaws[i]), sin(refined_yaws[i]), 0);
      ed_->refined_views_.push_back(view);
    }
    ed_->refined_views1_.clear();
    ed_->refined_views2_.clear();
    for (size_t i = 0; i < ed_->refined_points_.size(); ++i) {
      vector<Vector3d> v1, v2;
      frontier_finder_->percep_utils_->setPose(ed_->refined_points_[i], refined_yaws[i]);
      frontier_finder_->percep_utils_->getFOV(v1, v2);
      ed_->refined_views1_.insert(ed_->refined_views1_.end(), v1.begin(), v1.end());
      ed_->refined_views2_.insert(ed_->refined_views2_.end(), v2.begin(), v2.end());
    }
  }else if (ftrs_in_area.size() == 1){
    nxt_ftr = ftrs_in_area.front();
    aim_pos = nxt_ftr.viewpoints_.front().pos_;
    aim_yaw = nxt_ftr.viewpoints_.front().yaw_;
    found_path_2_nxt_ftr = planNextFtr(cur_pos, nxt_ftr, aim_pos, path_2_nxt_ftr, false);
    if (!found_path_2_nxt_ftr){
      ed_->frontier_to_explore_ = nxt_ftr;
      return FAIL;
    }
  }else {
    INFO_MSG_RED("Target Area [" << area_id << "] has no frontier!");
    return FAIL;
  }

  INFO_MSG_GREEN("[LLM-PLan] : spend time ->" << (ros::Time::now() - t1).toSec() *1e3 <<"ms");

  aim_vel.setZero();
  path_res.clear();
  path_res = path_2_nxt_ftr;
  ed_->frontier_to_explore_ = nxt_ftr;

  INFO_MSG_GREEN("[LLM_Plan] : next Frontier -> "<< nxt_ftr.id_ << " in area " << nxt_ftr.topo_father_->area_id_ << ", pos " << nxt_ftr.average_.transpose());
  INFO_MSG_GREEN("[LLM-Plan] : path to next goal (path size: [" << path_res.size() << "])");
  for (const auto& p : path_res) std::cout << p.transpose() << "->";
    std::cout << std::endl;

  ed_->path_next_goal_ = path_2_nxt_ftr;
  return SUCCEED;
}

void FrontierManager::findGlobalTour_SomeFtrs(std::vector<Frontier>& ftr_select, const Eigen::Vector3d& cur_pos,
                                 const Eigen::Vector3d cur_vel, const double& cur_yaw,
                                 const PolyHedronPtr& cur_poly, vector<int>& indices) {
  // get cost matrix between frontiers
  auto calCostBetween2Ftrs = [this](Frontier& f1, Frontier& f2) {
    vector<Eigen::Vector3d> path;
    double dis             = -1.0f;
    bool search_success    = false;
    auto& vp1 = f1.viewpoints_.front(); auto& vp2 = f2.viewpoints_.front();

    if (f1.topo_father_ == nullptr || f2.topo_father_ == nullptr) {
      dis = 9999;
      path.push_back(vp1.pos_); path.push_back(vp2.pos_);
    }else if (f1.topo_father_ == f2.topo_father_) {
      search_success = ViewNode::searchPath(vp1.pos_, vp2.pos_, path, dis);
      if (dis < 0.0f || !search_success) {
        path.push_back(vp1.pos_); path.push_back(vp2.pos_);
        dis = (vp1.pos_ - vp2.pos_).norm();
      }
    }
    if (dis < 0.0f && map_->isVisible(vp1.pos_, vp2.pos_, 0.0)) {
      search_success = ViewNode::searchPath(vp1.pos_, vp2.pos_, path, dis);
    }
    if (dis < 0.0f || !search_success) {
      frontier_finder_->getPathWithTopo(f1.topo_father_, vp1.pos_, f2.topo_father_, vp2.pos_, path, dis);
    }
    double path_cost = dis / ego_planner::ViewNode::vm_;
    double yaw_diff  = fabs(vp1.yaw_ - vp2.yaw_); yaw_diff = min(yaw_diff, 2.0 * M_PI - yaw_diff);
    double yaw_cost  = yaw_diff / ego_planner::ViewNode::yd_;
    double cost      = path_cost;

    f1.costs_2[f2.id_] = cost;
    f2.costs_2[f1.id_] = cost;
    f1.paths_2[f2.id_] = path;
    f2.paths_2[f1.id_] = path;
  };

  // get cost to home
  auto calCostToHome = [this](Frontier& f) {
    f.cost_to_home_2 = -1.0f;
    if (f.topo_father_ == nullptr)
      f.cost_to_home_2 = 9999.0f;
    else {
      std::vector<Eigen::Vector3d> path;
      f.cost_to_home_2 = scene_graph_->skeleton_gen_->
           astarSearch(f.viewpoints_.front().pos_, Eigen::Vector3d::Zero(), path, true) / ViewNode::vm_;
      if (f.cost_to_home_2 < 0.0)
        f.cost_to_home_2 = (f.viewpoints_.front().pos_).norm() / ViewNode::vm_;
    }
  };

  // get cost from cur_state to ftr
  auto calCostFromCurStateToFtr = [this, cur_pos, cur_poly, cur_yaw, cur_vel](Frontier& f) -> double {
    double dis = -1.0f;
    std::vector<Eigen::Vector3d> path;
    auto vp = f.viewpoints_.front();
    if (f.topo_father_ == nullptr) return 9999.0f;
    if (f.topo_father_ == cur_poly || map_->isVisible(vp.pos_, cur_pos, 0.0)) {
      if (!ViewNode::searchPath(cur_pos, vp.pos_, path, dis)) {
        path.clear();
        path.push_back(cur_pos), path.push_back(vp.pos_);
        dis = (cur_pos - vp.pos_).norm();
      }
    }else {
      frontier_finder_->getPathWithTopo(f.topo_father_, vp.pos_, cur_poly, cur_pos, path, dis);
    }
    if (dis > 0.0f) {
      double yaw_diff  = fabs(vp.yaw_ - cur_yaw); yaw_diff = min(yaw_diff, 2.0 * M_PI - yaw_diff);
      double yaw_cost  = yaw_diff / ego_planner::ViewNode::yd_;
      double path_cost = dis / ego_planner::ViewNode::vm_;

      Eigen::Vector3d dir_2_vp = (vp.pos_ - cur_pos).normalized();
      if (cur_vel.norm() > 1e-3) {
        Eigen::Vector3d vidr = cur_vel.normalized();
        double diff          = acos(vidr.dot(dir_2_vp));
        path_cost           += ego_planner::ViewNode::w_dir_ * diff;
      }else {
        Eigen::Vector3d vdir(cos(cur_yaw), sin(cur_yaw), 0.0);
        double diff          = acos(vdir.dot(dir_2_vp));
        path_cost           += ego_planner::ViewNode::w_dir_ * diff;
      }
      double cost = max(path_cost, yaw_cost);
      return cost;
    }else {
      return 9999.0f;
    }
  };

  ros::Time t1 = ros::Time::now();

  if (ftr_select.size() > 1) {
    for (int i = 0; i< ftr_select.size(); ++i) {
      ftr_select[i].id_ = i;
      ftr_select[i].costs_2.clear(), ftr_select[i].costs_2.resize(ftr_select.size());
      ftr_select[i].paths_2.clear(), ftr_select[i].paths_2.resize(ftr_select.size());
    }
    // matrix initialization
    unsigned int dim = ftr_select.size();
    Eigen::MatrixXd mat = Eigen::MatrixXd::Zero(dim + 2, dim + 2);
    mat.block(1, 0, dim, 1).setConstant(99999.0);      // vp -> cur_state max 不走回头路
    mat.block(dim + 1, 1, 1, dim).setConstant(99999.0);// home -> vp max      不从家离开
    mat(dim + 1, 0) = 0.0;                             // home -> cur_state free 免费传送门
    mat(0, dim + 1) = 99999.0;                         // 不允许直接回家
    // step1 ftr <-> ftr
    for (int i = 0; i < dim; ++i) {
      for (int j = i + 1; j < dim; ++j) {
        calCostBetween2Ftrs(ftr_select[i], ftr_select[j]);
        mat(i + 1, j + 1) = mat(j + 1, i + 1) = ftr_select[i].costs_2[ftr_select[j].id_];
      }
    }
    // step2 & 3 cur_state -> ftr | ftr -> home
    for (int i = 0; i < dim; ++i) {
      mat(0, i + 1) = calCostFromCurStateToFtr(ftr_select[i]);
      calCostToHome(ftr_select[i]);
      mat(i + 1, dim + 1) = ftr_select[i].cost_to_home_2;
    }

    // Solve TSP
    INFO_MSG("   * [Solve TSP] start...");
    solveTSP(mat, indices);
    std::vector<Eigen::Vector3d> global_tour;
    if (ftr_select.size() >= 1) {
      global_tour.push_back(cur_pos);
      global_tour.push_back(ftr_select[indices.front()].viewpoints_.front().pos_);
    }
    if (ftr_select.size() > 1) {
      for (int i = 0; i < indices.size() - 1; ++i) {
        global_tour.push_back(ftr_select[indices[i]].viewpoints_.front().pos_);
        global_tour.push_back(ftr_select[indices[i + 1]].viewpoints_.front().pos_);
      }
    }
    visualization_->drawVPGlobalPath(global_tour, 0.03, Eigen::Vector4d(0.0, 1.0, 0.5, 1.0), "global_tour", 0);

    // INFO_MSG("\n\n   * [Solve TSP] spend time :" << (ros::Time::now() - t1).toSec()* 1e3 << "ms");
    // INFO_MSG("   * [Solve TSP] cost matrix:");
    // INFO_MSG(mat);
  }else {
    if (!ftr_select.empty()) {
      indices.push_back(0);
    }
  }
}

void FrontierManager::findVPGlobalTour(std::vector<Viewpoint::Ptr> &vps, const Eigen::MatrixXd &cost_mat, const Eigen::Vector3d &cur_pos, const double &
                                       cur_yaw, const Eigen::Vector3d &cur_vel) {
  std::vector<int> indices;
  solveTSP(cost_mat, indices);
  std::vector<Viewpoint::Ptr> global_tour;
  for (int idx : indices)
    global_tour.push_back(vps.at(idx));

  std::vector<Eigen::Vector3d> global_tour_line;
  global_tour_line.push_back(cur_pos);
  global_tour_line.push_back(global_tour_line.at(0));
  for (int i = 0; i < global_tour.size() - 1; ++i) {
    global_tour_line.push_back(global_tour[i]->pos_);
    global_tour_line.push_back(global_tour[i + 1]->pos_);
  }
  visualization_->drawVPGlobalPath(global_tour_line, 0.05, Eigen::Vector4d(0.0, 1.0, 0.5, 1.0), "global_tour", 0);
}

void FrontierManager::findGlobalTour(
    const Vector3d& cur_pos, const Vector3d& cur_vel, const double& cur_yaw,
    vector<int>& indices, Eigen::MatrixXd& cost_mat) 
{
  // Get cost matrix for current state and clusters
  // Eigen::MatrixXd cost_mat;
  auto t1 = ros::Time::now();
  frontier_finder_->getFullCostMatrix(cur_pos, cur_vel, cur_yaw, cost_mat);
  double mat_time = (ros::Time::now() - t1).toSec();
  solveTSP(cost_mat, indices);

  // Get the path of optimal tour from path matrix
  t1 = ros::Time::now();
  frontier_finder_->getPathForTour(cur_pos, indices, ed_->global_tour_, ed_->path_next_goal_);

  double tsp_time = (ros::Time::now() - t1).toSec();
  ROS_WARN("Cost mat: %lf, TSP: %lf", mat_time, tsp_time);
}

/**
* @brief 通过构建和搜索一个分层的视点图，从未经筛选的多个局部前沿的候选视点中，提炼出一条最优的局部巡游路径。
* * 该函数将机器人的当前状态作为起点，将每个前沿的多个候选视点作为图中的节点层。
* 它构建一个从前一层的所有节点到后一层所有节点的有向图，然后使用Dijkstra算法
* 搜索从起点到终点（最后一个前沿的代表性视点）的最短路径。
* 这条最短路径所经过的节点序列，即为成本最低的精炼局部旅程。
* * @param[in]  cur_pos      机器人当前的位置。
* @param[in]  cur_vel      机器人当前的速度。
* @param[in]  cur_yaw      机器人当前的偏航角。
* @param[in]  n_points     一个嵌套向量，包含了多个局部前沿的候选视点位置。
* 外层vector的每个元素代表一个前沿，内层vector包含了该前沿的所有候选视点。
* @param[in]  n_yaws       一个嵌套向量，结构与n_points对应，存储了每个候选视点的期望偏航角。
* @param[out] refined_pts  用于存储最终计算出的最优路径点序列（不包含当前位置）。
* @param[out] refined_yaws 用于存储与最优路径点对应的偏航角序列。
*/
void FrontierManager::refineLocalTour(
    const Vector3d& cur_pos, const Vector3d& cur_vel, const double& cur_yaw,
    const vector<vector<Vector3d>>& n_points, const vector<vector<double>>& n_yaws,
    vector<Vector3d>& refined_pts, vector<double>& refined_yaws) 
{
  double create_time, search_time, parse_time;
  auto t1 = ros::Time::now();

  // Create graph for viewpoints selection
  GraphSearch<ViewNode> g_search;
  vector<ViewNode::Ptr> last_group, cur_group;

  // Add the current state
  ViewNode::Ptr first(new ViewNode(cur_pos, cur_yaw));
  first->vel_ = cur_vel;
  g_search.addNode(first);
  last_group.push_back(first);
  ViewNode::Ptr final_node;

  // Add viewpoints
  std::cout << "Local tour graph: ";
  for (size_t i = 0; i < n_points.size(); ++i) {
    // Create nodes for viewpoints of one frontier
    for (size_t j = 0; j < n_points[i].size(); ++j) {
      ViewNode::Ptr node(new ViewNode(n_points[i][j], n_yaws[i][j]));
      g_search.addNode(node);
      // Connect a node to nodes in last group
      for (auto nd : last_group)
        g_search.addEdge(nd->id_, node->id_);
      cur_group.push_back(node);

      // Only keep the first viewpoint of the last local frontier
      if (i == n_points.size() - 1) {
        final_node = node;
        break;
      }
    }
    // Store nodes for this group for connecting edges
    std::cout << cur_group.size() << ", ";
    last_group = cur_group;
    cur_group.clear();
  }
  std::cout << "" << std::endl;
  create_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  // Search optimal sequence
  vector<ViewNode::Ptr> path1;
  g_search.DijkstraSearch(first->id_, final_node->id_, path1);

  search_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  // Return searched sequence
  for (size_t i = 1; i < path1.size(); ++i) {
    refined_pts.push_back(path1[i]->pos_);
    refined_yaws.push_back(path1[i]->yaw_);
  }

  // Extract optimal local tour (for visualization)
  ed_->refined_tour_.clear();
  ed_->refined_tour_.push_back(cur_pos);
  for (auto pt : refined_pts) {
    vector<Vector3d> path;
    double dis;
    if (ViewNode::searchPath(ed_->refined_tour_.back(), pt, path, dis))
      ed_->refined_tour_.insert(ed_->refined_tour_.end(), path.begin(), path.end());
    else
      ed_->refined_tour_.push_back(pt);
  }

  parse_time = (ros::Time::now() - t1).toSec();
  // ROS_WARN("create: %lf, search: %lf, parse: %lf", create_time, search_time, parse_time);
}



void FrontierManager::shortenPath(vector<Vector3d>& path) 
{
  if (path.empty()) {
    ROS_ERROR("Empty path to shorten");
    return;
  }
  // Shorten the tour, only critical intermediate points are reserved.
  const double dist_thresh = 3.0;
  vector<Vector3d> short_tour = { path.front() };
  for (size_t i = 1; i < path.size() - 1; ++i) {
    if ((path[i] - short_tour.back()).norm() > dist_thresh)
      short_tour.push_back(path[i]);
    else {
      // Add waypoints to shorten path only to avoid collision
      if (!map_->isVisible(short_tour.back(), path[i + 1]))
      {
        short_tour.push_back(path[i]);
      }
    }
  }
  if ((path.back() - short_tour.back()).norm() > 1e-3) short_tour.push_back(path.back());

  // Ensure at least three points in the path
  if (short_tour.size() == 2)
    short_tour.insert(short_tour.begin() + 1, 0.5 * (short_tour[0] + short_tour[1]));
  path = short_tour;
}


void FrontierManager::TSPFormulate(const Eigen::MatrixXd& cost_mat)
{
  const int dimension = cost_mat.rows();

  // Write params and cost matrix to problem file
  ofstream prob_file(ep_->tsp_dir_ + "/single.tsp");
  // Problem specification part, follow the format of TSPLIB
  string prob_spec = "NAME : single\nTYPE : ATSP\nDIMENSION : " + to_string(dimension) +
      "\nEDGE_WEIGHT_TYPE : "
      "EXPLICIT\nEDGE_WEIGHT_FORMAT : FULL_MATRIX\nEDGE_WEIGHT_SECTION\n";

  prob_file << prob_spec;

  // Problem data part
  const int scale = 100;
  // Use Asymmetric TSP
  for (int i = 0; i < dimension; ++i) {
    for (int j = 0; j < dimension; ++j) {
      int int_cost = cost_mat(i, j) * scale;
      prob_file << int_cost << " ";
    }
    prob_file << "\n";
  }
  prob_file << "EOF";
  prob_file.close();
}

void FrontierManager::TSPGetRawRes(vector<int>& ids)
{
  ids.clear();
  // Read optimal tour from the tour section of result file
  ifstream res_file(ep_->tsp_dir_ + "/single.txt");
  string res;
  while (getline(res_file, res)) {
    // Go to tour section
    if (res.compare("TOUR_SECTION") == 0) break;
  }

  // Read path for ATSP formulation
  while (getline(res_file, res)) {
    // Read indices of frontiers in optimal tour
    int id = stoi(res);
    if (id == -1) break;
    ids.push_back(id);
  }

  res_file.close();
}

void FrontierManager::TSPGetRes(const vector<int>& ids, vector<int>& indices)
{
  for (auto id : ids)
  {
    if (id == 1)  // Ignore the current state
      continue;
    indices.push_back(id - 2);  // Idx of solver-2 == Idx of frontier
  }
}

void FrontierManager::TSPGetPartialRes(const vector<int>& ids, const vector<int>& frontier_indices, vector<int>& indices)
{
  for (auto id : ids)
  {
    indices.push_back(frontier_indices[id - 1]);  // Idx of solver-2 == Idx of frontier
  }
}

void FrontierManager::solveTSP(const Eigen::MatrixXd& cost_mat, vector<int>& indices)
{
  const int dimension = cost_mat.rows();

  TSPFormulate(cost_mat);

  // Call LKH TSP solver
  solveTSPLKH((ep_->tsp_dir_ + "/single.par").c_str());
  vector<int> raw_ids;
  TSPGetRawRes(raw_ids);
  TSPGetRes(raw_ids, indices);
  cout << "TSP indices: " << endl;
  for(auto ind:indices){cout<<ind<<", ";}
  cout << endl;
  ROS_ASSERT(indices.back()+2 == dimension); //
  
  indices.pop_back(); // pop the home index (the max one)
}

void FrontierManager::setCurrentTopoNode(PolyHedronPtr topo_node) {
  if (topo_node != nullptr) {
    last_mount_topo_ = cur_mount_topo_;
    cur_mount_topo_ = topo_node;
    frontier_finder_->setCurrentTopoNode(topo_node);
  }
}

void FrontierManager::visualize(const Eigen::Vector3d &pos)
{
  frontier_finder_->getFrontiers(ed_->frontiers_);
  frontier_finder_->getFrontiersWithInfo(ed_->frontiers_with_info_);

  //! Frontier Cells Vis
  frontier_finder_->getFrontierBoxes(ed_->frontier_boxes_);
  visualization_msgs::MarkerArray mk_array;
  visualization_msgs::Marker      mk_frontier_cell;
  for (int i = 0; i < ed_->frontiers_.size(); ++i) {
    visualization_->addFrontierCubesMarkerToArray(mk_array, mk_frontier_cell, ed_->frontiers_[i], 0.2,
                                                  visualization_->getColor(double(i) / double(ed_->frontiers_.size()), 0.8),
                                                  "frontier_cell", i, visualization_msgs::Marker::ADD);
  }
  for (int i = ed_->frontiers_.size(); i < 20 + ed_->frontiers_.size(); ++i) {
    mk_frontier_cell.id = i; mk_frontier_cell.color.a = 0.0;
    mk_frontier_cell.action = visualization_msgs::Marker::DELETE;
    mk_array.markers.push_back(mk_frontier_cell);
  }
  visualization_->drawFrontierCubesByMarkerArray(mk_array); mk_array.markers.clear();

  visFrontierInx();

  //! Viewpoints Vis
  vector<Eigen::Vector4d> viewpoints;
  vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> vp_yaw_pair_list;
  vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> arrows_pair_list;
  arrows_pair_list.clear();
  double arrow_len = 0.3;
  for(const auto& fc:ed_->frontiers_with_info_){
    int cnt = 0;
    for(auto vp:fc.viewpoints_){
      viewpoints.emplace_back(vp.pos_(0), vp.pos_(1), vp.pos_(2), 0);
      arrows_pair_list.emplace_back(fc.average_, vp.pos_);
      // yaw heading
      Vector3d t_p = vp.pos_;
      t_p(0) += cos(vp.yaw_)*arrow_len;
      t_p(1) += sin(vp.yaw_)*arrow_len;
      vp_yaw_pair_list.emplace_back(vp.pos_, t_p);
      if (cnt++ >= 5) break;
    }
  }
  vis_ptr_->visualize_pointcloud_intensity(viewpoints, "viewpoints");
  vis_ptr_->visualize_pairline(arrows_pair_list, "viewpoints_line", 0.06, visualization::Color::red);
  vis_ptr_->visualize_arrows(vp_yaw_pair_list, "viewpoints_yaw", visualization::Color::blue);

  // All viewpoints vis
  vp_yaw_pair_list.clear();
  for (const auto& vp_map : frontier_finder_->vp_handler_->vps_map_) {
    auto vp = vp_map.second;
    Vector3d t_p = vp->pos_;
    t_p(0) += cos(vp->yaw_)*arrow_len;
    t_p(1) += sin(vp->yaw_)*arrow_len;
    vp_yaw_pair_list.emplace_back(vp->pos_, t_p);
  }
  vis_ptr_->visualize_arrows(vp_yaw_pair_list, "all_viewpoints_yaw", visualization::Color::green);

  vector<vector<Vector3d>> dfrontiers;
  frontier_finder_->getDormantFrontiers(dfrontiers);
  for (size_t i = 0; i < dfrontiers.size(); ++i)
  {
    visualization_->drawCubes(dfrontiers[i], 0.1, Eigen::Vector4d(0, 0, 0, 0.5), "dead_frontier", i, 4);
  }

  for (size_t i = dfrontiers.size(); i < 5; ++i)
    visualization_->drawCubes({}, 0.1, Eigen::Vector4d(0, 0, 0, 0.5), "dead_frontier", i, 4);
  // frontier update range visualize
//  for (size_t i = 0; i < ed_->frontiers_with_info_.size(); i++){
//    Eigen::Vector3d center, scale;
//    for (int j = 0; j < 3; ++j)
//      center(j) = (ed_->frontiers_with_info_[i].box_min_(j) + ed_->frontiers_with_info_[i].box_max_(j)) / 2.0;
//    for (int j = 0; j < 2; ++j)
//      scale(j)  = abs(ed_->frontiers_with_info_[i].box_max_(j) - ed_->frontiers_with_info_[i].box_min_(j));
//    scale(2) = 0.1;
//    visualization_->drawFrontierRangeBox(center, scale,
//                                         visualization_->getColor(double(i) / ed_->frontiers_with_info_.size(), 0.4),
//                                         "frontier_range", 1000 + i, 4);
//  }
//  for (size_t i = ed_->frontiers_with_info_.size(); i < ed_->frontiers_with_info_.size() + 50; ++i) {
//    visualization_->deleteFrontierRangeBox(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0),
//                                           Eigen::Vector4d(0, 0, 0, 1), "frontier_range", 1000 + i, 4);
//  }

  // [gwq] TODO skeleton edges vis
  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> edge_list;
  for (const auto & ftr : ed_->frontiers_with_info_)
    if (ftr.topo_father_ != nullptr)
      edge_list.emplace_back(ftr.average_, ftr.topo_father_->center_);

  visualization_->drawFtrEdgeWithSkeleton(edge_list, 0.03, Eigen::Vector4d(0.051, 0.4, 0.6706, 1.0));

  //! TSP Vis
  vector<Eigen::Vector3d> global_tour_up = ed_->global_tour_;
  for (auto& p : global_tour_up) p.z() += 0.5;
  visualization_->drawLines(global_tour_up, 0.07, Eigen::Vector4d(0, 0.5, 0, 1), "global_tour", 0, 6);

  visualization_->drawLines(ed_->path_next_goal_, 0.05, Eigen::Vector4d(0, 1, 1, 1), "path_next_goal", 1, 6);
  vis_ptr_->visualize_pointcloud(ed_->path_next_goal_, "path_next_goal_pt");

  //! Local refined path Vis
  visualization_->drawSpheres(ed_->refined_points_, 0.2, Eigen::Vector4d(0, 0, 1, 1), "refined_pts", 0, 6);

  visualization_->drawLines(ed_->refined_points_, ed_->refined_views_, 0.05,
                            Eigen::Vector4d(0.5, 0, 1, 1), "refined_view", 0, 6);
  visualization_->drawLines(ed_->refined_tour_, 0.07, Eigen::Vector4d(0, 0, 1, 1), "refined_tour", 0, 6);
  visualization_->drawLines(ed_->refined_views1_, ed_->refined_views2_, 0.04, Eigen::Vector4d(0, 0, 0, 1),
                            "refined_view", 0, 6);
                            
  //! HGrid Vis
  visHgrid(pos);
}

void FrontierManager::visHgrid(const Eigen::Vector3d &pos) {
  visualization_msgs::MarkerArray mk_array;
  visualization_msgs::Marker      mk_text, mk_mesh, mk_box;

  // vis explore range
  Eigen::Vector3d map_box_max, map_box_min;
  map_->getGlobalBox(map_box_min, map_box_max);
  visualization_->drawExploreBoxByMarkerArray(mk_array, map_box_min, map_box_max, 0.2, Eigen::Vector4d(0, 0, 1, 0.5), "explore_box", 0, 6);
  mk_array.markers.clear();

  // Vis Mesh
  int hgrid_update_range = hgrid_->getHgridVisRange();
  int update_num         = (2 + hgrid_update_range * 2) * (2 + hgrid_update_range * 2);
  vector<int> hgrid_update_adr_list;
  hgrid_->getVisIdxInRange(hgrid_update_adr_list, pos);
  if (hgrid_update_adr_list.empty())
    return ;

  // vis grid mesh
  if (hgrid_update_range != -1){
    mk_mesh.id = 1;
    mk_mesh.ns = "mesh";
    mk_mesh.action = visualization_msgs::Marker::DELETE;
    mk_array.markers.push_back(mk_mesh);
  }
  vector<Eigen::Vector3d> pts1, pts2;
  hgrid_->getGridMarker(pts1, pts2, hgrid_update_adr_list);
  visualization_->drawLinesByMarkerArray(mk_array, pts1, pts2, 0.05, Eigen::Vector4d(1, 0, 1, 0.5), "mesh", 1, 6);

  // vis text
  // mk_array.markers.clear();
  // vector<Eigen::Vector3d> pts;
  // vector<string> texts;
  // if (hgrid_update_range != -1){
  //   mk_text.id = 0;
  //   mk_text.ns = "text";
  //   mk_text.action = visualization_msgs::Marker::DELETEALL;
  //   mk_array.markers.push_back(mk_text);
  // }
  // hgrid_->getGridMarker2(pts, texts, hgrid_update_adr_list);
  // static int last_text_num = 0;
  // visualization_->fillBasicInfo(mk_text, Eigen::Vector3d(0.5, 0.5, 0.5),  Eigen::Vector4d(0, 0, 0, 1), "text", 0, visualization_msgs::Marker::TEXT_VIEW_FACING);
  // for (size_t i = 0; i < pts.size(); ++i) {
  //   Eigen::Vector3d ptt = pts[i];
  //   visualization_->addHgridTextInfoMarkerToArray(mk_array, mk_text, ptt, texts[i], i);
  // }
  // visualization_->drawHgridTextByMarkerArray(mk_array);

  // vis hgrid state
  mk_array.markers.clear();
  vector<Eigen::Vector3d> center_list, scale_list;
  vector<bool> is_cover_list, is_active_list;
  if (hgrid_update_range != -1){
    mk_box.id = 0;
    mk_box.ns = "state";
    mk_box.action = visualization_msgs::Marker::DELETEALL;
    mk_array.markers.push_back(mk_box);
  }
  hgrid_->getGridStateMarker(center_list, scale_list, is_cover_list, is_active_list, hgrid_update_adr_list);
  visualization_->fillBasicInfo(mk_box, scale_list[0], Eigen::Vector4d(0, 0, 0, 0.3), "state", 0, visualization_msgs::Marker::CUBE);
  for (int i = 0; i < center_list.size(); ++i) {
    Eigen::Vector4d color;
    if (is_cover_list[i]) {
      color << 0, 1, 0, 0.0; // skip cover state
    }
    else color << 0, 0, 1, 0.3;

    if (is_active_list[i])
      visualization_->addHgridActiveInfoMarkerToArray(mk_array, mk_box, center_list[i], color, i, visualization_msgs::Marker::ADD);
    else
      visualization_->addHgridActiveInfoMarkerToArray(mk_array, mk_box, center_list[i], color, i, visualization_msgs::Marker::DELETE);
  }
  visualization_->drawHgridActiveInfoByMarkerArray(mk_array);
}

void FrontierManager::visFrontierInx()
{
  if (ed_->frontiers_with_info_.empty()) return;
  vector<string> prefixs;
  vector<Vector3d> msgs;
  for (auto & i : ed_->frontiers_with_info_){
    auto p = i.average_;
    msgs.push_back(p);
    string prefix = to_string(i.id_);
    prefixs.push_back(prefix);
  }
  vis_ptr_->visualize_texts(prefixs, msgs, 1.2, "frontier_inx");
}

// void FrontierManager::visBlacklist() {
//   int i = 0;
//   for (auto & point : ed_->topo_blacklist_){
//     pcl::PointXYZI topo_p = ed_->posegraph_used_by_blacklist_cal_.getCor(point.first);
//     Eigen::Vector3d topo_pos(topo_p.x, topo_p.y, topo_p.z);
//     visualization_->drawBlacklistText(topo_pos, "X", 1, Eigen::Vector4d(1, 0, 0, 1), "blacklist_point", i, 8);
//     i++;
//   }
//   // delete id that out of range
//   for (int j = 0; j <= 20; j++)
//     visualization_->drawBlacklistText(Eigen::Vector3d(0, 0, 0), "", 1, Eigen::Vector4d(1, 0, 0, 0),
//                                       "blacklist_point", i + j, 8);
// }

}  // namespace ego_planner

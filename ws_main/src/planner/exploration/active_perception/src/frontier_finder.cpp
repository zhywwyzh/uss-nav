#include <active_perception/frontier_finder.h>
#include <plan_env/raycast.h>


// #include <path_searching/astar2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <active_perception/graph_node.h>

// use PCL region growing segmentation
// #include <pcl/point_types.h>
// #include <pcl/search/search.h>
// #include <pcl/search/kdtree.h>
// #include <pcl/features/normal_3d.h>
// #include <pcl/segmentation/region_growing.h>
#include <pcl/filters/voxel_grid.h>

#include <Eigen/Eigenvalues>
#include <memory>

namespace ego_planner {
FrontierFinder::FrontierFinder(const MapInterface::Ptr& map, ros::NodeHandle& nh, SceneGraph::Ptr scene_graph) {
  this->edt_env_ = map;
  int voxel_num = edt_env_->getVoxelNum();
  frontier_flag_ = vector<char>(voxel_num, 0);
  fill(frontier_flag_.begin(), frontier_flag_.end(), 0);

  nh.param("frontier/cluster_min", cluster_min_, -1);
  nh.param("frontier/cluster_size_xy", cluster_size_xy_, -1.0);
  nh.param("frontier/cluster_size_z", cluster_size_z_, -1.0);
  nh.param("frontier/down_sample", down_sample_, -1);
  nh.param("frontier/floor", ftr_floor_, -1.0);
  nh.param("frontier/ceil", ftr_ceil_, -1.0);

  nh.param("frontier/min_candidate_dist", min_candidate_dist_, -1.0);
  nh.param("frontier/min_candidate_clearance_xy", min_candidate_clearance_xy_, -1.0);
  nh.param("frontier/min_candidate_clearance_z", min_candidate_clearance_z_, -1.0);
  nh.param("frontier/min_candidate_clearance_unknown_xy", min_candidate_clearance_unknown_xy_, -1.0);
  nh.param("frontier/min_candidate_clearance_unknown_z", min_candidate_clearance_unknown_z_, -1.0);
  nh.param("frontier/candidate_dphi", candidate_dphi_, -1.0);
  nh.param("frontier/candidate_phinum", candidate_phinum_, -1);
  nh.param("frontier/candidate_rmax", candidate_rmax_, -1.0);
  nh.param("frontier/candidate_rmin", candidate_rmin_, -1.0);
  nh.param("frontier/candidate_rnum", candidate_rnum_, -1);
  nh.param("frontier/candidate_zmax", candidate_zmax_, 1.0);
  nh.param("frontier/candidate_zmin", candidate_zmin_, -1.0);
  nh.param("frontier/candidate_znum", candidate_znum_, 1);
  nh.param("frontier/min_visib_num", min_visib_num_, -1);
  nh.param("frontier/min_view_finish_fraction", min_view_finish_fraction_, -1.0);
  nh.param("frontier/ftr_blacklist_radius", ftr_blacklist_radius_, 0.2);
  nh.param("frontier/print_info", is_print_info_, false);

  nh.param("exploration/radius_far_goal", far_dis_thres_, -1.0);

  nh.param("posegraph/keypose_min_gap", keypose_min_gap_, -1.0);
  nh.param("posegraph/connected_min_seqential_dis", connected_min_seqential_dis_, -1.0);
  nh.param("posegraph/connected_max_euler_dis", connected_max_euler_dis_, -1.0);

  visPtr_ = std::make_shared<visualization::Visualization>(nh);

  raycaster_  = std::make_unique<RayCaster>();
  resolution_ = edt_env_->getResolution();
  INFO_MSG_YELLOW("[ftr_finder] Get resolution: " << resolution_);
  // Eigen::Vector3d origin, size;
  // edt_env_->sdf_map_->getRegion(origin, size);
  // raycaster_->setParams(resolution_, origin);

  percep_utils_ = std::make_shared<PerceptionUtils>(nh);
  scene_graph_  = scene_graph;

  vp_handler_ = std::make_shared<ViewpointsHandler>(scene_graph, map);
  // posegraph_m_  = std::make_shared<MultiPoseGraph>();
  // posegraph_m_->main_inx = 0;
  // posegraph_.reset(new PoseGraph);

  self_last_add_inx_ = -1;

  // debug
  vector<Eigen::Vector3d> free_cell;
  vector<Eigen::Vector3d> uk_cell;
  visPtr_->visualize_pointcloud(free_cell, "free_cell");
  visPtr_->visualize_pointcloud(uk_cell, "uk_cell");
}

FrontierFinder::~FrontierFinder() {
}

void FrontierFinder::reCalculateAllFtrTopo(const Eigen::Vector3d &cur_pos) {
  for (auto& ftr : frontiers_) {
    if ((ftr.viewpoints_.front().pos_ - cur_pos).norm() > 4.5) continue;

    PolyHedronPtr last_topo = ftr.topo_father_;

    ftr.topo_father_ = scene_graph_->skeleton_gen_->getFrontierTopo(ftr.viewpoints_.front().pos_);
    if (ftr.topo_father_ != nullptr) {
      // INFO_MSG_YELLOW("[reCalFtr] viewpoint : " << ftr.viewpoints_.front().pos_.transpose() << ", topo_father: " << ftr.topo_father_->center_);
    }else {
      // INFO_MSG_RED("[reCalFtr] viewpoint : " << ftr.viewpoints_.front().pos_.transpose() << ", topo_father: " << "nullptr");
      ftr.topo_father_ = last_topo;
    }
  }
}

  /**
   * @brief 在地图的更新区域内搜索并维护前沿点 (Search and update frontiers within the map's updated region).
   * @brief 本函数是前沿点探测的核心。它首先清理掉因地图更新而失效的旧前沿点，然后在更新区域内扫描，寻找已知自由空间与未知空间的边界，从而生成新的前沿点。
   * @param [Input] c_pos             输入 (Input): 机器人当前的位置，用于 `isWellObserved` 等检查。
   * @param this->frontiers_  输入/输出 (In/Out): 存储有效前沿点的列表，函数会从中移除失效的前沿点。
   * @param this->dormant_frontiers_ 输入/输出 (In/Out): 存储休眠前沿点的列表，同样会移除失效部分。
   * @param this->tmp_frontiers_     输出 (Output): 存储本次搜索发现的所有新前沿点的列表，这是函数最主要的产出。
   * @param this->edt_env_           输入 (Input): 环境地图接口，提供栅格状态、更新区域等核心数据。
   */
void FrontierFinder::searchFrontiers(const Vector3d& c_pos)
{
  if (is_print_info_) ROS_WARN_STREAM("[FtrFinder] >>>>>>>>>>>>> SearchFrontiers <<<<<<<<<<<<<");

  edt_env_->mtxLock();

  ros::Time t1 = ros::Time::now();
  tmp_frontiers_.clear();

  // Bounding box of updated region
  Vector3d update_min, update_max;
  edt_env_->getUpdatedBox(update_min, update_max);
  if (is_print_info_) ROS_WARN_STREAM("   * update_min: " << update_min.transpose() << ", update_max: " << update_max.transpose());


  // reset frontier_flag
  fill(frontier_flag_.begin(), frontier_flag_.end(), 0);
  for (auto & frontier : frontiers_){
    for (const auto& cell : frontier.cells_){
      Eigen::Vector3i idx;
      idx = edt_env_->pos2GlobalIdx(cell);
      if (edt_env_->isInLocalMap(idx)) frontier_flag_[toadr(idx)] = 1;
    }
  }

  // Removed changed frontiers in updated map
  auto resetFlag = [&](list<Frontier>::iterator& iter, list<Frontier>& frontiers) {
    for (const auto& cell : iter->cells_) {
      Eigen::Vector3i idx = edt_env_->pos2GlobalIdx(cell);
      frontier_flag_[toadr(idx)] = 0;
    }
    iter = frontiers.erase(iter);
  };

  removed_ids_.clear();
  remove_frontiers_.clear();
  int rmv_idx = 0;
  if (is_print_info_) std::cout << "   * Before remove: " << frontiers_.size() << std::endl;
  if (is_print_info_)  INFO_MSG("   * update_min: " << update_min.transpose() << ", update_max: " << update_max.transpose());

  for (auto iter = frontiers_.begin(); iter != frontiers_.end();)
  {
    if (is_print_info_)    INFO_MSG("   * iter->id: " << iter->id_ << ", avg: " << iter->average_.transpose());
    if (is_print_info_)    INFO_MSG("   * boxmin: " << iter->box_min_.transpose() << ", boxmax: " << iter->box_max_.transpose());

    auto ft = *iter;
    // 判断是否一半在localmap内一半在外,如果更新该ftr会造成一半在外一半在内，可能都因为数量不足被删掉，所以保留
    if (isHalfInLocalMap(ft))
    {
      if (is_print_info_)      INFO_MSG_BLUE("   * half in local, reserve.");
      ++rmv_idx;
      ++iter;
    }
    else
    {
      // 如果有overlap，并且frontier变化，则重置flag，并删除该frontier
      if (haveOverlap(iter->box_min_, iter->box_max_, update_min, update_max) &&
          isFrontierChanged(*iter))
      {
        // 防止老的frontier在滚动localmap的unknown区域，这时判断没有被很好地观察，保留
        if (!isWellObserved(*iter, c_pos))
        {
          if (is_print_info_)          INFO_MSG_BLUE("have overlap, but not well observed, reserve.");
          ++rmv_idx;
          ++iter;
        }
        else
        {
          if (is_print_info_)          INFO_MSG_RED("have overlap, well observed, delete frontier: " << iter->id_);
          removed_ids_.push_back(rmv_idx);
          remove_frontiers_.push_back(ft);
          resetFlag(iter, frontiers_);
        }
      }
      else
      {
        if (is_print_info_)  INFO_MSG_RED("have overlap, delete frontier: " << iter->id_);
        ++rmv_idx;
        ++iter;
      }
    }
  }
  if (is_print_info_) std::cout << "After remove: " << frontiers_.size() << std::endl;

  for (auto iter = dormant_frontiers_.begin(); iter != dormant_frontiers_.end();) {
    if (haveOverlap(iter->box_min_, iter->box_max_, update_min, update_max) &&
        isFrontierChanged(*iter))
      resetFlag(iter, dormant_frontiers_);
    else
      ++iter;
  }

  // Search new frontier within box slightly inflated from updated box
  // Vector3d search_min = update_min - Vector3d(1, 1, 0.5);
  // Vector3d search_max = update_max + Vector3d(1, 1, 0.5);
  // Vector3d box_min, box_max;
  // edt_env_->sdf_map_->getBox(box_min, box_max);
  // for (int k = 0; k < 3; ++k) {
  //   search_min[k] = max(search_min[k], box_min[k]);
  //   search_max[k] = min(search_max[k], box_max[k]);
  // }
  // edt_env_->sdf_map_->posToIndex(search_min, min_id);
  // edt_env_->sdf_map_->posToIndex(search_max, max_id);

  Eigen::Vector3i min_id, max_id;
  edt_env_->getUpdatedBoxIdx(min_id, max_id);
  if (is_print_info_) ROS_WARN_STREAM("min_id: " << min_id.transpose() << ", max_id: " << max_id.transpose());

  vector<Eigen::Vector3d> free_cell;
  vector<Eigen::Vector3d> uk_cell;
  vector<Eigen::Vector3d> seed_cell;
  vector<Eigen::Vector3d> ftr_cell;

  if (is_print_info_)
  {
    for (int x = min_id(0) - 1; x <= max_id(0) + 1; ++x)
      for (int y = min_id(1) - 1; y <= max_id(1) + 1; ++y)
        for (int z = min_id(2) - 1; z <= max_id(2) + 1; ++z)
        {
          Eigen::Vector3i cur(x, y, z);
          Eigen::Vector3d p = edt_env_->globalIdx2Pos(cur);
          if (knownfree(cur)){
            free_cell.push_back(p);
          }
          if (edt_env_->getOccupancy(cur) == MapInterface::UNKNOWN)
          {
            uk_cell.push_back(p);
          }
        }
  }

  //! frontier的更新部分必须update_range一致(或者比update_range大)，否则ftr生成的边界在update_range里，找不到unknown邻居
  for (int x = min_id(0); x <= max_id(0); ++x)
    for (int y = min_id(1); y <= max_id(1); ++y)
      for (int z = min_id(2); z <= max_id(2); ++z)
      {
        // Scanning the updated region to find seeds of frontiers
        Eigen::Vector3i cur(x, y, z);
        Eigen::Vector3d p = edt_env_->globalIdx2Pos(cur);

        if (is_print_info_ && frontier_flag_[toadr(cur)] == 1 && knownfree(cur) && isNeighborUnknown(cur)){
          ftr_cell.push_back(p);
        }

        if (frontier_flag_[toadr(cur)] == 0 && knownfree(cur) && isNeighborUnknown(cur)) {
          // Expand from the seed cell to find a complete frontier cluster
          // INFO_MSG_GREEN("   * Expand frontier from seed cell: " << cur.transpose());
          expandFrontier(cur);
        }
      }
  if (is_print_info_) ROS_INFO_STREAM("   * New Ftr Search done. ");
  splitLargeFrontiers(tmp_frontiers_);

  // Delete frontiers in blacklist
  for (auto iter = tmp_frontiers_.begin(); iter != tmp_frontiers_.end();)
  {
    if (isBlacklisted(*iter))
    {
      iter = tmp_frontiers_.erase(iter);
      continue;
    }
    else
    {
      ++iter;
    }
  }

  vector<string> prefixs(ftr_blacklist_.size(), "X");
  visPtr_->visualize_texts(prefixs, ftr_blacklist_, 0.6, "ftr_blacklist");

  visPtr_->visualize_pointcloud(free_cell, "free_cell");
  visPtr_->visualize_pointcloud(uk_cell, "uk_cell");
  visPtr_->visualize_pointcloud(ftr_cell, "ftr_cell_raw");


  if (is_print_info_) ROS_WARN_STREAM("Frontier time consume: " << (ros::Time::now() - t1).toSec());
  edt_env_->mtxUnlock();
}
/**
 * @brief 扩展边界簇
 *
 * 该函数基于区域增长算法（距离聚类）来搜索边界簇。它从给定的第一个单元格开始，逐步扩展到相邻的单元格，直到无法找到符合条件的相邻单元格为止。
 * 符合条件的单元格必须位于局部地图和全局地图的边界框内，并且该单元格未被聚类过，同时其周围存在未知状态的单元格。
 * 此外，符合条件的单元格的高度值应在其预设的范围内（ftr_floor_到ftr_ceil_），以去除接近地面和过高的噪声。
 * 最终，当扩展的边界单元格数量超过最小阈值（cluster_min_）时，计算这些单元格的详细信息，并将它们作为边界簇存储到临时边界列表中。
 *
 * @param first 作为边界搜索起点的第一个单元格的全局索引（Eigen::Vector3i类型）
 *
 * @return void
 */
void FrontierFinder::expandFrontier(
    const Eigen::Vector3i& first /* , const int& depth, const int& parent_id */) {
  // std::cout << "depth: " << depth << std::endl;
  auto t1 = ros::Time::now();

  // Data for clustering
  queue<Eigen::Vector3i> cell_queue;
  vector<Eigen::Vector3d> expanded;
  Vector3d pos = edt_env_->globalIdx2Pos(first);

  expanded.push_back(pos);
  cell_queue.push(first);
  frontier_flag_[toadr(first)] = 1;

  // Search frontier cluster based on region growing (distance clustering)
  while (!cell_queue.empty()) {
    auto cur = cell_queue.front();
    cell_queue.pop();
    auto nbrs = allNeighbors(cur);
    for (auto nbr : nbrs) {
      // Qualified cell should be inside bounding box and frontier cell not clustered
      // ROS_INFO_STREAM("nbr: " << nbr.transpose());
      int adr = toadr(nbr);
      // ROS_INFO_STREAM("frontier_flag_: " << (int)frontier_flag_[adr]);
      // ROS_INFO_STREAM("knownfree: " << knownfree(nbr));
      // ROS_INFO_STREAM("isNeighborUnknown: " << isNeighborUnknown(nbr));
      if (frontier_flag_[adr] == 1 || !edt_env_->isInLocalMap(nbr) || !edt_env_->isInGlobalMap(nbr) ||
          !(knownfree(nbr) && isNeighborUnknown(nbr)))
        continue;

      pos = edt_env_->globalIdx2Pos(nbr);
      if (pos[2] < ftr_floor_ || pos[2] > ftr_ceil_) continue;  //TODO Remove noise close to ground & remove too high
      //ROS_INFO_STREAM("!!nbr: " << nbr.transpose() << " ,p :" << pos.transpose());
      expanded.push_back(pos);
      cell_queue.push(nbr);
      frontier_flag_[adr] = 1;
    }
  }
  if (static_cast<int>(expanded.size()) > cluster_min_)
  {
    // Compute detailed info
    Frontier frontier;
    frontier.cells_ = expanded;
    computeFrontierInfo(frontier);
    tmp_frontiers_.push_back(frontier);
  }
}

void FrontierFinder::splitLargeFrontiers(list<Frontier>& frontiers) {
  list<Frontier> splits, tmps;
  for (auto & frontier : frontiers) {
    // Check if each frontier needs to be split horizontally
    if (splitHorizontally(frontier, splits)) {
      tmps.insert(tmps.end(), splits.begin(), splits.end());
      splits.clear();
    } else
      tmps.push_back(frontier);
  }
  frontiers = tmps;
}

bool FrontierFinder::splitHorizontally(const Frontier& frontier, list<Frontier>& splits) {
  // Split a frontier into small piece if it is too large
  auto mean = frontier.average_.head<2>();
  bool need_split = false;
  for (auto cell : frontier.filtered_cells_) {
    if ((cell.head<2>() - mean).norm() > cluster_size_xy_) {
      need_split = true;
      break;
    }
  }
  if (!need_split) return false;

  // Compute principal component
  // Covariance matrix of cells
  Eigen::Matrix2d cov;
  cov.setZero();
  for (auto cell : frontier.filtered_cells_) {
    Eigen::Vector2d diff = cell.head<2>() - mean;
    cov += diff * diff.transpose();
  }
  cov /= double(frontier.filtered_cells_.size());

  // Find eigenvector corresponds to maximal eigenvector
  Eigen::EigenSolver<Eigen::Matrix2d> es(cov);
  auto values = es.eigenvalues().real();
  auto vectors = es.eigenvectors().real();
  int max_idx;
  double max_eigenvalue = -1000000;
  for (int i = 0; i < values.rows(); ++i) {
    if (values[i] > max_eigenvalue) {
      max_idx = i;
      max_eigenvalue = values[i];
    }
  }
  Eigen::Vector2d first_pc = vectors.col(max_idx);
  // std::cout << "max idx: " << max_idx << std::endl;
  // std::cout << "mean: " << mean.transpose() << ", first pc: " << first_pc.transpose() << std::endl;

  // Split the frontier into two groups along the first PC
  Frontier ftr1, ftr2;
  for (auto cell : frontier.cells_) {
    if ((cell.head<2>() - mean).dot(first_pc) >= 0)
      ftr1.cells_.push_back(cell);
    else
      ftr2.cells_.push_back(cell);
  }
  computeFrontierInfo(ftr1);
  computeFrontierInfo(ftr2);

  // Recursive call to split frontier that is still too large
  list<Frontier> splits2;
  if (splitHorizontally(ftr1, splits2)) {
    splits.insert(splits.end(), splits2.begin(), splits2.end());
    splits2.clear();
  } else
    splits.push_back(ftr1);

  if (splitHorizontally(ftr2, splits2))
    splits.insert(splits.end(), splits2.begin(), splits2.end());
  else
    splits.push_back(ftr2);

  return true;
}

void FrontierFinder::updateFrontierCostMatrix() {
  // std::cout << "cost mat size before remove: " << std::endl;
  // for (auto ftr : frontiers_)
  //   std::cout << "(" << ftr.costs_.size() << "," << ftr.paths_.size() << "), ";
  // std::cout << "" << std::endl;

// [gwq] 在地面还未起飞时，飞机有可能被识别在障碍物里，因此此时需要暂时停止frontier有关的topo路径搜索
//  int max_topo_idx = -1;
//  for (auto & frontier : frontiers_)
//    max_topo_idx = max(max_topo_idx, frontier.keypose_inx_);
//  if (max_topo_idx >= posegraph_->getSize()){
//    return ;
//  }
  if (is_print_info_) INFO_MSG_YELLOW("[Ftr] Update Frontier Cost Matrix ... ");

  if (!removed_ids_.empty()) {
    // Delete path and cost for removed clusters
    for (auto it = frontiers_.begin(); it != first_new_ftr_; ++it) {
      auto cost_iter = it->costs_.begin();
      auto path_iter = it->paths_.begin();
      int iter_idx = 0;
      for (size_t i = 0; i < removed_ids_.size(); ++i) {
        // Step iterator to the item to be removed
        while (iter_idx < removed_ids_[i]) {
          ++cost_iter;
          ++path_iter;
          ++iter_idx;
        }
        cost_iter = it->costs_.erase(cost_iter);
        path_iter = it->paths_.erase(path_iter);
      }
      // std::cout << "(" << it->costs_.size() << "," << it->paths_.size() << "), ";
    }
    removed_ids_.clear();
  }
  auto updateCost = [this](list<Frontier>::iterator& it1, list<Frontier>::iterator& it2) {
    // std::cout << "(" << it1->id_ << "->" << it2->id_ << "), ";
    // Search path from old cluster's top viewpoint to new cluster'
    Viewpoint& vui = it1->viewpoints_.front();
    Viewpoint& vuj = it2->viewpoints_.front();
    vector<Vector3d> path_ij;
    double dis_ij = -1.0;
    bool a_star_success = false;
    if (edt_env_->isVisible(vui.pos_, vuj.pos_, 0.0) && (vui.pos_ - vuj.pos_).norm() < this->far_dis_thres_)
    {
      a_star_success = ViewNode::searchPath(vui.pos_, vuj.pos_, path_ij, dis_ij);
    }
    if (dis_ij < 0.0 || !a_star_success)
    {
      if (it1->topo_father_ == nullptr || it2->topo_father_ == nullptr ){
        dis_ij = 9999;
        path_ij.push_back(vui.pos_);
        path_ij.push_back(vuj.pos_);
      }else {
        if (is_print_info_) {
          INFO_MSG("   * vui pos: " << vui.pos_.transpose() << " , vuj pos: " << vuj.pos_.transpose());
          INFO_MSG("   * vui topo inx: " << it1->topo_father_->center_ << " , vuj topo inx: " << it2->topo_father_->center_);
        }
        getPathWithTopo(it1->topo_father_, vui.pos_, it2->topo_father_, vuj.pos_, path_ij, dis_ij);
      }
    }
    // std::cout << "Dis [" << dis_ij << "] ";
    // for (auto p : path_ij){
    //   std::cout << p.head(2).transpose() << ", ";
    // }
    // std::cout << std::endl;

    // Cost of position change
    double pos_cost = dis_ij / ViewNode::vm_;
    // Cost of yaw change
    double diff = fabs(vuj.yaw_ - vui.yaw_);
    diff = min(diff, 2 * M_PI - diff);
    double yaw_cost = diff / ViewNode::yd_;
    double cost_ij = max(pos_cost, yaw_cost);

    // Insert item for both old and new clusters
    it1->costs_.push_back(cost_ij);
    it1->paths_.push_back(path_ij);
    reverse(path_ij.begin(), path_ij.end());
    it2->costs_.push_back(cost_ij);
    it2->paths_.push_back(path_ij);
  };

  // std::cout << "cost mat add: (id, id)" << std::endl;
  // Compute path and cost between old and new clusters
  for (auto it1 = frontiers_.begin(); it1 != first_new_ftr_; ++it1)
    for (auto it2 = first_new_ftr_; it2 != frontiers_.end(); ++it2)
      updateCost(it1, it2);
  // Compute path and cost between new clusters
  for (auto it1 = first_new_ftr_; it1 != frontiers_.end(); ++it1)
    for (auto it2 = it1; it2 != frontiers_.end(); ++it2) {
      if (it1 == it2) {
        // std::cout << "(" << it1->id_ << "," << it2->id_ << "), ";
        it1->costs_.push_back(0);
        it1->paths_.emplace_back();
      } else
        updateCost(it1, it2);
    }


  // Compute path to home for all clusters
  // cout<<"posegraph_m_->getPoseGraphSize(0):"<<posegraph_m_->getPoseGraphSize(0)<<endl;
  if (scene_graph_->skeleton_gen_->ready()){
    for (auto it = frontiers_.begin(); it != frontiers_.end(); ++it){
      PolyHedronPtr it_topo_father = scene_graph_->skeleton_gen_->mountCurTopoPoint(it->average_, true);
      if (it->topo_father_ == nullptr) {
        it->cost_to_home_ = 9999;
        it->path_to_home_.emplace_back(it->average_.x(), it->average_.y(), it->average_.z(), 0.0);
        it->path_to_home_.emplace_back(0.0, 0.0, 0.0, 0.0);
      }
      else {
        // [gwq] TODO 此处对于边界的HOME cost处理与源代码不同，可能会有bug, 由于skeleton包具有天然防止相同连接的功能，因此下面的 *判定* 注释掉了
        std::vector<Eigen::Vector3d> path;
        it->cost_to_home_ = scene_graph_->skeleton_gen_->astarSearch(it->average_, Eigen::Vector3d(0.0, 0.0, 0.0), path, true) / ViewNode::vm_;
        it->path_to_home_.clear();
        for (const auto & p : path)
          it->path_to_home_.emplace_back(p.x(), p.y(), p.z(), 0.0);
        // it->cost_to_home_ = posegraph_m_->calGraphPath(make_pair(0, it->keypose_inx_), make_pair(0, 0), it->path_to_home_) / ViewNode::vm_;
      }

      // handle the situation that home and frontier are attached to the same keypose
      // if (it->cost_to_home_ <= 0.0){
      //   auto home_p = posegraph_m_->posegraphes[0].getCor(0);
      //   Eigen::Vector4d home_pt(home_p.x, home_p.y, home_p.z, home_p.intensity);
      //   it->cost_to_home_ = (home_pt.head(3) - it->average_).norm() / ViewNode::vm_;
      //   it->path_to_home_.clear();
      //   it->path_to_home_.emplace_back(it->average_.x(), it->average_.y(), it->average_.z(), home_p.intensity);
      //   it->path_to_home_.push_back(home_pt);
      // }
      // it->cost_to_home_ = shortenPathIteration(path, it->path_to_home_, 4, 5, 3) / ViewNode::vm_;
      ROS_ASSERT(!it->path_to_home_.empty());
    }
  }

  if (is_print_info_)
  {
    std::cout << "" << std::endl;
    std::cout << "cost mat size final: " << std::endl;
    for (const auto& ftr : frontiers_)
      std::cout << "(" << ftr.costs_.size() << "," << ftr.paths_.size() << "), ";
    std::cout << "" << std::endl;
  }
}

void FrontierFinder::updateFrontierCostMatrix(list<Frontier> &frontiers)
{
  // std::cout << "cost mat size before remove: " << std::endl;
  // for (auto ftr : frontiers_)
  //   std::cout << "(" << ftr.costs_.size() << "," << ftr.paths_.size() << "), ";
  // std::cout << "" << std::endl;
  for (auto & ftr : frontiers) {
    ftr.costs_.clear();
    ftr.paths_.clear();
    ftr.path_to_home_.clear();
    ftr.cost_to_home_ = 0.0;
  }
  auto updateCost = [this](list<Frontier>::iterator& it1, list<Frontier>::iterator& it2) {
    // std::cout << "(" << it1->id_ << "->" << it2->id_ << "), ";
    // Search path from old cluster's top viewpoint to new cluster'
    Viewpoint& vui = it1->viewpoints_.front();
    Viewpoint& vuj = it2->viewpoints_.front();
    vector<Vector3d> path_ij;
    double dis_ij = -1.0;
    // 均使用topo搜索路径
    if (it1->topo_father_ == nullptr || it2->topo_father_ == nullptr )
    {
      ROS_ERROR_STREAM("[FrontierFinder] : topo search failed, keypose inx out of range");
      dis_ij = 9999;
      path_ij.push_back(vui.pos_);
      path_ij.push_back(vuj.pos_);
    }
    else
    {
      getTopoPath(vui.pos_, vuj.pos_, path_ij, dis_ij);
    }

    // std::cout << "Dis [" << dis_ij << "] ";
    // for (auto p : path_ij){
    //   std::cout << p.head(2).transpose() << ", ";
    // }
    // std::cout << std::endl;

    // Cost of position change
    double pos_cost = dis_ij / ViewNode::vm_;
    // Cost of yaw change
    double diff = fabs(vuj.yaw_ - vui.yaw_);
    diff = min(diff, 2 * M_PI - diff);
    double yaw_cost = diff / ViewNode::yd_;
    double cost_ij = max(pos_cost, yaw_cost);

    // Insert item for both old and new clusters
    it1->costs_.push_back(cost_ij);
    it1->paths_.push_back(path_ij);
    reverse(path_ij.begin(), path_ij.end());
    it2->costs_.push_back(cost_ij);
    it2->paths_.push_back(path_ij);
  };

  // Compute path and cost between all clusters
  for (auto it1 = frontiers.begin(); it1 != frontiers.end(); ++it1)
    for (auto it2 = it1; it2 != frontiers.end(); ++it2) {
      if (it1 == it2) {
         std::cout << "(" << it1->id_ << "," << it2->id_ << "), ";
        it1->costs_.push_back(0);
        it1->paths_.push_back({});
      } else{
//        std::cout << "(" << it1->id_ << "," << it2->id_ << "), " << std::endl;
        updateCost(it1, it2);
      }
    }


  // Compute path to home for all clusters
  // cout<<"posegraph_m_->getPoseGraphSize(0):"<<posegraph_m_->getPoseGraphSize(0)<<endl;
  if(scene_graph_->skeleton_gen_->ready()){
    for (auto it = frontiers.begin(); it != frontiers.end(); ++it){
      if (it->topo_father_ == nullptr)
        it->cost_to_home_ = 9999;
      else {
        std::vector<Eigen::Vector3d> path;
        it->path_to_home_.clear();
        it->cost_to_home_ = scene_graph_->skeleton_gen_->astarSearch(it->average_, Eigen::Vector3d(0.0, 0.0, 0.0), path, true) / ViewNode::vm_;
        for (const auto & p : path) it->path_to_home_.emplace_back(p.x(), p.y(), p.z(), 0.0);
        if (it->cost_to_home_ <= 0.0) it->cost_to_home_ = (it->average_ - Eigen::Vector3d(0.0, 0.0, 0.0)).norm() / ViewNode::vm_;
        // it->cost_to_home_ = posegraph_m_->calGraphPath(make_pair(0, it->keypose_inx_), make_pair(0, 0), it->path_to_home_) / ViewNode::vm_;
      }
      // handle the situation that home and frontier are attached to the same keypose
      // if (it->cost_to_home_ <= 0.0){
      //   auto home_p = posegraph_m_->posegraphes[0].getCor(0);
      //   Eigen::Vector4d home_pt(home_p.x, home_p.y, home_p.z, home_p.intensity);
      //   it->cost_to_home_ = (home_pt.head(3) - it->average_).norm() / ViewNode::vm_;
      //   it->path_to_home_.clear();
      //   it->path_to_home_.emplace_back(it->average_.x(), it->average_.y(), it->average_.z(), home_p.intensity);
      //   it->path_to_home_.push_back(home_pt);
      // }
      // it->cost_to_home_ = shortenPathIteration(path, it->path_to_home_, 4, 5, 3) / ViewNode::vm_;
      ROS_ASSERT(!it->path_to_home_.empty());
    }
  }

  if (is_print_info_)
  {
    std::cout << "" << std::endl;
    std::cout << "cost mat size final: " << std::endl;
    for (auto ftr : frontiers)
      std::cout << "(" << ftr.costs_.size() << "," << ftr.paths_.size() << "), ";
    std::cout << "" << std::endl;
  }
  INFO_MSG_GREEN("[ftrFinder]: recalculate frontier cost matrix Done");
}

void FrontierFinder::updateSceneGraphWithFtr() {
  for (auto area : scene_graph_->skeleton_gen_->area_handler_->area_map_) area.second->num_ftrs_ = 0;
  for (const auto& ftr: frontiers_) {
    auto it = scene_graph_->skeleton_gen_->area_handler_->area_map_.find(ftr.topo_father_->area_id_);
    if (it != scene_graph_->skeleton_gen_->area_handler_->area_map_.end()) it->second->num_ftrs_ ++;
  }
}


/**
 * @brief copy data from posegraph_ to posegraph_m_->posegraphes[0]
 */
// funtion updateMultiPoseGraph() has been replaced by skeleton library

// The result path contains the start & end points
void FrontierFinder::getTopoPath(const Vector3d& start_pose, const Vector3d& end_pose, vector<Vector3d>& path, double& dis)
{
  path.clear();

  // If the svp is directly visible and within the sensor_range
  if ((start_pose - end_pose).norm() < 2.0 * percep_utils_->getSensorMaxDist() &&
      edt_env_->isVisible(start_pose, end_pose)){
    path.push_back(start_pose);
    path.push_back(end_pose);
    dis = (start_pose - end_pose).norm();
  }
  // Else calculate from posegraph
  else{
    INFO_MSG("   ↓↓↓ skeleton astar search ↓↓↓");
    dis = scene_graph_->skeleton_gen_->astarSearch(start_pose, end_pose, path, true);
  }
}

void FrontierFinder::getPathWithTopo(const PolyHedronPtr &start_poly, const Eigen::Vector3d &start_pose,
                                 const PolyHedronPtr &end_poly, const Eigen::Vector3d &end_pose, vector<Eigen::Vector3d> &path, double &dis) {
  path.clear();
  dis = 0.0f;
  if (start_poly == nullptr || end_poly == nullptr) {
    INFO_MSG_RED("[FrontierFinder] : getTopoPath failed, topo father is null");
    return ;
  }
  // If the svp is directly visible and within the sensor_range
  if ((start_pose - end_pose).norm() < 2.0 * percep_utils_->getSensorMaxDist() &&
      edt_env_->isVisible(start_pose, end_pose) || (start_poly == end_poly)){
    path.push_back(start_pose);
    path.push_back(end_pose);
    dis = (start_pose - end_pose).norm();
  }
  else {
    dis = scene_graph_->skeleton_gen_->astarSearch(start_poly, end_poly, path);
    if ((path.front() - start_pose).norm() > 0.1) path.insert(path.begin(), start_pose), dis += (start_pose - path.front()).norm();
    if ((path.back() - end_pose).norm() > 0.1) path.push_back(end_pose), dis += (end_pose - path.back()).norm();
  }
}

bool FrontierFinder::canBeMerged(const Frontier& ftr1, const Frontier& ftr2) {
  Vector3d merged_avg =
      (ftr1.average_ * double(ftr1.cells_.size()) + ftr2.average_ * double(ftr2.cells_.size())) /
      (double(ftr1.cells_.size() + ftr2.cells_.size()));
  // Check if it can merge two frontier without exceeding size limit
  for (auto c1 : ftr1.cells_) {
    auto diff = c1 - merged_avg;
    if (diff.head<2>().norm() > cluster_size_xy_ || diff[2] > cluster_size_z_) return false;
  }
  for (auto c2 : ftr2.cells_) {
    auto diff = c2 - merged_avg;
    if (diff.head<2>().norm() > cluster_size_xy_ || diff[2] > cluster_size_z_) return false;
  }
  return true;
}

bool FrontierFinder::haveOverlap(
    const Vector3d& min1, const Vector3d& max1, const Vector3d& min2, const Vector3d& max2) {
  // Check if two box have overlap part
  Vector3d bmin, bmax;
  for (int i = 0; i < 3; ++i) {
    bmin[i] = max(min1[i], min2[i]);
    bmax[i] = min(max1[i], max2[i]);
    if (bmin[i] > bmax[i] + 1e-3) return false;
  }
  return true;
}

bool FrontierFinder::isFrontierChanged(const Frontier& ft) {
  for (auto cell : ft.cells_) {
    Eigen::Vector3i idx;
    // edt_env_->sdf_map_->posToIndex(cell, idx);
    idx = edt_env_->pos2GlobalIdx(cell);
    if (!(knownfree(idx) && isNeighborUnknown(idx))) return true;
  }
  return false;
}

bool FrontierFinder::isHalfInLocalMap(const Frontier& ft) 
{
    Eigen::Vector3d box_min_fix = ft.box_min_ - Vector3d(1.0, 1.0, 0.0) * edt_env_->getResolution();
    Eigen::Vector3d box_max_fix = ft.box_max_ + Vector3d(1.0, 1.0, 0.0) * edt_env_->getResolution(); // 如果ftr刚好在localmap边缘则可能没有unknown邻居，所以至少向内扩一格

    // 判断是否一半在localmap内一半在外,如果更新该ftr会造成一半在外一半在内，可能都因为数量不足被删掉，所以保留
    if (( edt_env_->isInLocalMap(box_min_fix) && !edt_env_->isInLocalMap(box_max_fix)) ||
        (!edt_env_->isInLocalMap(box_min_fix) &&  edt_env_->isInLocalMap(box_max_fix)))
    {
      return true;
    }
    else
    {
      return false;
    }
}

bool FrontierFinder::isWellObserved(const Frontier& ft, const Vector3d& pos) 
{
  int well_observed_count = 0;
  for (auto cell : ft.cells_) 
  {
    Eigen::Vector3i idx;
    // edt_env_->sdf_map_->posToIndex(cell, idx);
    idx = edt_env_->pos2GlobalIdx(cell);
    if (edt_env_->getOccupancy(idx) != MapInterface::UNKNOWN && edt_env_->isInLocalMap(idx))
    {
      well_observed_count++;
    }
  }
  if (well_observed_count > 0.8 * ft.cells_.size())
    return true;
  INFO_MSG("well_observed_count: " << well_observed_count << " | thres(20%): " << 0.2 * ft.cells_.size());
  if (well_observed_count > 0.2 * ft.cells_.size())
  {
    for (auto vp : ft.viewpoints_)
    {
      if ((vp.pos_ - pos).norm() < 0.5) return true; // 观测点距离当前位置小于0.5m，认为是well observed
    }
  }
  
  return false;
}

void FrontierFinder::computeFrontierInfo(Frontier& ftr) {
  // Compute average position and bounding box of cluster
  ftr.average_.setZero();
  ftr.box_max_ = ftr.cells_.front();
  ftr.box_min_ = ftr.cells_.front();
  ftr.topo_father_ = cur_mount_topo_;

  for (auto cell : ftr.cells_) {
    ftr.average_ += cell;
    for (int i = 0; i < 3; ++i) {
      ftr.box_min_[i] = min(ftr.box_min_[i], cell[i]);
      ftr.box_max_[i] = max(ftr.box_max_[i], cell[i]);
    }
  }
  ftr.average_ /= double(ftr.cells_.size());
  computeNormal(ftr);

  // Compute downsampled cluster
  downsample(ftr.cells_, ftr.filtered_cells_);
}

void FrontierFinder::computeNormal(Frontier& ftr){
  Eigen::MatrixXd X;
  X.resize(3, ftr.cells_.size());
  for(size_t i = 0; i < ftr.cells_.size(); i++){
      X.col(i) = (ftr.cells_[i] - ftr.average_);
  }

	Eigen::MatrixXd C = X * X.transpose();
	C = C / (X.cols());

	Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(C);
	Eigen::MatrixXd vec = eig.eigenvectors();
	Eigen::MatrixXd val = eig.eigenvalues();
  Eigen::MatrixXf::Index evalsMin;
  val.rowwise().sum().minCoeff(&evalsMin);//得到最小特征值的位置
  Eigen::Vector3d q;
  q << vec(0, evalsMin), vec(1, evalsMin), vec(2, evalsMin);//得到对应特征向量
  ftr.normal_ = q.normalized();
  
  double l = candidate_rmin_;
  Vector3d tpos1, tpos2;
  bool tpos1_free = false, tpos2_free = false;

  // ROS_WARN_STREAM("!!!!!!!!!!!!!!!!ftr.average_: " << ftr.average_.transpose() << ", ftr.normal_: " << ftr.normal_.transpose());
  
  if (abs(ftr.normal_.z()) > 0.8)
  {
    // ROS_WARN_STREAM("ftr.average_: " << ftr.average_.transpose() << ", ftr.normal_: " << -ftr.normal_.transpose());
    bool get_normal = false;
    for (double dphi = 0.0; dphi < 2 * M_PI; dphi += M_PI / 6.0)
    {
      Vector3d sample_pos = ftr.average_ + percep_utils_->getSensorMaxDist() / 2.0 * Vector3d(cos(dphi), sin(dphi), 0);
      if (edt_env_->getOccupancy(sample_pos) == MapInterface::FREE || (edt_env_->isInGlobalMap(sample_pos) && hgrid_->isCovered(sample_pos)))
      {
        ftr.normal_ = Vector3d(cos(dphi), sin(dphi), 0);
        get_normal = true;
        break;
      }
    }
    if (!get_normal) ROS_ERROR_STREAM("[ftr_finder] computeNormal error! ");
    // ROS_WARN_STREAM("sample ftr.average_: " << ftr.average_.transpose() << ", ftr.normal_: " << ftr.normal_.transpose());
    return;
  }


  // TODO sometimes the normal is along z-axis, should handle
  do{
    tpos1      = ftr.average_ + l * ftr.normal_;
    tpos2      = ftr.average_ - l * ftr.normal_;
    tpos1_free = (edt_env_->getOccupancy(tpos1) == MapInterface::FREE ||
                  hgrid_->isCovered(tpos1));
    tpos2_free = (edt_env_->getOccupancy(tpos2) == MapInterface::FREE || 
                  hgrid_->isCovered(tpos2));

    // ROS_INFO_STREAM("l: " << l << ", tpos1 [" <<  tpos1_free <<   "]: " << tpos1.transpose() << " -> " << edt_env_->getOccupancy(tpos1) << 
    // ", tpos2 [" << tpos2_free <<   "]: " << tpos2.transpose() << " -> " << edt_env_->getOccupancy(tpos2));
    l += (candidate_rmax_ - candidate_rmin_) / candidate_rnum_;
  }while((tpos1_free == tpos2_free) && l <= candidate_rmax_);
  
  if (!tpos1_free)
  {
    // ROS_WARN_STREAM("!tpos1_free ftr.average_: " << ftr.average_.transpose() << ", ftr.normal_: " << -ftr.normal_.transpose());
    ftr.normal_ = -1.0*ftr.normal_;
    if (!tpos2_free)
    {
      // ROS_WARN_STREAM("!tpos2_free ftr.average_: " << ftr.average_.transpose() << ", ftr.normal_: " << -ftr.normal_.transpose());
      bool get_normal = false;
      for (double dphi = 0.0; dphi < 2 * M_PI; dphi += M_PI / 6.0)
      {
        Vector3d sample_pos = ftr.average_ + percep_utils_->getSensorMaxDist() / 2.0 * Vector3d(cos(dphi), sin(dphi), 0);
        if (edt_env_->getOccupancy(sample_pos) == MapInterface::FREE || (edt_env_->isInGlobalMap(sample_pos) && hgrid_->isCovered(sample_pos)))
        {
          ftr.normal_ = Vector3d(cos(dphi), sin(dphi), 0);
          get_normal = true;
          break;
        }
      }
      if (!get_normal) ROS_ERROR_STREAM("[ftr_finder] computeNormal error! ");
      // ROS_WARN_STREAM("sample ftr.average_: " << ftr.average_.transpose() << ", ftr.normal_: " << -ftr.normal_.transpose());
    }
  }

}

void FrontierFinder::computeFrontiersToVisit(const Vector3d& c_pos) {
  first_new_ftr_ = frontiers_.end();
  int new_num = 0;
  int new_dormant_num = 0;
  // Try find viewpoints for each cluster and categorize them according to viewpoint number
  if (is_print_info_) {
    INFO_MSG_YELLOW("[Ftr] Compute frontiers to visit at pos: " << c_pos.transpose());
    INFO_MSG_YELLOW("   *tmp_frontiers_ size: " << tmp_frontiers_.size());
  }
  for (auto& tmp_ftr : tmp_frontiers_) 
  {
    // check if the ftr is directly attach to c_pos, avoid observing through seam
    // if (!checkFrontiers(tmp_ftr, c_pos)) 
    // {
    //   ROS_ERROR_STREAM("[ftr_finder] Delete ftr seen from seam: " << tmp_ftr.average_.transpose());
    //   continue;
    // }
    
    // Search viewpoints around frontier
    sampleViewpoints(tmp_ftr);
    if (!tmp_ftr.viewpoints_.empty())
    {
      ++new_num;
      list<Frontier>::iterator inserted = frontiers_.insert(frontiers_.end(), tmp_ftr);
      // Sort the viewpoints by coverage fraction, best view in front
      sort( inserted->viewpoints_.begin(), inserted->viewpoints_.end(),
          [](const Viewpoint& v1, const Viewpoint& v2) { return v1.visib_num_ > v2.visib_num_; });

      inserted->topo_father_ = scene_graph_->skeleton_gen_->getFrontierTopo(inserted->viewpoints_.front().pos_);
      if (inserted->topo_father_ != nullptr) {
        // INFO_MSG_YELLOW("[1stCalFtr] viewpoint : " << inserted->viewpoints_.front().pos_.transpose() << ", topo_father: " << inserted->topo_father_->center_);
      }else {
        // INFO_MSG_RED("[1stCalFtr] viewpoint : " << inserted->viewpoints_.front().pos_.transpose() << ", topo_father: " << "nullptr");
        inserted->topo_father_ = cur_mount_topo_;
      }
      if (first_new_ftr_ == frontiers_.end()) first_new_ftr_ = inserted;

      // vp_handler_->addViewpoint(inserted->viewpoints_.front(), scene_graph_->getCurPoly());

      // reassign keypose_inx_ if there is nearby frontier
      // Frontier ftr_near;
      // if (getNearRemovedFtr(*inserted, cluster_size_xy_ / 2.0, ftr_near))
      // {
      //   // check visible from current toponode (Only check when has nearby frontier)
      //   if (!canDirectArrive(c_pos, inserted->average_))
      //   // if (!edt_env_->isVisible(c_pos, inserted->average_))
      //   {
      //     inserted->keypose_inx_ = ftr_near.keypose_inx_;
      //   }
      // }
      // inserted->id_ = res_id;
    } 
    else 
    {
      INFO_MSG_RED("   Find no viewpoint avg: " << tmp_ftr.average_.transpose() << ", normal: " << tmp_ftr.normal_.transpose());
      // Find no viewpoint, move cluster to dormant list
      dormant_frontiers_.push_back(tmp_ftr);
      ++new_dormant_num;
    }
  }
  // Reset indices of frontiers
  int idx = 0;
  for (auto& ft : frontiers_)
    ft.id_ = idx++;
  if (is_print_info_)
  {
    std::cout << "\n   * new num: " << new_num << ", new dormant: " << new_dormant_num << std::endl;
    std::cout << "   * to visit: " << frontiers_.size() << ", dormant: " << dormant_frontiers_.size()
              << std::endl;
  }

}

bool FrontierFinder::checkFrontiers(Frontier& frontier, const Vector3d& c_pos)
{
  // Find a free ftr cell
  Eigen::Vector3d ftr_cell_c;
  bool founded = false;
  for (auto cell : frontier.filtered_cells_)
  {
    if (edt_env_->getInflateOccupancy(cell) == MapInterface::FREE)
    {
      ftr_cell_c = cell;
      founded = true;
      break;
    }
  }
  if (!founded)
  {
    ROS_ERROR_STREAM("[ftr_finder] checkFrontiers error! No free ftr cell!");
    return false;
  }

  if (edt_env_->isVisible(c_pos, ftr_cell_c, 0.0))
  {
    return true;
  }
  else
  {
    vector<Vector3d> path;
    double dis;
    bool a_star_success = ViewNode::searchPath(c_pos, ftr_cell_c, path, dis);
    double ideal_dis = (c_pos - frontier.average_).norm() + (ftr_cell_c - frontier.average_).norm();
    INFO_MSG("ideal dis to goal: " << ideal_dis << ", a* path length: " << dis << " | " << 1.3 * ideal_dis);

    if (!a_star_success || dis > 1.3 * ideal_dis) //TODO magic number 1.3
      return false;
  }
  return true;
}

void FrontierFinder::getViewpointsInfo(
    const Vector3d& cur_pos, const vector<int>& ids, const int& view_num, const double& max_decay,
    vector<vector<Eigen::Vector3d>>& points, vector<vector<double>>& yaws) 
{
  points.clear();
  yaws.clear();
  for (auto id : ids) {
    // Scan all frontiers to find one with the same id
    for (auto frontier : frontiers_) {
      if (frontier.id_ == id) {
        // Get several top viewpoints that are far enough
        vector<Eigen::Vector3d> pts;
        vector<double> ys;
        int visib_thresh = frontier.viewpoints_.front().visib_num_ * max_decay;
        for (auto view : frontier.viewpoints_) {
          if ((int)pts.size() >= view_num || view.visib_num_ <= visib_thresh) break;
          if ((view.pos_ - cur_pos).norm() < min_candidate_dist_) continue;
          pts.push_back(view.pos_);
          ys.push_back(view.yaw_);
        }
        if (pts.empty()) {
          // All viewpoints are very close, ignore the distance limit
          for (auto view : frontier.viewpoints_) {
            if ((int)pts.size() >= view_num || view.visib_num_ <= visib_thresh) break;
            pts.push_back(view.pos_);
            ys.push_back(view.yaw_);
          }
        }
        points.push_back(pts);
         yaws.push_back(ys);
      }
    }
  }
}

void FrontierFinder::getFrontiers(vector<vector<Eigen::Vector3d>>& clusters) {
  clusters.clear();
  for (auto frontier : frontiers_)
    // clusters.push_back(frontier.cells_);
    clusters.push_back(frontier.filtered_cells_);
}

void FrontierFinder::frontierForceDelete(const Frontier &frontier_to_delete) {
  std::cout << "\033[31m" << "[FrontierFinder]: force delete frontier avg_pos:(" << frontier_to_delete.average_.x()
                          << ", " << frontier_to_delete.average_.y() << ", " << frontier_to_delete.average_.z() << ") id:"
                          <<frontier_to_delete.id_<< "\033[0m" << std::endl;
  std::cout << "frontiers_size: " << frontiers_.size() << std::endl;
  for (auto iter = frontiers_.begin(); iter != frontiers_.end();)
  {
    if((iter->average_ - frontier_to_delete.average_).norm() < 0.05)
    {
      for (auto iter_other_frontier = frontiers_.begin(); iter_other_frontier != frontiers_.end(); iter_other_frontier ++)
      {
        if (iter == iter_other_frontier) continue;
        // delete cost info between deleted frontier and other frontiers

        try{
          auto iter_cost = iter_other_frontier->costs_.begin();
          std::advance(iter_cost, iter->id_);
          iter_other_frontier->costs_.erase(iter_cost);
          // delete path between deleted frontier and other frontiers
          auto iter_path = iter_other_frontier->paths_.begin();
          std::advance(iter_path, iter->id_);
          iter_other_frontier->paths_.erase(iter_path);
        } catch (const std::exception& e){
          std::cout << "[FrontierFinder] : delete frontier cost info failed, skip it!" << e.what() << std::endl;
        }
      }
      // erase frontier !
      INFO_MSG("delete frontier avg_pos:(" << iter->average_.transpose() << ") id:" << iter->id_);
      iter = frontiers_.erase(iter);
      std::cout << "erase done." << std::endl;
      break;
    }else
      iter ++;
  }
}

void FrontierFinder::frontierForceDeleteAll() {
  frontiers_.clear();
  dormant_frontiers_.clear();
  tmp_frontiers_.clear();
  removed_ids_.clear();
}

void FrontierFinder::getFrontiersWithInfo(vector<Frontier>& clusters) {
  clusters.clear();
  for (const auto frontier : frontiers_){
    clusters.push_back(frontier);
    // for(auto vp:frontier.viewpoints_A_){
    //   // cout<<vp.visib_num_<<endl;
    //   // ROS_ASSERT(vp.visib_num_ > 0);
    // }
    // for(auto vp:frontier.viewpoints_G_){
    //   // cout<<vp.visib_num_<<endl;
    //   // ROS_ASSERT(vp.visib_num_ > 0);
    // }
  }
  // ROS_INFO_STREAM("getFrontiersWithInfo num: " << clusters.size());
}

void FrontierFinder::getDormantFrontiers(vector<vector<Vector3d>>& clusters) {
  clusters.clear();
  for (auto ft : dormant_frontiers_)
    clusters.push_back(ft.cells_);
}

void FrontierFinder::getFrontierBoxes(vector<pair<Eigen::Vector3d, Eigen::Vector3d>>& boxes) {
  boxes.clear();
  for (auto frontier : frontiers_) {
    Vector3d center = (frontier.box_max_ + frontier.box_min_) * 0.5;
    Vector3d scale = frontier.box_max_ - frontier.box_min_;
    boxes.push_back(make_pair(center, scale));
  }
}
// has replaced by skeleton library
// void FrontierFinder::getLocalPosegraph(std::shared_ptr<PoseGraph> &posegraph_res){
//   posegraph_res = posegraph_;
// }
// here directly return robot 0's posegraph
// void FrontierFinder::getPoseGraph(MultiPoseGraph::Ptr& posegraph_res){
//   posegraph_->deepCopy(posegraph_res->posegraphes[0], posegraph_->getSize());
// }


/**
* @brief 根据给定的前沿点访问顺序(tour)，拼接生成一条完整的巡游路径以及通往当前最近目标的路径。
* * @param[in]  pos            机器人当前的位置，作为整个巡游路径的起点。
* @param[in]  frontier_ids   一个包含了按最优顺序排列的前沿点索引的向量，定义了本次巡游的访问顺序。
* @param[out] path_all       用于存储最终生成的完整巡游路径的点集。该路径从当前位置`pos`出发，按`frontier_ids`指定的顺序依次连接所有前沿点。
* @param[out] path_next_goal 用于存储从当前位置`pos`到`frontier_ids`中第一个前沿点的路径点集，即机器人下一步需要立即执行的路径。// ONLY FOR VISUALIZE
*/
// when call this func, the number of ftrs must >= 2
void FrontierFinder::getPathForTour(const Vector3d& pos, const vector<int>& frontier_ids,
                                    vector<Vector3d>& path_all, vector<Vector3d>& path_next_goal) 
{
  path_all.clear();
  path_next_goal.clear();
  // Make an frontier_indexer to access the frontier list easier
  vector<list<Frontier>::iterator> frontier_indexer;
  for (auto it = frontiers_.begin(); it != frontiers_.end(); ++it)
    frontier_indexer.push_back(it);

  //TODO Compute the path from current pos to the first frontier
  vector<Vector3d> segment;
  // ViewNode::searchPath(pos, frontier_indexer[frontier_ids[0]]->viewpoints_.front().pos_, segment);
    // path.push_back(pos);
  // path.push_back(frontier_indexer[frontier_ids[0]]->viewpoints_.front().pos_);
  double dis;
  auto ftr_first = frontier_indexer[frontier_ids[0]];
  dis = scene_graph_->skeleton_gen_->astarSearch(pos, ftr_first->viewpoints_.front().pos_, segment, true);
  if (dis < 0){
    ROS_ERROR_STREAM("[ftr_finder] getNearTopoNodes error! ");
    segment.push_back(pos);
    segment.push_back(ftr_first->viewpoints_.front().pos_);
  }

  path_all.insert(path_all.end(), segment.begin(), segment.end());
  path_next_goal.insert(path_next_goal.end(), segment.begin(), segment.end());

  // Get paths of tour passing all clusters
  for (size_t i = 0; i < frontier_ids.size() - 1; ++i) {
    // Move to path to next cluster
    auto path_iter = frontier_indexer[frontier_ids[i]]->paths_.begin();
    int next_idx = frontier_ids[i + 1];
    for (int j = 0; j < next_idx; ++j)
      ++path_iter;
    path_all.insert(path_all.end(), path_iter->begin(), path_iter->end());
  }

  // Path back home
  auto last_ftr = frontier_indexer[frontier_ids.back()];
  vector<Vector3d> segment_home;
  for (auto p : last_ftr->path_to_home_) segment_home.emplace_back(p.x(), p.y(), p.z());
  // path_all.insert(path_all.end(), segment_home.begin(), segment_home.end());
}

void FrontierFinder::getFullCostMatrix(
    const Vector3d& cur_pos, const Vector3d& cur_vel, const double& cur_yaw,
    Eigen::MatrixXd& mat) {
  if (false) {
    // Use symmetric TSP formulation
    int dim = static_cast<int>(frontiers_.size()) + 2;
    mat.resize(dim, dim);  // current pose (0), sites, and virtual depot finally

    int i = 1, j = 1;
    for (const auto& ftr : frontiers_) {
      for (auto cs : ftr.costs_)
        mat(i, j++) = cs;
      ++i;
      j = 1;
    }

    // Costs from current pose to sites
    for (auto ftr : frontiers_) {
      Viewpoint vj = ftr.viewpoints_.front();
      vector<Vector3d> path;
      //TODO
      // mat(0, j) = mat(j, 0) =
      //     ViewNode::computeCost(cur_pos, vj.pos_, cur_yaw[0], vj.yaw_, cur_vel, cur_yaw[1], path);
      mat(0, j) = mat(j, 0) = (cur_pos - vj.pos_).norm();
      ++j;
    }
    // Costs from depot to sites, the same large vaule
    for (j = 1; j < dim - 1; ++j) {
      mat(dim - 1, j) = mat(j, dim - 1) = 100;
    }
    // Zero cost to depot to ensure connection
    mat(0, dim - 1) = mat(dim - 1, 0) = -10000;

  } else {
    // Use Asymmetric TSP
    int dimen = static_cast<int>(frontiers_.size());
    mat.resize(dimen + 2, dimen + 2);
    std::cout << "[TSP] mat size: " << mat.rows() << ", " << mat.cols() << std::endl;
    // Fill block for clusters
    int i = 1, j = 1;
    for (auto ftr : frontiers_) {
      for (auto cs : ftr.costs_) {
        if (is_print_info_) std::cout << "(ftr_" << i-1 << ", ftr_" << j-1 << ") -> " << cs << std::endl;
        // << ", ";
        mat(i, j++) = cs;
      }
      ++i;
      j = 1;
    }
    // print matrix mat
    if (is_print_info_)
    {
      std::cout << "mat: " << std::endl;
      for (int i = 0; i < mat.rows(); ++i) {
        for (int j = 0; j < mat.cols(); ++j) {
          std::cout << mat(i, j) << " ";
        }
        std::cout << std::endl;
      }
    }

    if (is_print_info_) std::cout << "" << std::endl;
    double max_value = 99999.0;
    mat(0, 0) = 0.0;
    // cost from cluster to cur_pose
    mat.block(1,0,dimen,1).setConstant(max_value);
    mat(dimen+1, 0) = 0.0;
    // cost from home to cluster
    mat.block(dimen+1,1,1,dimen).setConstant(max_value);
    mat(dimen+1, dimen+1) = 0.0;
    mat(0, dimen+1) = max_value;

    // Fill block from current state to clusters
    j = 1;
    for (auto ftr : frontiers_) 
    {

      // << ", ";
      Viewpoint vj = ftr.viewpoints_.front();
      vector<Vector3d> path;
      if (is_print_info_) std::cout << "vj.pos_: " << vj.pos_.transpose() << std::endl;
      if (is_print_info_) std::cout << "(cur_p, " << j - 1 << ") -> " ;
      //TODO
      // mat(0, j++) = (cur_pos - vj.pos_).norm();
      // mat(0, j++) =
          // ViewNode::computeCost(cur_pos, vj.pos_, cur_yaw[0], vj.yaw_, cur_vel, cur_yaw[1], path);

      double dis_ij = -1.0;
      bool a_star_success = false;

      // 找当前位置最近的topo节点
      // map<double, int> nearby_v_set_res_start;
      // getNearTopoNodes(*posegraph_, cur_pos, percep_utils_->getSensorMaxDist(), true, nearby_v_set_res_start);

      if ((cur_pos - vj.pos_).norm() < this->far_dis_thres_)
      {
        a_star_success = ViewNode::searchPath(cur_pos, vj.pos_, path, dis_ij);
      }
      if (dis_ij < 0.0 || !a_star_success)
      {
        dis_ij = scene_graph_->skeleton_gen_->astarSearch(cur_pos, vj.pos_, path, true);

        if (dis_ij <= 0.0)
        {
          ROS_ERROR_STREAM("[ftr_finder] getNearTopoNodes error! ");
          path.push_back(cur_pos);
          path.push_back(vj.pos_);
        }
      }

      // Cost of position change
      double pos_cost = dis_ij / ViewNode::vm_;
      Vector3d dir_2_ftr = (vj.pos_ - cur_pos).normalized();
      // Consider velocity change
      if (cur_vel.norm() > 1e-3) 
      {
        // Vector3d dir = (vj.pos_ - cur_pos).normalized();
        // Vector3d dir(cos(vj.yaw_), sin(vj.yaw_), 0.0);
        Vector3d vdir = cur_vel.normalized();
        double diff = acos(vdir.dot(dir_2_ftr));
        pos_cost += ViewNode::w_dir_ * diff;
      }
      else
      {
        // Vector3d dir(cos(vj.yaw_), sin(vj.yaw_), 0.0);
        Vector3d vdir(cos(cur_yaw), sin(cur_yaw), 0.0);
        double diff = acos(vdir.dot(dir_2_ftr));
        pos_cost += ViewNode::w_dir_ * diff;
      }
      // Cost of yaw change
      double diff = fabs(cur_yaw - vj.yaw_);
      diff = min(diff, 2 * M_PI - diff);
      double yaw_cost = diff / ViewNode::yd_;
      
      mat(0, j++) = max(pos_cost, yaw_cost);
      std::cout << " (pos_cost, yaw_cost) = (" << pos_cost << ", " << yaw_cost << ")" << std::endl;

    }

    // Costs from ftrs to home
    i = 1;
    for (auto ftr : frontiers_) {
      // std::cout << "(0, " << j << ")"
      // << ", ";
      Viewpoint vj = ftr.viewpoints_.front();
      mat(i++, dimen+1) = ftr.cost_to_home_;
    }


    // std::cout << "" << std::endl;
  }
}


// Sample viewpoints around frontier's average position, check coverage to the frontier cells
void FrontierFinder::sampleViewpoints(Frontier& frontier) {
  // Evaluate sample viewpoints on circles, find ones that cover most cells
//  INFO_MSG_YELLOW("avg: " << frontier.average_.transpose() << ", normal: " << frontier.normal_.transpose());
  vector<Viewpoint> bad_viewpoints;
  double z_sample = candidate_znum_ == 1 ? 0 : candidate_zmax_;
  double dz = candidate_znum_ == 1 ? 100.0 : abs(candidate_zmax_ - candidate_zmin_) / candidate_znum_;
  for (; z_sample >= candidate_zmin_ - 1e-3; z_sample -= dz)
  {
    for (double rc = candidate_rmax_, dr = (candidate_rmax_ - candidate_rmin_) / candidate_rnum_;
         rc >= candidate_rmin_ - 1e-3; rc -= dr)
    {
      double phi_s = atan2(frontier.normal_(1), frontier.normal_(0));
      double phi_range = candidate_phinum_ * candidate_dphi_;
      vector<double> phi_vec;

      for (int step = 0; candidate_dphi_ * step < phi_range + 1e-3 && candidate_dphi_ * step < M_PI; step++)  // 越到后面偏得越多
      {   double phi = phi_s + candidate_dphi_ * step;
        phi_vec.push_back(phi);
        if (step == 0) continue;
        phi = phi_s - candidate_dphi_ * step;
        phi_vec.push_back(phi);
      }

      for (size_t i = 0; i < phi_vec.size(); i++)
      {
        // INFO_MSG("rc: " << rc << ", phi: " << phi_vec[i]);
//        Vector3d sample_pos = frontier.average_ + rc * Vector3d(cos(phi_vec[i]), sin(phi_vec[i]), z_sample);
        Vector3d sample_pos = frontier.average_ + rc * Vector3d(cos(phi_vec[i]), sin(phi_vec[i]), 0) + Vector3d(0, 0, z_sample);



//      INFO_MSG("sample_pos: " << sample_pos.transpose() << "\t, Occ: " << edt_env_->getOccupancy(sample_pos) <<
//        "\t, isNearUnknown(): " << isNearUnknown(sample_pos) << "\t, isNearOcc(): " << isNearOcc(sample_pos));

        bool is_bad = false;
        // Qualified viewpoint is in bounding box and in safe region
        if (!edt_env_->isInLocalMap(sample_pos) || !edt_env_->isInGlobalMap(sample_pos) ||
            edt_env_->getInflateOccupancy(sample_pos) == MapInterface::OCCUPIED)
          continue;

//      INFO_MSG_GREEN("ok");
        if (isNearUnknown(sample_pos) || isNearOcc(sample_pos))
        {
          is_bad = true;
//        INFO_MSG_YELLOW("bad");
        }

        // Compute average yaw
        auto& cells = frontier.filtered_cells_;
        Eigen::Vector3d ref_dir = (cells.front() - sample_pos).normalized();
        double avg_yaw = 0.0;
        for (size_t i = 1; i < cells.size(); ++i) {
          Eigen::Vector3d dir = (cells[i] - sample_pos).normalized();
          double yaw = acos(dir.dot(ref_dir));
          if (ref_dir.cross(dir)[2] < 0) yaw = -yaw;
          avg_yaw += yaw;
        }
        avg_yaw = avg_yaw / cells.size() + atan2(ref_dir[1], ref_dir[0]);
        wrapYaw(avg_yaw);

        // Viewpoint vp = { sample_pos, avg_yaw, 1 };
        // frontier.viewpoints_.push_back(vp);

        // Compute the fraction of covered and visible cells
        int visib_num = countVisibleCells(sample_pos, avg_yaw, cells);
        if (visib_num > min_visib_num_)
        {
          Viewpoint vp = { sample_pos, avg_yaw, visib_num };
          if (is_bad)
          {
            bad_viewpoints.push_back(vp);
          }
          else
          {
            frontier.viewpoints_.push_back(vp);
          }
          // return; // TODO Only sample one viewpoint here
          // int gain = findMaxGainYaw(sample_pos, frontier, sample_yaw);
        }
        else
        {
//        INFO_MSG_YELLOW("not enough visible cells (" << visib_num << " < " << min_visib_num_ << ")");
        }
      }
    }
  }
  if (frontier.viewpoints_.empty())
  {
    frontier.viewpoints_ = bad_viewpoints;
  }
  if (frontier.viewpoints_.empty())
  {
    map<double, Vector3d> cell_map;
    for (auto cell : frontier.cells_)
    {
      if (edt_env_->getInflateOccupancy(cell) == MapInterface::FREE)
      {
        cell_map.insert(make_pair((cell - frontier.average_).norm(), cell));
      }
    }
    if (!cell_map.empty())
    {
      Viewpoint vp = { cell_map.begin()->second, 0.0, 1 };
      frontier.viewpoints_.push_back(vp);
    }
  }

}

bool FrontierFinder::isAnyFrontierCovered(const Vector3d& c_pos) {
  Vector3d update_min, update_max;
  edt_env_->getUpdatedBox(update_min, update_max);

  auto checkChanges = [&](const list<Frontier>& frontiers) 
  {
    for (auto ftr : frontiers) 
    {
      if (!haveOverlap(ftr.box_min_, ftr.box_max_, update_min, update_max)) continue;
      const int change_thresh = min_view_finish_fraction_ * ftr.cells_.size();
      int change_num = 0;
      for (auto cell : ftr.cells_) 
      {
        Eigen::Vector3i idx;
        idx = edt_env_->pos2GlobalIdx(cell);
        // edt_env_->sdf_map_->posToIndex(cell, idx);
        if (!(knownfree(idx) && isNeighborUnknown(idx)) && ++change_num >= change_thresh)
        {
          // INFO_MSG("Frontier " << ftr.id_ << " has " << change_num << " cells changed >= " << change_thresh << ", update");
          // 进一步判断是否需要更新
          if (isHalfInLocalMap(ftr))
          {
            // INFO_MSG("But Frontier " << ftr.id_ << " is half in local map, no update");
            break;
          }
          if (!isWellObserved(ftr, c_pos))
          {
            // INFO_MSG("But Frontier " << ftr.id_ << " is not well observed, no update");
            break;
          }
          return true;
        }
      }
    }
    return false;
  };

  if (checkChanges(frontiers_) || checkChanges(dormant_frontiers_)) return true;

  return false;
}

bool FrontierFinder::isNearUnknown(const Eigen::Vector3d& pos) {
  const int vox_num_xy = floor(min_candidate_clearance_unknown_xy_ / resolution_);
  const int vox_num_z  = floor(min_candidate_clearance_unknown_z_ / resolution_);
  for (int x = -vox_num_xy; x <= vox_num_xy; ++x)
    for (int y = -vox_num_xy; y <= vox_num_xy; ++y)
      for (int z = -vox_num_z; z <= vox_num_z; ++z) {
        Eigen::Vector3d vox;
        vox << pos[0] + x * resolution_, pos[1] + y * resolution_, pos[2] + z * resolution_;
        if (edt_env_->getOccupancy(vox) == MapInterface::UNKNOWN) return true;
      }
  return false;
}

bool FrontierFinder::isNearOcc(const Eigen::Vector3d& pos) {
  const int vox_num_xy = floor(min_candidate_clearance_xy_ / resolution_);
  const int vox_num_z  = floor(min_candidate_clearance_z_ / resolution_);
  for (int x = -vox_num_xy; x <= vox_num_xy; ++x)
    for (int y = -vox_num_xy; y <= vox_num_xy; ++y)
      for (int z = -vox_num_z; z <= vox_num_z; ++z) {
        Eigen::Vector3d vox;
        vox << pos[0] + x * resolution_, pos[1] + y * resolution_, pos[2] + z * resolution_;
        if (edt_env_->getInflateOccupancy(vox) == MapInterface::OCCUPIED) return true;
      }
  return false;
}


int FrontierFinder::countVisibleCells(
    const Eigen::Vector3d& pos, const double& yaw, const vector<Eigen::Vector3d>& cluster) {
  percep_utils_->setPose(pos, yaw);
  int visib_num = 0;
  Eigen::Vector3i idx;
  for (auto cell : cluster) {
    // Check if frontier cell is inside FOV
    if (!percep_utils_->insideFOV(cell)) continue;

    // Check if frontier cell is visible (not occulded by obstacles)
    if (edt_env_->isVisible(cell, pos)) visib_num += 1;
  }
  return visib_num;
}

void FrontierFinder::downsample(
    const vector<Eigen::Vector3d>& cluster_in, vector<Eigen::Vector3d>& cluster_out) {
  // downsamping cluster
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudf(new pcl::PointCloud<pcl::PointXYZ>);
  for (auto cell : cluster_in)
    cloud->points.emplace_back(cell[0], cell[1], cell[2]);

  const double leaf_size = edt_env_->getResolution() * down_sample_;
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(leaf_size, leaf_size, leaf_size);
  sor.filter(*cloudf);

  cluster_out.clear();
  for (auto pt : cloudf->points)
    cluster_out.emplace_back(pt.x, pt.y, pt.z);
}

void FrontierFinder::wrapYaw(double& yaw) {
  while (yaw < -M_PI)
    yaw += 2 * M_PI;
  while (yaw > M_PI)
    yaw -= 2 * M_PI;
}

inline vector<Eigen::Vector3i> FrontierFinder::sixNeighbors(const Eigen::Vector3i& voxel) {
  vector<Eigen::Vector3i> neighbors(6);
  Eigen::Vector3i tmp;

  tmp = voxel - Eigen::Vector3i(1, 0, 0);
  neighbors[0] = tmp;
  tmp = voxel + Eigen::Vector3i(1, 0, 0);
  neighbors[1] = tmp;
  tmp = voxel - Eigen::Vector3i(0, 1, 0);
  neighbors[2] = tmp;
  tmp = voxel + Eigen::Vector3i(0, 1, 0);
  neighbors[3] = tmp;
  tmp = voxel - Eigen::Vector3i(0, 0, 1);
  neighbors[4] = tmp;
  tmp = voxel + Eigen::Vector3i(0, 0, 1);
  neighbors[5] = tmp;

  return neighbors;
}

inline vector<Eigen::Vector3i> FrontierFinder::tenNeighbors(const Eigen::Vector3i& voxel) {
  vector<Eigen::Vector3i> neighbors(10);
  Eigen::Vector3i tmp;
  int count = 0;

  for (int x = -1; x <= 1; ++x) {
    for (int y = -1; y <= 1; ++y) {
      if (x == 0 && y == 0) continue;
      tmp = voxel + Eigen::Vector3i(x, y, 0);
      neighbors[count++] = tmp;
    }
  }
  neighbors[count++] = tmp - Eigen::Vector3i(0, 0, 1);
  neighbors[count++] = tmp + Eigen::Vector3i(0, 0, 1);
  return neighbors;
}

inline vector<Eigen::Vector3i> FrontierFinder::allNeighbors(const Eigen::Vector3i& voxel) {
  vector<Eigen::Vector3i> neighbors(26);
  Eigen::Vector3i tmp;
  int count = 0;
  for (int x = -1; x <= 1; ++x)
    for (int y = -1; y <= 1; ++y)
      for (int z = -1; z <= 1; ++z) {
        if (x == 0 && y == 0 && z == 0) continue;
        tmp = voxel + Eigen::Vector3i(x, y, z);
        neighbors[count++] = tmp;
      }
  return neighbors;
}

inline bool FrontierFinder::isNeighborUnknown(const Eigen::Vector3i& voxel) {
  // At least one neighbor is unknown
  auto nbrs = sixNeighbors(voxel);
  for (const auto& nbr : nbrs) {
    Eigen::Vector3d nbr_p = edt_env_->globalIdx2Pos(nbr);
    if (edt_env_->getOccupancy(nbr) == MapInterface::UNKNOWN && !hgrid_->isCovered(nbr_p))
      return true;
  }
  return false;
}

inline int FrontierFinder::toadr(const Eigen::Vector3i& idx) {
  // return edt_env_->sdf_map_->toAddress(idx);
  return edt_env_->globalIdx2BufIdx(idx);
}

inline bool FrontierFinder::knownfree(const Eigen::Vector3i& idx) {
  // return edt_env_->sdf_map_->getOccupancy(idx) == SDFMap::FREE;
  return edt_env_->getOccupancy(idx) == MapInterface::FREE;
}

inline bool FrontierFinder::inmap(const Eigen::Vector3i& idx) {
  return (edt_env_->isInLocalMap(idx) && edt_env_->isInGlobalMap(idx));
}

// void FrontierFinder::updateLocalPosegraph(const PoseGraph &posegraph_in) {
//   posegraph_.reset(new PoseGraph(posegraph_in));
//   posegraph_->renewKdtree();
// }

void FrontierFinder::geometryPoint2EigenVector3d(const geometry_msgs::Point &p_in, Eigen::Vector3d &p_out) {
  p_out.x() = p_in.x; p_out.y() = p_in.y; p_out.z() = p_in.z;
}

void FrontierFinder::eigenVector3d2GeometryPoint(const Eigen::Vector3d &p_in, geometry_msgs::Point &p_out) {
  p_out.x = p_in.x(); p_out.y = p_in.y(); p_out.z = p_in.z();
}

geometry_msgs::Point FrontierFinder::eigenVector3d2GeometryPoint(const Eigen::Vector3d &p_in) {
  geometry_msgs::Point p_out;
  p_out.x = p_in.x(); p_out.y = p_in.y(); p_out.z = p_in.z();
  return p_out;
}

Eigen::Vector3d FrontierFinder::geometryPoint2EigenVector3d(const geometry_msgs::Point &p_in) {
  Eigen::Vector3d p_out;
  p_out.x() = p_in.x; p_out.y() = p_in.y; p_out.z() = p_in.z;
  return p_out;
}

bool FrontierFinder::isBlacklisted(const Frontier& frontier)
{
  for (auto black_center : ftr_blacklist_)
  {
    if ((frontier.average_ - black_center).norm() < ftr_blacklist_radius_)
    {
      return true;
    }
  }
  return false;
}

void FrontierFinder::setLastAddIdx(const int &idx) {
  self_last_add_inx_ = idx;
}

}  // namespace ego_planner
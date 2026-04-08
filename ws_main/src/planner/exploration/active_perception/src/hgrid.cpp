#include <active_perception/uniform_grid.h>
#include <active_perception/hgrid.h>
#include <active_perception/graph_node.h>
// #include <path_searching/astar2.h>
// #include <plan_env/sdf_map.h>
// #include <plan_env/edt_environment.h>

namespace ego_planner {

HGrid::HGrid(const MapInterface::Ptr& edt, ros::NodeHandle& nh):nh_(nh)
{
  this->edt_ = edt;
  nh.param("partitioning/consistent_cost", consistent_cost_, 3.5);
  nh.param("partitioning/consistent_cost2", consistent_cost2_, 3.5);
  nh.param("partitioning/use_swarm_tf", use_swarm_tf_, false);
  nh.param("partitioning/w_first", w_first_, 1.0);
  nh.param("partitioning/hgrid_vis_range", hgrid_vis_range_, -1);
  nh.param("partitioning/print_info", print_info_, false);
}

HGrid::~HGrid() {
}

void HGrid::init(const Eigen::Vector3d& map_min, const Eigen::Vector3d& map_max) 
{
  grid1_.reset(new UniformGrid(edt_, nh_));
  grid1_->init(map_min, map_max, 1);
  grid2_.reset(new UniformGrid(edt_, nh_));
  grid2_->init(map_min, map_max, 2);

  // Swarm tf
  setSwarmTf();
  // Wait for swarm basecoor transform and initialize grid
  // while (!updateBaseCoor()) {
  //   ROS_WARN("Wait for basecoor.");
  //   ros::Duration(0.5).sleep();
  //   ros::spinOnce();
  // }
  grid1_->initGridData(0);
//  grid2_->initGridData(grid1_->getGridNum());
  updateBaseCoor();
}

void HGrid::setSwarmTf() {
  grid1_->use_swarm_tf_ = grid2_->use_swarm_tf_ = use_swarm_tf_;
  double yaw = 0.0;
  rot_sw_ << cos(yaw), -sin(yaw), 0, sin(yaw), cos(yaw), 0, 0, 0, 1;
  trans_sw_ << 0.0, 0.0, 0;
  grid1_->rot_sw_ = grid2_->rot_sw_ = rot_sw_;
  grid1_->trans_sw_ = grid2_->trans_sw_ = trans_sw_;
}

bool HGrid::updateBaseCoor() {

  // Eigen::Vector4d tf;
  // if (!edt_->sdf_map_->getBaseCoor(1, tf)) return false;
  // double yaw = tf[3];
  // rot_sw_ << cos(yaw), -sin(yaw), 0, sin(yaw), cos(yaw), 0, 0, 0, 1;
  // trans_sw_ = tf.head<3>();

  rot_sw_ = Eigen::Matrix3d::Identity();
  trans_sw_ = Eigen::Vector3d::Zero();

  grid1_->rot_sw_ = grid2_->rot_sw_ = rot_sw_;
  grid1_->trans_sw_ = grid2_->trans_sw_ = trans_sw_;
  grid1_->updateBaseCoor();
//  grid2_->updateBaseCoor();

  return true;
}

void HGrid::inputFrontiers(const vector<vector<Eigen::Vector3d>>& avgs) {
  // Input frontier to both levels
  grid1_->inputFrontiers(avgs);
  //  grid2_->inputFrontiers(avgs);
}
/**
 * @note  update Hgrid data structure (P.S. update frontier before use this function)
 * @param drone_id id of drone (always 1 in FY-Project)
 * @param grid_ids linear ids of UPGRADED grids in data vector (level 1 and 2)
 * @param reallocated no use!
 * @param last_grid_ids
 * @param first_ids
 * @param second_ids
 */
void HGrid::updateGridData(const int& drone_id, vector<int>& grid_ids, bool reallocated,
    const vector<int>& last_grid_ids, vector<int>& first_ids, vector<int>& second_ids) {

  // Convert grid_ids to the ids of bi-level uniform grid
  vector<int> grid_ids1;
  const int grid_num1 = grid1_->grid_data_.size();
  for (auto id : grid_ids) {
    if (id < grid_num1)
      grid_ids1.push_back(id);
  }
  // Update at level 1
  vector<int> tmp_ids1 = grid_ids1;
  grid1_->updateGridData(drone_id, grid_ids1);

  grid_ids = grid_ids1;

  // Maintain consistency of next visited grid
  // if (reallocated) return;
  // 获取一致性网格，暂时不启用
  // getConsistentGrid(last_grid_ids, grid_ids, first_ids, second_ids);

  if (print_info_)
  {
    ROS_WARN_STREAM("[grid_ids]:");
    for (auto id : grid_ids){
      ROS_WARN_STREAM("id: " << id);
    }
  }
}

// 获取一致的下一个网格 no use at this project!
void HGrid::getConsistentGrid(const vector<int>& last_ids, const vector<int>& cur_ids,
    vector<int>& first_ids, vector<int>& second_ids) {

  if (last_ids.empty()) return;
  // Find the first two level 1 grids in last sequence
  const int grid_num1 = grid1_->grid_data_.size();
  int grid_id1 = last_ids[0];
  if (grid_id1 >= grid_num1) {
    int tmp = grid_id1 - grid_num1;
    fineToCoarseId(tmp, grid_id1);
  }

  // std::cout << "level 1 grid 1: " << grid_id1 << std::endl;

  int grid_id2 = -1;
  for (size_t i = 1; i < last_ids.size(); ++i) {
    if (last_ids[i] < grid_num1) {
      grid_id2 = last_ids[i];
      break;
    } else {
      int fine = last_ids[i] - grid_num1;
      int coarse;
      fineToCoarseId(fine, coarse);
      if (coarse != grid_id1) {
        grid_id2 = coarse;
        break;
      }
    }
  }
  // std::cout << "level 1 grid 2: " << grid_id2 << std::endl;

  first_ids.clear();
  // In the current sequence, try to find the first level 1 grid...
  for (auto id : cur_ids) {
    if (id == grid_id1) {
      first_ids = { id };
      // std::cout << "1" << std::endl;
    }
  }

  if (first_ids.empty()) {
    // or its sub-grids
    for (auto id : cur_ids) {
      if (id < grid_num1) continue;
      int coarse;
      fineToCoarseId(id - grid_num1, coarse);
      if (coarse == grid_id1) {
        first_ids.push_back(id);
      }
    }
  }

  vector<int>* ids_ptr;
  if (!first_ids.empty()) {
    // Already find the first, should find the second
    ids_ptr = &second_ids;
  } else {
    // No first yet, continue to find the first
    ids_ptr = &first_ids;
  }

  // Can not find first grid/sub-grid, try to find the second one
  if (grid_id2 == -1) return;

  for (auto id : cur_ids) {
    if (id == grid_id2) {
      *ids_ptr = { id };
      // std::cout << "3" << std::endl;
      return;
    }
  }

  for (auto id : cur_ids) {
    if (id < grid_num1) continue;
    int coarse;
    fineToCoarseId(id - grid_num1, coarse);
    if (coarse == grid_id2) {
      // std::cout << "4" << std::endl;
      ids_ptr->push_back(id);
    }
  }
  return;
}

void HGrid::coarseToFineId(const int& coarse, vector<int>& fines) {
  // 0: 0, 1
  // 1: 2, 3
  // 2: 4, 5
  fines.clear();
  Eigen::Vector3i cidx;  // coarse idx
  grid1_->adrToIndex(coarse, cidx);

  vector<Eigen::Vector3i> fine_idxs;
  fine_idxs.emplace_back(cidx[0] * 2, cidx[1] * 2, cidx[2]);
  fine_idxs.emplace_back(cidx[0] * 2 + 1, cidx[1] * 2, cidx[2]);
  fine_idxs.emplace_back(cidx[0] * 2, cidx[1] * 2 + 1, cidx[2]);
  fine_idxs.emplace_back(cidx[0] * 2 + 1, cidx[1] * 2 + 1, cidx[2]);

  for (auto idx : fine_idxs) {
    fines.push_back(grid2_->toAddress(idx));
  }
}

void HGrid::fineToCoarseId(const int& fine, int& coarse) {
  Eigen::Vector3i fidx;
  grid2_->adrToIndex(fine, fidx);

  Eigen::Vector3i cidx;
  cidx[0] = fidx[0] / 2;
  cidx[1] = fidx[1] / 2;
  cidx[2] = fidx[2];

  coarse = grid1_->toAddress(cidx);
}

int HGrid::getUnknownCellsNum(const int& grid_id) {
  // Get unknown cell number of a grid
  return getGrid(grid_id).unknown_num_;
}

Eigen::Vector3d HGrid::getCenter(const int& id) {
  return getGrid(id).center_;
}

GridInfo& HGrid::getGrid(const int& id) {
  int grid_num1 = grid1_->grid_data_.size();
  if (id < grid_num1)
    return grid1_->grid_data_[id];
  else
    return grid2_->grid_data_[id - grid_num1];
}

void HGrid::getActiveGrids(vector<int>& grid_ids) {
  grid_ids.clear();
  const int grid_num1 = grid1_->grid_data_.size();
  for (int i = 0; i < grid_num1; ++i) {
    if (grid1_->grid_data_[i].active_ && grid1_->grid_data_[i].is_cur_relevant_) {
      grid_ids.push_back(i);
    }
  }
}

bool HGrid::getNextGrid(const vector<int>& grid_ids, Eigen::Vector3d& grid_pos, double& grid_yaw) {
  // Current level 1 grid id
  const int grid_num1 = grid1_->grid_data_.size();
  int grid_id1;
  if (grid_ids[0] < grid_num1)
    grid_id1 = grid_ids[0];
  else {
    int fine = grid_ids[0] - grid_num1;  // level 2 id
    fineToCoarseId(fine, grid_id1);
  }

  // std::cout << "current level 1 id: " << grid_id1 << std::endl;

  // Find the next different level 1 grid id
  int grid_id2 = -1;
  for (size_t i = 1; i < grid_ids.size(); ++i) {
    if (grid_ids[i] < grid_num1) {
      grid_id2 = grid_ids[i];
      break;
    } else {
      int fine = grid_ids[i] - grid_num1;
      int coarse;
      fineToCoarseId(fine, coarse);
      if (coarse != grid_id1) {
        grid_id2 = grid_ids[i];
        break;
      }
    }
  }

  // std::cout << "next level 1 id: " << grid_id2 << std::endl;

  if (grid_id2 == -1) return false;

  auto& grid1 = getGrid(grid_id1);
  auto& grid2 = getGrid(grid_id2);
  grid_pos = grid2.center_;
  Eigen::Vector3d dir = grid2.center_ - grid1.center_;
  grid_yaw = atan2(dir[1], dir[0]);

  // std::cout << "grid pos: " << grid_pos.transpose() << std::endl;

  return true;
}

bool HGrid::isClose(const int& id1, const int& id2) {
  // Convert to coarse level ids
  const int grid_num1 = grid1_->grid_data_.size();

  int tmp_id1 = id1;
  if (tmp_id1 >= grid_num1) {
    int fine = tmp_id1 - grid_num1;
    fineToCoarseId(fine, tmp_id1);
  }
  int tmp_id2 = id2;
  if (tmp_id2 >= grid_num1) {
    int fine = tmp_id2 - grid_num1;
    fineToCoarseId(fine, tmp_id2);
  }
  Eigen::Vector3i idx1, idx2;
  grid1_->adrToIndex(tmp_id1, idx1);
  grid1_->adrToIndex(tmp_id2, idx2);

  for (int i = 0; i < 3; ++i) {
    if (abs(idx1[i] - idx2[i]) > 1) return false;
  }
  return true;

  // int diff = abs(tmp_id1 - tmp_id2);
  // if (diff == 1 || diff == grid1_->grid_num_[1]) return true;
  // return false;
}

bool HGrid::inSameLevel1(const int& id1, const int& id2) {
  // Check whether two level 2 grids are contained in the same level 1 grid
  const int grid_num1 = grid1_->grid_data_.size();
  if (id1 < grid_num1 || id2 < grid_num1) return false;

  int tmp1 = id1 - grid_num1;
  int tmp2 = id2 - grid_num1;
  int coarse1, coarse2;
  fineToCoarseId(tmp1, coarse1);
  fineToCoarseId(tmp2, coarse2);
  if (coarse1 == coarse2) return true;
  return false;
}

bool HGrid::isConsistent(const int& id1, const int& id2) {
  const int grid_num1 = grid1_->grid_data_.size();

  int tmp1 = id1;
  if (tmp1 >= grid_num1) {
    tmp1 -= grid_num1;
    int coarse;
    fineToCoarseId(tmp1, coarse);
    tmp1 = coarse;
  }
  int tmp2 = id2;
  if (tmp2 >= grid_num1) {
    tmp2 -= grid_num1;
    int coarse;
    fineToCoarseId(tmp2, coarse);
    tmp2 = coarse;
  }

  if (tmp1 == tmp2) return true;
  return false;
}

void HGrid::getFrontiersInGrid(const vector<int>& grid_ids, vector<int>& ftr_ids, vector<int>& ftr_nums) {
  ftr_ids.clear();
  int tmp = grid_ids.front();
  int grid_num1 = grid1_->grid_data_.size();

  if (tmp < grid_num1) {
    auto& grid = grid1_->grid_data_[tmp];
    for (auto pair : grid.contained_frontier_ids_) ftr_ids.push_back(pair.first);
  } else {
    // Find all frontier in the same level 1 grid
    tmp -= grid_num1;
    int coarse;
    fineToCoarseId(tmp, coarse);
    vector<int> fines;
    coarseToFineId(coarse, fines);

    vector<int> allocated_fines;  // level 2 grid allocated to current drone
    for (auto fine : fines) {
      for (auto id : grid_ids) {
        if (fine + grid_num1 == id) {
          allocated_fines.push_back(fine);
          break;
        }
      }
    }

    for (auto fine : allocated_fines) {
      auto& grid = grid2_->grid_data_[fine];
      for (auto pair : grid.contained_frontier_ids_) ftr_ids.push_back(pair.first);
    }
  }
}

void HGrid::checkFirstGrid(const int& id) 
{
  auto& grid = getGrid(id);
  if (print_info_)
  {
    std::cout << "[grid id]: " << id << " | " << grid.id_ << "; \t";
    std::cout << "[center]: " << grid.center_.transpose() << "; \t";
    std::cout << "[unknown_num]: " << grid.unknown_num_ << "; \t";
    std::cout << "[is_covered]: " << grid.is_covered_ << "; \t";
    std::cout << "[is_active]: " << grid.active_ << "; ";

    std::cout << std::endl;
    std::cout << "[relevant]: " << grid.is_cur_relevant_ << ", " << grid.is_prev_relevant_ << "; ";
  }
}

bool HGrid::isCovered(const Eigen::Vector3d& pos)
{
  return grid1_->isCovered(pos);
}

void HGrid::getHgridGlobalBox(Eigen::Vector3d &min_pt, Eigen::Vector3d &max_pt) {
  min_pt = grid1_->min_;
  max_pt = grid1_->max_;
}

int HGrid::getHgridVisRange() const {
  return hgrid_vis_range_;
}

void HGrid::getVisIdxInRange(vector<int>& vis_idx_list, const Eigen::Vector3d& pos) {
  Eigen::Vector3d vis_max_pos, vis_min_pos;
  vis_max_pos = pos + hgrid_vis_range_ * grid1_->grid_size_ * Eigen::Vector3d(1.0, 1.0, 0.0) + Eigen::Vector3d(0.0, 0.0, 2.0);
  vis_min_pos = pos - hgrid_vis_range_ * grid1_->grid_size_ * Eigen::Vector3d(1.0, 1.0, 0.0) - Eigen::Vector3d(0.0, 0.0, 2.0);
  grid1_->getOverlappedGrids(vis_idx_list, vis_min_pos, vis_max_pos, false);
}

void HGrid::getGridMarker(vector<Eigen::Vector3d>& pts1, vector<Eigen::Vector3d>& pts2, const vector<int> & vis_idx_list) {
  pts1.clear();
  pts2.clear();
  if (hgrid_vis_range_ == -1){
    for (auto& grid : grid1_->grid_data_) {
      if (!grid.active_) continue;
      for (int i = 0; i < 4; ++i) {
        pts1.push_back(grid.vertices_[i]);
        pts2.push_back(grid.vertices_[(i + 1) % 4]);
      }
    }
  }else{
    for (auto idx : vis_idx_list) {
      auto& grid = grid1_->grid_data_[idx];
      if (!grid.active_) continue;
      for (int i = 0; i < 4; ++i) {
        pts1.push_back(grid.vertices_[i]);
        pts2.push_back(grid.vertices_[(i + 1) % 4]);
      }
    }
  }
//  for (auto& pt : pts1) pt[2] = 0.5;
//  for (auto& pt : pts2) pt[2] = 0.5;
}

void HGrid::getGridStateMarker(vector<Eigen::Vector3d>& center_list, vector<Eigen::Vector3d>& scale_list, 
                    vector<bool>& is_cover_list, vector<bool>& isactive_list, const vector<int> & vis_idx_list)
{
  if (hgrid_vis_range_ == -1){
    for (auto& grid : grid1_->grid_data_) {
      isactive_list.push_back(grid.active_);

      Eigen::Vector3d center = (grid.vmax_ + grid.vmin_) / 2.0;
      center.z() = center.z();
      center_list.push_back(center);

      Eigen::Vector3d scale = grid.vmax_ - grid.vmin_;
      if (scale.z() <= 0.0) scale.z() = 0.1;
      scale_list.push_back(scale);
      is_cover_list.push_back(grid.is_covered_);
    }
  }else{
    for (auto idx : vis_idx_list){
      auto& grid = grid1_->grid_data_[idx];
      isactive_list.push_back(grid.active_);

      Eigen::Vector3d center = (grid.vmax_ + grid.vmin_) / 2.0;
      center.z() = center.z();
      center_list.push_back(center);

      Eigen::Vector3d scale = grid.vmax_ - grid.vmin_;
      if (scale.z() <= 0.0) scale.z() = 0.1;
      scale_list.push_back(scale);
      is_cover_list.push_back(grid.is_covered_);
    }
  }
  // print info
  if (print_info_){
    for (auto & grid : grid1_->grid_data_){
      if (!grid.contained_frontier_ids_.empty()){
        checkFirstGrid(grid.id_);
        vector<int> ftr_id_vec, ftr_num_vec;
        grid1_->getFrontiersInGrid(grid.id_, ftr_id_vec, ftr_num_vec);
        std::cout << "[ftr_id] (id, num): ";
        for (size_t i = 0; i < ftr_id_vec.size(); i++)
        {
          std::cout << " (" << ftr_id_vec[i] << ", " << ftr_num_vec[i] << ") ";
        }
        std::cout << std::endl;
      }
    }
  }
}

void HGrid::getGridMarker2(vector<Eigen::Vector3d>& pts, vector<string>& texts, const vector<int> & vis_idx_list) {
  pts.clear();
  texts.clear();
  if (hgrid_vis_range_ == -1){
    for (size_t i = 0; i < grid1_->grid_data_.size(); ++i) {
      auto& grid = grid1_->grid_data_[i];
      if (!grid.active_ || !grid.is_cur_relevant_) continue;
      pts.push_back(grid.center_);
      texts.push_back(to_string(i));
    }
  }else{
    for (auto idx : vis_idx_list) {
      auto& grid = grid1_->grid_data_[idx];
      if (!grid.active_ || !grid.is_cur_relevant_) continue;
      pts.push_back(grid.center_);
      texts.push_back(to_string(idx));
    }
  }
}

void HGrid::encodeHgridData(quadrotor_msgs::HgridMsg &msg) {
  INFO_MSG_GREEN("[Hgird] Pack Hgrid data to msg ...");
  ros::Time t1 = ros::Time::now();
  // struct data
  msg.grid_size         = grid1_->grid_size_;
  msg.multi_layer_hgrid = grid1_->multilayer_hgrid_;
  eigenVector3d2GeometryPoint(grid1_->min_, msg.min);
  eigenVector3d2GeometryPoint(grid1_->max_, msg.max);
  eigenVector3d2GeometryPoint(grid1_->resolution_, msg.resolution);
  // encode uniform grid (level 1) data
  for (auto &grid : grid1_->grid_data_){
    msg.id.push_back(grid.id_);
    msg.local_id.push_back(grid.local_id_);
    msg.unknown_num.push_back(grid.unknown_num_);
    msg.frontier_num.push_back(grid.frontier_num_); // no use
    geometry_msgs::Point tmp_curgrid_center;
    eigenVector3d2GeometryPoint(grid.center_, tmp_curgrid_center);
    msg.center.push_back(tmp_curgrid_center);
    // calculate frontier cell num witch grid contained
    std_msgs::UInt16MultiArray grid_frontier_cellnum, grid_contained_frontiers;
    for (auto &iter : grid.contained_frontier_ids_){
      if (iter.second == 1){
        grid_contained_frontiers.data.push_back(iter.first);  // frontier id
        grid_frontier_cellnum.data.push_back(grid.frontier_cell_nums_[iter.first]); // frontier cell num in current grid
      }
    }
    msg.frontier_cell_nums.push_back(grid_frontier_cellnum);
    msg.contained_frontier_ids.push_back(grid_contained_frontiers);
    msg.is_updated.push_back(grid.is_updated_);              // is necessary?
    msg.need_divide.push_back(grid.need_divide_);
    msg.active.push_back(grid.active_);
    msg.is_prev_relevant.push_back(grid.is_prev_relevant_);  // is necessary?
    msg.is_cur_relevant.push_back(grid.is_cur_relevant_);    // is necessary?
    msg.is_covered.push_back(grid.is_covered_);              // is necessary?
  }
  std_msgs::UInt16MultiArray unigrid_relevant_ids, unigrid_relevant_map;
  for (auto i : grid1_->relevant_id_)
    msg.relevant_id.data.push_back(i);
  for (auto iter : grid1_->relevant_map_) {
    if ( iter.second == 1 ){
      msg.relevant_map.data.push_back(iter.first);
    }
  }
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "world";
  INFO_MSG_GREEN("[Hgird] Pack Hgrid data to msg Done. time:" << (ros::Time::now() - t1).toSec()<< "s");
}

void HGrid::decodeHgridData(const quadrotor_msgs::HgridMsg &msg, ros::NodeHandle& nh) {
  ros::Time t1 = ros::Time::now();
  INFO_MSG_GREEN("[Hgird] Unpack Hgrid data from msg ...");
  // decode and init struct data
  Eigen::Vector3d msg_map_min;
  Eigen::Vector3d msg_map_max;
  geometryPoint2EigenVector3d(msg.min, msg_map_min);
  geometryPoint2EigenVector3d(msg.max, msg_map_max);
  grid1_ = std::make_unique<UniformGrid>(this->edt_, nh);
  grid1_->grid_size_        = msg.grid_size;
  grid1_->multilayer_hgrid_ = msg.multi_layer_hgrid;
  grid1_->init(msg_map_min, msg_map_max, 1);

  setSwarmTf();
  geometryPoint2EigenVector3d(msg.resolution, grid1_->resolution_);
  grid1_->initGridData(0);
  updateBaseCoor();
  for (auto &i : msg.relevant_id.data){
    grid1_->relevant_id_.push_back(i);
    grid1_->relevant_map_[i] = 1;
  }
  // update grid data
  for (int i = 0; i< msg.local_id.size(); i++){
    auto & grid = grid1_->grid_data_[msg.local_id[i]];
    grid.id_          = msg.id[i];
    grid.local_id_    = msg.local_id[i];
    grid.unknown_num_ = msg.unknown_num[i];
    grid.frontier_num_= msg.frontier_num[i];
    geometryPoint2EigenVector3d(msg.center[i], grid.center_);
    grid.contained_frontier_ids_.clear(); grid.frontier_cell_nums_.clear();
    for (int j = 0; j< msg.contained_frontier_ids[i].data.size(); j++){
      grid.contained_frontier_ids_[msg.contained_frontier_ids[i].data[j]] = 1;
      grid.frontier_cell_nums_[msg.contained_frontier_ids[i].data[j]] = msg.frontier_cell_nums[i].data[j];
    }
    grid.is_updated_  = msg.is_updated[i];
    grid.need_divide_ = msg.need_divide[i];
    grid.active_      = msg.active[i];
    grid.is_prev_relevant_ = msg.is_prev_relevant[i];
    grid.is_cur_relevant_  = msg.is_cur_relevant[i];
    grid.is_covered_       = msg.is_covered[i];
  }
  INFO_MSG_GREEN("[Hgird] Unpack Hgrid data from msg Done. time:" << (ros::Time::now() - t1).toSec()<< "s");
}

void HGrid::geometryPoint2EigenVector3d(const geometry_msgs::Point &p_in, Eigen::Vector3d &p_out) {
  p_out.x() = p_in.x; p_out.y() = p_in.y; p_out.z() = p_in.z;
}

void HGrid::eigenVector3d2GeometryPoint(const Eigen::Vector3d &p_in, geometry_msgs::Point &p_out) {
  p_out.x = p_in.x(); p_out.y = p_in.y(); p_out.z = p_in.z();
}
}  // namespace ego_planner
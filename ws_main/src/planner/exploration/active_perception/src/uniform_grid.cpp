#include <active_perception/uniform_grid.h>
#include <active_perception/graph_node.h>

#include <map_interface/map_interface.hpp>

namespace ego_planner {

UniformGrid::UniformGrid(
  const MapInterface::Ptr edt, ros::NodeHandle& nh) {

  this->edt_ = edt;

  // Read min, max, resolution here
  nh.param("partitioning/min_unknown", min_unknown_, 10000);
  nh.param("partitioning/min_frontier", min_frontier_, 100);
  nh.param("partitioning/min_free", min_free_, 3000);
  nh.param("partitioning/consistent_cost", consistent_cost_, 3.5);
  nh.param("partitioning/w_unknown", w_unknown_, 3.5);
  nh.param("partitioning/grid_size", grid_size_, 5.0);
  nh.param("partitioning/multilayer_hgrid", multilayer_hgrid_, false);
}

UniformGrid::~UniformGrid() {
}

void UniformGrid::init(const Eigen::Vector3d& min, const Eigen::Vector3d& max, const int& level)
{
  min_ = min;
  max_ = max;
  auto size = max_ - min_;

  // resolution_ = size / 3;
  for (int i = 0; i < 3; ++i) {
    int num = ceil(size[i] / grid_size_);
    resolution_[i] = size[i] / double(num);
    for (int j = 1; j < level; ++j) resolution_[i] *= 0.5; // 随着level增加，resolution * 0.5
  }
  if (!multilayer_hgrid_) {
    resolution_[2] = size[2];
  }
  initialized_ = false;
  level_ = level;
}

void UniformGrid::initGridData(const int& id_offset) {
  // reCalculate grid size | grid num
  Eigen::Vector3d size = max_ - min_;
  for (int i = 0; i < 3; ++i) grid_num_(i) = ceil(size(i) / resolution_(i));
//  for (int i = 0; i < 3; ++i) grid_num_(i) = ceil(size(i) / grid_size_); // [gwq] ??? 为什么不这么写？
  grid_data_.clear();
  grid_data_.resize(grid_num_[0] * grid_num_[1] * grid_num_[2]);

  if (id_offset == 0)
    std::cout << "[Hgrid] Init Hgrid LEVEL 0" << std::endl;
  else
    std::cout << "[Hgrid] Init Hgrid LEVEL 1" << std::endl;
  std::cout << "[Hgrid] Map size: " << size.transpose() << std::endl;
  std::cout << "[Hgrid] resolution:" << resolution_.transpose() << std::endl;
  std::cout << "[Hgrid] data size :" << grid_data_.size() << " | ("
          << grid_num_(0) << ", " << grid_num_(1) << ", " << grid_num_(2) << ")" <<std::endl;
  std::cout << "[Hgrid] grid num  :" << grid_num_.transpose() << std::endl;


  // Init each grid info
  for (int x = 0; x < grid_num_[0]; ++x) {
    for (int y = 0; y < grid_num_[1]; ++y) {
      for (int z = 0; z < grid_num_[2]; ++z) {
        Eigen::Vector3i id(x, y, z);
        int ads = toAddress(id);
        auto& grid = grid_data_[ads];
        grid.local_id_ = ads;
        grid.id_ = ads + id_offset;

        Eigen::Vector3d pos;
        indexToPos(id, 0.5, pos);
        if (use_swarm_tf_) {
          pos = rot_sw_ * pos + trans_sw_;
        }

        grid.center_ = pos;
        grid.unknown_num_ = resolution_[0] * resolution_[1] * resolution_[2] /
                            pow(edt_->getResolution(), 3);

        grid.is_prev_relevant_ = true;
        grid.is_cur_relevant_ = true;
        grid.is_covered_ = false;
        grid.need_divide_ = false;
        if (level_ == 1)
          grid.active_ = true;
        else
          grid.active_ = false;
      }
    }
  }
}

void UniformGrid::updateBaseCoor() {
  for (int i = 0; i < grid_data_.size(); ++i) {
    auto& grid = grid_data_[i];
    // if (!grid.active_) continue;
    Eigen::Vector3i id;
    adrToIndex(i, id);

    // Compute vertices and box of grid in current drone's frame
    Eigen::Vector3d left_bottom, right_top, left_top, right_bottom;
    indexToPos(id, 0.0, left_bottom);
    indexToPos(id, 1.0, right_top);
    if (multilayer_hgrid_){
      grid.vmin_ = left_bottom;
      grid.vmax_ = right_top;
    }
    // [gwq]此处原作者将Hgrid的信息压缩成了二维，本人已在updateGridInfo函数中加入三维扩展补丁
    left_top[0] = left_bottom[0];
    left_top[1] = right_top[1];
    left_top[2] = left_bottom[2];
    right_bottom[0] = right_top[0];
    right_bottom[1] = left_bottom[1];
    right_bottom[2] = left_bottom[2];
    right_top[2] = left_bottom[2];

    vector<Eigen::Vector3d> vertices = { left_bottom, right_bottom, right_top, left_top };
    if (use_swarm_tf_) {
      for (auto& vert : vertices) vert = rot_sw_ * vert + trans_sw_;
    }

    Eigen::Vector3d vmin, vmax;
    vmin = vmax = vertices[0];
    for (int j = 1; j < vertices.size(); ++j) {
      for (int k = 0; k < 2; ++k) {
        vmin[k] = min(vmin[k], vertices[j][k]);
        vmax[k] = max(vmax[k], vertices[j][k]);
      }
    }
    grid.vertices_ = vertices;
    if (!multilayer_hgrid_) {
      grid.vmin_ = vmin;
      grid.vmax_ = vmax;
    }
    // Compute normals of four separating lines
    grid.normals_.clear();
    for (int j = 0; j < 4; ++j) {
      Eigen::Vector3d dir = (vertices[(j + 1) % 4] - vertices[j]).normalized();
      grid.normals_.push_back(dir);
    }
  }
}



void UniformGrid::getOverlappedGrids(vector<int> & grid_adrs, const Eigen::Vector3d &update_min, const Eigen::Vector3d &update_max, bool show_info) {
  auto computeIntersection = [](const Eigen::Vector3d& min1, const Eigen::Vector3d& max1,
                                const Eigen::Vector3d& min2, const Eigen::Vector3d& max2) {
      Eigen::Vector3d min_int = min1.cwiseMax(min2);
      Eigen::Vector3d max_int = max1.cwiseMin(max2);
      // 检查区域是否为空
      if (min_int[0] > max_int[0] || min_int[1] > max_int[1] || min_int[2] > max_int[2]) {
        return std::make_pair(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0));
      }
      return std::make_pair(min_int, max_int);
  };
  Eigen::Vector3i grid_id_min, grid_id_max;
  auto intersection = computeIntersection(update_min, update_max, min_, max_);
  if (intersection.first.isApprox(Eigen::Vector3d::Zero()) && intersection.second.isApprox(Eigen::Vector3d::Zero()))
    return;
  // [gwq] 加入特判，如果求交集之后的box边界和hgrid边界重合，则向内收缩一定的值，保证索引正确
  for (int i = 0; i < 3; ++i) {
    if (abs(intersection.first[i] - min_[i]) < 1e-3) intersection.first[i] += 0.1;
    if (abs(intersection.second[i] - max_[i]) < 1e-3) intersection.second[i] -= 0.1;
  }
  posToIndex(intersection.first, grid_id_min);
  posToIndex(intersection.second, grid_id_max);
  if (show_info) {
    std::cout << "[HgridUpdate] : update min idx : " << grid_id_min.transpose() << " | adr: "
        << toAddress(grid_id_min.transpose()) << "min pos : " << intersection.first.transpose() << std::endl;
    std::cout << "[HgridUpdate] : update max idx : " << grid_id_max.transpose() << " | adr: "
        << toAddress(grid_id_max.transpose()) << "max pos : " << intersection.second.transpose() << std::endl;
  }
  // get all update grid idx
  grid_adrs.clear();
  std::unordered_map<int, int> update_id_flag;
  int adr_tmp = 0;
  for (int x = grid_id_min[0]; x <= grid_id_max[0]; ++x)
    for (int y = grid_id_min[1]; y <= grid_id_max[1]; ++y)
      for (int z = grid_id_min[2]; z <= grid_id_max[2]; ++z){
        adr_tmp = toAddress(Eigen::Vector3i(x, y, z));
        if (update_id_flag.find(adr_tmp) == update_id_flag.end()){
          update_id_flag[adr_tmp] = 1;
          grid_adrs.push_back(adr_tmp);
        }
      }
  if (show_info)
    std::cout << "[HgridUpdate] : update grid num: " << grid_adrs.size() << std::endl;

//  std::cout << "[HgridUpdate] : update grid adrs: " << std::endl;
//  for (auto& adr : grid_adrs)
//    std::cout << adr << " ";
//  std::cout << std::endl;
}
void UniformGrid::updateGridData(const int& drone_id, vector<int>& grid_ids) {

  // parti_ids are ids of grids that are assigned to THIS drone and should be divided
  // parti_ids_all are ids of ALL grids that should be divided
  for (auto& grid : grid_data_) {
    grid.is_updated_ = false;
  }
  ros::Time t1 = ros::Time::now();
  vector<int> update_grid_adrs;
  Vector3d update_min, update_max;
  edt_->getUpdatedBox(update_min, update_max);
#ifdef USE_GLOBAL_GRID_UPDATE
  auto have_overlap = [](
          const Vector3d& min1, const Vector3d& max1, const Vector3d& min2, const Vector3d& max2) {
      for (int m = 0; m < 2; ++m) {
        double bmin = max(min1[m], min2[m]);
        double bmax = min(max1[m], max2[m]);
        if (bmin > bmax + 1e-3) return false;
      }
      return true;
  };
  // For each grid, check overlap with updated box and update it if necessary
  // 朴素更新方法 （全遍历）
  std::cout << "[HgridUpdate] : Global update once ..." << std::endl;
  ros::Time t1 = ros::Time::now();
  for (int i = 0; i < grid_data_.size(); ++i) {
    auto& grid = grid_data_[i];
    if (!grid.active_) continue;
    if (grid.is_covered_) continue;
    if (!have_overlap(grid.vmin_, grid.vmax_, update_min, update_max)) continue;

    // Update the grid
    Eigen::Vector3i idx;
    adrToIndex(i, idx);
    updateGridInfo(idx);

    if (grid.need_divide_) {
      parti_ids_all.push_back(i);
      grid.active_ = false;
    }
  }
  // Update the list of relevant grid
  relevant_id_.clear();
  relevant_map_.clear();
  for (int i = 0; i < grid_data_.size(); ++i) {
    if (isRelevant(grid_data_[i])) {
      relevant_id_.push_back(i);
      relevant_map_[i] = 1;
    }
  }
#else
  // !!! 增量式更新
  // std::cout << "[HgridUpdate] : Incremental update once ..." << std::endl;
  getOverlappedGrids(update_grid_adrs, update_min, update_max, false);
  if(update_grid_adrs.empty())
    return ;

  for (auto &grid_adr : update_grid_adrs){
    auto & grid = grid_data_[grid_adr];
    if (!grid.active_) continue;
    if (grid.is_covered_) continue;
    // Update the grid
    Eigen::Vector3i idx;
    adrToIndex(grid_adr, idx);
    updateGridInfo(idx);

    if (grid.need_divide_){
      grid.active_ = false;
    }
  }
  relevant_map_.clear();
  relevant_id_.clear();
  for (int grid_adr : update_grid_adrs){
    if(isRelevant(grid_data_[grid_adr])){
      relevant_map_[grid_adr] = 1;
      relevant_id_.push_back(grid_adr);
    }
  }
#endif

  // Update the dominance grid of ego drone
  if (!initialized_) {
    if (drone_id == 1 && level_ == 1) grid_ids = relevant_id_;
    // else
    //   grid_ids = {};
    ROS_WARN("Init grid allocation.");
    initialized_ = true;
  } else {
    for (auto it = grid_ids.begin(); it != grid_ids.end();) {
      if (relevant_map_.find(*it) == relevant_map_.end()) {
        // Remove irrelevant ones
        // std::cout << "Remove irrelevant: " << *it << std::endl;
        it = grid_ids.erase(it);
      } else if (grid_data_[*it].need_divide_) {
        // Partition coarse grid
        // std::cout << "Remove divided: " << *it << std::endl;
        it = grid_ids.erase(it);
        // grid_data_[*it].active_ = false;
      } else {
        ++it;
      }
    }
  }
  // std::cout<<"[HgridUpdate] : update time: "<<(ros::Time::now()-t1).toSec() * 1e3<<"ms"<<std::endl;
}

void UniformGrid::updateGridInfo(const Eigen::Vector3i& id) {
  int adr = toAddress(id);
  auto& grid = grid_data_[adr];
  if (grid.is_updated_) {  // Ensure only one update to avoid repeated computation
    return;
  }
  grid.is_updated_ = true;

  grid.is_prev_relevant_ = grid.is_cur_relevant_;

  Eigen::Vector3d gmin, gmax;
  // indexToPos(id, 0.0, gmin);
  indexToPos(id, 1.0, gmax);  // Only the first 2 values of vmax is useful, should compute max here
  // Check if a voxel is inside the rotated box
  auto inside_box = [](const Eigen::Vector3d& vox, const GridInfo& grid) {
    // Check four separating planes(lines)
    for (int m = 0; m < 4; ++m) {
      if ((vox - grid.vertices_[m]).dot(grid.normals_[m]) <= 0.0) return false;
    }
    return true;
  };
  // [gwq] Hgrid 三维补丁
  auto inside_box_3d = [](const Eigen::Vector3d& vox, const GridInfo& grid) {
    Eigen::Vector3d tmp;
    tmp = vox.cwiseMin(grid.vmax_);
    tmp = vox.cwiseMax(grid.vmin_);
    if (tmp.isApprox(vox)) return true;
    else return false;
  };
  // Count known
  const double res = edt_->getResolution();
  grid.center_.setZero();
  grid.unknown_num_ = 0;
  int free = 0;
  for (double x = grid.vmin_[0]; x <= grid.vmax_[0]; x += res) {
    for (double y = grid.vmin_[1]; y <= grid.vmax_[1]; y += res) {
      for (double z = grid.vmin_[2]; z <= gmax[2]; z += res) {

        Eigen::Vector3d pos(x, y, z);
        if (!inside_box_3d(pos, grid)) continue;

        int state = edt_->getOccupancy(pos);
        if (state == MapInterface::FREE) {
          free += 1;
        } else if (state == MapInterface::UNKNOWN || !edt_->isInLocalMap(pos) ) {
          grid.center_ = (grid.center_ * grid.unknown_num_ + pos) / (grid.unknown_num_ + 1);
          grid.unknown_num_ += 1;
        }
      }
    }
  }

  grid.is_cur_relevant_ = isRelevant(grid);
  if (!grid.is_cur_relevant_) grid.is_covered_ = true;

  // ROS_ERROR_STREAM("min: " << grid.vmin_.transpose() << ", max: " << grid.vmax_.transpose());

  // cout << "level: " << level_ << ", grid id: " << id.transpose() << ", adr: " << adr
  //      << ", unknown: " << grid.unknown_num_ << ", center: " << grid.center_.transpose()
  //      << ", rele: " << grid.is_cur_relevant_ << endl;

  if (level_ == 1 && grid.active_ && free > min_free_) {
    grid.need_divide_ = true;
  }
}

int UniformGrid::toAddress(const Eigen::Vector3i& id) {
  return id[0] * grid_num_(1) * grid_num_(2) + id[1] * grid_num_(2) + id[2];
}

void UniformGrid::adrToIndex(const int& adr, Eigen::Vector3i& idx) {
  // id[0] * grid_num_(1) * grid_num_(2) + id[1] * grid_num_(2) + id[2];
  int tmp_adr = adr;
  const int a = grid_num_(1) * grid_num_(2);
  const int b = grid_num_(2);

  idx[0] = tmp_adr / a;
  tmp_adr = tmp_adr % a;
  idx[1] = tmp_adr / b;
  idx[2] = tmp_adr % b;
}

void UniformGrid::posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id) {
  for (int i = 0; i < 3; ++i) id(i) = floor((pos(i) - min_(i)) / resolution_[i]);
}

void UniformGrid::indexToPos(const Eigen::Vector3i& id, const double& inc, Eigen::Vector3d& pos) {
  // inc: 0 for min, 1 for max, 0.5 for mid point
  for (int i = 0; i < 3; ++i) pos(i) = (id(i) + inc) * resolution_[i] + min_(i);
}

void UniformGrid::activateGrids(const vector<int>& ids) {
  for (auto id : ids) {
    grid_data_[id].active_ = true;
  }
  extra_ids_ = ids;  // To avoid incomplete update
}

bool UniformGrid::insideGrid(const Eigen::Vector3i& id) {
  // Check inside min max
  for (int i = 0; i < 3; ++i) {
    if (id[i] < 0 || id[i] >= grid_num_[i]) {
      return false;
    }
  }
  return true;
}

void UniformGrid::inputFrontiers(const vector<vector<Eigen::Vector3d>>& frontiers) 
{
  for (auto& grid : grid_data_) 
  {
    grid.contained_frontier_ids_.clear();
    grid.frontier_cell_nums_.clear();
  }
  Eigen::Vector3i id;
  Eigen::Matrix3d Rt = rot_sw_.transpose();
  Eigen::Vector3d t_inv = -Rt * trans_sw_;

  for (int fc_i = 0; fc_i < frontiers.size(); ++fc_i) 
  {
    auto frt_cluster = frontiers[fc_i];
    for (auto ftr_cell : frt_cluster)
    {
      Eigen::Vector3d pos = ftr_cell;
      if (use_swarm_tf_) {
        pos = Rt * pos + t_inv;
      }
      posToIndex(pos, id);
      if (!insideGrid(id)) continue;
      auto& grid = grid_data_[toAddress(id)];
      grid.contained_frontier_ids_[fc_i] = 1;
      grid.frontier_cell_nums_[fc_i]++;
    }
  }
}

/**
 * @brief 判断格子是否参与更新
 * @param grid 格子信息
 * @return true 参与更新 false 不参与更新
 */
bool UniformGrid::isRelevant(const GridInfo& grid) {
  // return grid.unknown_num_ >= min_unknown_ || grid.frontier_num_ >= min_frontier_;
  // return grid.unknown_num_ >= min_unknown_ || !grid.frontier_cell_nums_.empty();
  return grid.unknown_num_ >= min_unknown_ || !grid.contained_frontier_ids_.empty();
}

bool UniformGrid::isCovered(const Eigen::Vector3d& pos)
{
  Eigen::Vector3i idx;
  posToIndex(pos, idx);
  const int ads = toAddress(idx);
  return (grid_data_[ads].is_covered_);
}


void UniformGrid::getCostMatrix(const vector<Eigen::Vector3d>& positions,
    const vector<Eigen::Vector3d>& velocities, const vector<int>& prev_first_grid,
    const vector<int>& grid_ids, Eigen::MatrixXd& mat) {
}

void UniformGrid::getGridTour(const vector<int>& ids, vector<Eigen::Vector3d>& tour) {
  tour.clear();
  for (int i = 0; i < ids.size(); ++i) {
    tour.push_back(grid_data_[ids[i]].center_);
  }
}

void UniformGrid::getFrontiersInGrid(const int& grid_id, vector<int>& ftr_ids, vector<int>& ftr_nums) {
  // Find frontier having more than 1/4 within the first grid
  if (grid_id >= grid_data_.size() || grid_id < 0)
  {
    ROS_ERROR_STREAM("[UniformGrid] grid_id exceed! : " << grid_id);
    return;
  }
  auto& first_grid = grid_data_[grid_id];
  ftr_ids.clear();
  // for (auto pair : first_grid.frontier_cell_nums_) {
  //   ftr_ids.push_back(pair.first);
  // }
  for (auto pair : first_grid.contained_frontier_ids_) 
  {
    ftr_ids.push_back(pair.first);
    ftr_nums.push_back(first_grid.frontier_cell_nums_[pair.first]);
  }
}

void UniformGrid::getGridMarker(vector<Eigen::Vector3d>& pts1, vector<Eigen::Vector3d>& pts2) {

  Eigen::Vector3d p1 = min_;
  Eigen::Vector3d p2 = min_ + Eigen::Vector3d(max_[0] - min_[0], 0, 0);
  for (int i = 0; i <= grid_num_[1]; ++i) {
    Eigen::Vector3d pt1 = p1 + Eigen::Vector3d(0, resolution_[1] * i, 0);
    Eigen::Vector3d pt2 = p2 + Eigen::Vector3d(0, resolution_[1] * i, 0);
    pts1.push_back(pt1);
    pts2.push_back(pt2);
  }

  p1 = min_;
  p2 = min_ + Eigen::Vector3d(0, max_[1] - min_[1], 0);
  for (int i = 0; i <= grid_num_[0]; ++i) {
    Eigen::Vector3d pt1 = p1 + Eigen::Vector3d(resolution_[0] * i, 0, 0);
    Eigen::Vector3d pt2 = p2 + Eigen::Vector3d(resolution_[0] * i, 0, 0);
    pts1.push_back(pt1);
    pts2.push_back(pt2);
  }
  for (auto& p : pts1) p[2] = 0.5;
  for (auto& p : pts2) p[2] = 0.5;
}
}  // namespace fast_planner

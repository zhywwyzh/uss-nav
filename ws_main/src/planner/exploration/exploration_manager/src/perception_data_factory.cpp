//
// Created by gwq on 8/16/24.
//
#include <memory>
#include <utility>

#include "exploration_manager/perception_data_factory.h"

namespace ego_planner{
// ------------------------- [PerceptionDataMsgFactory] ---------------------------- //
PerceptionDataMsgFactory::PerceptionDataMsgFactory(const quadrotor_msgs::PerceptionMsg msg_in,
                                                   const double & inflate_gridmap_resolution) {
  perception_msg = msg_in;
  // init posegraph
  quadrotor_msgs::MultiPoseGraph pg_msg = msg_in.posegraph_msg;
  for (int i = 0; i < pg_msg.key_pose_list_xyz.size(); ++i){
    pcl::PointXYZI p;
    p.x = pg_msg.key_pose_list_xyz[i].x; p.y = pg_msg.key_pose_list_xyz[i].y; p.z = pg_msg.key_pose_list_xyz[i].z;
    p.intensity = pg_msg.key_pose_list_intensity[i];
    posegraph.key_pose_list->push_back(p);
    posegraph.addVertex(i);
  }
  // unpack edges
  for (int i = 0; i < pg_msg.pose_edge_p_end.size(); ++i){
    for (int j = 0; j < pg_msg.pose_edge_p_end[i].data.size(); ++j){
      posegraph.pose_edge[i].insert(Edge(pg_msg.pose_edge_p_end[i].data[j], pg_msg.pose_edge_weight[i].data[j]));
    }
  }
  posegraph.renewKdtree();
  // creat map from key_pose to frontier idx
  for (int i = 0; i < perception_msg.ftr_msg.keypose_idx.size(); ++i)
    keypose_to_frontieridx[perception_msg.ftr_msg.keypose_idx[i]].push_back(i);

  // hgrid info init
  geoPt2Vec3d(perception_msg.hgrid_msg.min, hgrid_min_);
  geoPt2Vec3d(perception_msg.hgrid_msg.max, hgrid_max_);
  geoPt2Vec3d(perception_msg.hgrid_msg.resolution, hgrid_resoloution_);
  Eigen::Vector3d hgrid_size = hgrid_max_ - hgrid_min_;
  for (int i = 0; i < 3; ++i) grid_num_(i) = ceil(hgrid_size(i) / hgrid_resoloution_(i));

  // !! add redundancy data : inorder to merge hgrid conviently
  inflate_gridmap_resolution_ = inflate_gridmap_resolution;
  VOID_HGRID_DATA_ = grid_num_(0) * grid_num_(1) * grid_num_(2);

  quadrotor_msgs::HgridMsg &hgrid_data = perception_msg.hgrid_msg;
  hgrid_data.id.push_back(0);
  hgrid_data.local_id.push_back(0);
  hgrid_data.unknown_num.push_back(floor(hgrid_resoloution_[0] * hgrid_resoloution_[1] * hgrid_resoloution_[2] /
                                    pow(inflate_gridmap_resolution_, 3)));
  hgrid_data.frontier_num.push_back(0);
  std_msgs::UInt16MultiArray tmp;
  hgrid_data.frontier_cell_nums.push_back(tmp);
  hgrid_data.contained_frontier_ids.push_back(tmp);
  hgrid_data.is_updated.push_back(false);
  hgrid_data.need_divide.push_back(false);
  hgrid_data.active.push_back(true);
  hgrid_data.is_prev_relevant.push_back(true);
  hgrid_data.is_cur_relevant.push_back(true);
  hgrid_data.is_covered.push_back(false);
}

PerceptionDataMsgFactory::~PerceptionDataMsgFactory() = default;

int PerceptionDataMsgFactory::posToHgridAdress(const geometry_msgs::Point & pos) {
  Eigen::Vector3i hgrid_id_tmp;
  Eigen::Vector3d pos_tmp(pos.x, pos.y, pos.z);
  for (int i = 0; i < 3; ++i) hgrid_id_tmp[i] = floor((pos_tmp(i) - hgrid_min_(i)) / hgrid_resoloution_[i]);
  int adress = hgrid_id_tmp[0] * grid_num_(1) * grid_num_(2) + hgrid_id_tmp[1] * grid_num_(2) + hgrid_id_tmp[2];
  if (adress < 0 || adress >= grid_num_(0) * grid_num_(1) * grid_num_(2)) adress = VOID_HGRID_DATA_;  // out of range, fake as VOID_HGRID_DATA_
  return adress;
}

int PerceptionDataMsgFactory::posToHgridAdress(const Eigen::Vector3d & pos) {
  Eigen::Vector3i hgrid_id_tmp;
  Eigen::Vector3d pos_tmp = pos;
  for (int i = 0; i < 3; ++i) hgrid_id_tmp[i] = floor((pos_tmp(i) - hgrid_min_(i)) / hgrid_resoloution_[i]);
  int adress = hgrid_id_tmp[0] * grid_num_(1) * grid_num_(2) + hgrid_id_tmp[1] * grid_num_(2) + hgrid_id_tmp[2];
  if (adress < 0 || adress >= grid_num_(0) * grid_num_(1) * grid_num_(2)) adress = VOID_HGRID_DATA_;
  return adress;
}

Eigen::Vector3d PerceptionDataMsgFactory::adressToPos(const int & adress) {
  Eigen::Vector3i idx;
  Eigen::Vector3d pos;
  int tmp_adr = adress;
  const int a = grid_num_(1) * grid_num_(2);
  const int b = grid_num_(2);
  idx[0] = tmp_adr / a;
  tmp_adr = tmp_adr % a;
  idx[1] = tmp_adr / b;
  idx[2] = tmp_adr % b;
  for (int i = 0; i < 3; ++i) pos(i) = (idx[i] + 0.5) * hgrid_resoloution_[i] + hgrid_min_(i);
  return pos;
}

inline void PerceptionDataMsgFactory::geoPt2Vec3d(const geometry_msgs::Point &p_in, Eigen::Vector3d &p_out) {
  p_out.x() = p_in.x; p_out.y() = p_in.y; p_out.z() = p_in.z;
}
inline void PerceptionDataMsgFactory::vec3d2GeoPt(const Eigen::Vector3d &p_in, geometry_msgs::Point &p_out) {
  p_out.x = p_in.x(); p_out.y = p_in.y(); p_out.z = p_in.z();
}
inline geometry_msgs::Point PerceptionDataMsgFactory::vec3d2GeoPt(const Eigen::Vector3d &p_in) {
  geometry_msgs::Point p_out;
  p_out.x = p_in.x(); p_out.y = p_in.y(); p_out.z = p_in.z();
  return p_out;
}
inline Eigen::Vector3d PerceptionDataMsgFactory::geoPt2Vec3d(const geometry_msgs::Point &p_in) {
  Eigen::Vector3d p_out;
  p_out.x() = p_in.x; p_out.y() = p_in.y; p_out.z() = p_in.z;
  return p_out;
}
// ------------------------------------------------------------------------------- //
// ------------------------- [PerceptionMergeFactory] ---------------------------- //
// ------------------------------------------------------------------------------- //
PerceptionMergeFactory::PerceptionMergeFactory() = default;

PerceptionMergeFactory::~PerceptionMergeFactory() = default;

void PerceptionMergeFactory::mergeInit(quadrotor_msgs::PerceptionMsg &map1, quadrotor_msgs::PerceptionMsg &map2,
                                       const double inflate_gridmap_resolution) {
  inflate_gridmap_resolution_ = inflate_gridmap_resolution;
  map_fac1_ = std::make_unique<PerceptionDataMsgFactory>(map1, inflate_gridmap_resolution_);
  map_fac2_ = std::make_unique<PerceptionDataMsgFactory>(map2, inflate_gridmap_resolution_);
}


bool PerceptionMergeFactory::merge() {
  INFO_MSG("*** [MapMergeFac]: start merge two maps");
  PoseGraph posegraph_new;
  // step0 : create new map var
  // step1 : get pairs of keypose between posegraph1 and posegraph2
  //         choose pairs which close enough to create connction
  // step2 : merge posegraph

  //! 1. 找到 posegraph1 and posegraph2 之间的可能连接边，phase1_add_edge是找到了符合条件连接边，phase2_add_edge是没有找到，退而求其次找到最近边
  unordered_map<int, vector<pair<int, double>>> keyposeB_to_keyposeA_edges;
  map<double, int> nearby_v_set_res;
  map<double, pair<int, int>> nearest_edge_res; // [gwq] 记录两个posegraph相邻最近的可能边（distance, <A_adress, B_adress>）
  bool phase1_add_edge = false;
  bool phase2_add_edge = false;
  if (map_fac2_->posegraph.getSize() == 0) phase1_add_edge = phase2_add_edge = true;
  // loop posegraph1 info
  for (int i = 0; i < map_fac1_->posegraph.getSize(); i++) {
    map_fac2_->posegraph.getNearTopoNodesInRange(map_fac1_->posegraph.getCorPos(i), 1.0, nearby_v_set_res);
    if (!nearby_v_set_res.empty()) {
      for (auto &it : nearby_v_set_res)
        keyposeB_to_keyposeA_edges[it.second].emplace_back(make_pair(i, it.first));
      phase1_add_edge = true;
    }
    else{
      nearby_v_set_res.clear();
      map_fac2_->posegraph.getNearTopoNodesInRange(map_fac1_->posegraph.getCorPos(i), 3.0, nearby_v_set_res); 
      if (!nearby_v_set_res.empty()){
        nearest_edge_res[nearby_v_set_res.begin()->first] = make_pair(i, nearby_v_set_res.begin()->second);
        phase2_add_edge = true;
      }
    }
  }

  //! 2. 以posegraph1为基准，将posegraph2中所有keypose的边连接到posegraph1中，并更新keypose_to_frontieridx
  map_fac1_->posegraph.deepCopy(posegraph_new, map_fac1_->posegraph.getSize());

  // loop posegraph2 key point
  int idx_offset = map_fac1_->posegraph.getSize();
  for (int i = 0; i < map_fac2_->posegraph.getSize(); i++) {
    int current_idx = idx_offset + i;
    posegraph_new.addKeypose(map_fac2_->posegraph.getCorPos(i), 0.0);
    if (keyposeB_to_keyposeA_edges.find(i) != keyposeB_to_keyposeA_edges.end())
      for (auto &it : keyposeB_to_keyposeA_edges[i])
        posegraph_new.addPoseEdge(it.first, current_idx, it.second); // 增加posegraph1和posegraph2之间的连接边
    // 遍历与posegraphB当前keypose相关的所有边界，并重新连接到posegraph_new
    for (auto &ftr_idx : map_fac2_->keypose_to_frontieridx[i])
      map_fac2_->perception_msg.ftr_msg.keypose_idx[ftr_idx] = current_idx;
  }
  // loop posegraph2 edge
  for (int i = 0; i < map_fac2_->posegraph.key_pose_list->size(); ++i)
    for (auto & edge : map_fac2_->posegraph.pose_edge[i])
      posegraph_new.addPoseEdge(i + idx_offset, edge.v_inx + idx_offset, edge.weight); // 增加posegraph2自己的连接边

  if (!phase1_add_edge && phase2_add_edge) {
    INFO_MSG_YELLOW("*** [MapMergeFac]: no edge between posegraphs, use force connect two nearest points");
    posegraph_new.addPoseEdge(nearest_edge_res.begin()->second.first,
                              nearest_edge_res.begin()->second.second + idx_offset,
                              nearest_edge_res.begin()->first);
  }
  else if (!phase1_add_edge && !phase2_add_edge) {
    INFO_MSG_YELLOW("*** [MapMergeFac]: no edge between posegraphs, use force connect two initial points");
    posegraph_new.addPoseEdge(0, idx_offset,
                               (map_fac1_->posegraph.getCorPos(0) - map_fac2_->posegraph.getCorPos(0)).norm()); // 连接posegraph1的第一个点和posegraph2的第一个点
  }

  posegraph_new.encodePosegraphData(map_res_.posegraph_msg);
  INFO_MSG("*** [MapMergeFac]: posegraph merged, new size:" << posegraph_new.getSize());

  //! step 3 : merge frontiers
  map_res_.ftr_msg = map_fac1_->perception_msg.ftr_msg;
  // loop map2 frontier info and merge them into map_res
  quadrotor_msgs::FrontierMsg &ftr_msg2 = map_fac2_->perception_msg.ftr_msg;
  quadrotor_msgs::FrontierMsg &ftr_res  = map_res_.ftr_msg;
  idx_offset = map_fac1_->perception_msg.ftr_msg.id.size();
  for (int i = 0; i < ftr_msg2.id.size(); i++) {
    int current_id = idx_offset + i;
    ftr_res.id.push_back(current_id);  // 此时，新merge in ftr 的id和idx相等
    before_ftrID_to_after_ftrID_[ftr_msg2.id[i]] = current_id;
  }
  mergeTwoFrontiersExceptId(ftr_res, ftr_msg2);
  INFO_MSG("*** [MapMergeFac]: frontiers merged, new size:" << ftr_res.id.size());

  // merge hgrid
  auto &hgrid_res = map_res_.hgrid_msg;
  hgrid_min_res_ = geoPt2Vec3d(map_fac1_->perception_msg.hgrid_msg.min).cwiseMin(geoPt2Vec3d(map_fac2_->perception_msg.hgrid_msg.min));
  hgrid_max_res_ = geoPt2Vec3d(map_fac1_->perception_msg.hgrid_msg.max).cwiseMax(geoPt2Vec3d(map_fac2_->perception_msg.hgrid_msg.max));
  hgrid_res.min = vec3d2GeoPt(hgrid_min_res_);
  hgrid_res.max = vec3d2GeoPt(hgrid_max_res_);
  mergeTwoHgrids();
  INFO_MSG("*** [MapMergeFac]: Hgrid merged.");
  return true;
}

void PerceptionMergeFactory::mergeTwoHgrids() {
  quadrotor_msgs::HgridMsg & hgrid_res = map_res_.hgrid_msg;
  quadrotor_msgs::HgridMsg & hgrid1    = map_fac1_->perception_msg.hgrid_msg;
  quadrotor_msgs::HgridMsg & hgrid2    = map_fac2_->perception_msg.hgrid_msg;

  // calculate new grid num and max adress!
  hgrid_res.multi_layer_hgrid = hgrid1.multi_layer_hgrid;
  hgrid_res.grid_size   = hgrid1.grid_size;
  grid_size_res_        = hgrid_res.grid_size;
  size_res_             = geoPt2Vec3d(hgrid_res.max) - geoPt2Vec3d(hgrid_res.min);
  grid_data_res_size_   = 0;
  for (int i = 0; i < 3; ++i){
    int num = ceil(size_res_[i] / grid_size_res_);
    resolution_res_[i] = size_res_[i] / double(num);
  }
  hgrid_res.resolution = vec3d2GeoPt(resolution_res_);
  for (int i = 0; i < 3; ++i) grid_num_res_(i) = ceil(size_res_(i) / resolution_res_(i));
  grid_data_res_size_ = grid_num_res_[0] * grid_num_res_[1] * grid_num_res_[2];
  INFO_MSG("*** [MapMergeFac][Hgrid]: new grid size      :" << grid_size_res_);
  INFO_MSG("*** [MapMergeFac][Hgrid]: new grid resolution:" << resolution_res_.transpose());
  INFO_MSG("*** [MapMergeFac][Hgrid]: new grid num       :" << grid_num_res_.transpose());
  INFO_MSG("*** [MapMergeFac][Hgrid]: new grid data size :" << grid_data_res_size_);
  // init memory
  initMemoryOfHgird(hgrid_res, grid_data_res_size_);
  /* start merge */
  // loop ervery new grid data
  Eigen::Vector3d cur_grid_center;
  int adr1, adr2;
  for (int i = 0; i < grid_data_res_size_; i++) {
    cur_grid_center = getNewGridPosFromAdress(i, 0.5);
    adr1 = map_fac1_->posToHgridAdress(cur_grid_center);
    adr2 = map_fac2_->posToHgridAdress(cur_grid_center);
    hgrid_res.id[i] = i;
    hgrid_res.local_id[i] = i;
    hgrid_res.center[i] = vec3d2GeoPt(cur_grid_center);
    hgrid_res.unknown_num[i] = std::min(hgrid1.unknown_num[adr1], hgrid2.unknown_num[adr2]);
    hgrid_res.frontier_num[i] = hgrid1.frontier_num[adr1] + hgrid2.frontier_num[adr2];
    hgrid_res.frontier_cell_nums[i].data.insert(hgrid_res.frontier_cell_nums[i].data.end(),
                                                 hgrid1.frontier_cell_nums[adr1].data.begin(),
                                                  hgrid1.frontier_cell_nums[adr1].data.end());
    hgrid_res.frontier_cell_nums[i].data.insert(hgrid_res.frontier_cell_nums[i].data.end(),
                                                 hgrid2.frontier_cell_nums[adr2].data.begin(),
                                                  hgrid2.frontier_cell_nums[adr2].data.end());
    hgrid_res.contained_frontier_ids[i].data.insert(hgrid_res.contained_frontier_ids[i].data.end(),
                                                     hgrid1.contained_frontier_ids[adr1].data.begin(),
                                                      hgrid1.contained_frontier_ids[adr1].data.end());
    for (int j = 0; j < hgrid2.contained_frontier_ids[adr2].data.size(); ++j)
      hgrid_res.contained_frontier_ids[i].data.push_back(before_ftrID_to_after_ftrID_[hgrid2.contained_frontier_ids[adr2].data[j]]);

    hgrid_res.is_updated[i] = false;
    hgrid_res.need_divide[i] = false;
    hgrid_res.active[i] = hgrid1.active[adr1] || hgrid2.active[adr2];
    hgrid_res.is_prev_relevant[i] = hgrid1.is_prev_relevant[adr1] || hgrid2.is_prev_relevant[adr2];
    hgrid_res.is_cur_relevant[i] = hgrid1.is_cur_relevant[adr1] || hgrid2.is_cur_relevant[adr2];
    hgrid_res.is_covered[i] = hgrid1.is_covered[adr1] || hgrid2.is_covered[adr2];
  }
}

void PerceptionMergeFactory::getMergeResult(quadrotor_msgs::PerceptionMsg &map_res) {map_res = map_res_;}

// inc = 0.5 center
Eigen::Vector3d PerceptionMergeFactory::getNewGridPosFromAdress(const int &adress, const double &inc) {
  Eigen::Vector3i idx;
  Eigen::Vector3d pos;
  int tmp_adr = adress;
  const int a = grid_num_res_(1) * grid_num_res_(2);
  const int b = grid_num_res_(2);
  idx[0] = tmp_adr / a;
  tmp_adr = tmp_adr % a;
  idx[1] = tmp_adr / b;
  idx[2] = tmp_adr % b;
  for (int i = 0; i < 3; ++i) pos(i) = (idx[i] + 0.5) * resolution_res_[i] + hgrid_min_res_(i);
  return pos;
}

inline void PerceptionMergeFactory::geoPt2Vec3d(const geometry_msgs::Point &p_in, Eigen::Vector3d &p_out) {
  p_out.x() = p_in.x; p_out.y() = p_in.y; p_out.z() = p_in.z;
}
inline void PerceptionMergeFactory::vec3d2GeoPt(const Eigen::Vector3d &p_in, geometry_msgs::Point &p_out) {
  p_out.x = p_in.x(); p_out.y = p_in.y(); p_out.z = p_in.z();
}
inline geometry_msgs::Point PerceptionMergeFactory::vec3d2GeoPt(const Eigen::Vector3d &p_in) {
  geometry_msgs::Point p_out;
  p_out.x = p_in.x(); p_out.y = p_in.y(); p_out.z = p_in.z();
  return p_out;
}
inline Eigen::Vector3d PerceptionMergeFactory::geoPt2Vec3d(const geometry_msgs::Point &p_in) {
  Eigen::Vector3d p_out;
  p_out.x() = p_in.x; p_out.y() = p_in.y; p_out.z() = p_in.z;
  return p_out;
}

// FrontierMsg是frontier所有成员变量的vector的集合
void PerceptionMergeFactory::mergeTwoFrontiersExceptId(quadrotor_msgs::FrontierMsg &ftr_res,
                                                       const quadrotor_msgs::FrontierMsg &ftr_msg2) {
  ftr_res.cells.insert(ftr_res.cells.end(), ftr_msg2.cells.begin(), ftr_msg2.cells.end());
  ftr_res.filtered_cells.insert(ftr_res.filtered_cells.end(), ftr_msg2.filtered_cells.begin(), ftr_msg2.filtered_cells.end());
  ftr_res.average.insert(ftr_res.average.end(), ftr_msg2.average.begin(), ftr_msg2.average.end());
  ftr_res.normal.insert(ftr_res.normal.end(), ftr_msg2.normal.begin(), ftr_msg2.normal.end());
  ftr_res.keypose_idx.insert(ftr_res.keypose_idx.end(), ftr_msg2.keypose_idx.begin(), ftr_msg2.keypose_idx.end());
  ftr_res.viewpoints_pos.insert(ftr_res.viewpoints_pos.end(), ftr_msg2.viewpoints_pos.begin(), ftr_msg2.viewpoints_pos.end());
  ftr_res.viewpoints_yaw.insert(ftr_res.viewpoints_yaw.end(), ftr_msg2.viewpoints_yaw.begin(), ftr_msg2.viewpoints_yaw.end());
  ftr_res.viewpoints_visib_num.insert(ftr_res.viewpoints_visib_num.end(), ftr_msg2.viewpoints_visib_num.begin(), ftr_msg2.viewpoints_visib_num.end());
  ftr_res.box_min_.insert(ftr_res.box_min_.end(), ftr_msg2.box_min_.begin(), ftr_msg2.box_min_.end());
  ftr_res.box_max_.insert(ftr_res.box_max_.end(), ftr_msg2.box_max_.begin(), ftr_msg2.box_max_.end());
  ftr_res.paths.insert(ftr_res.paths.end(), ftr_msg2.paths.begin(), ftr_msg2.paths.end());
  ftr_res.costs.insert(ftr_res.costs.end(), ftr_msg2.costs.begin(), ftr_msg2.costs.end());
  ftr_res.path_to_home_3.insert(ftr_res.path_to_home_3.end(), ftr_msg2.path_to_home_3.begin(), ftr_msg2.path_to_home_3.end());
  ftr_res.path_to_home_4.insert(ftr_res.path_to_home_4.end(), ftr_msg2.path_to_home_4.begin(), ftr_msg2.path_to_home_4.end());
  ftr_res.cost_to_home.insert(ftr_res.cost_to_home.end(), ftr_msg2.cost_to_home.begin(), ftr_msg2.cost_to_home.end());
  ftr_res.topo_blacklist.insert(ftr_res.topo_blacklist.end(), ftr_msg2.topo_blacklist.begin(), ftr_msg2.topo_blacklist.end());
  ftr_res.ftr_blacklist.insert(ftr_res.ftr_blacklist.end(), ftr_msg2.ftr_blacklist.begin(), ftr_msg2.ftr_blacklist.end());
}

void PerceptionMergeFactory::initMemoryOfHgird(quadrotor_msgs::HgridMsg &hgrid_res, const int& buffer_size) {
  hgrid_res.id.resize(buffer_size);
  hgrid_res.local_id.resize(buffer_size);
  hgrid_res.unknown_num.resize(buffer_size);
  hgrid_res.frontier_num.resize(buffer_size);
  hgrid_res.center.resize(buffer_size);
  hgrid_res.frontier_cell_nums.resize(buffer_size);
  hgrid_res.contained_frontier_ids.resize(buffer_size);
  hgrid_res.is_updated.resize(buffer_size);
  hgrid_res.need_divide.resize(buffer_size);
  hgrid_res.active.resize(buffer_size);
  hgrid_res.is_prev_relevant.resize(buffer_size);
  hgrid_res.is_cur_relevant.resize(buffer_size);
  hgrid_res.is_covered.resize(buffer_size);
}
}
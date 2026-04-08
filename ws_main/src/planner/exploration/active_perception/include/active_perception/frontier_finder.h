#ifndef _FRONTIER_FINDER_H_
#define _FRONTIER_FINDER_H_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <memory>
#include <vector>
#include <list>
#include <utility>
#include <map_interface/map_interface.hpp>
#include <active_perception/hgrid.h>
#include <perception_utils/perception_utils.h>
#include <scene_graph/scene_graph.h>
#include <quadrotor_msgs/FrontierMsg.h>
#include <../include/active_perception/ftr_data_structure.h>
#include <../include/active_perception/ikd_Tree.h>
#include <../include/active_perception/viewpoint_handler.h>

// #include <active_perception/visualization.hpp>
using Eigen::Vector3d;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;
using std::list;
using std::pair;

namespace ego_planner {

class FrontierFinder {
public:
  FrontierFinder(const MapInterface::Ptr& map, ros::NodeHandle& nh, SceneGraph::Ptr scene_graph);
  ~FrontierFinder();

  typedef std::shared_ptr<FrontierFinder> Ptr;
  // ！Frontier Operation
  void reCalculateAllFtrTopo(const Eigen::Vector3d &cur_pos);
  //! Frontier Generation
  void searchFrontiers(const Vector3d& c_pos);
  void computeFrontiersToVisit(const Vector3d& c_pos);

  //! Frontier Infomation get
  void getFrontiers(vector<vector<Vector3d>>& clusters);
  void getFrontiersWithInfo(vector<Frontier>& clusters);
  void getDormantFrontiers(vector<vector<Vector3d>>& clusters);
  void getFrontierBoxes(vector<pair<Vector3d, Vector3d>>& boxes);
  bool isAnyFrontierCovered(const Vector3d& c_pos);


  void getTopoPath(const Vector3d& start_pose, const Vector3d& end_pose, vector<Vector3d>& path, double& dis);
  void getPathWithTopo(const PolyHedronPtr& start_poly, const Eigen::Vector3d & start_pose,
                   const PolyHedronPtr& end_poly, const Eigen::Vector3d & end_pose, vector<Eigen::Vector3d> &path, double& dis);
  // void getNearTopoNodes(PoseGraph& posegraph, const Vector3d& cur_pose, const double radius, unordered_map<int, int>& blacklist,
  //                       bool enable_visible_check, map<double, int>& nearby_v_set);
  // void getNearTopoNodes(PoseGraph& posegraph, const Vector3d& cur_pose, const double radius, bool enable_visible_check, map<double, int>& nearby_v_set);
  void setLastAddIdx(const int &idx);

  //! Hgrid Related
  inline void setHgrid(shared_ptr<HGrid> hgird_ptr) { hgrid_ = hgird_ptr; };

  //! Viewpoint Related
  // Get several viewpoints for a subset of frontiers
  void getViewpointsInfo(const Vector3d& cur_pos, const vector<int>& ids, const int& view_num,
                         const double& max_decay, vector<vector<Vector3d>>& points,
                         vector<vector<double>>& yaws);

  //! Cost Matrix
  void updateFrontierCostMatrix();
  void updateFrontierCostMatrix(list<Frontier> &frontiers);
  void getFullCostMatrix(const Vector3d& cur_pos, const Vector3d& cur_vel, const double& cur_yaw,
                         Eigen::MatrixXd& mat);

  //! Path Search
  void getPathForTour(const Vector3d& pos, const vector<int>& frontier_ids, 
                      vector<Vector3d>& path_all, vector<Vector3d>& path_next_goal);
  
  //! Others
  void wrapYaw(double& yaw);
  bool isNearOcc(const Vector3d& pos);

  // delete frontier interface
  void frontierForceDelete(const Frontier &frontier_to_delete);
  void frontierForceDeleteAll();
  // void addFtrBlacklist(const Eigen::Vector3d& pos) { ftr_blacklist_.push_back(pos); }
  bool isBlacklisted(const Frontier& frontier);

  PerceptionUtils::Ptr percep_utils_;
  double               far_dis_thres_;

public:
  //! Frontier Util
  void expandFrontier(const Eigen::Vector3i& first /* , const int& depth, const int& parent_id */);
  void splitLargeFrontiers(list<Frontier>& frontiers);
  bool splitHorizontally(const Frontier& frontier, list<Frontier>& splits);
  // void mergeFrontiers(Frontier& ftr1, const Frontier& ftr2);
  bool isFrontierChanged(const Frontier& ft); 
  bool isWellObserved(const Frontier& ft, const Vector3d& pos);
  bool isHalfInLocalMap(const Frontier& ft);
  bool haveOverlap(const Vector3d& min1, const Vector3d& max1, const Vector3d& min2,
                   const Vector3d& max2);
  void computeFrontierInfo(Frontier& frontier);
  void computeNormal(Frontier& ftr);
  void downsample(const vector<Vector3d>& cluster_in, vector<Vector3d>& cluster_out);
  bool checkFrontiers(Frontier& frontier, const Vector3d& c_pos);

  //! Viewpoint
  void sampleViewpoints(Frontier& frontier);
  int  countVisibleCells(const Vector3d& pos, const double& yaw, const vector<Vector3d>& cluster);


  // skeleton
  void setCurrentTopoNode(const PolyHedronPtr& topo_node) {
      if (topo_node != nullptr) {
          last_mount_topo_ = cur_mount_topo_;
          cur_mount_topo_ = topo_node;
      }
  };
  void updateSceneGraphWithFtr();
  
  //! Neighbor
  bool isNearUnknown(const Vector3d& pos);
  vector<Eigen::Vector3i> sixNeighbors(const Eigen::Vector3i& voxel);
  vector<Eigen::Vector3i> tenNeighbors(const Eigen::Vector3i& voxel);
  vector<Eigen::Vector3i> allNeighbors(const Eigen::Vector3i& voxel);
  bool isNeighborUnknown(const Eigen::Vector3i& voxel);


  // Wrapper of sdf map
  int  toadr(const Eigen::Vector3i& idx);
  bool knownfree(const Eigen::Vector3i& idx);
  bool inmap(const Eigen::Vector3i& idx);

  // Deprecated
  bool canBeMerged(const Frontier& ftr1, const Frontier& ftr2);

  // utils
  void geometryPoint2EigenVector3d(const geometry_msgs::Point &p_in, Eigen::Vector3d &p_out);
  void eigenVector3d2GeometryPoint(const Eigen::Vector3d &p_in, geometry_msgs::Point &p_out);
  geometry_msgs::Point eigenVector3d2GeometryPoint(const Eigen::Vector3d &p_in);
  Eigen::Vector3d geometryPoint2EigenVector3d(const geometry_msgs::Point &p_in);

  //! Data
  vector<char>                      frontier_flag_;
  list<Frontier>                    frontiers_, dormant_frontiers_, tmp_frontiers_, remove_frontiers_;
  vector<int>                       removed_ids_;
  list<Frontier>::iterator          first_new_ftr_;
  vector<Eigen::Vector3d>           ftr_blacklist_;
  PolyHedronPtr                     cur_mount_topo_{nullptr}, last_mount_topo_{nullptr};

  ViewpointsHandler::Ptr            vp_handler_;

  //! Sub-Class
  MapInterface::Ptr                 edt_env_;
  unique_ptr<RayCaster>             raycaster_;
  HGrid::Ptr                        hgrid_;
  SceneGraph::Ptr                   scene_graph_;

  visualization::Visualization::Ptr visPtr_;

  //! Params
  // cluster params
  double                            ftr_floor_, ftr_ceil_;
  int                               cluster_min_;
  double                            cluster_size_xy_, cluster_size_z_;
  int                               down_sample_;
  bool                              is_print_info_;
  double                            ftr_blacklist_radius_;  

  // viewpoint params
  double                            candidate_rmax_, candidate_rmin_, candidate_dphi_, min_candidate_dist_,
                                    min_candidate_clearance_xy_, min_candidate_clearance_z_, min_candidate_clearance_unknown_xy_, min_candidate_clearance_unknown_z_;
  double                            candidate_zmax_, candidate_zmin_;
  int                               min_visib_num_, candidate_rnum_, candidate_phinum_;
  int                               candidate_znum_;
  double                            min_view_finish_fraction_, resolution_;
  
  // posegraph params
  double                            connected_min_seqential_dis_, connected_max_euler_dis_;
  double                            keypose_min_gap_;
  int                               self_last_add_inx_;
  int                               cur_posegra_inx_;
};

}  // namespace ego_planner
#endif
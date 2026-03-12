//
// Created by gwq on 9/2/25.
//

#ifndef FTR_DATA_STRUCTURE_H
#define FTR_DATA_STRUCTURE_H

#include <Eigen/Eigen>
#include <list>
#include <vector>
#include <unordered_map>
#include <memory>
#include <scene_graph/data_structure.h>
#include <scene_graph/skeleton_cluster.h>

using Eigen::Vector3d;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;
using std::list;
using std::pair;


namespace ego_planner{
    // used for unordered_map
struct Vector3iHash {
    std::size_t operator()(const Eigen::Vector3i& vec) const {
      std::hash<int> hasher;
      size_t seed = 0;
      seed ^= hasher(vec.x()) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
      seed ^= hasher(vec.y()) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
      seed ^= hasher(vec.z()) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
      return seed;
    }
};

// Viewpoint to cover a frontier cluster
struct Viewpoint {
  typedef std::shared_ptr<Viewpoint> Ptr;
  unsigned int id_;
  // Position and heading
  Vector3d pos_;
  double yaw_;
  // double fraction_;
  int visib_num_;
  // TSP calculate
  std::map<unsigned int, double> costs_;
  double cost_to_home_;
  // topo
  PolyHedronPtr topo_father_{nullptr};

  Viewpoint(){}
  Viewpoint(Vector3d pos, double yaw, int visib_num): pos_(pos), yaw_(yaw), visib_num_(visib_num) {}
  Viewpoint(const Viewpoint& vp) {
    pos_          = vp.pos_;
    yaw_          = vp.yaw_;
    visib_num_    = vp.visib_num_;
    costs_        = vp.costs_;
    cost_to_home_ = vp.cost_to_home_;
  }
  void mount(const PolyHedronPtr& topo_father) {
      topo_father_ = topo_father;
  }
};

// A frontier cluster, the viewpoints to cover it
struct Frontier {
  // Complete voxels belonging to the cluster
  vector<Vector3d> cells_;
  // down-sampled voxels filtered by voxel grid filter
  vector<Vector3d> filtered_cells_;
  // Average position of all voxels
  Vector3d average_;
  // The normal
  Vector3d normal_;
  // Idx of cluster
  int id_;
  // The keypose it belong to
  int keypose_inx_;
  PolyHedronPtr topo_father_;
  // Viewpoints that can cover the cluster
  vector<Viewpoint> viewpoints_;
  // Bounding box of cluster, center & 1/2 side length
  Vector3d box_min_, box_max_;
  // Path and cost from this cluster to other clusters
  list<vector<Vector3d>> paths_;
  list<double> costs_;
  vector<Eigen::Vector4d> path_to_home_; // (x,y,z,state) state = 0.0(Ground) state = 255.0(Air)
  double cost_to_home_;

// LLM based
  std::vector<double> costs_2;
  std::vector<std::vector<Eigen::Vector3d>> paths_2;
  double cost_to_home_2;

  // to avoid repeating exploration on topological paths
  std::unordered_map<int, int> topo_blacklist_;
};
}



#endif //FTR_DATA_STRUCTURE_H

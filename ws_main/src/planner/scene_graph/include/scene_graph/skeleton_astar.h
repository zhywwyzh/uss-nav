//
// Created by gwq on 25-3-18.
//

#ifndef SKELETON_ASTAR_H
#define SKELETON_ASTAR_H

#include "Eigen/Eigen"
#include "ros/ros.h"
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"
#include "unordered_map"
#include "queue"
#include "../include/scene_graph/data_structure.h"

namespace skeleton_astar{

  class AstarNode{
  public:
    typedef std::shared_ptr<AstarNode> Ptr;
    PolyHedronPtr              polyhedron_;
    Eigen::Vector3d            pos_;
    double                     cost_g_, cost_h_, cost_f_;
    std::shared_ptr<AstarNode> parent_;
    AstarNode(PolyHedronPtr poly, double cost_g, double cost_h, std::shared_ptr<AstarNode> parent):
      polyhedron_(poly), cost_g_(cost_g), cost_h_(cost_h), parent_(parent){
      cost_f_ = cost_g_ + cost_h_;
      pos_ = polyhedron_->center_;
    }
    ~AstarNode(){}

    void calF(){
      cost_f_ = cost_g_ + cost_h_;
    }
  };

  class SkeletonAstar{
  public:
    typedef std::shared_ptr<SkeletonAstar> Ptr;
    struct AstarNodeCompare{
      bool operator()(const AstarNode::Ptr& node1, const AstarNode::Ptr& node2) const{
        return node1->cost_f_ > node2->cost_f_;
      }
    };
    struct Vector3dHash{
      std::size_t operator()(const Eigen::Vector3d& vector) const {
        std::size_t h1 = std::hash<double>()(vector.x());
        std::size_t h2 = std::hash<double>()(vector.y());
        std::size_t h3 = std::hash<double>()(vector.z());
        return h1 ^ (h2 << 1) ^ (h3 << 2);
      }
    };

    SkeletonAstar(ros::NodeHandle& nh){
      INFO_MSG_GREEN("[SkeletonAstar] Init complete !");
      nh_ = nh;
      tie_breaker_ = 1.0 + 1.0 / 1000;
      open_list_map_.clear();
      closed_list.clear();
      vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("skeleton_vis", 1);
    }
    ~SkeletonAstar(){}
    inline double getEuclHeu(Eigen::Vector3d pos_a, Eigen::Vector3d pos_b);
    bool astarSearch(PolyHedronPtr poly_start, PolyHedronPtr poly_end);
    void getPath(std::vector<Eigen::Vector3d>& path);
    void getNeighborPolyhedronsNotInCloseList(AstarNode::Ptr cur_node, std::vector<AstarNode::Ptr>& neighbor_nodes);
    void visualizePath();

  private:
    double tie_breaker_;
    ros::NodeHandle nh_;
    ros::Publisher  vis_pub_;
    Eigen::Vector3d end_pos_;
    std::vector<Eigen::Vector3d> path_;
    std::priority_queue<AstarNode::Ptr, std::vector<AstarNode::Ptr>, AstarNodeCompare> open_list_;
    std::unordered_map<Eigen::Vector3d, AstarNode::Ptr, Vector3dHash> open_list_map_;
    std::unordered_map<Eigen::Vector3d, AstarNode::Ptr, Vector3dHash> closed_list;
  };
}

#endif //SKELETON_ASTAR_H

//
// Created by gwq on 25-3-18.
//

#include "../include/scene_graph/skeleton_astar.h"

namespace skeleton_astar{

inline double SkeletonAstar::getEuclHeu(Eigen::Vector3d pos_a, Eigen::Vector3d pos_b){
  return tie_breaker_ * (pos_a - pos_b).norm();
}

void SkeletonAstar::getPath(std::vector<Eigen::Vector3d>& path){
  path = this->path_;
}

void SkeletonAstar::getNeighborPolyhedronsNotInCloseList(AstarNode::Ptr cur_node, std::vector<AstarNode::Ptr>& neighbor_nodes){
  for (const auto& edge : cur_node->polyhedron_->edges_){
    if (edge.poly_nxt_ != nullptr){
      if (closed_list.find(edge.poly_nxt_->center_) != closed_list.end()) continue;
      double nxt_cost_g = cur_node->cost_g_ + getEuclHeu(cur_node->pos_, edge.poly_nxt_->center_);
      AstarNode::Ptr neighbor_node = std::make_shared<AstarNode>
        (edge.poly_nxt_, nxt_cost_g, getEuclHeu(edge.poly_nxt_->center_, end_pos_), cur_node);
      neighbor_nodes.push_back(neighbor_node);
    }
  }
}

bool SkeletonAstar::astarSearch(PolyHedronPtr poly_start, PolyHedronPtr poly_end){
  closed_list.clear();
  open_list_map_.clear();
  while (!open_list_.empty())
    open_list_.pop();
  path_.clear();
  end_pos_ = poly_end->center_;

  AstarNode::Ptr start_node = std::make_shared<AstarNode>
      (poly_start, 0, getEuclHeu(poly_start->center_, poly_end->center_), nullptr);
  open_list_.push(start_node);
  open_list_map_[start_node->pos_] = start_node;

  while (!open_list_.empty()){
    // get node that has minimum f_score
    AstarNode::Ptr current_node = open_list_.top();
    // IF reach goal
    if ((current_node->pos_ - end_pos_).norm() < 1e-2){
      AstarNode::Ptr temp_node = current_node;
      while (temp_node!= nullptr){
        path_.push_back(temp_node->pos_);
        temp_node = temp_node->parent_;
      }
      std::reverse(path_.begin(), path_.end());
      return true;
    }

    open_list_.pop();
    open_list_map_.erase(current_node->pos_);
    closed_list[current_node->pos_] = current_node;

    // expand current node
    std::vector<AstarNode::Ptr> neighbor_nodes;
    getNeighborPolyhedronsNotInCloseList(current_node, neighbor_nodes);
    for (auto & neibor_node : neighbor_nodes){
      // close list check has been done in getNeighborPolyhedronsNotInCloseList
      if (open_list_map_.find(neibor_node->pos_) != open_list_map_.end()){
        AstarNode::Ptr temp_node = open_list_map_[neibor_node->pos_];
        if (temp_node->cost_g_ > neibor_node->cost_g_){
          temp_node->parent_ = current_node;
          temp_node->cost_g_ = neibor_node->cost_g_;
        }
      }else{
        // if not in open list, add to open list
        open_list_.push(neibor_node);
        open_list_map_[neibor_node->pos_] = neibor_node;
      }
    }
  }
  path_.clear();
  return false;
}

void SkeletonAstar::visualizePath(){
  if (path_.empty()) return;
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker marker_pt, marker_line;
  marker_pt.header.frame_id = "world";
  marker_pt.header.stamp = ros::Time::now();
  marker_pt.ns = "skeleton_astar";
  marker_pt.id = 0;
  marker_pt.type = visualization_msgs::Marker::POINTS;
  marker_pt.action = visualization_msgs::Marker::ADD;
  marker_pt.pose.orientation.w = 1.0;
  marker_pt.scale.x = 0.2; marker_pt.scale.y = 0.2; marker_pt.scale.z = 0.2;
  marker_pt.color.r = 1.0; marker_pt.color.g = 0.0; marker_pt.color.b = 0.0;
  marker_pt.color.a = 1.0;

  marker_line.header.frame_id = "world";
  marker_line.header.stamp = ros::Time::now();
  marker_line.ns = "skeleton_astar";
  marker_line.id = 1;
  marker_line.type = visualization_msgs::Marker::LINE_STRIP;
  marker_line.action = visualization_msgs::Marker::ADD;
  marker_line.pose.orientation.w = 1.0;
  marker_line.scale.x = 0.1; marker_line.scale.y = 0.1; marker_line.scale.z = 0.1;
  marker_line.color.r = 0.0; marker_line.color.g = 0.0; marker_line.color.b = 0.8;
  marker_line.color.a = 1.0;

  // point
  for (const auto& pt : path_){
    geometry_msgs::Point p;
    p.x = pt.x(); p.y = pt.y(); p.z = pt.z();
    marker_pt.points.push_back(p);
  }
  // lines
  for (int i = 0; i < path_.size() - 1; i++){
    geometry_msgs::Point p1, p2;
    p1.x = path_[i].x(); p1.y = path_[i].y(); p1.z = path_[i].z();
    p2.x = path_[i+1].x(); p2.y = path_[i+1].y(); p2.z = path_[i+1].z();
    marker_line.points.push_back(p1);
    marker_line.points.push_back(p2);
  }
  marker_array.markers.push_back(marker_pt);
  marker_array.markers.push_back(marker_line);

  vis_pub_.publish(marker_array);
}

}
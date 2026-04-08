#include <active_perception/graph_node.h>


// #include <path_searching/astar2.h>
// #include <plan_env/sdf_map.h>
// #include <plan_env/raycast.h>

namespace ego_planner {
// Static data
double ViewNode::vm_;
double ViewNode::am_;
double ViewNode::yd_;
double ViewNode::ydd_;
double ViewNode::w_dir_;
double ViewNode::sensor_range_;

dyn_a_star::AStar::Ptr  ViewNode::a_star_;
MapInterface::Ptr       ViewNode::map_;

// Graph node for viewpoints planning
ViewNode::ViewNode(const Vector3d& p, const double& y) {
  pos_ = p;
  yaw_ = y;
  parent_ = nullptr;
  vel_.setZero();  // vel is zero by default, should be set explicitly
}

double ViewNode::costTo(const ViewNode::Ptr& node) {
  vector<Vector3d> path;
  double c = ViewNode::computeCost(pos_, node->pos_, yaw_, node->yaw_, vel_, yaw_dot_, path);
  // std::cout << "cost from " << id_ << " to " << node->id_ << " is: " << c << std::endl;
  return c;
}

bool ViewNode::searchPath(const Vector3d& p1, const Vector3d& p2, vector<Vector3d>& path, double& dis_res) 
{
  path.clear();

  // If the svp is directly visible and within the sensor_range
  if ((p1 - p2).norm() < 2.0 * sensor_range_ &&
      map_->isVisible(p1, p2))
  {
    // std::cout << "searchPath: directly visible" << std::endl;
    path.push_back(p1);
    path.push_back(p2);
    dis_res = (p1 - p2).norm();
    return true;
  }
  // Else calculate using a_star
  else
  {
    dyn_a_star::ASTAR_RET ret = a_star_->AstarSearch(map_->getResolution(), p1, p2);
    if (ret == dyn_a_star::ASTAR_RET::SUCCESS)
    {
      path = a_star_->getPath();
      double dis = 0.0;
      for (size_t i = 0; i < path.size() - 1; i++) dis += (path[i] - path[i+1]).norm();
      dis_res = dis;
      return true;
    }
    else
    {
      // Use Astar early termination cost as an estimate
      ROS_WARN_STREAM("Astar failed to find path, using early termination cost as an estimate");

      path = { p1, p2 };
      return false;
    }
  }
}

double ViewNode::computeCost(const Vector3d& p1, const Vector3d& p2, const double& y1, const double& y2,
                             const Vector3d& v1, const double& yd1, vector<Vector3d>& path) 
{
  // Cost of position change
  double pos_cost;
  ViewNode::searchPath(p1, p2, path, pos_cost) / vm_;

  // Consider velocity change
  if (v1.norm() > 1e-3)
  {
    Vector3d dir = (p2 - p1).normalized();
    Vector3d vdir = v1.normalized();
    double diff = acos(vdir.dot(dir));
    pos_cost += w_dir_ * diff;
    // double vc = v1.dot(dir);
    // pos_cost += w_dir_ * pow(vm_ - fabs(vc), 2) / (2 * vm_ * am_);
    // if (vc < 0)
    //   pos_cost += w_dir_ * 2 * fabs(vc) / am_;
  }

  // Cost of yaw change
  double diff = fabs(y2 - y1);
  diff = min(diff, 2 * M_PI - diff);
  double yaw_cost = diff / yd_;
  return max(pos_cost, yaw_cost);

  // // Consider yaw rate change
  // if (fabs(yd1) > 1e-3)
  // {
  //   double diff1 = y2 - y1;
  //   while (diff1 < -M_PI)
  //     diff1 += 2 * M_PI;
  //   while (diff1 > M_PI)
  //     diff1 -= 2 * M_PI;
  //   double diff2 = diff1 > 0 ? diff1 - 2 * M_PI : 2 * M_PI + diff1;
  // }
  // else
  // {
  // }
}
}
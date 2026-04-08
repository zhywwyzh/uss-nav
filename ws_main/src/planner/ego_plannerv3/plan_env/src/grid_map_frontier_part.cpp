#include "plan_env/grid_map.h"

void GridMap::getUpdatedBox(Eigen::Vector3d& bmin, Eigen::Vector3d& bmax) 
{
  Eigen::Vector3i min_esdf = min3i(max3i(md_.update_range_lb3i_ /*- Eigen::Vector3i(5, 5, 5)*/, md_.rb_lb3i_),
                                   pos2GlobalIdx(md_.camera_pos_) - Eigen::Vector3i::Ones());
  Eigen::Vector3i max_esdf = max3i(min3i(md_.update_range_ub3i_ /*+ Eigen::Vector3i(5, 5, 5)*/, md_.rb_ub3i_),
                                   pos2GlobalIdx(md_.camera_pos_) + Eigen::Vector3i::Ones());

  bmin = globalIdx2Pos(min_esdf);
  bmax = globalIdx2Pos(max_esdf);
}

void GridMap::getUpdatedBoxIdx(Eigen::Vector3i& bmin_inx, Eigen::Vector3i& bmax_inx) 
{

  bmin_inx = min3i(max3i(md_.update_range_lb3i_ /*- Eigen::Vector3i(5, 5, 5)*/, md_.rb_lb3i_),
                   pos2GlobalIdx(md_.camera_pos_) - Eigen::Vector3i::Ones());
  bmax_inx = max3i(min3i(md_.update_range_ub3i_ /*+ Eigen::Vector3i(5, 5, 5)*/, md_.rb_ub3i_),
                   pos2GlobalIdx(md_.camera_pos_) + Eigen::Vector3i::Ones()); 
}

void GridMap::publishUpdateRange() 
{
  Eigen::Vector3d grid_min_pos, grid_max_pos, cube_pos, cube_scale;
  visualization_msgs::Marker mk;
  getUpdatedBox(grid_min_pos, grid_max_pos);

  cube_pos = 0.5 * (grid_min_pos + grid_max_pos);
  cube_scale = grid_max_pos - grid_min_pos;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::CUBE;
  mk.action = visualization_msgs::Marker::ADD;
  mk.id = 0;
  mk.pose.position.x = cube_pos(0);
  mk.pose.position.y = cube_pos(1);
  mk.pose.position.z = cube_pos(2);
  mk.scale.x = cube_scale(0);
  mk.scale.y = cube_scale(1);
  mk.scale.z = cube_scale(2);
  mk.color.a = 0.3;
  mk.color.r = 1.0;
  mk.color.g = 0.0;
  mk.color.b = 0.0;
  mk.pose.orientation.w = 1.0;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;

  update_range_pub_.publish(mk);
}

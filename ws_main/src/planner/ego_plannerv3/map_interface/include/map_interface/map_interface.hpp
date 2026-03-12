#ifndef _MAP_INTERFACE_H
#define _MAP_INTERFACE_H

#include <Eigen/Eigen>
#include <iostream>
#include <ros/console.h>
#include <ros/ros.h>
#include <string>
#include <plan_env/grid_map.h>
#include "ros/package.h"
#include <map_interface/parameter_server.hpp>
#include <path_searching/dyn_a_star.h>
#include <pcl/surface/gp3.h>

#define INFO_MSG(str) do {std::cout << str << std::endl; } while(false)
#define INFO_MSG_RED(str) do {std::cout << "\033[31m" << str << "\033[0m" << std::endl; } while(false)
#define INFO_MSG_GREEN(str) do {std::cout << "\033[32m" << str << "\033[0m" << std::endl; } while(false)
#define INFO_MSG_YELLOW(str) do {std::cout << "\033[33m" << str << "\033[0m" << std::endl; } while(false)
#define INFO_MSG_BLUE(str) do {std::cout << "\033[34m" << str << "\033[0m" << std::endl; } while(false)

namespace ego_planner {

// In order to maintain code easily, 
// we use an interface to isolate exploration from the underlying map,
// facilitating map code replacement.
class MapInterface{
 public:
  enum OCCUPANCY { FREE, OCCUPIED, UNKNOWN };

 private:
  MapManager::Ptr        map_;
  dyn_a_star::AStar::Ptr a_star_;
  std::shared_ptr<parameter_server::ParaeterSerer> para_ptr_;
  double resolution_;
  Eigen::Vector3d map_min_, map_max_;
  int drone_id_;
  string para_file_path_;

 public:
  MapInterface(ros::NodeHandle& nh, MapManager::Ptr map):map_(map)
  {
    // get global box range from parameter file
    std::string package_name("exploration_manager");
    para_file_path_ = ros::package::getPath(package_name) + "/launch/explore_para.yaml";
    INFO_MSG("[MapInterface] explore_para_path: " << para_file_path_);
    para_ptr_ = std::make_shared<parameter_server::ParaeterSerer>(para_file_path_);
    nh.param("fsm/drone_id", drone_id_, -1);
    resetGlobalBox();
    resolution_ = map_->sml_->getResolution();
    a_star_ = std::make_shared<dyn_a_star::AStar>();
    a_star_->initAstar(map_, Eigen::Vector3i(50, 50, 50));
  }

  typedef std::shared_ptr<MapInterface> Ptr;

  void   resetGlobalBox();
  void   resetGlobalBox(const Eigen::Vector3d & box_min, const Eigen::Vector3d & box_max);
  void   getGlobalBox(Eigen::Vector3d & box_min, Eigen::Vector3d & box_max) const;
  double getResolution() const;
  void   getUpdatedBox(Eigen::Vector3d& bmin, Eigen::Vector3d& bmax) const;
  void   getUpdatedBoxIdx(Eigen::Vector3i& bmin_inx, Eigen::Vector3i& bmax_inx) const;
  bool searchPath(const Eigen::Vector3d& start_pos, const Eigen::Vector3d& end_pos,
                  std::vector<Eigen::Vector3d>& path, double step_size);
  bool   searchPathConsiderUKRegion(const Eigen::Vector3d& start_pos, const Eigen::Vector3d& end_pos,
                                    std::vector<Eigen::Vector3d>& path, double step_size);

  Eigen::Vector3i pos2GlobalIdx(const Eigen::Vector3d &pos)   const;
  Eigen::Vector3d globalIdx2Pos(const Eigen::Vector3i &id)    const;
  size_t          globalIdx2BufIdx(const Eigen::Vector3i &id) const;

  void setTarget(const Eigen::Vector3d &target_pos, const bool &target_pos_valid);

  int getOccupancy(const Eigen::Vector3i &idx) const;
  int getOccupancy(const Eigen::Vector3d &pos) const;
  int getInflateOccupancy(const Eigen::Vector3d &pos) const;
  pair<int, int> getRawInflateOccupancy(const Eigen::Vector3d &pos) const;

  bool isInGlobalMap(const Eigen::Vector3i &idx) const;
  bool isInGlobalMap(const Eigen::Vector3d &pos) const;

  bool isInLocalMap(const Eigen::Vector3i &idx) const;
  bool isInLocalMap(const Eigen::Vector3d &pos) const;

  bool isVisible(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, double check_res = 0.0) const;

  inline int getVoxelNum() const { return map_->sml_->getVoxelNum(); };

  inline void mtxLock()   { map_->sml_->mtx_.lock(); };
  inline void mtxUnlock() { map_->sml_->mtx_.unlock(); };

  inline bool isInited()  { return map_->sml_->isInited(); };

  inline void Lock()   { map_->sml_->LockCopyToOutputAllMap(true); }
  inline void Unlock() { map_->sml_->LockCopyToOutputAllMap(false); }

  inline MapManager::Ptr getRawPtr() { return map_; }

};

inline void MapInterface::resetGlobalBox(const Eigen::Vector3d &box_min, const Eigen::Vector3d &box_max) {
  map_min_ = box_min;
  map_max_ = box_max;
}

inline void MapInterface::resetGlobalBox() 
{
  std::string new_param_name = "drone_" + std::to_string(drone_id_) + "_box_min_x";
  if (!para_ptr_->get_para(new_param_name.c_str(), map_min_[0]))
  {
    ROS_ERROR_STREAM("[map_interface] resetGlobalBox ERROR! can not get parameter.");
    map_min_[0] = -100.0;
  }

  new_param_name = "drone_" + std::to_string(drone_id_) + "_box_min_y";
  if (!para_ptr_->get_para(new_param_name.c_str(), map_min_[1]))
  {
    ROS_ERROR_STREAM("[map_interface] resetGlobalBox ERROR! can not get parameter.");
    map_min_[1] = -100.0;
  }

  new_param_name = "drone_" + std::to_string(drone_id_) + "_box_min_z";
  if (!para_ptr_->get_para(new_param_name.c_str(), map_min_[2]))
  {
    ROS_ERROR_STREAM("[map_interface] resetGlobalBox ERROR! can not get parameter.");
    map_min_[2] = 0.0;
  }
  ROS_WARN_STREAM("map_min_: " << map_min_.transpose());

  new_param_name = "drone_" + std::to_string(drone_id_) + "_box_max_x";
  if (!para_ptr_->get_para(new_param_name.c_str(), map_max_[0]))
  {
    ROS_ERROR_STREAM("[map_interface] resetGlobalBox ERROR! can not get parameter.");
    map_max_[0] = 100.0;
  }

  new_param_name = "drone_" + std::to_string(drone_id_) + "_box_max_y";
  if (!para_ptr_->get_para(new_param_name.c_str(), map_max_[1]))
  {
    ROS_ERROR_STREAM("[map_interface] resetGlobalBox ERROR! can not get parameter.");
    map_max_[1] = 100.0;
  }

  new_param_name = "drone_" + std::to_string(drone_id_) + "_box_max_z";
  if (!para_ptr_->get_para(new_param_name.c_str(), map_max_[2]))
  {
    ROS_ERROR_STREAM("[map_interface] resetGlobalBox ERROR! can not get parameter.");
    map_max_[2] = 2.0;
  }
  INFO_MSG_GREEN("[MapInterface] resetGlobalBox map_min_: " << map_min_.transpose() << " map_max_: " << map_max_.transpose());
}

inline void MapInterface::getGlobalBox(Eigen::Vector3d &box_min, Eigen::Vector3d &box_max) const{
  box_min = map_min_;
  box_max = map_max_;
}

inline double MapInterface::getResolution() const
{
  return map_->sml_->getResolution();
}

inline void MapInterface::getUpdatedBox(Eigen::Vector3d& bmin, Eigen::Vector3d& bmax) const
{
  map_->sml_->getUpdatedBox(bmin, bmax);
}

inline void MapInterface::getUpdatedBoxIdx(Eigen::Vector3i& bmin_inx, Eigen::Vector3i& bmax_inx) const
{
  map_->sml_->getUpdatedBoxIdx(bmin_inx, bmax_inx);
}

inline Eigen::Vector3i MapInterface::pos2GlobalIdx(const Eigen::Vector3d &pos) const
{
  return map_->sml_->pos2GlobalIdx(pos);
}

inline Eigen::Vector3d MapInterface::globalIdx2Pos(const Eigen::Vector3i &id) const
{
  return map_->sml_->globalIdx2Pos(id);
}

inline size_t MapInterface::globalIdx2BufIdx(const Eigen::Vector3i &id) const
{
  return map_->sml_->globalIdx2BufIdx(id);
}

inline void MapInterface::setTarget(const Eigen::Vector3d &target_pos, const bool &target_pos_valid)
{
  map_->sml_->setTarget(target_pos, target_pos_valid);
}

inline int MapInterface::getOccupancy(const Eigen::Vector3i &idx) const
{
  switch (map_->sml_->getRawOccupancyOutput(idx))
  {
  case GridMap::FREE:
    return MapInterface::FREE;
  case GridMap::OCCUPIED:
    return MapInterface::OCCUPIED;
  case GridMap::UNKNOWN:
    return MapInterface::UNKNOWN;
  case -1:
    return MapInterface::OCCUPIED;
  default:
    ROS_ERROR_STREAM("[map_interface] ERROR! getOccupancy fail: " << map_->sml_->getRawOccupancyOutput(idx));
  }
}

inline int MapInterface::getOccupancy(const Eigen::Vector3d &pos) const
{
  switch (map_->sml_->getRawOccupancyOutput(pos))
  {
  case GridMap::FREE:
    return MapInterface::FREE;
  case GridMap::OCCUPIED:
    return MapInterface::OCCUPIED;
  case GridMap::UNKNOWN:
    return MapInterface::UNKNOWN;
  case -1:
    return MapInterface::OCCUPIED;
  default:
    ROS_ERROR_STREAM("[map_interface] ERROR! getOccupancy fail: " << map_->sml_->getRawOccupancyOutput(pos));
  }
}

inline int MapInterface::getInflateOccupancy(const Eigen::Vector3d &pos) const
{
  switch (map_->sml_->getInflateOccupancy(pos))
  {
  case 0:
    return MapInterface::FREE;
  case GRID_MAP_OUTOFREGION_FLAG:
    return MapInterface::OCCUPIED;
  default: // > 0
    return MapInterface::OCCUPIED;
    // ROS_ERROR_STREAM("[map_interface] ERROR! getInflateOccupancy fail: " << map_->sml_->getInflateOccupancy(pos));
  }
}

// return pair(consider_uk, ignore_uk)
inline pair<int, int> MapInterface::getRawInflateOccupancy(const Eigen::Vector3d& pos) const{

  int consider_uk;
  const int res = map_->sml_->getRawInflateOccupancy(pos);

  int ignore_uk = (true && res > 0) ? MapInterface::OCCUPIED : MapInterface::FREE;

  if (res == 0) consider_uk = MapInterface::FREE;
  else if (res == GRID_MAP_OUTOFREGION_FLAG || res == GRID_MAP_UNKNOWN_FLAG) consider_uk = MapInterface::OCCUPIED;
  else consider_uk = MapInterface::OCCUPIED;

  return make_pair(consider_uk, ignore_uk);
}

inline bool MapInterface::isInGlobalMap(const Eigen::Vector3d &pos) const
{
  return (pos.x() > map_min_.x() && pos.y() > map_min_.y() && pos.z() > map_min_.z() &&
          pos.x() < map_max_.x() && pos.y() < map_max_.y() && pos.z() < map_max_.z());
}

inline bool MapInterface::isInGlobalMap(const Eigen::Vector3i &idx) const
{
  Eigen::Vector3d pos = globalIdx2Pos(idx);
  return (pos.x() > map_min_.x() && pos.y() > map_min_.y() && pos.z() > map_min_.z() &&
          pos.x() < map_max_.x() && pos.y() < map_max_.y() && pos.z() < map_max_.z());
}

inline bool MapInterface::isInLocalMap(const Eigen::Vector3i &idx) const
{
  return map_->sml_->isInBuf(idx);
}

inline bool MapInterface::isInLocalMap(const Eigen::Vector3d &pos) const
{
  return map_->sml_->isInBuf(pos);
}

inline bool MapInterface::isVisible(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, double check_res) const
{
  Eigen::Vector3d p1_2_p2 = p2 - p1;
  Eigen::Vector3d dir = p1_2_p2.normalized();
  double dis = p1_2_p2.norm();
  
  double check_r = check_res;
  if (check_r < resolution_) check_r = resolution_;
  if (check_r >= dis) check_r        = dis / 2.0;

  Eigen::Vector3d pt = p1;
  for (;(pt-p1).norm() < dis; pt += check_r*dir)
  {
    if (getInflateOccupancy(pt) == MapInterface::OCCUPIED || 
        getOccupancy(pt)        == MapInterface::UNKNOWN)
    {
      return false;
    }
  }
  return true;
}

inline bool MapInterface::searchPath(const Eigen::Vector3d& start_pos, const Eigen::Vector3d& end_pos,
                                     std::vector<Eigen::Vector3d>& path, double step_size = 0.1){
  path.clear();
  if ((start_pos - end_pos).norm() < 4.0 && isVisible(start_pos, end_pos)){
    path.push_back(start_pos); path.push_back(end_pos);
    return true;
  }

  dyn_a_star::ASTAR_RET res = a_star_->AstarSearch(step_size, start_pos, end_pos);
  if (res == dyn_a_star::ASTAR_RET::SUCCESS){
    path = a_star_->getPath();
    return true;
  }

  path = {start_pos, end_pos};
  return false;
}

inline bool MapInterface::searchPathConsiderUKRegion(const Eigen::Vector3d& start_pos, const Eigen::Vector3d& end_pos,
                                     std::vector<Eigen::Vector3d>& path, double step_size = 0.1){
  path.clear();
  if ((start_pos - end_pos).norm() < 4.0 && isVisible(start_pos, end_pos)){
    path.push_back(start_pos); path.push_back(end_pos);
    return true;
  }

  dyn_a_star::ASTAR_RET res = a_star_->AstarSearchConsideredUKRegion(step_size, start_pos, end_pos);
  if (res == dyn_a_star::ASTAR_RET::SUCCESS){
    path = a_star_->getPath();
    return true;
  }

  path = {start_pos, end_pos};
  return false;
}

}  // namespace ego_planner

#endif
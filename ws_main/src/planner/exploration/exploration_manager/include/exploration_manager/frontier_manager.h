#ifndef _FRONTIER_MANAGER_H_
#define _FRONTIER_MANAGER_H_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <memory>
#include <vector>
#include <list>
#include <utility>
#include <active_perception/graph_node.h>
#include <active_perception/graph_search.h>
#include <active_perception/frontier_finder.h>
#include <exploration_manager/expl_data.h>
#include <traj_utils/planning_visualization.h>
#include <quadrotor_msgs/GoalSet.h>
#include <nav_msgs/Odometry.h>
#include <active_perception/hgrid.h>
#include <scene_graph/scene_graph.h>


using Eigen::Vector3d;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;
using std::list;
using std::pair;

namespace ego_planner {

enum EXPL_RESULT { NO_FRONTIER, FAIL, SUCCEED};

enum PLAN_TO_WHAT {
    FRONTIER_TO_GOAL = 1,
    PATH_TO_GOAL = 2,
    OTHER = 3,
};

class FrontierManager{
 public:
  FrontierManager(ros::NodeHandle& nh, const MapInterface::Ptr& map, SceneGraph::Ptr scene_graph);
  ~FrontierManager() = default;

  typedef std::shared_ptr<FrontierManager> Ptr;

  //! Main Plan Func
  int planExploreRapid(const Vector3d& pos, const Vector3d& vel, 
                       Vector3d& aim_pos, Vector3d& aim_vel, vector<Eigen::Vector3d>& path_res);
  
  int planExploreTSP(const Vector3d& pos, const Vector3d& vel, const double& yaw,
                     Vector3d& aim_pos, Vector3d& aim_vel, double& aim_yaw, vector<Eigen::Vector3d>& path_res);

  int planTrackGoal(const Vector3d& pos, const Vector3d& vel,
                    const Vector3d& far_goal, vector<Eigen::Vector3d>& path_res);

  void findVPGlobalTour(std::vector<Viewpoint::Ptr> &vps, const Eigen::MatrixXd& cost_mat,
                        const Eigen::Vector3d &cur_pos, const double & cur_yaw, const Eigen::Vector3d &cur_vel);


  //  ----------------  LLM & SceneGraph related  ----------------
  int planLLMExploration(const int &area_id, const Eigen::Vector3d &cur_pos,
                         const Eigen::Vector3d cur_vel, const double &cur_yaw,
                         const PolyHedronPtr &cur_poly, Eigen::Vector3d &aim_pos, double &aim_yaw, Eigen::Vector3d &aim_vel, std::vector<Eigen::
                         Vector3d> &path_res);
  void findGlobalTour_SomeFtrs(std::vector<Frontier>& ftr_select, const Eigen::Vector3d& cur_pos,
                               const Eigen::Vector3d cur_vel, const double& cur_yaw,
                               const PolyHedronPtr& cur_poly, vector<int>& indices);
  //  ----------------  LLM & SceneGraphrelated  ----------------
  
  // utils
  void forceDeleteFrontier(Frontier ftr);
  void updateTopoBlacklist(Eigen::Vector3d goal, Eigen::Vector3d cur, double range,
                           std::unordered_map<int, int> &blacklist);
  Eigen::Vector3d getBlacklistTopoPos(int idx);

  //! Hgrid Related
  void updateHgrid();

  // skeleton update
  PolyHedronPtr cur_mount_topo_{nullptr}, last_mount_topo_{nullptr};
  void setCurrentTopoNode(PolyHedronPtr topo_node);

  //! Visualize
  void visualize(const Eigen::Vector3d &pos);
  void visFrontierInx();
  void visHgrid(const Eigen::Vector3d &pos);
  void visBlacklist();

 private:
  //! TSP Global Path
  // Find optimal tour for coarse viewpoints of all frontiers
  void findGlobalTour(const Vector3d& cur_pos, const Vector3d& cur_vel, const double& cur_yaw,
                      vector<int>& indices, Eigen::MatrixXd& cost_mat);

  void TSPFormulate(const Eigen::MatrixXd& cost_mat);
  void TSPGetRawRes(vector<int>& ids);
  void TSPGetRes(const vector<int>& ids, vector<int>& indices);
  void TSPGetPartialRes(const vector<int>& ids, const vector<int>& frontier_indices, vector<int>& indices);
  void solveTSP(const Eigen::MatrixXd& cost_mat, vector<int>& indices);
  void shortenPath(vector<Vector3d>& path);

  //! Local path refine
  void refineLocalTour(
      const Vector3d& cur_pos, const Vector3d& cur_vel, const double& cur_yaw,
      const vector<vector<Vector3d>>& n_points, const vector<vector<double>>& n_yaws,
      vector<Vector3d>& refined_pts, vector<double>& refined_yaws);

  bool planNextFtr(const Vector3d& pos, const Frontier& next_ftr, Vector3d& aim_pos, vector<Eigen::Vector3d>& path_res, bool force_direct_egoplan);



 public:
  //! Sub-Class
  FrontierFinder::Ptr               frontier_finder_;
  SceneGraph::Ptr                   scene_graph_;
  ExplorationData::Ptr              ed_;
  ExplorationParam::Ptr             ep_;
  HGrid::Ptr                        hgrid_;
  PLAN_TO_WHAT                      local_aim_type_;
  visualization::Visualization::Ptr vis_ptr_;

 private:
  MapInterface::Ptr                 map_;
  PlanningVisualization::Ptr        visualization_;
  ros::Timer                        frontier_timer, goal_timer;
  // ros::Publisher plan_goal_pub_;
  // ros::Subscriber odom_sub_, goal_sub_;

};

}  // namespace ego_planner
#endif

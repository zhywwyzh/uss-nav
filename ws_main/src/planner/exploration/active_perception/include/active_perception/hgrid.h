#ifndef _HGRID_H_
#define _HGRID_H_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <memory>
#include <vector>
#include <list>
#include <unordered_map>
#include <utility>
#include <map_interface/map_interface.hpp>
#include <quadrotor_msgs/HgridMsg.h>


using Eigen::Vector3d;
using std::list;
using std::pair;
using std::shared_ptr;
using std::unique_ptr;
using std::unordered_map;
using std::vector;

class RayCaster;

namespace ego_planner {

// class EDTEnvironment;
// class Astar;
class GridInfo;
class UniformGrid;
// Hierarchical grid, consists of ONE!! levels of grids
// rebuild it in order to adapt the FY-project
class HGrid {

public:
  HGrid(const MapInterface::Ptr& edt, ros::NodeHandle& nh);
  ~HGrid();

  void init(const Eigen::Vector3d& map_min, const Eigen::Vector3d& map_max);

  typedef std::shared_ptr<HGrid> Ptr;

  void updateGridData(const int& drone_id, vector<int>& grid_ids, bool reallocated,
      const vector<int>& last_grid_ids, vector<int>& first_ids, vector<int>& second_ids);

  bool updateBaseCoor();
  void inputFrontiers(const vector<vector<Eigen::Vector3d>>& avgs);
  void getConsistentGrid(const vector<int>& last_ids, const vector<int>& cur_ids,
      vector<int>& first_ids, vector<int>& second_ids);

  void getHgridGlobalBox(Eigen::Vector3d& min_pt, Eigen::Vector3d& max_pt);
  int  getHgridVisRange() const;
  void getVisIdxInRange(vector<int>& vis_idx_list, const Eigen::Vector3d& pos);
  void getGridMarker(vector<Eigen::Vector3d>& pts1, vector<Eigen::Vector3d>& pts2, const vector<int> & vis_idx_list);
  void getGridMarker2(vector<Eigen::Vector3d>& pts, vector<std::string>& texts, const vector<int> & vis_idx_list);

  void getGridStateMarker(vector<Eigen::Vector3d>& center_list, vector<Eigen::Vector3d>& scale_list, 
                          vector<bool>& is_cover_list, vector<bool>& isactive_list, const vector<int> & vis_idx_list);


  void checkFirstGrid(const int& id);
  int getUnknownCellsNum(const int& grid_id);
  Eigen::Vector3d getCenter(const int& grid_id);
  void getActiveGrids(vector<int>& grid_ids);
  bool isConsistent(const int& id1, const int& id2);
  bool isCovered(const Eigen::Vector3d& pos);
  void setSwarmTf();
  void encodeHgridData(quadrotor_msgs::HgridMsg &msg);
  void decodeHgridData(const quadrotor_msgs::HgridMsg &msg, ros::NodeHandle& nh);

  /* --- tool functions --- */
  void geometryPoint2EigenVector3d(const geometry_msgs::Point &p_in, Eigen::Vector3d &p_out);
  void eigenVector3d2GeometryPoint(const Eigen::Vector3d &p_in, geometry_msgs::Point &p_out);

  /* --- no use function and variables --- */
  void getFrontiersInGrid(const vector<int>& grid_ids, vector<int>& ftr_ids, vector<int>& ftr_nums);
  bool getNextGrid(const vector<int>& grid_ids, Eigen::Vector3d& grid_pos, double& grid_yaw);
//  void getCostMatrix(const vector<Eigen::Vector3d>& positions,
//                     const vector<Eigen::Vector3d>& velocities, const vector<vector<int>>& first_ids,
//                     const vector<vector<int>>& second_ids, const vector<int>& grid_ids, Eigen::MatrixXd& mat);
//  void getGridTour(const vector<int>& ids, const Eigen::Vector3d& pos,
//                     vector<Eigen::Vector3d>& tour, vector<Eigen::Vector3d>& tour2);
//  double getCostDroneToGrid(
//       const Eigen::Vector3d& pos, const int& grid_id, const vector<int>& first);
//  double getCostGridToGrid(const int& id1, const int& id2, const vector<vector<int>>& firsts,
//       const vector<vector<int>>& seconds, const int& drone_num);
//  unique_ptr<Astar> path_finder_;
private:
  void coarseToFineId(const int& coarse, vector<int>& fines);
  void fineToCoarseId(const int& fine, int& coarse);


  bool isClose(const int& id1, const int& id2);
  bool inSameLevel1(const int& id1, const int& id2);

  GridInfo& getGrid(const int& id);
  unique_ptr<UniformGrid> grid1_;  // Coarse level
  unique_ptr<UniformGrid> grid2_;  // Fine level      // no use

  MapInterface::Ptr edt_;

  /* parameters */
  ros::NodeHandle nh_;
  double          consistent_cost_;  // no use
  double          consistent_cost2_; // no use

  // Swarm tf
  Eigen::Matrix3d rot_sw_;
  Eigen::Vector3d trans_sw_;
  bool            use_swarm_tf_;     // no use
  double          w_first_;          // no use
  int             hgrid_vis_range_;
  bool            print_info_;
};

}  // namespace ego_planner
#endif
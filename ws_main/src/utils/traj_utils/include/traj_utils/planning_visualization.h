#ifndef _PLANNING_VISUALIZATION_H_
#define _PLANNING_VISUALIZATION_H_

#include <eigen3/Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <stdlib.h>

using std::vector;
using std::string;

namespace ego_planner
{
  class PlanningVisualization
  {
  private:
    ros::NodeHandle node;
    ros::Publisher goal_point_pub;
    ros::Publisher global_list_pub;
    ros::Publisher init_list_pub;
    ros::Publisher initofinit_list_pub;
    ros::Publisher optimal_list_pub;
    ros::Publisher failed_list_pub;
    ros::Publisher a_star_list_pub;

    ros::Publisher frontier_pub, hgrid_pub, text_pub, relevent_pub,
                   viewpoint_pub, hgrid_info_pub_, frontier_pub_;
    ros::Publisher topo_blacklist_pub_;
    ros::Publisher frontier_connection_with_skeleton_pub_;

    ros::Publisher vp_global_path_pub;


  public:
    PlanningVisualization(/* args */) {}
    ~PlanningVisualization() {}
    PlanningVisualization(ros::NodeHandle &nh);

    typedef std::shared_ptr<PlanningVisualization> Ptr;

    void drawVPGlobalPath(const vector<Eigen::Vector3d>& list, const double& scale, const Eigen::Vector4d& color, const string& ns, const int& id);

    void drawFtrEdgeWithSkeleton(const vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> &edges, const double &scale, const Eigen::Vector4d &color);

    void drawLinesByMarkerArray(visualization_msgs::MarkerArray &mk_array,
        const vector<Eigen::Vector3d>& list1, const vector<Eigen::Vector3d>& list2,
        const double& scale, const Eigen::Vector4d& color, const string& ns, const int& id,
        const int& pub_id);

    void drawExploreBoxByMarkerArray(visualization_msgs::MarkerArray &mk_array,
        const Eigen::Vector3d& min_pt, const Eigen::Vector3d &max_pt, const double &scale,
        const Eigen::Vector4d& color, const string& ns, const int& id, const int& pub_id);

    void drawLines(const vector<Eigen::Vector3d>& list1,const vector<Eigen::Vector3d>& list2,
                   const double& scale, const Eigen::Vector4d& color,const string& ns, const int& id, const int& pub_id);

    void drawLines(const vector<Eigen::Vector3d>& list, const double& scale,
        const Eigen::Vector4d& color, const string& ns, const int& id, const int& pub_id);

    void drawText(const Eigen::Vector3d& pos, const string& text, const double& scale,
        const Eigen::Vector4d& color, const string& ns, const int& id, const int& pub_id);

    void addHgridTextInfoMarkerToArray(visualization_msgs::MarkerArray & mk_array, visualization_msgs::Marker & mk,
                                       const Eigen::Vector3d & pos, const string & text, const int & id);

    void drawHgridTextByMarkerArray(const visualization_msgs::MarkerArray & mk_array);

    void drawBlacklistText(const Eigen::Vector3d& pos, const string& text, const double& scale,
                           const Eigen::Vector4d& color, const string& ns, const int& id, const int& pub_id);

    void drawBox(const Eigen::Vector3d& center, const Eigen::Vector3d& scale,
        const Eigen::Vector4d& color, const string& ns, const int& id, const int& pub_id);

    void addHgridActiveInfoMarkerToArray(visualization_msgs::MarkerArray & mk_array, visualization_msgs::Marker & mk,
                                          const Eigen::Vector3d & center, const Eigen::Vector4d & color, const int & id,
                                          const int &action);

    void drawHgridActiveInfoByMarkerArray(const visualization_msgs::MarkerArray & mk_array);


    void drawFrontierRangeBox(const Eigen::Vector3d& center, const Eigen::Vector3d& scale,
                               const Eigen::Vector4d& color, const string& ns, const int& id, const int& pub_id);

    void deleteFrontierRangeBox(const Eigen::Vector3d& center, const Eigen::Vector3d& scale,
                                const Eigen::Vector4d& color, const string& ns, const int& id, const int& pub_id);

    void deleteBox(const Eigen::Vector3d& center, const Eigen::Vector3d& scale,
        const Eigen::Vector4d& color, const string& ns, const int& id, const int& pub_id);

    void drawSpheres(const vector<Eigen::Vector3d>& list, const double& scale,
                    const Eigen::Vector4d& color, const string& ns, const int& id, const int& pub_id);

    void drawCubes(const vector<Eigen::Vector3d>& list, const double& scale, const Eigen::Vector4d& color,
                  const string& ns, const int& id, const int& pub_id);

    void addFrontierCubesMarkerToArray(visualization_msgs::MarkerArray &mk_array, visualization_msgs::Marker &mk,
                                       const vector<Eigen::Vector3d> &list, const double& scale,
                                       const Eigen::Vector4d& color, const string& ns, const int &id, const int& action);
    void deleteAllFrontierCells();

    void drawFrontierCubesByMarkerArray(const visualization_msgs::MarkerArray &mk_array);

    void fillBasicInfo(visualization_msgs::Marker& mk, const Eigen::Vector3d& scale,
                      const Eigen::Vector4d& color, const string& ns, const int& id, const int& shape);
    void fillGeometryInfo(visualization_msgs::Marker& mk, const vector<Eigen::Vector3d>& list);
    void fillGeometryInfo(visualization_msgs::Marker& mk, const vector<Eigen::Vector3d>& list1,
                          const vector<Eigen::Vector3d>& list2);

    Eigen::Vector4d getColor(const double& h, double alpha = 1.0);


    void displayMarkerList(ros::Publisher &pub, const vector<Eigen::Vector3d> &list, double scale,
                           Eigen::Vector4d color, int id,  bool show_sphere = true);
    void generatePathDisplayArray(visualization_msgs::MarkerArray &array,
                                  const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id);
    void generateArrowDisplayArray(visualization_msgs::MarkerArray &array,
                                   const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id);
    void displayGoalPoint(Eigen::Vector3d goal_point, Eigen::Vector4d color, const double scale, int id);
    void displayGlobalPathList(vector<Eigen::Vector3d> global_pts, const double scale, int id);
    void displayInitPathList(vector<Eigen::Vector3d> init_pts, const double scale, int id);
    void displayInitOfInitPathList(vector<Eigen::Vector3d> init_pts, const double scale, int id);
    void displayMultiInitPathList(vector<vector<Eigen::Vector3d>> init_trajs, const double scale);
    void displayMultiOptimalPathList(vector<vector<Eigen::Vector3d>> optimal_trajs, const double scale);
    void displayOptimalList(Eigen::MatrixXd optimal_pts, int id);
    void displayFailedList(Eigen::MatrixXd failed_pts, int id);
  };
} // namespace ego_planner
#endif
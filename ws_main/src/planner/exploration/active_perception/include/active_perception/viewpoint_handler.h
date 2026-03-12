//
// Created by gwq on 9/2/25.
//

#ifndef VIEWPOINT_HANDLER_H
#define VIEWPOINT_HANDLER_H
#include <../include/active_perception/graph_node.h>
#include <../include/active_perception/ftr_data_structure.h>
#include <../include/active_perception/ikd_Tree.h>
#include <scene_graph/scene_graph.h>
#include <map_interface/map_interface.hpp>

using ego_planner::Viewpoint;

class ViewpointsHandler {
public:
    typedef std::shared_ptr<ViewpointsHandler> Ptr;
    typedef ftr_finder::KD_TREE<ftr_finder::ikdTree_VPType> VP_KDTree;
    typedef VP_KDTree::PointVector VP_PointVector;
    ViewpointsHandler(SceneGraph::Ptr scene_graph, ego_planner::MapInterface::Ptr map_interface)
        : scene_graph_(scene_graph), map_interface_(map_interface) {};
    ~ViewpointsHandler() {};

    void updateVPWellObserved(Eigen::Vector3d cur_pos, double yaw, double range);
    void updateCostMatrix(const PolyhedronCluster::Ptr& cluster);
    double calCurStateCost(const Eigen::Vector3d cur_pos, const double cur_yaw, const Eigen::Vector3d &cur_vel, PolyHedronPtr cur_poly, Viewpoint::Ptr &vp);
    void calCurrentAreaCostMatrix(const PolyhedronCluster::Ptr& cluster, const Eigen::Vector3d &cur_pos, const double &cur_yaw, const Eigen::Vector3d &
                             cur_vel, Eigen::MatrixXd &cost_mat);
    void getPathWithTopo(const PolyHedronPtr &start_poly, const Eigen::Vector3d &start_pose,
                         const PolyHedronPtr &end_poly, const Eigen::Vector3d &end_pose, vector<Eigen::Vector3d> &path, double &dis);
    void getTopoPathToZeropoint(const Viewpoint::Ptr &vp);

    // KDTree related functions
    void addViewpoint(Viewpoint vp, PolyHedronPtr cur_poly);
    bool getViewpointInRange(Eigen::Vector3d cur_pos, double range, VP_KDTree::PointVector &vps_in_range);
    bool getViewpointNearest(Eigen::Vector3d cur_pos, int num, VP_KDTree::PointVector &vps_nearest);
    bool deleteViewpoint(unsigned int vp_id);

    std::map<unsigned int, Viewpoint::Ptr> vps_map_;
    std::vector<Viewpoint::Ptr>            vps_candidate_;
    Eigen::Vector3d                        cur_pos_;
    double                                 cur_yaw_;
    Eigen::Vector3d                        cur_vel_;

private:
    unsigned int cur_vp_id_{0};
    bool kdtree_inited_{false};
    std::shared_ptr<VP_KDTree> vps_kdtree_;
    SceneGraph::Ptr scene_graph_;
    ego_planner::MapInterface::Ptr map_interface_;
};

#endif //VIEWPOINT_HANDLER_H

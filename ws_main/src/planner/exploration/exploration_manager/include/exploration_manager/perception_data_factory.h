//
// Created by gwq on 8/16/24.
//

#ifndef SRC_PERCEPTION_DATA_FACTORY_H
#define SRC_PERCEPTION_DATA_FACTORY_H
#include <Eigen/Eigen>
#include <std_msgs/UInt16MultiArray.h>
#include <quadrotor_msgs/MultiPoseGraph.h>
#include <quadrotor_msgs/HgridMsg.h>
#include <quadrotor_msgs/FrontierMsg.h>
#include <quadrotor_msgs/PerceptionMsg.h>
#include <active_perception/frontier_finder.h>
#include <exploration_manager/frontier_manager.h>

namespace ego_planner{
class PerceptionDataMsgFactory {
public:
    PoseGraph posegraph;
    quadrotor_msgs::PerceptionMsg perception_msg;
    map<int, vector<int>> keypose_to_frontieridx;
    Eigen::Vector3d hgrid_min_, hgrid_max_, hgrid_resoloution_;
    Eigen::Vector3i grid_num_;

    int posToHgridAdress(const geometry_msgs::Point& pos);
    int posToHgridAdress(const Eigen::Vector3d & pos);
    Eigen::Vector3d adressToPos(const int& adress);
    PerceptionDataMsgFactory(const quadrotor_msgs::PerceptionMsg msg_in,
                             const double & inflate_gridmap_resolution);
    ~PerceptionDataMsgFactory();

private:
    // param
    double inflate_gridmap_resolution_;
    int    VOID_HGRID_DATA_;
    // tools
    static void geoPt2Vec3d(const geometry_msgs::Point &p_in, Eigen::Vector3d &p_out);
    static void vec3d2GeoPt(const Eigen::Vector3d &p_in, geometry_msgs::Point &p_out);
    static geometry_msgs::Point vec3d2GeoPt(const Eigen::Vector3d &p_in);
    static Eigen::Vector3d geoPt2Vec3d(const geometry_msgs::Point &p_in);
};

class PerceptionMergeFactory {
private:
    quadrotor_msgs::PerceptionMsg map_res_;
    unordered_map<int, int> before_ftrID_to_after_ftrID_;
    std::unique_ptr<PerceptionDataMsgFactory> map_fac1_, map_fac2_;

    // param
    double inflate_gridmap_resolution_;
    // new hgrid data
    double grid_size_res_;
    Eigen::Vector3d size_res_, resolution_res_, hgrid_min_res_, hgrid_max_res_;
    Eigen::Vector3i grid_num_res_;
    int grid_data_res_size_;

public:
    PerceptionMergeFactory();
    ~PerceptionMergeFactory();
    bool merge();
    void mergeInit(quadrotor_msgs::PerceptionMsg &map1, quadrotor_msgs::PerceptionMsg &map2,
                   const double inflate_gridmap_resolution);
    static void mergeTwoFrontiersExceptId(quadrotor_msgs::FrontierMsg &ftr_res,
                                   const quadrotor_msgs::FrontierMsg &ftr_msg2);
    static void initMemoryOfHgird(quadrotor_msgs::HgridMsg &hgrid_res, const int& buffer_size);
    void mergeTwoHgrids();
    void getMergeResult(quadrotor_msgs::PerceptionMsg &map_res);

    // tools
    Eigen::Vector3d getNewGridPosFromAdress(const int& adress, const double &inc);
    static void geoPt2Vec3d(const geometry_msgs::Point &p_in, Eigen::Vector3d &p_out);
    static void vec3d2GeoPt(const Eigen::Vector3d &p_in, geometry_msgs::Point &p_out);
    static geometry_msgs::Point vec3d2GeoPt(const Eigen::Vector3d &p_in);
    static Eigen::Vector3d geoPt2Vec3d(const geometry_msgs::Point &p_in);
};

}

#endif //SRC_PERCEPTION_DATA_FACTORY_H

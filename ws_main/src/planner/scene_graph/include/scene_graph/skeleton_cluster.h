//
// Created by gwq on 8/11/25.
//

#ifndef SPECTRAL_CLUSTER_H
#define SPECTRAL_CLUSTER_H

#include "../include/scene_graph/data_structure.h"
#include "../include/scene_graph/hungarian_alg.h"
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

#include "igraph/igraph.h"
#include "../libs/libleidenalg/include/Optimiser.h"
#include "../libs/libleidenalg/include/ModularityVertexPartition.h"
#include "../libs/libleidenalg/include/CPMVertexPartition.h"

struct Vector3dHash_SpecClus {
    std::size_t operator()(const Eigen::Vector3d& vector) const {
        std::size_t h1 = std::hash<double>()(vector.x());
        std::size_t h2 = std::hash<double>()(vector.y());
        std::size_t h3 = std::hash<double>()(vector.z());
        return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
};

class SpectralCluster {
public:
    typedef std::shared_ptr<SpectralCluster> Ptr;
    SpectralCluster(ros::NodeHandle& nh, double sigma_sq): nh_(nh), sigma_sq_(sigma_sq) {
        cluster_vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/skeleton/cluster_vis", 2);
    };
    ~SpectralCluster() = default;
    void calculate(std::vector<PolyHedronPtr>&polys_without_gate, std::vector<PolyhedronCluster>& clusters);

private:
    ros::NodeHandle& nh_;
    ros::Publisher cluster_vis_pub_;

    double sigma_sq_{1.0};    // bandwidth of kernel function
    unsigned int k_{0};       // number of clusters
    void calSimilarityMatrix(Eigen::MatrixXd& W, Eigen::MatrixXd& ED, std::vector<PolyHedronPtr> polys);
    void calDegreeMatrix(Eigen::MatrixXd& W, Eigen::MatrixXd& D);
    void calLaplacianMatrix(Eigen::MatrixXd& W, Eigen::MatrixXd& D, Eigen::MatrixXd& L);
    void calLaplacianEigen(Eigen::MatrixXd& L, Eigen::MatrixXd& U);
    std::vector<int> kmeans(const Eigen::MatrixXd& points, int k, int max_iter);

    void visualizeClusters(const std::vector<PolyhedronCluster>& clusters);
};

class AreaHandler {
public:
    typedef std::shared_ptr<AreaHandler> Ptr;
    AreaHandler(ros::NodeHandle& nh): nh_(nh) {
        cluster_vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/skeleton/cluster_vis", 2);
        edge_weight_vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/skeleton/edge_weight_vis", 2);
    };
    ~AreaHandler() = default;
    void getCurAreas(std::vector<PolyhedronCluster::Ptr>& clusters);
    int getAreaFromPoly(const PolyHedronPtr &poly);
    void incrementalUpdateAreas(vector<PolyHedronPtr>& new_polys);
    std::map<int, PolyhedronCluster::Ptr> area_map_;
    std::map<int, bool> areas_need_predict_, areas_need_delete_;

    // vector<int> areas_need_delete_;

private:
    ros::NodeHandle& nh_;
    ros::Publisher cluster_vis_pub_, edge_weight_vis_pub_;
    std::unordered_map<Eigen::Vector3d, int, Vector3dHash_SpecClus> poly_cluster_map_;
    std::mutex mutex_;

    int max_area_id_{0};                      // need +1 after add one area

    void mutexLock() {mutex_.lock();};
    void mutexUnlock() {mutex_.unlock();};
    void communityDetection(vector<PolyHedronPtr> &polys_all, std::unique_ptr<CPMVertexPartition>& partition_res, double resolution);
    void findCurAreaNbrs(int cur_area_id);
    void visualizeClusters();
    void visualizeEdgeWeights(const std::vector<PolyHedronPtr>& polys, const std::vector<igraph_integer_t>& edges_data, const std::vector<double>& edge_weights);
    void drawBoundingBox(visualization_msgs::Marker& marker, const Eigen::Vector3d& min, const Eigen::Vector3d& max,
                         int id, const Eigen::Vector3d &color, float line_width);
    inline geometry_msgs::Point eigenToGeoPt(const Eigen::Vector3d& pt);
};

#endif //SPECTRAL_CLUSTER_H

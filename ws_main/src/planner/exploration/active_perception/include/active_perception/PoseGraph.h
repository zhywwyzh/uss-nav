#ifndef _POSE_GRAPH_H_
#define _POSE_GRAPH_H_

#include <iostream>
#include <Eigen/Eigen>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <map>
#include <set>
#include <ros/ros.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <quadrotor_msgs/MultiPoseGraph.h>

#define INFO_MSG(str) do {std::cout << str << std::endl; } while(false)
#define INFO_MSG_RED(str) do {std::cout << "\033[31m" << str << "\033[0m" << std::endl; } while(false)
#define INFO_MSG_GREEN(str) do {std::cout << "\033[32m" << str << "\033[0m" << std::endl; } while(false)
#define INFO_MSG_YELLOW(str) do {std::cout << "\033[33m" << str << "\033[0m" << std::endl; } while(false)
#define INFO_MSG_BLUE(str) do {std::cout << "\033[34m" << str << "\033[0m" << std::endl; } while(false)

using namespace std;
using namespace Eigen;



class Edge{
public:
  int v_inx; // The inx of the end vertex of the edge
	double weight;

	Edge(int neighbour_vertex_inx) {
		this->v_inx = neighbour_vertex_inx;
		this->weight = 0;
	}

	Edge(int neighbour_vertex_inx, double weight) {
		this->v_inx = neighbour_vertex_inx;
		this->weight = weight;
	}

	bool operator<(const Edge& obj) const {
		return obj.v_inx > v_inx;
	}
    bool operator==(const Edge& obj) const {
    return obj.v_inx == v_inx;
	}
};

typedef map<int, set<Edge>> PoseEdge;

class PoseGraph{
public:
    
    map<int, set<Edge> > pose_edge;                          // Record the adjacent list (Not contain the sequential connectted edge)
    pcl::PointCloud<pcl::PointXYZI>::Ptr key_pose_list;      // Record the vertex of the graph into a poindcloud
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_keypose;
    typedef std::shared_ptr<PoseGraph> Ptr;

    PoseGraph(){
        key_pose_list.reset(new pcl::PointCloud<pcl::PointXYZI>);
    }
    PoseGraph(const PoseGraph& src){
        key_pose_list = src.key_pose_list->makeShared();
        pose_edge = src.pose_edge;
    }

    void addKeypose(const Vector3d& odom_pos, const float intensity);

    void push_back(pcl::PointXYZI pt);                      // Add sequential vertex
    void erase_back();                                      // Erase the end vertex
    void clear();
    bool contain(int inx) const;                            // Is the vertex in the poseedge
    void addVertex(int inx);                                // Add the vertex iff it's not in the poseedge 
    void deepCopy(Ptr pg_tar, int copy_size);
    void deepCopy(PoseGraph& pg_tar, int copy_size);

    pcl::PointXYZI getCor(int inx);
    Eigen::Vector3d getCorPos(int inx);
    void getEdge(int inx, set<Edge>& res);
    int getSize();
    int getEdgeSize();

    void addPoseEdge(int inx_s, int inx_e, double dis);
    void getPotenConnectSetSelf(int inx2, map<int, double>& res_set, double connect_eulerdis_range, double connect_seqendis_range);                              // Get the potential connectted vertex(close in euler dis and far in sequential dis)
    void getPotenConnectSetB(pcl::PointXYZI cur, map<int, double>& res_set, double connect_eulerdis_range, double connect_seqendis_range);                              // Get the potential connectted vertex(close in euler dis and far in sequential dis)
    void getNearTopoNodesInRange(const Vector3d& cur_pose, const double radius,
                                 map<double, int>& nearby_v_set);
    void renewKdtree();
    void renewKdtree(int inx_thr);
    double calSeqentialDis(int inx1, int inx2);
    void encodePosegraphData(quadrotor_msgs::MultiPoseGraph &msg);

    void dispPoseEdges();
};


class MultiPoseGraph{
public:
    typedef std::shared_ptr<MultiPoseGraph> Ptr; 
    map<int, PoseGraph> posegraphes;                 // Posegraph Ptr need reset  // Posegraph Vector
    map<pair<int, int>, map<int, set<Edge>> > table; // Record the edges between posegraphes
    int main_inx;
    
    MultiPoseGraph(){};
    MultiPoseGraph(const MultiPoseGraph& obj){
        for(auto it = obj.posegraphes.begin(); it != obj.posegraphes.end(); it++){
            posegraphes.insert({it->first, it->second});
        }
        table = obj.table;
    }

    const PoseGraph& getPosegraph(const int pg_id) const;

    void clear();
    bool contain(pair<int, int> p, int inx) const;                            // Is the vertex in the poseedge
    bool containPg(int inx) const;
    void addVertex(pair<int, int> p, int inx);                                // Add the vertex iff it's not in the poseedge 
    void addEdge(int ps, int pe, int inx_s, int inx_e, double dis);
    void getEdge(int ps, int pe, map<int, set<Edge> >& res);
    void getEdge(int ps, int pe, int vs, set<Edge>& res);
    int  getEdgeSize(int ps, int pe);
    int  getPoseGraphSize(int pg_id);

    // Graph Search
    void   getNeighbor(pair<int, int> p, multimap<double, pair<int,int>>& neighbor);
    double calHeris(pair<int, int> cur, pair<int, int> aim);
    double calGraphPath(pair<int, int> cur, pair<int, int> aim, vector<Vector4d>& path);
    double calGraphPath(pair<int, int> cur, pair<int, int> aim, vector<Vector3d>& path);
    void   dispEdgeTable();

    void   encodeMultiPoseGraphData(quadrotor_msgs::MultiPoseGraph &msg, const int pose_graph_id);
    void   decodeMultiPoseGraphData(const quadrotor_msgs::MultiPoseGraph &msg, const int pose_graph_id);

    geometry_msgs::Point pclPoint2geometryPoint(const pcl::PointXYZI& p_in);
    pcl::PointXYZI geometryPoint2pclPoint(const geometry_msgs::Point& p_in);
};


class MultiPoseGraphFactory{
public:
    typedef std::shared_ptr<MultiPoseGraphFactory> Ptr; 
    double connect_eulerdis_range;
    double connect_seqendis_range;
    void mergeotherPoseGraph(MultiPoseGraph::Ptr merged_mpg, int start_inx);
    void mergeotherPoseGraph(map<int, MultiPoseGraph>& allmpg, MultiPoseGraph::Ptr merged_mpg);
};

inline void PoseGraph::addKeypose(const Vector3d& odom_pos, const float intensity){
    pcl::PointXYZI pt;
    pt.x = odom_pos(0);
    pt.y = odom_pos(1);
    pt.z = odom_pos(2);
    pt.intensity = intensity;
    key_pose_list->push_back(pt);
}

inline void PoseGraph::push_back(pcl::PointXYZI pt){
    key_pose_list->push_back(pt);
}

inline int PoseGraph::getSize(){
    return key_pose_list->size();
}

inline void PoseGraph::clear(){
    key_pose_list->clear();
    pose_edge.clear();
}

inline void PoseGraph::erase_back(){
    key_pose_list->erase(key_pose_list->end()-1);
}

inline bool PoseGraph::contain(int inx) const{
    return pose_edge.find(inx) != pose_edge.end();
}

inline void PoseGraph::addVertex(int inx){
	if (!contain(inx)){
		set<Edge> edge_list;
		pose_edge[inx] = edge_list;
	}
}

inline void PoseGraph::renewKdtree(){
    kdtree_keypose.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    cloud = key_pose_list->makeShared();
    if(cloud->size() > 0)
        kdtree_keypose->setInputCloud(cloud);
}

inline pcl::PointXYZI PoseGraph::getCor(int inx){
    ROS_ASSERT(inx >=0 && inx < key_pose_list->size());
    return key_pose_list->points[inx];
}

inline Eigen::Vector3d PoseGraph::getCorPos(int inx){
    ROS_ASSERT(inx >=0 && inx < key_pose_list->size());
    pcl::PointXYZI pt = key_pose_list->points[inx];
    return Eigen::Vector3d(pt.x, pt.y, pt.z);
}

    
inline void PoseGraph::getEdge(int inx, set<Edge>& res){
    res = pose_edge[inx];
}


inline const PoseGraph& MultiPoseGraph::getPosegraph(const int pg_id) const
{
    auto it = posegraphes.find(pg_id);
    assert(it != posegraphes.end());
    return it->second;
}

#endif
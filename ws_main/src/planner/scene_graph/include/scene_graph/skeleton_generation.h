//
// Created by gwq on 25-2-27.
//

#ifndef SKELETON_GENERATION_H
#define SKELETON_GENERATION_H

#include <ros/ros.h>
#include <iostream>
#include <tuple>
#include <unordered_map>
#include <map_interface/map_interface.hpp>
#include <std_msgs/Empty.h>

#include "../include/scene_graph/skeleton_astar.h"
#include "../include/scene_graph/skeleton_cluster.h"
#include "../include/scene_graph/data_structure.h"
#include "../include/scene_graph/ikd_Tree.h"
#include "../libs/quickhull/QuickHull.hpp"


#define INFO_MSG(str)        do {std::cout << str << std::endl; } while(false)
#define INFO_MSG_RED(str)    do {std::cout << "\033[31m" << str << "\033[0m" << std::endl; } while(false)
#define INFO_MSG_GREEN(str)  do {std::cout << "\033[32m" << str << "\033[0m" << std::endl; } while(false)
#define INFO_MSG_YELLOW(str) do {std::cout << "\033[33m" << str << "\033[0m" << std::endl; } while(false)
#define INFO_MSG_BLUE(str)   do {std::cout << "\033[34m" << str << "\033[0m" << std::endl; } while(false)
#define INFO_MSG_CYAN(str)   do {std::cout << "\033[36m" << str << "\033[0m" << std::endl; } while(false)

// you can choose single map type by uncommenting one of the following macros
// #define _MAP_TYPE_POINT_CLOUD
// #define _MAP_TYPE_OCCUPANCY_MAP
#define _MAP_TYPE_MAP_INTERFACE

using PolyHedronKDTree = skeleton_gen::KD_TREE<skeleton_gen::ikdTree_PolyhedronType>;
using PolyHedronKDTreeVector = PolyHedronKDTree::PointVector;

using PolyHedronKDTree_FixedCenter = skeleton_gen::KD_TREE<skeleton_gen::ikdTree_PolyhedronType_FixedCenter>;
using PolyHedronKDTree_FixedCenterVector = PolyHedronKDTree_FixedCenter::PointVector;

using Vector3dKDTree = skeleton_gen::KD_TREE<skeleton_gen::ikdTree_Vectoe3dType>;
using Vector3dKDTreeVector = Vector3dKDTree::PointVector;

using skeleton_gen::KD_TREE;

class SkeletonGenerator {
  public:
    typedef std::shared_ptr<SkeletonGenerator> Ptr;
    typedef std::unique_ptr<SkeletonGenerator> UPtr;
    SpectralCluster::Ptr spectral_cluster_;
    AreaHandler::Ptr area_handler_;

    SkeletonGenerator(ros::NodeHandle& nh, ego_planner::MapInterface::Ptr &map_interface);
    ~SkeletonGenerator();
    bool ready() const;
    int  getNodeNum() const;

    bool expandSkeleton(const Eigen::Vector3d &start_point, double yaw);                        // 扩展骨架 （core function）
    PolyHedronPtr mountCurTopoPoint(const Eigen::Vector3d& cur_pos, bool ignore_connectivity);  // 寻找挂载点
    PolyHedronPtr getObjFatherNode(const Eigen::Vector3d& cur_pos);
    PolyHedronPtr getFrontierTopo(const Eigen::Vector3d& cur_pos);
    void getAllPolys(std::vector<PolyHedronPtr>& polyhedrons);
    void updateMountedTopoPoint(const Eigen::Vector3d& cur_pos);                                // 更新机器人挂载点
    bool doDenseCheckAndExpand(const Eigen::Vector3d &cur_pos, double yaw);                     // 检查是否需要扩展，并做处理
    double astarSearch(const Eigen::Vector3d& start_point, const Eigen::Vector3d& end_point,
                       std::vector<Eigen::Vector3d>& path, bool add_input_pts);
    double astarSearch(const PolyHedronPtr start_polyhedron, const PolyHedronPtr end_polyhedron,
                       std::vector<Eigen::Vector3d>& path);
    void resetForMapLoad();
    bool registerLoadedPolyhedron(const PolyHedronPtr& polyhedron);
    void finishMapLoad();

    // mutex for skeleton
    void lock(){mutex_.lock();};
    void unlock(){mutex_.unlock();};

    std::vector<PolyHedronPtr> cur_iter_polys_;
    PolyHedronPtr              cur_iter_first_poly_{nullptr};
    void refreshLoadedMapVisualization();
    void visualizePolyBelongsToArea();

  private:
    enum pointCollisionType{
      FREE = 0,
      OCCUPIED = 1,
      UNKNOWN = 2,
      CONTACT_POLYGON = 3,
    };
    struct Vector3dHash {
      std::size_t operator()(const Eigen::Vector3d& vector) const {
        std::size_t h1 = std::hash<double>()(vector.x());
        std::size_t h2 = std::hash<double>()(vector.y());
        std::size_t h3 = std::hash<double>()(vector.z());
        return h1 ^ (h2 << 1) ^ (h3 << 2);
      }
    };
    struct polyhedronHash {
      std::size_t operator()(const PolyHedronPtr polyhedron) const {
        Eigen::Vector3d vector = polyhedron->origin_center_;
        std::size_t h1 = std::hash<double>()(vector.x());
        std::size_t h2 = std::hash<double>()(vector.y());
        std::size_t h3 = std::hash<double>()(vector.z());
        return h1 ^ (h2 << 1) ^ (h3 << 2);
      }
    };
    static bool compareFrontier(PolyhedronFtrPtr f1, PolyhedronFtrPtr f2) {
      return f1->area_size_ > f2->area_size_;
    };
    std::mutex mutex_;

    //  ------- ROS related -------
    ros::NodeHandle nh_;
    ros::Publisher  skeleton_vis_pub_;
    ros::Subscriber map_inflate_sub_, cmd_sub_;

    //  ------- Utils -------
    ego_planner::MapInterface::Ptr      map_interface_;
    skeleton_astar::SkeletonAstar::Ptr  skeleton_astar_;

    //  ------- Parameters -------
    // 局部更新范围
    double _local_x_max, _local_x_min, _local_y_max, _local_y_min, _local_z_max, _local_z_min;
    Eigen::Vector3d _local_range_min, _local_range_max;   // (body frame update range)
    // Map representation
    // 0: point cloud; 1: occupancy map 2: map interface
    int _map_type;
    // Whether the map is in simulation
    bool _is_simulation;
    // An edge will be considered as a frontier if:
    // the dist to its nearest point exceeds this threshold
    double _frontier_creation_threshold;
    // Jump frontier
    double _frontier_jump_threshold;
    // Facets will be split into diff frontier if the angle between exceeds this threshold
    double _frontier_split_threshold;
    // A flowback will be created if number of contact vertices exceeds this threshold
    int _min_flowback_creation_threshold;
    // A flowback will not be created if the radius of contact vertices is below this threshold
    double _min_flowback_creation_radius_threshold;
    // A node will be discarded if its average vertex-center distance is below
    // this threshold
    double _min_node_radius;
    double _min_node_dense_radius;
    // A point on the ray will be considered as hit the pcl if:
    // the dist to its nearest point is below this margin
    // search_margin > sqrt((resolution/2)^2 + (raycast_step/2)^2)
    double _search_margin;
    // A ray will be discarded if length exceeds this max
    double _max_ray_length;
    // A new node will be set at the midpoint if length exceeds this max
    double _max_expansion_ray_length;
    // A node will be absorbed if difference of distance to floor with its parent exceeds this limit
    double _max_height_diff;
    // Number of sampings on the unit sphere
    int _sampling_density, _sampling_level;
    // Max number of facets grouped in a frontier
    int _max_facets_grouped;
    // Resolution for map, raycast,
    double _resolution;
    // Visualization
    double _truncated_z_high;
    double _truncated_z_low;
    // Expand time limit
    double _expand_time_limit;

    /* ------------------ Development Tune ------------------ */
    bool _debug_mode;
    bool _bad_loop;

    // Visualize only the final result or the expansion process
    bool _visualize_final_result_only;
    // Visualize all or only the newest polyhedron
    bool _visualize_all;
    // Visualize outwards normal for each frontier
    bool _visualize_outwards_normal;
    // Visualize neighborhood facets for each frontier
    bool _visualize_nbhd_facets;
    // Visualize only_black polygon or black_and_white polygon
    bool _visualize_black_polygon;

    /* ------------------ Judgement flags ------------------ */
    bool has_init_polyhedron_kdtree_{false};

    //  ------- basic data -------
    PolyHedronPtr   mount_polyhedron_, last_mount_polyhedron_;                          // current and last polyhedron mounted
    Eigen::Vector3d local_box_min_, local_box_max_;                                     // local skeleton update range (in world frame!!!)
    double          cur_yaw_;                                                           // current yaw of the robot
    Eigen::Vector3d cur_pos_;                                                           // current position of the robot
    std::unordered_map<Eigen::Vector3d, PolyHedronPtr, Vector3dHash> polyhedron_map_;   // hash map of all polyhedrons generated (pointers)

    KD_TREE<skeleton_gen::ikdTree_PolyhedronType>::Ptr               polyhedron_kd_tree_;
    KD_TREE<skeleton_gen::ikdTree_PolyhedronType_FixedCenter>::Ptr   polyhedron_kd_tree_fixed_center_;
    KD_TREE<skeleton_gen::ikdTree_Vectoe3dType>::Ptr                 remain_candidate_facet_expand_kd_tree_;
    Vector3dKDTreeVector                                             remain_candidate_facet_expand_pts_;

    std::vector<Eigen::Vector3d>              sphere_sample_directions_;                // directions for sphere sampling
    std::vector<quickhull::Vector3<double>>   sphere_sample_directions_qh_;             // same as above, for quickhull
    std::vector<std::vector<Eigen::Vector3d>> facet_vertex_directions_;                 // vertex samples for each facet
    deque<PolyhedronFtrPtr>                   expand_pending_frontiers_;                // frontiers waiting for expansion

    // temp data
    // std::unordered_map<PolyHedronPtr, PolyHedronPtr, polyhedronHash>   cur_iter_polyhedrons_;    // 当前轮次迭代的多面体，用作loop back检查

    // ROS functions
    void cmdCallback(const std_msgs::Empty::ConstPtr &msg);

    // fucntions
    void getROSParams();
    void sampleUnitSphere();

    // polyhedron generation processes
    void adjustExpandStartPt(Eigen::Vector3d &start_point);          // 调整扩展起点
    bool initNewPolyhedron(PolyHedronPtr new_polyhedron);            // 初始化一个新的多面体
    void initFacetVerticesDirection();                               // 初始化每个采样面的三个顶点的方向
    void generatePolyVertices(PolyHedronPtr poly);                   // 生成黑白多边形
    void centralizePolyhedronCoord(PolyHedronPtr polyhedron);        // 计算多面体几何中心
    double getRadiusOfPolyhedron(PolyHedronPtr polyhedron);          // 计算多面体半径
    pair<bool, Eigen::Vector3d> checkIfContainedByAnotherPolyhedron(PolyHedronPtr polyhedron);  // 是否包含于其他多面体
    void initFacetsFromPolyhedron(PolyHedronPtr polyhedron);         // 初始化多面体的面
    void findFacetsGroupFromVertices(PolyHedronPtr polyhedron, std::vector<VertexPtr> colli_v_group, std::vector<FacetPtr> &res);
    void findNeighborFacets(std::vector<FacetPtr> facets);
    void splitFrontier(PolyHedronPtr polyhedron, std::vector<FacetPtr> single_cluster, std::vector<PolyhedronFtrPtr> &res);
    bool initSingleFrontier(PolyhedronFtrPtr cur_ftr);               // 根据facet簇生成一个边界
    void verifyFrontier(PolyhedronFtrPtr ftr);                       // 验证当前边界的有效性
    void adjustFrontier(PolyhedronFtrPtr ftr);                       // 调整边界, 使其符合checking要求
    bool processAValidFrontier(PolyhedronFtrPtr cur_ftr);            // 处理有效的边界 (生成gate\链接多面体等...)
    void generateFrontiers(PolyHedronPtr polyhedron);                // 生成多面体的边界

    // topological functions
    void findNewTopoConnection(PolyHedronPtr polyhedron);                             // 给定节点，寻找其附近可能存在连接性的节点
    void findLoopbackConnectionFromCandidate(PolyHedronPtr polyhedron);               // 给定节点，寻找其loop back连接
    bool checkConnectivityBetweenPolyhedrons(PolyHedronPtr p1, PolyHedronPtr p2);    // 检查多面体的连通性

    // objects functions


    template<typename T>
    void readParam(std::string param_name, T &param_val, T default_val);
    bool checkInBoundingBox(const Eigen::Vector3d &point);
    bool checkInLocalUpdateRange(const Eigen::Vector3d &point);
    int  checkIfOnLocalFloorOrCeil(const Eigen::Vector3d &point);
    bool checkIfPolyhedronTooDense(const Eigen::Vector3d &center_pt);                 // 检查待生成多面体的中心是否距离其他多面体太近
    void getPolyhedronsInRange(const Eigen::Vector3d& pt, const double &radius, PolyHedronKDTreeVector & polyhedrons_in_range);
    void getPolyhedronsInRangeWithFixedCenter
      (const Eigen::Vector3d& pt, const double &radius, PolyHedronKDTree_FixedCenterVector & polyhedrons_in_range);
    void getPolyhedronsNNearestWithFixedCenter
      (const Eigen::Vector3d& pt, const int &k, PolyHedronKDTree_FixedCenterVector & polyhedrons_nearest);

    void getCandidateNxtPosInRange(const Eigen::Vector3d& pt, const double &radius, Vector3dKDTreeVector& candidate_nxt_pos_in_range);

    template<typename T>
    void getItemsInRangeAndSortByDistance(const Eigen::Vector3d& pt, const double &radius, std::vector<T, Eigen::aligned_allocator<T>>& polyhedrons_in_range);

    void recordNewPolyhedron(PolyHedronPtr polyhedron);
    tuple<Eigen::Vector3d, int, Eigen::Vector3d> rayCast(Eigen::Vector3d orin_point, Eigen::Vector3d direction, double max_ray_length, double step_size);
    bool searchPathInRawMap(Eigen::Vector3d start_point, Eigen::Vector3d end_point, std::vector<Eigen::Vector3d> &path, double step_size, bool
                            consider_uk, bool only_directly_vis);
    VertexPtr getVertexFromDirection(PolyHedronPtr polyhedron, const Eigen::Vector3d &direction);


    //inline pair<double, Eigen::Vector3d>   collisionCheck(const Eigen::Vector3d &point);
    static inline double getDistance(const Eigen::Vector3d &point1, const Eigen::Vector3d &point2);
    inline bool isSamePose(const Eigen::Vector3d &point1, const Eigen::Vector3d &point2);
    Eigen::Vector3d transPointToBodyFrame(const Eigen::Vector3d &point_in_world);

    // collision check whit facets

    pair<bool, Eigen::Vector3d> findContactWithFacetInDirection(const FacetPtr &facet, const Eigen::Vector3d &point, const Eigen::Vector3d &direction);
    static inline pair<bool, Eigen::Vector3d> rayPlaneIntersection (const Eigen::Vector3d& rayOrigin,
                                                                    const Eigen::Vector3d& rayDirection,
                                                                    double a, double b, double c, double d);
    static inline bool ifPointInTriangle(const Eigen::Vector3d &point, const std::vector<VertexPtr> &vertices);
    static inline bool ifPtInIpsilateralOfPlane(const Eigen::Vector3d &pt1, const Eigen::Vector3d &pt2, const FacetPtr facet);

    void visualizePolyhedroneInRange(Eigen::Vector3d center_pt, double radius);
    void visualizePolygons(std::vector<PolyHedronPtr> polyhedrons);
    void visualizeFrontiers(std::vector<PolyhedronFtrPtr> ftrs);
    void visualizeFacets(const std::vector<FacetPtr> &facets, const int &id);
    void drawFacets(std::vector<FacetPtr> facets, Eigen::Vector4d color, int id,
                    std::string ns, ros::Time stamp, visualization_msgs::Marker &marker);
    void drawPoints(const std::vector<Eigen::Vector3d> & points, double pt_scale, Eigen::Vector4d color,
                    int id, std::string ns, ros::Time stamp, visualization_msgs::Marker &marker);

    void visualizePolyhedronVertices(PolyHedronPtr polyhedron);
    void visualizeAllEdges();
    void visualizeLocalRange();

    // Debug vis
    void visualizeVertices(const std::vector<VertexPtr> &vertices, const int &id);
    void visualizeSphereSampleDirections();
};

#endif //SKELETON_GENERATION_H

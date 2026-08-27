//
// Created by gwq on 7/11/25.
//

#ifndef OBJECT_FACTORY_H
#define OBJECT_FACTORY_H

#define INFO_MSG(str)        do {std::cout << str << std::endl; } while(false)
#define INFO_MSG_RED(str)    do {std::cout << "\033[31m" << str << "\033[0m" << std::endl; } while(false)
#define INFO_MSG_GREEN(str)  do {std::cout << "\033[32m" << str << "\033[0m" << std::endl; } while(false)
#define INFO_MSG_YELLOW(str) do {std::cout << "\033[33m" << str << "\033[0m" << std::endl; } while(false)
#define INFO_MSG_BLUE(str)   do {std::cout << "\033[34m" << str << "\033[0m" << std::endl; } while(false)

#include "../scene_graph/data_structure.h"
#include "../scene_graph/ikd_Tree.h"
#include "../scene_graph/skeleton_generation.h"

#include <algorithm>
#include <deque>
#include <thread>
#include <chrono>
#include <future>
#include <mutex>
#include <queue>
#include <condition_variable>
#include <random>

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/centroid.h>
#include <pcl/common/impl/common.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>

#include <scene_graph/pt_cloud_tools.h>
#include <tf/transform_broadcaster.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <scene_graph/EncodeMask.h>


typedef message_filters::sync_policies::ApproximateTime
    <sensor_msgs::Image, scene_graph::EncodeMask> DepthRawMaskSyncPolicy;
typedef message_filters::sync_policies::ApproximateTime
        <sensor_msgs::CompressedImage, scene_graph::EncodeMask> DepthCompressedMaskSyncPolicy;

using ObjectKDTree = skeleton_gen::KD_TREE<skeleton_gen::ikdTree_ObjectDataType>;
using ObjectKDTreeNodeVector = ObjectKDTree::PointVector;

class ObjectFactory {
public:
    typedef std::shared_ptr<ObjectFactory> Ptr;
    typedef std::unique_ptr<ObjectFactory> UPtr;

    struct SemanticDataInput {
        cv::Mat cur_depth_, cur_rgb_;
        nav_msgs::Odometry cur_depth_odom_;
        Eigen::Matrix4d    cur_tf_;
        Eigen::Vector3d    cur_pos_;
        scene_graph::EncodeMask::ConstPtr cur_semantic_recv_msg_;
    };

    // 远距离检测记录: 深度超出挂载范围(max_ray_length)的目标检测, 仅保留方向信息
    // 用于探索规划的方向偏好引导, 不参与 scene-graph 物体挂载
    struct FarDetection {
        std::string     label;      // 检测标签
        Eigen::Vector3d world_pos;  // 粗略世界位置(远端深度噪声大, 仅参考)
        Eigen::Vector3d direction;  // 世界系XY单位方向(消费端主要使用)
        ros::Time       last_seen;  // 最后检出时间(TTL判定基准)
        double          conf;       // 检测置信度
    };

    ObjectFactory(ros::NodeHandle& nh);
    ObjectFactory(ros::NodeHandle& nh, SkeletonGeneratorPtr skel_gen_ptr);
    ObjectFactory(ros::NodeHandle& nh, const std::string& param_prefix,
                  const std::string& topic_prefix);
    ~ObjectFactory();

    void runThisModule();
    void stopThisModule();
    void startFreshSession();
    void cancelSession();
    std::vector<ObjectNode::Ptr> stopAndSnapshot();
    void lock(){mutex_.lock();};
    void unlock(){mutex_.unlock();};
    bool ok();

    // scene graph interface
    void getObjectEdgesWithArea(const std::unordered_map<Eigen::Vector3d, int, Vector3dHash_SpecClus>& poly_clusterId_map,
                                std::vector<std::vector<Eigen::Vector3d>>& edges);
    std::map<int, ObjectNode::Ptr>* getAllObjs(){return &object_map_;};
    // 查询活跃远检测(TTL内且label匹配); 内部惰性剔除过期记录, 线程安全
    std::vector<FarDetection> getActiveFarDetections(const std::string& label);
    bool objInGoodDetection(const ObjectNode::Ptr& obj_node) const {return obj_node->detection_count >= _detection_counter_thresh;};
    void resetForMapLoad();
    bool registerLoadedObject(const ObjectNode::Ptr& obj_node, bool need_more_detection);
    void finishMapLoad();
    void visualizeResult(bool force_full_refresh = false);

    // 返航辅助: fly-origin 挂载点查询/注册 + 状态日志发布
    // fly-origin 是一种特殊的虚拟 object, 用于在 topo 图上持久化绑定 "返航起点" 所在的稳定 polyhedron,
    // 避免每次返航都依赖脆弱的实时 mountCurTopoPoint 吸附 (origin 落在未建图区域时容易失败)
    ObjectNode::Ptr findFlyOrigin();
    // 在指定 polyhedron 上挂载/更新 fly-origin; 若已存在则更新 pos 与 polyhedron_father, 否则新建
    // 注意: detection_count 直接置为阈值, 不进入 objectFilterThread 的删除流程
    bool registerFlyOriginAtPoly(const Eigen::Vector3d& pos, const PolyHedronPtr& poly);
    // 发布 fly-origin 状态日志到 /if_hold_origin (std_msgs::String)
    void publishFlyOriginStatus(bool found);

    // area interface
    std::map<int, ObjectNode::Ptr> object_map_, object_map_needMoreDetection_;

private:
    void init();
    std::mutex mutex_;
    std::shared_ptr<SkeletonGenerator> skel_gen_ptr_;

    double _camera_fx, _camera_fy, _camera_cx, _camera_cy;
    double _lidar_cam_tx, _lidar_cam_ty, _lidar_cam_tz;
    double _lidar_cam_pitch, _lidar_cam_roll, _lidar_cam_yaw;
    double _voxel_size, _max_depth, _min_depth, _max_ray_length;
    double _std_dev_thresh;
    int    _mean_k, _max_threads;
    double _fov_vertical, _fov_horizontal;
    int    _cam_resolution_h, _cam_resolution_w;

    int    _obj_cloud_num_thresh, _detection_counter_thresh;

    int    _max_deque_size;

    // ---- 远距离检测(far target)参数与缓冲 ----
    bool   _far_target_enable{true};
    double _far_target_max_depth{15.0};        // 远检测有效深度上限
    double _far_target_ttl{8.0};               // 记录有效期(秒), 过期回归纯TSP
    double _far_target_merge_angle_deg{10.0};  // 多帧合并的方向夹角阈值
    int    _far_target_min_pixels{50};         // mask最小有效像素数(防噪声)
    std::vector<FarDetection> far_detections_; // 远检测缓冲区
    std::mutex far_mutex_;                     // 独立锁, 避免与语义队列锁竞争

    bool   _depth_cloud_disp_all;
    bool   _use_camera_intrinsics;
    bool   _use_realsense;

    bool skeleton_enabled_;
    bool accepting_input_{false};
    int active_processing_count_{0};
    std::string param_prefix_;
    std::string topic_prefix_;

    std::vector<Eigen::Vector3d> depth_directions_;

    ros::NodeHandle nh_;
    ros::Subscriber segment_result_sub_;
    ros::Publisher  obj_pt_cloud_all_pub_, odom_depth_pub_;
    ros::Publisher  obj_detection_vis_pub_, obj_all_vis_pub_, obj_update_vis_pub_, obj_update_pt_cloud_pub_;
    ros::Publisher  fly_origin_status_pub_;   // 发布 fly-origin 挂载状态到 /if_hold_origin
    tf::TransformBroadcaster depth_world_frame_tf_broadcaster_;

    // Object Data
    PolyHedronPtr cur_polyhedron_, last_polyhedron_;
    deque<SemanticDataInput> semantic_msg_queue_;
    SemanticDataInput cur_data_;
    int  object_max_id_{-1};
    bool object_kdtree_initialized_{false};
    std::vector<int> cur_update_ids_;
    std::vector<ObjectNode::Ptr> cur_update_objs_, cur_add_objs_, cur_update_all_;
    std::vector<std::pair<Eigen::Vector3d, ObjectNode::Ptr>> update_existing_objects_;

    std::vector<ObjectNode::Ptr> cur_observe_results_;
    skeleton_gen::KD_TREE<skeleton_gen::ikdTree_ObjectDataType>::Ptr object_kd_tree_;

    // Object Thread
    std::condition_variable condition_var_;
    std::condition_variable drain_condition_var_;
    bool allow_thread_run_{false};
    double _filter_run_duration;
    int    _obj_main_thread_run_hz;
    bool obj_filter_thread_running_{false}, obj_process_thread_running_{false};
    std::unique_ptr<std::thread> object_filter_thread_, object_process_thread_;
    void objectFilterThread();
    void objectProcessThread();

    void segmentationResultCallback(const scene_graph::EncodeMask::ConstPtr& msg);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr extractCloud(const cv::Mat& depth_img, const cv::Mat &rgb_img, const cv::Mat& mask, const Eigen::Vector3d &color);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteringCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_in);

    void pushDataInDeque(const SemanticDataInput& data);
    ObjectNode::Ptr processSingleObject(const ProcessedCLoudInput &input);
    void doSemanticProcessingOnce();

    // 点数不足未挂载时, 从mask提取远距离检测(深度需在挂载范围之外且不超过上限)
    bool extractFarDetection(const cv::Mat& depth_img, const cv::Mat& mask,
                             const Eigen::Matrix4d& tf, const Eigen::Vector3d& cam_pos,
                             const std::string& label, double conf);
    // 滑窗合并更新: 同label且方向夹角小于阈值的记录刷新, 否则新建
    void updateFarDetection(const FarDetection& det);
    // 物体挂载后清理同方向的同label记录(避免与抢占机制双重引导)
    void removeFarDetectionsNear(const std::string& label, const Eigen::Vector3d& pos);

    void calculateDepthDirectionsFromVerticalFov(double vertical_fov);
    void getOrientedBoundingBox(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &obb_corners);
    void getAxisAlignedBoundingBox(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &aab_corners);
    double calculateBoxIntersection(const pcl::PointCloud<pcl::PointXYZ>::Ptr& box1, const pcl::PointCloud<pcl::PointXYZ>::Ptr& box2);
    double calculateSpatialSimilarity(const ObjectNode::Ptr& obj1, const ObjectNode::Ptr& obj2);
    double calculateSemanticSimilarity(const ObjectNode::Ptr &obj1, const ObjectNode::Ptr &obj2);
    void mergeObjAIntoB(ObjectNode::Ptr& obj_src, ObjectNode::Ptr& obj_target);
    void mergeObjectIntoMap(ObjectNode::Ptr &cur_obj);

    // kdtree operate utils
    bool getObjectsInRange(const Eigen::Vector3d &center, double radius, ObjectKDTreeNodeVector &objects_in_range);
    bool getObjectsNearestN(const Eigen::Vector3d &center, int n, ObjectKDTreeNodeVector &objects_nearest_n);
    void addNewObject(ObjectNode::Ptr& obj_node);
    bool deleteObjectInTree(const ObjectNode::Ptr &obj_node);
    bool deleteObjectInTree(const std::vector<ObjectNode::Ptr> &obj_nodes);
    void updateObjectInTree(const ObjectNode::Ptr& obj_node);
    void updateExistingObjectInKdtree(const std::vector<std::pair<Eigen::Vector3d, ObjectNode::Ptr>> & update_existing_objects);

    // visualization utils
    visualization_msgs::Marker visualizeRefresh(const std::string ns, const int type, const ros::Time &timestamp);
    void visualizeObjBoundingBox(visualization_msgs::Marker & marker, const ObjectNode::Ptr& obj_node, int id, const ros::Time &timestamp, bool
                                 use_axis_box);
    void visualizeObjPosition(visualization_msgs::Marker & marker, const ObjectNode::Ptr& obj_node, int id, const ros::Time &timestamp);
    void visualizeObjLabel(visualization_msgs::Marker & marker, const ObjectNode::Ptr& obj_node, int id, const ros::Time &timestamp);
    void visualizeObjEdgeAll(visualization_msgs::Marker & marker);
    void visualizeUpdateObjects();
    void deVisualizeObjects(const std::vector<ObjectNode::Ptr> &objs_to_delete);

    Eigen::Vector3d getRandomColor();
    inline geometry_msgs::Point eigenToGeoPt(const Eigen::Vector3d& pt);
    inline geometry_msgs::Point pclToGeoPt(const pcl::PointXYZ& pt);
    template<typename T>
    void readParam(ros::NodeHandle &node, std::string param_name, T &param_val, T default_val);
    std::string prefixedParam(const std::string& name) const;
    std::string prefixedTopic(const std::string& name) const;
    cv::Mat decodeRealsenseCompressedDepth(const sensor_msgs::CompressedImage& msg);
};
#endif //OBJECT_FACTORY_H

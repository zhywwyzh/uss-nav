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

    ObjectFactory(ros::NodeHandle& nh);
    ObjectFactory(ros::NodeHandle& nh, SkeletonGeneratorPtr skel_gen_ptr);
    ~ObjectFactory();

    void runThisModule();
    void stopThisModule();
    void lock(){mutex_.lock();};
    void unlock(){mutex_.unlock();};
    bool ok();

    // scene graph interface
    void getObjectEdgesWithArea(const std::unordered_map<Eigen::Vector3d, int, Vector3dHash_SpecClus>& poly_clusterId_map,
                                std::vector<std::vector<Eigen::Vector3d>>& edges);
    std::map<int, ObjectNode::Ptr>* getAllObjs(){return &object_map_;};
    bool objInGoodDetection(const ObjectNode::Ptr& obj_node) const {return obj_node->detection_count >= _detection_counter_thresh;};

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

    bool   _depth_cloud_disp_all;
    bool   _use_camera_intrinsics;
    bool   _use_realsense;

    bool skeleton_enabled_;

    std::vector<Eigen::Vector3d> depth_directions_;

    ros::NodeHandle nh_;
    ros::Subscriber segment_result_sub_;
    ros::Publisher  obj_pt_cloud_all_pub_, odom_depth_pub_;
    ros::Publisher  obj_detection_vis_pub_, obj_all_vis_pub_, obj_update_vis_pub_, obj_update_pt_cloud_pub_;
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
    void visualizeResult();
    void deVisualizeObjects(const std::vector<ObjectNode::Ptr> &objs_to_delete);

    Eigen::Vector3d getRandomColor();
    inline geometry_msgs::Point eigenToGeoPt(const Eigen::Vector3d& pt);
    inline geometry_msgs::Point pclToGeoPt(const pcl::PointXYZ& pt);
    template<typename T>
    void readParam(ros::NodeHandle &node, std::string param_name, T &param_val, T default_val);
    cv::Mat decodeRealsenseCompressedDepth(const sensor_msgs::CompressedImage& msg);
};
#endif //OBJECT_FACTORY_H

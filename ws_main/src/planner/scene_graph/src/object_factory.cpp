#include "../include/scene_graph/object_factory.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

ObjectFactory::ObjectFactory(ros::NodeHandle& nh): nh_(nh){
    skeleton_enabled_ = false;
    init();
}

ObjectFactory::ObjectFactory(ros::NodeHandle &nh, SkeletonGeneratorPtr skel_gen_ptr): nh_(nh) {
    skeleton_enabled_ = true;
    skel_gen_ptr_     = skel_gen_ptr;
    init();
}

void ObjectFactory::init() {
    std::string seg_result_topic;
    readParam<double>(nh_, "obj/camera_fx", _camera_fx, 415.69219771027923);
    readParam<double>(nh_, "obj/camera_fy", _camera_fy, 415.69219771027923);
    readParam<double>(nh_, "obj/camera_cx", _camera_cx, 320.0);
    readParam<double>(nh_, "obj/camera_cy", _camera_cy, 240.0);
    readParam<double>(nh_, "obj/voxel_size", _voxel_size,0.01);
    readParam<double>(nh_, "obj/std_dev_thresh", _std_dev_thresh, 1.0);
    readParam<int>(nh_, "obj/mean_k", _mean_k, 50);
    readParam<bool>(nh_, "obj/use_camera_intrinsics", _use_camera_intrinsics, false);
    readParam<double>(nh_, "obj/max_depth", _max_depth, 5.0);
    readParam<double>(nh_, "obj/min_depth", _min_depth, 0.2);
    readParam<double>(nh_, "obj/fov_vertical", _fov_vertical, 57.0);
    readParam<double>(nh_, "obj/fov_horizontal", _fov_horizontal, 71.0);
    readParam<int>(nh_, "obj/cam_resolution_h", _cam_resolution_h, 480.0);
    readParam<int>(nh_, "obj/cam_resolution_w", _cam_resolution_w, 640.0);
    readParam<bool>(nh_, "obj/depth_cloud_disp_all", _depth_cloud_disp_all, false);
    readParam<std::string>(nh_, "obj/seg_result_topic", seg_result_topic, "/yoloe/encodemask");
    readParam<int>(nh_, "obj/max_threads", _max_threads, 8);                  // 添加最大线程数参数
    readParam<int>(nh_, "obj/obj_cloud_num_thresh", _obj_cloud_num_thresh, 20);
    readParam<int>(nh_, "obj/detection_counter_thresh", _detection_counter_thresh, 10);
    readParam<double>(nh_, "obj/filter_run_duration", _filter_run_duration, 3.0);
    readParam<int>(nh_, "obj/obj_main_thread_run_hz", _obj_main_thread_run_hz, 5);
    readParam<int>(nh_, "obj/max_deque_size", _max_deque_size, 3);
    readParam<double>(nh_, "obj/max_ray_length", _max_ray_length, 4.0);
    readParam<bool>(nh_, "obj/use_realsense", _use_realsense, false);

    // lidar-cam extrinsics
    readParam<double>(nh_, "obj/lidar_cam_tx", _lidar_cam_tx, 0.0);
    readParam<double>(nh_, "obj/lidar_cam_ty", _lidar_cam_ty, 0.0);
    readParam<double>(nh_, "obj/lidar_cam_tz", _lidar_cam_tz, 0.0);
    readParam<double>(nh_, "obj/lidar_cam_pitch", _lidar_cam_pitch, 0.0);
    readParam<double>(nh_, "obj/lidar_cam_roll", _lidar_cam_roll, 0.0);
    readParam<double>(nh_, "obj/lidar_cam_yaw", _lidar_cam_yaw, 0.0);
    _lidar_cam_pitch = _lidar_cam_pitch * M_PI / 180.0;
    _lidar_cam_roll  = _lidar_cam_roll  * M_PI / 180.0;
    _lidar_cam_yaw   = _lidar_cam_yaw   * M_PI / 180.0;

    calculateDepthDirectionsFromVerticalFov(_fov_vertical);
    object_kd_tree_ = std::make_shared<skeleton_gen::KD_TREE<skeleton_gen::ikdTree_ObjectDataType>>();

    segment_result_sub_      = nh_.subscribe<scene_graph::EncodeMask>(seg_result_topic, 5,
                                        &ObjectFactory::segmentationResultCallback, this, ros::TransportHints().tcpNoDelay());
    odom_depth_pub_          = nh_.advertise<nav_msgs::Odometry>("/depth_odom", 2);
    obj_detection_vis_pub_   = nh_.advertise<visualization_msgs::MarkerArray>("/object_detection_vis", 2);
    obj_all_vis_pub_         = nh_.advertise<visualization_msgs::MarkerArray>("/object_all_vis", 2);
    obj_update_vis_pub_      = nh_.advertise<visualization_msgs::MarkerArray>("/object_update_vis", 2);
    obj_update_pt_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/object_update_pointcloud", 2);
    obj_pt_cloud_all_pub_    = nh_.advertise<sensor_msgs::PointCloud2>("/object_pointcloud", 2);

    obj_process_thread_running_ = true;
    object_process_thread_ = std::make_unique<std::thread>(&ObjectFactory::objectProcessThread, this);
    object_process_thread_->detach();

    obj_filter_thread_running_ = true;
    object_filter_thread_ = std::make_unique<std::thread>(&ObjectFactory::objectFilterThread, this);
    object_filter_thread_->detach();
}


ObjectFactory::~ObjectFactory() {
    obj_filter_thread_running_ = false;
    obj_process_thread_running_ = false;
    INFO_MSG_GREEN("[ObjectFactory] ObjectFactory destructor called...");
    INFO_MSG_GREEN("-------------- Program exit --------------");
}

void ObjectFactory::runThisModule() {
    std::unique_lock<std::mutex> lock(mutex_);
    INFO_MSG_GREEN("\n   ******** Object factory module started! ********\n");
    allow_thread_run_ = true;
    lock.unlock();
    condition_var_.notify_all();
}

void ObjectFactory::stopThisModule() {
    INFO_MSG_RED("\n   ******** Object factory module stopped! ********\n");
    std::unique_lock<std::mutex> lock(mutex_);
    allow_thread_run_ = false;
    lock.unlock();
}

bool ObjectFactory::ok() {
    return allow_thread_run_;
}

void ObjectFactory::resetForMapLoad() {
    object_map_.clear();
    object_map_needMoreDetection_.clear();
    semantic_msg_queue_.clear();
    cur_update_ids_.clear();
    cur_update_objs_.clear();
    cur_add_objs_.clear();
    cur_update_all_.clear();
    update_existing_objects_.clear();
    cur_observe_results_.clear();
    cur_polyhedron_ = nullptr;
    last_polyhedron_ = nullptr;
    object_max_id_ = -1;
    object_kdtree_initialized_ = false;
    object_kd_tree_ = std::make_shared<skeleton_gen::KD_TREE<skeleton_gen::ikdTree_ObjectDataType>>();
    INFO_MSG_BLUE("[ObjFactory] Reset runtime state for map load.");
}

bool ObjectFactory::registerLoadedObject(const ObjectNode::Ptr& obj_node, bool need_more_detection) {
    if (obj_node == nullptr) return false;
    if (object_map_.find(obj_node->id) != object_map_.end()) {
        INFO_MSG_RED("[ObjFactory] Duplicate object id during map load: " << obj_node->id);
        return false;
    }

    object_map_[obj_node->id] = obj_node;
    if (need_more_detection) object_map_needMoreDetection_[obj_node->id] = obj_node;
    object_max_id_ = std::max(object_max_id_, obj_node->id);

    ObjectKDTreeNodeVector add_tmp_nodes;
    add_tmp_nodes.emplace_back(obj_node);
    if (object_kdtree_initialized_) {
        object_kd_tree_->Add_Points(add_tmp_nodes, false);
    } else {
        object_kd_tree_->Build(add_tmp_nodes);
        object_kdtree_initialized_ = true;
    }
    return true;
}

void ObjectFactory::finishMapLoad() {
    cur_update_ids_.clear();
    cur_update_objs_.clear();
    cur_add_objs_.clear();
    cur_update_all_.clear();
    update_existing_objects_.clear();
    cur_observe_results_.clear();
    INFO_MSG_GREEN("[ObjFactory] Finish map load. Objects: " << object_map_.size());
}

void ObjectFactory::objectProcessThread() {
    double run_duration = 1.0 / _obj_main_thread_run_hz;
    while (obj_process_thread_running_ && ros::ok()) {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_var_.wait(lock,[&]() { return allow_thread_run_;});
        lock.unlock();

        auto start_time = std::chrono::high_resolution_clock::now();
        doSemanticProcessingOnce();
        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end_time - start_time;
        if (elapsed < std::chrono::duration<double>(run_duration)) {
            std::this_thread::sleep_for(std::chrono::duration<double>(run_duration) - elapsed);
        }
    }
}

void ObjectFactory::objectFilterThread() {
    const std::chrono::duration<double> period(_filter_run_duration);

    while (obj_filter_thread_running_ && ros::ok()) {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_var_.wait(lock, [&]() { return allow_thread_run_;});

        auto start_time = std::chrono::high_resolution_clock::now();
        ros::Time thread_start_time = ros::Time::now();
        int delete_num = 0, detect_good_num = 0;
        std::vector<ObjectNode::Ptr> objects_to_delete;
        std::vector<ObjectNode::Ptr> objects_erase_from_temp;

        // ------------- object filter ------------- //
        if (object_max_id_ >= 2) {
            INFO_MSG("[ObjFilterThread] Object filter thread run Once ---- > ");
            for (const auto& obj_map_pair : object_map_needMoreDetection_) {
                auto obj = obj_map_pair.second;
                if ((thread_start_time - obj->last_detection_time).toSec() < _filter_run_duration) continue;
                if (obj->detection_count < _detection_counter_thresh) {
                    delete_num++;
                    objects_to_delete.push_back(obj);
                }else {
                    objects_erase_from_temp.push_back(obj);
                    detect_good_num ++;
                }
            }
            if (detect_good_num > 0) {
                INFO_MSG_GREEN("[ObjFilterThread] Enough detections objects info: ");
                for (const auto& obj : objects_erase_from_temp) {
                    INFO_MSG("   Label: " << obj->label << " id: "<< obj->id << " Pos: " << obj->pos.transpose()
                                     << " Conf: " << obj->conf << " Count: " << obj->detection_count);
                    object_map_needMoreDetection_.erase(obj->id);
                }
            }
            if (delete_num > 0) {
                INFO_MSG_YELLOW("[ObjFilterThread] Delete objects info: ");
                for (const auto& obj : objects_to_delete) {
                    INFO_MSG("   Label: " << obj->label << " id: "<< obj->id << " Pos: " << obj->pos.transpose()
                                                 << " Conf: " << obj->conf << " Count: " << obj->detection_count);
                }
                deleteObjectInTree(objects_to_delete);
                deVisualizeObjects(objects_to_delete);
                for (const auto& obj : objects_to_delete) {
                    object_map_.erase(obj->id);
                    object_map_needMoreDetection_.erase(obj->id);
                }
            }
        }

        lock.unlock();
        // 计算本次循环已用时间
        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end_time - start_time;

        // 如果执行时间小于目标周期，则等待剩余时间
        if (elapsed < period) {
            std::this_thread::sleep_for(period - elapsed);
        }
    }
}



// 处理单个mask的函数，将作为线程入口
ObjectNode::Ptr ObjectFactory::processSingleObject(const ProcessedCLoudInput &input) {
    ObjectNode::Ptr result;
    result = std::make_shared<ObjectNode>();
    result->label = input.label;
    result->color = input.pt_color;
    result->label_feature = input.label_feature;
    result->conf  = input.conf;
    result->last_detection_time = ros::Time::now();

    try {
        cv::Mat mask_fixed = input.mask.clone();

        // Mask Noise reduction
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
        cv::morphologyEx(mask_fixed, mask_fixed, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(mask_fixed, mask_fixed, cv::MORPH_CLOSE, kernel);

        // 提取点云
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = extractCloud(input.depth_img, input.rgb_img, mask_fixed, input.pt_color);
        if (cloud->empty()) return result;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud = filteringCloud(cloud);
        if (filtered_cloud->size() <= _obj_cloud_num_thresh) return result;
        pcl::transformPointCloud(*filtered_cloud, *result->cloud, input.tf);

        getOrientedBoundingBox(result->cloud, result->obb_corners);
        getAxisAlignedBoundingBox(result->cloud, result->obb_axis);
        // result->pos = (Eigen::Vector3d(result->obb_corners->points.at(0).x, result->obb_corners->points.at(0).y, result->obb_corners->points.at(0).z) +
        //              Eigen::Vector3d(result->obb_corners->points.at(6).x, result->obb_corners->points.at(6).y, result->obb_corners->points.at(6).z)) / 2.0f;
        pcl::PointXYZ center_pos;
        pcl::computeCentroid(*result->cloud, center_pos);
        result->pos = Eigen::Vector3d(center_pos.x, center_pos.y, center_pos.z);
        mergeObjectIntoMap(result);
    } catch (const std::exception& e) {
        INFO_MSG_RED("*** [ObjFactory] Error in mask process thread : " << e.what());
    }
    return result;
}

void ObjectFactory::doSemanticProcessingOnce() {
    // thread pool
    std::vector<std::shared_future<ObjectNode::Ptr>> futures;
    int active_threads = 0;
    std::mutex mtx;
    std::condition_variable cv;
    std::unique_lock<std::mutex> main_lock(mutex_);
    if (!semantic_msg_queue_.empty()) {
        cur_data_ = semantic_msg_queue_.front();
        semantic_msg_queue_.pop_front();

        cur_update_ids_.clear();
        cur_observe_results_.clear(); 
        cur_update_objs_.clear();
        cur_add_objs_.clear();
        cur_update_all_.clear();
        update_existing_objects_.clear();
        last_polyhedron_ = cur_polyhedron_;
        if (skeleton_enabled_)
            cur_polyhedron_ = skel_gen_ptr_->getObjFatherNode(cur_data_.cur_pos_);
        if (cur_polyhedron_ == nullptr) {
            cur_polyhedron_ = last_polyhedron_;
        }
    }else {
        return ;
    }
    main_lock.unlock();

    ros::Time t1 = ros::Time::now();

    if (_depth_cloud_disp_all) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pt_cloud =
            extractCloud(cur_data_.cur_depth_, cur_data_.cur_rgb_,
                        cv::Mat(cur_data_.cur_rgb_.size(), CV_8UC1), Eigen::Vector3d(255,255,255));
        sensor_msgs::PointCloud2 pt_cloud_msg;
        pcl::PointCloud<pcl::PointXYZRGB> output;
        pcl::transformPointCloud(*pt_cloud, output, cur_data_.cur_tf_);
        pcl::toROSMsg(output, pt_cloud_msg);
        pt_cloud_msg.header.frame_id = "world";
        pt_cloud_msg.header.stamp = cur_data_.cur_depth_odom_.header.stamp;
        obj_update_pt_cloud_pub_.publish(pt_cloud_msg);
        return;
    }

    for (int i = 0 ; i < cur_data_.cur_semantic_recv_msg_->masks.size() ; i++) {
        // 等待直到有可用线程
        std::unique_lock<std::mutex> lock(mtx);
        cv.wait(lock, [&]() { return active_threads < _max_threads; });
        active_threads++;
        lock.unlock();

        // mask处理
        cv_bridge::CvImagePtr mask_ptr = cv_bridge::toCvCopy(cur_data_.cur_semantic_recv_msg_->masks[i],
                                                             sensor_msgs::image_encodings::MONO8);
        cv::Mat mask = mask_ptr->image;
        cv::Mat mask_fixed;
        if (mask.size() == cv::Size(640, 640)) {
            cv::Rect roi(0, 80, 640, 480);
            mask_fixed = mask(roi).clone();
        } else if (mask.size() == cv::Size(640, 480)){
            mask_fixed = mask.clone();
        } else {
            ROS_ERROR_STREAM_THROTTLE(1.0, "[ObjFactory] Mask size is not fit, skip this mask ...");
            lock.lock();
            active_threads--;
            lock.unlock();
            cv.notify_one();
            continue;
        }
        // clone label feature
        Eigen::VectorXd label_feature(512);
        for (int j = 0; j < 512; j++)
            label_feature(j) = cur_data_.cur_semantic_recv_msg_->word_vectors[i].word_vector[j];

        // 启动新线程处理当前mask
        ProcessedCLoudInput input(cur_data_.cur_depth_, cur_data_.cur_rgb_, mask_fixed, cur_data_.cur_tf_,
                                  cur_data_.cur_semantic_recv_msg_->labels[i], label_feature,
                                  cur_data_.cur_semantic_recv_msg_->confs[i],getRandomColor());
        auto cur_future = std::async(
            std::launch::async,
            [this, input_data = std::move(input)]() {
                return this->processSingleObject(input_data);
            }
        );
        futures.emplace_back(std::move(cur_future));
        // 添加管理线程，监听子线程并通知主线程可以启动新线程
        auto& last_shared_future = futures.back();
        std::thread([&, sf = last_shared_future]() mutable {
            sf.wait(); // 等待当前future完成
            std::lock_guard<std::mutex> lock2(mtx);
            active_threads--;
            cv.notify_one(); // 通知可以启动新线程
        }).detach();
    }

    // 等待所有线程完成并收集结果
    for (auto& future : futures) {
        try {
            ObjectNode::Ptr obj = future.get();
            cur_observe_results_.push_back(obj);
            // if (obj->cloud->size() > _obj_cloud_num_thresh) {
            //     mergeObjectIntoMap(obj);
            // }
        } catch (const std::exception& e) {
            ROS_ERROR_STREAM("[ObjFactory] Error getting result from thread: " << e.what());
        }
    }
    updateExistingObjectInKdtree(update_existing_objects_);
    // INFO_MSG_YELLOW("disp obj 1 -> ");
    // for (const auto &obj : cur_update_all_)
    //     INFO_MSG_YELLOW("   " << obj->id << " " << obj->label << " " << obj->pos.transpose() << " " << obj->conf);
    // INFO_MSG_YELLOW("disp obj 2 -> ");
    // for (const auto &obj_id : cur_update_ids_)
    //     INFO_MSG_YELLOW("   " << object_map_[obj_id]->id << " " << object_map_[obj_id]->label << " " << object_map_[obj_id]->pos.transpose() << " " << object_map_[obj_id]->conf);

    ros::Time t2 = ros::Time::now();
    visualizeResult();
    if ((t2 - t1).toSec() * 1e3 > 60.0f)
        INFO_MSG("[ObjFactory] All threads finished, Processing time: " << (t2 - t1).toSec() * 1e3<< "ms");
}

void ObjectFactory::segmentationResultCallback(const scene_graph::EncodeMask::ConstPtr &msg) {
    SemanticDataInput se_input;
    se_input.cur_semantic_recv_msg_ = msg;
    // INFO_MSG("\n\n =================== New Observation ===================");
    se_input.cur_depth_odom_ = msg->current_odom;
    Eigen::Quaterniond quat(
        se_input.cur_depth_odom_.pose.pose.orientation.w,
        se_input.cur_depth_odom_.pose.pose.orientation.x,
        se_input.cur_depth_odom_.pose.pose.orientation.y,
        se_input.cur_depth_odom_.pose.pose.orientation.z
    );
    Eigen::Matrix3d rotation = quat.toRotationMatrix();
    Eigen::Vector3d translation(
        se_input.cur_depth_odom_.pose.pose.position.x,
        se_input.cur_depth_odom_.pose.pose.position.y,
        se_input.cur_depth_odom_.pose.pose.position.z
    );

    // lidar-camera transform
    Eigen::AngleAxisd rollAngle(_lidar_cam_roll, Eigen::Vector3d::UnitX());   // unit: rads
    Eigen::AngleAxisd pitchAngle(_lidar_cam_pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(_lidar_cam_yaw, Eigen::Vector3d::UnitZ());
    Eigen::Matrix3d R_l_c = (yawAngle * pitchAngle * rollAngle).toRotationMatrix();
    Eigen::Vector3d t_l_c(_lidar_cam_tx, _lidar_cam_ty, _lidar_cam_tz);
    Eigen::Matrix4d T_lidar_to_cam = Eigen::Matrix4d::Identity();
    T_lidar_to_cam.block<3, 3>(0, 0) = R_l_c;
    T_lidar_to_cam.block<3, 1>(0, 3) = t_l_c;

    se_input.cur_tf_  = Eigen::Matrix4d::Identity();
    se_input.cur_tf_.block<3, 3>(0, 0) = rotation;
    se_input.cur_tf_.block<3, 1>(0, 3) = translation;
    se_input.cur_tf_ = se_input.cur_tf_ * T_lidar_to_cam;
    se_input.cur_pos_ = translation;

    // tf broadcaster
    tf::Transform tf;
    tf.setOrigin(tf::Vector3(translation.x(), translation.y(), translation.z()));
    tf.setRotation(tf::Quaternion(quat.x(), quat.y(), quat.z(), quat.w()));
    depth_world_frame_tf_broadcaster_.sendTransform(tf::StampedTransform(tf, msg->header.stamp, "world", "depth_frame"));

    if (_use_realsense) {
        se_input.cur_depth_ = decodeRealsenseCompressedDepth(msg->current_depth);
    }else {
        cv::Mat depth_compressed_data = cv::Mat(msg->current_depth.data);
        cv::Mat depth_raw = cv::imdecode(depth_compressed_data, cv::IMREAD_ANYDEPTH);
        if (depth_raw.empty()) {
            ROS_ERROR("Failed to decode depth image");
            return;
        }
        se_input.cur_depth_ = depth_raw.clone();
    }

    cv::Mat rgb_compressed_data = cv::Mat(msg->current_rgb.data);
    cv::Mat rgb_raw = cv::imdecode(rgb_compressed_data, cv::IMREAD_COLOR);
    if (rgb_raw.empty()) {
        ROS_ERROR("Failed to decode rgb image");
        return;
    }
    se_input.cur_rgb_ = rgb_raw.clone();

    pushDataInDeque(se_input);
}

void ObjectFactory::pushDataInDeque(const SemanticDataInput &data) {
    std::unique_lock<std::mutex> lock(mutex_);
    if (semantic_msg_queue_.size() >= _max_deque_size)
        semantic_msg_queue_.pop_front();
    semantic_msg_queue_.emplace_back(data);
    lock.unlock();
}

void ObjectFactory::mergeObjAIntoB(ObjectNode::Ptr &obj_src, ObjectNode::Ptr &obj_target) {

    auto calculateAbsPitch = [](const Eigen::Vector3d& observer, const Eigen::Vector3d& target) {
        Eigen::Vector3d diff = target - observer;
        double horizontal_distance = std::sqrt(diff.x() * diff.x() + diff.y() * diff.y());
        double vertical_distance = diff.z();
        double pitch_angle = std::atan2(vertical_distance, horizontal_distance);
        return std::abs(pitch_angle);
    };

    update_existing_objects_.emplace_back(obj_target->pos, obj_target);
    cur_update_objs_.push_back(obj_target);
    cur_update_all_.push_back(obj_target);
    cur_update_ids_.push_back(obj_target->id);

    // point cloud merge
    obj_src->id = obj_target->id;
    if (calculateSemanticSimilarity(obj_src, obj_target) > 0.75){
        // 對多維向量加權平均
        double weight_src = obj_src->cloud->size() / (obj_src->cloud->size() + obj_target->cloud->size());
        obj_src->label_feature = (obj_src->label_feature * weight_src + obj_target->label_feature * (1 - weight_src)) / 2.0f;

    }else if (obj_src->conf > obj_target->conf + 0.1) {
        if (obj_src->label == obj_target->label)
            obj_target->conf = obj_src->conf;
        else if (obj_src->label != obj_target->label ) {
            INFO_MSG_YELLOW("* [ObjFactory] Label changed from [" << obj_target->label << "] to [" << obj_src->label << "]");
            INFO_MSG_YELLOW("* [ObjFactory] Confidence changed from [" << obj_target->conf << "] to [" << obj_src->conf << "]");
            INFO_MSG_YELLOW("* Obj pos: " << obj_target->pos.transpose());
            obj_target->label = obj_src->label;
            obj_target->label_feature = obj_src->label_feature;
            obj_target->conf = obj_src->conf;
        }
    }

    if (obj_target->detection_count < std::numeric_limits<unsigned int>::max())
        obj_target->detection_count ++;
    obj_target->last_detection_time = obj_src->last_detection_time;
    *obj_target->cloud += *obj_src->cloud;

    pcl::VoxelGrid<pcl::PointXYZRGB> grid;
    grid.setInputCloud(obj_target->cloud);
    grid.setLeafSize(static_cast<float>(_voxel_size), static_cast<float>(_voxel_size), static_cast<float>(_voxel_size));
    grid.filter(*obj_target->cloud);

    getOrientedBoundingBox(obj_target->cloud, obj_target->obb_corners);
    getAxisAlignedBoundingBox(obj_target->cloud, obj_target->obb_axis);
    // obj_target->pos = (Eigen::Vector3d(obj_target->obb_corners->points.at(0).x, obj_target->obb_corners->points.at(0).y, obj_target->obb_corners->points.at(0).z) +
    //                   Eigen::Vector3d(obj_target->obb_corners->points.at(6).x, obj_target->obb_corners->points.at(6).y, obj_target->obb_corners->points.at(6).z)) / 2.0f;
    pcl::PointXYZ center_pos;
    pcl::computeCentroid(*obj_target->cloud, center_pos);
    obj_target->pos = Eigen::Vector3d(center_pos.x, center_pos.y, center_pos.z);

    if (skeleton_enabled_) {
        if(obj_target->edge.polyhedron_father != nullptr && cur_polyhedron_ != nullptr){
            if((obj_target->edge.polyhedron_father->center_ - obj_target->pos).norm() > (cur_polyhedron_->center_ - obj_target->pos).norm()){
                double vis_angle = calculateAbsPitch(cur_polyhedron_->center_, obj_target->pos);
                if(vis_angle < M_PI / 6){ // 30度视角范围内才更新father polyhedron
                    obj_target->edge.polyhedron_father = cur_polyhedron_;
                }
            }
        }
    }
}

void ObjectFactory::mergeObjectIntoMap(ObjectNode::Ptr &cur_obj) {
    //  =========↓↓ merge obj into existing object ↓↓=========
        auto nbr_cmp = [this](const std::pair<double, ObjectNode::Ptr>& a,
                              const std::pair<double, ObjectNode::Ptr>& b) {
            if (a.first > b.first) return true;
            else return false;
        };
        int matched_id = -1;
        double overlap = 0.0;
        ObjectKDTreeNodeVector nodes_in_range;
        std::map<int, ObjectNode::Ptr> node_candidate;
        std::vector<std::pair<double, ObjectNode::Ptr>> nbrs_candidate1, nbrs_candidate2, nbrs_highly_overlap;
        std::vector<std::pair<int, double>> overlaps;
        std::map<int, double> semantic_sims;
        if (getObjectsInRange(cur_obj->pos, 1.0, nodes_in_range)) {
            for (const auto& node : nodes_in_range) node_candidate[node.obj_->id] = node.obj_;
            // step1 计算候选对象与当前对象之间的overlap
            for (const auto& nbr : node_candidate) {
                PointCloudOverlapCalculator overlap_calculator;
                overlap = overlap_calculator.calculateOverlapBInA(nbr.second->cloud, cur_obj->cloud, _voxel_size);
                overlaps.emplace_back(nbr.second->id, overlap);
                if (overlap > 0.1 && nbr.second != nullptr) {
                    nbrs_candidate1.emplace_back(overlap, nbr.second);
                }
                if (overlap > 0.5 && nbr.second != nullptr) {
                    nbrs_highly_overlap.emplace_back(overlap, nbr.second);
                }
            }
            sort(nbrs_candidate1.begin(), nbrs_candidate1.end(), nbr_cmp);
            // step2 计算候选对象与当前对象之间的semantic similarity
            for (const auto& nbr : nbrs_candidate1) {
                double semantic_sim = calculateSemanticSimilarity(cur_obj, nbr.second);
                semantic_sims[nbr.second->id] = semantic_sim;
                if (semantic_sim > 0.75 && nbr.second != nullptr)
                    nbrs_candidate2.emplace_back(semantic_sim, nbr.second);
            }

            sort(nbrs_candidate2.begin(), nbrs_candidate2.end(), nbr_cmp);
            //step3 merge obj into existing object
            if (!nbrs_candidate2.empty())
                matched_id = nbrs_candidate2.front().second->id;
            else {
                if (!nbrs_highly_overlap.empty()) {
                    PointCloudOverlapCalculator overlap_calculator;
                    if (overlap_calculator.calculateOverlapBInA(cur_obj->cloud, nbrs_highly_overlap.front().second->cloud, _voxel_size) > 0.5)
                        matched_id = nbrs_highly_overlap.front().second->id;
                }
            }
        }
        //  =========↑↑ merge obj into existing object ↑↑=========
        if (matched_id == -1) {
            std::unique_lock<std::mutex> lock(mutex_);
            addNewObject(cur_obj);
            // try {
            //     INFO_MSG("   ----- Print Add Info ------");
            //     INFO_MSG("   Label: " << cur_obj->label);
            //     INFO_MSG("   Pos: " << cur_obj->pos.transpose());
            //     INFO_MSG("   Conf: " << cur_obj->conf);
            //     INFO_MSG("   KD Tree nodes num : " << object_kd_tree_->validnum());
            //     INFO_MSG("   All Overlaps: ");
            //     for (const auto & i : overlaps) {
            //         // 查询map中是否有该对象
            //         if (semantic_sims.find(i.first) != semantic_sims.end())
            //             INFO_MSG("      [" << object_map_[i.first]->label << "] Overlap: " << i.second << "Semantic Sim: " << semantic_sims[i.first]);
            //         else
            //             INFO_MSG("      [" << object_map_[i.first]->label << "] Overlap: " << i.second);
            //     }
            //
            //     INFO_MSG("   Candidated1 Nbrs: (" << nbrs_candidate1.size() << ")");
            //     for (const auto& nbr : nbrs_candidate1)
            //         INFO_MSG("      Label: " << nbr.second->label << " Overlap: " << nbr.first);
            //     INFO_MSG("   Candidated2 Nbrs: (" << nbrs_candidate2.size() << ")");
            //     for (const auto& nbr : nbrs_candidate2)
            //         INFO_MSG("      Label: " << nbr.second->label << " Semantic Sim: " << nbr.first);
            //     INFO_MSG("   Highly Overlap Nbrs: (" << nbrs_highly_overlap.size() << ")");
            //     for (const auto& nbr : nbrs_highly_overlap)
            //         INFO_MSG("      Label: " << nbr.second->label << " Overlap: " << nbr.first);
            //
            //     if (cur_obj->edge.polyhedron_father != nullptr)
            //         INFO_MSG("   Attach To skeleton(Pos): " << cur_obj->edge.polyhedron_father->center_.transpose());
            //     else
            //         INFO_MSG_RED("   * Attach To skeleton(Pos): None");
            // }catch (std::exception& e) {
            //     ROS_ERROR_STREAM("[ObjFactory]: ** MergeObj Failed to print info, id: " << cur_obj->id << " " << e.what());
            // }
            lock.unlock();
        }else {
            std::unique_lock<std::mutex> lock(mutex_);
            auto update_obj = object_map_[matched_id];
            mergeObjAIntoB(cur_obj, update_obj);
            lock.unlock();

            // if (deleteObjectInTree(update_obj)) {
            //     mergeObjAIntoB(cur_obj, update_obj);
            //     cur_update_objs_.push_back(update_obj);
            //     cur_update_all_.push_back(update_obj);
            //     cur_update_ids_.push_back(matched_id);
            //     updateObjectInTree(update_obj);
            // }else {
            //     INFO_MSG_RED("[ObjFactory]: ** MergeObj Failed to delete object in tree, id: " << update_obj->id);
            // }
        }
}

/**
 * @brief 从深度图像和掩码图像中提取点云
 *
 * 该函数根据输入的深度图像和掩码图像，提取出符合条件的3D点云。
 * 深度图像中的每个像素值表示该点到相机的距离（深度），而掩码图像则指示哪些像素应该被包含在点云中。
 * 只有当掩码图像中对应像素的值大于0且深度值在_min_depth和_max_depth之间时，才会将该像素转换为3D点并添加到点云中。
 *
 * @param depth_img 深度图像，类型为cv::Mat。
 * @param rgb_img
 * @param mask 掩码图像，类型为cv::Mat，用于指示哪些像素应该被包含在点云中。
 * @param color
 * @return pcl::PointCloud<pcl::PointXYZRGB>::Ptr 指向提取的点云的智能指针。
 *         如果深度图像和掩码图像的大小不匹配，则返回nullptr。
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr ObjectFactory::extractCloud(const cv::Mat &depth_img, const cv::Mat &rgb_img,
                                                                   const cv::Mat &mask, const Eigen::Vector3d &color) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (depth_img.size() != mask.size() || depth_img.size() != rgb_img.size()) {
        INFO_MSG_RED("[ObjFactory] Image size not match, skip this mask ...");
        return nullptr;
    }

    // 1. 预分配内存，避免动态扩容带来的性能损耗
    cloud->points.reserve(depth_img.rows * depth_img.cols);

    int rows = depth_img.rows;
    int cols = depth_img.cols;
    bool use_intrinsics = _use_camera_intrinsics; // 本地变量缓存，略微加速访问
    float max_ray = _max_ray_length;
    float min_dep = _min_depth;

    // 针对 Realsense (uint16_t) 和 仿真 (uchar) 分开处理，
    // 这样可以在编译期确定指针类型，极大提升速度
    if (_use_realsense) {
        // === Realsense 模式 (深度图为 uint16_t) ===
        for (int v = 0; v < rows; ++v) {
            // 获取行指针 (Row Pointers)
            const uchar* ptr_mask = mask.ptr<uchar>(v);
            const uint16_t* ptr_depth = depth_img.ptr<uint16_t>(v);
            const cv::Vec3b* ptr_rgb = rgb_img.ptr<cv::Vec3b>(v);

            for (int u = 0; u < cols; ++u) {
                // 使用指针直接访问
                if (ptr_mask[u] == 255 || _depth_cloud_disp_all) {
                    float distance = static_cast<float>(ptr_depth[u]) / 1000.0f;

                    if (distance > min_dep && distance <= max_ray) {
                        pcl::PointXYZRGB point;
                        if (use_intrinsics) {
                            point.x = distance;
                            // 减少类型转换次数，合并计算
                            point.y = static_cast<float>(-(static_cast<double>(u) - _camera_cx) * distance / _camera_fx);
                            point.z = static_cast<float>(-(static_cast<double>(v) - _camera_cy) * distance / _camera_fy);
                        } else {
                            // 注意：depth_directions_ 通常是一维数组，这里需要线性索引
                            // 如果 depth_directions_ 很大，建议也在外部获取指针
                            int pixel_index = v * cols + u;
                            Eigen::Vector3d point_eigen = depth_directions_[pixel_index] * distance;
                            point.x = static_cast<float>(point_eigen.z());
                            point.y = static_cast<float>(-point_eigen.x());
                            point.z = static_cast<float>(-point_eigen.y());
                        }

                        // RGB 赋值
                        const cv::Vec3b& pixel = ptr_rgb[u];
                        point.r = pixel[2];
                        point.g = pixel[1];
                        point.b = pixel[0];
                        cloud->points.push_back(point);
                    }
                }
            }
        }
    } else {
        // === 仿真模式 (深度图为 uchar) ===
        for (int v = 0; v < rows; ++v) {
            const uchar* ptr_mask = mask.ptr<uchar>(v);
            const uchar* ptr_depth = depth_img.ptr<uchar>(v); // 注意这里类型是 uchar
            const cv::Vec3b* ptr_rgb = rgb_img.ptr<cv::Vec3b>(v);

            for (int u = 0; u < cols; ++u) {
                if (ptr_mask[u] == 255 || _depth_cloud_disp_all) {
                    // Simulation 深度计算公式
                    float distance = (255 - ptr_depth[u]) / 255.0f * _max_depth;

                    if (distance > min_dep && distance <= max_ray) {
                        pcl::PointXYZRGB point;
                        if (use_intrinsics) {
                            point.x = distance;
                            point.y = static_cast<float>(-(static_cast<double>(u) - _camera_cx) * distance / _camera_fx);
                            point.z = static_cast<float>(-(static_cast<double>(v) - _camera_cy) * distance / _camera_fy);
                        } else {
                            int pixel_index = v * cols + u;
                            Eigen::Vector3d point_eigen = depth_directions_[pixel_index] * distance;
                            point.x = static_cast<float>(point_eigen.z());
                            point.y = static_cast<float>(-point_eigen.x());
                            point.z = static_cast<float>(-point_eigen.y());
                        }

                        const cv::Vec3b& pixel = ptr_rgb[u];
                        point.r = pixel[2];
                        point.g = pixel[1];
                        point.b = pixel[0];
                        cloud->points.push_back(point);
                    }
                }
            }
        }
    }
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = false;
    return cloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr ObjectFactory::filteringCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_in) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    ros::Time t1 = ros::Time::now();
    // 体素滤波
    pcl::VoxelGrid<pcl::PointXYZRGB> grid;
    grid.setInputCloud(cloud_in);
    grid.setLeafSize(_voxel_size, _voxel_size, _voxel_size);
    grid.filter(*cloud_filtered);
    ros::Time t2 = ros::Time::now();
    // 统计滤波
    for (int i = 0; i < 1; i++) {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
        sor.setInputCloud(cloud_filtered);
        if (cloud_filtered->size() < _mean_k)
            sor.setMeanK(floor(cloud_filtered->size() / 1.5));
        else
            sor.setMeanK(_mean_k);
        sor.setStddevMulThresh(_std_dev_thresh);
        sor.filter(*cloud_filtered);
    }
    // INFO_MSG("cloud StatisticalOutlierRemoval time : " << (t2 - t1).toSec() * 1e3 << "ms");
    // INFO_MSG("cloud VoxelGrid time : " << (ros::Time::now() - t2).toSec() * 1e3 << "ms");
    return cloud_filtered;
}

void ObjectFactory::calculateDepthDirectionsFromVerticalFov(double vertical_fov) {
    depth_directions_.clear();
    double z = static_cast<double>(_cam_resolution_h) * 0.5f / tan(_fov_vertical * 0.5f * M_PI / 180.0f);
    depth_directions_.resize(_cam_resolution_h * _cam_resolution_w);

    for (int y = 0; y < _cam_resolution_h; y++) {
        for (int x = 0; x < _cam_resolution_w; x++) {
            double nx = -static_cast<double>(_cam_resolution_w) / 2.0f + x;
            double ny = -static_cast<double>(_cam_resolution_h) / 2.0f + y;
            double nz = z;
            depth_directions_[y * _cam_resolution_w + x] = Eigen::Vector3d(nx, ny, nz).normalized();
        }
    }
    INFO_MSG_GREEN("[ObjFactory] Depth directions calculated (num : " << depth_directions_.size() << ")");
}

Eigen::Vector3d ObjectFactory::getRandomColor() {
    // 生成随机颜色
    std::mt19937 rng(std::random_device{}());
    std::uniform_int_distribution<int> dist(0, 255);
    return Eigen::Vector3d(dist(rng), dist(rng), dist(rng));
}

void ObjectFactory::getAxisAlignedBoundingBox(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                                              pcl::PointCloud<pcl::PointXYZ>::Ptr &aab_corners) {
    // 获取轴对齐的边界框
    if (cloud->empty()) {
        PCL_ERROR("Couldn't read the pcd file\n");
        return ;
    }
    Eigen::Vector4f min_pt, max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);
    aab_corners->points.clear();
    aab_corners->points.resize(8);
    aab_corners->points[0] = pcl::PointXYZ(min_pt.x(), min_pt.y(), min_pt.z());
    aab_corners->points[1] = pcl::PointXYZ(max_pt.x(), min_pt.y(), min_pt.z());
    aab_corners->points[2] = pcl::PointXYZ(max_pt.x(), max_pt.y(), min_pt.z());
    aab_corners->points[3] = pcl::PointXYZ(min_pt.x(), max_pt.y(), min_pt.z());
    aab_corners->points[4] = pcl::PointXYZ(min_pt.x(), min_pt.y(), max_pt.z());
    aab_corners->points[5] = pcl::PointXYZ(max_pt.x(), min_pt.y(), max_pt.z());
    aab_corners->points[6] = pcl::PointXYZ(max_pt.x(), max_pt.y(), max_pt.z());
    aab_corners->points[7] = pcl::PointXYZ(min_pt.x(), max_pt.y(), max_pt.z());
}

void ObjectFactory::getOrientedBoundingBox(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                                            pcl::PointCloud<pcl::PointXYZ>::Ptr &obb_corners) {
    if (cloud->empty()) {
        PCL_ERROR("Couldn't read the pcd file\n");
        return ;
    }
    // 计算点云的质心&协方差矩阵
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);
    Eigen::Matrix3f covariance_matrix;
    pcl::computeCovarianceMatrixNormalized(*cloud, centroid, covariance_matrix);

    // 计算协方差矩阵的特征向量和特征值
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance_matrix, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigen_vectors = eigen_solver.eigenvectors();

    // 构建旋转矩阵
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block<3, 3>(0, 0) = eigen_vectors.transpose();
    transform.block<3, 1>(0, 3) = -1.f * (transform.block<3, 3>(0, 0) * centroid.head<3>());

    // 旋转点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::transformPointCloud(*cloud, *cloud_transformed, transform);

    // 计算旋转后点云的轴对齐边界框并计算角点
    Eigen::Vector4f min_pt, max_pt;
    pcl::getMinMax3D(*cloud_transformed, min_pt, max_pt);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr obb_corners(new pcl::PointCloud<pcl::PointXYZ>);
    obb_corners->points.clear();
    obb_corners->points.resize(8);
    obb_corners->points[0] = pcl::PointXYZ(min_pt.x(), min_pt.y(), min_pt.z());
    obb_corners->points[1] = pcl::PointXYZ(max_pt.x(), min_pt.y(), min_pt.z());
    obb_corners->points[2] = pcl::PointXYZ(max_pt.x(), max_pt.y(), min_pt.z());
    obb_corners->points[3] = pcl::PointXYZ(min_pt.x(), max_pt.y(), min_pt.z());
    obb_corners->points[4] = pcl::PointXYZ(min_pt.x(), min_pt.y(), max_pt.z());
    obb_corners->points[5] = pcl::PointXYZ(max_pt.x(), min_pt.y(), max_pt.z());
    obb_corners->points[6] = pcl::PointXYZ(max_pt.x(), max_pt.y(), max_pt.z());
    obb_corners->points[7] = pcl::PointXYZ(min_pt.x(), max_pt.y(), max_pt.z());

    // 反向旋转边界框的角点
    Eigen::Matrix4f inverse_transform = transform.inverse();
    pcl::transformPointCloud(*obb_corners, *obb_corners, inverse_transform);
}

double ObjectFactory::calculateBoxIntersection(const pcl::PointCloud<pcl::PointXYZ>::Ptr &box1,
                                   const pcl::PointCloud<pcl::PointXYZ>::Ptr &box2) {
    CloudIntersectionServer box_intersection_server;
    return box_intersection_server.calculateIntersectionVolume(box1, box2);
}

double ObjectFactory::calculateSpatialSimilarity(const ObjectNode::Ptr &obj1, const ObjectNode::Ptr &obj2) {

}

double ObjectFactory::calculateSemanticSimilarity(const ObjectNode::Ptr &obj1, const ObjectNode::Ptr &obj2) {
    // INFO_MSG("shape: " << obj1->label_feature.size() << " " << obj2->label_feature.size());
    Eigen::Matrix<double, 1, 1> sim_score = obj1->label_feature.transpose() * obj2->label_feature;
    return sim_score(0, 0);
}

void ObjectFactory::addNewObject(ObjectNode::Ptr &obj_node) {
    object_max_id_ ++;
    obj_node->id = object_max_id_;
    obj_node->detection_count = 1;
    object_map_[obj_node->id] = obj_node;
    object_map_needMoreDetection_[obj_node->id] = obj_node;
    ObjectKDTreeNodeVector add_tmp_nodes;
    add_tmp_nodes.emplace_back(obj_node);

    cur_add_objs_.push_back(obj_node);
    cur_update_all_.push_back(obj_node);
    cur_update_ids_.push_back(obj_node->id);

    if (skeleton_enabled_) {
        obj_node->edge.polyhedron_father = cur_polyhedron_;
    }
    
    if (object_kdtree_initialized_) {
        object_kd_tree_->Add_Points(add_tmp_nodes, false);
    }else {
        INFO_MSG_YELLOW("[ObjFactory]: Object KDTree not initialized, start build kdtree ...");
        object_kdtree_initialized_ = true;
        object_kd_tree_->Build(add_tmp_nodes);
        INFO_MSG_GREEN("[ObjFactory]: =========== Object KDTree initialized ==========");
    }
    INFO_MSG_GREEN("\n*** [ObjFactory] New Object [id: " << obj_node->id << ", label : " << obj_node->label << "] \n");
}

void ObjectFactory::updateExistingObjectInKdtree(const std::vector<std::pair<Eigen::Vector3d, ObjectNode::Ptr>> & update_existing_objects) {
    ObjectKDTreeNodeVector delete_pts;
    ObjectKDTreeNodeVector add_pts;
    for (const auto & exist_obj : update_existing_objects) {
        ObjectKDTreeNodeVector nodes_nearest_n;
        if (getObjectsNearestN(exist_obj.first, 1, nodes_nearest_n)) {
            if ((Eigen::Vector3d(nodes_nearest_n.front().x, nodes_nearest_n.front().y, nodes_nearest_n.front().z)- exist_obj.first).norm() > 0.1) {
                INFO_MSG_YELLOW("kdtree update may be wrong, search failed");
                continue;
            }
            delete_pts.push_back(nodes_nearest_n.front());
            add_pts.emplace_back(nodes_nearest_n.front().obj_);
        }
    }
    std::unique_lock<std::mutex> lock(mutex_);
    object_kd_tree_->Delete_Points(delete_pts);
    object_kd_tree_->Add_Points(add_pts, false);
    lock.unlock();
}

void ObjectFactory::updateObjectInTree(const ObjectNode::Ptr &obj_node) {
    ObjectKDTreeNodeVector add_tmp_nodes;
    std::lock_guard<std::mutex> lock(mutex_);
    add_tmp_nodes.emplace_back(obj_node);
    object_kd_tree_->Add_Points(add_tmp_nodes, false);
    // INFO_MSG("[ObjFactory]: Use ["<< obj_node->label <<"] Update [id : " << obj_node->id << " | label : " << obj_node->label << "]");
}

bool ObjectFactory::deleteObjectInTree(const ObjectNode::Ptr &obj_node) {
    ObjectKDTreeNodeVector nodes_nearest_n;
    if (getObjectsNearestN(obj_node->pos, 1, nodes_nearest_n)) {
        ObjectKDTreeNodeVector delete_pts;
        if ((Eigen::Vector3d(nodes_nearest_n.front().x, nodes_nearest_n.front().y, nodes_nearest_n.front().z) - obj_node->pos).norm() > 0.1)
            return false;
        delete_pts.emplace_back(nodes_nearest_n.front());
        object_kd_tree_->Delete_Points(delete_pts);
        return true;
    }
    return false;
}

bool ObjectFactory::deleteObjectInTree(const std::vector<ObjectNode::Ptr> &obj_nodes) {
    ObjectKDTreeNodeVector delete_pts;
    for (const auto & obj_node : obj_nodes) {
        ObjectKDTreeNodeVector nodes_nearest_n;
        if (getObjectsNearestN(obj_node->pos, 1, nodes_nearest_n)) {
            if ((Eigen::Vector3d(nodes_nearest_n.front().x, nodes_nearest_n.front().y, nodes_nearest_n.front().z) - obj_node->pos).norm() > 0.1)
                continue;
            delete_pts.emplace_back(nodes_nearest_n.front());
        }
    }
    if (delete_pts.empty())
        return false;
    object_kd_tree_->Delete_Points(delete_pts);
    return true;
}


bool ObjectFactory::getObjectsInRange(const Eigen::Vector3d &center, double radius,
                                      ObjectKDTreeNodeVector &objects_in_range) {
    if (object_kdtree_initialized_) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto sort_cmp = [center](const skeleton_gen::ikdTree_ObjectDataType & p1,
                                 const skeleton_gen::ikdTree_ObjectDataType & p2 ){
            return (p1.obj_->pos - center).norm() < (p2.obj_->pos - center).norm();
        };
        skeleton_gen::ikdTree_ObjectDataType search_data(nullptr);
        search_data.set_coordinate(center);
        object_kd_tree_->Radius_Search(search_data, radius, objects_in_range);
        if (objects_in_range.empty())
            return false;
        sort(objects_in_range.begin(), objects_in_range.end(), sort_cmp);
        return true;
    }else {
        INFO_MSG_RED("[ObjFactory]: Object KDTree not initialized, skip searching ...");
        return false;
    }
}

bool ObjectFactory::getObjectsNearestN(const Eigen::Vector3d &center, int n, ObjectKDTreeNodeVector &objects_nearest_n) {
    if (object_kdtree_initialized_) {
        auto sort_cmp = [center](const skeleton_gen::ikdTree_ObjectDataType & p1,
                                 const skeleton_gen::ikdTree_ObjectDataType & p2 ){
            return (p1.obj_->pos - center).norm() < (p2.obj_->pos - center).norm();
        };
        skeleton_gen::ikdTree_ObjectDataType search_data(nullptr);
        search_data.set_coordinate(center);

        std::vector<float> distances;
        double max_dist;
        object_kd_tree_->Nearest_Search(search_data, n, objects_nearest_n, distances, max_dist);
        if (objects_nearest_n.empty())
            return false;
        sort(objects_nearest_n.begin(), objects_nearest_n.end(), sort_cmp);
        return true;
    }else {
        INFO_MSG_RED("[ObjFactory]: Object KDTree not initialized, skip searching ...");
        return false;
    }
}

void ObjectFactory::getObjectEdgesWithArea(const std::unordered_map<Eigen::Vector3d, int, Vector3dHash_SpecClus> &poly_clusterId_map,
                                           std::vector<std::vector<Eigen::Vector3d> > &edges) {
    std::unique_lock<std::mutex> lock(mutex_);
    for (const auto & obj : object_map_) {
        if (obj.second->detection_count <= _detection_counter_thresh) continue;
        if (obj.second->edge.polyhedron_father == nullptr) continue;
        if (poly_clusterId_map.find(obj.second->edge.polyhedron_father->center_) != poly_clusterId_map.end())
            edges.at(poly_clusterId_map.at(obj.second->edge.polyhedron_father->center_)).push_back(obj.second->pos);
    }
    lock.unlock();
}

visualization_msgs::Marker ObjectFactory::visualizeRefresh(const std::string ns, const int type, const ros::Time &timestamp) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = timestamp;
    marker.ns     = ns;
    marker.id     = 0;
    marker.type   = type;
    marker.action = visualization_msgs::Marker::DELETEALL;
    return marker;
}

cv::Mat ObjectFactory::decodeRealsenseCompressedDepth(const sensor_msgs::CompressedImage &msg) {
    // 1. 校验格式 (通常是 "16UC1; compressedDepth")
    // 虽然不是必须，但用于调试是个好习惯
    // ROS compressedDepth 的标准头部长度是 12 字节
    const size_t header_size = 12;

    if (msg.data.size() <= header_size) {
        ROS_ERROR("Compressed depth data is too short!");
        return cv::Mat();
    }

    // 2. 解析头部参数 (如果你需要处理非 PNG 压缩的深度，比如有损压缩，需要用到这些)
    // 头部结构: [0-3: config/enum], [4-7: depth_max], [8-11: depth_quantization]
    // 这里的解析是为了展示完整的协议，如果只是解压 PNG，这部分其实可以跳过
    float depth_quant_a, depth_quant_b;
    memcpy(&depth_quant_a, &msg.data[4], sizeof(float));
    memcpy(&depth_quant_b, &msg.data[8], sizeof(float));

    // 3. 核心步骤：跳过 12 字节头部，提取纯图像数据
    // 我们构建一个指向 raw data + 12 的数据引用
    const std::vector<uint8_t> imageData(msg.data.begin() + header_size, msg.data.end());

    // 4. 使用 OpenCV 解码
    // 关键 flag: cv::IMREAD_UNCHANGED (或 -1)。
    // 只有这个 flag 才能保证解码出 16位 (CV_16U) 的原始深度，否则会被转成 8位 BGR。
    cv::Mat decoded_img = cv::imdecode(imageData, cv::IMREAD_UNCHANGED);

    if (decoded_img.empty()) {
        ROS_ERROR("Failed to decode compressed depth image");
        return cv::Mat();
    }
    return decoded_img;
}

template<typename T>
void ObjectFactory::readParam(ros::NodeHandle &node, std::string param_name, T &param_val, T default_val) {
    if (!node.param(param_name, param_val, default_val))
        INFO_MSG_YELLOW("[ObjFactory]: param | " << param_name << " not found, using default value: " << default_val);
    else
        INFO_MSG_GREEN("[ObjFactory]: param | " << param_name << " found: " << param_val);
}

geometry_msgs::Point ObjectFactory::eigenToGeoPt(const Eigen::Vector3d &pt) {
    geometry_msgs::Point geo_pt;
    geo_pt.x = pt.x(); geo_pt.y = pt.y(); geo_pt.z = pt.z();
    return geo_pt;
}

geometry_msgs::Point ObjectFactory::pclToGeoPt(const pcl::PointXYZ &pt) {
    geometry_msgs::Point geo_pt;
    geo_pt.x = pt.x; geo_pt.y = pt.y; geo_pt.z = pt.z;
    return geo_pt;
}

void ObjectFactory::visualizeResult(bool force_full_refresh) {
    static ros::Time last_time = ros::Time::now();

    if (force_full_refresh || ros::Time::now() - last_time > ros::Duration(2.0)) {
        last_time = ros::Time::now();
        visualization_msgs::MarkerArray marker_array;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pt_cloud_all = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
        marker_array.markers.push_back(visualizeRefresh("obj_position", visualization_msgs::Marker::SPHERE_LIST, ros::Time::now()));
        marker_array.markers.push_back(visualizeRefresh("obb_corners", visualization_msgs::Marker::LINE_LIST, ros::Time::now()));
        marker_array.markers.push_back(visualizeRefresh("obj_label", visualization_msgs::Marker::TEXT_VIEW_FACING, ros::Time::now()));
        for (const auto & obj : object_map_) {
            visualization_msgs::Marker marker;
            visualizeObjBoundingBox(marker, obj.second, obj.first, ros::Time::now(), true);
            marker_array.markers.push_back(marker);
            visualizeObjPosition(marker, obj.second, obj.first, ros::Time::now());
            marker_array.markers.push_back(marker);
            visualizeObjLabel(marker, obj.second, obj.first, ros::Time::now());
            marker_array.markers.push_back(marker);

            // 利用 obj.second->color 给点云染色
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pt_cloud_obj_color = obj.second->cloud;
            for (auto & pt : pt_cloud_obj_color->points) {
                pt.r = obj.second->color.x(); pt.g = obj.second->color.y(); pt.b = obj.second->color.z();
            }
            *pt_cloud_all += *pt_cloud_obj_color;

            // 使用点云原始颜色
            // *pt_cloud_all += *obj.second->cloud;
        }
        if (skeleton_enabled_) {
            visualization_msgs::Marker marker;
            visualizeObjEdgeAll(marker);
            marker_array.markers.push_back(marker);
        }
        sensor_msgs::PointCloud2 pt_cloud_all_msg;
        pcl::toROSMsg(*pt_cloud_all, pt_cloud_all_msg);
        pt_cloud_all_msg.header.frame_id = "world";
        pt_cloud_all_msg.header.stamp = ros::Time::now();
        obj_pt_cloud_all_pub_.publish(pt_cloud_all_msg);
        obj_all_vis_pub_.publish(marker_array);
    }

    nav_msgs::Odometry odom_msg = cur_data_.cur_depth_odom_;
    odom_depth_pub_.publish(odom_msg);
    visualizeUpdateObjects();
}

void ObjectFactory::visualizeUpdateObjects() {
    visualization_msgs::MarkerArray marker_array;
    sensor_msgs::PointCloud2 msg_out;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    ros::Time timestamp = ros::Time::now();

    for (const auto & obj : cur_update_all_) {
        *merged_cloud += *obj->cloud;
        const auto & obj_node = obj;
        visualization_msgs::Marker marker;

        visualizeObjBoundingBox(marker, obj_node, obj->id, timestamp, true);
        marker_array.markers.push_back(marker);
        visualizeObjPosition(marker, obj_node, obj->id, timestamp);
        marker_array.markers.push_back(marker);
        visualizeObjLabel(marker, obj_node, obj->id, timestamp);
        marker_array.markers.push_back(marker);
    }

    obj_update_vis_pub_.publish(marker_array);

    pcl::toROSMsg(*merged_cloud, msg_out);
    msg_out.header.frame_id = "world";
    msg_out.header.stamp = ros::Time::now();
    // obj_update_pt_cloud_pub_.publish(msg_out);
}

void ObjectFactory::deVisualizeObjects(const std::vector<ObjectNode::Ptr> &objs_to_delete){
    visualization_msgs::MarkerArray marker_array;
    ros::Time timestamp = ros::Time::now();
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = timestamp;
    for (const auto & obj_node : objs_to_delete) {
        marker.ns     = "obj_position";
        marker.id     = obj_node->id;
        marker.action = visualization_msgs::Marker::DELETE;
        marker_array.markers.push_back(marker);
        marker.ns     = "obb_corners";
        marker.id     = obj_node->id;
        marker.action = visualization_msgs::Marker::DELETE;
        marker_array.markers.push_back(marker);
        marker.ns     = "obj_label";
        marker.id     = obj_node->id;
        marker.action = visualization_msgs::Marker::DELETE;
        marker_array.markers.push_back(marker);
    }
    obj_update_vis_pub_.publish(marker_array);
    ros::Duration(0.001).sleep();
}

void ObjectFactory::visualizeObjBoundingBox(visualization_msgs::Marker &marker,
                                            const ObjectNode::Ptr& obj_node, int id, const ros::Time &timestamp, bool use_axis_box=false) {
    if (obj_node->obb_corners->size() != 8) {
        INFO_MSG_YELLOW("[ObjFactory] Object Corners size not match, skip this object ...");
        return ;
    }
    visualization_msgs::Marker box_marker;
    box_marker.header.frame_id = "world"; // 你可以根据需要更改frame_id
    box_marker.header.stamp = timestamp;
    box_marker.ns     = "obb_corners";
    box_marker.id     = id;
    box_marker.type   = visualization_msgs::Marker::LINE_LIST;
    box_marker.action = visualization_msgs::Marker::ADD;
    box_marker.pose.orientation.w = 1.0;
    box_marker.scale.x = 0.01; // 线的宽度
    box_marker.color.r = obj_node->color.x() / 255.0f;
    box_marker.color.g = obj_node->color.y() / 255.0f;
    box_marker.color.b = obj_node->color.z() / 255.0f;
    box_marker.color.a = 1.0f;

    // 添加包围盒的边
    pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> obb_corners;
    if (use_axis_box)
        obb_corners = obj_node->obb_axis;
    else
        obb_corners = obj_node->obb_corners;
    box_marker.points.push_back(pclToGeoPt(obb_corners->points[0]));
    box_marker.points.push_back(pclToGeoPt(obb_corners->points[1]));
    box_marker.points.push_back(pclToGeoPt(obb_corners->points[1]));
    box_marker.points.push_back(pclToGeoPt(obb_corners->points[2]));
    box_marker.points.push_back(pclToGeoPt(obb_corners->points[2]));
    box_marker.points.push_back(pclToGeoPt(obb_corners->points[3]));
    box_marker.points.push_back(pclToGeoPt(obb_corners->points[3]));
    box_marker.points.push_back(pclToGeoPt(obb_corners->points[0]));
    box_marker.points.push_back(pclToGeoPt(obb_corners->points[4]));
    box_marker.points.push_back(pclToGeoPt(obb_corners->points[5]));
    box_marker.points.push_back(pclToGeoPt(obb_corners->points[5]));
    box_marker.points.push_back(pclToGeoPt(obb_corners->points[6]));
    box_marker.points.push_back(pclToGeoPt(obb_corners->points[6]));
    box_marker.points.push_back(pclToGeoPt(obb_corners->points[7]));
    box_marker.points.push_back(pclToGeoPt(obb_corners->points[7]));
    box_marker.points.push_back(pclToGeoPt(obb_corners->points[4]));
    box_marker.points.push_back(pclToGeoPt(obb_corners->points[0]));
    box_marker.points.push_back(pclToGeoPt(obb_corners->points[4]));
    box_marker.points.push_back(pclToGeoPt(obb_corners->points[5]));
    box_marker.points.push_back(pclToGeoPt(obb_corners->points[1]));
    box_marker.points.push_back(pclToGeoPt(obb_corners->points[6]));
    box_marker.points.push_back(pclToGeoPt(obb_corners->points[2]));
    box_marker.points.push_back(pclToGeoPt(obb_corners->points[7]));
    box_marker.points.push_back(pclToGeoPt(obb_corners->points[3]));
    marker = box_marker;
}

void ObjectFactory::visualizeObjPosition(visualization_msgs::Marker &marker, const ObjectNode::Ptr &obj_node, int id, const ros::Time &timestamp) {
    visualization_msgs::Marker pos_marker;
    pos_marker.header.frame_id = "world"; // 你可以根据需要更改frame_id
    pos_marker.header.stamp = timestamp;
    pos_marker.ns     = "obj_position";
    pos_marker.id     = id;
    pos_marker.type   = visualization_msgs::Marker::SPHERE;
    pos_marker.action = visualization_msgs::Marker::ADD;
    pos_marker.pose.position = eigenToGeoPt(obj_node->pos);
    pos_marker.pose.orientation.w = 1.0;
    pos_marker.scale.x = 0.1; pos_marker.scale.y = 0.1;pos_marker.scale.z = 0.1;
    pos_marker.color.r = obj_node->color.x() / 255.0f;
    pos_marker.color.g = obj_node->color.y() / 255.0f;
    pos_marker.color.b = obj_node->color.z() / 255.0f;
    pos_marker.color.a = 1.0f;
    marker = pos_marker;
}

void ObjectFactory::visualizeObjLabel(visualization_msgs::Marker &marker, const ObjectNode::Ptr &obj_node, int id, const ros::Time &timestamp) {
    visualization_msgs::Marker label_marker;
    label_marker.header.frame_id = "world"; // 你可以根据需要更改frame_id
    label_marker.header.stamp = timestamp;
    label_marker.ns     = "obj_label";
    label_marker.id     = id;
    label_marker.type   = visualization_msgs::Marker::TEXT_VIEW_FACING;
    label_marker.action = visualization_msgs::Marker::ADD;
    label_marker.pose.position = eigenToGeoPt(obj_node->pos);
    label_marker.pose.position.z += 0.1;
    label_marker.pose.orientation.w = 1.0;
    label_marker.scale.z = 0.3;
    label_marker.color.r = obj_node->color.x() / 255.0f;
    label_marker.color.g = obj_node->color.y() / 255.0f;
    label_marker.color.b = obj_node->color.z() / 255.0f;
    label_marker.color.a = 1.0f;
    std::string label_text = "[" + std::to_string(obj_node->id) + "] " + obj_node->label;
    label_marker.text = label_text;
    marker = label_marker;
}

void ObjectFactory::visualizeObjEdgeAll(visualization_msgs::Marker &marker) {
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns     = "obj_edge_all";
    marker.type   = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.02; marker.scale.y = 0.02; marker.scale.z = 0.02;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;

    for (const auto & obj : object_map_) {
        if (obj.second->edge.polyhedron_father != nullptr) {
            marker.points.push_back(eigenToGeoPt(obj.second->pos));
            marker.points.push_back(eigenToGeoPt(obj.second->edge.polyhedron_father->center_));
        }
    }
    // INFO_MSG_YELLOW("--------------- ");
    // INFO_MSG_YELLOW("ObjEdgeAll Marker Points Size: " << marker.points.size());
    // INFO_MSG_YELLOW("--------------- ");
}

//
// Created by gwq on 12/7/25.
//
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <vector>

// 定义同步策略：由于传输可能有延迟，使用近似时间同步
typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::CompressedImage,
    sensor_msgs::CompressedImage
> MySyncPolicy;

class RealsenseProcessor {
private:
    ros::NodeHandle nh;

    // 消息过滤器订阅者
    message_filters::Subscriber<sensor_msgs::CompressedImage> color_sub;
    message_filters::Subscriber<sensor_msgs::CompressedImage> depth_sub;
    message_filters::Synchronizer<MySyncPolicy> sync;

    // 相机内参和畸变参数
    cv::Mat K, D;
    cv::Mat map1, map2;
    cv::Size image_size;
    cv::Mat new_camera_matrix;

public:
    RealsenseProcessor() :
        sync(MySyncPolicy(10), color_sub, depth_sub), // 同步队列长度 10
        image_size(640, 480)
    {
        // 1. 设置订阅 Topic
        // 请根据实际情况确认 Topic 名称
        // 如果开启了 align_depth，深度 topic 通常含有 aligned_depth_to_color
        std::string color_topic = "/camera/color/image_raw/compressed";
        std::string depth_topic = "/camera/aligned_depth_to_color/image_raw/compressedDepth";

        ROS_INFO("Subscribing to:\n  Color: %s\n  Depth: %s", color_topic.c_str(), depth_topic.c_str());

        color_sub.subscribe(nh, color_topic, 1);
        depth_sub.subscribe(nh, depth_topic, 1);

        // 注册回调
        sync.registerCallback(boost::bind(&RealsenseProcessor::callback, this, _1, _2));

        // 2. 初始化标定参数
        initCalibration();
    }

    // 初始化内参和去畸变映射表
    void initCalibration() {
        // 内参 K (根据您之前提供的数据)
        double k_data[] = {
            387.3385009765625, 0.0, 321.9053649902344,
            0.0, 386.7434387207031, 245.8605499267578,
            0.0, 0.0, 1.0
        };
        K = cv::Mat(3, 3, CV_64F, k_data);

        // 畸变系数 D
        double d_data[] = {
            -0.0561770461499691, 0.06432759761810303,
            -9.787999260879587e-06, 0.00038565468275919557,
            -0.0207914300262928
        };
        D = cv::Mat(1, 5, CV_64F, d_data);

        // 预计算重映射表 (Init Undistort Maps)
        // alpha=1 保留所有像素，alpha=0 裁剪黑边
        new_camera_matrix = cv::getOptimalNewCameraMatrix(K, D, image_size, 1, image_size, 0);
        cv::initUndistortRectifyMap(K, D, cv::Mat(), new_camera_matrix,
                                    image_size, CV_16SC2, map1, map2);

        ROS_INFO("Calibration initialized.");
    }

    // --- 修正后的深度解码函数 ---
    cv::Mat decodeRealsenseCompressedDepth(const sensor_msgs::CompressedImageConstPtr &msg) {
        // 1. 校验格式
        // ROS compressedDepth 的标准头部长度是 12 字节
        const size_t header_size = 12;

        if (msg->data.size() <= header_size) {
            ROS_ERROR("Compressed depth data is too short!");
            return cv::Mat();
        }

        // 2. 解析头部参数 (可选，用于调试)
        // [0-3: config/enum], [4-7: depth_max], [8-11: depth_quantization]
        // 这里我们主要为了解压，直接跳过即可，但保留读取逻辑供参考
        float depth_quant_a, depth_quant_b;
        memcpy(&depth_quant_a, &msg->data[4], sizeof(float));
        memcpy(&depth_quant_b, &msg->data[8], sizeof(float));
        // ROS_DEBUG("Depth Quant parameters: %f, %f", depth_quant_a, depth_quant_b);

        // 3. 核心步骤：跳过 12 字节头部，提取纯图像数据
        // 注意：这里需要从 msg->data (vector) 中切片
        const std::vector<uint8_t> imageData(msg->data.begin() + header_size, msg->data.end());

        // 4. 使用 OpenCV 解码
        // 关键 flag: cv::IMREAD_UNCHANGED (或 -1)。
        // 只有这个 flag 才能保证解码出 16位 (CV_16U) 的原始深度，否则会被转成 8位 BGR。
        cv::Mat decoded_img = cv::imdecode(imageData, cv::IMREAD_UNCHANGED);

        if (decoded_img.empty()) {
            ROS_ERROR("Failed to decode compressed depth image (imdecode returned empty)");
            return cv::Mat();
        }
        return decoded_img;
    }

    // 辅助：将16位深度转为伪彩色用于显示
    cv::Mat colorizeDepth(const cv::Mat& depth_16u) {
        cv::Mat depth_8u, depth_color;
        // 归一化：假设 0-3米 (3000mm) 范围
        depth_16u.convertTo(depth_8u, CV_8UC1, 255.0 / 3000.0);
        cv::applyColorMap(depth_8u, depth_color, cv::COLORMAP_JET);
        return depth_color;
    }

    void callback(const sensor_msgs::CompressedImageConstPtr& color_msg,
                  const sensor_msgs::CompressedImageConstPtr& depth_msg)
    {
        try {
            // --- A. 解码 Color (普通的 JPEG 压缩) ---
            cv::Mat color_raw = cv::imdecode(cv::Mat(color_msg->data), cv::IMREAD_COLOR);

            // --- B. 解码 Depth (使用修正后的函数处理 compressedDepth) ---
            cv::Mat depth_raw = decodeRealsenseCompressedDepth(depth_msg);

            if (color_raw.empty() || depth_raw.empty()) return;

            // --- C. 去畸变 (Undistort) ---
            cv::Mat color_undist, depth_undist;

            // 1. Color: 线性插值，图像平滑
            cv::remap(color_raw, color_undist, map1, map2, cv::INTER_LINEAR);

            // 2. Depth: 最近邻插值 (Nearest Neighbor)，严禁使用 Linear
            // 防止深度边缘出现不存在的中间值
            cv::remap(depth_raw, depth_undist, map1, map2, cv::INTER_NEAREST);

            // --- D. 可视化 ---
            // 拼接显示：左边原始，右边去畸变
            cv::Mat depth_raw_vis = colorizeDepth(depth_raw);
            cv::Mat depth_undist_vis = colorizeDepth(depth_undist);

            cv::Mat row_color, row_depth, combined;
            cv::hconcat(color_raw, color_undist, row_color);
            cv::hconcat(depth_raw_vis, depth_undist_vis, row_depth);
            cv::vconcat(row_color, row_depth, combined);

            // 标注
            cv::putText(combined, "Raw Color", cv::Point(20, 30), 0, 0.8, cv::Scalar(0,255,0), 2);
            cv::putText(combined, "Undistorted Color", cv::Point(640+20, 30), 0, 0.8, cv::Scalar(0,255,0), 2);
            cv::putText(combined, "Raw Depth", cv::Point(20, 480+30), 0, 0.8, cv::Scalar(255,255,255), 2);
            cv::putText(combined, "Undistorted Depth (NN)", cv::Point(640+20, 480+30), 0, 0.8, cv::Scalar(255,255,255), 2);

            cv::imshow("RealSense Undistort View", combined);
            cv::waitKey(1);

        } catch (std::exception& e) {
            ROS_ERROR("Exception in callback: %s", e.what());
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "realsense_undistort_node");

    RealsenseProcessor processor;

    ros::spin();
    return 0;
}
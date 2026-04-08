#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
// 假设 pt_cloud_tools.h 和 INFO_MSG 宏已正确包含和定义
// #include <../include/scene_graph/pt_cloud_tools.h>

#include <iostream>
#include <cmath>
#include <random>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

// 辅助函数：将十六进制颜色字符串转换为 RGB 结构体
struct RgbColor {
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

// 接受十六进制字符串（如 "#FF0000"）并返回 RgbColor
RgbColor hexToRgb(const std::string& hex_color) {
    if (hex_color.length() != 7 || hex_color[0] != '#') {
        // 默认返回白色
        return {255, 255, 255};
    }

    // 从位置 1, 3, 5 开始解析 R, G, B 的十六进制值
    unsigned int r_val, g_val, b_val;
    std::stringstream ss;

    // 解析 R
    ss << std::hex << hex_color.substr(1, 2);
    ss >> r_val;
    ss.clear(); // 清除状态标志

    // 解析 G
    ss << std::hex << hex_color.substr(3, 2);
    ss >> g_val;
    ss.clear();

    // 解析 B
    ss << std::hex << hex_color.substr(5, 2);
    ss >> b_val;

    return {(uint8_t)r_val, (uint8_t)g_val, (uint8_t)b_val};
}

// 📌 更改：函数现在接受一个颜色参数 (十六进制字符串)
pcl::PointCloud<pcl::PointXYZRGB>::Ptr generateRandomPointsInSphere(
    const Eigen::Vector4f& sphere_center,
    float sphere_radius,
    int num_points,
    const std::string& hex_color)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud->width = num_points;
    cloud->height = 1;
    cloud->is_dense = false;
    cloud->points.resize(cloud->width * cloud->height);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(-sphere_radius, sphere_radius);

    // 将颜色参数转换为 RGB 结构体
    RgbColor color = hexToRgb(hex_color);

    for (int i = 0; i < num_points; ++i)
    {
        pcl::PointXYZRGB point;
        float x, y, z;
        float distance;

        do {
            x = dis(gen);
            y = dis(gen);
            z = dis(gen);
            distance = std::sqrt(x * x + y * y + z * z);
        } while (distance > sphere_radius);

        point.x = x + sphere_center[0];
        point.y = y + sphere_center[1];
        point.z = z + sphere_center[2];

        // 📌 更改：使用固定的颜色
        point.r = color.r;
        point.g = color.g;
        point.b = color.b;

        cloud->points[i] = point;
    }

    return cloud;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "random_points_in_sphere");
    ros::NodeHandle nh("~");

    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2> ("/cloud_test", 1);

    // 📌 定义两个不同的颜色 (例如：红色和蓝色)
    const std::string color1_hex = "#FF0000"; // 红色
    const std::string color2_hex = "#0000FF"; // 蓝色
    int num_points = 10000;
    float radius = 1.0;

    // 📌 更改：调用时传递指定的颜色
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1 = generateRandomPointsInSphere(
        Eigen::Vector4f(0.0, 0.0, 0.0, 1.0), radius, num_points, color1_hex);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 = generateRandomPointsInSphere(
        Eigen::Vector4f(1.0, 0.0, 0.0, 1.0), radius, num_points, color2_hex);

    // --- 省略了原始代码中不相关的部分，例如 pt_cloud_tools.h 的依赖 ---

    // 对这两个点云做体素滤波
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setLeafSize(0.07f, 0.07f, 0.07f);

    sor.setInputCloud(cloud1);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud1(new pcl::PointCloud<pcl::PointXYZRGB>);
    sor.filter(*filtered_cloud1);

    sor.setInputCloud(cloud2);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud2(new pcl::PointCloud<pcl::PointXYZRGB>);
    sor.filter(*filtered_cloud2);

    // 重新赋值给 cloud1 和 cloud2 (如果你想使用过滤后的版本进行后续计算)
    cloud1 = filtered_cloud1;
    cloud2 = filtered_cloud2;


    // 假设 INFO_MSG 已定义，否则注释掉或替换为 ROS_INFO
    // INFO_MSG("cloud1 size : " << cloud1->size() << " cloud2 size : " << cloud2->size());
    ROS_INFO("cloud1 size : %zu, cloud2 size : %zu", cloud1->size(), cloud2->size());

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_all = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

    // 合并点云
    *cloud_all = *cloud1;
    *cloud_all += *cloud2;

    // 假设 PointCloudOverlapCalculator 可用，否则注释掉
    /*
    PointCloudOverlapCalculator cloud_similarity_server{};
    std::cout << "cloud similarity score: " << cloud_similarity_server.calculateOverlapBInA(cloud1, cloud2, 0.07f)<< std::endl;
    */

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud_all, output);
    output.header.frame_id = "world";

    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        pub.publish(output);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
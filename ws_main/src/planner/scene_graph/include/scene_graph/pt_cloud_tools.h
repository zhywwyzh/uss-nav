//
// Created by gwq on 7/17/25.
//

#ifndef BOX_INTERSECTION_SERVER_H
#define BOX_INTERSECTION_SERVER_H

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <vector>
#include <algorithm>
#include <memory>
#include <omp.h>

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

#define INFO_MSG(str)        do {std::cout << str << std::endl; } while(false)
#define INFO_MSG_RED(str)    do {std::cout << "\033[31m" << str << "\033[0m" << std::endl; } while(false)
#define INFO_MSG_GREEN(str)  do {std::cout << "\033[32m" << str << "\033[0m" << std::endl; } while(false)
#define INFO_MSG_YELLOW(str) do {std::cout << "\033[33m" << str << "\033[0m" << std::endl; } while(false)
#define INFO_MSG_BLUE(str)   do {std::cout << "\033[34m" << str << "\033[0m" << std::endl; } while(false)

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/filter.h>
#include <ros/ros.h>
#include <omp.h> // 引入 OpenMP 头文件

class PointCloudOverlapCalculator {
public:
    double calculateOverlapBInA(
        const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud_a,
        const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud_b,
        double distance_threshold = 0.01) {

        // 1. 基础检查
        if (!cloud_a || !cloud_b || cloud_a->empty() || cloud_b->empty()) {
            return 0.0;
        }

        // 2. 智能 NaN 处理 (避免不必要的内存拷贝)
        pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud_a_ptr = cloud_a;
        pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud_b_ptr = cloud_b;

        // 只有当点云包含 NaN 时，才执行过滤和拷贝
        if (!cloud_a->is_dense) {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGB>);
            std::vector<int> indices;
            pcl::removeNaNFromPointCloud(*cloud_a, *temp, indices);
            cloud_a_ptr = temp;
        }

        // 注意：对于 B 云，如果后面用 indices 访问，其实不需要移除 NaN，但为了逻辑简单，这里还是移除了
        if (!cloud_b->is_dense) {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGB>);
            std::vector<int> indices;
            pcl::removeNaNFromPointCloud(*cloud_b, *temp, indices);
            cloud_b_ptr = temp;
        }

        if (cloud_a_ptr->empty() || cloud_b_ptr->empty()) return 0.0;

        // 3. 构建 KD-Tree
        // 使用 PCL 的 KdTreeFLANN，速度通常比 pcl::search::KdTree 快
        pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
        kdtree.setInputCloud(cloud_a_ptr);

        int overlap_count = 0;
        double squared_dist_thresh = distance_threshold * distance_threshold;

        // 4. 并行计算 (OpenMP)
        // reduction(+:overlap_count) 保证了计数的原子性和线程安全
        #pragma omp parallel for reduction(+:overlap_count) num_threads(4)
        for (size_t i = 0; i < cloud_b_ptr->size(); ++i) {
            const auto& point = cloud_b_ptr->points[i];

            // 在并行区域内部声明 vector，确保每个线程有独立的内存空间
            // 虽然分配有开销，但 nearestKSearch 内部也会做 resize，且并行带来的收益远大于此开销
            std::vector<int> pointIdxNKNSearch(1);
            std::vector<float> pointNKNSquaredDistance(1);

            // 执行搜索
            if (kdtree.nearestKSearch(point, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
                if (pointNKNSquaredDistance[0] < squared_dist_thresh) {
                    overlap_count++;
                }
            }
        }

        return static_cast<double>(overlap_count) / static_cast<double>(cloud_b_ptr->size());
    }
};


class CloudIntersectionServer {
private:
    // 定义平面方程 ax + by + cz + d = 0
    struct Plane {
        float a, b, c, d;
    };

public:
    CloudIntersectionServer() = default;
    ~CloudIntersectionServer() = default;

    // 计算两个任意放置长方体的相交体积
    double calculateIntersectionVolume(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cuboid1,
                                      const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cuboid2) {
        // 验证输入
        if (!validateInput(cuboid1, cuboid2)) {
            ROS_ERROR("输入验证失败：每个长方体需要8个顶点");
            return 0.0;
        }

        // 提取长方体的6个面
        std::vector<Plane> faces1 = extractFaces(cuboid1);
        std::vector<Plane> faces2 = extractFaces(cuboid2);

        // 使用分离轴定理检测是否相交
        if (areBoxesSeparated(cuboid1, cuboid2, faces1, faces2)) {
            return 0.0;
        }

        // 计算交集多面体
        pcl::PointCloud<pcl::PointXYZ>::Ptr intersection = clipCuboidByPlanes(cuboid1, faces2);
        intersection = clipCuboidByPlanes(intersection, faces1);

        // 计算交集体积
        return computeConvexPolyhedronVolume(intersection);
    }

private:
    // 验证输入数据
    bool validateInput(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cuboid1,
                      const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cuboid2) const {
        return (cuboid1->size() == 8 && cuboid2->size() == 8);
    }

    // 提取长方体的6个面
    std::vector<Plane> extractFaces(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cuboid) const {
        std::vector<Plane> faces;

        // 底面 (0-1-2-3)
        faces.push_back(computePlane(cuboid->points[0], cuboid->points[1], cuboid->points[2]));
        // 顶面 (4-5-6-7)
        faces.push_back(computePlane(cuboid->points[4], cuboid->points[7], cuboid->points[6]));
        // 前面 (0-3-7-4)
        faces.push_back(computePlane(cuboid->points[0], cuboid->points[4], cuboid->points[7]));
        // 后面 (1-5-6-2)
        faces.push_back(computePlane(cuboid->points[1], cuboid->points[2], cuboid->points[6]));
        // 左面 (0-1-5-4)
        faces.push_back(computePlane(cuboid->points[0], cuboid->points[1], cuboid->points[5]));
        // 右面 (3-2-6-7)
        faces.push_back(computePlane(cuboid->points[3], cuboid->points[7], cuboid->points[6]));

        return faces;
    }

    // 计算平面方程
    Plane computePlane(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2, const pcl::PointXYZ& p3) const {
        Eigen::Vector3f v1(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
        Eigen::Vector3f v2(p3.x - p1.x, p3.y - p1.y, p3.z - p1.z);
        Eigen::Vector3f normal = v1.cross(v2).normalized();

        Plane plane;
        plane.a = normal.x();
        plane.b = normal.y();
        plane.c = normal.z();
        plane.d = -(normal.x() * p1.x + normal.y() * p1.y + normal.z() * p1.z);
        return plane;
    }

    // 判断点是否在平面的正方向
    bool isPointInFrontOfPlane(const pcl::PointXYZ& point, const Plane& plane) const {
        return plane.a * point.x + plane.b * point.y + plane.c * point.z + plane.d > 0;
    }

    // 使用分离轴定理检测两个长方体是否分离
    bool areBoxesSeparated(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cuboid1,
                          const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cuboid2,
                          const std::vector<Plane>& faces1,
                          const std::vector<Plane>& faces2) const {
        // 检查所有可能的分离轴: 两个长方体的面法线
        for (const auto& face : faces1) {
            Eigen::Vector3f axis(face.a, face.b, face.c);
            if (areShapesSeparated(cuboid1, cuboid2, axis))
                return true;
        }

        for (const auto& face : faces2) {
            Eigen::Vector3f axis(face.a, face.b, face.c);
            if (areShapesSeparated(cuboid1, cuboid2, axis))
                return true;
        }

        return false;
    }

    // 判断两个形状是否被给定轴分离
    bool areShapesSeparated(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& shape1,
                          const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& shape2,
                          const Eigen::Vector3f& axis) const {
        float min1 = FLT_MAX, max1 = -FLT_MAX;
        float min2 = FLT_MAX, max2 = -FLT_MAX;

        // 计算shape1在轴上的投影范围
        for (const auto& point : shape1->points) {
            float projection = point.x * axis.x() + point.y * axis.y() + point.z * axis.z();
            min1 = std::min(min1, projection);
            max1 = std::max(max1, projection);
        }

        // 计算shape2在轴上的投影范围
        for (const auto& point : shape2->points) {
            float projection = point.x * axis.x() + point.y * axis.y() + point.z * axis.z();
            min2 = std::min(min2, projection);
            max2 = std::max(max2, projection);
        }

        // 检查投影范围是否重叠
        return (max1 < min2) || (max2 < min1);
    }

    // 使用一组平面裁剪长方体
    pcl::PointCloud<pcl::PointXYZ>::Ptr clipCuboidByPlanes(
        const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cuboid,
        const std::vector<Plane>& planes) const {
        pcl::PointCloud<pcl::PointXYZ>::Ptr result =
            boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*cuboid);

        for (const auto& plane : planes) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr temp =
                boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

            // 对每一条边进行裁剪
            for (size_t i = 0; i < 12; ++i) {
                // 这里需要定义长方体的12条边
                size_t v1_idx, v2_idx;
                getEdgeVertices(i, v1_idx, v2_idx);

                const pcl::PointXYZ& v1 = result->points[v1_idx];
                const pcl::PointXYZ& v2 = result->points[v2_idx];

                bool v1_in = isPointInFrontOfPlane(v1, plane);
                bool v2_in = isPointInFrontOfPlane(v2, plane);

                if (v1_in && v2_in) {
                    // 两个点都在平面前方，保留第二个点
                    temp->push_back(v2);
                } else if (v1_in && !v2_in) {
                    // 第一个点在前方，第二个点在后方，计算交点
                    pcl::PointXYZ intersection = computeIntersection(v1, v2, plane);
                    temp->push_back(intersection);
                } else if (!v1_in && v2_in) {
                    // 第一个点在后方，第二个点在前方，计算交点并保留第二个点
                    pcl::PointXYZ intersection = computeIntersection(v1, v2, plane);
                    temp->push_back(intersection);
                    temp->push_back(v2);
                }
                // 如果两个点都在后方，则不保留任何点
            }

            // 去重
            removeDuplicatePoints(temp);
            result = temp;
        }

        return result;
    }

    // 计算线段与平面的交点
    pcl::PointXYZ computeIntersection(const pcl::PointXYZ& v1, const pcl::PointXYZ& v2, const Plane& plane) const {
        Eigen::Vector3f dir(v2.x - v1.x, v2.y - v1.y, v2.z - v1.z);
        float denominator = plane.a * dir.x() + plane.b * dir.y() + plane.c * dir.z();

        if (std::abs(denominator) < 1e-6) {
            // 线段与平面平行
            return v1;
        }

        // 手动实现clamp功能
        float t = -(plane.a * v1.x + plane.b * v1.y + plane.c * v1.z + plane.d) / denominator;
        t = std::max(0.0f, std::min(1.0f, t));

        pcl::PointXYZ result;
        result.x = v1.x + t * dir.x();
        result.y = v1.y + t * dir.y();
        result.z = v1.z + t * dir.z();
        return result;
    }

    // 移除重复点
    void removeDuplicatePoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) const {
        const float threshold = 1e-4;
        std::vector<bool> keep(cloud->size(), true);

        for (size_t i = 0; i < cloud->size(); ++i) {
            if (!keep[i]) continue;

            for (size_t j = i + 1; j < cloud->size(); ++j) {
                if (!keep[j]) continue;

                float dx = cloud->points[i].x - cloud->points[j].x;
                float dy = cloud->points[i].y - cloud->points[j].y;
                float dz = cloud->points[i].z - cloud->points[j].z;

                if (dx*dx + dy*dy + dz*dz < threshold*threshold) {
                    keep[j] = false;
                }
            }
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr temp =
            boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        for (size_t i = 0; i < cloud->size(); ++i) {
            if (keep[i]) {
                temp->push_back(cloud->points[i]);
            }
        }

        *cloud = *temp;
    }

    // 获取边的两个顶点索引
    void getEdgeVertices(size_t edge_idx, size_t& v1, size_t& v2) const {
        // 根据边的索引获取两个顶点的索引
        // 这里需要根据长方体顶点的排列顺序定义12条边
        static const size_t edges[12][2] = {
            {0, 1}, {1, 2}, {2, 3}, {3, 0},  // 底面边
            {4, 5}, {5, 6}, {6, 7}, {7, 4},  // 顶面边
            {0, 4}, {1, 5}, {2, 6}, {3, 7}   // 垂直边
        };

        v1 = edges[edge_idx][0];
        v2 = edges[edge_idx][1];
    }

    // 使用散度定理计算凸多面体体积
    double computeConvexPolyhedronVolume(const pcl::PointCloud<pcl::PointXYZ>::Ptr& polyhedron) const {
        if (polyhedron->size() < 4) return 0.0;

        // 这里使用简化的体积计算方法
        // 实际应用中需要实现完整的凸多面体体积计算

        // 示例：假设我们有一个四面体
        if (polyhedron->size() == 4) {
            Eigen::Vector3f v0(polyhedron->points[0].x, polyhedron->points[0].y, polyhedron->points[0].z);
            Eigen::Vector3f v1(polyhedron->points[1].x, polyhedron->points[1].y, polyhedron->points[1].z);
            Eigen::Vector3f v2(polyhedron->points[2].x, polyhedron->points[2].y, polyhedron->points[2].z);
            Eigen::Vector3f v3(polyhedron->points[3].x, polyhedron->points[3].y, polyhedron->points[3].z);

            // 四面体体积公式
            return std::abs((v1 - v0).dot((v2 - v0).cross(v3 - v0))) / 6.0;
        }

        // 对于更复杂的多面体，需要更复杂的计算方法
        // 这里返回0作为占位符
        return 0.0;
    }
};

#endif //BOX_INTERSECTION_SERVER_H

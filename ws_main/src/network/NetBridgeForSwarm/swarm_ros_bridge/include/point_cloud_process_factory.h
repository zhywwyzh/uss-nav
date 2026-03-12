//
// Created by gwq on 12/4/25.
//

#ifndef SRC_POINT_CLOUD_PROCESS_FACTORY_H
#define SRC_POINT_CLOUD_PROCESS_FACTORY_H

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "swarm_ros_bridge/PtCloudCompress.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/compression/octree_pointcloud_compression.h"
#include "pcl/filters/voxel_grid.h"
#include <memory>
#include <sstream>
#include <string>

/**
 * @class PointCloudProcessFactory
 * @brief A factory class for processing point clouds with compression and downsampling.
 *        Handles PointXYZ, PointXYZRGB, and PointXYZRGBA point cloud types.
 */
class PointCloudProcessFactory {
public:
  enum CloudType { NORMAL, RGB, RGBA };
  
  /**
   * @brief Constructor
   * @param cloud_type Type of point cloud: "normal" (PointXYZ), "rgb" (PointXYZRGB), "rgba" (PointXYZRGBA)
   * @param downsample_size Voxel leaf size for downsampling (-1 means no downsampling)
   * @param enable_compression Whether to enable compression
   */
  explicit PointCloudProcessFactory(const std::string& cloud_type = "normal",
                                    double downsample_size = -1,
                                    bool enable_compression = false);

  ~PointCloudProcessFactory() = default;

  /**
   * @brief Process a PointCloud2 message and return compressed data
   * @param msg The input PointCloud2 message
   * @param compressed_data Output: stringstream containing compressed data
   * @param original_width Output: original cloud width
   * @param original_height Output: original cloud height
   * @param original_frame_id Output: original frame ID
   * @return true if processing succeeded, false otherwise
   */
  bool processPointCloud(const sensor_msgs::PointCloud2& msg,
                        std::stringstream& compressed_data,
                        uint32_t& original_width,
                        uint32_t& original_height,
                        std::string& original_frame_id);

  /**
   * @brief Decompress point cloud data and return as PointCloud2 message
   * @param compressed_data stringstream containing compressed data
   * @param original_width Original cloud width
   * @param original_height Original cloud height
   * @param original_frame_id Original frame ID
   * @return Decompressed PointCloud2 message
   */
  sensor_msgs::PointCloud2 decompressPointCloud(std::stringstream& compressed_data, 
                                                uint32_t original_width,
                                                uint32_t original_height,
                                                const std::string& original_frame_id);

  /**
   * @brief Set the cloud type after construction
   * @param cloud_type "normal", "rgb", or "rgba"
   * @return true if valid type, false otherwise
   */
  bool setCloudType(const std::string& cloud_type);

  /**
   * @brief Set the downsampling parameters
   * @param downsample_size Voxel leaf size (-1 means no downsampling)
   */
  void setDownsampleSize(double downsample_size) { downsample_size_ = downsample_size; }

  /**
   * @brief Enable or disable compression
   * @param enable Whether to enable compression
   */
  void setCompressionEnabled(bool enable) { enable_compression_ = enable; }

private:
  CloudType cloud_type_;
  double downsample_size_;
  bool enable_compression_;

  // Helper functions for different cloud types
  bool processNormalCloud(const sensor_msgs::PointCloud2& msg,
                         std::stringstream& compressed_data,
                         uint32_t& original_width,
                         uint32_t& original_height,
                         std::string& original_frame_id);

  bool processRGBCloud(const sensor_msgs::PointCloud2& msg,
                       std::stringstream& compressed_data,
                       uint32_t& original_width,
                       uint32_t& original_height,
                       std::string& original_frame_id);

  bool processRGBACloud(const sensor_msgs::PointCloud2& msg,
                        std::stringstream& compressed_data,
                        uint32_t& original_width,
                        uint32_t& original_height,
                        std::string& original_frame_id);

  sensor_msgs::PointCloud2 decompressNormalCloud(std::stringstream& compressed_data,
                                                  const std::string& original_frame_id);

  sensor_msgs::PointCloud2 decompressRGBCloud(std::stringstream& compressed_data,
                                               const std::string& original_frame_id);

  sensor_msgs::PointCloud2 decompressRGBACloud(std::stringstream& compressed_data,
                                                const std::string& original_frame_id);

  /**
   * @brief Convert string cloud type to enum
   * @param cloud_type_str Cloud type string: "normal", "rgb", or "rgba"
   * @return Corresponding CloudType enum value
   */
  static CloudType stringToCloudType(const std::string& cloud_type_str);

  /**
   * @brief Downsample a PointXYZ cloud
   * @param cloud_in Input cloud
   * @return Downsampled cloud
   */
  template<typename PointT>
  typename pcl::PointCloud<PointT>::Ptr downsampleCloud(
      const typename pcl::PointCloud<PointT>::Ptr& cloud_in);
};

#endif //SRC_POINT_CLOUD_PROCESS_FACTORY_H
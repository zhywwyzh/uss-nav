//
// Created by gwq on 12/4/25.
//

#include "point_cloud_process_factory.h"
#include <stdexcept>
#include <cstring>

PointCloudProcessFactory::PointCloudProcessFactory(const std::string& cloud_type,
                                                   double downsample_size,
                                                   bool enable_compression)
    : downsample_size_(downsample_size), enable_compression_(enable_compression) {
  setCloudType(cloud_type);
}

bool PointCloudProcessFactory::setCloudType(const std::string& cloud_type) {
  if (cloud_type == "normal") {
    cloud_type_ = NORMAL;
  } else if (cloud_type == "rgb") {
    cloud_type_ = RGB;
  } else if (cloud_type == "rgba") {
    cloud_type_ = RGBA;
  } else {
    ROS_WARN("[PointCloudProcessFactory] Invalid cloud type: %s, using 'normal'", cloud_type.c_str());
    cloud_type_ = NORMAL;
    return false;
  }
  return true;
}

bool PointCloudProcessFactory::processPointCloud(const sensor_msgs::PointCloud2& msg,
                                                  std::stringstream& compressed_data,
                                                  uint32_t& original_width,
                                                  uint32_t& original_height,
                                                  std::string& original_frame_id) {
  try {
    switch (cloud_type_) {
      case NORMAL:
        return processNormalCloud(msg, compressed_data, original_width, original_height, original_frame_id);
      case RGB:
        return processRGBCloud(msg, compressed_data, original_width, original_height, original_frame_id);
      case RGBA:
        return processRGBACloud(msg, compressed_data, original_width, original_height, original_frame_id);
      default:
        ROS_ERROR("[PointCloudProcessFactory] Unknown cloud type");
        return false;
    }
  } catch (const std::exception& e) {
    ROS_ERROR("[PointCloudProcessFactory] Exception during point cloud processing: %s", e.what());
    return false;
  }
}

bool PointCloudProcessFactory::processNormalCloud(const sensor_msgs::PointCloud2& msg,
                                                   std::stringstream& compressed_data,
                                                   uint32_t& original_width,
                                                   uint32_t& original_height,
                                                   std::string& original_frame_id) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(msg, *cloud_in);

  // Store original metadata
  original_width = cloud_in->width;
  original_height = cloud_in->height;
  original_frame_id = cloud_in->header.frame_id;

  // Serialize timestamp to stringstream
  uint32_t sec = msg.header.stamp.sec;
  uint32_t nsec = msg.header.stamp.nsec;
  compressed_data.write(reinterpret_cast<const char*>(&sec), sizeof(uint32_t));
  compressed_data.write(reinterpret_cast<const char*>(&nsec), sizeof(uint32_t));

  // Perform downsampling if needed
  if (downsample_size_ > 0) {
    cloud_in = downsampleCloud<pcl::PointXYZ>(cloud_in);
  }

  // Compress if enabled
  if (enable_compression_) {
    std::unique_ptr<pcl::io::OctreePointCloudCompression<pcl::PointXYZ>> compressor;
    compressor = std::make_unique<pcl::io::OctreePointCloudCompression<pcl::PointXYZ>>(
        pcl::io::LOW_RES_ONLINE_COMPRESSION_WITHOUT_COLOR, false, 1e-3, 1e-3, false);
    compressor->encodePointCloud(cloud_in, compressed_data);
    return true;
  }

  // If no compression, serialize the cloud directly
  return true;
}

bool PointCloudProcessFactory::processRGBCloud(const sensor_msgs::PointCloud2& msg,
                                                std::stringstream& compressed_data,
                                                uint32_t& original_width,
                                                uint32_t& original_height,
                                                std::string& original_frame_id) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::fromROSMsg(msg, *cloud_in);

  // Store original metadata
  original_width = cloud_in->width;
  original_height = cloud_in->height;
  original_frame_id = cloud_in->header.frame_id;

  // Serialize timestamp to stringstream
  uint32_t sec = msg.header.stamp.sec;
  uint32_t nsec = msg.header.stamp.nsec;
  compressed_data.write(reinterpret_cast<const char*>(&sec), sizeof(uint32_t));
  compressed_data.write(reinterpret_cast<const char*>(&nsec), sizeof(uint32_t));

  // Perform downsampling if needed
  if (downsample_size_ > 0) {
    cloud_in = downsampleCloud<pcl::PointXYZRGB>(cloud_in);
  }

  // Compress if enabled
  if (enable_compression_) {
    std::unique_ptr<pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>> compressor;
    compressor = std::make_unique<pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>>(
        pcl::io::LOW_RES_ONLINE_COMPRESSION_WITH_COLOR, false, 1e-3, 1e-3, false);
    compressor->encodePointCloud(cloud_in, compressed_data);
    return true;
  }

  // If no compression, serialize the cloud directly
  return true;
}

bool PointCloudProcessFactory::processRGBACloud(const sensor_msgs::PointCloud2& msg,
                                                 std::stringstream& compressed_data,
                                                 uint32_t& original_width,
                                                 uint32_t& original_height,
                                                 std::string& original_frame_id) {
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZRGBA>());
  pcl::fromROSMsg(msg, *cloud_in);

  // Store original metadata
  original_width = cloud_in->width;
  original_height = cloud_in->height;
  original_frame_id = cloud_in->header.frame_id;

  // Serialize timestamp to stringstream
  uint32_t sec = msg.header.stamp.sec;
  uint32_t nsec = msg.header.stamp.nsec;
  compressed_data.write(reinterpret_cast<const char*>(&sec), sizeof(uint32_t));
  compressed_data.write(reinterpret_cast<const char*>(&nsec), sizeof(uint32_t));

  // Perform downsampling if needed
  if (downsample_size_ > 0) {
    cloud_in = downsampleCloud<pcl::PointXYZRGBA>(cloud_in);
  }

  // Compress if enabled
  if (enable_compression_) {
    std::unique_ptr<pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>> compressor;
    compressor = std::make_unique<pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>>(
        pcl::io::LOW_RES_ONLINE_COMPRESSION_WITH_COLOR, false, 1e-3, 1e-3, false);
    compressor->encodePointCloud(cloud_in, compressed_data);
    return true;
  }

  // If no compression, serialize the cloud directly
  return true;
}

sensor_msgs::PointCloud2 PointCloudProcessFactory::decompressPointCloud(
    std::stringstream& compressed_data,
    uint32_t original_width,
    uint32_t original_height,
    const std::string& original_frame_id) {
  try {
    // Read timestamp from the beginning of compressed_data
    uint32_t sec, nsec;
    compressed_data.read(reinterpret_cast<char*>(&sec), sizeof(uint32_t));
    compressed_data.read(reinterpret_cast<char*>(&nsec), sizeof(uint32_t));
    ros::Time timestamp(sec, nsec);

    sensor_msgs::PointCloud2 msg;
    switch (cloud_type_) {
      case NORMAL: {
        msg = decompressNormalCloud(compressed_data, original_frame_id);
        break;
      }
      case RGB: {
        msg = decompressRGBCloud(compressed_data, original_frame_id);
        break;
      }
      case RGBA: {
        msg = decompressRGBACloud(compressed_data, original_frame_id); 
        break;
      }
      default:
        ROS_ERROR("[PointCloudProcessFactory] Unknown cloud type for decompression");
        return sensor_msgs::PointCloud2();
    }
    
    // Set the timestamp from the compressed data
    msg.header.stamp = timestamp;
    return msg;
  } catch (const std::exception& e) {
    ROS_ERROR("[PointCloudProcessFactory] Exception during point cloud decompression: %s", e.what());
    return sensor_msgs::PointCloud2();
  }
}

sensor_msgs::PointCloud2 PointCloudProcessFactory::decompressNormalCloud(
    std::stringstream& compressed_data,
    const std::string& original_frame_id) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  std::unique_ptr<pcl::io::OctreePointCloudCompression<pcl::PointXYZ>> decompressor;
  decompressor = std::make_unique<pcl::io::OctreePointCloudCompression<pcl::PointXYZ>>(
      pcl::io::LOW_RES_ONLINE_COMPRESSION_WITHOUT_COLOR, false, 1e-3, 1e-3, false);
  decompressor->decodePointCloud(compressed_data, cloud);
  cloud->header.frame_id = original_frame_id;

  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*cloud, msg);
  return msg;
}

sensor_msgs::PointCloud2 PointCloudProcessFactory::decompressRGBCloud(
    std::stringstream& compressed_data,
    const std::string& original_frame_id) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  std::unique_ptr<pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>> decompressor;
  decompressor = std::make_unique<pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>>(
      pcl::io::LOW_RES_ONLINE_COMPRESSION_WITH_COLOR, false, 1e-3, 1e-3, false);
  decompressor->decodePointCloud(compressed_data, cloud);
  cloud->header.frame_id = original_frame_id;
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*cloud, msg);
  return msg;
}

sensor_msgs::PointCloud2 PointCloudProcessFactory::decompressRGBACloud(
    std::stringstream& compressed_data,
    const std::string& original_frame_id) {
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
  std::unique_ptr<pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>> decompressor;
  decompressor = std::make_unique<pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>>(
      pcl::io::LOW_RES_ONLINE_COMPRESSION_WITH_COLOR, false, 1e-3, 1e-3, false);
  decompressor->decodePointCloud(compressed_data, cloud);
  cloud->header.frame_id = original_frame_id;

  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*cloud, msg);
  return msg;
}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr PointCloudProcessFactory::downsampleCloud(
    const typename pcl::PointCloud<PointT>::Ptr& cloud_in) {
  typename pcl::PointCloud<PointT>::Ptr cloud_downsampled(new pcl::PointCloud<PointT>());
  pcl::VoxelGrid<PointT> voxel_filter;
  voxel_filter.setInputCloud(cloud_in);
  voxel_filter.setLeafSize(static_cast<float>(downsample_size_),
                          static_cast<float>(downsample_size_),
                          static_cast<float>(downsample_size_));
  voxel_filter.filter(*cloud_downsampled);
  return cloud_downsampled;
}

// Explicit template instantiation
template pcl::PointCloud<pcl::PointXYZ>::Ptr
PointCloudProcessFactory::downsampleCloud<pcl::PointXYZ>(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in);

template pcl::PointCloud<pcl::PointXYZRGB>::Ptr
PointCloudProcessFactory::downsampleCloud<pcl::PointXYZRGB>(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_in);

template pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
PointCloudProcessFactory::downsampleCloud<pcl::PointXYZRGBA>(
    const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud_in);
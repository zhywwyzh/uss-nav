# PointCloudProcessFactory 集成指南

## 概述

`PointCloudProcessFactory` 是一个专门用于处理点云数据的工厂类。它将点云的压缩、下采样和解压缩逻辑从 `TopicFactory` 中分离出来，提供更清晰的代码结构和更强的可复用性。

## 文件结构

```
swarm_ros_bridge/
├── include/
│   ├── point_cloud_process_factory.h     # 工厂类头文件
│   └── topic_factory.h                   # TopicFactory（已修改）
├── src/
│   ├── point_cloud_process_factory.cpp   # 工厂类实现
│   └── topic_factory.cpp                 # TopicFactory（已修改）
└── config/
    └── default.yaml                      # 配置文件示例
```

## 主要功能

### 1. PointCloudProcessFactory 类

#### 构造函数
```cpp
explicit PointCloudProcessFactory(const std::string& cloud_type = "normal",
                                  double downsample_size = -1,
                                  bool enable_compression = false);
```

**参数说明：**
- `cloud_type`: 点云类型（"normal"、"rgb" 或 "rgba"）
- `downsample_size`: 下采样叶子大小（-1 表示不下采样）
- `enable_compression`: 是否启用压缩

#### 核心方法

##### 1. processPointCloud - 处理点云
```cpp
bool processPointCloud(const sensor_msgs::PointCloud2& msg,
                      std::stringstream& compressed_data,
                      uint32_t& original_width,
                      uint32_t& original_height,
                      std::string& original_frame_id);
```

**功能：**
- 将 ROS PointCloud2 消息转换为相应类型的 PCL 点云
- 执行下采样（如需要）
- 执行压缩（如启用）
- 返回压缩后的字符串流

**返回值：** 成功返回 true，失败返回 false

##### 2. decompressPointCloud - 解压点云
```cpp
sensor_msgs::PointCloud2 decompressPointCloud(
    std::stringstream& compressed_data,
    uint32_t original_width,
    uint32_t original_height,
    const std::string& original_frame_id);
```

**功能：**
- 解压缩点云数据
- 恢复原始的点云元数据
- 转换为 ROS PointCloud2 消息格式

##### 3. 配置方法
```cpp
// 设置点云类型
bool setCloudType(const std::string& cloud_type);

// 设置下采样大小
void setDownsampleSize(double downsample_size);

// 启用或禁用压缩
void setCompressionEnabled(bool enable);
```

## 支持的点云类型

| 类型 | 说明 | 使用场景 |
|------|------|---------|
| `"normal"` | PointXYZ（仅X、Y、Z坐标） | 深度传感器、激光雷达 |
| `"rgb"` | PointXYZRGB（坐标+RGB颜色） | RGB-D相机、彩色点云 |
| `"rgba"` | PointXYZRGBA（坐标+RGBA颜色） | 带透明度的点云数据 |

## 与 TopicFactory 的集成

### 1. 初始化

在 `TopicFactory` 构造函数中，如果主题类型是 `sensor_msgs/PointCloud2`，会自动创建 `PointCloudProcessFactory` 实例：

```cpp
// Initialize point cloud processor for PointCloud2 topics
if (topic_cfg_.type_ == "sensor_msgs/PointCloud2") {
  cloud_processor_ = std::make_unique<PointCloudProcessFactory>(
      topic_cfg_.cloud_type_,
      topic_cfg_.cloud_downsample_,
      topic_cfg_.cloud_compress_);
}
```

### 2. 发送端处理（ptCloudProcess）

```cpp
template<typename T>
void TopicFactory::ptCloudProcess(const T &msg, size_t &data_len, 
                                  std::unique_ptr<uint8_t[]> &data) {
  // 使用工厂处理点云
  if (!cloud_processor_->processPointCloud(msg, compressed_data, 
                                           original_width, original_height, 
                                           original_frame_id)) {
    ROS_ERROR("[TopicFactory] Failed to process point cloud");
    return;
  }
  
  // 序列化压缩后的数据
  swarm_ros_bridge::PtCloudCompress msg_send_compressed;
  msg_send_compressed.compressed_data.data = compressed_data.str();
  // ... 继续序列化
}
```

### 3. 接收端处理（deserializePub）

```cpp
template <typename T>
void TopicFactory::deserializePub(uint8_t *buffer_ptr, size_t msg_size) {
  // 反序列化压缩数据
  swarm_ros_bridge::PtCloudCompress msg_compress;
  ser::deserialize(stream, msg_compress);
  
  // 使用工厂解压点云
  msg = cloud_processor_->decompressPointCloud(compressed_data,
                                               msg_compress.original_width,
                                               msg_compress.original_height,
                                               msg_compress.original_frame_id);
}
```

## 数据流示意图

### 发送端流程
```
ROS PointCloud2 Message
        ↓
PointCloudProcessFactory::processPointCloud()
        ↓
    ├─ 转换为PCL点云类型
    ├─ 下采样（如需）
    ├─ 压缩（如启用）
    └─ 输出: std::stringstream
        ↓
序列化为 PtCloudCompress 消息
        ↓
网络传输（ZMQ）
```

### 接收端流程
```
网络接收（ZMQ）
        ↓
反序列化 PtCloudCompress 消息
        ↓
PointCloudProcessFactory::decompressPointCloud()
        ↓
    ├─ 解压缩
    ├─ 转换为ROS点云
    └─ 恢复元数据
        ↓
发布到ROS话题
```

## 代码示例

### 基本使用

```cpp
// 创建工厂
PointCloudProcessFactory processor("rgb", 0.1, true);

// 处理点云
std::stringstream compressed_data;
uint32_t width, height;
std::string frame_id;

if (processor.processPointCloud(cloud_msg, compressed_data, 
                               width, height, frame_id)) {
  // 压缩成功，使用 compressed_data 进行传输
  std::string compressed_str = compressed_data.str();
}

// 解压点云
std::stringstream compressed_input;
compressed_input << received_data;
sensor_msgs::PointCloud2 decompressed = 
    processor.decompressPointCloud(compressed_input, width, height, frame_id);
```

### 动态配置

```cpp
// 在运行时更改点云类型
processor.setCloudType("rgba");
processor.setDownsampleSize(0.05);
processor.setCompressionEnabled(true);
```

## 性能优化建议

1. **内存管理**：
   - 工厂会自动管理临时的PCL点云对象
   - 避免频繁创建新的工厂实例

2. **压缩策略**：
   - 对于高频率话题，启用压缩可显著减少网络带宽
   - 无RGB数据的点云压缩效果最佳

3. **下采样**：
   - 根据点云大小和网络状况调整下采样参数
   - 建议范围：0.05-0.2（单位：米）

## 错误处理

工厂类会在以下情况返回失败：
- 无效的点云数据
- 点云类型不匹配
- 压缩/解压缩失败

所有错误都会通过 ROS_ERROR 打印详细日志。

## 向后兼容性

- 如果未指定 `cloudType`，默认使用 `"normal"` 类型
- 现有的配置文件无需修改即可继续使用

## 扩展性

要支持新的点云类型，只需：
1. 在 `PointCloudProcessFactory` 中添加新的 process/decompress 方法
2. 在 switch 语句中添加相应的分支
3. 更新配置文件的点云类型说明

## 常见问题

**Q: 如何选择合适的点云类型？**
A: 根据点云数据的格式选择：
- 没有颜色信息 → `"normal"`
- 有RGB颜色信息 → `"rgb"`
- 有RGBA颜色信息 → `"rgba"`

**Q: 压缩对点云质量有影响吗？**
A: 使用的是 LOW_RES_ONLINE_COMPRESSION，会保留关键特征信息。视觉上无明显差异。

**Q: 是否可以混合使用不同的点云类型？**
A: 可以，但发送端和接收端的类型配置必须一致，否则解压缩会失败。


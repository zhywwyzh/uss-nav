#ifndef TOPIC_BROADCASTER_BROADCASTER_NODE_H
#define TOPIC_BROADCASTER_BROADCASTER_NODE_H

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <scene_graph/EncodeMask.h>
#include <sensor_msgs/CompressedImage.h>

#include "topic_broadcaster/encodemask_codec.h"
#include "topic_broadcaster/frame_queue.h"
#include "topic_broadcaster/tcp_client.h"

// 广播器节点：
//  - 上行：订阅若干话题（raw=相机压缩图 / json=Odometry），按帧协议打包后
//    放入有界队列，由发送线程经【单条 TCP 双向连接】发往机载外部服务进程；
//  - 下行：接收线程解析外部服务回传的检测结果 JSON（topic_id=100），
//    解码为 scene_graph::EncodeMask 后发布回 ROS。
//
// 关键约束：外部服务未启动时节点绝不阻塞 —— 非阻塞 connect + 退避重连 +
// 有界队列丢帧 + 非阻塞 send（超时即放弃）。
class BroadcasterNode {
public:
    explicit BroadcasterNode(ros::NodeHandle& nh);
    ~BroadcasterNode();

private:
    // 上行话题配置（broadcast_topics/out 数组项）
    struct OutTopicCfg {
        std::string topic;      // 订阅话题名
        std::string codec;      // "raw"=CompressedImage 直传 ; "json"=Odometry
        double max_freq = 0.0;  // 节流上限(Hz)，0 表示不限制
        uint16_t topic_id = 0;  // 帧协议话题号：1=rgb 2=depth 3=odom
        uint64_t seq = 0;       // 该话题独立递增的帧序号
        ros::Time last_sent;    // 上次发送时间（节流判断用）
    };
    // 下行话题配置（broadcast_topics/in 数组项）
    struct InTopicCfg {
        std::string topic;  // 发布话题名
        std::string codec;  // "encodemask"
    };

    // 同步打包配置（sync_pack）：
    // rgb/depth 用 message_filters 近似时间同步强绑（容差 rgb_depth_slop），
    // odom 取最近缓存值（允许 odom_slop 延迟），三者合成一个组合帧(topic_id=50)
    // 上行，替代 out_topics 中 rgb/depth/odom 三条的独立发送，保证下游时间戳同步。
    struct SyncPackCfg {
        bool enable = false;                 // 是否启用同步打包
        std::string rgb_topic;               // rgb 压缩图话题
        std::string depth_topic;             // depth 压缩图话题
        std::string odom_topic;              // odom 话题
        double rgb_depth_slop = 0.05;        // rgb/depth 强同步容差(秒)
        double odom_slop = 0.1;              // odom 允许的延迟(秒)，取最近值即可
        double max_freq = 10.0;              // 组合帧节流上限(Hz)
        uint16_t topic_id = 50;              // 组合帧的帧协议话题号
    };

    void loadParams(ros::NodeHandle& pnh);          // 读取私有参数(~)
    void setupSubscribers(ros::NodeHandle& nh);     // 创建上行订阅
    void setupPublishers(ros::NodeHandle& nh);      // 创建下行发布
    void sendLoop();                                // 发送线程主循环
    void recvLoop();                                // 接收线程主循环

    // 上行回调（idx 为 out_topics_ 下标）
    void onImage(const sensor_msgs::CompressedImageConstPtr& msg, size_t idx);
    void onOdom(const nav_msgs::OdometryConstPtr& msg, size_t idx);

    // ---- 同步打包（sync_pack）----
    void setupSyncPack(ros::NodeHandle& nh);   // 创建同步订阅（rgb+depth 同步 + odom 独立）
    void onSyncRgbDepth(const sensor_msgs::CompressedImageConstPtr& rgb,
                        const sensor_msgs::CompressedImageConstPtr& depth);  // 同步回调→打包入队
    void onSyncOdom(const nav_msgs::OdometryConstPtr& msg);                  // odom 独立回调→缓存 JSON
    bool buildSyncPackFrame(const sensor_msgs::CompressedImageConstPtr& rgb,
                            const sensor_msgs::CompressedImageConstPtr& depth,
                            double stamp, std::vector<uint8_t>& frame);      // 组装 topic_id=50 组合帧

    // 下行处理：把 topic_id=100 的载荷(JSON)解码并发布
    void handleEncodeMask(const std::vector<uint8_t>& payload, double stamp);

    // 展示图发布：把下行 JSON 中可选字段 vis_b64（jpeg base64）解码为
    // sensor_msgs::CompressedImage 发布到 plot_pub_（/yoloe/plot），供 RViz/前端显示
    void publishPlotImage(const std::string& vis_b64);

    // ---- 连接参数 ----
    std::string server_ip_ = "127.0.0.1";  // 外部服务地址
    int server_port_ = 9010;               // 外部服务端口
    double reconnect_min_sec_ = 1.0;       // 重连最小退避(秒)
    double reconnect_max_sec_ = 5.0;       // 重连最大退避(秒，封顶)
    int send_timeout_ms_ = 100;            // 单帧发送超时(毫秒)
    int queue_size_ = 8;                   // 上行发送队列容量(帧)

    // ---- 话题配置 ----
    std::vector<OutTopicCfg> out_topics_;  // 上行话题
    std::vector<InTopicCfg> in_topics_;    // 下行话题

    // ---- 同步打包状态（sync_pack）----
    SyncPackCfg sync_pack_;            // 同步打包配置
    ros::Time sync_pack_last_sent_;    // 组合帧节流：上次发送时间
    uint64_t sync_pack_seq_ = 0;       // 组合帧独立递增的帧序号
    std::string latest_odom_json_;     // 最近 odom 的 JSON（odom_mutex_ 保护）
    ros::Time latest_odom_stamp_;      // 最近 odom 的接收时间戳（odom_mutex_ 保护）
    std::mutex odom_mutex_;            // 保护 latest_odom_* 的互斥锁
    // message_filters 同步订阅（rgb+depth，近似时间同步）
    std::unique_ptr<message_filters::Subscriber<sensor_msgs::CompressedImage>>
        sync_rgb_sub_, sync_depth_sub_;
    std::unique_ptr<message_filters::Synchronizer<
        message_filters::sync_policies::ApproximateTime<
            sensor_msgs::CompressedImage, sensor_msgs::CompressedImage>>>
        sync_sync_;
    std::vector<ros::Subscriber> extra_subs_;  // odom 独立订阅存放处

    // ---- ROS 句柄/订阅/发布 ----
    ros::NodeHandle nh_;
    std::vector<ros::Subscriber> subs_;        // 上行订阅
    std::vector<ros::Publisher> result_pubs_;  // 下行发布

    // ---- 检测展示图发布（/yoloe/plot 与 /yoloe/plot/compressed）----
    // 双话题同发一份 jpeg 压缩图：/yoloe/plot 为常规话题，
    // /yoloe/plot/compressed 供 RViz Image display 选 compressed transport 订阅。
    // 发布前按各话题订阅者数检查，无订阅者则跳过，省 CPU/带宽。
    std::string plot_topic_ = "/yoloe/plot";  // 常规展示图话题（可读参数覆盖）
    ros::Publisher plot_pub_;                 // 常规展示图发布器（sensor_msgs::CompressedImage, jpeg）
    std::string plot_compressed_topic_ = "/yoloe/plot/compressed";  // compressed 命名话题（可读参数覆盖）
    ros::Publisher plot_compressed_pub_;      // compressed 展示图发布器（sensor_msgs::CompressedImage, jpeg）

    // ---- 传输组件 ----
    TcpClient client_;  // 与外部服务的 TCP 连接（单条双向）
    std::shared_ptr<FrameQueue<std::vector<uint8_t>>> frame_queue_;  // 待发送帧队列
    std::thread send_thread_;  // 发送线程
    std::thread recv_thread_;  // 接收线程
    std::atomic<bool> stop_flag_{false};  // 停止标志（析构时置位）
};

#endif  // TOPIC_BROADCASTER_BROADCASTER_NODE_H

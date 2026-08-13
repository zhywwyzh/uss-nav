#ifndef TOPIC_BROADCASTER_BROADCASTER_NODE_H
#define TOPIC_BROADCASTER_BROADCASTER_NODE_H

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
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

    void loadParams(ros::NodeHandle& pnh);          // 读取私有参数(~)
    void setupSubscribers(ros::NodeHandle& nh);     // 创建上行订阅
    void setupPublishers(ros::NodeHandle& nh);      // 创建下行发布
    void sendLoop();                                // 发送线程主循环
    void recvLoop();                                // 接收线程主循环

    // 上行回调（idx 为 out_topics_ 下标）
    void onImage(const sensor_msgs::CompressedImageConstPtr& msg, size_t idx);
    void onOdom(const nav_msgs::OdometryConstPtr& msg, size_t idx);

    // 下行处理：把 topic_id=100 的载荷(JSON)解码并发布
    void handleEncodeMask(const std::vector<uint8_t>& payload, double stamp);

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

    // ---- ROS 句柄/订阅/发布 ----
    ros::NodeHandle nh_;
    std::vector<ros::Subscriber> subs_;        // 上行订阅
    std::vector<ros::Publisher> result_pubs_;  // 下行发布

    // ---- 传输组件 ----
    TcpClient client_;  // 与外部服务的 TCP 连接（单条双向）
    std::shared_ptr<FrameQueue<std::vector<uint8_t>>> frame_queue_;  // 待发送帧队列
    std::thread send_thread_;  // 发送线程
    std::thread recv_thread_;  // 接收线程
    std::atomic<bool> stop_flag_{false};  // 停止标志（析构时置位）
};

#endif  // TOPIC_BROADCASTER_BROADCASTER_NODE_H

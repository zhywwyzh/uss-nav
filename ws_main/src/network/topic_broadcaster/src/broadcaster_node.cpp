#include "topic_broadcaster/broadcaster_node.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <XmlRpcValue.h>
#include <arpa/inet.h>
#include <boost/bind.hpp>
#include <netinet/in.h>
#include <nlohmann/json.hpp>

// ---------------------------------------------------------------------------
// 帧协议工具函数（与外部 Python 服务共享的二进制协议，全部大端序）：
//   magic(4B "TLBC") + version(1B=1) + direction(1B) + topic_id(2B uint16) +
//   seq(8B uint64) + stamp(8B double) + payload_len(4B uint32) + payload
// 上行(direction=0)：topic_id 1=rgb 2=depth 3=odom；下行(direction=1)：topic_id 100
// ---------------------------------------------------------------------------
namespace {

// 帧头固定长度：4+1+1+2+8+8+4 = 28 字节
constexpr size_t kFrameHeaderLen = 28;
// 载荷最大长度保护（64MB），防止非法帧头导致缓冲区无限增长
constexpr uint32_t kMaxPayloadLen = 64u * 1024u * 1024u;
// 下行 EncodeMask 结果帧的 topic_id
constexpr uint16_t kDownTopicEncodeMask = 100;

// 主机字节序 uint64 -> 大端字节序（网络序）
inline uint64_t hostToBe64(uint64_t v) {
    return ((v & 0x00000000000000FFULL) << 56) |
           ((v & 0x000000000000FF00ULL) << 40) |
           ((v & 0x0000000000FF0000ULL) << 24) |
           ((v & 0x00000000FF000000ULL) << 8) |
           ((v & 0x000000FF00000000ULL) >> 8) |
           ((v & 0x0000FF0000000000ULL) >> 24) |
           ((v & 0x00FF000000000000ULL) >> 40) |
           ((v & 0xFF00000000000000ULL) >> 56);
}

// 大端字节序 -> 主机字节序 uint64
inline uint64_t be64ToHost(uint64_t v) { return hostToBe64(v); }

// 打包一帧：frame = 大端序帧头 + 原始载荷
bool buildFrame(uint8_t direction, uint16_t topic_id, uint64_t seq, double stamp,
                const uint8_t* payload, size_t payload_len,
                std::vector<uint8_t>& frame) {
    if (payload_len > kMaxPayloadLen) {
        return false;
    }
    frame.clear();
    frame.resize(kFrameHeaderLen + payload_len);
    uint8_t* p = frame.data();

    p[0] = 'T'; p[1] = 'L'; p[2] = 'B'; p[3] = 'C';  // magic
    p[4] = 1;                                        // version
    p[5] = direction;                                // direction: 0=上行 1=下行

    // topic_id(2B 大端)
    uint16_t tid_be = htons(topic_id);
    std::memcpy(p + 6, &tid_be, 2);

    // seq(8B 大端 uint64)
    uint64_t seq_be = hostToBe64(seq);
    std::memcpy(p + 8, &seq_be, 8);

    // stamp(8B 大端 double)：先按位拷贝到 uint64 再翻转字节序
    uint64_t stamp_bits = 0;
    std::memcpy(&stamp_bits, &stamp, sizeof(stamp));
    uint64_t stamp_be = hostToBe64(stamp_bits);
    std::memcpy(p + 16, &stamp_be, 8);

    // payload_len(4B 大端 uint32)
    uint32_t len_be = htonl((uint32_t)payload_len);
    std::memcpy(p + 24, &len_be, 4);

    if (payload_len > 0) {
        std::memcpy(p + kFrameHeaderLen, payload, payload_len);
    }
    return true;
}

// 尝试从接收缓冲区中取出一个完整下行帧。
// 成功返回 true 并填充各字段，consumed 表示本次消费的总字节数；
// 若缓冲区前部是垃圾数据，会向前扫描 magic "TLBC" 重新同步。
bool tryTakeFrame(std::vector<uint8_t>& buffer, size_t& consumed,
                  uint8_t& version, uint8_t& direction, uint16_t& topic_id,
                  uint64_t& seq, double& stamp, std::vector<uint8_t>& payload) {
    consumed = 0;
    if (buffer.empty()) {
        return false;
    }

    // 重新同步：在缓冲区中查找 magic
    size_t magic_pos = SIZE_MAX;
    for (size_t i = 0; i + 4 <= buffer.size(); ++i) {
        if (buffer[i] == 'T' && buffer[i + 1] == 'L' &&
            buffer[i + 2] == 'B' && buffer[i + 3] == 'C') {
            magic_pos = i;
            break;
        }
    }
    if (magic_pos == SIZE_MAX) {
        // 找不到任何 magic：超出帧头长度即整体丢弃，防止缓冲区无限增长
        if (buffer.size() > kFrameHeaderLen) {
            buffer.clear();
        }
        return false;
    }
    if (magic_pos > 0) {
        // 丢弃 magic 之前的垃圾字节后重试（递归，此时 magic 已在头部）
        buffer.erase(buffer.begin(), buffer.begin() + magic_pos);
        return tryTakeFrame(buffer, consumed, version, direction, topic_id,
                            seq, stamp, payload);
    }

    // 此时 buffer 以 "TLBC" 开头
    if (buffer.size() < kFrameHeaderLen) {
        return false;  // 帧头尚未收齐，等待更多数据
    }

    uint16_t tid_be = 0;
    std::memcpy(&tid_be, buffer.data() + 6, 2);
    topic_id = ntohs(tid_be);

    uint64_t seq_be = 0;
    std::memcpy(&seq_be, buffer.data() + 8, 8);
    seq = be64ToHost(seq_be);

    uint64_t stamp_be = 0;
    std::memcpy(&stamp_be, buffer.data() + 16, 8);
    uint64_t stamp_bits = be64ToHost(stamp_be);
    std::memcpy(&stamp, &stamp_bits, sizeof(stamp));

    uint32_t len_be = 0;
    std::memcpy(&len_be, buffer.data() + 24, 4);
    uint32_t payload_len = ntohl(len_be);
    if (payload_len > kMaxPayloadLen) {
        // 非法长度：丢弃当前 magic，继续重新同步
        buffer.erase(buffer.begin(), buffer.begin() + 4);
        return false;
    }
    if (buffer.size() < kFrameHeaderLen + payload_len) {
        return false;  // 帧未完整，等待更多数据
    }

    version = buffer[4];
    direction = buffer[5];
    payload.assign(buffer.data() + kFrameHeaderLen,
                   buffer.data() + kFrameHeaderLen + payload_len);
    consumed = kFrameHeaderLen + payload_len;
    return true;
}

// 将 Odometry 编码为上行 JSON 字符串（帧协议 topic_id=3 的载荷）
std::string encodeOdomJson(const nav_msgs::OdometryConstPtr& msg) {
    std::ostringstream oss;
    oss.precision(17);  // double 全精度，避免精度损失
    const geometry_msgs::Pose& pose = msg->pose.pose;
    oss << "{\"x\":" << pose.position.x << ",\"y\":" << pose.position.y
        << ",\"z\":" << pose.position.z << ",\"qx\":" << pose.orientation.x
        << ",\"qy\":" << pose.orientation.y << ",\"qz\":" << pose.orientation.z
        << ",\"qw\":" << pose.orientation.w << "}";
    return oss.str();
}

}  // namespace

// ---------------------------------------------------------------------------
// BroadcasterNode
// ---------------------------------------------------------------------------

BroadcasterNode::BroadcasterNode(ros::NodeHandle& nh) : nh_(nh) {
    // 私有参数在 launch 中通过 <rosparam command="load"> 载入，位于 ~ 命名空间
    ros::NodeHandle pnh("~");
    loadParams(pnh);
    setupSubscribers(nh_);
    setupPublishers(nh_);
    // 同步打包：enable 时建立 rgb/depth 同步订阅 + odom 独立订阅，
    // 此时 setupSubscribers 已跳过 out_topics 的独立订阅，避免双发
    if (sync_pack_.enable) {
        setupSyncPack(nh_);
    }

    // 有界发送队列：容量来自配置，队满时丢最旧帧
    frame_queue_ =
        std::make_shared<FrameQueue<std::vector<uint8_t>>>((size_t)queue_size_);

    // 启动发送 / 接收线程
    send_thread_ = std::thread(&BroadcasterNode::sendLoop, this);
    recv_thread_ = std::thread(&BroadcasterNode::recvLoop, this);
}

BroadcasterNode::~BroadcasterNode() {
    stop_flag_.store(true);  // 请求线程退出
    client_.disconnect();    // 断开连接，唤醒可能阻塞在 select 中的线程
    if (send_thread_.joinable()) {
        send_thread_.join();
    }
    if (recv_thread_.joinable()) {
        recv_thread_.join();
    }
}

void BroadcasterNode::loadParams(ros::NodeHandle& pnh) {
    // ---- 连接参数 ----
    pnh.param("connection/server_ip", server_ip_, std::string("127.0.0.1"));
    pnh.param("connection/server_port", server_port_, 9010);
    pnh.param("connection/reconnect_min_sec", reconnect_min_sec_, 1.0);
    pnh.param("connection/reconnect_max_sec", reconnect_max_sec_, 5.0);
    pnh.param("connection/send_timeout_ms", send_timeout_ms_, 100);
    pnh.param("connection/queue_size", queue_size_, 8);
    ROS_INFO("[Broadcaster] 外部服务 %s:%d, 重连退避 %.1f~%.1fs, 发送超时 %dms, 队列 %d",
             server_ip_.c_str(), server_port_, reconnect_min_sec_,
             reconnect_max_sec_, send_timeout_ms_, queue_size_);

    // ---- 同步打包（sync_pack）----
    // 参数需在 out_topics 解析之前读取，便于 enable 时跳过独立话题的发送配置
    pnh.param("sync_pack/enable", sync_pack_.enable, false);
    pnh.param("sync_pack/rgb_topic", sync_pack_.rgb_topic,
              std::string("/camera1/color_sync/image/compressed"));
    pnh.param("sync_pack/depth_topic", sync_pack_.depth_topic,
              std::string("/camera1/depth_sync/image/compressed"));
    pnh.param("sync_pack/odom_topic", sync_pack_.odom_topic,
              std::string("/unity_odom_sync"));
    pnh.param("sync_pack/rgb_depth_slop", sync_pack_.rgb_depth_slop, 0.05);
    pnh.param("sync_pack/odom_slop", sync_pack_.odom_slop, 0.1);
    pnh.param("sync_pack/max_freq", sync_pack_.max_freq, 10.0);
    // topic_id 用 int 中转（uint16_t 无 XmlRpcValue 直接转换），再收窄到 uint16_t
    int topic_id_int = 50;
    pnh.param("sync_pack/topic_id", topic_id_int, 50);
    sync_pack_.topic_id = (uint16_t)topic_id_int;
    if (sync_pack_.enable) {
        ROS_INFO("[Broadcaster] sync_pack 模式启用：rgb=%s depth=%s odom=%s "
                 "slop=%.3fs/%.3fs max_freq=%.1fHz topic_id=%u",
                 sync_pack_.rgb_topic.c_str(), sync_pack_.depth_topic.c_str(),
                 sync_pack_.odom_topic.c_str(), sync_pack_.rgb_depth_slop,
                 sync_pack_.odom_slop, sync_pack_.max_freq, sync_pack_.topic_id);
    }

    // ---- 检测展示图发布（下行 vis_b64 解码后的 jpeg 图）----
    // 双话题同发一份 jpeg 压缩图：/yoloe/plot 常规话题 +
    // /yoloe/plot/compressed（供 RViz Image display 选 compressed transport）。
    // 发布前按各话题订阅者数检查，无订阅者则跳过，省 CPU/带宽。
    pnh.param("plot_topic", plot_topic_, std::string("/yoloe/plot"));
    pnh.param("plot_compressed_topic", plot_compressed_topic_,
              std::string("/yoloe/plot/compressed"));
    ROS_INFO("[Broadcaster] 检测展示图双话题发布：%s 与 %s（按订阅者数检查后发布）",
             plot_topic_.c_str(), plot_compressed_topic_.c_str());

    // ---- 上行话题（broadcast_topics/out）----
    uint16_t auto_raw_id = 10;  // 未识别关键词的 raw 话题从 10 起自动编号
    if (pnh.hasParam("broadcast_topics/out")) {
        XmlRpc::XmlRpcValue out_cfg;
        pnh.getParam("broadcast_topics/out", out_cfg);
        if (out_cfg.getType() == XmlRpc::XmlRpcValue::TypeArray) {
            for (int i = 0; i < out_cfg.size(); ++i) {
                XmlRpc::XmlRpcValue& item = out_cfg[i];
                if (item.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
                    continue;
                }
                if (!item.hasMember("topic") || !item.hasMember("codec")) {
                    continue;
                }

                // sync_pack 模式下只发送组合帧，跳过 out_topics 中独立的
                // rgb/depth/odom 三条，避免同一数据双发
                if (sync_pack_.enable) {
                    ROS_INFO("[Broadcaster] sync_pack 模式，跳过独立话题发送: %s (codec=%s)",
                             ((std::string)item["topic"]).c_str(),
                             ((std::string)item["codec"]).c_str());
                    continue;
                }

                OutTopicCfg cfg;
                cfg.topic = (std::string)item["topic"];
                cfg.codec = (std::string)item["codec"];
                // 类型安全读取 max_freq：YAML 中整数会存为 XmlRpcValue::TypeInt，
                // 直接 (double) 转换会抛 XmlRpcException，需要按类型分支处理
                if (item.hasMember("max_freq")) {
                    XmlRpc::XmlRpcValue& mf = item["max_freq"];
                    if (mf.getType() == XmlRpc::XmlRpcValue::TypeInt) {
                        cfg.max_freq = (double)(int)mf;
                    } else if (mf.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
                        cfg.max_freq = (double)mf;
                    } else {
                        cfg.max_freq = 0.0;
                    }
                } else {
                    cfg.max_freq = 0.0;
                }
                cfg.last_sent = ros::Time(0);
                cfg.seq = 0;

                // topic_id 确定：优先取显式配置，否则按 codec/话题名自动推导
                // （YAML 整数存为 TypeInt，(int) 转换前先判断类型避免 XmlRpcException）
                if (item.hasMember("topic_id")) {
                    XmlRpc::XmlRpcValue& tid = item["topic_id"];
                    if (tid.getType() == XmlRpc::XmlRpcValue::TypeInt) {
                        cfg.topic_id = (uint16_t)(int)tid;
                    } else if (tid.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
                        cfg.topic_id = (uint16_t)(int)(double)tid;
                    } else {
                        cfg.topic_id = auto_raw_id++;
                    }
                } else if (cfg.codec == "json") {
                    cfg.topic_id = 3;  // odom 固定为 3
                } else {
                    std::string lower = cfg.topic;
                    std::transform(lower.begin(), lower.end(), lower.begin(),
                                   ::tolower);
                    if (lower.find("depth") != std::string::npos) {
                        cfg.topic_id = 2;  // depth 压缩图
                    } else if (lower.find("color") != std::string::npos ||
                               lower.find("rgb") != std::string::npos) {
                        cfg.topic_id = 1;  // rgb 压缩图
                    } else {
                        cfg.topic_id = auto_raw_id++;  // 其它 raw 话题自动编号
                    }
                }
                out_topics_.push_back(cfg);
                ROS_INFO("[Broadcaster] 上行话题 %s codec=%s max_freq=%.1fHz topic_id=%u",
                         cfg.topic.c_str(), cfg.codec.c_str(), cfg.max_freq,
                         cfg.topic_id);
            }
        }
    }

    // ---- 下行话题（broadcast_topics/in）----
    if (pnh.hasParam("broadcast_topics/in")) {
        XmlRpc::XmlRpcValue in_cfg;
        pnh.getParam("broadcast_topics/in", in_cfg);
        if (in_cfg.getType() == XmlRpc::XmlRpcValue::TypeArray) {
            for (int i = 0; i < in_cfg.size(); ++i) {
                XmlRpc::XmlRpcValue& item = in_cfg[i];
                if (item.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
                    continue;
                }
                if (!item.hasMember("topic") || !item.hasMember("codec")) {
                    continue;
                }
                InTopicCfg cfg;
                cfg.topic = (std::string)item["topic"];
                cfg.codec = (std::string)item["codec"];
                in_topics_.push_back(cfg);
                ROS_INFO("[Broadcaster] 下行话题 %s codec=%s", cfg.topic.c_str(),
                         cfg.codec.c_str());
            }
        }
    }
}

void BroadcasterNode::setupSubscribers(ros::NodeHandle& nh) {
    for (size_t i = 0; i < out_topics_.size(); ++i) {
        const OutTopicCfg& cfg = out_topics_[i];
        if (cfg.codec == "json") {
            // json 编解码：订阅 nav_msgs::Odometry
            subs_.push_back(nh.subscribe<nav_msgs::Odometry>(
                cfg.topic, 1, boost::bind(&BroadcasterNode::onOdom, this, _1, i)));
        } else {
            // raw 编解码：订阅 sensor_msgs::CompressedImage（默认分支）
            subs_.push_back(nh.subscribe<sensor_msgs::CompressedImage>(
                cfg.topic, 1, boost::bind(&BroadcasterNode::onImage, this, _1, i)));
        }
    }
}

void BroadcasterNode::setupPublishers(ros::NodeHandle& nh) {
    for (const InTopicCfg& cfg : in_topics_) {
        result_pubs_.push_back(nh.advertise<scene_graph::EncodeMask>(cfg.topic, 1));
    }
    // 检测展示图发布器：下行 vis_b64 解码后的 jpeg 压缩图，双话题各一个
    plot_pub_ = nh.advertise<sensor_msgs::CompressedImage>(plot_topic_, 2);
    plot_compressed_pub_ =
        nh.advertise<sensor_msgs::CompressedImage>(plot_compressed_topic_, 2);
}

// ---- 上行回调 ----

void BroadcasterNode::onImage(const sensor_msgs::CompressedImageConstPtr& msg,
                              size_t idx) {
    if (idx >= out_topics_.size()) {
        return;
    }
    OutTopicCfg& cfg = out_topics_[idx];

    // max_freq 节流：未到发送间隔则丢弃本帧
    ros::Time now = ros::Time::now();
    if (cfg.max_freq > 0.0) {
        if (cfg.last_sent.isValid() &&
            (now - cfg.last_sent).toSec() < 1.0 / cfg.max_freq) {
            return;
        }
        cfg.last_sent = now;
    }

    // 打包帧头 + CompressedImage.data 原字节（不解码不重编码）
    std::vector<uint8_t> frame;
    if (!buildFrame(0, cfg.topic_id, cfg.seq++, msg->header.stamp.toSec(),
                    msg->data.data(), msg->data.size(), frame)) {
        return;
    }
    if (frame_queue_->push(std::move(frame))) {
        ROS_WARN_THROTTLE(5.0, "[Broadcaster] 发送队列已满，丢弃最旧帧(topic=%s)",
                          cfg.topic.c_str());
    }
}

void BroadcasterNode::onOdom(const nav_msgs::OdometryConstPtr& msg, size_t idx) {
    if (idx >= out_topics_.size()) {
        return;
    }
    OutTopicCfg& cfg = out_topics_[idx];

    // max_freq 节流：未到发送间隔则丢弃本帧
    ros::Time now = ros::Time::now();
    if (cfg.max_freq > 0.0) {
        if (cfg.last_sent.isValid() &&
            (now - cfg.last_sent).toSec() < 1.0 / cfg.max_freq) {
            return;
        }
        cfg.last_sent = now;
    }

    // 序列化 odom 为 JSON 载荷后打包
    std::string json_str = encodeOdomJson(msg);
    std::vector<uint8_t> frame;
    if (!buildFrame(0, cfg.topic_id, cfg.seq++, msg->header.stamp.toSec(),
                    reinterpret_cast<const uint8_t*>(json_str.data()),
                    json_str.size(), frame)) {
        return;
    }
    if (frame_queue_->push(std::move(frame))) {
        ROS_WARN_THROTTLE(5.0, "[Broadcaster] 发送队列已满，丢弃最旧帧(topic=%s)",
                          cfg.topic.c_str());
    }
}

// ---- 同步打包（sync_pack）----

void BroadcasterNode::setupSyncPack(ros::NodeHandle& nh) {
    // rgb/depth 通过 message_filters 近似时间同步强绑（slop 映射为
    // ApproximateTime 的 maxIntervalDuration，即两消息时间差上限）
    sync_rgb_sub_ =
        std::make_unique<message_filters::Subscriber<sensor_msgs::CompressedImage>>(
            nh, sync_pack_.rgb_topic, 20, ros::TransportHints().tcpNoDelay());
    sync_depth_sub_ =
        std::make_unique<message_filters::Subscriber<sensor_msgs::CompressedImage>>(
            nh, sync_pack_.depth_topic, 20, ros::TransportHints().tcpNoDelay());

    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::CompressedImage, sensor_msgs::CompressedImage>
        SyncPackPolicy;
    sync_sync_ = std::make_unique<message_filters::Synchronizer<SyncPackPolicy>>(
        SyncPackPolicy(20), *sync_rgb_sub_, *sync_depth_sub_);
    // ApproximateTime 无构造参数 slop，用 setMaxIntervalDuration 表达容差
    sync_sync_->setMaxIntervalDuration(ros::Duration(sync_pack_.rgb_depth_slop));
    sync_sync_->registerCallback(
        boost::bind(&BroadcasterNode::onSyncRgbDepth, this, _1, _2));

    // odom 独立订阅：仅缓存最近值（JSON），供组合帧取用，不参与时间同步
    extra_subs_.push_back(nh.subscribe<nav_msgs::Odometry>(
        sync_pack_.odom_topic, 1,
        boost::bind(&BroadcasterNode::onSyncOdom, this, _1)));
    ROS_INFO("[Broadcaster] sync_pack 订阅建立：rgb/depth 同步(queue=20, slop=%.3fs)，odom=%s",
             sync_pack_.rgb_depth_slop, sync_pack_.odom_topic.c_str());
}

void BroadcasterNode::onSyncOdom(const nav_msgs::OdometryConstPtr& msg) {
    // 序列化并缓存最近 odom（组合帧在 buildSyncPackFrame 中取用）
    std::string json_str = encodeOdomJson(msg);
    std::lock_guard<std::mutex> lock(odom_mutex_);
    latest_odom_json_ = std::move(json_str);
    latest_odom_stamp_ = msg->header.stamp;
}

void BroadcasterNode::onSyncRgbDepth(
    const sensor_msgs::CompressedImageConstPtr& rgb,
    const sensor_msgs::CompressedImageConstPtr& depth) {
    // max_freq 节流：未到发送间隔则丢弃本帧（用 sync_pack 自己的 last_sent 记录）
    ros::Time now = ros::Time::now();
    if (sync_pack_.max_freq > 0.0) {
        if (sync_pack_last_sent_.isValid() &&
            (now - sync_pack_last_sent_).toSec() < 1.0 / sync_pack_.max_freq) {
            return;
        }
        sync_pack_last_sent_ = now;
    }

    // 组装组合帧（stamp 用 rgb 的时间戳），打包失败（如 odom 尚未到达）则丢帧
    std::vector<uint8_t> frame;
    if (!buildSyncPackFrame(rgb, depth, rgb->header.stamp.toSec(), frame)) {
        return;
    }
    if (frame_queue_->push(std::move(frame))) {
        ROS_WARN_THROTTLE(5.0, "[Broadcaster] 发送队列已满，丢弃最旧帧(sync_pack topic=%u)",
                          sync_pack_.topic_id);
    }
}

bool BroadcasterNode::buildSyncPackFrame(
    const sensor_msgs::CompressedImageConstPtr& rgb,
    const sensor_msgs::CompressedImageConstPtr& depth, double stamp,
    std::vector<uint8_t>& frame) {
    // 取最近 odom（同步打包允许 odom 相对 rgb/depth 有一定延迟）
    std::string odom_json;
    ros::Time odom_stamp;
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        if (latest_odom_json_.empty()) {
            // odom 尚未到达：放弃本帧，等待 odom 先到
            return false;
        }
        odom_json = latest_odom_json_;
        odom_stamp = latest_odom_stamp_;
    }
    // odom 过期告警（仅提示，仍取最近值打包，符合"允许延迟取最近值"的语义）
    double odom_delay = stamp - odom_stamp.toSec();
    if (odom_delay > sync_pack_.odom_slop) {
        ROS_WARN_THROTTLE(2.0,
                          "[Broadcaster] sync_pack odom 延迟 %.3fs 超过阈值 %.3fs，仍使用最近值",
                          odom_delay, sync_pack_.odom_slop);
    }

    // ---- 组装 payload（全部大端）----
    // 结构：u32 rgb_len + rgb_bytes + u32 depth_len + depth_bytes +
    //       u32 odom_len + odom_json(UTF-8)
    // Python 端用 struct.unpack(">III", ...) 依次解三个长度再切分
    size_t payload_len = 4 + rgb->data.size() + 4 + depth->data.size() + 4 +
                         odom_json.size();
    std::vector<uint8_t> payload;
    payload.reserve(payload_len);

    // 局部 lambda：追加一段 [长度(4B 大端) + 数据] 的字节
    auto append_segment = [&payload](const uint8_t* data, size_t len) {
        uint32_t len_be = htonl((uint32_t)len);
        payload.insert(payload.end(), reinterpret_cast<const uint8_t*>(&len_be),
                       reinterpret_cast<const uint8_t*>(&len_be) + 4);
        if (len > 0) {
            payload.insert(payload.end(), data, data + len);
        }
    };
    append_segment(rgb->data.data(), rgb->data.size());
    append_segment(depth->data.data(), depth->data.size());
    append_segment(reinterpret_cast<const uint8_t*>(odom_json.data()),
                   odom_json.size());

    // 打包：direction=0 上行，topic_id=50 组合帧，stamp=rgb 时间戳
    return buildFrame(0, sync_pack_.topic_id, sync_pack_seq_++, stamp,
                      payload.data(), payload.size(), frame);
}

// ---- 下行处理 ----

void BroadcasterNode::handleEncodeMask(const std::vector<uint8_t>& payload,
                                       double /*stamp*/) {
    // 帧头 stamp 与 JSON 内 stamp 通常一致；以 JSON 内为准（codec 负责解析）
    std::string json_str(payload.begin(), payload.end());

    // ---- 可选字段 vis_b64：检测展示图（jpeg base64）----
    // 非抛异常模式先解析一次 JSON 取 vis_b64；解析失败或字段缺失则
    // 跳过展示图处理，不影响下方 encodemask 主流程。
    // （decodeEncodeMaskJson 会再解析一次 JSON，为保持最小改动不合并两处解析）
    nlohmann::json j = nlohmann::json::parse(json_str, nullptr, false);
    if (!j.is_discarded() && j.is_object() && j.contains("vis_b64") &&
        j["vis_b64"].is_string() && !j["vis_b64"].get<std::string>().empty()) {
        publishPlotImage(j["vis_b64"].get<std::string>());
    }

    scene_graph::EncodeMask msg;
    if (!EncodeMaskCodec::decodeEncodeMaskJson(json_str, msg)) {
        return;
    }
    // 发布到所有下行话题
    for (ros::Publisher& pub : result_pubs_) {
        pub.publish(msg);
    }
}

void BroadcasterNode::publishPlotImage(const std::string& vis_b64) {
    // 复用 EncodeMaskCodec::base64Decode（宽松容错，非法字符跳过，不抛异常）
    std::vector<uint8_t> decoded = EncodeMaskCodec::base64Decode(vis_b64);
    if (decoded.empty()) {
        ROS_WARN_THROTTLE(5.0,
                          "[Broadcaster] vis_b64 base64 解码结果为空，跳过展示图发布");
        return;
    }

    // 组装 jpeg 压缩图消息（双话题共享同一份数据）
    sensor_msgs::CompressedImage msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world";
    msg.format = "jpeg";
    msg.data.assign(decoded.begin(), decoded.end());

    // 订阅者检查后分别发布：对应话题无订阅者则不发布，省 CPU/带宽
    int plot_subs = plot_pub_.getNumSubscribers();
    int plot_compressed_subs = plot_compressed_pub_.getNumSubscribers();
    if (plot_subs == 0 && plot_compressed_subs == 0) {
        ROS_DEBUG_THROTTLE(2.0, "[Broadcaster] %s 与 %s 均无订阅者，跳过展示图发布",
                           plot_topic_.c_str(), plot_compressed_topic_.c_str());
        return;
    }
    if (plot_subs > 0) {
        plot_pub_.publish(msg);  // 发布到 /yoloe/plot
    }
    if (plot_compressed_subs > 0) {
        plot_compressed_pub_.publish(msg);  // 发布到 /yoloe/plot/compressed
    }
    ROS_DEBUG_THROTTLE(2.0, "[Broadcaster] 已发布检测展示图(%zu 字节)：%s=%d 订阅者, %s=%d 订阅者",
                       decoded.size(), plot_topic_.c_str(), plot_subs,
                       plot_compressed_topic_.c_str(), plot_compressed_subs);
}

// ---- 发送线程 ----

void BroadcasterNode::sendLoop() {
    int consecutive_failures = 0;          // 连续发送失败计数
    double backoff = reconnect_min_sec_;   // 当前退避时间（指数递增，封顶）
    std::vector<uint8_t> frame;

    while (!stop_flag_.load()) {
        if (!client_.connected()) {
            // ---- 未连接：非阻塞尝试连接，失败则退避重试 ----
            if (client_.connect(server_ip_, server_port_, 500)) {
                ROS_INFO("[Broadcaster] 已连接外部服务 %s:%d", server_ip_.c_str(),
                         server_port_);
                backoff = reconnect_min_sec_;  // 连接成功，重置退避
                consecutive_failures = 0;
            } else {
                ROS_WARN_THROTTLE(5.0, "[Broadcaster] 连接 %s:%d 失败，%.1fs 后重试",
                                  server_ip_.c_str(), server_port_, backoff);
                std::this_thread::sleep_for(
                    std::chrono::milliseconds((int)(backoff * 1000.0)));
                backoff = std::min(backoff * 2.0, reconnect_max_sec_);
            }
            continue;
        }

        // ---- 已连接：从队列取帧发送（最多等待 50ms） ----
        if (!frame_queue_->pop(frame, 50.0)) {
            continue;  // 队列空，回到循环顶部（保持连接存活）
        }
        if (client_.send(frame.data(), frame.size(), send_timeout_ms_)) {
            consecutive_failures = 0;
        } else {
            consecutive_failures++;
            ROS_WARN_THROTTLE(2.0, "[Broadcaster] 发送失败(连续 %d 次)",
                              consecutive_failures);
            // 连续失败超过 3 次主动断开，触发重连（外部服务可能已重启）
            if (consecutive_failures >= 3) {
                ROS_WARN("[Broadcaster] 连续失败 %d 次，主动断开以触发重连",
                         consecutive_failures);
                client_.disconnect();
                consecutive_failures = 0;
            }
        }
    }
}

// ---- 接收线程 ----

void BroadcasterNode::recvLoop() {
    std::vector<uint8_t> buffer;  // 累积接收缓冲区，跨 recv 保留未完整帧
    buffer.reserve(4 * 1024 * 1024);

    while (!stop_flag_.load()) {
        if (!client_.connected()) {
            // 未连接：清空残留数据，等待发送线程完成重连
            buffer.clear();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        uint8_t chunk[65536];
        size_t got = 0;
        if (!client_.recvSome(chunk, sizeof(chunk), got, 100)) {
            // 超时无数据是正常情况；仅当对端关闭时才断开并清空缓冲
            if (client_.peerClosed()) {
                ROS_WARN("[Broadcaster] 对端关闭连接，等待重连");
                client_.disconnect();
                buffer.clear();
            }
            continue;
        }
        buffer.insert(buffer.end(), chunk, chunk + got);

        // 循环切帧：一次可能收到多帧，全部处理完
        while (true) {
            size_t consumed = 0;
            uint8_t version = 0;
            uint8_t direction = 0;
            uint16_t topic_id = 0;
            uint64_t seq = 0;
            double stamp = 0.0;
            std::vector<uint8_t> payload;
            if (!tryTakeFrame(buffer, consumed, version, direction, topic_id,
                              seq, stamp, payload)) {
                break;  // 数据不足或需重新同步，等待更多数据
            }
            buffer.erase(buffer.begin(), buffer.begin() + consumed);

            // 仅处理下行(direction==1)的 EncodeMask 结果帧(topic_id==100)
            if (direction == 1 && topic_id == kDownTopicEncodeMask) {
                handleEncodeMask(payload, stamp);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// main：入口
// ---------------------------------------------------------------------------

int main(int argc, char** argv) {
    ros::init(argc, argv, "topic_broadcaster");
    ros::NodeHandle nh;

    BroadcasterNode node(nh);

    // 2 线程异步 spinner 处理订阅回调；业务逻辑不包 try-catch，错误自然暴露
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}

#include "topic_broadcaster/encodemask_codec.h"

#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nlohmann/json.hpp>
#include <scene_graph/EncodeMask.h>
#include <scene_graph/WordVector.h>
#include <sensor_msgs/CompressedImage.h>

// ---------------------------------------------------------------------------
// base64 编解码（手写实现，无额外依赖）。
// 解码时非法字符跳过、'=' 与空白忽略，不抛异常、宽松容错；
// 编码为标准 RFC 4648 字母表 + '=' 填充。
// ---------------------------------------------------------------------------
namespace {

// 标准 base64 编码字母表
const char* kBase64Table =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

// 查表：返回字符对应的 6bit 数值，非法字符返回 -1
int base64Value(char c) {
    if (c >= 'A' && c <= 'Z') return c - 'A';
    if (c >= 'a' && c <= 'z') return c - 'a' + 26;
    if (c >= '0' && c <= '9') return c - '0' + 52;
    if (c == '+') return 62;
    if (c == '/') return 63;
    return -1;
}

// 从 JSON 数组中安全取第 idx 个元素为 double：
// 下标越界或元素不是数字时返回默认值，不抛异常（避免网络数据触发异常）
double jsonArrayAt(const nlohmann::json& arr, size_t idx, double default_val) {
    if (idx < arr.size() && arr[idx].is_number()) {
        return arr[idx].get<double>();
    }
    return default_val;
}

}  // namespace

std::vector<uint8_t> EncodeMaskCodec::base64Decode(const std::string& b64) {
    std::vector<uint8_t> out;
    out.reserve(b64.size() * 3 / 4);
    uint32_t acc = 0;
    int bits = 0;
    for (char c : b64) {
        if (c == '=' || c == '\n' || c == '\r' || c == ' ' || c == '\t') {
            continue;  // 忽略填充与空白
        }
        int v = base64Value(c);
        if (v < 0) {
            continue;  // 非法字符宽松跳过
        }
        acc = (acc << 6) | (uint32_t)v;
        bits += 6;
        if (bits >= 8) {
            bits -= 8;
            out.push_back((uint8_t)((acc >> bits) & 0xFF));
        }
    }
    return out;
}

std::string EncodeMaskCodec::base64Encode(const std::vector<uint8_t>& data) {
    std::string out;
    out.reserve(((data.size() + 2) / 3) * 4);
    size_t i = 0;
    // 每 3 字节 -> 4 个 base64 字符
    while (i + 3 <= data.size()) {
        uint32_t v = ((uint32_t)data[i] << 16) | ((uint32_t)data[i + 1] << 8) |
                     (uint32_t)data[i + 2];
        out.push_back(kBase64Table[(v >> 18) & 0x3F]);
        out.push_back(kBase64Table[(v >> 12) & 0x3F]);
        out.push_back(kBase64Table[(v >> 6) & 0x3F]);
        out.push_back(kBase64Table[v & 0x3F]);
        i += 3;
    }
    // 尾部剩余 1/2 字节，带 '=' 填充
    size_t rem = data.size() - i;
    if (rem == 1) {
        uint32_t v = (uint32_t)data[i] << 16;
        out.push_back(kBase64Table[(v >> 18) & 0x3F]);
        out.push_back(kBase64Table[(v >> 12) & 0x3F]);
        out.push_back('=');
        out.push_back('=');
    } else if (rem == 2) {
        uint32_t v = ((uint32_t)data[i] << 16) | ((uint32_t)data[i + 1] << 8);
        out.push_back(kBase64Table[(v >> 18) & 0x3F]);
        out.push_back(kBase64Table[(v >> 12) & 0x3F]);
        out.push_back(kBase64Table[(v >> 6) & 0x3F]);
        out.push_back('=');
    }
    return out;
}

namespace {

// null 安全的字段取值：nlohmann 的 value() 仅在 key 缺失时返回默认值，
// key 存在但值为 null 时会抛 type_error.302（Python 端 None 即序列化为 null）。
// 这里显式判类型，任何非期望类型都退回默认值，保证解析不抛异常。
std::string jsonStringOr(const nlohmann::json& j, const char* key,
                         const std::string& fallback = std::string()) {
    auto it = j.find(key);
    if (it == j.end() || !it->is_string()) {
        return fallback;
    }
    return it->get<std::string>();
}

double jsonNumberOr(const nlohmann::json& j, const char* key, double fallback) {
    auto it = j.find(key);
    if (it == j.end() || !it->is_number()) {
        return fallback;
    }
    return it->get<double>();
}

}  // namespace

bool EncodeMaskCodec::decodeEncodeMaskJson(const std::string& json_str,
                                           scene_graph::EncodeMask& out) {
    // 非抛异常模式解析：解析失败时返回空 json()（null），随后用 is_object 判断，
    // 避免因外部服务下发非法 JSON 而抛出异常
    nlohmann::json j = nlohmann::json::parse(json_str, nullptr, false);
    if (!j.is_object()) {
        ROS_ERROR("[Broadcaster] EncodeMask JSON 解析失败：不是合法 JSON 对象");
        return false;
    }

    // ---- stamp：秒转 ros::Time ----
    double stamp = jsonNumberOr(j, "stamp", 0.0);
    ros::Time t(stamp);
    out.header.stamp = t;
    out.header.frame_id = "world";

    // ---- current_odom：odom 数组 [x, y, z, qx, qy, qz, qw] ----
    out.current_odom.header.stamp = t;
    out.current_odom.header.frame_id = "world";
    if (j.contains("odom") && j["odom"].is_array() && j["odom"].size() >= 7) {
        out.current_odom.pose.pose.position.x = jsonArrayAt(j["odom"], 0, 0.0);
        out.current_odom.pose.pose.position.y = jsonArrayAt(j["odom"], 1, 0.0);
        out.current_odom.pose.pose.position.z = jsonArrayAt(j["odom"], 2, 0.0);
        out.current_odom.pose.pose.orientation.x = jsonArrayAt(j["odom"], 3, 0.0);
        out.current_odom.pose.pose.orientation.y = jsonArrayAt(j["odom"], 4, 0.0);
        out.current_odom.pose.pose.orientation.z = jsonArrayAt(j["odom"], 5, 0.0);
        out.current_odom.pose.pose.orientation.w = jsonArrayAt(j["odom"], 6, 0.0);
    }

    // ---- current_rgb：jpeg 压缩图 ----
    out.current_rgb.header.stamp = t;
    out.current_rgb.header.frame_id = "world";
    out.current_rgb.format = "jpeg";
    {
        std::vector<uint8_t> rgb_data = base64Decode(jsonStringOr(j, "rgb_b64"));
        out.current_rgb.data.assign(rgb_data.begin(), rgb_data.end());
    }

    // ---- current_depth：png 压缩图 ----
    out.current_depth.header.stamp = t;
    out.current_depth.header.frame_id = "world";
    out.current_depth.format = "png";
    {
        std::vector<uint8_t> depth_data = base64Decode(jsonStringOr(j, "depth_b64"));
        out.current_depth.data.assign(depth_data.begin(), depth_data.end());
    }

    // ---- objects：labels / confs / masks / word_vectors ----
    if (j.contains("objects") && j["objects"].is_array()) {
        for (const nlohmann::json& obj : j["objects"]) {
            if (!obj.is_object()) {
                continue;  // 单个对象格式非法则跳过
            }
            out.labels.push_back(jsonStringOr(obj, "label"));
            out.confs.push_back(jsonNumberOr(obj, "conf", 0.0));

            // mask：png 压缩图
            sensor_msgs::CompressedImage mask;
            mask.header.stamp = t;
            mask.header.frame_id = "world";
            mask.format = "png";
            {
                std::vector<uint8_t> mask_data = base64Decode(jsonStringOr(obj, "mask_b64"));
                // 防御：mask_b64 为空时填充 1×1 单像素占位图（全零灰度 PNG），避免下游 imdecode 崩溃
                if (mask_data.empty()) {
                    // cv2.imencode 生成的最小合法 1×1 灰度 PNG（70 字节）
                    static const uint8_t kPlaceholder[] = {
                        0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A,
                        0x00, 0x00, 0x00, 0x0D, 0x49, 0x48, 0x44, 0x52,
                        0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01,
                        0x08, 0x00, 0x00, 0x00, 0x00, 0x3A, 0x7E, 0x9B,
                        0x55, 0x00, 0x00, 0x00, 0x0D, 0x49, 0x44, 0x41,
                        0x54, 0x08, 0x1D, 0x01, 0x02, 0x00, 0xFD, 0xFF,
                        0x00, 0x00, 0x00, 0x02, 0x00, 0x01, 0xCD, 0xE3,
                        0xD1, 0x2B, 0x00, 0x00, 0x00, 0x00, 0x49, 0x45,
                        0x4E, 0x44, 0xAE, 0x42, 0x60, 0x82
                    };
                    mask_data.assign(kPlaceholder, kPlaceholder + sizeof(kPlaceholder));
                }
                mask.data.assign(mask_data.begin(), mask_data.end());
            }
            out.masks.push_back(mask);

            // word_vector：512 维浮点向量
            scene_graph::WordVector wv;
            if (obj.contains("word_vector") && obj["word_vector"].is_array()) {
                size_t n = std::min((size_t)512, obj["word_vector"].size());
                for (size_t k = 0; k < n; ++k) {
                    wv.word_vector[k] = jsonArrayAt(obj["word_vector"], k, 0.0);
                }
            }
            out.word_vectors.push_back(wv);
        }
    }

    return true;
}

bool EncodeMaskCodec::encodeEncodeMaskJson(const scene_graph::EncodeMask& in,
                                           std::string& json_str) {
    // 预留接口：未来需要把本机 EncodeMask 结果上行到其它服务时使用
    nlohmann::json j;
    j["stamp"] = in.header.stamp.toSec();
    j["odom"] = {in.current_odom.pose.pose.position.x,
                 in.current_odom.pose.pose.position.y,
                 in.current_odom.pose.pose.position.z,
                 in.current_odom.pose.pose.orientation.x,
                 in.current_odom.pose.pose.orientation.y,
                 in.current_odom.pose.pose.orientation.z,
                 in.current_odom.pose.pose.orientation.w};
    j["rgb_b64"] = base64Encode(in.current_rgb.data);
    j["depth_b64"] = base64Encode(in.current_depth.data);

    nlohmann::json objs = nlohmann::json::array();
    for (size_t i = 0; i < in.labels.size(); ++i) {
        nlohmann::json obj;
        obj["label"] = in.labels[i];
        obj["conf"] = (i < in.confs.size()) ? in.confs[i] : 0.0;
        if (i < in.masks.size()) {
            obj["mask_b64"] = base64Encode(in.masks[i].data);
        } else {
            obj["mask_b64"] = "";
        }
        std::vector<double> wv(512, 0.0);
        if (i < in.word_vectors.size()) {
            for (size_t k = 0; k < 512; ++k) {
                wv[k] = in.word_vectors[i].word_vector[k];
            }
        }
        obj["word_vector"] = wv;
        objs.push_back(obj);
    }
    j["objects"] = objs;
    json_str = j.dump();
    return true;
}

#ifndef TOPIC_BROADCASTER_ENCODEMASK_CODEC_H
#define TOPIC_BROADCASTER_ENCODEMASK_CODEC_H

#include <cstdint>
#include <string>
#include <vector>

// 直接包含 scene_graph 包生成的消息头（rosmsg 生成类型为模板类的 typedef，
// 不能前置声明，只能显式包含）
#include <scene_graph/EncodeMask.h>

// EncodeMask 消息 <-> JSON 编解码工具。
//
// 职责1（编码，预留接口，暂未使用）：将 scene_graph::EncodeMask 序列化为 JSON，
//   未来如需把本机检测结果上行到其它服务时可启用。
// 职责2（解码，核心）：解析外部服务下行的检测结果 JSON，重建
//   scene_graph::EncodeMask 消息并发布回 ROS。
//
// JSON 解析使用 nlohmann/json（系统已安装 /usr/include/nlohmann），
// base64 编解码为手写实现（无额外依赖，不抛异常，非法输入宽松处理）。
class EncodeMaskCodec {
public:
    // 解码：把下行 JSON 字符串解析并填充 out（scene_graph::EncodeMask）。
    // 成功返回 true；JSON 非法或字段缺失时返回 false（不抛异常）。
    static bool decodeEncodeMaskJson(const std::string& json_str,
                                     scene_graph::EncodeMask& out);

    // 编码：把 EncodeMask 消息序列化为 JSON 字符串（预留接口，暂未使用）。
    static bool encodeEncodeMaskJson(const scene_graph::EncodeMask& in,
                                     std::string& json_str);

private:
    // base64 解码：手写实现，非法字符跳过、'=' 与空白忽略，不抛异常。
    static std::vector<uint8_t> base64Decode(const std::string& b64);

    // base64 编码：标准 RFC 4648 表，带 '=' 填充。
    static std::string base64Encode(const std::vector<uint8_t>& data);
};

#endif  // TOPIC_BROADCASTER_ENCODEMASK_CODEC_H

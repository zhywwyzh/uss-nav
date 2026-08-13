#ifndef TOPIC_BROADCASTER_TCP_CLIENT_H
#define TOPIC_BROADCASTER_TCP_CLIENT_H

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>

// 非阻塞 TCP 客户端：
//  - 主动连接外部服务（外部服务为 TCP server 角色）；
//  - 所有操作均带超时上限，不阻塞、不抛异常，统一以 bool 返回成功与否；
//  - 外部服务未启动时 connect 会快速失败返回，由上层负责退避重连，
//    因此本类保证调用方任何时刻都不会被卡死。
class TcpClient {
public:
    TcpClient();
    ~TcpClient();

    // 非阻塞连接：将 socket 置为 O_NONBLOCK 后发起 connect，
    // 用 select 等待可写以判断连接结果，超过 timeout_ms 毫秒未连上则返回 false。
    // 失败时内部会关闭 fd 并复位为未连接状态。
    bool connect(const std::string& ip, int port, int timeout_ms = 500);

    // 断开连接：关闭 fd 并复位状态，可重复调用（幂等）。
    void disconnect();

    // 当前是否处于已连接状态。
    bool connected() const;

    // 非阻塞发送整段数据：发送缓冲满或超时返回 false（不抛异常不阻塞）。
    // 内部按总体截止时间控制，整个 send 过程阻塞不超过 timeout_ms 毫秒。
    bool send(const uint8_t* data, size_t len, int timeout_ms = 100);

    // 非阻塞读取一段数据：等待至多 timeout_ms 毫秒，有数据返回 true 并给出
    // 实际字节数 got；超时/出错返回 false。对端关闭连接时置位 peerClosed()。
    bool recvSome(uint8_t* buf, size_t max_len, size_t& got, int timeout_ms = 100);

    // 对端是否已关闭连接（recv 返回 0 时置位，connect/disconnect 时复位）。
    bool peerClosed() const;

private:
    int fd_;                        // socket 文件描述符，未连接时为 -1
    std::atomic<bool> connected_;   // 连接状态标志
    std::atomic<bool> peer_closed_; // 对端关闭标志
};

#endif  // TOPIC_BROADCASTER_TCP_CLIENT_H

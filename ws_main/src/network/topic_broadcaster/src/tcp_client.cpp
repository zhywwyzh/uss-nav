#include "topic_broadcaster/tcp_client.h"

#include <arpa/inet.h>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

// ---------------------------------------------------------------------------
// 非阻塞 TCP 客户端实现。
// 所有操作均带超时/失败返回，不抛异常；socket 始终保持 O_NONBLOCK，
// 配合 select 实现带超时的等待，保证任何情况下调用方都不会被卡死。
// ---------------------------------------------------------------------------

TcpClient::TcpClient() : fd_(-1), connected_(false), peer_closed_(false) {}

TcpClient::~TcpClient() { disconnect(); }

bool TcpClient::connect(const std::string& ip, int port, int timeout_ms) {
    disconnect();  // 从干净状态开始（幂等）

    // 仅支持点分十进制 IPv4 地址
    struct sockaddr_in addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons((uint16_t)port);
    if (inet_pton(AF_INET, ip.c_str(), &addr.sin_addr) != 1) {
        return false;  // 非 IPv4 地址（如主机名）不支持，直接失败
    }

    // 创建 socket 并置为非阻塞
    int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
        return false;
    }
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags < 0 || fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) {
        ::close(fd);
        return false;
    }

    // 非阻塞发起连接：成功立即返回 0，否则一般为 EINPROGRESS
    int ret = ::connect(fd, (struct sockaddr*)&addr, sizeof(addr));
    if (ret != 0) {
        if (errno == EINPROGRESS) {
            // 连接进行中：用 select 等待可写，超时未完成视为失败
            fd_set wfds;
            FD_ZERO(&wfds);
            FD_SET(fd, &wfds);
            struct timeval tv;
            tv.tv_sec = timeout_ms / 1000;
            tv.tv_usec = (timeout_ms % 1000) * 1000;
            int sel = ::select(fd + 1, nullptr, &wfds, nullptr, &tv);
            if (sel <= 0) {
                ::close(fd);
                return false;  // 超时或 select 出错（如外部服务未启动）
            }
            // 连接结果通过 SO_ERROR 反映
            int so_error = 0;
            socklen_t so_len = sizeof(so_error);
            getsockopt(fd, SOL_SOCKET, SO_ERROR, &so_error, &so_len);
            if (so_error != 0) {
                ::close(fd);
                return false;
            }
        } else {
            // 立即失败（如目标地址不可达）
            ::close(fd);
            return false;
        }
    }

    fd_ = fd;
    connected_.store(true);
    peer_closed_.store(false);
    return true;
}

void TcpClient::disconnect() {
    connected_.store(false);
    peer_closed_.store(false);
    if (fd_ >= 0) {
        // 优雅关闭：先 shutdown 写端，让内核把发送缓冲区的数据发完再发 FIN，
        // 避免直接 close() 时缓冲区有未确认数据导致发 RST（对端会看到 ECONNRESET）。
        // 即使有未读的接收数据，shutdown(SHUT_WR) 也只影响写端，不会发 RST。
        ::shutdown(fd_, SHUT_WR);
        ::close(fd_);
        fd_ = -1;
    }
}

bool TcpClient::connected() const { return connected_.load(); }

bool TcpClient::peerClosed() const { return peer_closed_.load(); }

bool TcpClient::send(const uint8_t* data, size_t len, int timeout_ms) {
    if (!connected_.load() || fd_ < 0 || data == nullptr) {
        return false;
    }

    // 总体截止时间：整个 send 过程阻塞不超过 timeout_ms 毫秒
    auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);

    size_t sent_total = 0;
    while (sent_total < len) {
        // MSG_NOSIGNAL：避免对端关闭时触发 SIGPIPE 导致进程退出
        ssize_t n = ::send(fd_, data + sent_total, len - sent_total, MSG_NOSIGNAL);
        if (n > 0) {
            sent_total += (size_t)n;
            continue;
        }
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            // 发送缓冲满：等待可写，但不超过截止时间
            auto now = std::chrono::steady_clock::now();
            if (now >= deadline) {
                return false;  // 超时，放弃本次发送
            }
            auto remain =
                std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count();
            fd_set wfds;
            FD_ZERO(&wfds);
            FD_SET(fd_, &wfds);
            struct timeval tv;
            tv.tv_sec = remain / 1000;
            tv.tv_usec = (remain % 1000) * 1000;
            int sel = ::select(fd_ + 1, nullptr, &wfds, nullptr, &tv);
            if (sel <= 0) {
                return false;  // 超时或出错
            }
            continue;  // 可写，重试发送
        }
        // 连接已断开等其它错误（ECONNRESET/EPIPE 等）
        return false;
    }
    return true;
}

bool TcpClient::recvSome(uint8_t* buf, size_t max_len, size_t& got, int timeout_ms) {
    got = 0;
    if (!connected_.load() || fd_ < 0 || buf == nullptr || max_len == 0) {
        return false;
    }

    // 等待可读（最多 timeout_ms 毫秒）
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(fd_, &rfds);
    struct timeval tv;
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    int sel = ::select(fd_ + 1, &rfds, nullptr, nullptr, &tv);
    if (sel <= 0) {
        return false;  // 超时无数据（正常情况）或 select 出错
    }

    ssize_t n = ::recv(fd_, buf, max_len, 0);
    if (n > 0) {
        got = (size_t)n;
        return true;
    }
    if (n == 0) {
        peer_closed_.store(true);  // 对端关闭连接
        return false;
    }
    return false;  // n < 0：EAGAIN 等错误，本次无数据
}

#ifndef TOPIC_BROADCASTER_FRAME_QUEUE_H
#define TOPIC_BROADCASTER_FRAME_QUEUE_H

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <mutex>
#include <utility>

// 有界并发队列模板：容量在构造时固定，队满时丢弃最旧元素（丢帧策略），
// 供发布线程 -> 发送线程之间传递待发送的整帧数据。
//
// 线程安全：内部使用 std::mutex + std::condition_variable 保护 std::deque。
template <typename T>
class FrameQueue {
public:
    // 构造：容量固定为 capacity，队列最多同时容纳 capacity 个元素
    explicit FrameQueue(size_t capacity) : capacity_(capacity) {}

    // 入队：队列满时先弹出最旧元素再入队，返回 true 表示丢弃过帧。
    // 入队后唤醒一个正在 pop 等待的线程。
    bool push(T&& item) {
        bool dropped = false;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (queue_.size() >= capacity_) {
                queue_.pop_front();  // 队满丢最旧
                dropped = true;
            }
            queue_.push_back(std::move(item));
        }
        cv_.notify_one();
        return dropped;
    }

    // 出队：取出队首元素放入 out。队列为空时最多等待 timeout_ms 毫秒，
    // 超时仍未取到数据返回 false。
    bool pop(T& out, double timeout_ms) {
        std::unique_lock<std::mutex> lock(mutex_);
        // 等待条件：队列非空；wait_for 可被虚假唤醒，因此用谓词重查
        cv_.wait_for(lock,
                     std::chrono::duration<double, std::milli>(timeout_ms),
                     [this] { return !queue_.empty(); });
        if (queue_.empty()) {
            return false;  // 超时
        }
        out = std::move(queue_.front());
        queue_.pop_front();
        return true;
    }

    // 清空队列（丢弃全部待发送帧）
    void clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.clear();
    }

    // 当前队列长度
    size_t size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.size();
    }

private:
    size_t capacity_;            // 队列容量（固定）
    std::deque<T> queue_;        // 实际存储
    mutable std::mutex mutex_;   // 保护 queue_ 的互斥锁
    std::condition_variable cv_; // 唤醒等待出队的线程
};

#endif  // TOPIC_BROADCASTER_FRAME_QUEUE_H

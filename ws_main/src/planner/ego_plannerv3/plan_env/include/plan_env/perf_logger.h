/**
 * @file perf_logger.h
 * @brief Ego-Planner 性能日志模块（跨 .so 共享单例）
 *
 * 设计目标：
 *   - 每次程序启动新建一个带时间戳的日志文件，仅记录性能插桩输出，
 *     不混入 ROS 原有日志。
 *   - 线程安全，可被多个回调线程并发调用。
 *   - 跨 .so 共享同一实例：通过 C 接口 + dlsym(RTLD_DEFAULT) 实现，
 *     避免每个 .so 各自持有独立的 Meyers Singleton 导致日志丢失。
 *
 * 架构说明：
 *   - 真正的 PerfLogger 单例只在 libplan_env.so 中定义一次（perf_logger.cpp）。
 *   - 其他 .so（active_perception、plan_manage 等）通过 dlsym 查找
 *     get_perf_logger_instance 符号，获取同一实例的指针。
 *   - init_perf_logger（C 接口）由 FSM 调用一次完成初始化。
 *
 * 使用方式：
 *   1. 在 FSM 构造时调用 PERF_INIT(dir, enable) 初始化（内部走 C 接口）。
 *   2. 需要计时时：
 *        auto t0 = PERF_NOW();
 *        ... // 待测代码
 *        PERF_LOG_ELAPSED_EX("TAG", t0, "key=value");
 *   3. 需要记录非计时事件时：
 *        PERF_LOG("TAG", "key=value key=value");
 *
 * 日志格式：
 *   [2026-08-04 12:34:56.789] [TAG] elapsed_ms=12.3 key=value
 */

#ifndef PLAN_ENV_PERF_LOGGER_H
#define PLAN_ENV_PERF_LOGGER_H

#include <chrono>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>

#include <dlfcn.h>  // dlsym / RTLD_DEFAULT

namespace ego_perf {

// 统一的计时点类型（单调时钟，不受系统时间调整影响）
using TimePoint = std::chrono::steady_clock::time_point;

// 获取当前计时点
inline TimePoint now() {
  return std::chrono::steady_clock::now();
}

// 计算两个计时点之间的毫秒数
inline double elapsedMs(const TimePoint& start, const TimePoint& end) {
  return std::chrono::duration<double, std::milli>(end - start).count();
}

/**
 * @brief 性能日志单例类
 *
 * 真正的实例只在 libplan_env.so 中创建（见 perf_logger.cpp）。
 * 其他 .so 通过 Instance() -> dlsym 查找共享实例指针。
 */
class PerfLogger {
public:
  /**
   * @brief 初始化日志文件
   * @param dir    日志目录（不存在会自动创建）
   * @param enable 是否启用；为 false 时所有 log 调用均为空操作
   */
  void init(const std::string& dir, bool enable) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (initialized_) return;  // 防止重复初始化
    initialized_ = true;
    enabled_ = enable;
    if (!enable) return;

    // 确保目录存在（mkdir -p）
    std::string cmd = "mkdir -p " + dir;
    std::system(cmd.c_str());

    // 生成带时间戳的文件名
    std::time_t t = std::time(nullptr);
    std::tm* tm = std::localtime(&t);
    std::ostringstream oss;
    oss << dir << "/ego_perf_"
        << std::put_time(tm, "%Y%m%d_%H%M%S") << ".log";

    // 打开文件（追加模式，防止异常重启丢日志）
    ofs_.open(oss.str(), std::ios::out | std::ios::app);
    if (ofs_.is_open()) {
      ofs_ << "# Ego-Planner Performance Log" << std::endl;
      ofs_ << "# Format: [timestamp] [TAG] key=value key=value ..." << std::endl;
      ofs_.flush();
    }
  }

  /**
   * @brief 记录一条日志
   * @param tag 模块标签，如 "GRID_MAP" / "FTR_SEARCH" / "ASTAR"
   * @param msg 键值对内容，如 "elapsed_ms=12.3 ftr_num=85"
   */
  void log(const std::string& tag, const std::string& msg) {
    if (!enabled_) return;
    std::lock_guard<std::mutex> lock(mtx_);
    if (!ofs_.is_open()) return;

    // 生成带毫秒精度的系统时间戳
    auto now_sys = std::chrono::system_clock::now();
    auto t = std::chrono::system_clock::to_time_t(now_sys);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                now_sys.time_since_epoch()) % 1000;
    std::tm* tm = std::localtime(&t);

    ofs_ << "[" << std::put_time(tm, "%Y-%m-%d %H:%M:%S")
         << "." << std::setfill('0') << std::setw(3) << ms.count() << "] "
         << "[" << tag << "] " << msg << std::endl;
    ofs_.flush();
  }

  /**
   * @brief 便捷方法：记录从 start 到当前的耗时
   * @param tag   模块标签
   * @param start 起始计时点（由 PERF_NOW() 获取）
   * @param extras 额外的键值对（可为空）
   */
  void logElapsed(const std::string& tag, const TimePoint& start,
                  const std::string& extras = "") {
    if (!enabled_) return;
    double ms = elapsedMs(start, now());
    std::ostringstream oss;
    oss << "elapsed_ms=" << std::fixed << std::setprecision(3) << ms;
    if (!extras.empty()) oss << " " << extras;
    log(tag, oss.str());
  }

  /// 是否已启用
  bool isEnabled() const { return enabled_; }

private:
  // 允许 sharedInstance()（perf_logger.cpp）访问私有构造函数
  friend PerfLogger& sharedInstance();
  // 允许 Instance()（perf_logger.h）访问私有构造函数创建 dummy
  friend PerfLogger& Instance();

  PerfLogger() = default;
  ~PerfLogger() {
    if (ofs_.is_open()) ofs_.close();
  }
  PerfLogger(const PerfLogger&) = delete;
  PerfLogger& operator=(const PerfLogger&) = delete;

  std::ofstream ofs_;
  std::mutex mtx_;
  bool initialized_ = false;
  bool enabled_ = false;
};

}  // namespace ego_perf

// 前向声明：perf_logger.cpp 中定义，PerfLogger 的 friend 需要先声明
namespace ego_perf { PerfLogger& sharedInstance(); }

// ---------------- 跨 .so 共享单例的 C 接口 ----------------
// 真正的实现在 perf_logger.cpp（编译进 libplan_env.so）。
// 其他 .so 通过 dlsym(RTLD_DEFAULT, "ego_perf_get_instance") 查找共享指针。
extern "C" {
// 获取共享 PerfLogger 实例指针（libplan_env.so 中定义）
__attribute__((visibility("default")))
ego_perf::PerfLogger* ego_perf_get_instance();

// 初始化共享 PerfLogger 实例（FSM 调用一次）
__attribute__((visibility("default")))
void ego_perf_init(const char* dir, int enable);
}

namespace ego_perf {

/**
 * @brief 获取共享 PerfLogger 实例
 *
 * 通过 dlsym(RTLD_DEFAULT) 查找 libplan_env.so 导出的 ego_perf_get_instance 符号，
 * 确保所有 .so 共享同一实例。如果查找失败（FSM 尚未初始化），返回本地 dummy。
 */
inline PerfLogger& Instance() {
  // 通过 dlsym 查找共享实例指针（仅查找一次，结果缓存到静态变量）
  static PerfLogger* shared = []() -> PerfLogger* {
    void* sym = dlsym(RTLD_DEFAULT, "ego_perf_get_instance");
    if (sym) {
      using GetFn = PerfLogger* (*)();
      GetFn fn = reinterpret_cast<GetFn>(sym);
      return fn();
    }
    return nullptr;
  }();
  if (shared) return *shared;
  // fallback：dlsym 未找到（libplan_env.so 尚未加载或未初始化），
  // 返回本地 dummy 实例（enabled_=false，所有 log 调用被跳过）
  static PerfLogger dummy;
  return dummy;
}

}  // namespace ego_perf

// ---------------- 便捷宏 ----------------
// PERF_INIT 通过 C 接口初始化共享实例（确保跨 .so 生效）
// 注意：ego_perf_init 是 extern "C" 函数，位于全局命名空间，不是 ego_perf:: 内
#define PERF_INIT(dir, enable) ::ego_perf_init((dir).c_str(), (enable) ? 1 : 0)
#define PERF_LOG(tag, msg) ::ego_perf::Instance().log(tag, msg)
#define PERF_LOG_ELAPSED(tag, start) ::ego_perf::Instance().logElapsed(tag, start)
#define PERF_LOG_ELAPSED_EX(tag, start, extras) ::ego_perf::Instance().logElapsed(tag, start, extras)
#define PERF_NOW() ::ego_perf::now()

#endif  // PLAN_ENV_PERF_LOGGER_H

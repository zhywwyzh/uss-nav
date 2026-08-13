/**
 * @file perf_logger.cpp
 * @brief Ego-Planner 性能日志模块 - 跨 .so 共享单例实现
 *
 * 本文件编译进 libplan_env.so，提供 PerfLogger 的真正单例实例。
 * 其他 .so（active_perception、plan_manage 等）通过 dlsym(RTLD_DEFAULT)
 * 查找 ego_perf_get_instance 符号，获取同一实例指针，从而共享日志输出。
 *
 * 之所以需要这套机制：Meyers Singleton（static local）在 C++ 中每个 .so
 * 会持有独立副本，导致 FSM 调用 PERF_INIT 初始化的是 FSM 进程内的实例，
 * 而 grid_map / frontier_finder / planner_manager 等模块各自 .so 中的实例
 * 仍处于 enabled_=false 状态，log 调用被静默跳过。
 */

#include <plan_env/perf_logger.h>

namespace ego_perf {

/**
 * @brief 真正的共享单例实例（仅在此文件中定义一次）
 *
 * 通过函数内 static 局部变量实现线程安全的延迟初始化。
 * 所有 .so 通过 ego_perf_get_instance() C 接口获取同一实例指针。
 */
PerfLogger& sharedInstance() {
  static PerfLogger instance;
  return instance;
}

}  // namespace ego_perf

// ---------------- C 接口实现（extern "C" + visibility("default")）----------------
// 这些符号会被导出到动态符号表，其他 .so 可通过 dlsym(RTLD_DEFAULT) 查找。

extern "C" {

/// 获取共享 PerfLogger 实例指针
__attribute__((visibility("default")))
ego_perf::PerfLogger* ego_perf_get_instance() {
  return &ego_perf::sharedInstance();
}

/// 初始化共享 PerfLogger 实例（由 FSM 调用一次）
/// @param dir    日志目录（C 字符串）
/// @param enable 是否启用（1=启用，0=关闭）
__attribute__((visibility("default")))
void ego_perf_init(const char* dir, int enable) {
  ego_perf::sharedInstance().init(dir ? std::string(dir) : "", enable != 0);
}

}  // extern "C"

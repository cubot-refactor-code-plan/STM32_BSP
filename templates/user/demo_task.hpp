/**
 * @file demo_task.hpp
 * @brief 示例业务（模板）：两个不依赖任何外设的任务，演示 user_init() 怎样创建多个业务任务
 *
 * @details 接入后即可编译运行，GDB 挂接查看 user_alive_count / user_monitor 确认任务在跑；
 *          写自己的业务后删掉本文件与 demo_task.cpp，并去掉 user_init() 里对应的两行。
 */

#ifndef __DEMO_TASK_HPP__
#define __DEMO_TASK_HPP__

#include <stdint.h>


/* ==================== 调试观察量 ==================== */

/** @brief 运行监视结果（GDB：print user_monitor） */
struct UserMonitor
{
  uint32_t heap_free;       ///< 当前 FreeRTOS 堆剩余（字节）
  uint32_t heap_min_free;   ///< 开机以来 FreeRTOS 堆最低剩余（字节）：接近 0 就该加 configTOTAL_HEAP_SIZE
  uint32_t task_num;        ///< 当前任务数
  uint32_t self_stack_free; ///< 本任务栈最低剩余（字）；INCLUDE_uxTaskGetStackHighWaterMark = 0 时恒为 0
};

extern volatile uint32_t user_alive_count; ///< 心跳计数：每秒加一（GDB：print user_alive_count）
extern volatile UserMonitor user_monitor;  ///< 运行监视结果，每秒刷新


/* ==================== 任务入口 ==================== */

/**
 * @brief 心跳业务：每秒把 user_alive_count 加一
 * @param argument 未使用（NULL）
 */
void heartbeat_task(void *argument);

/**
 * @brief 监视业务：每秒记录 FreeRTOS 堆余量与任务数到 user_monitor
 * @param argument 未使用（NULL）
 */
void monitor_task(void *argument);

#endif // __DEMO_TASK_HPP__

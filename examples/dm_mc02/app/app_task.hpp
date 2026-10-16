/**
 * @file app_task.hpp
 * @author Rh
 * @brief 应用层任务声明 —— 除默认任务外的业务任务
 * @version 0.1
 * @date 2026-08-14
 *
 * @copyright Copyright (c) 2026
 *
 * @note 任务函数均以 extern "C" 声明（FreeRTOS 以 C 方式调用）。
 *       任务实现位于 User/App/task/，由 all_init() 统一创建。
 */

#ifndef __APP_TASK_HPP__
#define __APP_TASK_HPP__

#ifdef __cplusplus
extern "C"
{
#endif

  /**
   * @brief 按键任务：200 ms 轮询 key_user，按事件驱动蜂鸣器提示
   * @param argument 任务参数（未使用，NULL）
   * @note 轮询周期与 key_user 的消抖 / 长按配置对应（debounce 1 → 200 ms，long_press 5 → 1 s）；
   *       beep 为阻塞调用，实际周期会多出一个响铃时长。
   */
  void key_task(void *argument);


#ifdef __cplusplus
}
#endif

#endif // __APP_TASK_HPP__

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
   * @brief 系统级 1 kHz 维护任务：CAN Bus-Off 恢复、每 10 ms 一次的串口收发补救、推进全部 Online 对象的离线计时
   * @param argument 任务参数（未使用，NULL）
   * @note 由 all_init() 创建，不应由业务代码直接调用。
   */
  void sys_task(void *argument);

  /**
   * @brief 以 1 kHz 轮询各 CAN 原始缓冲，成功分发则消费，否则转入回退缓冲
   * @param argument 任务参数（未使用，NULL）
   * @note 原始缓冲仅由本任务读取；receive() 读取未被节点成功处理的帧。
   */
  void can_rx_task(void *argument);
  // void dji_motor_task(void *argument);


#ifdef __cplusplus
}
#endif

#endif // __APP_TASK_HPP__

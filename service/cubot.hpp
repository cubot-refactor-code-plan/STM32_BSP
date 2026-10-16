/**
 * @file cubot.hpp
 * @author ChoseB
 * @brief cubot_bsp 的入口：建系统状态、创建库自带的后台任务、置运行态
 * @version 0.1
 * @date 2026-10-17
 *
 * @copyright Copyright (c) 2026
 *
 * @details 宿主工程在 freertos.c 的 MX_FREERTOS_Init()（USER CODE 段）里按顺序调用：
 *
 *            cubot_init();    // 1. 建系统事件组 + 创建库任务（任务入口先等运行态，此时还不跑）
 *            board_init();    // 2. 用户：各外设实例 init()、设备 init()、注册 CAN 节点等
 *            xTaskCreate(...);// 3. 用户：创建业务任务（入口先调 SysFlagWaitRunning()）
 *            cubot_start();   // 4. 置运行位：前面有初始化失败记录就在这里停机（fault_record 记下行号）
 *
 *          库任务（都按 1 kHz 节拍，见各自实现）：
 *            - cubot_sys_task    CAN Bus-Off 恢复、每 10 ms 串口收发补救、推进 Online 离线计时；
 *            - cubot_can_rx_task 把各 CAN 收到的帧按 ID 分发给接收节点，没人要的进回退缓冲；
 *            - cubot_can_tx_task 按槽位聚合发送 CAN 控制帧（保底心跳 + 100 ms 指令有效期）；
 *              没有任何发送节点时自行退出。
 *          它们遍历 BspCan / BspUartBase 的实例登记表，不引用任何板级实例。
 *
 * @note 任务优先级与栈深可在包含本文件之前用同名宏覆盖（阶段 D 收进 cubot_config.h）。
 */

#ifndef __CUBOT_HPP__
#define __CUBOT_HPP__

#include "FreeRTOS.h" // IWYU pragma: keep (tskIDLE_PRIORITY)

/** @brief 系统维护任务优先级：高于业务任务，低于 CAN 收发任务 */
#ifndef CUBOT_SYS_TASK_PRIORITY
#  define CUBOT_SYS_TASK_PRIORITY (tskIDLE_PRIORITY + 7)
#endif

/** @brief 系统维护任务栈深（字） */
#ifndef CUBOT_SYS_TASK_STACK
#  define CUBOT_SYS_TASK_STACK 256
#endif

/** @brief CAN 收 / 发任务优先级：最高，1 kHz 节拍内必须跑完 */
#ifndef CUBOT_CAN_TASK_PRIORITY
#  define CUBOT_CAN_TASK_PRIORITY (tskIDLE_PRIORITY + 8)
#endif

/** @brief CAN 收 / 发任务栈深（字）：接收节点回调在 cubot_can_rx_task 里同步执行 */
#ifndef CUBOT_CAN_TASK_STACK
#  define CUBOT_CAN_TASK_STACK 512
#endif

#ifdef __cplusplus
extern "C"
{
#endif

  /**
   * @brief 库初始化：建系统事件组、创建库任务
   * @note 必须最先调用（CAN 节点注册等要用系统事件组）；在调度器启动前调用，失败就地断言停机。
   */
  void cubot_init(void);

  /**
   * @brief 初始化完成：置运行位，放行所有等待运行态的任务
   * @note 有初始化失败记录（SysInitError）时断言停机，fault_record 记下本次调用位置。
   */
  void cubot_start(void);

  /** @brief 系统维护任务（由 cubot_init() 创建，业务代码不要直接调用） */
  void cubot_sys_task(void *argument);

  /** @brief CAN 接收分发任务（由 cubot_init() 创建） */
  void cubot_can_rx_task(void *argument);

  /** @brief CAN 发送任务（由 cubot_init() 创建） */
  void cubot_can_tx_task(void *argument);

#ifdef __cplusplus
}
#endif

#endif // __CUBOT_HPP__

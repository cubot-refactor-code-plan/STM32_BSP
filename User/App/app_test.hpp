/**
 * @file app_test.hpp
 * @author Rh
 * @brief 应用层测试任务声明
 * @version 0.1
 * @date 2026-08-14
 *
 * @copyright Copyright (c) 2026
 *
 * @note 测试任务函数均以 extern "C" 声明（FreeRTOS 以 C 方式调用）。
 *       任务实现位于 User/App/test/，由 all_init() 统一创建。
 */

#ifndef __APP_TEST_HPP__
#define __APP_TEST_HPP__

/** @brief 编译并创建 Online 实机自检任务；设为 0 可停用。 */
#define APP_TEST_ONLINE_CHECK_ENABLED 0

/** @brief CAN1 零输出故障注入与 Bus-Off 恢复实机测试。 */
#define APP_TEST_CAN_RECOVERY_ENABLED 0

#ifdef __cplusplus
extern "C"
{
#endif

  void can_recovery_test_init(void);
  void can_recovery_test_task(void *argument);

  /**
   * @brief Online 状态机自检任务
   * @param argument 任务参数（未使用，NULL）
   * @note 仅在 APP_TEST_ONLINE_CHECK_ENABLED 非零时创建。
   */
  void online_check_test_task(void *argument);

#ifdef __cplusplus
}
#endif

#endif // __APP_TEST_HPP__

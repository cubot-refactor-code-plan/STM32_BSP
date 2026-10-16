#include "api_main.h"
#include "FreeRTOS.h" // IWYU pragma: keep
#include "main.h"     // IWYU pragma: keep
#include "task.h"

#include <stdint.h>
#include <stdio.h>

/* BSP */
#include "bsp_cfg.hpp"

/* Device */
#include "device_cfg.hpp"

/* Service */
#include "can_tx_node.hpp"
#include "status.hpp"

/* 任务声明 */
#include "app_task.hpp"
#include "app_test.hpp"


/**
 * @brief FreeRTOS后的相关初始化
 *
 * @note 在main.c的MX_FREERTOS_INIT函数中调用
 *       用于创建FreeRTOS任务和初始化外设驱动
 *
 *       初始化顺序：
 *         1. 统一创建信号量（必须在创建任何任务之前！）
 *         2. bsp_init()     —— 外设 BSP 初始化
 *         3. device_init()  —— 设备层初始化
 *         4. ...
 */
void all_init()
{
  // 初始化错误记录信号量
  configASSERT(SysFlagInit() == Status::OK);

  /* 初始化BSP设备 */
  bsp_init();

  /* 初始化设备 */
  device_init();

#if APP_TEST_CAN_RECOVERY_ENABLED
  can_recovery_test_init();
#endif

  /* 系统维护与 CAN 发送任务（均为 1 kHz，CAN 发送优先级更高） */
  configASSERT(xTaskCreate(sys_task, "sys", 256, NULL, tskIDLE_PRIORITY + 7, NULL) == pdPASS);
  configASSERT(xTaskCreate(can_rx_task, "can_rx", 512, NULL, tskIDLE_PRIORITY + 8, NULL) == pdPASS);
  configASSERT(xTaskCreate(can_tx_task, "can_tx", 512, NULL, tskIDLE_PRIORITY + 8, NULL) == pdPASS);

#if APP_TEST_CAN_RECOVERY_ENABLED
  configASSERT(xTaskCreate(can_recovery_test_task, "can_fault", 512, NULL, tskIDLE_PRIORITY + 5, NULL) == pdPASS);
#endif

#if APP_TEST_ONLINE_CHECK_ENABLED
  configASSERT(xTaskCreate(online_check_test_task,
                           "online_test",
                           256,
                           NULL,
                           tskIDLE_PRIORITY + 3,
                           NULL) == pdPASS);
#endif

  // 有初始化失败记录（各注册函数调过 SysInitError）就在这里停机，fault_record 记下本行
  configASSERT(SysCompleteInit() == Status::OK);

}


/**
 * @brief ST的默认任务，弱定义，在这里定义，方便寻找
 *
 * @note 任务名保留 CubeMX 生成的 StartDefaultTask（便于 CubeMX 重新生成时同名兼容）；
 *       自定义任务函数命名遵循小写蛇形规范（如 sys_task）。
 *
 * @note freertos调用cpp的函数，需要extern "C"修饰，避免C++的名称修饰导致找不到函数
 *
 * @param argument 任务参数
 */
extern "C" void StartDefaultTask(void *argument)
{
  (void)argument; // 未使用参数

  for (;;)
  {
    osDelay(1);
  }
}

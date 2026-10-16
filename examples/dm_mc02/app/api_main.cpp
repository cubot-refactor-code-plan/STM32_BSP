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
#include "cubot.hpp"
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
 *       初始化顺序（见 service/cubot.hpp）：
 *         1. cubot_init()   —— 库入口：系统事件组 + 库任务（CAN 收发、系统维护）
 *         2. bsp_init()     —— 本板外设实例 init()（实例在 init() 时登记进库的登记表）
 *         3. device_init()  —— 本板设备
 *         4. 创建业务任务与测试任务
 *         5. cubot_start()  —— 置运行位，放行所有任务
 */
void all_init()
{
  // 库入口：建系统事件组、创建库任务（CAN 收发、系统维护；它们先等运行态）
  cubot_init();

  /* 初始化BSP设备 */
  bsp_init();

  /* 初始化设备 */
  device_init();

#if APP_TEST_CAN_RECOVERY_ENABLED
  can_recovery_test_init();
#endif

  /* 按键任务：200 ms 轮询，短按 / 长按发不同提示音 */
  configASSERT(xTaskCreate(key_task, "key", 256, NULL, tskIDLE_PRIORITY + 5, NULL) == pdPASS);

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

  // 初始化完成：置运行位；有初始化失败记录就在这里停机（fault_record 记下位置）
  cubot_start();

}


/**
 * @brief ST的默认任务，弱定义，在这里定义，方便寻找
 *
 * @note 任务名保留 CubeMX 生成的 StartDefaultTask（便于 CubeMX 重新生成时同名兼容）；
 *       自定义任务函数命名遵循小写蛇形规范（如 key_task）。
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

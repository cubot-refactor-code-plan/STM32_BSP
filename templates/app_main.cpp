#include "app_main.h"

#include "board_cfg.hpp"
#include "system.hpp"
#include "status.hpp"

#include "FreeRTOS.h" // IWYU pragma: keep
#include "task.h"


/* ==================== 业务任务 ==================== */

volatile uint32_t app_alive_count = 0U; ///< 业务任务循环次数（调试时挂接查看）

/**
 * @brief 示例业务任务：每秒从串口 1 打印一行，开机响一声
 * @note 任务入口先等运行态：初始化失败时 SysFlagWaitRunning() 让任务自行退出，不会跑在半初始化的系统上。
 */
static void app_task(void *argument)
{
  (void)argument;
  SysFlagWaitRunning();
  buzzer.beep(3000U, 80U);
  TickType_t wake = xTaskGetTickCount();
  for (;;)
  {
    ++app_alive_count;
    (void)bsp_uart1.printf("alive %lu\r\n", static_cast<unsigned long>(app_alive_count));
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(1000U));
  }
}


/* ==================== 入口 ==================== */

extern "C" void app_init(void)
{
  // 1. 库入口：系统事件组 + 库任务（sys / can_rx / can_tx，调度器启动后才运行）
  system_init();

  // 2. 板级初始化：外设实例 init()、设备 init()、注册 CAN 节点
  board_init();

  // 3. 业务任务（优先级低于库任务：默认 CAN +8、系统维护 +7）
  configASSERT(xTaskCreate(app_task, "app", 512, NULL, tskIDLE_PRIORITY + 3, NULL) == pdPASS);

  // 4. 放行：置运行位；初始化阶段有失败记录（例如 CAN 节点注册失败）就在这里断言停机
  system_start();
}

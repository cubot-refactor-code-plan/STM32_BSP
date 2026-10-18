#include "demo_task.hpp"

#include "status.hpp"

#include "FreeRTOS.h" // IWYU pragma: keep
#include "task.h"


volatile uint32_t    user_alive_count = 0U;
volatile UserMonitor user_monitor     = {};


/* ==================== 任务 ==================== */

/**
 * @note 任务入口先等运行态：初始化失败时 SysFlagWaitRunning() 让任务自行退出，不会跑在半初始化的系统上。
 */
void heartbeat_task(void *argument)
{
  (void)argument;
  SysFlagWaitRunning();
  TickType_t wake = xTaskGetTickCount();
  for (;;)
  {
    user_alive_count = user_alive_count + 1U;
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(1000U));
  }
}

void monitor_task(void *argument)
{
  (void)argument;
  SysFlagWaitRunning();
  TickType_t wake = xTaskGetTickCount();
  for (;;)
  {
    user_monitor.heap_free       = static_cast<uint32_t>(xPortGetFreeHeapSize());
    user_monitor.heap_min_free   = static_cast<uint32_t>(xPortGetMinimumEverFreeHeapSize());
    user_monitor.task_num        = static_cast<uint32_t>(uxTaskGetNumberOfTasks());
#if INCLUDE_uxTaskGetStackHighWaterMark
    user_monitor.self_stack_free = static_cast<uint32_t>(uxTaskGetStackHighWaterMark(NULL));
#endif
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(1000U));
  }
}

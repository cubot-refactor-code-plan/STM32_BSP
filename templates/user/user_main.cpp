#include "user_main.hpp"

#include "demo_task.hpp"

#include "FreeRTOS.h" // IWYU pragma: keep (configASSERT)
#include "task.h"


/* ==================== 业务任务初始化 ==================== */

void user_init(void)
{
  // 每个业务一行断言：任务创建失败（多半是 FreeRTOS 堆不够）就地停机，fault_record 记下的行号直接指出是哪个业务。
  // 优先级低于库任务（默认 CAN +8、系统维护 +7）；栈深单位是字（4 字节）。
  configASSERT(xTaskCreate(heartbeat_task, "heartbeat", 128, NULL, tskIDLE_PRIORITY + 2, NULL) == pdPASS);
  configASSERT(xTaskCreate(monitor_task, "monitor", 256, NULL, tskIDLE_PRIORITY + 1, NULL) == pdPASS);
}

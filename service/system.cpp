#include "system.hpp"
#include "hardware_config_defaults.hpp"

#include "stm32h7xx_hal.h" // IWYU pragma: keep（HAL 模块开关）
#include "status.hpp"
#include "FreeRTOS.h" // IWYU pragma: keep
#include "task.h"

static_assert(SYS_TASK_PRIORITY < configMAX_PRIORITIES && CAN_TASK_PRIORITY < configMAX_PRIORITIES,
              "hardware_config.h: 库任务优先级必须小于 configMAX_PRIORITIES");


// ---------------- 库入口 ----------------


extern "C" void system_init(void)
{
  // 系统事件组最先建：CAN 节点注册、SysFlagWaitRunning() 都依赖它
  configASSERT(SysFlagInit() == Status::OK);

  // 调度器启动前任务不会运行（CAN 收发任务还会先等运行态），所以可以早于用户的外设 init()；
  // 任务名受 configMAX_TASK_NAME_LEN（16）限制
  configASSERT(xTaskCreate(sys_task, "sys", SYS_TASK_STACK, NULL, SYS_TASK_PRIORITY, NULL) == pdPASS);
#ifdef HAL_FDCAN_MODULE_ENABLED
  // 宿主工程没开 FDCAN 时不建 CAN 收发任务
  configASSERT(xTaskCreate(can_rx_task, "can_rx", CAN_TASK_STACK, NULL, CAN_TASK_PRIORITY, NULL) == pdPASS);
  configASSERT(xTaskCreate(can_tx_task, "can_tx", CAN_TASK_STACK, NULL, CAN_TASK_PRIORITY, NULL) == pdPASS);
#endif
}

extern "C" void system_start(void)
{
  // 有初始化失败记录（各注册函数调过 SysInitError）就在这里停机，fault_record 记下本行
  configASSERT(SysCompleteInit() == Status::OK);
}


// ----------------

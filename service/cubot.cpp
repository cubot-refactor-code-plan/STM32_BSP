#include "cubot.hpp"

#include "status.hpp"
#include "task.h"


// ---------------- 库入口 ----------------


extern "C" void cubot_init(void)
{
  // 系统事件组最先建：CAN 节点注册、SysFlagWaitRunning() 都依赖它
  configASSERT(SysFlagInit() == Status::OK);

  // 库任务一创建就先等运行态，所以可以早于用户的外设 init()；任务名受 configMAX_TASK_NAME_LEN（16）限制
  configASSERT(xTaskCreate(cubot_sys_task, "cubot_sys", CUBOT_SYS_TASK_STACK, NULL, CUBOT_SYS_TASK_PRIORITY, NULL) == pdPASS);
  configASSERT(xTaskCreate(cubot_can_rx_task, "cubot_can_rx", CUBOT_CAN_TASK_STACK, NULL, CUBOT_CAN_TASK_PRIORITY, NULL) == pdPASS);
  configASSERT(xTaskCreate(cubot_can_tx_task, "cubot_can_tx", CUBOT_CAN_TASK_STACK, NULL, CUBOT_CAN_TASK_PRIORITY, NULL) == pdPASS);
}

extern "C" void cubot_start(void)
{
  // 有初始化失败记录（各注册函数调过 SysInitError）就在这里停机，fault_record 记下本行
  configASSERT(SysCompleteInit() == Status::OK);
}


// ----------------

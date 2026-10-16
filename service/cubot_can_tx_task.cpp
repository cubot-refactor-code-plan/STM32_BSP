#include "stm32h7xx_hal.h" // IWYU pragma: keep（HAL 模块开关）

// 宿主工程没在 CubeMX 里开 FDCAN（HAL_FDCAN_MODULE_ENABLED 未定义）时，本文件不参与编译
#ifdef HAL_FDCAN_MODULE_ENABLED

#include "can_tx_node.hpp"
#include "cubot.hpp"

#include <string.h>

extern "C" void cubot_can_tx_task(void *argument)
{
  (void)argument;
  SysFlagWaitRunning();
  CanTxNode::frozen = true;
  // 总线来自 BspCan 的登记表（各实例 init() 时登记，运行态之前已定型），库里不引用板级实例
  TickType_t wake_time = xTaskGetTickCount();
  bool has_nodes = false;
  for (uint32_t bus = 0U; bus < BspCan::instance_count(); ++bus)
  {
    BspCan *const can = BspCan::instance_at(bus);
    for (CanTxNode *node = can->tx_head; node != nullptr; node = node->next)
    {
      node->last_send_tick = wake_time;
      has_nodes = true;
    }
  }
  if (!has_nodes)
  {
    osThreadExit();
    return;
  }
  for (;;)
  {
    for (uint32_t bus = 0U; bus < BspCan::instance_count(); ++bus)
    {
      BspCan *const can = BspCan::instance_at(bus);
      for (CanTxNode *node = can->tx_head; node != nullptr; node = node->next)
      {
        if (osMutexAcquire(node->bufferMutex, pdMS_TO_TICKS(1U)) != osOK)
          continue;
        // 指令有效期：超过 CMD_EXPIRE_MS 没被 filldata() 刷新的槽位清零，
        // 避免控制任务卡死后保底心跳一直重发最后一次电流
        const TickType_t now = xTaskGetTickCount();
        for (uint8_t slot = 0; slot < node->division; ++slot)
        {
          const uint8_t mask = static_cast<uint8_t>(1U << slot);
          if ((node->bufferRegister & mask) != 0U && (node->bufferExpired & mask) == 0U &&
              now - node->bufferFillTick[slot] >= pdMS_TO_TICKS(CanTxNode::CMD_EXPIRE_MS))
          {
            memset(node->txBuffer.data + slot * 8U / node->division, 0, 8U / node->division);
            node->bufferExpired |= mask;
            ++node->expiredCount;
          }
        }
        const bool ready = node->bufferRegister != 0U && node->bufferUnsend == node->bufferRegister;
        const bool due = xTaskGetTickCount() - node->last_send_tick >= pdMS_TO_TICKS(CUBOT_CAN_TX_HEARTBEAT_MS);
        if ((ready || due) && can->send(node->txBuffer.std_id, node->txBuffer.data) == Status::OK)
        {
          node->bufferUnsend = 0;
          node->last_send_tick = xTaskGetTickCount();
        }
        osMutexRelease(node->bufferMutex);
      }
    }
    vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(1U));
  }
}

#endif // HAL_FDCAN_MODULE_ENABLED

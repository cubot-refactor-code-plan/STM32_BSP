#include "cubot.hpp"
#include "cubot_defaults.hpp"
#include "bsp_can.hpp"  // 未开 FDCAN 时为空
#include "bsp_uart.hpp" // 未开 UART 时为空

#include "FreeRTOS.h" // IWYU pragma: keep
#include "online_check.hpp"
#include "task.h"

#include <stdint.h>


volatile uint32_t cubot_sys_loop_count = 0U;       ///< 维护任务循环次数（调试观察）
volatile Status   cubot_sys_online_status = Status::NOT_INIT; ///< 最近一次 Online::update() 的结果

// 串口补救巡检的周期（本任务 1 拍 = 1 ms）：CUBOT_UART_PATROL_PERIOD_MS，默认 10 ms。
// tx_recover() 判“通道空闲却有数据”要连续两次巡检都看到才动手，10 ms 间隔与 main 分支实测一致；
// rx_recover() 只在 RX 三处状态不一致时才动作，正常时每次只读一次寄存器。
constexpr uint32_t UART_PATROL_PERIOD = CUBOT_UART_PATROL_PERIOD_MS;



extern "C" void cubot_sys_task(void *argument)
{
  (void)argument; // 任务不需要外部参数。
  static_assert(configTICK_RATE_HZ == 1000U,
                "cubot_sys_task requires a 1 kHz FreeRTOS tick");

  TickType_t wake_time = xTaskGetTickCount();
  for (;;)
  {
#ifdef HAL_FDCAN_MODULE_ENABLED
    // CAN 硬件 Bus-Off 自恢复：遍历 BspCan 登记表（各实例 init() 时登记）
    for (uint32_t i = 0U; i < BspCan::instance_count(); ++i)
    {
      (void)BspCan::instance_at(i)->service_recovery();
    }
#endif
#ifdef HAL_UART_MODULE_ENABLED
    // 串口补救：卡死的发送续发 + 停摆的接收重建（ISR 只计数，恢复只能在任务侧做），遍历 BspUart 登记表
    if ((cubot_sys_loop_count % UART_PATROL_PERIOD) == 0U)
    {
      for (uint32_t i = 0U; i < BspUartBase::instance_count(); ++i)
      {
        BspUartBase *const uart = BspUartBase::instance_at(i);
        (void)uart->tx_recover();
        (void)uart->rx_recover();
      }
    }
#endif
    // 在线情况更新
    cubot_sys_online_status = Online::update();
    ++cubot_sys_loop_count;
    vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(1U));
  }
}

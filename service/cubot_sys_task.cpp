#include "cubot.hpp"
#include "bsp_can.hpp"
#include "bsp_uart.hpp"

#include "FreeRTOS.h" // IWYU pragma: keep
#include "online_check.hpp"
#include "task.h"

#include <stdint.h>


volatile uint32_t cubot_sys_loop_count = 0U;       ///< 维护任务循环次数（调试观察）
volatile Status   cubot_sys_online_status = Status::NOT_INIT; ///< 最近一次 Online::update() 的结果

// 串口补救巡检的周期：每 10 拍（10 ms）一次。
// tx_recover() 判“通道空闲却有数据”要连续两次巡检都看到才动手，10 ms 间隔与 main 分支实测一致；
// rx_recover() 只在 RX 三处状态不一致时才动作，正常时每次只读一次寄存器。
constexpr uint32_t UART_PATROL_PERIOD = 10U;



extern "C" void cubot_sys_task(void *argument)
{
  (void)argument; // 任务不需要外部参数。
  static_assert(configTICK_RATE_HZ == 1000U,
                "cubot_sys_task requires a 1 kHz FreeRTOS tick");

  TickType_t wake_time = xTaskGetTickCount();
  for (;;)
  {
    // CAN 硬件 Bus-Off 自恢复：遍历 BspCan 登记表（各实例 init() 时登记）
    for (uint32_t i = 0U; i < BspCan::instance_count(); ++i)
    {
      (void)BspCan::instance_at(i)->service_recovery();
    }
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
    // 在线情况更新
    cubot_sys_online_status = Online::update();
    ++cubot_sys_loop_count;
    vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(1U));
  }
}

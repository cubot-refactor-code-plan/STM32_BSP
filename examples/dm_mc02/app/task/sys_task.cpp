#include "app_task.hpp"
#include "bsp_cfg.hpp"

#include "FreeRTOS.h" // IWYU pragma: keep
#include "online_check.hpp"
#include "task.h"

#include <stdint.h>


volatile uint32_t sys_task_loop_count = 0U;
volatile Status sys_task_online_status = Status::NOT_INIT;

// 串口补救巡检的周期：每 10 拍（10 ms）一次。
// tx_recover() 判“通道空闲却有数据”要连续两次巡检都看到才动手，10 ms 间隔与 main 分支实测一致；
// rx_recover() 只在 RX 三处状态不一致时才动作，正常时每次只读一次寄存器。
constexpr uint32_t UART_PATROL_PERIOD = 10U;

// 串口组（ISR 只计数，收发恢复只能在任务侧做）
BspUart<128> *const patrol_uarts[] = {&bsp_uart1, &bsp_uart3, &bsp_uart4, &bsp_uart5, &bsp_uart7, &bsp_uart8, &bsp_uart9, &bsp_uart10};


extern "C" void sys_task(void *argument)
{
  (void)argument; // 任务不需要外部参数。
  static_assert(configTICK_RATE_HZ == 1000U,
                "sys_task requires a 1 kHz FreeRTOS tick");

  TickType_t wake_time = xTaskGetTickCount();
  for (;;)
  {
    // can硬件bus-off自恢复
    bsp_can1.service_recovery();
    bsp_can2.service_recovery();
    bsp_can3.service_recovery();
    // 串口补救：卡死的发送续发 + 停摆的接收重建
    if ((sys_task_loop_count % UART_PATROL_PERIOD) == 0U)
    {
      for (auto *uart : patrol_uarts)
      {
        uart->tx_recover();
        uart->rx_recover();
      }
    }
    // 在线情况更新
    sys_task_online_status = Online::update();
    ++sys_task_loop_count;
    vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(1U));
  }
}

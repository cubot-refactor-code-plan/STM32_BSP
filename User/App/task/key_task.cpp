#include "app_task.hpp"
#include "bsp_cfg.hpp"
#include "bsp_key.hpp"    // IWYU pragma: keep (BspKey::Event)
#include "device_cfg.hpp" // buzzer
#include "FreeRTOS.h"     // IWYU pragma: keep
#include "task.h"

extern "C" void key_task(void *argument)
{
  (void)argument;

  for (;;)
  {
    // 200 ms 轮询（对应 key_user 的 debounce 1 / long_press 5 ⇒ 200 ms 消抖 / 1 s 长按）
    switch (key_user.poll())
    {
      case BspKey::Event::PRESSED: // 3 kHz / 40 ms：按下轻嘀
        buzzer.beep(3000, 40);
        break;
      case BspKey::Event::RELEASED: // 4 kHz / 100 ms：短按松手，高而短（长按后的松手不会产生 RELEASED）
        buzzer.beep(4000, 100);
        break;
      case BspKey::Event::LONG_PRESS: // 2 kHz / 500 ms：长按，低而长
        buzzer.beep(2000, 500);
        break;
      default:
        break;
    }

    vTaskDelay(pdMS_TO_TICKS(200)); // beep 是阻塞的，周期会多出一个响铃时长
  }
}

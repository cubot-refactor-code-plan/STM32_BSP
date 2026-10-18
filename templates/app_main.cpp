#include "app_main.h"

#include "board_cfg.hpp"
#include "system.hpp"
#include "user_main.hpp"


/* ==================== 入口 ==================== */

extern "C" void app_init(void)
{
  // 1. 库入口：系统事件组 + 库任务（sys / can_rx / can_tx，调度器启动后才运行）
  system_init();

  // 2. 板级初始化：外设实例 init()、设备 init()、注册 CAN 节点
  board_init();

  // 3. 业务任务：全部在 user/user_main.cpp 的 user_init() 里创建（每个业务一行断言）
  user_init();

  // 4. 放行：置运行位；初始化阶段有失败记录（例如 CAN 节点注册失败）就在这里断言停机
  system_start();
}

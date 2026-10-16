# templates：接入 cubot_bsp 时复制到宿主工程的文件

按仓库根 `README.md` 第 1 节的步骤使用；完整可运行的例子见 `examples/dm_mc02/`。

| 文件 | 放到哪里 | 作用 |
| --- | --- | --- |
| `linker_sections.ld` | 宿主链接脚本 `SECTIONS { }` 里、`.data` 之前 | `.dma_buffer`（串口实例，DMA 可达的 RAM_D1）与 `.noinit`（`fault_record`，复位保留）。缺了 CMake 配置阶段报错 |
| `FreeRTOSConfig_user_code.h` | `Core/Inc/FreeRTOSConfig.h` 的 USER CODE 1 段 | `configASSERT` → `fault_on_assert()` 记录现场 |
| `freertos_user_code.c` | `Core/Src/freertos.c` 的 Includes / 4 / 5 / Init 段 | 栈溢出与分配失败钩子记录现场；`MX_FREERTOS_Init()` 调 `app_init()` |
| `cubot_config.h` | `Core/Inc/`（stm32cubemx 的包含目录） | 可选：覆盖库的默认配置（任务优先级 / 栈深、队列深度、节点上限、指令有效期、巡检周期等） |
| `board_cfg.hpp` / `board_cfg.cpp` | 你的应用目录（如 `app/`） | 外设与设备实例（句柄、波特率、定时器参数）和 `board_init()` |
| `app_main.h` / `app_main.cpp` | 你的应用目录 | `app_init()`：`cubot_init()` → `board_init()` → 业务任务 → `cubot_start()`，含一个示例任务 |

`board_cfg.*` 用的是 `hfdcan1` / `huart1` / `htim12`，换成你工程里实际开的外设；没开的外设连同 `#include` 一起删掉（库会按 HAL 模块开关跳过对应驱动）。

应用源文件加进宿主 CMake（CubeMX 生成的 `project()` 默认已启用 C++）：

```cmake
add_subdirectory(third_party/cubot_bsp)
target_link_libraries(${CMAKE_PROJECT_NAME} cubot_bsp)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE app/board_cfg.cpp app/app_main.cpp)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE app)
```

验证（2026-10-17）：在示例工程里删掉 `app/`，换成本目录的模板与片段、按上面四行接入，编译 0 warning，上板运行位已置、示例任务每秒计数、串口实例落在 RAM_D1。

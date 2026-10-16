# CUBOT BSP：可作为 git submodule 接入的 STM32H7 板级库

> 分支 `mini-bsp`。目标：从 DM-MC02 整车工程里提炼出一个库，**任何 STM32H7 CubeMX 工程**都能把它作为 git submodule 接入。
>
> 期望的使用方式：**CubeMX 配置一个新工程 → 接入本 submodule → 修改少量配置 → 编写业务代码。**
>
> 当前进度：阶段 A（删减，与 main 对齐）已完成；本仓库目前仍是一个完整的 CubeMX 工程（DM-MC02 板），目录搬迁与去板级耦合按下文「分阶段实施」推进。下文标注「目标」的接口与目录**尚未实现**。

---

## 1. 目标用法（阶段 B~D 完成后成立）

### 1.1 CubeMX 新建工程时的要求

| 项目 | 要求 | 原因 |
| --- | --- | --- |
| 芯片 | STM32H7 系列 | 驱动用到 FDCAN、DMA Stream、AXI SRAM（RAM_D1） |
| 工具链 | Project Manager → Toolchain = **CMake** | 库以 CMake 子目录方式接入，依赖 CubeMX 生成的 `stm32cubemx` INTERFACE 目标 |
| RTOS | FreeRTOS，**CMSIS_V2** 接口；tick **1 kHz** | 代码里有 `static_assert(configTICK_RATE_HZ == 1000)` |
| FreeRTOS 钩子 | `configCHECK_FOR_STACK_OVERFLOW = 2`、`configUSE_MALLOC_FAILED_HOOK = 1` | 钩子里调库的 `fault_on_stack_overflow()` / `fault_on_malloc_failed()` 记录现场 |
| HAL 时基 | 不用 SysTick（选一个空闲定时器） | SysTick 给 FreeRTOS |
| FDCAN | 经典 CAN、RX FIFO0；开中断 0 / 1 | `BspCan` 用 FIFO0 收、TX FIFO 发、TX-FIFO-EMPTY 中断续发 |
| UART | 需要的串口开 **RX DMA + TX DMA + 全局中断**（只收的串口可不配 TX DMA） | `BspUart` 用 IDLE + DMA 收、DMA 发 |
| TIM PWM | 需要的通道配成 PWM，记下定时器时钟、PSC、ARR | `BspPwm` 的参数全部由用户配置传入 |
| GPIO | 按键、片选等按需配置并起 Label | `BspGpio` / `BspKey` 只封装电平读写 |

### 1.2 接入 submodule

```bash
git submodule add <本仓库地址> third_party/cubot_bsp
```

在 CubeMX 生成的根 `CMakeLists.txt` 末尾加两行（目标库名 `cubot_bsp`，暂定）：

```cmake
add_subdirectory(third_party/cubot_bsp)
target_link_libraries(${CMAKE_PROJECT_NAME} cubot_bsp)
```

库自己 `enable_language(CXX)`（C++11），并链接 `stm32cubemx` 取 HAL / FreeRTOS 的头文件与宏，所以宿主工程不需要再配 include 路径。

### 1.3 修改的配置（CubeMX 重新生成不会覆盖 USER CODE 段，但会覆盖链接脚本）

1. **链接脚本**加两个段（DMA 访问不到 DTCM，所以串口实例要放进 RAM_D1；故障记录要跨复位保留）：

   ```ld
   .dma_buffer (NOLOAD) :
   {
     . = ALIGN(32);
     *(.dma_buffer)
     *(.dma_buffer*)
     . = ALIGN(32);
   } >RAM_D1

   .noinit (NOLOAD) :
   {
     . = ALIGN(4);
     *(.noinit)
     *(.noinit*)
     . = ALIGN(4);
   } >RAM_D1
   ```

   CubeMX 重新生成会用默认内容覆盖链接脚本，生成后要把这两段加回来（目标：`templates/` 里提供片段与检查脚本）。

2. **`FreeRTOSConfig.h` 的 USER CODE 1 段**把 `configASSERT` 接到故障记录（原型直接写在这里，因为内核源码看不到库的头文件）：

   ```c
   void fault_on_assert(const char *file, int line);
   void fault_on_stack_overflow(const char *task_name);
   void fault_on_malloc_failed(void);
   #define configASSERT( x ) if ((x) == 0) { fault_on_assert(__FILE__, __LINE__); }
   ```

3. **`freertos.c` 的钩子桩**（USER CODE 4 / 5）分别调 `fault_on_stack_overflow(pcTaskName)`、`fault_on_malloc_failed()`。

4. **板级实例配置**（用户工程里的一个 `.cpp`，目标：从 `templates/board_cfg.cpp` 复制后改）：

   ```cpp
   // 哪些外设、用哪个句柄、波特率 / 定时器参数——全部由用户决定，库里不写死任何板级常量
   BspCan bsp_can1({&hfdcan1, "CAN1"});
   __attribute__((section(".dma_buffer"))) BspUart<128> bsp_uart1({&huart1, true, 115200U});
   BspPwm bsp_pwm_buzzer({&htim12, TIM_CHANNEL_2, 275000000UL, 23U, 1999U});

   void board_init(void)
   {
     configASSERT(bsp_can1.init() == Status::OK);       // 每条 init() 单独一行断言，失败时记录里的行号直接指出是哪一路
     configASSERT(bsp_uart1.init() == Status::OK);
     configASSERT(bsp_pwm_buzzer.init() == Status::OK);
   }
   ```

5. **初始化入口**：`freertos.c` 的 `MX_FREERTOS_Init()` USER CODE 段里调一次库入口（目标名 `cubot_init()`：建系统事件组 → 调用户的 `board_init()` → 建库自带的任务 → 置运行位）。

### 1.4 编写业务代码

```cpp
DjiMotor<Motor3508> chassis_lf(bsp_can1, 1U);   // 初始化期注册收发节点

extern "C" void chassis_task(void *)
{
  SysFlagWaitRunning();                         // 等系统进入运行态；初始化失败时任务自行退出
  for (;;)
  {
    const MotorData &d = chassis_lf.data();     // 反馈由 CAN 接收任务按 ID 分发、解包
    (void)chassis_lf.FillData(1000);            // 只写发送槽位，CAN 发送任务统一发出；100 ms 不刷新自动清零
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}
```

---

## 2. 库包含什么（当前状态）

依赖方向：`App → Module → Device → Bsp`，`Service` / `Algorithm` 贯穿各层；下层不包含上层头文件。

| 层 | 模块 | 功能 | 实机状态 |
| --- | --- | --- | --- |
| Bsp | `bsp_can` / `bsp_can_recovery` | FDCAN 收发（MessageBuffer）、DLC > 8 帧丢弃计数、Bus-Off 非阻塞恢复 | DLC 过滤：内部回环实测；Bus-Off 恢复：2026-09-30 实测 |
| Bsp | `bsp_uart` | IDLE + DMA 收、多任务安全发（写锁 + printf 锁）、发送卡死续发、接收停摆重建 | 串口跳线实测（2026-10-16） |
| Bsp | `bsp_pwm` | 通用 PWM：占空比 / 脉宽 / 频率，参数全由配置传入 | 蜂鸣器通道实测频率 |
| Bsp | `bsp_key` / `bsp_gpio` | 轮询消抖按键 / GPIO 电平 | 按键实测 |
| Service | `status` | 统一状态码、系统事件标志、等待运行态 | 实测 |
| Service | `fault_record` | 断言 / 栈溢出 / 分配失败现场记录，`.noinit` 跨复位保留 | 实测 |
| Service | `can_rx_node` / `can_tx_node` | CAN 按 ID 分发接收、槽位聚合发送、100 ms 指令有效期 | 回环实测 |
| Service | `online_check`、`motor_definition` | 在线计时、电机通用数据 | 编译通过 |
| Device | `dji_motor` | M3508 / M2006 / GM6020，每电机一个对象 | C620 + M3508 实测（2026-09-01 / 10-01，当时版本；本轮改动后未接电机复测） |
| Device | `device_buzzer` | 无源蜂鸣器（音调 / 音量 / 时长） | 实测 |
| Device | `dm_imu` | 空文件占位，待按 CAN 节点层重写 | — |
| Algorithm | `pid` | 位置式 PID | 未测；已知问题：“微分先行”实为对目标值求导 |

当前属于示例板、搬迁后进 `examples/dm_mc02` 的代码：`User/Bsp/bsp_cfg.*`、`User/Device/device_cfg.*`（实例与初始化）、`User/App/`（`api_main`、`sys_task`、`can_rx_task` / `can_tx_task`、`key_task`、测试）。

已按 main 移除（2026-10-16）：TinyUSB / `bsp_usb`、QSPI 外置 Flash 与 XIP、EmmV5、协议层（DataPack / `protocol_uart`）、LCD 与菜单、达妙电机空声明、ADC1 与 CMSIS-DSP、`bsp_buzzer`（换成 `bsp_pwm` + `device_buzzer`）。CAN 节点分发层与 `DjiMotor` 按决定保留（可移植库需要一条总线挂多类设备）。

---

## 3. 设计

### 3.1 目标目录结构（阶段 B）

```txt
cubot_bsp/                    仓库根 = 库
├─ CMakeLists.txt             定义 cubot_bsp 库目标；链接 stm32cubemx（INTERFACE）取 HAL / FreeRTOS
├─ bsp/  service/  device/  algorithm/   来自现在的 User/ 各层
├─ templates/                 board_cfg.cpp、FreeRTOSConfig.h USER CODE 片段、链接脚本片段、cubot_config.h
├─ docs/                      编码规范、器件手册
└─ examples/
   └─ dm_mc02/                现在的 CubeMX 工程整体 git mv 进来
      ├─ DM_MC02.ioc、Core/、Drivers/、Middlewares/、startup、链接脚本、cmake/、CMakePresets.json
      ├─ Flash/、.vscode/
      ├─ app/                 板级用户代码：实例配置、api_main、业务任务、测试
      └─ CMakeLists.txt       add_subdirectory(../.. cubot_bsp) 接入本库
```

- `.ioc` 与 `Core/`、`Drivers/`、`Middlewares/` 一起搬，相对路径不变，CubeMX 仍可在 `examples/dm_mc02/` 里打开、重新生成。
- 示例既是使用示范，也是实机回归工程：`cd examples/dm_mc02 && cmake --preset Debug && cmake --build --preset Debug`。
- 代价：用户克隆 submodule 时会带上示例的 HAL / FreeRTOS（数十 MB），git 无法排除。
- 烧录脚本现在用目录名找 ELF，搬迁后要改成用 CMake 工程名。

### 3.2 去板级耦合（阶段 C）

现在与 DM-MC02 绑定的地方及改法：

| 现状 | 改法 |
| --- | --- |
| `bsp_can.cpp` / `bsp_uart.cpp` 的 HAL 回调按句柄 if-else 分发给写死的 `bsp_can1..3`、`bsp_uart1..10` | 实例在 `init()` 时登记到静态表，回调查表分发；表容量可配置 |
| 驱动头文件包含 CubeMX 按外设生成的 `fdcan.h` / `usart.h` / `tim.h`（没开该外设就不存在） | 只包含 `main.h` / HAL 头；按 `HAL_FDCAN_MODULE_ENABLED`、`HAL_UART_MODULE_ENABLED`、`HAL_TIM_MODULE_ENABLED` 条件编译各驱动 |
| 实例与初始化写在库里（`bsp_cfg.cpp`、`device_cfg.cpp`） | 移到用户工程（`templates/board_cfg.cpp` 示范） |
| 库的后台任务（CAN 收 / 发分发、`sys_task` 巡检：Bus-Off 恢复、串口补救、在线计时）由 `api_main` 创建 | 由库入口 `cubot_init()` 创建，优先级 / 栈深可配置 |
| 库占用 HAL 回调 `HAL_FDCAN_RxFifo0Callback`、`HAL_FDCAN_TxFifoEmptyCallback`、`HAL_UARTEx_RxEventCallback`、`HAL_UART_TxCpltCallback`、`HAL_UART_ErrorCallback` | 保留由库定义，文档写明宿主工程不能再定义同名回调；需要时提供用户钩子 |

### 3.3 配置化（阶段 D）

用户工程提供 `cubot_config.h`（不提供就用库内默认值）：CAN 收发队列深度、实例表容量、库任务优先级与栈深、CAN 指令有效期（默认 100 ms）、巡检周期（默认 10 ms）、在线超时等。

### 3.4 约束与限制

- 只支持 STM32H7（FDCAN、DMA Stream、RAM_D1）。
- 假定 **D-Cache 关闭**；开启时 `.dma_buffer` 要配成 MPU 非缓存区。
- 需要 FreeRTOS + CMSIS-RTOS v2；C++11；tick 1 kHz。
- `BspUart` 卡死恢复时，若 TX DMA 开着 DMA FIFO，可能丢 FIFO 里预读的字节（约 8~16 字节/次）。
- 运行期可恢复的失败返回 `Status`，初始化失败一律 `configASSERT` 停机并写 `fault_record`（见 `docs/编码规范.md` 第 8 节）。

### 3.5 分阶段实施

| 阶段 | 内容 | 状态 |
| --- | --- | --- |
| A | 删减，与 main 对齐（见第 2 节） | ✅ 2026-10-16，每项编译 / 烧录 / 挂接检查 |
| B | 目录搬迁：CubeMX 工程 `git mv` 到 `examples/dm_mc02`，`User/` 拆成库目录，板级代码进示例，烧录脚本改用 CMake 工程名 | 待做 |
| C | 去板级耦合（3.2） | 待做 |
| D | 配置化（3.3）与 `templates/` | 待做 |
| E | 验证：示例工程实机回归；再用一个全新 CubeMX 工程按第 1 节从零接入，证明可移植 | 待做 |

---

## 4. 当前仓库：编译、烧录与调试（DM-MC02 板）

环境：`cmake`（≥ 3.22）、`ninja`、Arm GNU Toolchain（`arm-none-eabi-gcc` / `-gdb`）在 `PATH` 中；CMSIS-DAP 探针 + OpenOCD（本机须用 `/usr/local/bin/openocd` 0.12，0.11 驱动不了 Horco 探针）。工作区目录名、CMake 工程名与 ELF 名须保持 `DM_MC02`（脚本按目录名找 ELF）。

```bash
cmake --preset Debug                 # 首次配置
cmake --build --preset Debug         # 编译，产物 build/Debug/DM_MC02.elf
bash Flash/OpenOCD_flash.sh          # 单段烧录：program ELF verify reset exit
bash Flash/JLink_flash.sh            # J-Link（经 OpenOCD jlink 适配器，未实测）
```

- 探针配置见 `Flash/README.md`：固定 HID 后端；**不注册 OCTOSPI2 外置 Flash bank**（注册后 OpenOCD 在 program / reset 时会碰 OCTOSPI2，固件不重新初始化它时会把调试口卡死，需要重新上电）。
- VS Code：`Terminal: Run Task` 里有编译与烧录任务；调试配置 `cmsis-dap-debug`（Cortex-Debug + OpenOCD），外设寄存器描述 `Flash/STM32H723.svd`。
- 测试开关在 `User/App/app_test.hpp`，默认全部为 0：Online 自检、CAN1 Bus-Off 恢复测试（见 `User/App/test/can/README.md`）。
- 查看故障现场：GDB 挂接后 `print fault_record`（种类、次数、文件行号、tick、IPSR、任务名）。

示例板引脚取舍（DM-MC02）：SPI3 与串口 3 / 串口 5 引脚冲突，保留 U3、U5（舍弃 SPI3 与 RS485-3）；I2C1 与串口 4 复用，选串口 4。

---

## 5. 开发约定

- 编码规范：`docs/编码规范.md`（命名、注释、`Config` 构造模式、分层、断言与初始化失败）。格式由 `.clang-format` 固定，手动执行。
- C 调 C++ 只通过 `api_main.h` 转接一次（`main.c` / `freertos.c` → `all_init()` / 任务入口），HAL 与 FreeRTOS 回调用 `extern "C"`。
- 间接包含的头文件行尾加 `// IWYU pragma: keep`，避免 clangd 误报。
- 很多文件是 CRLF 行尾：修改时保持原行尾，提交前用 `git diff --numstat` 与 `git diff --ignore-cr-at-eol --numstat` 对比，确认没有整文件的行尾改动。
- CubeMX 重新生成后检查链接脚本 `STM32H723xG_flash.ld`：会被默认内容覆盖，丢掉 `.dma_buffer` / `.noinit`，必须还原。

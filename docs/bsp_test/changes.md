# 测试过程中的代码与配置改动

按时间顺序记录。库（`third_party/stm32_bsp`）的改动单独提交在 submodule 里，这里写提交号。

## 1. CubeMX：RS485 两路串口（2026-10-17）

**原因**：RS485 测试要用板上两路 485（引脚图 `CtrBoard-H7管脚标注图.pdf`：485-1 = USART2 TX PD5 / RX PD6 / DE PD4，485-2 = USART3 TX PD8 / RX PD9 / DE PB14）。
`BspUart` 要求 RX DMA + TX DMA + 串口全局中断。

**改动前**（用户已在 CubeMX 里改了一半）：

- USART3：已加 RS485 硬件 DE（PB14）✅，但 TX/RX 仍在 PC10/PC11（扩展口 SPI3 引脚，不接 485 收发器）❌。
- USART2：RS485 模式、DE PD4，但**没有 DMA、没有中断** ❌。
- DMA1/DMA2 共 16 路，已用 15 路，只剩 DMA1_Stream0。

**改动**（与用户确认后）：

| 项 | 改动 |
| --- | --- |
| USART2 RX DMA | 接管 UART9 RX 的 DMA2_Stream1（参数原样：Normal、FIFO 关） |
| USART2 TX DMA | 接管 UART9 TX 的 DMA2_Stream2（参数原样：Normal、FIFO 开、半满阈值） |
| USART2 中断 | `USART2_IRQn` 使能，优先级 5（同 USART3） |
| UART9 | 外设保留，**不再有 DMA**（扩展口 PD14/PD15，本工程未使用） |
| USART3 引脚 | TX/RX 从 PC10/PC11 改到 PD8/PD9（AF7），DE PB14 不变 |

**过程**：先用 EmberProbe `mcu-cubemx`（生成候选 `.ioc` → `--prepare` → 用户确认 → `--start`），两次失败（见 `flash_debug_eval.md`），
最后用同一份已校验的候选 `.ioc` 走 CubeMX 命令行（`STM32CubeMX -q`，脚本 `config load / project path / project generate / exit`）生成。

**结果**：`MC02_with_submodule.ioc`、`Core/Src/usart.c`、`Core/Src/stm32h7xx_it.c`、`Core/Inc/stm32h7xx_it.h` 变化；
USER CODE 段、链接脚本、`FreeRTOSConfig.h`、`freertos.c` 未受影响；编译 0 warning。

## 2. 测试固件（宿主工程，不改库）

| 文件 | 内容 |
| --- | --- |
| `app/app_test.hpp` | 测试开关：总开关、UART / RS485 / CAN 各组、浸泡时长、物理插拔模式 |
| `app/board_cfg.*` | 新增实例 `bsp_can2`、`bsp_uart2/3`（`BspUart<128>`，115200，RS485）、`bsp_uart7`（`BspUart<128>`，921600）、`bsp_uart10`（`BspUart<256>`，921600） |
| `app/app_main.cpp` | 开关打开时 `bsp_test_init()`（在 `system_start()` 前注册测试 CAN 节点）并创建测试任务 |
| `app/test/bsp_test.*` | 结果表 `bsp_test_results[]`、串口错误类型监视任务、系统检查 |
| `app/test/test_uart.cpp` | UART / RS485 用例与物理插拔用例 |
| `app/test/test_can.cpp` | CAN 用例与 CAN 插拔用例 |
| `CMakeLists.txt` | 加入 `app/test/*.cpp` |

### 测试代码自身的修正

| 轮次 | 问题 | 修正 |
| --- | --- | --- |
| 1 | `set_baud()` 用 `HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_USART234578)` 取内核时钟，这版 HAL 对 PCLK 时钟源返回 0，BRR 被写成 0，U7 波特率发生器停转，U04a 在 2 Mbps 档卡死（OpenOCD 读到 UART7 BRR = 0） | 按 `usart.c` 的时钟源选择改用 `HAL_RCC_GetPCLK1Freq()`（USART2/3/7）/ `HAL_RCC_GetPCLK2Freq()`（USART10），并断言非 0 |
| 1 | `wait_done()` 在数据还没到达时（bytes = 0 = last）立即判为“已安静”，U03 读到 0 字节 | 先等满一个安静窗口再比较 |
| 1 | 吞吐用 DWT 微秒计数，超过 7.8 s 回绕（U02 算出 94 B/s） | 改用 1 ms tick 计时 |
| 1 | 编辑器保存时格式化，`U03b` 出现两行 | 删除重复行 |
| 1 | 只有计数增量，看不出串口错误类型 | 新增监视任务，每 1 ms 读 `huart->ErrorCode`，按 NE / FE / ORE / 其他 统计到每个用例的 `err[]` |
| 2~3 | `case_tx_full` 在 `rx_reset()` 清零计数之后才判定，误判失败 | 改用已记下的 `v[]` 判定 |
| 3 | 运行中 `init()` 失败后 `receive()` 立即返回 IO_ERROR，测试接收任务空转饿死低优先级任务（runner 卡住） | 非 OK、非 TIMEOUT 时让出 1 ms |
| 3 | U13 运行中 `init()` 时测试接收任务正阻塞在该实例流缓冲区上（会释放后使用） | 先让接收任务离开 `receive()` 再 `init()` |
| 3 | C06b 两个同优先级发送任务成功时不让出 CPU，一路饿死另一路 | 成功后 `taskYIELD()` |
| 5 | 新增诊断：D01 收发字节日志、每次 `receive()` 的时刻与余量（`BSP_TEST_DIAG_LOG`，RAM_D1，平时不编译） | — |
| 6 | U05 / U07 判据随库修复更新（接收容量 2 倍；printf 超时行会拼成坏行） | — |
| 7~8 | 新增 R03b（换向保护时间）、C06c/d/e（队头阻塞、运行中切队列模式、同 ID 顺序） | — |
| 9 | `rx_reset()` 之后接收任务可能还在关闭态的 2 ms 睡眠里，4 Mbps 下开头 800 字节溢出 | `rx_reset()` 末尾等 3 ms |
| 11 | 浸泡挪到最前的实验开关 `BSP_TEST_SOAK_FIRST`（定位 -O2 卡死用） | — |
| 物理 | `case_unplug` 用 `tx_req = 0` 停发送端，而发送任务的 `--tx_req` 不是原子操作，0 被覆盖回去，P2 后 runner 卡住（当时用调试器写 0 解除） | 改为独立的 `tx_abort` 标志（**只编译验证，未上板复跑**） |

## 3. 库（`third_party/stm32_bsp`）修复

代码改动已由用户提交到 submodule：`d32a996`（`[fix]通信驱动bug修复`）；README 的文档更新留给下一次提交。每项都有上板回归。

| 编号 | 文件 | 问题（实测证据） | 修改 | 验证 |
| --- | --- | --- | --- | --- |
| L1 | `bsp/bsp_uart.cpp` | `uart_irqn()` 没有 USART2 / USART6，重建接收时这两路不屏蔽中断（静态检查；R07 未观察到实际故障） | 补上表项 | R07 通过 |
| L2 | `bsp/bsp_uart.cpp/.hpp` | 运行中再 `init()`（文档说可重复调用）返回 IO_ERROR 且实例失效：先释放了流缓冲区，接收仍是 BUSY_RX，重新启动失败（U13） | 已初始化的实例先停收发（关本串口中断、中止 RX / TX）再释放资源；文档注明此时不能有任务阻塞在 send / receive 上 | U13 |
| L2b | `bsp/bsp_uart.cpp` | L2 后仍失败：`HAL_DMA_Abort()` 5 ms 超时把句柄留在 ERROR，流稍后才停，单次 `repair_dma_handle()` 来不及复位 | 新增 `stop_dma_stream()`：请求停流、有限时间等 EN 清零、复位句柄与 HAL 状态 | U13 通过（第 7 轮起） |
| L3 | `bsp/bsp_uart.cpp/.hpp` | 绑到没配 DMA 的串口时 `init()` 返回 OK（HAL 不报错），实例永远收不到（U13） | 没有 RX DMA、或启用发送却没有 TX DMA → BAD_ARG | U13 |
| L4 | `bsp/bsp_uart.cpp`、`service/hardware_config_defaults.hpp` | UART 硬件 FIFO 关着，IDLE / TC 后到重新启动 DMA 的空窗里第 2 个字节即 ORE；2 Mbps 以上连续流几乎全丢（U04、R05、U02 的 ORE） | 取消注释 `HAL_UARTEx_EnableFifoMode()`，加配置项 `UART_ENABLE_FIFO`（默认 1） | U04 / R05 / U02（第 4 轮实验 + 第 6 轮起） |
| L5 | `bsp/bsp_uart.cpp/.hpp` | 接收流缓冲区与 DMA 缓冲一样大，TC 投递把它填满，紧接的 IDLE 投递被丢（诊断 1） | 接收流缓冲区改为 2 × BUFFER_SIZE（`RX_STREAM_SIZE`） | U01 / U07 / 各复测 |
| L6 | `bsp/bsp_can.cpp` | DLC < 8 的帧固定拷 8 字节，DLC 之后带出栈上旧数据（C05） | 按 DLC 拷贝，尾部保持 0 | C05 |
| L7 | `bsp/bsp_can.cpp` | 全局过滤用 `FDCAN_FILTER_REMOTE`（HAL 含义是“接收”），远程帧带无意义数据进回退缓冲（C05）；用户确认改为拒收 | `FDCAN_REJECT_REMOTE` | C05 |
| L8 | `bsp/bsp_can.hpp`、`service/can_rx_task.cpp` | 回退缓冲满时丢帧只置事件标志，没有计数（C06a） | 新增 `diagnostics.rx_return_dropped` | C06a |
| L9 / L9b | `bsp/bsp_uart.cpp/.hpp` | 卡死判定只看“本段启动后过了多久”，阈值按 `Config::baudrate`；填错时正常发送被误判中止，开 FIFO 后还丢数据（U15：150 帧丢 97）。L9 只看 DMA 计数仍会在段尾误判 | 推进 = DMA 计数变化 **或** UART 仍在移位（TC 未置位） | U15 150/150、误判 0；U12a/b 真卡死仍能恢复 |
| L10 | `service/hardware_config_defaults.hpp`、`templates/hardware_config.h`、README | CAN 接收队列 / 回退缓冲 8 帧，满载溢出（C06a、C06b） | 默认改 16 | C06a / C06b |
| L11 | `service/can_rx_task.cpp`、`hardware_config_defaults.hpp` | `can_rx_task` 每 tick 每路写死最多 8 帧，低于 1 Mbps 满载帧率（C06b `rx_dropped` 657） | 配置项 `CAN_RX_DISPATCH_PER_TICK`（默认 16，1~64） | C06b 16627/16627 |
| L12 | `bsp/bsp_can.cpp/.hpp` | 用 `HAL_FDCAN_GetTxFifoFreeLevel()`（TFFL）判硬件空位，TX 队列模式下 TFFL 恒为 0，一帧都发不出（C06e 卡住） | 新增 `tx_hw_has_room()`，用 TFQF（满标志），两种模式都有效 | C06e 1000/1000 不乱序 |
| L13 | `bsp/bsp_uart.cpp` | HAL 在 IDLE 时先读剩余计数再停 DMA，中间到达的字节不计入 size，静默丢 1 字节（诊断 2） | `on_idle_isr()` 里按停流后的剩余计数重算 size | U04 / R05 4 Mbps 无损、D01 200/200 |
| 文档 | `README.md`、`bsp_uart.hpp`、`templates/hardware_config.h` | 新默认值、新配置项、FIFO 与远程帧行为、实测记录位置 | 已更新 | — |
| 待办 | `undo.md`（git 忽略） | HAL `HAL_DMA_Abort()` 读-改-写竞争（诊断 3） | RX 改 DMA 循环模式（用户确认下次做） | — |

另：submodule 的示例工程 `examples/dm_mc02` 用改后的库编译通过（0 warning），未上板。

## 4. CubeMX：HAL 时基中断优先级（2026-10-17，诊断 3 的缓解）

- 改动：`NVIC.TIM8_UP_TIM13_IRQn` 优先级 15 → 4（`TICK_INT_PRIORITY` = 4），高于串口 / CAN 中断（5）。TIM8 中断只做 `HAL_IncTick()`，不调 FreeRTOS API，可以高于 `configMAX_SYSCALL_INTERRUPT_PRIORITY`。
- 作用：HAL 在串口中断里调用的 `HAL_DMA_Abort()` 若命中竞争，5 ms 后能超时返回，不再整机卡死。
- 过程：EmberProbe `mcu-cubemx` 再次以 “Updater is busy” 误报失败（当时没有打开 CubeMX 界面），改用同一份已校验的候选 `.ioc` 走 CubeMX 命令行生成；只有 `stm32h7xx_hal_conf.h` 的 `TICK_INT_PRIORITY` 与 `.ioc` 变化。
- 验证：-O2 浸泡约 16 min 无卡死（未改前 9 s 内必现）。

## 5. 其他

- EmberProbe 工作区配置的 ELF 在测试中切到过 `build/Release`、`build/RelWithDebInfo`，最后已改回 `build/Debug`。
- `build/RelWithDebInfo` 是测试 -O2 时另配的构建目录（`-O2 -g`，Release 预设是 `-g0`，没有调试信息、EmberProbe 读不了结构体）。
- 工程根目录留有 EmberProbe 生成失败时的诊断目录 `.emberprobe-cubemx-*` 与 `.emberprobe-cubemx-audit/`（未提交，skill 要求不要删诊断日志，由用户决定是否清理 / 忽略）。
- 测试开关已恢复：`APP_BSP_TEST_ENABLED 0`，默认固件编译 0 warning。

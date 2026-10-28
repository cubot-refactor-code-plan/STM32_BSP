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

## 6. CubeMX：SPI2 接 BMI088（2026-10-28）

- 改动：删掉 UART8 RX / TX 的 DMA（UART8 在 `app/board_cfg.cpp` 没有实例；DMA1 / DMA2 只剩 DMA1_Stream0 一个空闲流，全双工 DMA 需要两个）；
  SPI2_RX → DMA1_Stream0（Normal、Byte、High），SPI2_TX → DMA1_Stream7（Normal、Byte、Low），两个流中断优先级 5；SPI2 全局中断，优先级 5；
  SPI2 `MasterKeepIOState = ENABLE`（一次片选内多段传输之间 HAL 会关 SPE，不保持 IO 时 SCK 浮空，模式 3 的器件会看到多余时钟沿）。
- 不改：TIM3 / PB1（BMI088 加热电阻，按要求不碰）。
- 过程：直接编辑 `.ioc` 成候选，CubeMX 命令行（`STM32CubeMX -q`，`config load / project path / project generate / exit`）生成。
  变化只有 `spi.c`（DMA、中断、KeepIOState）、`dma.c`、`stm32h7xx_it.c/.h`（DMA1_Stream0、SPI2 中断，删 DMA2_Stream0）、`usart.c`（删 UART8 DMA）；`tim.c` 未变，行尾未变，链接脚本这次没被改。

## 7. SPI2 / BMI088：库实现与测试（2026-10-27 ~ 10-28，记录见 `spi_bmi088.md`）

库（`third_party/stm32_bsp`）：

| 编号 | 文件 | 内容 |
| --- | --- | --- |
| 新增 | `bsp/bsp_spi.cpp` | `BspSpi`：登记表、互斥锁、轮询 / DMA（阈值）、弹跳缓冲与分段、DMA 超时 `HAL_SPI_Abort`、`HAL_SPI_TxRxCpltCallback` / `HAL_SPI_ErrorCallback`；`BspSpiDevice`：一次片选事务、寄存器读写约定 |
| 新增 | `device/device_bmi088.cpp` | 上电序列（假读切 SPI、软复位、ID、加速度计上电）、写后回读校验、`update()` → `ImuData` / 原始值 / 温度、数据就绪中断计数与任务通知 |
| 头文件 | `bsp/bsp_spi.hpp` | 私有 `unregister_self()`、`BspSpiDevice::transaction()`；零拷贝 / 分段说明；KeepIOState 要求 |
| 头文件 | `device/device_bmi088.hpp` | 私有 `write_verify()` 加比较掩码（GYRO_BANDWIDTH 读回 bit7 恒为 1） |
| L-SPI1 | `bsp/bsp_spi.cpp` | 第 1 轮 S03 / S04：`transfer_locked()` 只要 tx 或 rx 为空就两边都走弹跳缓冲，`read_reg()`（数据段 tx 为空）永远不能零拷贝，`bounce_cnt` 也把填充算成了中转 → 两个方向各自判断能否直接用，`bounce_cnt` 只计调用方缓冲 DMA 访问不到的段 |
| L-BMI1 | `device/device_bmi088.hpp/.cpp`、`templates/board_cfg.cpp` | 第 5 / 6 轮：±6 g 在碰撞 / 剧烈晃动下饱和（P06 ±6 g 段 25 帧），读数被截断 → `Config` 默认加速度计量程改 ±24 g；`Bmi088RawData::saturated` 位与 `acc_saturation_count()` / `gyro_saturation_count()`；板级 `imu_bmi088` 改 ±24 g |
| L-BMI2 | `device/device_bmi088.hpp/.cpp`、`service/hardware_config_defaults.hpp`、`templates/hardware_config.h`、`templates/board_cfg.cpp`、README | 收尾后用户发现正常固件里 `imu_bmi088` 的 `_data` / `_raw_data` 全是 0：`DeviceBmi088` 是拉取式，测试关掉后没人调 `update()` → 参照 `DjiMotor`（库任务刷新）改为默认自动刷新：`Config::auto_update`（默认 true）时 `init()` 建内部任务 "imu"，陀螺仪数据就绪唤醒（无中断时 10 ms 兜底）后 `update()`，成功后给 `attach_task()` 的任务置 `UPDATED_BIT`；`update()` 在临界区里写回，新增 `snapshot()`、`update_count()` / `update_fail_count()` / `last_status()`；配置项 `BMI088_TASK_PRIORITY`（+6）/ `BMI088_TASK_STACK`（256）。正常固件实测 1 s 刷新 2013 次 = 陀螺仪中断数、0 失败、acc.z ≈ 9.8 m/s²。随后按用户要求 `Config` 默认陀螺仪 ODR 改为 1000 Hz（`HZ1000_BW116`），自动刷新 1 kHz |
| L-SPI2 | `bsp/bsp_spi.hpp`、`device/device_bmi088.hpp/.cpp` | 低优先级任务争用总线时可能等锁超过默认 10 ms（P05 一次）；用户决定默认值不变 → 文档写明低优先级使用者自行传更长的 `timeout_ms`，`DeviceBmi088::update()` 增加可选 `timeout_ms` 参数（编译通过，未上板） |
| 其他 | `service/imu_definition.hpp`（用户新建）、`device/dm_imu.*`（用户删除） | IMU 通用数据 `ImuData`；BMI088 按 `DjiMotor` 的写法把它存在对象里（`data()` / `raw_data()` / `temperature()`） |
| 文档 | `README.md`、`templates/board_cfg.cpp`、`docs/bsp_test/` | 1.1 节加 SPI 要求；模块表 / 目录树加 `bsp_spi`、`device_bmi088`、`imu_definition`，删 `dm_imu`；新增 3.7 节（SPI / BMI088 实机测试）；模板加注释掉的 SPI / BMI088 示例；同步 `docs/bsp_test/` 副本 |

另：示例工程 `examples/dm_mc02`（开着 SPI）用改后的库编译通过（0 warning），未上板。

宿主工程：

- `app/board_cfg.*`：`bsp_spi2`（`.dma_buffer`）、两个片选设备、`imu_bmi088`，`board_init()` 各一行断言；`HAL_GPIO_EXTI_Callback` 转调 BMI088 数据就绪（PE10 / PE12）。
- `app/board_cfg.cpp`：`imu_bmi088` 加速度计量程第 6 轮起改为 ±24 g（L-BMI1）。
- `app/test/test_spi_imu.cpp`（新增）：H / S / I / P01~P05 自动用例、P06 人工晃动对比；加热监视任务；诊断记录（每秒时间线、静止时孤立不合理帧、接口失败）。
  L-BMI2 之后改为跟着库的自动刷新走（I06 / P01 / P02 / P05 / P06 等 `UPDATED_BIT` 读 `snapshot()`，临时对象传 `auto_update = false`，S03 / S05 的轮询计数改为下限判断）；只编译验证，未重跑。
- `Core/Inc/hardware_config.h`：加 SPI、BMI088 两段注释掉的配置项。
- `app/test/bsp_test.*`、根 `CMakeLists.txt`（加测试源文件）。
- `app/app_test.hpp`：新增 `BSP_TEST_SUITE_SPI_IMU`、`BSP_TEST_SPI_STRESS_SECONDS`、`BSP_TEST_SPI_STRESS_ONLY`；测完已恢复 `APP_BSP_TEST_ENABLED 0`、`BSP_TEST_SOAK_MINUTES 0`，
  UART / RS485 / CAN 组开关恢复为 1。正常固件 Debug / RelWithDebInfo 编译 0 warning。
- 烧写：全部用 EmberProbe `mcu-flash`（Debug 与 RelWithDebInfo 两个 ELF 路径各一次 24 h 授权）；`build/RelWithDebInfo` 为 -O2 另配的构建目录。


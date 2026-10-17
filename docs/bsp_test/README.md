# stm32_bsp 接口实机测试（2026-10-17）

> 本目录是宿主工程 `MC02_with_submodule` 的 `docs/bsp_test/` 的副本（2026-10-17）。文中的路径都以**宿主工程根目录**为准：
> `app/`、`Core/`、`CMakeLists.txt` 指宿主工程，`third_party/stm32_bsp/` 即本库。测试代码在宿主工程 `app/test/`，不在本库里。

对 submodule `third_party/stm32_bsp` 的 UART、RS485、CAN 接口在 DM-MC02 上做实机测试：常规用法、参数边界、满载、故障注入、长时间浸泡、物理插拔。
测试中发现的库问题当场修复并回归（库改动未提交，见 `changes.md`）；同时首次用 EmberProbe skill 烧写调试，与命令行方案对比（`flash_debug_eval.md`）。

| 文件 | 内容 |
| --- | --- |
| `uart_u7_u10.md` | UART U7 ⇄ U10：用例、方案、现象、结论，3 个专项诊断 |
| `rs485_u2_u3.md` | RS485 U2 ⇄ U3 |
| `can1_can2.md` | CAN1 ⇄ CAN2 |
| `changes.md` | 所有代码与配置改动（CubeMX、测试代码、库修复 L1~L13）及原因 |
| `flash_debug_eval.md` | EmberProbe 与 OpenOCD / GDB / CubeMX 命令行的对比与今后分工 |
| `raw/` | 各轮原始结果表 |

## 环境

- 板：DM-MC02（STM32H723VGT6，550 MHz），USB 供电。
- 接线：U7 TX PE8 → U10 RX PE2、U10 TX PE3 → U7 RX PE7；485 口 1（USART2）A/B ↔ 485 口 2（USART3）A/B；CAN1 H/L ↔ CAN2 H/L，**无终端电阻**。
- 软件：宿主工程 `a0fbcb3` + 工作区改动；submodule `97e138b` + 工作区改动；Arm GNU Toolchain 15.3.1；FreeRTOS（CMSIS_V2）；STM32Cube HAL H7 V1.11.6；CubeMX 6.18.1。
- 构建：Debug（-O0 -g3）为主；另配 `build/RelWithDebInfo`（-O2 -g）验证优化后的时序。
- 工具：EmberProbe 0.8.6 skill（烧写 / 读变量 / 配置 / 芯片信息 / CubeMX）、OpenOCD 0.12.0+dev（CMSIS-DAP）、arm-none-eabi-gdb。

## 怎么跑

1. 开关在 `app/app_test.hpp`：`APP_BSP_TEST_ENABLED 1`，按需打开 `BSP_TEST_SUITE_*`、`BSP_TEST_SOAK_MINUTES`、`BSP_TEST_PHYSICAL`、`BSP_TEST_DIAG_LOG`。
2. 编译、烧写，上电自动运行；结果在 `bsp_test_results[]`（`name / state / code / v[10] / err[4]`），汇总 `bsp_test_count / pass / fail / finished`。
   state：0 未运行 / 1 运行中 / 2 通过 / 3 失败 / 4 跳过 / 5 等待人工；`err[]` = 用例期间串口 NE / FE / ORE / 其他错误事件。
3. 物理插拔模式按 `bsp_test_prompt` 的提示操作（断开 / 接回由固件自己检测，各等 120 s）。
4. 测完把 `APP_BSP_TEST_ENABLED` 改回 0（当前状态）。

## 测试过程

| 轮 | 固件 | 结果 | 主要发现 |
| --- | --- | --- | --- |
| 0 | — | — | CubeMX 修正 RS485 两路（USART2 加 DMA / 中断、USART3 改 PD8/PD9） |
| 1~2 | 测试固件初版 | — | 测试代码问题若干（BRR 写 0 等）；U01 基本收发就丢帧 |
| 3 | 基线（库未改） | 27 / 47 | 运行中 `init()` 毁实例；没 DMA 也 init 成功；≥2 Mbps 连续流几乎全丢；921600 下静默丢帧；CAN DLC 尾部旧数据、远程帧进应用、满载丢帧 |
| 4 | 实验：测试侧开 UART FIFO | 20 / 36 | FIFO 解决 ≥2 Mbps 溢出；921600 的丢帧不受影响且**确定性复现** |
| 5 | 诊断：字节日志 | — | 定位为接收流缓冲区被 DMA 写满投递填满（诊断 1） |
| 6~7 | 库修复 L1~L11 | 37 / 47 → 43 / 49 | 大部分修复验证；运行中 `init()` 仍失败（DMA 句柄卡 ERROR）→ L2b；确认 CAN 队头阻塞 |
| 8~9 | +L9b、L12，4 Mbps 字节日志 | 40 / 48 → 45 / 52 | 队列模式发不出帧（L12）；HAL IDLE 计数竞争（诊断 2 → L13） |
| 10 | +L13 | **50 / 52** | UART、RS485 全部通过（4 Mbps 无损）；剩 C06c/d 设计限制 |
| 11 | -O2 + 浸泡 | 卡死 | -O2 下 U14 起步 9 s 内整机卡死，定位为 HAL `HAL_DMA_Abort()` 读-改-写竞争（诊断 3） |
| 12 | -O2 + TIM8 时基优先级 4 + 浸泡 5 min | **51 / 55**，16 min 无卡死 | 卡死缓解为偶发 5 ms 超时 + 少量丢数（U14 0.04%）；根治记入库 `undo.md` |
| 物理 | Debug | 3 / 3 | 拔插 UART RX、485 A 线、CAN_H 均自动恢复，恢复后无损 |

## 结论汇总

**通过（最终）**：UART 18 项（不含注入后复测与诊断用例；U14 浸泡在 -O2 下有 0.04% 丢失，见下）、RS485 9 项（R03 在 -O2 下偶发 1/1000 超时）、CAN 12 项（C06c/d 为设计限制）、系统检查、物理插拔 3 项。重点数据：

- UART 921600 双向满负荷 8000/8000、约 90 KB/s；连续流 921600~4 Mbps 全部无损（4 Mbps 约 203 KB/s）；帧错误 / 溢出 / DMA 停流 / TX 卡死注入后都能自恢复，复测无损。
- RS485 收发器发送时不回波；请求-应答往返 4.5 ms（115200）；冲突后自恢复；4 Mbps 连续流无损。
- CAN 无终端电阻 1 Mbps 双向无错；双向满载 2 s 无丢帧（修复后）；20 轮真实 Bus-Off 全部自动恢复（最长 102 ms）；对端离线只到 error-passive；-O2 浸泡 5 min 双向 30 万帧无丢失。

**发现并修复的库问题（13 项，`changes.md` 第 3 节）**：
UART 7 项（运行中 `init()` 失效、未配 DMA 不报错、未开 FIFO 导致高速溢出、接收流缓冲区过小、IDLE 计数竞争、卡死判定依赖 Config 波特率、`uart_irqn()` 缺 USART2/6）；
CAN 6 项（DLC 尾部旧数据、远程帧进入应用、回退丢帧无计数、接收队列与分发速率低于满载、TX 队列模式下发不出帧）。

**仍未解决 / 设计层面，已记录**：

| 问题 | 现状 | 建议 |
| --- | --- | --- |
| HAL V1.11.6 `HAL_DMA_Abort()` 读-改-写竞争，-O2 下在串口中断里卡死 | 宿主 TIM8 时基优先级改 4 后不卡死，命中时短暂丢数（-O2 浸泡 0.04%）；-O2 下 R03 偶发 1/1000 超时疑同源 | RX 改 DMA 循环模式（`third_party/stm32_bsp/undo.md`）；其他工程用本库前先检查 HAL 时基优先级 |
| CAN 总线被更高优先级流量占满时，本节点的周期帧堆满 TX 缓冲，高优先级帧也进不去（C06c/d） | 未改 | 发送节点同 ID 只保留最新值；总线负载规划 |
| `printf()` 入队等 10 ms，满时发出半行并返回 TIMEOUT（U07） | 文档行为 | 调用方检查返回值或用 `send()` |
| 卡死恢复中止发送时丢 DMA FIFO 与 UART TX FIFO 里的字节（U12b） | 已知限制，已写入文档 | 只影响故障恢复路径 |

**测试未覆盖**：CAN 终端电阻存在时的对比；FDCAN3、其他串口；D-Cache 开启；-O2 下的物理插拔。

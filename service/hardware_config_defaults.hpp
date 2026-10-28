/**
 * @file hardware_config_defaults.hpp
 * @author ChoseB
 * @brief 库的可配置项：先读用户的 hardware_config.h（有的话），没配的项用这里的默认值，并检查取值范围
 * @version 0.1
 * @date 2026-10-17
 *
 * @copyright Copyright (c) 2026
 *
 * @details 用户配置文件：
 *          - 文件名固定为 hardware_config.h，放在宿主工程任一包含目录里即可，推荐 Core/Inc/
 *            （和 FreeRTOSConfig.h、stm32h7xx_hal_conf.h 放一起；CubeMX 重新生成不会删它）；
 *          - 只写想改的项，其余用默认值；模板见 templates/hardware_config.h（列出全部项与说明）；
 *          - 没有这个文件也能编译，全部用默认值。配置阶段 CMake 会打印找到的是哪个文件。
 *
 *          库内所有 .cpp 都经由本文件取配置，不要在库里直接包含 hardware_config.h。
 *
 * @note 本文件只放宏（C / C++ 都能包含）；取值检查用 static_assert，只在 C++ 下生效。
 */

#ifndef __HARDWARE_CONFIG_DEFAULTS_HPP__
#define __HARDWARE_CONFIG_DEFAULTS_HPP__

/* ==================== 用户配置 ==================== */

#if defined(__has_include)
#  if __has_include("hardware_config.h")
#    include "hardware_config.h"
#    define HARDWARE_CONFIG_FOUND 1 ///< 找到并使用了用户的 hardware_config.h
#  endif
#endif
#ifndef HARDWARE_CONFIG_FOUND
#  define HARDWARE_CONFIG_FOUND 0
#endif

/* ==================== 库任务（system_init() 创建） ==================== */

/** @brief 系统维护任务 sys_task 的优先级：高于业务任务，低于 CAN 收发任务 */
#ifndef SYS_TASK_PRIORITY
#  define SYS_TASK_PRIORITY (tskIDLE_PRIORITY + 7)
#endif

/** @brief sys_task 栈深（字） */
#ifndef SYS_TASK_STACK
#  define SYS_TASK_STACK 256
#endif

/** @brief CAN 收 / 发任务的优先级：最高，1 kHz 节拍内必须跑完 */
#ifndef CAN_TASK_PRIORITY
#  define CAN_TASK_PRIORITY (tskIDLE_PRIORITY + 8)
#endif

/** @brief CAN 收 / 发任务栈深（字）：接收节点回调在 can_rx_task 里同步执行，回调用栈多就加大 */
#ifndef CAN_TASK_STACK
#  define CAN_TASK_STACK 512
#endif

/* ==================== CAN ==================== */

/** @brief BspCan 登记表容量（STM32H7 最多 FDCAN1~3） */
#ifndef CAN_MAX_INSTANCES
#  define CAN_MAX_INSTANCES 3
#endif

/** @brief 每路 CAN 的接收队列深度（帧）：ISR 写入，can_rx_task 每毫秒取走 */
#ifndef CAN_RX_QUEUE_FRAMES
#  define CAN_RX_QUEUE_FRAMES 16 // 1 Mbps 满载约 8~9 帧/ms，can_rx_task 每 1 ms 取一次，8 帧会溢出（2026-10-17 实测）
#endif

/** @brief 每路 CAN 的回退队列深度（帧）：没有接收节点认领的帧，供 BspCan::receive() 读取 */
#ifndef CAN_RX_RETURN_FRAMES
#  define CAN_RX_RETURN_FRAMES 16 // 应不小于 CAN_RX_DISPATCH_PER_TICK：一轮分发可能一次推进这么多帧
#endif
#ifndef CAN_RX_DISPATCH_PER_TICK
#  define CAN_RX_DISPATCH_PER_TICK 16 // can_rx_task 每 1 ms 每条总线最多分发的帧数（原写死 8，低于 1 Mbps 满载帧率）
#endif

/** @brief 每路 CAN 的发送队列深度（帧） */
#ifndef CAN_TX_QUEUE_FRAMES
#  define CAN_TX_QUEUE_FRAMES 8
#endif

/** @brief 接收节点总数上限（全部总线合计） */
#ifndef CAN_RX_NODE_MAX
#  define CAN_RX_NODE_MAX 50
#endif

/** @brief 发送节点总数上限（全部总线合计） */
#ifndef CAN_TX_NODE_MAX
#  define CAN_TX_NODE_MAX 50
#endif

/**
 * @brief 控制指令有效期 (ms)：发送槽位超过这么久没被 FillData() 刷新就清零
 * @note 防止控制任务卡死 / 被删后电机一直按最后一次的电流转；100 ms ≈ 20 次保底心跳（2026-10-16 开发者定）。
 */
#ifndef CAN_CMD_EXPIRE_MS
#  define CAN_CMD_EXPIRE_MS 100
#endif

/** @brief 保底心跳 (ms)：发送节点即使没有凑齐所有槽位，也至少这么久整帧发一次 */
#ifndef CAN_TX_HEARTBEAT_MS
#  define CAN_TX_HEARTBEAT_MS 5
#endif

/** @brief Bus-Off 恢复的重试间隔 (ms)：硬件仍停在 INIT 时，隔这么久再请求一次退出 INIT / 取消旧发送 */
#ifndef CAN_BUSOFF_RETRY_MS
#  define CAN_BUSOFF_RETRY_MS 100
#endif

/* ==================== UART ==================== */

/** @brief BspUart 登记表容量（STM32H7 最多 11 路 U(S)ART 含 LPUART1，默认留 1 个余量） */
#ifndef UART_MAX_INSTANCES
#  define UART_MAX_INSTANCES 12
#endif

/**
 * @brief 串口补救巡检周期 (ms)：sys_task 每隔这么久对全部串口做一次 tx_recover() / rx_recover()
 * @note tx_recover() 要连续两次巡检都看到“通道空闲却有数据”才动手，所以发送卡死最迟约 2 个周期后续发。
 */
#ifndef UART_PATROL_PERIOD_MS
#  define UART_PATROL_PERIOD_MS 10
#endif

/** @brief BspUart::printf() 取格式化缓冲锁、以及入队的等待上限 (ms)；超时返回错误，不长时间挂住调用任务 */
#ifndef UART_ENABLE_FIFO
#  define UART_ENABLE_FIFO 1 // BspUart::init() 打开 UART 硬件 FIFO（16 字节）；0 = 保持 CubeMX 的设置
#endif
#ifndef UART_PRINTF_TIMEOUT_MS
#  define UART_PRINTF_TIMEOUT_MS 10
#endif

/* ==================== SPI ==================== */

/** @brief BspSpi 登记表容量（STM32H723 有 SPI1~6） */
#ifndef SPI_MAX_INSTANCES
#  define SPI_MAX_INSTANCES 6
#endif

/**
 * @brief 每个 BspSpi 实例的 DMA 弹跳缓冲（字节，收发各一份）
 * @note 调用方缓冲在 DTCM（任务栈、默认 .bss）时经它中转；更长的传输按它分段，片选全程保持。
 */
#ifndef SPI_DMA_BUFFER_SIZE
#  define SPI_DMA_BUFFER_SIZE 256
#endif

/** @brief BspSpi / BspSpiDevice 传输接口的默认超时 (ms)：等总线锁、以及每段等 DMA 完成 */
#ifndef SPI_TRANSFER_TIMEOUT_MS
#  define SPI_TRANSFER_TIMEOUT_MS 10
#endif

/* ==================== BMI088 ==================== */

/** @brief DeviceBmi088 自动刷新任务的优先级：低于库的 sys / CAN 任务，高于一般业务任务 */
#ifndef BMI088_TASK_PRIORITY
#  define BMI088_TASK_PRIORITY (tskIDLE_PRIORITY + 6)
#endif

/** @brief DeviceBmi088 自动刷新任务栈深（字） */
#ifndef BMI088_TASK_STACK
#  define BMI088_TASK_STACK 256
#endif

/* ==================== 取值检查 ==================== */

#ifdef __cplusplus
static_assert(SYS_TASK_STACK >= 128 && CAN_TASK_STACK >= 128, "hardware_config.h: 库任务栈深至少 128 字");
static_assert(CAN_MAX_INSTANCES >= 1 && CAN_MAX_INSTANCES <= 3, "hardware_config.h: CAN_MAX_INSTANCES 取 1~3（STM32H7 只有 FDCAN1~3）");
static_assert(CAN_RX_QUEUE_FRAMES >= 1 && CAN_RX_RETURN_FRAMES >= 1 && CAN_TX_QUEUE_FRAMES >= 1, "hardware_config.h: CAN 队列深度至少 1 帧");
static_assert(CAN_RX_DISPATCH_PER_TICK >= 1 && CAN_RX_DISPATCH_PER_TICK <= 64, "hardware_config.h: CAN_RX_DISPATCH_PER_TICK 取 1~64");
static_assert(CAN_RX_NODE_MAX >= 1 && CAN_RX_NODE_MAX <= 255, "hardware_config.h: CAN_RX_NODE_MAX 取 1~255（计数为 uint8_t）");
static_assert(CAN_TX_NODE_MAX >= 1 && CAN_TX_NODE_MAX <= 255, "hardware_config.h: CAN_TX_NODE_MAX 取 1~255（计数为 uint8_t）");
static_assert(CAN_TX_HEARTBEAT_MS >= 1, "hardware_config.h: CAN_TX_HEARTBEAT_MS 至少 1 ms");
static_assert(CAN_CMD_EXPIRE_MS > CAN_TX_HEARTBEAT_MS, "hardware_config.h: 指令有效期必须长于保底心跳，否则每帧之间就会被清零");
static_assert(CAN_BUSOFF_RETRY_MS >= 1, "hardware_config.h: CAN_BUSOFF_RETRY_MS 至少 1 ms");
static_assert(UART_MAX_INSTANCES >= 1 && UART_MAX_INSTANCES <= 255, "hardware_config.h: UART_MAX_INSTANCES 取 1~255");
static_assert(UART_PATROL_PERIOD_MS >= 1, "hardware_config.h: UART_PATROL_PERIOD_MS 至少 1 ms");
static_assert(UART_PRINTF_TIMEOUT_MS >= 1, "hardware_config.h: UART_PRINTF_TIMEOUT_MS 至少 1 ms");
static_assert(SPI_MAX_INSTANCES >= 1 && SPI_MAX_INSTANCES <= 6, "hardware_config.h: SPI_MAX_INSTANCES 取 1~6（STM32H7 只有 SPI1~6）");
static_assert(SPI_DMA_BUFFER_SIZE >= 16 && SPI_DMA_BUFFER_SIZE <= 65535, "hardware_config.h: SPI_DMA_BUFFER_SIZE 取 16~65535（单次 DMA 最多 65535 字节）");
static_assert(SPI_TRANSFER_TIMEOUT_MS >= 1, "hardware_config.h: SPI_TRANSFER_TIMEOUT_MS 至少 1 ms");
static_assert(BMI088_TASK_STACK >= 128, "hardware_config.h: BMI088_TASK_STACK 至少 128 字");
#endif

#endif // __HARDWARE_CONFIG_DEFAULTS_HPP__

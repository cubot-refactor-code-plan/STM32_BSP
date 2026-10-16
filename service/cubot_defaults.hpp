/**
 * @file cubot_defaults.hpp
 * @author ChoseB
 * @brief 库的可配置项：先读用户的 cubot_config.h（有的话），没配的项用这里的默认值，并检查取值范围
 * @version 0.1
 * @date 2026-10-17
 *
 * @copyright Copyright (c) 2026
 *
 * @details 用户配置文件：
 *          - 文件名固定为 cubot_config.h，放在宿主工程任一包含目录里即可，推荐 Core/Inc/
 *            （和 FreeRTOSConfig.h、stm32h7xx_hal_conf.h 放一起；CubeMX 重新生成不会删它）；
 *          - 只写想改的项，其余用默认值；模板见 templates/cubot_config.h（列出全部项与说明）；
 *          - 没有这个文件也能编译，全部用默认值。配置阶段 CMake 会打印找到的是哪个文件。
 *
 *          库内所有 .cpp 都经由本文件取配置，不要在库里直接包含 cubot_config.h。
 *
 * @note 本文件只放宏（C / C++ 都能包含）；取值检查用 static_assert，只在 C++ 下生效。
 */

#ifndef __CUBOT_DEFAULTS_HPP__
#define __CUBOT_DEFAULTS_HPP__

/* ==================== 用户配置 ==================== */

#if defined(__has_include)
#  if __has_include("cubot_config.h")
#    include "cubot_config.h"
#    define CUBOT_USER_CONFIG_FOUND 1 ///< 找到并使用了用户的 cubot_config.h
#  endif
#endif
#ifndef CUBOT_USER_CONFIG_FOUND
#  define CUBOT_USER_CONFIG_FOUND 0
#endif

/* ==================== 库任务（cubot_init() 创建） ==================== */

/** @brief 系统维护任务 cubot_sys_task 的优先级：高于业务任务，低于 CAN 收发任务 */
#ifndef CUBOT_SYS_TASK_PRIORITY
#  define CUBOT_SYS_TASK_PRIORITY (tskIDLE_PRIORITY + 7)
#endif

/** @brief cubot_sys_task 栈深（字） */
#ifndef CUBOT_SYS_TASK_STACK
#  define CUBOT_SYS_TASK_STACK 256
#endif

/** @brief CAN 收 / 发任务的优先级：最高，1 kHz 节拍内必须跑完 */
#ifndef CUBOT_CAN_TASK_PRIORITY
#  define CUBOT_CAN_TASK_PRIORITY (tskIDLE_PRIORITY + 8)
#endif

/** @brief CAN 收 / 发任务栈深（字）：接收节点回调在 cubot_can_rx_task 里同步执行，回调用栈多就加大 */
#ifndef CUBOT_CAN_TASK_STACK
#  define CUBOT_CAN_TASK_STACK 512
#endif

/* ==================== CAN ==================== */

/** @brief BspCan 登记表容量（STM32H7 最多 FDCAN1~3） */
#ifndef CUBOT_CAN_MAX_INSTANCES
#  define CUBOT_CAN_MAX_INSTANCES 3
#endif

/** @brief 每路 CAN 的接收队列深度（帧）：ISR 写入，cubot_can_rx_task 每毫秒取走 */
#ifndef CUBOT_CAN_RX_QUEUE_FRAMES
#  define CUBOT_CAN_RX_QUEUE_FRAMES 8
#endif

/** @brief 每路 CAN 的回退队列深度（帧）：没有接收节点认领的帧，供 BspCan::receive() 读取 */
#ifndef CUBOT_CAN_RX_RETURN_FRAMES
#  define CUBOT_CAN_RX_RETURN_FRAMES 8
#endif

/** @brief 每路 CAN 的发送队列深度（帧） */
#ifndef CUBOT_CAN_TX_QUEUE_FRAMES
#  define CUBOT_CAN_TX_QUEUE_FRAMES 8
#endif

/** @brief 接收节点总数上限（全部总线合计） */
#ifndef CUBOT_CAN_RX_NODE_MAX
#  define CUBOT_CAN_RX_NODE_MAX 50
#endif

/** @brief 发送节点总数上限（全部总线合计） */
#ifndef CUBOT_CAN_TX_NODE_MAX
#  define CUBOT_CAN_TX_NODE_MAX 50
#endif

/**
 * @brief 控制指令有效期 (ms)：发送槽位超过这么久没被 FillData() 刷新就清零
 * @note 防止控制任务卡死 / 被删后电机一直按最后一次的电流转；100 ms ≈ 20 次保底心跳（2026-10-16 开发者定）。
 */
#ifndef CUBOT_CAN_CMD_EXPIRE_MS
#  define CUBOT_CAN_CMD_EXPIRE_MS 100
#endif

/** @brief 保底心跳 (ms)：发送节点即使没有凑齐所有槽位，也至少这么久整帧发一次 */
#ifndef CUBOT_CAN_TX_HEARTBEAT_MS
#  define CUBOT_CAN_TX_HEARTBEAT_MS 5
#endif

/** @brief Bus-Off 恢复的重试间隔 (ms)：硬件仍停在 INIT 时，隔这么久再请求一次退出 INIT / 取消旧发送 */
#ifndef CUBOT_CAN_BUSOFF_RETRY_MS
#  define CUBOT_CAN_BUSOFF_RETRY_MS 100
#endif

/* ==================== UART ==================== */

/** @brief BspUart 登记表容量（STM32H7 最多 11 路 U(S)ART 含 LPUART1，默认留 1 个余量） */
#ifndef CUBOT_UART_MAX_INSTANCES
#  define CUBOT_UART_MAX_INSTANCES 12
#endif

/**
 * @brief 串口补救巡检周期 (ms)：cubot_sys_task 每隔这么久对全部串口做一次 tx_recover() / rx_recover()
 * @note tx_recover() 要连续两次巡检都看到“通道空闲却有数据”才动手，所以发送卡死最迟约 2 个周期后续发。
 */
#ifndef CUBOT_UART_PATROL_PERIOD_MS
#  define CUBOT_UART_PATROL_PERIOD_MS 10
#endif

/** @brief BspUart::printf() 取格式化缓冲锁、以及入队的等待上限 (ms)；超时返回错误，不长时间挂住调用任务 */
#ifndef CUBOT_UART_PRINTF_TIMEOUT_MS
#  define CUBOT_UART_PRINTF_TIMEOUT_MS 10
#endif

/* ==================== 取值检查 ==================== */

#ifdef __cplusplus
static_assert(CUBOT_SYS_TASK_STACK >= 128 && CUBOT_CAN_TASK_STACK >= 128, "cubot_config.h: 库任务栈深至少 128 字");
static_assert(CUBOT_CAN_MAX_INSTANCES >= 1 && CUBOT_CAN_MAX_INSTANCES <= 3, "cubot_config.h: CUBOT_CAN_MAX_INSTANCES 取 1~3（STM32H7 只有 FDCAN1~3）");
static_assert(CUBOT_CAN_RX_QUEUE_FRAMES >= 1 && CUBOT_CAN_RX_RETURN_FRAMES >= 1 && CUBOT_CAN_TX_QUEUE_FRAMES >= 1, "cubot_config.h: CAN 队列深度至少 1 帧");
static_assert(CUBOT_CAN_RX_NODE_MAX >= 1 && CUBOT_CAN_RX_NODE_MAX <= 255, "cubot_config.h: CUBOT_CAN_RX_NODE_MAX 取 1~255（计数为 uint8_t）");
static_assert(CUBOT_CAN_TX_NODE_MAX >= 1 && CUBOT_CAN_TX_NODE_MAX <= 255, "cubot_config.h: CUBOT_CAN_TX_NODE_MAX 取 1~255（计数为 uint8_t）");
static_assert(CUBOT_CAN_TX_HEARTBEAT_MS >= 1, "cubot_config.h: CUBOT_CAN_TX_HEARTBEAT_MS 至少 1 ms");
static_assert(CUBOT_CAN_CMD_EXPIRE_MS > CUBOT_CAN_TX_HEARTBEAT_MS, "cubot_config.h: 指令有效期必须长于保底心跳，否则每帧之间就会被清零");
static_assert(CUBOT_CAN_BUSOFF_RETRY_MS >= 1, "cubot_config.h: CUBOT_CAN_BUSOFF_RETRY_MS 至少 1 ms");
static_assert(CUBOT_UART_MAX_INSTANCES >= 1 && CUBOT_UART_MAX_INSTANCES <= 255, "cubot_config.h: CUBOT_UART_MAX_INSTANCES 取 1~255");
static_assert(CUBOT_UART_PATROL_PERIOD_MS >= 1, "cubot_config.h: CUBOT_UART_PATROL_PERIOD_MS 至少 1 ms");
static_assert(CUBOT_UART_PRINTF_TIMEOUT_MS >= 1, "cubot_config.h: CUBOT_UART_PRINTF_TIMEOUT_MS 至少 1 ms");
#endif

#endif // __CUBOT_DEFAULTS_HPP__

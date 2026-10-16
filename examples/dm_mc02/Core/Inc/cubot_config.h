/**
 * @file cubot_config.h
 * @brief cubot_bsp 的用户配置——DM-MC02 示例（由 templates/cubot_config.h 复制）
 *
 * @note 本板 3 路 CAN、8 路串口，库的默认值都够用，所以下面全部保持注释；
 *       放在这里是为了示范配置文件的位置（Core/Inc/，与 FreeRTOSConfig.h 同目录）。
 *
 * @details - 放在 stm32cubemx 目标的包含目录里（Core/Inc/ 就是），库和你的代码才会看到同一份配置；
 *            CMake 配置阶段会打印“cubot_bsp: 使用配置 <路径>”。
 *          - 没写的项用库内默认值（service/cubot_defaults.hpp，下面注释里的数字就是默认值）。
 *          - 取值越界在编译时报 static_assert；登记表 / 节点数不够用时在 init() / cubot_start() 断言停机。
 *          - 只放宏，C / C++ 都能包含；优先级可以写成 tskIDLE_PRIORITY + n（用到的地方已包含 FreeRTOS.h）。
 */

#ifndef CUBOT_CONFIG_H
#define CUBOT_CONFIG_H

/* ==================== 库任务（cubot_init() 创建） ==================== */

// #define CUBOT_SYS_TASK_PRIORITY     (tskIDLE_PRIORITY + 7) /* 系统维护：Bus-Off 恢复、串口补救、Online 计时；要高于业务任务 */
// #define CUBOT_SYS_TASK_STACK        256                    /* 字 */
// #define CUBOT_CAN_TASK_PRIORITY     (tskIDLE_PRIORITY + 8) /* CAN 收 / 发任务：最高，1 kHz 节拍内必须跑完 */
// #define CUBOT_CAN_TASK_STACK        512                    /* 字；接收节点回调在 CAN 接收任务里执行，回调重就加大 */

/* ==================== CAN ==================== */

// #define CUBOT_CAN_MAX_INSTANCES     3    /* BspCan 实例上限（1~3） */
// #define CUBOT_CAN_RX_QUEUE_FRAMES   8    /* 每路接收队列（帧），ISR → CAN 接收任务 */
// #define CUBOT_CAN_RX_RETURN_FRAMES  8    /* 每路回退队列（帧），没有接收节点认领的帧，BspCan::receive() 读取 */
// #define CUBOT_CAN_TX_QUEUE_FRAMES   8    /* 每路发送队列（帧） */
// #define CUBOT_CAN_RX_NODE_MAX       50   /* 接收节点总数上限（全部总线合计，≤255） */
// #define CUBOT_CAN_TX_NODE_MAX       50   /* 发送节点总数上限（全部总线合计，≤255） */
// #define CUBOT_CAN_CMD_EXPIRE_MS     100  /* 控制指令有效期：发送槽位这么久没 FillData() 就清零（必须大于心跳） */
// #define CUBOT_CAN_TX_HEARTBEAT_MS   5    /* 保底心跳：发送节点至少这么久整帧发一次 */
// #define CUBOT_CAN_BUSOFF_RETRY_MS   100  /* Bus-Off 恢复的重试间隔 */

/* ==================== UART ==================== */

// #define CUBOT_UART_MAX_INSTANCES    12   /* BspUart 实例上限 */
// #define CUBOT_UART_PATROL_PERIOD_MS 10   /* 串口补救巡检周期：卡死的发送约 2 个周期内续发，停摆的接收 1 个周期内重建 */
// #define CUBOT_UART_PRINTF_TIMEOUT_MS 10  /* BspUart::printf() 取锁 / 入队的等待上限 */

#endif /* CUBOT_CONFIG_H */

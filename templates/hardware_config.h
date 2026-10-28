/**
 * @file hardware_config.h
 * @brief stm32_bsp 的用户配置（模板）：复制到宿主工程的 Core/Inc/，只取消注释想改的项
 *
 * @details - 放在 stm32cubemx 目标的包含目录里（Core/Inc/ 就是），库和你的代码才会看到同一份配置；
 *            CMake 配置阶段会打印“stm32_bsp: 使用配置 <路径>”。
 *          - 没写的项用库内默认值（service/hardware_config_defaults.hpp，下面注释里的数字就是默认值）。
 *          - 取值越界在编译时报 static_assert；登记表 / 节点数不够用时在 init() / system_start() 断言停机。
 *          - 只放宏，C / C++ 都能包含；优先级可以写成 tskIDLE_PRIORITY + n（用到的地方已包含 FreeRTOS.h）。
 */

#ifndef HARDWARE_CONFIG_H
#define HARDWARE_CONFIG_H

/* ==================== 库任务（system_init() 创建） ==================== */

// #define SYS_TASK_PRIORITY          (tskIDLE_PRIORITY + 7) /* 系统维护：Bus-Off 恢复、串口补救、Online 计时；要高于业务任务 */
// #define SYS_TASK_STACK             256                    /* 字 */
// #define CAN_TASK_PRIORITY          (tskIDLE_PRIORITY + 8) /* CAN 收 / 发任务：最高，1 kHz 节拍内必须跑完 */
// #define CAN_TASK_STACK             512                    /* 字；接收节点回调在 CAN 接收任务里执行，回调重就加大 */

/* ==================== CAN ==================== */

// #define CAN_MAX_INSTANCES          3                      /* BspCan 实例上限（1~3） */
// #define CAN_RX_QUEUE_FRAMES        16                     /* 每路接收队列（帧），ISR → CAN 接收任务；1 Mbps 满载约 8~9 帧/ms */
// #define CAN_RX_RETURN_FRAMES       16                     /* 每路回退队列（帧），没有接收节点认领的帧，BspCan::receive() 读取；应 ≥ CAN_RX_DISPATCH_PER_TICK */
// #define CAN_RX_DISPATCH_PER_TICK   16                     /* CAN 接收任务每 1 ms 每路最多分发的帧数（1~64），要高于总线满载帧率 */
// #define CAN_TX_QUEUE_FRAMES        8                      /* 每路发送队列（帧） */
// #define CAN_RX_NODE_MAX            50                     /* 接收节点总数上限（全部总线合计，≤255） */
// #define CAN_TX_NODE_MAX            50                     /* 发送节点总数上限（全部总线合计，≤255） */
// #define CAN_CMD_EXPIRE_MS          100                    /* 控制指令有效期：发送槽位这么久没 FillData() 就清零（必须大于心跳） */
// #define CAN_TX_HEARTBEAT_MS        5                      /* 保底心跳：发送节点至少这么久整帧发一次 */
// #define CAN_BUSOFF_RETRY_MS        100                    /* Bus-Off 恢复的重试间隔 */

/* ==================== UART ==================== */

// #define UART_MAX_INSTANCES         12                     /* BspUart 实例上限 */
// #define UART_PATROL_PERIOD_MS      10                     /* 串口补救巡检周期：卡死的发送约 2 个周期内续发，停摆的接收 1 个周期内重建 */
// #define UART_PRINTF_TIMEOUT_MS     10                     /* BspUart::printf() 取锁 / 入队的等待上限 */

/* ==================== SPI ==================== */

// #define SPI_MAX_INSTANCES          6                      /* BspSpi 实例上限（1~6） */
// #define SPI_DMA_BUFFER_SIZE        256                    /* 每个 BspSpi 的 DMA 弹跳缓冲（字节，收发各一份）；调用方缓冲在 DTCM 时经它中转、按它分段 */
// #define SPI_TRANSFER_TIMEOUT_MS    10                     /* 传输接口默认超时：等总线锁、每段等 DMA 完成 */

/* ==================== BMI088 ==================== */

// #define BMI088_TASK_PRIORITY       (tskIDLE_PRIORITY + 6) /* DeviceBmi088 自动刷新任务：低于库的 sys / CAN 任务 */
// #define BMI088_TASK_STACK          256                    /* 字 */

#endif /* HARDWARE_CONFIG_H */

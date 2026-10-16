/**
 * @file board_cfg.hpp
 * @brief 板级实例配置（模板）：本工程用哪些外设、句柄、参数——全部在这里，库里不写死任何板级常量
 *
 * @details 用法：复制 board_cfg.hpp / board_cfg.cpp 到你的工程（例如 app/），按自己的 CubeMX 配置改：
 *          - 只包含工程里真有的 CubeMX 头文件（没开 FDCAN 就没有 fdcan.h，库里的 CAN 驱动也不会编译）；
 *          - 每个外设实例对应 CubeMX 里配好的一个句柄；串口实例必须放进 .dma_buffer 段；
 *          - board_init() 在 system_init() 之后、system_start() 之前调用（见 app_main.cpp）。
 *          完整示例见 examples/dm_mc02/app/board/。
 */

#ifndef __BOARD_CFG_HPP__
#define __BOARD_CFG_HPP__

// 本工程 CubeMX 生成的句柄与引脚宏（按实际开的外设增删）
#include "fdcan.h" // IWYU pragma: keep（hfdcan1）
#include "main.h"  // IWYU pragma: keep（引脚宏）
#include "tim.h"   // IWYU pragma: keep（htim12）
#include "usart.h" // IWYU pragma: keep（huart1）

#include "bsp_can.hpp"
#include "bsp_pwm.hpp"
#include "bsp_uart.hpp"
#include "device_buzzer.hpp"


/* ==================== 函数声明 ==================== */

/** @brief 板级初始化：各实例 init()（实例在 init() 时登记进库的登记表） */
void board_init(void);


/* ==================== 实例声明 ==================== */

extern BspCan       bsp_can1;   ///< CAN1
extern BspUart<128> bsp_uart1;  ///< 串口 1（128 字节收发缓冲）
extern BspPwm       bsp_pwm_buzzer; ///< 蜂鸣器 PWM 通道
extern DeviceBuzzer buzzer;     ///< 无源蜂鸣器（设备层，绑定 bsp_pwm_buzzer）

#endif // __BOARD_CFG_HPP__

/**
 * @file bsp_cfg.hpp
 * @author ALL
 * @brief 作为bsp层的总初始化和extern部分，更方便管理和移植
 * @version 0.1
 * @date 2026-04-18
 *
 * @copyright Copyright (c) 2026
 */

#ifndef __BSP_CFG_HPP__
#define __BSP_CFG_HPP__

// 本板的 CubeMX 句柄与引脚宏（库只依赖 stm32h7xx_hal.h，板级配置自己引入）
#include "fdcan.h" // IWYU pragma: keep（hfdcan1..3）
#include "main.h"  // IWYU pragma: keep（引脚宏）
#include "tim.h"   // IWYU pragma: keep（htim12）
#include "usart.h" // IWYU pragma: keep（huart1..10）

#include "bsp_pwm.hpp"
#include "bsp_can.hpp"
#include "bsp_gpio.hpp"
#include "bsp_key.hpp"
#include "bsp_uart.hpp"


/* ==================== 函数　声明 ==================== */

void bsp_init();


/* ==================== 全局　声明 ==================== */

///< CAN 一共三个 都初始化了
extern BspCan bsp_can1;
extern BspCan bsp_can2;
extern BspCan bsp_can3;


///< UART
extern BspUart<128> bsp_uart1;
extern BspUart<128> bsp_uart3;
extern BspUart<128> bsp_uart4;
extern BspUart<128> bsp_uart5;
extern BspUart<128> bsp_uart7;
extern BspUart<128> bsp_uart8;
extern BspUart<128> bsp_uart9;
extern BspUart<128> bsp_uart10;


///< GPIO 输出引脚（电源控制、片选等）
extern BspGpio power_24v_2;
extern BspGpio power_24v_1;
extern BspGpio power_5v;
extern BspGpio gyro_acc_cs;
extern BspGpio gyro_gyro_cs;
extern BspGpio dcmi_pwdn;
extern BspGpio btb_gpio;


///< 无源蜂鸣器 PWM：TIM12 CH2 PB15（由 Device 层 DeviceBuzzer 驱动）
extern BspPwm bsp_pwm_buzzer;

///< 按键
extern BspKey key_user;

#endif // __BSP_CFG_HPP__
#include "board_cfg.hpp"

#include "FreeRTOS.h" // IWYU pragma: keep (configASSERT)


/* ==================== 外设实例 ==================== */

/**
 * @brief CAN：句柄 + 名字（用于 RTOS 对象命名）
 * @note CubeMX 里：经典 CAN、RX FIFO0，开 FDCAN 中断 0 / 1。
 */
BspCan bsp_can1({&hfdcan1, "CAN1"});

/**
 * @brief 串口：句柄 + 是否启用发送 + 波特率（必须与 CubeMX 一致，发送卡死判定按它估算）
 * @note 必须放进 .dma_buffer：实例里就是 DMA 收发缓冲，H7 的 DMA1 / DMA2 访问不到默认 .bss 所在的 DTCM
 *       （放错了 init() 返回 BAD_ARG）。只收不发的串口第二个参数给 false，CubeMX 可以不配 TX DMA。
 *       CubeMX 里：RX DMA（Normal）+ TX DMA + 串口全局中断。
 */
__attribute__((section(".dma_buffer"))) BspUart<128> bsp_uart1({&huart1, true, 115200U});

/**
 * @brief PWM：句柄、通道、定时器时钟 (Hz)、PSC、ARR —— 从 CubeMX 的时钟树与 tim.c 抄
 * @note 示例为 DM-MC02 的 TIM12 CH2（275 MHz、PSC 23、ARR 1999）；换板必须改成自己的数。
 */
BspPwm bsp_pwm_buzzer({&htim12, TIM_CHANNEL_2, 275000000UL, 23U, 1999U});

/*
 * SPI 与片选设备（CubeMX 开了 SPI 才有；board_cfg.hpp 里加 #include "spi.h"、"bsp_spi.hpp"）：
 *   总线：句柄 + 是否用 DMA + 走 DMA 的最小长度（字节）；用 DMA 时必须放进 .dma_buffer（实例里有弹跳缓冲）
 *     __attribute__((section(".dma_buffer"))) BspSpi bsp_spi2({&hspi2, true, 16U});
 *   片选设备：{片选端口, 片选引脚, 读标志位, 读时地址后的无效字节数}
 *     BspSpiDevice bmi088_acc_spi(bsp_spi2, {GYRO_ACC_CS_GPIO_Port, GYRO_ACC_CS_Pin, 0x80U, 1U});
 *     BspSpiDevice bmi088_gyro_spi(bsp_spi2, {GYRO_GYRO_CS_GPIO_Port, GYRO_GYRO_CS_Pin, 0x80U, 0U});
 *   board_init() 里总线先于设备：configASSERT(bsp_spi2.init() == Status::OK); ……
 */


/* ==================== 设备实例 ==================== */

///< 无源蜂鸣器：构造即可用（PWM 通道由 board_init() 启动）
DeviceBuzzer buzzer(bsp_pwm_buzzer);

/*
 * 板载 BMI088（需要上面的两个片选设备；board_cfg.hpp 里加 #include "device_bmi088.hpp"）：
 *     DeviceBmi088 imu_bmi088(bmi088_acc_spi, bmi088_gyro_spi, {DeviceBmi088::AccRange::G24, DeviceBmi088::GyroRange::DPS2000});
 *   board_init() 里片选设备之后：configASSERT(imu_bmi088.init() == Status::OK);
 *   数据就绪中断由宿主的 HAL_GPIO_EXTI_Callback 转调 imu_bmi088.on_acc_drdy_isr() / on_gyro_drdy_isr()。
 */


/* ==================== 初始化 ==================== */

void board_init(void)
{
  // 每条 init() 单独一行断言：失败就地停机，fault_record 记下的行号直接指出是哪一路没起来
  configASSERT(bsp_can1.init() == Status::OK);
  configASSERT(bsp_uart1.init() == Status::OK);
  configASSERT(bsp_pwm_buzzer.init() == Status::OK);

  // 电机等 CAN 设备在这里 init()（会注册 CAN 收发节点），例如：
  //   configASSERT(chassis_lf.init() == Status::OK);   // DjiMotor<Motor3508> chassis_lf(bsp_can1, 1U);
}

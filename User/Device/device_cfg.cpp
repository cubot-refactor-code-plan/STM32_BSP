#include "device_cfg.hpp"

#include "FreeRTOS.h" // IWYU pragma: keep (configASSERT)


/* ==================== 全局实例化 ==================== */

///< LCD（SPI1 DMA，240x280 竖屏）
Lcd lcd;

/* ==================== 初始化函数 ==================== */

/**
 * @brief 设备层统一初始化
 *
 * @note LCD 在此初始化（复位、背光、寄存器配置，SPI1 DMA）。
 */
void device_init()
{
  /* LCD 初始化（SPI1 DMA） */
  configASSERT(lcd.init() == Status::OK); // 失败就地停机，fault_record 记下本行

  /* TODO: 电机业务接入时在此初始化（见上方注释） */
}

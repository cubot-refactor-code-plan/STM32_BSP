#include "device_cfg.hpp"

#include "FreeRTOS.h" // IWYU pragma: keep (configASSERT)
#include "bsp_cfg.hpp" // IWYU pragma: keep (bsp_pwm_buzzer)


/* ==================== 全局实例化 ==================== */

///< 板载无源蜂鸣器：构造即可用，没有 init()（PWM 通道由 bsp_init() 启动）
DeviceBuzzer buzzer(bsp_pwm_buzzer);


/* ==================== 初始化函数 ==================== */

/**
 * @brief 设备层统一初始化
 *
 * @note 每个设备的 init() 单独一行 configASSERT，失败就地停机，fault_record 记下行号。
 *       当前默认设备只有板载蜂鸣器（无 init()）。
 */
void device_init()
{
}

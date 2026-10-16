#include "device_cfg.hpp"

#include "FreeRTOS.h" // IWYU pragma: keep (configASSERT)


/* ==================== 初始化函数 ==================== */

/**
 * @brief 设备层统一初始化
 *
 * @note 每个设备的 init() 单独一行 configASSERT，失败就地停机，fault_record 记下行号。
 *       当前没有默认实例化的设备。
 */
void device_init()
{
}

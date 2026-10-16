#include "stm32h7xx_hal.h" // IWYU pragma: keep（HAL 模块开关）

// 宿主工程没在 CubeMX 里开 FDCAN（HAL_FDCAN_MODULE_ENABLED 未定义）时，本文件不参与编译
#ifdef HAL_FDCAN_MODULE_ENABLED

#include "bsp_can.hpp"

bool BspCan::tx_available() const
{
  return _hfdcan != nullptr && _hfdcan->State == HAL_FDCAN_STATE_BUSY &&
         !diagnostics.recovering &&
         (_hfdcan->Instance->PSR & FDCAN_PSR_BO) == 0U &&
         (_hfdcan->Instance->CCCR & FDCAN_CCCR_INIT) == 0U;
}

Status BspCan::service_recovery()
{
  if (_hfdcan == nullptr || _tx_message_buffer == nullptr ||
      _hfdcan->State != HAL_FDCAN_STATE_BUSY)
    return Status::NOT_INIT;
  const TickType_t now = xTaskGetTickCount();
  taskENTER_CRITICAL();
  const bool bus_off = (_hfdcan->Instance->PSR & FDCAN_PSR_BO) != 0U;
  if (bus_off && !diagnostics.recovering)
  {
    diagnostics.recovering = true;
    ++diagnostics.bus_off_events;
    _recovery_started = now;
    _recovery_attempted = now - pdMS_TO_TICKS(CUBOT_CAN_BUSOFF_RETRY_MS); // 首次立即尝试
    // 均为非阻塞缓冲；此临界区排除发送任务/ISR 与接收任务。
    xMessageBufferReset(_tx_message_buffer);
    xMessageBufferReset(_rx_message_buffer);
    // 取消硬件中旧控制帧，取消请求在重新启动后由硬件完成。
    _hfdcan->Instance->TXBCR = _hfdcan->Instance->TXBRP;
  }
  if (diagnostics.recovering)
  {
    // 只在硬件再次置 INIT 时启动恢复，避免反复打断 129x11 位恢复序列。
    if ((_hfdcan->Instance->CCCR & FDCAN_CCCR_INIT) != 0U &&
        now - _recovery_attempted >= pdMS_TO_TICKS(CUBOT_CAN_BUSOFF_RETRY_MS))
    {
      _hfdcan->Instance->TXBCR = _hfdcan->Instance->TXBRP;
      CLEAR_BIT(_hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
      _recovery_attempted = now;
      ++diagnostics.recovery_attempts;
    }
    // 先等旧发送请求彻底取消，才允许新的控制帧入队。
    if (!bus_off && (_hfdcan->Instance->CCCR & FDCAN_CCCR_INIT) == 0U &&
        _hfdcan->Instance->TXBRP == 0U)
    {
      diagnostics.recovering = false;
      ++diagnostics.recovery_successes;
      const TickType_t elapsed = now - _recovery_started;
      if (elapsed > diagnostics.recovery_max_ticks)
        diagnostics.recovery_max_ticks = elapsed;
    }
    else if (now - _recovery_attempted >= pdMS_TO_TICKS(CUBOT_CAN_BUSOFF_RETRY_MS))
    {
      // 尚未取消的发送请求继续请求取消，不忙等、不重新初始化外设。
      _hfdcan->Instance->TXBCR = _hfdcan->Instance->TXBRP;
    }
  }
  const bool recovering = diagnostics.recovering;
  taskEXIT_CRITICAL();
  return recovering ? Status::BUSY : Status::OK;
}

#endif // HAL_FDCAN_MODULE_ENABLED

#include "stm32h7xx_hal.h" // IWYU pragma: keep（HAL 模块开关）

// 宿主工程没在 CubeMX 里开 FDCAN（HAL_FDCAN_MODULE_ENABLED 未定义）时，本文件不参与编译
#ifdef HAL_FDCAN_MODULE_ENABLED

#include "bsp_can.hpp"
#include <string.h>


/* ==================== 中断回调函数 ==================== */

extern "C"
{
  /**
   * @brief FDCAN接收FIFO0中断回调
   */
  void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
  {
    // 按句柄查登记表；没有登记的 FDCAN（宿主工程自己管理的）直接忽略
    if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)
    {
      BspCan *can = BspCan::find(hfdcan);
      if (can != nullptr)
      {
        can->process_fifo0_isr();
      }
    }
  }

  /**
   * @brief FDCAN发送 FIFO 变空回调，用于继续排空软件发送缓冲区
   */
  void HAL_FDCAN_TxFifoEmptyCallback(FDCAN_HandleTypeDef *hfdcan)
  {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    BspCan *can = BspCan::find(hfdcan);
    if (can != nullptr)
    {
      can->trigger_tx_from_isr(&xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}


/* ==================== 实例登记表 ==================== */

BspCan  *BspCan::_registry[BspCan::MAX_INSTANCES] = {};
uint32_t BspCan::_registry_num                    = 0U;

uint32_t BspCan::instance_count()
{
  return _registry_num;
}

BspCan *BspCan::instance_at(uint32_t index)
{
  return (index < _registry_num) ? _registry[index] : nullptr;
}

/** @brief 线性查找（最多 3 项），ISR 中调用 */
BspCan *BspCan::find(const FDCAN_HandleTypeDef *hfdcan)
{
  for (uint32_t i = 0U; i < _registry_num; ++i)
  {
    if (_registry[i]->_hfdcan == hfdcan)
    {
      return _registry[i];
    }
  }
  return nullptr;
}

/** @brief 登记本实例；同一句柄只允许一个实例，重复登记自己无副作用 */
Status BspCan::register_self()
{
  if (_hfdcan == nullptr)
  {
    return Status::BAD_ARG;
  }
  for (uint32_t i = 0U; i < _registry_num; ++i)
  {
    if (_registry[i] == this)
    {
      return Status::OK;
    }
    if (_registry[i]->_hfdcan == _hfdcan)
    {
      return Status::BAD_ARG; // 两个实例绑定同一条总线：回调只会交给其中一个，属于配置错误
    }
  }
  if (_registry_num >= MAX_INSTANCES)
  {
    return Status::FULL;
  }
  _registry[_registry_num] = this;
  ++_registry_num;
  return Status::OK;
}


/* ==================== 类函数实现 ==================== */

BspCan::BspCan(const Config &cfg)

  : _tx_message_buffer(nullptr),
    rx_head(nullptr),
    tx_head(nullptr),
    _rx_message_buffer(nullptr),
    _rx_return_buffer(nullptr),
    _hfdcan(cfg.hfdcan),
    _name(cfg.name),
    _work_mode(cfg.mode)
{
}

BspCan::~BspCan()
{
  // 先从登记表摘除，回调与库任务就不会再碰到本实例
  for (uint32_t i = 0U; i < _registry_num; ++i)
  {
    if (_registry[i] == this)
    {
      for (uint32_t j = i + 1U; j < _registry_num; ++j)
      {
        _registry[j - 1U] = _registry[j];
      }
      --_registry_num;
      _registry[_registry_num] = nullptr;
      break;
    }
  }
  if (_hfdcan != nullptr)
  {
    HAL_FDCAN_Stop(_hfdcan);
  }
  if (_rx_message_buffer != nullptr)
  {
    vMessageBufferDelete(_rx_message_buffer);
    _rx_message_buffer = nullptr;
  }
  if (_tx_message_buffer != nullptr)
  {
    vMessageBufferDelete(_tx_message_buffer);
    _tx_message_buffer = nullptr;
  }
  if (_rx_return_buffer != nullptr)
  {
    vMessageBufferDelete(_rx_return_buffer);
    _rx_return_buffer = nullptr;
  }
}

Status BspCan::init()
{
  // 登记要早于打开中断：HAL 回调与库任务靠登记表找到本实例
  const Status reg = register_self();
  if (reg != Status::OK)
    return reg; // BAD_ARG：句柄为空或已被别的实例占用；FULL：登记表满

  // 创建接收消息缓冲区（每条消息另占 4 字节长度头；深度见 hardware_config_defaults.hpp）
  _rx_message_buffer = xMessageBufferCreate((sizeof(CanRxMsg) + 4) * CAN_RX_QUEUE_FRAMES);
  if (_rx_message_buffer == nullptr)
    return Status::IO_ERROR;

  // 分发任务写入、receive() 的唯一消费者读取。
  _rx_return_buffer = xMessageBufferCreate((sizeof(CanRxMsg) + 4) * CAN_RX_RETURN_FRAMES);
  if (_rx_return_buffer == nullptr)
  {
    vMessageBufferDelete(_rx_message_buffer);
    _rx_message_buffer = nullptr;
    return Status::IO_ERROR;
  }

  // 创建发送消息缓冲区
  _tx_message_buffer = xMessageBufferCreate((sizeof(CanTxMsg) + 4) * CAN_TX_QUEUE_FRAMES);
  if (_tx_message_buffer == nullptr)
  {
    vMessageBufferDelete(_rx_message_buffer);
    _rx_message_buffer = nullptr;
    vMessageBufferDelete(_rx_return_buffer);
    _rx_return_buffer = nullptr;
    return Status::IO_ERROR;
  }

  // 配置过滤器
  if (config_filter() != Status::OK)
    return Status::IO_ERROR;

  // 启动硬件
  if (start_hardware() != Status::OK)
    return Status::IO_ERROR;

  // 开启接收和发送 FIFO 变空中断。TX_COMPLETE 需要指定 TXBTIE buffer
  // 位；FIFO 模式下传入 BufferIndexes=0 不会产生发送完成回调。
  if (start_reception() != Status::OK)
    return Status::IO_ERROR;

  return Status::OK;
}

Status BspCan::send(uint32_t std_id, uint8_t *data)
{
  if (data == nullptr || std_id > 0x7FFU)
    return Status::BAD_ARG;
  if (_tx_message_buffer == nullptr)
    return Status::NOT_INIT;

  CanTxMsg txMsg;
  txMsg.std_id = std_id;
  memcpy(txMsg.data, data, 8);

  // 与 ISR 消费、sys_task 清队列串行化，避免 MessageBuffer 多读者竞争。
  taskENTER_CRITICAL();
  if (!tx_available())
  {
    taskEXIT_CRITICAL();
    return Status::BUSY;
  }
  const size_t sent = xMessageBufferSend(_tx_message_buffer, &txMsg, sizeof(txMsg), 0);
  trigger_tx();
  taskEXIT_CRITICAL();
  return sent == sizeof(txMsg) ? Status::OK : Status::FULL;
}

Status BspCan::receive(CanRxMsg *msg, uint32_t timeout)
{
  if (msg == nullptr)
    return Status::BAD_ARG;
  if (_rx_return_buffer == nullptr)
    return Status::NOT_INIT;

  size_t received = xMessageBufferReceive(_rx_return_buffer, msg, sizeof(CanRxMsg), timeout);
  return (received > 0) ? Status::OK : Status::TIMEOUT;
}

Status BspCan::config_filter()
{
  FDCAN_FilterTypeDef sFilterConfig;
  sFilterConfig.IdType       = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex  = 0;
  sFilterConfig.FilterType   = FDCAN_FILTER_RANGE;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1    = 0x000;
  sFilterConfig.FilterID2    = 0x7FF;

  if (HAL_FDCAN_ConfigFilter(_hfdcan, &sFilterConfig) != HAL_OK)
    return Status::IO_ERROR;

  // 远程帧直接由硬件拒收：本驱动只处理 8 字节数据帧，没有任何地方消费远程帧。
  // 注意 HAL 的 FDCAN_FILTER_REMOTE 意思是“远程帧按数据帧一样过滤”（即接收），拒收要用 FDCAN_REJECT_REMOTE ——
  // 原来用的 FILTER_REMOTE 让远程帧带着无意义的数据进了 receive() 的回退缓冲（2026-10-17 实测）。
  if (HAL_FDCAN_ConfigGlobalFilter(_hfdcan,
                                   FDCAN_ACCEPT_IN_RX_FIFO0,
                                   FDCAN_ACCEPT_IN_RX_FIFO0,
                                   FDCAN_REJECT_REMOTE,
                                   FDCAN_REJECT_REMOTE)
      != HAL_OK)
  {
    return Status::IO_ERROR;
  }
  return Status::OK;
}

Status BspCan::start_hardware()
{
  if (HAL_FDCAN_Start(_hfdcan) != HAL_OK)
    return Status::IO_ERROR;
  return Status::OK;
}

Status BspCan::start_reception()
{
  // 开启FIFO0新消息中断
  if (HAL_FDCAN_ActivateNotification(_hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    return Status::IO_ERROR;

  // 软件发送缓冲区积压时，在硬件 FIFO 发送完后继续排空。
  if (HAL_FDCAN_ActivateNotification(_hfdcan, FDCAN_IT_TX_FIFO_EMPTY, 0) != HAL_OK)
    return Status::IO_ERROR;

  return Status::OK;
}

/**
 * @brief 触发一次发送（任务上下文调用）
 */
/**
 * @brief 硬件 TX FIFO / 队列还有空位
 *
 * @note 用 TXFQS.TFQF（满标志）判断，FIFO 模式与队列模式都有效。原来用 HAL_FDCAN_GetTxFifoFreeLevel()
 *       读 TFFL，而 TFFL 在 Tx Queue 模式（CubeMX TxFifoQueueMode = Queue，TXBC.TFQM = 1）下恒读 0 ——
 *       配成队列模式时本驱动一帧都发不出去（2026-10-17 运行中切到队列模式实测）。
 */
bool BspCan::tx_hw_has_room() const
{
  return (_hfdcan->Instance->TXFQS & FDCAN_TXFQS_TFQF) == 0U;
}

void BspCan::trigger_tx()
{
  if (_tx_message_buffer == nullptr)
    return;

  taskENTER_CRITICAL();
  if (tx_available() && tx_hw_has_room())
  {
    CanTxMsg txMsg;
    size_t   len = xMessageBufferReceive(_tx_message_buffer, &txMsg, sizeof(CanTxMsg), 0);
    if (len > 0)
    {
      FDCAN_TxHeaderTypeDef txHeader;
      txHeader.Identifier          = txMsg.std_id;
      txHeader.IdType              = FDCAN_STANDARD_ID;
      txHeader.TxFrameType         = FDCAN_DATA_FRAME;
      txHeader.DataLength          = FDCAN_DLC_BYTES_8;
      txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
      txHeader.BitRateSwitch       = FDCAN_BRS_OFF;
      txHeader.FDFormat            = FDCAN_CLASSIC_CAN;
      txHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
      txHeader.MessageMarker       = 0;

      if (HAL_FDCAN_AddMessageToTxFifoQ(_hfdcan, &txHeader, txMsg.data) != HAL_OK)
        ++diagnostics.tx_dropped;
    }
  }
  taskEXIT_CRITICAL();
}

/**
 * @brief 触发一次发送（ISR上下文调用）
 * @param pxHigherPriorityTaskWoken 需初始化为pdFALSE，若唤醒高优先级任务则置为pdTRUE
 */
void BspCan::trigger_tx_from_isr(BaseType_t *pxHigherPriorityTaskWoken)
{
  if (_tx_message_buffer == nullptr)
    return;

  if (tx_available() && tx_hw_has_room())
  {
    CanTxMsg txMsg;
    size_t   len = xMessageBufferReceiveFromISR(_tx_message_buffer, &txMsg, sizeof(CanTxMsg), pxHigherPriorityTaskWoken);
    if (len > 0)
    {
      FDCAN_TxHeaderTypeDef txHeader;
      txHeader.Identifier          = txMsg.std_id;
      txHeader.IdType              = FDCAN_STANDARD_ID;
      txHeader.TxFrameType         = FDCAN_DATA_FRAME;
      txHeader.DataLength          = FDCAN_DLC_BYTES_8;
      txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
      txHeader.BitRateSwitch       = FDCAN_BRS_OFF;
      txHeader.FDFormat            = FDCAN_CLASSIC_CAN;
      txHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
      txHeader.MessageMarker       = 0;

      if (HAL_FDCAN_AddMessageToTxFifoQ(_hfdcan, &txHeader, txMsg.data) != HAL_OK)
        ++diagnostics.tx_dropped;
    }
  }
}

/**
 * @brief FIFO0 中断处理（从硬件FIFO读取并放入Message Buffer）
 */
void BspCan::process_fifo0_isr()
{
  if (_hfdcan == nullptr || _rx_message_buffer == nullptr)
    return;

  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  // HAL_FDCAN_GetRxMessage 按 DLC 查表拷贝 DLCtoBytes[] 个字节（DLC 9~15 → 12~64 字节），
  // 而经典 CAN 允许 DLC 为 9~15（数据仍是 8 字节），CanRxMsg::data 只有 8 字节。
  // 直接读进 rxMsg.data 会写穿中断栈：实测一帧 DLC=15 就让返回地址被帧数据覆盖、进 MemManage。
  // 所以先读进 64 字节缓冲，只把 DLC ≤ 8 的帧拷进 CanRxMsg。
  constexpr uint32_t CAN_RX_RAW_BYTES = 64U;
  uint8_t            raw[CAN_RX_RAW_BYTES];

  // 新消息中断可能合并多帧；读取本次积压快照，避免只取一帧导致 FIFO 长期满载。
  const uint32_t pending = HAL_FDCAN_GetRxFifoFillLevel(_hfdcan, FDCAN_RX_FIFO0);
  for (uint32_t i = 0; i < pending; ++i)
  {
    CanRxMsg rxMsg = {};
    if (HAL_FDCAN_GetRxMessage(_hfdcan, FDCAN_RX_FIFO0, &rxMsg.header, raw) != HAL_OK)
      break;

    // 只支持 8 字节经典帧：DLC > 8 的帧丢弃并计数（帧已从硬件 FIFO 取出，不会堵住 FIFO）
    if (rxMsg.header.DataLength > FDCAN_DLC_BYTES_8)
    {
      ++diagnostics.rx_len_drop;
      continue;
    }
    // 只拷 DLC 个字节：raw 是栈上的缓冲，DLC < 8 时后面是上一帧 / 其他函数留下的内容，
    // 原来固定拷 8 字节会把这些旧数据带进 data[DLC..7]（2026-10-17 实测）。rxMsg 已清零，尾部保持 0。
    static const uint32_t dlc_codes[9] = {FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2,
                                          FDCAN_DLC_BYTES_3, FDCAN_DLC_BYTES_4, FDCAN_DLC_BYTES_5,
                                          FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7, FDCAN_DLC_BYTES_8};
    uint32_t len = 0;
    while (len < 8U && dlc_codes[len] != rxMsg.header.DataLength)
      ++len;
    memcpy(rxMsg.data, raw, len);

    if (xMessageBufferSendFromISR(_rx_message_buffer, &rxMsg, sizeof(rxMsg),
                                  &xHigherPriorityTaskWoken) != sizeof(rxMsg))
      ++diagnostics.rx_dropped;
  }
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

#endif // HAL_FDCAN_MODULE_ENABLED

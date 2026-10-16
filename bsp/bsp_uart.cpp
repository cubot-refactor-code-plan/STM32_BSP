#include "stm32h7xx_hal.h" // IWYU pragma: keep（HAL 模块开关）

// 宿主工程没在 CubeMX 里开 UART（HAL_UART_MODULE_ENABLED 未定义）时，本文件不参与编译
#ifdef HAL_UART_MODULE_ENABLED

#include "bsp_uart.hpp"
#include "FreeRTOS.h"  // IWYU pragma: keep
#include <stdarg.h>
#include <stdio.h>


// ---------------- 模板实例化 ----------------

// 显式实例化：BUFFER_SIZE = 128
template class BspUart<128>;
template class BspUart<256>;


// ----------------


// ---------------- 超时换算 ----------------

namespace
{
/**
 * @brief 毫秒 → tick；portMAX_DELAY 原样保留（表示一直等）
 *
 * @note 本工程 FreeRTOS V10.6.2 的 pdMS_TO_TICKS 用 32 位 TickType_t 计算，
 *       pdMS_TO_TICKS(portMAX_DELAY) 会溢出成约 4 294 966 tick（≈71.6 分钟），不再是“一直等”。
 */
TickType_t ms_to_ticks(uint32_t ms)
{
  return (ms == portMAX_DELAY) ? portMAX_DELAY : pdMS_TO_TICKS(ms);
}

/**
 * @brief 查本工程 8 路串口对应的 NVIC 中断号；不认识的实例返回 false
 *
 * @note rx_recover() 要在任务里做「中止 → 重新启动」，期间必须挡住本串口的中断：
 *       否则 IDLE 中断可能在中止到一半时进 on_idle_isr() 重新启动接收，与任务侧的中止交错，
 *       实测会把 RX DMA 句柄留在 HAL_DMA_STATE_ERROR（见 repair_dma_handle()）。
 */
bool uart_irqn(const USART_TypeDef *instance, IRQn_Type *irqn)
{
  if (instance == USART1)  { *irqn = USART1_IRQn;  return true; }
  if (instance == USART3)  { *irqn = USART3_IRQn;  return true; }
  if (instance == UART4)   { *irqn = UART4_IRQn;   return true; }
  if (instance == UART5)   { *irqn = UART5_IRQn;   return true; }
  if (instance == UART7)   { *irqn = UART7_IRQn;   return true; }
  if (instance == UART8)   { *irqn = UART8_IRQn;   return true; }
  if (instance == UART9)   { *irqn = UART9_IRQn;   return true; }
  if (instance == USART10) { *irqn = USART10_IRQn; return true; }
  return false;
}

/**
 * @brief DMA 流已经停了、HAL 句柄却不是 READY 时，把句柄状态复位（只在恢复路径里调用）
 *
 * @return true=确实复位了一次
 *
 * @note 实测（2026-10-16，USART10 RX）：句柄一旦停在 HAL_DMA_STATE_ERROR，
 *       HAL_DMA_Start_IT() 一律返回 HAL_ERROR（ErrorCode = BUSY），HAL_DMA_Abort() 又因为
 *       状态不是 BUSY 直接返回、不复位状态 —— HAL 自己出不来，rx_recover() 每 10 ms 重试都失败。
 * @note 只在流的 EN 位已清时复位：EN 还在说明 DMA 真在搬运，这时只请求停流，下次巡检再复位。
 */
bool repair_dma_handle(DMA_HandleTypeDef *hdma)
{
  if (hdma == nullptr || hdma->State == HAL_DMA_STATE_READY || IS_DMA_STREAM_INSTANCE(hdma->Instance) == 0U)
  {
    return false;
  }
  if ((((DMA_Stream_TypeDef *)hdma->Instance)->CR & DMA_SxCR_EN) != 0U)
  {
    __HAL_DMA_DISABLE(hdma);
    return false;
  }
  hdma->ErrorCode = HAL_DMA_ERROR_NONE;
  hdma->State     = HAL_DMA_STATE_READY;
  __HAL_UNLOCK(hdma);
  return true;
}
} // namespace


// ---------------- HAL 回调分发 ----------------

extern "C"
{
  /**
   * @brief IDLE / TC 接收事件回调：按句柄查登记表，交给对应实例
   * @note 没有登记的串口（宿主工程自己管理的）直接忽略。
   */
  void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
  {
    BaseType_t   xHigherPriorityTaskWoken = pdFALSE;
    BspUartBase *uart                     = BspUartBase::find(huart);
    if (uart != nullptr)
    {
      uart->on_idle_isr(Size, &xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }

  /**
   * @brief TX Complete 回调：从流缓冲区取下一批数据继续发送
   * @note 只收不发的实例（transmit_enable = false）不会有 TX DMA，也就不会进这里。
   */
  void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
  {
    BaseType_t   xHigherPriorityTaskWoken = pdFALSE;
    BspUartBase *uart                     = BspUartBase::find(huart);
    if (uart != nullptr)
    {
      uart->start_transmission_from_isr(&xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }

  /**
   * @brief 错误回调：这里只累加计数，不做恢复
   * @note 恢复要调阻塞版 HAL 接口，中断里不能做；由库的巡检任务调 rx_recover() / tx_recover() 完成。
   */
  void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
  {
    BspUartBase *uart = BspUartBase::find(huart);
    if (uart != nullptr)
    {
      uart->on_error_isr();
    }
  }
}


// ----------------


// ---------------- 实例登记表 ----------------

BspUartBase *BspUartBase::_registry[BspUartBase::MAX_INSTANCES] = {};
uint32_t     BspUartBase::_registry_num                         = 0U;

/** @brief 析构时摘除：后面的实例前移，保持表紧凑 */
BspUartBase::~BspUartBase()
{
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
}

/** @brief 登记本实例；同一句柄只允许一个实例，重复登记自己无副作用 */
Status BspUartBase::register_self()
{
  if (_huart == nullptr)
  {
    return Status::BAD_ARG;
  }
  for (uint32_t i = 0U; i < _registry_num; ++i)
  {
    if (_registry[i] == this)
    {
      return Status::OK; // 重复 init()：已经登记过
    }
    if (_registry[i]->_huart == _huart)
    {
      return Status::BAD_ARG; // 两个实例绑定同一个串口：回调只会交给其中一个，属于配置错误
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

uint32_t BspUartBase::instance_count()
{
  return _registry_num;
}

BspUartBase *BspUartBase::instance_at(uint32_t index)
{
  return (index < _registry_num) ? _registry[index] : nullptr;
}

/** @brief 线性查找（最多十几项），ISR 中调用 */
BspUartBase *BspUartBase::find(const UART_HandleTypeDef *huart)
{
  for (uint32_t i = 0U; i < _registry_num; ++i)
  {
    if (_registry[i]->_huart == huart)
    {
      return _registry[i];
    }
  }
  return nullptr;
}


/**
 * @brief BspUart<BUFFER_SIZE> 成员函数定义
 *
 * 接收：IDLE 中断判定帧结束，DMA 普通模式收一包。
 * 发送：任务或 TX Complete 中断从流缓冲区取一包，交给 DMA 发出。
 * 线程安全：流缓冲区单写单读——写者只有持 _tx_write_lock 的 send()，读者是持 _tx_lock 的任务侧启动
 *           与 TX 完成中断（两者靠「通道是否空闲」互斥）；printf() 由 _printf_lock 保护格式化缓冲。
 */

// ----------------


// ---------------- 公有接口 ----------------

/** @brief 只做赋值，RTOS 资源与接收启动放在 init() */
template <size_t BUFFER_SIZE>
BspUart<BUFFER_SIZE>::BspUart(const Config &cfg) : BspUartBase(cfg.huart), _transmit_enable(cfg.transmit_enable), _baudrate(cfg.baudrate)
{
  // 构造函数只做赋值；运行时逻辑（FreeRTOS资源创建）推迟到 init()
}

/**
 * @brief 创建 FreeRTOS 对象并启动 IDLE 接收（须在STM32初始化之后调用）
 *
 * @return Status OK=成功；BAD_ARG=句柄为空；IO_ERROR=资源创建或接收启动失败
 *
 * @note 可重复调用：开头先 cleanup_resources() 释放上一次的资源，避免指针被覆盖造成泄漏。
 *       任一步失败都会走 abort_reception() + cleanup_resources()，不留下半初始化状态。
 */
template <size_t BUFFER_SIZE>
Status BspUart<BUFFER_SIZE>::init()
{
  // 句柄非法：后续所有 HAL 调用都不可用，提前拒绝（与 bsp_can::init() 一致）
  if (_huart == nullptr)
  {
    return Status::BAD_ARG;
  }

  // 可重复调用：先释放上一次残留的软件资源，避免二次 init() 直接覆盖指针造成泄漏
  // （与 bsp_can::init() 的可重入行为保持一致）
  cleanup_resources();

  // 创建接收流缓冲区
  _rx_stream_buffer = xStreamBufferCreate(BUFFER_SIZE, 1);
  if (_rx_stream_buffer == nullptr)
  {
    cleanup_resources();     // 清理已创建的资源
    return Status::IO_ERROR; // 流缓冲区创建失败
  }

  if (_transmit_enable)
  {
    // 创建发送流缓冲区
    _tx_stream_buffer = xStreamBufferCreate(BUFFER_SIZE, 1);
    if (_tx_stream_buffer == nullptr)
    {
      cleanup_resources();     // 清理已创建的资源
      return Status::IO_ERROR; // 发送流缓冲区创建失败
    }
  }
  else
  {
    _tx_stream_buffer = nullptr;
  }

  // TX 入队锁（流缓冲区只允许一个写者，多任务 send() 靠它排队）
  _tx_write_lock = xSemaphoreCreateMutex();
  if (_tx_write_lock == nullptr)
  {
    cleanup_resources();
    return Status::IO_ERROR;
  }

  // 发送状态：DMA 缓冲里没有未发完的数据
  _tx_chunk_off    = 0;
  _tx_chunk_len    = 0;
  _tx_chunk_active = false;

  // TX 启动锁（任务侧串行化「判忙 → 取包 → 启动 DMA」）
  _tx_lock = xSemaphoreCreateMutex();
  if (_tx_lock == nullptr)
  {
    cleanup_resources();     // 清理已创建的资源
    return Status::IO_ERROR; // 互斥量创建失败
  }

  // printf 锁（保护实例级共享的 _printf_buffer，消除并发 printf 互相覆盖）
  _printf_lock = xSemaphoreCreateMutex();
  if (_printf_lock == nullptr)
  {
    cleanup_resources();
    return Status::IO_ERROR; // printf 锁创建失败
  }

  // 打开 UART 硬件 FIFO。CubeMX 默认把它关掉，此时接收只有 RDR 一级缓冲：
  // 只要 DMA 晚一点来搬（多路 DMA 抢总线仲裁时就可能），下一个字节到达就 ORE 丢掉，
  // 整条接收流随之错位。FIFO 有 16 字节，能吸收这段搬运抖动。
  // (void)HAL_UARTEx_EnableFifoMode(_huart);

  // 先登记再开接收中断：HAL 回调靠登记表找到本实例
  const Status reg = register_self();
  if (reg != Status::OK)
  {
    cleanup_resources();
    return reg; // BAD_ARG：句柄已被别的实例占用；FULL：登记表满
  }

  // 启动接收（HAL 内部会清 IDLE 标志并使能 IDLE 中断；失败则释放资源并上报）
  if (!arm_reception())
  {
    abort_reception();       // 复位 RX 状态并关掉 IDLE 中断源
    cleanup_resources();     // 释放已创建的资源
    return Status::IO_ERROR; // 接收启动失败
  }

  return Status::OK; // 初始化成功
}

/** @brief 停掉 RX/TX DMA 并释放软件资源 */
template <size_t BUFFER_SIZE>
BspUart<BUFFER_SIZE>::~BspUart()
{
  // 句柄非法时无可停的外设，直接释放软件资源
  if (_huart == nullptr)
  {
    cleanup_resources();
    return;
  }

  _rx_should_run = false;
  HAL_UART_DMAStop(_huart); // 同时中止 RX 与 TX 的 DMA

  cleanup_resources();
}

/**
 * @brief 发送数据：写入发送流缓冲区并启动 DMA
 *
 * @param data       待发数据
 * @param size       字节数（不能超过流缓冲区容量 BUFFER_SIZE，否则调用方应自行分包）
 * @param timeout_ms 入队等待上限（毫秒）；portMAX_DELAY = 一直等到全部入队
 * @param written    实际入队的字节数（可为 nullptr）
 * @return Status OK=全部入队；TIMEOUT=仅部分入队；BAD_ARG=参数非法；IO_ERROR=未启用发送
 */
template <size_t BUFFER_SIZE>
Status BspUart<BUFFER_SIZE>::send(const uint8_t *data, size_t size, uint32_t timeout_ms, size_t *written)
{
  if (data == nullptr || size == 0)
  {
    return Status::BAD_ARG; // 参数非法
  }

  // 单次发送不能超过流缓冲区容量，否则 xStreamBufferSend 会永久阻塞（死锁）
  if (size > BUFFER_SIZE)
  {
    return Status::BAD_ARG; // 数据超长，调用方应分包发送
  }

  if (!_transmit_enable || _tx_stream_buffer == nullptr || _tx_write_lock == nullptr)
  {
    return Status::IO_ERROR; // 未启用发送或发送缓冲区未初始化
  }

  // 流缓冲区只允许一个写者：多个任务同时 send() 在这里排队，一次写入的字节不会和别人交错。
  // 等锁与等空间共用同一个超时（最坏约 2 × timeout_ms）。
  size_t bytes_written = 0;
  if (xSemaphoreTake(_tx_write_lock, ms_to_ticks(timeout_ms)) == pdTRUE)
  {
    bytes_written = xStreamBufferSend(_tx_stream_buffer, data, size, ms_to_ticks(timeout_ms));
    xSemaphoreGive(_tx_write_lock);
  }

  // 如果发送缓冲区中有数据，启动发送
  if (bytes_written > 0)
  {
    (void)start_transmission();
  }

  // 未全部入队说明有字节被丢弃（调用方通常只检查返回码），累加计数
  if (bytes_written < size)
  {
    _tx_drop_bytes += static_cast<uint32_t>(size - bytes_written);
  }

  if (written != nullptr)
  {
    *written = bytes_written;
  }

  return (bytes_written == size) ? Status::OK : Status::TIMEOUT;
}

/**
 * @brief 格式化输出（printf 风格，超长自动截断；仅任务上下文）
 *
 * @note 实例级的 _printf_buffer 由 _printf_lock 保护，多任务并发调用安全。
 *       入队等待固定为 10 ms，不随调用方传入，避免发送卡住时
 *       把调用任务长时间挂起；未全部入队时返回 Status::TIMEOUT（已入队部分照常发出）。
 */
template <size_t BUFFER_SIZE>
Status BspUart<BUFFER_SIZE>::printf(const char *fmt, ...)
{
  // 取锁等待上限 (ms)：避免某一个任务长期持有格式化缓冲
  constexpr uint32_t PRINTF_LOCK_TIMEOUT_MS = 10U;

  // 入队等待上限 (ms)：发送卡住时不把调用任务无限挂住
  constexpr uint32_t PRINTF_TX_TIMEOUT_MS = 10U;

  if (fmt == nullptr)
  {
    return Status::BAD_ARG; // 参数非法
  }

  if (!_transmit_enable || _tx_stream_buffer == nullptr || _printf_lock == nullptr)
  {
    return Status::IO_ERROR; // 未启用发送 / 未初始化
  }

  // _printf_buffer 是实例级共享缓冲：必须整体互斥，
  // 否则两个任务并发 printf 会在 vsnprintf 写一半时被对方覆盖。
  if (xSemaphoreTake(_printf_lock, pdMS_TO_TICKS(PRINTF_LOCK_TIMEOUT_MS)) != pdTRUE)
  {
    return Status::IO_ERROR;
  }

  va_list args;
  va_start(args, fmt);
  int len = vsnprintf(_printf_buffer, sizeof(_printf_buffer), fmt, args);
  va_end(args);

  if (len <= 0)
  {
    xSemaphoreGive(_printf_lock);
    return Status::BAD_ARG; // 格式化失败或空输出
  }

  // vsnprintf 返回的是期望写入的完整长度，实际写入可能被截断
  if (static_cast<size_t>(len) >= sizeof(_printf_buffer))
  {
    len = static_cast<int>(sizeof(_printf_buffer)) - 1;
  }

  // 注意：send() 内部依次取 _tx_write_lock、_tx_lock，取锁顺序固定（printf → write → tx），不会死锁。
  //       这里用有限超时：发送卡住时不把调用任务无限挂住。
  const Status st = send(reinterpret_cast<const uint8_t *>(_printf_buffer),
                         static_cast<size_t>(len),
                         PRINTF_TX_TIMEOUT_MS);
  xSemaphoreGive(_printf_lock);
  return st;
}

/**
 * @brief 从接收流缓冲区读取数据
 *
 * @param buffer   接收缓冲区
 * @param size     最多读取的字节数
 * @param timeout  超时时间（毫秒）；portMAX_DELAY = 一直等
 * @param received 实际读到的字节数（可为 nullptr）
 * @return Status OK=读到数据；TIMEOUT=无数据；BAD_ARG=参数非法；IO_ERROR=缓冲区未创建
 *
 * @note 字节流语义：不保证一次读到完整一帧，只保证有序、不重、不丢（除非流缓冲区满）。
 *       帧边界由协议层自行确定（长度前缀 / 定界符 / 校验和）。
 */
template <size_t BUFFER_SIZE>
Status BspUart<BUFFER_SIZE>::receive(uint8_t *buffer, size_t size, uint32_t timeout, size_t *received)
{
  if (buffer == nullptr || size == 0)
  {
    return Status::BAD_ARG; // 参数非法
  }

  if (_rx_stream_buffer == nullptr)
  {
    return Status::IO_ERROR; // 流缓冲区未创建
  }

  size_t bytes_read = xStreamBufferReceive(_rx_stream_buffer, buffer, size, ms_to_ticks(timeout));
  if (received != nullptr)
  {
    *received = bytes_read;
  }
  return (bytes_read > 0) ? Status::OK : Status::TIMEOUT;
}

/**
 * @brief TX 恢复（非阻塞取锁；卡死时会调阻塞版 HAL 中止发送，只能在任务上下文调用）
 *
 * @return true=本次确实重新启动了一次发送
 *
 * @note 两种情况：
 *       ① 通道空闲，但流缓冲区或 DMA 缓冲里还有没发的数据（丢了一次 TX 完成中断、或 DMA 启动失败）
 *          → 重新启动，先发 DMA 缓冲里留下的那一段。要求**连续两次**巡检都看到才动手：
 *          send() 入队后、自己启动 DMA 前，可能正好被更高优先级的 cubot_sys_task 打断，这一瞬间也是
 *          「空闲但有数据」，属于正常情况，不应算作卡死；
 *       ② 通道忙，但连续 stall_ms 没有推进 → 中止发送，按 DMA 剩余计数算出已发多少，
 *          只续发剩下的（不重复、不丢失），之后再从流缓冲区取新数据。
 * @note 判据看「流缓冲区 + DMA 缓冲」两处：只看流缓冲区会漏掉“数据都在 DMA 缓冲里、DMA 卡住”的情况。
 * @note stall_ms 按 Config::baudrate 估算为「一整包发送耗时的 2 倍」。
 */
template <size_t BUFFER_SIZE>
bool BspUart<BUFFER_SIZE>::tx_recover()
{
  if (_huart == nullptr || _tx_stream_buffer == nullptr || _tx_lock == nullptr || !_transmit_enable || _baudrate == 0U)
  {
    return false; // 未初始化 / 未启用发送 / 波特率未填
  }

  // 卡死判定阈值 (ms) = 一包发送耗时的 2 倍（向上取整）。
  // 一包耗时 = BUFFER_SIZE × 10 bit ÷ 波特率，乘 1000 换成 ms；×2 是留余量。
  const uint32_t stall_ms = (BUFFER_SIZE * 20U * 1000U + _baudrate - 1U) / _baudrate;

  // 没有任何待发数据 → 不存在卡死
  if (xStreamBufferBytesAvailable(_tx_stream_buffer) == 0U && _tx_chunk_len == 0U)
  {
    _tx_idle_pending_seen = false;
    return false;
  }

  // 巡检不阻塞：别的任务正在启动发送就下轮再看
  if (xSemaphoreTake(_tx_lock, 0) != pdTRUE)
  {
    return false;
  }

  bool restarted = false;

  if (_huart->gState == HAL_UART_STATE_READY)
  {
    // ① 通道空闲却有数据：第一次看到只记下，下一次巡检仍是这样才重新启动
    //    （start_chunk 会先发 DMA 缓冲里留下的那一段）
    if (_tx_idle_pending_seen)
    {
      if (repair_dma_handle(_huart->hdmatx))
      {
        _tx_dma_repair_cnt++; // 通道空闲但 DMA 句柄卡在非 READY：复位后才能再启动
      }
      restarted             = start_chunk(false, nullptr);
      _tx_idle_pending_seen = false;
    }
    else
    {
      _tx_idle_pending_seen = true;
    }
  }
  else
  {
    _tx_idle_pending_seen = false; // 通道在忙，「空闲但有数据」的观察作废

    if ((xTaskGetTickCount() - _tx_last_activity_tick) >= pdMS_TO_TICKS(stall_ms))
    {
      // ② 通道忙但长时间没推进：中止，再按 DMA 剩余计数算出这一段已发出多少。
      //    先中止再读计数：中止后 DMA 停住，读到的剩余数才是准的（中止前读，之后可能又发出几个字节，续发时就会重复）。
      (void)HAL_UART_AbortTransmit(_huart); // 阻塞版 HAL，只能在任务上下文调用
      if (repair_dma_handle(_huart->hdmatx))
      {
        _tx_dma_repair_cnt++;
      }
      settle_chunk();
      restarted = start_chunk(false, nullptr);
    }
  }

  xSemaphoreGive(_tx_lock);

  if (restarted)
  {
    _tx_stall_rec_cnt++; // 诊断：恢复成功一次
  }
  return restarted;
}

/**
 * @brief RX 恢复：RX 异常停止时复位接收通道并重新启动（阻塞）
 *
 * @return true=本次确实重建了一次
 *
 * @note 必须放在任务侧：复位 RX 要调阻塞版 HAL_UART_AbortReceive()，ISR 里不能做。
 */
template <size_t BUFFER_SIZE>
bool BspUart<BUFFER_SIZE>::rx_recover()
{
  if (_huart == nullptr || _rx_stream_buffer == nullptr || _huart->hdmarx == nullptr)
  {
    return false; // 未初始化 / 本路没配 RX DMA
  }

  // 本驱动没期望收数（主动停止 / 还没启动过）→ 不是故障
  if (!_rx_should_run)
  {
    return false;
  }

  // 三处状态一致表明 RX 正常 → 直接返回（正常时每次调用只做一次查询）
  if (rx_health() == RxHealth::OK)
  {
    return false;
  }

  // RX 已停止或三处状态不一致：复位接收通道后重新启动，只动 RX、不碰 TX DMA。
  // 整个「中止 → 复位句柄 → 启动」期间关掉本串口的 NVIC 中断，免得 IDLE 中断在中间重新启动接收。
  IRQn_Type  irqn    = static_cast<IRQn_Type>(0);
  const bool has_irq = uart_irqn(_huart->Instance, &irqn);
  if (has_irq)
  {
    HAL_NVIC_DisableIRQ(irqn);
  }
  abort_reception();
  if (repair_dma_handle(_huart->hdmarx))
  {
    _rx_dma_repair_cnt++; // 句柄卡在 ERROR：HAL 自己恢复不了，这里复位
  }
  const bool armed = arm_reception();
  if (has_irq)
  {
    HAL_NVIC_EnableIRQ(irqn);
  }
  if (!armed)
  {
    return false; // 仍失败，下个周期再试（_rx_should_run 保持为 true）
  }

  _rx_error_cnt++; // 诊断：确实重建了一次
  return true;
}

// ----------------


// ---------------- ISR 入口 ----------------

/**
 * @brief IDLE/TC 接收完成：投递数据并重新启动接收（ISR）
 *
 * @param size 本次收到的字节数
 * @note HT 事件也会进本回调，但它不是帧边界，直接忽略。本函数只碰 RX，不碰 TX DMA。
 */
template <size_t BUFFER_SIZE>
void BspUart<BUFFER_SIZE>::on_idle_isr(uint16_t size, BaseType_t *pxHigherPriorityTaskWoken)
{
  if (_huart == nullptr)
  {
    return; // 句柄缺失（理论上不会走到：回调是按 huart 分发的）
  }

  // HAL 在半传输（HT）事件时也会回调本函数：HT 不是帧边界，直接忽略
  if (_huart->RxEventType == HAL_UART_RXEVENT_HT)
  {
    return;
  }

  // 到此处 RxState 已为 READY：HAL 在 IDLE/TC 事件中已停掉 RX DMA，
  // 这里只需投递数据并重新启动接收，全程不触碰 TX DMA（不打断正在发送的数据）
  if (_rx_stream_buffer != nullptr && size > 0)
  {
    // 流缓冲区满时 SendFromISR 只会写入一部分，剩余字节被丢弃。
    // 累加诊断计数记录丢弃的字节数（正常应恒为 0）。
    size_t pushed = xStreamBufferSendFromISR(_rx_stream_buffer, _rx_dma_buffer, size, pxHigherPriorityTaskWoken);
    _rx_drop_bytes += static_cast<uint32_t>(size - pushed);
  }

  // 重新启动接收。若失败也不在此重试（阻塞）
  // 任务侧 rx_recover() 会在 10 ms 内通过 rx_health() 发现异常并重建。
  (void)arm_reception();
}

/**
 * @brief 发送完成后续传：从流缓冲区取下一批交给 DMA（ISR，由 TX Complete 中断调用）
 *
 * @note 这里不加 _tx_lock：gState 由 TX 完成中断在回调时触发：必然没有正在进行的 TX、
 *       也没有挂起的完成中断，任务侧只需互斥其他任务。
 */
template <size_t BUFFER_SIZE>
void BspUart<BUFFER_SIZE>::start_transmission_from_isr(BaseType_t *pxHigherPriorityTaskWoken)
{
  if (_huart == nullptr || _tx_stream_buffer == nullptr || !_transmit_enable)
  {
    return; // 未启用发送或发送缓冲区未初始化
  }

  // 通道必须空闲才能续传。正常情况下这里必然空闲：
  // 不空闲说明通道被别的传输占了，此时不取数据，交给 tx_recover() 处理。
  const bool tx_channel_idle = (_huart->gState == HAL_UART_STATE_READY);
  if (!tx_channel_idle)
  {
    _tx_isr_skip_cnt++;
    return; // 数据仍在流缓冲区，等巡检重新启动发送（tx_recover函数）
  }

  // 本回调 = 在传的那一段已全部发完
  if (_tx_chunk_active)
  {
    _tx_chunk_len    = 0;
    _tx_chunk_active = false;
  }

  (void)start_chunk(true, pxHigherPriorityTaskWoken);
}

/**
 * @brief UART 错误回调（ISR）：只累加计数，不做恢复
 * @note HAL 的 Abort 接口内部是阻塞版 HAL_DMA_Abort，ISR 里不能调用，
 *       恢复动作交给任务侧的 rx_recover() / tx_recover()。
 */
template <size_t BUFFER_SIZE>
void BspUart<BUFFER_SIZE>::on_error_isr()
{
  if (_huart == nullptr)
  {
    return;
  }

  // HAL 在 DMA 出错时会停掉对应通道，gState 离开 READY → 记一次 TX 侧故障
  if (_huart->gState != HAL_UART_STATE_READY)
  {
    _tx_error_cnt++;
  }
}

// ----------------


// ---------------- 查询接口 ----------------

/** @brief 发送流缓冲区剩余空间（字节） */
template <size_t BUFFER_SIZE>
size_t BspUart<BUFFER_SIZE>::get_tx_free_space()
{
  if (_tx_stream_buffer != nullptr)
  {
    return xStreamBufferSpacesAvailable(_tx_stream_buffer);
  }
  return 0;
}

/** @brief 接收流缓冲区当前可读字节数 */
template <size_t BUFFER_SIZE>
size_t BspUart<BUFFER_SIZE>::get_rx_available_data()
{
  if (_rx_stream_buffer != nullptr)
  {
    return xStreamBufferBytesAvailable(_rx_stream_buffer);
  }
  return 0;
}

/**
 * @brief RX 健康查询：交叉核对三处状态（HAL RxState / CR3.DMAR / DMA_SxCR.EN）
 *
 * @return RxHealth OK=三处一致"在收"；STOPPED=三处一致"停"；INCONSISTENT=三处不一致
 *
 * @note 只看 RxState 不够：RxState 可能仍为 BUSY_RX 而 DMA 已经停了。
 */
template <size_t BUFFER_SIZE>
typename BspUart<BUFFER_SIZE>::RxHealth BspUart<BUFFER_SIZE>::rx_health() const
{
  if (_huart == nullptr || _huart->Instance == nullptr)
  {
    return RxHealth::STOPPED;
  }

  // 1. HAL 软件状态
  const bool hal_busy = (_huart->RxState == HAL_UART_STATE_BUSY_RX);

  // 2. UART 侧：是否允许产生 RX DMA 请求
  const bool dmaren = ((_huart->Instance->CR3 & USART_CR3_DMAR) != 0U);

  // 3. DMA 侧：通道是否真的在搬运
  bool dma_en = false;
  if (_huart->hdmarx != nullptr && IS_DMA_STREAM_INSTANCE(_huart->hdmarx->Instance) != 0U)
  {
    dma_en = ((((DMA_Stream_TypeDef *)(_huart->hdmarx->Instance))->CR & DMA_SxCR_EN) != 0U);
  }

  if (hal_busy && dmaren && dma_en)
  {
    return RxHealth::OK; // 三处一致：正在接收
  }
  if (!hal_busy && !dmaren && !dma_en)
  {
    return RxHealth::STOPPED; // 三处一致：已停止
  }
  return RxHealth::INCONSISTENT; // 三处状态不一致 → 需要重建
}

/** @brief 读取 RX 诊断计数 */
template <size_t BUFFER_SIZE>
typename BspUart<BUFFER_SIZE>::RxDiag BspUart<BUFFER_SIZE>::rx_diag() const
{
  RxDiag diag;
  diag.arm_fail_cnt = _rx_arm_fail_cnt;
  diag.drop_bytes   = _rx_drop_bytes;
  diag.error_cnt    = _rx_error_cnt;
  diag.dma_repair_cnt = _rx_dma_repair_cnt;
  return diag;
}

/** @brief 读取 TX 诊断计数 */
template <size_t BUFFER_SIZE>
typename BspUart<BUFFER_SIZE>::TxDiag BspUart<BUFFER_SIZE>::tx_diag() const
{
  TxDiag diag;
  diag.drop_bytes        = _tx_drop_bytes;
  diag.start_fail_cnt    = _tx_start_fail_cnt;
  diag.isr_skip_cnt      = _tx_isr_skip_cnt;
  diag.stall_recover_cnt = _tx_stall_rec_cnt;
  diag.error_cnt         = _tx_error_cnt;
  diag.dma_repair_cnt    = _tx_dma_repair_cnt;
  return diag;
}

// ----------------


// ---------------- 私有实现 ----------------

/**
 * @brief 启动 DMA 接收（仅 RX）
 *
 * @return true=成功；false=HAL 拒绝（BUSY/ERROR），已累加 arm_fail_cnt
 *
 * @note 调用前 RxState 必须为 READY。开头先清掉残留的 UART 错误标志（ORE/FE/NE/PE），
 *       否则 HAL_UARTEx_ReceiveToIdle_DMA() 会因存在挂起错误而拒绝启动。
 */
template <size_t BUFFER_SIZE>
bool BspUart<BUFFER_SIZE>::arm_reception()
{
  // 先清掉残留的 UART 错误标志：否则 HAL 会因"启动前已有挂起的 ORE/FE/NE"而拒绝启动
  // （HAL_UARTEx_ReceiveToIdle_DMA 内部会检测到 ReceptionType 被重置，返回 HAL_ERROR）
  //
  // 注：DMA 侧的 TC/HT/TE/FE/DME 标志无需在此清除——HAL 已负责：
  //     HAL_DMA_Abort() 用 IFCR 一次清掉该 Stream 全部 6 个标志
  //     （abort_reception() 与 HAL 的 IDLE 分支都会走到），
  //     HAL_DMA_IRQHandler() 也会清 TC/HT。
  __HAL_UART_CLEAR_FLAG(_huart, UART_CLEAR_PEF | UART_CLEAR_FEF | UART_CLEAR_NEF | UART_CLEAR_OREF);

  // 启动多字节 DMA 接收（IDLE 模式）；HAL 会自动清 IDLE 标志并使能 IDLE 中断
  if (HAL_UARTEx_ReceiveToIdle_DMA(_huart, _rx_dma_buffer, BUFFER_SIZE) != HAL_OK)
  {
    _rx_arm_fail_cnt++; // 诊断：启动失败计数（正常应恒为 0）
    return false;       // 启动失败：由调用方决定重试或上报
  }
  _rx_should_run = true;
  return true;
}

/** @brief 中止 RX 通道并复位接收状态（只动 RX，不碰 TX DMA） */
template <size_t BUFFER_SIZE>
void BspUart<BUFFER_SIZE>::abort_reception()
{
  HAL_UART_AbortReceive(_huart);
  ATOMIC_CLEAR_BIT(_huart->Instance->CR1, USART_CR1_IDLEIE); // 防止残留 IDLE 中断源
}

/**
 * @brief 从发送流缓冲区取一包并启动 DMA（任务上下文）
 *
 * @param wait 取 _tx_lock 的等待时间（ticks）；send() 传 portMAX_DELAY，巡检传 0
 * @return true=本次确实启动了一次 DMA
 *
 * @note "判忙 + 取包 + 启动 DMA" 必须整体原子，否则两个任务可能同时写 _tx_dma_buffer，
 *       覆盖 DMA 正在读取的数据。
 */
template <size_t BUFFER_SIZE>
bool BspUart<BUFFER_SIZE>::start_transmission(TickType_t wait)
{
  if (_huart == nullptr || _tx_stream_buffer == nullptr)
  {
    return false; // 句柄/缓冲未就绪
  }

  if (_tx_lock == nullptr)
  {
    return false; // 互斥量未创建
  }

  // 判忙 + 取包 + 启动 DMA 必须整体原子：否则两个任务可能同时写入 _tx_dma_buffer，
  // 覆盖 DMA 正在读取的数据（已取出的字节会被丢掉）。
  // wait=0（巡检）时拿不到锁立即返回，不阻塞周期任务。
  if (xSemaphoreTake(_tx_lock, wait) != pdTRUE)
  {
    return false;
  }

  bool started = false;

  // 必须严格用 READY 判定（而不是"不等于 BUSY_TX"）：ERROR/TIMEOUT 状态下
  // HAL_UART_Transmit_DMA 会被 HAL 拒绝
  if (_huart->gState == HAL_UART_STATE_READY)
  {
    started = start_chunk(false, nullptr);
  }

  xSemaphoreGive(_tx_lock);
  return started;
}

/**
 * @brief 启动一次 DMA 发送：先重发 DMA 缓冲里留下的那一段，没有才从流缓冲区取新数据
 *
 * @note 启动失败时数据留在 _tx_dma_buffer（_tx_chunk_active = false），由下一次启动或
 *       tx_recover() 先把它发掉，再取新数据——不往流缓冲区写回，所以字节顺序不会乱，
 *       中断侧也就不会成为流缓冲区的第二个写者。
 */
template <size_t BUFFER_SIZE>
bool BspUart<BUFFER_SIZE>::start_chunk(bool from_isr, BaseType_t *pxHigherPriorityTaskWoken)
{
  if (_tx_chunk_active)
  {
    // 任务侧看到通道空闲、这一段却还标着“在传”：DMA 被 HAL 停掉了（出错）或完成中断丢了，
    // 按 DMA 剩余计数结算，只续发没发出去的部分
    settle_chunk();
  }

  if (_tx_chunk_len == 0U)
  {
    // 没有留下的数据：从流缓冲区取一包新的
    const size_t n = from_isr
                       ? xStreamBufferReceiveFromISR(_tx_stream_buffer, _tx_dma_buffer, BUFFER_SIZE, pxHigherPriorityTaskWoken)
                       : xStreamBufferReceive(_tx_stream_buffer, _tx_dma_buffer, BUFFER_SIZE, 0);
    if (n == 0U)
    {
      return false; // 没有待发数据
    }
    _tx_chunk_off = 0;
    _tx_chunk_len = n;
  }

  const TickType_t now = from_isr ? xTaskGetTickCountFromISR() : xTaskGetTickCount();

  if (HAL_UART_Transmit_DMA(_huart, _tx_dma_buffer + _tx_chunk_off, static_cast<uint16_t>(_tx_chunk_len)) != HAL_OK)
  {
    // 启动失败：数据留在 DMA 缓冲里等下次重发；也记一次“推进”，免得被当成卡死而中止
    _tx_chunk_active       = false;
    _tx_last_activity_tick = now;
    _tx_start_fail_cnt++;
    return false;
  }

  _tx_chunk_active       = true;
  _tx_last_activity_tick = now; // 记录推进时刻（供 tx_recover 判卡死）
  return true;
}

/**
 * @brief 结算一段已停止的 DMA 发送：按 DMA 剩余计数算出已发出多少，把这一段的起点往后挪
 *
 * @note 调用前提：DMA 已停（中止过、出错停了或已完成），否则读到的剩余计数还会变，续发时就会重复。
 * @note 起点跟着往后挪：连续结算多次时，每次都相对“这一段的当前起点”算，不会回到缓冲区开头。
 * @warning TX DMA 开着 FIFO 时，NDTR 把已预读进 DMA FIFO、还没写到串口的字节也算作已发出，
 *          这里会把它们跳过（丢失）。直接模式下 NDTR 准确，不丢不重。见 bsp_uart.hpp 文件头。
 */
template <size_t BUFFER_SIZE>
void BspUart<BUFFER_SIZE>::settle_chunk()
{
  if (!_tx_chunk_active || _tx_chunk_len == 0U)
  {
    _tx_chunk_active = false;
    return;
  }

  size_t sent = 0;
  if (_huart->hdmatx != nullptr)
  {
    const size_t left_cnt = static_cast<size_t>(__HAL_DMA_GET_COUNTER(_huart->hdmatx));
    if (left_cnt <= _tx_chunk_len)
    {
      sent = _tx_chunk_len - left_cnt; // 已发出的字节数
    }
  }

  _tx_chunk_off    = _tx_chunk_off + sent;
  _tx_chunk_len    = _tx_chunk_len - sent;
  _tx_chunk_active = false;
}

/** @brief 释放所有已创建的 FreeRTOS 资源 */
template <size_t BUFFER_SIZE>
void BspUart<BUFFER_SIZE>::cleanup_resources()
{
  // 释放接收流缓冲区
  if (_rx_stream_buffer != nullptr)
  {
    vStreamBufferDelete(_rx_stream_buffer);
    _rx_stream_buffer = nullptr;
  }

  // 释放发送流缓冲区
  if (_tx_stream_buffer != nullptr)
  {
    vStreamBufferDelete(_tx_stream_buffer);
    _tx_stream_buffer = nullptr;
  }

  // 释放 TX 入队锁
  if (_tx_write_lock != nullptr)
  {
    vSemaphoreDelete(_tx_write_lock);
    _tx_write_lock = nullptr;
  }

  // 释放 TX 启动锁
  if (_tx_lock != nullptr)
  {
    vSemaphoreDelete(_tx_lock);
    _tx_lock = nullptr;
  }

  // 释放 printf 锁
  if (_printf_lock != nullptr)
  {
    vSemaphoreDelete(_printf_lock);
    _printf_lock = nullptr;
  }
}

// ----------------

#endif // HAL_UART_MODULE_ENABLED

#include "stm32h7xx_hal.h" // IWYU pragma: keep（HAL 模块开关）

// 宿主工程没在 CubeMX 里开 SPI（HAL_SPI_MODULE_ENABLED 未定义）时，本文件不参与编译
#ifdef HAL_SPI_MODULE_ENABLED

#include "bsp_spi.hpp"

#include <string.h>

#include "task.h" // IWYU pragma: keep（xTaskGetSchedulerState）


// ---------------- 内部工具 ----------------

namespace
{
/**
 * @brief 毫秒 → tick；portMAX_DELAY 原样保留（表示一直等）
 *
 * @note pdMS_TO_TICKS(portMAX_DELAY) 会按 32 位溢出，不再是“一直等”（见 bsp_uart.cpp 同名函数）。
 */
TickType_t ms_to_ticks(uint32_t ms)
{
  return (ms == portMAX_DELAY) ? portMAX_DELAY : pdMS_TO_TICKS(ms);
}

/**
 * @brief 地址能否被 DMA1 / DMA2 访问
 *
 * @note STM32H7 的 ITCM（0x0000_0000 起）与 DTCM（0x2000_0000 起，到 AXI SRAM 0x2400_0000 之前）
 *       只连到 CPU 与 MDMA，DMA1 / DMA2 访问会出传输错误。任务栈、默认 .bss 都在 DTCM。
 */
bool dma_reachable(const void *addr)
{
  const uintptr_t a    = reinterpret_cast<uintptr_t>(addr);
  const bool      itcm = a < 0x08000000U;
  const bool      dtcm = a >= 0x20000000U && a < 0x24000000U;
  return !itcm && !dtcm;
}

/** @brief 调度器是否已经启动（启动前不取锁、不等信号量，一律轮询） */
bool scheduler_running()
{
  return xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED;
}

/** @brief HAL 单次传输长度上限（Size 是 uint16_t） */
constexpr size_t HAL_MAX_SEGMENT = 0xFFFFU;
} // namespace

// ----------------


// ---------------- HAL 回调（库独占，宿主工程不能再定义） ----------------

extern "C"
{
  /**
   * @brief DMA 全双工传输完成（SPI EOT 中断里由 HAL 调用）：按句柄查登记表，交给对应实例
   * @note 没有登记的 SPI（宿主工程自己管理的）直接忽略。
   */
  void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
  {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    BspSpi    *spi                      = BspSpi::find(hspi);
    if (spi != nullptr)
    {
      spi->on_txrx_cplt_isr(&xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }

  /** @brief 传输错误（模式错误、溢出、DMA 错误等）：交给对应实例，等待方返回 IO_ERROR */
  void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
  {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    BspSpi    *spi                      = BspSpi::find(hspi);
    if (spi != nullptr)
    {
      spi->on_error_isr(&xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}

// ----------------


// ---------------- 实例登记表 ----------------

BspSpi  *BspSpi::_registry[BspSpi::MAX_INSTANCES] = {};
uint32_t BspSpi::_registry_num                    = 0U;

/** @brief 登记本实例；同一句柄只允许一个实例 */
Status BspSpi::register_self()
{
  if (_hspi == nullptr)
  {
    return Status::BAD_ARG;
  }
  for (uint32_t i = 0U; i < _registry_num; ++i)
  {
    if (_registry[i] == this)
    {
      return Status::OK;
    }
    if (_registry[i]->_hspi == _hspi)
    {
      return Status::BAD_ARG; // 两个实例绑定同一路 SPI：回调只会交给其中一个，属于配置错误
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

/** @brief 摘除：后面的实例前移，保持表紧凑 */
void BspSpi::unregister_self()
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
      return;
    }
  }
}

uint32_t BspSpi::instance_count()
{
  return _registry_num;
}

BspSpi *BspSpi::instance_at(uint32_t index)
{
  return (index < _registry_num) ? _registry[index] : nullptr;
}

/** @brief 线性查找（最多 6 项），ISR 中调用 */
BspSpi *BspSpi::find(const SPI_HandleTypeDef *hspi)
{
  for (uint32_t i = 0U; i < _registry_num; ++i)
  {
    if (_registry[i]->_hspi == hspi)
    {
      return _registry[i];
    }
  }
  return nullptr;
}

// ----------------


// ---------------- 总线：构造与初始化 ----------------

BspSpi::BspSpi(const Config &cfg) : _hspi(cfg.hspi), _use_dma(cfg.use_dma), _dma_threshold(cfg.dma_threshold)
{
  // 构造函数只做赋值；FreeRTOS 资源创建推迟到 init()
}

BspSpi::~BspSpi()
{
  unregister_self();
  if (_lock != nullptr)
  {
    vSemaphoreDelete(_lock);
    _lock = nullptr;
  }
  if (_done != nullptr)
  {
    vSemaphoreDelete(_done);
    _done = nullptr;
  }
  _init = false;
}

Status BspSpi::init()
{
  // 运行中重建锁会让正在等锁的任务拿着失效句柄，所以只允许一次
  if (_init || _hspi == nullptr)
  {
    return Status::BAD_ARG;
  }

  if (_use_dma)
  {
    // 没链接 DMA 时 HAL_SPI_TransmitReceive_DMA() 直接返回错误，提前拒绝，免得运行时每次大传输都失败
    if (_hspi->hdmarx == nullptr || _hspi->hdmatx == nullptr)
    {
      return Status::BAD_ARG;
    }
    // 弹跳缓冲是对象成员，DMA 直接读写：实例必须放进 .dma_buffer
    if (!dma_reachable(_tx_bounce) || !dma_reachable(_rx_bounce))
    {
      return Status::BAD_ARG;
    }
  }

  const Status reg = register_self();
  if (reg != Status::OK)
  {
    return reg;
  }

  _lock = xSemaphoreCreateMutex(); // 互斥量自带优先级继承
  _done = xSemaphoreCreateBinary();
  if (_lock == nullptr || _done == nullptr)
  {
    if (_lock != nullptr)
    {
      vSemaphoreDelete(_lock);
      _lock = nullptr;
    }
    if (_done != nullptr)
    {
      vSemaphoreDelete(_done);
      _done = nullptr;
    }
    unregister_self();
    return Status::IO_ERROR;
  }

  _init = true;
  return Status::OK;
}

bool BspSpi::is_init() const
{
  return _init;
}

// ----------------


// ---------------- 总线：事务接口 ----------------

Status BspSpi::lock(uint32_t timeout_ms)
{
  if (!_init)
  {
    return Status::NOT_INIT;
  }
  if (!scheduler_running())
  {
    return Status::OK; // 调度器启动前只有一个执行流
  }
  if (xSemaphoreTake(_lock, ms_to_ticks(timeout_ms)) != pdTRUE)
  {
    ++_lock_timeout_cnt;
    return Status::TIMEOUT;
  }
  return Status::OK;
}

void BspSpi::unlock()
{
  if (_init && scheduler_running())
  {
    xSemaphoreGive(_lock);
  }
}

/**
 * @note 片选由调用方全程保持，这里拆成的多段对器件来说是一整帧
 *       （CubeMX 里 MasterKeepIOState 必须为 ENABLE，否则段与段之间 SPE 关闭时 SCK 浮空，会出假时钟沿）。
 */
Status BspSpi::transfer_locked(const uint8_t *tx, uint8_t *rx, size_t len, uint32_t timeout_ms)
{
  if (len == 0U)
  {
    return Status::BAD_ARG;
  }
  if (!_init)
  {
    return Status::NOT_INIT;
  }

  const bool use_dma = _use_dma && len >= _dma_threshold && scheduler_running();

  // 每个方向单独决定：调用方缓冲能直接用（轮询总能用，DMA 要在 DMA 可达内存里）就直接用，
  // 否则走弹跳缓冲（tx 为空 = 弹跳缓冲填 0xFF，rx 为空 = 收进弹跳缓冲丢掉）
  const bool tx_direct = tx != nullptr && (!use_dma || dma_reachable(tx));
  const bool rx_direct = rx != nullptr && (!use_dma || dma_reachable(rx));
  // 调用方缓冲 DMA 访问不到、必须中转（只计这种，填充 / 丢弃不算）
  const bool bounced = use_dma && ((tx != nullptr && !tx_direct) || (rx != nullptr && !rx_direct));
  // 任一方向用弹跳缓冲，每段就不能超过它
  const size_t max_seg = (tx_direct && rx_direct) ? HAL_MAX_SEGMENT : SPI_DMA_BUFFER_SIZE;

  size_t done = 0U;
  while (done < len)
  {
    const size_t   seg    = (len - done > max_seg) ? max_seg : (len - done);
    const uint8_t *seg_tx = _tx_bounce;
    uint8_t       *seg_rx = rx_direct ? rx + done : _rx_bounce;
    if (tx_direct)
    {
      seg_tx = tx + done;
    }
    else if (tx != nullptr)
    {
      memcpy(_tx_bounce, tx + done, seg);
    }
    else
    {
      memset(_tx_bounce, 0xFF, seg);
    }

    if (bounced)
    {
      ++_bounce_cnt;
    }
    const Status st = use_dma ? transfer_dma(seg_tx, seg_rx, seg, timeout_ms) : transfer_poll(seg_tx, seg_rx, seg, timeout_ms);
    if (st != Status::OK)
    {
      return st;
    }
    if (!rx_direct && rx != nullptr)
    {
      memcpy(rx + done, _rx_bounce, seg);
    }
    done += seg;
  }
  return Status::OK;
}

Status BspSpi::transfer_poll(const uint8_t *tx, uint8_t *rx, size_t len, uint32_t timeout_ms)
{
  const HAL_StatusTypeDef hs = HAL_SPI_TransmitReceive(_hspi, tx, rx, static_cast<uint16_t>(len), timeout_ms);
  if (hs == HAL_OK)
  {
    ++_poll_cnt;
    return Status::OK;
  }
  if (hs == HAL_TIMEOUT)
  {
    // HAL 超时返回时已关掉 SPE、句柄回到 READY，下一次可以直接用
    ++_timeout_cnt;
    return Status::TIMEOUT;
  }
  ++_error_cnt;
  return Status::IO_ERROR;
}

Status BspSpi::transfer_dma(const uint8_t *tx, uint8_t *rx, size_t len, uint32_t timeout_ms)
{
  // 清掉上一次可能残留的完成信号（例如超时后才到的完成中断）
  (void)xSemaphoreTake(_done, 0U);
  _xfer_error = false;

  if (HAL_SPI_TransmitReceive_DMA(_hspi, tx, rx, static_cast<uint16_t>(len)) != HAL_OK)
  {
    ++_error_cnt;
    return Status::IO_ERROR;
  }

  if (xSemaphoreTake(_done, ms_to_ticks(timeout_ms)) != pdTRUE)
  {
    // 停掉 DMA 与 SPI，句柄回到 READY；中止过程中若有完成 / 错误回调进来，信号量在下一次开始前清掉
    (void)HAL_SPI_Abort(_hspi);
    ++_timeout_cnt;
    return Status::TIMEOUT;
  }

  if (_xfer_error)
  {
    return Status::IO_ERROR; // error_cnt 已在错误回调里累加
  }
  ++_dma_cnt;
  return Status::OK;
}

// ----------------


// ---------------- 总线：ISR 入口与查询 ----------------

void BspSpi::on_txrx_cplt_isr(BaseType_t *pxHigherPriorityTaskWoken)
{
  if (_done != nullptr)
  {
    xSemaphoreGiveFromISR(_done, pxHigherPriorityTaskWoken);
  }
}

void BspSpi::on_error_isr(BaseType_t *pxHigherPriorityTaskWoken)
{
  _xfer_error = true;
  ++_error_cnt;
  if (_done != nullptr)
  {
    xSemaphoreGiveFromISR(_done, pxHigherPriorityTaskWoken);
  }
}

BspSpi::Diag BspSpi::diag() const
{
  Diag d;
  d.poll_cnt         = _poll_cnt;
  d.dma_cnt          = _dma_cnt;
  d.bounce_cnt       = _bounce_cnt;
  d.timeout_cnt      = _timeout_cnt;
  d.error_cnt        = _error_cnt;
  d.lock_timeout_cnt = _lock_timeout_cnt;
  return d;
}

// ----------------


// ---------------- 片选设备 ----------------

BspSpiDevice::BspSpiDevice(BspSpi &bus, const Config &cfg) : _bus(bus), _cs(BspGpio::Config(cfg.cs_port, cfg.cs_pin)), _read_flag(cfg.read_flag), _read_dummy(cfg.read_dummy)
{
  // 构造函数只做赋值
}

Status BspSpiDevice::init()
{
  if (!_bus.is_init())
  {
    return Status::NOT_INIT;
  }
  if (_cs.init(BspGpio::Config(_cs.get_port(), _cs.get_pin())) != Status::OK)
  {
    return Status::BAD_ARG;
  }
  deselect();
  _init = true;
  return Status::OK;
}

Status BspSpiDevice::transaction(const uint8_t *head, size_t head_len, size_t skip, const uint8_t *tx, uint8_t *rx, size_t len, uint32_t timeout_ms)
{
  if (!_init)
  {
    return Status::NOT_INIT;
  }

  Status st = _bus.lock(timeout_ms);
  if (st != Status::OK)
  {
    return st;
  }

  select();
  if (head_len > 0U)
  {
    st = _bus.transfer_locked(head, nullptr, head_len, timeout_ms);
  }
  if (st == Status::OK && skip > 0U)
  {
    st = _bus.transfer_locked(nullptr, nullptr, skip, timeout_ms);
  }
  if (st == Status::OK)
  {
    st = _bus.transfer_locked(tx, rx, len, timeout_ms);
  }
  deselect();

  _bus.unlock();
  return st;
}

Status BspSpiDevice::transfer(const uint8_t *tx, uint8_t *rx, size_t len, uint32_t timeout_ms)
{
  if (len == 0U)
  {
    return Status::BAD_ARG;
  }
  return transaction(nullptr, 0U, 0U, tx, rx, len, timeout_ms);
}

Status BspSpiDevice::write_then_read(const uint8_t *tx, size_t tx_len, uint8_t *rx, size_t rx_len, uint32_t timeout_ms)
{
  if (tx == nullptr || tx_len == 0U || rx == nullptr || rx_len == 0U)
  {
    return Status::BAD_ARG;
  }
  return transaction(tx, tx_len, 0U, nullptr, rx, rx_len, timeout_ms);
}

Status BspSpiDevice::read_reg(uint8_t reg, uint8_t *data, size_t len, uint32_t timeout_ms)
{
  if (data == nullptr || len == 0U)
  {
    return Status::BAD_ARG;
  }
  const uint8_t addr = static_cast<uint8_t>(reg | _read_flag);
  return transaction(&addr, 1U, _read_dummy, nullptr, data, len, timeout_ms);
}

Status BspSpiDevice::write_reg(uint8_t reg, const uint8_t *data, size_t len, uint32_t timeout_ms)
{
  if (data == nullptr || len == 0U)
  {
    return Status::BAD_ARG;
  }
  const uint8_t addr = static_cast<uint8_t>(reg & static_cast<uint8_t>(~_read_flag));
  return transaction(&addr, 1U, 0U, data, nullptr, len, timeout_ms);
}

Status BspSpiDevice::read_reg8(uint8_t reg, uint8_t &value, uint32_t timeout_ms)
{
  return read_reg(reg, &value, 1U, timeout_ms);
}

Status BspSpiDevice::write_reg8(uint8_t reg, uint8_t value, uint32_t timeout_ms)
{
  return write_reg(reg, &value, 1U, timeout_ms);
}

// ----------------

#endif // HAL_SPI_MODULE_ENABLED

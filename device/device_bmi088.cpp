#include "stm32h7xx_hal.h" // IWYU pragma: keep（HAL 模块开关）

// 宿主工程没在 CubeMX 里开 SPI（HAL_SPI_MODULE_ENABLED 未定义）时，本文件不参与编译
#ifdef HAL_SPI_MODULE_ENABLED

#include "device_bmi088.hpp"


// ---------------- 寄存器表（BMI088 数据手册 BST-BMI088-DS001） ----------------

namespace
{
// 加速度计
constexpr uint8_t ACC_CHIP_ID      = 0x00U; ///< 芯片 ID，读值 0x1E
constexpr uint8_t ACC_X_LSB        = 0x12U; ///< ACC_X_LSB … ACC_Z_MSB 共 6 字节
constexpr uint8_t ACC_TEMP_MSB     = 0x22U; ///< TEMP_MSB、TEMP_LSB 共 2 字节
constexpr uint8_t ACC_CONF         = 0x40U; ///< [7:4] 滤波（0xA = normal），[3:0] ODR
constexpr uint8_t ACC_RANGE        = 0x41U; ///< 量程
constexpr uint8_t ACC_INT1_IO_CTRL = 0x53U; ///< INT1 引脚：输出使能 / 开漏 / 电平
constexpr uint8_t ACC_INT_MAP_DATA = 0x58U; ///< 中断映射：bit2 = 数据就绪到 INT1
constexpr uint8_t ACC_PWR_CONF     = 0x7CU; ///< 0x00 = 工作，0x03 = 挂起
constexpr uint8_t ACC_PWR_CTRL     = 0x7DU; ///< 0x04 = 打开加速度计，0x00 = 关闭
constexpr uint8_t ACC_SOFTRESET    = 0x7EU; ///< 写 0xB6 软复位

constexpr uint8_t ACC_CHIP_ID_VALUE = 0x1EU;

// 陀螺仪
constexpr uint8_t GYRO_CHIP_ID           = 0x00U; ///< 芯片 ID，读值 0x0F
constexpr uint8_t GYRO_RATE_X_LSB        = 0x02U; ///< RATE_X_LSB … RATE_Z_MSB 共 6 字节
constexpr uint8_t GYRO_RANGE             = 0x0FU; ///< 量程
constexpr uint8_t GYRO_BANDWIDTH         = 0x10U; ///< 输出数据率 / 带宽（读回 bit7 恒为 1）
constexpr uint8_t GYRO_LPM1              = 0x11U; ///< 0x00 = 正常模式
constexpr uint8_t GYRO_SOFTRESET         = 0x14U; ///< 写 0xB6 软复位
constexpr uint8_t GYRO_INT_CTRL          = 0x15U; ///< bit7 = 数据就绪中断使能
constexpr uint8_t GYRO_INT3_INT4_IO_CONF = 0x16U; ///< INT3：bit0 电平（1 = 高有效），bit1 开漏
constexpr uint8_t GYRO_INT3_INT4_IO_MAP  = 0x18U; ///< bit0 = 数据就绪到 INT3

constexpr uint8_t GYRO_CHIP_ID_VALUE = 0x0FU;

constexpr uint8_t SOFTRESET_CMD = 0xB6U; ///< 两颗芯片共用的软复位命令字

constexpr float GRAVITY = 9.80665f; ///< 标准重力加速度 (m/s²)

/** @brief 陀螺仪量程（°/s），下标 = GyroRange 枚举值 */
constexpr float GYRO_FULL_SCALE_DPS[] = {2000.0f, 1000.0f, 500.0f, 250.0f, 125.0f};
} // namespace

// ----------------


// ---------------- 构造与初始化 ----------------

DeviceBmi088::DeviceBmi088(BspSpiDevice &acc, BspSpiDevice &gyro, const Config &cfg) : _acc(acc), _gyro(gyro), _cfg(cfg)
{
  // 构造函数只做赋值；上电配置在 init()
}

/**
 * @note 顺序依据数据手册：加速度计上电后处于 I²C 模式，CS 上的一个上升沿（一次假读）才切到 SPI；
 *       软复位后同样回到 I²C 模式，要再假读一次。加速度计复位后处于挂起模式，要先打开再写配置。
 */
Status DeviceBmi088::init()
{
  _init = false;

  uint8_t id = 0U;
  Status  st = Status::OK;

  // 1) 加速度计切到 SPI，软复位，再切一次
  st = _acc.read_reg8(ACC_CHIP_ID, id);
  if (st != Status::OK)
  {
    return st;
  }
  st = _acc.write_reg8(ACC_SOFTRESET, SOFTRESET_CMD);
  if (st != Status::OK)
  {
    return st;
  }
  delay_ms(1U);
  st = _acc.read_reg8(ACC_CHIP_ID, id);
  if (st != Status::OK)
  {
    return st;
  }

  // 2) 陀螺仪软复位（复位后约 30 ms 可访问）
  st = _gyro.write_reg8(GYRO_SOFTRESET, SOFTRESET_CMD);
  if (st != Status::OK)
  {
    return st;
  }
  delay_ms(30U);

  // 3) 芯片 ID：读不对说明没焊、接错或片选接反
  st = _acc.read_reg8(ACC_CHIP_ID, id);
  if (st != Status::OK)
  {
    return st;
  }
  if (id != ACC_CHIP_ID_VALUE)
  {
    return Status::IO_ERROR;
  }
  st = _gyro.read_reg8(GYRO_CHIP_ID, id);
  if (st != Status::OK)
  {
    return st;
  }
  if (id != GYRO_CHIP_ID_VALUE)
  {
    return Status::IO_ERROR;
  }

  // 4) 加速度计上电：打开后等 50 ms 再切到工作模式
  st = write_verify(_acc, ACC_PWR_CTRL, 0x04U);
  if (st != Status::OK)
  {
    return st;
  }
  delay_ms(50U);
  st = write_verify(_acc, ACC_PWR_CONF, 0x00U);
  if (st != Status::OK)
  {
    return st;
  }

  // 5) 量程与数据率
  st = write_verify(_acc, ACC_CONF, static_cast<uint8_t>(0xA0U | static_cast<uint8_t>(_cfg.acc_odr)));
  if (st != Status::OK)
  {
    return st;
  }
  st = write_verify(_acc, ACC_RANGE, static_cast<uint8_t>(_cfg.acc_range));
  if (st != Status::OK)
  {
    return st;
  }
  st = write_verify(_gyro, GYRO_RANGE, static_cast<uint8_t>(_cfg.gyro_range));
  if (st != Status::OK)
  {
    return st;
  }
  st = write_verify(_gyro, GYRO_BANDWIDTH, static_cast<uint8_t>(_cfg.gyro_odr), 0x7FU);
  if (st != Status::OK)
  {
    return st;
  }
  st = write_verify(_gyro, GYRO_LPM1, 0x00U);
  if (st != Status::OK)
  {
    return st;
  }

  // 6) 数据就绪中断：加速度计 INT1、陀螺仪 INT3，推挽、高有效
  if (_cfg.drdy_int)
  {
    st = write_verify(_acc, ACC_INT1_IO_CTRL, 0x0AU); // bit3 输出使能、bit1 高有效、bit2 = 0 推挽
    if (st != Status::OK)
    {
      return st;
    }
    st = write_verify(_acc, ACC_INT_MAP_DATA, 0x04U);
    if (st != Status::OK)
    {
      return st;
    }
    st = write_verify(_gyro, GYRO_INT3_INT4_IO_CONF, 0x01U, 0x0FU); // 高 4 位保留
    if (st != Status::OK)
    {
      return st;
    }
    st = write_verify(_gyro, GYRO_INT3_INT4_IO_MAP, 0x01U);
    if (st != Status::OK)
    {
      return st;
    }
    st = write_verify(_gyro, GYRO_INT_CTRL, 0x80U);
    if (st != Status::OK)
    {
      return st;
    }
  }

  // 7) 换算系数：加速度计满量程 = 1.5 g × 2^(range+1)
  _acc_scale  = 1.5f * static_cast<float>(1U << (static_cast<uint8_t>(_cfg.acc_range) + 1U)) * GRAVITY / 32768.0f;
  _gyro_scale = GYRO_FULL_SCALE_DPS[static_cast<uint8_t>(_cfg.gyro_range)] * (PI / 180.0f) / 32768.0f;

  _init = true;

  // 8) 自动刷新：第一次成功时建内部任务（之后重复 init() 不再建）
  if (_cfg.auto_update && _worker == nullptr)
  {
    TaskHandle_t worker = nullptr;
    if (xTaskCreate(task_entry, "imu", BMI088_TASK_STACK, this, BMI088_TASK_PRIORITY, &worker) != pdPASS)
    {
      return Status::IO_ERROR;
    }
    _worker = worker;
  }
  return Status::OK;
}

// ----------------


// ---------------- 读数 ----------------

Status DeviceBmi088::update(uint32_t timeout_ms)
{
  const Status st = read_frame(timeout_ms);
  if (st == Status::OK)
  {
    ++_update_cnt;
  }
  else
  {
    ++_update_fail_cnt;
  }
  _last_status = st;
  return st;
}

Status DeviceBmi088::read_frame(uint32_t timeout_ms)
{
  if (!_init)
  {
    return Status::NOT_INIT;
  }

  uint8_t acc[6]  = {};
  uint8_t gyro[6] = {};
  uint8_t temp[2] = {};

  Status st = _acc.read_reg(ACC_X_LSB, acc, sizeof(acc), timeout_ms);
  if (st != Status::OK)
  {
    return st;
  }
  st = _gyro.read_reg(GYRO_RATE_X_LSB, gyro, sizeof(gyro), timeout_ms);
  if (st != Status::OK)
  {
    return st;
  }
  st = _acc.read_reg(ACC_TEMP_MSB, temp, sizeof(temp), timeout_ms);
  if (st != Status::OK)
  {
    return st;
  }

  // 先在局部变量里拼好、换算好，三组都读成功才一次性写回，避免内部数据一半新一半旧
  Bmi088RawData raw = {};
  for (uint32_t i = 0U; i < 3U; ++i)
  {
    raw.acc[i]  = static_cast<int16_t>(static_cast<uint16_t>(acc[2U * i]) | (static_cast<uint16_t>(acc[2U * i + 1U]) << 8));
    raw.gyro[i] = static_cast<int16_t>(static_cast<uint16_t>(gyro[2U * i]) | (static_cast<uint16_t>(gyro[2U * i + 1U]) << 8));
  }
  // 11 位补码：MSB 是高 8 位，LSB 的 [7:5] 是低 3 位
  int16_t t11 = static_cast<int16_t>((static_cast<uint16_t>(temp[0]) << 3) | (static_cast<uint16_t>(temp[1]) >> 5));
  if (t11 > 1023)
  {
    t11 = static_cast<int16_t>(t11 - 2048);
  }
  raw.temp = t11;

  // 饱和：ADC 顶到满量程，读数被截断（量程不够）
  uint8_t sat = 0U;
  for (uint32_t i = 0U; i < 3U; ++i)
  {
    if (raw.acc[i] == INT16_MAX || raw.acc[i] == INT16_MIN)
    {
      sat = static_cast<uint8_t>(sat | (1U << i));
    }
    if (raw.gyro[i] == INT16_MAX || raw.gyro[i] == INT16_MIN)
    {
      sat = static_cast<uint8_t>(sat | (1U << (3U + i)));
    }
  }
  raw.saturated = sat;

  ImuData data        = ImuData();
  float   temperature = 0.0f;
  convert(raw, data, temperature);

  // 自动刷新任务与手动调用、snapshot() 可能并发：写回放在临界区里（几十字节拷贝）
  taskENTER_CRITICAL();
  _raw_data    = raw;
  _data        = data;
  _temperature = temperature;
  if ((sat & 0x07U) != 0U)
  {
    ++_acc_sat_cnt;
  }
  if ((sat & 0x38U) != 0U)
  {
    ++_gyro_sat_cnt;
  }
  taskEXIT_CRITICAL();
  return Status::OK;
}

void DeviceBmi088::snapshot(ImuData &data, Bmi088RawData *raw) const
{
  taskENTER_CRITICAL();
  data = _data;
  if (raw != nullptr)
  {
    *raw = _raw_data;
  }
  taskEXIT_CRITICAL();
}

void DeviceBmi088::convert(const Bmi088RawData &raw, ImuData &data, float &temperature) const
{
  data.acc.x = static_cast<float>(raw.acc[0]) * _acc_scale;
  data.acc.y = static_cast<float>(raw.acc[1]) * _acc_scale;
  data.acc.z = static_cast<float>(raw.acc[2]) * _acc_scale;

  data.gyro.roll  = static_cast<float>(raw.gyro[0]) * _gyro_scale;
  data.gyro.pitch = static_cast<float>(raw.gyro[1]) * _gyro_scale;
  data.gyro.yaw   = static_cast<float>(raw.gyro[2]) * _gyro_scale;

  temperature = static_cast<float>(raw.temp) * 0.125f + 23.0f;
}

// ----------------


// ---------------- 自动刷新任务 ----------------

void DeviceBmi088::task_entry(void *argument)
{
  static_cast<DeviceBmi088 *>(argument)->run();
}

void DeviceBmi088::run()
{
  SysFlagWaitRunning(); // 初始化期有失败时这里让任务自行退出

  for (;;)
  {
    // 陀螺仪数据就绪中断唤醒；没有中断（没开 drdy_int、宿主没转调 EXTI）时最多等 10 ms 也刷新一次
    (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10U));
    if (!_init)
    {
      continue; // 正在重新 init()
    }
    if (update() == Status::OK)
    {
      TaskHandle_t task = _notify_task;
      if (task != nullptr)
      {
        (void)xTaskNotify(task, UPDATED_BIT, eSetBits);
      }
    }
  }
}

// ----------------


// ---------------- 数据就绪中断 ----------------

void DeviceBmi088::attach_task(TaskHandle_t task)
{
  _notify_task = task;
}

void DeviceBmi088::on_acc_drdy_isr(BaseType_t *pxHigherPriorityTaskWoken)
{
  ++_acc_drdy_cnt;
  if (_worker != nullptr)
  {
    return; // 自动模式按陀螺仪节奏刷新，加速度计中断只计数
  }
  TaskHandle_t task = _notify_task;
  if (task != nullptr)
  {
    xTaskNotifyFromISR(task, ACC_DRDY_BIT, eSetBits, pxHigherPriorityTaskWoken);
  }
}

void DeviceBmi088::on_gyro_drdy_isr(BaseType_t *pxHigherPriorityTaskWoken)
{
  ++_gyro_drdy_cnt;
  TaskHandle_t worker = _worker;
  if (worker != nullptr)
  {
    vTaskNotifyGiveFromISR(worker, pxHigherPriorityTaskWoken);
    return;
  }
  TaskHandle_t task = _notify_task;
  if (task != nullptr)
  {
    xTaskNotifyFromISR(task, GYRO_DRDY_BIT, eSetBits, pxHigherPriorityTaskWoken);
  }
}

// ----------------


// ---------------- 私有工具 ----------------

void DeviceBmi088::delay_ms(uint32_t ms)
{
  if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED)
  {
    HAL_Delay(ms);
  }
  else
  {
    const TickType_t ticks = pdMS_TO_TICKS(ms);
    vTaskDelay(ticks > 0U ? ticks : 1U);
  }
}

Status DeviceBmi088::write_verify(BspSpiDevice &dev, uint8_t reg, uint8_t value, uint8_t mask)
{
  Status st = dev.write_reg8(reg, value);
  if (st != Status::OK)
  {
    return st;
  }
  delay_ms(1U);

  uint8_t back = 0U;
  st           = dev.read_reg8(reg, back);
  if (st != Status::OK)
  {
    return st;
  }
  return ((back & mask) == (value & mask)) ? Status::OK : Status::IO_ERROR;
}

// ----------------

#endif // HAL_SPI_MODULE_ENABLED

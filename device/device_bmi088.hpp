/**
 * @file device_bmi088.hpp
 * @author ChoseB
 * @brief BMI088 六轴 IMU 设备驱动（加速度计 + 陀螺仪，SPI）
 * @version 0.1
 * @date 2026-10-20
 *
 * @copyright Copyright (c) 2026
 *
 * @details BMI088 是两颗独立的芯片：加速度计与陀螺仪各有片选、各有寄存器表，挂在同一条 SPI 总线上。
 *          本类绑定两个 BspSpiDevice（不拥有），负责上电配置、读数换算、姿态解算与数据就绪中断计数。
 *          与 DjiMotor 一样，读数存在对象内部并由库自动刷新：Config::auto_update = true（默认）时，init() 建一个内部任务，
 *          每次陀螺仪数据就绪就刷新一帧；用户直接读 snapshot() / data()（IMU 通用数据 ImuData，imu_definition.hpp）、
 *          raw_data()（寄存器原始值）、temperature()（芯片温度）。
 *          Config::attitude = true（默认）时，内部任务每帧再用 VQF（algorithm/vqf.hpp，六轴：陀螺仪 + 加速度计，带陀螺仪零偏估计）
 *          解算姿态，和 gyro / acc 一起写进 ImuData 的 quaternion / euler_angle / euler_radian。
 *          安装方向旋转、温控不在这里（属于上层）。
 *
 * @note 用法（自动刷新，默认）：
 *
 *   // 1) 两个片选设备（见 bsp_spi.hpp）；加速度计读寄存器时地址后有 1 个无效字节
 *   BspSpiDevice bmi088_acc_spi(bsp_spi2, {GYRO_ACC_CS_GPIO_Port, GYRO_ACC_CS_Pin, 0x80U, 1U});
 *   BspSpiDevice bmi088_gyro_spi(bsp_spi2, {GYRO_GYRO_CS_GPIO_Port, GYRO_GYRO_CS_Pin, 0x80U, 0U});
 *
 *   // 2) 设备实例：{加速度计量程, 陀螺仪量程, 加速度计 ODR, 陀螺仪 ODR / 带宽, 是否开数据就绪中断, 是否自动刷新, 是否解算姿态}
 *   DeviceBmi088 imu_bmi088(bmi088_acc_spi, bmi088_gyro_spi, {DeviceBmi088::AccRange::G24, DeviceBmi088::GyroRange::DPS2000});
 *
 *   // 3) 初始化阶段，SPI 总线与两个片选设备之后（自动刷新任务在这里创建，调度器启动、system_start() 之后开始刷新）
 *   configASSERT(imu_bmi088.init() == Status::OK);
 *
 *   // 4) 数据就绪中断：库不占用 HAL_GPIO_EXTI_Callback（宿主的按键等也用 EXTI），由宿主转调。
 *   //    没转调时自动刷新任务退化为每 10 ms 刷新一次（默认陀螺仪 1000 Hz，即每 1 ms 刷新一帧）
 *   extern "C" void HAL_GPIO_EXTI_Callback(uint16_t pin)
 *   {
 *     BaseType_t woken = pdFALSE;
 *     if (pin == GYRO_ACC_INT_Pin)  imu_bmi088.on_acc_drdy_isr(&woken);
 *     if (pin == GYRO_GYRO_INT_Pin) imu_bmi088.on_gyro_drdy_isr(&woken);
 *     portYIELD_FROM_ISR(woken);
 *   }
 *
 *   // 5) 取数据（任意任务）：snapshot() 拷一份同一帧的数据
 *   ImuData d;
 *   imu_bmi088.snapshot(d);             // d.gyro.yaw / pitch / roll rad/s，d.acc.x / y / z m/s²，d.euler_angle 度，d.quaternion w/x/y/z
 *   float t = imu_bmi088.temperature(); // °C
 *
 *   // 6) 想跟着数据节奏走（例如姿态解算）：attach_task() 后，每刷新一次就收到 UPDATED_BIT
 *   imu_bmi088.attach_task(xTaskGetCurrentTaskHandle());
 *   xTaskNotifyWait(0U, DeviceBmi088::UPDATED_BIT, nullptr, portMAX_DELAY);
 *
 * @note 手动刷新：Config::auto_update = false 时不建任务，attach_task() 的任务在数据就绪中断里收到 ACC_DRDY_BIT / GYRO_DRDY_BIT，
 *       由调用方自己 update()。临时对象（测试里切量程等）必须用手动模式。手动模式不解算姿态（update() 只刷新读数）。
 * @note 自动刷新的负载：每次 update() 约 75 µs（-O2）~ 140 µs（-O0）；默认陀螺仪 1000 Hz（HZ1000_BW116）时约占 CPU 7.5%~14%，
 *       改成 2000 Hz（HZ2000_BW230 / HZ2000_BW532）翻倍，嫌高可以再调低。任务优先级 / 栈深见 BMI088_TASK_PRIORITY / BMI088_TASK_STACK。
 *       姿态解算另加每帧一次 VQF 双精度更新（H7 有双精度 FPU）。
 * @warning 自动刷新的对象 init() 成功后必须存活到系统停止（内部任务持有它的指针），和 DjiMotor 一样。
 *
 * @note 姿态（Config::attitude）：
 *       - 坐标系：VQF 的六轴姿态，参考系 z 轴竖直向上、航向（yaw 零点）为上电时任意值；没有磁力计，yaw 会随零偏估计残差慢慢漂。
 *         quaternion 为“芯片本体 → 参考系”的旋转（w / x / y / z），欧拉角按 Z-Y-X（yaw → pitch → roll）分解，与 gyro.yaw / pitch / roll 同轴。
 *         注意 pitch 是绕芯片 Y 轴的右手转角：芯片 X 轴朝上抬时为负。pitch = ±90° 处 yaw / roll 万向节锁，控制请用四元数（algorithm/quaternion.hpp）。
 *       - 节奏：VQF 按陀螺仪 ODR 的周期积分，所以只在陀螺仪数据就绪中断唤醒的那一帧解算；没有中断（没开 drdy_int、宿主没转调 EXTI）
 *         时内部任务每 10 ms 超时刷新一次读数，但姿态不更新。两次刷新之间漏掉的数据就绪按最新一帧补积分，漏帧数见 attitude_skip_count()。
 *       - 收敛：上电后倾角约在 tauAcc（3 s）量级内收敛；陀螺仪零偏在静止约 1.5 s 后开始快速估计，静止上电最稳。
 *       - data().gyro 仍是未去零偏的原始换算值；姿态内部用的是去零偏后的角速度。
 *
 * @note 坐标轴为芯片本体坐标（数据手册 X / Y / Z），安装方向的旋转由上层处理：
 *       acc.x / y / z = 芯片 X / Y / Z；gyro.roll / pitch / yaw = 绕芯片 X / Y / Z 的角速度。
 * @note init() 需要等待芯片复位 / 上电（共约 100 ms）：调度器启动前用 HAL_Delay()，之后用 vTaskDelay()。
 * @warning 读写接口内部要取 SPI 总线锁、阻塞等传输完成，只能在任务上下文调用（init() 也可在调度器启动前调用）。
 */

#ifndef __DEVICE_BMI088_HPP__
#define __DEVICE_BMI088_HPP__

#include "stm32h7xx_hal.h" // IWYU pragma: keep（HAL 模块开关来自 stm32h7xx_hal_conf.h）

// 宿主工程没在 CubeMX 里开 SPI（HAL_SPI_MODULE_ENABLED 未定义）时，本模块整体不参与编译
#ifdef HAL_SPI_MODULE_ENABLED

#include <stdint.h>

#include "FreeRTOS.h" // IWYU pragma: keep
#include "task.h"     // IWYU pragma: keep（TaskHandle_t）

#include "bsp_spi.hpp"        // 两个片选设备
#include "imu_definition.hpp" // IMU 通用数据
#include "status.hpp"         // 统一状态码

class VQF; // algorithm/vqf.hpp：姿态解算，只在 .cpp 里用


/**
 * @brief BMI088 寄存器原始值（补码），BMI088 特有；换算后的物理量见 ImuData
 */
struct Bmi088RawData
{
  int16_t acc[3];    ///< ACC_X/Y/Z
  int16_t gyro[3];   ///< RATE_X/Y/Z
  int16_t temp;      ///< 11 位温度值（已符号扩展）
  uint8_t saturated; ///< 本帧顶到满量程（32767 / -32768）的轴：bit0~2 = 加速度 X/Y/Z，bit3~5 = 角速度 X/Y/Z；非 0 时该轴读数被截断
};


/** @brief BMI088 设备类（不拥有 SPI 设备；自动刷新模式下 init() 创建一个内部任务） */
class DeviceBmi088
{
public:
  // ---------------- 配置 ----------------

  /** @brief 加速度计量程（枚举值 = ACC_RANGE 寄存器值） */
  enum class AccRange : uint8_t
  {
    G3  = 0x00U, ///< ±3 g
    G6  = 0x01U, ///< ±6 g
    G12 = 0x02U, ///< ±12 g
    G24 = 0x03U, ///< ±24 g
  };

  /** @brief 陀螺仪量程（枚举值 = GYRO_RANGE 寄存器值） */
  enum class GyroRange : uint8_t
  {
    DPS2000 = 0x00U, ///< ±2000 °/s
    DPS1000 = 0x01U, ///< ±1000 °/s
    DPS500  = 0x02U, ///< ±500 °/s
    DPS250  = 0x03U, ///< ±250 °/s
    DPS125  = 0x04U, ///< ±125 °/s
  };

  /** @brief 加速度计输出数据率（枚举值 = ACC_CONF 的 ODR 位；滤波固定为 normal） */
  enum class AccOdr : uint8_t
  {
    HZ12_5 = 0x05U, ///< 12.5 Hz
    HZ25   = 0x06U, ///< 25 Hz
    HZ50   = 0x07U, ///< 50 Hz
    HZ100  = 0x08U, ///< 100 Hz
    HZ200  = 0x09U, ///< 200 Hz
    HZ400  = 0x0AU, ///< 400 Hz
    HZ800  = 0x0BU, ///< 800 Hz
    HZ1600 = 0x0CU, ///< 1600 Hz
  };

  /** @brief 陀螺仪输出数据率 / 滤波带宽（枚举值 = GYRO_BANDWIDTH 寄存器值） */
  enum class GyroOdr : uint8_t
  {
    HZ2000_BW532 = 0x00U, ///< 2000 Hz，带宽 532 Hz
    HZ2000_BW230 = 0x01U, ///< 2000 Hz，带宽 230 Hz
    HZ1000_BW116 = 0x02U, ///< 1000 Hz，带宽 116 Hz
    HZ400_BW47   = 0x03U, ///< 400 Hz，带宽 47 Hz
    HZ200_BW23   = 0x04U, ///< 200 Hz，带宽 23 Hz
    HZ100_BW12   = 0x05U, ///< 100 Hz，带宽 12 Hz
    HZ200_BW64   = 0x06U, ///< 200 Hz，带宽 64 Hz
    HZ100_BW32   = 0x07U, ///< 100 Hz，带宽 32 Hz
  };

  /** @brief 设备配置（可匿名按序传入：{acc_range, gyro_range, acc_odr, gyro_odr, drdy_int, auto_update, attitude}） */
  struct Config
  {
    /** @brief 按序构造（参数顺序 = 字段顺序） */
    Config(AccRange acc_range = AccRange::G24, GyroRange gyro_range = GyroRange::DPS2000, AccOdr acc_odr = AccOdr::HZ1600, GyroOdr gyro_odr = GyroOdr::HZ1000_BW116, bool drdy_int = true, bool auto_update = true, bool attitude = true) : acc_range(acc_range), gyro_range(gyro_range), acc_odr(acc_odr), gyro_odr(gyro_odr), drdy_int(drdy_int), auto_update(auto_update), attitude(attitude)
    {
    }

    AccRange  acc_range;  ///< 加速度计量程
    GyroRange gyro_range; ///< 陀螺仪量程
    AccOdr    acc_odr;    ///< 加速度计输出数据率
    GyroOdr   gyro_odr;   ///< 陀螺仪输出数据率 / 带宽
    bool      drdy_int;   ///< true = 加速度计 INT1、陀螺仪 INT3 输出数据就绪（推挽、高有效、对应 EXTI 上升沿）
    bool      auto_update;///< true = init() 建内部任务，每次陀螺仪数据就绪自动 update()（没有中断时每 10 ms 一次）；false = 调用方自己 update()
    bool      attitude;   ///< true = 内部任务每帧用 VQF 解算姿态，写 ImuData 的四元数 / 欧拉角；只在 auto_update 下生效，且要有陀螺仪数据就绪中断
  };

  static constexpr uint32_t ACC_DRDY_BIT  = 1UL << 0; ///< 手动模式下 attach_task() 的任务通知位：加速度计数据就绪
  static constexpr uint32_t GYRO_DRDY_BIT = 1UL << 1; ///< 手动模式下 attach_task() 的任务通知位：陀螺仪数据就绪
  static constexpr uint32_t UPDATED_BIT   = 1UL << 2; ///< 自动模式下 attach_task() 的任务通知位：内部任务刚刷新完一帧

  // ----------------
  // ---------------- 构造与初始化 ----------------

  /** @brief 必须绑定两个 SPI 设备，禁用默认构造 */
  DeviceBmi088() = delete;

  /**
   * @brief 构造函数（只做赋值）
   * @param acc  加速度计片选设备（read_dummy 必须为 1）
   * @param gyro 陀螺仪片选设备（read_dummy 必须为 0）
   * @param cfg  量程、数据率、中断
   */
  DeviceBmi088(BspSpiDevice &acc, BspSpiDevice &gyro, const Config &cfg = Config());

  /**
   * @brief 上电配置：加速度计切到 SPI 模式（一次假读）→ 两颗软复位 → 校验芯片 ID
   *        → 加速度计上电 → 量程 / 数据率 → 数据就绪中断 → 回读校验
   *
   * @note 在初始化阶段调用（SPI 总线与两个片选设备 init() 之后）；可以重复调用（重新复位、重新配置），
   *       期间自动刷新任务跳过刷新。自动模式下第一次成功时创建内部任务与姿态解算器（之后不再重复创建，重复 init() 时姿态从头解算）。
   *
   * @return OK=成功；NOT_INIT=SPI 设备未 init；IO_ERROR=芯片 ID 不对（没焊 / 接错 / 片选接反）、配置回读不一致、
   *         刷新任务创建失败或姿态解算器分配失败；TIMEOUT=SPI 传输超时
   */
  Status init();

  // ----------------
  // ---------------- 读数 ----------------

  /**
   * @brief 经 SPI 读一组加速度、角速度、温度，刷新原始值并按当前量程换算到内部数据
   *
   * @param timeout_ms 每次寄存器读取等总线锁、等传输完成的上限；与别的高优先级任务共用总线的低优先级任务应传更长的值（见 bsp_spi.hpp）
   * @return OK=已刷新；NOT_INIT=init() 没成功过；其余同 BspSpiDevice::read_reg()（失败时内部数据保持上一次的值）
   *
   * @note 只写 ImuData 的 gyro / acc，四元数与欧拉角保持原值（手动模式下一直是 0，不代表实际姿态）；
   *       姿态只由自动刷新任务解算（见文件头“姿态”）。
   * @note 自动模式下由内部任务调用，用户一般不用调；手动调用也安全（总线锁 + 临界区），但那一帧不进姿态解算。
   */
  Status update(uint32_t timeout_ms = SPI_TRANSFER_TIMEOUT_MS);

  /**
   * @brief 拷贝一份同一帧的数据（临界区内拷贝，跨任务读用它）
   * @param data 换算后的 IMU 通用数据
   * @param raw  可选：同一帧的寄存器原始值；nullptr = 不要
   */
  void snapshot(ImuData &data, Bmi088RawData *raw = nullptr) const;

  /**
   * @brief 获取只读 IMU 通用数据
   * @return 内部 ImuData 的常量引用。
   * @warning 返回实时引用而非快照：自动刷新时多个字段可能来自相邻两帧；需要同一帧的数据用 snapshot()。
   */
  const ImuData &data() const
  {
    return _data;
  }

  /**
   * @brief 获取 BMI088 寄存器原始值
   * @return 内部 Bmi088RawData 的常量引用。
   * @warning 同 data()。
   */
  const Bmi088RawData &raw_data() const
  {
    return _raw_data;
  }

  /** @brief 芯片温度，单位：°C（芯片约每 1.28 s 更新一次；update() 成功过之前为 0） */
  float temperature() const
  {
    return _temperature;
  }

  /**
   * @brief 加速度计饱和帧数：update() 读到任一轴为 32767 / -32768 的累计次数（只增不减）
   * @note 剧烈冲击下持续增加说明量程不够（默认 ±24 g 已是最大档）。
   */
  uint32_t acc_saturation_count() const
  {
    return _acc_sat_cnt;
  }

  /** @brief 陀螺仪饱和帧数（同上；±2000 °/s 是最大档） */
  uint32_t gyro_saturation_count() const
  {
    return _gyro_sat_cnt;
  }

  /** @brief update() 成功次数（自动 + 手动，只增不减）；自动模式下约等于陀螺仪数据就绪次数 */
  uint32_t update_count() const
  {
    return _update_cnt;
  }

  /** @brief update() 失败次数（只增不减；重新 init 期间的 NOT_INIT 也算） */
  uint32_t update_fail_count() const
  {
    return _update_fail_cnt;
  }

  /** @brief 最近一次 update() 的返回值 */
  Status last_status() const
  {
    return _last_status;
  }

  /**
   * @brief 姿态漏帧数：自动刷新时两次刷新之间陀螺仪数据就绪多于 1 次的累计多出次数（只增不减）
   * @note 漏掉的样本按最新一帧补积分，偶尔几次无妨；持续增加说明内部任务跟不上 ODR（优先级太低或 ODR 太高）。
   */
  uint32_t attitude_skip_count() const
  {
    return _attitude_skip_cnt;
  }

  // ----------------
  // ---------------- 数据就绪中断 ----------------

  /**
   * @brief 绑定一个任务接收通知（eSetBits）
   * @note 自动模式：内部任务每成功刷新一帧给它置 UPDATED_BIT；手动模式：数据就绪中断里给它置 ACC_DRDY_BIT / GYRO_DRDY_BIT。
   * @param task 任务句柄；nullptr = 解绑
   */
  void attach_task(TaskHandle_t task);

  /** @brief 加速度计数据就绪（INT1 上升沿，宿主 EXTI 回调里转调）：计数；手动模式下通知绑定的任务 */
  void on_acc_drdy_isr(BaseType_t *pxHigherPriorityTaskWoken);

  /** @brief 陀螺仪数据就绪（INT3 上升沿，宿主 EXTI 回调里转调）：计数；自动模式下唤醒内部任务，手动模式下通知绑定的任务 */
  void on_gyro_drdy_isr(BaseType_t *pxHigherPriorityTaskWoken);

  /** @brief 加速度计数据就绪中断累计次数 */
  uint32_t acc_drdy_count() const
  {
    return _acc_drdy_cnt;
  }

  /** @brief 陀螺仪数据就绪中断累计次数 */
  uint32_t gyro_drdy_count() const
  {
    return _gyro_drdy_cnt;
  }

  // ----------------

private:
  /** @brief 等 ms 毫秒：调度器启动前 HAL_Delay()，之后 vTaskDelay() */
  static void delay_ms(uint32_t ms);

  /**
   * @brief 写一个寄存器后读回比对（只比 mask 里的位）；不一致返回 IO_ERROR
   * @note 写后等 1 ms：加速度计在挂起模式下两次写之间至少要隔 450 µs。
   */
  static Status write_verify(BspSpiDevice &dev, uint8_t reg, uint8_t value, uint8_t mask = 0xFFU);

  /** @brief 原始值按当前量程换算（纯计算） */
  void convert(const Bmi088RawData &raw, ImuData &data, float &temperature) const;

  /**
   * @brief 读一帧、换算，可选解算姿态，最后一次性写回内部数据；并计数
   * @param gyro_steps 姿态解算要积分的陀螺仪样本数；0 = 不解算（手动 update()、超时唤醒）
   */
  Status refresh(uint32_t timeout_ms, uint32_t gyro_steps);

  /** @brief refresh() 的读取、换算与写回部分（计数在 refresh() 里） */
  Status read_frame(uint32_t timeout_ms, uint32_t gyro_steps);

  /** @brief 用本帧 gyro / acc 推进 VQF（陀螺仪积分 gyro_steps 次），把四元数与欧拉角填进 data */
  void solve_attitude(ImuData &data, uint32_t gyro_steps);

  /** @brief 自动刷新任务入口（参数为 this） */
  static void task_entry(void *argument);

  /** @brief 自动刷新任务主循环：等陀螺仪数据就绪（最多 10 ms）→ 刷新（+ 姿态）→ 通知绑定的任务 */
  void run();

  BspSpiDevice &_acc;  ///< 加速度计片选设备
  BspSpiDevice &_gyro; ///< 陀螺仪片选设备
  Config        _cfg;  ///< 配置

  ImuData       _data        = ImuData();       ///< IMU 通用数据（只填 gyro / acc）
  Bmi088RawData _raw_data    = Bmi088RawData(); ///< 寄存器原始值
  float         _temperature = 0.0f;            ///< 芯片温度 (°C)

  uint32_t _acc_sat_cnt  = 0U; ///< 加速度计饱和帧数
  uint32_t _gyro_sat_cnt = 0U; ///< 陀螺仪饱和帧数

  float _acc_scale  = 0.0f; ///< 原始值 → m/s²（init() 按量程算好）
  float _gyro_scale = 0.0f; ///< 原始值 → rad/s
  bool  _init       = false; ///< init() 是否成功过

  volatile uint32_t _update_cnt        = 0U;         ///< update() 成功次数
  volatile uint32_t _update_fail_cnt   = 0U;         ///< update() 失败次数
  volatile Status   _last_status       = Status::OK; ///< 最近一次 update() 的结果
  volatile uint32_t _attitude_skip_cnt = 0U;         ///< 姿态漏帧数

  VQF *_vqf = nullptr; ///< 姿态解算器（auto_update 且 attitude 时 init() 分配，之后不释放）

  TaskHandle_t volatile _worker        = nullptr; ///< 自动刷新任务（auto_update 且 init() 成功后才有）
  TaskHandle_t volatile _notify_task   = nullptr; ///< attach_task() 绑定的任务
  volatile uint32_t     _acc_drdy_cnt  = 0;       ///< INT1 次数
  volatile uint32_t     _gyro_drdy_cnt = 0;       ///< INT3 次数
};

#endif // HAL_SPI_MODULE_ENABLED

#endif // __DEVICE_BMI088_HPP__

/**
 * @file bsp_spi.hpp
 * @author ChoseB
 * @brief SPI 主机驱动：总线（BspSpi）+ 片选设备（BspSpiDevice），DMA 与轮询混合
 * @version 0.1
 * @date 2026-10-20
 *
 * @copyright Copyright (c) 2026
 *
 * @details 两层：
 *          - BspSpi 对应一路 SPI 外设（一个 HAL 句柄）：总线互斥锁、DMA 完成等待、弹跳缓冲、登记表；
 *          - BspSpiDevice 对应总线上的一个片选：每次调用自己取锁、拉片选、传输、放片选、放锁，
 *            另外提供「寄存器读写」这一最常见的器件访问方式。设备层（如 DeviceBmi088）只拿 BspSpiDevice。
 *
 *          传输方式按长度自动选：短于 Config::dma_threshold 的直接 HAL 轮询（几个字节时 DMA 的启动、中断、
 *          任务切换开销比传输本身还大）；不短于阈值的走 DMA，调用任务阻塞在完成信号量上，CPU 让给别的任务。
 *
 * @note 用法：
 *
 *   // 1) 总线实例写在板级配置里：句柄 + 是否用 DMA + DMA 阈值（字节）。
 *   //    用 DMA 时实例必须放进 .dma_buffer（实例里有弹跳缓冲，H7 的 DMA1 / DMA2 访问不到 DTCM）
 *   __attribute__((section(".dma_buffer"))) BspSpi bsp_spi2({&hspi2, true, 16U});
 *
 *   // 2) 片选设备：总线 + {片选端口, 片选引脚, 读标志位, 读时地址后的空字节数}
 *   BspSpiDevice bmi088_acc_spi(bsp_spi2, {GYRO_ACC_CS_GPIO_Port, GYRO_ACC_CS_Pin, 0x80U, 1U});
 *   BspSpiDevice bmi088_gyro_spi(bsp_spi2, {GYRO_GYRO_CS_GPIO_Port, GYRO_GYRO_CS_Pin, 0x80U, 0U});
 *
 *   // 3) 初始化阶段（system_init() 之后、system_start() 之前），总线先于设备
 *   configASSERT(bsp_spi2.init() == Status::OK);
 *   configASSERT(bmi088_acc_spi.init() == Status::OK);
 *
 *   // 4) 访问（任务上下文）
 *   uint8_t id = 0U;
 *   bmi088_gyro_spi.read_reg8(0x00U, id);             // 读 1 个寄存器
 *   bmi088_acc_spi.read_reg(0x12U, buf, 6U);          // 从 0x12 连读 6 字节（自动跳过 1 个空字节）
 *   bmi088_acc_spi.write_reg8(0x7DU, 0x04U);          // 写 1 个寄存器
 *   bmi088_acc_spi.transfer(tx, rx, n);               // 一次片选内的全双工传输（协议自己拼）
 *
 *   // 5) 一次片选内要分几步、中间还要做判断的事务：自己持锁、自己控片选
 *   bsp_spi2.lock();  dev 片选拉低 → bsp_spi2.transfer_locked(...) ×n → 片选拉高;  bsp_spi2.unlock();
 *
 * @note 同一总线上的所有设备共用 CubeMX 里配好的 CPOL / CPHA / 速率，本驱动不按设备切换模式；
 *       模式或速率不兼容的器件请挂在不同总线上。
 * @note 不能在中断里调用任何传输接口（要取锁、要阻塞等完成）。数据就绪中断里只通知任务，由任务来读。
 * @note 调度器启动前（board_init() 里设备 init() 读写寄存器时）：一律走轮询、不取锁。
 *       此时 BASEPRI 屏蔽了可调用 FreeRTOS API 的中断（SPI / DMA 中断优先级 ≥ 5），DMA 等不到完成。
 * @note 库定义了 HAL_SPI_TxRxCpltCallback、HAL_SPI_ErrorCallback，宿主工程不能再定义。
 *       CubeMX 里（use_dma = true 时）：SPI 的 RX / TX DMA（Normal、Byte）+ SPI 全局中断，优先级 ≥ 5。
 * @note CubeMX 里 SPI 的 MasterKeepIOState 必须为 ENABLE：一次片选内的多段传输（地址、无效字节、数据）之间 HAL 会关 SPE，
 *       不保持 IO 时 SCK 浮空，CPOL = 1 的器件会看到多余的时钟沿。
 * @note 库假定 D-Cache 关闭，DMA 缓冲不做 Cache 维护。
 * @note 所有超时参数单位都是毫秒；传 portMAX_DELAY 表示一直等。
 * @note 总线锁不按先来后到：互斥量释放时不直接交给等待者，高优先级任务频繁访问同一总线时，
 *       低优先级任务可能等锁超过默认的 SPI_TRANSFER_TIMEOUT_MS（实测 4 个任务争用时最低优先级者等过 9.3 ms 后超时）。
 *       低优先级的使用者应自己传更长的 timeout_ms（例如 50 ms，或 portMAX_DELAY）；默认值按高优先级、周期性访问取值。
 */

#ifndef __BSP_SPI_HPP__
#define __BSP_SPI_HPP__

#include "stm32h7xx_hal.h" // IWYU pragma: keep（HAL 模块开关来自 stm32h7xx_hal_conf.h）

// 宿主工程没在 CubeMX 里开 SPI（HAL_SPI_MODULE_ENABLED 未定义）时，本模块整体不参与编译
#ifdef HAL_SPI_MODULE_ENABLED

#include <stddef.h>
#include <stdint.h>

#include "FreeRTOS.h" // IWYU pragma: keep
#include "semphr.h"   // IWYU pragma: keep（总线锁、完成信号量）

#include "bsp_gpio.hpp"                 // 片选引脚
#include "hardware_config_defaults.hpp" // 可配置项（用户 hardware_config.h + 默认值）
#include "status.hpp"                   // 统一状态码


/* ==================== 总线 ==================== */

/**
 * @brief SPI 总线（一路 SPI 外设，主机模式）
 *
 * @note 登记时机：init() 时登记进静态登记表；HAL 回调按 hspi 句柄查表，
 *       查不到的 SPI（宿主工程自己管的）直接忽略。登记只发生在初始化阶段。
 */
class BspSpi
{
public:
  // ---------------- 配置 ----------------

  /** @brief 总线配置（可匿名按序传入：{hspi, use_dma, dma_threshold}） */
  struct Config
  {
    /** @brief 按序构造（参数顺序 = 字段顺序） */
    Config( SPI_HandleTypeDef *hspi = nullptr, 
            bool use_dma = true, 
            uint16_t dma_threshold = 16U) : 
            hspi(hspi), 
            use_dma(use_dma), 
            dma_threshold(dma_threshold)
    {
    }

    SPI_HandleTypeDef *hspi;          ///< SPI 句柄（CubeMX 配成全双工主机、软件 NSS）
    bool               use_dma;       ///< true = 不短于阈值的传输走 DMA；false = 全部轮询（CubeMX 可以不配 DMA / 中断，实例也不必放 .dma_buffer）
    uint16_t           dma_threshold; ///< 走 DMA 的最小长度（字节）；更短的轮询
  };

  /** @brief 诊断计数：只增不减；timeout / error / lock_timeout 一切正常时应为 0 */
  struct Diag
  {
    uint32_t poll_cnt;         ///< 轮询方式完成的传输段数
    uint32_t dma_cnt;          ///< DMA 方式完成的传输段数
    uint32_t bounce_cnt;       ///< 调用方缓冲 DMA 访问不到、经弹跳缓冲中转的段数
    uint32_t timeout_cnt;      ///< 传输超时（已 Abort）的次数
    uint32_t error_cnt;        ///< HAL 返回错误或进错误回调的次数
    uint32_t lock_timeout_cnt; ///< 等总线锁超时的次数
  };

  // ----------------
  // ---------------- 构造与初始化 ----------------

  /** @brief 构造函数（只做赋值，FreeRTOS 资源创建推迟到 init()） */
  BspSpi(const Config &cfg);

  /** @brief 析构：从登记表摘除并释放锁与信号量 */
  ~BspSpi();

  // 登记表里存的是对象地址，复制会让两个对象争同一个句柄
  BspSpi(const BspSpi &)            = delete;
  BspSpi &operator=(const BspSpi &) = delete;

  /**
   * @brief 检查配置、登记，创建总线锁（互斥量，带优先级继承）与 DMA 完成信号量
   *
   * @note 在初始化阶段调用（system_init() 之后、system_start() 之前），先于挂在它上面的 BspSpiDevice::init()。
   *       不能重复调用（运行中重建锁会让正在等锁的任务失效）。
   *
   * @return OK=成功；IO_ERROR=资源创建失败；
   *         BAD_ARG=句柄为空、已被另一个实例登记、use_dma 却没链接 RX / TX DMA、
   *                 use_dma 却不在 DMA 可达的内存里（没放进 .dma_buffer）、已经 init 过；
   *         FULL=SPI 登记表已满
   */
  Status init();

  /** @brief 是否已 init() 成功 */
  bool is_init() const;

  // ----------------
  // ---------------- 事务接口（多数情况用 BspSpiDevice，不直接调这些） ----------------

  /**
   * @brief 取总线锁（同一任务不可重入）
   *
   * @param timeout_ms 最多等多久；portMAX_DELAY 表示一直等
   * @return OK=拿到了（调度器启动前直接返回 OK）；TIMEOUT=超时（计入 lock_timeout_cnt）；NOT_INIT=未 init
   */
  Status lock(uint32_t timeout_ms = SPI_TRANSFER_TIMEOUT_MS);

  /** @brief 放总线锁（调度器启动前为空操作） */
  void unlock();

  /**
   * @brief 全双工传输一段（调用方必须已持锁，并已拉低片选；片选由调用方管）
   *
   * @param tx         发送数据；nullptr = 发 0xFF 填充
   * @param rx         接收缓冲；nullptr = 丢弃收到的字节
   * @param len        字节数（> 0）
   * @param timeout_ms 每个 DMA 段等完成的上限；轮询段同样用它作 HAL 超时
   * @return OK=完成；BAD_ARG=len 为 0；NOT_INIT=未 init；
   *         TIMEOUT=超时（已 HAL_SPI_Abort，总线可继续用）；IO_ERROR=HAL 错误（模式错误、溢出、DMA 错误等）
   *
   * @note len < dma_threshold、或 use_dma = false、或调度器未启动 → 轮询；否则 DMA。
   * @note 两个方向各自决定：DMA 时缓冲在 DMA 可达内存里就直接用（零拷贝），在 DTCM（例如任务栈、默认 .bss）
   *       就经实例里的弹跳缓冲中转；tx / rx 为空时也用弹跳缓冲（填 0xFF / 收了丢掉）。任一方向用到弹跳缓冲时
   *       按 SPI_DMA_BUFFER_SIZE 分段，片选全程保持，对器件来说是一整帧。
   */
  Status transfer_locked(const uint8_t *tx, uint8_t *rx, size_t len, uint32_t timeout_ms = SPI_TRANSFER_TIMEOUT_MS);

  // ----------------
  // ---------------- ISR 入口（HAL 回调按句柄查表后调用） ----------------

  /** @brief DMA 传输完成（HAL_SPI_TxRxCpltCallback）：释放完成信号量 */
  void on_txrx_cplt_isr(BaseType_t *pxHigherPriorityTaskWoken);

  /** @brief 传输出错（HAL_SPI_ErrorCallback）：记错误并释放完成信号量，等待方返回 IO_ERROR */
  void on_error_isr(BaseType_t *pxHigherPriorityTaskWoken);

  // ----------------
  // ---------------- 查询 ----------------

  /** @brief 绑定的 SPI 句柄 */
  SPI_HandleTypeDef *handle() const
  {
    return _hspi;
  }

  /** @brief 读诊断计数 */
  Diag diag() const;

  /** @brief 已登记的 SPI 实例数 */
  static uint32_t instance_count();

  /** @brief 第 index 个已登记实例；越界返回 nullptr */
  static BspSpi *instance_at(uint32_t index);

  /** @brief 按 HAL 句柄查已登记实例；查不到返回 nullptr（ISR 中可调用） */
  static BspSpi *find(const SPI_HandleTypeDef *hspi);

  // ----------------

private:
  // ---------------- 私有实现 ----------------

  /** @brief 一段轮询传输（HAL_SPI_TransmitReceive） */
  Status transfer_poll(const uint8_t *tx, uint8_t *rx, size_t len, uint32_t timeout_ms);

  /** @brief 一段 DMA 传输（len ≤ 0xFFFF；缓冲必须 DMA 可达）：启动后阻塞等完成信号量 */
  Status transfer_dma(const uint8_t *tx, uint8_t *rx, size_t len, uint32_t timeout_ms);

  /**
   * @brief 把本实例加入登记表
   * @return OK=已登记；BAD_ARG=句柄为空或已被另一个实例登记；FULL=登记表满
   */
  Status register_self();

  /** @brief 从登记表摘除本实例（init() 中途失败、析构时调用；不在表里则无操作） */
  void unregister_self();

  /** @brief 本实例是否已 init() 成功 */
  bool _init = false;

  SPI_HandleTypeDef *_hspi;          ///< SPI 句柄
  bool               _use_dma;       ///< 是否启用 DMA
  uint16_t           _dma_threshold; ///< 走 DMA 的最小长度（字节）

  SemaphoreHandle_t _lock = nullptr; ///< 总线锁：一次片选事务内独占总线
  SemaphoreHandle_t _done = nullptr; ///< DMA 完成信号量：完成 / 错误回调里释放

  volatile bool _xfer_error = false; ///< 本次 DMA 段是否进了错误回调

  // 诊断量：任务里累加（error 也可能在中断里），任务里读取
  volatile uint32_t _poll_cnt         = 0; ///< 轮询完成的段数
  volatile uint32_t _dma_cnt          = 0; ///< DMA 完成的段数
  volatile uint32_t _bounce_cnt       = 0; ///< 经弹跳缓冲的段数
  volatile uint32_t _timeout_cnt      = 0; ///< 超时次数
  volatile uint32_t _error_cnt        = 0; ///< 错误次数
  volatile uint32_t _lock_timeout_cnt = 0; ///< 等锁超时次数

  uint8_t _tx_bounce[SPI_DMA_BUFFER_SIZE]; ///< DMA 发送弹跳缓冲（也用来放 0xFF 填充）
  uint8_t _rx_bounce[SPI_DMA_BUFFER_SIZE]; ///< DMA 接收弹跳缓冲（也用来丢弃收到的字节）

  /** @brief 登记表容量：SPI_MAX_INSTANCES（默认 6，STM32H723 有 SPI1~6） */
  static constexpr uint32_t MAX_INSTANCES = SPI_MAX_INSTANCES;

  static BspSpi  *_registry[MAX_INSTANCES]; ///< 已登记实例（只在初始化阶段写，中断与任务只读）
  static uint32_t _registry_num;            ///< 已登记数量

  // ----------------
};


/* ==================== 片选设备 ==================== */

/**
 * @brief 总线上的一个 SPI 设备（一个片选）
 *
 * @note 每个公有传输接口 = 取总线锁 → 片选拉低 → 一段或几段 transfer_locked() → 片选拉高 → 放锁，
 *       所以不同任务访问同一总线上的不同设备是安全的，片选之间不会交叠。
 * @note 寄存器访问约定（大多数传感器都是这样）：地址字节的 read_flag 位置 1 表示读、清 0 表示写；
 *       读时地址后面跟 read_dummy 个无效字节再出数据（BMI088 加速度计是 1，陀螺仪是 0）。
 *       不符合这个约定的器件（如 SPI Flash 的命令 + 3 字节地址）用 transfer() / write_then_read()。
 */
class BspSpiDevice
{
public:
  // ---------------- 配置 ----------------

  /** @brief 设备配置（可匿名按序传入：{cs_port, cs_pin, read_flag, read_dummy}） */
  struct Config
  {
    /** @brief 按序构造（参数顺序 = 字段顺序） */
    Config( GPIO_TypeDef *cs_port = nullptr, 
            uint16_t cs_pin = 0U, 
            uint8_t read_flag = 0x80U, 
            uint8_t read_dummy = 0U) : 
            cs_port(cs_port), 
            cs_pin(cs_pin), 
            read_flag(read_flag), 
            read_dummy(read_dummy)
    {
    }

    GPIO_TypeDef *cs_port;    ///< 片选端口（CubeMX 配成推挽输出、初始高电平）
    uint16_t      cs_pin;     ///< 片选引脚掩码（GPIO_PIN_x），低有效
    uint8_t       read_flag;  ///< 读寄存器时在地址字节上置的位（写时清掉）
    uint8_t       read_dummy; ///< 读寄存器时地址后面跟的无效字节数
  };

  // ----------------
  // ---------------- 构造与初始化 ----------------

  /** @brief 必须绑定一条总线，禁用默认构造 */
  BspSpiDevice() = delete;

  /**
   * @brief 构造函数（只做赋值）
   * @param bus 所在总线（不拥有）
   * @param cfg 片选与寄存器访问约定
   */
  BspSpiDevice(BspSpi &bus, const Config &cfg);

  /**
   * @brief 检查总线已 init、片选合法，并把片选拉高
   * @return OK=成功；NOT_INIT=总线还没 init；BAD_ARG=片选端口 / 引脚非法
   */
  Status init();

  // ----------------
  // ---------------- 传输 ----------------

  /**
   * @brief 一次片选内的全双工传输
   *
   * @param tx  发送数据；nullptr = 发 0xFF
   * @param rx  接收缓冲；nullptr = 丢弃
   * @param len 字节数（> 0）
   * @param timeout_ms 等锁、以及每段等完成的上限
   * @return OK；BAD_ARG=len 为 0；NOT_INIT=设备未 init；TIMEOUT=等锁或传输超时；IO_ERROR=HAL 错误
   */
  Status transfer(const uint8_t *tx, uint8_t *rx, size_t len, uint32_t timeout_ms = SPI_TRANSFER_TIMEOUT_MS);

  /**
   * @brief 一次片选内先发 tx_len 字节、再收 rx_len 字节（收的时候发 0xFF）
   *
   * @param tx     要先发的数据（命令 / 地址），不能为 nullptr
   * @param tx_len 字节数（> 0）
   * @param rx     接收缓冲，不能为 nullptr
   * @param rx_len 字节数（> 0）
   * @param timeout_ms 同 transfer()
   * @return 同 transfer()；tx / rx 为空或长度为 0 返回 BAD_ARG
   */
  Status write_then_read(const uint8_t *tx, size_t tx_len, uint8_t *rx, size_t rx_len, uint32_t timeout_ms = SPI_TRANSFER_TIMEOUT_MS);

  /**
   * @brief 从 reg 起连读 len 字节（地址置 read_flag，自动跳过 read_dummy 个无效字节）
   * @return 同 transfer()；data 为空或 len 为 0 返回 BAD_ARG
   */
  Status read_reg(uint8_t reg, uint8_t *data, size_t len, uint32_t timeout_ms = SPI_TRANSFER_TIMEOUT_MS);

  /**
   * @brief 从 reg 起连写 len 字节（地址清 read_flag）
   * @return 同 transfer()；data 为空或 len 为 0 返回 BAD_ARG
   */
  Status write_reg(uint8_t reg, const uint8_t *data, size_t len, uint32_t timeout_ms = SPI_TRANSFER_TIMEOUT_MS);

  /** @brief 读 1 个寄存器 */
  Status read_reg8(uint8_t reg, uint8_t &value, uint32_t timeout_ms = SPI_TRANSFER_TIMEOUT_MS);

  /** @brief 写 1 个寄存器 */
  Status write_reg8(uint8_t reg, uint8_t value, uint32_t timeout_ms = SPI_TRANSFER_TIMEOUT_MS);

  // ----------------
  // ---------------- 手动事务用 ----------------

  /** @brief 所在总线（手动事务：bus().lock() → select() → bus().transfer_locked() … → deselect() → bus().unlock()） */
  BspSpi &bus() const
  {
    return _bus;
  }

  /** @brief 片选拉低（调用方必须已持总线锁） */
  void select() const
  {
    _cs.reset();
  }

  /** @brief 片选拉高 */
  void deselect() const
  {
    _cs.set();
  }

  // ----------------

private:
  /**
   * @brief 一次片选内的完整事务：取锁 → 片选低 → 头部（命令 / 地址）→ 跳过 skip 字节 → 数据段 → 片选高 → 放锁
   *
   * @param head     头部字节（发送，收到的丢弃）；head_len = 0 时不发
   * @param head_len 头部长度
   * @param skip     头部之后要时钟出去并丢弃的字节数（读寄存器的无效字节）
   * @param tx       数据段发送；nullptr = 发 0xFF
   * @param rx       数据段接收；nullptr = 丢弃
   * @param len      数据段长度（> 0）
   * @param timeout_ms 等锁、以及每段的上限
   */
  Status transaction(const uint8_t *head, size_t head_len, size_t skip, const uint8_t *tx, uint8_t *rx, size_t len, uint32_t timeout_ms);

  BspSpi &_bus;            ///< 所在总线
  BspGpio _cs;             ///< 片选引脚（低有效）
  uint8_t _read_flag;      ///< 读标志位
  uint8_t _read_dummy;     ///< 读时的无效字节数
  bool    _init = false;   ///< 是否已 init() 成功
};

#endif // HAL_SPI_MODULE_ENABLED

#endif // __BSP_SPI_HPP__

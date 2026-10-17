/**
 * @file bsp_uart.hpp
 * @author Rh
 * @brief 串口驱动（FreeRTOS 流缓冲区 + IDLE 中断 + DMA）
 * @version 0.3
 * @date 2026-09-10
 *
 * @todo 收上来的数据要交给上层分发（按设备 / 协议注册）
 *
 * @copyright Copyright (c) 2026
 *
 * @details 接收端用 IDLE 中断判断一帧收完，发送端由 TX Complete 中断接着发下一批，
 *          收发都走 DMA，不需要 CPU 逐字节搬运。
 *
 * @note 模板参数是缓冲区大小（字节）。DMA 收发缓冲区与发送流缓冲区用这个尺寸，单次 send() 不能超过它；
 *       接收流缓冲区是它的 2 倍（DMA 缓冲写满时一次投递 BUFFER_SIZE 字节，留出余量给紧接着的 IDLE 投递）。
 *
 * @note 用法：
 *
 *   // 1) 模板实例化在 bsp_uart.cpp
 *   template class BspUart<128>;
 *
 *   // 2) 全局实例写在用户的板级配置里（示例：examples/dm_mc02/app/board/bsp_cfg.cpp）：句柄 + 是否启用发送 + 波特率
 *   __attribute__((section(".dma_buffer"))) BspUart<128> bsp_uart1({&huart1, true, 115200});
 *   // 3) init() 在初始化阶段调用（system_init() 之后、system_start() 之前，调度器启动前即可）；
 *   //    init() 时登记进 BspUartBase 登记表，HAL 回调与库的串口巡检靠它找到实例
 *   configASSERT(bsp_uart1.init() == Status::OK);
 *
 *   // 4) 收发
 *   bsp_uart1.send(buffer, 8);           // 写进发送流缓冲区，DMA 自动发出去
 *   bsp_uart1.printf("val=%d\r\n", 42);  // 格式化输出（非阻塞，DMA 发送）
 *   bsp_uart1.receive(buffer, 8);        // 从接收流缓冲区读，读走的字节会从缓冲区移除
 *
 * @note 多个任务同时写同一个串口是安全的：send() 先用 _tx_write_lock 保证流缓冲区只有一个写者
 *       （FreeRTOS 流缓冲区要求单写单读），再用 _tx_lock 把「判忙 → 取包 → 启动 DMA」串起来；
 *       printf() 另有 _printf_lock 保护格式化缓冲。三把锁的取用顺序固定为 printf → write → tx，不会死锁。
 * @note 中断侧从不往流缓冲区写：DMA 启动失败的那一包留在 _tx_dma_buffer 里，下次启动时先发它，字节顺序不变。
 *
 * @warning 已知限制：TX DMA 开着 DMA FIFO（CubeMX 配置）时，DMA 会把字节预读进自己的 FIFO，剩余计数（NDTR）
 *          已经把它们算作“发出”；init() 还会打开 UART 硬件 FIFO（16 字节）。tx_recover() 中止卡死的发送后按 NDTR 续发，
 *          DMA FIFO 预读的字节与 UART TX FIFO 里的字节会丢。只影响「卡死后恢复」这条路径，正常发送不受影响。
 *
 * @note 所有超时参数单位都是毫秒；传 portMAX_DELAY 表示一直等。
 */

#ifndef __BSP_UART_HPP__
#define __BSP_UART_HPP__

#include "stm32h7xx_hal.h" // IWYU pragma: keep（HAL 模块开关来自 stm32h7xx_hal_conf.h）

// 宿主工程没在 CubeMX 里开 UART（HAL_UART_MODULE_ENABLED 未定义）时，本模块整体不参与编译
#ifdef HAL_UART_MODULE_ENABLED

#include "FreeRTOS.h" // IWYU pragma: keep
#include "semphr.h"   // IWYU pragma: keep (TX 启动锁)
#include "stream_buffer.h"
#include "task.h"  // IWYU pragma: keep

#include "hardware_config_defaults.hpp" // 可配置项（用户 hardware_config.h + 默认值）
#include "status.hpp"         // 统一状态码


/**
 * @brief 串口驱动（IDLE 中断 + DMA + FreeRTOS 流缓冲区）
 *
 * @tparam BUFFER_SIZE 收发缓冲区大小（字节），也是流缓冲区容量
 *
 * @note 接收：IDLE/TC 中断把收到的数据投进流缓冲区，然后立即重新启动接收，准备接下一帧。
 *       半传输（HT）事件也会进回调，但它不是帧边界，直接忽略。
 *       发送：数据先写进发送流缓冲区，再由任务或 TX Complete 中断交给 DMA。
 */
/**
 * @brief 串口实例的公共基类：实例登记表 + 中断 / 巡检入口
 *
 * @note 为什么要有它：BspUart 是模板，不同缓冲区大小是不同类型，HAL 回调与库的巡检任务
 *       没法用一个指针数组统一指向它们。基类不带模板参数，登记表存基类指针，
 *       入口函数做成虚函数，由 BspUart<N> 实现。
 * @note 登记时机：BspUart::init() 在打开接收中断之前登记；HAL 回调按 huart 句柄查表，
 *       查不到的串口（宿主工程自己管的、没有用本驱动的）直接忽略。
 *       登记只发生在初始化阶段（调度器启动前、或运行态之前），中断与任务只读这张表。
 */
class BspUartBase
{
public:
  /** @brief 析构时从登记表摘除 */
  virtual ~BspUartBase();

  /** @brief 收到一帧数据（IDLE/TC 中断）；见 BspUart::on_idle_isr() */
  virtual void on_idle_isr(uint16_t size, BaseType_t *pxHigherPriorityTaskWoken) = 0;

  /** @brief 上一批发完了，接着发下一批（TX Complete 中断）；见 BspUart::start_transmission_from_isr() */
  virtual void start_transmission_from_isr(BaseType_t *pxHigherPriorityTaskWoken) = 0;

  /** @brief UART 错误回调（ISR）；见 BspUart::on_error_isr() */
  virtual void on_error_isr() = 0;

  /** @brief TX 卡死巡检（任务上下文）；见 BspUart::tx_recover() */
  virtual bool tx_recover() = 0;

  /** @brief RX 停摆巡检（任务上下文）；见 BspUart::rx_recover() */
  virtual bool rx_recover() = 0;

  /** @brief 绑定的 UART 句柄 */
  UART_HandleTypeDef *handle() const
  {
    return _huart;
  }

  /** @brief 已登记的串口实例数 */
  static uint32_t instance_count();

  /** @brief 第 index 个已登记实例；越界返回 nullptr */
  static BspUartBase *instance_at(uint32_t index);

  /** @brief 按 HAL 句柄查已登记实例；查不到返回 nullptr（ISR 中可调用） */
  static BspUartBase *find(const UART_HandleTypeDef *huart);

  // 登记表里存的是对象地址，复制会让两个对象争同一个句柄
  BspUartBase(const BspUartBase &)            = delete;
  BspUartBase &operator=(const BspUartBase &) = delete;

protected:
  /** @brief 只保存句柄，不登记（登记在 init() 里做，构造函数只做赋值） */
  explicit BspUartBase(UART_HandleTypeDef *huart) : _huart(huart)
  {
  }

  /**
   * @brief 把本实例加入登记表（重复登记无副作用）
   * @return OK=已登记；BAD_ARG=句柄为空，或另一个实例已经登记了同一个句柄；FULL=登记表已满
   */
  Status register_self();

  UART_HandleTypeDef *_huart; ///< UART 句柄

private:
  /** @brief 登记表容量：UART_MAX_INSTANCES（默认 12：STM32H7 最多 11 路 U(S)ART 含 LPUART1，留 1 个余量） */
  static constexpr uint32_t MAX_INSTANCES = UART_MAX_INSTANCES;

  static BspUartBase *_registry[MAX_INSTANCES]; ///< 已登记实例
  static uint32_t     _registry_num;            ///< 已登记数量
};


template <size_t BUFFER_SIZE = 256>
class BspUart : public BspUartBase
{
public:
  // ---------------- 公有接口 ----------------

  /** @brief 串口配置（可匿名按序传入：{huart, transmit_enable, baudrate}） */
  struct Config
  {
    /** @brief 按序构造（参数顺序 = 字段顺序） */
    Config(UART_HandleTypeDef *huart = nullptr, bool transmit_enable = true, uint32_t baudrate = 115200U) : huart(huart), transmit_enable(transmit_enable), baudrate(baudrate)
    {
    }

    UART_HandleTypeDef *huart;           ///< UART 句柄
    bool                transmit_enable; ///< 是否启用发送
    uint32_t            baudrate;        ///< 波特率 (bit/s)，TX 卡死判定要用它算一包发完要多久
  };

  /**
   * @brief RX 的健康状态
   *
   * @note 只信 HAL 的 RxState 是不够的：它可能还停在 BUSY_RX，而 DMA 早就停了。
   *       所以这里同时看三处 —— HAL 的 RxState、UART 的 CR3.DMAR、DMA 的 CR.EN。
   */
  enum class RxHealth : uint8_t
  {
    OK = 0,       ///< 三处一致：在收
    STOPPED,      ///< 三处一致：停了（没启动过或主动停的，不算故障）
    INCONSISTENT, ///< 三处不一致：RX 有问题，需要重启
  };

  /** @brief RX 诊断计数：只增不减，一切正常时应全为 0 */
  struct RxDiag
  {
    uint32_t arm_fail_cnt; ///< 启动接收失败的次数（包括 init 时的第一次和运行期重启）
    uint32_t drop_bytes;   ///< 接收流缓冲区满，被丢掉的字节数
    uint32_t error_cnt;    ///< rx_recover() 真正重建 RX 的次数
    uint32_t dma_repair_cnt; ///< rx_recover() 发现 RX DMA 句柄卡在非 READY（如 ERROR）并复位的次数
  };

  /** @brief TX 诊断计数：只增不减，一切正常时应全为 0 */
  struct TxDiag
  {
    uint32_t drop_bytes;        ///< 入队失败（缓冲区满或超时）丢掉的字节数
    uint32_t start_fail_cnt;    ///< HAL_UART_Transmit_DMA 启动失败的次数
    uint32_t isr_skip_cnt;      ///< 中断里发现通道不空闲，跳过续传的次数
    uint32_t stall_recover_cnt; ///< 巡检发现卡死并恢复成功的次数
    uint32_t error_cnt;         ///< TX 触发错误回调的次数（最终由 tx_recover() 恢复）
    uint32_t dma_repair_cnt;    ///< tx_recover() 发现 TX DMA 句柄卡在非 READY 并复位的次数
  };

  /** @brief 构造函数（只做赋值，FreeRTOS 资源创建推迟到 init()） */
  BspUart(const Config &cfg);

  /**
   * @brief 创建流缓冲区和两把锁，然后启动 IDLE 接收
   *
   * @note 在初始化阶段调用（system_init() 之后、system_start() 之前；调度器启动前也可以创建 FreeRTOS 对象）。
   *       可以重复调用：已初始化的实例先停掉收发、释放上一次的资源再重建；
   *       此时不能有任务阻塞在本实例的 send() / receive() 上（流缓冲区会被删除重建）。
   *
   * @return OK=成功；IO_ERROR=资源创建失败或接收没能启动；
   *         BAD_ARG=句柄为空、已被另一个实例登记、没配 RX DMA（或启用发送却没配 TX DMA），
   *         或实例不在 DMA 可达的内存里（没放进 .dma_buffer）；
   *         FULL=串口登记表已满
   * @note 在打开接收中断之前把本实例登记进 BspUartBase 的登记表，HAL 回调靠它找到本实例。
   */
  Status init();

  /** @brief 析构：停掉 RX/TX DMA 并释放所有软件资源 */
  ~BspUart() override;

  /**
   * @brief 把数据写进发送流缓冲区，并启动 DMA 发出去
   *
   * @param data       要发的数据
   * @param size       字节数；超过流缓冲区容量会直接返回 BAD_ARG，需要调用方自己分包
   * @param timeout_ms 入队最多等多久（毫秒）；portMAX_DELAY 表示一直等到全部写进去
   * @param written    返回实际写进去的字节数，不需要可以传 nullptr
   * @return OK=全部入队；TIMEOUT=只进去一部分；BAD_ARG=参数非法；IO_ERROR=没启用发送
   *
   * @note 返回 OK 只说明数据进了缓冲区，此时 DMA 可能才刚开始发。
   */
  Status send(const uint8_t *data, size_t size, uint32_t timeout_ms = portMAX_DELAY, size_t *written = nullptr);

  /**
   * @brief 格式化输出，用法和 printf 一样（只能在任务里调用，超长会截断）
   *
   * @note 格式化用的缓冲区是这个实例共用的，所以加了 _printf_lock，
   *       多个任务同时调用也不会互相覆盖。
   * @note 入队等待固定 10 ms，不像 send() 那样给 portMAX_DELAY：万一发送卡住，
   *       不至于把调用任务一直阻塞住。没全部写进去就返回 TIMEOUT，已经进去的部分照常发。
   */
  Status printf(const char *fmt, ...);

  /**
   * @brief 从接收流缓冲区读数据
   *
   * @note 读到的不保证是完整的一帧 —— 驱动只负责把字节搬过来，怎么切帧是协议层的事
   *       （长度前缀 / 定界符 / 校验和都可以）。除非流缓冲区满，否则不会丢字节。
   *
   * @param buffer   收数据的地方
   * @param size     最多读多少字节
   * @param timeout  最多等多久（毫秒）；portMAX_DELAY 表示一直等
   * @param received 返回实际读到的字节数，不需要可以传 nullptr
   * @return OK=读到了；TIMEOUT=没有数据；BAD_ARG=参数非法；IO_ERROR=接收缓冲区没创建
   */
  Status receive(uint8_t *buffer, size_t size, uint32_t timeout = portMAX_DELAY, size_t *received = nullptr);

  /**
   * @brief TX 卡死后的恢复：缓冲区里还有数据，却长时间没有推进，就复位通道重发
   *
   * @note 多久算"长时间"：取一包发完所需时间的 2 倍，按 Config::baudrate 算出来。
   *       正常发送时每发一包都会刷新 _tx_last_activity_tick，所以不会把正常发送误判成卡死。
   * @note 重发前先用 DMA 剩余计数算出在途数据已经发出去多少，只补后面的部分，
   *       既不重复也不丢；在途数据如果已经发完，就直接从流缓冲区取新数据。
   * @note 取锁不等待，可以放进周期任务里调用（比如 sys_task 的 10 ms 巡检）。
   *
   * @return true=这次确实恢复成功了一次
   */
  bool tx_recover() override;

  /**
   * @brief RX 异常时的恢复：复位接收通道，再重新启动
   *
   * @note 触发条件：本该在收（_rx_should_run 为 true），但三处状态不一致。
   *       一切正常时本函数只读一次状态就返回，不会打断正在收的数据。
   * @note 必须放在任务里：复位接收要调阻塞版的 HAL_UART_AbortReceive()，中断里不能用。
   *       所以中断里出错只累加计数，真正的修复由本函数在 10 ms 内完成。
   * @note 「中止 → 重新启动」期间关掉本串口的 NVIC 中断，避免 IDLE 中断在中间重新启动接收；
   *       RX DMA 句柄若卡在 HAL_DMA_STATE_ERROR（HAL 自己出不来），流停稳后在这里复位，计入 dma_repair_cnt。
   *
   * @return true=这次确实重建了一次（累计次数见 rx_diag().error_cnt）
   */
  bool rx_recover() override;

  // ----------------

  // ---------------- ISR 入口 ----------------

  /**
   * @brief 收到一帧数据（IDLE/TC 中断触发）
   *
   * @param size 这一帧的字节数
   * @param pxHigherPriorityTaskWoken 中断里需要唤醒的高优先级任务
   */
  void on_idle_isr(uint16_t size, BaseType_t *pxHigherPriorityTaskWoken) override;

  /**
   * @brief 上一批发完了，接着发缓冲区里的下一批（TX Complete 中断触发）
   *
   * @param pxHigherPriorityTaskWoken 需初始化为 pdFALSE，唤醒了高优先级任务则置 pdTRUE
   */
  void start_transmission_from_isr(BaseType_t *pxHigherPriorityTaskWoken) override;

  /**
   * @brief UART 错误回调（ISR）
   *
   * @note 这里只累加计数，不做恢复：HAL 的 Abort 接口内部是阻塞版 HAL_DMA_Abort，
   *       中断里不能用。RX 交给任务侧的 rx_recover()，TX 交给 tx_recover()。
   */
  void on_error_isr() override;

  // ----------------
  // ---------------- 查询接口 ----------------

  /** @brief 发送流缓冲区还剩多少空间（字节） */
  size_t get_tx_free_space();

  /** @brief 接收流缓冲区现在有多少可读字节 */
  size_t get_rx_available_data();

  /**
   * @brief 查 RX 是不是真的在收（核对 RxHealth 里说的那三处状态）
   *
   * @note 本该在收的时候返回非 OK，说明出了故障；本来就没在收（主动停的）不算。
   */
  RxHealth rx_health() const;

  /** @brief 读 RX 诊断计数 */
  RxDiag rx_diag() const;

  /** @brief 读 TX 诊断计数 */
  TxDiag tx_diag() const;

  // ----------------

private:
  // ---------------- 私有实现 ----------------

  // 成员变量

  static constexpr size_t RX_STREAM_SIZE = 2U * BUFFER_SIZE; ///< 接收流缓冲区容量（见 init()）

  StreamBufferHandle_t _rx_stream_buffer = nullptr; ///< 接收流缓冲区
  StreamBufferHandle_t _tx_stream_buffer = nullptr; ///< 发送流缓冲区
  SemaphoreHandle_t    _tx_write_lock    = nullptr; ///< 发送入队锁：保证发送流缓冲区只有一个写者
  SemaphoreHandle_t    _tx_lock          = nullptr; ///< 发送启动锁：保证「判忙 → 取包 → 启动 DMA」互斥
  SemaphoreHandle_t    _printf_lock      = nullptr; ///< printf 锁：保护实例共用的格式化缓冲区

  bool     _rx_should_run = true;       ///< 是否希望 RX 在收；某次失败不会把它永久置成 false
  bool     _transmit_enable;            ///< 是否启用发送
  uint32_t _baudrate;                   ///< 波特率 (bit/s)，TX 卡死判定要用它算阈值
  uint8_t  _rx_dma_buffer[BUFFER_SIZE]; ///< DMA 接收缓冲区
  uint8_t  _tx_dma_buffer[BUFFER_SIZE]; ///< DMA 发送缓冲区
  char     _printf_buffer[BUFFER_SIZE]; ///< printf 格式化用的缓冲区

  // RX 诊断量：中断里累加，任务里读取
  volatile uint32_t _rx_arm_fail_cnt = 0; ///< 启动接收失败的次数
  volatile uint32_t _rx_drop_bytes   = 0; ///< 流缓冲区满而丢掉的字节数
  volatile uint32_t _rx_error_cnt    = 0; ///< 重建 RX 的次数
  volatile uint32_t _rx_dma_repair_cnt = 0; ///< 复位卡死的 RX DMA 句柄的次数

  // TX 诊断量：中断里累加，任务里读取
  volatile uint32_t _tx_drop_bytes     = 0; ///< 入队失败而丢掉的字节数
  volatile uint32_t _tx_start_fail_cnt = 0; ///< 启动 DMA 失败的次数（任务和中断合计）
  volatile uint32_t _tx_isr_skip_cnt   = 0; ///< ISR 因通道不空闲跳过续传的次数
  volatile uint32_t _tx_stall_rec_cnt  = 0; ///< 巡检发现卡死并恢复成功的次数
  volatile uint32_t _tx_error_cnt      = 0; ///< TX 进错误回调的次数
  volatile uint32_t _tx_dma_repair_cnt = 0; ///< 复位卡死的 TX DMA 句柄的次数

  volatile TickType_t _tx_last_activity_tick = 0; ///< 最近一次发送有推进（启动 / DMA 计数变化）的时刻，tx_recover() 用它判断卡死
  uint32_t            _tx_last_ndtr          = 0; ///< tx_recover() 上次看到的 TX DMA 剩余计数（只有 tx_recover() 读写）

  // _tx_dma_buffer 里还没确认发完的那一段：[_tx_chunk_off, _tx_chunk_off + _tx_chunk_len)。
  // 只在通道空闲时由任务（持 _tx_lock）或 TX 完成中断改写，两者不会同时发生。
  volatile size_t _tx_chunk_off    = 0;     ///< 这一段在 _tx_dma_buffer 里的起点
  volatile size_t _tx_chunk_len    = 0;     ///< 这一段的长度；0 = 没有未发完的数据
  volatile bool   _tx_chunk_active = false; ///< true = 这一段已交给 DMA 在传；false = 启动失败、等待重发

  bool _tx_idle_pending_seen = false; ///< 上一次 tx_recover() 是否已看到「通道空闲但有数据」（只有 tx_recover() 读写）

  // 内部实现

  /**
   * @brief 启动 DMA 接收（只管 RX；调用前 RxState 必须是 READY）
   *
   * @note 启动前先清掉残留的 UART 错误标志（ORE/FE/NE/PE），
   *       否则 HAL 会因为存在挂起的错误而拒绝启动。
   *
   * @return true=成功；false=HAL 拒绝（BUSY/ERROR），已计入 arm_fail_cnt
   */
  bool arm_reception();

  /** @brief 中止 RX 通道并复位接收状态（只动 RX，不碰 TX DMA） */
  void abort_reception();

  /** @brief 释放已创建的 FreeRTOS 资源（两个流缓冲区、三把锁） */
  void cleanup_resources();

  /** @brief 从发送流缓冲区取一包交给 DMA；wait=0 表示取不到锁就立刻返回 */
  bool start_transmission(TickType_t wait = portMAX_DELAY);

  /**
   * @brief 启动一次 DMA 发送：先重发启动失败留下的那一段，没有才从流缓冲区取新数据
   *
   * @param from_isr true = 在 TX 完成中断里调用（用 FromISR 版 API）
   * @param pxHigherPriorityTaskWoken from_isr 时透传给 FromISR API，否则可为 nullptr
   * @return true=本次确实启动了 DMA
   *
   * @note 调用前提：通道空闲（gState == READY），且任务侧已持有 _tx_lock。
   */
  bool start_chunk(bool from_isr, BaseType_t *pxHigherPriorityTaskWoken);

  /** @brief 结算已停止的那一段：按 DMA 剩余计数把起点后移、长度减去已发出的字节（DMA 必须已停） */
  void settle_chunk();

  // ----------------
};


#endif // HAL_UART_MODULE_ENABLED

#endif // __BSP_UART_HPP__

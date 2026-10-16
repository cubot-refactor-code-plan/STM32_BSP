#ifndef CAN_TX_NODE_HPP
#define CAN_TX_NODE_HPP

#include "stm32h7xx_hal.h" // IWYU pragma: keep（HAL 模块开关来自 stm32h7xx_hal_conf.h）

// 宿主工程没在 CubeMX 里开 FDCAN（HAL_FDCAN_MODULE_ENABLED 未定义）时，本模块整体不参与编译
#ifdef HAL_FDCAN_MODULE_ENABLED

#include "bsp_can.hpp"

/**
 * @brief 等待系统初始化完成后，按 1 ms 周期扫描 CAN 发送节点。
 * @param argument 未使用的任务参数。
 * @note 节点须在系统进入运行状态前注册；无节点时任务自行退出。
 */
extern "C" void cubot_can_tx_task(void *argument);

class CanTxNode
{
private:
  CanTxMsg      txBuffer; ///< 消息本身
  uint8_t       division; ///< 同一 CAN 帧划分的等长数据槽位数
  Status        _statu;
  uint8_t       bufferRegister; ///< 消息被注册占用情况
  uint8_t       bufferUnsend;
  osMutexId_t   bufferMutex;
  osMutexAttr_t bufferMutex_Attr;
  char          mutexName[40];
  bool          _registered; ///< 是否已加入发送节点链表

  static uint8_t    node_num;
  static bool      frozen;
  CanTxNode        *next;
  TickType_t        last_send_tick; ///< 发送任务启动时初始化，成功发送后更新

  /**
   * @brief 控制指令有效期 (ms)：某个槽位超过这么久没被 filldata() 刷新，发送任务就把它清零
   *
   * @note cubot_can_tx_task 每 CUBOT_CAN_TX_HEARTBEAT_MS（默认 5 ms）保底重发一次整帧；没有有效期的话，
   *       控制任务卡死 / 被删后，电机会一直按最后一次写入的电流转。默认 100 ms ≈ 20 次保底心跳，
   *       能容忍控制任务偶发卡顿，又能在 0.1 s 内让电机停下（2026-10-16 开发者定）；
   *       在 cubot_config.h 里用 CUBOT_CAN_CMD_EXPIRE_MS 修改。
   * @note 只清超时的那个槽位，同一帧里其他电机不受影响。
   */
  static constexpr uint32_t CMD_EXPIRE_MS = CUBOT_CAN_CMD_EXPIRE_MS;

  TickType_t bufferFillTick[8]; ///< 各槽位最近一次 filldata() 的 tick（节点锁内读写）
  uint8_t    bufferExpired;     ///< 已因超时清零的槽位（置位后不再重复清零；filldata() 清除对应位）
  uint32_t   expiredCount;      ///< 槽位因超时被清零的累计次数（诊断用）

  CanTxNode(uint32_t can_id, uint8_t division = 1);
  ~CanTxNode();
  /**
   * @brief 创建互斥量并将节点注册到所属 CAN 的发送链表
   * @return Status::OK 初始化成功；Status::BAD_ARG 配置非法；
   *         Status::BUSY 已经初始化；Status::FULL 内存或 RTOS 资源不足。
   * @note 必须在 osKernelInitialize() 之后且非 ISR 上下文调用。
   */
  Status init(BspCan &can);

public:
  Status filldata(uint8_t data[], uint8_t slot = 0);

  friend CanTxNode *regist(BspCan &can_item, uint32_t can_id);
  friend CanTxNode *regist(BspCan &can_item, uint32_t can_id, uint8_t division, uint8_t slot);
  friend Status     unregist(CanTxNode *node, uint8_t slot);

  CanTxNode(const CanTxNode &)            = delete;
  CanTxNode &operator=(const CanTxNode &) = delete;
  CanTxNode(CanTxNode &&)                 = delete;
  CanTxNode &operator=(CanTxNode &&)      = delete;

  friend void cubot_can_tx_task(void *argument);
};

CanTxNode *regist(BspCan &can_item, uint32_t can_id);
CanTxNode *regist(BspCan &can_item, uint32_t can_id, uint8_t division, uint8_t slot);

/**
 * @brief 清零并释放指定发送槽位，保留节点供发送任务继续扫描。
 * @param node 注册时返回的节点。
 * @param slot 待释放的槽位。
 * @return OK 表示释放成功；其他状态表示参数、注册状态或锁操作错误。
 * @note 仅在调度器启动前或正常任务上下文调用；运行时等待节点锁。
 *       清零帧由发送任务后续发送，不保证返回时已发送到总线。
 */
Status unregist(CanTxNode *node, uint8_t slot);

#endif // HAL_FDCAN_MODULE_ENABLED

#endif // CAN_TX_NODE_HPP

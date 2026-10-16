/**
 * @file fault_record.hpp
 * @author ChoseB
 * @brief 故障现场记录 —— 断言失败、栈溢出、内存分配失败时留下现场再停机
 * @version 0.1
 * @date 2026-10-14
 *
 * @copyright Copyright (c) 2026
 *
 * @details 三个入口（C 链接，内核与 freertos.c 也能调）：
 *            - fault_on_assert()         —— configASSERT 失败（定义见 FreeRTOSConfig.h 的 USER CODE 1）
 *            - fault_on_stack_overflow() —— vApplicationStackOverflowHook（freertos.c 的 USER CODE 4）
 *            - fault_on_malloc_failed()  —— vApplicationMallocFailedHook（freertos.c 的 USER CODE 5）
 *          都先关中断、把现场写进 fault_record，再停机：接着调试器时执行 BKPT 直接停在现场，
 *          没接调试器就死循环（与原来的 configASSERT 行为一致）。
 *
 * @note fault_record 放在 .noinit 段（链接脚本里 RAM_D1 上的 NOLOAD 段）：启动代码不清零，
 *       看门狗 / 复位键 / 调试器复位之后仍能读到上一次的现场；掉电后内容随机，靠 magic 判断是否有效。
 *       用调试器查看：`print fault_record`。
 *
 * @note 记录本身不调用任何可能再次断言的 RTOS 接口；重入（记录过程中又触发故障）直接停机。
 *
 * @note RAM_D1（AXI SRAM）带 ECC：不足 64 位的写入先留在控制器的缓冲里，等写到另一个 64 位字时才落盘，
 *       缓冲里的那一笔遇到复位就丢（实测：最后写的行号 / 任务名复位后变回旧值）。所以停机前要往
 *       单独的 flush 字再写一次，把前面的写入挤出去。
 */

#ifndef __SERVICE_FAULT_RECORD_HPP__
#define __SERVICE_FAULT_RECORD_HPP__

#include <stdint.h>


/** @brief 故障种类 */
enum class FaultKind : uint32_t
{
  NONE           = 0, ///< 没有记录
  ASSERT         = 1, ///< configASSERT 失败
  STACK_OVERFLOW = 2, ///< 任务栈溢出
  MALLOC_FAILED  = 3, ///< pvPortMalloc 失败
};

/** @brief 故障现场（布局固定，调试器直接读；按 8 字节对齐，见文件头 ECC 说明） */
struct alignas(8) FaultRecord
{
  static constexpr uint32_t MAGIC = 0x46415554U; ///< "FAUT"：magic 等于它才说明内容有效

  uint32_t  magic;    ///< 等于 MAGIC 才有效（掉电后是随机值）
  FaultKind kind;     ///< 最近一次故障的种类
  uint32_t  count;    ///< 上电以来累计故障次数（复位不清零）
  uint32_t  line;     ///< 断言所在行号（其他种类为 0）
  uint32_t  tick;     ///< 故障时的系统 tick（调度器未启动时为 0）
  uint32_t  ipsr;     ///< 故障时的 IPSR：0 = 任务上下文，非 0 = 异常号（在中断里断言）
  char      file[48]; ///< 断言所在源文件名（只留文件名，不含目录）
  char      task[16]; ///< 溢出的任务名 / 故障时正在运行的任务名（调度器未启动时为空）
  uint32_t  flush[2]; ///< 独占一个 64 位字：停机前写它一次，把前面还在 ECC 缓冲里的写入挤到 SRAM（内容无意义）
};

/** @brief 最近一次故障现场（.noinit，复位后保留） */
extern FaultRecord fault_record;

#ifdef __cplusplus
extern "C"
{
#endif

  /** @brief configASSERT 失败：记下文件名与行号后停机（不返回） */
  void fault_on_assert(const char *file, int line);

  /** @brief 栈溢出钩子：记下溢出的任务名后停机（不返回） */
  void fault_on_stack_overflow(const char *task_name);

  /** @brief 内存分配失败钩子：记下当前任务名后停机（不返回） */
  void fault_on_malloc_failed(void);

#ifdef __cplusplus
}
#endif

#endif // __SERVICE_FAULT_RECORD_HPP__

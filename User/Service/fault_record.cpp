#include "fault_record.hpp"

#include "FreeRTOS.h" // IWYU pragma: keep (taskDISABLE_INTERRUPTS / configMAX_TASK_NAME_LEN)
#include "main.h"     // IWYU pragma: keep (CoreDebug / __BKPT / __get_IPSR)
#include "task.h"

#include <stddef.h> // offsetof


// ---------------- 全局实例 ----------------


/** @brief 放在 .noinit：启动代码不清零，复位后还能读到上一次的现场 */
__attribute__((section(".noinit"))) FaultRecord fault_record;

static_assert(sizeof(fault_record.task) >= configMAX_TASK_NAME_LEN, "task 字段放不下任务名");
static_assert(offsetof(FaultRecord, flush) % 8U == 0U, "flush 必须独占一个 64 位 ECC 字");


// ----------------
// ---------------- 私有实现 ----------------

namespace
{
/** @brief 正在记录现场：记录途中再出故障就不再记录，直接停机 */
volatile bool in_fault = false;

/** @brief 有界拷贝字符串并补 0（不依赖 libc，避免在故障路径里再出问题） */
void copy_text(char *dst, uint32_t cap, const char *src)
{
  uint32_t i = 0U;
  if (src != nullptr)
  {
    for (; (i + 1U) < cap && src[i] != '\0'; ++i)
    {
      dst[i] = src[i];
    }
  }
  dst[i] = '\0';
}

/** @brief 取路径里最后一个 '/' 或 '\\' 之后的文件名 */
const char *base_name(const char *path)
{
  if (path == nullptr)
  {
    return nullptr;
  }
  const char *name = path;
  for (const char *p = path; *p != '\0'; ++p)
  {
    if (*p == '/' || *p == '\\')
    {
      name = p + 1;
    }
  }
  return name;
}

/**
 * @brief 关中断并写入公共字段；返回 false 表示是重入，调用方不要再写
 *
 * @note 调度器已启动才去读 tick 和当前任务名：未启动时当前任务控制块可能为空，
 *       pcTaskGetName() 里的断言会再次进入故障路径。
 */
bool begin_record(FaultKind kind)
{
  taskDISABLE_INTERRUPTS();
  if (in_fault)
  {
    return false;
  }
  in_fault = true;

  const bool valid  = (fault_record.magic == FaultRecord::MAGIC);
  fault_record.count = valid ? (fault_record.count + 1U) : 1U;
  fault_record.magic = FaultRecord::MAGIC;
  fault_record.kind  = kind;
  fault_record.line  = 0U;
  fault_record.ipsr  = __get_IPSR();
  fault_record.file[0] = '\0';
  fault_record.task[0] = '\0';
  fault_record.tick    = 0U;

  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
  {
    fault_record.tick = xTaskGetTickCount(); // 关中断下直接读计数，不会断言
    if (fault_record.ipsr == 0U || xTaskGetCurrentTaskHandle() != nullptr)
    {
      copy_text(fault_record.task, sizeof(fault_record.task), pcTaskGetName(xTaskGetCurrentTaskHandle()));
    }
  }
  return true;
}

/** @brief 停机：先把记录挤出 ECC 写缓冲，再按有无调试器 BKPT 或死循环 */
[[noreturn]] void halt(void)
{
  // 写另一个 64 位字，把最后一笔不足 64 位的写入从 AXI SRAM 的 ECC 缓冲里挤出去（否则复位会丢）
  fault_record.flush[0] = 0U;
  __DSB();

  if ((CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) != 0U)
  {
    __BKPT(0); // 没接调试器时执行 BKPT 会升级成 HardFault，所以先判 C_DEBUGEN
  }
  for (;;)
  {
  }
}
} // namespace


// ----------------
// ---------------- 函数定义 ----------------


extern "C" void fault_on_assert(const char *file, int line)
{
  if (begin_record(FaultKind::ASSERT))
  {
    copy_text(fault_record.file, sizeof(fault_record.file), base_name(file));
    fault_record.line = static_cast<uint32_t>(line);
  }
  halt();
}

extern "C" void fault_on_stack_overflow(const char *task_name)
{
  if (begin_record(FaultKind::STACK_OVERFLOW))
  {
    copy_text(fault_record.task, sizeof(fault_record.task), task_name); // 以钩子给的任务名为准
  }
  halt();
}

extern "C" void fault_on_malloc_failed(void)
{
  (void)begin_record(FaultKind::MALLOC_FAILED);
  halt();
}


// ----------------

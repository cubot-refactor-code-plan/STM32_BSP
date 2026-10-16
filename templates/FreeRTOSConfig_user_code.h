/*
 * 片段（不是独立头文件）：复制到 CubeMX 生成的 Core/Inc/FreeRTOSConfig.h 的
 * USER CODE BEGIN 1 / USER CODE END 1 之间。
 *
 * 作用：configASSERT 失败时先把文件名 / 行号 / tick / 任务名记进 fault_record（.noinit，复位后保留）
 * 再停机；栈溢出与分配失败钩子同样记录（钩子见 freertos_user_code.c）。实现在库的
 * service/fault_record.cpp。内核源码所在的目标看不到库的头文件，所以原型直接写在这里。
 *
 * 同时在 CubeMX 的 FreeRTOS 配置里打开：CHECK_FOR_STACK_OVERFLOW = Option2、USE_MALLOC_FAILED_HOOK = Enabled。
 */

/* USER CODE BEGIN 1 */
#if defined(__ICCARM__) || defined(__CC_ARM) || defined(__GNUC__)
#ifdef __cplusplus
extern "C" {
#endif
void fault_on_assert(const char *file, int line);
void fault_on_stack_overflow(const char *task_name);
void fault_on_malloc_failed(void);
#ifdef __cplusplus
}
#endif
#endif
#define configASSERT( x ) if ((x) == 0) { fault_on_assert(__FILE__, __LINE__); }
/* USER CODE END 1 */

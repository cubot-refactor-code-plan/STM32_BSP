/*
 * 片段（不要编译这个文件）：分别复制到 CubeMX 生成的 Core/Src/freertos.c 对应的 USER CODE 段。
 * 钩子函数的原型 CubeMX 已经生成好（需在 CubeMX 里打开栈溢出检查与分配失败钩子）。
 */

/* USER CODE BEGIN Includes */
#include "app_main.h"
/* USER CODE END Includes */

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, char *pcTaskName)
{
  (void)xTask;
  fault_on_stack_overflow(pcTaskName); /* 记下溢出的任务名后停机，不返回 */
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
void vApplicationMallocFailedHook(void)
{
  fault_on_malloc_failed(); /* 记下当前任务名后停机，不返回 */
}
/* USER CODE END 5 */

/* MX_FREERTOS_Init() 里： */
  /* USER CODE BEGIN Init */
  app_init(); /* 库入口 → 板级初始化 → 业务任务 → 放行，见 app_main.cpp */
  /* USER CODE END Init */

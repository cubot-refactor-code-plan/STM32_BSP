/**
 * @file app_main.h
 * @brief C 入口（模板）：freertos.c 的 MX_FREERTOS_Init() 只调这一个函数
 */

#ifndef __APP_MAIN_H__
#define __APP_MAIN_H__

#ifdef __cplusplus
extern "C"
{
#endif

  /** @brief 应用初始化：库入口 → 板级初始化 → 业务任务 → 放行（在调度器启动前调用） */
  void app_init(void);

#ifdef __cplusplus
}
#endif

#endif // __APP_MAIN_H__

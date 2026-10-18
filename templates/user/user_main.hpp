/**
 * @file user_main.hpp
 * @brief 业务任务初始化（模板）：app_init() 第 3 步只调 user_init()，各业务的任务都在它里面创建
 *
 * @details 本目录（宿主工程根下的 user/）放业务代码，CMake 自动收集这里的 .c / .cpp，新增业务文件不用改 CMake。
 *          加一个业务：
 *            1. 新建 user/xxx.hpp / xxx.cpp，头文件声明任务入口 void xxx_task(void *argument)；
 *            2. 任务入口第一句 SysFlagWaitRunning()，再进循环；
 *            3. 在 user_main.cpp 的 user_init() 里包含 xxx.hpp，加一行 configASSERT(xTaskCreate(...) == pdPASS)。
 */

#ifndef __USER_MAIN_HPP__
#define __USER_MAIN_HPP__

/**
 * @brief 创建全部业务任务（app_init() 里 board_init() 之后、system_start() 之前调用）
 * @note 只创建任务、不等外设：调度器还没启动，任务在 system_start() 置运行位后才开始跑。
 */
void user_init(void);

#endif // __USER_MAIN_HPP__

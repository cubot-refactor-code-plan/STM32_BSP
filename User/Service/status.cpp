#include "status.hpp"

osEventFlagsId_t sysEvent = NULL;
const osEventFlagsAttr_t sysEvent_Attr = {
    .name = "SysEvent"
};

Status SysFlagInit(void)
{
    sysEvent = osEventFlagsNew(&sysEvent_Attr);
    if (sysEvent == NULL)
    {
        return Status::FULL;
    }
    osEventFlagsSet(sysEvent, (1U << (uint8_t)Status::OK));
    return Status::OK;
}

Status SysFlagSet(Status statu)
{
    if (sysEvent == NULL)
    {
        return Status::NOT_INIT;
    }
    if (statu == Status::OK)
    {
        return Status::BAD_ARG;
    }
    osEventFlagsSet(sysEvent, (1U << (uint8_t)statu));
    osEventFlagsClear(sysEvent, (1U << (uint8_t)Status::OK));
    return Status::OK;
}

/**
 * @brief 初始化全部完成：置运行位
 * @return OK=已置运行位；NOT_SUPPORTED=此前有初始化失败记录（SysInitError），运行位不置；NOT_INIT=事件组未创建
 * @note 调用方（all_init）用 configASSERT 检查返回值：有失败记录就停机并留下现场，而不是带着半初始化的系统继续跑。
 */
Status SysCompleteInit(void)
{
    if (sysEvent == NULL)
    {
        return Status::NOT_INIT;
    }
    if ((osEventFlagsGet(sysEvent) & SYS_FLAG_INIT_FAIL_BIT) != 0U)
    {
        return Status::NOT_SUPPORTED;
    }
    osEventFlagsSet(sysEvent, SYS_FLAG_RUNNING_BIT);
    return Status::OK;
}

void SysInitError(void)
{
    osEventFlagsSet(sysEvent, SYS_FLAG_INIT_FAIL_BIT);
    osEventFlagsClear(sysEvent, SYS_FLAG_RUNNING_BIT);
}

Status SysInitError(Status statu)
{
    SysInitError();
    return SysFlagSet(statu);
}

uint32_t SysFlagWait(Status statu, uint32_t timeout)
{
    return osEventFlagsWait(sysEvent, (1U << (uint8_t)statu), osFlagsWaitAll | osFlagsNoClear, timeout);
}

void SysFlagWaitRunning(void)
{
    if (sysEvent == NULL)   // 连事件本身都挂了那就别运行了，当然这个可能性还是太低了
    {
        osThreadExit();
    }

    // 同时等运行位与初始化失败位：只等运行位的话，初始化失败时它永不置位，任务会永久阻塞
    const uint32_t flags = osEventFlagsWait(sysEvent,
                                            SYS_FLAG_RUNNING_BIT | SYS_FLAG_INIT_FAIL_BIT,
                                            osFlagsWaitAny | osFlagsNoClear,
                                            osWaitForever);
    if (((flags & osFlagsError) != 0U) || ((flags & SYS_FLAG_RUNNING_BIT) == 0U))
    {
        osThreadExit(); // 等到的是初始化失败：任务自行退出
    }
}

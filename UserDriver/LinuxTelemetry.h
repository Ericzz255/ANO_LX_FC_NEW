#ifndef __LINUX_TELEMETRY_H
#define __LINUX_TELEMETRY_H

#include "SysConfig.h"

/**
 * @brief 从飞控USART2 TX向Linux地面站发送一帧飞行状态数据。
 * @note 由20Hz调度任务调用；串口参数为115200-8-N-1。
 * @note 本函数只发送数据，不要求Linux板卡向飞控回传数据。
 */
void LinuxTelemetry_Send(void);

#endif

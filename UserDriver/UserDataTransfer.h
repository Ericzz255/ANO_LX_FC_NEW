#ifndef __USER_DATA_TRANSFER_H
#define __USER_DATA_TRANSFER_H

#include "SysConfig.h"

/*
 * 匿名上位机V7的F1诊断数据：
 *  1 int16 当前SLAM X，机头前方为正，单位cm
 *  2 int16 当前SLAM Y，机体左侧为正，单位cm
 */

/**
 * @brief 组帧并通过UART5发送F1、F2用户自定义数据。
 * @note  由20Hz调度任务调用。
 */
void UserDataTransfer_Task(void);

#endif

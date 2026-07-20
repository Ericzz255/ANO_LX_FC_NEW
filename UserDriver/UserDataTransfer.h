#ifndef __USER_DATA_TRANSFER_H
#define __USER_DATA_TRANSFER_H

#include "SysConfig.h"

/*
 * 匿名上位机V7的F1诊断数据：
 *  1 int16 当前SLAM X，机头前方为正，单位cm
 *  2 int16 当前SLAM Y，机体左侧为正，单位cm
 *  3 int16 飞控估计X水平速度，单位cm/s
 *  4 int16 飞控估计Y水平速度，单位cm/s
 *  5 int16 SLAM位置有效标志
 *  6 int16 光流图像质量
 *  7 int16 MODE1光流速度有效标志
 *  8 int16 禁飞区1，编码为A*10+B（例如A9B1显示91）
 *  9 int16 禁飞区2，编码为A*10+B
 * 10 int16 禁飞区3，编码为A*10+B
 * 未收到完整有效的三个禁飞区时，USERDATA8~10均为0。
 */

/**
 * @brief 组帧并通过UART5发送F1用户自定义数据。
 * @note  由20Hz调度任务调用。
 */
void UserDataTransfer_Task(void);

#endif

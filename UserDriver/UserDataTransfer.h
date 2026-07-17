#ifndef __USER_DATA_TRANSFER_H
#define __USER_DATA_TRANSFER_H

#include "SysConfig.h"

/*
 * 匿名上位机V7用户帧F1数据映射。
 *
 * F1数据位：
 *  1 uint16 激光高度，2 uint16 目标高度，3 int16 高度误差
 *  4 int16 高度环输出vel_z，5 int16 飞控反馈vel_z
 *  6 int16 光流X速度，7 int16 光流Y速度
 *  8 uint8 光流质量，9 uint8 飞控实际模式
 * 10 int16 遥控CH3（油门）
 *
 * USERDATA_1~10映射F1数据位1~10。
 * 所有数据的传输缩放均设置为1.0E+0。
 */

/**
 * @brief 设置USERDATA_2显示的目标高度。
 */
void UserDataTransfer_SetTargetHeight(u16 target_height_cm);

/**
 * @brief 组帧并通过UART5发送F1用户自定义数据。
 * @note  由20Hz调度任务调用。
 */
void UserDataTransfer_Task(void);

#endif

#ifndef __USER_DATA_TRANSFER_H
#define __USER_DATA_TRANSFER_H

#include "SysConfig.h"

/*
 * 匿名上位机V7用户帧F1数据映射。
 *
 * 上位机“高级收码 -> 用户帧F1”中，按下列顺序设置数据类型：
 *  1  uint16  激光高度(cm)
 *  2  uint16  目标高度(cm)
 *  3  int16   高度误差(cm)
 *  4  int16   外部高度环输出vel_z(cm/s)
 *  5  int16   飞控反馈实际vel_z(cm/s)
 *  6  int16   光流X速度(cm/s)
 *  7  int16   光流Y速度(cm/s)
 *  8  uint8   光流质量
 *  9  uint8   光流速度有效状态
 * 10  uint8   光流模块连接状态
 * 11  uint8   光流模块工作状态
 * 12  uint8   飞控实际模式
 * 13  uint8   飞控解锁状态
 * 14  uint8   遥控失控保护状态
 * 15  int16   遥控CH1
 * 16  int16   遥控CH2
 * 17  int16   遥控CH3
 * 18  int16   遥控CH4
 * 19  uint8   高度帧更新计数
 * 20  uint8   光流帧更新计数
 *
 * USERDATA_1~20的数据容器依次映射到F1的数据位置1~20。
 * 所有数据的传输缩放均设置为1.0E+0。
 */

/**
 * @brief 设置USERDATA_2显示的目标高度。
 */
void UserDataTransfer_SetTargetHeight(u16 target_height_cm);

/**
 * @brief 发送一帧F1用户自定义数据。
 * @note  建议由20Hz调度任务调用。
 */
void UserDataTransfer_Task(void);

#endif

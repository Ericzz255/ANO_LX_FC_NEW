#ifndef __USER_DATA_TRANSFER_H
#define __USER_DATA_TRANSFER_H

#include "SysConfig.h"

/*
 * 匿名上位机V7用户帧F1/F2数据映射。
 *
 * F1数据位：
 *  1 uint16 激光高度，2 uint16 目标高度，3 int16 高度误差
 *  4 int16 高度环输出vel_z，5 int16 飞控反馈vel_z
 *  6 int16 光流X速度，7 int16 光流Y速度
 *  8 uint8 光流质量，9 uint8 光流速度有效状态
 * 10 uint8 光流模块连接状态
 *
 * F2数据位：
 *  1 uint8 光流模块工作状态，2 uint8 飞控实际模式
 *  3 uint8 飞控解锁状态，4 uint8 遥控失控保护状态
 *  5~8 int16 遥控CH1~CH4
 *  9 uint8 高度帧更新计数，10 uint8 光流帧更新计数
 *
 * USERDATA_1~10映射F1数据位1~10；
 * USERDATA_11~20映射F2数据位1~10。
 * 所有数据的传输缩放均设置为1.0E+0。
 */

/**
 * @brief 设置USERDATA_2显示的目标高度。
 */
void UserDataTransfer_SetTargetHeight(u16 target_height_cm);

/**
 * @brief 向匿名协议发送缓存中填充F1或F2用户自定义数据区。
 * @note  帧头、长度、校验和UART5发送由ANO_DT_LX统一处理。
 */
void UserDataTransfer_FillPayload(u8 frame_id, u8 *buffer, u8 *cnt);

#endif

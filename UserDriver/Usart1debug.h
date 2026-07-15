#ifndef __USART1_DEBUG_H
#define __USART1_DEBUG_H

#include "McuConfig.h"

/**
 * @brief 初始化USART1调试输出（PA9 TX，115200，8N1）
 */
void Usart1Debug_Init(void);

/**
 * @brief 输出一帧已通过CRC校验的SLAM坐标
 * @note  输出格式：SLAM_OK,X=-123,Y=456\r\n
 */
void Usart1Debug_SendSlamCoordinate(s16 x, s16 y);

#endif

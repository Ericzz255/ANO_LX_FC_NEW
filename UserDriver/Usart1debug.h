#ifndef __USART1_DEBUG_H
#define __USART1_DEBUG_H

#include "McuConfig.h"

/* USART1链路自检开关：1=每500ms输出测试帧，0=关闭。 */
#define USART1_DEBUG_SELF_TEST 1

/**
 * @brief 初始化USART1调试输出（PA9 TX，115200，8N1）
 */
void Usart1Debug_Init(void);

/**
 * @brief 输出一帧已通过CRC校验的SLAM坐标
 * @note  输出格式：SLAM_OK,X=-123,Y=456\r\n
 */
void Usart1Debug_SendSlamCoordinate(s16 x, s16 y);

/**
 * @brief USART1链路自检任务，每调用一次发送一帧递增计数
 * @note  输出格式：UART1_TEST,COUNT=123\r\n
 */
void Usart1Debug_TestTask(void);

#endif

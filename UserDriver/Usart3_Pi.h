/**
 * @file Usart3_Pi.h
 * @brief USART3 驱动头文件 —— MaixCam视觉模块通信
 *
 * 硬件连接: MaixCam via UART3 (PB10/PB11, 115200)
 * 主要用途: 发送格子坐标查询，接收视觉识别结果
 *
 * 查询协议: 飞控发送 "#GRID,AxBx*CS\r\n" 给MaixCam
 * 响应协议: 帧头 0x45 + 10字节数据 + 帧尾 0x46
 *           [0]:grid_x(1-9) [1]:grid_y(1-7) [2]:检测数N [3-7]:字符(最多5)
 *           [8]:有效标志(0x01) [9]:校验和(前9字节之和低8位)
 */

#ifndef __USART3_PI_H
#define __USART3_PI_H

#include "McuConfig.h"

void Pi_DataAnl(u8 com_data);
u8 Pi_GetData_Flag(void);
void Pi_GetData(u8* store_array);

/**
 * @brief MaixCam 专用数据就绪标志（不被 Loop_50Hz 的 SLAM 读取消耗）
 */
u8 MaixCam_GetData_Flag(void);

/**
 * @brief 清空接收状态机并丢弃旧数据
 * @note  在发送查询前调用，防止收到MaixCam的旧数据
 */
void Pi_ClearRxState(void);

/**
 * @brief 发送格子坐标查询给MaixCam
 * @param grid_a 列号A(1-9)
 * @param grid_b 行号B(1-7)
 * @note  发送格式: "#GRID,AxBx*CS\r\n"
 */
void Pi_SendGridQuery(u8 grid_a, u8 grid_b);

#endif

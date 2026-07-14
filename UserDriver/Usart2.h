/**
 * @file Usart2.h
 * @brief USART2 驱动头文件 —— 无线串口 DL20（地面站通信）
 *
 * 硬件连接: 无线串口模块 DL20
 * 主要用途: 地面站(GCS)与飞控(FC)之间的通信
 * 核心功能: 接收地面站指令，主要用于设置禁飞区参数
 */

#ifndef __USART2_GS_H
#define __USART2_GS_H

#include "McuConfig.h"

/**
 * @brief 地面站数据逐字节解析（状态机）
 * @param com_data 从串口2接收到的单字节数据
 * @note  由 Drv_Uart.c 的 drvU2DataCheck() 在 ANO_LX_Task() 1ms周期中逐字节调用。
 *        帧格式：0x45(头) + 6字节有效数据(A1,B1,A2,B2,A3,B3) + 0x46(尾)。
 *        解析完成后置位内部标志，供 GS_GetData_Flag 查询。
 */
void GS_DataAnl(u8 com_data);

/**
 * @brief 查询地面站数据接收完成标志
 * @return SET(1) 有新帧已就绪，同时自动清零；RESET(0) 暂无新数据
 * @note  典型调用者：Ano_Scheduler.c 的 Loop_50Hz（20ms周期任务）
 */
u8 GS_GetData_Flag(void);

/**
 * @brief 拷贝最新接收到的有效数据到外部缓冲区
 * @param store_array 外部缓冲区指针，长度至少 6 字节
 * @note  典型调用者：Ano_Scheduler.c 的 Loop_50Hz，在 GS_GetData_Flag 返回 SET 后调用。
 *        数据映射：
 *        [0][1] -> barriers[0].row/col, [2][3] -> barriers[1].row/col, [4][5] -> barriers[2].row/col
 */
void GS_GetData(u8* store_array);

/**
 * @brief 查询地面站是否发送了路径规划触发命令
 * @return SET(1) 已收到 0x55+0xA1+0x65 命令，标志自动清零；RESET(0) 未收到
 * @note  地面端完成路径规划后发送此命令给飞控，帧头/帧尾均与禁飞区帧不同，避免冲突
 */
u8 GS_PlanCmd_Received(void);

#endif

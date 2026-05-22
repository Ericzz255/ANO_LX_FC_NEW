/**
 * @file Usart3_Pi.c
 * @brief USART3 驱动 —— 树莓派(Raspberry Pi)通信
 *
 * 硬件连接: 树莓派 via UART
 * 主要用途: 接收树莓派通过 SLAM 算法解算的 N10P 雷达定位数据
 * 核心功能: 为无人机提供外部定位信息，辅助飞控进行位置估计与导航
 *
 * 数据协议: 帧头 0x45，帧尾 0x46，有效数据长度 PI_VALID_BYTE_LENGTH (4字节)
 */

#include "Usart3_Pi.h"
#include "Drv_Uart.h"

/* 树莓派一帧有效数据长度：4字节 = X坐标高8位 + X坐标低8位 + Y坐标高8位 + Y坐标低8位 */
#define PI_VALID_BYTE_LENGTH 4

/* 一帧数据接收完成标志（由 Pi_DataAnl 置位，由 Pi_GetData_Flag 清零） */
static u8 g_Pi_dataAnlScs_flag = RESET;
/* 接收缓存区，大小需 >= PI_VALID_BYTE_LENGTH */
static u8 g_Pi_val_data[50];

/**
 * @brief 树莓派定位数据逐字节解析（状态机）
 * @note  调用时机：由 Drv_Uart.c 的 drvU3DataCheck() 在串口接收中断上下文外逐字节调用，
 *        最终在 ANO_LX_Task() -> DrvUartDataCheck() -> drvU3DataCheck() 流程中被周期执行。
 * @note  数据格式：帧头 0x45 + 4字节有效数据 + 帧尾 0x46
 *        有效数据含义（由 Ano_Scheduler.c 的 Loop_50Hz 消费）：
 *        [0]:now_x 高8位, [1]:now_x 低8位,
 *        [2]:now_y 高8位, [3]:now_y 低8位
 *        组合方式：now_x = (s16)((data[0] << 8) | data[1])
 */
void Pi_DataAnl(u8 com_data)
{
	static u8 rx_state = 0;
	static u8 check_sum = 0;
	static u8 pack_data_pointer = 0;

	/* 若上一帧尚未被取走，则丢弃新数据，防止覆盖 */
	if (!g_Pi_dataAnlScs_flag)
	{
		/* ---- state 0: 等待帧头 0x45 ---- */
		if (rx_state == 0)
		{
			check_sum = 0;
			if (com_data == 0x45)
			{
				rx_state = 1;
				check_sum += com_data;
			}
			else
			{
				rx_state = 0;
			}
		}
		/* ---- state 1: 接收有效数据区 ---- */
		else if (rx_state == 1)
		{
			*(g_Pi_val_data + pack_data_pointer) = com_data;
			pack_data_pointer++;
			check_sum += com_data;
			if (pack_data_pointer >= PI_VALID_BYTE_LENGTH)
			{
				rx_state = 2;      /* 数据收满，转去等待帧尾 */
				pack_data_pointer = 0;
			}
		}
		/* ---- state 2: 等待帧尾 0x46 ---- */
		else if (rx_state == 2)
		{
			if (com_data == 0x46)
			{
				rx_state = 0;
				g_Pi_dataAnlScs_flag = SET; /* 标记一帧接收完成 */
			}
			else
			{
				rx_state = 0; /* 帧尾错误，重新同步 */
			}
		}
		else
		{
			rx_state = 0;
			check_sum = 0;
			pack_data_pointer = 0;
		}
	}
}

/**
 * @brief 查询树莓派数据接收完成标志
 * @return SET(1) 表示新帧已就绪，同时自动清零标志；RESET(0) 表示暂无新数据
 * @note  调用者：Ano_Scheduler.c 的 Loop_50Hz
 */
u8 Pi_GetData_Flag(void)
{
	if (g_Pi_dataAnlScs_flag)
	{
		g_Pi_dataAnlScs_flag = RESET;
		return SET;
	}
	return RESET;
}

/**
 * @brief 拷贝最新接收到的有效数据到外部缓冲区
 * @param store_array 外部接收缓冲区，长度至少 PI_VALID_BYTE_LENGTH
 * @note  调用者：Ano_Scheduler.c 的 Loop_50Hz，在 Pi_GetData_Flag 返回 SET 后调用
 */
void Pi_GetData(u8* store_array)
{
	for (u8 i = 0; i < PI_VALID_BYTE_LENGTH; i++)
	{
		*(store_array++) = *(g_Pi_val_data + i);
	}
}

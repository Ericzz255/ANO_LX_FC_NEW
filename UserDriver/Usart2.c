/**
 * @file Usart2.c
 * @brief USART2 驱动 —— 无线串口 DL20（地面站通信）
 *
 * 硬件连接: 无线串口模块 DL20
 * 主要用途: 地面站(GCS)与飞控(FC)之间的通信
 * 核心功能: 接收地面站禁飞区数据，发送飞控步进反馈
 *
 * 接收协议: 三个独立坐标帧，每帧为 0x45 + A + B + 0x46
 * 发送协议: 单字符(从 "abcde012345678" 中选)，表示当前到达的路径点
 */

#include "Usart2.h"
#include "Drv_Uart.h"

/* 一组地图包含3个不同禁飞格，每格由一个4字节坐标帧传输。 */
#define GS_VALID_BYTE_LENGTH 6
#define GS_BARRIER_COUNT     3

/* 一帧数据接收完成标志（由 GS_DataAnl 置位，由 GS_GetData_Flag 清零） */
static u8 g_GS_dataAnlScs_flag = RESET;
/* 接收缓存区，大小需 >= GS_VALID_BYTE_LENGTH */
static u8 g_GS_val_data[20];

/* 路径规划触发命令标志（收到 0x55+0xA1+0x65 后置位） */
static u8 g_GS_planCmd_flag = RESET;

/**
 * @brief 地面站数据逐字节解析（状态机）
 * @note  调用时机：由 Drv_Uart.c 的 drvU2DataCheck() 在串口接收中断上下文外逐字节调用，
 *        最终在 ANO_LX_Task() -> DrvUartDataCheck() -> drvU2DataCheck() 流程中被周期执行。
 * @note  坐标帧：0x45 + A + B + 0x46。示例 A9B1 = 45 09 01 46。
 *        连续收到3个合法且不同的坐标后，才发布一组完整地图。
 *        有效数据含义（由 Ano_Scheduler.c 的 Loop_50Hz 消费）：
 *        [0]:A1(1~9), [1]:B1(1~7), [2]:A2, [3]:B2, [4]:A3, [5]:B3，均为1-based坐标
 * @note  同时检测路径规划触发命令：0x55 + 0xA1 + 0x65（3字节小帧，头尾与禁飞区帧不同）
 */
void GS_DataAnl(u8 com_data)
{
	/* ---------- 路径规划触发命令检测（独立状态机） ---------- */
	static u8 plan_rx_state = 0;
	if (!g_GS_planCmd_flag)
	{
		if (plan_rx_state == 0)
		{
			if (com_data == 0x55)
				plan_rx_state = 1;
		}
		else if (plan_rx_state == 1)
		{
			if (com_data == 0xA1)
				plan_rx_state = 2;
			else
				plan_rx_state = (com_data == 0x55) ? 1 : 0;
		}
		else if (plan_rx_state == 2)
		{
			if (com_data == 0x65)
				g_GS_planCmd_flag = SET;
			plan_rx_state = 0;
		}
	}

	/* ---------- 禁飞区4字节坐标帧解析 ---------- */
	static u8 rx_state = 0;
	static u8 coordinate_a = 0;
	static u8 coordinate_b = 0;
	static u8 pending_barriers[GS_VALID_BYTE_LENGTH];
	static u8 pending_count = 0;
	u8 i;
	u8 duplicate;

	/* 若上一帧尚未被取走，则丢弃新数据，防止覆盖 */
	if (!g_GS_dataAnlScs_flag)
	{
		/* ---- state 0: 等待帧头0x45 ---- */
		if (rx_state == 0)
		{
			if (com_data == 0x45)
				rx_state = 1;
		}
		/* ---- state 1: A列(1..9) ---- */
		else if (rx_state == 1)
		{
			coordinate_a = com_data;
			rx_state = 2;
		}
		/* ---- state 2: B行(1..7) ---- */
		else if (rx_state == 2)
		{
			coordinate_b = com_data;
			rx_state = 3;
		}
		/* ---- state 3: 等待帧尾 0x46 ---- */
		else if (rx_state == 3)
		{
			rx_state = (com_data == 0x45) ? 1 : 0;
			if (com_data == 0x46 &&
				coordinate_a >= 1 && coordinate_a <= 9 &&
				coordinate_b >= 1 && coordinate_b <= 7)
			{
				duplicate = RESET;
				for (i = 0; i < pending_count; i++)
				{
					if (pending_barriers[i * 2] == coordinate_a &&
						pending_barriers[i * 2 + 1] == coordinate_b)
					{
						duplicate = SET;
						break;
					}
				}

				if (!duplicate)
				{
					pending_barriers[pending_count * 2] = coordinate_a;
					pending_barriers[pending_count * 2 + 1] = coordinate_b;
					pending_count++;

					if (pending_count >= GS_BARRIER_COUNT)
					{
						for (i = 0; i < GS_VALID_BYTE_LENGTH; i++)
							g_GS_val_data[i] = pending_barriers[i];
						pending_count = 0;
						g_GS_dataAnlScs_flag = SET;
					}
				}
			}
		}
		else
		{
			rx_state = 0;
		}
	}
}

/**
 * @brief 查询地面站数据接收完成标志
 * @return SET(1) 表示新帧已就绪；RESET(0) 表示暂无新数据
 * @note  标志由 GS_GetData 在复制完成后清零，防止复制期间接收缓存被覆盖。
 * @note  调用者：Ano_Scheduler.c 的 Loop_50Hz
 */
u8 GS_GetData_Flag(void)
{
	if (g_GS_dataAnlScs_flag)
	{
		return SET;
	}
	return RESET;
}

/**
 * @brief 拷贝最新接收到的有效数据到外部缓冲区
 * @param store_array 外部接收缓冲区，长度至少 6 字节
 * @note  调用者：Ano_Scheduler.c 的 Loop_50Hz，在 GS_GetData_Flag 返回 SET 后调用。
 *        数据映射：
 *        [0][1] -> barriers[0].row/col, [2][3] -> barriers[1].row/col, [4][5] -> barriers[2].row/col
 */
void GS_GetData(u8* store_array)
{
	u8 i;

	if (store_array == 0 || !g_GS_dataAnlScs_flag)
	{
		return;
	}

	for (i = 0; i < GS_VALID_BYTE_LENGTH; i++)
	{
		*(store_array++) = *(g_GS_val_data + i);
	}
	g_GS_dataAnlScs_flag = RESET;
}

/**
 * @brief 查询地面站路径规划触发命令标志
 * @return SET(1) 已收到 0x55+0xA1+0x65 命令，同时自动清零；RESET(0) 暂无
 */
u8 GS_PlanCmd_Received(void)
{
	if (g_GS_planCmd_flag)
	{
		g_GS_planCmd_flag = RESET;
		return SET;
	}
	return RESET;
}

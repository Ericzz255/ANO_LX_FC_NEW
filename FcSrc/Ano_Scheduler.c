/******************** (C) COPYRIGHT 2017 ANO Tech ********************************
 * 作者    ：匿名科创
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
 * 描述    ：任务调度
**********************************************************************************/
#include "Ano_Scheduler.h"
#include "User_Task.h"
#include "Path_Planning.h"
#include "Usart2.h"
#include "Usart3_Pi.h"
#include "Drv_Uart.h"
#include "ANO_LX.h"
#include <stdio.h>
//////////////////////////////////////////////////////////////////////
//用户程序调度器
//////////////////////////////////////////////////////////////////////

#define SLAM_UART2_DEBUG 1

/* 禁飞区接收状态（文件级变量，供 GS_Barrier_Received() 供 User_Task.c 查询） */
static u8 gs_barrier_received = 0;
static u8 gs_data[10];

/**
 * @brief 通过USART2打印一帧已通过CRC校验的SLAM坐标
 * @note  输出格式：SLAM_OK,X=-123,Y=456\r\n
 */
static void SLAM_Uart2_Debug_Send(s16 x, s16 y)
{
#if SLAM_UART2_DEBUG
	char debug_buf[32];
	int debug_len = snprintf(debug_buf, sizeof(debug_buf),
							 "SLAM_OK,X=%d,Y=%d\r\n", (int)x, (int)y);

	if (debug_len > 0)
	{
		if (debug_len >= sizeof(debug_buf))
		{
			debug_len = sizeof(debug_buf) - 1;
		}
		DrvUart2SendBuf((u8 *)debug_buf, (u8)debug_len);
	}
#else
	(void)x;
	(void)y;
#endif
}

static void Loop_1000Hz(void) //1ms执行一次
{
	//////////////////////////////////////////////////////////////////////

	//////////////////////////////////////////////////////////////////////
}

static void Loop_500Hz(void) //2ms执行一次
{
	//////////////////////////////////////////////////////////////////////

	//////////////////////////////////////////////////////////////////////
}

static void Loop_200Hz(void) //5ms执行一次
{
	//////////////////////////////////////////////////////////////////////

	//////////////////////////////////////////////////////////////////////
}

static void Loop_100Hz(void) //10ms执行一次
{
	//////////////////////////////////////////////////////////////////////

	//////////////////////////////////////////////////////////////////////
}

static void Loop_50Hz(void) //20ms执行一次
{
	// 读取已通过CRC16校验的树莓派定位数据
	static u8 pi_data[4];
	if (Pi_GetData_Flag())
	{
		Pi_GetData(pi_data);
		now_x = (s16)((pi_data[0] << 8) | pi_data[1]);
		now_y = (s16)((pi_data[2] << 8) | pi_data[3]);
		SLAM_Uart2_Debug_Send(now_x, now_y);
	}

	/* 读取地面站禁飞区数据（一帧6字节：A1,B1,A2,B2,A3,B3） */
	if (!gs_barrier_received && GS_GetData_Flag())
	{
		GS_GetData(gs_data);

		// 坐标转换：地面端发的是1-based坐标
		// A: 列号(1-9)，B: 行号(1-7)
		// barriers[].row = B，barriers[].col = A
		u8 a1 = gs_data[0], b1 = gs_data[1];
		u8 a2 = gs_data[2], b2 = gs_data[3];
		u8 a3 = gs_data[4], b3 = gs_data[5];

		u8 valid = 1;
		if (a1 < 1 || a1 > 9 || b1 < 1 || b1 > 7) valid = 0;
		if (a2 < 1 || a2 > 9 || b2 < 1 || b2 > 7) valid = 0;
		if (a3 < 1 || a3 > 9 || b3 < 1 || b3 > 7) valid = 0;

		if (valid)
		{
			barriers[0].row = b1; barriers[0].col = a1;
			barriers[1].row = b2; barriers[1].col = a2;
			barriers[2].row = b3; barriers[2].col = a3;
			gs_barrier_received = 1;
			// 回传确认标志给地面站（0xFF表示已接收）
			u8 ack = 0xFF;
			DrvUart2SendBuf(&ack, 1);
		}
	}

	UserTask_OneKeyCmd();
	//////////////////////////////////////////////////////////////////////
}

u8 GS_Barrier_Received(void)
{
	return gs_barrier_received;
}

static void Loop_20Hz(void) //50ms执行一次
{
}

static void Loop_2Hz(void) //500ms执行一次
{
	
}
//////////////////////////////////////////////////////////////////////
//调度器初始化
//////////////////////////////////////////////////////////////////////
//系统任务配置，创建不同执行频率的“线程”
static sched_task_t sched_tasks[] =
	{
		{Loop_1000Hz, 1000, 0, 0},
		{Loop_500Hz, 500, 0, 0},
		{Loop_200Hz, 200, 0, 0},
		{Loop_100Hz, 100, 0, 0},
		{Loop_50Hz, 50, 0, 0},
		{Loop_20Hz, 20, 0, 0},
		{Loop_2Hz, 2, 0, 0},
};
//根据数组长度，判断线程数量
#define TASK_NUM (sizeof(sched_tasks) / sizeof(sched_task_t))

void Scheduler_Setup(void)
{
	uint8_t index = 0;
	//初始化任务表
	for (index = 0; index < TASK_NUM; index++)
	{
		//计算每个任务的延时周期数
		sched_tasks[index].interval_ticks = TICK_PER_SECOND / sched_tasks[index].rate_hz;
		//最短周期为1，也就是1ms
		if (sched_tasks[index].interval_ticks < 1)
		{
			sched_tasks[index].interval_ticks = 1;
		}
	}
}
//这个函数放到main函数的while(1)中，不停判断是否有线程应该执行
void Scheduler_Run(void)
{
	uint8_t index = 0;
	//循环判断所有线程，是否应该执行

	for (index = 0; index < TASK_NUM; index++)
	{
		//获取系统当前时间，单位MS
		uint32_t tnow = GetSysRunTimeMs();
		//进行判断，如果当前时间减去上一次执行的时间，大于等于该线程的执行周期，则执行线程
		if (tnow - sched_tasks[index].last_run >= sched_tasks[index].interval_ticks)
		{

			//更新线程的执行时间，用于下一次判断
			sched_tasks[index].last_run = tnow;
			//执行线程函数，使用的是函数指针
			sched_tasks[index].task_func();
		}
	}
}

/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/

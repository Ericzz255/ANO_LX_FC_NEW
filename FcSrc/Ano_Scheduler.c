/******************** (C) COPYRIGHT 2017 ANO Tech ********************************
 * 作者    ：匿名科创
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
 * 描述    ：任务调度
**********************************************************************************/
#include "Ano_Scheduler.h"
#include "User_Task.h"
#include "Usart2.h"
#include "Usart3_Pi.h"
#include "Drv_Uart.h"
#include "ANO_LX.h"
#include <stdio.h>
//////////////////////////////////////////////////////////////////////
//用户程序调度器
//////////////////////////////////////////////////////////////////////

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
	// //////////////////////////////////////////////////////////////////////
	// // 延时约3秒后使能UART3接收中断，避开树莓派启动冲击
	// static u16 uart3_rx_delay_cnt = 0;
	// if (uart3_rx_delay_cnt < 150)
	// {
	// 	uart3_rx_delay_cnt++;
	// 	if (uart3_rx_delay_cnt == 150)
	// 	{
	// 		DrvUart3RxEnable();
	// 	}
	// }
	//////////////////////////////////////////////////////////////////////
	// 读取树莓派定位数据
	static u8 pi_data[10];
	if (Pi_GetData_Flag())
	{
		Pi_GetData(pi_data);
		now_x = (s16)((pi_data[0] << 8) | pi_data[1]);
		now_y = (s16)((pi_data[2] << 8) | pi_data[3]);
	}

	/* 暂时注释掉地面站禁飞区接收，当前不接地面站，只测树莓派坐标
	// 读取地面站禁飞区数据（逐个接收，每帧2字节：x,y）
	static u8 gs_data[10];
	static u8 barrier_idx = 0;
	if (barrier_idx < BARRIER_COUNT && GS_GetData_Flag())
	{
		GS_GetData(gs_data);

		// 坐标转换：屏发的是1-based坐标
		// x: 从右到左 1~9（A9=1, A1=9）
		// y: 从下到上 1~7（B1=1, B7=7）
		// 飞控内部用0-based: row=y-1, col=x-1
		u8 x = gs_data[0];
		u8 y = gs_data[1];
		if (x >= 1 && x <= 9 && y >= 1 && y <= 7)
		{
			barriers[barrier_idx].row = y - 1;
			barriers[barrier_idx].col = x - 1;
			barrier_idx++;

			// 回传当前已接收的禁飞区数量给地面站
			u8 ack = barrier_idx;
			DrvUart2SendBuf(&ack, 1);

			if (barrier_idx >= BARRIER_COUNT)
			{
				run_path_planner();
				// 规划完成后不清零，防止飞行中被覆盖
			}
		}
	}
	*/

	UserTask_OneKeyCmd();
	//////////////////////////////////////////////////////////////////////
}

static void Loop_20Hz(void) //50ms执行一次
{
	// 每200ms发送一帧定长二进制调试数据，避免TxBuffer溢出和帧错位
	static u8 print_cnt = 0;
	if (++print_cnt >= 4)  // 50ms * 4 = 200ms
	{
		print_cnt = 0;
		u8 buf[12];
		buf[0] = 0xAA;                  // 帧头1
		buf[1] = 0x55;                  // 帧头2
		buf[2] = (u8)(now_x >> 8);     // now_x 高8位
		buf[3] = (u8)(now_x);           // now_x 低8位
		buf[4] = (u8)(now_y >> 8);     // now_y 高8位
		buf[5] = (u8)(now_y);           // now_y 低8位
		buf[6] = (u8)(rt_tar.st_data.vel_x >> 8);  // vel_x 高8位
		buf[7] = (u8)(rt_tar.st_data.vel_x);       // vel_x 低8位
		buf[8] = (u8)(rt_tar.st_data.vel_y >> 8);  // vel_y 高8位
		buf[9] = (u8)(rt_tar.st_data.vel_y);       // vel_y 低8位
		buf[10] = 0x0D;                 // 帧尾1
		buf[11] = 0x0A;                 // 帧尾2
		DrvUart2SendBuf(buf, 12);
	}
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

/**
 * @file Usart3_Pi.c
 * @brief USART3 驱动 —— MaixCam视觉模块通信
 *
 * 硬件连接: MaixCam via UART3 (PB10/PB11, 115200)
 * 主要用途: 发送格子坐标给MaixCam，接收视觉识别结果
 *
 * 查询协议: 飞控发送 "#GRID,AxBx*CS\r\n" 给MaixCam
 * 响应协议: 帧头 0x45 + 10字节数据 + 帧尾 0x46
 *           [0]:grid_x(1-9) [1]:grid_y(1-7) [2]:检测数N [3-7]:字符(最多5)
 *           [8]:有效标志(0x01) [9]:校验和(前9字节之和低8位)
 */

#include "Usart3_Pi.h"
#include "Drv_Uart.h"

/* MaixCam响应帧有效数据长度：10字节 */
#define PI_VALID_BYTE_LENGTH 10

/* 一帧数据接收完成标志（供 Loop_50Hz SLAM 数据读取使用） */
static u8 g_Pi_dataAnlScs_flag = RESET;
/* MaixCam 专用数据就绪标志（供 User_Task.c case 8 使用，不被 Loop_50Hz 消耗） */
static u8 g_Maixcam_data_ready = RESET;
/* 接收缓存区 */
static u8 g_Pi_val_data[50];

/* 解析状态机变量（文件级，供 Pi_ClearRxState 重置） */
static u8 g_Pi_rx_state = 0;
static u8 g_Pi_pack_data_pointer = 0;
static u8 g_Pi_rx_checksum = 0;

/* 外部声明 UART3 环形缓冲区计数器（用于清空旧数据） */
extern u8 U3RxInCnt;
extern u8 U3RxoutCnt;

/**
 * @brief MaixCam数据逐字节解析（状态机）
 * @note  数据格式：帧头 0x45 + 10字节 + 帧尾 0x46
 *        [0]:grid_x [1]:grid_y [2]:N [3-7]:chars [8]:valid [9]:checksum
 */
void Pi_DataAnl(u8 com_data)
{
	if (!g_Pi_dataAnlScs_flag)
	{
		if (g_Pi_rx_state == 0)
		{
			g_Pi_rx_checksum = 0;
			if (com_data == 0x45)
			{
				g_Pi_rx_state = 1;
			}
		}
		else if (g_Pi_rx_state == 1)
		{
			*(g_Pi_val_data + g_Pi_pack_data_pointer) = com_data;
			if (g_Pi_pack_data_pointer < 9)
			{
				g_Pi_rx_checksum += com_data;
			}
			g_Pi_pack_data_pointer++;
			if (g_Pi_pack_data_pointer >= PI_VALID_BYTE_LENGTH)
			{
				g_Pi_rx_state = 2;
				g_Pi_pack_data_pointer = 0;
			}
		}
		else if (g_Pi_rx_state == 2)
		{
			if (com_data == 0x46 && g_Pi_val_data[9] == g_Pi_rx_checksum)
			{
				g_Pi_rx_state = 0;
				g_Pi_dataAnlScs_flag = SET;
				g_Maixcam_data_ready = SET; /* 同时设置 MaixCam 专用标志 */
			}
			else
			{
				g_Pi_rx_state = 0;
			}
		}
		else
		{
			g_Pi_rx_state = 0;
			g_Pi_pack_data_pointer = 0;
		}
	}
}

/**
 * @brief 清空接收状态机并丢弃旧数据
 * @note  在发送查询前调用，清空解析状态和环形缓冲区，防止收到MaixCam的旧数据
 */
void Pi_ClearRxState(void)
{
	g_Pi_dataAnlScs_flag = RESET;
	g_Maixcam_data_ready = RESET;
	g_Pi_rx_state = 0;
	g_Pi_pack_data_pointer = 0;
	g_Pi_rx_checksum = 0;
	/* 丢弃UART3环形缓冲区中已接收但未处理的旧数据 */
	U3RxInCnt = U3RxoutCnt;
}

/**
 * @brief 查询MaixCam数据接收完成标志
 * @return SET(1) 有检测结果已就绪；RESET(0) 暂无
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
 * @brief 拷贝MaixCam检测数据到外部缓冲区
 * @param store_array 外部缓冲区，长度至少 10 字节
 */
void Pi_GetData(u8* store_array)
{
	for (u8 i = 0; i < PI_VALID_BYTE_LENGTH; i++)
	{
		*(store_array++) = *(g_Pi_val_data + i);
	}
}

/**
 * @brief 查询MaixCam专用数据就绪标志（不被 Loop_50Hz 的 SLAM 数据读取消耗）
 * @return SET(1) MaixCam检测结果已就绪；RESET(0) 暂无
 * @note  与 Pi_GetData_Flag() 共用同一数据缓冲区，但使用独立标志位，
 *         避免 Loop_50Hz 中读取 SLAM 数据时意外清除 MaixCam 响应标志。
 */
u8 MaixCam_GetData_Flag(void)
{
	if (g_Maixcam_data_ready)
	{
		g_Maixcam_data_ready = RESET;
		return SET;
	}
	return RESET;
}

/**
 * @brief 发送格子坐标查询给MaixCam
 * @param grid_a 列号A(1-9)
 * @param grid_b 行号B(1-7)
 * @note  发送格式: "#GRID,AxBx*CS\r\n"
 */
void Pi_SendGridQuery(u8 grid_a, u8 grid_b)
{
	char buf[32];
	/* grid_a=列号A(1-9), grid_b=行号B(1-7), 发送格式 #GRID,A<col>B<row>*CS */
	u8 cs = ('A' + grid_a + 'B' + grid_b) & 0xFF;
	snprintf(buf, sizeof(buf), "#GRID,A%dB%d*%02X\r\n", grid_a, grid_b, cs);
	DrvUart3SendBuf((u8*)buf, strlen(buf));
}

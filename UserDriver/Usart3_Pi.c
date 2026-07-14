/**
 * @file Usart3_Pi.c
 * @brief USART3 驱动 —— 树莓派 SLAM 定位数据接收
 *
 * 硬件连接: N10P 雷达 -> 树莓派 SLAM -> USART3 (115200)
 * 帧格式: 45 53 05 SEQ X_H X_L Y_H Y_L CRC_H CRC_L
 * CRC16-CCITT计算范围: LEN、SEQ、X_H、X_L、Y_H、Y_L
 */

#include "Usart3_Pi.h"

#define PI_HEADER_1           0x45
#define PI_HEADER_2           0x53
#define PI_PAYLOAD_LENGTH     5
#define PI_POSITION_LENGTH    4
#define PI_CRC16_INITIAL      0xFFFF

typedef enum
{
	PI_RX_WAIT_HEADER_1 = 0,
	PI_RX_WAIT_HEADER_2,
	PI_RX_WAIT_LENGTH,
	PI_RX_READ_PAYLOAD,
	PI_RX_READ_CRC_HIGH,
	PI_RX_READ_CRC_LOW
} PiRxState;

static volatile u8 g_Pi_dataAnlScs_flag = RESET;
static u8 g_Pi_val_data[PI_POSITION_LENGTH];

static u16 Pi_Crc16Update(u16 crc, u8 data)
{
	u8 bit;
	crc ^= (u16)data << 8;
	for (bit = 0; bit < 8; bit++)
	{
		if (crc & 0x8000)
		{
			crc = (u16)((crc << 1) ^ 0x1021);
		}
		else
		{
			crc <<= 1;
		}
	}
	return crc;
}

/**
 * @brief 树莓派定位数据逐字节解析状态机
 * @note 固定长度字段允许坐标数据包含0x45或0x53；只有CRC正确才发布坐标。
 */
void Pi_DataAnl(u8 com_data)
{
	static PiRxState rx_state = PI_RX_WAIT_HEADER_1;
	static u8 payload[PI_PAYLOAD_LENGTH];
	static u8 payload_index = 0;
	static u16 crc_calculated = PI_CRC16_INITIAL;
	static u16 crc_received = 0;
	u8 i;

	switch (rx_state)
	{
	case PI_RX_WAIT_HEADER_1:
		if (com_data == PI_HEADER_1)
		{
			rx_state = PI_RX_WAIT_HEADER_2;
		}
		break;

	case PI_RX_WAIT_HEADER_2:
		if (com_data == PI_HEADER_2)
		{
			rx_state = PI_RX_WAIT_LENGTH;
		}
		else if (com_data != PI_HEADER_1)
		{
			rx_state = PI_RX_WAIT_HEADER_1;
		}
		break;

	case PI_RX_WAIT_LENGTH:
		if (com_data == PI_PAYLOAD_LENGTH)
		{
			payload_index = 0;
			crc_calculated = Pi_Crc16Update(PI_CRC16_INITIAL, com_data);
			rx_state = PI_RX_READ_PAYLOAD;
		}
		else
		{
			rx_state = (com_data == PI_HEADER_1) ? PI_RX_WAIT_HEADER_2 : PI_RX_WAIT_HEADER_1;
		}
		break;

	case PI_RX_READ_PAYLOAD:
		payload[payload_index++] = com_data;
		crc_calculated = Pi_Crc16Update(crc_calculated, com_data);
		if (payload_index >= PI_PAYLOAD_LENGTH)
		{
			rx_state = PI_RX_READ_CRC_HIGH;
		}
		break;

	case PI_RX_READ_CRC_HIGH:
		crc_received = (u16)com_data << 8;
		rx_state = PI_RX_READ_CRC_LOW;
		break;

	case PI_RX_READ_CRC_LOW:
		crc_received |= com_data;
		if (crc_received == crc_calculated)
		{
			/* payload[0]为序号，payload[1..4]为X/Y坐标。 */
			for (i = 0; i < PI_POSITION_LENGTH; i++)
			{
				g_Pi_val_data[i] = payload[i + 1];
			}
			g_Pi_dataAnlScs_flag = SET;
		}
		rx_state = (com_data == PI_HEADER_1) ? PI_RX_WAIT_HEADER_2 : PI_RX_WAIT_HEADER_1;
		break;
	}
}

u8 Pi_GetData_Flag(void)
{
	if (g_Pi_dataAnlScs_flag)
	{
		g_Pi_dataAnlScs_flag = RESET;
		return SET;
	}
	return RESET;
}

void Pi_GetData(u8 *store_array)
{
	u8 i;
	for (i = 0; i < PI_POSITION_LENGTH; i++)
	{
		store_array[i] = g_Pi_val_data[i];
	}
}

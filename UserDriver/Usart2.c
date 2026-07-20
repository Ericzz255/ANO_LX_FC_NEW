/**
 * @file Usart2.c
 * @brief USART2 wireless link to the ground station.
 *
 * Frame: 0x45 + A1 B1 A2 B2 A3 B3 + 0x46
 */

#include "Usart2.h"

#define GS_VALID_BYTE_LENGTH 6

static u8 g_GS_dataAnlScs_flag = RESET;
static u8 g_GS_val_data[GS_VALID_BYTE_LENGTH];

/*
 * Keep this parser identical to the previously verified project:
 * wait for 0x45, receive exactly six coordinate bytes, then wait for 0x46.
 */
void GS_DataAnl(u8 com_data)
{
	static u8 rx_state = 0;
	static u8 check_sum = 0;
	static u8 pack_data_pointer = 0;

	if (!g_GS_dataAnlScs_flag)
	{
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
		else if (rx_state == 1)
		{
			g_GS_val_data[pack_data_pointer] = com_data;
			pack_data_pointer++;
			check_sum += com_data;
			if (pack_data_pointer >= GS_VALID_BYTE_LENGTH)
			{
				rx_state = 2;
				pack_data_pointer = 0;
			}
		}
		else if (rx_state == 2)
		{
			if (com_data == 0x46)
			{
				rx_state = 0;
				g_GS_dataAnlScs_flag = SET;
			}
			else
			{
				rx_state = 0;
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

/* The verified old implementation clears the ready flag when queried. */
u8 GS_GetData_Flag(void)
{
	if (g_GS_dataAnlScs_flag)
	{
		g_GS_dataAnlScs_flag = RESET;
		return SET;
	}
	return RESET;
}

void GS_GetData(u8 *store_array)
{
	u8 i;

	if (store_array == 0)
	{
		return;
	}

	for (i = 0; i < GS_VALID_BYTE_LENGTH; i++)
	{
		store_array[i] = g_GS_val_data[i];
	}
}

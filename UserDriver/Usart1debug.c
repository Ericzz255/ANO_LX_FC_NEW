#include "Usart1debug.h"
#include "Drv_Uart.h"
#include <stdio.h>

#define USART1_DEBUG_ENABLED     1
#define USART1_DEBUG_BAUDRATE    115200U
#define USART1_DEBUG_BUFFER_SIZE 32U

void Usart1Debug_Init(void)
{
	DrvUart1Init(USART1_DEBUG_BAUDRATE);
}

void Usart1Debug_SendSlamCoordinate(s16 x, s16 y)
{
#if USART1_DEBUG_ENABLED
	char debug_buf[USART1_DEBUG_BUFFER_SIZE];
	int debug_len = snprintf(debug_buf, sizeof(debug_buf),
							 "SLAM_OK,X=%d,Y=%d\r\n", (int)x, (int)y);

	if (debug_len > 0)
	{
		if (debug_len >= sizeof(debug_buf))
		{
			debug_len = sizeof(debug_buf) - 1;
		}
		DrvUart1SendBuf((u8 *)debug_buf, (u8)debug_len);
	}
#else
	(void)x;
	(void)y;
#endif
}

#ifndef __USART2_GS_H
#define __USART2_GS_H

#include "McuConfig.h"

/*
 * USART2 ground-station frame:
 *   0x45 A1 B1 A2 B2 A3 B3 0x46
 * A is the column (1..9), B is the row (1..7).
 */
void GS_DataAnl(u8 com_data);
u8 GS_GetData_Flag(void);
void GS_GetData(u8 *store_array);

#endif

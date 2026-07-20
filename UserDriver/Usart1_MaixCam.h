#ifndef __USART1_MAIXCAM_H
#define __USART1_MAIXCAM_H

#include "McuConfig.h"

#define MAIXCAM_RESPONSE_DATA_LENGTH 10U

/*
 * USART1 connection:
 *   STM32 PA9  (TX) -> MaixCam RX
 *   STM32 PA10 (RX) <- MaixCam TX
 *   Common GND, 115200 baud, 8N1
 *
 * Query:
 *   "#GRID,A<column>B<row>*<checksum>\r\n"
 *
 * Response:
 *   0x45 + 10 data bytes + 0x46
 *   data[0] : grid column, 1 to 9
 *   data[1] : grid row, 1 to 7
 *   data[2] : detected item count
 *   data[3] to data[7] : detected characters, at most 5
 *   data[8] : valid flag, 0x01 means valid
 *   data[9] : low 8 bits of the sum of data[0] to data[8]
 */

void MaixCam_Init(void);
void MaixCam_DataAnl(u8 com_data);
u8 MaixCam_GetDataFlag(void);
void MaixCam_GetData(u8 *store_array);
void MaixCam_ClearRxState(void);
void MaixCam_SendGridQuery(u8 grid_column, u8 grid_row);

#endif

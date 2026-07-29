#ifndef _GROUND_STATION_RX_H_
#define _GROUND_STATION_RX_H_

#include "SysConfig.h"

#define GROUND_STATION_RX_MAX_PAYLOAD 32U

typedef struct
{
    u32 received_bytes;
    u32 valid_frames;
    u32 crc_errors;
    u32 length_errors;
    u32 last_activity_ms;
    u32 last_valid_frame_ms;
    u8 last_type;
    u8 last_sequence;
} ground_station_rx_stats_t;

/*
 * USART2 byte-stream parser.
 * Frame: AA 55 TYPE LEN PAYLOAD[LEN] CRC16_LO CRC16_HI.
 * CRC16-CCITT-FALSE covers TYPE, LEN and PAYLOAD.
 */
void GroundStationRx_GetOneByte(u8 data);

/* Returns SET after a valid frame has been received within timeout_ms. */
u8 GroundStationRx_IsOnline(u32 timeout_ms);

/* Copies the current receive diagnostics for ground-station debugging. */
void GroundStationRx_GetStats(ground_station_rx_stats_t *stats);

#endif

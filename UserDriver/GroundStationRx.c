#include "GroundStationRx.h"
#include "Drv_Sys.h"

typedef enum
{
    GS_RX_WAIT_HEADER_1 = 0,
    GS_RX_WAIT_HEADER_2,
    GS_RX_READ_TYPE,
    GS_RX_READ_LENGTH,
    GS_RX_READ_PAYLOAD,
    GS_RX_READ_CRC_LOW,
    GS_RX_READ_CRC_HIGH
} ground_station_rx_state_t;

static ground_station_rx_state_t rx_state = GS_RX_WAIT_HEADER_1;
static u8 rx_type;
static u8 rx_length;
static u8 rx_index;
static u8 rx_payload[GROUND_STATION_RX_MAX_PAYLOAD];
static u16 rx_crc;
static u16 received_crc;
static volatile ground_station_rx_stats_t rx_stats;

static u16 GroundStationRx_Crc16Update(u16 crc, u8 data)
{
    u8 i;

    crc ^= (u16)data << 8;
    for (i = 0; i < 8U; i++)
    {
        if ((crc & 0x8000U) != 0U)
        {
            crc = (u16)((crc << 1) ^ 0x1021U);
        }
        else
        {
            crc <<= 1;
        }
    }

    return crc;
}

static void GroundStationRx_Reset(void)
{
    rx_state = GS_RX_WAIT_HEADER_1;
    rx_length = 0U;
    rx_index = 0U;
}

void GroundStationRx_GetOneByte(u8 data)
{
    rx_stats.received_bytes++;
    rx_stats.last_activity_ms = GetSysRunTimeMs();

    switch (rx_state)
    {
    case GS_RX_WAIT_HEADER_1:
        if (data == 0xAAU)
        {
            rx_state = GS_RX_WAIT_HEADER_2;
        }
        break;

    case GS_RX_WAIT_HEADER_2:
        if (data == 0x55U)
        {
            rx_state = GS_RX_READ_TYPE;
        }
        else if (data != 0xAAU)
        {
            rx_state = GS_RX_WAIT_HEADER_1;
        }
        break;

    case GS_RX_READ_TYPE:
        rx_type = data;
        rx_crc = GroundStationRx_Crc16Update(0xFFFFU, data);
        rx_state = GS_RX_READ_LENGTH;
        break;

    case GS_RX_READ_LENGTH:
        rx_length = data;
        rx_index = 0U;
        rx_crc = GroundStationRx_Crc16Update(rx_crc, data);

        if (rx_length > GROUND_STATION_RX_MAX_PAYLOAD)
        {
            rx_stats.length_errors++;
            GroundStationRx_Reset();
        }
        else if (rx_length == 0U)
        {
            rx_state = GS_RX_READ_CRC_LOW;
        }
        else
        {
            rx_state = GS_RX_READ_PAYLOAD;
        }
        break;

    case GS_RX_READ_PAYLOAD:
        rx_payload[rx_index++] = data;
        rx_crc = GroundStationRx_Crc16Update(rx_crc, data);
        if (rx_index >= rx_length)
        {
            rx_state = GS_RX_READ_CRC_LOW;
        }
        break;

    case GS_RX_READ_CRC_LOW:
        received_crc = data;
        rx_state = GS_RX_READ_CRC_HIGH;
        break;

    case GS_RX_READ_CRC_HIGH:
        received_crc |= (u16)data << 8;
        if (received_crc == rx_crc)
        {
            rx_stats.valid_frames++;
            rx_stats.last_valid_frame_ms = GetSysRunTimeMs();
            rx_stats.last_type = rx_type;
            rx_stats.last_sequence =
                (rx_length > 0U) ? rx_payload[0] : 0U;
        }
        else
        {
            rx_stats.crc_errors++;
        }
        GroundStationRx_Reset();
        break;

    default:
        GroundStationRx_Reset();
        break;
    }
}

u8 GroundStationRx_IsOnline(u32 timeout_ms)
{
    if (rx_stats.valid_frames == 0U)
    {
        return RESET;
    }

    return ((u32)(GetSysRunTimeMs() - rx_stats.last_valid_frame_ms) <=
            timeout_ms) ? SET : RESET;
}

void GroundStationRx_GetStats(ground_station_rx_stats_t *stats)
{
    if (stats == 0)
    {
        return;
    }

    stats->received_bytes = rx_stats.received_bytes;
    stats->valid_frames = rx_stats.valid_frames;
    stats->crc_errors = rx_stats.crc_errors;
    stats->length_errors = rx_stats.length_errors;
    stats->last_activity_ms = rx_stats.last_activity_ms;
    stats->last_valid_frame_ms = rx_stats.last_valid_frame_ms;
    stats->last_type = rx_stats.last_type;
    stats->last_sequence = rx_stats.last_sequence;
}

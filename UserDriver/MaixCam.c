#include "MaixCam.h"
#include "Drv_Sys.h"
#include "Drv_Uart.h"

#define MAIXCAM_HEADER_1              0xAAU
#define MAIXCAM_HEADER_2              0x4DU

#define MAIXCAM_TYPE_TRACKING         0x01U
#define MAIXCAM_TYPE_POSITION         0x02U
#define MAIXCAM_TYPE_MODE_COMMAND     0x80U

#define MAIXCAM_TRACKING_LENGTH       8U
#define MAIXCAM_POSITION_LENGTH       10U
#define MAIXCAM_MAX_PAYLOAD           16U
#define MAIXCAM_FLAG_VALID            (1U << 0)

typedef enum
{
    MAIX_RX_WAIT_HEADER_1 = 0,
    MAIX_RX_WAIT_HEADER_2,
    MAIX_RX_READ_TYPE,
    MAIX_RX_READ_LENGTH,
    MAIX_RX_READ_PAYLOAD,
    MAIX_RX_READ_CRC_LOW,
    MAIX_RX_READ_CRC_HIGH
} maixcam_rx_state_t;

static maixcam_rx_state_t rx_state = MAIX_RX_WAIT_HEADER_1;
static u8 rx_type;
static u8 rx_length;
static u8 rx_index;
static u8 rx_payload[MAIXCAM_MAX_PAYLOAD];
static u16 rx_crc;
static u16 received_crc;
static u32 last_rx_byte_ms;

static volatile maixcam_tracking_t latest_tracking;
static volatile maixcam_position_t latest_position;
static volatile u8 tracking_valid;
static volatile u8 position_valid;
static volatile maixcam_stats_t maixcam_stats;

static u16 MaixCam_Crc16Update(u16 crc, u8 data)
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

static s16 MaixCam_ReadS16Le(const u8 *data)
{
    return (s16)((u16)data[0] | ((u16)data[1] << 8));
}

static void MaixCam_ResetParser(void)
{
    rx_state = MAIX_RX_WAIT_HEADER_1;
    rx_length = 0U;
    rx_index = 0U;
}

static void MaixCam_PublishTracking(s16 x,
                                    s16 y,
                                    u8 confidence,
                                    u8 sequence,
                                    u8 valid)
{
    latest_tracking.x = x;
    latest_tracking.y = y;
    latest_tracking.confidence = confidence;
    latest_tracking.sequence = sequence;
    latest_tracking.update_ms = GetSysRunTimeMs();
    tracking_valid = valid;
}

static void MaixCam_PublishPosition(s16 x_cm,
                                    s16 y_cm,
                                    s16 yaw_cdeg,
                                    u8 quality,
                                    u8 sequence,
                                    u8 valid)
{
    latest_position.x_cm = x_cm;
    latest_position.y_cm = y_cm;
    latest_position.yaw_cdeg = yaw_cdeg;
    latest_position.quality = quality;
    latest_position.sequence = sequence;
    latest_position.update_ms = GetSysRunTimeMs();
    position_valid = valid;
}

static void MaixCam_ProcessFrame(void)
{
    maixcam_stats.valid_frames++;

    if (rx_type == MAIXCAM_TYPE_TRACKING &&
        rx_length == MAIXCAM_TRACKING_LENGTH)
    {
        MaixCam_PublishTracking(
            MaixCam_ReadS16Le(&rx_payload[2]),
            MaixCam_ReadS16Le(&rx_payload[4]),
            rx_payload[6],
            rx_payload[0],
            (rx_payload[1] & MAIXCAM_FLAG_VALID) ? SET : RESET);
    }
    else if (rx_type == MAIXCAM_TYPE_POSITION &&
             rx_length == MAIXCAM_POSITION_LENGTH)
    {
        MaixCam_PublishPosition(
            MaixCam_ReadS16Le(&rx_payload[2]),
            MaixCam_ReadS16Le(&rx_payload[4]),
            MaixCam_ReadS16Le(&rx_payload[6]),
            rx_payload[8],
            rx_payload[0],
            (rx_payload[1] & MAIXCAM_FLAG_VALID) ? SET : RESET);
    }
    else
    {
        maixcam_stats.unknown_frames++;
    }
}

static void MaixCam_ParseFramedByte(u8 data)
{
    switch (rx_state)
    {
    case MAIX_RX_WAIT_HEADER_1:
        if (data == MAIXCAM_HEADER_1)
        {
            rx_state = MAIX_RX_WAIT_HEADER_2;
        }
        break;

    case MAIX_RX_WAIT_HEADER_2:
        if (data == MAIXCAM_HEADER_2)
        {
            rx_state = MAIX_RX_READ_TYPE;
        }
        else if (data != MAIXCAM_HEADER_1)
        {
            rx_state = MAIX_RX_WAIT_HEADER_1;
        }
        break;

    case MAIX_RX_READ_TYPE:
        rx_type = data;
        rx_crc = MaixCam_Crc16Update(0xFFFFU, data);
        rx_state = MAIX_RX_READ_LENGTH;
        break;

    case MAIX_RX_READ_LENGTH:
        rx_length = data;
        rx_index = 0U;
        rx_crc = MaixCam_Crc16Update(rx_crc, data);
        if (rx_length > MAIXCAM_MAX_PAYLOAD)
        {
            maixcam_stats.length_errors++;
            MaixCam_ResetParser();
        }
        else if (rx_length == 0U)
        {
            rx_state = MAIX_RX_READ_CRC_LOW;
        }
        else
        {
            rx_state = MAIX_RX_READ_PAYLOAD;
        }
        break;

    case MAIX_RX_READ_PAYLOAD:
        rx_payload[rx_index++] = data;
        rx_crc = MaixCam_Crc16Update(rx_crc, data);
        if (rx_index >= rx_length)
        {
            rx_state = MAIX_RX_READ_CRC_LOW;
        }
        break;

    case MAIX_RX_READ_CRC_LOW:
        received_crc = data;
        rx_state = MAIX_RX_READ_CRC_HIGH;
        break;

    case MAIX_RX_READ_CRC_HIGH:
        received_crc |= (u16)data << 8;
        if (received_crc == rx_crc)
        {
            MaixCam_ProcessFrame();
        }
        else
        {
            maixcam_stats.crc_errors++;
        }
        MaixCam_ResetParser();
        break;

    default:
        MaixCam_ResetParser();
        break;
    }
}

void MaixCam_GetOneByte(u8 data)
{
    u32 now_ms = GetSysRunTimeMs();

    if (maixcam_stats.received_bytes > 0U &&
        (u32)(now_ms - last_rx_byte_ms) > 50U)
    {
        MaixCam_ResetParser();
    }

    last_rx_byte_ms = now_ms;
    maixcam_stats.received_bytes++;
    MaixCam_ParseFramedByte(data);
}

u8 MaixCam_GetTracking(maixcam_tracking_t *tracking, u32 timeout_ms)
{
    if (tracking == 0 || tracking_valid == RESET)
    {
        return RESET;
    }

    tracking->x = latest_tracking.x;
    tracking->y = latest_tracking.y;
    tracking->confidence = latest_tracking.confidence;
    tracking->sequence = latest_tracking.sequence;
    tracking->update_ms = latest_tracking.update_ms;

    return ((u32)(GetSysRunTimeMs() - tracking->update_ms) <=
            timeout_ms) ? SET : RESET;
}

u8 MaixCam_GetPosition(maixcam_position_t *position, u32 timeout_ms)
{
    if (position == 0 || position_valid == RESET)
    {
        return RESET;
    }

    position->x_cm = latest_position.x_cm;
    position->y_cm = latest_position.y_cm;
    position->yaw_cdeg = latest_position.yaw_cdeg;
    position->quality = latest_position.quality;
    position->sequence = latest_position.sequence;
    position->update_ms = latest_position.update_ms;

    return ((u32)(GetSysRunTimeMs() - position->update_ms) <=
            timeout_ms) ? SET : RESET;
}

void MaixCam_GetStats(maixcam_stats_t *stats)
{
    if (stats == 0)
    {
        return;
    }

    stats->received_bytes = maixcam_stats.received_bytes;
    stats->valid_frames = maixcam_stats.valid_frames;
    stats->crc_errors = maixcam_stats.crc_errors;
    stats->length_errors = maixcam_stats.length_errors;
    stats->unknown_frames = maixcam_stats.unknown_frames;
}

void MaixCam_SendMode(u8 mode)
{
    static u8 command_sequence;
    u8 frame[8];
    u8 index = 0U;
    u16 crc;

    frame[index++] = MAIXCAM_HEADER_1;
    frame[index++] = MAIXCAM_HEADER_2;
    frame[index++] = MAIXCAM_TYPE_MODE_COMMAND;
    frame[index++] = 2U;
    frame[index++] = command_sequence++;
    frame[index++] = mode;

    crc = 0xFFFFU;
    crc = MaixCam_Crc16Update(crc, frame[2]);
    crc = MaixCam_Crc16Update(crc, frame[3]);
    crc = MaixCam_Crc16Update(crc, frame[4]);
    crc = MaixCam_Crc16Update(crc, frame[5]);
    frame[index++] = (u8)(crc & 0xFFU);
    frame[index++] = (u8)((crc >> 8) & 0xFFU);

    DrvUart1SendBuf(frame, index);
}

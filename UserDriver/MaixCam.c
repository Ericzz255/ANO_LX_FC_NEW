#include "MaixCam.h"
#include "Drv_Sys.h"
#include "Drv_Uart.h"

#define MAIXCAM_HEADER_1              0xAAU
#define MAIXCAM_HEADER_2              0x4DU

#define MAIXCAM_TYPE_TRACKING         0x01U
#define MAIXCAM_TYPE_POSITION_RSVD    0x02U
#define MAIXCAM_TYPE_MODE_COMMAND     0x80U
#define MAIXCAM_TYPE_MODE_ACK         0x81U

#define MAIXCAM_TRACKING_LENGTH       8U
#define MAIXCAM_MODE_COMMAND_LENGTH   2U
#define MAIXCAM_MODE_ACK_LENGTH       4U
#define MAIXCAM_MAX_PAYLOAD           32U
#define MAIXCAM_INTERBYTE_TIMEOUT_MS  50U
#define MAIXCAM_MODE_REPEAT_MS        500U

#define MAIXCAM_FLAG_TARGET_VALID     (1U << 0)
#define MAIXCAM_FLAG_ALL_FOUR         (1U << 1)
#define MAIXCAM_FLAG_CALIBRATED       (1U << 2)
#define MAIXCAM_FLAG_HELD             (1U << 3)
#define MAIXCAM_REQUIRED_FLAGS        0x07U
#define MAIXCAM_VALID_TAG_MASK        0x0FU
#define MAIXCAM_VALID_QUALITY         255U

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
static volatile maixcam_mode_status_t mode_status;
static volatile maixcam_stats_t maixcam_stats;
static volatile u8 tracking_control_valid;
static u8 tracking_sequence_seen;
static u8 last_tracking_sequence;
static u8 next_command_sequence;
static u8 last_command_sequence;
static u8 mode_command_sent;
static u32 last_mode_command_ms;

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

static u16 MaixCam_Crc16(const u8 *data, u8 length)
{
    u16 crc = 0xFFFFU;

    while (length-- > 0U)
    {
        crc = MaixCam_Crc16Update(crc, *data++);
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

static u8 MaixCam_TrackingFlagsAreValid(u8 flags)
{
    if ((flags & 0x0FU) != MAIXCAM_REQUIRED_FLAGS)
    {
        return RESET;
    }

    if ((flags & 0xF0U) != 0U ||
        (flags & MAIXCAM_FLAG_HELD) != 0U)
    {
        return RESET;
    }

    return SET;
}

static void MaixCam_InvalidateTracking(void)
{
    tracking_control_valid = RESET;
}

static void MaixCam_ProcessTracking(void)
{
    u8 sequence = rx_payload[0];
    u8 flags = rx_payload[1];
    u8 quality = rx_payload[6];
    u8 tag_mask = rx_payload[7];
    u8 frame_valid;
    s16 error_x_e4;
    s16 error_y_e4;

    if (tracking_sequence_seen != 0U &&
        sequence == last_tracking_sequence)
    {
        maixcam_stats.sequence_errors++;
        MaixCam_InvalidateTracking();
        return;
    }

    tracking_sequence_seen = 1U;
    last_tracking_sequence = sequence;
    error_x_e4 = MaixCam_ReadS16Le(&rx_payload[2]);
    error_y_e4 = MaixCam_ReadS16Le(&rx_payload[4]);

    latest_tracking.error_x_e4 = error_x_e4;
    latest_tracking.error_y_e4 = error_y_e4;
    latest_tracking.error_x = (float)error_x_e4 / 10000.0f;
    latest_tracking.error_y = (float)error_y_e4 / 10000.0f;
    latest_tracking.flags = flags;
    latest_tracking.quality = quality;
    latest_tracking.tag_mask = tag_mask;
    latest_tracking.sequence = sequence;
    latest_tracking.update_ms = GetSysRunTimeMs();

    frame_valid =
        (MaixCam_TrackingFlagsAreValid(flags) != RESET &&
         quality == MAIXCAM_VALID_QUALITY &&
         tag_mask == MAIXCAM_VALID_TAG_MASK &&
         mode_status.confirmed != RESET &&
         mode_status.current_mode == MAIXCAM_MODE_TRACKING &&
         mode_status.requested_mode == MAIXCAM_MODE_TRACKING) ?
        SET : RESET;

    tracking_control_valid = frame_valid;
}

static void MaixCam_ProcessModeAck(void)
{
    u8 command_sequence = rx_payload[0];
    u8 current_mode = rx_payload[1];
    u8 result = rx_payload[2];
    u8 protocol_major = rx_payload[3];

    maixcam_stats.mode_ack_frames++;
    mode_status.command_sequence = command_sequence;
    mode_status.current_mode = current_mode;
    mode_status.result = result;
    mode_status.protocol_major = protocol_major;
    mode_status.update_ms = GetSysRunTimeMs();

    if (protocol_major != MAIXCAM_PROTOCOL_MAJOR ||
        current_mode > MAIXCAM_MODE_TRACKING ||
        result > 2U)
    {
        maixcam_stats.protocol_errors++;
        mode_status.confirmed = RESET;
        MaixCam_InvalidateTracking();
        return;
    }

    mode_status.confirmed =
        (command_sequence == last_command_sequence &&
         result == 0U &&
         current_mode == mode_status.requested_mode) ? SET : RESET;

    if (mode_status.confirmed == RESET ||
        current_mode != MAIXCAM_MODE_TRACKING)
    {
        MaixCam_InvalidateTracking();
    }
}

static void MaixCam_ProcessFrame(void)
{
    if (rx_type == MAIXCAM_TYPE_TRACKING)
    {
        if (rx_length != MAIXCAM_TRACKING_LENGTH)
        {
            maixcam_stats.length_errors++;
            MaixCam_InvalidateTracking();
            return;
        }

        maixcam_stats.valid_frames++;
        MaixCam_ProcessTracking();
    }
    else if (rx_type == MAIXCAM_TYPE_MODE_ACK)
    {
        if (rx_length != MAIXCAM_MODE_ACK_LENGTH)
        {
            maixcam_stats.length_errors++;
            MaixCam_InvalidateTracking();
            return;
        }

        maixcam_stats.valid_frames++;
        MaixCam_ProcessModeAck();
    }
    else
    {
        /* TYPE 0x02 is reserved in V1.0 and is treated as unknown. */
        maixcam_stats.unknown_frames++;
        if (rx_type == MAIXCAM_TYPE_POSITION_RSVD)
        {
            maixcam_stats.protocol_errors++;
        }
        MaixCam_InvalidateTracking();
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
            MaixCam_InvalidateTracking();
            MaixCam_ResetParser();
        }
        else if ((rx_type == MAIXCAM_TYPE_TRACKING &&
                  rx_length != MAIXCAM_TRACKING_LENGTH) ||
                 (rx_type == MAIXCAM_TYPE_MODE_ACK &&
                  rx_length != MAIXCAM_MODE_ACK_LENGTH))
        {
            maixcam_stats.length_errors++;
            MaixCam_InvalidateTracking();
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
            MaixCam_InvalidateTracking();
        }
        MaixCam_ResetParser();
        break;

    default:
        MaixCam_InvalidateTracking();
        MaixCam_ResetParser();
        break;
    }
}

static void MaixCam_SendModeCommand(void)
{
    u8 frame[8];
    u8 index = 0U;
    u16 crc;
    u8 command_sequence = next_command_sequence++;

    frame[index++] = MAIXCAM_HEADER_1;
    frame[index++] = MAIXCAM_HEADER_2;
    frame[index++] = MAIXCAM_TYPE_MODE_COMMAND;
    frame[index++] = MAIXCAM_MODE_COMMAND_LENGTH;
    frame[index++] = command_sequence;
    frame[index++] = mode_status.requested_mode;

    crc = MaixCam_Crc16(&frame[2], 4U);
    frame[index++] = (u8)(crc & 0xFFU);
    frame[index++] = (u8)((crc >> 8) & 0xFFU);

    last_command_sequence = command_sequence;
    mode_status.command_sequence = command_sequence;
    mode_command_sent = 1U;
    last_mode_command_ms = GetSysRunTimeMs();
    DrvUart1SendBuf(frame, index);
}

u8 MaixCam_ProtocolSelfTest(void)
{
    static const u8 tracking_crc_data[10] =
    {
        0x01U, 0x08U, 0x01U, 0x07U, 0xE8U,
        0x03U, 0x0CU, 0xFEU, 0xFFU, 0x0FU
    };
    static const u8 command_crc_data[4] =
    {
        0x80U, 0x02U, 0x10U, 0x01U
    };
    static const u8 ack_crc_data[6] =
    {
        0x81U, 0x04U, 0x10U, 0x01U, 0x00U, 0x01U
    };

    return (MaixCam_Crc16(tracking_crc_data,
                          sizeof(tracking_crc_data)) == 0xD4CDU &&
            MaixCam_Crc16(command_crc_data,
                          sizeof(command_crc_data)) == 0x24CAU &&
            MaixCam_Crc16(ack_crc_data,
                          sizeof(ack_crc_data)) == 0x2A20U) ?
           SET : RESET;
}

void MaixCam_Init(void)
{
    MaixCam_ResetParser();
    tracking_control_valid = RESET;
    tracking_sequence_seen = 0U;
    mode_status.requested_mode = MAIXCAM_MODE_IDLE;
    mode_status.current_mode = MAIXCAM_MODE_IDLE;
    mode_status.result = 0U;
    mode_status.protocol_major = 0U;
    mode_status.confirmed = RESET;
    mode_command_sent = 0U;
    maixcam_stats.self_test_pass = MaixCam_ProtocolSelfTest();
}

void MaixCam_GetOneByte(u8 data)
{
    u32 now_ms = GetSysRunTimeMs();

    if (maixcam_stats.received_bytes > 0U &&
        (u32)(now_ms - last_rx_byte_ms) >
        MAIXCAM_INTERBYTE_TIMEOUT_MS)
    {
        MaixCam_InvalidateTracking();
        MaixCam_ResetParser();
    }

    last_rx_byte_ms = now_ms;
    maixcam_stats.received_bytes++;
    MaixCam_ParseFramedByte(data);
}

u8 MaixCam_GetTracking(maixcam_tracking_t *tracking)
{
    if (tracking == 0 ||
        maixcam_stats.self_test_pass == RESET ||
        tracking_control_valid == RESET)
    {
        return RESET;
    }

    tracking->error_x_e4 = latest_tracking.error_x_e4;
    tracking->error_y_e4 = latest_tracking.error_y_e4;
    tracking->error_x = latest_tracking.error_x;
    tracking->error_y = latest_tracking.error_y;
    tracking->flags = latest_tracking.flags;
    tracking->quality = latest_tracking.quality;
    tracking->tag_mask = latest_tracking.tag_mask;
    tracking->sequence = latest_tracking.sequence;
    tracking->update_ms = latest_tracking.update_ms;

    if ((u32)(GetSysRunTimeMs() - tracking->update_ms) >
        MAIXCAM_TRACKING_TIMEOUT_MS)
    {
        MaixCam_InvalidateTracking();
        return RESET;
    }

    return SET;
}

void MaixCam_SetMode(u8 mode)
{
    if (mode > MAIXCAM_MODE_TRACKING)
    {
        maixcam_stats.protocol_errors++;
        return;
    }

    if (mode_status.requested_mode != mode)
    {
        mode_status.requested_mode = mode;
        mode_status.confirmed = RESET;
        mode_command_sent = 0U;
        MaixCam_InvalidateTracking();
    }
}

void MaixCam_Task(void)
{
    u32 now_ms = GetSysRunTimeMs();

    if (tracking_control_valid != RESET &&
        (u32)(now_ms - latest_tracking.update_ms) >
        MAIXCAM_TRACKING_TIMEOUT_MS)
    {
        MaixCam_InvalidateTracking();
    }

    if (maixcam_stats.self_test_pass != RESET &&
        (mode_command_sent == 0U ||
         (u32)(now_ms - last_mode_command_ms) >=
         MAIXCAM_MODE_REPEAT_MS))
    {
        MaixCam_SendModeCommand();
    }
}

void MaixCam_GetModeStatus(maixcam_mode_status_t *status)
{
    if (status == 0)
    {
        return;
    }

    status->command_sequence = mode_status.command_sequence;
    status->requested_mode = mode_status.requested_mode;
    status->current_mode = mode_status.current_mode;
    status->result = mode_status.result;
    status->protocol_major = mode_status.protocol_major;
    status->confirmed = mode_status.confirmed;
    status->update_ms = mode_status.update_ms;
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
    stats->sequence_errors = maixcam_stats.sequence_errors;
    stats->protocol_errors = maixcam_stats.protocol_errors;
    stats->mode_ack_frames = maixcam_stats.mode_ack_frames;
    stats->self_test_pass = maixcam_stats.self_test_pass;
}

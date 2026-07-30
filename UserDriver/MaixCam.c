#include "MaixCam.h"
#include "Drv_Sys.h"

#define MAIXCAM_HEADER_1              0xAAU
#define MAIXCAM_HEADER_2              0x5AU
#define MAIXCAM_TYPE_TRACKING         0x21U
#define MAIXCAM_TRACKING_LENGTH       13U
#define MAIXCAM_MAX_PAYLOAD           32U
#define MAIXCAM_INTERBYTE_TIMEOUT_MS  50U

#define MAIXCAM_FLAG_TARGET_VALID     (1U << 0)
#define MAIXCAM_FLAG_ALL_FOUR         (1U << 1)
#define MAIXCAM_FLAG_CALIBRATED       (1U << 2)
#define MAIXCAM_FLAG_HELD             (1U << 3)
#define MAIXCAM_FLAG_SEARCH_ACTIVE    (1U << 4)
#define MAIXCAM_FLAG_PARTIAL_CENTERED (1U << 5)
#define MAIXCAM_FLAG_DIAGONAL_TRACK   (1U << 6)
#define MAIXCAM_FLAG_RESERVED         (1U << 7)

#define MAIXCAM_ALL_TAG_MASK          0x0FU
#define MAIXCAM_DIAGONAL_1_2_MASK     0x06U
#define MAIXCAM_DIAGONAL_0_3_MASK     0x09U
#define MAIXCAM_SYNTHETIC_QUALITY_OK  255U

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

static u16 MaixCam_ReadU16Le(const u8 *data)
{
    return (u16)data[0] | ((u16)data[1] << 8);
}

static s16 MaixCam_ReadS16Le(const u8 *data)
{
    return (s16)MaixCam_ReadU16Le(data);
}

static void MaixCam_ResetParser(void)
{
    rx_state = MAIX_RX_WAIT_HEADER_1;
    rx_length = 0U;
    rx_index = 0U;
}

static void MaixCam_InvalidateTracking(void)
{
    tracking_control_valid = RESET;
}

static u8 MaixCam_TagGeometryIsValid(u8 flags, u8 tag_mask)
{
    u8 all_four = flags & MAIXCAM_FLAG_ALL_FOUR;
    u8 diagonal = flags & MAIXCAM_FLAG_DIAGONAL_TRACK;

    if ((tag_mask & 0xF0U) != 0U ||
        (all_four != 0U && diagonal != 0U) ||
        (all_four == 0U && diagonal == 0U))
    {
        return RESET;
    }

    if (all_four != 0U)
    {
        return (tag_mask == MAIXCAM_ALL_TAG_MASK) ? SET : RESET;
    }

    if ((tag_mask & MAIXCAM_DIAGONAL_1_2_MASK) ==
            MAIXCAM_DIAGONAL_1_2_MASK ||
        (tag_mask & MAIXCAM_DIAGONAL_0_3_MASK) ==
            MAIXCAM_DIAGONAL_0_3_MASK)
    {
        return SET;
    }

    return RESET;
}

static u8 MaixCam_TrackingFlagsAreValid(u8 flags, u8 tag_mask)
{
    if ((flags & MAIXCAM_FLAG_TARGET_VALID) == 0U ||
        (flags & MAIXCAM_FLAG_CALIBRATED) == 0U ||
        (flags & (MAIXCAM_FLAG_HELD |
                  MAIXCAM_FLAG_SEARCH_ACTIVE |
                  MAIXCAM_FLAG_PARTIAL_CENTERED |
                  MAIXCAM_FLAG_RESERVED)) != 0U)
    {
        return RESET;
    }

    return MaixCam_TagGeometryIsValid(flags, tag_mask);
}

static void MaixCam_ProcessTracking(void)
{
    u8 sequence = rx_payload[0];
    u8 flags = rx_payload[1];
    u8 tag_mask = rx_payload[10];
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
    error_x_e4 = MaixCam_ReadS16Le(&rx_payload[6]);
    error_y_e4 = MaixCam_ReadS16Le(&rx_payload[8]);

    latest_tracking.center_x_px = MaixCam_ReadU16Le(&rx_payload[2]);
    latest_tracking.center_y_px = MaixCam_ReadU16Le(&rx_payload[4]);
    latest_tracking.error_x_e4 = error_x_e4;
    latest_tracking.error_y_e4 = error_y_e4;
    latest_tracking.error_x = (float)error_x_e4 / 10000.0f;
    latest_tracking.error_y = (float)error_y_e4 / 10000.0f;
    latest_tracking.flags = flags;
    latest_tracking.tag_mask = tag_mask;
    latest_tracking.fps_x10 = MaixCam_ReadU16Le(&rx_payload[11]);
    latest_tracking.sequence = sequence;
    latest_tracking.update_ms = GetSysRunTimeMs();

    frame_valid =
        (mode_status.requested_mode == MAIXCAM_MODE_TRACKING &&
         MaixCam_TrackingFlagsAreValid(flags, tag_mask) != RESET) ?
        SET : RESET;

    latest_tracking.quality =
        (frame_valid != RESET) ? MAIXCAM_SYNTHETIC_QUALITY_OK : 0U;
    tracking_control_valid = frame_valid;
}

static void MaixCam_ProcessFrame(void)
{
    if (rx_type != MAIXCAM_TYPE_TRACKING)
    {
        maixcam_stats.unknown_frames++;
        MaixCam_InvalidateTracking();
        return;
    }

    if (rx_length != MAIXCAM_TRACKING_LENGTH)
    {
        maixcam_stats.length_errors++;
        MaixCam_InvalidateTracking();
        return;
    }

    maixcam_stats.valid_frames++;
    MaixCam_ProcessTracking();
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
        if (rx_length > MAIXCAM_MAX_PAYLOAD ||
            (rx_type == MAIXCAM_TYPE_TRACKING &&
             rx_length != MAIXCAM_TRACKING_LENGTH))
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

u8 MaixCam_ProtocolSelfTest(void)
{
    static const u8 tracking_crc_data[15] =
    {
        0x21U, 0x0DU, 0x01U, 0x07U, 0x40U,
        0x01U, 0xF0U, 0x00U, 0xE8U, 0x03U,
        0x0CU, 0xFEU, 0x0FU, 0xC8U, 0x00U
    };
    static const u8 no_target_crc_data[15] =
    {
        0x21U, 0x0DU, 0x02U, 0x04U, 0xFFU,
        0xFFU, 0xFFU, 0xFFU, 0x00U, 0x00U,
        0x00U, 0x00U, 0x00U, 0xC8U, 0x00U
    };

    return (MaixCam_Crc16(tracking_crc_data,
                          sizeof(tracking_crc_data)) == 0x220BU &&
            MaixCam_Crc16(no_target_crc_data,
                          sizeof(no_target_crc_data)) == 0xC416U) ?
           SET : RESET;
}

void MaixCam_Init(void)
{
    MaixCam_ResetParser();
    tracking_control_valid = RESET;
    tracking_sequence_seen = 0U;
    mode_status.command_sequence = 0U;
    mode_status.requested_mode = MAIXCAM_MODE_IDLE;
    mode_status.current_mode = MAIXCAM_MODE_IDLE;
    mode_status.result = 0U;
    mode_status.protocol_major = 0U;
    mode_status.confirmed = SET;
    mode_status.update_ms = GetSysRunTimeMs();
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

    tracking->center_x_px = latest_tracking.center_x_px;
    tracking->center_y_px = latest_tracking.center_y_px;
    tracking->error_x_e4 = latest_tracking.error_x_e4;
    tracking->error_y_e4 = latest_tracking.error_y_e4;
    tracking->error_x = latest_tracking.error_x;
    tracking->error_y = latest_tracking.error_y;
    tracking->flags = latest_tracking.flags;
    tracking->quality = latest_tracking.quality;
    tracking->tag_mask = latest_tracking.tag_mask;
    tracking->sequence = latest_tracking.sequence;
    tracking->fps_x10 = latest_tracking.fps_x10;
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
        mode_status.current_mode = mode;
        mode_status.confirmed = SET;
        mode_status.update_ms = GetSysRunTimeMs();
        MaixCam_InvalidateTracking();
    }
}

void MaixCam_Task(void)
{
    if (tracking_control_valid != RESET &&
        (u32)(GetSysRunTimeMs() - latest_tracking.update_ms) >
        MAIXCAM_TRACKING_TIMEOUT_MS)
    {
        MaixCam_InvalidateTracking();
    }
}

u8 MaixCam_IsLinkAlive(void)
{
    if (maixcam_stats.self_test_pass == RESET ||
        maixcam_stats.valid_frames == 0U)
    {
        return RESET;
    }

    return ((u32)(GetSysRunTimeMs() - latest_tracking.update_ms) <=
            MAIXCAM_LINK_TIMEOUT_MS) ? SET : RESET;
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

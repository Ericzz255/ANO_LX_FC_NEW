#ifndef _MAIXCAM_H_
#define _MAIXCAM_H_

#include "SysConfig.h"

#define MAIXCAM_PROTOCOL_MAJOR       1U
#define MAIXCAM_MODE_IDLE            0U
#define MAIXCAM_MODE_TRACKING        1U
#define MAIXCAM_TRACKING_TIMEOUT_MS  150U

typedef struct
{
    s16 error_x_e4;
    s16 error_y_e4;
    float error_x;
    float error_y;
    u8 flags;
    u8 quality;
    u8 tag_mask;
    u8 sequence;
    u32 update_ms;
} maixcam_tracking_t;

typedef struct
{
    u8 command_sequence;
    u8 requested_mode;
    u8 current_mode;
    u8 result;
    u8 protocol_major;
    u8 confirmed;
    u32 update_ms;
} maixcam_mode_status_t;

typedef struct
{
    u32 received_bytes;
    u32 valid_frames;
    u32 crc_errors;
    u32 length_errors;
    u32 unknown_frames;
    u32 sequence_errors;
    u32 protocol_errors;
    u32 mode_ack_frames;
    u8 self_test_pass;
} maixcam_stats_t;

void MaixCam_Init(void);

/* USART1 byte callback, called by drvU1DataCheck(). */
void MaixCam_GetOneByte(u8 data);

/*
 * Returns SET only when the V1.0 control-validity conditions are all met.
 * error_x/error_y are normalized body-frame optical errors, not pixels or cm.
 */
u8 MaixCam_GetTracking(maixcam_tracking_t *tracking);

void MaixCam_SetMode(u8 mode);
void MaixCam_Task(void);
void MaixCam_GetModeStatus(maixcam_mode_status_t *status);
void MaixCam_GetStats(maixcam_stats_t *stats);
u8 MaixCam_ProtocolSelfTest(void);

#endif

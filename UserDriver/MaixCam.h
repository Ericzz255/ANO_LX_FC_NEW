#ifndef _MAIXCAM_H_
#define _MAIXCAM_H_

#include "SysConfig.h"

#define MAIXCAM_MODE_IDLE          0U
#define MAIXCAM_MODE_TRACKING      1U
#define MAIXCAM_MODE_LOCALIZATION  2U
#define MAIXCAM_MODE_TRACK_AND_LOC 3U

typedef struct
{
    s16 x;
    s16 y;
    u8 confidence;
    u8 sequence;
    u32 update_ms;
} maixcam_tracking_t;

typedef struct
{
    s16 x_cm;
    s16 y_cm;
    s16 yaw_cdeg;
    u8 quality;
    u8 sequence;
    u32 update_ms;
} maixcam_position_t;

typedef struct
{
    u32 received_bytes;
    u32 valid_frames;
    u32 crc_errors;
    u32 length_errors;
    u32 unknown_frames;
} maixcam_stats_t;

/* USART1 byte callback, called by drvU1DataCheck(). */
void MaixCam_GetOneByte(u8 data);

/*
 * Returns SET only when the corresponding result is marked valid and fresh.
 * timeout_ms is normally 200..300 ms for flight control.
 */
u8 MaixCam_GetTracking(maixcam_tracking_t *tracking, u32 timeout_ms);
u8 MaixCam_GetPosition(maixcam_position_t *position, u32 timeout_ms);

void MaixCam_GetStats(maixcam_stats_t *stats);

/* Sends a framed operating-mode command to MaixCAM through USART1. */
void MaixCam_SendMode(u8 mode);

#endif

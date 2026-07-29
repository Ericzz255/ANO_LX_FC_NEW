#include "VisionFollowControl.h"
#include "MaixCam.h"
#include "ANO_LX.h"

#define VISION_FOLLOW_PERIOD_S            0.02f
#define VISION_FOLLOW_MIN_FRAME_DT_S      0.02f
#define VISION_FOLLOW_MAX_FRAME_DT_S      0.20f
#define VISION_FOLLOW_DEADBAND            0.008f
#define VISION_FOLLOW_CENTER_TOLERANCE     0.015f
#define VISION_FOLLOW_KP                  100.0f
#define VISION_FOLLOW_KI                  8.0f
#define VISION_FOLLOW_KD                  2.0f
#define VISION_FOLLOW_INTEGRAL_LIMIT      1.50f
#define VISION_FOLLOW_MAX_VEL_CMPS        15.0f
#define VISION_FOLLOW_MAX_VEL_STEP_CMPS   2

typedef struct
{
    u8 active;
    u8 sequence_seen;
    u8 last_sequence;
    u32 last_frame_ms;
    float integral_x;
    float integral_y;
    float last_error_x;
    float last_error_y;
    s16 target_vel_x;
    s16 target_vel_y;
    s16 output_vel_x;
    s16 output_vel_y;
} VisionFollowControlState;

static VisionFollowControlState vision_follow;

static float VisionFollowControl_AbsFloat(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float VisionFollowControl_LimitFloat(float value,
                                            float minimum,
                                            float maximum)
{
    if (value > maximum)
    {
        return maximum;
    }
    if (value < minimum)
    {
        return minimum;
    }

    return value;
}

static s16 VisionFollowControl_FloatToS16(float value)
{
    value = VisionFollowControl_LimitFloat(
        value,
        -VISION_FOLLOW_MAX_VEL_CMPS,
        VISION_FOLLOW_MAX_VEL_CMPS);

    return (value >= 0.0f) ?
           (s16)(value + 0.5f) :
           (s16)(value - 0.5f);
}

static s16 VisionFollowControl_ApplySlewLimit(s16 target, s16 current)
{
    s16 delta = target - current;

    if (delta > VISION_FOLLOW_MAX_VEL_STEP_CMPS)
    {
        delta = VISION_FOLLOW_MAX_VEL_STEP_CMPS;
    }
    else if (delta < -VISION_FOLLOW_MAX_VEL_STEP_CMPS)
    {
        delta = -VISION_FOLLOW_MAX_VEL_STEP_CMPS;
    }

    return current + delta;
}

static void VisionFollowControl_ClearPid(void)
{
    vision_follow.sequence_seen = 0U;
    vision_follow.last_sequence = 0U;
    vision_follow.last_frame_ms = 0U;
    vision_follow.integral_x = 0.0f;
    vision_follow.integral_y = 0.0f;
    vision_follow.last_error_x = 0.0f;
    vision_follow.last_error_y = 0.0f;
    vision_follow.target_vel_x = 0;
    vision_follow.target_vel_y = 0;
}

void VisionFollowControl_Begin(void)
{
    VisionFollowControl_ClearPid();
    vision_follow.output_vel_x =
        VisionFollowControl_FloatToS16(
            (float)rt_tar.st_data.vel_x);
    vision_follow.output_vel_y =
        VisionFollowControl_FloatToS16(
            (float)rt_tar.st_data.vel_y);
    vision_follow.active = 1U;
}

void VisionFollowControl_Reset(void)
{
    VisionFollowControl_ClearPid();
    vision_follow.active = 0U;
    vision_follow.output_vel_x = 0;
    vision_follow.output_vel_y = 0;
    rt_tar.st_data.vel_x = 0;
    rt_tar.st_data.vel_y = 0;
}

static void VisionFollowControl_UpdatePid(
    const maixcam_tracking_t *tracking)
{
    float error_x = tracking->error_x;
    float error_y = tracking->error_y;
    float derivative_x = 0.0f;
    float derivative_y = 0.0f;
    float frame_dt_s = VISION_FOLLOW_PERIOD_S;
    float velocity_x;
    float velocity_y;

    if (VisionFollowControl_AbsFloat(error_x) <
        VISION_FOLLOW_DEADBAND)
    {
        error_x = 0.0f;
    }
    if (VisionFollowControl_AbsFloat(error_y) <
        VISION_FOLLOW_DEADBAND)
    {
        error_y = 0.0f;
    }

    if (vision_follow.sequence_seen != 0U)
    {
        frame_dt_s =
            (float)((u32)(tracking->update_ms -
                          vision_follow.last_frame_ms)) /
            1000.0f;
        frame_dt_s = VisionFollowControl_LimitFloat(
            frame_dt_s,
            VISION_FOLLOW_MIN_FRAME_DT_S,
            VISION_FOLLOW_MAX_FRAME_DT_S);

        if (error_x != 0.0f)
        {
            derivative_x =
                (error_x - vision_follow.last_error_x) /
                frame_dt_s;
        }
        if (error_y != 0.0f)
        {
            derivative_y =
                (error_y - vision_follow.last_error_y) /
                frame_dt_s;
        }
    }

    vision_follow.integral_x += error_x * frame_dt_s;
    vision_follow.integral_y += error_y * frame_dt_s;
    vision_follow.integral_x = VisionFollowControl_LimitFloat(
        vision_follow.integral_x,
        -VISION_FOLLOW_INTEGRAL_LIMIT,
        VISION_FOLLOW_INTEGRAL_LIMIT);
    vision_follow.integral_y = VisionFollowControl_LimitFloat(
        vision_follow.integral_y,
        -VISION_FOLLOW_INTEGRAL_LIMIT,
        VISION_FOLLOW_INTEGRAL_LIMIT);

    velocity_x = VISION_FOLLOW_KP * error_x +
                 VISION_FOLLOW_KI * vision_follow.integral_x +
                 VISION_FOLLOW_KD * derivative_x;
    velocity_y = VISION_FOLLOW_KP * error_y +
                 VISION_FOLLOW_KI * vision_follow.integral_y +
                 VISION_FOLLOW_KD * derivative_y;

    vision_follow.target_vel_x =
        VisionFollowControl_FloatToS16(velocity_x);
    vision_follow.target_vel_y =
        VisionFollowControl_FloatToS16(velocity_y);
    vision_follow.last_error_x = error_x;
    vision_follow.last_error_y = error_y;
    vision_follow.last_sequence = tracking->sequence;
    vision_follow.last_frame_ms = tracking->update_ms;
    vision_follow.sequence_seen = 1U;
}

u8 VisionFollowControl_Update(void)
{
    maixcam_tracking_t tracking;

    if (MaixCam_GetTracking(&tracking) == RESET)
    {
        VisionFollowControl_Reset();
        return RESET;
    }

    if (vision_follow.active == 0U)
    {
        VisionFollowControl_Begin();
    }

    if (vision_follow.sequence_seen == 0U ||
        tracking.sequence != vision_follow.last_sequence)
    {
        VisionFollowControl_UpdatePid(&tracking);
    }

    vision_follow.output_vel_x =
        VisionFollowControl_ApplySlewLimit(
            vision_follow.target_vel_x,
            vision_follow.output_vel_x);
    vision_follow.output_vel_y =
        VisionFollowControl_ApplySlewLimit(
            vision_follow.target_vel_y,
            vision_follow.output_vel_y);

    rt_tar.st_data.vel_x = vision_follow.output_vel_x;
    rt_tar.st_data.vel_y = vision_follow.output_vel_y;
    return SET;
}

u8 VisionFollowControl_IsTargetCentered(void)
{
    maixcam_tracking_t tracking;

    if (vision_follow.active == 0U ||
        MaixCam_GetTracking(&tracking) == RESET)
    {
        return RESET;
    }

    if (VisionFollowControl_AbsFloat(tracking.error_x) <=
            VISION_FOLLOW_CENTER_TOLERANCE &&
        VisionFollowControl_AbsFloat(tracking.error_y) <=
            VISION_FOLLOW_CENTER_TOLERANCE)
    {
        return SET;
    }

    return RESET;
}

s16 VisionFollowControl_GetOutputVelX(void)
{
    return vision_follow.output_vel_x;
}

s16 VisionFollowControl_GetOutputVelY(void)
{
    return vision_follow.output_vel_y;
}

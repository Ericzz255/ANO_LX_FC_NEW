#include "HorizontalControl.h"
#include "ANO_LX.h"
#include "LX_FC_State.h"

/*
 * 水平位置外环：SLAM位置误差(cm) -> 水平速度目标(cm/s)。
 * 凌霄飞控内部继续完成水平速度内环。
 */
#define HORIZONTAL_HOLD_PERIOD_MS              20U     /* 控制周期(ms) */
#define HORIZONTAL_HOLD_KP_X                   0.35f   /* X轴位置环P增益 */
#define HORIZONTAL_HOLD_KD_X                   0.08f   /* X轴位置环D增益 */
#define HORIZONTAL_HOLD_KP_Y                   0.40f   /* Y轴位置环P增益 */
#define HORIZONTAL_HOLD_KD_Y                   0.05f   /* Y轴位置环D增益 */
#define HORIZONTAL_HOLD_DEADBAND_CM            3       /* 位置死区(cm)，误差小于此值不控 */
#define HORIZONTAL_HOLD_MAX_VEL_CMPS           10      /* 位置环输出最大速度(cm/s) */
#define HORIZONTAL_HOLD_MAX_VEL_STEP_CMPS      2       /* 单周期速度增量限幅(cm/s) */
#define HORIZONTAL_HOLD_SENSOR_TIMEOUT_MS      300U    /* 传感器超时时间(ms) */
#define HORIZONTAL_HOLD_MAX_ERROR_CM           50      /* 首次闭环测试最大允许位置误差(cm) */
#define HORIZONTAL_HOLD_DIVERGENCE_CHECK_MS    1000U   /* 发散检测时间窗(ms) */
#define HORIZONTAL_HOLD_DIVERGENCE_GROWTH_CM   5       /* 时间窗内允许的误差增长(cm) */

#define HORIZONTAL_HOLD_FAULT_NONE              0U
#define HORIZONTAL_HOLD_FAULT_MAX_ERROR         1U
#define HORIZONTAL_HOLD_FAULT_DIVERGENCE        2U

s16 now_x = 0;
s16 now_y = 0;

static s16 hold_target_x = 0;
static s16 hold_target_y = 0;
static u8 slam_position_update_cnt = 0;
static u8 slam_position_valid = 0;

typedef struct
{
    u8 initialized;
    u8 last_update_cnt;
    u16 stale_ms;
    u16 divergence_check_ms;
    u8 fault_code;
    s16 last_vel_x;
    s16 last_vel_y;
    s32 last_error_x_cm;
    s32 last_error_y_cm;
    s32 divergence_reference_error_cm;
} HorizontalControlState;

static HorizontalControlState horizontal_control;

static s32 HorizontalControl_AbsS32(s32 value)
{
    return (value >= 0) ? value : -value;
}

static s32 HorizontalControl_MaxAbsError(s32 error_x_cm, s32 error_y_cm)
{
    s32 abs_error_x_cm = HorizontalControl_AbsS32(error_x_cm);
    s32 abs_error_y_cm = HorizontalControl_AbsS32(error_y_cm);

    return (abs_error_x_cm >= abs_error_y_cm) ?
           abs_error_x_cm : abs_error_y_cm;
}

static s16 HorizontalControl_S32ToS16(s32 value)
{
    if (value > 32767)
    {
        return 32767;
    }
    if (value < -32768)
    {
        return -32768;
    }

    return (s16)value;
}

static s16 HorizontalControl_LimitVelocity(float velocity_cmps)
{
    if (velocity_cmps > HORIZONTAL_HOLD_MAX_VEL_CMPS)
    {
        velocity_cmps = HORIZONTAL_HOLD_MAX_VEL_CMPS;
    }
    else if (velocity_cmps < -HORIZONTAL_HOLD_MAX_VEL_CMPS)
    {
        velocity_cmps = -HORIZONTAL_HOLD_MAX_VEL_CMPS;
    }

    if (velocity_cmps >= 0.0f)
    {
        return (s16)(velocity_cmps + 0.5f);
    }

    return (s16)(velocity_cmps - 0.5f);
}

static s16 HorizontalControl_ApplySlewLimit(s16 target, s16 current)
{
    s16 delta = target - current;

    if (delta > HORIZONTAL_HOLD_MAX_VEL_STEP_CMPS)
    {
        delta = HORIZONTAL_HOLD_MAX_VEL_STEP_CMPS;
    }
    else if (delta < -HORIZONTAL_HOLD_MAX_VEL_STEP_CMPS)
    {
        delta = -HORIZONTAL_HOLD_MAX_VEL_STEP_CMPS;
    }

    return current + delta;
}

static s16 HorizontalControl_CalculateVelocity(s32 error_cm,
                                               s32 *last_error_cm,
                                               float kp,
                                               float kd)
{
    float velocity_cmps;

    velocity_cmps = kp * (float)error_cm +
                    kd * (float)(error_cm - *last_error_cm);
    *last_error_cm = error_cm;

    /*
     * SLAM位置与0x41水平速度均采用机头X正、机左Y正。
     * 误差采用“目标-当前位置”，因此PID输出无需额外取负。
     */
    return HorizontalControl_LimitVelocity(velocity_cmps);
}

void HorizontalControl_SetPosition(s16 x_cm, s16 y_cm)
{
    now_x = x_cm;
    now_y = y_cm;
    slam_position_update_cnt++;
    slam_position_valid = 1;
}

void HorizontalControl_StopOutput(void)
{
    horizontal_control.last_vel_x = 0;
    horizontal_control.last_vel_y = 0;
    rt_tar.st_data.vel_x = 0;
    rt_tar.st_data.vel_y = 0;
}

void HorizontalControl_Reset(void)
{
    horizontal_control.initialized = 0;
    horizontal_control.last_update_cnt = slam_position_update_cnt;
    horizontal_control.stale_ms = 0;
    horizontal_control.divergence_check_ms = 0;
    horizontal_control.fault_code = HORIZONTAL_HOLD_FAULT_NONE;
    horizontal_control.last_error_x_cm = 0;
    horizontal_control.last_error_y_cm = 0;
    horizontal_control.divergence_reference_error_cm = 0;
    HorizontalControl_StopOutput();
}

void HorizontalControl_CaptureTarget(void)
{
    if (slam_position_valid)
    {
        hold_target_x = now_x;
        hold_target_y = now_y;
        horizontal_control.initialized = 1;
        horizontal_control.last_update_cnt = slam_position_update_cnt;
        horizontal_control.stale_ms = 0;
        horizontal_control.divergence_check_ms = 0;
        horizontal_control.last_error_x_cm = 0;
        horizontal_control.last_error_y_cm = 0;
        horizontal_control.divergence_reference_error_cm = 0;
    }
}

static void HorizontalControl_LatchFault(u8 fault_code)
{
    horizontal_control.fault_code = fault_code;
    horizontal_control.divergence_check_ms = 0;
    HorizontalControl_StopOutput();
}

static void HorizontalControl_CheckDivergence(s32 error_x_cm,
                                              s32 error_y_cm,
                                              u8 position_updated)
{
    s32 max_error_cm;

    if (position_updated == 0 ||
        (horizontal_control.last_vel_x == 0 &&
         horizontal_control.last_vel_y == 0))
    {
        return;
    }

    max_error_cm = HorizontalControl_MaxAbsError(error_x_cm, error_y_cm);

    if (horizontal_control.divergence_check_ms == 0)
    {
        horizontal_control.divergence_reference_error_cm = max_error_cm;
    }

    horizontal_control.divergence_check_ms += HORIZONTAL_HOLD_PERIOD_MS;

    if (horizontal_control.divergence_check_ms >=
        HORIZONTAL_HOLD_DIVERGENCE_CHECK_MS)
    {
        if (max_error_cm >
            horizontal_control.divergence_reference_error_cm +
            HORIZONTAL_HOLD_DIVERGENCE_GROWTH_CM)
        {
            HorizontalControl_LatchFault(
                HORIZONTAL_HOLD_FAULT_DIVERGENCE);
            return;
        }

        horizontal_control.divergence_reference_error_cm = max_error_cm;
        horizontal_control.divergence_check_ms = 0;
    }
}

void HorizontalControl_Update(void)
{
    s32 forward_error_cm;
    s32 left_error_cm;
    s16 target_vel_x;
    s16 target_vel_y;
    u8 position_updated = 0;

    if (fc_sta.fc_mode_sta != 2 || slam_position_valid == 0)
    {
        HorizontalControl_StopOutput();
        return;
    }

    if (horizontal_control.fault_code != HORIZONTAL_HOLD_FAULT_NONE)
    {
        HorizontalControl_StopOutput();
        return;
    }

    if (horizontal_control.last_update_cnt != slam_position_update_cnt)
    {
        horizontal_control.last_update_cnt = slam_position_update_cnt;
        horizontal_control.stale_ms = 0;
        position_updated = 1;
    }
    else if (horizontal_control.stale_ms <
             HORIZONTAL_HOLD_SENSOR_TIMEOUT_MS + HORIZONTAL_HOLD_PERIOD_MS)
    {
        horizontal_control.stale_ms += HORIZONTAL_HOLD_PERIOD_MS;
    }

    if (horizontal_control.stale_ms > HORIZONTAL_HOLD_SENSOR_TIMEOUT_MS)
    {
        horizontal_control.initialized = 0;
        HorizontalControl_StopOutput();
        return;
    }

    if (horizontal_control.initialized == 0)
    {
        HorizontalControl_CaptureTarget();
        HorizontalControl_StopOutput();
        return;
    }

    forward_error_cm = (s32)hold_target_x - (s32)now_x;
    left_error_cm = (s32)hold_target_y - (s32)now_y;

    /* 首次闭环测试中，误差越界后锁死输出，必须CH6回中复位。 */
    if (HorizontalControl_AbsS32(forward_error_cm) >
            HORIZONTAL_HOLD_MAX_ERROR_CM ||
        HorizontalControl_AbsS32(left_error_cm) >
            HORIZONTAL_HOLD_MAX_ERROR_CM)
    {
        HorizontalControl_LatchFault(HORIZONTAL_HOLD_FAULT_MAX_ERROR);
        return;
    }

    if (HorizontalControl_AbsS32(forward_error_cm) <=
        HORIZONTAL_HOLD_DEADBAND_CM)
    {
        target_vel_x = 0;
        horizontal_control.last_error_x_cm = forward_error_cm;
    }
    else
    {
        target_vel_x =
            HorizontalControl_CalculateVelocity(
                forward_error_cm,
                &horizontal_control.last_error_x_cm,
                HORIZONTAL_HOLD_KP_X,
                HORIZONTAL_HOLD_KD_X);
    }

    if (HorizontalControl_AbsS32(left_error_cm) <=
        HORIZONTAL_HOLD_DEADBAND_CM)
    {
        target_vel_y = 0;
        horizontal_control.last_error_y_cm = left_error_cm;
    }
    else
    {
        target_vel_y =
            HorizontalControl_CalculateVelocity(
                left_error_cm,
                &horizontal_control.last_error_y_cm,
                HORIZONTAL_HOLD_KP_Y,
                HORIZONTAL_HOLD_KD_Y);
    }

    horizontal_control.last_vel_x =
        HorizontalControl_ApplySlewLimit(target_vel_x,
                                         horizontal_control.last_vel_x);
    horizontal_control.last_vel_y =
        HorizontalControl_ApplySlewLimit(target_vel_y,
                                         horizontal_control.last_vel_y);

    rt_tar.st_data.vel_x = horizontal_control.last_vel_x;
    rt_tar.st_data.vel_y = horizontal_control.last_vel_y;

    HorizontalControl_CheckDivergence(forward_error_cm,
                                      left_error_cm,
                                      position_updated);
}

s16 HorizontalControl_GetTargetX(void)
{
    return hold_target_x;
}

s16 HorizontalControl_GetTargetY(void)
{
    return hold_target_y;
}

s16 HorizontalControl_GetErrorX(void)
{
    return HorizontalControl_S32ToS16((s32)hold_target_x - (s32)now_x);
}

s16 HorizontalControl_GetErrorY(void)
{
    return HorizontalControl_S32ToS16((s32)hold_target_y - (s32)now_y);
}

s16 HorizontalControl_GetOutputVelX(void)
{
    return horizontal_control.last_vel_x;
}

s16 HorizontalControl_GetOutputVelY(void)
{
    return horizontal_control.last_vel_y;
}

u8 HorizontalControl_GetFaultCode(void)
{
    return horizontal_control.fault_code;
}

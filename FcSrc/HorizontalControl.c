#include "HorizontalControl.h"
#include "ANO_LX.h"
#include "Drv_Sys.h"
#include "LX_FC_EXT_Sensor.h"
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
#define HORIZONTAL_HOLD_DEADBAND_CM            5       /* 定点保持误差小于5cm时进入死区 */
#define HORIZONTAL_HOLD_MIN_VEL_CMPS           5       /* 死区外最小有效速度(cm/s) */
#define HORIZONTAL_HOLD_MAX_VEL_CMPS           15      /* 位置环输出最大速度(cm/s) */
#define HORIZONTAL_HOLD_MAX_VEL_STEP_CMPS      4       /* 单周期速度增量限幅(cm/s) */
#define HORIZONTAL_HOLD_SENSOR_TIMEOUT_MS      300U    /* 传感器超时时间(ms) */
#define SLAM_READY_CONFIRM_MS                  3000U   /* 连续稳定后才允许使用SLAM */
#define SLAM_READY_MAX_STEP_CM                 20      /* 初始化/飞行期间单帧最大跳变 */
#define SLAM_READY_MAX_DRIFT_CM                10      /* 初始化窗口内允许的坐标漂移 */

#define HORIZONTAL_HOLD_FAULT_NONE              0U
#define HORIZONTAL_HOLD_FAULT_SENSOR_TIMEOUT    1U

s16 now_x = 0;
s16 now_y = 0;

static s16 hold_target_x = 0;
static s16 hold_target_y = 0;
static u8 slam_position_update_cnt = 0;
static u8 slam_position_valid = 0;
static u32 slam_last_update_ms = 0;
static u8 slam_ready_candidate_valid = 0;
static s16 slam_candidate_origin_x_cm = 0;
static s16 slam_candidate_origin_y_cm = 0;
static s16 slam_candidate_last_x_cm = 0;
static s16 slam_candidate_last_y_cm = 0;
static u32 slam_candidate_start_ms = 0;
static u32 slam_candidate_last_ms = 0;

typedef struct
{
    u8 initialized;
    u8 last_update_cnt;
    u16 stale_ms;
    u8 fault_code;
    s16 last_vel_x;
    s16 last_vel_y;
    s32 last_error_x_cm;
    s32 last_error_y_cm;
} HorizontalControlState;

static HorizontalControlState horizontal_control;

static s32 HorizontalControl_AbsS32(s32 value)
{
    return (value >= 0) ? value : -value;
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
    else if (velocity_cmps > 0.0f &&
             velocity_cmps < HORIZONTAL_HOLD_MIN_VEL_CMPS)
    {
        velocity_cmps = HORIZONTAL_HOLD_MIN_VEL_CMPS;
    }
    else if (velocity_cmps < 0.0f &&
             velocity_cmps > -HORIZONTAL_HOLD_MIN_VEL_CMPS)
    {
        velocity_cmps = -HORIZONTAL_HOLD_MIN_VEL_CMPS;
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

static void HorizontalControl_BeginSlamQualification(s16 x_cm,
                                                     s16 y_cm,
                                                     u32 now_ms)
{
    slam_position_valid = 0;
    slam_ready_candidate_valid = 1;
    slam_candidate_origin_x_cm = x_cm;
    slam_candidate_origin_y_cm = y_cm;
    slam_candidate_last_x_cm = x_cm;
    slam_candidate_last_y_cm = y_cm;
    slam_candidate_start_ms = now_ms;
    slam_candidate_last_ms = now_ms;
}

void HorizontalControl_SetPosition(s16 x_cm, s16 y_cm)
{
    u32 now_ms = GetSysRunTimeMs();
    u32 sample_gap_ms;
    s32 step_x_cm;
    s32 step_y_cm;

    now_x = x_cm;
    now_y = y_cm;

    if (slam_ready_candidate_valid == 0)
    {
        HorizontalControl_BeginSlamQualification(x_cm, y_cm, now_ms);
        return;
    }

    sample_gap_ms = (u32)(now_ms - slam_candidate_last_ms);
    step_x_cm = (s32)x_cm - (s32)slam_candidate_last_x_cm;
    step_y_cm = (s32)y_cm - (s32)slam_candidate_last_y_cm;

    if (sample_gap_ms > HORIZONTAL_HOLD_SENSOR_TIMEOUT_MS ||
        HorizontalControl_AbsS32(step_x_cm) > SLAM_READY_MAX_STEP_CM ||
        HorizontalControl_AbsS32(step_y_cm) > SLAM_READY_MAX_STEP_CM)
    {
        /*
         * During initialization this restarts the three-second readiness
         * window. If SLAM was already active, clearing valid makes the
         * running horizontal controller enter its sensor-fault path.
         */
        HorizontalControl_BeginSlamQualification(x_cm, y_cm, now_ms);
        return;
    }

    if (slam_position_valid == 0 &&
        (HorizontalControl_AbsS32(
             (s32)x_cm - (s32)slam_candidate_origin_x_cm) >
             SLAM_READY_MAX_DRIFT_CM ||
         HorizontalControl_AbsS32(
             (s32)y_cm - (s32)slam_candidate_origin_y_cm) >
             SLAM_READY_MAX_DRIFT_CM))
    {
        /*
         * The aircraft is stationary before takeoff. Slow SLAM convergence
         * or drift outside this box therefore restarts readiness timing.
         */
        HorizontalControl_BeginSlamQualification(x_cm, y_cm, now_ms);
        return;
    }

    slam_candidate_last_x_cm = x_cm;
    slam_candidate_last_y_cm = y_cm;
    slam_candidate_last_ms = now_ms;

    if (slam_position_valid == 0)
    {
        if ((u32)(now_ms - slam_candidate_start_ms) <
            SLAM_READY_CONFIRM_MS)
        {
            return;
        }

        slam_position_valid = 1;
    }

    slam_position_update_cnt++;
    slam_last_update_ms = now_ms;
    LX_FC_EXT_Sensor_SetSlamPosition(x_cm, y_cm);
}

u8 HorizontalControl_HasValidPosition(void)
{
    if (slam_position_valid == 0)
    {
        return 0;
    }

    return ((u32)(GetSysRunTimeMs() - slam_last_update_ms) <=
            HORIZONTAL_HOLD_SENSOR_TIMEOUT_MS);
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
    horizontal_control.fault_code = HORIZONTAL_HOLD_FAULT_NONE;
    horizontal_control.last_error_x_cm = 0;
    horizontal_control.last_error_y_cm = 0;
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
        horizontal_control.last_error_x_cm = 0;
        horizontal_control.last_error_y_cm = 0;
    }
}

u8 HorizontalControl_SetTarget(s16 target_x_cm, s16 target_y_cm)
{
    if (slam_position_valid == 0 ||
        horizontal_control.initialized == 0 ||
        horizontal_control.fault_code != HORIZONTAL_HOLD_FAULT_NONE)
    {
        return 0;
    }

    hold_target_x = target_x_cm;
    hold_target_y = target_y_cm;

    /*
     * 目标阶跃时同步微分历史值，避免产生微分冲击。
     * P项和速度斜坡限制仍会平滑地建立前飞速度。
     */
    horizontal_control.last_error_x_cm =
        (s32)hold_target_x - (s32)now_x;
    horizontal_control.last_error_y_cm =
        (s32)hold_target_y - (s32)now_y;

    return 1;
}

static void HorizontalControl_LatchFault(u8 fault_code)
{
    horizontal_control.fault_code = fault_code;
    HorizontalControl_StopOutput();
}

void HorizontalControl_Update(void)
{
    s32 forward_error_cm;
    s32 left_error_cm;
    s16 target_vel_x;
    s16 target_vel_y;

    if (fc_sta.fc_mode_sta != 2)
    {
        HorizontalControl_StopOutput();
        return;
    }

    if (slam_position_valid == 0)
    {
        if (horizontal_control.initialized != 0)
        {
            HorizontalControl_LatchFault(
                HORIZONTAL_HOLD_FAULT_SENSOR_TIMEOUT);
        }
        else
        {
            HorizontalControl_StopOutput();
        }
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
    }
    else if (horizontal_control.stale_ms <
             HORIZONTAL_HOLD_SENSOR_TIMEOUT_MS + HORIZONTAL_HOLD_PERIOD_MS)
    {
        horizontal_control.stale_ms += HORIZONTAL_HOLD_PERIOD_MS;
    }

    if (horizontal_control.stale_ms > HORIZONTAL_HOLD_SENSOR_TIMEOUT_MS)
    {
        HorizontalControl_LatchFault(
            HORIZONTAL_HOLD_FAULT_SENSOR_TIMEOUT);
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

u8 HorizontalControl_TargetReached(s16 tolerance_cm)
{
    s32 error_x_cm;
    s32 error_y_cm;

    if (tolerance_cm < 0)
    {
        tolerance_cm = -tolerance_cm;
    }

    if (slam_position_valid == 0 ||
        horizontal_control.initialized == 0 ||
        horizontal_control.fault_code != HORIZONTAL_HOLD_FAULT_NONE)
    {
        return 0;
    }

    error_x_cm = (s32)hold_target_x - (s32)now_x;
    error_y_cm = (s32)hold_target_y - (s32)now_y;

    return (HorizontalControl_AbsS32(error_x_cm) <= tolerance_cm &&
            HorizontalControl_AbsS32(error_y_cm) <= tolerance_cm);
}

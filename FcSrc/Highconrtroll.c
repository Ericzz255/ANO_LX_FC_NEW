#include "Highcontroll.h"
#include "ANO_LX.h"
#include "Ano_Math.h"
#include "Drv_AnoOf.h"

/*
 * 高度外环：激光高度误差(cm) -> 垂直速度目标(cm/s)。
 * 凌霄飞控内部继续完成垂直速度内环，因此这里不直接控制油门。
 *
 * 调参建议：
 *   1. 先只调 HEIGHT_HOLD_KP，过冲明显就减小；
 *   2. 存在长期静差时再缓慢增加 HEIGHT_HOLD_KI；
 *   3. 首次试飞建议 HEIGHT_HOLD_MAX_VEL_CMPS 不超过 25cm/s。
 */
#define HEIGHT_HOLD_PERIOD_S           0.02f
#define HEIGHT_HOLD_KP                 0.80f
#define HEIGHT_HOLD_KI                 0.06f
#define HEIGHT_HOLD_DEADBAND_CM        3.0f
#define HEIGHT_HOLD_MAX_VEL_CMPS       25.0f
#define HEIGHT_HOLD_INTEGRAL_LIMIT     80.0f
#define HEIGHT_HOLD_FILTER_ALPHA       0.35f
#define HEIGHT_HOLD_SENSOR_TIMEOUT_MS  300U
#define HEIGHT_HOLD_MIN_VALID_CM       5U
#define HEIGHT_HOLD_MAX_VALID_CM       500U

typedef struct
{
    float integral;
    float filtered_alt_cm;
    u8 last_alt_update_cnt;
    u16 stale_ms;
    u8 initialized;
} HeightControlState;

static HeightControlState height_control;

void HeightControl_Reset(void)
{
    height_control.integral = 0.0f;
    height_control.filtered_alt_cm = 0.0f;
    height_control.last_alt_update_cnt = ano_of.alt_update_cnt;
    height_control.stale_ms = 0;
    height_control.initialized = 0;
    rt_tar.st_data.vel_z = 0;
}

void HeightControl_Update(float target_alt_cm)
{
    float measured_alt_cm;
    float error_cm;
    float vel_cmd_cmps;

    if (height_control.last_alt_update_cnt != ano_of.alt_update_cnt)
    {
        height_control.last_alt_update_cnt = ano_of.alt_update_cnt;
        height_control.stale_ms = 0;
        measured_alt_cm = (float)ano_of.of_alt_cm;

        if (ano_of.of_alt_cm < HEIGHT_HOLD_MIN_VALID_CM ||
            ano_of.of_alt_cm > HEIGHT_HOLD_MAX_VALID_CM)
        {
            height_control.integral = 0.0f;
            height_control.initialized = 0;
            rt_tar.st_data.vel_z = 0;
            return;
        }

        if (height_control.initialized == 0)
        {
            height_control.filtered_alt_cm = measured_alt_cm;
            height_control.initialized = 1;
        }
        else
        {
            height_control.filtered_alt_cm +=
                HEIGHT_HOLD_FILTER_ALPHA *
                (measured_alt_cm - height_control.filtered_alt_cm);
        }
    }
    else if (height_control.stale_ms < HEIGHT_HOLD_SENSOR_TIMEOUT_MS + 20U)
    {
        height_control.stale_ms += 20U;
    }

    if (height_control.initialized == 0 ||
        height_control.stale_ms > HEIGHT_HOLD_SENSOR_TIMEOUT_MS)
    {
        height_control.integral = 0.0f;
        rt_tar.st_data.vel_z = 0;
        return;
    }

    error_cm = target_alt_cm - height_control.filtered_alt_cm;

    /*
     * 进入死区后主动清零，避免沿用上一次爬升/下降速度。
     * 同时清积分，防止退出死区时因残余积分产生跳变。
     */
    if (ABS(error_cm) <= HEIGHT_HOLD_DEADBAND_CM)
    {
        height_control.integral = 0.0f;
        rt_tar.st_data.vel_z = 0;
        return;
    }

    height_control.integral += error_cm * HEIGHT_HOLD_PERIOD_S;
    height_control.integral = LIMIT(height_control.integral,
                                    -HEIGHT_HOLD_INTEGRAL_LIMIT,
                                    HEIGHT_HOLD_INTEGRAL_LIMIT);

    vel_cmd_cmps = HEIGHT_HOLD_KP * error_cm +
                   HEIGHT_HOLD_KI * height_control.integral;
    vel_cmd_cmps = LIMIT(vel_cmd_cmps,
                         -HEIGHT_HOLD_MAX_VEL_CMPS,
                         HEIGHT_HOLD_MAX_VEL_CMPS);

    /* 正值上升、负值下降。 */
    if (vel_cmd_cmps >= 0.0f)
    {
        rt_tar.st_data.vel_z = (s16)(vel_cmd_cmps + 0.5f);
    }
    else
    {
        rt_tar.st_data.vel_z = (s16)(vel_cmd_cmps - 0.5f);
    }
}

u8 HeightControl_TargetReached(float target_alt_cm, float tolerance_cm)
{
    if (tolerance_cm < 0.0f)
    {
        tolerance_cm = -tolerance_cm;
    }

    if (height_control.initialized == 0 ||
        height_control.stale_ms > HEIGHT_HOLD_SENSOR_TIMEOUT_MS)
    {
        return 0;
    }

    return (ABS(target_alt_cm - height_control.filtered_alt_cm) <=
            tolerance_cm);
}

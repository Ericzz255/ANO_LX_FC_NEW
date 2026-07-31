#include "User_Task.h"
#include "Drv_RcIn.h"
#include "LX_FC_Fun.h"
#include "LX_FC_State.h"
#include "Highcontroll.h"
#include "HorizontalControl.h"
#include "CarPoseXyUart.h"
#include "Drv_PwmOut.h"

#define MISSION_HEIGHT_CM               80U
#define TAKEOFF_STABILIZE_MS            3000U
#define USER_TASK_PERIOD_MS             20U
#define HEIGHT_TOLERANCE_CM             5.0f
#define RETURN_POSITION_TOLERANCE_CM     5
#define RETURN_POSITION_STABLE_MS        1000U
#define CAR_START_FORWARD_OFFSET_MM      500L
#define CAR_START_RIGHT_OFFSET_MM        375L
#define CAR_LAP_DEPARTURE_RADIUS_MM       1000UL
#define CAR_LAP_RETURN_RADIUS_MM          300UL
#define LAP_FINISH_POSITION_TOLERANCE_CM  10

/*
 * Bench-test mission skeleton for the D problem:
 * 0 idle
 * 1 capture aircraft/car origins, then enter programmable mode
 * 2 wait for RC unlock
 * 3 wait after unlock
 * 4 take off to the contest cruise height
 * 5 hold at 80 cm for three continuous seconds
 * 6 follow the vehicle until it completes one full lap
 * 7 finish the frozen final follow target
 * 8 release the payload and return to absolute SLAM coordinate (0, 0)
 * 9 land after holding the return point for one continuous second
 *
 * CH6 is only a bench trigger; the contest start command will later come
 * from the vehicle over the wireless link.
 */
static u8 mission_step = 0;
static u8 car_target_active = 0U;
static u8 car_origin_captured = 0U;
static u8 return_target_set = 0U;
static u16 return_stable_ms = 0U;
static u8 car_lap_departed = 0U;
static u8 car_lap_complete = 0U;
static u32 last_car_update_count = 0U;
static s16 mission_origin_x_cm = 0;
static s16 mission_origin_y_cm = 0;
static s32 car_origin_x_mm = 0L;
static s32 car_origin_y_mm = 0L;

static s16 UserTask_MmToCm(s32 value_mm)
{
    s32 value_cm;

    if (value_mm >= 0)
    {
        value_cm = (value_mm + 5L) / 10L;
    }
    else
    {
        value_cm = (value_mm - 5L) / 10L;
    }

    if (value_cm > 32767L)
    {
        value_cm = 32767L;
    }
    else if (value_cm < -32768L)
    {
        value_cm = -32768L;
    }

    return (s16)value_cm;
}

static s16 UserTask_ClampToS16(s32 value)
{
    if (value > 32767L)
    {
        return 32767;
    }
    if (value < -32768L)
    {
        return (s16)-32768;
    }
    return (s16)value;
}

static u32 UserTask_AbsS32ToU32(s32 value)
{
    return (value >= 0) ? (u32)value : (u32)(-value);
}

/* Max + 3/8 min is a low-cost approximation of sqrt(x*x + y*y). */
static u32 UserTask_ApproxDistanceMm(s32 delta_x_mm, s32 delta_y_mm)
{
    u32 abs_x_mm = UserTask_AbsS32ToU32(delta_x_mm);
    u32 abs_y_mm = UserTask_AbsS32ToU32(delta_y_mm);
    u32 maximum_mm;
    u32 minimum_mm;

    if (abs_x_mm >= abs_y_mm)
    {
        maximum_mm = abs_x_mm;
        minimum_mm = abs_y_mm;
    }
    else
    {
        maximum_mm = abs_y_mm;
        minimum_mm = abs_x_mm;
    }

    return maximum_mm + (minimum_mm * 3U) / 8U;
}

static void UserTask_UpdateCarLap(const car_pose_xy_t *pose)
{
    u32 origin_distance_mm;

    origin_distance_mm = UserTask_ApproxDistanceMm(
        pose->x_mm - car_origin_x_mm,
        pose->y_mm - car_origin_y_mm);

    if (origin_distance_mm >= CAR_LAP_DEPARTURE_RADIUS_MM)
    {
        car_lap_departed = 1U;
    }

    if (car_lap_departed != 0U &&
        origin_distance_mm <= CAR_LAP_RETURN_RADIUS_MM)
    {
        car_lap_complete = 1U;
    }
}

static u8 UserTask_CaptureCarOrigin(void)
{
    car_pose_xy_t pose;

    if (car_origin_captured != 0U)
    {
        return SET;
    }

    if (CarPoseXyUart_GetPose(&pose) == RESET)
    {
        return RESET;
    }

    car_origin_x_mm = pose.x_mm;
    car_origin_y_mm = pose.y_mm;
    car_origin_captured = 1U;
    return SET;
}

/*
 * Use only the vehicle's field-frame X/Y. Yaw and velocity are deliberately
 * ignored. When the 150 ms validity window expires, capture the aircraft's
 * current SLAM position once so an old vehicle coordinate cannot keep pulling
 * the aircraft.
 */
static u8 UserTask_UpdateCarTarget(void)
{
    car_pose_xy_t pose;
    s32 relative_x_mm;
    s32 relative_y_mm;
    s32 target_x_cm;
    s32 target_y_cm;

    if (car_origin_captured == 0U ||
        CarPoseXyUart_GetPose(&pose) == RESET)
    {
        if (car_target_active != 0U)
        {
            HorizontalControl_CaptureTarget();
            car_target_active = 0U;
        }
        return RESET;
    }

    if (car_target_active != 0U &&
        pose.update_count == last_car_update_count)
    {
        return SET;
    }

    /*
     * At mission start the vehicle is 50 cm forward and 37.5 cm right
     * of the aircraft. In the shared X-forward/Y-left convention:
     * vehicle_start = aircraft_start + (+500 mm, -375 mm).
     */
    relative_x_mm =
        pose.x_mm - car_origin_x_mm +
        CAR_START_FORWARD_OFFSET_MM;
    relative_y_mm =
        pose.y_mm - car_origin_y_mm -
        CAR_START_RIGHT_OFFSET_MM;
    target_x_cm =
        (s32)mission_origin_x_cm +
        (s32)UserTask_MmToCm(relative_x_mm);
    target_y_cm =
        (s32)mission_origin_y_cm +
        (s32)UserTask_MmToCm(relative_y_mm);

    if (HorizontalControl_SetTarget(
            UserTask_ClampToS16(target_x_cm),
            UserTask_ClampToS16(target_y_cm)) == 0U)
    {
        return RESET;
    }

    UserTask_UpdateCarLap(&pose);
    last_car_update_count = pose.update_count;
    car_target_active = 1U;
    return SET;
}

static void UserTask_ResetMission(void)
{
    mission_step = 0;
    car_target_active = 0U;
    car_origin_captured = 0U;
    return_target_set = 0U;
    return_stable_ms = 0U;
    car_lap_departed = 0U;
    car_lap_complete = 0U;
    last_car_update_count = 0U;
    mission_origin_x_cm = 0;
    mission_origin_y_cm = 0;
    car_origin_x_mm = 0L;
    car_origin_y_mm = 0L;
    HorizontalControl_Reset();
    HeightControl_Reset();
}

static void UserTask_EnterLanding(void)
{
    mission_step = 9;
    car_target_active = 0U;
    return_target_set = 0U;
    return_stable_ms = 0U;
    HorizontalControl_StopOutput();
    HeightControl_Reset();
}

u8 UserTask_GetMissionStep(void)
{
    return mission_step;
}

void UserTask_OneKeyCmd(void)
{
    static u8 land_command_sent = 0;
    static u16 state_timer_ms = 0;
    u16 ch6 = rc_in.rc_ch.st_data.ch_[ch_6_aux2];

    if (rc_in.fail_safe != 0)
    {
        UserTask_ResetMission();
        land_command_sent = 0;
        state_timer_ms = 0;
        return;
    }

    /* CH6 low: manual landing for bench safety. */
    if (ch6 > 800 && ch6 < 1200)
    {
        HorizontalControl_StopOutput();
        HeightControl_Reset();
        if (land_command_sent == 0)
        {
            land_command_sent = OneKey_Land();
        }
        mission_step = 0;
        state_timer_ms = 0;
        return;
    }

    /* CH6 high runs the temporary bench-test mission. */
    if (ch6 <= 1800 || ch6 >= 2200)
    {
        UserTask_ResetMission();
        land_command_sent = 0;
        state_timer_ms = 0;
        return;
    }

    if (mission_step == 0)
    {
        UserTask_ResetMission();
        DrvDropMagnetSet(1U);
        mission_step = 1;
        land_command_sent = 0;
        state_timer_ms = 0;
    }

    switch (mission_step)
    {
    case 1:
        if (UserTask_CaptureCarOrigin() != RESET &&
            HorizontalControl_HasValidPosition() &&
            LX_Change_Mode(2))
        {
            HorizontalControl_Reset();
            HorizontalControl_CaptureTarget();
            mission_origin_x_cm = now_x;
            mission_origin_y_cm = now_y;
            mission_step = 2;
        }
        break;

    case 2:
        /* Unlock authority belongs to the RC; never send an unlock command. */
        if (fc_sta.unlock_sta != 0)
        {
            state_timer_ms = 0;
            mission_step = 3;
        }
        break;

    case 3:
        state_timer_ms += USER_TASK_PERIOD_MS;
        if (state_timer_ms >= 2000U)
        {
            state_timer_ms = 0;
            mission_step = 4;
        }
        break;

    case 4:
        if (OneKey_Takeoff(MISSION_HEIGHT_CM) != 0U)
        {
            mission_step = 5;
        }
        break;

    case 5:
        HeightControl_Update((float)MISSION_HEIGHT_CM);
        HorizontalControl_Update();

        if (HorizontalControl_GetFaultCode() != 0)
        {
            UserTask_EnterLanding();
        }
        else if (HeightControl_TargetReached((float)MISSION_HEIGHT_CM,
                                             HEIGHT_TOLERANCE_CM))
        {
            if (state_timer_ms < TAKEOFF_STABILIZE_MS)
            {
                state_timer_ms += USER_TASK_PERIOD_MS;
            }

            if (state_timer_ms >= TAKEOFF_STABILIZE_MS &&
                UserTask_UpdateCarTarget() != RESET)
            {
                state_timer_ms = 0;
                mission_step = 6;
            }
        }
        else
        {
            state_timer_ms = 0;
        }
        break;

    case 6:
        HeightControl_Update((float)MISSION_HEIGHT_CM);
        UserTask_UpdateCarTarget();
        HorizontalControl_Update();
        if (HorizontalControl_GetFaultCode() != 0)
        {
            UserTask_EnterLanding();
        }
        else if (car_lap_complete != 0U)
        {
            car_target_active = 0U;
            mission_step = 7;
        }
        break;

    case 7:
        HeightControl_Update((float)MISSION_HEIGHT_CM);
        HorizontalControl_Update();
        if (HorizontalControl_GetFaultCode() != 0)
        {
            UserTask_EnterLanding();
        }
        else if (HorizontalControl_TargetReached(
                     LAP_FINISH_POSITION_TOLERANCE_CM))
        {
            return_target_set = 0U;
            return_stable_ms = 0U;
            mission_step = 8;
        }
        break;

    case 8:
        DrvDropMagnetSet(0U);

        HeightControl_Update((float)MISSION_HEIGHT_CM);

        if (return_target_set == 0U)
        {
            if (HorizontalControl_SetTarget(0, 0) == 0U)
            {
                UserTask_EnterLanding();
                break;
            }

            return_target_set = 1U;
            return_stable_ms = 0U;
        }

        HorizontalControl_Update();
        if (HorizontalControl_GetFaultCode() != 0)
        {
            UserTask_EnterLanding();
        }
        else if (HorizontalControl_TargetReached(
                     RETURN_POSITION_TOLERANCE_CM))
        {
            if (return_stable_ms < RETURN_POSITION_STABLE_MS)
            {
                return_stable_ms += USER_TASK_PERIOD_MS;
            }

            if (return_stable_ms >= RETURN_POSITION_STABLE_MS)
            {
                UserTask_EnterLanding();
            }
        }
        else
        {
            return_stable_ms = 0U;
        }
        break;

    case 9:
        HorizontalControl_StopOutput();
        HeightControl_Reset();
        if (land_command_sent == 0)
        {
            land_command_sent = OneKey_Land();
        }
        break;

    default:
        UserTask_ResetMission();
        state_timer_ms = 0;
        land_command_sent = 0;
        break;
    }
}

#include "User_Task.h"
#include "Drv_RcIn.h"
#include "LX_FC_Fun.h"
#include "LX_FC_State.h"
#include "Highcontroll.h"
#include "HorizontalControl.h"
#include "Drv_AnoOf.h"
#include "Drv_PwmOut.h"
#include "CarPoseXyUart.h"

#define MISSION_HEIGHT_CM               75U
#define TAKEOFF_STABILIZE_MS            3000U
#define USER_TASK_PERIOD_MS             20U
#define HEIGHT_TOLERANCE_CM             5.0f
#define RETURN_POSITION_TOLERANCE_CM     5
#define RETURN_POSITION_STABLE_MS        3000U
#define MANUAL_HOLD_MIN_HEIGHT_CM         5U
#define MANUAL_HOLD_MAX_HEIGHT_CM         500U
#define BLIND_WAYPOINT_TOLERANCE_CM       13
#define BLIND_WAYPOINT_TIMEOUT_MS          20000UL
#define BLIND_LAP_TIMEOUT_MS              120000UL
#define BLIND_RETURN_TIMEOUT_MS            30000UL

/*
 * Relative geofence around the mapped route. Coordinates use the aircraft
 * SLAM convention: X points from A to B and positive Y points left. The
 * outbound route and return span X=0..312.5 cm and Y=-187.5..0 cm relative
 * to takeoff.
 */
#define BLIND_GEOFENCE_MIN_X_CM           (-20L)
#define BLIND_GEOFENCE_MAX_X_CM            345L
#define BLIND_GEOFENCE_MIN_Y_CM          (-220L)
#define BLIND_GEOFENCE_MAX_Y_CM             30L

typedef struct
{
    s16 x_cm;
    s16 y_cm;
} blind_waypoint_t;

/*
 * The first target is B; A is deliberately skipped. The remaining points
 * sample the upper 75 cm-radius turn through C and continue to D.
 * Half-centimetre map coordinates are rounded to integer centimetres.
 */
static const blind_waypoint_t blind_waypoints[] =
{
    { 238,  -38}, /* B */
    { 275,  -48},
    { 302,  -75},
    { 313, -113},
    { 302, -150},
    { 275, -177},
    { 238, -188}, /* C */
    {  88, -188}  /* D, start return */
};

#define BLIND_WAYPOINT_COUNT \
    ((u8)(sizeof(blind_waypoints) / sizeof(blind_waypoints[0])))

/*
 * Blind-flight mission using only the aircraft's own SLAM X/Y:
 * 0 idle
 * 1 capture aircraft SLAM origin, then enter programmable mode
 * 2 wait for RC unlock
 * 3 wait after unlock
 * 4 take off to the contest cruise height
 * 5 hold at 75 cm for three continuous seconds
 * 6 fly directly to B, follow the upper turn through C, and finish at D
 * 7 release the payload, return to absolute (0, 0), and hold three seconds
 * 8 land
 *
 * CH6 remains the safety enable. A fresh vehicle TAKEOFF_FLAG=1 starts the
 * mission once; vehicle position never participates in flight control.
 */
static u8 mission_step = 0;
static u8 return_target_set = 0U;
static u16 return_stable_ms = 0U;
static u8 manual_hold_requested = 0U;
static u8 manual_hold_active = 0U;
static float manual_hold_height_cm = 0.0f;
static u8 blind_waypoint_index = 0U;
static u8 blind_waypoint_target_set = 0U;
static u32 blind_waypoint_elapsed_ms = 0UL;
static u32 blind_lap_elapsed_ms = 0UL;
static u32 return_elapsed_ms = 0UL;
static s16 mission_origin_x_cm = 0;
static s16 mission_origin_y_cm = 0;

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

static u8 UserTask_SetBlindWaypoint(u8 index)
{
    s32 target_x_cm;
    s32 target_y_cm;

    if (index >= BLIND_WAYPOINT_COUNT)
    {
        return RESET;
    }

    target_x_cm = (s32)mission_origin_x_cm +
                  (s32)blind_waypoints[index].x_cm;
    target_y_cm = (s32)mission_origin_y_cm +
                  (s32)blind_waypoints[index].y_cm;

    if (HorizontalControl_SetTarget(
            UserTask_ClampToS16(target_x_cm),
            UserTask_ClampToS16(target_y_cm)) == 0U)
    {
        return RESET;
    }

    blind_waypoint_target_set = 1U;
    blind_waypoint_elapsed_ms = 0UL;
    return SET;
}

static u8 UserTask_IsInsideBlindGeofence(void)
{
    s32 relative_x_cm = (s32)now_x - (s32)mission_origin_x_cm;
    s32 relative_y_cm = (s32)now_y - (s32)mission_origin_y_cm;

    return (relative_x_cm >= BLIND_GEOFENCE_MIN_X_CM &&
            relative_x_cm <= BLIND_GEOFENCE_MAX_X_CM &&
            relative_y_cm >= BLIND_GEOFENCE_MIN_Y_CM &&
            relative_y_cm <= BLIND_GEOFENCE_MAX_Y_CM) ? SET : RESET;
}

static void UserTask_ResetMission(void)
{
    mission_step = 0;
    return_target_set = 0U;
    return_stable_ms = 0U;
    manual_hold_requested = 0U;
    manual_hold_active = 0U;
    manual_hold_height_cm = 0.0f;
    blind_waypoint_index = 0U;
    blind_waypoint_target_set = 0U;
    blind_waypoint_elapsed_ms = 0UL;
    blind_lap_elapsed_ms = 0UL;
    return_elapsed_ms = 0UL;
    mission_origin_x_cm = 0;
    mission_origin_y_cm = 0;
    HorizontalControl_Reset();
    HeightControl_Reset();
}

static void UserTask_EnterLanding(void)
{
    mission_step = 8;
    blind_waypoint_target_set = 0U;
    return_target_set = 0U;
    return_stable_ms = 0U;
    return_elapsed_ms = 0UL;
    manual_hold_requested = 0U;
    manual_hold_active = 0U;
    manual_hold_height_cm = 0.0f;
    HorizontalControl_StopOutput();
    HeightControl_Reset();
}

static void UserTask_ManualHold(void)
{
    if (manual_hold_requested == 0U)
    {
        UserTask_ResetMission();
        manual_hold_requested = 1U;
    }

    mission_step = 10U;

    if (fc_sta.fc_mode_sta != 2U)
    {
        HorizontalControl_StopOutput();
        HeightControl_Reset();
        LX_Change_Mode(2U);
        return;
    }

    if (manual_hold_active == 0U)
    {
        if (HorizontalControl_HasValidPosition() == 0U ||
            AnoOF_AltitudeIsValid() == 0U ||
            ano_of.of_alt_cm < MANUAL_HOLD_MIN_HEIGHT_CM ||
            ano_of.of_alt_cm > MANUAL_HOLD_MAX_HEIGHT_CM)
        {
            HorizontalControl_StopOutput();
            HeightControl_Reset();
            return;
        }

        HorizontalControl_Reset();
        HorizontalControl_CaptureTarget();
        HeightControl_Reset();
        manual_hold_height_cm = (float)ano_of.of_alt_cm;
        manual_hold_active = 1U;
    }

    HeightControl_Update(manual_hold_height_cm);

    if (HorizontalControl_GetFaultCode() != 0U)
    {
        if (HorizontalControl_HasValidPosition() == 0U)
        {
            HorizontalControl_StopOutput();
            return;
        }

        HorizontalControl_Reset();
        HorizontalControl_CaptureTarget();
    }

    HorizontalControl_Update();
}

u8 UserTask_GetMissionStep(void)
{
    return mission_step;
}

u8 UserTask_GetBlindWaypointIndex(void)
{
    return blind_waypoint_index;
}

void UserTask_OneKeyCmd(void)
{
    static u8 land_command_sent = 0;
    static u16 state_timer_ms = 0;
    static u8 car_takeoff_start_armed = 1U;
    u16 ch6 = rc_in.rc_ch.st_data.ch_[ch_6_aux2];
    u8 car_takeoff_flag = CarPoseXyUart_GetTakeoffFlag();

    /* A new start requires the vehicle flag to return to zero first. */
    if (car_takeoff_flag == RESET)
    {
        car_takeoff_start_armed = 1U;
    }

    if (rc_in.fail_safe != 0)
    {
        car_takeoff_start_armed = 0U;
        UserTask_ResetMission();
        land_command_sent = 0;
        state_timer_ms = 0;
        return;
    }

    /* CH6 low: capture and hold the current position and altitude. */
    if (ch6 > 800 && ch6 < 1200)
    {
        land_command_sent = 0;
        state_timer_ms = 0;
        UserTask_ManualHold();
        return;
    }

    /* CH6 middle: capture and hold the current position and altitude. */
    if (ch6 >= 1200 && ch6 <= 1800)
    {
        land_command_sent = 0;
        state_timer_ms = 0;
        UserTask_ManualHold();
        return;
    }

    /* CH6 high enables, but does not itself start, the automatic mission. */
    if (ch6 <= 1800 || ch6 >= 2200)
    {
        UserTask_ResetMission();
        land_command_sent = 0;
        state_timer_ms = 0;
        return;
    }

    if (manual_hold_requested != 0U)
    {
        UserTask_ResetMission();
        land_command_sent = 0;
        state_timer_ms = 0;
    }

    if (mission_step == 0)
    {
        if (car_takeoff_flag == RESET ||
            car_takeoff_start_armed == 0U)
        {
            return;
        }

        car_takeoff_start_armed = 0U;
        UserTask_ResetMission();
        DrvDropMagnetSet(1U);
        mission_step = 1;
        land_command_sent = 0;
        state_timer_ms = 0;
    }

    switch (mission_step)
    {
    case 1:
        if (HorizontalControl_HasValidPosition() &&
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

            if (state_timer_ms >= TAKEOFF_STABILIZE_MS)
            {
                state_timer_ms = 0;
                blind_waypoint_index = 0U;
                blind_waypoint_target_set = 0U;
                blind_waypoint_elapsed_ms = 0UL;
                blind_lap_elapsed_ms = 0UL;

                if (UserTask_SetBlindWaypoint(
                        blind_waypoint_index) != RESET)
                {
                    mission_step = 6;
                }
                else
                {
                    UserTask_EnterLanding();
                }
            }
        }
        else
        {
            state_timer_ms = 0;
        }
        break;

    case 6:
        HeightControl_Update((float)MISSION_HEIGHT_CM);

        if (UserTask_IsInsideBlindGeofence() == RESET)
        {
            UserTask_EnterLanding();
            break;
        }

        if (blind_waypoint_target_set == 0U &&
            UserTask_SetBlindWaypoint(blind_waypoint_index) == RESET)
        {
            UserTask_EnterLanding();
            break;
        }

        HorizontalControl_Update();
        if (HorizontalControl_GetFaultCode() != 0U)
        {
            UserTask_EnterLanding();
            break;
        }

        blind_waypoint_elapsed_ms += USER_TASK_PERIOD_MS;
        blind_lap_elapsed_ms += USER_TASK_PERIOD_MS;

        if (blind_waypoint_elapsed_ms > BLIND_WAYPOINT_TIMEOUT_MS ||
            blind_lap_elapsed_ms > BLIND_LAP_TIMEOUT_MS)
        {
            UserTask_EnterLanding();
            break;
        }

        if (HorizontalControl_TargetReached(
                BLIND_WAYPOINT_TOLERANCE_CM))
        {
            if ((u8)(blind_waypoint_index + 1U) <
                BLIND_WAYPOINT_COUNT)
            {
                blind_waypoint_index++;
                blind_waypoint_target_set = 0U;
            }
            else
            {
                return_target_set = 0U;
                return_stable_ms = 0U;
                return_elapsed_ms = 0UL;
                mission_step = 7;
            }
        }
        break;

    case 7:
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
            return_elapsed_ms = 0UL;
        }

        HorizontalControl_Update();
        return_elapsed_ms += USER_TASK_PERIOD_MS;

        if (HorizontalControl_GetFaultCode() != 0U ||
            UserTask_IsInsideBlindGeofence() == RESET ||
            return_elapsed_ms > BLIND_RETURN_TIMEOUT_MS)
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

    case 8:
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

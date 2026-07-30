#include "User_Task.h"
#include "Drv_RcIn.h"
#include "LX_FC_Fun.h"
#include "LX_FC_State.h"
#include "Highcontroll.h"
#include "HorizontalControl.h"
#include "VisionFollowControl.h"
#include "MaixCam.h"
#include "Drv_PwmOut.h"

#define MISSION_HEIGHT_CM               50U
#define TAKEOFF_STABILIZE_MS            3000U
#define TARGET_CENTER_HOLD_MS           3500U
#define USER_TASK_PERIOD_MS             20U
#define HEIGHT_TOLERANCE_CM             5.0f
#define POSITION_TOLERANCE_CM           5
#define FORWARD_OFFSET_X_CM             200

/*
 * Bench-test mission skeleton for the D problem:
 * 0 idle
 * 1 wait for valid position and enter programmable mode
 * 2 wait for RC unlock
 * 3 wait after unlock
 * 4 take off to the contest cruise height
 * 5 hold at 50 cm for three continuous seconds
 * 6 move forward 200 cm (X positive) and search for the target
 * 8 release the payload
 * 9 land
 * 10 keep over the target; land after 3.5 s continuously centered
 *
 * CH6 is only a bench trigger; the contest start command will later come
 * from the vehicle over the wireless link.
 */
static u8 mission_step = 0;
static s16 mission_origin_x_cm = 0;
static s16 mission_origin_y_cm = 0;
static u8 visual_hold_initialized = 0;
static u16 target_center_timer_ms = 0;

static void UserTask_ResetMission(void)
{
    mission_step = 0;
    visual_hold_initialized = 0U;
    target_center_timer_ms = 0U;
    MaixCam_SetMode(MAIXCAM_MODE_IDLE);
    VisionFollowControl_Reset();
    HorizontalControl_Reset();
    HeightControl_Reset();
}

static void UserTask_EnterLanding(void)
{
    mission_step = 9;
    visual_hold_initialized = 0U;
    target_center_timer_ms = 0U;
    MaixCam_SetMode(MAIXCAM_MODE_IDLE);
    VisionFollowControl_Reset();
    HorizontalControl_StopOutput();
    HeightControl_Reset();
}

static u8 UserTask_TryEnterVisualFollow(void)
{
    maixcam_tracking_t tracking;

    if (MaixCam_GetTracking(&tracking) == RESET)
    {
        return RESET;
    }

    VisionFollowControl_Begin();
    visual_hold_initialized = 0U;
    target_center_timer_ms = 0U;
    mission_step = 10;
    return SET;
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
        MaixCam_SetMode(MAIXCAM_MODE_IDLE);
        VisionFollowControl_Reset();
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
            MaixCam_SetMode(MAIXCAM_MODE_TRACKING);
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
            state_timer_ms += USER_TASK_PERIOD_MS;
            if (state_timer_ms >= TAKEOFF_STABILIZE_MS &&
                HorizontalControl_SetTarget(
                    (s16)(mission_origin_x_cm + FORWARD_OFFSET_X_CM),
                    mission_origin_y_cm))
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
        if (UserTask_TryEnterVisualFollow() != RESET)
        {
            VisionFollowControl_Update();
            break;
        }

        HorizontalControl_Update();
        if (HorizontalControl_GetFaultCode() != 0)
        {
            UserTask_EnterLanding();
        }
        else if (HorizontalControl_TargetReached(POSITION_TOLERANCE_CM))
        {
            mission_step = 8;
        }
        break;

    case 8:
        DrvDropMagnetSet(0U);
        UserTask_EnterLanding();
        break;

    case 9:
        HorizontalControl_StopOutput();
        HeightControl_Reset();
        if (land_command_sent == 0)
        {
            land_command_sent = OneKey_Land();
        }
        break;

    case 10:
        HeightControl_Update((float)MISSION_HEIGHT_CM);
        if (VisionFollowControl_Update() != RESET)
        {
            visual_hold_initialized = 0U;
            if (VisionFollowControl_IsTargetCentered() != RESET)
            {
                target_center_timer_ms += USER_TASK_PERIOD_MS;
                if (target_center_timer_ms >= TARGET_CENTER_HOLD_MS)
                {
                    UserTask_EnterLanding();
                }
            }
            else
            {
                target_center_timer_ms = 0U;
            }
        }
        else
        {
            target_center_timer_ms = 0U;
            if (visual_hold_initialized == 0U)
            {
                HorizontalControl_Reset();
                HorizontalControl_CaptureTarget();
                visual_hold_initialized = 1U;
            }

            HorizontalControl_Update();
            if (HorizontalControl_GetFaultCode() != 0)
            {
                UserTask_EnterLanding();
            }
        }
        break;

    default:
        UserTask_ResetMission();
        state_timer_ms = 0;
        land_command_sent = 0;
        break;
    }
}

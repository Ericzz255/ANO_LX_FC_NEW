#include "User_Task.h"
#include "Ano_Scheduler.h"
#include "Drv_RcIn.h"
#include "LX_FC_Fun.h"
#include "LX_FC_State.h"
#include "Highcontroll.h"
#include "HorizontalControl.h"
#include "Path_Planning.h"

#define MISSION_HEIGHT_CM               50U
#define HEIGHT_HOLD_START_DELAY_MS      3000U
#define TAKEOFF_STABILIZE_MS            4000U
#define WAYPOINT_TOLERANCE_CM           5
#define WAYPOINT_STABLE_MS              500U
#define WAYPOINT_TIMEOUT_MS             15000U
#define USER_TASK_PERIOD_MS             20U

static u8 mission_step = 0;
static u8 mission_status = 0;
static u16 waypoint_index = 0;
static u8 waypoint_target_set = 0;

static void UserTask_ResetMissionState(void)
{
    mission_step = 0;
    mission_status = 0;
    waypoint_index = 0;
    waypoint_target_set = 0;
    HorizontalControl_Reset();
    HeightControl_Reset();
}

static void UserTask_EnterLanding(u8 status)
{
    mission_status = status;
    mission_step = 9;
    waypoint_target_set = 0;
    HorizontalControl_StopOutput();
    HeightControl_Reset();
}

u8 UserTask_GetMissionStep(void)
{
    return mission_step;
}

u8 UserTask_GetMissionStatus(void)
{
    return mission_status;
}

u16 UserTask_GetWaypointIndex(void)
{
    return waypoint_index;
}

u16 UserTask_GetPathLength(void)
{
    if (final_path_length <= 0)
    {
        return 0;
    }
    return (u16)final_path_length;
}

void UserTask_OneKeyCmd(void)
{
    static u8 one_key_land_f = 1;
    static u8 one_key_mission_f = 0;
    static u8 mission_land_f = 0;
    static u16 delay_cnt_ms = 0;
    static u16 waypoint_stable_ms = 0;
    static u16 waypoint_timeout_ms = 0;

    if (rc_in.fail_safe != 0)
    {
        UserTask_ResetMissionState();
        one_key_mission_f = 0;
        mission_land_f = 0;
        delay_cnt_ms = 0;
        waypoint_stable_ms = 0;
        waypoint_timeout_ms = 0;
        return;
    }

    if (rc_in.rc_ch.st_data.ch_[ch_6_aux2] > 800 &&
        rc_in.rc_ch.st_data.ch_[ch_6_aux2] < 1200)
    {
        if (one_key_land_f == 0)
        {
            one_key_land_f = OneKey_Land();
        }
    }
    else
    {
        one_key_land_f = 0;
    }

    if (rc_in.rc_ch.st_data.ch_[ch_6_aux2] > 1800 &&
        rc_in.rc_ch.st_data.ch_[ch_6_aux2] < 2200)
    {
        if (one_key_mission_f == 0)
        {
            one_key_mission_f = 1;
            mission_step = 1;
            mission_status = 0;
            waypoint_index = 0;
            waypoint_target_set = 0;
            mission_land_f = 0;
            delay_cnt_ms = 0;
            waypoint_stable_ms = 0;
            waypoint_timeout_ms = 0;
            HorizontalControl_Reset();
            HeightControl_Reset();
        }
    }
    else
    {
        one_key_mission_f = 0;
    }

    if (one_key_mission_f == 0)
    {
        UserTask_ResetMissionState();
        mission_land_f = 0;
        delay_cnt_ms = 0;
        waypoint_stable_ms = 0;
        waypoint_timeout_ms = 0;
        return;
    }

    switch (mission_step)
    {
    case 0:
        break;

    case 1:
        mission_step += LX_Change_Mode(2);
        break;

    case 2:
        if (fc_sta.fc_mode_sta != 2)
        {
            LX_Change_Mode(2);
        }
        else if (GS_Barrier_Received() &&
                 HorizontalControl_HasValidPosition())
        {
            mission_step = 3;
        }
        break;

    case 3:
        if (fc_sta.fc_mode_sta != 2)
        {
            mission_step = 1;
            break;
        }

        if (!HorizontalControl_HasValidPosition())
        {
            mission_status = 6;
            break;
        }

        if (run_path_planner())
        {
            mission_status = 1;
            waypoint_index = 0;
            waypoint_target_set = 0;
            HorizontalControl_Reset();
            HorizontalControl_CaptureTarget();
            mission_step = 4;
        }
        else
        {
            mission_status = 2;
        }
        break;

    case 4:
        if (fc_sta.fc_mode_sta != 2)
        {
            mission_step = 1;
        }
        else if (!HorizontalControl_HasValidPosition())
        {
            mission_status = 6;
            mission_step = 2;
        }
        else
        {
            mission_step += FC_Unlock();
        }
        break;

    case 5:
        delay_cnt_ms += USER_TASK_PERIOD_MS;
        if (delay_cnt_ms >= 2000U)
        {
            delay_cnt_ms = 0;
            mission_step = 6;
        }
        break;

    case 6:
        mission_step += OneKey_Takeoff(MISSION_HEIGHT_CM);
        break;

    case 7:
        if (fc_sta.fc_mode_sta != 2)
        {
            UserTask_EnterLanding(5);
            break;
        }

        delay_cnt_ms += USER_TASK_PERIOD_MS;
        if (delay_cnt_ms >= HEIGHT_HOLD_START_DELAY_MS)
        {
            HeightControl_Update((float)MISSION_HEIGHT_CM);
            HorizontalControl_Update();
        }
        else
        {
            HeightControl_Reset();
            HorizontalControl_StopOutput();
        }

        if (HorizontalControl_GetFaultCode() != 0)
        {
            UserTask_EnterLanding(4);
        }
        else if (delay_cnt_ms >= TAKEOFF_STABILIZE_MS)
        {
            delay_cnt_ms = 0;
            waypoint_index = 0;
            waypoint_target_set = 0;
            waypoint_stable_ms = 0;
            waypoint_timeout_ms = 0;
            mission_step = 8;
        }
        break;

    case 8:
    {
        s16 target_x_cm;
        s16 target_y_cm;

        if (fc_sta.fc_mode_sta != 2)
        {
            UserTask_EnterLanding(5);
            break;
        }

        if (HorizontalControl_GetFaultCode() != 0)
        {
            UserTask_EnterLanding(4);
            break;
        }

        if (waypoint_index >= (u16)final_path_length)
        {
            UserTask_EnterLanding(7);
            break;
        }

        if (waypoint_target_set == 0)
        {
            if (!PathPlanner_PointToSlam(final_path[waypoint_index],
                                         &target_x_cm,
                                         &target_y_cm) ||
                !HorizontalControl_SetTarget(target_x_cm, target_y_cm))
            {
                UserTask_EnterLanding(4);
                break;
            }
            waypoint_target_set = 1;
            waypoint_stable_ms = 0;
            waypoint_timeout_ms = 0;
        }

        HeightControl_Update((float)MISSION_HEIGHT_CM);
        HorizontalControl_Update();

        if (HorizontalControl_GetFaultCode() != 0)
        {
            UserTask_EnterLanding(4);
            break;
        }

        if (waypoint_timeout_ms < WAYPOINT_TIMEOUT_MS)
        {
            waypoint_timeout_ms += USER_TASK_PERIOD_MS;
        }
        if (waypoint_timeout_ms >= WAYPOINT_TIMEOUT_MS)
        {
            UserTask_EnterLanding(3);
            break;
        }

        if (HorizontalControl_TargetReached(WAYPOINT_TOLERANCE_CM))
        {
            if (waypoint_stable_ms < WAYPOINT_STABLE_MS)
            {
                waypoint_stable_ms += USER_TASK_PERIOD_MS;
            }

            if (waypoint_stable_ms >= WAYPOINT_STABLE_MS)
            {
                send_step_feedback((int)waypoint_index);
                waypoint_index++;
                waypoint_target_set = 0;
                waypoint_stable_ms = 0;
                waypoint_timeout_ms = 0;

                if (waypoint_index >= (u16)final_path_length)
                {
                    UserTask_EnterLanding(7);
                }
            }
        }
        else
        {
            waypoint_stable_ms = 0;
        }
    }
    break;

    case 9:
        HorizontalControl_StopOutput();
        HeightControl_Reset();
        if (mission_land_f == 0)
        {
            mission_land_f = OneKey_Land();
        }
        break;

    default:
        UserTask_ResetMissionState();
        delay_cnt_ms = 0;
        waypoint_stable_ms = 0;
        waypoint_timeout_ms = 0;
        mission_land_f = 0;
        break;
    }
}

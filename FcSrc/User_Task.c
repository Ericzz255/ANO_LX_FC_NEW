#include "User_Task.h"
#include "Drv_RcIn.h"
#include "LX_FC_Fun.h"
#include "Highcontroll.h"
#include "HorizontalControl.h"
#include "Path_Planning.h"

#define MISSION_HEIGHT_CM               50U
#define TAKEOFF_STABILIZE_MS            3000U
#define WAYPOINT_TOLERANCE_CM           5
#define WAYPOINT_STABLE_MS              500U
#define USER_TASK_PERIOD_MS             20U

/*
 * 0 idle                                   空闲
 * 1 send mode 2 command, wait for SLAM and plan the fixed map   发送模式2,等待SLAM并规划固定地图
 * 2 unlock                                 解锁
 * 3 wait after unlock                      解锁后等待
 * 4 take off                               起飞
 * 5 stabilize                              稳定悬停
 * 6 traverse waypoints                     遍历航点
 * 7 land                                   降落
 */
static u8 mission_step = 0;
static u16 waypoint_index = 0;
static u8 waypoint_target_loaded = 0;

static void UserTask_ResetMission(void)
{
    mission_step = 0;
    waypoint_index = 0;
    waypoint_target_loaded = 0;
    HorizontalControl_Reset();
    HeightControl_Reset();
}

static void UserTask_EnterLanding(void)
{
    mission_step = 7;
    waypoint_target_loaded = 0;
    HorizontalControl_StopOutput();
    HeightControl_Reset();
}

u8 UserTask_GetMissionStep(void)
{
    return mission_step;
}

u16 UserTask_GetWaypointIndex(void)
{
    return waypoint_index;
}

u16 UserTask_GetPathLength(void)
{
    return (final_path_length > 0) ? (u16)final_path_length : 0;
}

void UserTask_OneKeyCmd(void)
{
    static u8 land_command_sent = 0;
    static u16 state_timer_ms = 0;
    static u16 waypoint_stable_ms = 0;
    u16 ch6 = rc_in.rc_ch.st_data.ch_[ch_6_aux2];

    if (rc_in.fail_safe != 0)
    {
        UserTask_ResetMission();
        land_command_sent = 0;
        state_timer_ms = 0;
        waypoint_stable_ms = 0;
        return;
    }

    /* CH6 low: manual landing. */
    if (ch6 > 800 && ch6 < 1200)
    {
        HorizontalControl_StopOutput();
        HeightControl_Reset();
        if (land_command_sent == 0)
        {
            land_command_sent = OneKey_Land();
        }
        mission_step = 0;
        waypoint_index = 0;
        waypoint_target_loaded = 0;
        state_timer_ms = 0;
        waypoint_stable_ms = 0;
        return;
    }

    /* Only CH6 high runs the automatic mission. */
    if (ch6 <= 1800 || ch6 >= 2200)
    {
        UserTask_ResetMission();
        land_command_sent = 0;
        state_timer_ms = 0;
        waypoint_stable_ms = 0;
        return;
    }

    if (mission_step == 0)
    {
        UserTask_ResetMission();
        mission_step = 1;
        land_command_sent = 0;
        state_timer_ms = 0;
        waypoint_stable_ms = 0;
    }

    switch (mission_step)
    {
    case 1:
        if (HorizontalControl_HasValidPosition() &&
            LX_Change_Mode(2))
        {
            /*
             * The fixed map is deterministic.  Planning is run once here;
             * its return value no longer blocks unlock.
             */
            run_path_planner();
            HorizontalControl_Reset();
            HorizontalControl_CaptureTarget();
            mission_step = 2;
        }
        break;

    case 2:
        mission_step += FC_Unlock();
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
        mission_step += OneKey_Takeoff(MISSION_HEIGHT_CM);
        break;

    case 5:
        state_timer_ms += USER_TASK_PERIOD_MS;
        HeightControl_Update((float)MISSION_HEIGHT_CM);
        HorizontalControl_Update();

        /* The remaining horizontal fault is SLAM data timeout. */
        if (HorizontalControl_GetFaultCode() != 0)
        {
            UserTask_EnterLanding();
        }
        else if (state_timer_ms >= TAKEOFF_STABILIZE_MS)
        {
            state_timer_ms = 0;
            waypoint_index = 0;
            waypoint_target_loaded = 0;
            waypoint_stable_ms = 0;
            mission_step = 6;
        }
        break;

    case 6:
    {
        s16 target_x_cm;
        s16 target_y_cm;

        if (HorizontalControl_GetFaultCode() != 0)
        {
            UserTask_EnterLanding();
            break;
        }

        if (waypoint_index >= (u16)final_path_length)
        {
            UserTask_EnterLanding();
            break;
        }

        if (waypoint_target_loaded == 0)
        {
            if (PathPlanner_PointToSlam(final_path[waypoint_index],
                                        &target_x_cm,
                                        &target_y_cm) &&
                HorizontalControl_SetTarget(target_x_cm, target_y_cm))
            {
                waypoint_target_loaded = 1;
                waypoint_stable_ms = 0;
            }
        }

        HeightControl_Update((float)MISSION_HEIGHT_CM);
        HorizontalControl_Update();

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
                waypoint_target_loaded = 0;
                waypoint_stable_ms = 0;
            }
        }
        else
        {
            waypoint_stable_ms = 0;
        }
    }
    break;

    case 7:
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
        waypoint_stable_ms = 0;
        land_command_sent = 0;
        break;
    }
}

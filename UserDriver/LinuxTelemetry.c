#include "LinuxTelemetry.h"
#include "ANO_LX.h"
#include "Drv_AnoOf.h"
#include "Drv_RcIn.h"
#include "Drv_Uart.h"
#include "HorizontalControl.h"
#include "LX_FC_State.h"
#include "Path_Planning.h"
#include "User_Task.h"

#define LINUX_TELEMETRY_HEADER_1          0xAAU
#define LINUX_TELEMETRY_HEADER_2          0x55U
#define LINUX_TELEMETRY_TYPE_FLIGHT_STATE 0x01U
#define LINUX_TELEMETRY_PAYLOAD_LENGTH    18U
#define LINUX_TELEMETRY_FRAME_LENGTH      24U

#define LINUX_STATUS_SLAM_VALID           (1U << 0)
#define LINUX_STATUS_HEIGHT_VALID         (1U << 1)
#define LINUX_STATUS_UNLOCKED             (1U << 2)
#define LINUX_STATUS_BARRIERS_READY        (1U << 3)
#define LINUX_STATUS_PATH_READY            (1U << 4)
#define LINUX_STATUS_RC_FAILSAFE           (1U << 5)

#define LINUX_HEIGHT_MIN_VALID_CM          5U
#define LINUX_HEIGHT_MAX_VALID_CM          500U

static void LinuxTelemetry_PutU16(u8 *buffer, u8 *index, u16 value)
{
    buffer[(*index)++] = (u8)(value & 0xFFU);
    buffer[(*index)++] = (u8)((value >> 8) & 0xFFU);
}

static void LinuxTelemetry_PutS16(u8 *buffer, u8 *index, s16 value)
{
    LinuxTelemetry_PutU16(buffer, index, (u16)value);
}

/* CRC16-CCITT-FALSE: init=0xFFFF, poly=0x1021, no reflection/xorout. */
static u16 LinuxTelemetry_Crc16(const u8 *data, u8 length)
{
    u16 crc = 0xFFFFU;
    u8 i;

    while (length-- > 0U)
    {
        crc ^= (u16)(*data++) << 8;
        for (i = 0; i < 8U; i++)
        {
            if ((crc & 0x8000U) != 0U)
            {
                crc = (u16)((crc << 1) ^ 0x1021U);
            }
            else
            {
                crc <<= 1;
            }
        }
    }

    return crc;
}

void LinuxTelemetry_Send(void)
{
    static u8 sequence = 0;
    u8 frame[LINUX_TELEMETRY_FRAME_LENGTH];
    u8 index = 0;
    u8 status = 0;
    u8 mission_step = UserTask_GetMissionStep();
    u16 height_cm;
    u16 waypoint_index = UserTask_GetWaypointIndex();
    u16 path_length = UserTask_GetPathLength();
    u16 waypoint_progress = 0;
    u16 crc;

    if (HorizontalControl_HasValidPosition())
    {
        status |= LINUX_STATUS_SLAM_VALID;
    }
    if (ano_of.link_sta != 0U &&
        ano_of.work_sta != 0U &&
        ano_of.of_alt_cm >= LINUX_HEIGHT_MIN_VALID_CM &&
        ano_of.of_alt_cm <= LINUX_HEIGHT_MAX_VALID_CM)
    {
        status |= LINUX_STATUS_HEIGHT_VALID;
    }
    if (fc_sta.unlock_sta != 0U)
    {
        status |= LINUX_STATUS_UNLOCKED;
    }
    if (PathPlanner_HasBarrierConfiguration())
    {
        status |= LINUX_STATUS_BARRIERS_READY;
    }
    if (path_length > 0U)
    {
        status |= LINUX_STATUS_PATH_READY;
    }
    if (rc_in.fail_safe != 0U)
    {
        status |= LINUX_STATUS_RC_FAILSAFE;
    }

    if (ano_of.of_alt_cm > 65535U)
    {
        height_cm = 65535U;
    }
    else
    {
        height_cm = (u16)ano_of.of_alt_cm;
    }

    /*
     * Send display-oriented progress:
     * 0/N before traversal, 1/N for the first active waypoint, N/N on landing.
     */
    if (path_length > 0U)
    {
        if (mission_step == 6U)
        {
            waypoint_progress =
                (waypoint_index < path_length) ?
                (u16)(waypoint_index + 1U) : path_length;
        }
        else if (mission_step >= 7U)
        {
            waypoint_progress = path_length;
        }
    }

    frame[index++] = LINUX_TELEMETRY_HEADER_1;
    frame[index++] = LINUX_TELEMETRY_HEADER_2;
    frame[index++] = LINUX_TELEMETRY_TYPE_FLIGHT_STATE;
    frame[index++] = LINUX_TELEMETRY_PAYLOAD_LENGTH;
    frame[index++] = sequence++;
    frame[index++] = fc_sta.fc_mode_sta;
    frame[index++] = mission_step;
    frame[index++] = status;
    LinuxTelemetry_PutS16(frame, &index, now_x);
    LinuxTelemetry_PutS16(frame, &index, now_y);
    LinuxTelemetry_PutU16(frame, &index, height_cm);
    LinuxTelemetry_PutS16(frame, &index, fc_vel.st_data.vel_x);
    LinuxTelemetry_PutS16(frame, &index, fc_vel.st_data.vel_y);
    LinuxTelemetry_PutU16(frame, &index, waypoint_progress);
    LinuxTelemetry_PutU16(frame, &index, path_length);

    /* CRC covers TYPE, LENGTH and the complete payload; header is excluded. */
    crc = LinuxTelemetry_Crc16(&frame[2],
                               (u8)(index - 2U));
    LinuxTelemetry_PutU16(frame, &index, crc);

    if (index == LINUX_TELEMETRY_FRAME_LENGTH)
    {
        DrvUart2SendBuf(frame, index);
    }
}

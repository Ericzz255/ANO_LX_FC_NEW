#include "LinuxTelemetry.h"
#include "ANO_LX.h"
#include "Drv_AnoOf.h"
#include "Drv_RcIn.h"
#include "Drv_Uart.h"
#include "CarPoseXyUart.h"
#include "HorizontalControl.h"
#include "LX_FC_State.h"
#include "User_Task.h"

#define LINUX_TELEMETRY_HEADER_1          0xAAU
#define LINUX_TELEMETRY_HEADER_2          0x55U
#define LINUX_TELEMETRY_TYPE_FLIGHT_STATE 0x01U
#define LINUX_TELEMETRY_PAYLOAD_LENGTH    16U
#define LINUX_TELEMETRY_FRAME_LENGTH      22U

#define LINUX_STATUS_POSITION_VALID       (1U << 0)
#define LINUX_STATUS_HEIGHT_VALID         (1U << 1)
#define LINUX_STATUS_UNLOCKED             (1U << 2)
#define LINUX_STATUS_RC_FAILSAFE           (1U << 3)
#define LINUX_STATUS_CAR_POSITION_VALID    (1U << 4)
#define LINUX_STATUS_CAR_LINK_ALIVE        (1U << 5)
#define LINUX_STATUS_TAKEOFF_FLAG          (1U << 6)

static void LinuxTelemetry_PutU16(u8 *buffer, u8 *index, u16 value)
{
    buffer[(*index)++] = (u8)(value & 0xFFU);
    buffer[(*index)++] = (u8)((value >> 8) & 0xFFU);
}

static void LinuxTelemetry_PutS16(u8 *buffer, u8 *index, s16 value)
{
    LinuxTelemetry_PutU16(buffer, index, (u16)value);
}

static s16 LinuxTelemetry_MmToCm(s32 value_mm)
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
        return 32767;
    }
    if (value_cm < -32768L)
    {
        return (s16)-32768;
    }

    return (s16)value_cm;
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
    u16 height_cm;
    s16 car_x_cm = 0;
    s16 car_y_cm = 0;
    car_pose_xy_t car_pose;
    u16 crc;

    if (HorizontalControl_HasValidPosition())
    {
        status |= LINUX_STATUS_POSITION_VALID;
    }

    if (AnoOF_AltitudeIsValid() != 0U)
    {
        status |= LINUX_STATUS_HEIGHT_VALID;
    }

    if (fc_sta.unlock_sta != 0U)
    {
        status |= LINUX_STATUS_UNLOCKED;
    }

    if (rc_in.fail_safe != 0U)
    {
        status |= LINUX_STATUS_RC_FAILSAFE;
    }

    if (CarPoseXyUart_IsLinkAlive() != 0U)
    {
        status |= LINUX_STATUS_CAR_LINK_ALIVE;
    }

    if (CarPoseXyUart_GetTakeoffFlag() != 0U)
    {
        status |= LINUX_STATUS_TAKEOFF_FLAG;
    }

    if (CarPoseXyUart_GetPose(&car_pose) != 0U)
    {
        status |= LINUX_STATUS_CAR_POSITION_VALID;
        car_x_cm = LinuxTelemetry_MmToCm(car_pose.x_mm);
        car_y_cm = LinuxTelemetry_MmToCm(car_pose.y_mm);
    }

    if (ano_of.of_alt_cm > 65535U)
    {
        height_cm = 65535U;
    }
    else
    {
        height_cm = (u16)ano_of.of_alt_cm;
    }

    frame[index++] = LINUX_TELEMETRY_HEADER_1;
    frame[index++] = LINUX_TELEMETRY_HEADER_2;
    frame[index++] = LINUX_TELEMETRY_TYPE_FLIGHT_STATE;
    frame[index++] = LINUX_TELEMETRY_PAYLOAD_LENGTH;
    frame[index++] = sequence++;
    frame[index++] = fc_sta.fc_mode_sta;
    frame[index++] = UserTask_GetMissionStep();
    frame[index++] = status;
    LinuxTelemetry_PutS16(frame, &index, now_x);
    LinuxTelemetry_PutS16(frame, &index, now_y);
    LinuxTelemetry_PutU16(frame, &index, height_cm);
    LinuxTelemetry_PutU16(frame, &index, fc_bat.st_data.voltage_100);
    LinuxTelemetry_PutS16(frame, &index, car_x_cm);
    LinuxTelemetry_PutS16(frame, &index, car_y_cm);

    crc = LinuxTelemetry_Crc16(&frame[2],
                               (u8)(index - 2U));
    LinuxTelemetry_PutU16(frame, &index, crc);

    if (index == LINUX_TELEMETRY_FRAME_LENGTH)
    {
        DrvUart2SendBuf(frame, index);
    }
}

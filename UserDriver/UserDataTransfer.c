#include "UserDataTransfer.h"
#include "ANO_LX.h"
#include "Drv_Uart.h"
#include "HorizontalControl.h"
#include "User_Task.h"
#include "CarPoseXyUart.h"

#define USER_DATA_DEST_ADDR   HW_ALL
#define USER_DATA_BUFFER_SIZE 32U

static u8 user_data_buffer[USER_DATA_BUFFER_SIZE];

static void UserData_PutS16(u8 *buffer, u8 *cnt, s16 value)
{
    buffer[(*cnt)++] = BYTE0(value);
    buffer[(*cnt)++] = BYTE1(value);
}

static void UserDataTransfer_FillPayloadF1(u8 *buffer, u8 *cnt)
{
    /* USERDATA1..2: current position, centimetres. */
    UserData_PutS16(buffer, cnt, now_x);
    UserData_PutS16(buffer, cnt, now_y);
    /* USERDATA3: current blind-flight mission step. */
    UserData_PutS16(buffer, cnt, (s16)UserTask_GetMissionStep());
    /* USERDATA4: 1 after SLAM position initialization while data is fresh. */
    UserData_PutS16(buffer, cnt,
                    HorizontalControl_HasValidPosition() ? 1 : 0);
    /* USERDATA5: current blind-flight waypoint index. */
    UserData_PutS16(buffer, cnt,
                    (s16)UserTask_GetBlindWaypointIndex());
    /* USERDATA6..7: current horizontal target, centimetres. */
    UserData_PutS16(buffer, cnt, HorizontalControl_GetTargetX());
    UserData_PutS16(buffer, cnt, HorizontalControl_GetTargetY());
    /* USERDATA8: current vehicle takeoff flag. */
    UserData_PutS16(buffer, cnt,
                    CarPoseXyUart_GetTakeoffFlag() ? 1 : 0);
}

static void UserDataTransfer_SendFrame(u8 frame_id,
                                       void (*fill_payload)(u8 *, u8 *))
{
    u8 cnt = 0;
    u8 sum_check = 0;
    u8 add_check = 0;
    u8 i;

    user_data_buffer[cnt++] = 0xAA;
    user_data_buffer[cnt++] = USER_DATA_DEST_ADDR;
    user_data_buffer[cnt++] = frame_id;
    user_data_buffer[cnt++] = 0;

    fill_payload(user_data_buffer, &cnt);
    user_data_buffer[3] = cnt - 4U;

    for (i = 0; i < cnt; i++)
    {
        sum_check += user_data_buffer[i];
        add_check += sum_check;
    }

    user_data_buffer[cnt++] = sum_check;
    user_data_buffer[cnt++] = add_check;
    UartSendLXIMU(user_data_buffer, cnt);
}

void UserDataTransfer_Task(void)
{
    UserDataTransfer_SendFrame(0xf1, UserDataTransfer_FillPayloadF1);
}

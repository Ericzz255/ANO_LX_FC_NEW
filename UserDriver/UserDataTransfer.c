#include "UserDataTransfer.h"
#include "ANO_LX.h"
#include "Drv_Uart.h"
#include "HorizontalControl.h"
#include "CarPoseXyUart.h"

#define USER_DATA_DEST_ADDR   HW_ALL
#define USER_DATA_BUFFER_SIZE 32U

static u8 user_data_buffer[USER_DATA_BUFFER_SIZE];

static void UserData_PutS16(u8 *buffer, u8 *cnt, s16 value)
{
    buffer[(*cnt)++] = BYTE0(value);
    buffer[(*cnt)++] = BYTE1(value);
}

static s16 UserData_ClampS32ToS16(s32 value)
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

static void UserDataTransfer_FillPayloadF1(u8 *buffer, u8 *cnt)
{
    car_pose_xy_t car_pose;
    u8 car_pose_valid = CarPoseXyUart_GetPose(&car_pose);

    /* USERDATA1..2: current position, centimetres. */
    UserData_PutS16(buffer, cnt, now_x);
    UserData_PutS16(buffer, cnt, now_y);
    /* USERDATA3: 1 while a fresh car pose is eligible for control. */
    UserData_PutS16(buffer, cnt, car_pose_valid ? 1 : 0);
    /* USERDATA4: 1 after SLAM position initialization while data is fresh. */
    UserData_PutS16(buffer, cnt,
                    HorizontalControl_HasValidPosition() ? 1 : 0);
    /* USERDATA5: 1 while structure- and CRC-valid UART1 frames arrive. */
    UserData_PutS16(buffer, cnt,
                    CarPoseXyUart_IsLinkAlive() ? 1 : 0);
    /* USERDATA6..7: current valid vehicle position, millimetres. */
    UserData_PutS16(buffer, cnt,
                    car_pose_valid ?
                    UserData_ClampS32ToS16(car_pose.x_mm) : 0);
    UserData_PutS16(buffer, cnt,
                    car_pose_valid ?
                    UserData_ClampS32ToS16(car_pose.y_mm) : 0);
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

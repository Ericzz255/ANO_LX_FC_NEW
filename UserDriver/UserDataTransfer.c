#include "UserDataTransfer.h"
#include "ANO_LX.h"
#include "Drv_AnoOf.h"
#include "Drv_RcIn.h"
#include "Drv_Uart.h"
#include "LX_FC_State.h"

#define USER_DATA_DEST_ADDR   HW_ALL
#define USER_DATA_BUFFER_SIZE 32U

static u16 user_target_height_cm = 50U;
static u8 user_data_buffer[USER_DATA_BUFFER_SIZE];

static void UserData_PutU8(u8 *buffer, u8 *cnt, u8 value)
{
    buffer[(*cnt)++] = value;
}

static void UserData_PutU16(u8 *buffer, u8 *cnt, u16 value)
{
    buffer[(*cnt)++] = BYTE0(value);
    buffer[(*cnt)++] = BYTE1(value);
}

static void UserData_PutS16(u8 *buffer, u8 *cnt, s16 value)
{
    buffer[(*cnt)++] = BYTE0(value);
    buffer[(*cnt)++] = BYTE1(value);
}

void UserDataTransfer_SetTargetHeight(u16 target_height_cm)
{
    user_target_height_cm = target_height_cm;
}

static void UserDataTransfer_FillPayload(u8 *buffer, u8 *cnt)
{
    u16 measured_height_cm;
    s16 height_error_cm;

    if (ano_of.of_alt_cm > 65535U)
    {
        measured_height_cm = 65535U;
    }
    else
    {
        measured_height_cm = (u16)ano_of.of_alt_cm;
    }

    height_error_cm = (s16)((s32)user_target_height_cm -
                            (s32)measured_height_cm);

    UserData_PutU16(buffer, cnt, measured_height_cm);
    UserData_PutU16(buffer, cnt, user_target_height_cm);
    UserData_PutS16(buffer, cnt, height_error_cm);
    UserData_PutS16(buffer, cnt, rt_tar.st_data.vel_z);
    UserData_PutS16(buffer, cnt, fc_vel.st_data.vel_z);
    UserData_PutS16(buffer, cnt, ano_of.of1_dx);
    UserData_PutS16(buffer, cnt, ano_of.of1_dy);
    UserData_PutU8(buffer, cnt, ano_of.of_quality);
    UserData_PutU8(buffer, cnt, fc_sta.fc_mode_sta);
    UserData_PutS16(buffer, cnt, rc_in.rc_ch.st_data.ch_[ch_3_thr]);
}

static void UserDataTransfer_SendFrame(u8 frame_id)
{
    u8 cnt = 0;
    u8 sum_check = 0;
    u8 add_check = 0;
    u8 i;

    user_data_buffer[cnt++] = 0xAA;
    user_data_buffer[cnt++] = USER_DATA_DEST_ADDR;
    user_data_buffer[cnt++] = frame_id;
    user_data_buffer[cnt++] = 0;

    UserDataTransfer_FillPayload(user_data_buffer, &cnt);
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
    UserDataTransfer_SendFrame(0xf1);
}

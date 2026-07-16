#include "UserDataTransfer.h"
#include "ANO_LX.h"
#include "Drv_AnoOf.h"
#include "Drv_RcIn.h"
#include "Drv_Uart.h"
#include "LX_FC_State.h"

#define USER_DATA_FRAME_ID      0xF1
#define USER_DATA_DEST_ADDR     HW_ALL
#define USER_DATA_BUFFER_SIZE   40U

static u16 user_target_height_cm = 50U;
static u8 user_data_buffer[USER_DATA_BUFFER_SIZE];

static void UserData_PutU8(u8 *cnt, u8 value)
{
    user_data_buffer[(*cnt)++] = value;
}

static void UserData_PutU16(u8 *cnt, u16 value)
{
    user_data_buffer[(*cnt)++] = BYTE0(value);
    user_data_buffer[(*cnt)++] = BYTE1(value);
}

static void UserData_PutS16(u8 *cnt, s16 value)
{
    user_data_buffer[(*cnt)++] = BYTE0(value);
    user_data_buffer[(*cnt)++] = BYTE1(value);
}

void UserDataTransfer_SetTargetHeight(u16 target_height_cm)
{
    user_target_height_cm = target_height_cm;
}

void UserDataTransfer_Task(void)
{
    u8 cnt = 0;
    u8 sum_check = 0;
    u8 add_check = 0;
    u8 i;
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

    /* 匿名V7帧头：HEAD、目标地址、功能码、数据长度。 */
    user_data_buffer[cnt++] = 0xAA;
    user_data_buffer[cnt++] = USER_DATA_DEST_ADDR;
    user_data_buffer[cnt++] = USER_DATA_FRAME_ID;
    user_data_buffer[cnt++] = 0;

    /* F1数据位置1~7：高度闭环与速度数据。 */
    UserData_PutU16(&cnt, measured_height_cm);
    UserData_PutU16(&cnt, user_target_height_cm);
    UserData_PutS16(&cnt, height_error_cm);
    UserData_PutS16(&cnt, rt_tar.st_data.vel_z);
    UserData_PutS16(&cnt, fc_vel.st_data.vel_z);
    UserData_PutS16(&cnt, ano_of.of1_dx);
    UserData_PutS16(&cnt, ano_of.of1_dy);

    /* F1数据位置8~14：传感器、飞控与遥控状态。 */
    UserData_PutU8(&cnt, ano_of.of_quality);
    UserData_PutU8(&cnt, ano_of.of1_sta);
    UserData_PutU8(&cnt, ano_of.link_sta);
    UserData_PutU8(&cnt, ano_of.work_sta);
    UserData_PutU8(&cnt, fc_sta.fc_mode_sta);
    UserData_PutU8(&cnt, fc_sta.unlock_sta);
    UserData_PutU8(&cnt, rc_in.fail_safe);

    /* F1数据位置15~20：遥控前四通道和数据更新计数。 */
    UserData_PutS16(&cnt, rc_in.rc_ch.st_data.ch_[ch_1_rol]);
    UserData_PutS16(&cnt, rc_in.rc_ch.st_data.ch_[ch_2_pit]);
    UserData_PutS16(&cnt, rc_in.rc_ch.st_data.ch_[ch_3_thr]);
    UserData_PutS16(&cnt, rc_in.rc_ch.st_data.ch_[ch_4_yaw]);
    UserData_PutU8(&cnt, ano_of.alt_update_cnt);
    UserData_PutU8(&cnt, ano_of.of_update_cnt);

    user_data_buffer[3] = cnt - 4U;

    /* 匿名V7双校验：从0xAA累加至DATA区末尾。 */
    for (i = 0; i < cnt; i++)
    {
        sum_check += user_data_buffer[i];
        add_check += sum_check;
    }

    user_data_buffer[cnt++] = sum_check;
    user_data_buffer[cnt++] = add_check;

    /* UART5连接凌霄IMU，由凌霄链路将广播帧输出至匿名上位机。 */
    UartSendLXIMU(user_data_buffer, cnt);
}

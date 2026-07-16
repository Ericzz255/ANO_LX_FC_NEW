#include "UserDataTransfer.h"
#include "ANO_LX.h"
#include "Drv_AnoOf.h"
#include "Drv_RcIn.h"
#include "LX_FC_State.h"

static u16 user_target_height_cm = 50U;

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

void UserDataTransfer_FillPayload(u8 frame_id, u8 *buffer, u8 *cnt)
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

    if (frame_id == 0xf1)
    {
        UserData_PutU16(buffer, cnt, measured_height_cm);
        UserData_PutU16(buffer, cnt, user_target_height_cm);
        UserData_PutS16(buffer, cnt, height_error_cm);
        UserData_PutS16(buffer, cnt, rt_tar.st_data.vel_z);
        UserData_PutS16(buffer, cnt, fc_vel.st_data.vel_z);
        UserData_PutS16(buffer, cnt, ano_of.of1_dx);
        UserData_PutS16(buffer, cnt, ano_of.of1_dy);
        UserData_PutU8(buffer, cnt, ano_of.of_quality);
        UserData_PutU8(buffer, cnt, ano_of.of1_sta);
        UserData_PutU8(buffer, cnt, ano_of.link_sta);
    }
    else if (frame_id == 0xf2)
    {
        UserData_PutU8(buffer, cnt, ano_of.work_sta);
        UserData_PutU8(buffer, cnt, fc_sta.fc_mode_sta);
        UserData_PutU8(buffer, cnt, fc_sta.unlock_sta);
        UserData_PutU8(buffer, cnt, rc_in.fail_safe);
        UserData_PutS16(buffer, cnt, rc_in.rc_ch.st_data.ch_[ch_1_rol]);
        UserData_PutS16(buffer, cnt, rc_in.rc_ch.st_data.ch_[ch_2_pit]);
        UserData_PutS16(buffer, cnt, rc_in.rc_ch.st_data.ch_[ch_3_thr]);
        UserData_PutS16(buffer, cnt, rc_in.rc_ch.st_data.ch_[ch_4_yaw]);
        UserData_PutU8(buffer, cnt, ano_of.alt_update_cnt);
        UserData_PutU8(buffer, cnt, ano_of.of_update_cnt);
    }
}

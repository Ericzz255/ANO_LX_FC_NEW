#include "UserDataTransfer.h"
#include "ANO_LX.h"
#include "Drv_AnoOf.h"
#include "Drv_RcIn.h"
#include "Drv_Uart.h"
#include "HorizontalControl.h"

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
    /* 数据位1：当前SLAM X，机头前方为正。 */
    UserData_PutS16(buffer, cnt, now_x);
    /* 数据位2：当前SLAM Y，机体左侧为正。 */
    UserData_PutS16(buffer, cnt, now_y);
    /* 数据位3、4：起飞前锁定的SLAM目标X、Y。 */
    UserData_PutS16(buffer, cnt, HorizontalControl_GetTargetX());
    UserData_PutS16(buffer, cnt, HorizontalControl_GetTargetY());
    /* 数据位5、6：位置误差，计算方式为目标值减当前值。 */
    UserData_PutS16(buffer, cnt, HorizontalControl_GetErrorX());
    UserData_PutS16(buffer, cnt, HorizontalControl_GetErrorY());
    /* 数据位7、8：实际发送的水平速度目标，单位cm/s。 */
    UserData_PutS16(buffer, cnt, HorizontalControl_GetOutputVelX());
    UserData_PutS16(buffer, cnt, HorizontalControl_GetOutputVelY());
    /*
     * 数据位9：保护状态。
     * 0=正常，1=位置误差越界，2=误差持续发散。
     */
    UserData_PutS16(buffer, cnt,
                    (s16)HorizontalControl_GetFaultCode());
    /* 数据位10：飞控内部估计的X水平速度，单位cm/s。 */
    UserData_PutS16(buffer, cnt, fc_vel.st_data.vel_x);
}

static void UserDataTransfer_FillPayloadF2(u8 *buffer, u8 *cnt)
{
    /* 数据位11：飞控内部估计的Y水平速度，单位cm/s。 */
    UserData_PutS16(buffer, cnt, fc_vel.st_data.vel_y);
    /* 数据位12：光流质量。 */
    UserData_PutS16(buffer, cnt, (s16)ano_of.of_quality);
    /* 数据位13、14：飞控横滚角、俯仰角，单位0.01度。 */
    UserData_PutS16(buffer, cnt, fc_att.st_data.rol_x100);
    UserData_PutS16(buffer, cnt, fc_att.st_data.pit_x100);
    /* 数据位15～18：保留。 */
    UserData_PutS16(buffer, cnt, 0);
    UserData_PutS16(buffer, cnt, 0);
    UserData_PutS16(buffer, cnt, 0);
    UserData_PutS16(buffer, cnt, 0);
    /* 数据位19、20：遥控器CH1、CH2原始通道值。 */
    UserData_PutS16(buffer, cnt,
                    rc_in.rc_ch.st_data.ch_[ch_1_rol]);
    UserData_PutS16(buffer, cnt,
                    rc_in.rc_ch.st_data.ch_[ch_2_pit]);
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
    /*
     * 匿名灵活格式帧每帧最多携带10个数据：
     * F1对应USERDATA_1～10，F2对应USERDATA_11～20。
     */
    UserDataTransfer_SendFrame(0xf1, UserDataTransfer_FillPayloadF1);
    UserDataTransfer_SendFrame(0xf2, UserDataTransfer_FillPayloadF2);
}

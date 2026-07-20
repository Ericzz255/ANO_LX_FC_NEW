#include "UserDataTransfer.h"
#include "ANO_LX.h"
#include "Drv_Uart.h"
#include "Drv_AnoOf.h"
#include "HorizontalControl.h"
#include "Path_Planning.h"

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
    /* 数据位3：飞控内部估计的X水平速度，单位cm/s。 */
    UserData_PutS16(buffer, cnt, fc_vel.st_data.vel_x);
    /* 数据位4：飞控内部估计的Y水平速度，单位cm/s。 */
    UserData_PutS16(buffer, cnt, fc_vel.st_data.vel_y);
    /* USERDATA5: 1 = SLAM position is ready and fresh, 0 = not ready. */
    UserData_PutS16(buffer, cnt,
                    HorizontalControl_HasValidPosition() ? 1 : 0);
    /* USERDATA6: optical-flow image quality, range 0 to 255. */
    UserData_PutS16(buffer, cnt, (s16)ano_of.of_quality);
    /* USERDATA7: MODE1 optical-flow velocity valid flag, 0 or 1. */
    UserData_PutS16(buffer, cnt, (s16)ano_of.of1_sta);
    /*
     * USERDATA8..10: three accepted no-fly cells, encoded as A*10+B.
     * Examples: A1B7 -> 17, A9B1 -> 91.  Zero means that no complete,
     * valid three-cell configuration has been received yet.
     */
    if (PathPlanner_HasBarrierConfiguration())
    {
        u8 i;
        for (i = 0; i < BARRIER_COUNT; i++)
        {
            UserData_PutS16(buffer, cnt,
                            (s16)(barriers[i].col * 10 +
                                  barriers[i].row));
        }
    }
    else
    {
        u8 i;
        for (i = 0; i < BARRIER_COUNT; i++)
        {
            UserData_PutS16(buffer, cnt, 0);
        }
    }
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
    /* F1 sends flight diagnostics and the accepted no-fly configuration. */
    UserDataTransfer_SendFrame(0xf1, UserDataTransfer_FillPayloadF1);
}

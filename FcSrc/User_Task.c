/**
 * @file    User_Task.c
 * @brief   模式2高度闭环测试状态机
 * @details CH6高位启动测试：切换定点模式、解锁、一键起飞，然后使用0x41实时
 *          控制帧的vel_z测试激光高度闭环。当前不执行路径规划和水平移动。
 *
 * @硬件平台  匿名科创凌霄飞控 ANO_LX_FC (STM32F407)
 * @遥控通道  CH5中位: 保持模式2
 *             CH6高位: 启动测试 | CH6中位: 取消/复位 | CH6低位: 一键降落
 */

#include "User_Task.h"
#include "Drv_RcIn.h"
#include "LX_FC_Fun.h"
#include "ANO_LX.h"
#include "LX_FC_State.h"
#include "Highcontroll.h"
#include "UserDataTransfer.h"

#define MISSION_HEIGHT_CM          50U
#define HEIGHT_HOLD_START_DELAY_MS 3000U

/**
 * @brief  当前飞机水平位置（由树莓派 SLAM 通过 USART3 实时更新）
 * @note   单位：厘米。now_x 对应左右方向（飞机左侧为正），
 *         now_y 对应前后方向（机头前方为正）。
 */
s16 now_x = 0;
s16 now_y = 0;

/*============================ 高度闭环测试状态机 ============================*/
/**
 * @brief  模式2高度闭环测试
 * @note   【调用周期】：20ms（由 Ano_Scheduler.c 的 Loop_50Hz 调用）
 *
 *         【测试前要求】：
 *           - CH5必须保持中位，使RC_Data_Task持续选择模式2；
 *           - 遥控器前4通道保持中位，特别是油门通道不能拉低；
 *           - 光流与激光高度数据必须有效。
 *
 *         【CH6触发方式】：
 *           - CH6 低位 (800~1200) : 一键降落（独立逻辑，随时可用）
 *           - CH6 中位 (1200~1800): 取消任务，清零速度，复位所有状态
 *           - CH6 高位 (1800~2200): 启动高度闭环测试
 *
 *         【测试流程】（mission_step状态机）：
 *           case 0 : 空闲/复位状态
 *           case 1 : 切换为定点模式 (LX_Change_Mode(2))
 *           case 2 : 解锁电机 (FC_Unlock())
 *           case 3 : 延时 2s，等待解锁稳定
 *           case 4 : 一键起飞到 MISSION_HEIGHT_CM
 *           case 5 : 等待起飞，3s后高度闭环开始接管
 *           case 6 : 持续高度闭环，保持 MISSION_HEIGHT_CM
 *
 *         【安全保护】：
 *           - CH6 中位：立即清零 vel_x/vel_y/vel_z，重置所有状态
 *           - 失控保护(fail_safe)：遥控器信号丢失，同样清零并复位
 *           - 实际模式不是模式2时：停止实时速度输出
 */
void UserTask_OneKeyCmd(void)
{
    static u8 one_key_land_f = 1;
    static u8 one_key_mission_f = 0;
    static u8 mission_step = 0;
    static u16 delay_cnt_ms = 0;

    UserDataTransfer_SetTargetHeight(MISSION_HEIGHT_CM);

    if (rc_in.fail_safe == 0)
    {
        if (rc_in.rc_ch.st_data.ch_[ch_6_aux2] > 800 && rc_in.rc_ch.st_data.ch_[ch_6_aux2] < 1200)
        {
            if (one_key_land_f == 0)
            {
                one_key_land_f = OneKey_Land();
            }
        }
        else
        {
            one_key_land_f = 0;
        }

        if (rc_in.rc_ch.st_data.ch_[ch_6_aux2] > 1800 && rc_in.rc_ch.st_data.ch_[ch_6_aux2] < 2200)
        {
            if (one_key_mission_f == 0)
            {
                one_key_mission_f = 1;
                mission_step = 1;
                delay_cnt_ms = 0;
            }
        }
        else
        {
            one_key_mission_f = 0;
        }

        if (one_key_mission_f == 1)
        {
            switch (mission_step)
            {
            case 0:
            {
                delay_cnt_ms = 0;
            }
            break;

            case 1:
            {
                mission_step += LX_Change_Mode(2);
            }
            break;

            case 2:
            {
                /* 等待飞控状态帧确认已进入模式2，再发送解锁命令。 */
                if (fc_sta.fc_mode_sta == 2)
                {
                    mission_step += FC_Unlock();
                }
                else
                {
                    LX_Change_Mode(2);
                }
            }
            break;

            case 3:
            {
                delay_cnt_ms += 20;
                if (delay_cnt_ms >= 2000)
                {
                    delay_cnt_ms = 0;
                    mission_step++;
                }
            }
            break;

            case 4:
            {
                mission_step += OneKey_Takeoff(MISSION_HEIGHT_CM);
            }
            break;

            case 5:
            {
                delay_cnt_ms += 20;

                rt_tar.st_data.vel_x = 0;
                rt_tar.st_data.vel_y = 0;

                if (delay_cnt_ms >= HEIGHT_HOLD_START_DELAY_MS &&
                    fc_sta.fc_mode_sta == 2)
                {
                    HeightControl_Update((float)MISSION_HEIGHT_CM);
                }
                else
                {
                    HeightControl_Reset();
                }

                if (delay_cnt_ms >= 4000)
                {
                    delay_cnt_ms = 0;
                    mission_step++;
                }
            }
            break;

            case 6:
            {
                rt_tar.st_data.vel_x = 0;
                rt_tar.st_data.vel_y = 0;

                if (fc_sta.fc_mode_sta == 2)
                {
                    HeightControl_Update((float)MISSION_HEIGHT_CM);
                }
                else
                {
                    HeightControl_Reset();
                }
            }
            break;

            default:
                HeightControl_Reset();
                mission_step = 0;
                break;
            }
        }
        else
        {
            rt_tar.st_data.vel_x = 0;
            rt_tar.st_data.vel_y = 0;
            HeightControl_Reset();

            mission_step = 0;
            delay_cnt_ms = 0;
        }
    }
    else
    {
        rt_tar.st_data.vel_x = 0;
        rt_tar.st_data.vel_y = 0;
        HeightControl_Reset();

        mission_step = 0;
        one_key_mission_f = 0;
        delay_cnt_ms = 0;
    }
}

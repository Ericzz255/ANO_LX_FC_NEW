/**
 * @file    User_Task.c
 * @brief   模式2高度与水平位置闭环测试状态机
 * @details CH6高位启动测试：切换定点模式、解锁、一键起飞，然后使用0x41实时
 *          控制帧的vel_z测试激光高度闭环，并使用SLAM位置生成vel_x/vel_y。
 *
 * @硬件平台  匿名科创凌霄飞控 ANO_LX_FC (STM32F407)
 * @遥控通道  CH5中位: 保持模式2
 *             CH6高位: 启动测试 | CH6中位: 取消/复位 | CH6低位: 一键降落
 */

#include "User_Task.h"
#include "Drv_RcIn.h"
#include "LX_FC_Fun.h"
#include "LX_FC_State.h"
#include "Highcontroll.h"
#include "HorizontalControl.h"

#define MISSION_HEIGHT_CM          50U
#define MISSION_TARGET_X_CM        100
#define MISSION_TARGET_Y_CM        0
#define MISSION_TARGET_TOLERANCE_CM 5
#define MISSION_TARGET_STABLE_MS   1000U
#define HEIGHT_HOLD_START_DELAY_MS 3000U

/*========================= 高度与水平闭环测试状态机 =========================*/
/**
 * @brief  模式2高度与水平位置闭环测试
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
 *           case 5 : 等待起飞，3s后高度与水平闭环开始接管
 *           case 6 : 将水平目标设为SLAM绝对坐标(100,0)
 *           case 7 : 飞向新目标，进入5cm范围并稳定1s
 *           case 8 : 一键降落
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
    static u8 mission_land_f = 0;
    static u8 mission_step = 0;
    static u16 delay_cnt_ms = 0;
    static u16 target_stable_cnt_ms = 0;

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
                target_stable_cnt_ms = 0;
                mission_land_f = 0;
                HorizontalControl_Reset();
                HorizontalControl_CaptureTarget();
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

                if (delay_cnt_ms >= HEIGHT_HOLD_START_DELAY_MS &&
                    fc_sta.fc_mode_sta == 2)
                {
                    HeightControl_Update((float)MISSION_HEIGHT_CM);
                    HorizontalControl_Update();
                }
                else
                {
                    HeightControl_Reset();
                    HorizontalControl_StopOutput();
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
                if (fc_sta.fc_mode_sta == 2)
                {
                    HeightControl_Update((float)MISSION_HEIGHT_CM);
                    if (HorizontalControl_SetTarget(
                            MISSION_TARGET_X_CM,
                            MISSION_TARGET_Y_CM))
                    {
                        target_stable_cnt_ms = 0;
                        mission_step++;
                    }
                    HorizontalControl_Update();
                }
                else
                {
                    HeightControl_Reset();
                    HorizontalControl_Reset();
                }
            }
            break;

            case 7:
            {
                if (fc_sta.fc_mode_sta == 2)
                {
                    HeightControl_Update((float)MISSION_HEIGHT_CM);
                    HorizontalControl_Update();

                    if (HorizontalControl_TargetReached(
                            MISSION_TARGET_TOLERANCE_CM))
                    {
                        if (target_stable_cnt_ms < MISSION_TARGET_STABLE_MS)
                        {
                            target_stable_cnt_ms += 20;
                        }
                        else
                        {
                            target_stable_cnt_ms = 0;
                            mission_step++;
                        }
                    }
                    else
                    {
                        target_stable_cnt_ms = 0;
                    }
                }
                else
                {
                    HeightControl_Reset();
                    HorizontalControl_Reset();
                    target_stable_cnt_ms = 0;
                }
            }
            break;

            case 8:
            {
                HorizontalControl_StopOutput();
                HeightControl_Reset();

                if (mission_land_f == 0)
                {
                    mission_land_f = OneKey_Land();
                }
            }
            break;

            default:
                HeightControl_Reset();
                HorizontalControl_Reset();
                mission_step = 0;
                break;
            }
        }
        else
        {
            HorizontalControl_Reset();
            HeightControl_Reset();

            mission_step = 0;
            delay_cnt_ms = 0;
            target_stable_cnt_ms = 0;
            mission_land_f = 0;
        }
    }
    else
    {
        HorizontalControl_Reset();
        HeightControl_Reset();

        mission_step = 0;
        one_key_mission_f = 0;
        delay_cnt_ms = 0;
        target_stable_cnt_ms = 0;
        mission_land_f = 0;
    }
}

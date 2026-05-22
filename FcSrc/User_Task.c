/**
 * @file    User_Task.c
 * @brief   一键程控任务状态机
 * @details 本文件为 "260506 无人机目标检测" 任务的程控状态机，包含：
 *          1) 水平位置 PID 控制器（当前未使用，保留备用）
 *          2) 一键程控任务状态机：由遥控器 CH6 高位触发，自动完成
 *             起飞 -> 路径规划 -> 逐格移动（含停留检测）-> 降落的完整流程。
 *
 * @author  Eric2195
 * @version 当前版本：Horizontal_Move 固定时间版（2026-05-21）
 *
 * @硬件平台  匿名科创凌霄飞控 ANO_LX_FC (STM32F407)
 * @遥控通道  CH6 高位(>1800): 启动任务  |  中位: 取消/复位  |  低位(<1200): 一键降落
 */

#include "User_Task.h"
#include "Path_Planning.h"
#include "Drv_RcIn.h"
#include "LX_FC_Fun.h"
#include "Ano_Math.h"
#include "ANO_LX.h"
#include "LX_FC_State.h"

// /*============================ PID 参数与函数 ============================*/
// /**
//  * @note PID 部分说明
//  * 以下 y_move_pid / x_move_pid 为水平位置闭环控制器的雏形，
//  * 当前任务流程（UserTask_OneKeyCmd case 7）采用 Horizontal_Move 协议指令
//  * 做固定时间逐格移动，未调用本 PID。
//  * 保留原因：若后续需要基于雷达/光流坐标做实时闭环修正，可直接启用。
//  *
//  * 坐标映射约定（与雷达 SLAM 一致）：
//  *   - Y 方向：机头正前方为正，对应 vel_x（rt_tar.st_data.vel_x）
//  *   - X 方向：飞机左侧为正，对应 vel_y（rt_tar.st_data.vel_y）
//  */

// /* 水平位置 Y 方向（机头前后）PD 参数 */
// #define KP1 0.40f
// #define KD1 0.05f

// /* 水平位置 X 方向（飞机左右）PD 参数 */
// #define KP2 0.35f
// #define KD2 0.08f

// float y_move_pid(s16 cy)
// {
//     static float err_old;
//     float err_d;
//     float pid_out_y;
//     err_d = cy - err_old;
//     err_old = cy;
//     pid_out_y = KP1 * cy + KD1 * err_d;
//     pid_out_y = -LIMIT(pid_out_y, -10, 10);
//     return pid_out_y;
// }

// float x_move_pid(s16 cx)
// {
//     static float err_old;
//     float err_d;
//     float pid_out_x;
//     err_d = cx - err_old;
//     err_old = cx;
//     pid_out_x = KP2 * cx + KD2 * err_d;
//     pid_out_x = -LIMIT(pid_out_x, -10, 10);
//     return pid_out_x;
// }

/**
 * @brief  当前飞机水平位置（由树莓派 SLAM 通过 USART3 实时更新）
 * @note   单位：厘米。now_x 对应左右方向（飞机左侧为正），
 *         now_y 对应前后方向（机头前方为正）。
 */
s16 now_x = 0;
s16 now_y = 0;

/*============================ 一键程控任务状态机 ============================*/
/**
 * @brief  一键程控任务主状态机
 * @note   【调用周期】：20ms（由 Ano_Scheduler.c 的 Loop_50Hz 调用）
 *
 *         【触发方式】（看遥控器 CH6 通道值）：
 *           - CH6 低位 (800~1200) : 一键降落（独立逻辑，随时可用）
 *           - CH6 中位 (1200~1800): 取消任务，清零速度，复位所有状态
 *           - CH6 高位 (1800~2200): 启动任务流程
 *
 *         【任务主流程】（mission_step 状态机）：
 *           case 0 : 空闲/复位状态
 *           case 1 : 切换为程控模式 (LX_Change_Mode(3))
 *           case 2 : 解锁电机 (FC_Unlock())
 *           case 3 : 延时 2s，等待解锁稳定
 *           case 4 : 一键起飞到 50cm (OneKey_Takeoff(50))
 *           case 5 : 悬停稳定 3s
 *           case 6 : 执行路径规划 (run_path_planner())
 *           case 7 : 航点跟踪：逐格移动 + 停留检测（内含子状态机 move_sub_step）
 *           case 8 : 降落 (OneKey_Land())
 *
 *         【case 7 子状态机】（move_sub_step）：
 *           sub_step 0 : 计算下一格方向，发送 Horizontal_Move 指令
 *           sub_step 1 : 等待移动完成（固定 6s）
 *           sub_step 2 : 停留检测 1s（目标检测窗口），完成后 wp_idx++
 *
 *         【安全保护】：
 *           - CH6 中位：立即清零 vel_x/vel_y/vel_z，重置所有状态
 *           - 失控保护(fail_safe)：遥控器信号丢失，同样清零并复位
 */
void UserTask_OneKeyCmd(void)
{
    static u8 one_key_land_f = 1;
    static u8 one_key_mission_f = 0;
    static u8 mission_step = 0;
    static u16 delay_cnt_ms = 0;
    static u16 hover_delay_ms = 0;
    static u8 wp_idx = 0;
    static u8 move_sub_step = 0;
    static u16 move_wait_ms = 0;

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
                hover_delay_ms = 0;
            }
        }
        else
        {
            one_key_mission_f = 0;
        }

        if (one_key_mission_f == 1)
        {
            switch(mission_step)
            {
                case 0:
                {
                    delay_cnt_ms = 0;
                    hover_delay_ms = 0;
                }
                break;

                case 1:
                {
                    mission_step += LX_Change_Mode(3);
                }
                break;

                case 2:
                {
                    mission_step += FC_Unlock();
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
                    mission_step += OneKey_Takeoff(50);
                }
                break;

                case 5:
                {
                    delay_cnt_ms += 20;
                    if (delay_cnt_ms >= 3000)
                    {
                        delay_cnt_ms = 0;
                        mission_step++;
                    }
                }
                break;

                case 6:
                {
                    run_path_planner();
                    if (final_path_length > 0)
                    {
                        mission_step++;
                    }
                    else
                    {
                        mission_step = 8;
                    }
                }
                break;

                case 7:
                {
                    if (wp_idx < final_path_length - 1)
                    {
                        if (move_sub_step == 0)
                        {
                            Point cur = final_path[wp_idx];
                            Point next = final_path[wp_idx + 1];
                            int dr = next.row - cur.row;
                            int dc = next.col - cur.col;

                            u16 angle = 0;
                            if (dr == 1 && dc == 0)       angle = 0;
                            else if (dr == -1 && dc == 0) angle = 180;
                            else if (dr == 0 && dc == 1)  angle = 270;
                            else if (dr == 0 && dc == -1) angle = 90;

                            if (Horizontal_Move(GRID_SIZE_CM, 15, angle))
                            {
                                move_sub_step = 1;
                                move_wait_ms = 0;
                            }
                        }
                        else if (move_sub_step == 1)
                        {
                            move_wait_ms += 20;
                            if (move_wait_ms >= 6000)
                            {
                                move_wait_ms = 0;
                                move_sub_step = 2;
                            }
                        }
                        else if (move_sub_step == 2)
                        {
                            move_wait_ms += 20;
                            if (move_wait_ms >= 1000)
                            {
                                move_wait_ms = 0;
                                move_sub_step = 0;
                                wp_idx++;
                            }
                        }
                    }
                    else
                    {
                        wp_idx = 0;
                        move_sub_step = 0;
                        move_wait_ms = 0;
                        mission_step++;
                    }
                }
                break;

                case 8:
                {
                    mission_step += OneKey_Land();
                }
                break;

                default:
                    break;
            }
        }
        else
        {
            rt_tar.st_data.vel_x = 0;
            rt_tar.st_data.vel_y = 0;
            rt_tar.st_data.vel_z = 0;

            mission_step = 0;
            delay_cnt_ms = 0;
            hover_delay_ms = 0;
            wp_idx = 0;
            move_sub_step = 0;
            move_wait_ms = 0;
        }
    }
    else
    {
        rt_tar.st_data.vel_x = 0;
        rt_tar.st_data.vel_y = 0;
        rt_tar.st_data.vel_z = 0;

        mission_step = 0;
        one_key_mission_f = 0;
        delay_cnt_ms = 0;
        hover_delay_ms = 0;
        wp_idx = 0;
        move_sub_step = 0;
        move_wait_ms = 0;
    }
}

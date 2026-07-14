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
#include "Ano_Scheduler.h"
#include "Usart2.h"
#include "Usart3_Pi.h"

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
 *           case 4 : 等待地面站发送3个禁飞区坐标 (GS_Barrier_Received())
 *           case 5 : 等待地面站按下路径规划按钮 (GS_PlanCmd_Received())
 *           case 6 : 一键起飞到 110cm (OneKey_Takeoff(110))
 *           case 7 : 悬停稳定 4s
 *           case 8 : 航点跟踪：逐格移动 + MaixCam视觉查询 + 反馈（内含子状态机 move_sub_step）
 *           case 9 : 降落 (OneKey_Land())
 *
 *         【case 8 子状态机】（move_sub_step）：
 *           sub_step 0 : 计算下一格方向，发送 Horizontal_Move 指令
 *           sub_step 1 : 等待移动完成（固定 6s），发送查询给MaixCam，等2s接收响应
 *                        收到则转发检测结果给地面端，完成后直接回到 sub_step 0 处理下一格
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
            switch (mission_step)
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
                /* 等待地面站发送3个禁飞区坐标 */
                if (GS_Barrier_Received())
                {
                    mission_step++;
                }
            }
            break;

            case 5:
            {
                /* 等待地面站发送路径规划触发命令(0x55+0xA1+0x65) */
                if (GS_PlanCmd_Received())
                {
                    run_path_planner();
                    if (final_path_length > 0)
                    {
                        mission_step++;
                    }
                    else
                    {
                        mission_step = 9; /* 路径规划失败，跳过起飞直接降落 */
                    }
                }
            }
            break;

            case 6:
            {
                mission_step += OneKey_Takeoff(110);
            }
            break;

            case 7:
            {
                delay_cnt_ms += 20;
                if (delay_cnt_ms >= 4000)
                {
                    delay_cnt_ms = 0;
                    mission_step++;
                }
            }
            break;

            case 8:
            {
                static u8 maixcam_query_sent = 0;

                if (wp_idx < final_path_length - 1)
                {
                    if (move_sub_step == 0)
                    {
                        Point cur = final_path[wp_idx];
                        Point next = final_path[wp_idx + 1];
                        int dr = next.row - cur.row;
                        int dc = next.col - cur.col;

                        u16 angle = 0;
                        if (dr == 1 && dc == 0)
                            angle = 0;
                        else if (dr == -1 && dc == 0)
                            angle = 180;
                        else if (dr == 0 && dc == 1)
                            angle = 270;
                        else if (dr == 0 && dc == -1)
                            angle = 90;

                        if (Horizontal_Move(GRID_SIZE_CM, 25, angle))
                        {
                            move_sub_step = 1;
                            move_wait_ms = 0;
                            maixcam_query_sent = 0;
                        }
                    }
                    else if (move_sub_step == 1)
                    {
                        move_wait_ms += 20;
                        if (move_wait_ms >= 3000)
                        {
                            /* 移动完成，发送格子坐标给MaixCam查询视觉 */
                            if (!maixcam_query_sent)
                            {
                                Pi_ClearRxState(); /* 丢弃移动期间堆积的旧数据 */
                                Point p = final_path[wp_idx + 1];
                                u8 grid_a = (u8)(COLS - p.col); /* A=列(1-9) */
                                u8 grid_b = (u8)(p.row + 1);    /* B=行(1-7) */
                                Pi_SendGridQuery(grid_a, grid_b);
                                maixcam_query_sent = 1;
                                move_wait_ms = 0;
                            }
                            else if (move_wait_ms >= 1500)
                            {
                                /* 等待MaixCam响应（最多2s），收到则转发给地面端 */
                                if (MaixCam_GetData_Flag())
                                {
                                    u8 pi_data[20];
                                    Pi_GetData(pi_data);
                                    u8 n = pi_data[2]; /* 检测到的字符数量 */
                                    if (n > 0 && pi_data[8] == 0x01)
                                    {
                                        u8 grid_x = pi_data[0];
                                        u8 grid_y = pi_data[1];
                                        u8 i;
                                        for (i = 0; i < n && i < 5; i++)
                                        {
                                            u8 ch = pi_data[3 + i];
                                            if (ch != 0)
                                            {
                                                u8 fb_buf[6];
                                                fb_buf[0] = 0x5A;
                                                fb_buf[1] = ch;
                                                fb_buf[2] = grid_x;
                                                fb_buf[3] = grid_y;
                                                fb_buf[4] = 0x5F;
                                                fb_buf[5] = ch + grid_x + grid_y;
                                                DrvUart2SendBuf(fb_buf, 6);
                                            }
                                        }
                                    }
                                }
                                /* 处理完直接进入下一格 */
                                maixcam_query_sent = 0;
                                wp_idx++;
                                move_wait_ms = 0;
                                move_sub_step = 0; /* 回到 sub_step 0 发送下一个 Horizontal_Move */
                            }
                        }
                    }
                }
                else
                {
                    wp_idx = 0;
                    move_sub_step = 0;
                    move_wait_ms = 0;
                    maixcam_query_sent = 0;
                    mission_step++;
                }
            }
            break;

            case 9:
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

#ifndef __PATH_PLANNING_H
#define __PATH_PLANNING_H

#include "User_Task.h"

#define ROWS 7
#define COLS 9
#define MAX_CELLS (ROWS * COLS)
#define MAX_PATH_LENGTH 150
#define BARRIER_COUNT 3

/* 障碍物列表：barriers[].row=B(行1-7), barriers[].col=A(列1-9) */
extern Point barriers[BARRIER_COUNT];

/* 路径规划结果（供状态机读取） */
extern Point final_path[MAX_PATH_LENGTH];
extern int final_path_length;

/* 路径规划入口 */
void run_path_planner(void);

/* 路径步进反馈：将当前路径点索引映射为字符并通过串口2发给地面站 */
void send_step_feedback(int wp_index);

#endif

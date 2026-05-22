#ifndef __PATH_PLANNING_H
#define __PATH_PLANNING_H

#include "User_Task.h"

#define ROWS 7
#define COLS 9
#define MAX_CELLS (ROWS * COLS)
#define MAX_PATH_LENGTH 150
#define BARRIER_COUNT 3

/* 路径规划结果（供状态机读取） */
extern Point final_path[MAX_PATH_LENGTH];
extern int final_path_length;

/* 调试输出路径点 */
extern PathPoint path_points[MAX_PATH_LENGTH];
extern uint8_t path_len_routine;

/* 路径规划入口 */
void run_path_planner(void);

#endif

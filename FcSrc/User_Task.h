#ifndef __USER_TASK_H
#define __USER_TASK_H

#include "SysConfig.h"

#define ROWS 7
#define COLS 9
#define MAX_CELLS (ROWS * COLS)
#define MAX_PATH_LENGTH 150
#define BARRIER_COUNT 3
#define MAX_PATH_POINTS 100
#define GRID_SIZE_CM 50

typedef struct {
    int16_t x;
    int16_t y;
} PathPoint;

typedef struct {
    int row, col;
} Point;

extern PathPoint path_points[MAX_PATH_POINTS];
extern uint8_t path_len_routine;

extern Point barriers[BARRIER_COUNT];
extern int grid[ROWS][COLS];

extern s16 now_x;
extern s16 now_y;

void UserTask_OneKeyCmd(void);
float y_move_pid(s16 cy);
float x_move_pid(s16 cx);

void run_path_planner(void);
void init_grid(void);
int check_connectivity(void);
void generate_barriers(void);
void collect_accessible_cells(void);
void real_routine(void);

#endif
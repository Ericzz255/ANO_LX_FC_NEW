#ifndef __PATH_PLANNING_H
#define __PATH_PLANNING_H

#include "User_Task.h"

#define ROWS 7
#define COLS 9
#define MAX_CELLS (ROWS * COLS)
#define MAX_PATH_LENGTH 150
#define BARRIER_COUNT 3

/* barriers[].row=B(1..7), barriers[].col=A(1..9). */
extern Point barriers[BARRIER_COUNT];

extern Point final_path[MAX_PATH_LENGTH];
extern int final_path_length;

/*
 * Atomically replaces all three barriers.
 * Input order is A1,B1,A2,B2,A3,B3 (A=1..9, B=1..7).
 * Returns 1 only for three valid, distinct cells.
 */
u8 PathPlanner_SetBarriers(const u8 barrier_data[BARRIER_COUNT * 2]);
u8 PathPlanner_HasBarrierConfiguration(void);

void generate_barriers(void);
int snake_tsp(Point order[]);
int snake_tsp_col(Point order[]);
int find_shortest_path(Point start, Point end, Point path[], int max_len);

/* Returns 1 only when a bounded route starting at grid(0,0) was produced. */
u8 run_path_planner(void);

/* grid row -> SLAM +X, grid column -> SLAM +Y, each cell is 50 cm. */
u8 PathPlanner_PointToSlam(Point point, s16 *x_cm, s16 *y_cm);

void send_step_feedback(int wp_index);

#endif

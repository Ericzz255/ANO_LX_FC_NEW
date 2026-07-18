#ifndef __USER_TASK_H
#define __USER_TASK_H

#include "SysConfig.h"

#define GRID_SIZE_CM 50

typedef struct
{
    int16_t x;
    int16_t y;
} PathPoint;

typedef struct
{
    int row;
    int col;
} Point;

void UserTask_OneKeyCmd(void);

u8 UserTask_GetMissionStep(void);
u8 UserTask_GetMissionStatus(void);
u16 UserTask_GetWaypointIndex(void);
u16 UserTask_GetPathLength(void);

#endif

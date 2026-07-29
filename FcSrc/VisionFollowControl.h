#ifndef __VISION_FOLLOW_CONTROL_H
#define __VISION_FOLLOW_CONTROL_H

#include "SysConfig.h"

/*
 * MaixCAM normalized body-frame optical error -> body-frame velocity target.
 * The controller writes only rt_tar.vel_x/vel_y, never motor PWM.
 */
void VisionFollowControl_Begin(void);
void VisionFollowControl_Reset(void);
u8 VisionFollowControl_Update(void);
u8 VisionFollowControl_IsTargetCentered(void);

s16 VisionFollowControl_GetOutputVelX(void);
s16 VisionFollowControl_GetOutputVelY(void);

#endif

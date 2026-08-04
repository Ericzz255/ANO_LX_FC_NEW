#ifndef __POSITION_FUSION_H
#define __POSITION_FUSION_H

#include "SysConfig.h"

/*
 * SLAM-dominant horizontal position fusion.
 *
 * The Orange Pi SLAM position remains the absolute reference. Optical flow
 * contributes only a low-weight velocity measurement between SLAM frames.
 */
void PositionFusion_Reset(void);
void PositionFusion_PushSlam(s16 x_cm, s16 y_cm);
void PositionFusion_Update(float dt_s);
u8 PositionFusion_GetPosition(s16 *x_cm, s16 *y_cm);
u8 PositionFusion_GetVelocity(s16 *vx_cmps, s16 *vy_cmps);
u8 PositionFusion_IsOpticalFlowActive(void);

#endif

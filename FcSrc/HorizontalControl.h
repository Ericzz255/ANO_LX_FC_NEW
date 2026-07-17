#ifndef __HORIZONTAL_CONTROL_H
#define __HORIZONTAL_CONTROL_H

#include "SysConfig.h"

/**
 * @brief 当前SLAM水平位置，单位cm。
 * @note  X轴为机头前方正方向，Y轴为机体左侧正方向。
 */
extern s16 now_x;
extern s16 now_y;

/**
 * @brief 提交一帧新的SLAM水平位置。
 * @param x_cm 机头前后方向位置，机头前方为正，单位cm。
 * @param y_cm 机体左右方向位置，机体左侧为正，单位cm。
 */
void HorizontalControl_SetPosition(s16 x_cm, s16 y_cm);

/**
 * @brief 清除水平闭环状态，并将水平速度目标置零。
 */
void HorizontalControl_Reset(void);

/**
 * @brief 停止水平速度输出，但保留当前锁定目标。
 */
void HorizontalControl_StopOutput(void);

/**
 * @brief 将最新的有效SLAM位置锁定为水平保持目标。
 */
void HorizontalControl_CaptureTarget(void);

/**
 * @brief 更新水平位置外环，将位置误差转换为水平速度目标。
 * @note  按20ms（50Hz）调用周期设计。
 */
void HorizontalControl_Update(void);

/**
 * @brief 调试数据读取接口，用于用户数据帧观察闭环状态。
 */
s16 HorizontalControl_GetTargetX(void);
s16 HorizontalControl_GetTargetY(void);
s16 HorizontalControl_GetErrorX(void);
s16 HorizontalControl_GetErrorY(void);
s16 HorizontalControl_GetOutputVelX(void);
s16 HorizontalControl_GetOutputVelY(void);
u8 HorizontalControl_GetFaultCode(void);

#endif

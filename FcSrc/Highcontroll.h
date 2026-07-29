#ifndef __HIGHCONTROLL_H
#define __HIGHCONTROLL_H

#include "SysConfig.h"

/**
 * @brief 清除高度闭环状态，并把垂直速度目标置零。
 */
void HeightControl_Reset(void);

/**
 * @brief 高度外环更新：激光高度误差(cm)转换为垂直速度目标(cm/s)。
 * @param target_alt_cm 目标离地高度，单位 cm。
 * @note  当前参数按 20ms（50Hz）调用周期设计。
 */
void HeightControl_Update(float target_alt_cm);

/**
 * @brief 判断滤波后的当前高度是否进入目标高度容差范围。
 */
u8 HeightControl_TargetReached(float target_alt_cm, float tolerance_cm);

#endif

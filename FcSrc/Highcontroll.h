#ifndef __HIGHCONTROLL_H
#define __HIGHCONTROLL_H

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

#endif

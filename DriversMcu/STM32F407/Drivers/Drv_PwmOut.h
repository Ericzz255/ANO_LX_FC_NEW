#ifndef _PWM_OUT_H_
#define _PWM_OUT_H_

#include "SysConfig.h"

//注意，若打开ESC校准功能，将可能发生不可预料的损坏或者人身伤害，后果自负。
//一定需要校准时，请拆掉螺旋桨，尽量避免发生意外。
//校准ESC成功后，记得关闭此功能，避免出现意外。
#define ESC_CALI 0 //1：打开；0：关闭。

//#define PWM_FRE_HZ        400
//#define PWM_FRE_HZ        350    //天行者电调可以尝试用 350 hz

/*
 * 电磁铁MOSFET模块控制：
 * 凌霄飞控IOA接口3号脚=PB0控制信号，模块GND与飞控共地。
 * 上电初始化时预置高电平并启用内部上拉；高电平吸合、低电平释放。
 */
#define DROP_MAGNET_ACTIVE_HIGH        1

void DrvPwmOutInit(void);
void DrvMotorPWMSet(int16_t pwm[]); //范围0-1000
void DrvDropMagnetSet(uint8_t enable);

#endif

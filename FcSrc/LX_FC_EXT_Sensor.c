/*==========================================================================
 * 描述    ：凌霄飞控外置传感器处理
 * 更新时间：2020-02-06 
 * 作者		 ：匿名科创-Jyoun
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
 * 项目合作：18084888982，18061373080
============================================================================
 * 匿名科创团队感谢大家的支持，欢迎大家进群互相交流、讨论、学习。
 * 若您觉得匿名有不好的地方，欢迎您拍砖提意见。
 * 若您觉得匿名好，请多多帮我们推荐，支持我们。
 * 匿名开源程序代码欢迎您的引用、延伸和拓展，不过在希望您在使用时能注明出处。
 * 君子坦荡荡，小人常戚戚，匿名坚决不会请水军、请喷子，也从未有过抹黑同行的行为。  
 * 开源不易，生活更不容易，希望大家互相尊重、互帮互助，共同进步。
 * 只有您的支持，匿名才能做得更好。  
===========================================================================*/
#include "LX_FC_EXT_Sensor.h"
#include "Drv_AnoOf.h"
#include "ANO_DT_LX.h"

_fc_ext_sensor_st ext_sens;

/*
 * SLAM is a non-body-fixed position sensor. Feed its absolute X/Y position
 * to the LingXiao IMU through 0x32 instead of presenting optical flow as the
 * horizontal motion source through 0x33.
 */
void LX_FC_EXT_Sensor_SetSlamPosition(s16 x_cm, s16 y_cm)
{
	ext_sens.gen_pos.st_data.ulhca_pos_cm[0] = (s32)x_cm;
	ext_sens.gen_pos.st_data.ulhca_pos_cm[1] = (s32)y_cm;
	ext_sens.gen_pos.st_data.ulhca_pos_cm[2] = (s32)0x80000000UL;
	dt.fun[0x32].WTS = 1;
}

static inline void General_Distance_Data_Handle()
{
	static u8 of_alt_update_cnt;
	if (of_alt_update_cnt != ano_of.alt_update_cnt)
	{
		//
		of_alt_update_cnt = ano_of.alt_update_cnt;
		//
		ext_sens.gen_dis.st_data.direction = 0;
		ext_sens.gen_dis.st_data.angle_100 = 270;
		ext_sens.gen_dis.st_data.distance_cm = ano_of.of_alt_cm;
		//触发发送
		dt.fun[0x34].WTS = 1;
	}
}

void LX_FC_EXT_Sensor_Task(float dT_s) //1ms
{
	/*
	 * Do not send optical-flow X/Y through 0x33. Horizontal motion
	 * estimation is driven by SLAM position frames sent through 0x32.
	 */
	//
	General_Distance_Data_Handle();
}

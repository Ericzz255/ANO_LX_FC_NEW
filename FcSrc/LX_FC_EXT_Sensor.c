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
#include "Drv_Sys.h"
#include "ANO_DT_LX.h"

_fc_ext_sensor_st ext_sens;

#define SLAM_VELOCITY_WINDOW_MS          200U
#define SLAM_VELOCITY_RESET_TIMEOUT_MS   300U
#define SLAM_VELOCITY_FILTER_ALPHA       0.60f
#define SLAM_VELOCITY_DEADBAND_CMPS      3.0f
#define SLAM_VELOCITY_REJECT_CMPS        150.0f

typedef struct
{
	u8 initialized;
	u8 sample_count;
	s16 older_x_cm;
	s16 older_y_cm;
	u32 older_ms;
	s16 newer_x_cm;
	s16 newer_y_cm;
	u32 newer_ms;
	float filtered_x_cmps;
	float filtered_y_cmps;
} _slam_velocity_estimator_st;

static _slam_velocity_estimator_st slam_velocity;

static void SlamVelocity_ResetEstimator(s16 x_cm,
										s16 y_cm,
										u32 now_ms)
{
	slam_velocity.initialized = 1U;
	slam_velocity.sample_count = 1U;
	slam_velocity.older_x_cm = x_cm;
	slam_velocity.older_y_cm = y_cm;
	slam_velocity.older_ms = now_ms;
	slam_velocity.newer_x_cm = x_cm;
	slam_velocity.newer_y_cm = y_cm;
	slam_velocity.newer_ms = now_ms;
	slam_velocity.filtered_x_cmps = 0.0f;
	slam_velocity.filtered_y_cmps = 0.0f;
}

static float SlamVelocity_Abs(float value)
{
	return (value >= 0.0f) ? value : -value;
}

static s16 SlamVelocity_RoundToS16(float value)
{
	if (value >= 0.0f)
	{
		return (s16)(value + 0.5f);
	}

	return (s16)(value - 0.5f);
}

static void SlamVelocity_Publish(void)
{
	float velocity_x_cmps = slam_velocity.filtered_x_cmps;
	float velocity_y_cmps = slam_velocity.filtered_y_cmps;

	if (SlamVelocity_Abs(velocity_x_cmps) <
		SLAM_VELOCITY_DEADBAND_CMPS)
	{
		velocity_x_cmps = 0.0f;
	}
	if (SlamVelocity_Abs(velocity_y_cmps) <
		SLAM_VELOCITY_DEADBAND_CMPS)
	{
		velocity_y_cmps = 0.0f;
	}

	ext_sens.gen_vel.st_data.hca_velocity_cmps[0] =
		SlamVelocity_RoundToS16(velocity_x_cmps);
	ext_sens.gen_vel.st_data.hca_velocity_cmps[1] =
		SlamVelocity_RoundToS16(velocity_y_cmps);
	ext_sens.gen_vel.st_data.hca_velocity_cmps[2] = (s16)0x8000;
	dt.fun[0x33].WTS = 1;
}

/*
 * SLAM is a non-body-fixed position sensor. Feed its absolute X/Y position
 * to the LingXiao IMU through 0x32. Derive the 0x33 horizontal velocity from
 * the same SLAM source so the LingXiao velocity loop remains available
 * without using optical-flow motion data.
 */
void LX_FC_EXT_Sensor_SetSlamPosition(s16 x_cm, s16 y_cm)
{
	u32 now_ms = GetSysRunTimeMs();
	u32 elapsed_ms;
	u32 latest_sample_ms;

	ext_sens.gen_pos.st_data.ulhca_pos_cm[0] = (s32)x_cm;
	ext_sens.gen_pos.st_data.ulhca_pos_cm[1] = (s32)y_cm;
	ext_sens.gen_pos.st_data.ulhca_pos_cm[2] = (s32)0x80000000UL;
	dt.fun[0x32].WTS = 1;

	if (slam_velocity.initialized == 0U)
	{
		SlamVelocity_ResetEstimator(x_cm, y_cm, now_ms);
		SlamVelocity_Publish();
		return;
	}

	latest_sample_ms = (slam_velocity.sample_count >= 2U) ?
		slam_velocity.newer_ms : slam_velocity.older_ms;

	if ((u32)(now_ms - latest_sample_ms) >
		SLAM_VELOCITY_RESET_TIMEOUT_MS)
	{
		/*
		 * A long gap invalidates the previous differentiation baseline.
		 * Reinitialize at zero speed; subsequent fresh samples rebuild it.
		 */
		SlamVelocity_ResetEstimator(x_cm, y_cm, now_ms);
	}
	else if (slam_velocity.sample_count < 2U)
	{
		slam_velocity.newer_x_cm = x_cm;
		slam_velocity.newer_y_cm = y_cm;
		slam_velocity.newer_ms = now_ms;
		slam_velocity.sample_count = 2U;
	}
	else
	{
		elapsed_ms = (u32)(now_ms - slam_velocity.older_ms);

		if (elapsed_ms >= SLAM_VELOCITY_WINDOW_MS)
		{
			float raw_x_cmps =
				(float)((s32)x_cm -
						(s32)slam_velocity.older_x_cm) *
				1000.0f / (float)elapsed_ms;
			float raw_y_cmps =
				(float)((s32)y_cm -
						(s32)slam_velocity.older_y_cm) *
				1000.0f / (float)elapsed_ms;

			/*
			 * A speed outside the mission envelope is treated as a SLAM
			 * coordinate jump. Re-anchor without feeding the jump to the IMU.
			 */
			if (SlamVelocity_Abs(raw_x_cmps) <=
					SLAM_VELOCITY_REJECT_CMPS &&
				SlamVelocity_Abs(raw_y_cmps) <=
					SLAM_VELOCITY_REJECT_CMPS)
			{
				slam_velocity.filtered_x_cmps +=
					SLAM_VELOCITY_FILTER_ALPHA *
					(raw_x_cmps -
					 slam_velocity.filtered_x_cmps);
				slam_velocity.filtered_y_cmps +=
					SLAM_VELOCITY_FILTER_ALPHA *
					(raw_y_cmps -
					 slam_velocity.filtered_y_cmps);
			}

			/*
			 * Roll the 200 ms window forward by one 10 Hz sample. This
			 * keeps velocity updates at 10 Hz instead of reducing them
			 * to 5 Hz.
			 */
			slam_velocity.older_x_cm =
				slam_velocity.newer_x_cm;
			slam_velocity.older_y_cm =
				slam_velocity.newer_y_cm;
			slam_velocity.older_ms =
				slam_velocity.newer_ms;
		}

		slam_velocity.newer_x_cm = x_cm;
		slam_velocity.newer_y_cm = y_cm;
		slam_velocity.newer_ms = now_ms;
	}

	SlamVelocity_Publish();
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
	 * Optical-flow X/Y is deliberately not used. Horizontal position and
	 * velocity are both published when a fresh SLAM sample is received.
	 */
	//
	General_Distance_Data_Handle();
}

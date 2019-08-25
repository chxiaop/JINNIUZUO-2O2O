/** 
  * @file     bsp_TriggerMotor.c
  * @version  v2.0
  * @date     July,8th 2019
	*
  * @brief    拨弹电机基础配置包含1.拨弹电机PID初始化
	*								          2.拨弹电机控制
	*								          3.拨弹电机速度位置闭环控制
	*
  *	@author   Fatmouse,part of the code reference Link's code
  *
  */
	
#include "bsp_TriggerMotor.h"
#include "shoot_task.h"
#include "bsp_can.h"
#include "pid.h"
#include "gimbal_task.h"
#include "remote_msg.h"
#include "modeswitch_task.h"
#include "control_def.h"
#include "shoot_task.h"
uint8_t single_flag;
heat_limit_t heat_limit;
int32_t t,p;
uint8_t ML_flag;
uint8_t ML_key_up = 1;
#define MATH_ABS(x)     ((x > 0) ? (x) : (-x))
/**
  * @brief          拨弹电机初始化，初始化PID，电机指针
  * @author         Fatmouse
  * @param[in]      void
  * @retval         返回空
  */

void TriggerMotor_init(void)
{
	//shoot.shoot_mode = CONTROL_MODE_STOP;
	PID_struct_init(&pid_trigger_spd, POSITION_PID, 28000, 15000,
									4.0f,0.005f,0.0f); 
	PID_struct_init(&pid_trigger_angle, POSITION_PID, 4320, 200,
									0.3f,0.001f,0.0f); 
}


void TriggerMotor_status(void)
{
	if(rc.sw2 == RC_UP)
	{
		single_flag =1;
	}
	if(rc.sw2 == RC_MI&&single_flag)
	{
		shoot.shoot_mode = CONTROL_MODE_BULLET_SINGLE;
		single_flag =0;
	}
	if(rc.sw2 == RC_DN)
	{
		shoot.shoot_mode = CONTROL_MODE_BULLET;
	}
	
}
/**
  * @brief          拨弹电机PID控制
  * @param[in]      void
  * @retval         void
  */
void TriggerMotor_control(void)
{
	Judgement_Heat_data_Update();
	switch (shoot.shoot_mode)
	{
		case CONTROL_MODE_BULLET:
		{
			if((heat_limit.shoot_heat_cooling_limit - heat_limit.shoot_current_heat)<100)
			{
				shoot.shoot_mode = CONTROL_MODE_STOP;
			}
			else
			{
			t = motor_trigger.total_angle;
			p = shoot.pid.last_trigger_angle_ref;
				if((int32_t)shoot.pid.last_trigger_angle_ref - motor_trigger.total_angle < 1000)
				{
					if(TIM3->CCR3 > 1000)
					{
						shoot.pid.trigger_angle_ref += 36859;
						shoot.pid.last_trigger_angle_ref = shoot.pid.trigger_angle_ref;
					}
				}
			}				
		}break;
		case CONTROL_MODE_BULLET_SINGLE:
		{
			if(TIM3->CCR3 >= 1000)
		{
			if(rc.mouse.l && ML_key_up) 
		 {
			ML_key_up = 0;
			ML_flag = ~ML_flag;	
			shoot.pid.trigger_angle_ref += 36859;
				shoot.pid.last_trigger_angle_ref  = shoot.pid.trigger_angle_ref;
		 }
	 }
		}break;
		case CONTROL_MODE_STOP:
		{		
		}break;
	  default:
		{
		}break;
	}
	shoot.pid.trigger_angle_fdb = motor_trigger.total_angle;
	pid_calc(&pid_trigger_angle, shoot.pid.trigger_angle_fdb,shoot.pid.trigger_angle_ref);//编码器环
	
	shoot.pid.trigger_spd_fdb = motor_trigger.speed_rpm;
	shoot.pid.trigger_spd_ref = pid_trigger_angle.pos_out;
	//shoot.pid.trigger_spd_ref = rc.ch4*-13.0f;
	pid_calc(&pid_trigger_spd, shoot.pid.trigger_spd_fdb,shoot.pid.trigger_spd_ref);//速度环
	
	gimbal.current[2] = pid_trigger_spd.pos_out;
}
/**
  * @brief          热量数据更新
  * @author         link
  * @param[in]      void
  * @retval         void
  */
void Judgement_Heat_data_Update(void)
{   
		//机器人id
		heat_limit.robot_id=judge_recv_mesg.robot_state_data.robot_id;
		
		//当前热量
		heat_limit.shoot_current_heat=judge_recv_mesg.power_heat_data.shooter_heat0;
		
		//冷却速率
		heat_limit.shoot_heat_cooling_rate=judge_recv_mesg.robot_state_data.shooter_heat0_cooling_rate;
		
		//热量临界
		heat_limit.shoot_heat_cooling_limit=judge_recv_mesg.robot_state_data.shooter_heat0_cooling_limit;
}

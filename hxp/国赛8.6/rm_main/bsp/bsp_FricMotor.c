/** 
  * @file     bsp_FricMotor.c
  * @version  v2.1
  * @date     July,8th 2019
	*
  * @brief    °üº¬1.Ä¦²ÁÂÖµç»úµÄ³õÊ¼»¯
									2.Ä¦²ÁÂÖµç»úPWM¿ØÖÆ
									3.Ä¦²ÁÂÖµç»úÐ±ÆÂÆô¶¯
                  4.Ä¦²ÁÂÖ×ªËÙ·´À¡
									5.Ä¦²ÁÂÖPID¿ØÖÆ
	*
  *	@author   Fatmouse,part of the code reference Link's code
  *
  */
#include "bsp_FricMotor.h"
#include "tim.h"
#include "pid.h"
#include "math_calcu.h"
#include "remote_msg.h"
#include "shoot_task.h"
Slope_Struct shoot_Fric_pwm_L;
Slope_Struct shoot_Fric_pwm_R; 
uint16_t last_pwm_r;
uint16_t last_pwm_l;
void FricMotor_init(void)
{ 
	//950-2000
	//Æô¶¯Ê±£¬ÓÍÃÅ´òµ½×îµÍ
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3); //×óÄ¦²ÁÂÖ	
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4); //ÓÒÄ¦²ÁÂÖ
	
  TIM3->CCR3 = 800;
	TIM3->CCR4 = 800;
		shoot_Fric_pwm_L.limit_target = 800;//³õÊ¼»¯Ä¦²ÁÂÖÐ±ÆÂº¯Êý
	  shoot_Fric_pwm_L.real_target  = 800  ;
	  shoot_Fric_pwm_L.change_scale = 0.5;
	  shoot_Fric_pwm_R.limit_target = 800;//³õÊ¼»¯Ä¦²ÁÂÖÐ±ÆÂº¯Êý
	  shoot_Fric_pwm_R.real_target  = 800  ;
	  shoot_Fric_pwm_R.change_scale = 0.5;
	
}


/**
	*@func   		void FricGunControl(uint8_t Control)
	*@bref			Ä¦²ÁÂÖÆðÍ£
	*@param[in] Control£º0ÎªÍ£Ö¹£¬1ÎªÆô¶¯
  *@retval    void
	*@note			900ÒÔÉÏÆð×ª 
	*/
void FricGunControl(uint16_t pwm1,uint16_t pwm2)
{
	shoot_Fric_pwm_L.limit_target=800+pwm1;
	shoot_Fric_pwm_R.limit_target=800+pwm2;
	
	Slope_On(&shoot_Fric_pwm_L); //×óÄ¦²ÁÂÖÐ±ÆÂÆô¶¯
	Slope_On(&shoot_Fric_pwm_R); //ÓÒÄ¦²ÁÂÖÐ±ÆÂÆô¶¯ 
	
	TIM3->CCR3=shoot_Fric_pwm_R.real_target;
	TIM3->CCR4=shoot_Fric_pwm_L.real_target;
}     

void fricmotor_status(void)
{
	switch (shoot.shoot_speed)
	{
		case FRIC_SPEED_ENERGY:
		{

			laser_on();
			FricGunControl(energy_speed,energy_speed);

		}break;
		case FRIC_SPEED_HIGH:
		{
			laser_on();
			FricGunControl(high_speed,high_speed);
		}break;
		case FRIC_SPEED_LOW:
		{
			laser_on();
			FricGunControl(low_speed,low_speed);
		}break;
		case FRIC_SPEED_STOP:
		{
			laser_off();
			FricGunControl(init_speed,init_speed);
		}break;
		default:
		{
		}
	}
}



/****************** laser references ********************/
void laser_on(void)
{
	//HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);
}
void laser_off(void)
{
//  HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_RESET);
}

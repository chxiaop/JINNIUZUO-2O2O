/** 
  * @file     shoot_task.c
  * @version  v2.0
  * @date     July,6th 2019
	*
  * @brief    ������
	*
  *	@author   Fatmouse
  *
  */
#define __SHOOT_TASK_GLOBALS
#include "shoot_task.h"
#include "cmsis_os.h"
#include "comm_task.h"
#include "remote_msg.h"
#include "modeswitch_task.h"
#include "bsp_CoverServo.h"
#include "bsp_FricMotor.h"
#include "bsp_TriggerMotor.h"
#include "visionfire_task.h"
shoot_t shoot;
uint16_t TEXT_ADD;
extern TaskHandle_t can_msg_send_task_t;
extern uint8_t ML_key_up;
void shoot_task(void const *argu)
{
  CoverServo_switch();
	fricmotor_status();

	if(ctrl_mode==KEYBOARD_MODE)
	{
		if(rc.mouse.l)
		{
			//shoot.shoot_mode = CONTROL_MODE_BULLET;
			if(vision_msg.vision_mode == little_energy_mode || vision_msg.vision_mode == macro_energy_mode)
			{
			shoot.shoot_mode = CONTROL_MODE_BULLET_SINGLE;
			}
			else 
			{
			 shoot.shoot_mode = CONTROL_MODE_BULLET;
			}
		}
		else
		{
			shoot.shoot_mode = CONTROL_MODE_STOP;			
		}	
	  if(!rc.mouse.l)
		{
			ML_key_up = 1;
		}
		
	}
	//TriggerMotor_status();
	TriggerMotor_control();

	
//	osSignalSet(can_msg_send_task_t, SHOOT_CONTROL_MSG_SEND);

}

void shoot_init()
{
  shoot.shoot_mode = CONTROL_MODE_STOP;
	CoverServo_init();
	FricMotor_init();
	TriggerMotor_init();
}

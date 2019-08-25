/** 
  * @file     modeswitch_task.c
  * @version  v2.0
  * @date     July,6th 2019
	*
  * @brief    ģʽѡ�������ļ�
	*
  *	@author   Fatmouse
  *
  */
	
#include "modeswitch_task.h"
#include "chassis_task.h"
#include "cmsis_os.h"
#include "remote_msg.h"
#include "gimbal_task.h"
#include "shoot_task.h"
#include "bsp_can.h"
#include "supercap_task.h"
#include "control_def.h"
#include "bsp_TriggerMotor.h"
#include "visionfire_task.h"
#include "judge_send_task.h"
extern osTimerId chassis_timer_id;
extern osTimerId gimbal_timer_id;
extern osTimerId shoot_timer_id;
extern osTimerId supercap_timer_id;
extern osTimerId judge_sendTimer_id;
ctrl_mode_e ctrl_mode;
ctrl_mode_e last_ctrl_mode;
uint8_t house_sw_flag;
uint8_t retato;
uint8_t dance_symbol;
uint8_t Q_key_up,E_key_up;
void mode_switch_task(void const *argu)
{
	//�������̺���̨��������ʱ������
  osTimerStart(gimbal_timer_id, GIMBAL_PERIOD);
	osTimerStart(chassis_timer_id, CHASSIS_PERIOD);
	osTimerStart(shoot_timer_id, SHOOT_PERIOD);
	osTimerStart(supercap_timer_id, SUPERCAP_PERIOD);
	osTimerStart(judge_sendTimer_id,JUDGE_SEND_PERIOD);
   uint32_t mode_wake_time = osKernelSysTick();
	for(;;)
	{
		taskENTER_CRITICAL();

		get_sw_mode();
		
	  taskEXIT_CRITICAL();
		
		osDelayUntil(&mode_wake_time, INFO_GET_PERIOD);
	}
}

static void sw1_mode_handler(void)
{

	
	switch (rc.sw1)
  {
		case RC_UP:
		{
			ctrl_mode = REMOTER_MODE;
		}
		break;
		case RC_MI:
		{
      ctrl_mode = CHASSIS_REMOTER_MODE	;
		}
		break;
		case RC_DN:
		{

			ctrl_mode = PROTECT_MODE;
		}
		break;
		default:
		break;
  }

}

static void sw2_mode_handler(void)
{ 
	if(ctrl_mode == CHASSIS_REMOTER_MODE )
	{
		switch (rc.sw2)
	  {
		  case RC_UP:
		  {
			 retato = 0;
			 dance_symbol = 0;
		  }
			 break;
		  case RC_MI:
		  {
			 retato = 0;
			 if(!retato)
			 {
				dance_symbol = 1;
			 }
	    }
			 break;
		  case RC_DN:
		  {
			dance_symbol = 0;
			if(!dance_symbol)
			 {
				retato = 1;
		   }
	    }
			 break;
		  default:
		  {
		  }
	  }
  }
	if(ctrl_mode == REMOTER_MODE )
	{
		switch (rc.sw2)
	  {
		  case RC_UP:
		  {
			shoot.shoot_speed = FRIC_SPEED_STOP;
			shoot.shoot_mode = CONTROL_MODE_STOP;
		  }
			 break;
		  case RC_MI:
		  { 
		   if(ctrl_mode == REMOTER_MODE || ctrl_mode == PROTECT_MODE)	
       {
	    	shoot.shoot_mode = CONTROL_MODE_STOP;
	    	shoot.shoot_speed = FRIC_SPEED_LOW;
	     }
	    }
			 break;
		  case RC_DN:
		  {
			shoot.shoot_mode = CONTROL_MODE_BULLET;
	    }
			 break;
		  default:
		  {
		  }
	  }
	}
}

void get_sw_mode(void)
{
	sw1_mode_handler();
	sw2_mode_handler();
}

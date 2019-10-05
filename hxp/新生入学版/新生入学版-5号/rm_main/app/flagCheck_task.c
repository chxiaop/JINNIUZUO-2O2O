/** 
  * @file     flagCheck_task.c
  * @version  v1.0
  * @date     Oct,5th 2019
	*
  * @brief    ??????????????
	*
  *	@author   Apophis
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
#include "flagCheck_task.h"

extern osThreadId flag_check_task_t;
extern ctrl_mode_e ctrl_mode;
flag_t flag;

/**
  * @brief flagCheck_task
  * @param     
  * @attention  
	* @note  
  */
void flag_check_task(void const *argu)
{
	for(;;)
	{
		flag.status.flag= 100;
    unlock_init();
		osDelay(100);
	}
}
	/**
  * @brief unlock_init
  * @param     
  * @attention  
	* @note  
  */
void unlock_init(void)
{
	if(ctrl_mode == PROTECT_MODE&&shoot.shoot_mode == CONTROL_MODE_STOP)
	{
	 if(rc.ch4==-660)
	 {
		if(rc.ch3==660)
		{
		 flag.unlock = 1;
		}
	 }					
	}
}			

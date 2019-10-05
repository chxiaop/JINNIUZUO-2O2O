/** 
  * @file     status_task.c
  * @version  v2.0
  * @date     July,6th 2019
	*
  * @brief  ״̬ 状态监控任务
	*
  *	@author   Fatmouse
  *
  */
#include "status_task.h"
#include "tim.h"
#include "bsp_imu.h"
uint8_t sum;
status_t status;
/**
  * @brief status_task
  * @param     
  * @attention  
	* @note  
  */
void status_task(void const *argu)
{
  for(;;)
	{
		HAL_GPIO_TogglePin(GPIOD, LED_A_Pin);
		//imu_status();
		osDelay(100);
	}
}

void status_init()
{

}
void status_init_success()
{
	status.dbus_status = 0;
	status.imu_status = 0;
	for(int i = 0;i<4;i++)
	{	
	  status.chassis_status[i] = 0;
	}
	status.gimbal_status[0] = 0;
	status.gimbal_status[1] = 0;
}

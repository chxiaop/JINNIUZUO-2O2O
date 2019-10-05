/** 
  * @file     status_task.h
  * @version  v2.0
  * @date     July,6th 2019
	*
  * @brief    ״̬�������ṹ�嶨��
	*
  *	@author   Fatmouse
  *
  */
#ifndef _STATUS_TASK_H_
#define _STATUS_TASK_H_

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
typedef struct
{
  uint8_t imu_status;	
  uint8_t dbus_status;
  uint8_t chassis_status[4];
	uint8_t gimbal_status[2];
	uint8_t vision_status;
} status_t;

extern status_t status;
void status_task(void const *argu);
void status_init(void);
void status_init_success(void);
#endif

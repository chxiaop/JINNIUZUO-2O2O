/**
  * @file     flagCheck_task.h
  * @version  v1.0
  * @date     Oct,5th 2019
  *
  * @brief    
  *
  *	@author   Apophis
  *
  */
#ifdef  __FLAGCHECK_TASK_GLOBALS
#define __FLAGCHECK_TASK_EXT
#else
#define __FLAGCHECK_TASK_EXT extern
#endif

#ifndef __FLAGCHECK_TASK_H__
#define __FLAGCHECK_TASK_H__

#include "stm32f4xx_hal.h"
#include "pid.h"


typedef struct
{
  /* gimbal*/
	float flag;
} flag_gimbal_t;

typedef struct
{
  /* chassis*/
	float flag;
} flag_chassis_t;

typedef struct
{
  /* shoot*/
	float flag;
} flag_shoot_t;

typedef struct
{
  /* status*/
	float flag;
} flag_status_t;

typedef struct
{
  /* status*/
	float a1;
	float a2;
	float a3;
	float a4;
	float a5;
	float a6;
	float a7;
	float a8;
	float a9;
} flag_check_t;

typedef struct
{
  /* gimbal information */
  flag_gimbal_t      gimbal;
	
  /* chassis information */
  flag_chassis_t     chassis;
	
  /* shoot information */
  flag_shoot_t       shoot;
	
  /* status information */
  flag_status_t      status;
	
  flag_check_t      check;
  /* read from flash */
  uint8_t unlock;
   
} flag_t;

__FLAGCHECK_TASK_EXT flag_t flag;


void unlock_init(void);
void flag_check_task(void const *argu);
#endif

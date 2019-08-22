/**
  * @file     gimbal_task.h
  * @version  v2.0
  * @date     July,6th 2019
  *
  * @brief
  *
  *	@author   Fatmouse,part of the code reference li zh's code
  *
  */
#ifdef  __GIMBAL_TASK_GLOBALS
#define __GIMBAL_TASK_EXT
#else
#define __GIMBAL_TASK_EXT extern
#endif

#ifndef __GIMBAL_TASK_H__
#define __GIMBAL_TASK_H__

#include "stm32f4xx_hal.h"
#include "pid.h"

#define GIMBAL_PERIOD	1

#define yaw_range_angle 30
#define yaw_range_ecd  300
#define pit_range_ecd  600
#define pit_range_angle  15
typedef enum
{
	GIMBAL_REMOTER         = 0,
	GIMBAL_KEYBOARD        = 1,
	GIMBAL_VISION          = 2,
	GIMBAL_ENERGY          = 3,
}
gimbal_mode_e;
typedef enum
{
  PIT         = 0,
  YAW         = 1,
} gimbal_id_e;

typedef struct
{
  /* position loop ecd*/
  float yaw_ecd_ref;
  float pit_ecd_ref;
  float yaw_ecd_fdb;
  float pit_ecd_fdb;
  /* position loop angle*/
  float yaw_angle_ref;
  float pit_angle_ref;
  float yaw_angle_fdb;
  float pit_angle_fdb;
  /* speed loop */
  float yaw_spd_ref;
  float pit_spd_ref;
  float yaw_spd_fdb;
  float pit_spd_fdb;
	
  float pit_relative_ecd;	//�仯������
  float yaw_relative_ecd;
  /* unit: degree */
  float pit_relative_angle;
  float yaw_relative_angle;
	
  float pit_error_ecd;
  float yaw_error_ecd;
  /* unit: degree */
  float pit_error_angle;
  float yaw_error_angle;


} gim_pid_t;

typedef struct
{
//  float gyro_angle;
  /* uint: degree/s */
  float yaw_palstance;
  float pit_palstance;
} gim_sensor_t;

typedef struct
{
	gimbal_id_e id;
  /* ctrl mode */
  gimbal_mode_e ctrl_mode;

  /* gimbal information */
	int16_t         yaw_ecd_fdb;
	int16_t         yaw_ecd_ref;
  gim_sensor_t  sensor;
  float         ecd_offset_angle;
  float         yaw_offset_angle;
  
  /* gimbal ctrl parameter */
  gim_pid_t     pid;
	
  /* read from flash */
  int32_t       pit_center_offset;
  int32_t       yaw_center_offset;
  int16_t         current[3];  //yaw 0  pit  1  trigger  2

} gimbal_t;

extern gimbal_t gimbal;


void gimbal_task(void const *argu);
void gimbal_control(gimbal_mode_e gimbal_mode);
void normal_calcu(void);
void vision_calcu(pid_t pid_pit_vision,pid_t pid_yaw_vision);
void lock_calcu(void);
void gimbal_param_init(void);
#endif

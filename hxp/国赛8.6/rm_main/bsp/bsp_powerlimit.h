/** 
  * @file     bsp_powerlimit.h
  * @version  v2.0
  * @date     July,6th 2019
	*
  * @brief    功率控制驱动
	*
  *	@author   Fatmouse
  *
  */
	
#ifndef __BSP_POWERLIMIT_H

#define __BSP_POWERLIMIT_H

#include "stm32f4xx_hal.h"
#include "pid.h"

#define POWER_LIMIT 200

typedef enum
{
	NORMAL_LIMIT = 0x00,
	SUPERCAP_LIMIT = 0x01,
} limit_mode_e;

typedef enum
{
  POWER_LIMIT_ZERO   = 0x000,
  POWER_LIMIT_FORTY  = 0x040,
  POWER_LIMIT_EIGHTY = 0x076,
} power_limit_grade_e;

typedef enum
{
	POWER_LIMIT_ONE    = 1,
	POWER_LIMIT_FIVE   = 5,
	POWER_LIMIT_TEN    = 10,
  POWER_LIMIT_TWENTY = 20,
	POWER_LIMIT_40  = 40,
	POWER_LIMIT_60  = 60,
	POWER_LIMIT_80  = 80,
} power_limit_error_e;


typedef struct
{
	uint8_t limit_mode;
  uint16_t error;  //当前误差
	uint16_t grade;  //当前误差等级
	uint16_t limit;  //
	uint16_t supercap_power;      //超级电容模组反馈回来的实时充电功率
	uint16_t supercap_energy;     //超级电容模组反馈回来的实时能量百分比
	uint16_t chassis_power;//裁判系统反馈回来的底盘实时功率
  uint16_t chassis_power_buffer;//裁判系统反馈回来的底盘缓冲能量
	int MAX_WHEEL_RPM;
} power_control_t;

extern power_control_t power_control;	

extern int16_t chassis_current[4]; 
//extern int16_t chassis_cur_pid_out[4];

void power_limit_init(void);
void get_chassis_current(void);
void power_limit_control(power_control_t* power_control);
#endif

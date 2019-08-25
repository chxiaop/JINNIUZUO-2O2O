/**
  * @file     chassis_task.h
  * @version  v2.0
  * @date     July,6th 2019
  *
  * @brief  
  *
  *	@author   Fatmouse
  *
  */
#ifndef __CLASSIS_TASK_H__
#define __CLASSIS_TASK_H__

#include "stm32f4xx_hal.h"
#include "bsp_powerlimit.h"
//������������
#define CHASSIS_PERIOD 2

//ҡ������
#define DANCE_SPEED    40
#define DANCE_RANGE    900
#define power_control_limit_set 70

typedef enum
{
  chassis_control_ramp      = 0,
  chassis_supercap_control_ramp,
} last_control_way_e;
/* chassis parameter structure */

typedef struct
{
  float           vx; // forward/back
  float           vy; // left/right
  float           vw; // rotate
  
//  chassis_mode_e  ctrl_mode;

  int16_t         wheel_spd_fdb[4];//����ֵ
  int16_t         wheel_spd_ref[4];//Ŀ��ֵ
  int16_t         current[4];

	int16_t				current_fdb[4];	
	int16_t				current_fdb_sum;
	float					war_sum;
  int16_t         position_ref;
	float 					position_error;
	float           angle_error;
} chassis_t;

typedef struct
{
  uint16_t right_fdb;  //�Ҳ�
  uint16_t fwl_fdb;    //��ǰ��
  uint16_t fwr_fdb;    //��ǰ��
  uint16_t right_ref;  
  uint16_t fwl_ref;    
  uint16_t fwr_ref;    	


} tof_t;

extern chassis_t chassis;
extern tof_t tof;
void chassis_task(void const *argu);
void chassis_init(void);
void mecanum_calc(float vx, float vy, float vw, int16_t speed[],power_control_t* power_control);
void sparate_move(void);
void dance_move(void);
void chance_Top(void);
#endif

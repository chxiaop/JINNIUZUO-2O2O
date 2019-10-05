/** 
  * @file     math_calcu.h
  * @version  v2.0
  * @date     July,6th 2019
	*
  * @brief    ��ѧ���㺯��,б��������
	*
  *	@author   Fatmouse
  *
  */
#ifndef _MATH_CALCU_H_
#define _MATH_CALCU_H_

#include "arm_math.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

#define chassis_period 10.0f

extern float chassis_input;
extern float chassis_max;
typedef struct
{
//	float value;
//	float inital;
	float change_scale;
	float real_target;
	float limit_target;
	TickType_t ticks;
	TickType_t last_ticks;
//	TicksTypedef *SlopeTick;
}Slope_Struct;

void Slope_On(Slope_Struct *V);
typedef struct
{
    float input;        //��������
    float out;          //�������
    float min_value;    //�޷���Сֵ
    float max_value;    //�޷����ֵ
    float frame_period; //ʱ����
} ramp_function_source_t;

extern ramp_function_source_t chassis_x_ramp;
extern ramp_function_source_t chassis_y_ramp;
extern ramp_function_source_t chassis_super_x_ramp;
extern ramp_function_source_t chassis_super_y_ramp;
void ramp_calc(ramp_function_source_t *ramp_source_type, float frame_period, float input, float max, float min);
void chassis_ramp(void);
void chassis_super_ramp(void);
#endif

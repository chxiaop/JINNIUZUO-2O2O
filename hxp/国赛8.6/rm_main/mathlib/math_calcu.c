/** 
  * @file     math_calcu.c
  * @version  v2.0
  * @date     July,6th 2019
	*
  * @brief    ��ѧ���㺯��
	*
  *	@author   Fatmouse
  *
  */
#include "math_calcu.h"
#include "remote_msg.h"
float chassis_input;
float chassis_max;
ramp_function_source_t chassis_x_ramp;
ramp_function_source_t chassis_y_ramp;
ramp_function_source_t chassis_super_x_ramp;
ramp_function_source_t chassis_super_y_ramp;
/**
  * @brief          б���������㣬���������ֵ���е��ӣ� ���뵥λΪ /s ��һ������������ֵ
  * @author         RM
  * @param[in]      б�������ṹ��
  * @param[in]      ����ֵ
  * @param[in]      �˲�����
  * @retval         ���ؿ�
  */
void ramp_calc(ramp_function_source_t *ramp_source_type, float frame_period, float input, float max, float min)
{
	ramp_source_type->max_value = max;
	ramp_source_type->min_value = min;
	  ramp_source_type->frame_period = frame_period;

    ramp_source_type->input = input;

    ramp_source_type->out += ramp_source_type->input * ramp_source_type->frame_period;

    if (ramp_source_type->out > ramp_source_type->max_value)
    {
        ramp_source_type->out = ramp_source_type->max_value;
    }
    else if (ramp_source_type->out < ramp_source_type->min_value)
    {
        ramp_source_type->out = ramp_source_type->min_value;
    }
}

void chassis_ramp()
{
  if(rc.kb.bit.W)
	{
	  ramp_calc(&chassis_x_ramp,1.0f,chassis_input, chassis_max, 0.0f);			
	}
	else if(rc.kb.bit.S)
	{
		ramp_calc(&chassis_x_ramp,1.0f,-chassis_input, 0.0f, -chassis_max);			
	}
	else
	{
		if(chassis_x_ramp.out > 0)
		{
			ramp_calc(&chassis_x_ramp,1.0f,-chassis_input,chassis_max , 0.0f);			
		}
		else if(chassis_x_ramp.out < 0)
		{
			ramp_calc(&chassis_x_ramp,1.0f,chassis_input, 0.0f, -chassis_max);	
		}
	}
	if(rc.kb.bit.D)
	{
	  ramp_calc(&chassis_y_ramp,1.0f,chassis_input, chassis_max*0.6f, 0.0f);			
	}
	else if(rc.kb.bit.A)
	{
	  ramp_calc(&chassis_y_ramp,1.0f,-chassis_input, 0.0f, -chassis_max*0.6f);			
	}
	else
	{
		if(chassis_y_ramp.out > 0)
		{
			ramp_calc(&chassis_y_ramp,1.0f,-chassis_input, chassis_max, 0.0f);			
		}
		else if(chassis_y_ramp.out < 0)
		{
			ramp_calc(&chassis_y_ramp,1.0f,chassis_input, 0.0f, -chassis_max);	
		}
	}	
}



void Slope_On(Slope_Struct *V)
{
	V->last_ticks = V->ticks;
	V->ticks = osKernelSysTick();
	if(V->real_target !=V->limit_target)
	{
		if(V->real_target < V->limit_target)//�Ӳ���
		{
			V->real_target = V->real_target + V->change_scale* (V->ticks-V->last_ticks);
			//V->real_target = V->real_target + 0x20;
			if(V->real_target > V->limit_target)//�޷�
			{
				V->real_target =  V->limit_target;
			}
		}	
		else if(V->real_target > V->limit_target)//������
		{
			V->real_target =  V->real_target - V->change_scale* (V->ticks-V->last_ticks);
			if(V->real_target < V->limit_target)//�޷�
			{
				V->real_target =  (short)V->limit_target;
			}
		}
	}
}

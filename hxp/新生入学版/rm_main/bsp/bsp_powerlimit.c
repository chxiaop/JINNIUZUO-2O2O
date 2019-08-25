/** 
  * @file     bsp_powerlimit.c
  * @version  v2.0
  * @date     April,29th 2019
	*
  * @brief    ���ʿ����㷨
	*
  *	@author   Fatmouse
  *
  */
      
#include "bsp_powerlimit.h"
#include "bsp_can.h"
#include "cmsis_os.h"
#include "chassis_task.h"
#include "pid.h"
#include "control_def.h"
#include "judgement_info.h"

#define MATH_ABS(x)     ((x > 0) ? (x) : (-x))

power_control_t power_control;

int16_t chassis_current[4]; //���ʿ��ư巴���ĵ��̵��ĸ�ߵ���

// please extern in chassis file.
int16_t chassis_cur_pid_out[4];  //���̵���ĸ�߻����

/**
  * @brief  get the current of chassis
  * @param  void
  * @retval void
  * @note   put to the CAN RX_interrupt task.
  */
void get_chassis_current(void)
{
	int i;
	for (i = 0; i <4 ; i++)
	{
		chassis_current[i] = chassis.current_fdb[i];
		
		if (moto_chassis[i].speed_rpm < 0) 
			chassis_current[i] = -chassis_current[i];
	}
}


void power_limit_control(power_control_t* power_control)
{
	power_control->limit = power_control_limit_set + judge_recv_mesg.power_heat_data.chassis_power_buffer*0.5;
	power_control->chassis_power =judge_recv_mesg.power_heat_data.chassis_power;
	switch (power_control->limit_mode)
	{
		case NORMAL_LIMIT:
		{
			if(power_control->limit >= power_control->chassis_power - power_control->supercap_power)
			{
				power_control->error = power_control->limit - (power_control->chassis_power- power_control->supercap_power);
				//����������20��ʱ��
				if(power_control->error >= POWER_LIMIT_40)
				{
					power_control->MAX_WHEEL_RPM += 15;
				}
				else if(power_control->error >= POWER_LIMIT_TWENTY)
				{
					power_control->MAX_WHEEL_RPM += 15;
				}
				//���������10��20֮���ʱ��
				else if(power_control->error >= POWER_LIMIT_TEN)
				{
					power_control->MAX_WHEEL_RPM += 10;
				}
				//���������5��10֮���ʱ��
				else if(power_control->error >= POWER_LIMIT_FIVE)
				{
					power_control->MAX_WHEEL_RPM += 5;
				}
				//���������0��1֮���ʱ��
				else if(power_control->error > POWER_LIMIT_ONE)
				{
					power_control->MAX_WHEEL_RPM += 2;
				}
				//���������0��ʱ��
				else
				{
					power_control->MAX_WHEEL_RPM ++;
				}
			}
			else
			{
				power_control->error = (power_control->chassis_power- power_control->supercap_power) - power_control->limit;
				if(power_control->error >= POWER_LIMIT_80)
				{
					power_control->MAX_WHEEL_RPM -= 200;
				}
				else if(power_control->error >= POWER_LIMIT_60)
				{
					power_control->MAX_WHEEL_RPM -= 100;
				}
				else if(power_control->error >= POWER_LIMIT_40)
				{
					power_control->MAX_WHEEL_RPM -= 55;
				}
				else if(power_control->error >= POWER_LIMIT_TWENTY)
				{
					power_control->MAX_WHEEL_RPM -= 30;
				}
				else if(power_control->error >= POWER_LIMIT_TEN)
				{
					power_control->MAX_WHEEL_RPM -= 25;
				}
				else if(power_control->error >= POWER_LIMIT_FIVE)
				{
					power_control->MAX_WHEEL_RPM -= 10;
				}
				else if(power_control->error > POWER_LIMIT_ONE)
				{
					power_control->MAX_WHEEL_RPM -= 5;
				}
				else
				{
					power_control->MAX_WHEEL_RPM --;
				}
			}
		}break;
		case SUPERCAP_LIMIT:
		{
			if(power_control->limit >= power_control->chassis_power - power_control->supercap_power)
			{
				power_control->error = power_control->limit - (power_control->chassis_power- power_control->supercap_power);
				//����������20��ʱ��
				if(power_control->error >= POWER_LIMIT_40)
				{
					power_control->MAX_WHEEL_RPM += 15;
				}
				else if(power_control->error >= POWER_LIMIT_TWENTY)
				{
					power_control->MAX_WHEEL_RPM += 15;
				}
				//���������10��20֮���ʱ��
				else if(power_control->error >= POWER_LIMIT_TEN)
				{
					power_control->MAX_WHEEL_RPM += 10;
				}
				//���������5��10֮���ʱ��
				else if(power_control->error >= POWER_LIMIT_FIVE)
				{
					power_control->MAX_WHEEL_RPM += 5;
				}
				//���������0��1֮���ʱ��
				else if(power_control->error > POWER_LIMIT_ONE)
				{
					power_control->MAX_WHEEL_RPM += 2;
				}
				//���������0��ʱ��
				else
				{
					power_control->MAX_WHEEL_RPM ++;
				}
			}
			else
			{
				power_control->error = (power_control->chassis_power- power_control->supercap_power) - power_control->limit;
				if(power_control->error >= POWER_LIMIT_80)
				{
					power_control->MAX_WHEEL_RPM -= 200;
				}
				else if(power_control->error >= POWER_LIMIT_60)
				{
					power_control->MAX_WHEEL_RPM -= 100;
				}
				else if(power_control->error >= POWER_LIMIT_40)
				{
					power_control->MAX_WHEEL_RPM -= 55;
				}
				else if(power_control->error >= POWER_LIMIT_TWENTY)
				{
					power_control->MAX_WHEEL_RPM -= 30;
				}
				else if(power_control->error >= POWER_LIMIT_TEN)
				{
					power_control->MAX_WHEEL_RPM -= 25;
				}
				else if(power_control->error >= POWER_LIMIT_FIVE)
				{
					power_control->MAX_WHEEL_RPM -= 10;
				}
				else if(power_control->error > POWER_LIMIT_ONE)
				{
					power_control->MAX_WHEEL_RPM -= 5;
				}
				else
				{
					power_control->MAX_WHEEL_RPM --;
				}
			}
		}break;
		default:
		{
		}
	}
	//��ֹ�����ܷ�
	if(power_control->MAX_WHEEL_RPM>=14000)
	{
	  power_control->MAX_WHEEL_RPM = 14000;
	}
	else if(power_control->MAX_WHEEL_RPM < 0)
	{
	  power_control->MAX_WHEEL_RPM = 0;
	}
	judge_send_mesg.show_in_client_data.data1 = power_control->supercap_energy;
//	judge_send_mesg.show_in_client_data.data2 = judge_recv_mesg.power_heat_data.chassis_power;
}

void power_limit_init()
{ 
  //power_control.grade = POWER_LIMIT_FORTY;
	power_control.MAX_WHEEL_RPM = 1000;
	power_control.error = 0;
}


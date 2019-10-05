/** 
  * @file     chassis_task.c
  * @version  v2.0
  * @date     July,6th 2019
	*
  * @brief    ������̬�����ļ�
	*
  *	@author   fatmouse
  *
  */
#include "chassis_task.h"
#include "gimbal_task.h"
#include "cmsis_os.h"
#include "usart.h"
#include "comm_task.h"
#include "string.h"
#include "modeswitch_task.h"
#include "remote_msg.h"
#include "pid.h"
#include "stdlib.h"
#include "bsp_can.h"
#include "data_processing.h"
#include "bsp_powerlimit.h"
#include "math_calcu.h"
#include "math.h"
#include "control_def.h"
#include "visionfire_task.h"
#include "judge_send_task.h"
#include "judgement_info.h"
chassis_t chassis;
tof_t tof;
int8_t dance_flag = 1;
int16_t dance_error = 0;
uint8_t last_control_way;
int8_t Top_flag = 0 ;
uint8_t F_key_up = 1;
double retato_x;
extern uint8_t retato;
extern uint8_t dance_symbol;
extern TaskHandle_t can_msg_send_task_t;
extern send_judge_t    judge_send_mesg;

/**
  * @brief chassis_task
  * @param     
  * @attention  
	* @note  
  */
void chassis_task(void const *argu)
{

		
	switch(ctrl_mode)
	{
		case PROTECT_MODE:
		{
			taskENTER_CRITICAL();
				
			chassis.vx = 0;
			chassis.vy = 0;
			chassis.vw = 0;
				
			taskEXIT_CRITICAL();
			break;
		}
		case ENERGY_MODE:
		{
			chassis.vx = 0;
			chassis.vy = 0;
			chassis.vw = 0;
			break;
		}
		case VISION_MODE:
		case KEYBOARD_MODE:
		{
			if(supercap_control.supercap_switch == 0x01)//超级电容模式
			{
				chassis_input = 100.0f;
        chassis_max = 14000.0f;
				power_control.limit_mode = SUPERCAP_LIMIT; 
			}
			else
			{
				chassis_input = 50.0f;
        chassis_max = 6300.0f;
				power_control.limit_mode = NORMAL_LIMIT; 
			}
			chassis_ramp();
			//�����������
			chassis.angle_error =  chassis.position_error * (2.0f*PI/8191.0f);
			chassis.vx = (float)(chassis_x_ramp.out * cos(chassis.angle_error) + (-1.0f)*chassis_y_ramp.out * sin(chassis.angle_error));
			chassis.vy = (float)(chassis_x_ramp.out * sin(chassis.angle_error) - (-1.0f)*chassis_y_ramp.out * cos(chassis.angle_error));
			dance_move();
			chassis.position_ref = gimbal.yaw_center_offset +dance_error*dance_error_scale;
			last_control_way = chassis_control_ramp;
		
			chassis.position_error = circle_error(chassis.position_ref,moto_yaw.ecd,8191); 
      chance_Top();
			if(Top_flag == 0)
			{
			chassis.vw = -1.0f*pid_calc(&pid_chassis_angle,chassis.position_ref,chassis.position_ref + chassis.position_error);
			}
			else
			{
				chassis.vw = 8000.0f;
			}
      if (vision_msg.vision_mode == little_energy_mode || vision_msg.vision_mode == macro_energy_mode)
				{
				  chassis.vx = 0;
			    chassis.vy = 0;
			    chassis.vw = 0;
				}
     
			break;
		}		
		case CHASSIS_REMOTER_MODE:
		case REMOTER_MODE:
		{
				chassis.angle_error =  chassis.position_error * (2.0f*PI/8191.0f);
				chassis.vx = (float)(rc.ch2*rc_ch2_scale * cos(chassis.angle_error) + (-1.0f)*rc.ch1*rc_ch1_scale * sin(chassis.angle_error));
				chassis.vy = (float)(rc.ch2*rc_ch2_scale * sin(chassis.angle_error) - (-1.0f)*rc.ch1*rc_ch1_scale * cos(chassis.angle_error));
        chassis.position_error = circle_error(chassis.position_ref,moto_yaw.ecd,8191);
			if(retato == 1)
			{			
			chassis.vw = 4000;			
			}
			else
			{
			chassis.position_ref = gimbal.yaw_center_offset;
			chassis.position_error = circle_error(chassis.position_ref,moto_yaw.ecd,8191);
      chassis.vw = -1.0f*pid_calc(&pid_chassis_angle,chassis.position_ref,chassis.position_ref + chassis.position_error); 
			}
			break;
		}
		default:
		{
			break;
		}
	}		
	judge_send_mesg.show_in_client_data.data3 = Top_flag;
	power_limit_control(&power_control);
//	power_control.MAX_WHEEL_RPM = 9000;
	mecanum_calc(chassis.vx,chassis.vy, chassis.vw, chassis.wheel_spd_ref,&power_control);
	
	for (uint8_t i = 0; i < 4; i++)
  {
    chassis.wheel_spd_fdb[i] = moto_chassis[i].speed_rpm;
  }
	
	for (int i = 0; i < 4; i++)
	{
		chassis.current[i] = (int16_t)pid_calc(&pid_chassis_spd[i], chassis.wheel_spd_fdb[i], chassis.wheel_spd_ref[i]);
	}

	memcpy(motor_cur.chassis_cur, chassis.current, sizeof(chassis.current));
	
	osSignalSet(can_msg_send_task_t, CHASSIS_MOTOR_MSG_SEND);
}
/**
  * @brief ���ֽ��㺯��
  * @param input : ?=+vx(mm/s)  ?=+vy(mm/s)  ccw=+vw(deg/s)
  *        output: every wheel speed(rpm)
  * @note  1=FL 2=FR 3=BL 4=BR
  */
void mecanum_calc(float vx, float vy, float vw, int16_t speed[],power_control_t* power_control)
{
  int16_t wheel_rpm[4];
  float   max = 0;
  
  wheel_rpm[0] =     vx + vy - vw;
  wheel_rpm[1] = -1*(vx - vy + vw);
  wheel_rpm[2] =     vx - vy - vw;
  wheel_rpm[3] = -1*(vx + vy + vw);

	
	//find max item
  for (uint8_t i = 0; i < 4; i++)
  {
    if (abs(wheel_rpm[i]) > max)
      max = abs(wheel_rpm[i]);
  }
  //equal proportion
  if (max > power_control->MAX_WHEEL_RPM)
  {
    float rate = power_control->MAX_WHEEL_RPM / max;
    for (uint8_t i = 0; i < 4; i++)
      wheel_rpm[i] *= rate;
  }
	memcpy(speed, wheel_rpm, 4*sizeof(int16_t));
}


void chassis_init()
{
	retato_x = 0.7;
	//the four chassis motor pid
	for(int i=0; i<4; i++)
	{
		PID_struct_init(&pid_chassis_spd[i], POSITION_PID, 15000, 15000,
									3.0f,	0.00f,	0.0f	);  
	}
	PID_struct_init(&pid_chassis_angle, POSITION_PID, 4800, 500,
									10.0f,	0.0f,0.0f	);  
}
void sparate_move(void)
{
	 if(ABS(chassis.position_error) <= 300 ) 
	 chassis.position_ref = moto_yaw.ecd;		
	 if(chassis.position_error > 300 )
	 {	
	 chassis.position_ref = gimbal.yaw_center_offset - 300;
	 if(chassis.position_ref > 8191)
	 chassis.position_ref = chassis.position_ref - 8191;
	 }
		if(chassis.position_error < -300 )
	 {	
	 chassis.position_ref = gimbal.yaw_center_offset + 300;
	 if(chassis.position_ref < 0)
	 chassis.position_ref = chassis.position_ref + 8191;
	 }
	 chassis.position_error = circle_error(chassis.position_ref,moto_yaw.ecd,8191);
}
void dance_move(void)
{
	if(rc.kb.bit.CTRL)
	{
	  if(dance_flag == 1 && dance_error<30)
	  {
	    dance_error++;
	  }
	  else if(dance_flag == 0 && dance_error>-30)
	  {
	    dance_error--;
	  }
	  else
	  {
	  	if(dance_flag == 1)
	  	{
	  		dance_flag = 0;
	  	}
	  	else
		  {
	  	  dance_flag = 1;
	  	}
	  }
  }
	else
	{
		dance_error = 0;
	}
}


void chance_Top(void)
{
		if(rc.kb.bit.F && F_key_up) 
		{
			F_key_up = 0;
			Top_flag = ~Top_flag;	
		}
		else if(!rc.kb.bit.F)F_key_up = 1;
}

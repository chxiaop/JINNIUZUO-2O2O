/** 
  * @file     gimbal_task. c
  * @version  v2.0
  * @date     July,6th 2019
	*
  * @brief    ��̨
	*
  *	@author   Fatmouse,part of the code reference li zh's code
  *
  */
#define __GIMBAL_TASK_GLOBALS
#include "chassis_task.h"
#include "gimbal_task.h"
#include "status_task.h"
#include "mytype.h"
#include "cmsis_os.h"
#include "comm_task.h"
#include "string.h"
#include "modeswitch_task.h"
#include "remote_msg.h"
#include "pid.h"
#include "stdlib.h"
#include "bsp_can.h"
#include "bsp_JY901.h"
#include "data_processing.h"
#include "visionfire_task.h"
#include "control_def.h"
#include "bsp_macroenergy.h"
#include "bsp_TriggerMotor.h"

gimbal_t gimbal;
extern TaskHandle_t can_msg_send_task_t;
uint16_t pwm1=800,pwm2;
int16_t Befor,After;
void gimbal_param_init(void)
{
  memset(&gimbal, 0, sizeof(gimbal_t));
	
  /* pitch axis motor pid parameter */
  PID_struct_init(&pid_pit_angle, POSITION_PID, 500, 300,
                  0.0f, 0.0, 0.0); 
  PID_struct_init(&pid_pit_ecd, POSITION_PID, 5000, 300,
                  pid_pit_ecd_P, 0.0, 0.0); 
  PID_struct_init(&pid_pit_spd, POSITION_PID, 5000, 3500,
                  pid_pit_spd_P, pid_pit_spd_I, 0.0);
	
	PID_struct_init(&pid_pitch_aim, POSITION_PID,15000, 1000,
                  5.0f, 0.00005f, 0.0f);
		PID_struct_init(&pid_pitch_energy, POSITION_PID,15000, 1000,
                  4.0f, 0.0005f, 0.0f);
	
  /* yaw axis motor pid parameter */
  PID_struct_init(&pid_yaw_angle, POSITION_PID, 15000, 300,
                  pid_yaw_angle_P, pid_yaw_angle_I, pid_yaw_angle_D); 
  PID_struct_init(&pid_yaw_ecd, POSITION_PID, 500, 300,
                  0.8f, 0.0f, 1.0f); 
  PID_struct_init(&pid_yaw_spd, POSITION_PID, 28000, 5000,
											pid_yaw_spd_P, pid_yaw_spd_I, pid_yaw_spd_D);
	
	PID_struct_init(&pid_yaw_aim, POSITION_PID, 50000, 200,
                 100.0f, 0.01f, 0.0f);
	PID_struct_init(&pid_yaw_energy, POSITION_PID, 15000, 200,
                  100.0f, 0.1f, 0.0f);

	
	gimbal.pit_center_offset = gimbal_pit_center_offset;
	gimbal.yaw_center_offset = gimbal_yaw_center_offset;
	gimbal.yaw_ecd_ref = gimbal.yaw_center_offset;
	gimbal.pid.pit_ecd_ref = gimbal.pit_center_offset;
	gimbal.pid.yaw_angle_ref = 180.0f;
	
}

/**
  * @brief gimbal_task
  * @param     
  * @attention  
	* @note  
  */
void gimbal_task(void const *argu)
{
		//����ģʽ�ص����棬�Ե�ǰʵ��ֵΪĿ��ֵ
	if( ctrl_mode == PROTECT_MODE)
	{
		gimbal.pid.yaw_relative_ecd = 0;
		gimbal.pid.yaw_angle_ref = imu_gimbal.yaw;
	}
	switch(ctrl_mode)
	{
		case PROTECT_MODE:
		{
			 taskENTER_CRITICAL();
			
			gimbal.current[0] = 0;
			gimbal.current[1] = 0;
			gimbal.current[2] = 0;
			 taskEXIT_CRITICAL();

		}break;			
		case REMOTER_MODE:
		{
			 taskENTER_CRITICAL();
			
			gimbal_control(GIMBAL_REMOTER);
			
			 taskEXIT_CRITICAL();
		}break;		
		case KEYBOARD_MODE:
		{
			taskENTER_CRITICAL();

			if(rc.mouse.r == 1 && vision_msg.vision_status == lock_target_status)
			{
				if(vision_msg.vision_mode == aim_mode)
				{
					gimbal_control(GIMBAL_VISION);
				}
				else if (vision_msg.vision_mode == little_energy_mode || vision_msg.vision_mode == macro_energy_mode)
				{
				  gimbal_control(GIMBAL_ENERGY);
				}
			}
			else
			{
			    gimbal_control(GIMBAL_KEYBOARD);
				  vision_msg.aim_flag = 0;
				  vision_msg.aim_count = 0;
		  }

			 taskEXIT_CRITICAL();
		}break;		
		case ENERGY_MODE:
		{
			 taskENTER_CRITICAL();
			
			gimbal_control(GIMBAL_ENERGY);
			
			 taskEXIT_CRITICAL();
		}break;		
		case SEPARATE_MODE:	
		case DANCE_MODE:
		{
			taskENTER_CRITICAL();
			
      gimbal_control(GIMBAL_REMOTER);
			
			taskEXIT_CRITICAL();
		}break;
		case VISION_MODE:
		{
			 taskENTER_CRITICAL();
			
		  //gimbal_control(GIMBAL_VISION);
	//		gimbal.pid.yaw_angle_ref = 	gimbal.pid.yaw_angle_fdb;			
			 taskEXIT_CRITICAL();
		}
		default:
		{
		}break;
	}		

	last_ctrl_mode = ctrl_mode;
	memcpy(motor_cur.gimbal_cur, gimbal.current, sizeof(gimbal.current));
  osSignalSet(can_msg_send_task_t, GIMBAL_MOTOR_MSG_SEND);
}


void gimbal_control(gimbal_mode_e gimbal_mode)
{

  switch(gimbal_mode)
	{
	  case GIMBAL_REMOTER:
		{
			//pit ecd
			gimbal.pid.pit_relative_ecd += rc.ch4 * rc_ch4_scale;//ң�����ı����ֵ����
			//yaw angle
			gimbal.pid.yaw_angle_ref += rc.ch3 * rc_ch3_scale;	//�Ƕ��趨
			normal_calcu();
			//lock_calcu();
			TriggerMotor_control();
		}break;
		case GIMBAL_KEYBOARD:
		{	
			//pit ecd
			gimbal.pid.pit_relative_ecd += rc.mouse.y * -0.5f;//ң�����ı����ֵ����
			//yaw angle
			gimbal.pid.yaw_angle_ref += rc.mouse.x * -0.02f;	//�Ƕ��趨

			normal_calcu();
		}break;
		case GIMBAL_VISION:
		{
			//yaw angle
			vision_calcu(pid_pitch_aim,pid_yaw_aim);			
			gimbal.pid.pit_relative_ecd = moto_pit.ecd - gimbal.pit_center_offset;
			gimbal.pid.yaw_angle_ref = 	imu_gimbal.yaw;
		}
		break;
		case GIMBAL_ENERGY:
		{
		  vision_calcu(pid_pitch_energy,pid_yaw_energy);			
			gimbal.pid.pit_relative_ecd = moto_pit.ecd - gimbal.pit_center_offset;
			gimbal.pid.yaw_angle_ref = 	imu_gimbal.yaw;
		}
		break;
		default:
		{
		}break;
	}
	gimbal.current[0] = -1.0f*pid_yaw_spd.pos_out;	//�����������Զ�������ǽ��ٶ�
	gimbal.current[1] = -1.0f*pid_pit_spd.pos_out;
}

void normal_calcu()
{
	abs_limit(&gimbal.pid.pit_relative_ecd , pit_range_ecd , 0);	//�Ա���ֵ���������޷�
	gimbal.pid.pit_ecd_ref = gimbal.pid.pit_relative_ecd + gimbal.pit_center_offset;	//����+�е�=Ŀ��
//	if (gimbal.pid.pit_ecd_ref > 8191) gimbal.pid.pit_ecd_ref = gimbal.pid.pit_ecd_ref - 8191;
//	if ( gimbal.pid.pit_ecd_ref  <  gimbal_pit_limit_down)
//		gimbal.pid.pit_ecd_ref = gimbal_pit_limit_down;
	gimbal.pid.pit_error_ecd = circle_error(gimbal.pid.pit_ecd_ref,gimbal.pid.pit_ecd_fdb,8191);//����������
	gimbal.pid.pit_ecd_fdb = moto_pit.ecd;	//�������ֵ����
	pid_calc(&pid_pit_ecd, gimbal.pid.pit_ecd_fdb, gimbal.pid.pit_ecd_fdb + gimbal.pid.pit_error_ecd);//λ�û�
	//pit spd
	gimbal.pid.pit_spd_ref = pid_pit_ecd.pos_out; 	//λ�û����Ϊ�ٶȻ��趨
	//gimbal.pid.pit_spd_ref = 0; 
	gimbal.pid.pit_spd_fdb = gimbal.sensor.pit_palstance;	//�����ǽ��ٶȷ���
	pid_calc(&pid_pit_spd, gimbal.pid.pit_spd_fdb, gimbal.pid.pit_spd_ref);//�ٶȻ�
	
	if(gimbal.pid.yaw_angle_ref<0)gimbal.pid.yaw_angle_ref += 360;	//����������������
	else if(gimbal.pid.yaw_angle_ref>360)gimbal.pid.yaw_angle_ref-=360;
			
	float error_yaw_angle = circle_error(gimbal.pid.yaw_angle_ref,gimbal.pid.yaw_angle_fdb,360);//���㻷�����
	gimbal.pid.yaw_angle_fdb = imu_gimbal.yaw;		//�Ƕȷ���Ϊyaw�Ƕ�
	pid_calc(&pid_yaw_angle, gimbal.pid.yaw_angle_fdb, gimbal.pid.yaw_angle_fdb + error_yaw_angle);//�ǶȻ�
	//yaw spd
	gimbal.pid.yaw_spd_ref = pid_yaw_angle.pos_out;	//�ǶȻ����Ϊ�ٶȻ��趨
//	gimbal.pid.yaw_spd_ref = 100;
	gimbal.pid.yaw_spd_fdb = imu_gimbal.wz;	//�����ǽ��ٶȷ���
	pid_calc(&pid_yaw_spd, gimbal.pid.yaw_spd_fdb, gimbal.pid.yaw_spd_ref);	//�ٶȻ�
}
void vision_calcu(pid_t pid_pitch_vision,pid_t pid_yaw_vision)
{
	// offset_table(vision_msg.vision_distance);
	 KalmanFilter(imu_gimbal.yaw - vision_msg.vision_yaw/100.0f,kal_yaw.Q,kal_yaw.R);
	 kal_predict =  4.0f * (imu_gimbal.yaw - vision_msg.vision_yaw/100.0f - kal_yaw.x_now);
	kal_predict1 = kal_predict;
	 kal_predict = kal_predict > 5 ? 5:kal_predict;
	 kal_predict = kal_predict < -5 ? -5:kal_predict;
	
//   KalmanFilter(imu_gimbal.yaw - vision_msg.vision_yaw/100.0f,kal_yaw.Q,kal_yaw.R);
	// kal_predict =  3.5f * (imu_gimbal.yaw - vision_msg.vision_yaw/100.0f - kal_yaw.x_now);
//	 kal_predict = kal_predict > 3 ? 3:kal_predict;
//	 kal_predict = kal_predict < -3 ? -3:kal_predict;
  //if(vision_msg.vision_distance>= 0)
	pid_calc(&pid_pitch_vision,gimbal.pid.pit_ecd_fdb , gimbal.pid.pit_ecd_fdb - (vision_msg.vision_pitch * 22.75f)/100.0f );//�����Ӿ���
	gimbal.pid.pit_spd_ref = pid_pitch_vision.pos_out; 	//λ�û����Ϊ�ٶȻ��趨
	gimbal.pid.pit_spd_fdb = gimbal.sensor.pit_palstance;	//�����ǽ��ٶȷ���
	pid_calc(&pid_pit_spd, gimbal.pid.pit_spd_fdb, gimbal.pid.pit_spd_ref);//�ٶȻ�
  //if(vision_msg.vision_distance>= 0)
	gimbal.pid.yaw_angle_fdb = imu_gimbal.yaw;		//�Ƕȷ���Ϊyaw�Ƕ�
	
	if (vision_msg.vision_mode == aim_mode)
	{
	vision_msg.aim_count++;
	if(vision_msg.aim_count > 500)
	{
		vision_msg.aim_flag = 1;
		vision_msg.aim_count = 0;
	}
	pid_calc(&pid_yaw_vision, gimbal.pid.yaw_angle_fdb,  gimbal.pid.yaw_angle_fdb - vision_msg.vision_yaw/100.0f + kal_predict);
	Befor =  gimbal.pid.yaw_angle_fdb - vision_msg.vision_yaw/100.0f ;
	After =  gimbal.pid.yaw_angle_fdb - vision_msg.vision_yaw/100.0f +kal_predict;//�Ӿ���
	}
	else 
	{
		pid_calc(&pid_yaw_vision, gimbal.pid.yaw_angle_fdb,  gimbal.pid.yaw_angle_fdb - vision_msg.vision_yaw/100.0f);
  }
  gimbal.pid.yaw_spd_ref = pid_yaw_vision.pos_out;	//�ǶȻ����Ϊ�ٶȻ��趨
  gimbal.pid.yaw_spd_fdb = gimbal.sensor.yaw_palstance;	//�����ǽ��ٶȷ���
  pid_calc(&pid_yaw_spd, gimbal.pid.yaw_spd_fdb, gimbal.pid.yaw_spd_ref);	//�ٶȻ�
}
void lock_calcu()
{
	abs_limit(&gimbal.pid.pit_relative_ecd , pit_range_ecd , 0);	//�Ա���ֵ���������޷�
	gimbal.pid.pit_ecd_ref = gimbal.pid.pit_relative_ecd + gimbal.pit_center_offset;	//����+�е�=Ŀ��
	gimbal.pid.pit_error_ecd = circle_error(gimbal.pid.pit_ecd_ref,gimbal.pid.pit_ecd_fdb,8191);//����������
	gimbal.pid.pit_ecd_fdb = moto_pit.ecd;	//�������ֵ����
	pid_calc(&pid_pit_ecd, gimbal.pid.pit_ecd_fdb, gimbal.pid.pit_ecd_fdb + gimbal.pid.pit_error_ecd);//λ�û�
	//pit spd
	gimbal.pid.pit_spd_ref = pid_pit_ecd.pos_out; 	//λ�û����Ϊ�ٶȻ��趨
	//gimbal.pid.pit_spd_ref = 0; 
	gimbal.pid.pit_spd_fdb = gimbal.sensor.pit_palstance;	//�����ǽ��ٶȷ���
	pid_calc(&pid_pit_spd, gimbal.pid.pit_spd_fdb, gimbal.pid.pit_spd_ref);//�ٶȻ�
//	pid_calc(&pid_pit_spd, gimbal.pid.pit_spd_fdb, 0.0f);//�ٶȻ�			
	//gimbal.pid.yaw_relative_ecd = 0;	
	abs_limit(&gimbal.pid.yaw_relative_ecd , yaw_range_ecd , 0);	//�Ա���ֵ���������޷�
	gimbal.pid.yaw_ecd_ref = gimbal.pid.yaw_relative_ecd + gimbal.yaw_center_offset;	//����+�е�=Ŀ��
	gimbal.pid.yaw_error_ecd = circle_error(gimbal.pid.yaw_ecd_ref,gimbal.pid.yaw_ecd_fdb,8191);//����������
	gimbal.pid.yaw_ecd_fdb = moto_yaw.ecd;	//�������ֵ����
	pid_calc(&pid_yaw_ecd, gimbal.pid.yaw_ecd_fdb, gimbal.pid.yaw_ecd_fdb + gimbal.pid.yaw_error_ecd);//λ�û�
	//yaw spd
	gimbal.pid.yaw_spd_ref = pid_yaw_ecd.pos_out; 	//λ�û����Ϊ�ٶȻ��趨
//  gimbal.pid.yaw_spd_ref = 0; 
	gimbal.pid.yaw_spd_fdb = gimbal.sensor.yaw_palstance;	//�����ǽ��ٶȷ���
	pid_calc(&pid_yaw_spd, gimbal.pid.yaw_spd_fdb, gimbal.pid.yaw_spd_ref);//�ٶȻ�
}

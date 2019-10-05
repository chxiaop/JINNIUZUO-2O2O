/** 
  * @file control_def.h
  * @version 2.0
  * @date July,2nd 2019
  *
  * @brief  定义操作相关宏定义，将NO后面的数字改为对应步兵编码
  *
  * @author Fatmouse
  *
  */
#ifndef _CONTROL_DEF_H_
#define _CONTROL_DEF_H_

#include "stm32f4xx_hal.h"

#define NO5_DEF //更改参数~

#ifdef NO4_DEF //
/***********shoot************/
#define COVER_START 1200
#define COVER_END 2200
/***********chassis**********/
#define rc_ch2_scale 5
#define rc_ch1_scale 10
#define dance_error_scale 30
/***********gimbal***********/
#define rc_ch4_scale 0.025f
#define rc_ch3_scale -0.0005f

#define gimbal_pit_limit_down    1700
#define gimbal_pit_center_offset 2100
#define gimbal_yaw_center_offset 6800
/*********PID-Gimbal*********/
#define pid_pit_angle_P 0.0f
#define pid_pit_angle_I 0.0f
#define pid_pit_angle_D 0.0f

#define pid_pit_ecd_P 6.0f
#define pid_pit_ecd_I 0.0f
#define pid_pit_ecd_D 0.0f

#define pid_pit_spd_P 3.0f
#define pid_pit_spd_I 0.1f
#define pid_pit_spd_D 0.0f

#define pid_yaw_angle_P 200.0f
#define pid_yaw_angle_I 0.0f
#define pid_yaw_angle_D 0.0f

#define pid_yaw_ecd_P 0.0f
#define pid_yaw_ecd_I 0.0f
#define pid_yaw_ecd_D 0.0f



#define pid_yaw_spd_P 30.0f
#define pid_yaw_spd_I 0.1f
#define pid_yaw_spd_D 0.0f

#define pid_yaw_vision_P 0.0f
#define pid_yaw_vision_I 0.0f
#define pid_yaw_vision_D 0.0f

#define pid_pitch_vision_P 0.3f
#define pid_pitch_vision_I 0.00005f
#define pid_pitch_vision_D 0.0f
#endif
#ifdef NO5_DEF  
/***********shoot************/
#define COVER_START 1200
#define COVER_END 2200
/***********chassis**********/
#define rc_ch2_scale 5
#define rc_ch1_scale 10
#define dance_error_scale 30
/***********gimbal***********/
#define rc_ch4_scale 0.025f
#define rc_ch3_scale -0.0005f

#define gimbal_pit_limit_down    5700
#define gimbal_pit_center_offset 6200
#define gimbal_yaw_center_offset 6800
/*********PID-Gimbal*********/
#define pid_pit_angle_P 0.0f
#define pid_pit_angle_I 0.0f
#define pid_pit_angle_D 0.0f

#define pid_pit_ecd_P 2.5f
#define pid_pit_ecd_I 0.0f
#define pid_pit_ecd_D 0.0f

#define pid_pit_spd_P 20.0f
#define pid_pit_spd_I 0.03f
#define pid_pit_spd_D 0.0f

#define pid_yaw_angle_P 50.0f
#define pid_yaw_angle_I 0.0f
#define pid_yaw_angle_D 0.0f

#define pid_yaw_ecd_P 0.0f
#define pid_yaw_ecd_I 0.0f
#define pid_yaw_ecd_D 0.0f



#define pid_yaw_spd_P 50.0f
#define pid_yaw_spd_I 0.05f
#define pid_yaw_spd_D 0.0f

#define pid_yaw_vision_P 0.0f
#define pid_yaw_vision_I 0.0f
#define pid_yaw_vision_D 0.0f

#define pid_pitch_vision_P 0.f
#define pid_pitch_vision_I 0.0f
#define pid_pitch_vision_D 0.0f
#endif
#ifdef NO3_DEF
/***********shoot************/
#define COVER_START 500
#define COVER_END 1500
/***********chassis**********/
#define rc_ch2_scale 5
#define rc_ch1_scale 10
#define dance_error_scale 30
/***********gimbal***********/
#define rc_ch4_scale 0.025f
#define rc_ch3_scale -0.0005f

#define gimbal_pit_limit_down    7400
#define gimbal_pit_center_offset 7800
#define gimbal_yaw_center_offset 6838
/*********PID-Gimbal*********/
#define pid_pit_angle_P 0.0f
#define pid_pit_angle_I 0.0f
#define pid_pit_angle_D 0.0f

#define pid_pit_ecd_P 1.0f
#define pid_pit_ecd_I 0.0f
#define pid_pit_ecd_D 0.0f

#define pid_pit_spd_P 25.0f
#define pid_pit_spd_I 0.1f
#define pid_pit_spd_D 0.0f

#define pid_yaw_angle_P 25.0f
#define pid_yaw_angle_I 0.0f
#define pid_yaw_angle_D 70.0f

#define pid_yaw_ecd_P 0.0f
#define pid_yaw_ecd_I 0.0f
#define pid_yaw_ecd_D 0.0f

#define pid_yaw_spd_P 90.0f
#define pid_yaw_spd_I 0.3f
#define pid_yaw_spd_D 0.0f

#define pid_yaw_vision_P 0.0f
#define pid_yaw_vision_I 0.0f
#define pid_yaw_vision_D 0.0f

#define pid_pitch_vision_P 0.3f
#define pid_pitch_vision_I 0.00005f
#define pid_pitch_vision_D 0.0f
#endif
#endif

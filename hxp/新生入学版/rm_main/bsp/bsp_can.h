/** 
  * @file     bap_can.h
  * @version  v2.0
  * @date     July,6th 2019
  *
  * @brief  
  *
  * @author   Fatmouse
  *
  */
 
#ifndef _BSP_CAN_H_
#define _BSP_CAN_H_

#ifdef  __BSP_CAN_GLOBALS
#define __BSP_CAN_EXT
#else
#define __BSP_CAN_EXT extern
#endif
#include "bsp_JY901.h"
#include "can.h"
#include "comm_task.h"
#define CHASSIS_CAN   hcan1
#define GIMBAL_CAN   hcan1


/* CAN send and receive ID */
typedef enum
{
  CAN_3508_M1_ID       = 0x201,
  CAN_3508_M2_ID       = 0x202,
  CAN_3508_M3_ID       = 0x203,
  CAN_3508_M4_ID       = 0x204,
	
  CAN_YAW_MOTOR_ID     = 0x205,
  CAN_PIT_MOTOR_ID     = 0x206, 
	CAN_TRIGGER_MOTOR_ID = 0x207,
	
	CAN_POWER_ID			   = 0x301,	
	
	CAN_CHASSIS_ALL_ID   = 0x200,
  CAN_GIMBAL_ALL_ID    = 0x1ff,
	
	CAN_SUPERCAP_FDB_ID  = 0x303,
	CAN_SHOOT_FDB_ID     = 0x305,
} can_msg_id_e;

typedef struct
{
  uint8_t CAN_Tx_data[8];
} CAN_RecvMsg;

/* can receive motor parameter structure */
#define FILTER_BUF 5

typedef struct
{
  uint16_t ecd;
  uint16_t last_ecd;
  
  int16_t  speed_rpm;
  int16_t  given_current;

  int32_t  round_cnt;
  int32_t  total_ecd;
  int32_t  total_angle;
  
  uint16_t offset_ecd;
  uint32_t msg_cnt;
  
  int32_t  ecd_raw_rate;
  int32_t  rate_buf[FILTER_BUF];
  uint8_t  buf_cut;
  int32_t  filter_rate;
} moto_measure_t;

__BSP_CAN_EXT moto_measure_t moto_chassis[4] ;
__BSP_CAN_EXT moto_measure_t moto_pit;
__BSP_CAN_EXT moto_measure_t moto_yaw;
__BSP_CAN_EXT moto_measure_t motor_trigger;
extern CAN_RecvMsg chassis_can_tx_data;
extern CAN_RecvMsg gimbal_can_tx_data;
extern uint8_t Rx1Data[8];

void encoder_data_handler(moto_measure_t* ptr, CAN_HandleTypeDef* hcan);
void get_moto_offset(moto_measure_t* ptr, CAN_HandleTypeDef* hcan);

void can_device_init(void);
void my_can_filter_init_recv_all(void);

void send_gimbal_cur(int16_t yaw_iq, int16_t pit_iq, int16_t trigger_iq);
void send_chassis_cur(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
void send_supercap_message(supercap_message_t supercap_control);
void GYRO_data_handler_0(imu_typedef* ptr, CAN_HandleTypeDef* hcan);
void GYRO_data_handler_1(imu_typedef* ptr, CAN_HandleTypeDef* hcan);


#endif

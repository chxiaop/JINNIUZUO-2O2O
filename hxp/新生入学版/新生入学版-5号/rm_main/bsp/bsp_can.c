/**
	* @file     bap_can.c
	* @version  v2.0
	* @date     July,6th 2019
  *
  * @brief    底层can通讯的配置，包括发送数据帧，CAN滤波组，接受中断回调函数
  *
  *	@author   Fatmouse
  *
  */
#define __BSP_CAN_GLOBALS
#include "bsp_can.h"
#include "pid.h"
#include "bsp_JY901.h"
#include "chassis_task.h"
#include "status_task.h"
#include "bsp_powerlimit.h"
#include "string.h"
#include "bsp_imu.h"
CAN_RecvMsg chassis_can_tx_data;
CAN_RecvMsg gimbal_can_tx_data;
CAN_RecvMsg shoot_can_tx_data;
CAN_RecvMsg supercap_can_tx_data;
HAL_StatusTypeDef HAL_tx_Status;


CAN_TxHeaderTypeDef Tx1Message;
CAN_RxHeaderTypeDef Rx1Message;

CAN_TxHeaderTypeDef Tx2Message;

uint8_t CAN_Rx_data[8];
uint8_t CAN_Client_data[8];
uint16_t Power_Cotrl_cur[4];
uint8_t imu_can_data[8];

/**
  * @brief     CAN接受中断回调函数
  * @param     CAN_Rx_data ：CAN节点反馈的数据帧
  * @attention 
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&Rx1Message, CAN_Rx_data);
  switch (Rx1Message.StdId)
  {
		case CAN_3508_M1_ID:
    case CAN_3508_M2_ID:
    case CAN_3508_M3_ID:
    case CAN_3508_M4_ID:
    {
      static uint8_t i;
      i = Rx1Message.StdId - CAN_3508_M1_ID;
			moto_chassis[i].msg_cnt++ <= 50 ? get_moto_offset(&moto_chassis[i], &hcan1) : encoder_data_handler(&moto_chassis[i], &hcan1);
      status.chassis_status[i] = 1;
		}
    break;
    case CAN_YAW_MOTOR_ID:
    {
      encoder_data_handler(&moto_yaw, hcan);
			status.gimbal_status[0] = 1;
    }
    break;
    case CAN_PIT_MOTOR_ID:
    {
      encoder_data_handler(&moto_pit, hcan);
			status.gimbal_status[1] = 1;
    }
    break;
		case CAN_SUPERCAP_FDB_ID:
		{
		  power_control.supercap_energy = CAN_Rx_data[0] << 8 | CAN_Rx_data[1];
			power_control.supercap_power=  CAN_Rx_data[2] << 8 | CAN_Rx_data[3];
		}
		break;
		case CAN_TRIGGER_MOTOR_ID:
    {
			encoder_data_handler(&motor_trigger, hcan);
		}
    break;
//		case RM_IMU_PARAM_ID:
//		case RM_IMU_QUAT_ID:
//		case RM_IMU_GYRO_ID: 
//		case RM_IMU_ACCEL_ID:
//		case RM_IMU_MAG_ID: 
//		{
//			 memcpy(&imu_can_data,&CAN_Rx_data,8);
//			 imudata_decoding(Rx1Message.StdId,Rx1Message.DLC);
//		}

		default:
    {
    }
    break;
	};
	__HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
  * @brief     get motor initialize offset value
  * @param     ptr: Pointer to a moto_measure_t structure
  * @retval    None
  * @attention this function should be called after system can init
  */
void get_moto_offset(moto_measure_t* ptr, CAN_HandleTypeDef* hcan)
{
  ptr->ecd        = (uint16_t)(CAN_Rx_data[0] << 8 | CAN_Rx_data[1]);
  ptr->offset_ecd = ptr->ecd;
}

/**
  * @brief     get motor rpm and calculate motor round_count/total_encoder/total_angle
  * @param     ptr: Pointer to a moto_measure_t structure
  * @attention this function should be called after get_moto_offset() function
  */
void encoder_data_handler(moto_measure_t* ptr, CAN_HandleTypeDef* hcan)
{ 
	//转子转速
	ptr->speed_rpm     = (int16_t)(CAN_Rx_data[2] << 8 | CAN_Rx_data[3]);//6623无速度反馈
  ptr->given_current = (int16_t)(CAN_Rx_data[4] << 8 | CAN_Rx_data[5]);
	
	//机械角度
  ptr->last_ecd = ptr->ecd;
  ptr->ecd      = (uint16_t)(CAN_Rx_data[0] << 8 | CAN_Rx_data[1]);

  //相对开机后的角度
  if (ptr->ecd - ptr->last_ecd > 4096)
  {
    ptr->round_cnt--;
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd - 8192;
  }
  else if (ptr->ecd - ptr->last_ecd < -4096)
  {
    ptr->round_cnt++;
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd + 8192;
  }
  else
  {
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd;
  }

  ptr->total_angle = ptr->round_cnt * 8192 + ptr->ecd - ptr->offset_ecd;
  
}



/**
  * @brief   can filter initialization
  * @param   CAN_HandleTypeDef
  * @retval  None
  */
void my_can_filter_init_recv_all(void)
{
  //can1 filter config
  CAN_FilterTypeDef  can_filter;

	can_filter.FilterBank           = 0;
  can_filter.FilterMode           = CAN_FILTERMODE_IDMASK;
  can_filter.FilterScale          = CAN_FILTERSCALE_32BIT;
  can_filter.FilterIdHigh         = 0x0000;
  can_filter.FilterIdLow          = 0x0000;
  can_filter.FilterMaskIdHigh     = 0x0000;
  can_filter.FilterMaskIdLow      = 0x0000;
  can_filter.FilterFIFOAssignment = CAN_FilterFIFO0;
	can_filter.SlaveStartFilterBank = 0;  
  can_filter.FilterActivation     = ENABLE;

  HAL_CAN_ConfigFilter(&hcan1, &can_filter);
  while (HAL_CAN_ConfigFilter(&hcan1, &can_filter) != HAL_OK);
	
	can_filter.FilterBank           = 14;
  HAL_CAN_ConfigFilter(&hcan2, &can_filter);  
	while (HAL_CAN_ConfigFilter(&hcan2, &can_filter) != HAL_OK);
}

/**
  * @brief  send current which pid calculate to esc. message to calibrate 6025 gimbal motor esc
  * @param  current value corresponding motor(yaw/pitch/trigger)
  */
void send_gimbal_cur(int16_t yaw_iq, int16_t pit_iq, int16_t trigger_iq)
{
	uint8_t FreeTxNum = 0;  

  Tx2Message.StdId   = 0x1ff;
  Tx2Message.IDE     = CAN_ID_STD;
  Tx2Message.RTR     = CAN_RTR_DATA;
  Tx2Message.DLC     = 8;
  /* adding minus due to clockwise rotation of the gimbal motor with positive current */
  gimbal_can_tx_data.CAN_Tx_data[0] = -yaw_iq >> 8;
  gimbal_can_tx_data.CAN_Tx_data[1] = -yaw_iq;
  /* adding minus due to clockwise rotation of the gimbal motor with positive current */
  gimbal_can_tx_data.CAN_Tx_data[2] = -pit_iq >> 8;
  gimbal_can_tx_data.CAN_Tx_data[3] = -pit_iq;
  gimbal_can_tx_data.CAN_Tx_data[4] = trigger_iq >> 8;
  gimbal_can_tx_data.CAN_Tx_data[5] = trigger_iq;
  gimbal_can_tx_data.CAN_Tx_data[6] = 0;
  gimbal_can_tx_data.CAN_Tx_data[7] = 0;

	
	//查询发送邮箱是否为空
	FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);  
	while(FreeTxNum == 0) 
	{  
    FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);  
  }
	
	HAL_CAN_AddTxMessage(&GIMBAL_CAN, &Tx2Message,gimbal_can_tx_data.CAN_Tx_data,(uint32_t*)CAN_TX_MAILBOX0);

}

/**
  * @brief  send calculated current to motor
  * @param  3508 motor current
  */
void send_chassis_cur(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{
	uint8_t FreeTxNum = 0;  
	
	Tx1Message.StdId = 0x200;
	Tx1Message.IDE 	 = CAN_ID_STD;
	Tx1Message.RTR   = CAN_RTR_DATA;
  Tx1Message.DLC   = 0x08;
	
	chassis_can_tx_data.CAN_Tx_data[0] = iq1 >> 8;
	chassis_can_tx_data.CAN_Tx_data[1] = iq1;
	chassis_can_tx_data.CAN_Tx_data[2] = iq2 >> 8;
	chassis_can_tx_data.CAN_Tx_data[3] = iq2;
	chassis_can_tx_data.CAN_Tx_data[4] = iq3 >> 8;
	chassis_can_tx_data.CAN_Tx_data[5] = iq3;
	chassis_can_tx_data.CAN_Tx_data[6] = iq4 >> 8;
	chassis_can_tx_data.CAN_Tx_data[7] = iq4;
	
	//查询发送邮箱是否为空
	FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);  
	while(FreeTxNum == 0) 
	{  
    FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);  
  }
	
	HAL_CAN_AddTxMessage(&CHASSIS_CAN, &Tx1Message,chassis_can_tx_data.CAN_Tx_data,(uint32_t*)CAN_TX_MAILBOX0);
}

/**
  * @brief  send calculated current to motor
  * @param  3508 motor current
  */
void send_supercap_message(supercap_message_t supercap_control)
{
	uint8_t FreeTxNum = 0;  
	
	Tx1Message.StdId = 0x401;
	Tx1Message.IDE 	 = CAN_ID_STD;
	Tx1Message.RTR   = CAN_RTR_DATA;
  Tx1Message.DLC   = 0x08;
	
	supercap_can_tx_data.CAN_Tx_data[0] = supercap_control.supercap_mode;
	supercap_can_tx_data.CAN_Tx_data[1] = supercap_control.supercap_switch;
	supercap_can_tx_data.CAN_Tx_data[2] = 0;
	supercap_can_tx_data.CAN_Tx_data[3] = 0;
	supercap_can_tx_data.CAN_Tx_data[4] = 0;
	supercap_can_tx_data.CAN_Tx_data[5] = 0;
	supercap_can_tx_data.CAN_Tx_data[6] = 0;
	supercap_can_tx_data.CAN_Tx_data[7] = 0;
	
	//查询发送邮箱是否为空
	FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);  
	while(FreeTxNum == 0) 
	{  
    FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);  
  }
	
	HAL_CAN_AddTxMessage(&hcan1, &Tx1Message,supercap_can_tx_data.CAN_Tx_data,(uint32_t*)CAN_TX_MAILBOX0);
}


/**
  * @brief  init the can transmit and receive
  * @param  None
  */
void can_device_init(void)
{
  my_can_filter_init_recv_all();
	HAL_Delay(100);
	HAL_CAN_Start(&hcan1);
 	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_Start(&hcan2);
	HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING);
}

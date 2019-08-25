/** 
  * @file     bsp_uart.h
  * @version  v2.0
  * @date     July,6th 2019
	*
  * @brief    串口相关定义
	*
  *	@author   Fatmouse
  *
  */
#ifndef __BSP_UART_H__
#define __BSP_UART_H__

#ifdef  __BSP_UART_GLOBALS
#define __BSP_UART_EXT
#else
#define __BSP_UART_EXT extern
#endif

#include "usart.h"
#include "remote_msg.h"
#include "bsp_JY901.h"
#include "visionfire_task.h"

#define UART_RX_DMA_SIZE (1024)
#define UART_TX_DMA_SIZE (28)
#define DBUS_HUART       huart1 /* for dji remote controler reciever */
#define IMU_HUART        huart2
#define JUDGE_HUART      huart3
#define VISION_HUART     huart6


__BSP_UART_EXT uint8_t   dbus_buf[DBUS_BUFLEN];
__BSP_UART_EXT uint8_t   imu_buf[IMU_BUFLEN];
__BSP_UART_EXT uint8_t   vision_buf[VISION_BUFLEN];

void uart_receive_handler(UART_HandleTypeDef *huart);
void uart_init(void);
void USART_SendData(USART_TypeDef* USARTx, uint16_t Data);
void judgement_uart_init(void);
void uart_dma_full_signal(UART_HandleTypeDef *huart);
uint8_t dma_current_memory_target(DMA_Stream_TypeDef *dma_stream);
uint16_t dma_current_data_counter(DMA_Stream_TypeDef *dma_stream);

extern uint8_t judge_dma_rxbuff[2][UART_RX_DMA_SIZE];
extern uint8_t judge_dma_txbuff[UART_TX_DMA_SIZE];
#endif


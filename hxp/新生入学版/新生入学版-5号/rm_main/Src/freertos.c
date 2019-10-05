/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     
#include "stm32f4xx_hal.h"
#include "comm_task.h"
#include "chassis_task.h"
#include "modeswitch_task.h"
#include "gimbal_task.h"
#include "shoot_task.h"
#include "visionfire_task.h"
#include "status_task.h"
#include "supercap_task.h"
#include "judge_unpack_task.h"
#include "judgement_info.h"
#include "judge_send_task.h"
#include "flagCheck_task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osThreadId can_msg_send_task_t;    //CAN通信任务
osThreadId mode_sw_task_t;         //遥控检测任务
osThreadId vision_fire_t;
osThreadId status_task_t;
osThreadId flag_check_task_t;      //状态检测任务

TaskHandle_t judge_unpack_task_t;
osTimerId judge_sendTimer_id;
osTimerId chassis_timer_id;
osTimerId gimbal_timer_id;         //云台软件定时器
osTimerId shoot_timer_id;
osTimerId supercap_timer_id;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
			 taskENTER_CRITICAL();
		
   osTimerDef(chassisTimer, chassis_task);
  chassis_timer_id = osTimerCreate(osTimer(chassisTimer), osTimerPeriodic, NULL);
	
  osTimerDef(gimbalTimer, gimbal_task);
  gimbal_timer_id = osTimerCreate(osTimer(gimbalTimer), osTimerPeriodic, NULL);
	
	  osTimerDef(shootTimer, shoot_task);
  shoot_timer_id = osTimerCreate(osTimer(shootTimer), osTimerPeriodic, NULL);
	
		  osTimerDef(supercaptimer, supercap_task);
  supercap_timer_id = osTimerCreate(osTimer(supercaptimer), osTimerPeriodic, NULL);
	 
	 osTimerDef(judge_sendTimer, judge_send_task);
  judge_sendTimer_id = osTimerCreate(osTimer(judge_sendTimer), osTimerPeriodic, NULL);
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
			 /* high priority task */
		
	 osThreadDef(canTask, can_msg_send_task, osPriorityAboveNormal, 0, 256);
  can_msg_send_task_t = osThreadCreate(osThread(canTask), NULL);
	
	 osThreadDef(flagTask, flag_check_task, osPriorityAboveNormal, 0, 256);
  flag_check_task_t = osThreadCreate(osThread(flagTask), NULL);

   osThreadDef(visionTask, vision_fire_task, osPriorityAboveNormal, 0, 256);
  osThreadId vision_fire_t = osThreadCreate(osThread(visionTask), NULL);
		
	 	 /* low priority task */
	  osThreadDef(unpackTask, judge_unpack_task, osPriorityNormal, 0, 256);
  judge_unpack_task_t = osThreadCreate(osThread(unpackTask), NULL);
	
	 osThreadDef(modeTask, mode_switch_task, osPriorityNormal, 0, 256);
	mode_sw_task_t = osThreadCreate(osThread(modeTask), NULL);
	
	 osThreadDef(statusTask, status_task, osPriorityLow, 0, 256);
	status_task_t = osThreadCreate(osThread(statusTask), NULL);

	 		taskEXIT_CRITICAL();
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(100);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

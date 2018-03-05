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
  * Copyright (c) 2018 STMicroelectronics International N.V. 
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     
#include "chassis_task.h"
#include "shoot_task.h"
#include "gimbal_task.h"
#include "detect_task.h"
#include "record_task.h"
#include "imu_task.h"
#include "comm_task.h"
#include "modeswitch_task.h"
#include "info_get_task.h"
#include "bsp_uart.h"
#include "sys_config.h"
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;

/* USER CODE BEGIN Variables */
TaskHandle_t shoot_task_t;
TaskHandle_t can_msg_send_task_t;

TaskHandle_t mode_sw_task_t;
TaskHandle_t info_get_task_t;
TaskHandle_t detect_task_t;
TaskHandle_t imu_task_t;

TaskHandle_t freq_info_task_t;
TaskHandle_t judge_unpack_task_t;
TaskHandle_t pc_unpack_task_t;

osTimerId chassis_timer_id;
osTimerId gimbal_timer_id;

volatile int stack_over_flow_warning = 0;
/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);

extern void MX_FATFS_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 4 */
__weak void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
#if 0
    if (xTask == chassis_task_t)
      stack_over_flow_warning = 1;
    else if (xTask == gimbal_task_t)
      stack_over_flow_warning = 2;
    else if (xTask == detect_task_t)
      stack_over_flow_warning = 3;
    else if (xTask == record_task_t)
      stack_over_flow_warning = 4;
    else if (xTask == imu_task_t)
      stack_over_flow_warning = 5;
    else if (xTask == freq_info_task_t)
      stack_over_flow_warning = 6;
    else if (xTask == mode_sw_task_t)
      stack_over_flow_warning = 7;
    else if (xTask == info_get_task_t)
      stack_over_flow_warning = 8;
    else
      stack_over_flow_warning = 9;
#endif
    while (1)
    {
    }
}
/* USER CODE END 4 */

/* Init FreeRTOS */

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
  
    /* real time control task */
  taskENTER_CRITICAL();
  
    osTimerDef(chassisTimer, chassis_task);
    chassis_timer_id = osTimerCreate(osTimer(chassisTimer), osTimerPeriodic, NULL);
    
    osTimerDef(gimTimer, gimbal_task);
    gimbal_timer_id = osTimerCreate(osTimer(gimTimer), osTimerPeriodic, NULL);
  
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  
    /* high priority task */
    osThreadDef(shotTask, shoot_task, osPriorityAboveNormal, 0, 128);
    shoot_task_t = osThreadCreate(osThread(shotTask), NULL);
  
    osThreadDef(canTask, can_msg_send_task, osPriorityAboveNormal, 0, 128);
    can_msg_send_task_t = osThreadCreate(osThread(canTask), NULL);
    
    /* low priority task */
    osThreadDef(modeTask, mode_switch_task, osPriorityNormal, 0, 128);
    mode_sw_task_t = osThreadCreate(osThread(modeTask), NULL);
    
    osThreadDef(infoTask, info_get_task, osPriorityNormal, 0, 128);
    info_get_task_t = osThreadCreate(osThread(infoTask), NULL);
    
    osThreadDef(errTask, detect_task, osPriorityNormal, 0, 128);
    detect_task_t = osThreadCreate(osThread(errTask), NULL);

    osThreadDef(imuTask, imu_task, osPriorityNormal, 0, 128);
    imu_task_t = osThreadCreate(osThread(imuTask), NULL);
    
    /* unpack task */
    osThreadDef(unpackTask, judge_unpack_task, osPriorityNormal, 0, 512);
    judge_unpack_task_t = osThreadCreate(osThread(unpackTask), NULL);
    
    osThreadDef(pcunpackTask, pc_unpack_task, osPriorityNormal, 0, 512);
    pc_unpack_task_t = osThreadCreate(osThread(pcunpackTask), NULL);
    
  taskEXIT_CRITICAL();
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{
  /* init code for FATFS */
  MX_FATFS_Init();

  /* USER CODE BEGIN StartDefaultTask */
  //osThreadResume(record_task_t);
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Application */
//TaskHandle_t chassis_task_t;
//TaskHandle_t gimbal_task_t;
//TaskHandle_t record_task_t;

//    osThreadDef(chassisTask, chassis_task, osPriorityAboveNormal, 0, 128);
//    chassis_task_t = osThreadCreate(osThread(chassisTask), NULL);

//    osThreadDef(gimbalTask, gimbal_task, osPriorityAboveNormal, 0, 128);
//    gimbal_task_t = osThreadCreate(osThread(gimbalTask), NULL);

//    osThreadDef(rcdTask, record_task, osPriorityNormal, 0, 512);
//    record_task_t = osThreadCreate(osThread(rcdTask), NULL);
//    
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "can.h"
#include "robot_arm.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
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

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for led_running_tas */
osThreadId_t led_running_tasHandle;
const osThreadAttr_t led_running_tas_attributes = {
  .name = "led_running_tas",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for CAN_Rx_Task */
osThreadId_t CAN_Rx_TaskHandle;
const osThreadAttr_t CAN_Rx_Task_attributes = {
  .name = "CAN_Rx_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityRealtime3,
};
/* Definitions for CAN_Tx_Task */
osThreadId_t CAN_Tx_TaskHandle;
const osThreadAttr_t CAN_Tx_Task_attributes = {
  .name = "CAN_Tx_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityRealtime2,
};
/* Definitions for UART_Task */
osThreadId_t UART_TaskHandle;
const osThreadAttr_t UART_Task_attributes = {
  .name = "UART_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityRealtime1,
};
/* Definitions for CMD_Queue */
osMessageQueueId_t CMD_QueueHandle;
const osMessageQueueAttr_t CMD_Queue_attributes = {
  .name = "CMD_Queue"
};
/* Definitions for Status_Queue */
osMessageQueueId_t Status_QueueHandle;
const osMessageQueueAttr_t Status_Queue_attributes = {
  .name = "Status_Queue"
};
/* Definitions for servo_rx_sem */
osSemaphoreId_t servo_rx_semHandle;
const osSemaphoreAttr_t servo_rx_sem_attributes = {
  .name = "servo_rx_sem"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void led_running(void *argument);
void CAN_Rx_task(void *argument);
void CAN_Tx_task(void *argument);
void UART_task(void *argument);

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

  /* Create the semaphores(s) */
  /* creation of servo_rx_sem */
  servo_rx_semHandle = osSemaphoreNew(1, 1, &servo_rx_sem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  while (osSemaphoreAcquire(servo_rx_semHandle, 0) == osOK) { }
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of CMD_Queue */
  CMD_QueueHandle = osMessageQueueNew (16, sizeof(CmdMsg_t), &CMD_Queue_attributes);

  /* creation of Status_Queue */
  Status_QueueHandle = osMessageQueueNew (16, sizeof(ServoMsg_t), &Status_Queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of led_running_tas */
  led_running_tasHandle = osThreadNew(led_running, NULL, &led_running_tas_attributes);

  /* creation of CAN_Rx_Task */
  CAN_Rx_TaskHandle = osThreadNew(CAN_Rx_task, NULL, &CAN_Rx_Task_attributes);

  /* creation of CAN_Tx_Task */
  CAN_Tx_TaskHandle = osThreadNew(CAN_Tx_task, NULL, &CAN_Tx_Task_attributes);

  /* creation of UART_Task */
  UART_TaskHandle = osThreadNew(UART_task, NULL, &UART_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_led_running */
/**
* @brief Function implementing the led_running_tas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_led_running */
void led_running(void *argument)
{
  /* USER CODE BEGIN led_running */
  /* Infinite loop */
  for(;;)
  {
    HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
    osDelay(300);
  }
  /* USER CODE END led_running */
}

/* USER CODE BEGIN Header_CAN_Rx_task */
/**
* @brief Function implementing the CAN_Rx_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CAN_Rx_task */
void CAN_Rx_task(void *argument)
{
  /* USER CODE BEGIN CAN_Rx_task */
  CAN_Frame_t frame;
  CmdMsg_t cmd;
  /* Infinite loop */
  for(;;)
  {
    if(CAN_GetRxFrame(&frame) == 0){
      osDelay(1);
      continue;
    }
    if(frame.id != CAN_ID_CMD_DOWN) continue;

    if(frame.len < 6)continue;

    if(frame.data[0] > SEVRO_NUMBER) continue;

    cmd.id  = frame.data[0];
    cmd.ask = frame.data[1];       
    cmd.target_pos = (uint16_t)(frame.data[2] | (frame.data[3] << 8));
    cmd.time       = (uint16_t)(frame.data[4] | (frame.data[5] << 8));

    if (osMessageQueuePut(CMD_QueueHandle, &cmd, 0U, 0U) != osOK)
    {
      //cmd_drop++;
    }
    
  }
  /* USER CODE END CAN_Rx_task */
}

/* USER CODE BEGIN Header_CAN_Tx_task */
/**
* @brief Function implementing the CAN_Tx_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CAN_Tx_task */
void CAN_Tx_task(void *argument)
{
  /* USER CODE BEGIN CAN_Tx_task */
  ServoMsg_t *st = malloc(sizeof(ServoMsg_t));
  uint8_t pkg[8];
  /* Infinite loop */
  for(;;)
  {
    if(osMessageQueueGet(Status_QueueHandle,st,NULL,portMAX_DELAY) == osOK){

      memset(pkg,0,sizeof(pkg));
      CAN_Servo_status_send_pkg(st,pkg);
      CAN_Send(GETANGLE,(uint8_t*)pkg,8);
    
    }
    osDelay(1);
  }
  /* USER CODE END CAN_Tx_task */
}

/* USER CODE BEGIN Header_UART_task */
/**
* @brief Function implementing the UART_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UART_task */
void UART_task(void *argument)
{
  /* USER CODE BEGIN UART_task */
  ServoMsg_t *st = malloc(sizeof(ServoMsg_t));
  CmdMsg_t cmd;
  double _temp_v[2];
  memset(_temp_v,0,sizeof(_temp_v));
  /* Infinite loop */
  for(;;)
  {
    if(osMessageQueueGet(CMD_QueueHandle,&cmd,NULL,portMAX_DELAY) == osOK){
      switch(cmd.ask){
        case ASKFORSETPOS:{
          Servo_position_set(cmd.id,cmd.target_pos,cmd.time);
          break;
        }
        case ASKFORSTATUS:{
          st->id = cmd.id;
          Servo_temp_and_v_get(cmd.id,_temp_v);
          st->temp = _temp_v[0];
          st->volt = _temp_v[1];
          st->cur_pos = Servo_position_get(cmd.id);
          osMessageQueuePut(Status_QueueHandle,st,0,0);
          break;
        }
        case ASKFORALL:{
          Servo_position_set(cmd.id,cmd.target_pos,cmd.time);
          Servo_temp_and_v_get(cmd.id,_temp_v);
          st->temp = _temp_v[0];
          st->volt = _temp_v[1];
          st->cur_pos = Servo_position_get(cmd.id);
          osMessageQueuePut(Status_QueueHandle,st,0,0);
          break;
        }
        default : break;
      }
      osDelay(1);
      continue;
    }
    
    osDelay(1);
  }
  /* USER CODE END UART_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */


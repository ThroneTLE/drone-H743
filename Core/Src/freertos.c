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
#include "FreeRTOS.h"
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_tasks.h"

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
/* Definitions for LEDtestTask */
osThreadId_t LEDtestTaskHandle;
const osThreadAttr_t LEDtestTask_attributes = {
  .name = "LEDtestTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for imuTask */
osThreadId_t imuTaskHandle;
const osThreadAttr_t imuTask_attributes = {
  .name = "imuTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for messageTask */
osThreadId_t messageTaskHandle;
const osThreadAttr_t messageTask_attributes = {
  .name = "messageTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for UARTTask */
osThreadId_t UARTTaskHandle;
const osThreadAttr_t UARTTask_attributes = {
  .name = "UARTTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for uartTxQueue */
osMessageQueueId_t uartTxQueueHandle;
const osMessageQueueAttr_t uartTxQueue_attributes = {
  .name = "uartTxQueue"
};
/* Definitions for imuSampleQueue */
osMessageQueueId_t imuSampleQueueHandle;
const osMessageQueueAttr_t imuSampleQueue_attributes = {
  .name = "imuSampleQueue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void LEDTask(void *argument);
void imu_read(void *argument);
void message_push(void *argument);
void UART_fun(void *argument);

extern void MX_USB_DEVICE_Init(void);
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
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of uartTxQueue */
  uartTxQueueHandle = osMessageQueueNew (32, sizeof(APP_UART_TxMessage), &uartTxQueue_attributes);

  /* creation of imuSampleQueue */
  imuSampleQueueHandle = osMessageQueueNew (4, sizeof(APP_IMU_SampleMessage), &imuSampleQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of LEDtestTask */
  LEDtestTaskHandle = osThreadNew(LEDTask, NULL, &LEDtestTask_attributes);

  /* creation of imuTask */
  imuTaskHandle = osThreadNew(imu_read, NULL, &imuTask_attributes);

  /* creation of messageTask */
  messageTaskHandle = osThreadNew(message_push, NULL, &messageTask_attributes);

  /* creation of UARTTask */
  UARTTaskHandle = osThreadNew(UART_fun, NULL, &UARTTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_LEDTask */
/**
  * @brief  Function implementing the LEDtestTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_LEDTask */
void LEDTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN LEDTask */
  APP_Task_LED_Init();
  /* Infinite loop */
  for(;;)
  {
    APP_Task_LED_Step();
  }
  /* USER CODE END LEDTask */
}

/* USER CODE BEGIN Header_imu_read */
/**
* @brief Function implementing the imuTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_imu_read */
void imu_read(void *argument)
{
  /* USER CODE BEGIN imu_read */
  APP_Task_IMU_Init();
  /* Infinite loop */
  for(;;)
  {
    APP_Task_IMU_Step();
  }
  /* USER CODE END imu_read */
}

/* USER CODE BEGIN Header_message_push */
/**
* @brief Function implementing the messageTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_message_push */
void message_push(void *argument)
{
  /* USER CODE BEGIN message_push */
  APP_Task_Message_Init();
  /* Infinite loop */
  for(;;)
  {
    APP_Task_Message_Step();
  }
  /* USER CODE END message_push */
}

/* USER CODE BEGIN Header_UART_fun */
/**
* @brief Function implementing the UARTTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UART_fun */
void UART_fun(void *argument)
{
  /* USER CODE BEGIN UART_fun */
  APP_Task_UART_Init();
  /* Infinite loop */
  for(;;)
  {
    APP_Task_UART_Step();
  }
  /* USER CODE END UART_fun */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */


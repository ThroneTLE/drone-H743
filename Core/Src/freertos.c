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
#include "app_diag.h"
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
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for UARTTask */
osThreadId_t UARTTaskHandle;
const osThreadAttr_t UARTTask_attributes = {
  .name = "UARTTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for backgroundTask */
osThreadId_t backgroundTaskHandle;
const osThreadAttr_t backgroundTask_attributes = {
  .name = "backgroundTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
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
/* Definitions for backgroundReqQueue */
osMessageQueueId_t backgroundReqQueueHandle;
const osMessageQueueAttr_t backgroundReqQueue_attributes = {
  .name = "backgroundReqQueue"
};
/* Definitions for backgroundRespQueue */
osMessageQueueId_t backgroundRespQueueHandle;
const osMessageQueueAttr_t backgroundRespQueue_attributes = {
  .name = "backgroundRespQueue"
};
/* Definitions for flashBusMutex */
osMutexId_t flashBusMutexHandle;
const osMutexAttr_t flashBusMutex_attributes = {
  .name = "flashBusMutex",
  .attr_bits = osMutexRecursive,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void LEDTask(void *argument);
void imu_read(void *argument);
void message_push(void *argument);
void UART_fun(void *argument);
void BackgroundTask(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
   (void)xTask;
   APP_Diag_RecordStackOverflow(pcTaskName);
   taskDISABLE_INTERRUPTS();
   for(;;)
   {
   }
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
void vApplicationMallocFailedHook(void)
{
   /* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created. It is also called by various parts of the
   demo application. If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
   APP_Diag_RecordMallocFailed();
   taskDISABLE_INTERRUPTS();
   for(;;)
   {
   }
}
/* USER CODE END 5 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Create the recursive mutex(es) */
  /* creation of flashBusMutex */
  flashBusMutexHandle = osMutexNew(&flashBusMutex_attributes);

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

  /* creation of backgroundReqQueue */
  backgroundReqQueueHandle = osMessageQueueNew (8, sizeof(APP_BackgroundRequest), &backgroundReqQueue_attributes);

  /* creation of backgroundRespQueue */
  backgroundRespQueueHandle = osMessageQueueNew (8, sizeof(APP_BackgroundResponse), &backgroundRespQueue_attributes);

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

  /* creation of backgroundTask */
  backgroundTaskHandle = osThreadNew(BackgroundTask, NULL, &backgroundTask_attributes);

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

/* USER CODE BEGIN Header_BackgroundTask */
/**
* @brief Function implementing the backgroundTask thread.
* @param argument: Not used
* @retval None
*
* 功能：后台低优先级任务。
* 这个任务负责执行不应该阻塞控制环的慢操作，例如参数从 FLASH 自动加载、
* 参数异步保存、维护模式 FLASH 块读写。它是任务执行体；真正的数据规则
* 放在 Services 层的 Param/Background API 中。
*/
/* USER CODE END Header_BackgroundTask */
void BackgroundTask(void *argument)
{
  /* USER CODE BEGIN BackgroundTask */
  APP_Task_Background_Init();
  /* Infinite loop */
  for(;;)
  {
    APP_Task_Background_Step();
  }
  /* USER CODE END BackgroundTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */


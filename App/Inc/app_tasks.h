#ifndef APP_TASKS_H
#define APP_TASKS_H

#include "cmsis_os2.h"

#include "app_background.h"
#include "app_messages.h"
#include "rtos_objects.h"

extern osMessageQueueId_t uartTxQueueHandle;
extern osMessageQueueId_t imuSampleQueueHandle;
extern osMessageQueueId_t backgroundReqQueueHandle;
extern osMessageQueueId_t backgroundRespQueueHandle;
extern osThreadId_t LEDtestTaskHandle;
extern osThreadId_t imuTaskHandle;
extern osThreadId_t messageTaskHandle;
extern osThreadId_t UARTTaskHandle;
extern osThreadId_t backgroundTaskHandle;

void APP_Task_LED_Init(void);
void APP_Task_LED_Step(void);
void APP_Task_IMU_Init(void);
void APP_Task_IMU_Step(void);
void APP_Task_GPS_Init(void);
void APP_Task_GPS_Step(void);
void APP_Task_MAG_Init(void);
void APP_Task_MAG_Step(void);
void APP_Task_Message_Init(void);
void APP_Task_Message_Step(void);
void APP_Task_UART_Init(void);
void APP_Task_UART_Step(void);
void APP_Task_MaintUART_Init(void);
void APP_Task_MaintUART_Step(void);
void APP_Task_Background_Init(void);
void APP_Task_Background_Step(void);

#endif

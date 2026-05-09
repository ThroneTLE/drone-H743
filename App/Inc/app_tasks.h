#ifndef APP_TASKS_H
#define APP_TASKS_H

#include "cmsis_os2.h"

#include "app_messages.h"

extern osMessageQueueId_t uartTxQueueHandle;
extern osMessageQueueId_t imuSampleQueueHandle;
extern osThreadId_t UARTTaskHandle;

void APP_Task_LED_Init(void);
void APP_Task_LED_Step(void);
void APP_Task_IMU_Init(void);
void APP_Task_IMU_Step(void);
void APP_Task_Message_Init(void);
void APP_Task_Message_Step(void);
void APP_Task_UART_Init(void);
void APP_Task_UART_Step(void);
void APP_Task_MaintUART_Init(void);
void APP_Task_MaintUART_Step(void);

#endif

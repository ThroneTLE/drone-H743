#ifndef APP_TASKS_H
#define APP_TASKS_H

#include "cmsis_os2.h"

#include "app_background.h"
#include "app_messages.h"
#include "rtos_objects.h"

extern osMessageQueueId_t uartTxQueueHandle;
extern osMessageQueueId_t SensorSampleQueueHandle;
extern osMessageQueueId_t backgroundReqQueueHandle;
extern osMessageQueueId_t backgroundRespQueueHandle;
extern osThreadId_t StabilizerHandle;
extern osThreadId_t SensorTaskHandle;
extern osThreadId_t messageTaskHandle;
extern osThreadId_t UARTTaskHandle;
extern osThreadId_t backgroundTaskHandle;
extern osSemaphoreId_t imuDataReadySemaphore;
extern volatile uint8_t vofaStreamActive;

void APP_Task_LED_Init(void);
void APP_Task_LED_Step(void);
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

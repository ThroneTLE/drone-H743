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
/*
 * 遥测任务。以前没在这里 extern，于是 `RTOS?` 的任务栈报告里没有它——
 * 2026-09-11 排查"stream=1 却一个字节都不来"时，恰恰因为看不到这个任务的
 * 存活与栈余量，多花了好几轮。诊断要能看见每一个会卡住的任务。
 */
extern osThreadId_t VOFA_TaskHandle;
extern osSemaphoreId_t imuDataReadySemaphore;
extern volatile uint8_t vofaStreamActive;

void APP_Task_LED_Init(void);
void APP_Task_LED_Step(void);
void APP_Task_GPS_Init(void);
void APP_Task_GPS_Step(void);
void APP_Task_OpticalFlow_Init(void);
void APP_Task_OpticalFlow_Step(void);
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

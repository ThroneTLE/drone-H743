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
/*
 * 状态灯自己起一个 1 kHz 的低优先级任务，线程体也在 App 侧——freertos.c 里
 * 只留 `USER CODE BEGIN RTOS_THREADS` 中的一行调用，CubeMX 重生成不会冲掉。
 *
 * 为什么非要独立任务：这三根脚（PE2/PE3/PE4）没有定时器复用，亮度只能靠软件
 * 按固定节拍翻转。原来 LED 是搭在 UART 任务里走的，而那个任务的节拍是
 * `osThreadFlagsWait(..., 20ms)` —— 有串口流量就快、没有就慢，节拍本身不稳。
 * 拿它当调光时基，呼吸会随串口忙闲忽明忽暗。
 *
 * 也不挂在 1 kHz 的 SensorTask 上：那是 IMU DRDY 驱动的，DRDY 一停它就退到
 * 20 ms 轮询兜底——**恰好在传感器出问题的时候，报故障的灯先跟着一起坏**。
 */
void APP_Task_LED_Start(void);
extern osThreadId_t LEDTaskHandle;
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

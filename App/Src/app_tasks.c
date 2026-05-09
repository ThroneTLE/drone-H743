#include "app_tasks.h"
#include "app_message.h"
#include "app_led.h"
#include "app_imu.h"
#include "app_uart.h"
#include "app_maint_uart.h"

void APP_Task_LED_Init(void)
{
    APP_LED_Task_Init();
}

void APP_Task_LED_Step(void)
{
    APP_LED_Task_Step();
}

void APP_Task_IMU_Init(void)
{
    APP_IMU_Task_Init();
}

void APP_Task_IMU_Step(void)
{
    APP_IMU_Task_Step();
}

void APP_Task_Message_Init(void)
{
    APP_Message_Task_Init();
}

void APP_Task_Message_Step(void)
{
    APP_Message_Task_Step();
}

void APP_Task_UART_Init(void)
{
    APP_UART_Task_Init();
}

void APP_Task_UART_Step(void)
{
    APP_UART_Task_Step();
}

void APP_Task_MaintUART_Init(void)
{
    APP_MaintUART_Init();
}

void APP_Task_MaintUART_Step(void)
{
    APP_MaintUART_Step();
}

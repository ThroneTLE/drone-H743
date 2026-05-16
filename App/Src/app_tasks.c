#include "app_tasks.h"
#include "app_background.h"
#include "app_message.h"
#include "app_led.h"
#include "app_sensor.h"
#include "app_gps.h"
#include "app_mag.h"
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

void APP_Task_GPS_Init(void)
{
    APP_GPS_Init();
}

void APP_Task_GPS_Step(void)
{
    APP_GPS_Step();
}

void APP_Task_MAG_Init(void)
{
    APP_MAG_Init();
}

void APP_Task_MAG_Step(void)
{
    APP_MAG_Step();
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

void APP_Task_Background_Init(void)
{
    APP_Background_Init();
}

void APP_Task_Background_Step(void)
{
    APP_Background_Step();
}

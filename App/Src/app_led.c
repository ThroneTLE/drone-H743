#include "app_led.h"
#include "bsp_led.h"
#include "cmsis_os2.h"

void APP_LED_Task_Init(void)
{
}

void APP_LED_Task_Step(void)
{
    BSP_LED_Toggle(LED_RED);
    BSP_LED_Toggle(LED_2);
    BSP_LED_Toggle(LED_3);
    BSP_LED_Toggle(LED_4);
    osDelay(1000);
}

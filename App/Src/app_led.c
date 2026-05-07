#include "app_led.h"

#include "app_aiwb2.h"
#include "bsp_led.h"
#include "cmsis_os2.h"

void APP_LED_Task_Init(void)
{
    BSP_LED_Off(LED_2);
    BSP_LED_Off(LED_3);
}

void APP_LED_Task_Step(void)
{
    BSP_LED_Toggle(LED_RED);

    if (APP_AiWB2_IsTransparent() != 0U) {
        BSP_LED_On(LED_2);
    } else {
        BSP_LED_Off(LED_2);
    }

    BSP_LED_Off(LED_3);

    osDelay(1000);
}

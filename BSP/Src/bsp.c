#include "bsp.h"

void BSP_Init(void)
{
    BSP_Board_Init();
    (void)BSP_PWM_Init();
    BSP_AiWB2_PowerInit();
    BSP_LED_Init();
    BSP_System_Init();
}

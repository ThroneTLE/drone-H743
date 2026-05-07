#include "bsp.h"

void BSP_Init(void)
{
    BSP_AiWB2_PowerInit();
    BSP_LED_Init();
    BSP_System_Init();
    /* Future: BSP_UART_Init(), BSP_I2C_Init(), etc. */
}

#include "bsp.h"

void BSP_Init(void)
{
    BSP_Board_Init();
    /*
     * 维护口的发送队列要在任何人往它写字节之前绑好——APP_MaintUART_Init 上电
     * 第一件事就是发一行 BOOT 文本，那时队列必须已经可用。
     */
    BSP_UART_MaintInit();
    (void)BSP_PWM_Init();
    BSP_AiWB2_PowerInit();
    BSP_LED_Init();
    BSP_System_Init();
}

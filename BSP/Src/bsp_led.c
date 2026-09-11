#include "bsp_led.h"
#include "bsp_gpio.h"

/*
 * MicoAir743v2 板载三色 LED：红 PE3 / 绿 PE2 / 蓝 PE4（低有效由硬件决定，
 * 这里只管拉高拉低，极性在实机上核）。
 *
 * LED_1 在本板上**不对应任何引脚**。它原本是老板子 Ai-WB2 的使能脚 PC6，而 PC6 在
 * MicoAir 上是 USART6_TX（ELRS 接收机那条线）。所以映射到 BSP_GPIO_COUNT：
 * BSP_GPIO_Write/Read 都会边界检查后直接返回。下面几个入口依旧对 LED_1 提前返回，
 * 两道保险叠在一起——就算哪天有人删掉提前返回，也只是空操作，
 * 而不是去驱动遥控链路的发送脚。调用点（app_uart.c / bsp_uart.c）因此一个字不用改。
 *
 * LED_4 落到没被占用的 PD10，留作备用指示。
 */
static const BSP_GPIO_Pin led_map[LED_COUNT] = {
    [LED_RED] = BSP_GPIO_PE3,
    [LED_1]   = BSP_GPIO_COUNT,
    [LED_2]   = BSP_GPIO_PE2,
    [LED_3]   = BSP_GPIO_PE4,
    [LED_4]   = BSP_GPIO_PD10,
};

void BSP_LED_Init(void) { /* GPIO already init by CubeMX */ }

void BSP_LED_On(BSP_LED_ID id)
{
    if (id >= LED_COUNT) return;
    if (id == LED_1) return;
    BSP_GPIO_Write(led_map[id], 1);
}

void BSP_LED_Off(BSP_LED_ID id)
{
    if (id >= LED_COUNT) return;
    if (id == LED_1) return;
    BSP_GPIO_Write(led_map[id], 0);
}

void BSP_LED_Toggle(BSP_LED_ID id)
{
    if (id >= LED_COUNT) return;
    if (id == LED_1) return;
    BSP_GPIO_Toggle(led_map[id]);
}

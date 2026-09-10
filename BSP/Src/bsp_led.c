#include "bsp_led.h"
#include "bsp_gpio.h"

/*
 * MicoAir743v2 板载三色 LED：红 PE3 / 绿 PE2 / 蓝 PE4（低有效由硬件决定，
 * 这里只管拉高拉低，极性在实机上核）。
 *
 * LED_1 仍然映射到 PC6，因为那个脚在本工程里是 Ai-WB2 的使能，不是灯；
 * 下面几个入口对 LED_1 一律直接返回，保持原有的"别去动它"行为。
 * LED_4 落到没被占用的 PD10，留作备用指示。
 */
static const BSP_GPIO_Pin led_map[LED_COUNT] = {
    [LED_RED] = BSP_GPIO_PE3,
    [LED_1]   = BSP_GPIO_PC6,
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

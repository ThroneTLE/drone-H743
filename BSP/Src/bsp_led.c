#include "bsp_led.h"
#include "bsp_gpio.h"

static const BSP_GPIO_Pin led_map[LED_COUNT] = {
    [LED_RED] = BSP_GPIO_PC13,
    [LED_1]   = BSP_GPIO_PC6,
    [LED_2]   = BSP_GPIO_PC7,
    [LED_3]   = BSP_GPIO_PC8,
    [LED_4]   = BSP_GPIO_PC9,
};

void BSP_LED_Init(void) { /* GPIO already init by CubeMX */ }

void BSP_LED_On(BSP_LED_ID id)
{
    if (id >= LED_COUNT) return;
    BSP_GPIO_Write(led_map[id], 1);
}

void BSP_LED_Off(BSP_LED_ID id)
{
    if (id >= LED_COUNT) return;
    BSP_GPIO_Write(led_map[id], 0);
}

void BSP_LED_Toggle(BSP_LED_ID id)
{
    if (id >= LED_COUNT) return;
    BSP_GPIO_Toggle(led_map[id]);
}

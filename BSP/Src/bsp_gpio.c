#include "bsp_gpio.h"

typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
} BSP_GPIO_Map;

static const BSP_GPIO_Map gpio_map[BSP_GPIO_COUNT] = {
    [BSP_GPIO_PC13] = {GPIOC, GPIO_PIN_13},
    [BSP_GPIO_PC6]  = {GPIOC, GPIO_PIN_6},
    [BSP_GPIO_PC7]  = {GPIOC, GPIO_PIN_7},
    [BSP_GPIO_PC8]  = {GPIOC, GPIO_PIN_8},
    [BSP_GPIO_PC9]  = {GPIOC, GPIO_PIN_9},
    [BSP_GPIO_PB5]  = {GPIOB, GPIO_PIN_5},
    [BSP_GPIO_PD8]  = {GPIOD, GPIO_PIN_8},
    [BSP_GPIO_PD9]  = {GPIOD, GPIO_PIN_9},
    [BSP_GPIO_PD10] = {GPIOD, GPIO_PIN_10},
};

void BSP_GPIO_Init(void)
{
    /* GPIO init is handled by CubeMX MX_GPIO_Init() */
}

void BSP_GPIO_Write(BSP_GPIO_Pin pin, uint8_t state)
{
    if (pin >= BSP_GPIO_COUNT) return;
    HAL_GPIO_WritePin(gpio_map[pin].port, gpio_map[pin].pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

uint8_t BSP_GPIO_Read(BSP_GPIO_Pin pin)
{
    if (pin >= BSP_GPIO_COUNT) return 0;
    return HAL_GPIO_ReadPin(gpio_map[pin].port, gpio_map[pin].pin) == GPIO_PIN_SET;
}

void BSP_GPIO_Toggle(BSP_GPIO_Pin pin)
{
    if (pin >= BSP_GPIO_COUNT) return;
    HAL_GPIO_TogglePin(gpio_map[pin].port, gpio_map[pin].pin);
}

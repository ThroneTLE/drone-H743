#ifndef BSP_GPIO_H
#define BSP_GPIO_H

#include "main.h"

typedef enum {
    BSP_GPIO_PC13 = 0,
    BSP_GPIO_PC6,
    BSP_GPIO_PC7,
    BSP_GPIO_PC8,
    BSP_GPIO_PC9,
    BSP_GPIO_PB5,
    BSP_GPIO_PD8,
    BSP_GPIO_PD9,
    BSP_GPIO_PD10,
    BSP_GPIO_COUNT
} BSP_GPIO_Pin;

void BSP_GPIO_Init(void);
void BSP_GPIO_Write(BSP_GPIO_Pin pin, uint8_t state);
uint8_t BSP_GPIO_Read(BSP_GPIO_Pin pin);
void BSP_GPIO_Toggle(BSP_GPIO_Pin pin);

#endif

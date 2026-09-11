#ifndef BSP_GPIO_H
#define BSP_GPIO_H

#include "main.h"

/*
 * 通用 GPIO 句柄表。
 *
 * 2026-09-10 移植到 MicoAir743v2：原来的 PC8 / PC9 / PD8 / PD9 在这块板上分别是
 * SDMMC1 的 D0/D1 与 USART3（GPS），不能再当普通 IO 用，已从表中移除；
 * 三色 LED 换到板载的 PE3(红) / PE2(绿) / PE4(蓝)，蜂鸣器落在 PD15。
 *
 * PC6 / PC7 从表里拿掉了：ELRS 搬到板载 RC 口之后它们是 USART6_TX / USART6_RX，
 * 留在这张"普通 IO"表里等于告诉调用者可以随便读写，而那是遥控链路那条线。
 * PB5 在本板上是 UART5_RX，本工程没启用 UART5，仍当普通输入保留。
 */
typedef enum {
    BSP_GPIO_PC13 = 0,
    BSP_GPIO_PB5,
    BSP_GPIO_PE2,
    BSP_GPIO_PE3,
    BSP_GPIO_PE4,
    BSP_GPIO_PD15,
    BSP_GPIO_PD10,
    BSP_GPIO_COUNT
} BSP_GPIO_Pin;

void BSP_GPIO_Init(void);
void BSP_GPIO_Write(BSP_GPIO_Pin pin, uint8_t state);
uint8_t BSP_GPIO_Read(BSP_GPIO_Pin pin);
void BSP_GPIO_Toggle(BSP_GPIO_Pin pin);

#endif

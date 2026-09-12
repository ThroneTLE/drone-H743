#ifndef BSP_RGB_LED_H
#define BSP_RGB_LED_H

#include <stdint.h>

/*
 * 板载三色 LED 的绑定：哪三根脚、什么极性。换板时**只改这一个文件**。
 *
 * 上层拿到的接口只有"这一拍三个通道各亮不亮"，不出现引脚号，也不出现高低电平。
 */

void BSP_RgbLed_Init(void);

/* bit0=R、bit1=G、bit2=B，1 = 这一拍点亮。极性转换在本文件里做完。 */
void BSP_RgbLed_WriteBits(uint8_t bits);

/* 诊断：本板把三个通道接在哪几根脚上、是不是低电平点亮。 */
const char *BSP_RgbLed_PinName(uint8_t channel);
uint8_t BSP_RgbLed_IsActiveLow(void);

#endif

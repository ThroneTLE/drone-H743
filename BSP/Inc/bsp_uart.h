#ifndef BSP_UART_H
#define BSP_UART_H

#include "main.h"

#include <stdint.h>

HAL_StatusTypeDef BSP_UART_Transmit_USART1(const uint8_t *data,
                                           uint16_t length,
                                           uint32_t timeout_ms);

#endif

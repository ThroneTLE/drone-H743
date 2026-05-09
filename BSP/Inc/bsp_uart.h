#ifndef BSP_UART_H
#define BSP_UART_H

#include "main.h"

#include <stdint.h>

#define BSP_UART_USART1_OUTPUT_ENABLED 1U

void BSP_UART_Release_USART1_ForExternalDebug(void);

HAL_StatusTypeDef BSP_UART_Transmit_USART1(const uint8_t *data,
                                           uint16_t length,
                                           uint32_t timeout_ms);
HAL_StatusTypeDef BSP_UART_Transmit_UART8(const uint8_t *data,
                                          uint16_t length,
                                          uint32_t timeout_ms);
uint32_t BSP_UART_GetUSART1TxCount(void);

#endif

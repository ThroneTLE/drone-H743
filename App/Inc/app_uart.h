#ifndef APP_UART_H
#define APP_UART_H

#include <stdint.h>
#include "main.h"

void APP_UART_GetStats(uint32_t *rx_bytes,
                       uint32_t *rx_lines,
                       uint32_t *rx_overflows,
                       uint32_t *rx_errors);
void APP_UART_Task_Init(void);
void APP_UART_Task_Step(void);
void APP_UART_NotifyTxPending(void);
void APP_UART_OnRxEvent(UART_HandleTypeDef *huart, uint16_t size);
void APP_UART_OnTxComplete(UART_HandleTypeDef *huart);
void APP_UART_OnError(UART_HandleTypeDef *huart);

#endif

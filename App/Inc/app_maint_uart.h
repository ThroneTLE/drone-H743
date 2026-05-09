#ifndef APP_MAINT_UART_H
#define APP_MAINT_UART_H

#include <stdint.h>
#include "main.h"

void APP_MaintUART_Init(void);
void APP_MaintUART_Step(void);
void APP_MaintUART_Write(const char *text, uint16_t length);
void APP_MaintUART_OnRxCplt(UART_HandleTypeDef *huart);
void APP_MaintUART_OnError(UART_HandleTypeDef *huart);

#endif

#ifndef APP_UART_H
#define APP_UART_H

#include <stdint.h>

void APP_UART_GetStats(uint32_t *rx_bytes,
                       uint32_t *rx_lines,
                       uint32_t *rx_overflows,
                       uint32_t *rx_errors);
void APP_UART_GetRxEventStats(uint32_t *rx_events,
                              uint32_t *rx_restarts,
                              uint32_t *last_rx_event_size);
void APP_UART_Task_Init(void);
void APP_UART_Task_Step(void);
void APP_UART_NotifyTxPending(void);
/*
 * 数传口的中断事件入口。实例匹配由 BSP 做（BSP_UartEvents_Register 注册到
 * BSP_UART_ROLE_TELEMETRY），所以这里既不需要、也拿不到 HAL 句柄。
 * 全部在中断上下文执行：只置标志、唤醒任务。
 */
void APP_UART_OnRxEvent(uint16_t size);
void APP_UART_OnTxComplete(void);
void APP_UART_OnError(void);

#endif

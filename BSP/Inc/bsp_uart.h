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

/* ---------------- 诊断：任意 UART 事务 ---------------- */

/*
 * 让上位机直接往某个串口发几个字节、再把回来的收上来，不必为每次试验重新编译。
 *
 * 总线号用板上的实例编号（1/2/3/6/7/8），与丝印和 hwdef 一致。
 * **UART7 是半双工总线舵机口**——往上面发字节会让舵机动，所以是否允许调用
 * 由 App 层按解锁状态把关，这里只负责老实收发。
 */

#define BSP_UART_XFER_MAX 64U

/* 总线号 → HAL 句柄。未知或未启用的总线返回 NULL。 */
UART_HandleTypeDef *BSP_UART_GetHandle(uint8_t bus_index);

/* 该总线是否是半双工（发完要切回接收），供诊断如实回报。 */
uint8_t BSP_UART_IsHalfDuplex(uint8_t bus_index);

/*
 * 先发 tx_len 字节，再在 timeout_ms 内尽量收满 rx_len 字节。
 * *rx_got 回报实际收到多少（收不满不算错——对面可能本来就没那么多话说）。
 */
HAL_StatusTypeDef BSP_UART_DebugXfer(uint8_t bus_index,
                                     const uint8_t *tx, uint16_t tx_len,
                                     uint8_t *rx, uint16_t rx_len,
                                     uint16_t *rx_got,
                                     uint32_t timeout_ms);

#endif

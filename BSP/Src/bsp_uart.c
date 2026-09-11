#include "bsp_uart.h"

#include "bsp_led.h"
#include "usart.h"

#define BSP_UART_TX_LED_ENABLED 0U

static volatile uint32_t bsp_uart_usart1_tx_count;

void BSP_UART_Release_USART1_ForExternalDebug(void)
{
#if (BSP_UART_USART1_OUTPUT_ENABLED == 0U)
    (void)HAL_UART_DeInit(&huart1);
#endif
}

HAL_StatusTypeDef BSP_UART_Transmit_USART1(const uint8_t *data,
                                           uint16_t length,
                                           uint32_t timeout_ms)
{
#if (BSP_UART_USART1_OUTPUT_ENABLED == 0U)
    (void)data;
    (void)length;
    (void)timeout_ms;
    return HAL_OK;
#else
    if ((data == 0) || (length == 0U)) {
        return HAL_OK;
    }

    ++bsp_uart_usart1_tx_count;
#if (BSP_UART_TX_LED_ENABLED != 0U)
    BSP_LED_On(LED_1);
#endif
    return HAL_UART_Transmit(&huart1, (uint8_t *)data, length, timeout_ms);
#endif
}

HAL_StatusTypeDef BSP_UART_Transmit_UART8(const uint8_t *data,
                                          uint16_t length,
                                          uint32_t timeout_ms)
{
    if ((data == 0) || (length == 0U)) {
        return HAL_OK;
    }

    return HAL_UART_Transmit(&huart8, (uint8_t *)data, length, timeout_ms);
}

uint32_t BSP_UART_GetUSART1TxCount(void)
{
    return bsp_uart_usart1_tx_count;
}

/* ============================================================ 诊断：任意 UART 事务 */

UART_HandleTypeDef *BSP_UART_GetHandle(uint8_t bus_index)
{
    switch (bus_index) {
    case 1U: return &huart1;
    case 2U: return &huart2;
    case 3U: return &huart3;
    case 6U: return &huart6;
    case 7U: return &huart7;
    case 8U: return &huart8;
    default: return NULL;
    }
}

uint8_t BSP_UART_IsHalfDuplex(uint8_t bus_index)
{
    /* UART7 走总线舵机的单线协议，收发共用一根线。其余都是全双工。 */
    return (bus_index == 7U) ? 1U : 0U;
}

/*
 * 阻塞式收发。接收用逐字节 HAL_UART_Receive 而不是一次收满，是为了让
 * "对面只回了半句"也能把已经收到的部分交出来——诊断里这比"要么收满要么报超时"
 * 有用得多：帧长不对、波特率不对，往往正是从残缺的回包上看出来的。
 */
HAL_StatusTypeDef BSP_UART_DebugXfer(uint8_t bus_index,
                                     const uint8_t *tx, uint16_t tx_len,
                                     uint8_t *rx, uint16_t rx_len,
                                     uint16_t *rx_got,
                                     uint32_t timeout_ms)
{
    UART_HandleTypeDef *huart = BSP_UART_GetHandle(bus_index);
    HAL_StatusTypeDef hal = HAL_OK;
    uint16_t got = 0U;

    if (rx_got != NULL) { *rx_got = 0U; }

    if ((huart == NULL) ||
        (tx_len > (uint16_t)BSP_UART_XFER_MAX) ||
        (rx_len > (uint16_t)BSP_UART_XFER_MAX) ||
        ((tx_len != 0U) && (tx == NULL)) ||
        ((rx_len != 0U) && (rx == NULL))) {
        return HAL_ERROR;
    }

    if (tx_len != 0U) {
        if (BSP_UART_IsHalfDuplex(bus_index) != 0U) {
            (void)HAL_HalfDuplex_EnableTransmitter(huart);
        }
        hal = HAL_UART_Transmit(huart, (uint8_t *)(uintptr_t)tx, tx_len,
                                timeout_ms);
        if (BSP_UART_IsHalfDuplex(bus_index) != 0U) {
            (void)HAL_HalfDuplex_EnableReceiver(huart);
        }
        if (hal != HAL_OK) { return hal; }
    }

    while (got < rx_len) {
        if (HAL_UART_Receive(huart, &rx[got], 1U, timeout_ms) != HAL_OK) {
            break;   /* 收不满不算错，把已收到的交出去 */
        }
        got++;
    }

    if (rx_got != NULL) { *rx_got = got; }
    return HAL_OK;
}

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

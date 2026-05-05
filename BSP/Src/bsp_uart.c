#include "bsp_uart.h"

#include "usart.h"

HAL_StatusTypeDef BSP_UART_Transmit_USART1(const uint8_t *data,
                                           uint16_t length,
                                           uint32_t timeout_ms)
{
    if ((data == 0) || (length == 0U)) {
        return HAL_OK;
    }

    return HAL_UART_Transmit(&huart1, (uint8_t *)data, length, timeout_ms);
}

#include "app_uart.h"

#include "app_tasks.h"
#include "bsp_led.h"
#include "bsp_uart.h"

void APP_UART_Task_Init(void)
{
    BSP_LED_Off(LED_1);
}

void APP_UART_Task_Step(void)
{
    APP_UART_TxMessage tx_message;

    if (uartTxQueueHandle == 0) {
        osDelay(10U);
        return;
    }

    if (osMessageQueueGet(uartTxQueueHandle, &tx_message, 0U, osWaitForever) != osOK) {
        return;
    }

    if (tx_message.length == 0U) {
        return;
    }

    BSP_LED_On(LED_1);
    (void)BSP_UART_Transmit_USART1((const uint8_t *)tx_message.text,
                                   tx_message.length,
                                   100U);
    BSP_LED_Off(LED_1);
}

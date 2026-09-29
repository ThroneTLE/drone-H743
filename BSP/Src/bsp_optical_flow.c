#include "bsp_optical_flow.h"

#include "bsp_board.h"

#include <string.h>

#define BSP_OPTICAL_FLOW_DMA_RX_SIZE 64U

__attribute__((section(".dma_buffer"), aligned(32)))
static uint8_t flow_dma_rx_buffer[BSP_OPTICAL_FLOW_DMA_RX_SIZE];
static DRV_OPTICAL_FLOW_Device flow_dev;

BSP_OPTICAL_FLOW_StatusCode BSP_OPTICAL_FLOW_Init(void)
{
    DRV_OPTICAL_FLOW_Bus bus = *BSP_Board_GetOpticalFlowBus();

    bus.rx_dma_buffer = flow_dma_rx_buffer;
    bus.rx_dma_size = BSP_OPTICAL_FLOW_DMA_RX_SIZE;
    return DRV_OPTICAL_FLOW_Init(&flow_dev, &bus);
}

void BSP_OPTICAL_FLOW_Service(void)
{
    DRV_OPTICAL_FLOW_Service(&flow_dev);
}

/*
 * 回调认哪个串口，从板级绑定里问（与 bsp_gps.c 同一做法），不写死实例名。
 * 2026-09-29 光流从 USART2 迁到 UART4 时，这三处原本写死的 USART2 就是
 * "绑定改了、回调没跟着改"的现成隐患。
 */
static uint8_t flow_owns_uart(const UART_HandleTypeDef *huart)
{
    const DRV_OPTICAL_FLOW_Bus *bus = BSP_Board_GetOpticalFlowBus();

    if ((huart == NULL) || (bus == NULL) || (bus->huart == NULL)) { return 0U; }
    return (huart->Instance == bus->huart->Instance) ? 1U : 0U;
}

void BSP_OPTICAL_FLOW_OnUartRxCplt(UART_HandleTypeDef *huart)
{
    if (flow_owns_uart(huart) == 0U) {
        return;
    }
    DRV_OPTICAL_FLOW_OnUartRxCplt(&flow_dev);
}

void BSP_OPTICAL_FLOW_OnUartRxEvent(UART_HandleTypeDef *huart, uint16_t size)
{
    if (flow_owns_uart(huart) == 0U) {
        return;
    }
    DRV_OPTICAL_FLOW_OnUartRxEvent(&flow_dev, size);
}

void BSP_OPTICAL_FLOW_OnUartError(UART_HandleTypeDef *huart)
{
    if (flow_owns_uart(huart) == 0U) {
        return;
    }
    DRV_OPTICAL_FLOW_OnUartError(&flow_dev, huart->ErrorCode);
}

void BSP_OPTICAL_FLOW_GetStatus(BSP_OPTICAL_FLOW_Status *status)
{
    if (status == NULL) {
        return;
    }

    memset(status, 0, sizeof(*status));
    status->initialized = flow_dev.initialized;
    status->rx_active = flow_dev.rx_active;
    status->bytes = flow_dev.bytes;
    status->frames = flow_dev.frames;
    status->checksum_errors = flow_dev.checksum_errors;
    status->frame_errors = flow_dev.frame_errors;
    status->ignored_messages = flow_dev.ignored_messages;
    status->short_payload_errors = flow_dev.short_payload_errors;
    status->rx_restarts = flow_dev.rx_restarts;
    status->dma_events = flow_dev.dma_events;
    status->dma_last_size = flow_dev.dma_last_size;
    status->uart_errors = flow_dev.uart_errors;
    status->last_uart_error = flow_dev.last_uart_error;
    status->last_rx_ms = flow_dev.last_rx_ms;
    status->baud_rate = flow_dev.bus.baud_rate;
    status->latest = flow_dev.latest;
    status->raw_stats = flow_dev.raw_stats;
}

void BSP_OPTICAL_FLOW_Invalidate(void)
{
    DRV_OPTICAL_FLOW_Invalidate(&flow_dev);
}

BSP_OPTICAL_FLOW_StatusCode BSP_OPTICAL_FLOW_TransmitRaw(const uint8_t *data,
                                                         uint16_t length,
                                                         uint32_t timeout_ms)
{
    return DRV_OPTICAL_FLOW_TransmitRaw(&flow_dev, data, length, timeout_ms);
}

uint16_t BSP_OPTICAL_FLOW_ReceiveRaw(uint8_t *data,
                                     uint16_t max_length,
                                     uint32_t per_byte_timeout_ms)
{
    return DRV_OPTICAL_FLOW_ReceiveRaw(&flow_dev, data, max_length,
                                       per_byte_timeout_ms);
}

BSP_OPTICAL_FLOW_StatusCode BSP_OPTICAL_FLOW_TransceiveRaw(const uint8_t *tx_data,
                                                           uint16_t tx_length,
                                                           uint8_t *rx_data,
                                                           uint16_t rx_length,
                                                           uint16_t *rx_count,
                                                           uint32_t timeout_ms)
{
    return DRV_OPTICAL_FLOW_TransceiveRaw(&flow_dev, tx_data, tx_length,
                                          rx_data, rx_length, rx_count,
                                          timeout_ms);
}

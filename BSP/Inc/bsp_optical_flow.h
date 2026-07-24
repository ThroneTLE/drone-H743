#ifndef BSP_OPTICAL_FLOW_H
#define BSP_OPTICAL_FLOW_H

#include "drv_optical_flow.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef DRV_OPTICAL_FLOW_Status BSP_OPTICAL_FLOW_StatusCode;
typedef DRV_OPTICAL_FLOW_Frame BSP_OPTICAL_FLOW_Frame;
typedef DRV_OPTICAL_FLOW_RawStats BSP_OPTICAL_FLOW_RawStats;

typedef struct {
    uint8_t initialized;
    uint8_t rx_active;
    uint32_t bytes;
    uint32_t frames;
    uint32_t checksum_errors;
    uint32_t frame_errors;
    uint32_t ignored_messages;
    uint32_t short_payload_errors;
    uint32_t rx_restarts;
    uint32_t dma_events;
    uint32_t dma_last_size;
    uint32_t uart_errors;
    uint32_t last_uart_error;
    uint32_t last_rx_ms;
    uint32_t baud_rate;
    BSP_OPTICAL_FLOW_Frame latest;
    BSP_OPTICAL_FLOW_RawStats raw_stats;
} BSP_OPTICAL_FLOW_Status;

BSP_OPTICAL_FLOW_StatusCode BSP_OPTICAL_FLOW_Init(void);
void BSP_OPTICAL_FLOW_Service(void);
void BSP_OPTICAL_FLOW_OnUartRxCplt(UART_HandleTypeDef *huart);
void BSP_OPTICAL_FLOW_OnUartRxEvent(UART_HandleTypeDef *huart, uint16_t size);
void BSP_OPTICAL_FLOW_OnUartError(UART_HandleTypeDef *huart);
void BSP_OPTICAL_FLOW_GetStatus(BSP_OPTICAL_FLOW_Status *status);
void BSP_OPTICAL_FLOW_Invalidate(void);
BSP_OPTICAL_FLOW_StatusCode BSP_OPTICAL_FLOW_TransmitRaw(const uint8_t *data,
                                                         uint16_t length,
                                                         uint32_t timeout_ms);
uint16_t BSP_OPTICAL_FLOW_ReceiveRaw(uint8_t *data,
                                     uint16_t max_length,
                                     uint32_t per_byte_timeout_ms);
BSP_OPTICAL_FLOW_StatusCode BSP_OPTICAL_FLOW_TransceiveRaw(const uint8_t *tx_data,
                                                           uint16_t tx_length,
                                                           uint8_t *rx_data,
                                                           uint16_t rx_length,
                                                           uint16_t *rx_count,
                                                           uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif

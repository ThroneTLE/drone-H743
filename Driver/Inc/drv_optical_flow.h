#ifndef DRV_OPTICAL_FLOW_H
#define DRV_OPTICAL_FLOW_H

#include "main.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DRV_OPTICAL_FLOW_BAUD_RATE 115200U
#define DRV_OPTICAL_FLOW_MSP_HEAD_0 '$'
#define DRV_OPTICAL_FLOW_MSP_HEAD_1 'X'
#define DRV_OPTICAL_FLOW_MSP_HEAD_2 '<'
#define DRV_OPTICAL_FLOW_MAX_PAYLOAD_LEN 64U
#define DRV_OPTICAL_FLOW_FRAME_LEN (DRV_OPTICAL_FLOW_MAX_PAYLOAD_LEN + 9U)
#define DRV_OPTICAL_FLOW_RANGE_MSG_ID 0x1F01U
#define DRV_OPTICAL_FLOW_FLOW_MSG_ID  0x1F02U
#define DRV_OPTICAL_FLOW_RANGE_PAYLOAD_LEN 5U
#define DRV_OPTICAL_FLOW_FLOW_PAYLOAD_LEN 9U
#define DRV_OPTICAL_FLOW_VALID     1U
#define DRV_OPTICAL_FLOW_MIN_DISTANCE_MM 10UL
#define DRV_OPTICAL_FLOW_RAW_WINDOW 32U

typedef enum {
    DRV_OPTICAL_FLOW_OK = 0,
    DRV_OPTICAL_FLOW_ERROR,
    DRV_OPTICAL_FLOW_INVALID_ARG
} DRV_OPTICAL_FLOW_Status;

typedef struct {
    UART_HandleTypeDef *huart;
    uint32_t baud_rate;
    uint32_t timeout_ms;
    uint8_t *rx_dma_buffer;
    uint16_t rx_dma_size;
    void (*delay_ms)(uint32_t ms);
    void (*cache_invalidate)(const void *addr, uint32_t len);
} DRV_OPTICAL_FLOW_Bus;

typedef struct {
    uint32_t time_ms;
    uint32_t distance_mm;
    uint8_t strength;
    uint8_t precision;
    uint8_t tof_status;
    int16_t flow_vel_x;
    int16_t flow_vel_y;
    uint8_t flow_quality;
    uint8_t flow_status;
    uint16_t sample_interval_us;
    uint8_t distance_valid;
    uint8_t flow_valid;
    uint8_t valid;
    uint16_t msp_cmd;
    uint8_t msp_flags;
    uint32_t received_ms;
    uint32_t distance_received_ms;
    uint32_t flow_received_ms;
} DRV_OPTICAL_FLOW_Frame;

typedef struct {
    uint16_t count;
    int16_t flow_vel_x_mean;
    int16_t flow_vel_y_mean;
    uint16_t sample_interval_mean_us;
    uint32_t distance_mean_mm;
    uint8_t strength_mean;
    uint8_t flow_quality_mean;
    int16_t flow_vel_x_peak_to_peak;
    int16_t flow_vel_y_peak_to_peak;
    uint16_t sample_interval_peak_to_peak_us;
    uint32_t distance_peak_to_peak_mm;
} DRV_OPTICAL_FLOW_RawStats;

typedef struct {
    DRV_OPTICAL_FLOW_Bus bus;
    uint8_t initialized;
    uint8_t rx_active;
    uint8_t rx_byte;
    uint8_t frame[DRV_OPTICAL_FLOW_FRAME_LEN];
    uint16_t offset;
    uint16_t expected_len;
    uint16_t rx_dma_pos;
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
    DRV_OPTICAL_FLOW_Frame latest;
    DRV_OPTICAL_FLOW_Frame raw_window[DRV_OPTICAL_FLOW_RAW_WINDOW];
    uint16_t raw_window_pos;
    uint16_t raw_window_count;
    DRV_OPTICAL_FLOW_RawStats raw_stats;
} DRV_OPTICAL_FLOW_Device;

DRV_OPTICAL_FLOW_Status DRV_OPTICAL_FLOW_Init(DRV_OPTICAL_FLOW_Device *dev,
                                              const DRV_OPTICAL_FLOW_Bus *bus);
void DRV_OPTICAL_FLOW_Service(DRV_OPTICAL_FLOW_Device *dev);
void DRV_OPTICAL_FLOW_OnUartRxCplt(DRV_OPTICAL_FLOW_Device *dev);
void DRV_OPTICAL_FLOW_OnUartRxEvent(DRV_OPTICAL_FLOW_Device *dev,
                                    uint16_t size);
void DRV_OPTICAL_FLOW_OnUartError(DRV_OPTICAL_FLOW_Device *dev,
                                  uint32_t error_code);
void DRV_OPTICAL_FLOW_Invalidate(DRV_OPTICAL_FLOW_Device *dev);
uint8_t DRV_OPTICAL_FLOW_ConsumeByte(DRV_OPTICAL_FLOW_Device *dev, uint8_t byte);
DRV_OPTICAL_FLOW_Status DRV_OPTICAL_FLOW_TransmitRaw(DRV_OPTICAL_FLOW_Device *dev,
                                                     const uint8_t *data,
                                                     uint16_t length,
                                                     uint32_t timeout_ms);
uint16_t DRV_OPTICAL_FLOW_ReceiveRaw(DRV_OPTICAL_FLOW_Device *dev,
                                     uint8_t *data,
                                     uint16_t max_length,
                                     uint32_t per_byte_timeout_ms);
DRV_OPTICAL_FLOW_Status DRV_OPTICAL_FLOW_TransceiveRaw(DRV_OPTICAL_FLOW_Device *dev,
                                                       const uint8_t *tx_data,
                                                       uint16_t tx_length,
                                                       uint8_t *rx_data,
                                                       uint16_t rx_length,
                                                       uint16_t *rx_count,
                                                       uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif

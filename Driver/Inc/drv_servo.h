#ifndef DRV_SERVO_H
#define DRV_SERVO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#include <stdint.h>

typedef enum {
    DRV_SERVO_OK = 0,
    DRV_SERVO_ERROR,
    DRV_SERVO_INVALID_PARAM,
    DRV_SERVO_TIMEOUT,
    DRV_SERVO_BUSY
} DRV_SERVO_Status;

typedef struct {
    uint8_t  id;
    uint16_t pulse_us;
} DRV_SERVO_MoveCmd;

typedef struct {
    UART_HandleTypeDef *huart;
    uint32_t            timeout_ms;
} DRV_SERVO_Bus;

typedef struct {
    DRV_SERVO_Bus bus;
} DRV_SERVO_Device;

typedef struct {
    uint32_t tx_start_count;
    uint32_t tx_complete_count;
    uint32_t tx_busy_count;
    uint32_t tx_error_count;
    uint32_t tx_recover_count;
    uint32_t last_length;
    uint32_t last_status;
    uint32_t last_uart_state;
    uint32_t last_uart_error;
    uint32_t last_dma_state;
    uint32_t last_dma_error;
} DRV_SERVO_Diag;

typedef enum {
    DRV_SERVO_FEEDBACK_EVENT_NONE = 0,
    DRV_SERVO_FEEDBACK_EVENT_VALID,
    DRV_SERVO_FEEDBACK_EVENT_TIMEOUT,
    DRV_SERVO_FEEDBACK_EVENT_PARSE_ERROR,
    DRV_SERVO_FEEDBACK_EVENT_UART_ERROR
} DRV_SERVO_FeedbackEventType;

typedef struct {
    uint32_t sequence;
    uint32_t timestamp_ms;
    uint32_t rtt_ms;
    uint16_t position_us;
    uint16_t rx_length;
    uint8_t id;
    uint8_t type;
} DRV_SERVO_FeedbackEvent;

typedef struct {
    uint32_t request_count;
    uint32_t response_count;
    uint32_t timeout_count;
    uint32_t parse_error_count;
    uint32_t uart_error_count;
    uint32_t busy_count;
    uint32_t rtt_sum_ms;
    uint32_t max_rtt_ms;
    uint8_t pending;
    uint8_t pending_id;
    DRV_SERVO_FeedbackEvent last_event;
} DRV_SERVO_FeedbackDiag;

#define DRV_SERVO_MIN_PULSE_US    500U
#define DRV_SERVO_MAX_PULSE_US    2500U
#define DRV_SERVO_MAX_TIME_MS     9999U
#define DRV_SERVO_POSITION_MIN    0U
#define DRV_SERVO_POSITION_MAX    1000U
#define DRV_SERVO_MAX_RESPONSE_LEN 64U

DRV_SERVO_Status DRV_SERVO_SendRaw(DRV_SERVO_Device *dev, const char *command);
uint16_t DRV_SERVO_ReadResponse(DRV_SERVO_Device *dev, char *buf, uint16_t max_len);
uint32_t DRV_SERVO_GetBaudRate(const DRV_SERVO_Device *dev);
DRV_SERVO_Status DRV_SERVO_SetBaudRate(DRV_SERVO_Device *dev, uint32_t baud_rate);
uint16_t DRV_SERVO_PositionToPulse(uint16_t position);
DRV_SERVO_Status DRV_SERVO_Move(DRV_SERVO_Device *dev, uint8_t id,
                                uint16_t pulse_us, uint16_t time_ms);
DRV_SERVO_Status DRV_SERVO_MovePosition(DRV_SERVO_Device *dev, uint8_t id,
                                        uint16_t position, uint16_t time_ms);
DRV_SERVO_Status DRV_SERVO_MoveMany(DRV_SERVO_Device *dev,
                                    const DRV_SERVO_MoveCmd *moves,
                                    uint8_t count, uint16_t time_ms);
DRV_SERVO_Status DRV_SERVO_MoveManyAsync(DRV_SERVO_Device *dev,
                                         const DRV_SERVO_MoveCmd *moves,
                                         uint8_t count, uint16_t time_ms);
DRV_SERVO_Status DRV_SERVO_RequestPositionAsync(DRV_SERVO_Device *dev,
                                                uint8_t id,
                                                uint32_t timeout_ms);
void DRV_SERVO_Service(DRV_SERVO_Device *dev, uint32_t now_ms);
uint8_t DRV_SERVO_IsBusIdle(const DRV_SERVO_Device *dev);
void DRV_SERVO_GetDiag(DRV_SERVO_Diag *diag);
void DRV_SERVO_GetFeedbackDiag(DRV_SERVO_FeedbackDiag *diag);
void DRV_SERVO_OnUartTxComplete(UART_HandleTypeDef *huart);
void DRV_SERVO_OnUartRxEvent(UART_HandleTypeDef *huart, uint16_t size);
void DRV_SERVO_OnUartError(UART_HandleTypeDef *huart);
DRV_SERVO_Status DRV_SERVO_ReadVersion(DRV_SERVO_Device *dev, uint8_t id);
DRV_SERVO_Status DRV_SERVO_ReadId(DRV_SERVO_Device *dev, uint8_t id);
DRV_SERVO_Status DRV_SERVO_SetId(DRV_SERVO_Device *dev, uint8_t old_id, uint8_t new_id);
DRV_SERVO_Status DRV_SERVO_ReadPosition(DRV_SERVO_Device *dev, uint8_t id);
DRV_SERVO_Status DRV_SERVO_SetMode(DRV_SERVO_Device *dev, uint8_t id, uint8_t mode);
DRV_SERVO_Status DRV_SERVO_ReadMode(DRV_SERVO_Device *dev, uint8_t id);
DRV_SERVO_Status DRV_SERVO_ReleaseTorque(DRV_SERVO_Device *dev, uint8_t id);
DRV_SERVO_Status DRV_SERVO_RestoreTorque(DRV_SERVO_Device *dev, uint8_t id);
DRV_SERVO_Status DRV_SERVO_Pause(DRV_SERVO_Device *dev, uint8_t id);
DRV_SERVO_Status DRV_SERVO_Continue(DRV_SERVO_Device *dev, uint8_t id);
DRV_SERVO_Status DRV_SERVO_Stop(DRV_SERVO_Device *dev, uint8_t id);
DRV_SERVO_Status DRV_SERVO_SetBaud(DRV_SERVO_Device *dev, uint8_t id, uint8_t baud_code);
DRV_SERVO_Status DRV_SERVO_SaveCenter(DRV_SERVO_Device *dev, uint8_t id);
DRV_SERVO_Status DRV_SERVO_SetStartupPosition(DRV_SERVO_Device *dev, uint8_t id);
DRV_SERVO_Status DRV_SERVO_ClearStartupPosition(DRV_SERVO_Device *dev, uint8_t id);
DRV_SERVO_Status DRV_SERVO_RestoreStartupPosition(DRV_SERVO_Device *dev, uint8_t id);
DRV_SERVO_Status DRV_SERVO_SetMinPosition(DRV_SERVO_Device *dev, uint8_t id);
DRV_SERVO_Status DRV_SERVO_SetMaxPosition(DRV_SERVO_Device *dev, uint8_t id);
DRV_SERVO_Status DRV_SERVO_FactoryResetKeepId(DRV_SERVO_Device *dev, uint8_t id);
DRV_SERVO_Status DRV_SERVO_FactoryResetFull(DRV_SERVO_Device *dev, uint8_t id);

#ifdef __cplusplus
}
#endif

#endif

#include "drv_servo.h"

#include "bsp_cache.h"

#include <stdio.h>
#include <string.h>

#define DRV_SERVO_MAX_BAUDRATE     1000000U
#define DRV_SERVO_MIN_BAUDRATE     1200U
#define DRV_SERVO_MAX_ID           255U
#define DRV_SERVO_MAX_ITEMS       8U
#define DRV_SERVO_TX_TIMEOUT_MS   100U
#define DRV_SERVO_MIN_MODE        1U
#define DRV_SERVO_MAX_MODE        8U
#define DRV_SERVO_MAX_BAUD_CODE   7U
#define DRV_SERVO_COMMAND_BUFFER_SIZE 128U
#define DRV_SERVO_DMA_MIN_TIMEOUT_MS 10U
#define DRV_SERVO_UART_BITS_PER_BYTE 10U
#define DRV_SERVO_POSITION_RESPONSE_LEN 10U
#define DRV_SERVO_FEEDBACK_MIN_TIMEOUT_MS 2U
#define DRV_SERVO_FEEDBACK_MAX_TIMEOUT_MS 100U

typedef enum {
    SERVO_ASYNC_IDLE = 0,
    SERVO_ASYNC_MOVE_TX,
    SERVO_ASYNC_FEEDBACK_TX,
    SERVO_ASYNC_FEEDBACK_RX
} ServoAsyncState;

__attribute__((section(".dma_buffer"), aligned(32)))
static uint8_t servo_dma_tx_buffer[DRV_SERVO_COMMAND_BUFFER_SIZE];
__attribute__((section(".dma_buffer"), aligned(32)))
static uint8_t servo_dma_rx_buffer[32];
static volatile DRV_SERVO_Diag servo_diag;
static volatile DRV_SERVO_FeedbackDiag servo_feedback_diag;
static volatile uint32_t servo_dma_start_tick_ms;
static volatile uint32_t servo_dma_timeout_ms;
static UART_HandleTypeDef * volatile servo_dma_huart;
static volatile ServoAsyncState servo_async_state;
static volatile uint32_t servo_feedback_start_tick_ms;
static volatile uint32_t servo_feedback_timeout_ms;
static volatile uint8_t servo_feedback_pending_id;

static uint8_t servo_parse_position_response(const uint8_t *data,
                                             uint16_t length,
                                             uint8_t expected_id,
                                             uint16_t *position_us)
{
    uint32_t parsed_id;
    uint32_t parsed_position;

    if ((data == NULL) || (position_us == NULL) ||
        (length != DRV_SERVO_POSITION_RESPONSE_LEN) ||
        (data[0] != '#') || (data[4] != 'P') || (data[9] != '!')) {
        return 0U;
    }

    for (uint32_t i = 1U; i <= 3U; ++i) {
        if ((data[i] < '0') || (data[i] > '9')) {
            return 0U;
        }
    }
    for (uint32_t i = 5U; i <= 8U; ++i) {
        if ((data[i] < '0') || (data[i] > '9')) {
            return 0U;
        }
    }

    parsed_id = (uint32_t)(data[1] - '0') * 100U +
                (uint32_t)(data[2] - '0') * 10U +
                (uint32_t)(data[3] - '0');
    parsed_position = (uint32_t)(data[5] - '0') * 1000U +
                      (uint32_t)(data[6] - '0') * 100U +
                      (uint32_t)(data[7] - '0') * 10U +
                      (uint32_t)(data[8] - '0');

    if ((parsed_id != expected_id) ||
        (parsed_position < DRV_SERVO_MIN_PULSE_US) ||
        (parsed_position > DRV_SERVO_MAX_PULSE_US)) {
        return 0U;
    }

    *position_us = (uint16_t)parsed_position;
    return 1U;
}

static void servo_feedback_record_event(DRV_SERVO_FeedbackEventType type,
                                        uint8_t id,
                                        uint16_t position_us,
                                        uint16_t rx_length,
                                        uint32_t timestamp_ms,
                                        uint32_t rtt_ms)
{
    uint32_t sequence = servo_feedback_diag.last_event.sequence + 1U;

    switch (type) {
    case DRV_SERVO_FEEDBACK_EVENT_VALID:
        servo_feedback_diag.response_count++;
        servo_feedback_diag.rtt_sum_ms += rtt_ms;
        if (rtt_ms > servo_feedback_diag.max_rtt_ms) {
            servo_feedback_diag.max_rtt_ms = rtt_ms;
        }
        break;
    case DRV_SERVO_FEEDBACK_EVENT_TIMEOUT:
        servo_feedback_diag.timeout_count++;
        break;
    case DRV_SERVO_FEEDBACK_EVENT_PARSE_ERROR:
        servo_feedback_diag.parse_error_count++;
        break;
    case DRV_SERVO_FEEDBACK_EVENT_UART_ERROR:
        servo_feedback_diag.uart_error_count++;
        break;
    default:
        break;
    }

    servo_feedback_diag.last_event.timestamp_ms = timestamp_ms;
    servo_feedback_diag.last_event.rtt_ms = rtt_ms;
    servo_feedback_diag.last_event.position_us = position_us;
    servo_feedback_diag.last_event.rx_length = rx_length;
    servo_feedback_diag.last_event.id = id;
    servo_feedback_diag.last_event.type = (uint8_t)type;
    __DMB();
    servo_feedback_diag.last_event.sequence = sequence;
}

static void servo_feedback_return_to_tx(UART_HandleTypeDef *huart)
{
    if (huart != NULL) {
        if (huart->RxState != HAL_UART_STATE_READY) {
            (void)HAL_UART_AbortReceive(huart);
        }
        HAL_HalfDuplex_EnableTransmitter(huart);
    }

    servo_async_state = SERVO_ASYNC_IDLE;
    servo_dma_start_tick_ms = 0U;
    servo_dma_timeout_ms = 0U;
    servo_dma_huart = NULL;
    servo_feedback_start_tick_ms = 0U;
    servo_feedback_timeout_ms = 0U;
    servo_feedback_pending_id = 0U;
    servo_feedback_diag.pending = 0U;
    servo_feedback_diag.pending_id = 0U;
}

static uint32_t servo_estimate_dma_timeout_ms(UART_HandleTypeDef *huart,
                                              uint16_t length)
{
    uint32_t baud = ((huart != NULL) && (huart->Init.BaudRate != 0U)) ?
                    huart->Init.BaudRate : 115200U;
    uint32_t bits = (uint32_t)length * DRV_SERVO_UART_BITS_PER_BYTE;
    uint32_t wire_ms = ((bits * 1000U) + baud - 1U) / baud;
    uint32_t timeout_ms = wire_ms + 5U;

    return (timeout_ms < DRV_SERVO_DMA_MIN_TIMEOUT_MS) ?
           DRV_SERVO_DMA_MIN_TIMEOUT_MS : timeout_ms;
}

static void servo_diag_capture_uart(UART_HandleTypeDef *huart)
{
    servo_diag.last_uart_state = (huart != NULL) ? (uint32_t)huart->gState : 0U;
    servo_diag.last_uart_error = (huart != NULL) ? huart->ErrorCode : 0U;
    servo_diag.last_dma_state = ((huart != NULL) && (huart->hdmatx != NULL)) ?
                                (uint32_t)huart->hdmatx->State : 0U;
    servo_diag.last_dma_error = ((huart != NULL) && (huart->hdmatx != NULL)) ?
                                huart->hdmatx->ErrorCode : 0U;
}

static void servo_try_recover_stuck_dma(UART_HandleTypeDef *huart)
{
    uint32_t elapsed_ms;

    if ((huart == NULL) || (servo_dma_start_tick_ms == 0U) ||
        (huart->gState == HAL_UART_STATE_READY)) {
        return;
    }

    elapsed_ms = HAL_GetTick() - servo_dma_start_tick_ms;
    if (elapsed_ms < servo_dma_timeout_ms) {
        return;
    }

    (void)HAL_UART_AbortTransmit(huart);
    servo_dma_start_tick_ms = 0U;
    servo_dma_timeout_ms = 0U;
    servo_dma_huart = NULL;
    if (servo_async_state == SERVO_ASYNC_FEEDBACK_TX) {
        servo_feedback_record_event(DRV_SERVO_FEEDBACK_EVENT_TIMEOUT,
                                    servo_feedback_pending_id,
                                    0U,
                                    0U,
                                    HAL_GetTick(),
                                    HAL_GetTick() - servo_feedback_start_tick_ms);
        servo_feedback_return_to_tx(huart);
    } else {
        servo_async_state = SERVO_ASYNC_IDLE;
    }
    servo_diag.tx_recover_count++;
    servo_diag_capture_uart(huart);
}

static DRV_SERVO_Status servo_from_hal(HAL_StatusTypeDef status)
{
    switch (status) {
    case HAL_OK:      return DRV_SERVO_OK;
    case HAL_BUSY:    return DRV_SERVO_BUSY;
    case HAL_TIMEOUT:  return DRV_SERVO_TIMEOUT;
    default:           return DRV_SERVO_ERROR;
    }
}

static uint8_t servo_is_valid_move(const DRV_SERVO_MoveCmd *move)
{
    if (move == NULL) { return 0U; }
    if ((move->pulse_us < DRV_SERVO_MIN_PULSE_US) ||
        (move->pulse_us > DRV_SERVO_MAX_PULSE_US)) { return 0U; }
    return 1U;
}

static DRV_SERVO_Status servo_build_move_many_command(const DRV_SERVO_MoveCmd *moves,
                                                      uint8_t count,
                                                      uint16_t time_ms,
                                                      char *command,
                                                      uint16_t command_size,
                                                      uint16_t *length)
{
    int written;
    uint32_t used = 0U;

    if ((moves == NULL) || (command == NULL) || (length == NULL) ||
        (count == 0U) || (count > DRV_SERVO_MAX_ITEMS) ||
        (time_ms > DRV_SERVO_MAX_TIME_MS) ||
        (command_size < 3U)) {
        return DRV_SERVO_INVALID_PARAM;
    }

    command[used++] = '{';
    command[used++] = 'G';
    command[used++] = '0';
    command[used++] = '0';
    command[used++] = '0';
    command[used++] = '0';

    for (uint8_t index = 0U; index < count; ++index) {
        if (servo_is_valid_move(&moves[index]) == 0U) {
            return DRV_SERVO_INVALID_PARAM;
        }

        written = snprintf(&command[used], (size_t)command_size - used,
                           "#%03uP%04uT%04u!",
                           (unsigned int)moves[index].id,
                           (unsigned int)moves[index].pulse_us,
                           (unsigned int)time_ms);
        if ((written < 0) || ((uint32_t)written >= ((uint32_t)command_size - used))) {
            return DRV_SERVO_INVALID_PARAM;
        }
        used += (uint32_t)written;
    }

    if ((used + 2U) > command_size) { return DRV_SERVO_INVALID_PARAM; }

    command[used++] = '}';
    command[used] = '\0';
    *length = (uint16_t)used;

    return DRV_SERVO_OK;
}

static DRV_SERVO_Status servo_send_id_command(DRV_SERVO_Device *dev,
                                              uint8_t id, const char *suffix)
{
    char command[24];
    int written;

    if (suffix == NULL) { return DRV_SERVO_INVALID_PARAM; }

    written = snprintf(command, sizeof(command), "#%03uP%s!", (unsigned int)id, suffix);
    if ((written < 0) || ((uint32_t)written >= sizeof(command))) {
        return DRV_SERVO_INVALID_PARAM;
    }

    return DRV_SERVO_SendRaw(dev, command);
}

DRV_SERVO_Status DRV_SERVO_SendRaw(DRV_SERVO_Device *dev, const char *command)
{
    size_t length;
    uint32_t timeout;

    if ((dev == NULL) || (dev->bus.huart == NULL) || (command == NULL)) {
        return DRV_SERVO_INVALID_PARAM;
    }
    if (servo_async_state != SERVO_ASYNC_IDLE) {
        return DRV_SERVO_BUSY;
    }

    length = strlen(command);
    if (length == 0U) { return DRV_SERVO_INVALID_PARAM; }

    timeout = (dev->bus.timeout_ms != 0U) ? dev->bus.timeout_ms
                                          : DRV_SERVO_TX_TIMEOUT_MS;

    return servo_from_hal(HAL_UART_Transmit(dev->bus.huart, (uint8_t *)command,
                                            (uint16_t)length, timeout));
}

uint16_t DRV_SERVO_ReadResponse(DRV_SERVO_Device *dev, char *buf, uint16_t max_len)
{
    uint16_t count = 0U;
    uint32_t byte_timeout = 10U;

    if ((dev == NULL) || (dev->bus.huart == NULL) ||
        (buf == NULL) || (max_len == 0U) ||
        (servo_async_state != SERVO_ASYNC_IDLE)) {
        return 0U;
    }

    HAL_HalfDuplex_EnableReceiver(dev->bus.huart);

    while (count < max_len) {
        HAL_StatusTypeDef status;
        status = HAL_UART_Receive(dev->bus.huart, (uint8_t *)&buf[count], 1U,
                                  byte_timeout);
        if (status != HAL_OK) { break; }
        count++;
    }

    HAL_HalfDuplex_EnableTransmitter(dev->bus.huart);
    return count;
}

uint32_t DRV_SERVO_GetBaudRate(const DRV_SERVO_Device *dev)
{
    return dev->bus.huart->Init.BaudRate;
}

DRV_SERVO_Status DRV_SERVO_SetBaudRate(DRV_SERVO_Device *dev, uint32_t baud_rate)
{
    UART_HandleTypeDef *huart;

    if ((dev == NULL) || (dev->bus.huart == NULL)) {
        return DRV_SERVO_INVALID_PARAM;
    }
    if (servo_async_state != SERVO_ASYNC_IDLE) {
        return DRV_SERVO_BUSY;
    }
    huart = dev->bus.huart;

    if ((baud_rate < DRV_SERVO_MIN_BAUDRATE) || (baud_rate > DRV_SERVO_MAX_BAUDRATE)) {
        return DRV_SERVO_INVALID_PARAM;
    }

    huart->Init.BaudRate = baud_rate;
    return servo_from_hal(HAL_HalfDuplex_Init(huart));
}

uint16_t DRV_SERVO_PositionToPulse(uint16_t position)
{
    uint32_t span = DRV_SERVO_MAX_PULSE_US - DRV_SERVO_MIN_PULSE_US;

    if (position > DRV_SERVO_POSITION_MAX) {
        position = DRV_SERVO_POSITION_MAX;
    }

    return (uint16_t)(DRV_SERVO_MIN_PULSE_US +
                      (((uint32_t)position * span) / DRV_SERVO_POSITION_MAX));
}

DRV_SERVO_Status DRV_SERVO_Move(DRV_SERVO_Device *dev, uint8_t id,
                                uint16_t pulse_us, uint16_t time_ms)
{
    char command[32];
    int written;

    if ((pulse_us < DRV_SERVO_MIN_PULSE_US) || (pulse_us > DRV_SERVO_MAX_PULSE_US) ||
        (time_ms > DRV_SERVO_MAX_TIME_MS)) {
        return DRV_SERVO_INVALID_PARAM;
    }

    written = snprintf(command, sizeof(command), "#%03uP%04uT%04u!",
                       (unsigned int)id, (unsigned int)pulse_us, (unsigned int)time_ms);
    if ((written < 0) || ((uint32_t)written >= sizeof(command))) {
        return DRV_SERVO_INVALID_PARAM;
    }

    return DRV_SERVO_SendRaw(dev, command);
}

DRV_SERVO_Status DRV_SERVO_MovePosition(DRV_SERVO_Device *dev, uint8_t id,
                                        uint16_t position, uint16_t time_ms)
{
    if (position > DRV_SERVO_POSITION_MAX) {
        return DRV_SERVO_INVALID_PARAM;
    }

    return DRV_SERVO_Move(dev, id, DRV_SERVO_PositionToPulse(position), time_ms);
}

DRV_SERVO_Status DRV_SERVO_MoveMany(DRV_SERVO_Device *dev,
                                    const DRV_SERVO_MoveCmd *moves,
                                    uint8_t count, uint16_t time_ms)
{
    char command[DRV_SERVO_COMMAND_BUFFER_SIZE];
    uint16_t length = 0U;
    DRV_SERVO_Status status;

    status = servo_build_move_many_command(moves, count, time_ms,
                                           command, (uint16_t)sizeof(command),
                                           &length);
    if (status != DRV_SERVO_OK) {
        return status;
    }
    return DRV_SERVO_SendRaw(dev, command);
}

DRV_SERVO_Status DRV_SERVO_MoveManyAsync(DRV_SERVO_Device *dev,
                                         const DRV_SERVO_MoveCmd *moves,
                                         uint8_t count, uint16_t time_ms)
{
    uint16_t length = 0U;
    DRV_SERVO_Status status;
    HAL_StatusTypeDef hal_status;

    if ((dev == NULL) || (dev->bus.huart == NULL)) {
        return DRV_SERVO_INVALID_PARAM;
    }

    if (dev->bus.huart->hdmatx == NULL) {
        servo_diag.last_status = DRV_SERVO_ERROR;
        servo_diag_capture_uart(dev->bus.huart);
        return DRV_SERVO_ERROR;
    }

    servo_try_recover_stuck_dma(dev->bus.huart);
    if ((servo_async_state != SERVO_ASYNC_IDLE) ||
        (dev->bus.huart->gState != HAL_UART_STATE_READY) ||
        (dev->bus.huart->RxState != HAL_UART_STATE_READY)) {
        servo_diag.tx_busy_count++;
        servo_diag.last_status = DRV_SERVO_BUSY;
        servo_diag_capture_uart(dev->bus.huart);
        return DRV_SERVO_BUSY;
    }

    status = servo_build_move_many_command(moves, count, time_ms,
                                           (char *)servo_dma_tx_buffer,
                                           (uint16_t)sizeof(servo_dma_tx_buffer),
                                           &length);
    if (status != DRV_SERVO_OK) {
        servo_diag.last_status = status;
        return status;
    }

    BSP_Cache_CleanDCache(servo_dma_tx_buffer, length);
    hal_status = HAL_UART_Transmit_DMA(dev->bus.huart, servo_dma_tx_buffer, length);

    if (hal_status == HAL_BUSY) {
        servo_diag.tx_busy_count++;
        servo_diag.last_status = DRV_SERVO_BUSY;
        servo_diag_capture_uart(dev->bus.huart);
        return DRV_SERVO_BUSY;
    }

    status = servo_from_hal(hal_status);
    servo_diag.last_status = status;
    servo_diag_capture_uart(dev->bus.huart);

    if (status == DRV_SERVO_OK) {
        servo_async_state = SERVO_ASYNC_MOVE_TX;
        servo_diag.tx_start_count++;
        servo_diag.last_length = length;
        servo_dma_timeout_ms = servo_estimate_dma_timeout_ms(dev->bus.huart, length);
        servo_dma_start_tick_ms = HAL_GetTick();
        servo_dma_huart = dev->bus.huart;
    } else {
        servo_async_state = SERVO_ASYNC_IDLE;
        servo_diag.tx_error_count++;
    }

    return status;
}

DRV_SERVO_Status DRV_SERVO_RequestPositionAsync(DRV_SERVO_Device *dev,
                                                uint8_t id,
                                                uint32_t timeout_ms)
{
    int written;
    uint16_t length;
    HAL_StatusTypeDef hal_status;
    DRV_SERVO_Status status;

    if ((dev == NULL) || (dev->bus.huart == NULL) ||
        (dev->bus.huart->hdmatx == NULL) ||
        (dev->bus.huart->hdmarx == NULL) ||
        (timeout_ms < DRV_SERVO_FEEDBACK_MIN_TIMEOUT_MS) ||
        (timeout_ms > DRV_SERVO_FEEDBACK_MAX_TIMEOUT_MS)) {
        return DRV_SERVO_INVALID_PARAM;
    }

    servo_try_recover_stuck_dma(dev->bus.huart);
    if ((servo_async_state != SERVO_ASYNC_IDLE) ||
        (dev->bus.huart->gState != HAL_UART_STATE_READY) ||
        (dev->bus.huart->RxState != HAL_UART_STATE_READY)) {
        servo_feedback_diag.busy_count++;
        return DRV_SERVO_BUSY;
    }

    written = snprintf((char *)servo_dma_tx_buffer,
                       sizeof(servo_dma_tx_buffer),
                       "#%03uPRAD!",
                       (unsigned int)id);
    if ((written <= 0) || ((uint32_t)written >= sizeof(servo_dma_tx_buffer))) {
        return DRV_SERVO_INVALID_PARAM;
    }
    length = (uint16_t)written;

    BSP_Cache_CleanDCache(servo_dma_tx_buffer, length);
    hal_status = HAL_UART_Transmit_DMA(dev->bus.huart,
                                      servo_dma_tx_buffer,
                                      length);
    status = servo_from_hal(hal_status);
    if (status != DRV_SERVO_OK) {
        if (status == DRV_SERVO_BUSY) {
            servo_feedback_diag.busy_count++;
        } else {
            servo_feedback_diag.uart_error_count++;
        }
        return status;
    }

    servo_async_state = SERVO_ASYNC_FEEDBACK_TX;
    servo_feedback_pending_id = id;
    servo_feedback_start_tick_ms = HAL_GetTick();
    servo_feedback_timeout_ms = timeout_ms;
    servo_feedback_diag.request_count++;
    servo_feedback_diag.pending = 1U;
    servo_feedback_diag.pending_id = id;
    servo_diag.tx_start_count++;
    servo_diag.last_length = length;
    servo_diag.last_status = DRV_SERVO_OK;
    servo_dma_timeout_ms = servo_estimate_dma_timeout_ms(dev->bus.huart, length);
    servo_dma_start_tick_ms = servo_feedback_start_tick_ms;
    servo_dma_huart = dev->bus.huart;
    servo_diag_capture_uart(dev->bus.huart);
    return DRV_SERVO_OK;
}

void DRV_SERVO_Service(DRV_SERVO_Device *dev, uint32_t now_ms)
{
    UART_HandleTypeDef *huart;
    uint32_t elapsed_ms;

    if ((dev == NULL) || (dev->bus.huart == NULL)) {
        return;
    }
    huart = dev->bus.huart;

    servo_try_recover_stuck_dma(huart);
    if ((servo_async_state != SERVO_ASYNC_FEEDBACK_TX) &&
        (servo_async_state != SERVO_ASYNC_FEEDBACK_RX)) {
        return;
    }

    elapsed_ms = now_ms - servo_feedback_start_tick_ms;
    if (elapsed_ms < servo_feedback_timeout_ms) {
        return;
    }

    if (servo_async_state == SERVO_ASYNC_FEEDBACK_TX) {
        (void)HAL_UART_AbortTransmit(huart);
    } else {
        (void)HAL_UART_AbortReceive(huart);
    }
    servo_feedback_record_event(DRV_SERVO_FEEDBACK_EVENT_TIMEOUT,
                                servo_feedback_pending_id,
                                0U,
                                0U,
                                now_ms,
                                elapsed_ms);
    servo_feedback_return_to_tx(huart);
}

uint8_t DRV_SERVO_IsBusIdle(const DRV_SERVO_Device *dev)
{
    if ((dev == NULL) || (dev->bus.huart == NULL)) {
        return 0U;
    }

    return ((servo_async_state == SERVO_ASYNC_IDLE) &&
            (dev->bus.huart->gState == HAL_UART_STATE_READY) &&
            (dev->bus.huart->RxState == HAL_UART_STATE_READY)) ? 1U : 0U;
}

void DRV_SERVO_GetDiag(DRV_SERVO_Diag *diag)
{
    if (diag == NULL) { return; }
    *diag = servo_diag;
}

void DRV_SERVO_GetFeedbackDiag(DRV_SERVO_FeedbackDiag *diag)
{
    uint32_t sequence_before;
    uint32_t sequence_after;

    if (diag == NULL) { return; }

    do {
        sequence_before = servo_feedback_diag.last_event.sequence;
        __DMB();
        *diag = servo_feedback_diag;
        __DMB();
        sequence_after = servo_feedback_diag.last_event.sequence;
    } while (sequence_before != sequence_after);
}

void DRV_SERVO_OnUartTxComplete(UART_HandleTypeDef *huart)
{
    HAL_StatusTypeDef rx_status;

    if ((huart == NULL) || (huart != servo_dma_huart)) {
        return;
    }

    servo_dma_start_tick_ms = 0U;
    servo_dma_timeout_ms = 0U;
    servo_diag.tx_complete_count++;
    servo_diag_capture_uart(huart);

    if (servo_async_state == SERVO_ASYNC_MOVE_TX) {
        servo_async_state = SERVO_ASYNC_IDLE;
        servo_dma_huart = NULL;
        return;
    }
    if (servo_async_state != SERVO_ASYNC_FEEDBACK_TX) {
        servo_async_state = SERVO_ASYNC_IDLE;
        servo_dma_huart = NULL;
        return;
    }

    HAL_HalfDuplex_EnableReceiver(huart);
    BSP_Cache_InvalidateDCache(servo_dma_rx_buffer,
                              (uint32_t)sizeof(servo_dma_rx_buffer));
    /* PRAD replies are fixed-length frames. A servo may pause between bytes,
     * so an IDLE event is not a frame boundary here. */
    rx_status = HAL_UART_Receive_DMA(huart,
                                    servo_dma_rx_buffer,
                                    DRV_SERVO_POSITION_RESPONSE_LEN);
    if (rx_status != HAL_OK) {
        servo_feedback_record_event(DRV_SERVO_FEEDBACK_EVENT_UART_ERROR,
                                    servo_feedback_pending_id,
                                    0U,
                                    0U,
                                    HAL_GetTick(),
                                    HAL_GetTick() - servo_feedback_start_tick_ms);
        servo_feedback_return_to_tx(huart);
        return;
    }

    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    servo_async_state = SERVO_ASYNC_FEEDBACK_RX;
}

void DRV_SERVO_OnUartRxComplete(UART_HandleTypeDef *huart)
{
    uint16_t position_us = 0U;
    uint32_t now_ms;
    uint32_t rtt_ms;
    DRV_SERVO_FeedbackEventType event_type;

    if ((huart == NULL) || (huart != servo_dma_huart) ||
        (servo_async_state != SERVO_ASYNC_FEEDBACK_RX)) {
        return;
    }

    now_ms = HAL_GetTick();
    rtt_ms = now_ms - servo_feedback_start_tick_ms;
    BSP_Cache_InvalidateDCache(servo_dma_rx_buffer,
                              (uint32_t)sizeof(servo_dma_rx_buffer));
    event_type = (servo_parse_position_response(servo_dma_rx_buffer,
                                                DRV_SERVO_POSITION_RESPONSE_LEN,
                                                servo_feedback_pending_id,
                                                &position_us) != 0U) ?
                 DRV_SERVO_FEEDBACK_EVENT_VALID :
                 DRV_SERVO_FEEDBACK_EVENT_PARSE_ERROR;
    servo_feedback_record_event(event_type,
                                servo_feedback_pending_id,
                                position_us,
                                DRV_SERVO_POSITION_RESPONSE_LEN,
                                now_ms,
                                rtt_ms);
    servo_feedback_return_to_tx(huart);
}

void DRV_SERVO_OnUartError(UART_HandleTypeDef *huart)
{
    if ((huart == NULL) || (huart != servo_dma_huart)) {
        return;
    }

    servo_diag.tx_error_count++;
    servo_diag.last_status = DRV_SERVO_ERROR;
    servo_diag_capture_uart(huart);

    if ((servo_async_state == SERVO_ASYNC_FEEDBACK_TX) ||
        (servo_async_state == SERVO_ASYNC_FEEDBACK_RX)) {
        servo_feedback_record_event(DRV_SERVO_FEEDBACK_EVENT_UART_ERROR,
                                    servo_feedback_pending_id,
                                    0U,
                                    0U,
                                    HAL_GetTick(),
                                    HAL_GetTick() - servo_feedback_start_tick_ms);
        servo_feedback_return_to_tx(huart);
    } else {
        servo_async_state = SERVO_ASYNC_IDLE;
        servo_dma_start_tick_ms = 0U;
        servo_dma_timeout_ms = 0U;
        servo_dma_huart = NULL;
    }
}

DRV_SERVO_Status DRV_SERVO_ReadVersion(DRV_SERVO_Device *dev, uint8_t id)
{ return servo_send_id_command(dev, id, "VER"); }

DRV_SERVO_Status DRV_SERVO_ReadId(DRV_SERVO_Device *dev, uint8_t id)
{ return servo_send_id_command(dev, id, "ID"); }

DRV_SERVO_Status DRV_SERVO_SetId(DRV_SERVO_Device *dev, uint8_t old_id, uint8_t new_id)
{
    char suffix[8];
    int written;
    written = snprintf(suffix, sizeof(suffix), "ID%03u", (unsigned int)new_id);
    if ((written < 0) || ((uint32_t)written >= sizeof(suffix))) {
        return DRV_SERVO_INVALID_PARAM;
    }
    return servo_send_id_command(dev, old_id, suffix);
}

DRV_SERVO_Status DRV_SERVO_ReadPosition(DRV_SERVO_Device *dev, uint8_t id)
{ return servo_send_id_command(dev, id, "RAD"); }

DRV_SERVO_Status DRV_SERVO_SetMode(DRV_SERVO_Device *dev, uint8_t id, uint8_t mode)
{
    char suffix[6];
    int written;
    if ((mode < DRV_SERVO_MIN_MODE) || (mode > DRV_SERVO_MAX_MODE)) {
        return DRV_SERVO_INVALID_PARAM;
    }
    written = snprintf(suffix, sizeof(suffix), "MOD%u", (unsigned int)mode);
    if ((written < 0) || ((uint32_t)written >= sizeof(suffix))) {
        return DRV_SERVO_INVALID_PARAM;
    }
    return servo_send_id_command(dev, id, suffix);
}

DRV_SERVO_Status DRV_SERVO_ReadMode(DRV_SERVO_Device *dev, uint8_t id)
{ return servo_send_id_command(dev, id, "MOD"); }

DRV_SERVO_Status DRV_SERVO_ReleaseTorque(DRV_SERVO_Device *dev, uint8_t id)
{ return servo_send_id_command(dev, id, "ULK"); }

DRV_SERVO_Status DRV_SERVO_RestoreTorque(DRV_SERVO_Device *dev, uint8_t id)
{ return servo_send_id_command(dev, id, "ULR"); }

DRV_SERVO_Status DRV_SERVO_Pause(DRV_SERVO_Device *dev, uint8_t id)
{ return servo_send_id_command(dev, id, "DPT"); }

DRV_SERVO_Status DRV_SERVO_Continue(DRV_SERVO_Device *dev, uint8_t id)
{ return servo_send_id_command(dev, id, "DCT"); }

DRV_SERVO_Status DRV_SERVO_Stop(DRV_SERVO_Device *dev, uint8_t id)
{ return servo_send_id_command(dev, id, "DST"); }

DRV_SERVO_Status DRV_SERVO_SetBaud(DRV_SERVO_Device *dev, uint8_t id, uint8_t baud_code)
{
    char suffix[6];
    int written;
    if (baud_code > DRV_SERVO_MAX_BAUD_CODE) { return DRV_SERVO_INVALID_PARAM; }
    written = snprintf(suffix, sizeof(suffix), "BD%u", (unsigned int)baud_code);
    if ((written < 0) || ((uint32_t)written >= sizeof(suffix))) {
        return DRV_SERVO_INVALID_PARAM;
    }
    return servo_send_id_command(dev, id, suffix);
}

DRV_SERVO_Status DRV_SERVO_SaveCenter(DRV_SERVO_Device *dev, uint8_t id)
{ return servo_send_id_command(dev, id, "SCK"); }

DRV_SERVO_Status DRV_SERVO_SetStartupPosition(DRV_SERVO_Device *dev, uint8_t id)
{ return servo_send_id_command(dev, id, "CSD"); }

DRV_SERVO_Status DRV_SERVO_ClearStartupPosition(DRV_SERVO_Device *dev, uint8_t id)
{ return servo_send_id_command(dev, id, "CSM"); }

DRV_SERVO_Status DRV_SERVO_RestoreStartupPosition(DRV_SERVO_Device *dev, uint8_t id)
{ return servo_send_id_command(dev, id, "CSR"); }

DRV_SERVO_Status DRV_SERVO_SetMinPosition(DRV_SERVO_Device *dev, uint8_t id)
{ return servo_send_id_command(dev, id, "SMI"); }

DRV_SERVO_Status DRV_SERVO_SetMaxPosition(DRV_SERVO_Device *dev, uint8_t id)
{ return servo_send_id_command(dev, id, "SMX"); }

DRV_SERVO_Status DRV_SERVO_FactoryResetKeepId(DRV_SERVO_Device *dev, uint8_t id)
{ return servo_send_id_command(dev, id, "CLEO"); }

DRV_SERVO_Status DRV_SERVO_FactoryResetFull(DRV_SERVO_Device *dev, uint8_t id)
{ return servo_send_id_command(dev, id, "CLE"); }

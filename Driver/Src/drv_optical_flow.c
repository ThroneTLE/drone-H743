#include "drv_optical_flow.h"

#include <string.h>

#define MSP2_PREAMBLE_LEN 3U
#define MSP2_HEADER_LEN 5U
#define MSP2_CHECKSUM_LEN 1U
#define MSP2_FRAME_HEADER_LEN (MSP2_PREAMBLE_LEN + MSP2_HEADER_LEN)
#define MSP2_MIN_FRAME_LEN (MSP2_FRAME_HEADER_LEN + MSP2_CHECKSUM_LEN)
#define MSP2_CRC8_DVB_S2_POLY 0xD5U

static uint16_t flow_get_u16_le(const uint8_t *data)
{
    return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8U));
}

static uint32_t flow_get_u32_le(const uint8_t *data)
{
    return (uint32_t)data[0] |
           ((uint32_t)data[1] << 8U) |
           ((uint32_t)data[2] << 16U) |
           ((uint32_t)data[3] << 24U);
}

static int32_t flow_get_i32_le(const uint8_t *data)
{
    return (int32_t)flow_get_u32_le(data);
}

static uint8_t flow_crc8_dvb_s2(uint8_t crc, uint8_t byte)
{
    crc ^= byte;
    for (uint8_t bit = 0U; bit < 8U; ++bit) {
        if ((crc & 0x80U) != 0U) {
            crc = (uint8_t)((uint8_t)(crc << 1U) ^ MSP2_CRC8_DVB_S2_POLY);
        } else {
            crc = (uint8_t)(crc << 1U);
        }
    }
    return crc;
}

static int16_t flow_i32_to_i16_sat(int32_t value)
{
    if (value > 32767L) {
        return 32767;
    }
    if (value < -32768L) {
        return -32768;
    }
    return (int16_t)value;
}

static uint8_t flow_u32_to_u8_sat(uint32_t value)
{
    if (value > 255UL) {
        return 255U;
    }
    return (uint8_t)value;
}

static uint16_t flow_u32_to_u16_sat(uint32_t value)
{
    if (value > 65535UL) {
        return 65535U;
    }
    return (uint16_t)value;
}

static void flow_reset_parser(DRV_OPTICAL_FLOW_Device *dev)
{
    dev->offset = 0U;
    dev->expected_len = 0U;
}

static uint16_t flow_sample_interval_us(const DRV_OPTICAL_FLOW_Device *dev,
                                        uint32_t time_ms,
                                        uint32_t received_ms)
{
    uint32_t dt_ms = 0UL;

    if (dev == NULL) {
        return 0U;
    }

    if ((dev->latest.time_ms != 0UL) && (time_ms >= dev->latest.time_ms)) {
        dt_ms = time_ms - dev->latest.time_ms;
    } else if ((dev->latest.received_ms != 0UL) &&
               (received_ms >= dev->latest.received_ms)) {
        dt_ms = received_ms - dev->latest.received_ms;
    }

    return flow_u32_to_u16_sat(dt_ms * 1000UL);
}

static uint8_t flow_checksum_ok(const uint8_t *frame, uint16_t length)
{
    uint8_t checksum = 0U;

    if ((frame == NULL) || (length < MSP2_MIN_FRAME_LEN)) {
        return 0U;
    }

    for (uint16_t i = MSP2_PREAMBLE_LEN; i < (uint16_t)(length - 1U); ++i) {
        checksum = flow_crc8_dvb_s2(checksum, frame[i]);
    }

    return (checksum == frame[length - 1U]) ? 1U : 0U;
}

static void flow_update_raw_stats(DRV_OPTICAL_FLOW_Device *dev,
                                  const DRV_OPTICAL_FLOW_Frame *frame)
{
    int32_t vx_sum = 0L;
    int32_t vy_sum = 0L;
    uint32_t interval_sum = 0UL;
    uint32_t distance_sum = 0UL;
    uint32_t strength_sum = 0UL;
    uint32_t quality_sum = 0UL;
    int16_t vx_min;
    int16_t vx_max;
    int16_t vy_min;
    int16_t vy_max;
    uint16_t interval_min;
    uint16_t interval_max;
    uint32_t distance_min;
    uint32_t distance_max;
    uint16_t count;

    if ((dev == NULL) || (frame == NULL)) {
        return;
    }

    dev->raw_window[dev->raw_window_pos] = *frame;
    dev->raw_window_pos++;
    if (dev->raw_window_pos >= DRV_OPTICAL_FLOW_RAW_WINDOW) {
        dev->raw_window_pos = 0U;
    }
    if (dev->raw_window_count < DRV_OPTICAL_FLOW_RAW_WINDOW) {
        dev->raw_window_count++;
    }

    count = dev->raw_window_count;
    if (count == 0U) {
        return;
    }

    vx_min = dev->raw_window[0].flow_vel_x;
    vx_max = vx_min;
    vy_min = dev->raw_window[0].flow_vel_y;
    vy_max = vy_min;
    interval_min = dev->raw_window[0].sample_interval_us;
    interval_max = interval_min;
    distance_min = dev->raw_window[0].distance_mm;
    distance_max = distance_min;

    for (uint16_t i = 0U; i < count; ++i) {
        const DRV_OPTICAL_FLOW_Frame *sample = &dev->raw_window[i];
        vx_sum += sample->flow_vel_x;
        vy_sum += sample->flow_vel_y;
        interval_sum += sample->sample_interval_us;
        distance_sum += sample->distance_mm;
        strength_sum += sample->strength;
        quality_sum += sample->flow_quality;

        if (sample->flow_vel_x < vx_min) { vx_min = sample->flow_vel_x; }
        if (sample->flow_vel_x > vx_max) { vx_max = sample->flow_vel_x; }
        if (sample->flow_vel_y < vy_min) { vy_min = sample->flow_vel_y; }
        if (sample->flow_vel_y > vy_max) { vy_max = sample->flow_vel_y; }
        if (sample->sample_interval_us < interval_min) {
            interval_min = sample->sample_interval_us;
        }
        if (sample->sample_interval_us > interval_max) {
            interval_max = sample->sample_interval_us;
        }
        if (sample->distance_mm < distance_min) {
            distance_min = sample->distance_mm;
        }
        if (sample->distance_mm > distance_max) {
            distance_max = sample->distance_mm;
        }
    }

    dev->raw_stats.count = count;
    dev->raw_stats.flow_vel_x_mean =
        flow_i32_to_i16_sat(vx_sum / (int32_t)count);
    dev->raw_stats.flow_vel_y_mean =
        flow_i32_to_i16_sat(vy_sum / (int32_t)count);
    dev->raw_stats.sample_interval_mean_us =
        flow_u32_to_u16_sat(interval_sum / (uint32_t)count);
    dev->raw_stats.distance_mean_mm = distance_sum / (uint32_t)count;
    dev->raw_stats.strength_mean =
        flow_u32_to_u8_sat(strength_sum / (uint32_t)count);
    dev->raw_stats.flow_quality_mean =
        flow_u32_to_u8_sat(quality_sum / (uint32_t)count);
    dev->raw_stats.flow_vel_x_peak_to_peak =
        flow_i32_to_i16_sat((int32_t)vx_max - (int32_t)vx_min);
    dev->raw_stats.flow_vel_y_peak_to_peak =
        flow_i32_to_i16_sat((int32_t)vy_max - (int32_t)vy_min);
    dev->raw_stats.sample_interval_peak_to_peak_us =
        (uint16_t)(interval_max - interval_min);
    dev->raw_stats.distance_peak_to_peak_mm = distance_max - distance_min;
}

static uint32_t flow_timeout_ms(const DRV_OPTICAL_FLOW_Device *dev)
{
    if ((dev != NULL) && (dev->bus.timeout_ms != 0U)) {
        return dev->bus.timeout_ms;
    }
    return 100U;
}

static void flow_restart_rx(DRV_OPTICAL_FLOW_Device *dev)
{
    HAL_StatusTypeDef status;

    if ((dev == NULL) || (dev->bus.huart == NULL)) {
        return;
    }

    if ((dev->bus.rx_dma_buffer != NULL) && (dev->bus.rx_dma_size != 0U) &&
        (dev->bus.huart->hdmarx != NULL)) {
        dev->rx_dma_pos = 0U;
        status = HAL_UARTEx_ReceiveToIdle_DMA(dev->bus.huart,
                                              dev->bus.rx_dma_buffer,
                                              dev->bus.rx_dma_size);
        if (status == HAL_OK) {
            __HAL_DMA_DISABLE_IT(dev->bus.huart->hdmarx, DMA_IT_HT);
        }
    } else {
        status = HAL_UART_Receive_IT(dev->bus.huart, &dev->rx_byte, 1U);
    }

    if (status == HAL_OK) {
        dev->rx_active = 1U;
        dev->rx_restarts++;
    } else {
        dev->rx_active = 0U;
        dev->uart_errors++;
        dev->last_uart_error = dev->bus.huart->ErrorCode;
    }
}

static uint16_t flow_dma_write_pos(DRV_OPTICAL_FLOW_Device *dev,
                                   uint16_t event_size)
{
    uint32_t remaining;

    if ((dev == NULL) || (dev->bus.huart == NULL) ||
        (dev->bus.huart->hdmarx == NULL) || (dev->bus.rx_dma_size == 0U)) {
        return 0U;
    }

    remaining = __HAL_DMA_GET_COUNTER(dev->bus.huart->hdmarx);
    if (remaining <= dev->bus.rx_dma_size) {
        return (uint16_t)(dev->bus.rx_dma_size - remaining);
    }

    if (event_size <= dev->bus.rx_dma_size) {
        return event_size;
    }

    return dev->rx_dma_pos;
}

static void flow_abort_rx(DRV_OPTICAL_FLOW_Device *dev)
{
    if ((dev == NULL) || (dev->bus.huart == NULL)) {
        return;
    }

    (void)HAL_UART_AbortReceive_IT(dev->bus.huart);
    if (dev->bus.huart->hdmarx != NULL) {
        (void)HAL_DMA_Abort(dev->bus.huart->hdmarx);
    }
    dev->rx_active = 0U;
}

static void flow_publish_latest(DRV_OPTICAL_FLOW_Device *dev,
                                DRV_OPTICAL_FLOW_Frame *parsed,
                                uint32_t received_ms)
{
    parsed->sample_interval_us =
        flow_sample_interval_us(dev, parsed->time_ms, received_ms);
    parsed->valid = ((parsed->distance_valid != 0U) &&
                     (parsed->flow_valid != 0U)) ?
                    DRV_OPTICAL_FLOW_VALID : 0U;
    parsed->received_ms = received_ms;

    dev->latest = *parsed;
    dev->last_rx_ms = parsed->received_ms;
    flow_update_raw_stats(dev, &dev->latest);
    dev->frames++;
}

static uint8_t flow_accept_msp2_frame(DRV_OPTICAL_FLOW_Device *dev,
                                      uint16_t length)
{
    const uint8_t *payload;
    DRV_OPTICAL_FLOW_Frame parsed;
    uint32_t received_ms;
    uint16_t cmd;
    uint16_t payload_len;
    uint8_t flags;

    if ((dev == NULL) || (length < MSP2_MIN_FRAME_LEN)) {
        return 0U;
    }

    flags = dev->frame[3];
    cmd = flow_get_u16_le(&dev->frame[4]);
    payload_len = flow_get_u16_le(&dev->frame[6]);
    payload = &dev->frame[MSP2_FRAME_HEADER_LEN];
    received_ms = HAL_GetTick();

    if ((cmd != DRV_OPTICAL_FLOW_RANGE_MSG_ID) &&
        (cmd != DRV_OPTICAL_FLOW_FLOW_MSG_ID)) {
        dev->ignored_messages++;
        return 0U;
    }

    if (((cmd == DRV_OPTICAL_FLOW_RANGE_MSG_ID) &&
         (payload_len < DRV_OPTICAL_FLOW_RANGE_PAYLOAD_LEN)) ||
        ((cmd == DRV_OPTICAL_FLOW_FLOW_MSG_ID) &&
         (payload_len < DRV_OPTICAL_FLOW_FLOW_PAYLOAD_LEN))) {
        dev->short_payload_errors++;
        return 0U;
    }

    parsed = dev->latest;
    parsed.msp_cmd = cmd;
    parsed.msp_flags = flags;
    parsed.time_ms = received_ms;

    if (cmd == DRV_OPTICAL_FLOW_RANGE_MSG_ID) {
        uint8_t quality = payload[0];
        parsed.distance_mm = flow_get_u32_le(&payload[1]);
        parsed.strength = quality;
        parsed.precision = 0U;
        parsed.tof_status = quality;
        parsed.distance_valid =
            ((quality != 0U) &&
             (parsed.distance_mm >= DRV_OPTICAL_FLOW_MIN_DISTANCE_MM)) ? 1U : 0U;
        parsed.distance_received_ms = received_ms;
    } else {
        parsed.flow_quality = payload[0];
        parsed.flow_status = payload[0];
        parsed.flow_vel_x = flow_i32_to_i16_sat(flow_get_i32_le(&payload[1]));
        parsed.flow_vel_y = flow_i32_to_i16_sat(flow_get_i32_le(&payload[5]));
        parsed.flow_valid = (parsed.flow_quality != 0U) ? 1U : 0U;
        parsed.flow_received_ms = received_ms;
    }

    flow_publish_latest(dev, &parsed, received_ms);
    return 1U;
}

uint8_t DRV_OPTICAL_FLOW_ConsumeByte(DRV_OPTICAL_FLOW_Device *dev, uint8_t byte)
{
    if (dev == NULL) {
        return 0U;
    }

    dev->bytes++;

    if (dev->offset == 0U) {
        if (byte != (uint8_t)DRV_OPTICAL_FLOW_MSP_HEAD_0) {
            return 0U;
        }
        dev->frame[dev->offset++] = byte;
        return 0U;
    }

    if (((dev->offset == 1U) &&
         (byte != (uint8_t)DRV_OPTICAL_FLOW_MSP_HEAD_1)) ||
        ((dev->offset == 2U) &&
         (byte != (uint8_t)DRV_OPTICAL_FLOW_MSP_HEAD_2))) {
        dev->frame_errors++;
        flow_reset_parser(dev);
        if (byte == (uint8_t)DRV_OPTICAL_FLOW_MSP_HEAD_0) {
            dev->frame[dev->offset++] = byte;
        }
        return 0U;
    }

    if (dev->offset >= DRV_OPTICAL_FLOW_FRAME_LEN) {
        dev->frame_errors++;
        flow_reset_parser(dev);
        if (byte == (uint8_t)DRV_OPTICAL_FLOW_MSP_HEAD_0) {
            dev->frame[dev->offset++] = byte;
        }
        return 0U;
    }

    dev->frame[dev->offset++] = byte;

    if (dev->offset == MSP2_FRAME_HEADER_LEN) {
        uint16_t payload_len = flow_get_u16_le(&dev->frame[6]);
        if (payload_len > DRV_OPTICAL_FLOW_MAX_PAYLOAD_LEN) {
            dev->frame_errors++;
            flow_reset_parser(dev);
            return 0U;
        }
        dev->expected_len =
            (uint16_t)MSP2_FRAME_HEADER_LEN + payload_len +
            (uint16_t)MSP2_CHECKSUM_LEN;
    }

    if ((dev->expected_len == 0U) || (dev->offset < dev->expected_len)) {
        return 0U;
    }

    {
        uint16_t completed_len = dev->expected_len;
        uint8_t restart_head =
            (byte == (uint8_t)DRV_OPTICAL_FLOW_MSP_HEAD_0) ? 1U : 0U;

        flow_reset_parser(dev);
        if (flow_checksum_ok(dev->frame, completed_len) == 0U) {
            dev->checksum_errors++;
            if (restart_head != 0U) {
                dev->frame[dev->offset++] = byte;
            }
            return 0U;
        }

        return flow_accept_msp2_frame(dev, completed_len);
    }
}

DRV_OPTICAL_FLOW_Status DRV_OPTICAL_FLOW_Init(DRV_OPTICAL_FLOW_Device *dev,
                                              const DRV_OPTICAL_FLOW_Bus *bus)
{
    UART_HandleTypeDef *huart;

    if ((dev == NULL) || (bus == NULL) || (bus->huart == NULL)) {
        return DRV_OPTICAL_FLOW_INVALID_ARG;
    }

    huart = bus->huart;
    memset(dev, 0, sizeof(*dev));
    dev->bus = *bus;
    if (dev->bus.baud_rate == 0U) {
        dev->bus.baud_rate = DRV_OPTICAL_FLOW_BAUD_RATE;
    }

    (void)HAL_UART_AbortReceive(huart);
    (void)HAL_UART_DeInit(huart);
    huart->Init.BaudRate = dev->bus.baud_rate;
    huart->Init.WordLength = UART_WORDLENGTH_8B;
    huart->Init.StopBits = UART_STOPBITS_1;
    huart->Init.Parity = UART_PARITY_NONE;
    huart->Init.Mode = UART_MODE_TX_RX;
    huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    if (HAL_UART_Init(huart) != HAL_OK) {
        dev->last_uart_error = huart->ErrorCode;
        dev->uart_errors++;
        return DRV_OPTICAL_FLOW_ERROR;
    }

    dev->initialized = 1U;
    flow_restart_rx(dev);
    return (dev->rx_active != 0U) ? DRV_OPTICAL_FLOW_OK : DRV_OPTICAL_FLOW_ERROR;
}

void DRV_OPTICAL_FLOW_Service(DRV_OPTICAL_FLOW_Device *dev)
{
    if ((dev == NULL) || (dev->bus.huart == NULL) || (dev->initialized == 0U)) {
        return;
    }

    if (dev->bus.huart->RxState != HAL_UART_STATE_BUSY_RX) {
        flow_restart_rx(dev);
    }
}

void DRV_OPTICAL_FLOW_OnUartRxCplt(DRV_OPTICAL_FLOW_Device *dev)
{
    if ((dev == NULL) || (dev->initialized == 0U)) {
        return;
    }

    (void)DRV_OPTICAL_FLOW_ConsumeByte(dev, dev->rx_byte);
    flow_restart_rx(dev);
}

void DRV_OPTICAL_FLOW_OnUartRxEvent(DRV_OPTICAL_FLOW_Device *dev,
                                    uint16_t size)
{
    uint16_t write_pos;

    if ((dev == NULL) || (dev->initialized == 0U) ||
        (dev->bus.rx_dma_buffer == NULL) || (dev->bus.rx_dma_size == 0U)) {
        return;
    }

    dev->dma_events++;
    dev->dma_last_size = (uint32_t)size;
    if (dev->bus.cache_invalidate != NULL) {
        dev->bus.cache_invalidate(dev->bus.rx_dma_buffer,
                                  dev->bus.rx_dma_size);
    }

    write_pos = flow_dma_write_pos(dev, size);
    if (write_pos >= dev->bus.rx_dma_size) {
        write_pos = 0U;
    }

    while (dev->rx_dma_pos != write_pos) {
        (void)DRV_OPTICAL_FLOW_ConsumeByte(dev,
                                           dev->bus.rx_dma_buffer[dev->rx_dma_pos]);
        dev->rx_dma_pos++;
        if (dev->rx_dma_pos >= dev->bus.rx_dma_size) {
            dev->rx_dma_pos = 0U;
        }
    }
}

void DRV_OPTICAL_FLOW_OnUartError(DRV_OPTICAL_FLOW_Device *dev,
                                  uint32_t error_code)
{
    UART_HandleTypeDef *huart;

    if ((dev == NULL) || (dev->bus.huart == NULL)) {
        return;
    }

    huart = dev->bus.huart;
    dev->uart_errors++;
    dev->last_uart_error = error_code;
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF |
                                 UART_CLEAR_PEF | UART_CLEAR_FEF);
    huart->ErrorCode = HAL_UART_ERROR_NONE;
    flow_abort_rx(dev);
    flow_reset_parser(dev);
    flow_restart_rx(dev);
}

void DRV_OPTICAL_FLOW_Invalidate(DRV_OPTICAL_FLOW_Device *dev)
{
    if (dev == NULL) {
        return;
    }

    if (dev->bus.huart != NULL) {
        flow_abort_rx(dev);
    }
    dev->initialized = 0U;
    dev->rx_active = 0U;
    flow_reset_parser(dev);
}

DRV_OPTICAL_FLOW_Status DRV_OPTICAL_FLOW_TransmitRaw(DRV_OPTICAL_FLOW_Device *dev,
                                                     const uint8_t *data,
                                                     uint16_t length,
                                                     uint32_t timeout_ms)
{
    UART_HandleTypeDef *huart;
    HAL_StatusTypeDef status;

    if ((dev == NULL) || (data == NULL) || (length == 0U) ||
        (dev->bus.huart == NULL)) {
        return DRV_OPTICAL_FLOW_INVALID_ARG;
    }

    huart = dev->bus.huart;
    flow_abort_rx(dev);
    flow_reset_parser(dev);
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF |
                                 UART_CLEAR_PEF | UART_CLEAR_FEF);
    huart->ErrorCode = HAL_UART_ERROR_NONE;

    status = HAL_UART_Transmit(huart, (uint8_t *)data, length,
                               (timeout_ms != 0U) ? timeout_ms :
                               flow_timeout_ms(dev));
    if (status != HAL_OK) {
        dev->uart_errors++;
        dev->last_uart_error = huart->ErrorCode;
        flow_restart_rx(dev);
        return DRV_OPTICAL_FLOW_ERROR;
    }

    flow_restart_rx(dev);
    return DRV_OPTICAL_FLOW_OK;
}

uint16_t DRV_OPTICAL_FLOW_ReceiveRaw(DRV_OPTICAL_FLOW_Device *dev,
                                     uint8_t *data,
                                     uint16_t max_length,
                                     uint32_t per_byte_timeout_ms)
{
    UART_HandleTypeDef *huart;
    uint16_t count = 0U;

    if ((dev == NULL) || (data == NULL) || (max_length == 0U) ||
        (dev->bus.huart == NULL)) {
        return 0U;
    }

    huart = dev->bus.huart;
    flow_abort_rx(dev);
    flow_reset_parser(dev);

    while (count < max_length) {
        HAL_StatusTypeDef status =
            HAL_UART_Receive(huart, &data[count], 1U,
                             (per_byte_timeout_ms != 0U) ?
                             per_byte_timeout_ms : flow_timeout_ms(dev));
        if (status != HAL_OK) {
            dev->last_uart_error = huart->ErrorCode;
            break;
        }
        count++;
    }

    flow_restart_rx(dev);
    return count;
}

DRV_OPTICAL_FLOW_Status DRV_OPTICAL_FLOW_TransceiveRaw(DRV_OPTICAL_FLOW_Device *dev,
                                                       const uint8_t *tx_data,
                                                       uint16_t tx_length,
                                                       uint8_t *rx_data,
                                                       uint16_t rx_length,
                                                       uint16_t *rx_count,
                                                       uint32_t timeout_ms)
{
    UART_HandleTypeDef *huart;
    HAL_StatusTypeDef status;
    uint16_t count = 0U;
    uint32_t effective_timeout;

    if (rx_count != NULL) {
        *rx_count = 0U;
    }

    if ((dev == NULL) || (tx_data == NULL) || (tx_length == 0U) ||
        (rx_data == NULL) || (rx_length == 0U) || (dev->bus.huart == NULL)) {
        return DRV_OPTICAL_FLOW_INVALID_ARG;
    }

    huart = dev->bus.huart;
    effective_timeout = (timeout_ms != 0U) ? timeout_ms : flow_timeout_ms(dev);
    flow_abort_rx(dev);
    flow_reset_parser(dev);
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF |
                                 UART_CLEAR_PEF | UART_CLEAR_FEF);
    huart->ErrorCode = HAL_UART_ERROR_NONE;

    status = HAL_UART_Transmit(huart, (uint8_t *)tx_data, tx_length,
                               effective_timeout);
    if (status != HAL_OK) {
        dev->uart_errors++;
        dev->last_uart_error = huart->ErrorCode;
        flow_restart_rx(dev);
        return DRV_OPTICAL_FLOW_ERROR;
    }

    while (count < rx_length) {
        status = HAL_UART_Receive(huart, &rx_data[count], 1U,
                                  effective_timeout);
        if (status != HAL_OK) {
            dev->last_uart_error = huart->ErrorCode;
            break;
        }
        count++;
    }

    if (rx_count != NULL) {
        *rx_count = count;
    }
    flow_restart_rx(dev);
    return DRV_OPTICAL_FLOW_OK;
}

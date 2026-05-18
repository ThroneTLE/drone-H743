#include "app_elrs.h"

#include "usart.h"
#include "bsp_cache.h"

#include <string.h>

/* ---- constants ---- */

#define APP_ELRS_DMA_RX_SIZE      256U
#define APP_ELRS_TX_BUF_SIZE      CRSF_MAX_FRAME_SIZE

/* ---- DMA buffers in RAM_D2 ---- */

__attribute__((section(".dma_buffer"), aligned(32)))
static uint8_t dma_rx_buf[APP_ELRS_DMA_RX_SIZE];

__attribute__((section(".dma_buffer"), aligned(32)))
static uint8_t tx_buf[APP_ELRS_TX_BUF_SIZE];

/* ---- state ---- */

static uint16_t dma_rx_pos;
static volatile uint8_t dma_started;
static volatile uint8_t tx_busy;
static uint8_t  tx_len;
static uint32_t rx_events;
static uint32_t rx_errors;
static uint32_t rx_restarts;

/* ---- helpers ---- */

static void ClearErrors(void)
{
    uint32_t err = HAL_UART_GetError(&huart4);
    if (err == HAL_UART_ERROR_NONE) return;

    __HAL_UART_CLEAR_FLAG(&huart4,
                          UART_CLEAR_OREF | UART_CLEAR_NEF |
                          UART_CLEAR_PEF | UART_CLEAR_FEF);
    huart4.ErrorCode = HAL_UART_ERROR_NONE;
    dma_started = 0U;
    (void)HAL_UART_AbortReceive(&huart4);
    rx_errors++;
}

static void StartRxDma(void)
{
    if (huart4.hdmarx == NULL)
        return;

    dma_rx_pos = 0U;
    HAL_StatusTypeDef status =
        HAL_UARTEx_ReceiveToIdle_DMA(&huart4, dma_rx_buf, APP_ELRS_DMA_RX_SIZE);
    if (status != HAL_OK) {
        rx_errors++;
        return;
    }

    __HAL_DMA_DISABLE_IT(huart4.hdmarx, DMA_IT_HT);
    BSP_Cache_InvalidateDCache(dma_rx_buf, APP_ELRS_DMA_RX_SIZE);
    dma_started = 1U;
    rx_restarts++;
}

static uint8_t DmaNeedsRestart(void)
{
    DMA_HandleTypeDef *hdma = huart4.hdmarx;
    if (hdma == NULL) return 1U;
    if (dma_started == 0U) return 1U;
    if (huart4.RxState != HAL_UART_STATE_BUSY_RX) return 1U;

    DMA_Stream_TypeDef *stream = (DMA_Stream_TypeDef *)hdma->Instance;
    return ((stream->CR & DMA_SxCR_EN) == 0U) ? 1U : 0U;
}

static uint16_t DmaWritePos(void)
{
    DMA_HandleTypeDef *hdma = huart4.hdmarx;
    if (hdma == NULL || dma_started == 0U)
        return dma_rx_pos;

    uint32_t remaining = __HAL_DMA_GET_COUNTER(hdma);
    if (remaining > APP_ELRS_DMA_RX_SIZE)
        return dma_rx_pos;
    return (uint16_t)(APP_ELRS_DMA_RX_SIZE - remaining);
}

/* ---- telemetry TX ---- */

static void StartTxDma(const uint8_t *frame, uint8_t len)
{
    if (huart4.hdmatx == NULL)
        return;

    tx_busy = 1U;
    memcpy(tx_buf, frame, len);
    tx_len = len;
    BSP_Cache_CleanDCache(tx_buf, len);

    if (HAL_UART_Transmit_DMA(&huart4, tx_buf, len) != HAL_OK) {
        tx_busy = 0U;
        rx_errors++;
    }
}

static void SendTelemetryFrame(uint8_t type, const uint8_t *payload, uint8_t payload_len)
{
    uint8_t frame[CRSF_MAX_FRAME_SIZE];
    uint8_t len = DRV_ELRS_BuildTelemetry(type, payload, payload_len, frame);
    if (len == 0U) return;

    if (tx_busy != 0U)
        return;  /* drop — previous frame still in flight */

    StartTxDma(frame, len);
}

/* ---- public API ---- */

void APP_ELRS_Init(void)
{
    DRV_ELRS_Init();

    dma_rx_pos  = 0U;
    dma_started = 0U;
    tx_busy     = 0U;
    tx_len      = 0U;
    rx_events   = 0U;
    rx_errors   = 0U;
    rx_restarts = 0U;

    StartRxDma();
}

void APP_ELRS_Step(void)
{
    ClearErrors();

    /* consume new bytes from DMA circular buffer */
    BSP_Cache_InvalidateDCache(dma_rx_buf, APP_ELRS_DMA_RX_SIZE);
    uint16_t write_pos = DmaWritePos();

    while (dma_rx_pos != write_pos) {
        DRV_ELRS_ProcessByte(dma_rx_buf[dma_rx_pos]);
        dma_rx_pos++;
        if (dma_rx_pos >= APP_ELRS_DMA_RX_SIZE)
            dma_rx_pos = 0U;
    }

    if (DmaNeedsRestart() != 0U) {
        dma_started = 0U;
        StartRxDma();
    }

    /* check TX completion */
    if (tx_busy != 0U) {
        if (huart4.gState == HAL_UART_STATE_READY &&
            huart4.ErrorCode == HAL_UART_ERROR_NONE) {
            tx_busy = 0U;
        }
    }
}

void APP_ELRS_GetChannels(uint16_t us_out[CRSF_CHANNEL_COUNT])
{
    DRV_ELRS_GetChannels(NULL, us_out);
}

/* ---- telemetry ---- */

void APP_ELRS_SendTelemetryAttitude(int16_t pitch_rad_x10000,
                                    int16_t roll_rad_x10000,
                                    int16_t yaw_rad_x10000)
{
    uint8_t payload[6];
    payload[0] = (uint8_t)(pitch_rad_x10000 >> 8);
    payload[1] = (uint8_t)(pitch_rad_x10000);
    payload[2] = (uint8_t)(roll_rad_x10000 >> 8);
    payload[3] = (uint8_t)(roll_rad_x10000);
    payload[4] = (uint8_t)(yaw_rad_x10000 >> 8);
    payload[5] = (uint8_t)(yaw_rad_x10000);
    SendTelemetryFrame(CRSF_FRAME_ATTITUDE, payload, sizeof(payload));
}

void APP_ELRS_SendTelemetryBaro(int32_t altitude_dm)
{
    uint8_t payload[3];
    uint16_t packed;
    if (altitude_dm < -10000)
        packed = 0U;
    else if (altitude_dm > (int32_t)0x7FFE * 10 - 5)
        packed = 0xFFFEU;
    else if (altitude_dm < (int32_t)0x8000 - 10000)
        packed = (uint16_t)(altitude_dm + 10000);
    else
        packed = (uint16_t)(((altitude_dm + 5) / 10) | 0x8000);

    payload[0] = (uint8_t)(packed >> 8);
    payload[1] = (uint8_t)(packed);
    payload[2] = 0;
    SendTelemetryFrame(CRSF_FRAME_BARO_ALTITUDE, payload, sizeof(payload));
}

void APP_ELRS_SendTelemetryBattery(uint16_t voltage_dv, uint16_t current_da,
                                   uint32_t capacity_mah, uint8_t remaining_pct)
{
    uint8_t payload[8];
    payload[0] = (uint8_t)(voltage_dv >> 8);
    payload[1] = (uint8_t)(voltage_dv);
    payload[2] = (uint8_t)(current_da >> 8);
    payload[3] = (uint8_t)(current_da);
    payload[4] = (uint8_t)(capacity_mah >> 16);
    payload[5] = (uint8_t)(capacity_mah >> 8);
    payload[6] = (uint8_t)(capacity_mah);
    payload[7] = remaining_pct;
    SendTelemetryFrame(CRSF_FRAME_BATTERY_SENSOR, payload, sizeof(payload));
}

void APP_ELRS_SendTelemetryGps(int32_t lat_e7, int32_t lon_e7,
                               uint16_t speed_kmh_x10,
                               uint16_t heading_deg_x100,
                               uint16_t altitude_m, uint8_t satellites)
{
    uint8_t payload[15];
    uint16_t alt_plus_1000 = (uint16_t)(1000U + altitude_m);
    payload[0]  = (uint8_t)((uint32_t)lat_e7 >> 24);
    payload[1]  = (uint8_t)((uint32_t)lat_e7 >> 16);
    payload[2]  = (uint8_t)((uint32_t)lat_e7 >> 8);
    payload[3]  = (uint8_t)(lat_e7);
    payload[4]  = (uint8_t)((uint32_t)lon_e7 >> 24);
    payload[5]  = (uint8_t)((uint32_t)lon_e7 >> 16);
    payload[6]  = (uint8_t)((uint32_t)lon_e7 >> 8);
    payload[7]  = (uint8_t)(lon_e7);
    payload[8]  = (uint8_t)(speed_kmh_x10 >> 8);
    payload[9]  = (uint8_t)(speed_kmh_x10);
    payload[10] = (uint8_t)(heading_deg_x100 >> 8);
    payload[11] = (uint8_t)(heading_deg_x100);
    payload[12] = (uint8_t)(alt_plus_1000 >> 8);
    payload[13] = (uint8_t)(alt_plus_1000);
    payload[14] = satellites;
    SendTelemetryFrame(CRSF_FRAME_GPS, payload, sizeof(payload));
}

/* ---- HAL callbacks ---- */

void APP_ELRS_OnRxEvent(uint16_t size)
{
    (void)size;
    rx_events++;
}

void APP_ELRS_OnTxComplete(void)
{
    tx_busy = 0U;
}

void APP_ELRS_OnError(void)
{
    tx_busy = 0U;
    dma_started = 0U;
    rx_errors++;
}

/* ---- diagnostics ---- */

uint32_t APP_ELRS_GetRcFrames(void)  { return DRV_ELRS_GetRcFrames(); }
uint32_t APP_ELRS_GetCrcErrors(void) { return DRV_ELRS_GetCrcErrors(); }
const DRV_ELRS_LinkStats *APP_ELRS_GetLinkStats(void) { return DRV_ELRS_GetLinkStats(); }

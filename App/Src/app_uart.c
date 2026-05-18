#include "app_uart.h"

#include "app_aiwb2.h"
#include "app_control.h"
#include "app_elrs.h"
#include "app_maint_uart.h"
#include "app_tasks.h"
#include "bsp_led.h"
#include "bsp_gps.h"
#include "bsp_uart.h"
#include "bsp_cache.h"

#include "usart.h"

#include <stdio.h>
#include <string.h>

#define APP_UART_BOOT_DIAG_ENABLED    0U
#define APP_UART_LINE_DEBUG_ENABLED   0U
#define APP_UART_PERIODIC_STATS_ENABLED 0U
#define APP_UART_DIRECT_CONTROL_ENABLED 1U
#define APP_UART_TX_WAIT_FOR_TRANSPARENT 1U //是否等待WIFI模块初始化
#define APP_UART_LEGACY_ASCII_CONTROL_ENABLED 1U
#define APP_UART_RX_USE_DMA 1U
#define APP_UART_RX_LINE_SIZE 128U
#define APP_UART_DMA_RX_SIZE  256U
#define APP_UART_TX_LED_ENABLED 0U
#define APP_UART_TX_LED_PULSE_MS 80U
#define APP_UART_RX_IDLE_LINE_MS 60U
#define APP_UART_DEBUG_LINE_LIMIT 64U
#define APP_UART_EVENT_RX     0x00000001U
#define APP_UART_EVENT_TX     0x00000002U
#define APP_UART_EVENT_KICK   0x00000004U
#define APP_UART_EVENT_ERROR  0x00000008U
#define APP_UART_WAIT_MS      20U

__attribute__((section(".dma_buffer"), aligned(32)))
static uint8_t app_uart_dma_rx_buffer[APP_UART_DMA_RX_SIZE];
__attribute__((section(".dma_buffer"), aligned(32)))
static uint8_t app_uart_tx_frame_buffer[APP_UART_TX_TEXT_SIZE + 16U];

static char app_uart_rx_line[APP_UART_RX_LINE_SIZE];
static APP_UART_TxMessage app_uart_tx_pending_message;
static uint16_t app_uart_rx_used;
static uint16_t app_uart_dma_rx_pos;
static uint32_t app_uart_rx_bytes;
static uint32_t app_uart_rx_lines;
static uint32_t app_uart_rx_idle_lines;
static uint32_t app_uart_rx_overflows;
static uint32_t app_uart_rx_errors;
static uint32_t app_uart_rx_events;
static uint32_t app_uart_rx_restarts;
static uint32_t app_uart_last_rx_event_size;
static uint32_t app_uart_last_stats_ms;
static uint32_t app_uart_last_rx_byte_ms;
static uint32_t app_uart_tx_led_until_ms;
static uint32_t app_uart_tx_count;
static uint32_t app_uart_last_tx_count;
static uint32_t app_uart_debug_lines;
static uint8_t app_uart_control_initialized;
static volatile uint8_t app_uart_dma_started;
static volatile uint8_t app_uart_it_rx_ready;
static volatile uint16_t app_uart_it_rx_size;
static volatile uint8_t app_uart_tx_busy;
static uint8_t app_uart_tx_pending_valid;

static char *app_uart_normalize_line(char *line, uint16_t length);
static void app_uart_ensure_control_ready(void);
static uint8_t app_uart_rx_dma_needs_restart(void);
static void app_uart_sync_tx_state(void);

static void app_uart_invalidate_rx_dma_buffer(void)
{
    BSP_Cache_InvalidateDCache(app_uart_dma_rx_buffer, APP_UART_DMA_RX_SIZE);
}

static void app_uart_clean_tx_dma_buffer(uint32_t length)
{
    if (length == 0U) { return; }
    BSP_Cache_CleanDCache(app_uart_tx_frame_buffer, length);
}

static void app_uart_signal(uint32_t flags)
{
    if (UARTTaskHandle != 0) {
        (void)osThreadFlagsSet(UARTTaskHandle, flags);
    }
}

static uint8_t app_uart_time_reached(uint32_t now_ms, uint32_t deadline_ms)
{
    return ((int32_t)(now_ms - deadline_ms) >= 0) ? 1U : 0U;
}

static void app_uart_prepare_tx_dma(void)
{
    DMA_HandleTypeDef *hdma = huart1.hdmatx;

    if (hdma == 0) {
        return;
    }

    if (hdma->Init.Mode == DMA_NORMAL) {
        return;
    }

    (void)HAL_DMA_Abort(hdma);
    (void)HAL_DMA_DeInit(hdma);
    hdma->Init.Mode = DMA_NORMAL;
    if (HAL_DMA_Init(hdma) != HAL_OK) {
        ++app_uart_rx_errors;
    }
}

static void app_uart_start_rx_dma(void)
{
    HAL_StatusTypeDef status;

#if (APP_UART_RX_USE_DMA != 0U)
    if (huart1.hdmarx == 0) {
        ++app_uart_rx_errors;
        return;
    }
#endif

    app_uart_dma_rx_pos = 0U;
#if (APP_UART_RX_USE_DMA != 0U)
    status = HAL_UARTEx_ReceiveToIdle_DMA(&huart1,
                                          app_uart_dma_rx_buffer,
                                          APP_UART_DMA_RX_SIZE);
#else
    app_uart_it_rx_ready = 0U;
    app_uart_it_rx_size = 0U;
    status = HAL_UARTEx_ReceiveToIdle_IT(&huart1,
                                         app_uart_dma_rx_buffer,
                                         APP_UART_DMA_RX_SIZE);
#endif
    if (status != HAL_OK) {
        ++app_uart_rx_errors;
        app_uart_dma_started = 0U;
        return;
    }

#if (APP_UART_RX_USE_DMA != 0U)
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
    app_uart_invalidate_rx_dma_buffer();
#endif
    app_uart_dma_started = 1U;
    ++app_uart_rx_restarts;
}

static uint8_t app_uart_rx_dma_needs_restart(void)
{
#if (APP_UART_RX_USE_DMA != 0U)
    DMA_HandleTypeDef *hdma = huart1.hdmarx;
    DMA_Stream_TypeDef *stream;

    if (hdma == 0) {
        return 1U;
    }

    if (app_uart_dma_started == 0U) {
        return 1U;
    }

    if (huart1.RxState != HAL_UART_STATE_BUSY_RX) {
        return 1U;
    }

    stream = (DMA_Stream_TypeDef *)hdma->Instance;
    if ((stream->CR & DMA_SxCR_EN) == 0U) {
        return 1U;
    }

    return 0U;
#else
    if (app_uart_dma_started == 0U) {
        return 1U;
    }

    if ((app_uart_it_rx_ready == 0U) &&
        (huart1.RxState != HAL_UART_STATE_BUSY_RX)) {
        return 1U;
    }

    return 0U;
#endif
}

static char *app_uart_normalize_line(char *line, uint16_t length)
{
    uint16_t start_index = 0U;
    uint16_t end_index = length;
    uint16_t out_index = 0U;

    while ((start_index < end_index) &&
           ((uint8_t)line[start_index] <= (uint8_t)' ')) {
        ++start_index;
    }

    if ((start_index < end_index) && (line[start_index] == '>')) {
        ++start_index;
        while ((start_index < end_index) &&
               ((uint8_t)line[start_index] <= (uint8_t)' ')) {
            ++start_index;
        }
    }

    while ((end_index > start_index) &&
           ((uint8_t)line[end_index - 1U] <= (uint8_t)' ')) {
        --end_index;
    }

    while (start_index < end_index) {
        line[out_index++] = line[start_index++];
    }

    line[out_index] = '\0';

    return line;
}

static void app_uart_report_line_debug(const uint8_t *data, uint16_t length)
{
#if (APP_UART_LINE_DEBUG_ENABLED == 0U)
    (void)data;
    (void)length;
    return;
#else
    char text[160];
    uint32_t offset = 0U;
    uint16_t shown = length;
    int written;

    if ((app_uart_control_initialized != 0U) ||
        (app_uart_debug_lines >= APP_UART_DEBUG_LINE_LIMIT)) {
        return;
    }

    if (shown > 16U) {
        shown = 16U;
    }

    written = snprintf(text,
                       sizeof(text),
                       "BOOT uart_line len=%u hex=",
                       (unsigned int)length);
    if (written <= 0) {
        return;
    }
    offset = (uint32_t)written;

    for (uint16_t index = 0U; index < shown; ++index) {
        written = snprintf(&text[offset],
                           sizeof(text) - offset,
                           "%02X%s",
                           (unsigned int)data[index],
                           ((index + 1U) < shown) ? " " : "");
        if (written <= 0) {
            return;
        }
        offset += (uint32_t)written;
        if (offset >= (sizeof(text) - 4U)) {
            break;
        }
    }

    if (shown < length) {
        written = snprintf(&text[offset], sizeof(text) - offset, " ...");
        if (written > 0) {
            offset += (uint32_t)written;
        }
    }

    if (offset < (sizeof(text) - 3U)) {
        text[offset++] = '\r';
        text[offset++] = '\n';
        text[offset] = '\0';
    }

    (void)BSP_UART_Transmit_USART1((const uint8_t *)text,
                                   (uint16_t)offset,
                                   100U);
    ++app_uart_debug_lines;
#endif
}

static void app_uart_clear_errors(void)
{
    uint32_t error = HAL_UART_GetError(&huart1);

    if (error == HAL_UART_ERROR_NONE) {
        return;
    }

    __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_OREF | UART_CLEAR_NEF |
                                   UART_CLEAR_PEF | UART_CLEAR_FEF);
    huart1.ErrorCode = HAL_UART_ERROR_NONE;
    app_uart_rx_used = 0U;
    app_uart_dma_started = 0U;
    (void)HAL_UART_AbortReceive(&huart1);
    ++app_uart_rx_errors;
}

void APP_UART_Task_Init(void)
{
#if (APP_UART_RX_USE_DMA != 0U)
    static const char boot_text[] = "BOOT uart_task_init rx=dma\r\n";
#else
    static const char boot_text[] = "BOOT uart_task_init rx=it\r\n";
#endif

#if (APP_UART_TX_LED_ENABLED != 0U)
    BSP_LED_Off(LED_1);
#endif
    app_uart_rx_used = 0U;
    app_uart_rx_bytes = 0U;
    app_uart_rx_lines = 0U;
    app_uart_rx_idle_lines = 0U;
    app_uart_rx_overflows = 0U;
    app_uart_rx_errors = 0U;
    app_uart_rx_events = 0U;
    app_uart_rx_restarts = 0U;
    app_uart_last_rx_event_size = 0U;
    app_uart_last_stats_ms = HAL_GetTick();
    app_uart_last_rx_byte_ms = app_uart_last_stats_ms;
    app_uart_tx_led_until_ms = app_uart_last_stats_ms;
    app_uart_tx_count = 0U;
    app_uart_last_tx_count = 0U;
    app_uart_debug_lines = 0U;
    app_uart_control_initialized = 0U;
    app_uart_dma_started = 0U;
    app_uart_it_rx_ready = 0U;
    app_uart_it_rx_size = 0U;
    app_uart_tx_busy = 0U;
    app_uart_tx_pending_valid = 0U;
#if (APP_UART_BOOT_DIAG_ENABLED != 0U)
    (void)BSP_UART_Transmit_USART1((const uint8_t *)boot_text,
                                   (uint16_t)(sizeof(boot_text) - 1U),
                                   100U);
#else
    (void)boot_text;
#endif
    APP_AiWB2_Init();
    APP_Task_MaintUART_Init();
    app_uart_prepare_tx_dma();
    app_uart_start_rx_dma();
}

static void app_uart_ensure_control_ready(void)
{
    static const char init_begin_text[] = "BOOT control_init_begin\r\n";
    static const char init_done_text[] = "BOOT control_init_done\r\n";

    if (app_uart_control_initialized != 0U) {
        return;
    }

#if (APP_UART_DIRECT_CONTROL_ENABLED == 0U)
    if (APP_AiWB2_IsTransparent() == 0U) {
        return;
    }
#elif (APP_UART_TX_WAIT_FOR_TRANSPARENT != 0U)
    if (APP_AiWB2_IsTransparent() == 0U) {
        return;
    }
#endif

#if (APP_UART_BOOT_DIAG_ENABLED != 0U)
    (void)BSP_UART_Transmit_USART1((const uint8_t *)init_begin_text,
                                   (uint16_t)(sizeof(init_begin_text) - 1U),
                                   100U);
#else
    (void)init_begin_text;
#endif
    APP_Control_Init();
#if (APP_UART_BOOT_DIAG_ENABLED != 0U)
    (void)BSP_UART_Transmit_USART1((const uint8_t *)init_done_text,
                                   (uint16_t)(sizeof(init_done_text) - 1U),
                                   100U);
#else
    (void)init_done_text;
#endif
    app_uart_control_initialized = 1U;
}

static void app_uart_handle_line(char *line, uint16_t length)
{
    char *normalized;
    uint8_t module_event;

    app_uart_report_line_debug((const uint8_t *)line, length);
    normalized = app_uart_normalize_line(line, length);
    if (*normalized == '\0') {
        return;
    }

    module_event = APP_AiWB2_ShouldConsumeTransparentLine(normalized);

    /* Sensor_Data:1/Stop: 直接到控制，不经过 AiWB2 */
    if ((strcmp(normalized, "Sensor_Data:1") == 0) ||
        (strcmp(normalized, "Sensor_Data:0") == 0)) {
        app_uart_ensure_control_ready();
        if (app_uart_control_initialized != 0U) {
            APP_Control_ProcessLine(normalized);
        }
        return;
    }

#if (APP_UART_LEGACY_ASCII_CONTROL_ENABLED != 0U)
    if (APP_AiWB2_IsControlPayload(normalized) != 0U) {
        app_uart_ensure_control_ready();
        if (app_uart_control_initialized != 0U) {
            APP_Control_ProcessLine(normalized);
            return;
        }
    }
#endif

    if (module_event != 0U) {
        APP_AiWB2_ProcessLine(normalized);
        return;
    }

    APP_AiWB2_ProcessLine(normalized);
}

static void app_uart_process_rx_byte(uint8_t byte)
{
    app_uart_last_rx_byte_ms = HAL_GetTick();
    ++app_uart_rx_bytes;

    if ((byte == '\n') || (byte == '\r')) {
        if (app_uart_rx_used > 0U) {
            app_uart_rx_line[app_uart_rx_used] = '\0';
            app_uart_handle_line(app_uart_rx_line, app_uart_rx_used);
            app_uart_rx_used = 0U;
            ++app_uart_rx_lines;
        }
        return;
    }

    if (app_uart_rx_used < (APP_UART_RX_LINE_SIZE - 1U)) {
        app_uart_rx_line[app_uart_rx_used++] = (char)byte;
    } else {
        app_uart_rx_used = 0U;
        ++app_uart_rx_overflows;
    }
}

static void app_uart_flush_idle_line(uint32_t now_ms)
{
    if (app_uart_rx_used == 0U) {
        return;
    }

    if (app_uart_time_reached(now_ms,
                              app_uart_last_rx_byte_ms + APP_UART_RX_IDLE_LINE_MS) == 0U) {
        return;
    }

    app_uart_rx_line[app_uart_rx_used] = '\0';
    app_uart_handle_line(app_uart_rx_line, app_uart_rx_used);
    app_uart_rx_used = 0U;
    ++app_uart_rx_lines;
    ++app_uart_rx_idle_lines;
}

#if (APP_UART_RX_USE_DMA != 0U)
static uint16_t app_uart_dma_write_pos(void)
{
    uint32_t remaining;

    if ((huart1.hdmarx == 0) || (app_uart_dma_started == 0U)) {
        return app_uart_dma_rx_pos;
    }

    remaining = __HAL_DMA_GET_COUNTER(huart1.hdmarx);
    if (remaining > APP_UART_DMA_RX_SIZE) {
        return app_uart_dma_rx_pos;
    }

    return (uint16_t)(APP_UART_DMA_RX_SIZE - remaining);
}
#endif

static void app_uart_poll_rx(void)
{
    app_uart_clear_errors();

#if (APP_UART_RX_USE_DMA == 0U)
    if (app_uart_it_rx_ready != 0U) {
        uint16_t rx_size = app_uart_it_rx_size;

        if (rx_size > APP_UART_DMA_RX_SIZE) {
            rx_size = APP_UART_DMA_RX_SIZE;
        }

        for (uint16_t index = 0U; index < rx_size; ++index) {
            app_uart_process_rx_byte(app_uart_dma_rx_buffer[index]);
        }

        app_uart_dma_rx_pos = 0U;
        app_uart_it_rx_ready = 0U;
        app_uart_it_rx_size = 0U;
        app_uart_dma_started = 0U;
        app_uart_start_rx_dma();
    }
#else
    uint16_t write_pos;

    app_uart_invalidate_rx_dma_buffer();
    write_pos = app_uart_dma_write_pos();
    while (app_uart_dma_rx_pos != write_pos) {
        app_uart_process_rx_byte(app_uart_dma_rx_buffer[app_uart_dma_rx_pos]);
        ++app_uart_dma_rx_pos;
        if (app_uart_dma_rx_pos >= APP_UART_DMA_RX_SIZE) {
            app_uart_dma_rx_pos = 0U;
        }
    }
#endif

    if (app_uart_rx_dma_needs_restart() != 0U) {
        app_uart_dma_started = 0U;
        app_uart_start_rx_dma();
    }
}

static void app_uart_poll_tx(void)
{
    uint16_t frame_length = 0U;
    HAL_StatusTypeDef status;

    app_uart_sync_tx_state();

    if (uartTxQueueHandle == 0) {
        return;
    }

    if (app_uart_control_initialized == 0U) {
        return;
    }
#if (APP_UART_DIRECT_CONTROL_ENABLED == 0U)
    if (APP_AiWB2_IsTransparent() == 0U) {
        return;
    }
#elif (APP_UART_TX_WAIT_FOR_TRANSPARENT != 0U)
    if (APP_AiWB2_IsTransparent() == 0U) {
        return;
    }
#endif
    if (app_uart_tx_busy != 0U) {
        return;
    }

    if (app_uart_tx_pending_valid == 0U) {
        if (osMessageQueueGet(uartTxQueueHandle, &app_uart_tx_pending_message, 0U, 0U) != osOK) {
            return;
        }
        app_uart_tx_pending_valid = 1U;
    }

    if (app_uart_tx_pending_message.length == 0U) {
        app_uart_tx_pending_valid = 0U;
        return;
    }

    frame_length = app_uart_tx_pending_message.length;
    if (frame_length == 0U) {
        app_uart_tx_pending_valid = 0U;
        return;
    }
    if (frame_length > (uint16_t)sizeof(app_uart_tx_frame_buffer)) {
        frame_length = (uint16_t)sizeof(app_uart_tx_frame_buffer);
    }
    memcpy(app_uart_tx_frame_buffer,
           app_uart_tx_pending_message.text,
           frame_length);

    if (huart1.hdmatx == 0) {
        status = BSP_UART_Transmit_USART1(app_uart_tx_frame_buffer,
                                          frame_length,
                                          100U);
        if (status == HAL_OK) {
            app_uart_tx_pending_valid = 0U;
            ++app_uart_tx_count;
#if (APP_UART_TX_LED_ENABLED != 0U)
            BSP_LED_On(LED_1);
#endif
            app_uart_tx_led_until_ms = HAL_GetTick() + APP_UART_TX_LED_PULSE_MS;
        }
        return;
    }

    app_uart_clean_tx_dma_buffer(frame_length);
    status = HAL_UART_Transmit_DMA(&huart1,
                                   app_uart_tx_frame_buffer,
                                   frame_length);
    if (status == HAL_OK) {
        app_uart_tx_busy = 1U;
        app_uart_tx_pending_valid = 0U;
        ++app_uart_tx_count;
#if (APP_UART_TX_LED_ENABLED != 0U)
        BSP_LED_On(LED_1);
#endif
        app_uart_tx_led_until_ms = HAL_GetTick() + APP_UART_TX_LED_PULSE_MS;
    } else if (status != HAL_BUSY) {
        ++app_uart_rx_errors;
    }
}

static void app_uart_sync_tx_state(void)
{
    uint32_t cr1;
    uint32_t cr3;
    uint32_t isr;

    if (app_uart_tx_busy == 0U) {
        return;
    }

    cr1 = huart1.Instance->CR1;
    cr3 = huart1.Instance->CR3;
    isr = huart1.Instance->ISR;

    /*
     * TX DMA normal mode on H7 completes in two phases:
     * 1) DMA transfer complete clears DMAT and enables TCIE
     * 2) UART TC interrupt calls HAL_UART_TxCpltCallback
     *
     * During debug we have seen cases where HAL has already transitioned back
     * to READY with no error, but our local busy flag remains set. Rely on HAL
     * state as the source of truth so the TX path cannot deadlock waiting for a
     * callback edge we may have missed.
     */
    if ((huart1.gState == HAL_UART_STATE_READY) &&
        (huart1.ErrorCode == HAL_UART_ERROR_NONE)) {
        app_uart_tx_busy = 0U;
        return;
    }

    if (((cr3 & USART_CR3_DMAT) == 0U) &&
        ((cr1 & USART_CR1_TCIE) == 0U) &&
        ((isr & USART_ISR_TC) != 0U) &&
        (huart1.ErrorCode == HAL_UART_ERROR_NONE)) {
        app_uart_tx_busy = 0U;
    }
}

static void app_uart_update_tx_led(uint32_t now_ms)
{
#if (APP_UART_TX_LED_ENABLED == 0U)
    (void)now_ms;
    return;
#else
    uint32_t tx_count = app_uart_tx_count;

    if (tx_count != app_uart_last_tx_count) {
        app_uart_last_tx_count = tx_count;
        app_uart_tx_led_until_ms = now_ms + APP_UART_TX_LED_PULSE_MS;
        BSP_LED_On(LED_1);
        return;
    }

    if (app_uart_time_reached(now_ms, app_uart_tx_led_until_ms) != 0U) {
        BSP_LED_Off(LED_1);
    }
#endif
}

void APP_UART_Task_Step(void)
{
    uint32_t flags;
    uint32_t now_ms;

    app_uart_poll_rx();
    APP_Task_MaintUART_Step();
    now_ms = HAL_GetTick();
    app_uart_flush_idle_line(now_ms);
    APP_AiWB2_Tick();

    /* PC13 LED: 初始慢闪 → 透传快闪 */
    {
        static uint32_t led_toggle_ms;
        uint32_t period_ms = APP_AiWB2_IsTransparent() ? 120U : 500U;

        if ((now_ms - led_toggle_ms) >= period_ms) {
            led_toggle_ms = now_ms;
            BSP_LED_Toggle(LED_RED);
        }
    }

    app_uart_ensure_control_ready();
    if ((app_uart_control_initialized != 0U)
#if (APP_UART_DIRECT_CONTROL_ENABLED == 0U)
        && (APP_AiWB2_IsTransparent() != 0U)
#endif
       ) {
        APP_Control_Tick();
    }
#if (APP_UART_PERIODIC_STATS_ENABLED != 0U)
    if ((app_uart_control_initialized != 0U) &&
#if (APP_UART_DIRECT_CONTROL_ENABLED == 0U)
        (APP_AiWB2_IsTransparent() != 0U) &&
#endif
        ((now_ms - app_uart_last_stats_ms) >= 2000U)) {
        app_uart_last_stats_ms = now_ms;
        APP_Control_ReportUartStats(app_uart_rx_bytes,
                                    app_uart_rx_lines,
                                    app_uart_rx_overflows,
                                    app_uart_rx_errors);
    } else if ((app_uart_control_initialized == 0U) &&
               ((now_ms - app_uart_last_stats_ms) >= 2000U)) {
        char stats_text[192];
        int written;

        app_uart_last_stats_ms = now_ms;
        written = snprintf(stats_text,
                           sizeof(stats_text),
                           "BOOT uart_wait_control rx_bytes=%lu rx_lines=%lu rx_idle=%lu rx_overflows=%lu rx_errors=%lu rx_state=%u dma_started=%u trans=%u ctrl=%u\r\n",
                           (unsigned long)app_uart_rx_bytes,
                           (unsigned long)app_uart_rx_lines,
                           (unsigned long)app_uart_rx_idle_lines,
                           (unsigned long)app_uart_rx_overflows,
                           (unsigned long)app_uart_rx_errors,
                           (unsigned int)huart1.RxState,
                           (unsigned int)app_uart_dma_started,
                           (unsigned int)APP_AiWB2_IsTransparent(),
                           (unsigned int)app_uart_control_initialized);
        if (written > 0) {
            uint16_t length = (uint16_t)written;

            if ((uint32_t)written >= sizeof(stats_text)) {
                length = (uint16_t)(sizeof(stats_text) - 1U);
            }
            (void)BSP_UART_Transmit_USART1((const uint8_t *)stats_text,
                                           length,
                                           100U);
        }
    }
#else
    app_uart_last_stats_ms = now_ms;
#endif
    app_uart_poll_tx();
    app_uart_update_tx_led(HAL_GetTick());
    flags = osThreadFlagsWait(APP_UART_EVENT_RX |
                              APP_UART_EVENT_TX |
                              APP_UART_EVENT_KICK |
                              APP_UART_EVENT_ERROR,
                              osFlagsWaitAny,
                              APP_UART_WAIT_MS);
    (void)flags;
}

void APP_UART_GetStats(uint32_t *rx_bytes,
                       uint32_t *rx_lines,
                       uint32_t *rx_overflows,
                       uint32_t *rx_errors)
{
    if (rx_bytes != NULL) {
        *rx_bytes = app_uart_rx_bytes;
    }
    if (rx_lines != NULL) {
        *rx_lines = app_uart_rx_lines;
    }
    if (rx_overflows != NULL) {
        *rx_overflows = app_uart_rx_overflows;
    }
    if (rx_errors != NULL) {
        *rx_errors = app_uart_rx_errors;
    }
}

void APP_UART_GetRxEventStats(uint32_t *rx_events,
                              uint32_t *rx_restarts,
                              uint32_t *last_rx_event_size)
{
    if (rx_events != NULL) {
        *rx_events = app_uart_rx_events;
    }
    if (rx_restarts != NULL) {
        *rx_restarts = app_uart_rx_restarts;
    }
    if (last_rx_event_size != NULL) {
        *last_rx_event_size = app_uart_last_rx_event_size;
    }
}

void APP_UART_NotifyTxPending(void)
{
    app_uart_signal(APP_UART_EVENT_KICK);
}

void APP_UART_OnRxEvent(UART_HandleTypeDef *huart, uint16_t size)
{
    if ((huart == NULL) || (huart->Instance != USART1)) {
        return;
    }

    app_uart_last_rx_event_size = (uint32_t)size;
    ++app_uart_rx_events;
#if (APP_UART_RX_USE_DMA == 0U)
    app_uart_it_rx_size = size;
    app_uart_it_rx_ready = 1U;
    app_uart_dma_started = 0U;
#endif
    app_uart_signal(APP_UART_EVENT_RX);
}

void APP_UART_OnTxComplete(UART_HandleTypeDef *huart)
{
    if ((huart == NULL) || (huart->Instance != USART1)) {
        return;
    }

    app_uart_tx_busy = 0U;
    app_uart_signal(APP_UART_EVENT_TX);
}

void APP_UART_OnError(UART_HandleTypeDef *huart)
{
    if ((huart == NULL) || (huart->Instance != USART1)) {
        return;
    }

    if (app_uart_tx_busy != 0U) {
        app_uart_tx_busy = 0U;
    }
    app_uart_dma_started = 0U;
    app_uart_it_rx_ready = 0U;
    app_uart_it_rx_size = 0U;
    app_uart_signal(APP_UART_EVENT_ERROR);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == UART4) {
        APP_ELRS_OnRxEvent(Size);
        return;
    }
    APP_UART_OnRxEvent(huart, Size);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART4) {
        APP_ELRS_OnTxComplete();
        return;
    }
    APP_UART_OnTxComplete(huart);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    BSP_GPS_OnUartRxCplt(huart);
    APP_MaintUART_OnRxCplt(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART4) {
        APP_ELRS_OnError();
        return;
    }
    APP_UART_OnError(huart);
    BSP_GPS_OnUartError(huart);
    APP_MaintUART_OnError(huart);
}

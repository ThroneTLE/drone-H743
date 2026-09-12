#include "app_uart.h"

#include "app_aiwb2.h"
#include "app_control.h"
#include "app_elrs.h"
#include "app_led.h"
#include "app_maint_uart.h"
#include "app_optical_flow.h"
#include "app_telem_stream.h"
#include "app_usb_cdc.h"
#include "bsp_optical_flow.h"
#include "app_tasks.h"
#include "bsp_aiwb2_power.h"
#include "bsp_led.h"
#include "bsp_uart.h"
#include "bsp_uart_events.h"
#include "bsp_cache.h"
#include "drv_servo.h"
#include "svc_timestamp.h"

#include "bsp_uart_link.h"

#include <stdio.h>
#include <string.h>

#define APP_UART_BOOT_DIAG_ENABLED    0U
#define APP_UART_LINE_DEBUG_ENABLED   0U
#define APP_UART_PERIODIC_STATS_ENABLED 0U
#define APP_UART_DISABLE_USART1       0U  /* 设为 1 释放 USART1 给烧录工具 */
#define APP_UART_DIRECT_SERIAL_MODE   1U  /* 设为 1 绕过 WiFi 状态机，USART1 直连数传 */
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
#define APP_UART_SOCKET_SEND_PROMPT_TIMEOUT_MS 200U
#define APP_UART_SOCKET_SEND_RESULT_TIMEOUT_MS 1000U

__attribute__((section(".dma_buffer"), aligned(32)))
static uint8_t app_uart_dma_rx_buffer[APP_UART_DMA_RX_SIZE];
__attribute__((section(".dma_buffer"), aligned(32)))
static uint8_t app_uart_tx_frame_buffer[APP_UART_TX_TEXT_SIZE + 16U];

typedef enum {
    APP_UART_SOCKET_TX_IDLE = 0,
    APP_UART_SOCKET_TX_WAIT_PROMPT,
    APP_UART_SOCKET_TX_WAIT_PAYLOAD_DMA,
    APP_UART_SOCKET_TX_WAIT_RESULT
} APP_UART_SocketTxState;

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
static APP_UART_SocketTxState app_uart_socket_tx_state;
static uint16_t app_uart_socket_payload_length;
static uint32_t app_uart_socket_deadline_ms;

static char *app_uart_normalize_line(char *line, uint16_t length);
static void app_uart_ensure_control_ready(void);
static uint8_t app_uart_rx_dma_needs_restart(void);
static void app_uart_sync_tx_state(void);
static void app_uart_socket_tx_reset(void);

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

static void app_uart_socket_tx_reset(void)
{
    app_uart_socket_tx_state = APP_UART_SOCKET_TX_IDLE;
    app_uart_socket_payload_length = 0U;
    app_uart_socket_deadline_ms = 0U;
    app_uart_tx_busy = 0U;
    (void)APP_AiWB2_TakeSocketSendPrompt();
    (void)APP_AiWB2_TakeSocketSendResult();
}

static void app_uart_prepare_tx_dma(void)
{
    BSP_UartLink_ForceTxDmaNormalMode(BSP_UART_ROLE_TELEMETRY);
}

static void app_uart_start_rx_dma(void)
{
#if (APP_UART_DISABLE_USART1 != 0U)
    (void)app_uart_rx_errors;
    return;
#else
    uint8_t started;

#if (APP_UART_RX_USE_DMA != 0U)
    if (BSP_UartLink_HasRxDma(BSP_UART_ROLE_TELEMETRY) == 0U) {
        ++app_uart_rx_errors;
        return;
    }
#endif

    app_uart_dma_rx_pos = 0U;
#if (APP_UART_RX_USE_DMA != 0U)
    started = BSP_UartLink_StartRxToIdle(BSP_UART_ROLE_TELEMETRY,
                                         app_uart_dma_rx_buffer,
                                         APP_UART_DMA_RX_SIZE);
#else
    app_uart_it_rx_ready = 0U;
    app_uart_it_rx_size = 0U;
    started = BSP_UartLink_StartRxToIdleIt(BSP_UART_ROLE_TELEMETRY,
                                           app_uart_dma_rx_buffer,
                                           APP_UART_DMA_RX_SIZE);
#endif
    if (started == 0U) {
        ++app_uart_rx_errors;
        app_uart_dma_started = 0U;
        return;
    }

#if (APP_UART_RX_USE_DMA != 0U)
    app_uart_invalidate_rx_dma_buffer();
#endif
    app_uart_dma_started = 1U;
    ++app_uart_rx_restarts;
#endif
}

static uint8_t app_uart_rx_dma_needs_restart(void)
{
#if (APP_UART_RX_USE_DMA != 0U)
    if (app_uart_dma_started == 0U) {
        return 1U;
    }

    return (BSP_UartLink_RxIsRunning(BSP_UART_ROLE_TELEMETRY) == 0U) ? 1U : 0U;
#else
    if (app_uart_dma_started == 0U) {
        return 1U;
    }

    if ((app_uart_it_rx_ready == 0U) &&
        (BSP_UartLink_RxIsRunning(BSP_UART_ROLE_TELEMETRY) == 0U)) {
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
    /* 取和清是同一次调用，否则两步之间新来的错误会被无声吃掉。 */
    if (BSP_UartLink_TakeErrors(BSP_UART_ROLE_TELEMETRY) == 0U) {
        return;
    }

    BSP_UartLink_FlushRx(BSP_UART_ROLE_TELEMETRY);
    app_uart_rx_used = 0U;
    app_uart_dma_started = 0U;
    BSP_UartLink_AbortRx(BSP_UART_ROLE_TELEMETRY);
    ++app_uart_rx_errors;
}

static const BSP_UartRoleHandlers app_uart_role_handlers = {
    .rx_event = APP_UART_OnRxEvent,
    .rx_byte  = NULL,        /* 数传走 IDLE+DMA，不用逐字节中断 */
    .tx_cplt  = APP_UART_OnTxComplete,
    .error    = APP_UART_OnError,
};

void APP_UART_Task_Init(void)
{
    BSP_UartEvents_Register(BSP_UART_ROLE_TELEMETRY, &app_uart_role_handlers);
    APP_USB_CDC_Init();

#if (APP_UART_DISABLE_USART1 != 0U)
    /* 彻底释放引脚：DeInit 关闭时钟、NVIC，GPIO 复位为高阻态 */
    BSP_UartLink_Release(BSP_UART_ROLE_TELEMETRY);
    return;
#else
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
    app_uart_last_stats_ms = SVC_Timestamp_Ms();
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
    app_uart_socket_tx_reset();
#if (APP_UART_BOOT_DIAG_ENABLED != 0U)
    (void)BSP_UART_Transmit_USART1((const uint8_t *)boot_text,
                                   (uint16_t)(sizeof(boot_text) - 1U),
                                   100U);
#else
    (void)boot_text;
#endif
#if (APP_UART_DIRECT_SERIAL_MODE == 0U)
    APP_AiWB2_Init();
#else
    BSP_AiWB2_SetEnabled(0U);  /* 数传模式：关闭 WiFi 模块电源 */
#endif
    APP_Task_MaintUART_Init();
    app_uart_prepare_tx_dma();
    app_uart_start_rx_dma();
#endif
}

static void app_uart_ensure_control_ready(void)
{
    static const char init_begin_text[] = "BOOT control_init_begin\r\n";
    static const char init_done_text[] = "BOOT control_init_done\r\n";

    if (app_uart_control_initialized != 0U) {
        return;
    }

#if (APP_UART_DIRECT_SERIAL_MODE == 0U)
#if (APP_UART_DIRECT_CONTROL_ENABLED == 0U)
    if ((APP_AiWB2_IsTransparent() == 0U) &&
        (APP_AiWB2_GetState() != APP_AIWB2_STATE_SOCKET_READY)) {
        return;
    }
#elif (APP_UART_TX_WAIT_FOR_TRANSPARENT != 0U)
    if ((APP_AiWB2_IsTransparent() == 0U) &&
        (APP_AiWB2_GetState() != APP_AIWB2_STATE_SOCKET_READY)) {
        return;
    }
#endif
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

    app_uart_report_line_debug((const uint8_t *)line, length);
    normalized = app_uart_normalize_line(line, length);
    if (*normalized == '\0') {
        return;
    }

    /*
     * `TELEM SINK auto` 靠这条记录决定 `TELEM STREAM on` 之后往哪条链路发帧。
     * 放在这里而不是每个分支里：数传直连、AiWB2 透传、legacy ASCII 三条路
     * 都从这个漏斗过，漏一条就会出现"命令从数传进来、帧却发去了 USB"。
     */
    APP_TelemStream_NoteCommandSource(APP_TELEM_SINK_UART);

#if (APP_UART_DIRECT_SERIAL_MODE != 0U)
    /* 数传直连模式：所有 RX 数据直接送控制处理，不经过 WiFi 状态机 */
    app_uart_ensure_control_ready();
    if (app_uart_control_initialized != 0U) {
        APP_Control_ProcessLine(normalized);
    }
    return;
#else
    {
        uint8_t module_event;

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
#endif
}

static void app_uart_process_rx_byte(uint8_t byte)
{
    app_uart_last_rx_byte_ms = SVC_Timestamp_Ms();
    ++app_uart_rx_bytes;

#if (APP_UART_DIRECT_SERIAL_MODE == 0U)
    if ((byte == (uint8_t)'>') && (app_uart_rx_used == 0U)) {
        APP_AiWB2_ProcessLine(">");
        ++app_uart_rx_lines;
        return;
    }
#endif

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
    if (app_uart_dma_started == 0U) {
        return app_uart_dma_rx_pos;
    }

    return BSP_UartLink_RxFilled(BSP_UART_ROLE_TELEMETRY,
                                 APP_UART_DMA_RX_SIZE);
}
#endif

static void app_uart_poll_rx(void)
{
#if (APP_UART_DISABLE_USART1 != 0U)
    return;
#else
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
#endif
}

static void app_uart_poll_tx(void)
{
#if (APP_UART_DISABLE_USART1 != 0U)
    return;
#else
    uint16_t frame_length = 0U;
    HAL_StatusTypeDef status;
#if (APP_UART_DIRECT_SERIAL_MODE == 0U)
    uint32_t now_ms = SVC_Timestamp_Ms();
#endif

    app_uart_sync_tx_state();

    if (uartTxQueueHandle == 0) {
        return;
    }

    if (app_uart_control_initialized == 0U) {
        return;
    }

#if (APP_UART_DIRECT_SERIAL_MODE != 0U)
    /* 数传直连模式：跳过所有 AiWB2 socket 协议，直接 DMA 发送 */

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
    if (frame_length > (uint16_t)sizeof(app_uart_tx_frame_buffer)) {
        frame_length = (uint16_t)sizeof(app_uart_tx_frame_buffer);
    }

    memcpy(app_uart_tx_frame_buffer,
           app_uart_tx_pending_message.text,
           frame_length);

    if (BSP_UartLink_HasTxDma(BSP_UART_ROLE_TELEMETRY) == 0U) {
        status = BSP_UART_Transmit_USART1(app_uart_tx_frame_buffer,
                                          frame_length,
                                          100U);
        if (status == HAL_OK) {
            ++app_uart_tx_count;
        } else {
            ++app_uart_rx_errors;
        }
    } else {
        app_uart_clean_tx_dma_buffer(frame_length);
        status = BSP_UartLink_TransmitDma(BSP_UART_ROLE_TELEMETRY,
                                                                        app_uart_tx_frame_buffer,
                                                                        frame_length)
                                                    ? HAL_OK : HAL_ERROR;
        if (status == HAL_OK) {
            app_uart_tx_busy = 1U;
            ++app_uart_tx_count;
        } else if (status != HAL_BUSY) {
            ++app_uart_rx_errors;
        }
    }

    app_uart_tx_pending_valid = 0U;
#if (APP_UART_TX_LED_ENABLED != 0U)
    BSP_LED_On(LED_1);
#endif
    app_uart_tx_led_until_ms = SVC_Timestamp_Ms() + APP_UART_TX_LED_PULSE_MS;

#else
    /* 原始 AiWB2 socket 发送协议 */

    if (app_uart_socket_tx_state == APP_UART_SOCKET_TX_WAIT_PROMPT) {
        if (APP_AiWB2_TakeSocketSendPrompt() != 0U) {
            if (BSP_UartLink_HasTxDma(BSP_UART_ROLE_TELEMETRY) == 0U) {
                status = BSP_UART_Transmit_USART1(app_uart_tx_frame_buffer,
                                                  app_uart_socket_payload_length,
                                                  100U);
                if (status == HAL_OK) {
                    app_uart_socket_tx_state = APP_UART_SOCKET_TX_WAIT_RESULT;
                    app_uart_socket_deadline_ms =
                        now_ms + APP_UART_SOCKET_SEND_RESULT_TIMEOUT_MS;
                    ++app_uart_tx_count;
                } else {
                    ++app_uart_rx_errors;
                    app_uart_socket_tx_reset();
                }
                return;
            }

            app_uart_clean_tx_dma_buffer(app_uart_socket_payload_length);
            status = BSP_UartLink_TransmitDma(BSP_UART_ROLE_TELEMETRY,
                                                                            app_uart_tx_frame_buffer,
                                                                            app_uart_socket_payload_length)
                                                        ? HAL_OK : HAL_ERROR;
            if (status == HAL_OK) {
                app_uart_tx_busy = 1U;
                app_uart_socket_tx_state = APP_UART_SOCKET_TX_WAIT_PAYLOAD_DMA;
                app_uart_socket_deadline_ms =
                    now_ms + APP_UART_SOCKET_SEND_RESULT_TIMEOUT_MS;
                ++app_uart_tx_count;
            } else if (status != HAL_BUSY) {
                ++app_uart_rx_errors;
                app_uart_socket_tx_reset();
            }
        } else if (app_uart_time_reached(now_ms, app_uart_socket_deadline_ms) != 0U) {
            ++app_uart_rx_errors;
            app_uart_socket_tx_reset();
        }
        return;
    }

    if (app_uart_socket_tx_state == APP_UART_SOCKET_TX_WAIT_PAYLOAD_DMA) {
        if (app_uart_tx_busy == 0U) {
            app_uart_socket_tx_state = APP_UART_SOCKET_TX_WAIT_RESULT;
            app_uart_socket_deadline_ms =
                now_ms + APP_UART_SOCKET_SEND_RESULT_TIMEOUT_MS;
        } else if (app_uart_time_reached(now_ms, app_uart_socket_deadline_ms) != 0U) {
            ++app_uart_rx_errors;
            app_uart_socket_tx_reset();
        }
        return;
    }

    if (app_uart_socket_tx_state == APP_UART_SOCKET_TX_WAIT_RESULT) {
        int8_t result = APP_AiWB2_TakeSocketSendResult();

        if (result > 0) {
            app_uart_socket_tx_reset();
        } else if (result < 0) {
            ++app_uart_rx_errors;
            app_uart_socket_tx_reset();
        } else if (app_uart_time_reached(now_ms, app_uart_socket_deadline_ms) != 0U) {
            ++app_uart_rx_errors;
            app_uart_socket_tx_reset();
        }
        return;
    }

    if (app_uart_tx_busy != 0U) {
        return;
    }

    if (app_uart_tx_pending_valid == 0U) {
        if (osMessageQueueGet(uartTxQueueHandle, &app_uart_tx_pending_message, 0U, 0U) != osOK) {
            return;
        }
        app_uart_tx_pending_valid = 1U;
    }

    if (APP_AiWB2_IsSocketReady() == 0U) {
        app_uart_tx_pending_valid = 0U;
        return;
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
    /*
     * Text control replies and VOFA binary frames share the Ai-WB2 socket.
     * The message function tags are metadata; routing here is by socket
     * readiness, so IDENT/AIRFRAME text must not be dropped.
     */
    memcpy(app_uart_tx_frame_buffer,
           app_uart_tx_pending_message.text,
           frame_length);

    {
        char command[40];
        int written = snprintf(command,
                               sizeof(command),
                               "AT+SOCKETSEND=%lu,%u\r\n",
                               (unsigned long)APP_AiWB2_GetSocketConId(),
                               (unsigned int)frame_length);

        if ((written <= 0) || ((uint32_t)written >= sizeof(command))) {
            app_uart_tx_pending_valid = 0U;
            ++app_uart_rx_errors;
            return;
        }

        (void)APP_AiWB2_TakeSocketSendPrompt();
        (void)APP_AiWB2_TakeSocketSendResult();
        status = BSP_UART_Transmit_USART1((const uint8_t *)command,
                                          (uint16_t)written,
                                          100U);
        if (status != HAL_OK) {
            app_uart_tx_pending_valid = 0U;
            ++app_uart_rx_errors;
            return;
        }
    }

    app_uart_socket_payload_length = frame_length;
    app_uart_socket_tx_state = APP_UART_SOCKET_TX_WAIT_PROMPT;
    app_uart_socket_deadline_ms = now_ms + APP_UART_SOCKET_SEND_PROMPT_TIMEOUT_MS;
    app_uart_tx_pending_valid = 0U;
#if (APP_UART_TX_LED_ENABLED != 0U)
    BSP_LED_On(LED_1);
#endif
    app_uart_tx_led_until_ms = SVC_Timestamp_Ms() + APP_UART_TX_LED_PULSE_MS;
#endif /* APP_UART_DIRECT_SERIAL_MODE */
#endif /* APP_UART_DISABLE_USART1 */
}

static void app_uart_sync_tx_state(void)
{
    if (app_uart_tx_busy == 0U) {
        return;
    }

    /*
     * 发送完成中断有可能被错过——调试时见过驱动已经回到空闲、而本地的 busy
     * 标志还挂着。只等回调的话发送路径会就此死锁，表现为"数传突然不说话了"，
     * 所以拿硬件的实际状态兜底，见 BSP_UartLink_TxIsComplete()。
     */
    if (BSP_UartLink_TxIsComplete(BSP_UART_ROLE_TELEMETRY) != 0U) {
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
    now_ms = SVC_Timestamp_Ms();
    app_uart_flush_idle_line(now_ms);
#if (APP_UART_DIRECT_SERIAL_MODE == 0U)
    APP_AiWB2_Tick();
#endif

    APP_OpticalFlow_ServiceRecovery();
    APP_LED_Task_Step();
    APP_USB_CDC_Task_Step();

    app_uart_ensure_control_ready();
    if ((app_uart_control_initialized != 0U)
#if (APP_UART_DIRECT_SERIAL_MODE == 0U)
#if (APP_UART_DIRECT_CONTROL_ENABLED == 0U)
        && ((APP_AiWB2_IsTransparent() != 0U) ||
            (APP_AiWB2_GetState() == APP_AIWB2_STATE_SOCKET_READY))
#endif
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
                           "BOOT uart_wait_control rx_bytes=%lu rx_lines=%lu rx_idle=%lu rx_overflows=%lu rx_errors=%lu rx_running=%u dma_started=%u trans=%u ctrl=%u\r\n",
                           (unsigned long)app_uart_rx_bytes,
                           (unsigned long)app_uart_rx_lines,
                           (unsigned long)app_uart_rx_idle_lines,
                           (unsigned long)app_uart_rx_overflows,
                           (unsigned long)app_uart_rx_errors,
                           (unsigned int)BSP_UartLink_RxIsRunning(
                               BSP_UART_ROLE_TELEMETRY),
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
    app_uart_update_tx_led(SVC_Timestamp_Ms());
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

void APP_UART_OnRxEvent(uint16_t size)
{
    app_uart_last_rx_event_size = (uint32_t)size;
    ++app_uart_rx_events;
#if (APP_UART_RX_USE_DMA == 0U)
    app_uart_it_rx_size = size;
    app_uart_it_rx_ready = 1U;
    app_uart_dma_started = 0U;
#endif
    app_uart_signal(APP_UART_EVENT_RX);
}

void APP_UART_OnTxComplete(void)
{
    app_uart_tx_busy = 0U;
    app_uart_signal(APP_UART_EVENT_TX);
}

void APP_UART_OnError(void)
{
    if (app_uart_tx_busy != 0U) {
        app_uart_tx_busy = 0U;
    }
    app_uart_dma_started = 0U;
    app_uart_it_rx_ready = 0U;
    app_uart_it_rx_size = 0U;
    app_uart_signal(APP_UART_EVENT_ERROR);
}


#include "app_control_internal.h"

#include "app_imu_capture.h"
#include "app_maint_uart.h"
#include "app_messages.h"
#include "app_tasks.h"
#include "app_uart.h"
#include "app_usb_cdc.h"

#include <stddef.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Preserve the moved function body's original private-state spelling. */
#define control_maint_output_active app_control_internal_maint_output_active()

void app_control_queue_proto_text(uint16_t function, const char *format, ...)
{
    APP_UART_TxMessage tx_message;
    APP_UART_TxMessage dropped;
    va_list args;
    int written;

    if ((format == NULL) || (uartTxQueueHandle == 0)) {
        return;
    }

    tx_message.function = function;
    va_start(args, format);
    written = vsnprintf(tx_message.text, sizeof(tx_message.text), format, args);
    va_end(args);

    if (written < 0) {
        return;
    }

    if ((uint32_t)written >= sizeof(tx_message.text)) {
        tx_message.length = (uint16_t)(sizeof(tx_message.text) - 1U);
        tx_message.text[tx_message.length] = '\0';
    } else {
        tx_message.length = (uint16_t)written;
    }

    /*
     * Mirror structured text to USB CDC so the V0 validation page can use the
     * virtual COM port instead of the slower USART1/WiFi path.  IMUCAP export
     * owns the CDC byte stream while active; injecting text there would corrupt
     * its binary framing, so USB mirroring is deliberately suspended.
     */
    if (APP_IMU_Capture_IsExportActive() == 0U) {
        (void)APP_USB_CDC_Write((const uint8_t *)tx_message.text,
                                tx_message.length,
                                APP_CONTROL_USB_TEXT_TX_TIMEOUT_MS);
    }

    if (control_maint_output_active == 0U) {
        if (osMessageQueuePut(uartTxQueueHandle, &tx_message, 0U, 0U) != osOK) {
            (void)osMessageQueueGet(uartTxQueueHandle, &dropped, 0U, 0U);
            (void)osMessageQueuePut(uartTxQueueHandle, &tx_message, 0U, 0U);
        }
        APP_UART_NotifyTxPending();
        /*
         * 蓝牙也要收到**没人问也该来的**那些行（READY、心跳、流式回报）。
         *
         * 结构化文本一直是无条件镜像到 USB 的，数传那条走队列，而蓝牙以前只在
         * "正在处理一条蓝牙命令"期间才有输出——于是蓝牙上看不到 READY，
         * 也看不到任何异步回报，和 USB 不是同一个口径。
         *
         * 只在蓝牙链路最近用过时才发：没连蓝牙的时候每条文本都要在 UART8 上
         * 阻塞发一遍，白白拖慢 UART 任务。
         */
        if (APP_MaintUART_IsLinkActive() != 0U) {
            APP_MaintUART_Write(tx_message.text, tx_message.length);
        }
    } else {
        APP_MaintUART_Write(tx_message.text, tx_message.length);
    }
}

uint32_t app_control_tokenize(char *buffer, char **tokens, uint32_t max_tokens)
{
    uint32_t count = 0U;
    char *token;

    if ((buffer == NULL) || (tokens == NULL) || (max_tokens == 0U)) {
        return 0U;
    }

    token = strtok(buffer, " \t\r\n");
    while ((token != NULL) && (count < max_tokens)) {
        tokens[count++] = token;
        token = strtok(NULL, " \t\r\n");
    }

    return count;
}

uint8_t app_control_parse_u32(const char *text, uint32_t *value)
{
    char *end_ptr;
    unsigned long parsed;

    if ((text == NULL) || (value == NULL) || (*text == '\0')) {
        return 0U;
    }

    parsed = strtoul(text, &end_ptr, 10);
    if ((end_ptr == text) || (*end_ptr != '\0')) {
        return 0U;
    }

    *value = (uint32_t)parsed;
    return 1U;
}

uint8_t app_control_parse_i32(const char *text, int32_t *value)
{
    char *end_ptr;
    long parsed;

    if ((text == NULL) || (value == NULL) || (*text == '\0')) {
        return 0U;
    }

    parsed = strtol(text, &end_ptr, 10);
    if ((end_ptr == text) || (*end_ptr != '\0')) {
        return 0U;
    }

    *value = (int32_t)parsed;
    return 1U;
}

uint32_t app_control_crc32_update(uint32_t crc, const uint8_t *data, uint32_t len)
{
    for (uint32_t i = 0U; i < len; ++i) {
        crc ^= data[i];
        for (uint32_t bit = 0U; bit < 8U; ++bit) {
            crc = (crc & 1U) ? ((crc >> 1U) ^ 0xEDB88320UL) : (crc >> 1U);
        }
    }
    return crc;
}

uint32_t app_control_crc32(const uint8_t *data, uint32_t len)
{
    return app_control_crc32_update(0xFFFFFFFFUL, data, len) ^ 0xFFFFFFFFUL;
}

const char *app_control_token_value(char **tokens,
                                           uint32_t count,
                                           const char *key)
{
    size_t key_len;

    if ((tokens == NULL) || (key == NULL)) {
        return NULL;
    }

    key_len = strlen(key);
    for (uint32_t index = 1U; index < count; ++index) {
        if ((strncmp(tokens[index], key, key_len) == 0) &&
            (tokens[index][key_len] == '=')) {
            return &tokens[index][key_len + 1U];
        }
    }

    return NULL;
}

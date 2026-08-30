/**
 * @file app_control_core.c
 * @brief Core infrastructure for app_control: string/number parsers,
 *        protocol text queuing, param sync queries, and calibration safety.
 */

#include "app_control_core.h"

#include "app_boot.h"
#include "app_control.h"
#include "app_flight_calibration.h"
#include "app_stabilizer.h"
#include "svc_param.h"
#include "svc_timestamp.h"

#include "cmsis_os2.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define APP_CONTROL_CORE_USB_TEXT_TX_TIMEOUT_MS 5U
#define APP_CONTROL_CORE_TX_TEXT_SIZE           256U

typedef struct {
    uint16_t function;
    uint16_t length;
    char     text[APP_CONTROL_CORE_TX_TEXT_SIZE];
} AppControlCoreTxMessage;

/* ── Extern handles & transport functions ───────────────────────────────── */
extern osMessageQueueId_t uartTxQueueHandle;
extern uint8_t control_maint_output_active;
extern APP_FlightCalibration control_imucal_confirmed;
extern uint32_t control_imucal_confirmed_generation;
extern uint8_t control_imucal_confirmed_valid;

uint8_t APP_IMU_Capture_IsExportActive(void);
uint32_t APP_USB_CDC_Write(const uint8_t *buffer, uint16_t length, uint32_t timeout_ms);
void APP_UART_NotifyTxPending(void);
void APP_MaintUART_Write(const char *text, uint16_t length);

/* ── Common String & Number Parsers ──────────────────────────────────────── */

uint32_t app_control_core_tokenize(char *buffer, char **tokens, uint32_t max_tokens)
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

const char *app_control_core_token_value(char **tokens, uint32_t count, const char *key)
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

uint8_t app_control_core_parse_u32(const char *text, uint32_t *value)
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

uint8_t app_control_core_parse_i32(const char *text, int32_t *value)
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

uint8_t app_control_core_parse_u32_auto(const char *text, uint32_t *value)
{
    char *end_ptr;
    unsigned long parsed;

    if ((text == NULL) || (value == NULL) || (*text == '\0')) {
        return 0U;
    }

    parsed = strtoul(text, &end_ptr, 0);
    if ((end_ptr == text) || (*end_ptr != '\0')) {
        return 0U;
    }

    *value = (uint32_t)parsed;
    return 1U;
}

uint8_t app_control_core_token_u32(char **tokens,
                                   uint32_t count,
                                   uint32_t index,
                                   uint32_t default_value,
                                   uint32_t *value)
{
    if (value == NULL) {
        return 0U;
    }

    if (index >= count) {
        *value = default_value;
        return 1U;
    }

    return app_control_core_parse_u32_auto(tokens[index], value);
}

const char *app_control_core_after_param_separator(const char *text)
{
    if (text == NULL) {
        return NULL;
    }

    if (text[0] == ':') {
        return text + 1U;
    }

    return NULL;
}

uint32_t app_control_core_crc32_update(uint32_t crc, const uint8_t *data, uint32_t len)
{
    for (uint32_t i = 0U; i < len; ++i) {
        crc ^= data[i];
        for (uint32_t bit = 0U; bit < 8U; ++bit) {
            crc = (crc & 1U) ? ((crc >> 1U) ^ 0xEDB88320UL) : (crc >> 1U);
        }
    }
    return crc;
}

uint32_t app_control_core_crc32(const uint8_t *data, uint32_t len)
{
    return app_control_core_crc32_update(0xFFFFFFFFUL, data, len) ^ 0xFFFFFFFFUL;
}

/* ── Protocol & Text Formatting Output ──────────────────────────────────── */

void app_control_core_queue_proto_text(uint16_t function, const char *format, ...)
{
    AppControlCoreTxMessage tx_message;
    AppControlCoreTxMessage dropped;
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

    if (APP_IMU_Capture_IsExportActive() == 0U) {
        (void)APP_USB_CDC_Write((const uint8_t *)tx_message.text,
                                tx_message.length,
                                APP_CONTROL_CORE_USB_TEXT_TX_TIMEOUT_MS);
    }

    if (control_maint_output_active == 0U) {
        if (osMessageQueuePut(uartTxQueueHandle, &tx_message, 0U, 0U) != osOK) {
            (void)osMessageQueueGet(uartTxQueueHandle, &dropped, 0U, 0U);
            (void)osMessageQueuePut(uartTxQueueHandle, &tx_message, 0U, 0U);
        }
        APP_UART_NotifyTxPending();
    } else {
        APP_MaintUART_Write(tx_message.text, tx_message.length);
    }
}

/* ── Calibration & Param Core Observer Queries ──────────────────────────── */

void app_control_core_get_persisted_snapshot(AppControlCorePersistedSnapshot *snapshot)
{
    if (snapshot == NULL) {
        return;
    }
    snapshot->record = control_imucal_confirmed;
    snapshot->generation = control_imucal_confirmed_generation;
    snapshot->valid = control_imucal_confirmed_valid;
    snapshot->dirty = SVC_Param_IsDirty();
}

const APP_FlightCalibration *app_control_core_get_persisted_record(void)
{
    return (control_imucal_confirmed_valid != 0U) ? &control_imucal_confirmed : NULL;
}

uint32_t app_control_core_get_persisted_generation(void)
{
    return control_imucal_confirmed_generation;
}

uint8_t app_control_core_is_persisted_valid(void)
{
    return control_imucal_confirmed_valid;
}

uint8_t app_control_core_is_param_dirty(void)
{
    return SVC_Param_IsDirty();
}

/* ── Calibration Common Safety Check ────────────────────────────────────── */

const char *app_control_core_check_calibration_safety(
    uint8_t require_sequence_progress,
    uint32_t apply_sequence)
{
    StabilizerValidationImuSnapshot snapshot;
    uint64_t now_us;

    memset(&snapshot, 0, sizeof(snapshot));
    if (APP_Stabilizer_ReadValidationImuSnapshot(&snapshot) == 0U) {
        return "snapshot_invalid";
    }
    now_us = SVC_Timestamp_Us();
    if ((snapshot.timestamp_us == 0ULL) ||
        (now_us < snapshot.timestamp_us) ||
        ((now_us - snapshot.timestamp_us) > APP_CONTROL_CORE_SNAPSHOT_MAX_AGE_US)) {
        return "snapshot_stale";
    }
    if (snapshot.armed != 0U) {
        return "armed";
    }
    if ((snapshot.esc_pulse_us[0] > APP_CONTROL_CORE_ESC_SAFE_MAX_US) ||
        (snapshot.esc_pulse_us[1] > APP_CONTROL_CORE_ESC_SAFE_MAX_US)) {
        return "esc_high";
    }
    if ((require_sequence_progress != 0U) &&
        (APP_Boot_HasSequenceAdvanced(snapshot.sequence, apply_sequence) == 0U)) {
        return "sequence_stalled";
    }
    return NULL;
}

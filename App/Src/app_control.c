#include "app_control.h"

#include "app_aiwb2.h"
#include "app_baro.h"
#include "app_flash.h"
#include "app_imu.h"
#include "app_maint_uart.h"
#include "app_messages.h"
#include "app_proto.h"
#include "app_tasks.h"
#include "app_uart.h"
#include "bsp_bus_servo.h"
#include "bsp_aiwb2_power.h"
#include "bsp_baro.h"
#include "bsp_flash.h"
#include "bsp_icm42688.h"
#include "bsp_uart.h"

#include <stddef.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define APP_CONTROL_CFG_MAGIC       0x44524346UL
#define APP_CONTROL_CFG_VERSION     2U
#define APP_CONTROL_CFG_ADDRESS     (BSP_GD25Q32_FLASH_SIZE_BYTES - 4096UL)
#define APP_CONTROL_CFG_LEGACY_SIZE ((uint16_t)offsetof(APP_ControlConfig, rate_pid))
#define APP_CONTROL_MAX_LINE        128U
#define APP_CONTROL_HEARTBEAT_ENABLED 1U
#define APP_CONTROL_BOOT_READY_ENABLED 1U
#define APP_CONTROL_BARO_STREAM_DEFAULT_PERIOD_MS 50U
#define APP_CONTROL_BARO_STREAM_MIN_PERIOD_MS 20U

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    APP_ControlConfig config;
    uint32_t checksum;
} APP_ControlFlashRecord;

static APP_ControlConfig control_config;
#if (APP_CONTROL_HEARTBEAT_ENABLED != 0U)
static uint32_t control_last_heartbeat_ms;
static uint8_t control_reported_hw_once;
#endif
static uint8_t control_baro_stream_enabled;
static uint32_t control_baro_stream_period_ms;
static uint32_t control_last_baro_stream_ms;
static uint8_t control_wifi_reset_pending;
static uint32_t control_wifi_reset_deadline_ms;
static uint16_t control_response_override_function;
static uint8_t control_initialized;
static uint8_t control_maint_output_active;

static void app_control_handle_param(char **tokens, uint32_t count);
static void app_control_report_pid_legacy(void);
static void app_control_report_wifi(void);
static void app_control_queue_proto_text(uint16_t function, const char *format, ...);
static void app_control_process_proto_request(uint16_t function,
                                              const uint8_t *payload,
                                              uint16_t payload_length);
static void app_control_dispatch_tokens(char **tokens, uint32_t count, uint8_t emit_ack);
static uint16_t app_control_response_function_for_request(uint16_t function);
static uint32_t app_control_tokenize(char *buffer, char **tokens, uint32_t max_tokens);
static uint8_t app_control_payload_to_line(const uint8_t *payload,
                                           uint16_t payload_length,
                                           char *buffer,
                                           uint16_t buffer_size);
static void app_control_handle_wifi(char **tokens, uint32_t count);
static void app_control_service_wifi_reset(void);
static void app_control_tick_common(uint8_t emit_heartbeat);

static uint16_t app_control_detect_function(const char *format)
{
    if (format == NULL) {
        return APP_PROTO_MSG_TEXT_LINE;
    }

    if (strncmp(format, "RX ", 3U) == 0) {
        return APP_PROTO_MSG_CMD_RX;
    }
    if (strncmp(format, "ACK ", 4U) == 0) {
        return APP_PROTO_MSG_CMD_ACK;
    }
    if (strncmp(format, "ERR ", 4U) == 0) {
        return APP_PROTO_MSG_CMD_ERR;
    }
    if (strncmp(format, "OK ", 3U) == 0) {
        return APP_PROTO_MSG_CMD_OK;
    }

    return APP_PROTO_MSG_TEXT_LINE;
}

static const char *app_control_imu_stage_name(uint8_t stage)
{
    switch ((BSP_ICM42688_InitStage)stage) {
    case BSP_ICM42688_INIT_STAGE_NONE:
        return "none";
    case BSP_ICM42688_INIT_STAGE_BANK_SELECT:
        return "bank";
    case BSP_ICM42688_INIT_STAGE_RESET:
        return "reset";
    case BSP_ICM42688_INIT_STAGE_WHO_AM_I:
        return "who";
    case BSP_ICM42688_INIT_STAGE_GYRO_CONFIG:
        return "gyro_cfg";
    case BSP_ICM42688_INIT_STAGE_ACCEL_CONFIG:
        return "accel_cfg";
    case BSP_ICM42688_INIT_STAGE_FILTER_CONFIG:
        return "filter_cfg";
    case BSP_ICM42688_INIT_STAGE_PWR_MGMT:
        return "pwr";
    case BSP_ICM42688_INIT_STAGE_SIGNAL_RESET:
        return "sig_reset";
    case BSP_ICM42688_INIT_STAGE_READY:
        return "ready";
    default:
        return "unknown";
    }
}

static uint8_t app_control_flash_ok(const APP_Flash_Status *status)
{
    return ((status != NULL) &&
            (status->probe_status == 0) &&
            (status->status1_status == 0) &&
            (status->read_status == 0)) ? 1U : 0U;
}

static const char *app_control_flash_stage(const APP_Flash_Status *status)
{
    if (status == NULL) {
        return "unknown";
    }

    if (status->probe_status != 0) {
        return "probe";
    }

    if (status->status1_status != 0) {
        return "status";
    }

    if (status->read_status != 0) {
        return "read";
    }

    return "ready";
}

static uint8_t app_control_baro_ok(const APP_Baro_Status *status)
{
    return ((status != NULL) && (status->init_status == 0)) ? 1U : 0U;
}

static const char *app_control_baro_stage(const APP_Baro_Status *status)
{
    if (status == NULL) {
        return "unknown";
    }

    if (status->split_status != 0) {
        return "split_id";
    }

    if (status->txrx_status != 0) {
        return "txrx_id";
    }

    if (status->init_status != 0) {
        return "init";
    }

    return "ready";
}

static uint32_t app_control_checksum(const uint8_t *data, uint32_t length)
{
    uint32_t sum = 0xA5A55A5AUL;

    for (uint32_t index = 0U; index < length; ++index) {
        sum = (sum << 5U) | (sum >> 27U);
        sum ^= data[index];
        sum += 0x9E3779B9UL;
    }

    return sum;
}

static void app_control_queue_text(const char *format, ...)
{
    APP_UART_TxMessage tx_message;
    APP_UART_TxMessage dropped;
    va_list args;
    int written;

    if ((format == NULL) || (uartTxQueueHandle == 0)) {
        return;
    }

    tx_message.function = (control_response_override_function != 0U) ?
                          control_response_override_function :
                          app_control_detect_function(format);
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

static void app_control_queue_proto_text(uint16_t function, const char *format, ...)
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

static uint32_t app_control_tokenize(char *buffer, char **tokens, uint32_t max_tokens)
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

static uint8_t app_control_payload_to_line(const uint8_t *payload,
                                           uint16_t payload_length,
                                           char *buffer,
                                           uint16_t buffer_size)
{
    uint16_t used = 0U;

    if ((buffer == NULL) || (buffer_size == 0U)) {
        return 0U;
    }

    if ((payload == NULL) && (payload_length != 0U)) {
        return 0U;
    }

    for (uint16_t index = 0U; index < payload_length; ++index) {
        uint8_t byte = payload[index];

        if ((byte == (uint8_t)'\r') || (byte == (uint8_t)'\n') || (byte == 0U)) {
            continue;
        }

        if ((byte < (uint8_t)' ') || (byte > (uint8_t)'~')) {
            return 0U;
        }

        if (used >= (uint16_t)(buffer_size - 1U)) {
            return 0U;
        }

        buffer[used++] = (char)byte;
    }

    buffer[used] = '\0';
    return 1U;
}

static uint16_t app_control_copy_payload_text(char *buffer,
                                              uint16_t capacity,
                                              const uint8_t *payload,
                                              uint16_t payload_length)
{
    uint16_t used = 0U;

    if ((buffer == NULL) || (capacity == 0U)) {
        return 0U;
    }

    if ((payload == NULL) || (payload_length == 0U)) {
        buffer[0] = '\0';
        return 0U;
    }

    while ((used < payload_length) && (used < (uint16_t)(capacity - 1U))) {
        uint8_t byte = payload[used];

        if ((byte == 0U) || (byte == (uint8_t)'\r') || (byte == (uint8_t)'\n')) {
            break;
        }

        buffer[used] = (char)byte;
        ++used;
    }

    buffer[used] = '\0';
    return used;
}

static uint16_t app_control_response_function_for_request(uint16_t function)
{
    switch (function) {
    case APP_PROTO_REQ_PING:
        return APP_PROTO_MSG_PONG;
    case APP_PROTO_REQ_STATUS:
        return APP_PROTO_MSG_STATUS_FLASH;
    case APP_PROTO_REQ_CONFIG:
        return APP_PROTO_MSG_CONFIG_SUMMARY;
    case APP_PROTO_REQ_PARAMS:
        return APP_PROTO_MSG_PARAM_RECORD;
    case APP_PROTO_REQ_PID:
        return APP_PROTO_MSG_PID_RECORD;
    case APP_PROTO_REQ_BARO:
    case APP_PROTO_REQ_BARO_STREAM:
        return APP_PROTO_MSG_BARO_STATE;
    case APP_PROTO_REQ_FLASH:
        return APP_PROTO_MSG_FLASH_RECORD;
    case APP_PROTO_REQ_IMU:
        return APP_PROTO_MSG_IMU_STATE;
    case APP_PROTO_REQ_MODULES:
        return APP_PROTO_MSG_MODULES_SUMMARY;
    case APP_PROTO_REQ_CAPS:
        return APP_PROTO_MSG_CAPS_RECORD;
    case APP_PROTO_REQ_SAVE:
        return APP_PROTO_MSG_SAVE_RESULT;
    case APP_PROTO_REQ_LOAD:
        return APP_PROTO_MSG_LOAD_RESULT;
    case APP_PROTO_REQ_DEFAULTS:
        return APP_PROTO_MSG_DEFAULTS_RESULT;
    case APP_PROTO_REQ_PARAM_SET:
        return APP_PROTO_MSG_PARAM_RECORD;
    case APP_PROTO_REQ_PID_SET:
        return APP_PROTO_MSG_PID_RECORD;
    case APP_PROTO_REQ_SERVO_MOVE:
    case APP_PROTO_REQ_SERVO_MOVE_ALL:
    case APP_PROTO_REQ_SERVO_ID:
    case APP_PROTO_REQ_SERVO_SETID:
    case APP_PROTO_REQ_SERVO_MODE:
    case APP_PROTO_REQ_SERVO_ENABLE:
    case APP_PROTO_REQ_SERVO_ACTION:
    case APP_PROTO_REQ_SERVO_RAW:
        return APP_PROTO_MSG_SERVO_RESULT;
    case APP_PROTO_REQ_WIFI:
        return APP_PROTO_MSG_WIFI_RECORD;
    default:
        return 0U;
    }
}

static const char *app_control_aiwb2_state_name(APP_AiWB2_State state)
{
    switch (state) {
    case APP_AIWB2_STATE_START_DELAY:
        return "start_delay";
    case APP_AIWB2_STATE_WAIT_PROBE:
        return "wait_probe";
    case APP_AIWB2_STATE_ESCAPE_BEFORE:
        return "escape_before";
    case APP_AIWB2_STATE_ESCAPE_AFTER:
        return "escape_after";
    case APP_AIWB2_STATE_SEND_COMMAND:
        return "send_command";
    case APP_AIWB2_STATE_WAIT_COMMAND:
        return "wait_command";
    case APP_AIWB2_STATE_WAIT_BOOT_CONNECT:
        return "wait_connect";
    case APP_AIWB2_STATE_TRANSPARENT:
        return "transparent";
    case APP_AIWB2_STATE_RETRY_DELAY:
        return "retry_delay";
    default:
        return "unknown";
    }
}

static void app_control_defaults(APP_ControlConfig *config)
{
    if (config == NULL) {
        return;
    }

    memset(config, 0, sizeof(*config));
    config->servo[0].id = 1U;
    config->servo[0].pulse_us = 1500U;
    config->servo[0].time_ms = 500U;
    config->servo[0].mode = 1U;
    config->servo[0].enabled = 1U;
    config->servo[1].id = 2U;
    config->servo[1].pulse_us = 1500U;
    config->servo[1].time_ms = 500U;
    config->servo[1].mode = 1U;
    config->servo[1].enabled = 1U;

    for (uint32_t axis = 0U; axis < APP_CONTROL_PID_AXIS_COUNT; ++axis) {
        config->rate_pid[axis].kp = 120;
        config->rate_pid[axis].ki = 0;
        config->rate_pid[axis].kd = 8;
        config->rate_pid[axis].integral_limit = 300;
        config->rate_pid[axis].output_limit = 500;

        config->angle_pid[axis].kp = 80;
        config->angle_pid[axis].ki = 0;
        config->angle_pid[axis].kd = 0;
        config->angle_pid[axis].integral_limit = 200;
        config->angle_pid[axis].output_limit = 450;
    }

    config->altitude_pid.kp = 100;
    config->altitude_pid.ki = 0;
    config->altitude_pid.kd = 20;
    config->altitude_pid.integral_limit = 200;
    config->altitude_pid.output_limit = 400;
}

static uint8_t app_control_valid_servo_index(uint32_t index)
{
    return (index < APP_CONTROL_SERVO_COUNT) ? 1U : 0U;
}

static uint8_t app_control_parse_u32(const char *text, uint32_t *value)
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

static uint8_t app_control_parse_u32_auto(const char *text, uint32_t *value)
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

static const char *app_control_token_value(char **tokens,
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

static void app_control_protocol_err(uint32_t id,
                                     const char *mod,
                                     const char *op,
                                     const char *code)
{
    app_control_queue_text("ERR id=%lu mod=%s op=%s code=%s\r\n",
                           (unsigned long)id,
                           (mod != NULL) ? mod : "?",
                           (op != NULL) ? op : "?",
                           (code != NULL) ? code : "ERR");
}

static uint8_t app_control_parse_i32(const char *text, int32_t *value)
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

static int16_t app_control_clamp_i16(int32_t value)
{
    if (value > 32767L) {
        return 32767;
    }

    if (value < -32768L) {
        return -32768;
    }

    return (int16_t)value;
}

static void app_control_report_pid_line(const char *name,
                                        uint32_t index,
                                        const APP_ControlPidConfig *pid)
{
    if ((name == NULL) || (pid == NULL)) {
        return;
    }

    app_control_queue_proto_text(APP_PROTO_MSG_PARAM_RECORD,
                                 "PARAM %s%lu kp=%d ki=%d kd=%d ilim=%d out=%d\r\n",
                                 name,
                                 (unsigned long)index,
                                 (int)pid->kp,
                                 (int)pid->ki,
                                 (int)pid->kd,
                                 (int)pid->integral_limit,
                                 (int)pid->output_limit);
}

static void app_control_report_params(void)
{
    for (uint32_t axis = 0U; axis < APP_CONTROL_PID_AXIS_COUNT; ++axis) {
        app_control_report_pid_line("rate", axis, &control_config.rate_pid[axis]);
    }

    for (uint32_t axis = 0U; axis < APP_CONTROL_PID_AXIS_COUNT; ++axis) {
        app_control_report_pid_line("angle", axis, &control_config.angle_pid[axis]);
    }

    app_control_report_pid_line("alt", 0U, &control_config.altitude_pid);
}

static void app_control_report_caps(void)
{
    app_control_queue_proto_text(APP_PROTO_MSG_CAPS_RECORD,
                                 "RSP id=0 mod=CAPS op=LIST proto=mspv2-lite-v1 resp=frame+typed req=frame+typed\r\n");
    app_control_queue_proto_text(APP_PROTO_MSG_CAPS_RECORD,
                                 "RSP id=0 mod=CAPS op=LIST legacy=PING,STATUS?,CONFIG?,SAVE,LOAD,SERVO raw=custom-tab\r\n");
    app_control_queue_proto_text(APP_PROTO_MSG_CAPS_RECORD,
                                 "RSP id=0 mod=CAPS op=LIST mods=MODULES,SPL06,ICM42688,PARAM,FLASH,WIFI\r\n");
    app_control_queue_proto_text(APP_PROTO_MSG_CAPS_RECORD,
                                 "RSP id=0 mod=CAPS op=LIST ops=SPL06:STATUS,READ,SAMPLE ICM42688:STATUS,DIAG\r\n");
    app_control_queue_proto_text(APP_PROTO_MSG_CAPS_RECORD,
                                 "RSP id=0 mod=CAPS op=LIST ops=WIFI:STATUS,EN,RESET legacy=WIFI?,WIFI_EN?\r\n");
}

static void app_control_report_wifi(void)
{
    app_control_queue_proto_text(APP_PROTO_MSG_WIFI_RECORD,
                                 "WIFI en=%u pin=PC6 last=%u writes=%lu state=%s transparent=%u retry=%lu socket=%ld cycling=%u wait_ms=%lu prov=%u cmd=%lu/%lu\r\n",
                                 (unsigned int)BSP_AiWB2_IsEnabled(),
                                 (unsigned int)BSP_AiWB2_GetLastWrittenState(),
                                 (unsigned long)BSP_AiWB2_GetWriteCount(),
                                 app_control_aiwb2_state_name(APP_AiWB2_GetState()),
                                 (unsigned int)APP_AiWB2_IsTransparent(),
                                 (unsigned long)APP_AiWB2_GetRetryCount(),
                                 (long)APP_AiWB2_GetLastSocketError(),
                                 (unsigned int)APP_AiWB2_IsPowerRecycleActive(),
                                 (unsigned long)APP_AiWB2_GetDeadlineRemainingMs(),
                                 (unsigned int)APP_AiWB2_IsProvisionActive(),
                                 (unsigned long)APP_AiWB2_GetCommandIndex(),
                                 (unsigned long)APP_AiWB2_GetCommandCount());
}

static void app_control_report_config(void)
{
    app_control_queue_proto_text(APP_PROTO_MSG_CONFIG_SUMMARY,
                                 "CFG loaded=%u valid=%u flash_st=%u\r\n",
                                 (unsigned int)control_config.loaded_from_flash,
                                 (unsigned int)control_config.flash_valid,
                                 (unsigned int)control_config.last_flash_status);
    for (uint32_t index = 0U; index < APP_CONTROL_SERVO_COUNT; ++index) {
        const APP_ControlServoConfig *servo = &control_config.servo[index];

        app_control_queue_proto_text(APP_PROTO_MSG_CONFIG_SERVO,
                                     "CFG servo%lu id=%u pulse=%u time=%u mode=%u en=%u\r\n",
                                     (unsigned long)index,
                                     (unsigned int)servo->id,
                                     (unsigned int)servo->pulse_us,
                                     (unsigned int)servo->time_ms,
                                     (unsigned int)servo->mode,
                                     (unsigned int)servo->enabled);
    }
    app_control_report_params();
    app_control_report_wifi();
}

static void app_control_report_flash(void)
{
    APP_Flash_Status flash_status;

    APP_Flash_GetStatus(&flash_status);
    app_control_queue_proto_text(APP_PROTO_MSG_FLASH_RECORD,
                                 "FLASH ok=%u stage=%s probe=%ld status=%ld read=%ld id=%02X%02X%02X exp=C84016 sr1=%02X\r\n",
                                 (unsigned int)app_control_flash_ok(&flash_status),
                                 app_control_flash_stage(&flash_status),
                                 (long)flash_status.probe_status,
                                 (long)flash_status.status1_status,
                                 (long)flash_status.read_status,
                                 (unsigned int)flash_status.manufacturer_id,
                                 (unsigned int)flash_status.memory_type,
                                 (unsigned int)flash_status.capacity_id,
                                 (unsigned int)flash_status.status1);
    app_control_queue_proto_text(APP_PROTO_MSG_FLASH_RECORD,
                                 "FLASH cfg_addr=0x%06lX cfg_valid=%u cfg_last=%u\r\n",
                                 (unsigned long)APP_CONTROL_CFG_ADDRESS,
                                 (unsigned int)control_config.flash_valid,
                                 (unsigned int)control_config.last_flash_status);
}

static void app_control_report_baro(void)
{
    APP_Baro_Snapshot snapshot;

    APP_Baro_ReadSnapshot(&snapshot);
    app_control_queue_proto_text(APP_PROTO_MSG_BARO_STATE,
                                 "BARO ok=%u stage=%s init=%ld split=%ld txrx=%ld raw_st=%ld id=0x%02X split_id=0x%02X txrx_id=0x%02X bmp=0x%02X\r\n",
                                 (unsigned int)app_control_baro_ok(&snapshot.status),
                                 app_control_baro_stage(&snapshot.status),
                                 (long)snapshot.status.init_status,
                                 (long)snapshot.status.split_status,
                                 (long)snapshot.status.txrx_status,
                                 (long)snapshot.raw_status,
                                 (unsigned int)snapshot.id,
                                 (unsigned int)snapshot.status.split_id,
                                 (unsigned int)snapshot.status.txrx_id,
                                 (unsigned int)snapshot.status.bmp280_id);
    app_control_queue_proto_text(APP_PROTO_MSG_BARO_DIAG,
                                 "BARO diag exp=0x10 cs=%u miso=%u scaled=%u coef_st=%ld coef_srce=0x%02X tmp_ext=%u c0=%d c1=%d c00=%ld c10=%ld\r\n",
                                 (unsigned int)snapshot.status.cs_level,
                                 (unsigned int)snapshot.status.miso_level,
                                 (unsigned int)snapshot.scaled_valid,
                                 (long)snapshot.coef_status,
                                 (unsigned int)snapshot.coef_srce,
                                 (unsigned int)((snapshot.tmp_cfg & 0x80U) != 0U),
                                 (int)snapshot.c0,
                                 (int)snapshot.c1,
                                 (long)snapshot.c00,
                                 (long)snapshot.c10);
    app_control_queue_proto_text(APP_PROTO_MSG_BARO_RAW,
                                 "BARO raw pressure=%ld temp=%ld pressure_pa=%ld temp_cdeg=%ld prs_cfg=0x%02X tmp_cfg=0x%02X meas_cfg=0x%02X cfg=0x%02X int=0x%02X fifo=0x%02X\r\n",
                                 (long)snapshot.pressure_raw,
                                 (long)snapshot.temperature_raw,
                                 (long)snapshot.pressure_pa,
                                 (long)snapshot.temperature_cdeg,
                                 (unsigned int)snapshot.prs_cfg,
                                 (unsigned int)snapshot.tmp_cfg,
                                 (unsigned int)snapshot.meas_cfg,
                                 (unsigned int)snapshot.cfg_reg,
                                 (unsigned int)snapshot.int_sts,
                                 (unsigned int)snapshot.fifo_sts);
    app_control_queue_proto_text(APP_PROTO_MSG_BARO_DIAG,
                                 "BARO regs0=%02X%02X%02X%02X%02X%02X%02X regs1=%02X%02X%02X%02X%02X%02X%02X\r\n",
                                 (unsigned int)snapshot.raw_regs[0],
                                 (unsigned int)snapshot.raw_regs[1],
                                 (unsigned int)snapshot.raw_regs[2],
                                 (unsigned int)snapshot.raw_regs[3],
                                 (unsigned int)snapshot.raw_regs[4],
                                 (unsigned int)snapshot.raw_regs[5],
                                 (unsigned int)snapshot.raw_regs[6],
                                 (unsigned int)snapshot.raw_regs[7],
                                 (unsigned int)snapshot.raw_regs[8],
                                 (unsigned int)snapshot.raw_regs[9],
                                 (unsigned int)snapshot.raw_regs[10],
                                 (unsigned int)snapshot.raw_regs[11],
                                 (unsigned int)snapshot.raw_regs[12],
                                 (unsigned int)snapshot.raw_regs[13]);
}

static void app_control_emit_baro_event(void)
{
    APP_Baro_Snapshot snapshot;

    APP_Baro_ReadSnapshot(&snapshot);
    app_control_queue_proto_text(APP_PROTO_MSG_BARO_STREAM,
                                 "BARO ok=%u stage=%s raw_st=%ld coef_st=%ld scaled=%u pressure_pa=%ld pressure=%ld temp_cdeg=%ld raw_pressure=%ld raw_temperature=%ld prs_cfg=0x%02X tmp_cfg=0x%02X meas_cfg=0x%02X int=0x%02X count=%lu\r\n",
                                 (unsigned int)app_control_baro_ok(&snapshot.status),
                                 app_control_baro_stage(&snapshot.status),
                                 (long)snapshot.raw_status,
                                 (long)snapshot.coef_status,
                                 (unsigned int)snapshot.scaled_valid,
                                 (long)snapshot.pressure_pa,
                                 (long)snapshot.pressure_pa,
                                 (long)snapshot.temperature_cdeg,
                                 (long)snapshot.pressure_raw,
                                 (long)snapshot.temperature_raw,
                                 (unsigned int)snapshot.prs_cfg,
                                 (unsigned int)snapshot.tmp_cfg,
                                 (unsigned int)snapshot.meas_cfg,
                                 (unsigned int)snapshot.int_sts,
                                 (unsigned long)(snapshot.status.report_done != 0U));
}

static void app_control_report_imu(void)
{
    APP_IMU_Status imu_status;

    APP_IMU_GetStatus(&imu_status);
    app_control_queue_proto_text(APP_PROTO_MSG_IMU_STATE,
                                 "IMU ok=%u stage=%s stage_id=%u st=%ld err=%ld who=0x%02X exp=0x%02X n=%lu\r\n",
                                 (unsigned int)imu_status.initialized,
                                 app_control_imu_stage_name(imu_status.init_stage),
                                 (unsigned int)imu_status.init_stage,
                                 (long)imu_status.last_status,
                                 (long)imu_status.last_error,
                                 (unsigned int)imu_status.who_am_i,
                                 (unsigned int)BSP_ICM42688_WHO_AM_I_VALUE,
                                 (unsigned long)imu_status.sample_count);
    app_control_queue_proto_text(APP_PROTO_MSG_IMU_SCALED,
                                 "IMU scaled ax_mg=%d ay_mg=%d az_mg=%d gx_mdps=%ld gy_mdps=%ld gz_mdps=%ld temp_cdeg=%d roll=%d pitch=%d yaw=%d\r\n",
                                 (int)imu_status.accel_x_mg,
                                 (int)imu_status.accel_y_mg,
                                 (int)imu_status.accel_z_mg,
                                 (long)imu_status.gyro_x_mdps,
                                 (long)imu_status.gyro_y_mdps,
                                 (long)imu_status.gyro_z_mdps,
                                 (int)imu_status.temperature_cdeg,
                                 (int)imu_status.roll_cdeg,
                                 (int)imu_status.pitch_cdeg,
                                 (int)imu_status.yaw_cdeg);
    app_control_queue_proto_text(APP_PROTO_MSG_IMU_STATE,
                                 "IMU diag valid=%u m0_tok=0x%02X m0_msb=0x%02X m0_b0=0x%02X m3_tok=0x%02X m3_msb=0x%02X m3_b0=0x%02X best_mode=%u best_hdr=%u\r\n",
                                 (unsigned int)imu_status.diag_valid,
                                 (unsigned int)imu_status.diag_mode0_tokmas,
                                 (unsigned int)imu_status.diag_mode0_msb,
                                 (unsigned int)imu_status.diag_mode0_bit0,
                                 (unsigned int)imu_status.diag_mode3_tokmas,
                                 (unsigned int)imu_status.diag_mode3_msb,
                                 (unsigned int)imu_status.diag_mode3_bit0,
                                 (unsigned int)imu_status.diag_best_mode,
                                 (unsigned int)imu_status.diag_best_header);
    app_control_queue_proto_text(APP_PROTO_MSG_IMU_STATE,
                                 "IMU burst m0_b0=%02X%02X%02X%02X m3_tok=%02X%02X%02X%02X\r\n",
                                 (unsigned int)imu_status.diag_burst_m0_b0_1,
                                 (unsigned int)imu_status.diag_burst_m0_b0_2,
                                 (unsigned int)imu_status.diag_burst_m0_b0_3,
                                 (unsigned int)imu_status.diag_burst_m0_b0_4,
                                 (unsigned int)imu_status.diag_burst_m3_tok_1,
                                 (unsigned int)imu_status.diag_burst_m3_tok_2,
                                 (unsigned int)imu_status.diag_burst_m3_tok_3,
                                 (unsigned int)imu_status.diag_burst_m3_tok_4);
}

static void app_control_report_modules(void)
{
    APP_Flash_Status flash_status;
    APP_Baro_Status baro_status;
    APP_IMU_Status imu_status;

    APP_Flash_GetStatus(&flash_status);
    APP_Baro_GetStatus(&baro_status);
    APP_IMU_GetStatus(&imu_status);

    app_control_queue_proto_text(APP_PROTO_MSG_MODULES_SUMMARY,
                                 "RSP id=0 mod=MODULES op=STATUS flash=%u flash_stage=%s baro=%u baro_stage=%s imu=%u\r\n",
                                 (unsigned int)app_control_flash_ok(&flash_status),
                                 app_control_flash_stage(&flash_status),
                                 (unsigned int)app_control_baro_ok(&baro_status),
                                 app_control_baro_stage(&baro_status),
                                 (unsigned int)imu_status.initialized);
    app_control_queue_proto_text(APP_PROTO_MSG_MODULES_SUMMARY,
                                 "RSP id=0 mod=MODULES op=STATUS imu_stage=%s cfg_valid=%u cfg_loaded=%u servo_slots=%u wifi_en=%u\r\n",
                                 app_control_imu_stage_name(imu_status.init_stage),
                                 (unsigned int)control_config.flash_valid,
                                 (unsigned int)control_config.loaded_from_flash,
                                 (unsigned int)APP_CONTROL_SERVO_COUNT,
                                 (unsigned int)BSP_AiWB2_IsEnabled());
}

static void app_control_req_spl06(uint32_t id, const char *op)
{
    APP_Baro_Snapshot snapshot;

    if (op == NULL) {
        app_control_protocol_err(id, "SPL06", "?", "NO_OP");
        return;
    }

    APP_Baro_ReadSnapshot(&snapshot);

    if (strcmp(op, "STATUS") == 0) {
        app_control_queue_text("RSP id=%lu mod=SPL06 op=STATUS ok=%u stage=%s init=%ld raw=%ld who=0x%02X exp=0x10\r\n",
                               (unsigned long)id,
                               (unsigned int)app_control_baro_ok(&snapshot.status),
                               app_control_baro_stage(&snapshot.status),
                               (long)snapshot.status.init_status,
                               (long)snapshot.raw_status,
                               (unsigned int)snapshot.id);
        app_control_queue_text("RSP id=%lu mod=SPL06 op=STATUS split=%ld txrx=%ld sid=0x%02X tid=0x%02X cs=%u miso=%u\r\n",
                               (unsigned long)id,
                               (long)snapshot.status.split_status,
                               (long)snapshot.status.txrx_status,
                               (unsigned int)snapshot.status.split_id,
                               (unsigned int)snapshot.status.txrx_id,
                               (unsigned int)snapshot.status.cs_level,
                               (unsigned int)snapshot.status.miso_level);
        return;
    }

    if ((strcmp(op, "SAMPLE") == 0) || (strcmp(op, "READ") == 0)) {
        app_control_queue_text("RSP id=%lu mod=SPL06 op=%s ok=%u raw_st=%ld coef_st=%ld scaled=%u press_raw=%ld temp_raw=%ld pressure_pa=%ld temp_cdeg=%ld\r\n",
                               (unsigned long)id,
                               op,
                               (snapshot.raw_status == (int32_t)BSP_SPL06_OK) ? 1U : 0U,
                               (long)snapshot.raw_status,
                               (long)snapshot.coef_status,
                               (unsigned int)snapshot.scaled_valid,
                               (long)snapshot.pressure_raw,
                               (long)snapshot.temperature_raw,
                               (long)snapshot.pressure_pa,
                               (long)snapshot.temperature_cdeg);
        app_control_queue_text("RSP id=%lu mod=SPL06 op=%s cfg prs=0x%02X tmp=0x%02X meas=0x%02X cfg=0x%02X int=0x%02X fifo=0x%02X\r\n",
                               (unsigned long)id,
                               op,
                               (unsigned int)snapshot.prs_cfg,
                               (unsigned int)snapshot.tmp_cfg,
                               (unsigned int)snapshot.meas_cfg,
                               (unsigned int)snapshot.cfg_reg,
                               (unsigned int)snapshot.int_sts,
                               (unsigned int)snapshot.fifo_sts);
        app_control_queue_text("RSP id=%lu mod=SPL06 op=%s regs=%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X\r\n",
                               (unsigned long)id,
                               op,
                               (unsigned int)snapshot.raw_regs[0],
                               (unsigned int)snapshot.raw_regs[1],
                               (unsigned int)snapshot.raw_regs[2],
                               (unsigned int)snapshot.raw_regs[3],
                               (unsigned int)snapshot.raw_regs[4],
                               (unsigned int)snapshot.raw_regs[5],
                               (unsigned int)snapshot.raw_regs[6],
                               (unsigned int)snapshot.raw_regs[7],
                               (unsigned int)snapshot.raw_regs[8],
                               (unsigned int)snapshot.raw_regs[9],
                               (unsigned int)snapshot.raw_regs[10],
                               (unsigned int)snapshot.raw_regs[11],
                               (unsigned int)snapshot.raw_regs[12],
                               (unsigned int)snapshot.raw_regs[13]);
        return;
    }

    app_control_protocol_err(id, "SPL06", op, "BAD_OP");
}

static void app_control_req_icm42688(uint32_t id, const char *op)
{
    APP_IMU_Status imu_status;

    if (op == NULL) {
        app_control_protocol_err(id, "ICM42688", "?", "NO_OP");
        return;
    }

    APP_IMU_GetStatus(&imu_status);

    if ((strcmp(op, "STATUS") == 0) || (strcmp(op, "DIAG") == 0)) {
        app_control_queue_text("RSP id=%lu mod=ICM42688 op=%s ok=%u stage=%s stage_id=%u who=0x%02X exp=0x%02X code=%ld\r\n",
                               (unsigned long)id,
                               op,
                               (unsigned int)imu_status.initialized,
                               app_control_imu_stage_name(imu_status.init_stage),
                               (unsigned int)imu_status.init_stage,
                               (unsigned int)imu_status.who_am_i,
                               (unsigned int)BSP_ICM42688_WHO_AM_I_VALUE,
                               (long)imu_status.last_error);
        app_control_queue_text("RSP id=%lu mod=ICM42688 op=%s st=%ld n=%lu ax=%d ay=%d az=%d gx=%ld gy=%ld gz=%ld t=%d\r\n",
                               (unsigned long)id,
                               op,
                               (long)imu_status.last_status,
                               (unsigned long)imu_status.sample_count,
                               (int)imu_status.accel_x_mg,
                               (int)imu_status.accel_y_mg,
                               (int)imu_status.accel_z_mg,
                               (long)imu_status.gyro_x_mdps,
                               (long)imu_status.gyro_y_mdps,
                               (long)imu_status.gyro_z_mdps,
                               (int)imu_status.temperature_cdeg);
        app_control_queue_text("RSP id=%lu mod=ICM42688 op=%s diag valid=%u m0_tok=0x%02X m0_msb=0x%02X m0_b0=0x%02X m3_tok=0x%02X m3_msb=0x%02X m3_b0=0x%02X best_mode=%u best_hdr=%u\r\n",
                               (unsigned long)id,
                               op,
                               (unsigned int)imu_status.diag_valid,
                               (unsigned int)imu_status.diag_mode0_tokmas,
                               (unsigned int)imu_status.diag_mode0_msb,
                               (unsigned int)imu_status.diag_mode0_bit0,
                               (unsigned int)imu_status.diag_mode3_tokmas,
                               (unsigned int)imu_status.diag_mode3_msb,
                               (unsigned int)imu_status.diag_mode3_bit0,
                               (unsigned int)imu_status.diag_best_mode,
                               (unsigned int)imu_status.diag_best_header);
        app_control_queue_text("RSP id=%lu mod=ICM42688 op=%s burst m0_b0=%02X%02X%02X%02X m3_tok=%02X%02X%02X%02X\r\n",
                               (unsigned long)id,
                               op,
                               (unsigned int)imu_status.diag_burst_m0_b0_1,
                               (unsigned int)imu_status.diag_burst_m0_b0_2,
                               (unsigned int)imu_status.diag_burst_m0_b0_3,
                               (unsigned int)imu_status.diag_burst_m0_b0_4,
                               (unsigned int)imu_status.diag_burst_m3_tok_1,
                               (unsigned int)imu_status.diag_burst_m3_tok_2,
                               (unsigned int)imu_status.diag_burst_m3_tok_3,
                               (unsigned int)imu_status.diag_burst_m3_tok_4);
        return;
    }

    app_control_protocol_err(id, "ICM42688", op, "BAD_OP");
}

static void app_control_handle_req(char **tokens, uint32_t count)
{
    const char *id_text = app_control_token_value(tokens, count, "id");
    const char *mod = app_control_token_value(tokens, count, "mod");
    const char *op = app_control_token_value(tokens, count, "op");
    uint32_t id = 0U;

    if ((id_text == NULL) || (app_control_parse_u32_auto(id_text, &id) == 0U)) {
        app_control_protocol_err(0U, (mod != NULL) ? mod : "?", (op != NULL) ? op : "?", "BAD_ID");
        return;
    }

    if (mod == NULL) {
        app_control_protocol_err(id, "?", (op != NULL) ? op : "?", "NO_MOD");
        return;
    }

    if (strcmp(mod, "SPL06") == 0) {
        app_control_req_spl06(id, op);
        return;
    }

    if (strcmp(mod, "ICM42688") == 0) {
        app_control_req_icm42688(id, op);
        return;
    }

    if (strcmp(mod, "WIFI") == 0) {
        if (op == NULL) {
            app_control_protocol_err(id, "WIFI", "?", "NO_OP");
            return;
        }
        if (strcmp(op, "STATUS") == 0) {
            app_control_queue_text("RSP id=%lu mod=WIFI op=STATUS en=%u pin=PC6 last=%u writes=%lu state=%s transparent=%u retry=%lu socket=%ld cycling=%u wait_ms=%lu prov=%u cmd=%lu/%lu\r\n",
                                   (unsigned long)id,
                                   (unsigned int)BSP_AiWB2_IsEnabled(),
                                   (unsigned int)BSP_AiWB2_GetLastWrittenState(),
                                   (unsigned long)BSP_AiWB2_GetWriteCount(),
                                   app_control_aiwb2_state_name(APP_AiWB2_GetState()),
                                   (unsigned int)APP_AiWB2_IsTransparent(),
                                   (unsigned long)APP_AiWB2_GetRetryCount(),
                                   (long)APP_AiWB2_GetLastSocketError(),
                                   (unsigned int)APP_AiWB2_IsPowerRecycleActive(),
                                   (unsigned long)APP_AiWB2_GetDeadlineRemainingMs(),
                                   (unsigned int)APP_AiWB2_IsProvisionActive(),
                                   (unsigned long)APP_AiWB2_GetCommandIndex(),
                                   (unsigned long)APP_AiWB2_GetCommandCount());
            return;
        }
        app_control_protocol_err(id, "WIFI", op, "BAD_OP");
        return;
    }

    app_control_protocol_err(id, mod, (op != NULL) ? op : "?", "BAD_MOD");
}

static void app_control_report_status(void)
{
    APP_Flash_Status flash_status;
    APP_Baro_Status baro_status;
    APP_IMU_Status imu_status;
    uint32_t uart_rx_bytes = 0U;
    uint32_t uart_rx_lines = 0U;
    uint32_t uart_rx_overflows = 0U;
    uint32_t uart_rx_errors = 0U;
    uint32_t uart_rx_events = 0U;
    uint32_t uart_rx_restarts = 0U;
    uint32_t uart_last_rx_event_size = 0U;

    APP_Flash_GetStatus(&flash_status);
    APP_Baro_GetStatus(&baro_status);
    APP_IMU_GetStatus(&imu_status);
    APP_UART_GetStats(&uart_rx_bytes,
                      &uart_rx_lines,
                      &uart_rx_overflows,
                      &uart_rx_errors);
    APP_UART_GetRxEventStats(&uart_rx_events,
                             &uart_rx_restarts,
                             &uart_last_rx_event_size);

    app_control_queue_proto_text(APP_PROTO_MSG_HW_FLASH,
                                 "HW FLASH ok=%u stage=%s probe=%ld sr=%ld read=%ld id=%02X%02X%02X exp=C84016 sr1=%02X\r\n",
                                 (unsigned int)app_control_flash_ok(&flash_status),
                                 app_control_flash_stage(&flash_status),
                                 (long)flash_status.probe_status,
                                 (long)flash_status.status1_status,
                                 (long)flash_status.read_status,
                                 (unsigned int)flash_status.manufacturer_id,
                                 (unsigned int)flash_status.memory_type,
                                 (unsigned int)flash_status.capacity_id,
                                 (unsigned int)flash_status.status1);
    app_control_queue_proto_text(APP_PROTO_MSG_HW_BARO,
                                 "HW SPL06 ok=%u stage=%s init=%ld split=%ld txrx=%ld id=%02X split_id=%02X txrx_id=%02X exp=10 cs=%u miso=%u\r\n",
                                 (unsigned int)app_control_baro_ok(&baro_status),
                                 app_control_baro_stage(&baro_status),
                                 (long)baro_status.init_status,
                                 (long)baro_status.split_status,
                                 (long)baro_status.txrx_status,
                                 (unsigned int)baro_status.product_id,
                                 (unsigned int)baro_status.split_id,
                                 (unsigned int)baro_status.txrx_id,
                                 (unsigned int)baro_status.cs_level,
                                 (unsigned int)baro_status.miso_level);
    app_control_queue_proto_text(APP_PROTO_MSG_HW_IMU,
                                 "HW ICM42688 ok=%u stage=%s st=%ld err=%ld who=%02X exp=%02X n=%lu\r\n",
                                 (unsigned int)imu_status.initialized,
                                 app_control_imu_stage_name(imu_status.init_stage),
                                 (long)imu_status.last_status,
                                 (long)imu_status.last_error,
                                 (unsigned int)imu_status.who_am_i,
                                 (unsigned int)BSP_ICM42688_WHO_AM_I_VALUE,
                                 (unsigned long)imu_status.sample_count);
    app_control_queue_proto_text(APP_PROTO_MSG_HW_IMU,
                                 "HW ICM42688 diag valid=%u m0_tok=%02X m0_msb=%02X m0_b0=%02X m3_tok=%02X m3_msb=%02X m3_b0=%02X best_mode=%u best_hdr=%u\r\n",
                                 (unsigned int)imu_status.diag_valid,
                                 (unsigned int)imu_status.diag_mode0_tokmas,
                                 (unsigned int)imu_status.diag_mode0_msb,
                                 (unsigned int)imu_status.diag_mode0_bit0,
                                 (unsigned int)imu_status.diag_mode3_tokmas,
                                 (unsigned int)imu_status.diag_mode3_msb,
                                 (unsigned int)imu_status.diag_mode3_bit0,
                                 (unsigned int)imu_status.diag_best_mode,
                                 (unsigned int)imu_status.diag_best_header);
    app_control_queue_proto_text(APP_PROTO_MSG_HW_IMU,
                                 "HW ICM42688 burst m0_b0=%02X%02X%02X%02X m3_tok=%02X%02X%02X%02X\r\n",
                                 (unsigned int)imu_status.diag_burst_m0_b0_1,
                                 (unsigned int)imu_status.diag_burst_m0_b0_2,
                                 (unsigned int)imu_status.diag_burst_m0_b0_3,
                                 (unsigned int)imu_status.diag_burst_m0_b0_4,
                                 (unsigned int)imu_status.diag_burst_m3_tok_1,
                                 (unsigned int)imu_status.diag_burst_m3_tok_2,
                                 (unsigned int)imu_status.diag_burst_m3_tok_3,
                                 (unsigned int)imu_status.diag_burst_m3_tok_4);

    app_control_queue_proto_text(APP_PROTO_MSG_STATUS_FLASH,
                                 "STATUS flash probe=%ld sr_st=%ld read=%ld id=%02X%02X%02X sr1=%02X\r\n",
                                 (long)flash_status.probe_status,
                                 (long)flash_status.status1_status,
                                 (long)flash_status.read_status,
                                 (unsigned int)flash_status.manufacturer_id,
                                 (unsigned int)flash_status.memory_type,
                                 (unsigned int)flash_status.capacity_id,
                                 (unsigned int)flash_status.status1);
    app_control_queue_proto_text(APP_PROTO_MSG_STATUS_BARO,
                                 "STATUS baro init=%ld split=%ld txrx=%ld id=0x%02X split_id=0x%02X txrx_id=0x%02X bmp=0x%02X cs=%u miso=%u\r\n",
                                 (long)baro_status.init_status,
                                 (long)baro_status.split_status,
                                 (long)baro_status.txrx_status,
                                 (unsigned int)baro_status.product_id,
                                 (unsigned int)baro_status.split_id,
                                 (unsigned int)baro_status.txrx_id,
                                 (unsigned int)baro_status.bmp280_id,
                                 (unsigned int)baro_status.cs_level,
                                 (unsigned int)baro_status.miso_level);
    app_control_queue_proto_text(APP_PROTO_MSG_STATUS_IMU,
                                 "STATUS imu init=%u stage=%s st=%ld err=%ld who=0x%02X n=%lu ax=%d ay=%d az=%d gx=%ld gy=%ld gz=%ld t=%d\r\n",
                                 (unsigned int)imu_status.initialized,
                                 app_control_imu_stage_name(imu_status.init_stage),
                                 (long)imu_status.last_status,
                                 (long)imu_status.last_error,
                                 (unsigned int)imu_status.who_am_i,
                                 (unsigned long)imu_status.sample_count,
                                 (int)imu_status.accel_x_mg,
                                 (int)imu_status.accel_y_mg,
                                 (int)imu_status.accel_z_mg,
                                 (long)imu_status.gyro_x_mdps,
                                 (long)imu_status.gyro_y_mdps,
                                 (long)imu_status.gyro_z_mdps,
                                 (int)imu_status.temperature_cdeg);
    app_control_queue_proto_text(APP_PROTO_MSG_UART_STATS,
                                 "UART1 rx_bytes=%lu rx_lines=%lu rx_overflows=%lu rx_errors=%lu rx_evt=%lu rx_rst=%lu rx_evt_size=%lu\r\n",
                                 (unsigned long)uart_rx_bytes,
                                 (unsigned long)uart_rx_lines,
                                 (unsigned long)uart_rx_overflows,
                                 (unsigned long)uart_rx_errors,
                                 (unsigned long)uart_rx_events,
                                 (unsigned long)uart_rx_restarts,
                                 (unsigned long)uart_last_rx_event_size);
    app_control_report_wifi();
}

static void app_control_report_uart_stats(uint32_t rx_bytes,
                                          uint32_t rx_lines,
                                          uint32_t rx_overflows,
                                          uint32_t rx_errors)
{
    uint32_t rx_events = 0U;
    uint32_t rx_restarts = 0U;
    uint32_t last_rx_event_size = 0U;

    APP_UART_GetRxEventStats(&rx_events,
                             &rx_restarts,
                             &last_rx_event_size);
    app_control_queue_proto_text(APP_PROTO_MSG_UART_STATS,
                                 "UART1 rx_bytes=%lu rx_lines=%lu rx_overflows=%lu rx_errors=%lu rx_evt=%lu rx_rst=%lu rx_evt_size=%lu\r\n",
                                 (unsigned long)rx_bytes,
                                 (unsigned long)rx_lines,
                                 (unsigned long)rx_overflows,
                                 (unsigned long)rx_errors,
                                 (unsigned long)rx_events,
                                 (unsigned long)rx_restarts,
                                 (unsigned long)last_rx_event_size);
}

static BSP_GD25Q32_Status app_control_load_config(void)
{
    APP_ControlFlashRecord record;
    BSP_GD25Q32_Status status;
    uint32_t checksum;

    status = BSP_FLASH_ReadData(APP_CONTROL_CFG_ADDRESS,
                                (uint8_t *)&record,
                                sizeof(record));
    if (status != BSP_GD25Q32_OK) {
        return status;
    }

    if ((record.magic != APP_CONTROL_CFG_MAGIC) ||
        (((record.version != APP_CONTROL_CFG_VERSION) ||
          (record.size != sizeof(record.config))) &&
         ((record.version != 1U) ||
          (record.size != APP_CONTROL_CFG_LEGACY_SIZE)))) {
        return BSP_GD25Q32_BAD_ID;
    }

    checksum = app_control_checksum((const uint8_t *)&record.config,
                                    record.size);
    if (checksum != record.checksum) {
        return BSP_GD25Q32_ERROR;
    }

    if (record.version == APP_CONTROL_CFG_VERSION) {
        control_config = record.config;
    } else {
        APP_ControlConfig migrated_config;

        app_control_defaults(&migrated_config);
        migrated_config.loaded_from_flash = record.config.loaded_from_flash;
        migrated_config.flash_valid = record.config.flash_valid;
        migrated_config.last_flash_status = record.config.last_flash_status;
        for (uint32_t index = 0U; index < APP_CONTROL_SERVO_COUNT; ++index) {
            migrated_config.servo[index] = record.config.servo[index];
        }
        control_config = migrated_config;
    }
    control_config.loaded_from_flash = 1U;
    control_config.flash_valid = 1U;
    return BSP_GD25Q32_OK;
}

static BSP_GD25Q32_Status app_control_save_config(void)
{
    APP_ControlFlashRecord record;
    BSP_GD25Q32_Status status;

    memset(&record, 0xFF, sizeof(record));
    record.magic = APP_CONTROL_CFG_MAGIC;
    record.version = APP_CONTROL_CFG_VERSION;
    record.size = sizeof(record.config);
    record.config = control_config;
    record.config.loaded_from_flash = 1U;
    record.config.flash_valid = 1U;
    record.checksum = app_control_checksum((const uint8_t *)&record.config,
                                           sizeof(record.config));

    status = BSP_FLASH_EraseSector(APP_CONTROL_CFG_ADDRESS);
    if (status != BSP_GD25Q32_OK) {
        return status;
    }

    return BSP_FLASH_WriteData(APP_CONTROL_CFG_ADDRESS,
                               (const uint8_t *)&record,
                               sizeof(record));
}

static void app_control_servo_move_configured(void)
{
    BSP_BusServoMove moves[APP_CONTROL_SERVO_COUNT];
    uint8_t count = 0U;
    uint16_t time_ms = control_config.servo[0].time_ms;
    BSP_BusServoStatus status;

    for (uint32_t index = 0U; index < APP_CONTROL_SERVO_COUNT; ++index) {
        if (control_config.servo[index].enabled == 0U) {
            continue;
        }

        moves[count].id = control_config.servo[index].id;
        moves[count].pulse_us = control_config.servo[index].pulse_us;
        if (control_config.servo[index].time_ms > time_ms) {
            time_ms = control_config.servo[index].time_ms;
        }
        ++count;
    }

    if (count == 0U) {
        app_control_queue_text("ERR servo no enabled channels\r\n");
        return;
    }

    status = BSP_BusServo_MoveMany(moves, count, time_ms);
    app_control_queue_text("OK servo move_all st=%u count=%u time=%u\r\n",
                           (unsigned int)status,
                           (unsigned int)count,
                           (unsigned int)time_ms);
}

static void app_control_handle_servo(char **tokens, uint32_t count)
{
    uint32_t index;
    uint32_t value;
    BSP_BusServoStatus status;

    if (count < 2U) {
        app_control_queue_text("ERR servo missing subcmd\r\n");
        return;
    }

    if (strcmp(tokens[1], "MOVE") == 0) {
        uint32_t pulse;
        uint32_t time_ms;
        if ((count < 5U) ||
            (app_control_parse_u32(tokens[2], &index) == 0U) ||
            (app_control_parse_u32(tokens[3], &pulse) == 0U) ||
            (app_control_parse_u32(tokens[4], &time_ms) == 0U) ||
            (app_control_valid_servo_index(index) == 0U)) {
            app_control_queue_text("ERR usage SERVO MOVE index pulse time\r\n");
            return;
        }

        control_config.servo[index].pulse_us = (uint16_t)pulse;
        control_config.servo[index].time_ms = (uint16_t)time_ms;
        status = BSP_BusServo_Move(control_config.servo[index].id,
                                   control_config.servo[index].pulse_us,
                                   control_config.servo[index].time_ms);
        app_control_queue_text("OK servo%lu move st=%u id=%u pulse=%u time=%u\r\n",
                               (unsigned long)index,
                               (unsigned int)status,
                               (unsigned int)control_config.servo[index].id,
                               (unsigned int)control_config.servo[index].pulse_us,
                               (unsigned int)control_config.servo[index].time_ms);
        return;
    }

    if (strcmp(tokens[1], "MOVEALL") == 0) {
        app_control_servo_move_configured();
        return;
    }

    if (strcmp(tokens[1], "ID") == 0) {
        if ((count < 4U) ||
            (app_control_parse_u32(tokens[2], &index) == 0U) ||
            (app_control_parse_u32(tokens[3], &value) == 0U) ||
            (app_control_valid_servo_index(index) == 0U) ||
            (value > 255U)) {
            app_control_queue_text("ERR usage SERVO ID index id\r\n");
            return;
        }

        control_config.servo[index].id = (uint8_t)value;
        app_control_queue_text("OK servo%lu id=%u\r\n",
                               (unsigned long)index,
                               (unsigned int)control_config.servo[index].id);
        return;
    }

    if (strcmp(tokens[1], "SETID") == 0) {
        uint32_t new_id;
        if ((count < 4U) ||
            (app_control_parse_u32(tokens[2], &index) == 0U) ||
            (app_control_parse_u32(tokens[3], &new_id) == 0U) ||
            (app_control_valid_servo_index(index) == 0U) ||
            (new_id > 255U)) {
            app_control_queue_text("ERR usage SERVO SETID index new_id\r\n");
            return;
        }

        status = BSP_BusServo_SetId(control_config.servo[index].id, (uint8_t)new_id);
        control_config.servo[index].id = (uint8_t)new_id;
        app_control_queue_text("OK servo%lu setid st=%u id=%u\r\n",
                               (unsigned long)index,
                               (unsigned int)status,
                               (unsigned int)new_id);
        return;
    }

    if (strcmp(tokens[1], "MODE") == 0) {
        if ((count < 4U) ||
            (app_control_parse_u32(tokens[2], &index) == 0U) ||
            (app_control_parse_u32(tokens[3], &value) == 0U) ||
            (app_control_valid_servo_index(index) == 0U)) {
            app_control_queue_text("ERR usage SERVO MODE index mode\r\n");
            return;
        }

        status = BSP_BusServo_SetMode(control_config.servo[index].id, (uint8_t)value);
        if (status == BSP_BUS_SERVO_OK) {
            control_config.servo[index].mode = (uint8_t)value;
        }
        app_control_queue_text("OK servo%lu mode st=%u mode=%u\r\n",
                               (unsigned long)index,
                               (unsigned int)status,
                               (unsigned int)control_config.servo[index].mode);
        return;
    }

    if (strcmp(tokens[1], "ENABLE") == 0) {
        if ((count < 4U) ||
            (app_control_parse_u32(tokens[2], &index) == 0U) ||
            (app_control_parse_u32(tokens[3], &value) == 0U) ||
            (app_control_valid_servo_index(index) == 0U)) {
            app_control_queue_text("ERR usage SERVO ENABLE index 0|1\r\n");
            return;
        }

        control_config.servo[index].enabled = (value != 0U) ? 1U : 0U;
        app_control_queue_text("OK servo%lu enabled=%u\r\n",
                               (unsigned long)index,
                               (unsigned int)control_config.servo[index].enabled);
        return;
    }

    if (strcmp(tokens[1], "CMD") == 0) {
        if ((count < 4U) ||
            (app_control_parse_u32(tokens[2], &index) == 0U) ||
            (app_control_valid_servo_index(index) == 0U)) {
            app_control_queue_text("ERR usage SERVO CMD index action\r\n");
            return;
        }

        if (strcmp(tokens[3], "VER") == 0) {
            status = BSP_BusServo_ReadVersion(control_config.servo[index].id);
        } else if (strcmp(tokens[3], "PID") == 0) {
            status = BSP_BusServo_ReadId(control_config.servo[index].id);
        } else if (strcmp(tokens[3], "RAD") == 0) {
            status = BSP_BusServo_ReadPosition(control_config.servo[index].id);
        } else if (strcmp(tokens[3], "MOD?") == 0) {
            status = BSP_BusServo_ReadMode(control_config.servo[index].id);
        } else if (strcmp(tokens[3], "ULK") == 0) {
            status = BSP_BusServo_ReleaseTorque(control_config.servo[index].id);
        } else if (strcmp(tokens[3], "ULR") == 0) {
            status = BSP_BusServo_RestoreTorque(control_config.servo[index].id);
        } else if (strcmp(tokens[3], "DPT") == 0) {
            status = BSP_BusServo_Pause(control_config.servo[index].id);
        } else if (strcmp(tokens[3], "DCT") == 0) {
            status = BSP_BusServo_Continue(control_config.servo[index].id);
        } else if (strcmp(tokens[3], "DST") == 0) {
            status = BSP_BusServo_Stop(control_config.servo[index].id);
        } else if (strcmp(tokens[3], "BD") == 0) {
            uint32_t baud_code;
            if ((count < 5U) || (app_control_parse_u32(tokens[4], &baud_code) == 0U)) {
                app_control_queue_text("ERR usage SERVO CMD index BD code\r\n");
                return;
            }
            status = BSP_BusServo_SetBaud(control_config.servo[index].id, (uint8_t)baud_code);
        } else if (strcmp(tokens[3], "SCK") == 0) {
            status = BSP_BusServo_SaveCenter(control_config.servo[index].id);
        } else if (strcmp(tokens[3], "CSD") == 0) {
            status = BSP_BusServo_SetStartupPosition(control_config.servo[index].id);
        } else if (strcmp(tokens[3], "CSM") == 0) {
            status = BSP_BusServo_ClearStartupPosition(control_config.servo[index].id);
        } else if (strcmp(tokens[3], "CSR") == 0) {
            status = BSP_BusServo_RestoreStartupPosition(control_config.servo[index].id);
        } else if (strcmp(tokens[3], "SMI") == 0) {
            status = BSP_BusServo_SetMinPosition(control_config.servo[index].id);
        } else if (strcmp(tokens[3], "SMX") == 0) {
            status = BSP_BusServo_SetMaxPosition(control_config.servo[index].id);
        } else if (strcmp(tokens[3], "CLEO") == 0) {
            status = BSP_BusServo_FactoryResetKeepId(control_config.servo[index].id);
        } else if (strcmp(tokens[3], "CLE") == 0) {
            status = BSP_BusServo_FactoryResetFull(control_config.servo[index].id);
        } else {
            app_control_queue_text("ERR unknown servo action %s\r\n", tokens[3]);
            return;
        }

        app_control_queue_text("OK servo%lu cmd=%s st=%u\r\n",
                               (unsigned long)index,
                               tokens[3],
                               (unsigned int)status);
        return;
    }

    if (strcmp(tokens[1], "RAW") == 0) {
        if (count < 3U) {
            app_control_queue_text("ERR usage SERVO RAW command\r\n");
            return;
        }

        status = BSP_BusServo_SendRaw(tokens[2]);
        app_control_queue_text("OK servo raw st=%u\r\n", (unsigned int)status);
        return;
    }

    app_control_queue_text("ERR unknown servo subcmd %s\r\n", tokens[1]);
}

static void app_control_handle_wifi(char **tokens, uint32_t count)
{
    char raw_command[APP_CONTROL_MAX_LINE];
    uint32_t value;
    uint32_t pulse_ms = 500U;

    if ((count == 1U) ||
        ((count >= 2U) &&
         ((strcmp(tokens[1], "?") == 0) || (strcmp(tokens[1], "STATUS") == 0)))) {
        app_control_report_wifi();
        return;
    }

    if (strcmp(tokens[1], "AT") == 0) {
        uint32_t offset = 0U;

        if (count < 3U) {
            app_control_queue_text("ERR usage WIFI AT command\r\n");
            return;
        }

        raw_command[0] = '\0';
        for (uint32_t i = 2U; i < count; ++i) {
            int written = snprintf(&raw_command[offset],
                                   sizeof(raw_command) - offset,
                                   "%s%s",
                                   (i > 2U) ? " " : "",
                                   tokens[i]);
            if ((written < 0) || ((uint32_t)written >= (sizeof(raw_command) - offset))) {
                app_control_queue_text("ERR wifi at too long\r\n");
                return;
            }
            offset += (uint32_t)written;
        }

        if (APP_AiWB2_SendRawCommand(raw_command) == 0U) {
            app_control_queue_text("ERR wifi at bad command\r\n");
            return;
        }

        app_control_queue_text("OK wifi at %s\r\n", raw_command);
        return;
    }

    if (strcmp(tokens[1], "DIAG") == 0) {
        APP_AiWB2_SendDiagCommands();
        app_control_queue_text("OK wifi diag queued\r\n");
        return;
    }

    if ((strcmp(tokens[1], "EN") == 0) || (strcmp(tokens[1], "ENABLE") == 0)) {
        if ((count < 3U) || (app_control_parse_u32(tokens[2], &value) == 0U)) {
            app_control_queue_text("ERR usage WIFI EN 0|1\r\n");
            return;
        }

        control_wifi_reset_pending = 0U;
        BSP_AiWB2_SetEnabled((value != 0U) ? 1U : 0U);
        app_control_queue_text("OK wifi en=%u pin=PC6\r\n",
                               (unsigned int)BSP_AiWB2_IsEnabled());
        return;
    }

    if (strcmp(tokens[1], "RESET") == 0) {
        if ((count >= 3U) && (app_control_parse_u32(tokens[2], &pulse_ms) == 0U)) {
            app_control_queue_text("ERR usage WIFI RESET [ms]\r\n");
            return;
        }
        if (pulse_ms > 5000U) {
            pulse_ms = 5000U;
        }

        BSP_AiWB2_SetEnabled(0U);
        control_wifi_reset_pending = 1U;
        control_wifi_reset_deadline_ms = HAL_GetTick() + pulse_ms;
        app_control_queue_text("OK wifi reset queued ms=%lu pin=PC6\r\n",
                               (unsigned long)pulse_ms);
        return;
    }

    if ((strcmp(tokens[1], "STA") == 0) || (strcmp(tokens[1], "PROVISION") == 0)) {
        if (count < 6U) {
            app_control_queue_text("ERR usage WIFI STA ssid password host port\r\n");
            return;
        }

        if (APP_AiWB2_StartProvision(tokens[2], tokens[3], tokens[4], tokens[5]) == 0U) {
            app_control_queue_text("ERR wifi sta bad args\r\n");
            return;
        }

        app_control_queue_text("OK wifi sta queued ssid=%s host=%s port=%s\r\n",
                               tokens[2],
                               tokens[4],
                               tokens[5]);
        return;
    }

    app_control_queue_text("ERR unknown wifi subcmd %s\r\n", tokens[1]);
}

static void app_control_service_wifi_reset(void)
{
    if (control_wifi_reset_pending == 0U) {
        return;
    }

    if ((int32_t)(HAL_GetTick() - control_wifi_reset_deadline_ms) < 0) {
        return;
    }

    control_wifi_reset_pending = 0U;
    BSP_AiWB2_SetEnabled(1U);
    APP_AiWB2_Init();
    app_control_queue_text("OK wifi reset done en=%u\r\n",
                           (unsigned int)BSP_AiWB2_IsEnabled());
}

static APP_ControlPidConfig *app_control_find_pid(const char *group, uint32_t index)
{
    if (group == NULL) {
        return NULL;
    }

    if (strcmp(group, "RATE") == 0) {
        return (index < APP_CONTROL_PID_AXIS_COUNT) ? &control_config.rate_pid[index] : NULL;
    }

    if (strcmp(group, "ANGLE") == 0) {
        return (index < APP_CONTROL_PID_AXIS_COUNT) ? &control_config.angle_pid[index] : NULL;
    }

    if (strcmp(group, "ALT") == 0) {
        return (index == 0U) ? &control_config.altitude_pid : NULL;
    }

    return NULL;
}

static void app_control_handle_baro(char **tokens, uint32_t count)
{
    uint32_t value;

    if ((count == 1U) || ((count >= 2U) && (strcmp(tokens[1], "?") == 0))) {
        app_control_report_baro();
        return;
    }

    if (strcmp(tokens[1], "STREAM") != 0) {
        app_control_queue_text("ERR usage BARO STREAM 0|1 [period_ms]\r\n");
        return;
    }

    if ((count < 3U) || (app_control_parse_u32(tokens[2], &value) == 0U)) {
        app_control_queue_text("ERR usage BARO STREAM 0|1 [period_ms]\r\n");
        return;
    }

    control_baro_stream_enabled = (value != 0U) ? 1U : 0U;
    control_baro_stream_period_ms = APP_CONTROL_BARO_STREAM_DEFAULT_PERIOD_MS;
    if ((count >= 4U) && (app_control_parse_u32(tokens[3], &value) != 0U) && (value > 0U)) {
        control_baro_stream_period_ms = value;
    }
    if ((control_baro_stream_enabled != 0U) &&
        (control_baro_stream_period_ms < APP_CONTROL_BARO_STREAM_MIN_PERIOD_MS)) {
        control_baro_stream_period_ms = APP_CONTROL_BARO_STREAM_MIN_PERIOD_MS;
    }
    control_last_baro_stream_ms = 0U;
    app_control_queue_text("OK baro stream=%u period_ms=%lu\r\n",
                           (unsigned int)control_baro_stream_enabled,
                           (unsigned long)control_baro_stream_period_ms);
}

static void app_control_handle_pid(char **tokens, uint32_t count)
{
    char *mapped_tokens[10] = {0};

    if ((count == 1U) || ((count >= 2U) && (strcmp(tokens[1], "?") == 0)) ||
        ((count >= 2U) && (strcmp(tokens[1], "GET") == 0))) {
        app_control_report_pid_legacy();
        return;
    }

    if ((count < 3U) || (strcmp(tokens[1], "SET") != 0)) {
        app_control_queue_text("ERR usage PID SET roll|pitch|yaw kp= ki= kd=\r\n");
        return;
    }

    mapped_tokens[0] = "PARAM";
    mapped_tokens[1] = "SET";
    if (strcmp(tokens[2], "roll") == 0) {
        mapped_tokens[2] = "RATE";
        mapped_tokens[3] = "0";
    } else if (strcmp(tokens[2], "pitch") == 0) {
        mapped_tokens[2] = "RATE";
        mapped_tokens[3] = "1";
    } else if (strcmp(tokens[2], "yaw") == 0) {
        mapped_tokens[2] = "RATE";
        mapped_tokens[3] = "2";
    } else {
        app_control_queue_text("ERR pid axis %s\r\n", tokens[2]);
        return;
    }

    mapped_tokens[4] = (char *)app_control_token_value(tokens, count, "kp");
    mapped_tokens[5] = (char *)app_control_token_value(tokens, count, "ki");
    mapped_tokens[6] = (char *)app_control_token_value(tokens, count, "kd");
    mapped_tokens[7] = "300";
    mapped_tokens[8] = "500";

    if ((mapped_tokens[4] == NULL) || (mapped_tokens[5] == NULL) || (mapped_tokens[6] == NULL)) {
        app_control_queue_text("ERR usage PID SET roll|pitch|yaw kp= ki= kd=\r\n");
        return;
    }

    app_control_handle_param(mapped_tokens, 9U);
}

static void app_control_service_streams(void)
{
    uint32_t now_ms = HAL_GetTick();

    if ((control_baro_stream_enabled == 0U) || (control_baro_stream_period_ms == 0U)) {
        return;
    }

    if ((control_last_baro_stream_ms != 0U) &&
        ((now_ms - control_last_baro_stream_ms) < control_baro_stream_period_ms)) {
        return;
    }

    control_last_baro_stream_ms = now_ms;
    app_control_emit_baro_event();
}

static void app_control_report_pid_legacy(void)
{
    static const char *axis_names[APP_CONTROL_PID_AXIS_COUNT] = {"roll", "pitch", "yaw"};

    for (uint32_t axis = 0U; axis < APP_CONTROL_PID_AXIS_COUNT; ++axis) {
        app_control_queue_proto_text(APP_PROTO_MSG_PID_RECORD,
                                     "PID axis=%s kp=%d ki=%d kd=%d ilim=%d out=%d\r\n",
                                     axis_names[axis],
                                     (int)control_config.rate_pid[axis].kp,
                                     (int)control_config.rate_pid[axis].ki,
                                     (int)control_config.rate_pid[axis].kd,
                                     (int)control_config.rate_pid[axis].integral_limit,
                                     (int)control_config.rate_pid[axis].output_limit);
    }
}

static void app_control_handle_param(char **tokens, uint32_t count)
{
    uint32_t index;
    int32_t kp;
    int32_t ki;
    int32_t kd;
    int32_t integral_limit;
    int32_t output_limit;
    APP_ControlPidConfig *pid;

    if ((count == 1U) || ((count >= 2U) && (strcmp(tokens[1], "?") == 0))) {
        app_control_report_params();
        return;
    }

    if (strcmp(tokens[1], "GET") == 0) {
        app_control_report_params();
        return;
    }

    if (strcmp(tokens[1], "SET") != 0) {
        app_control_queue_text("ERR usage PARAM SET RATE|ANGLE|ALT index kp ki kd ilim out\r\n");
        return;
    }

    if ((count < 9U) ||
        (app_control_parse_u32(tokens[3], &index) == 0U) ||
        (app_control_parse_i32(tokens[4], &kp) == 0U) ||
        (app_control_parse_i32(tokens[5], &ki) == 0U) ||
        (app_control_parse_i32(tokens[6], &kd) == 0U) ||
        (app_control_parse_i32(tokens[7], &integral_limit) == 0U) ||
        (app_control_parse_i32(tokens[8], &output_limit) == 0U)) {
        app_control_queue_text("ERR usage PARAM SET RATE|ANGLE|ALT index kp ki kd ilim out\r\n");
        return;
    }

    pid = app_control_find_pid(tokens[2], index);
    if (pid == NULL) {
        app_control_queue_text("ERR param target %s%lu\r\n",
                               tokens[2],
                               (unsigned long)index);
        return;
    }

    pid->kp = app_control_clamp_i16(kp);
    pid->ki = app_control_clamp_i16(ki);
    pid->kd = app_control_clamp_i16(kd);
    pid->integral_limit = app_control_clamp_i16(integral_limit);
    pid->output_limit = app_control_clamp_i16(output_limit);
    app_control_queue_text("OK param %s%lu kp=%d ki=%d kd=%d ilim=%d out=%d\r\n",
                           tokens[2],
                           (unsigned long)index,
                           (int)pid->kp,
                           (int)pid->ki,
                           (int)pid->kd,
                           (int)pid->integral_limit,
                           (int)pid->output_limit);
}

void APP_Control_Init(void)
{
    BSP_GD25Q32_Status load_status;

    if (control_initialized != 0U) {
        return;
    }

    app_control_defaults(&control_config);
    control_wifi_reset_pending = 0U;
    control_wifi_reset_deadline_ms = 0U;
    load_status = app_control_load_config();
    control_config.last_flash_status = (uint8_t)load_status;
    if (load_status != BSP_GD25Q32_OK) {
        control_config.loaded_from_flash = 0U;
        control_config.flash_valid = 0U;
    }

#if (APP_CONTROL_BOOT_READY_ENABLED != 0U)
    app_control_queue_text("READY drone-H743 tcp-control servo_slots=2 cfg_loaded=%u cfg_valid=%u\r\n",
                           (unsigned int)control_config.loaded_from_flash,
                           (unsigned int)control_config.flash_valid);
#endif
    control_initialized = 1U;
}

void APP_Control_Tick(void)
{
    app_control_tick_common(1U);
}

void APP_Control_MaintTick(void)
{
    uint8_t saved_output = control_maint_output_active;

    control_maint_output_active = 1U;
    app_control_tick_common(0U);
    control_maint_output_active = saved_output;
}

static void app_control_tick_common(uint8_t emit_heartbeat)
{
    uint32_t uart_rx_bytes = 0U;
    uint32_t uart_rx_lines = 0U;
    uint32_t uart_rx_overflows = 0U;
    uint32_t uart_rx_errors = 0U;
    uint32_t uart_rx_events = 0U;
    uint32_t uart_rx_restarts = 0U;
    uint32_t uart_last_rx_event_size = 0U;

    app_control_service_wifi_reset();
    app_control_service_streams();
    if (emit_heartbeat == 0U) {
        return;
    }
#if (APP_CONTROL_HEARTBEAT_ENABLED != 0U)
    {
        uint32_t now_ms = HAL_GetTick();

        if ((now_ms - control_last_heartbeat_ms) < 2000U) {
            return;
        }

        APP_UART_GetStats(&uart_rx_bytes,
                          &uart_rx_lines,
                          &uart_rx_overflows,
                          &uart_rx_errors);
        APP_UART_GetRxEventStats(&uart_rx_events,
                                 &uart_rx_restarts,
                                 &uart_last_rx_event_size);
        control_last_heartbeat_ms = now_ms;
        app_control_queue_text("READY ms=%lu servo0_id=%u servo1_id=%u cfg_valid=%u wifi=%u wifi_last=%u wifi_writes=%lu trans=%u rx_bytes=%lu rx_lines=%lu rx_ovf=%lu rx_err=%lu rx_evt=%lu rx_rst=%lu rx_evt_size=%lu\r\n",
                               (unsigned long)now_ms,
                               (unsigned int)control_config.servo[0].id,
                               (unsigned int)control_config.servo[1].id,
                               (unsigned int)control_config.flash_valid,
                               (unsigned int)BSP_AiWB2_IsEnabled(),
                               (unsigned int)BSP_AiWB2_GetLastWrittenState(),
                               (unsigned long)BSP_AiWB2_GetWriteCount(),
                               (unsigned int)APP_AiWB2_IsTransparent(),
                               (unsigned long)uart_rx_bytes,
                               (unsigned long)uart_rx_lines,
                               (unsigned long)uart_rx_overflows,
                               (unsigned long)uart_rx_errors,
                               (unsigned long)uart_rx_events,
                               (unsigned long)uart_rx_restarts,
                               (unsigned long)uart_last_rx_event_size);

        if (control_reported_hw_once == 0U) {
            control_reported_hw_once = 1U;
            app_control_report_status();
        }
    }
#endif
}

static void app_control_dispatch_tokens(char **tokens, uint32_t count, uint8_t emit_ack)
{
    if ((tokens == NULL) || (count == 0U)) {
        return;
    }

    if (emit_ack != 0U) {
        app_control_queue_text("ACK %s\r\n", tokens[0]);
    }

    if (strcmp(tokens[0], "PING") == 0) {
        app_control_queue_proto_text(APP_PROTO_MSG_PONG, "PONG drone-H743\r\n");
    } else if (strcmp(tokens[0], "MODULES?") == 0) {
        app_control_report_modules();
    } else if (strcmp(tokens[0], "CAPS?") == 0) {
        app_control_report_caps();
    } else if (strcmp(tokens[0], "REQ") == 0) {
        app_control_handle_req(tokens, count);
    } else if (strcmp(tokens[0], "STATUS?") == 0) {
        app_control_report_status();
    } else if (strcmp(tokens[0], "FLASH?") == 0) {
        app_control_report_flash();
    } else if (strcmp(tokens[0], "BARO?") == 0) {
        app_control_report_baro();
    } else if (strcmp(tokens[0], "BARO") == 0) {
        app_control_handle_baro(tokens, count);
    } else if (strcmp(tokens[0], "IMU?") == 0) {
        app_control_report_imu();
    } else if (strcmp(tokens[0], "PARAM?") == 0) {
        app_control_report_params();
    } else if (strcmp(tokens[0], "PID?") == 0) {
        app_control_report_pid_legacy();
    } else if (strcmp(tokens[0], "CONFIG?") == 0) {
        app_control_report_config();
    } else if ((strcmp(tokens[0], "WIFI?") == 0) ||
               (strcmp(tokens[0], "WIFI_EN?") == 0)) {
        app_control_report_wifi();
    } else if (strcmp(tokens[0], "WIFI") == 0) {
        app_control_handle_wifi(tokens, count);
    } else if (strcmp(tokens[0], "WIFI_EN") == 0) {
        char *wifi_tokens[3] = {"WIFI", "EN", NULL};
        if (count < 2U) {
            app_control_queue_text("ERR usage WIFI_EN 0|1\r\n");
            return;
        }
        wifi_tokens[2] = tokens[1];
        app_control_handle_wifi(wifi_tokens, 3U);
    } else if (strcmp(tokens[0], "SAVE") == 0) {
        BSP_GD25Q32_Status save_status = app_control_save_config();
        control_config.last_flash_status = (uint8_t)save_status;
        if (save_status == BSP_GD25Q32_OK) {
            control_config.loaded_from_flash = 1U;
            control_config.flash_valid = 1U;
        }
        app_control_queue_text("OK save st=%u\r\n", (unsigned int)save_status);
    } else if (strcmp(tokens[0], "LOAD") == 0) {
        BSP_GD25Q32_Status load_status = app_control_load_config();
        control_config.last_flash_status = (uint8_t)load_status;
        app_control_queue_text("OK load st=%u\r\n", (unsigned int)load_status);
    } else if (strcmp(tokens[0], "DEFAULTS") == 0) {
        app_control_defaults(&control_config);
        app_control_queue_text("OK defaults\r\n");
    } else if (strcmp(tokens[0], "PARAM") == 0) {
        app_control_handle_param(tokens, count);
    } else if (strcmp(tokens[0], "PID") == 0) {
        app_control_handle_pid(tokens, count);
    } else if (strcmp(tokens[0], "SERVO") == 0) {
        app_control_handle_servo(tokens, count);
    } else {
        app_control_queue_text("ERR unknown cmd %s\r\n", tokens[0]);
    }
}

void APP_Control_ProcessLine(const char *line)
{
    char buffer[APP_CONTROL_MAX_LINE];
    char *tokens[10];
    uint32_t count;

    if ((line == NULL) || (*line == '\0')) {
        return;
    }

    app_control_queue_text("RX %s\r\n", line);

    (void)snprintf(buffer, sizeof(buffer), "%s", line);
    count = app_control_tokenize(buffer, tokens, (uint32_t)(sizeof(tokens) / sizeof(tokens[0])));

    if (count == 0U) {
        return;
    }

    app_control_dispatch_tokens(tokens, count, 1U);
}

void APP_Control_ProcessMaintLine(const char *line)
{
    uint8_t saved_output = control_maint_output_active;

    control_maint_output_active = 1U;
    APP_Control_ProcessLine(line);
    control_maint_output_active = saved_output;
}

static void app_control_process_proto_request(uint16_t function,
                                              const uint8_t *payload,
                                              uint16_t payload_length)
{
    char text[APP_CONTROL_MAX_LINE];
    uint16_t saved_override = control_response_override_function;

    control_response_override_function = app_control_response_function_for_request(function);

    switch (function) {
    case APP_PROTO_REQ_PING:
        app_control_dispatch_tokens((char *[]){"PING"}, 1U, 0U);
        break;

    case APP_PROTO_REQ_STATUS:
        app_control_dispatch_tokens((char *[]){"STATUS?"}, 1U, 0U);
        break;

    case APP_PROTO_REQ_CONFIG:
        app_control_dispatch_tokens((char *[]){"CONFIG?"}, 1U, 0U);
        break;

    case APP_PROTO_REQ_PARAMS:
        app_control_dispatch_tokens((char *[]){"PARAM?"}, 1U, 0U);
        break;

    case APP_PROTO_REQ_PID:
        app_control_dispatch_tokens((char *[]){"PID?"}, 1U, 0U);
        break;

    case APP_PROTO_REQ_BARO:
        app_control_dispatch_tokens((char *[]){"BARO?"}, 1U, 0U);
        break;

    case APP_PROTO_REQ_FLASH:
        app_control_dispatch_tokens((char *[]){"FLASH?"}, 1U, 0U);
        break;

    case APP_PROTO_REQ_IMU:
        app_control_dispatch_tokens((char *[]){"IMU?"}, 1U, 0U);
        break;

    case APP_PROTO_REQ_MODULES:
        app_control_dispatch_tokens((char *[]){"MODULES?"}, 1U, 0U);
        break;

    case APP_PROTO_REQ_CAPS:
        app_control_dispatch_tokens((char *[]){"CAPS?"}, 1U, 0U);
        break;

    case APP_PROTO_REQ_WIFI:
        if (app_control_copy_payload_text(text, sizeof(text), payload, payload_length) == 0U) {
            app_control_dispatch_tokens((char *[]){"WIFI?"}, 1U, 0U);
        } else if (app_control_payload_to_line(payload, payload_length, text, sizeof(text)) != 0U) {
            char *tokens[10];
            uint32_t count = app_control_tokenize(text,
                                                  tokens,
                                                  (uint32_t)(sizeof(tokens) / sizeof(tokens[0])));
            if (count != 0U) {
                app_control_dispatch_tokens(tokens, count, 0U);
            } else {
                app_control_queue_text("ERR empty payload fn=0x%04X\r\n", (unsigned int)function);
            }
        } else {
            app_control_queue_text("ERR invalid payload fn=0x%04X\r\n", (unsigned int)function);
        }
        break;

    case APP_PROTO_REQ_SAVE:
        app_control_dispatch_tokens((char *[]){"SAVE"}, 1U, 0U);
        break;

    case APP_PROTO_REQ_LOAD:
        app_control_dispatch_tokens((char *[]){"LOAD"}, 1U, 0U);
        break;

    case APP_PROTO_REQ_DEFAULTS:
        app_control_dispatch_tokens((char *[]){"DEFAULTS"}, 1U, 0U);
        break;

    case APP_PROTO_REQ_BARO_STREAM:
        if (app_control_copy_payload_text(text, sizeof(text), payload, payload_length) == 0U) {
            app_control_dispatch_tokens((char *[]){"BARO", "STREAM", "1"}, 3U, 0U);
        } else if (app_control_payload_to_line(payload, payload_length, text, sizeof(text)) != 0U) {
            char *tokens[10];
            uint32_t count = app_control_tokenize(text,
                                                  tokens,
                                                  (uint32_t)(sizeof(tokens) / sizeof(tokens[0])));
            if (count != 0U) {
                app_control_dispatch_tokens(tokens, count, 0U);
            } else {
                app_control_queue_text("ERR empty payload fn=0x%04X\r\n", (unsigned int)function);
            }
        } else {
            app_control_queue_text("ERR invalid payload fn=0x%04X\r\n", (unsigned int)function);
        }
        break;

    case APP_PROTO_REQ_PARAM_SET:
    case APP_PROTO_REQ_PID_SET:
    case APP_PROTO_REQ_SERVO_MOVE:
    case APP_PROTO_REQ_SERVO_MOVE_ALL:
    case APP_PROTO_REQ_SERVO_ID:
    case APP_PROTO_REQ_SERVO_SETID:
    case APP_PROTO_REQ_SERVO_MODE:
    case APP_PROTO_REQ_SERVO_ENABLE:
    case APP_PROTO_REQ_SERVO_ACTION:
    case APP_PROTO_REQ_SERVO_RAW:
    case APP_PROTO_MSG_CMD_LINE:
        if (app_control_payload_to_line(payload, payload_length, text, sizeof(text)) == 0U) {
            app_control_queue_text("ERR empty payload fn=0x%04X\r\n", (unsigned int)function);
        } else {
            char *tokens[10];
            uint32_t count = app_control_tokenize(text,
                                                  tokens,
                                                  (uint32_t)(sizeof(tokens) / sizeof(tokens[0])));
            if (count != 0U) {
                app_control_dispatch_tokens(tokens, count, 0U);
            }
        }
        break;

    default:
        app_control_queue_text("ERR unknown fn 0x%04X\r\n", (unsigned int)function);
        break;
    }

    control_response_override_function = saved_override;
}

void APP_Control_ProcessProtoRequest(uint16_t function,
                                     const uint8_t *payload,
                                     uint16_t payload_length)
{
    app_control_process_proto_request(function, payload, payload_length);
}

void APP_Control_GetConfig(APP_ControlConfig *config)
{
    if (config == NULL) {
        return;
    }

    *config = control_config;
}

void APP_Control_ReportUartStats(uint32_t rx_bytes,
                                  uint32_t rx_lines,
                                  uint32_t rx_overflows,
                                  uint32_t rx_errors)
{
    app_control_report_uart_stats(rx_bytes, rx_lines, rx_overflows, rx_errors);
}

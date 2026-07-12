#include "app_control.h"

#include "app_aiwb2.h"
#include "app_baro.h"
#include "app_flash.h"
#include "app_flight_log.h"
#include "app_diag.h"
#include "app_gps.h"
#include "app_ident.h"
#include "app_optical_flow.h"
#include "app_rangefinder.h"
#include "app_sensor.h"
#include "app_mag.h"
#include "app_maint_uart.h"
#include "app_messages.h"
#include "app_nav_estimator.h"
#include "app_proto.h"
#include "app_tasks.h"
#include "app_uart.h"
#include "bsp_bus_servo.h"
#include "bsp_aiwb2_power.h"
#include "bsp_baro.h"
#include "bsp_optical_flow.h"
#include "app_flash_service.h"
#include "bsp_imu.h"
#include "bsp_pwm.h"
#include "bsp_uart.h"
#include "drv_airframe_model.h"
#include "drv_coax_ctrl.h"
#include "drv_motor.h"

#include "FreeRTOS.h"
#include "main.h"
#include "task.h"
#include "tim.h"

#include <math.h>
#include <stddef.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define APP_CONTROL_CFG_MAGIC       0x44524346UL
#define APP_CONTROL_CFG_VERSION     8U
#define APP_CONTROL_CFG_ADDRESS     (APP_FLASH_SERVICE_SIZE_BYTES - 4096UL)
#define APP_CONTROL_MAX_LINE        128U
#define APP_CONTROL_HEARTBEAT_ENABLED 0U
#define APP_CONTROL_BOOT_READY_ENABLED 0U
#define APP_CONTROL_ASCII_RX_ECHO_ENABLED 0U
#define APP_CONTROL_ASCII_ACK_ENABLED 0U
#define APP_CONTROL_BARO_STREAM_DEFAULT_PERIOD_MS 50U
#define APP_CONTROL_BARO_STREAM_MIN_PERIOD_MS 20U
#define APP_CONTROL_FLASH_BENCH_MAX_LEN 4096U
#define APP_CONTROL_FLASH_BENCH_DEFAULT_ADDR 0U
#define APP_CONTROL_FLASH_BENCH_DEFAULT_LEN 512U
#define APP_CONTROL_FLASH_BENCH_DEFAULT_LOOPS 100U
#define APP_CONTROL_FLASH_SCRATCH_ADDR (APP_FLASH_SERVICE_SIZE_BYTES - 4U * 4096UL)
#define APP_CONTROL_IDENT_DEFAULT_MIN_PERCENT 0U
#define APP_CONTROL_IDENT_DEFAULT_MAX_PERCENT 100U
#define APP_CONTROL_IDENT_DEFAULT_STEP_PERCENT 5U
#define APP_CONTROL_IDENT_DEFAULT_DWELL_MS 2000U
#define APP_CONTROL_ALLOW_RAW_PWM_COMMANDS 0U
#define APP_CONTROL_ALLOW_RAW_MOTOR_COMMANDS 0U
#define APP_CONTROL_ALLOW_IDENT_MOTOR_TEST 0U
#define APP_CONTROL_FLASH_AUTOSAVE_DELAY_MS 1500U
#define APP_CONTROL_DEG_TO_RAD 0.017453292519943295f
#define APP_CONTROL_SERVO_ANGLE_MAX_DEG 90.0f
#define APP_CONTROL_FLOW_RAW_MAX_BYTES 32U

typedef struct {
    uint8_t loaded_from_flash;
    uint8_t flash_valid;
    uint8_t last_flash_status;
    APP_ControlServoConfig servo[APP_CONTROL_SERVO_COUNT];
} APP_ControlConfigV1;

typedef struct {
    int16_t kp;
    int16_t ki;
    int16_t kd;
    int16_t integral_limit;
    int16_t output_limit;
} APP_ControlLegacyPidConfig;

typedef struct {
    uint8_t loaded_from_flash;
    uint8_t flash_valid;
    uint8_t last_flash_status;
    APP_ControlServoConfig servo[APP_CONTROL_SERVO_COUNT];
    APP_ControlLegacyPidConfig rate_pid[3];
    APP_ControlLegacyPidConfig angle_pid[3];
    APP_ControlLegacyPidConfig altitude_pid;
} APP_ControlConfigV2;

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    APP_ControlConfigV1 config;
    uint32_t checksum;
} APP_ControlFlashRecordV1;

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    APP_ControlConfigV2 config;
    uint32_t checksum;
} APP_ControlFlashRecordV2;

typedef struct {
    float pos_x_kp;
    float pos_y_kp;
    float pos_z_kp;
    float vel_x_kd;
    float vel_y_kd;
    float vel_z_kd;
    float rotation_error_gain;
    float accel_xy_limit_m_s2;
    float accel_z_limit_m_s2;
    float mass_kg;
    float gravity_m_s2;
    float min_total_force_n;
    float max_total_force_n;
    float tilt_lever_arm_m;
    float roll_angle_kp;
    float roll_rate_kd;
    float pitch_angle_kp;
    float pitch_rate_kd;
    float tilt_limit_rad;
    float yaw_angle_kp;
    float yaw_rate_kd;
    float yaw_rate_limit_rad_s;
    float yaw_inertia;
    float thrust_coeff_n_per_rad2;
    float yaw_torque_coeff_n_m_per_rad2;
    float motor_omega_max_rad_s;
} APP_ControlCoaxParamsV4;

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    APP_ControlConfig config;
    APP_ControlCoaxParamsV4 coax_params;
    uint32_t checksum;
} APP_ControlFlashRecordV3;

typedef APP_ControlFlashRecordV3 APP_ControlFlashRecordV4;

typedef struct {
    float pos_x_kp;
    float pos_y_kp;
    float pos_z_kp;
    float vel_x_kd;
    float vel_y_kd;
    float vel_z_kd;
    float rotation_error_gain;
    float accel_xy_limit_m_s2;
    float accel_z_limit_m_s2;
    float vel_loop_enable;
    float vel_loop_x_kp;
    float vel_loop_x_ki;
    float vel_loop_x_kd;
    float vel_loop_y_kp;
    float vel_loop_y_ki;
    float vel_loop_y_kd;
    float vel_loop_output_limit_m_s2;
    float vel_loop_i_limit_m_s2;
    float mass_kg;
    float gravity_m_s2;
    float min_total_force_n;
    float max_total_force_n;
    float tilt_lever_arm_m;
    float roll_angle_kp;
    float roll_rate_kd;
    float pitch_angle_kp;
    float pitch_rate_kd;
    float tilt_limit_rad;
    float yaw_angle_kp;
    float yaw_rate_kd;
    float yaw_rate_limit_rad_s;
    float yaw_inertia;
    float thrust_coeff_n_per_rad2;
    float yaw_torque_coeff_n_m_per_rad2;
    float motor_omega_max_rad_s;
} APP_ControlCoaxParamsV7;

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    APP_ControlConfig config;
    APP_ControlCoaxParamsV7 coax_params;
    uint32_t checksum;
} APP_ControlFlashRecordV5;

typedef APP_ControlFlashRecordV5 APP_ControlFlashRecordV6;
typedef APP_ControlFlashRecordV5 APP_ControlFlashRecordV7;

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    APP_ControlConfig config;
    DRV_COAX_CTRL_Params coax_params;
    uint32_t checksum;
} APP_ControlFlashRecordV8;

_Static_assert(offsetof(DRV_COAX_CTRL_Params, indi_enable) ==
               sizeof(APP_ControlCoaxParamsV7),
               "V7 coax params must remain a prefix of current params");

static APP_ControlConfig control_config;
#if (APP_CONTROL_HEARTBEAT_ENABLED != 0U)
static uint32_t control_last_heartbeat_ms;
static uint8_t control_reported_hw_once;
#endif
static uint8_t control_wifi_reset_pending;
static uint32_t control_wifi_reset_deadline_ms;
static uint8_t control_initialized;
static uint8_t control_maint_output_active;
static uint8_t control_dwt_ready;
static uint8_t control_flash_autosave_pending;
static uint32_t control_flash_autosave_deadline_ms;
static uint8_t ident_active;
static uint8_t ident_motor;
static uint32_t ident_min_percent;
static uint32_t ident_max_percent;
static uint32_t ident_step_percent;
static uint32_t ident_dwell_ms;
static uint32_t ident_current_percent;
static uint32_t ident_next_ms;
static uint32_t ident_seq;
__attribute__((section(".dma_buffer"), aligned(32)))
static uint8_t control_flash_buf_a[APP_CONTROL_FLASH_BENCH_MAX_LEN];
__attribute__((section(".dma_buffer"), aligned(32)))
static uint8_t control_flash_buf_b[APP_CONTROL_FLASH_BENCH_MAX_LEN];

static void app_control_handle_param(char **tokens, uint32_t count);
static void app_control_report_pid_legacy(void);
static uint8_t app_control_handle_pid_slider_line(const char *line);
static void app_control_report_wifi(void);
void APP_Control_QueueText(const char *format, ...);
static void app_control_queue_proto_text(uint16_t function, const char *format, ...);
static void app_control_handle_flight_log(char **tokens, uint32_t count);
static void app_control_dispatch_tokens(char **tokens, uint32_t count, uint8_t emit_ack);
static uint32_t app_control_tokenize(char *buffer, char **tokens, uint32_t max_tokens);
static uint8_t app_control_parse_u32(const char *text, uint32_t *value);
static uint8_t app_control_payload_to_line(const uint8_t *payload,
                                           uint16_t payload_length,
                                           char *buffer,
                                           uint16_t buffer_size);
static void app_control_handle_wifi(char **tokens, uint32_t count);
static void app_control_handle_motor(char **tokens, uint32_t count);
static void app_control_handle_ident(char **tokens, uint32_t count);
static void app_control_ident_step(void);
static void app_control_service_wifi_reset(void);
static void app_control_schedule_flash_autosave(void);
static void app_control_service_flash_autosave(void);
static void app_control_tick_common(uint8_t emit_heartbeat);
static void app_control_report_rtos(void);
static void app_control_handle_flash(char **tokens, uint32_t count);
static void app_control_handle_flow(char **tokens, uint32_t count);
static void app_control_req_m9n(uint32_t id, const char *op);
static void app_control_req_mag(uint32_t id, const char *op);
static const char *app_control_age_text(uint32_t age_ms, char *buffer, uint16_t size);

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

static const char *app_control_age_text(uint32_t age_ms, char *buffer, uint16_t size)
{
    if ((buffer == NULL) || (size == 0U)) {
        return "?";
    }

    if (age_ms == 0xFFFFFFFFUL) {
        (void)snprintf(buffer, size, "none");
    } else {
        (void)snprintf(buffer, size, "%lu", (unsigned long)age_ms);
    }

    return buffer;
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
    uint8_t id_ok;

    if ((status == NULL) || (status->init_status != 0)) {
        return 0U;
    }

    id_ok = ((status->product_id == BSP_SPL06_ID_VALUE) ||
             (status->split_id == BSP_SPL06_ID_VALUE) ||
             (status->txrx_id == BSP_SPL06_ID_VALUE)) ? 1U : 0U;
    return id_ok;
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

    if ((status->product_id != BSP_SPL06_ID_VALUE) &&
        (status->split_id != BSP_SPL06_ID_VALUE) &&
        (status->txrx_id != BSP_SPL06_ID_VALUE)) {
        return "who_id";
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

void APP_Control_QueueText(const char *format, ...)
{
    APP_UART_TxMessage tx_message;
    APP_UART_TxMessage dropped;
    va_list args;
    int written;

    if ((format == NULL) || (uartTxQueueHandle == 0)) {
        return;
    }

    tx_message.function = 0U;
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

static void app_control_handle_flight_log(char **tokens, uint32_t count)
{
    APP_FlightLogStatus status;
    APP_FlightLogCommandStatus cmd_status;

    if ((tokens == NULL) || (count == 0U)) {
        return;
    }

    if (strcmp(tokens[0], "FLOG?") == 0) {
        APP_FlightLog_GetStatus(&status);
        APP_Control_QueueText("FLOG initialized=%u recording=%u export=%u pending=%u "
                              "used_bytes=%lu used_sectors=%lu records=%lu "
                              "dropped=%lu buffered=%lu session=%lu "
                              "export_sent=%lu export_total=%lu flash_status=%lu "
                              "region=0x%06lX..0x%06lX rate=%u baud=%u\r\n",
                              (unsigned int)status.initialized,
                              (unsigned int)status.recording,
                              (unsigned int)status.export_active,
                              (unsigned int)status.export_pending,
                              (unsigned long)status.used_bytes,
                              (unsigned long)status.used_sectors,
                              (unsigned long)status.total_records,
                              (unsigned long)status.dropped_records,
                              (unsigned long)status.buffered_records,
                              (unsigned long)status.session_id,
                              (unsigned long)status.export_bytes_sent,
                              (unsigned long)status.export_total_bytes,
                              (unsigned long)status.last_flash_status,
                              (unsigned long)APP_FLIGHT_LOG_REGION_START,
                              (unsigned long)APP_FLIGHT_LOG_REGION_END_EXCL,
                              (unsigned int)APP_FLIGHT_LOG_RATE_HZ,
                              (unsigned int)APP_FLIGHT_LOG_EXPORT_BAUD);
        return;
    }

    if ((count >= 2U) && (strcmp(tokens[0], "FLOG") == 0) &&
        (strcmp(tokens[1], "DUMP") == 0)) {
        cmd_status = APP_FlightLog_StartDump();
        if (cmd_status != APP_FLIGHT_LOG_CMD_OK) {
            APP_Control_QueueText("FLOG ERROR start %s\r\n",
                                  APP_FlightLog_CommandStatusText(cmd_status));
        }
        return;
    }

    if ((count >= 2U) && (strcmp(tokens[0], "FLOG") == 0) &&
        (strcmp(tokens[1], "CANCEL") == 0)) {
        cmd_status = APP_FlightLog_CancelDump();
        APP_Control_QueueText("FLOG CANCEL %s\r\n",
                              APP_FlightLog_CommandStatusText(cmd_status));
        return;
    }

    if ((count >= 2U) && (strcmp(tokens[0], "FLOG") == 0) &&
        (strcmp(tokens[1], "TESTFILL") == 0)) {
        uint32_t sectors = 15U;

        if ((count >= 3U) && (app_control_parse_u32(tokens[2], &sectors) == 0U)) {
            APP_Control_QueueText("ERR usage FLOG TESTFILL [sectors]\r\n");
            return;
        }

        cmd_status = APP_FlightLog_TestFill(sectors);
        APP_Control_QueueText("FLOG TESTFILL %s sectors=%lu\r\n",
                              APP_FlightLog_CommandStatusText(cmd_status),
                              (unsigned long)sectors);
        return;
    }

    APP_Control_QueueText("ERR usage FLOG? | FLOG DUMP | FLOG CANCEL | FLOG TESTFILL [sectors]\r\n");
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
    case APP_AIWB2_STATE_WAIT_TRANSPARENT_OK:
        return "wait_transparent_ok";
    case APP_AIWB2_STATE_TRANSPARENT:
        return "transparent";
    case APP_AIWB2_STATE_SOCKET_READY:
        return "socket_ready";
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
    config->servo[0].pulse_us = DRV_COAX_CTRL_SERVO_ALPHA_CENTER_US;
    config->servo[0].time_ms = 500U;
    config->servo[0].mode = 1U;
    config->servo[0].enabled = 1U;
    config->servo[1].id = 2U;
    config->servo[1].pulse_us = DRV_COAX_CTRL_SERVO_BETA_CENTER_US;
    config->servo[1].time_ms = 500U;
    config->servo[1].mode = 1U;
    config->servo[1].enabled = 1U;

    DRV_COAX_CTRL_ResetParams();
}

static uint8_t app_control_valid_servo_index(uint32_t index)
{
    return (index < APP_CONTROL_SERVO_COUNT) ? 1U : 0U;
}

static uint8_t app_control_valid_motor(uint32_t motor)
{
    return (motor <= DRV_MOTOR_ID_2) ? 1U : 0U;
}

static uint16_t app_control_servo_angle_to_pulse(uint32_t angle)
{
    if (angle > 180U) { angle = 180U; }
    return (uint16_t)(DRV_COAX_CTRL_SERVO_PHYSICAL_MIN_US +
                      ((angle * (DRV_COAX_CTRL_SERVO_PHYSICAL_MAX_US -
                                 DRV_COAX_CTRL_SERVO_PHYSICAL_MIN_US)) / 180U));
}

static uint16_t app_control_servo_clamp_pulse(uint32_t index, uint16_t pulse_us)
{
    uint16_t min_us = (index == 0U) ? DRV_COAX_CTRL_SERVO_ALPHA_MIN_US
                                    : DRV_COAX_CTRL_SERVO_BETA_MIN_US;
    uint16_t max_us = (index == 0U) ? DRV_COAX_CTRL_SERVO_ALPHA_MAX_US
                                    : DRV_COAX_CTRL_SERVO_BETA_MAX_US;

    if (pulse_us < min_us) {
        return min_us;
    }
    if (pulse_us > max_us) {
        return max_us;
    }
    return pulse_us;
}

static uint8_t app_control_parse_vofa_pwm(const char *text,
                                          uint32_t *channel,
                                          uint32_t *percent)
{
    unsigned int parsed_channel;
    unsigned int parsed_percent;

    if ((text == NULL) || (channel == NULL) || (percent == NULL)) {
        return 0U;
    }

    if (sscanf(text, "PWM%u:%u", &parsed_channel, &parsed_percent) != 2) {
        return 0U;
    }

    *channel = (uint32_t)parsed_channel;
    *percent = (uint32_t)parsed_percent;
    return 1U;
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

static void app_control_report_pwm(void)
{
    uint32_t moder0 = (GPIOA->MODER >> 0U) & 0x3U;
    uint32_t afr0 = (GPIOA->AFR[0] >> 0U) & 0xFU;

    APP_Control_QueueText("PWM tim2 cr1=0x%08lX ccER=0x%08lX ccmr1=0x%08lX ccmr2=0x%08lX psc=%lu arr=%lu cnt=%lu\r\n",
                          (unsigned long)TIM2->CR1,
                          (unsigned long)TIM2->CCER,
                          (unsigned long)TIM2->CCMR1,
                          (unsigned long)TIM2->CCMR2,
                          (unsigned long)TIM2->PSC,
                          (unsigned long)TIM2->ARR,
                          (unsigned long)TIM2->CNT);
    APP_Control_QueueText("PWM ccr=%lu,%lu,%lu,%lu esc_us=%u,%u servo_us=%u,%u start=%u,%u,%u,%u pa0_moder=%lu pa0_af=%lu odr=0x%08lX idr=0x%08lX\r\n",
                          (unsigned long)TIM2->CCR1,
                          (unsigned long)TIM2->CCR2,
                          (unsigned long)TIM2->CCR3,
                          (unsigned long)TIM2->CCR4,
                          (unsigned int)BSP_PWM_GetEscPulse(1U),
                          (unsigned int)BSP_PWM_GetEscPulse(2U),
                          (unsigned int)BSP_PWM_GetServoPulse(1U),
                          (unsigned int)BSP_PWM_GetServoPulse(2U),
                          (unsigned int)BSP_PWM_GetStartStatus(1U),
                          (unsigned int)BSP_PWM_GetStartStatus(2U),
                          (unsigned int)BSP_PWM_GetStartStatus(3U),
                          (unsigned int)BSP_PWM_GetStartStatus(4U),
                          (unsigned long)moder0,
                          (unsigned long)afr0,
                          (unsigned long)GPIOA->ODR,
                          (unsigned long)GPIOA->IDR);
}

static void app_control_report_motor(void)
{
    APP_Control_QueueText("MOTOR both_id=0 m1_pct=%lu m1_pulse=%u m2_pct=%lu m2_pulse=%u\r\n",
                          (unsigned long)DRV_Motor_GetPercent(DRV_MOTOR_ID_1),
                          (unsigned int)DRV_Motor_GetPulse(DRV_MOTOR_ID_1),
                          (unsigned long)DRV_Motor_GetPercent(DRV_MOTOR_ID_2),
                          (unsigned int)DRV_Motor_GetPulse(DRV_MOTOR_ID_2));
}

static void app_control_report_ident(void)
{
    APP_Ident_ReportStatus();
}

static uint32_t app_control_crc32_update(uint32_t crc, const uint8_t *data, uint32_t len)
{
    for (uint32_t i = 0U; i < len; ++i) {
        crc ^= data[i];
        for (uint32_t bit = 0U; bit < 8U; ++bit) {
            crc = (crc & 1U) ? ((crc >> 1U) ^ 0xEDB88320UL) : (crc >> 1U);
        }
    }
    return crc;
}

static uint32_t app_control_crc32(const uint8_t *data, uint32_t len)
{
    return app_control_crc32_update(0xFFFFFFFFUL, data, len) ^ 0xFFFFFFFFUL;
}

static uint8_t app_control_token_u32(char **tokens,
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

    return app_control_parse_u32_auto(tokens[index], value);
}

static uint32_t app_control_time_us(void)
{
    if (control_dwt_ready == 0U) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0U;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
        control_dwt_ready = 1U;
    }

    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0U) {
        uint32_t hz_per_us = SystemCoreClock / 1000000UL;
        if (hz_per_us != 0U) {
            return DWT->CYCCNT / hz_per_us;
        }
    }

    return HAL_GetTick() * 1000UL;
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

static const char *app_control_after_param_separator(const char *text)
{
    if (text == NULL) {
        return NULL;
    }

    if (text[0] == ':') {
        return text + 1U;
    }

    if (((uint8_t)text[0] == 0xEFU) &&
        ((uint8_t)text[1] == 0xBCU) &&
        ((uint8_t)text[2] == 0x9AU)) {
        return text + 3U;
    }

    return NULL;
}

static uint8_t app_control_named_value_line(const char *line,
                                            const char *name,
                                            char *value_text,
                                            uint32_t value_size)
{
    const char *match;
    const char *cursor;
    const char *value_start;
    uint32_t value_len = 0U;

    if ((line == NULL) || (name == NULL) || (value_text == NULL) ||
        (value_size == 0U)) {
        return 0U;
    }

    match = strstr(line, name);
    if (match == NULL) {
        return 0U;
    }

    cursor = match + strlen(name);
    while (*cursor == ' ') {
        ++cursor;
    }

    value_start = app_control_after_param_separator(cursor);
    if (value_start == NULL) {
        return 0U;
    }

    while (*value_start == ' ') {
        ++value_start;
    }

    while ((value_start[value_len] > ' ') &&
           (value_start[value_len] != ',') &&
           (value_len < (value_size - 1U))) {
        value_text[value_len] = value_start[value_len];
        ++value_len;
    }
    value_text[value_len] = '\0';

    return (value_len != 0U) ? 1U : 0U;
}

static void app_control_protocol_err(uint32_t id,
                                     const char *mod,
                                     const char *op,
                                     const char *code)
{
    APP_Control_QueueText("ERR id=%lu mod=%s op=%s code=%s\r\n",
                           (unsigned long)id,
                           (mod != NULL) ? mod : "?",
                           (op != NULL) ? op : "?",
                           (code != NULL) ? code : "ERR");
}

static uint8_t app_control_parse_f32(const char *text, float *value)
{
    char *end_ptr;
    float parsed;

    if ((text == NULL) || (value == NULL) || (*text == '\0')) {
        return 0U;
    }

    parsed = strtof(text, &end_ptr);
    if ((end_ptr == text) || (*end_ptr != '\0') || !isfinite(parsed)) {
        return 0U;
    }

    *value = parsed;
    return 1U;
}

static void app_control_format_float(float value, char *buffer, uint32_t size)
{
    int scaled;

    if ((buffer == NULL) || (size == 0U)) {
        return;
    }

    scaled = (int)((value * 1000000.0f) +
                   ((value >= 0.0f) ? 0.5f : -0.5f));
    (void)snprintf(buffer,
                   size,
                   "%s%d.%06u",
                   (scaled < 0) ? "-" : "",
                   abs(scaled / 1000000),
                   (unsigned int)abs(scaled % 1000000));
}

static void app_control_report_coax_param(const char *name, float value)
{
    char value_text[24];

    app_control_format_float(value, value_text, (uint32_t)sizeof(value_text));
    app_control_queue_proto_text(APP_PROTO_MSG_PARAM_RECORD,
                                 "PARAM name=%s value=%s\r\n",
                                 name,
                                 value_text);
}

static void app_control_report_coax_param_by_name(const char *name)
{
    float value;

    if ((name != NULL) && (DRV_COAX_CTRL_GetParam(name, &value) != 0U)) {
        app_control_report_coax_param(name, value);
    }
}

static void app_control_report_params(void)
{
    float value;

    for (uint32_t index = 0U; index < DRV_COAX_CTRL_ParamCount(); ++index) {
        const char *name = DRV_COAX_CTRL_ParamName(index);
        if ((name != NULL) && (DRV_COAX_CTRL_GetParam(name, &value) != 0U)) {
            app_control_report_coax_param(name, value);
        }
    }
}

static void app_control_report_indi(void)
{
    DRV_COAX_CTRL_INDIDebug debug;
    char roll_accel[24];
    char pitch_accel[24];
    char roll_virtual[24];
    char pitch_virtual[24];
    char roll_effectiveness[24];
    char pitch_effectiveness[24];
    char roll_correction[24];
    char pitch_correction[24];

    DRV_COAX_CTRL_GetLastINDIDebug(&debug);
    app_control_format_float(debug.angular_accel_rad_s2[0], roll_accel, sizeof(roll_accel));
    app_control_format_float(debug.angular_accel_rad_s2[1], pitch_accel, sizeof(pitch_accel));
    app_control_format_float(debug.virtual_accel_rad_s2[0], roll_virtual, sizeof(roll_virtual));
    app_control_format_float(debug.virtual_accel_rad_s2[1], pitch_virtual, sizeof(pitch_virtual));
    app_control_format_float(debug.effectiveness_rad_s2_per_rad[0],
                             roll_effectiveness,
                             sizeof(roll_effectiveness));
    app_control_format_float(debug.effectiveness_rad_s2_per_rad[1],
                             pitch_effectiveness,
                             sizeof(pitch_effectiveness));
    app_control_format_float(debug.correction_rad[0], roll_correction, sizeof(roll_correction));
    app_control_format_float(debug.correction_rad[1], pitch_correction, sizeof(pitch_correction));

    APP_Control_QueueText("INDI active=%u roll_accel=%s pitch_accel=%s "
                          "roll_virtual=%s pitch_virtual=%s "
                          "roll_effectiveness=%s pitch_effectiveness=%s "
                          "roll_correction=%s pitch_correction=%s\r\n",
                          (unsigned int)debug.active,
                          roll_accel,
                          pitch_accel,
                          roll_virtual,
                          pitch_virtual,
                          roll_effectiveness,
                          pitch_effectiveness,
                          roll_correction,
                          pitch_correction);
}

static void app_control_force_airframe_params(DRV_COAX_CTRL_Params *params)
{
    if (params == NULL) {
        return;
    }

    params->mass_kg = DRV_AIRFRAME_MASS_KG;
    params->min_total_force_n = DRV_AIRFRAME_WEIGHT_N;
    params->max_total_force_n = DRV_AIRFRAME_MAX_TOTAL_FORCE_N;
}

static void app_control_apply_new_coax_param_defaults(DRV_COAX_CTRL_Params *params)
{
    DRV_COAX_CTRL_Params defaults;

    if (params == NULL) {
        return;
    }

    DRV_COAX_CTRL_GetDefaultParams(&defaults);
    params->vel_loop_enable = defaults.vel_loop_enable;
    params->vel_loop_x_kp = defaults.vel_loop_x_kp;
    params->vel_loop_x_ki = defaults.vel_loop_x_ki;
    params->vel_loop_x_kd = defaults.vel_loop_x_kd;
    params->vel_loop_y_kp = defaults.vel_loop_y_kp;
    params->vel_loop_y_ki = defaults.vel_loop_y_ki;
    params->vel_loop_y_kd = defaults.vel_loop_y_kd;
    params->vel_loop_output_limit_m_s2 = defaults.vel_loop_output_limit_m_s2;
    params->vel_loop_i_limit_m_s2 = defaults.vel_loop_i_limit_m_s2;
}

static void app_control_apply_v7_safety_defaults(DRV_COAX_CTRL_Params *params)
{
    DRV_COAX_CTRL_Params defaults;

    if (params == NULL) {
        return;
    }

    DRV_COAX_CTRL_GetDefaultParams(&defaults);
    params->tilt_limit_rad = defaults.tilt_limit_rad;
    app_control_force_airframe_params(params);
}

static void app_control_migrate_coax_params_v4(const APP_ControlCoaxParamsV4 *legacy,
                                               DRV_COAX_CTRL_Params *params)
{
    if ((legacy == NULL) || (params == NULL)) {
        return;
    }

    DRV_COAX_CTRL_GetDefaultParams(params);
    params->pos_x_kp = legacy->pos_x_kp;
    params->pos_y_kp = legacy->pos_y_kp;
    params->pos_z_kp = legacy->pos_z_kp;
    params->vel_x_kd = legacy->vel_x_kd;
    params->vel_y_kd = legacy->vel_y_kd;
    params->vel_z_kd = legacy->vel_z_kd;
    params->rotation_error_gain = legacy->rotation_error_gain;
    params->accel_xy_limit_m_s2 = legacy->accel_xy_limit_m_s2;
    params->accel_z_limit_m_s2 = legacy->accel_z_limit_m_s2;
    params->mass_kg = legacy->mass_kg;
    params->gravity_m_s2 = legacy->gravity_m_s2;
    params->min_total_force_n = legacy->min_total_force_n;
    params->max_total_force_n = legacy->max_total_force_n;
    params->tilt_lever_arm_m = legacy->tilt_lever_arm_m;
    params->roll_angle_kp = legacy->roll_angle_kp;
    params->roll_rate_kd = legacy->roll_rate_kd;
    params->pitch_angle_kp = legacy->pitch_angle_kp;
    params->pitch_rate_kd = legacy->pitch_rate_kd;
    params->tilt_limit_rad = legacy->tilt_limit_rad;
    params->yaw_angle_kp = legacy->yaw_angle_kp;
    params->yaw_rate_kd = legacy->yaw_rate_kd;
    params->yaw_rate_limit_rad_s = legacy->yaw_rate_limit_rad_s;
    params->yaw_inertia = legacy->yaw_inertia;
    params->thrust_coeff_n_per_rad2 = legacy->thrust_coeff_n_per_rad2;
    params->yaw_torque_coeff_n_m_per_rad2 = legacy->yaw_torque_coeff_n_m_per_rad2;
    params->motor_omega_max_rad_s = legacy->motor_omega_max_rad_s;
    app_control_force_airframe_params(params);
    app_control_apply_new_coax_param_defaults(params);
}

static void app_control_migrate_coax_params_v7(const APP_ControlCoaxParamsV7 *legacy,
                                               DRV_COAX_CTRL_Params *params)
{
    if ((legacy == NULL) || (params == NULL)) {
        return;
    }

    DRV_COAX_CTRL_GetDefaultParams(params);
    memcpy(params, legacy, sizeof(*legacy));
}

static void app_control_report_airframe(void)
{
    char mass_kg[24];
    char cg_z_m[24];
    char imu_z_m[24];
    char attach_z_m[24];
    char attach_to_cg_m[24];
    char rope_m[24];
    char rod_to_cg_m[24];
    char servo_deg_per_us[24];
    char servo_us_per_deg[24];
    char max_force_n[24];
    char hover_pct[24];

    app_control_format_float(DRV_AIRFRAME_MASS_KG, mass_kg, (uint32_t)sizeof(mass_kg));
    app_control_format_float(DRV_AIRFRAME_CG_Z_M, cg_z_m, (uint32_t)sizeof(cg_z_m));
    app_control_format_float(DRV_AIRFRAME_IMU_Z_M, imu_z_m, (uint32_t)sizeof(imu_z_m));
    app_control_format_float(DRV_AIRFRAME_TETHER_ATTACH_Z_M, attach_z_m, (uint32_t)sizeof(attach_z_m));
    app_control_format_float(DRV_AIRFRAME_TETHER_ATTACH_TO_CG_M, attach_to_cg_m, (uint32_t)sizeof(attach_to_cg_m));
    app_control_format_float(DRV_AIRFRAME_TETHER_ROPE_M, rope_m, (uint32_t)sizeof(rope_m));
    app_control_format_float(DRV_AIRFRAME_TETHER_ROD_TO_CG_M, rod_to_cg_m, (uint32_t)sizeof(rod_to_cg_m));
    app_control_format_float(DRV_AIRFRAME_SERVO_DEG_PER_US, servo_deg_per_us, (uint32_t)sizeof(servo_deg_per_us));
    app_control_format_float(DRV_AIRFRAME_SERVO_US_PER_DEG, servo_us_per_deg, (uint32_t)sizeof(servo_us_per_deg));
    app_control_format_float(DRV_AIRFRAME_MAX_TOTAL_FORCE_N, max_force_n, (uint32_t)sizeof(max_force_n));
    app_control_format_float(DRV_AIRFRAME_HOVER_THRUST_PERCENT, hover_pct, (uint32_t)sizeof(hover_pct));

    app_control_queue_proto_text(APP_PROTO_MSG_AIRFRAME_RECORD,
                                 "AIRFRAME mass_kg=%s cg_z_m=%s imu_z_m=%s tether_attach_z_m=%s tether_attach_to_cg_m=%s rope_m=%s rod_to_cg_m=%s servo_deg_per_us=%s servo_us_per_deg=%s thrust_scope=%s max_total_force_n=%s hover_thrust_pct=%s\r\n",
                                 mass_kg,
                                 cg_z_m,
                                 imu_z_m,
                                 attach_z_m,
                                 attach_to_cg_m,
                                 rope_m,
                                 rod_to_cg_m,
                                 servo_deg_per_us,
                                 servo_us_per_deg,
                                 DRV_AIRFRAME_THRUST_TABLE_SCOPE,
                                 max_force_n,
                                 hover_pct);
}

static void app_control_report_caps(void)
{
    app_control_queue_proto_text(APP_PROTO_MSG_CAPS_RECORD,
                                 "RSP id=0 mod=CAPS op=LIST proto=mspv2-lite-v1 resp=frame+typed req=frame+typed\r\n");
    app_control_queue_proto_text(APP_PROTO_MSG_CAPS_RECORD,
                                 "RSP id=0 mod=CAPS op=LIST legacy=PING,STATUS?,CONFIG?,SAVE,LOAD,SERVO raw=custom-tab\r\n");
    app_control_queue_proto_text(APP_PROTO_MSG_CAPS_RECORD,
                                 "RSP id=0 mod=CAPS op=LIST mods=MODULES,SPL06,ICM42688,FLOW,MAG,PARAM,FLASH,RTOS,WIFI\r\n");
    app_control_queue_proto_text(APP_PROTO_MSG_CAPS_RECORD,
                                 "RSP id=0 mod=CAPS op=LIST ops=SPL06:STATUS,READ,SAMPLE ICM42688:STATUS,DIAG FLOW:STATUS MAG:STATUS,DIAG\r\n");
    app_control_queue_proto_text(APP_PROTO_MSG_CAPS_RECORD,
                                 "RSP id=0 mod=CAPS op=LIST ops=WIFI:STATUS,EN,RESET legacy=WIFI?,WIFI_EN?\r\n");
    app_control_queue_proto_text(APP_PROTO_MSG_CAPS_RECORD,
                                 "RSP id=0 mod=CAPS op=LIST ops=FLASH:VERIFY,BENCH_READ,SCRATCH RTOS:STATUS legacy=RTOS?\r\n");
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

    APP_Flash_RefreshStatus();
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

static void app_control_report_task_stack(const char *name, osThreadId_t handle)
{
    if ((name == NULL) || (handle == NULL)) {
        return;
    }

    app_control_queue_proto_text(APP_PROTO_MSG_RTOS_RECORD,
                                 "RTOS task=%s free_stack_words=%lu\r\n",
                                 name,
                                 (unsigned long)uxTaskGetStackHighWaterMark((TaskHandle_t)handle));
}

static void app_control_report_rtos(void)
{
    APP_DiagFaultInfo faults;

    APP_Diag_GetFaultInfo(&faults);
    app_control_queue_proto_text(APP_PROTO_MSG_RTOS_RECORD,
                                 "RTOS heap_free=%lu heap_min=%lu q_uart=%lu/%lu q_background_req=%lu/%lu q_background_resp=%lu/%lu fault_stack=%u fault_task=%s fault_malloc=%u malloc_count=%lu\r\n",
                                 (unsigned long)xPortGetFreeHeapSize(),
                                 (unsigned long)xPortGetMinimumEverFreeHeapSize(),
                                 (unsigned long)((uartTxQueueHandle != NULL) ? osMessageQueueGetCount(uartTxQueueHandle) : 0U),
                                 (unsigned long)((uartTxQueueHandle != NULL) ? osMessageQueueGetCapacity(uartTxQueueHandle) : 0U),
                                 (unsigned long)((backgroundReqQueueHandle != NULL) ? osMessageQueueGetCount(backgroundReqQueueHandle) : 0U),
                                 (unsigned long)((backgroundReqQueueHandle != NULL) ? osMessageQueueGetCapacity(backgroundReqQueueHandle) : 0U),
                                 (unsigned long)((backgroundRespQueueHandle != NULL) ? osMessageQueueGetCount(backgroundRespQueueHandle) : 0U),
                                 (unsigned long)((backgroundRespQueueHandle != NULL) ? osMessageQueueGetCapacity(backgroundRespQueueHandle) : 0U),
                                 (unsigned int)faults.stack_overflow_seen,
                                 (faults.stack_overflow_task[0] != '\0') ? faults.stack_overflow_task : "-",
                                 (unsigned int)faults.malloc_failed_seen,
                                 (unsigned long)faults.malloc_failed_count);

    app_control_report_task_stack("SENSOR", SensorTaskHandle);
    app_control_report_task_stack("MSG", messageTaskHandle);
    app_control_report_task_stack("UART", UARTTaskHandle);
    app_control_report_task_stack("BACKGROUND", backgroundTaskHandle);
}

static void app_control_flash_verify(char **tokens, uint32_t count)
{
    uint32_t address;
    uint32_t length;
    uint32_t crc_blocking;
    uint32_t crc_dma;
    APP_FlashService_Status st_blocking;
    APP_FlashService_Status st_dma;
    int cmp = 0;

    if (!app_control_token_u32(tokens, count, 2U, APP_CONTROL_FLASH_BENCH_DEFAULT_ADDR, &address) ||
        !app_control_token_u32(tokens, count, 3U, APP_CONTROL_FLASH_BENCH_DEFAULT_LEN, &length)) {
        APP_Control_QueueText("ERR usage FLASH VERIFY [addr] [len]\r\n");
        return;
    }

    if ((length == 0U) || (length > APP_CONTROL_FLASH_BENCH_MAX_LEN) ||
        (address >= APP_FLASH_SERVICE_SIZE_BYTES) ||
        (length > (APP_FLASH_SERVICE_SIZE_BYTES - address))) {
        APP_Control_QueueText("ERR flash verify range addr=0x%06lX len=%lu max=%lu\r\n",
                               (unsigned long)address,
                               (unsigned long)length,
                               (unsigned long)APP_CONTROL_FLASH_BENCH_MAX_LEN);
        return;
    }

    st_blocking = APP_FlashService_ReadData(address, control_flash_buf_a, length);
    st_dma = APP_FlashService_ReadDataFast(address, control_flash_buf_b, length);
    crc_blocking = app_control_crc32(control_flash_buf_a, length);
    crc_dma = app_control_crc32(control_flash_buf_b, length);
    if ((st_blocking == APP_FLASH_SERVICE_OK) && (st_dma == APP_FLASH_SERVICE_OK)) {
        cmp = memcmp(control_flash_buf_a, control_flash_buf_b, length);
    }

    app_control_queue_proto_text(APP_PROTO_MSG_FLASH_BENCH,
                                 "FLASH verify addr=0x%06lX len=%lu st_block=%u st_dma=%u crc_block=0x%08lX crc_dma=0x%08lX match=%u\r\n",
                                 (unsigned long)address,
                                 (unsigned long)length,
                                 (unsigned int)st_blocking,
                                 (unsigned int)st_dma,
                                 (unsigned long)crc_blocking,
                                 (unsigned long)crc_dma,
                                 (unsigned int)((cmp == 0) &&
                                                (st_blocking == APP_FLASH_SERVICE_OK) &&
                                                (st_dma == APP_FLASH_SERVICE_OK)));
}

static void app_control_flash_bench_read(char **tokens, uint32_t count)
{
    uint32_t address;
    uint32_t length;
    uint32_t loops;
    uint32_t mode;
    uint32_t start_us;
    uint32_t elapsed_us;
    uint32_t crc = 0xFFFFFFFFUL;
    APP_FlashService_Status status = APP_FLASH_SERVICE_OK;
    uint32_t ok_loops = 0U;
    uint64_t total_bytes;
    uint32_t bps;

    if (!app_control_token_u32(tokens, count, 3U, APP_CONTROL_FLASH_BENCH_DEFAULT_ADDR, &address) ||
        !app_control_token_u32(tokens, count, 4U, APP_CONTROL_FLASH_BENCH_DEFAULT_LEN, &length) ||
        !app_control_token_u32(tokens, count, 5U, APP_CONTROL_FLASH_BENCH_DEFAULT_LOOPS, &loops) ||
        !app_control_token_u32(tokens, count, 6U, 1U, &mode)) {
        APP_Control_QueueText("ERR usage FLASH BENCH READ [addr] [len] [loops] [mode 0=blocking 1=dma]\r\n");
        return;
    }

    if ((length == 0U) || (length > APP_CONTROL_FLASH_BENCH_MAX_LEN) ||
        (loops == 0U) ||
        (address >= APP_FLASH_SERVICE_SIZE_BYTES) ||
        (length > (APP_FLASH_SERVICE_SIZE_BYTES - address))) {
        APP_Control_QueueText("ERR flash bench range addr=0x%06lX len=%lu loops=%lu max=%lu\r\n",
                               (unsigned long)address,
                               (unsigned long)length,
                               (unsigned long)loops,
                               (unsigned long)APP_CONTROL_FLASH_BENCH_MAX_LEN);
        return;
    }

    start_us = app_control_time_us();
    for (uint32_t i = 0U; i < loops; ++i) {
        if (mode == 0U) {
            status = APP_FlashService_ReadData(address, control_flash_buf_a, length);
        } else {
            status = APP_FlashService_ReadDataFast(address, control_flash_buf_a, length);
        }
        if (status != APP_FLASH_SERVICE_OK) {
            break;
        }
        crc = app_control_crc32_update(crc, control_flash_buf_a, length);
        ok_loops++;
    }
    elapsed_us = app_control_time_us() - start_us;
    crc ^= 0xFFFFFFFFUL;
    total_bytes = (uint64_t)ok_loops * (uint64_t)length;
    bps = (elapsed_us != 0U) ?
          (uint32_t)((total_bytes * 1000000ULL) / (uint64_t)elapsed_us) : 0U;

    app_control_queue_proto_text(APP_PROTO_MSG_FLASH_BENCH,
                                 "FLASH bench_read mode=%s addr=0x%06lX len=%lu loops=%lu ok=%lu st=%u bytes=%lu time_us=%lu bps=%lu crc=0x%08lX\r\n",
                                 (mode == 0U) ? "block" : "dma",
                                 (unsigned long)address,
                                 (unsigned long)length,
                                 (unsigned long)loops,
                                 (unsigned long)ok_loops,
                                 (unsigned int)status,
                                 (unsigned long)((total_bytes > 0xFFFFFFFFULL) ?
                                     0xFFFFFFFFUL : (uint32_t)total_bytes),
                                 (unsigned long)elapsed_us,
                                 (unsigned long)bps,
                                 (unsigned long)crc);
}

static void app_control_flash_wren_probe(void)
{
    uint8_t before = 0U;
    uint8_t after = 0U;
    APP_FlashService_Status status =
        APP_FlashService_WriteEnableProbe(&before, &after);

    app_control_queue_proto_text(APP_PROTO_MSG_FLASH_BENCH,
                                 "FLASH wren st=%u before=0x%02X after=0x%02X wel=%u busy=%u\r\n",
                                 (unsigned int)status,
                                 (unsigned int)before,
                                 (unsigned int)after,
                                 (unsigned int)((after & 0x02U) != 0U),
                                 (unsigned int)((after & 0x01U) != 0U));
}

static void app_control_flash_status_regs(void)
{
    uint8_t sr1 = 0U;
    uint8_t sr2 = 0U;
    uint8_t sr3 = 0U;
    APP_FlashService_Status st1 = APP_FlashService_ReadStatus1(&sr1);
    APP_FlashService_Status st2 = APP_FlashService_ReadStatus2(&sr2);
    APP_FlashService_Status st3 = APP_FlashService_ReadStatus3(&sr3);

    app_control_queue_proto_text(APP_PROTO_MSG_FLASH_BENCH,
                                 "FLASH sr st=%u/%u/%u sr1=0x%02X sr2=0x%02X sr3=0x%02X wip=%u wel=%u bp=0x%02X cmp=%u qe=%u srp=%u%u\r\n",
                                 (unsigned int)st1,
                                 (unsigned int)st2,
                                 (unsigned int)st3,
                                 (unsigned int)sr1,
                                 (unsigned int)sr2,
                                 (unsigned int)sr3,
                                 (unsigned int)((sr1 & 0x01U) != 0U),
                                 (unsigned int)((sr1 & 0x02U) != 0U),
                                 (unsigned int)((sr1 >> 2U) & 0x1FU),
                                 (unsigned int)((sr2 & 0x40U) != 0U),
                                 (unsigned int)((sr2 & 0x02U) != 0U),
                                 (unsigned int)((sr2 & 0x01U) != 0U),
                                 (unsigned int)((sr1 & 0x80U) != 0U));
}

static void app_control_flash_unprotect(void)
{
    uint8_t sr1_before = 0U;
    uint8_t sr2_before = 0U;
    uint8_t sr1_after = 0U;
    uint8_t sr2_after = 0U;
    APP_FlashService_Status status =
        APP_FlashService_ClearProtection(&sr1_before,
                                         &sr2_before,
                                         &sr1_after,
                                         &sr2_after);

    app_control_queue_proto_text(APP_PROTO_MSG_FLASH_BENCH,
                                 "FLASH unprotect st=%u sr1_before=0x%02X sr2_before=0x%02X sr1_after=0x%02X sr2_after=0x%02X bp_after=0x%02X cmp_after=%u\r\n",
                                 (unsigned int)status,
                                 (unsigned int)sr1_before,
                                 (unsigned int)sr2_before,
                                 (unsigned int)sr1_after,
                                 (unsigned int)sr2_after,
                                 (unsigned int)((sr1_after >> 2U) & 0x1FU),
                                 (unsigned int)((sr2_after & 0x40U) != 0U));
}

static void app_control_flash_scratch_test(char **tokens, uint32_t count)
{
    const uint32_t length = APP_FLASH_SERVICE_PAGE_SIZE;
    uint32_t address;
    uint32_t erase_kb;
    APP_FlashService_Status erase_status;
    APP_FlashService_Status write_status = APP_FLASH_SERVICE_ERROR;
    APP_FlashService_Status read_status;
    APP_FlashService_Status sr_status;
    uint8_t status1 = 0U;
    uint32_t ff_count = 0U;
    uint32_t crc = 0U;
    int match = 0;

    if (!app_control_token_u32(tokens,
                               count,
                               3U,
                               APP_CONTROL_FLASH_SCRATCH_ADDR,
                               &address) ||
        !app_control_token_u32(tokens, count, 4U, 4U, &erase_kb) ||
        (address > (APP_FLASH_SERVICE_SIZE_BYTES - APP_FLASH_SERVICE_SECTOR_SIZE))) {
        APP_Control_QueueText("ERR usage FLASH SCRATCH TEST [addr] [erase_kb 4|32|64]\r\n");
        return;
    }
    if (erase_kb == 64U) {
        if (((address % APP_FLASH_SERVICE_BLOCK64K_SIZE) != 0U) ||
            (address > (APP_FLASH_SERVICE_SIZE_BYTES - APP_FLASH_SERVICE_BLOCK64K_SIZE))) {
            APP_Control_QueueText("ERR flash scratch addr align for 64K erase\r\n");
            return;
        }
        erase_status = APP_FlashService_EraseBlock64K(address);
    } else if (erase_kb == 32U) {
        if (((address % APP_FLASH_SERVICE_BLOCK32K_SIZE) != 0U) ||
            (address > (APP_FLASH_SERVICE_SIZE_BYTES - APP_FLASH_SERVICE_BLOCK32K_SIZE))) {
            APP_Control_QueueText("ERR flash scratch addr align for 32K erase\r\n");
            return;
        }
        erase_status = APP_FlashService_EraseBlock32K(address);
    } else if (erase_kb == 4U) {
        if ((address % APP_FLASH_SERVICE_SECTOR_SIZE) != 0U) {
            APP_Control_QueueText("ERR flash scratch addr align for 4K erase\r\n");
            return;
        }
        erase_status = APP_FlashService_EraseSector(address);
    } else {
        APP_Control_QueueText("ERR usage FLASH SCRATCH TEST [addr] [erase_kb 4|32|64]\r\n");
        return;
    }

    read_status = APP_FlashService_ReadData(address, control_flash_buf_a, length);
    if (read_status == APP_FLASH_SERVICE_OK) {
        for (uint32_t index = 0U; index < length; ++index) {
            if (control_flash_buf_a[index] == 0xFFU) {
                ++ff_count;
            }
        }
    }

    if ((erase_status == APP_FLASH_SERVICE_OK) &&
        (read_status == APP_FLASH_SERVICE_OK) &&
        (ff_count == length)) {
        for (uint32_t index = 0U; index < length; ++index) {
            control_flash_buf_b[index] =
                (uint8_t)(0xA5U ^ (uint8_t)index ^ (uint8_t)(index >> 3U));
        }
        write_status = APP_FlashService_WriteData(address,
                                                  control_flash_buf_b,
                                                  length);
        read_status = APP_FlashService_ReadData(address,
                                                control_flash_buf_a,
                                                length);
        if ((write_status == APP_FLASH_SERVICE_OK) &&
            (read_status == APP_FLASH_SERVICE_OK)) {
            match = memcmp(control_flash_buf_a, control_flash_buf_b, length);
            crc = app_control_crc32(control_flash_buf_a, length);
        }
    }

    sr_status = APP_FlashService_ReadStatus1(&status1);
    app_control_queue_proto_text(APP_PROTO_MSG_FLASH_BENCH,
                                 "FLASH scratch_test addr=0x%06lX len=%lu erase_kb=%lu erase_st=%u read_st=%u ff=%lu write_st=%u match=%u crc=0x%08lX sr_st=%u sr1=0x%02X\r\n",
                                 (unsigned long)address,
                                 (unsigned long)length,
                                 (unsigned long)erase_kb,
                                 (unsigned int)erase_status,
                                 (unsigned int)read_status,
                                 (unsigned long)ff_count,
                                 (unsigned int)write_status,
                                 (unsigned int)((match == 0) &&
                                                (write_status == APP_FLASH_SERVICE_OK) &&
                                                (read_status == APP_FLASH_SERVICE_OK)),
                                 (unsigned long)crc,
                                 (unsigned int)sr_status,
                                 (unsigned int)status1);
}

static void app_control_handle_flash(char **tokens, uint32_t count)
{
    if (count < 2U) {
        app_control_report_flash();
        return;
    }

    if (strcmp(tokens[1], "VERIFY") == 0) {
        app_control_flash_verify(tokens, count);
    } else if ((strcmp(tokens[1], "BENCH") == 0) &&
               (count >= 3U) &&
               (strcmp(tokens[2], "READ") == 0)) {
        app_control_flash_bench_read(tokens, count);
    } else if ((strcmp(tokens[1], "SR?") == 0) ||
               (strcmp(tokens[1], "STATUS?") == 0)) {
        app_control_flash_status_regs();
    } else if ((strcmp(tokens[1], "WREN?") == 0) ||
               (strcmp(tokens[1], "WEL?") == 0)) {
        app_control_flash_wren_probe();
    } else if ((strcmp(tokens[1], "UNPROTECT") == 0) ||
               (strcmp(tokens[1], "UNLOCK") == 0)) {
        app_control_flash_unprotect();
    } else if ((strcmp(tokens[1], "SCRATCH") == 0) &&
               (count >= 3U) &&
               (strcmp(tokens[2], "TEST") == 0)) {
        app_control_flash_scratch_test(tokens, count);
    } else if (strcmp(tokens[1], "SCRATCH?") == 0) {
        app_control_queue_proto_text(APP_PROTO_MSG_FLASH_BENCH,
                                     "FLASH scratch addr=0x%06lX size=%lu note=reserved_test_sector\r\n",
                                     (unsigned long)APP_CONTROL_FLASH_SCRATCH_ADDR,
                                     (unsigned long)4096UL);
    } else {
        APP_Control_QueueText("ERR usage FLASH VERIFY|BENCH READ|SR?|WREN?|UNPROTECT|SCRATCH?|SCRATCH TEST\r\n");
    }
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
    APP_OPTICAL_FLOW_Status flow_status;
    APP_MAG_Status mag_status;

    APP_Flash_GetStatus(&flash_status);
    APP_Baro_GetStatus(&baro_status);
    APP_IMU_GetStatus(&imu_status);
    APP_OpticalFlow_GetStatus(&flow_status);
    APP_MAG_GetStatus(&mag_status);

    app_control_queue_proto_text(APP_PROTO_MSG_MODULES_SUMMARY,
                                 "RSP id=0 mod=MODULES op=STATUS flash=%u flash_stage=%s baro=%u baro_stage=%s imu=%u flow=%u mag=%u\r\n",
                                 (unsigned int)app_control_flash_ok(&flash_status),
                                 app_control_flash_stage(&flash_status),
                                 (unsigned int)app_control_baro_ok(&baro_status),
                                 app_control_baro_stage(&baro_status),
                                 (unsigned int)imu_status.initialized,
                                 (unsigned int)flow_status.initialized,
                                 (unsigned int)mag_status.initialized);
    app_control_queue_proto_text(APP_PROTO_MSG_MODULES_SUMMARY,
                                 "RSP id=0 mod=MODULES op=STATUS imu_stage=%s mag_type=%s cfg_valid=%u cfg_loaded=%u servo_slots=%u wifi_en=%u\r\n",
                                 app_control_imu_stage_name(imu_status.init_stage),
                                 APP_MAG_GetTypeName(mag_status.type),
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
        APP_Control_QueueText("RSP id=%lu mod=SPL06 op=STATUS ok=%u stage=%s init=%ld raw=%ld who=0x%02X exp=0x10\r\n",
                               (unsigned long)id,
                               (unsigned int)app_control_baro_ok(&snapshot.status),
                               app_control_baro_stage(&snapshot.status),
                               (long)snapshot.status.init_status,
                               (long)snapshot.raw_status,
                               (unsigned int)snapshot.id);
        APP_Control_QueueText("RSP id=%lu mod=SPL06 op=STATUS split=%ld txrx=%ld sid=0x%02X tid=0x%02X cs=%u miso=%u\r\n",
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
        APP_Control_QueueText("RSP id=%lu mod=SPL06 op=%s ok=%u raw_st=%ld coef_st=%ld scaled=%u press_raw=%ld temp_raw=%ld pressure_pa=%ld temp_cdeg=%ld\r\n",
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
        APP_Control_QueueText("RSP id=%lu mod=SPL06 op=%s cfg prs=0x%02X tmp=0x%02X meas=0x%02X cfg=0x%02X int=0x%02X fifo=0x%02X\r\n",
                               (unsigned long)id,
                               op,
                               (unsigned int)snapshot.prs_cfg,
                               (unsigned int)snapshot.tmp_cfg,
                               (unsigned int)snapshot.meas_cfg,
                               (unsigned int)snapshot.cfg_reg,
                               (unsigned int)snapshot.int_sts,
                               (unsigned int)snapshot.fifo_sts);
        APP_Control_QueueText("RSP id=%lu mod=SPL06 op=%s regs=%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X\r\n",
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
        APP_Control_QueueText("RSP id=%lu mod=ICM42688 op=%s ok=%u stage=%s stage_id=%u who=0x%02X exp=0x%02X code=%ld\r\n",
                               (unsigned long)id,
                               op,
                               (unsigned int)imu_status.initialized,
                               app_control_imu_stage_name(imu_status.init_stage),
                               (unsigned int)imu_status.init_stage,
                               (unsigned int)imu_status.who_am_i,
                               (unsigned int)BSP_ICM42688_WHO_AM_I_VALUE,
                               (long)imu_status.last_error);
        APP_Control_QueueText("RSP id=%lu mod=ICM42688 op=%s st=%ld n=%lu ax=%d ay=%d az=%d gx=%ld gy=%ld gz=%ld t=%d\r\n",
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
        APP_Control_QueueText("RSP id=%lu mod=ICM42688 op=%s diag valid=%u m0_tok=0x%02X m0_msb=0x%02X m0_b0=0x%02X m3_tok=0x%02X m3_msb=0x%02X m3_b0=0x%02X best_mode=%u best_hdr=%u\r\n",
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
        APP_Control_QueueText("RSP id=%lu mod=ICM42688 op=%s burst m0_b0=%02X%02X%02X%02X m3_tok=%02X%02X%02X%02X\r\n",
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

static void app_control_req_m9n(uint32_t id, const char *op)
{
    APP_GPS_Status gps_status;
    uint32_t now_ms = HAL_GetTick();
    uint32_t age_ms = 0U;
    char age_text[16];

    if (op == NULL) {
        app_control_protocol_err(id, "M9N", "?", "NO_OP");
        return;
    }

    APP_GPS_GetStatus(&gps_status);
    if (gps_status.last_rx_ms != 0U) {
        age_ms = now_ms - gps_status.last_rx_ms;
    } else {
        age_ms = 0xFFFFFFFFUL;
    }

    if ((strcmp(op, "STATUS") == 0) || (strcmp(op, "DIAG") == 0)) {
        APP_Control_QueueText("RSP id=%lu mod=M9N op=%s ok=%u init=%ld fix=%u valid=%u sv=%u age_ms=%s packets=%lu nav=%lu nmea=%lu gga=%lu\r\n",
                               (unsigned long)id,
                               op,
                               (unsigned int)gps_status.initialized,
                               (long)gps_status.init_status,
                               (unsigned int)gps_status.fix_type,
                               (unsigned int)gps_status.valid_fix,
                               (unsigned int)gps_status.num_sv,
                               app_control_age_text(age_ms, age_text, (uint16_t)sizeof(age_text)),
                               (unsigned long)gps_status.packets,
                               (unsigned long)gps_status.nav_pvt_packets,
                               (unsigned long)gps_status.nmea_sentences,
                               (unsigned long)gps_status.nmea_gga_sentences);
        APP_Control_QueueText("RSP id=%lu mod=M9N op=%s baud=%lu bytes=%lu cksum=%lu nmea_ck=%lu ovf=%lu nmea_ovf=%lu rst=%lu uerr=%lu last_err=0x%lX cfg=%lu\r\n",
                               (unsigned long)id,
                               op,
                               (unsigned long)gps_status.baud_rate,
                               (unsigned long)gps_status.bytes,
                               (unsigned long)gps_status.checksum_errors,
                               (unsigned long)gps_status.nmea_checksum_errors,
                               (unsigned long)gps_status.payload_overflows,
                               (unsigned long)gps_status.nmea_overflows,
                               (unsigned long)gps_status.rx_restarts,
                               (unsigned long)gps_status.uart_errors,
                               (unsigned long)gps_status.last_uart_error,
                               (unsigned long)gps_status.config_writes);
        APP_Control_QueueText("RSP id=%lu mod=M9N op=%s lon=%ld lat=%ld hmsl_mm=%ld hacc_mm=%lu vacc_mm=%lu vn=%ld ve=%ld vd=%ld head_e5=%ld utc=%04u-%02u-%02uT%02u:%02u:%02u\r\n",
                               (unsigned long)id,
                               op,
                               (long)gps_status.lon_deg_e7,
                               (long)gps_status.lat_deg_e7,
                               (long)gps_status.hmsl_mm,
                               (unsigned long)gps_status.hacc_mm,
                               (unsigned long)gps_status.vacc_mm,
                               (long)gps_status.vel_n_mm_s,
                               (long)gps_status.vel_e_mm_s,
                               (long)gps_status.vel_d_mm_s,
                               (long)gps_status.heading_motion_deg_e5,
                               (unsigned int)gps_status.year,
                               (unsigned int)gps_status.month,
                               (unsigned int)gps_status.day,
                               (unsigned int)gps_status.hour,
                               (unsigned int)gps_status.minute,
                               (unsigned int)gps_status.second);
        return;
    }

    app_control_protocol_err(id, "M9N", op, "BAD_OP");
}

static void app_control_req_mag(uint32_t id, const char *op)
{
    APP_MAG_Status mag_status;

    if (op == NULL) {
        app_control_protocol_err(id, "MAG", "?", "NO_OP");
        return;
    }

    APP_MAG_GetStatus(&mag_status);

    if ((strcmp(op, "STATUS") == 0) || (strcmp(op, "DIAG") == 0)) {
        APP_Control_QueueText("RSP id=%lu mod=MAG op=%s ok=%u init=%ld st=%ld type=%s addr=0x%02X who=0x%02X n=%lu raw=%d,%d,%d mgauss=%ld,%ld,%ld\r\n",
                               (unsigned long)id,
                               op,
                               (unsigned int)mag_status.initialized,
                               (long)mag_status.init_status,
                               (long)mag_status.last_status,
                               APP_MAG_GetTypeName(mag_status.type),
                               (unsigned int)mag_status.address,
                               (unsigned int)mag_status.who_am_i,
                               (unsigned long)mag_status.sample_count,
                               (int)mag_status.raw_x,
                               (int)mag_status.raw_y,
                               (int)mag_status.raw_z,
                               (long)mag_status.x_mgauss,
                               (long)mag_status.y_mgauss,
                               (long)mag_status.z_mgauss);
        APP_Control_QueueText("RSP id=%lu mod=MAG op=%s probe ist=%u hmc=%u qmc=%u hmc_id=%02X%02X%02X\r\n",
                               (unsigned long)id,
                               op,
                               (unsigned int)mag_status.detected_ist8310,
                               (unsigned int)mag_status.detected_hmc5883,
                               (unsigned int)mag_status.detected_qmc5883,
                               (unsigned int)mag_status.hmc_id_a,
                               (unsigned int)mag_status.hmc_id_b,
                               (unsigned int)mag_status.hmc_id_c);
        return;
    }

    app_control_protocol_err(id, "MAG", op, "BAD_OP");
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

    if (strcmp(mod, "M9N") == 0) {
        app_control_req_m9n(id, op);
        return;
    }

    if (strcmp(mod, "MAG") == 0) {
        app_control_req_mag(id, op);
        return;
    }

    if (strcmp(mod, "WIFI") == 0) {
        if (op == NULL) {
            app_control_protocol_err(id, "WIFI", "?", "NO_OP");
            return;
        }
        if (strcmp(op, "STATUS") == 0) {
            APP_Control_QueueText("RSP id=%lu mod=WIFI op=STATUS en=%u pin=PC6 last=%u writes=%lu state=%s transparent=%u retry=%lu socket=%ld cycling=%u wait_ms=%lu prov=%u cmd=%lu/%lu\r\n",
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
    APP_OPTICAL_FLOW_Status flow_status;
    APP_MAG_Status mag_status;
    DRV_NAV_EKF_Diagnostics ekf_diag;
    int32_t ekf_nis_milli;
    int32_t ekf_gate_milli;
    int32_t ekf_innov_x_mm_s;
    int32_t ekf_innov_y_mm_s;
    int32_t ekf_noise_mm_s;
    int32_t ekf_vx_mm_s;
    int32_t ekf_vy_mm_s;
    int32_t ekf_bias_x_mm_s2;
    int32_t ekf_bias_y_mm_s2;
    int32_t ekf_p0_u;
    int32_t ekf_p1_u;
    int32_t ekf_p2_u;
    int32_t ekf_p3_u;
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
    APP_OpticalFlow_GetStatus(&flow_status);
    APP_MAG_GetStatus(&mag_status);
    APP_NavEstimator_GetVelocityEKF(&ekf_diag);
    ekf_nis_milli = (int32_t)(ekf_diag.last_nis * 1000.0f);
    ekf_gate_milli = (int32_t)(ekf_diag.last_gate_nis * 1000.0f);
    ekf_innov_x_mm_s = (int32_t)(ekf_diag.last_innovation_m_s[0] * 1000.0f);
    ekf_innov_y_mm_s = (int32_t)(ekf_diag.last_innovation_m_s[1] * 1000.0f);
    ekf_noise_mm_s = (int32_t)(ekf_diag.last_flow_noise_m_s * 1000.0f);
    ekf_vx_mm_s = (int32_t)(ekf_diag.vel_m_s[0] * 1000.0f);
    ekf_vy_mm_s = (int32_t)(ekf_diag.vel_m_s[1] * 1000.0f);
    ekf_bias_x_mm_s2 = (int32_t)(ekf_diag.accel_bias_m_s2[0] * 1000.0f);
    ekf_bias_y_mm_s2 = (int32_t)(ekf_diag.accel_bias_m_s2[1] * 1000.0f);
    ekf_p0_u = (int32_t)(ekf_diag.covariance_diag[0] * 1000000.0f);
    ekf_p1_u = (int32_t)(ekf_diag.covariance_diag[1] * 1000000.0f);
    ekf_p2_u = (int32_t)(ekf_diag.covariance_diag[2] * 1000000.0f);
    ekf_p3_u = (int32_t)(ekf_diag.covariance_diag[3] * 1000000.0f);
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
    app_control_queue_proto_text(APP_PROTO_MSG_GPS_RECORD,
                                 "HW FLOW ok=%u init=%ld baud=%lu bytes=%lu frames=%lu valid=0x%02X age_ms=%lu source=%s vel_valid=%u\r\n",
                                 (unsigned int)flow_status.initialized,
                                 (long)flow_status.init_status,
                                 (unsigned long)flow_status.baud_rate,
                                 (unsigned long)flow_status.bytes,
                                 (unsigned long)flow_status.frames,
                                 (unsigned int)flow_status.valid,
                                 (unsigned long)flow_status.age_ms,
                                 APP_OpticalFlow_VelSourceName(flow_status.velocity_source),
                                 (unsigned int)flow_status.velocity_valid);
    app_control_queue_proto_text(APP_PROTO_MSG_MAG_RECORD,
                                 "HW MAG ok=%u init=%ld st=%ld type=%s addr=0x%02X who=0x%02X n=%lu x=%ld y=%ld z=%ld\r\n",
                                 (unsigned int)mag_status.initialized,
                                 (long)mag_status.init_status,
                                 (long)mag_status.last_status,
                                 APP_MAG_GetTypeName(mag_status.type),
                                 (unsigned int)mag_status.address,
                                 (unsigned int)mag_status.who_am_i,
                                 (unsigned long)mag_status.sample_count,
                                 (long)mag_status.x_mgauss,
                                 (long)mag_status.y_mgauss,
                                 (long)mag_status.z_mgauss);

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
    app_control_queue_proto_text(APP_PROTO_MSG_GPS_RECORD,
                                 "STATUS flow init=%u st=%ld valid=0x%02X frames=%lu cksum=%lu age=%lu h=%.3f vx=%.3f vy=%.3f source=%s\r\n",
                                 (unsigned int)flow_status.initialized,
                                 (long)flow_status.init_status,
                                 (unsigned int)flow_status.valid,
                                 (unsigned long)flow_status.frames,
                                 (unsigned long)flow_status.checksum_errors,
                                 (unsigned long)flow_status.age_ms,
                                 (double)flow_status.height_m,
                                 (double)flow_status.vx_m_s,
                                 (double)flow_status.vy_m_s,
                                 APP_OpticalFlow_VelSourceName(flow_status.velocity_source));
    APP_Control_QueueText("STATUS ekf init=%u pred=%lu upd=%lu rej=%lu skip=%lu nis_milli=%ld gate_milli=%ld innov_mm_s=%ld,%ld noise_mm_s=%ld vx_mm_s=%ld vy_mm_s=%ld bias_mm_s2=%ld,%ld p_u=%ld,%ld,%ld,%ld\r\n",
                          (unsigned int)ekf_diag.initialized,
                          (unsigned long)ekf_diag.predict_count,
                          (unsigned long)ekf_diag.flow_update_count,
                          (unsigned long)ekf_diag.flow_reject_count,
                          (unsigned long)ekf_diag.flow_skip_count,
                          (long)ekf_nis_milli,
                          (long)ekf_gate_milli,
                          (long)ekf_innov_x_mm_s,
                          (long)ekf_innov_y_mm_s,
                          (long)ekf_noise_mm_s,
                          (long)ekf_vx_mm_s,
                          (long)ekf_vy_mm_s,
                          (long)ekf_bias_x_mm_s2,
                          (long)ekf_bias_y_mm_s2,
                          (long)ekf_p0_u,
                          (long)ekf_p1_u,
                          (long)ekf_p2_u,
                          (long)ekf_p3_u);
    app_control_queue_proto_text(APP_PROTO_MSG_MAG_RECORD,
                                 "STATUS mag init=%u st=%ld type=%s addr=0x%02X who=0x%02X n=%lu raw=%d,%d,%d mgauss=%ld,%ld,%ld\r\n",
                                 (unsigned int)mag_status.initialized,
                                 (long)mag_status.last_status,
                                 APP_MAG_GetTypeName(mag_status.type),
                                 (unsigned int)mag_status.address,
                                 (unsigned int)mag_status.who_am_i,
                                 (unsigned long)mag_status.sample_count,
                                 (int)mag_status.raw_x,
                                 (int)mag_status.raw_y,
                                 (int)mag_status.raw_z,
                                 (long)mag_status.x_mgauss,
                                 (long)mag_status.y_mgauss,
                                 (long)mag_status.z_mgauss);
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

static APP_FlashService_Status app_control_load_config(void)
{
    APP_ControlFlashRecordV8 record;
    APP_FlashService_Status status;
    uint32_t checksum;

    status = APP_FlashService_ReadData(APP_CONTROL_CFG_ADDRESS,
                                (uint8_t *)&record,
                                sizeof(record));
    if (status != APP_FLASH_SERVICE_OK) {
        return status;
    }

    if (record.magic != APP_CONTROL_CFG_MAGIC) {
        return APP_FLASH_SERVICE_BAD_ID;
    }

    if ((record.version == APP_CONTROL_CFG_VERSION) &&
        (record.size == (sizeof(record.config) + sizeof(record.coax_params)))) {
        checksum = app_control_checksum((const uint8_t *)&record.config,
                                        record.size);
        if (checksum != record.checksum) {
            return APP_FLASH_SERVICE_ERROR;
        }
        control_config = record.config;
        DRV_COAX_CTRL_SetParams(&record.coax_params);
    } else if (((record.version == 5U) ||
                (record.version == 6U) ||
                (record.version == 7U)) &&
               (record.size == (sizeof(APP_ControlConfig) +
                                sizeof(APP_ControlCoaxParamsV7)))) {
        const APP_ControlFlashRecordV7 *legacy =
            (const APP_ControlFlashRecordV7 *)&record;
        DRV_COAX_CTRL_Params migrated_params;

        checksum = app_control_checksum((const uint8_t *)&legacy->config,
                                        legacy->size);
        if (checksum != legacy->checksum) {
            return APP_FLASH_SERVICE_ERROR;
        }
        control_config = legacy->config;
        app_control_migrate_coax_params_v7(&legacy->coax_params,
                                           &migrated_params);
        if ((legacy->version == 5U) || (legacy->version == 6U)) {
            app_control_apply_v7_safety_defaults(&migrated_params);
        }
        DRV_COAX_CTRL_SetParams(&migrated_params);
    } else if (((record.version == 3U) || (record.version == 4U)) &&
               (record.size == (sizeof(APP_ControlConfig) + sizeof(APP_ControlCoaxParamsV4)))) {
        const APP_ControlFlashRecordV4 *legacy =
            (const APP_ControlFlashRecordV4 *)&record;
        DRV_COAX_CTRL_Params migrated_params;

        checksum = app_control_checksum((const uint8_t *)&legacy->config,
                                        legacy->size);
        if (checksum != legacy->checksum) {
            return APP_FLASH_SERVICE_ERROR;
        }
        control_config = legacy->config;
        app_control_migrate_coax_params_v4(&legacy->coax_params, &migrated_params);
        DRV_COAX_CTRL_SetParams(&migrated_params);
    } else if ((record.version == 2U) &&
               (record.size == sizeof(APP_ControlConfigV2))) {
        const APP_ControlFlashRecordV2 *legacy =
            (const APP_ControlFlashRecordV2 *)&record;
        APP_ControlConfig migrated_config;

        checksum = app_control_checksum((const uint8_t *)&legacy->config,
                                        legacy->size);
        if (checksum != legacy->checksum) {
            return APP_FLASH_SERVICE_ERROR;
        }
        app_control_defaults(&migrated_config);
        migrated_config.loaded_from_flash = legacy->config.loaded_from_flash;
        migrated_config.flash_valid = legacy->config.flash_valid;
        migrated_config.last_flash_status = legacy->config.last_flash_status;
        for (uint32_t index = 0U; index < APP_CONTROL_SERVO_COUNT; ++index) {
            migrated_config.servo[index] = legacy->config.servo[index];
        }
        control_config = migrated_config;
    } else if ((record.version == 1U) &&
               (record.size == sizeof(APP_ControlConfigV1))) {
        const APP_ControlFlashRecordV1 *legacy =
            (const APP_ControlFlashRecordV1 *)&record;
        APP_ControlConfig migrated_config;

        checksum = app_control_checksum((const uint8_t *)&legacy->config,
                                        legacy->size);
        if (checksum != legacy->checksum) {
            return APP_FLASH_SERVICE_ERROR;
        }
        app_control_defaults(&migrated_config);
        migrated_config.loaded_from_flash = legacy->config.loaded_from_flash;
        migrated_config.flash_valid = legacy->config.flash_valid;
        migrated_config.last_flash_status = legacy->config.last_flash_status;
        for (uint32_t index = 0U; index < APP_CONTROL_SERVO_COUNT; ++index) {
            migrated_config.servo[index] = legacy->config.servo[index];
        }
        control_config = migrated_config;
    } else {
        return APP_FLASH_SERVICE_BAD_ID;
    }
    control_config.loaded_from_flash = 1U;
    control_config.flash_valid = 1U;
    return APP_FLASH_SERVICE_OK;
}

static APP_FlashService_Status app_control_save_config(void)
{
    APP_ControlFlashRecordV8 record;
    APP_FlashService_Status status;

    memset(&record, 0xFF, sizeof(record));
    record.magic = APP_CONTROL_CFG_MAGIC;
    record.version = APP_CONTROL_CFG_VERSION;
    record.size = (uint16_t)(sizeof(record.config) + sizeof(record.coax_params));
    record.config = control_config;
    record.config.loaded_from_flash = 1U;
    record.config.flash_valid = 1U;
    DRV_COAX_CTRL_GetParams(&record.coax_params);
    record.checksum = app_control_checksum((const uint8_t *)&record.config,
                                           record.size);

    status = APP_FlashService_EraseSector(APP_CONTROL_CFG_ADDRESS);
    if (status != APP_FLASH_SERVICE_OK) {
        return status;
    }

    return APP_FlashService_WriteData(APP_CONTROL_CFG_ADDRESS,
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
        moves[count].pulse_us =
            app_control_servo_clamp_pulse(index,
                                          control_config.servo[index].pulse_us);
        if (control_config.servo[index].time_ms > time_ms) {
            time_ms = control_config.servo[index].time_ms;
        }
        ++count;
    }

    if (count == 0U) {
        APP_Control_QueueText("ERR servo no enabled channels\r\n");
        return;
    }

    status = BSP_BusServo_MoveMany(moves, count, time_ms);
    APP_Control_QueueText("OK servo move_all st=%u count=%u time=%u\r\n",
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
        APP_Control_QueueText("ERR servo missing subcmd\r\n");
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
            APP_Control_QueueText("ERR usage SERVO MOVE index pulse time\r\n");
            return;
        }

        control_config.servo[index].pulse_us =
            app_control_servo_clamp_pulse(index, (uint16_t)pulse);
        control_config.servo[index].time_ms = (uint16_t)time_ms;
        status = BSP_BusServo_Move(control_config.servo[index].id,
                                   control_config.servo[index].pulse_us,
                                   control_config.servo[index].time_ms);
        APP_Control_QueueText("OK servo%lu move st=%u id=%u pulse=%u time=%u\r\n",
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

    if (strcmp(tokens[1], "ANGLE") == 0) {
        uint32_t angle;
        uint32_t time_ms;
        uint16_t pulse;

        if ((count < 4U) ||
            (app_control_parse_u32(tokens[2], &index) == 0U) ||
            (app_control_parse_u32(tokens[3], &angle) == 0U) ||
            (app_control_valid_servo_index(index) == 0U) ||
            (angle > 180U)) {
            APP_Control_QueueText("ERR usage SERVO ANGLE index degree [time_ms]\r\n");
            return;
        }

        pulse = app_control_servo_angle_to_pulse(angle);

        if (count >= 5U) {
            if (app_control_parse_u32(tokens[4], &time_ms) == 0U) { time_ms = 500U; }
        } else {
            time_ms = control_config.servo[index].time_ms;
        }

        control_config.servo[index].pulse_us =
            app_control_servo_clamp_pulse(index, pulse);
        control_config.servo[index].time_ms = (uint16_t)time_ms;
        status = BSP_BusServo_Move(control_config.servo[index].id,
                                   control_config.servo[index].pulse_us,
                                   control_config.servo[index].time_ms);
        APP_Control_QueueText("OK servo%lu angle st=%u id=%u deg=%u pulse=%u\r\n",
                              (unsigned long)index, (unsigned int)status,
                              (unsigned int)control_config.servo[index].id,
                              (unsigned int)angle,
                              (unsigned int)control_config.servo[index].pulse_us);
        return;
    }

    if (strcmp(tokens[1], "ID") == 0) {
        if ((count < 4U) ||
            (app_control_parse_u32(tokens[2], &index) == 0U) ||
            (app_control_parse_u32(tokens[3], &value) == 0U) ||
            (app_control_valid_servo_index(index) == 0U) ||
            (value > 255U)) {
            APP_Control_QueueText("ERR usage SERVO ID index id\r\n");
            return;
        }

        control_config.servo[index].id = (uint8_t)value;
        APP_Control_QueueText("OK servo%lu id=%u\r\n",
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
            APP_Control_QueueText("ERR usage SERVO SETID index new_id\r\n");
            return;
        }

        status = BSP_BusServo_SetId(control_config.servo[index].id, (uint8_t)new_id);
        control_config.servo[index].id = (uint8_t)new_id;
        APP_Control_QueueText("OK servo%lu setid st=%u id=%u\r\n",
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
            APP_Control_QueueText("ERR usage SERVO MODE index mode\r\n");
            return;
        }

        status = BSP_BusServo_SetMode(control_config.servo[index].id, (uint8_t)value);
        if (status == BSP_BUS_SERVO_OK) {
            control_config.servo[index].mode = (uint8_t)value;
        }
        APP_Control_QueueText("OK servo%lu mode st=%u mode=%u\r\n",
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
            APP_Control_QueueText("ERR usage SERVO ENABLE index 0|1\r\n");
            return;
        }

        control_config.servo[index].enabled = (value != 0U) ? 1U : 0U;
        APP_Control_QueueText("OK servo%lu enabled=%u\r\n",
                               (unsigned long)index,
                               (unsigned int)control_config.servo[index].enabled);
        return;
    }

    if (strcmp(tokens[1], "CMD") == 0) {
        if ((count < 4U) ||
            (app_control_parse_u32(tokens[2], &index) == 0U) ||
            (app_control_valid_servo_index(index) == 0U)) {
            APP_Control_QueueText("ERR usage SERVO CMD index action\r\n");
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
                APP_Control_QueueText("ERR usage SERVO CMD index BD code\r\n");
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
            APP_Control_QueueText("ERR unknown servo action %s\r\n", tokens[3]);
            return;
        }

        APP_Control_QueueText("OK servo%lu cmd=%s st=%u\r\n",
                               (unsigned long)index,
                               tokens[3],
                               (unsigned int)status);
        return;
    }

    if (strcmp(tokens[1], "RAW") == 0) {
        char response[DRV_SERVO_MAX_RESPONSE_LEN + 1];
        uint16_t rx_len;

        if (count < 3U) {
            APP_Control_QueueText("ERR usage SERVO RAW command\r\n");
            return;
        }

        status = BSP_BusServo_SendRaw(tokens[2]);
        if (status == BSP_BUS_SERVO_OK) {
            rx_len = BSP_BusServo_ReadResponse(response, DRV_SERVO_MAX_RESPONSE_LEN);
            if (rx_len > 0U) {
                response[rx_len] = '\0';
                APP_Control_QueueText("OK servo raw st=%u rsp=%s\r\n", (unsigned int)status, response);
            } else {
                APP_Control_QueueText("OK servo raw st=%u rsp=(none)\r\n", (unsigned int)status);
            }
        } else {
            APP_Control_QueueText("ERR servo raw tx st=%u\r\n", (unsigned int)status);
        }
        return;
    }

    if (strcmp(tokens[1], "BAUDRATE") == 0) {
        uint32_t rate;
        if ((count < 3U) || (app_control_parse_u32(tokens[2], &rate) == 0U)) {
            APP_Control_QueueText("ERR usage SERVO BAUDRATE rate\r\n");
            return;
        }
        status = BSP_BusServo_SetBaudRate(rate);
        APP_Control_QueueText("OK servo baudrate st=%u rate=%u cur=%u\r\n",
                              (unsigned int)status, (unsigned int)rate,
                              (unsigned int)BSP_BusServo_GetBaudRate());
        return;
    }

    APP_Control_QueueText("ERR unknown servo subcmd %s\r\n", tokens[1]);
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
            APP_Control_QueueText("ERR usage WIFI AT command\r\n");
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
                APP_Control_QueueText("ERR wifi at too long\r\n");
                return;
            }
            offset += (uint32_t)written;
        }

        if (APP_AiWB2_SendRawCommand(raw_command) == 0U) {
            APP_Control_QueueText("ERR wifi at bad command\r\n");
            return;
        }

        APP_Control_QueueText("OK wifi at %s\r\n", raw_command);
        return;
    }

    if (strcmp(tokens[1], "DIAG") == 0) {
        APP_AiWB2_SendDiagCommands();
        APP_Control_QueueText("OK wifi diag queued\r\n");
        return;
    }

    if ((strcmp(tokens[1], "EN") == 0) || (strcmp(tokens[1], "ENABLE") == 0)) {
        if ((count < 3U) || (app_control_parse_u32(tokens[2], &value) == 0U)) {
            APP_Control_QueueText("ERR usage WIFI EN 0|1\r\n");
            return;
        }

        control_wifi_reset_pending = 0U;
        BSP_AiWB2_SetEnabled((value != 0U) ? 1U : 0U);
        APP_Control_QueueText("OK wifi en=%u pin=PC6\r\n",
                               (unsigned int)BSP_AiWB2_IsEnabled());
        return;
    }

    if (strcmp(tokens[1], "RESET") == 0) {
        if ((count >= 3U) && (app_control_parse_u32(tokens[2], &pulse_ms) == 0U)) {
            APP_Control_QueueText("ERR usage WIFI RESET [ms]\r\n");
            return;
        }
        if (pulse_ms > 5000U) {
            pulse_ms = 5000U;
        }

        BSP_AiWB2_SetEnabled(0U);
        control_wifi_reset_pending = 1U;
        control_wifi_reset_deadline_ms = HAL_GetTick() + pulse_ms;
        APP_Control_QueueText("OK wifi reset queued ms=%lu pin=PC6\r\n",
                               (unsigned long)pulse_ms);
        return;
    }

    if ((strcmp(tokens[1], "STA") == 0) || (strcmp(tokens[1], "PROVISION") == 0)) {
        const char *local_port;

        if (count < 5U) {
            APP_Control_QueueText("ERR usage WIFI STA ssid password local_port\r\n");
            return;
        }

        local_port = (count >= 6U) ? tokens[5] : tokens[4];
        if (APP_AiWB2_StartProvision(tokens[2], tokens[3], APP_AIWB2_LINK_UDP_SERVER, "0.0.0.0", local_port) == 0U) {
            APP_Control_QueueText("ERR wifi sta bad args\r\n");
            return;
        }

        APP_Control_QueueText("OK wifi sta queued ssid=%s udp_server_port=%s\r\n",
                               tokens[2],
                               local_port);
        return;
    }

    if (strcmp(tokens[1], "AP") == 0) {
        const char *local_port;
        const char *channel;

        if (count < 4U) {
            APP_Control_QueueText("ERR usage WIFI AP ssid password [local_port] [channel]\r\n");
            return;
        }

        local_port = (count >= 5U) ? tokens[4] : "7777";
        channel = (count >= 6U) ? tokens[5] : "6";
        if (APP_AiWB2_StartSoftAp(tokens[2], tokens[3], channel, local_port) == 0U) {
            APP_Control_QueueText("ERR wifi ap bad args\r\n");
            return;
        }

        APP_Control_QueueText("OK wifi ap queued ssid=%s ip=192.168.43.1 udp_server_port=%s channel=%s\r\n",
                              tokens[2],
                              local_port,
                              channel);
        return;
    }

    APP_Control_QueueText("ERR unknown wifi subcmd %s\r\n", tokens[1]);
}

static void app_control_ident_stop(const char *reason)
{
    if (reason == NULL) {
        reason = "stop";
    }

    ident_active = 0U;
    (void)DRV_Motor_StopAll();
    APP_Control_QueueText("IDENT stop reason=%s seq=%lu ms=%lu\r\n",
                          reason,
                          (unsigned long)ident_seq,
                          (unsigned long)HAL_GetTick());
}

static void app_control_ident_emit_sample(void)
{
#if (APP_CONTROL_ALLOW_IDENT_MOTOR_TEST != 0U)
    DRV_MOTOR_Status status;

    status = DRV_Motor_SetPercent(ident_motor, ident_current_percent);
    if (status != DRV_MOTOR_OK) {
        APP_Control_QueueText("IDENT err seq=%lu motor=%u pct=%lu st=%u\r\n",
                              (unsigned long)ident_seq,
                              (unsigned int)ident_motor,
                              (unsigned long)ident_current_percent,
                              (unsigned int)status);
        app_control_ident_stop("motor_error");
        return;
    }

    APP_Control_QueueText("IDENT sample seq=%lu motor=%u pct=%lu pulse=%u dwell_ms=%lu ms=%lu\r\n",
                          (unsigned long)ident_seq,
                          (unsigned int)ident_motor,
                          (unsigned long)ident_current_percent,
                          (unsigned int)DRV_Motor_PercentToPulse(ident_current_percent),
                          (unsigned long)ident_dwell_ms,
                          (unsigned long)HAL_GetTick());
    ++ident_seq;
#else
    app_control_ident_stop("disabled");
#endif
}

static void app_control_ident_step(void)
{
    uint32_t now_ms;

    if (ident_active == 0U) {
        return;
    }

    now_ms = HAL_GetTick();
    if ((int32_t)(now_ms - ident_next_ms) < 0) {
        return;
    }

    if (ident_current_percent > ident_max_percent) {
        app_control_ident_stop("done");
        return;
    }

    app_control_ident_emit_sample();
    if (ident_current_percent > (ident_max_percent - ident_step_percent)) {
        ident_current_percent = ident_max_percent + 1U;
    } else {
        ident_current_percent += ident_step_percent;
    }
    ident_next_ms = now_ms + ident_dwell_ms;
}

static void app_control_handle_motor(char **tokens, uint32_t count)
{
    if (count < 2U) {
        APP_Control_QueueText("ERR usage MOTOR SET 0|1|2 pct | MOTOR STOP | MOTOR?\r\n");
        return;
    }

    if (strcmp(tokens[1], "SET") == 0) {
        uint32_t motor;
        uint32_t percent;
        DRV_MOTOR_Status status;

        if ((count < 4U) ||
            (app_control_parse_u32(tokens[2], &motor) == 0U) ||
            (app_control_parse_u32(tokens[3], &percent) == 0U) ||
            (app_control_valid_motor(motor) == 0U) ||
            (percent > DRV_MOTOR_PERCENT_MAX)) {
            APP_Control_QueueText("ERR usage MOTOR SET 0|1|2 0..100\r\n");
            return;
        }

        if ((percent != 0U) && (APP_CONTROL_ALLOW_RAW_MOTOR_COMMANDS == 0U)) {
            APP_Control_QueueText("ERR raw motor disabled; use ARM switch\r\n");
            return;
        }

        status = (percent == 0U) ? DRV_Motor_Stop(motor) : DRV_Motor_SetPercent(motor, percent);
        APP_Control_QueueText("OK motor%lu st=%u pct=%lu pulse=%u\r\n",
                              (unsigned long)motor,
                              (unsigned int)status,
                              (unsigned long)percent,
                              (unsigned int)DRV_Motor_GetPulse(motor));
    } else if (strcmp(tokens[1], "STOP") == 0) {
        ident_active = 0U;
        APP_Control_QueueText("OK motor stop st=%u\r\n",
                              (unsigned int)DRV_Motor_StopAll());
    } else {
        APP_Control_QueueText("ERR unknown motor subcmd %s\r\n", tokens[1]);
    }
}

static void app_control_handle_ident(char **tokens, uint32_t count)
{
    if (count < 2U) {
        APP_Control_QueueText("ERR usage IDENT ARM|DISARM|STOP|STEP|DOUBLET|PRBS|CENTER|APPLY|?\r\n");
        return;
    }

    if ((strcmp(tokens[1], "?") == 0) || (strcmp(tokens[1], "STATUS") == 0)) {
        APP_Ident_ReportStatus();
        return;
    }

    if (strcmp(tokens[1], "ARM") == 0) {
        (void)APP_Ident_Arm();
        return;
    }

    if (strcmp(tokens[1], "DISARM") == 0) {
        APP_Ident_Disarm();
        return;
    }

    if (strcmp(tokens[1], "STOP") == 0) {
        APP_Ident_Stop("command");
        return;
    }

    if (strcmp(tokens[1], "CENTER") == 0) {
        uint32_t alpha_us;
        uint32_t beta_us;
        const char *alpha_text = app_control_token_value(tokens, count, "alpha_us");
        const char *beta_text = app_control_token_value(tokens, count, "beta_us");

        if ((alpha_text == NULL) || (beta_text == NULL) ||
            (app_control_parse_u32(alpha_text, &alpha_us) == 0U) ||
            (app_control_parse_u32(beta_text, &beta_us) == 0U) ||
            (alpha_us > 65535U) || (beta_us > 65535U)) {
            APP_Control_QueueText("ERR usage IDENT CENTER alpha_us=<v> beta_us=<v>\r\n");
            return;
        }
        (void)APP_Ident_SetCenter((uint16_t)alpha_us, (uint16_t)beta_us);
        return;
    }

    if (strcmp(tokens[1], "APPLY") == 0) {
        const char *kp_text;
        const char *kd_text;

        if (count < 3U) {
            APP_Control_QueueText("ERR usage IDENT APPLY roll|pitch [kp=<v>] [kd=<v>]\r\n");
            return;
        }
        kp_text = app_control_token_value(tokens, count, "kp");
        kd_text = app_control_token_value(tokens, count, "kd");
        if ((kp_text == NULL) && (kd_text == NULL)) {
            APP_Control_QueueText("ERR usage IDENT APPLY roll|pitch [kp=<v>] [kd=<v>]\r\n");
            return;
        }
        (void)APP_Ident_ApplyPid(tokens[2], kp_text, kd_text);
        return;
    }

    if (strcmp(tokens[1], "STEP") == 0) {
        uint32_t duration_ms;
        int32_t pulse_us;
        const char *pulse_text;
        const char *duration_text;

        if (count < 3U) {
            APP_Control_QueueText("ERR usage IDENT STEP roll|pitch pulse_us=<v> duration_ms=<v>\r\n");
            return;
        }
        pulse_text = app_control_token_value(tokens, count, "pulse_us");
        duration_text = app_control_token_value(tokens, count, "duration_ms");
        if ((pulse_text == NULL) || (duration_text == NULL) ||
            (app_control_parse_i32(pulse_text, &pulse_us) == 0U) ||
            (app_control_parse_u32(duration_text, &duration_ms) == 0U)) {
            APP_Control_QueueText("ERR usage IDENT STEP roll|pitch pulse_us=<v> duration_ms=<v>\r\n");
            return;
        }
        (void)APP_Ident_StartStep(tokens[2], pulse_us, duration_ms);
        return;
    }

    if (strcmp(tokens[1], "DOUBLET") == 0) {
        uint32_t hold_ms;
        uint32_t repeat;
        int32_t pulse_us;
        const char *pulse_text;
        const char *hold_text;
        const char *repeat_text;

        if (count < 3U) {
            APP_Control_QueueText("ERR usage IDENT DOUBLET roll|pitch pulse_us=<v> hold_ms=<v> repeat=<v>\r\n");
            return;
        }
        pulse_text = app_control_token_value(tokens, count, "pulse_us");
        hold_text = app_control_token_value(tokens, count, "hold_ms");
        repeat_text = app_control_token_value(tokens, count, "repeat");
        if ((pulse_text == NULL) || (hold_text == NULL) || (repeat_text == NULL) ||
            (app_control_parse_i32(pulse_text, &pulse_us) == 0U) ||
            (app_control_parse_u32(hold_text, &hold_ms) == 0U) ||
            (app_control_parse_u32(repeat_text, &repeat) == 0U)) {
            APP_Control_QueueText("ERR usage IDENT DOUBLET roll|pitch pulse_us=<v> hold_ms=<v> repeat=<v>\r\n");
            return;
        }
        (void)APP_Ident_StartDoublet(tokens[2], pulse_us, hold_ms, repeat);
        return;
    }

    if (strcmp(tokens[1], "PRBS") == 0) {
        uint32_t bit_ms;
        uint32_t duration_ms;
        uint32_t seed = 1U;
        int32_t pulse_us;
        const char *pulse_text;
        const char *bit_text;
        const char *duration_text;
        const char *seed_text;

        if (count < 3U) {
            APP_Control_QueueText("ERR usage IDENT PRBS roll|pitch pulse_us=<v> bit_ms=<v> duration_ms=<v> [seed=<v>]\r\n");
            return;
        }
        pulse_text = app_control_token_value(tokens, count, "pulse_us");
        bit_text = app_control_token_value(tokens, count, "bit_ms");
        duration_text = app_control_token_value(tokens, count, "duration_ms");
        seed_text = app_control_token_value(tokens, count, "seed");
        if ((seed_text != NULL) && (app_control_parse_u32(seed_text, &seed) == 0U)) {
            APP_Control_QueueText("ERR ident seed\r\n");
            return;
        }
        if ((pulse_text == NULL) || (bit_text == NULL) || (duration_text == NULL) ||
            (app_control_parse_i32(pulse_text, &pulse_us) == 0U) ||
            (app_control_parse_u32(bit_text, &bit_ms) == 0U) ||
            (app_control_parse_u32(duration_text, &duration_ms) == 0U)) {
            APP_Control_QueueText("ERR usage IDENT PRBS roll|pitch pulse_us=<v> bit_ms=<v> duration_ms=<v> [seed=<v>]\r\n");
            return;
        }
        (void)APP_Ident_StartPrbs(tokens[2], pulse_us, bit_ms, duration_ms, seed);
        return;
    }

    if (strcmp(tokens[1], "START") == 0) {
        uint32_t motor;
        uint32_t min_percent = APP_CONTROL_IDENT_DEFAULT_MIN_PERCENT;
        uint32_t max_percent = APP_CONTROL_IDENT_DEFAULT_MAX_PERCENT;
        uint32_t step_percent = APP_CONTROL_IDENT_DEFAULT_STEP_PERCENT;
        uint32_t dwell_ms = APP_CONTROL_IDENT_DEFAULT_DWELL_MS;

        if ((count < 3U) ||
            (app_control_parse_u32(tokens[2], &motor) == 0U) ||
            (app_control_valid_motor(motor) == 0U)) {
            APP_Control_QueueText("ERR usage IDENT START 0|1|2 [min max step dwell_ms]\r\n");
            return;
        }
        if ((count >= 4U) && (app_control_parse_u32(tokens[3], &min_percent) == 0U)) {
            APP_Control_QueueText("ERR ident min\r\n");
            return;
        }
        if ((count >= 5U) && (app_control_parse_u32(tokens[4], &max_percent) == 0U)) {
            APP_Control_QueueText("ERR ident max\r\n");
            return;
        }
        if ((count >= 6U) && (app_control_parse_u32(tokens[5], &step_percent) == 0U)) {
            APP_Control_QueueText("ERR ident step\r\n");
            return;
        }
        if ((count >= 7U) && (app_control_parse_u32(tokens[6], &dwell_ms) == 0U)) {
            APP_Control_QueueText("ERR ident dwell\r\n");
            return;
        }
        if ((min_percent > DRV_MOTOR_PERCENT_MAX) ||
            (max_percent > DRV_MOTOR_PERCENT_MAX) ||
            (min_percent > max_percent) ||
            (step_percent == 0U) ||
            (dwell_ms == 0U)) {
            APP_Control_QueueText("ERR ident range min=%lu max=%lu step=%lu dwell=%lu\r\n",
                                  (unsigned long)min_percent,
                                  (unsigned long)max_percent,
                                  (unsigned long)step_percent,
                                  (unsigned long)dwell_ms);
            return;
        }

        if (APP_CONTROL_ALLOW_IDENT_MOTOR_TEST == 0U) {
            APP_Control_QueueText("ERR ident disabled for prop safety\r\n");
            return;
        }

        (void)DRV_Motor_StopAll();
        ident_motor = (uint8_t)motor;
        ident_min_percent = min_percent;
        ident_max_percent = max_percent;
        ident_step_percent = step_percent;
        ident_dwell_ms = dwell_ms;
        ident_current_percent = min_percent;
        ident_next_ms = HAL_GetTick();
        ident_seq = 0U;
        ident_active = 1U;
        APP_Control_QueueText("IDENT start motor=%u min=%lu max=%lu step=%lu dwell_ms=%lu ms=%lu\r\n",
                              (unsigned int)ident_motor,
                              (unsigned long)ident_min_percent,
                              (unsigned long)ident_max_percent,
                              (unsigned long)ident_step_percent,
                              (unsigned long)ident_dwell_ms,
                              (unsigned long)HAL_GetTick());
        app_control_ident_step();
    } else {
        APP_Control_QueueText("ERR unknown ident subcmd %s\r\n", tokens[1]);
    }
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
    APP_Control_QueueText("OK wifi reset done en=%u\r\n",
                           (unsigned int)BSP_AiWB2_IsEnabled());
}

static void app_control_schedule_flash_autosave(void)
{
    control_flash_autosave_pending = 1U;
    control_flash_autosave_deadline_ms =
        HAL_GetTick() + APP_CONTROL_FLASH_AUTOSAVE_DELAY_MS;
}

static void app_control_service_flash_autosave(void)
{
    APP_FlashService_Status save_status;

    if (control_flash_autosave_pending == 0U) {
        return;
    }

    if ((int32_t)(HAL_GetTick() - control_flash_autosave_deadline_ms) < 0) {
        return;
    }

    control_flash_autosave_pending = 0U;
    save_status = app_control_save_config();
    control_config.last_flash_status = (uint8_t)save_status;
    if (save_status == APP_FLASH_SERVICE_OK) {
        control_config.loaded_from_flash = 1U;
        control_config.flash_valid = 1U;
    }
}

static void app_control_handle_baro(char **tokens, uint32_t count)
{
    if ((count == 1U) || ((count >= 2U) && (strcmp(tokens[1], "?") == 0))) {
        app_control_report_baro();
        return;
    }

    APP_Control_QueueText("OK baro stream=0 (streaming removed)\r\n");
}

static uint8_t app_control_parse_hex_byte(const char *text, uint8_t *value)
{
    char *end = NULL;
    unsigned long parsed;

    if ((text == NULL) || (value == NULL) || (text[0] == '\0')) {
        return 0U;
    }

    parsed = strtoul(text, &end, 16);
    if ((end == text) || (*end != '\0') || (parsed > 0xFFUL)) {
        return 0U;
    }

    *value = (uint8_t)parsed;
    return 1U;
}

static void app_control_handle_flow(char **tokens, uint32_t count)
{
    uint8_t tx_bytes[APP_CONTROL_FLOW_RAW_MAX_BYTES];
    uint8_t rx_bytes[16];
    uint32_t tx_count;
    uint16_t rx_count;
    BSP_OPTICAL_FLOW_StatusCode status;

    if ((count == 1U) || ((count >= 2U) && (strcmp(tokens[1], "?") == 0))) {
        APP_Control_QueueText("ERR usage FLOW TX hex... | FLOW RX [max] | FLOW XCV rx_len hex... | FLOW PINGAB\r\n");
        return;
    }

    if (strcmp(tokens[1], "PINGAB") == 0) {
        static const uint8_t ab_command[] = {
            0xAAU, 0xABU, 0x96U, 0x26U, 0xBCU, 0x50U, 0x5CU
        };
        status = BSP_OPTICAL_FLOW_TransceiveRaw(ab_command,
                                                (uint16_t)sizeof(ab_command),
                                                rx_bytes, 3U, &rx_count,
                                                100U);
        APP_Control_QueueText("FLOW PINGAB tx_st=%ld rx_n=%u rx=%02X,%02X,%02X\r\n",
                              (long)status,
                              (unsigned int)rx_count,
                              (unsigned int)((rx_count > 0U) ? rx_bytes[0] : 0U),
                              (unsigned int)((rx_count > 1U) ? rx_bytes[1] : 0U),
                              (unsigned int)((rx_count > 2U) ? rx_bytes[2] : 0U));
        return;
    }

    if (strcmp(tokens[1], "RX") == 0) {
        uint32_t max_rx = 3U;
        if (count >= 3U) {
            max_rx = strtoul(tokens[2], NULL, 0);
        }
        if ((max_rx == 0U) || (max_rx > sizeof(rx_bytes))) {
            APP_Control_QueueText("ERR usage FLOW RX 1..16\r\n");
            return;
        }
        rx_count = BSP_OPTICAL_FLOW_ReceiveRaw(rx_bytes, (uint16_t)max_rx, 100U);
        APP_Control_QueueText("FLOW RX n=%u data=%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X\r\n",
                              (unsigned int)rx_count,
                              (unsigned int)((rx_count > 0U) ? rx_bytes[0] : 0U),
                              (unsigned int)((rx_count > 1U) ? rx_bytes[1] : 0U),
                              (unsigned int)((rx_count > 2U) ? rx_bytes[2] : 0U),
                              (unsigned int)((rx_count > 3U) ? rx_bytes[3] : 0U),
                              (unsigned int)((rx_count > 4U) ? rx_bytes[4] : 0U),
                              (unsigned int)((rx_count > 5U) ? rx_bytes[5] : 0U),
                              (unsigned int)((rx_count > 6U) ? rx_bytes[6] : 0U),
                              (unsigned int)((rx_count > 7U) ? rx_bytes[7] : 0U));
        return;
    }

    if (strcmp(tokens[1], "XCV") == 0) {
        uint32_t max_rx;

        if ((count < 4U) || ((count - 3U) > APP_CONTROL_FLOW_RAW_MAX_BYTES)) {
            APP_Control_QueueText("ERR usage FLOW XCV rx_len hex... max=%u\r\n",
                                  (unsigned int)APP_CONTROL_FLOW_RAW_MAX_BYTES);
            return;
        }

        max_rx = strtoul(tokens[2], NULL, 0);
        if ((max_rx == 0U) || (max_rx > sizeof(rx_bytes))) {
            APP_Control_QueueText("ERR usage FLOW XCV rx_len 1..16 hex...\r\n");
            return;
        }

        tx_count = count - 3U;
        for (uint32_t i = 0U; i < tx_count; ++i) {
            if (app_control_parse_hex_byte(tokens[i + 3U], &tx_bytes[i]) == 0U) {
                APP_Control_QueueText("ERR flow hex %s\r\n", tokens[i + 3U]);
                return;
            }
        }

        status = BSP_OPTICAL_FLOW_TransceiveRaw(tx_bytes, (uint16_t)tx_count,
                                                rx_bytes, (uint16_t)max_rx,
                                                &rx_count, 100U);
        APP_Control_QueueText("FLOW XCV st=%ld tx_n=%lu rx_n=%u data=%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X\r\n",
                              (long)status,
                              (unsigned long)tx_count,
                              (unsigned int)rx_count,
                              (unsigned int)((rx_count > 0U) ? rx_bytes[0] : 0U),
                              (unsigned int)((rx_count > 1U) ? rx_bytes[1] : 0U),
                              (unsigned int)((rx_count > 2U) ? rx_bytes[2] : 0U),
                              (unsigned int)((rx_count > 3U) ? rx_bytes[3] : 0U),
                              (unsigned int)((rx_count > 4U) ? rx_bytes[4] : 0U),
                              (unsigned int)((rx_count > 5U) ? rx_bytes[5] : 0U),
                              (unsigned int)((rx_count > 6U) ? rx_bytes[6] : 0U),
                              (unsigned int)((rx_count > 7U) ? rx_bytes[7] : 0U));
        return;
    }

    if (strcmp(tokens[1], "TX") != 0) {
        APP_Control_QueueText("ERR unknown flow subcmd %s\r\n", tokens[1]);
        return;
    }

    if ((count < 3U) || ((count - 2U) > APP_CONTROL_FLOW_RAW_MAX_BYTES)) {
        APP_Control_QueueText("ERR usage FLOW TX hex... max=%u\r\n",
                              (unsigned int)APP_CONTROL_FLOW_RAW_MAX_BYTES);
        return;
    }

    tx_count = count - 2U;
    for (uint32_t i = 0U; i < tx_count; ++i) {
        if (app_control_parse_hex_byte(tokens[i + 2U], &tx_bytes[i]) == 0U) {
            APP_Control_QueueText("ERR flow hex %s\r\n", tokens[i + 2U]);
            return;
        }
    }

    status = BSP_OPTICAL_FLOW_TransmitRaw(tx_bytes, (uint16_t)tx_count, 100U);
    APP_Control_QueueText("FLOW TX st=%ld n=%lu\r\n",
                          (long)status,
                          (unsigned long)tx_count);
}

static void app_control_handle_pid(char **tokens, uint32_t count)
{
    const char *kp_text;
    const char *kd_text;
    float kp;
    float kd;
    const char *kp_name;
    const char *kd_name;

    if ((count == 1U) || ((count >= 2U) && (strcmp(tokens[1], "?") == 0)) ||
        ((count >= 2U) && (strcmp(tokens[1], "GET") == 0))) {
        app_control_report_pid_legacy();
        return;
    }

    if ((count < 3U) || (strcmp(tokens[1], "SET") != 0)) {
        APP_Control_QueueText("ERR usage PID SET roll|pitch|yaw kp= ki= kd=\r\n");
        return;
    }

    if (strcmp(tokens[2], "roll") == 0) {
        kp_name = "coax.roll_angle_kp";
        kd_name = "coax.roll_rate_kd";
    } else if (strcmp(tokens[2], "pitch") == 0) {
        kp_name = "coax.pitch_angle_kp";
        kd_name = "coax.pitch_rate_kd";
    } else if (strcmp(tokens[2], "yaw") == 0) {
        kp_name = "coax.yaw_angle_kp";
        kd_name = "coax.yaw_rate_kd";
    } else {
        APP_Control_QueueText("ERR pid axis %s\r\n", tokens[2]);
        return;
    }

    kp_text = app_control_token_value(tokens, count, "kp");
    kd_text = app_control_token_value(tokens, count, "kd");

    if ((kp_text == NULL) && (kd_text == NULL)) {
        APP_Control_QueueText("ERR usage PID SET roll|pitch|yaw [kp=<float>] [kd=<float>]\r\n");
        return;
    }

    if (kp_text != NULL) {
        if (app_control_parse_f32(kp_text, &kp) == 0U) {
            APP_Control_QueueText("ERR usage PID SET roll|pitch|yaw [kp=<float>] [kd=<float>]\r\n");
            return;
        }
        if (DRV_COAX_CTRL_SetParam(kp_name, kp) == 0U) {
            APP_Control_QueueText("ERR pid target %s\r\n", tokens[2]);
            return;
        }
        app_control_schedule_flash_autosave();
    }

    if (kd_text != NULL) {
        if (app_control_parse_f32(kd_text, &kd) == 0U) {
            APP_Control_QueueText("ERR usage PID SET roll|pitch|yaw [kp=<float>] [kd=<float>]\r\n");
            return;
        }
        if (DRV_COAX_CTRL_SetParam(kd_name, kd) != 0U) {
            app_control_schedule_flash_autosave();
        }
    }

    APP_Control_QueueText("OK pid axis=%s target=coax\r\n", tokens[2]);
    app_control_report_coax_param_by_name(kp_name);
    app_control_report_coax_param_by_name(kd_name);
    app_control_report_pid_legacy();
}

static uint8_t app_control_handle_pid_slider_line(const char *line)
{
    typedef struct {
        const char *slider_name;
        const char *param_name;
    } APP_ControlPidSliderMap;

    static const APP_ControlPidSliderMap map[] = {
        { "roll_rate_kd",   "coax.roll_rate_kd"   },
        { "pitch_rate_kd",  "coax.pitch_rate_kd"  },
        { "yaw_angle_kp",   "coax.yaw_angle_kp"   },
        { "yaw_rate_kd",    "coax.yaw_rate_kd"    },
        { "vel_x_kd",       "coax.vel_x_kd"       },
        { "vel_y_kd",       "coax.vel_y_kd"       },
        { "vel_z_kd",       "coax.vel_z_kd"       },
        { "accel_xy",       "coax.accel_xy_limit_m_s2" },
        { "accel_z",        "coax.accel_z_limit_m_s2"  },
        { "accel_xy_limit_m_s2", "coax.accel_xy_limit_m_s2" },
        { "accel_z_limit_m_s2",  "coax.accel_z_limit_m_s2"  },
        { "vel_loop_enable", "coax.vel_loop_enable" },
        { "vel_loop_x_kp",  "coax.vel_loop_x_kp" },
        { "vel_loop_x_ki",  "coax.vel_loop_x_ki" },
        { "vel_loop_x_kd",  "coax.vel_loop_x_kd" },
        { "vel_loop_y_kp",  "coax.vel_loop_y_kp" },
        { "vel_loop_y_ki",  "coax.vel_loop_y_ki" },
        { "vel_loop_y_kd",  "coax.vel_loop_y_kd" },
        { "vel_loop_out",   "coax.vel_loop_output_limit_m_s2" },
        { "vel_loop_i",     "coax.vel_loop_i_limit_m_s2" },
        { "aw_angle_kp",    "coax.yaw_angle_kp"   },
        { "aw_rate_kd",     "coax.yaw_rate_kd"    },
    };

    char value_text[24];

    if ((line == NULL) || (*line == '\0')) {
        return 0U;
    }

    for (uint32_t map_index = 0U; map_index < (sizeof(map) / sizeof(map[0])); ++map_index) {
        float value;

        if (app_control_named_value_line(line,
                                         map[map_index].slider_name,
                                         value_text,
                                         (uint32_t)sizeof(value_text)) == 0U) {
            continue;
        }

        if ((value_text[0] == '\0') ||
            (app_control_parse_f32(value_text, &value) == 0U)) {
            return 0U;
        }

        if (strcmp(map[map_index].param_name, "coax.tilt_limit_rad") == 0) {
            if ((value <= 0.0f) || (value > APP_CONTROL_SERVO_ANGLE_MAX_DEG)) {
                APP_Control_QueueText("ERR angle range\r\n");
                return 1U;
            }
            value *= APP_CONTROL_DEG_TO_RAD;
        }

        if (DRV_COAX_CTRL_SetParam(map[map_index].param_name, value) == 0U) {
            APP_Control_QueueText("ERR pid slider %s\r\n", map[map_index].slider_name);
        } else {
            app_control_schedule_flash_autosave();
            app_control_report_coax_param_by_name(map[map_index].param_name);
            app_control_report_pid_legacy();
        }
        return 1U;
    }

    return 0U;
}

static uint8_t app_control_handle_param_value_line(const char *line)
{
    char value_text[24];

    if ((line == NULL) || (*line == '\0')) {
        return 0U;
    }

    for (uint32_t index = 0U; index < DRV_COAX_CTRL_ParamCount(); ++index) {
        const char *name = DRV_COAX_CTRL_ParamName(index);
        float value;

        if (name == NULL) {
            continue;
        }

        if (app_control_named_value_line(line,
                                         name,
                                         value_text,
                                         (uint32_t)sizeof(value_text)) == 0U) {
            continue;
        }

        if (app_control_parse_f32(value_text, &value) == 0U) {
            APP_Control_QueueText("ERR param value %s\r\n", name);
            return 1U;
        }

        if (DRV_COAX_CTRL_SetParam(name, value) == 0U) {
            APP_Control_QueueText("ERR param target %s\r\n", name);
            return 1U;
        }

        app_control_schedule_flash_autosave();
        app_control_report_coax_param_by_name(name);
        app_control_report_pid_legacy();
        return 1U;
    }

    return 0U;
}


static void app_control_report_pid_legacy(void)
{
    float kp;
    float kd;
    char kp_text[24];
    char kd_text[24];

    (void)DRV_COAX_CTRL_GetParam("coax.roll_angle_kp", &kp);
    (void)DRV_COAX_CTRL_GetParam("coax.roll_rate_kd", &kd);
    app_control_format_float(kp, kp_text, (uint32_t)sizeof(kp_text));
    app_control_format_float(kd, kd_text, (uint32_t)sizeof(kd_text));
    app_control_queue_proto_text(APP_PROTO_MSG_PID_RECORD,
                                 "PID axis=roll kp=%s ki=0 kd=%s source=coax\r\n",
                                 kp_text,
                                 kd_text);

    (void)DRV_COAX_CTRL_GetParam("coax.pitch_angle_kp", &kp);
    (void)DRV_COAX_CTRL_GetParam("coax.pitch_rate_kd", &kd);
    app_control_format_float(kp, kp_text, (uint32_t)sizeof(kp_text));
    app_control_format_float(kd, kd_text, (uint32_t)sizeof(kd_text));
    app_control_queue_proto_text(APP_PROTO_MSG_PID_RECORD,
                                 "PID axis=pitch kp=%s ki=0 kd=%s source=coax\r\n",
                                 kp_text,
                                 kd_text);

    (void)DRV_COAX_CTRL_GetParam("coax.yaw_angle_kp", &kp);
    (void)DRV_COAX_CTRL_GetParam("coax.yaw_rate_kd", &kd);
    app_control_format_float(kp, kp_text, (uint32_t)sizeof(kp_text));
    app_control_format_float(kd, kd_text, (uint32_t)sizeof(kd_text));
    app_control_queue_proto_text(APP_PROTO_MSG_PID_RECORD,
                                 "PID axis=yaw kp=%s ki=0 kd=%s source=coax\r\n",
                                 kp_text,
                                 kd_text);
}

static void app_control_handle_param(char **tokens, uint32_t count)
{
    const char *name;
    const char *value_text;
    float value;
    char formatted[24];

    if ((count == 1U) || ((count >= 2U) && (strcmp(tokens[1], "?") == 0))) {
        app_control_report_params();
        return;
    }

    if (strcmp(tokens[1], "GET") == 0) {
        app_control_report_params();
        return;
    }

    if (strcmp(tokens[1], "SET") != 0) {
        APP_Control_QueueText("ERR usage PARAM SET coax.name value\r\n");
        return;
    }

    if (count >= 4U) {
        name = tokens[2];
        value_text = tokens[3];
    } else {
        name = app_control_token_value(tokens, count, "name");
        value_text = app_control_token_value(tokens, count, "value");
        if (value_text == NULL) {
            value_text = app_control_token_value(tokens, count, "val");
        }
    }

    if ((name == NULL) ||
        (value_text == NULL) ||
        (app_control_parse_f32(value_text, &value) == 0U)) {
        APP_Control_QueueText("ERR usage PARAM SET coax.name value\r\n");
        return;
    }

    if (DRV_COAX_CTRL_SetParam(name, value) == 0U) {
        APP_Control_QueueText("ERR param target %s\r\n", name);
        return;
    }
    app_control_schedule_flash_autosave();

    app_control_format_float(value, formatted, (uint32_t)sizeof(formatted));
    APP_Control_QueueText("OK param name=%s value=%s\r\n", name, formatted);
    app_control_report_coax_param_by_name(name);
    app_control_report_pid_legacy();
}

void APP_Control_Init(void)
{
    APP_FlashService_Status load_status;

    if (control_initialized != 0U) {
        return;
    }

    app_control_defaults(&control_config);
    control_wifi_reset_pending = 0U;
    control_wifi_reset_deadline_ms = 0U;
    APP_Ident_Init();
    load_status = app_control_load_config();
    control_config.last_flash_status = (uint8_t)load_status;
    if (load_status != APP_FLASH_SERVICE_OK) {
        control_config.loaded_from_flash = 0U;
        control_config.flash_valid = 0U;
    }

#if (APP_CONTROL_BOOT_READY_ENABLED != 0U)
    APP_Control_QueueText("READY drone-H743 tcp-control servo_slots=2 cfg_loaded=%u cfg_valid=%u\r\n",
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
    app_control_service_wifi_reset();
    app_control_service_flash_autosave();
    app_control_ident_step();

    if (emit_heartbeat == 0U) {
        return;
    }

#if (APP_CONTROL_HEARTBEAT_ENABLED != 0U)
    {
        uint32_t now_ms = HAL_GetTick();
        uint32_t uart_rx_bytes = 0U;
        uint32_t uart_rx_lines = 0U;
        uint32_t uart_rx_overflows = 0U;
        uint32_t uart_rx_errors = 0U;
        uint32_t uart_rx_events = 0U;
        uint32_t uart_rx_restarts = 0U;
        uint32_t uart_last_rx_event_size = 0U;

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
        APP_Control_QueueText("READY ms=%lu servo0_id=%u servo1_id=%u cfg_valid=%u wifi=%u wifi_last=%u wifi_writes=%lu trans=%u rx_bytes=%lu rx_lines=%lu rx_ovf=%lu rx_err=%lu rx_evt=%lu rx_rst=%lu rx_evt_size=%lu\r\n",
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

    if ((emit_ack != 0U) && (APP_CONTROL_ASCII_ACK_ENABLED != 0U)) {
        APP_Control_QueueText("ACK %s\r\n", tokens[0]);
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
    } else if (strcmp(tokens[0], "RTOS?") == 0) {
        app_control_report_rtos();
    } else if (strcmp(tokens[0], "FLASH?") == 0) {
        app_control_report_flash();
    } else if (strcmp(tokens[0], "FLASH") == 0) {
        app_control_handle_flash(tokens, count);
    } else if (strcmp(tokens[0], "BARO?") == 0) {
        app_control_report_baro();
    } else if (strcmp(tokens[0], "BARO") == 0) {
        app_control_handle_baro(tokens, count);
    } else if (strcmp(tokens[0], "IMU?") == 0) {
        app_control_report_imu();
    } else if (strcmp(tokens[0], "FLOW?") == 0) {
        APP_OpticalFlow_Report();
    } else if (strcmp(tokens[0], "FLOW") == 0) {
        app_control_handle_flow(tokens, count);
    } else if (strcmp(tokens[0], "RANGE?") == 0) {
        APP_Rangefinder_Report();
    } else if (strcmp(tokens[0], "GPS?") == 0) {
        APP_GPS_Report();
    } else if (strcmp(tokens[0], "MAG?") == 0) {
        APP_MAG_Report();
    } else if (strcmp(tokens[0], "PARAM?") == 0) {
        app_control_report_params();
    } else if (strcmp(tokens[0], "INDI?") == 0) {
        app_control_report_indi();
    } else if (strcmp(tokens[0], "AIRFRAME?") == 0) {
        app_control_report_airframe();
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
            APP_Control_QueueText("ERR usage WIFI_EN 0|1\r\n");
            return;
        }
        wifi_tokens[2] = tokens[1];
        app_control_handle_wifi(wifi_tokens, 3U);
    } else if (strcmp(tokens[0], "SAVE") == 0) {
        APP_FlashService_Status save_status = app_control_save_config();
        control_config.last_flash_status = (uint8_t)save_status;
        if (save_status == APP_FLASH_SERVICE_OK) {
            control_config.loaded_from_flash = 1U;
            control_config.flash_valid = 1U;
        }
        APP_Control_QueueText("OK save st=%u\r\n", (unsigned int)save_status);
    } else if (strcmp(tokens[0], "LOAD") == 0) {
        APP_FlashService_Status load_status = app_control_load_config();
        control_config.last_flash_status = (uint8_t)load_status;
        APP_Control_QueueText("OK load st=%u\r\n", (unsigned int)load_status);
    } else if (strcmp(tokens[0], "DEFAULTS") == 0) {
        app_control_defaults(&control_config);
        APP_Control_QueueText("OK defaults\r\n");
    } else if (strcmp(tokens[0], "PARAM") == 0) {
        app_control_handle_param(tokens, count);
    } else if (strcmp(tokens[0], "PID") == 0) {
        app_control_handle_pid(tokens, count);
    } else if (strcmp(tokens[0], "MOTOR?") == 0) {
        app_control_report_motor();
    } else if (strcmp(tokens[0], "MOTOR") == 0) {
        app_control_handle_motor(tokens, count);
    } else if (strcmp(tokens[0], "IDENT?") == 0) {
        app_control_report_ident();
    } else if (strcmp(tokens[0], "IDENT") == 0) {
        app_control_handle_ident(tokens, count);
    } else if (strcmp(tokens[0], "PWM?") == 0) {
        app_control_report_pwm();
    } else if (strncmp(tokens[0], "PWM", 3) == 0) {
        uint32_t channel;
        uint32_t percent;

        if ((app_control_parse_vofa_pwm(tokens[0], &channel, &percent) != 0U) &&
            (channel >= 1U) && (channel <= BSP_PWM_ESC_CHANNEL_COUNT) &&
            (percent <= BSP_PWM_ESC_MAX_PERCENT)) {
            if ((percent != 0U) && (APP_CONTROL_ALLOW_RAW_PWM_COMMANDS == 0U)) {
                APP_Control_QueueText("ERR raw pwm disabled; use ARM switch\r\n");
            } else {
                BSP_PWM_Status status = (percent == 0U) ?
                    BSP_PWM_DisableEsc(channel) :
                    BSP_PWM_SetEscPercent(channel, percent);
                uint16_t pulse = BSP_PWM_GetEscPulse(channel);

                APP_Control_QueueText("OK pwm%lu st=%u pct=%lu pulse=%u\r\n",
                                      (unsigned long)channel,
                                      (unsigned int)status,
                                      (unsigned long)percent,
                                      (unsigned int)pulse);
            }
        } else {
            APP_Control_QueueText("ERR usage PWM1..2:0..100\r\n");
        }
    } else if (strncmp(tokens[0], "Servor", 6) == 0) {
        unsigned int parsed_index;
        unsigned int parsed_angle;
        uint32_t vofa_index;
        uint32_t index;
        uint32_t angle;
        if ((sscanf(tokens[0], "Servor%u:%u", &parsed_index, &parsed_angle) == 2) &&
            ((vofa_index = (uint32_t)parsed_index) >= 1U) &&
            (vofa_index <= APP_CONTROL_SERVO_COUNT) &&
            ((angle = (uint32_t)parsed_angle) <= 180U)) {
            uint16_t pulse;
            BSP_BusServoStatus status;

            index = vofa_index - 1U;
            pulse = app_control_servo_angle_to_pulse(angle);

            control_config.servo[index].pulse_us =
                app_control_servo_clamp_pulse(index, pulse);
            status = BSP_BusServo_Move(control_config.servo[index].id,
                                       control_config.servo[index].pulse_us,
                                       control_config.servo[index].time_ms);
            APP_Control_QueueText("OK servo%lu vofa_angle st=%u id=%u deg=%u pulse=%u\r\n",
                                  (unsigned long)index,
                                  (unsigned int)status,
                                  (unsigned int)control_config.servo[index].id,
                                  (unsigned int)angle,
                                  (unsigned int)control_config.servo[index].pulse_us);
        }
    } else if (strcmp(tokens[0], "SERVO") == 0) {
        app_control_handle_servo(tokens, count);
    } else if ((strcmp(tokens[0], "FLOG?") == 0) ||
               (strcmp(tokens[0], "FLOG") == 0)) {
        app_control_handle_flight_log(tokens, count);
    } else if (strcmp(tokens[0], "Sensor_Data:1") == 0) {
        vofaStreamActive = 1U;
        APP_Control_QueueText("OK IMU stream started\r\n");
    } else if (strcmp(tokens[0], "Sensor_Data:0") == 0) {
        vofaStreamActive = 0U;
        APP_Control_QueueText("OK IMU stream stopped\r\n");
    } else {
        APP_Control_QueueText("ERR unknown cmd %s\r\n", tokens[0]);
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

    if (app_control_handle_pid_slider_line(line) != 0U) {
        return;
    }

    if (app_control_handle_param_value_line(line) != 0U) {
        return;
    }

    if (APP_CONTROL_ASCII_RX_ECHO_ENABLED != 0U) {
        APP_Control_QueueText("RX %s\r\n", line);
    }

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

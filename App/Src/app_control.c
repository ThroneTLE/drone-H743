#include "app_control.h"

#include "app_aiwb2.h"
#include "app_acceptance.h"
#include "app_baro.h"
#include "app_boot.h"
#include "app_flash.h"
#include "app_firmware_identity.h"
#include "app_flight_calibration.h"
#include "app_flight_log.h"
#include "app_imu_capture.h"
#include "app_diag.h"
#include "app_gps.h"
#include "app_ident.h"
#include "app_optical_flow.h"
#include "app_rangefinder.h"
#include "app_servo_cal.h"
#include "app_servo_jog.h"
#include "app_servo_feedback.h"
#include "app_servo_feedback_bench.h"
#include "app_sensor.h"
#include "app_elrs.h"
#include "app_rc_config.h"
#include "app_stabilizer.h"
#include "app_mag.h"
#include "app_maint_uart.h"
#include "app_messages.h"
#include "app_nav_estimator.h"
#include "app_proto.h"
#include "app_tasks.h"
#include "app_telemetry.h"
#include "app_uart.h"
#include "app_usb_cdc.h"
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
#include "drv_frame_contract.h"
#include "drv_motor.h"
#include "svc_param.h"
#include "svc_timestamp.h"

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
#define APP_CONTROL_CFG_VERSION     17U
#define APP_CONTROL_CFG_VERSION_V16 16U
#define APP_CONTROL_CFG_VERSION_V15 15U
#define APP_CONTROL_CFG_ADDRESS     (APP_FLASH_SERVICE_SIZE_BYTES - 4096UL)
#define APP_CONTROL_MAX_LINE        128U
/*
 * 一条 IMU? 会连发 8 行文本，上位机必须把其中两行（provenance + 采样值）凑成同一
 * 个 seq 才算一份有效快照。之前这里只给 2ms：HAL_GetTick 是 1ms 粒度，实际预算只有
 * 1~2 个 USB 帧，主机稍微晚一帧收包整行就被丢掉，而返回值又被 (void) 吃掉。结果是
 * 快照永远凑不齐、validation_latest_host_time 永远是 0，界面显示"安全快照已过期"。
 * 同一个任务里 IMUCAP 导出本来就按 50ms/块阻塞，所以 10ms 在该任务的时间尺度内。
 *
 * 契约：APP_Control_QueueText / app_control_queue_proto_text 只允许在通信任务
 * 上下文（APP_UART_Task_Step -> APP_Control_Tick 及命令分发）调用——它们会同步
 * 阻塞等 USB CDC，最坏 3x 本超时。控制环模块（如 app_servo_cal 的 500Hz 状态机）
 * 一律改置事件标志，由 app_control_tick_common 里的 notice 服务补发。
 */
#define APP_CONTROL_USB_TEXT_TX_TIMEOUT_MS 10U
/* 与 STABILIZER_RC_LOSS_TIMEOUT_MS 一致：上位机看到的 fresh 要和控制环判定同源。 */
#define APP_CONTROL_RC_FRESH_TIMEOUT_MS 500U
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
#define APP_CONTROL_TILT_LIMIT_MAX_DEG 28.0f
#define APP_CONTROL_TILT_LIMIT_DEFAULT_RAD 0.4886922f
#define APP_CONTROL_TILT_LIMIT_LEGACY_18_RAD 0.31415927f
#define APP_CONTROL_TILT_LIMIT_LEGACY_25_RAD 0.43633231f
#define APP_CONTROL_TILT_LIMIT_LEGACY_EPS_RAD 0.001f
#define APP_CONTROL_FLOW_RAW_MAX_BYTES 32U
#define APP_CONTROL_IMUCAL_SNAPSHOT_MAX_AGE_US 100000ULL
#define APP_CONTROL_IMUCAL_ESC_SAFE_MAX_US 1100U

typedef struct {
    float pos_x_kp;
    float pos_y_kp;
    float pos_z_kp;
    float pos_z_ki;
    float vel_x_kd;
    float vel_y_kd;
    float vel_z_kd;
    float vel_loop_enable;
    float vel_loop_x_kp;
    float vel_loop_x_ki;
    float vel_loop_x_kd;
    float vel_loop_y_kp;
    float vel_loop_y_ki;
    float vel_loop_y_kd;
    float roll_angle_kp;
    float pitch_angle_kp;
    float roll_rate_kd;
    float pitch_rate_kd;
    float tilt_limit_rad;
    float yaw_angle_kp;
    float yaw_rate_kd;
} APP_ControlCoaxTunableParams;

typedef struct {
    float pos_x_kp;
    float pos_y_kp;
    float pos_z_kp;
    float vel_x_kd;
    float vel_y_kd;
    float vel_z_kd;
    float vel_loop_enable;
    float vel_loop_x_kp;
    float vel_loop_x_ki;
    float vel_loop_x_kd;
    float vel_loop_y_kp;
    float vel_loop_y_ki;
    float vel_loop_y_kd;
    float roll_angle_kp;
    float pitch_angle_kp;
    float roll_rate_kd;
    float pitch_rate_kd;
    float tilt_limit_rad;
    float yaw_angle_kp;
    float yaw_rate_kd;
} APP_ControlCoaxTunableParamsV15;

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    APP_ControlConfig config;
    APP_ControlCoaxTunableParams coax_tunables;
    APP_RcConfig rc_config;
    uint32_t checksum;
} APP_ControlFlashRecord;

/* V16 = 加入 rc_config 之前的布局，仅用于迁移读取，不要再往里加字段。 */
typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    APP_ControlConfig config;
    APP_ControlCoaxTunableParams coax_tunables;
    uint32_t checksum;
} APP_ControlFlashRecordV16;

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    APP_ControlConfig config;
    APP_ControlCoaxTunableParamsV15 coax_tunables;
    uint32_t checksum;
} APP_ControlFlashRecordV15;

static APP_ControlConfig control_config;
/*
 * RAM 中的遥控映射工作副本。RCMAP SET 只改这里并立刻 Publish（所见即所得，
 * 上位机能马上在实时条上看到反向/端点的效果），RCMAP COMMIT 才落 Flash。
 */
static APP_RcConfig control_rc_config;
static uint8_t control_rc_config_dirty;
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
static uint8_t control_imuframe_confirmed_code;
static uint8_t control_imuframe_pending_code;
static uint8_t control_imuframe_pending_valid;
static uint8_t control_imuframe_generation_valid;
static uint8_t control_imuframe_last_dirty;
static uint8_t control_imuframe_boot_selection_pending;
static uint32_t control_imuframe_param_generation;
static uint32_t control_imuframe_last_request;
static APP_FlightCalibrationUpload control_imucal_upload;
static APP_FlightCalibration control_imucal_confirmed;
static APP_FlightCalibration control_imucal_preview;
static APP_FlightCalibration control_imucal_pending_record;
static uint32_t control_imucal_confirmed_generation;
static uint32_t control_imucal_preview_generation;
static uint32_t control_imucal_apply_sequence;
static uint32_t control_imucal_last_request;
static uint8_t control_imucal_confirmed_valid;
static uint8_t control_imucal_applied;
static uint8_t control_imucal_commit_pending;
static const char *control_imucal_last_event;
static const char *control_imucal_last_reason;
static APP_FlightCalibration control_servocal_preview;
static APP_FlightCalibration control_servocal_pending_record;
static uint32_t control_servocal_preview_generation;
static uint32_t control_servocal_last_request;
static uint8_t control_servocal_applied;
static uint8_t control_servocal_commit_pending;
static const char *control_servocal_last_event;
static const char *control_servocal_last_reason;
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
static void app_control_report_usb_cdc_stats(void);
static void app_control_apply_rc_config(const APP_RcConfig *config);
static APP_FlashService_Status app_control_save_config(void);
void APP_Control_QueueText(const char *format, ...);
static void app_control_queue_proto_text(uint16_t function, const char *format, ...);
static uint8_t app_control_send_boot_scheduled(void);
static void app_control_handle_flight_log(char **tokens, uint32_t count);
static void app_control_handle_imu_capture(char **tokens, uint32_t count);
static void app_control_dispatch_tokens(char **tokens, uint32_t count, uint8_t emit_ack);
static uint32_t app_control_tokenize(char *buffer, char **tokens, uint32_t max_tokens);
static uint8_t app_control_parse_u32(const char *text, uint32_t *value);
static uint8_t app_control_parse_i32(const char *text, int32_t *value);
static const char *app_control_token_value(char **tokens,
                                           uint32_t count,
                                           const char *key);
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
static void app_control_handle_boot(char **tokens, uint32_t count);
static void app_control_service_boot(void);
static void app_control_handle_acceptance(char **tokens, uint32_t count);
static void app_control_report_acceptance(void);
static void app_control_handle_imuframe(char **tokens, uint32_t count);
static void app_control_imuframe_sync_param(void);
static void app_control_report_imuframe(const char *event, uint32_t request_id);
static void app_control_report_imucal(void);
static void app_control_handle_imucal(char **tokens, uint32_t count);
static void app_control_service_imucal(void);
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

    (void)APP_USB_CDC_Write((const uint8_t *)tx_message.text,
                            tx_message.length,
                            APP_CONTROL_USB_TEXT_TX_TIMEOUT_MS);

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

static void app_control_imucal_clear_candidate(void)
{
    APP_FlightCalibration_UploadReset(&control_imucal_upload);
    memset(&control_imucal_preview, 0, sizeof(control_imucal_preview));
    memset(&control_imucal_pending_record, 0,
           sizeof(control_imucal_pending_record));
    control_imucal_preview_generation = 0U;
    control_imucal_apply_sequence = 0U;
    control_imucal_applied = 0U;
    control_imucal_commit_pending = 0U;
    APP_Stabilizer_SetImuCalibrationCandidateArmLock(0U);
}

static void app_control_imucal_set_event(const char *event,
                                         const char *reason)
{
    control_imucal_last_event = (event != NULL) ? event : "status";
    control_imucal_last_reason = (reason != NULL) ? reason : "none";
}

static void app_control_servocal_clear_preview(void)
{
    memset(&control_servocal_preview, 0, sizeof(control_servocal_preview));
    memset(&control_servocal_pending_record, 0,
           sizeof(control_servocal_pending_record));
    control_servocal_preview_generation = 0U;
    control_servocal_applied = 0U;
    control_servocal_commit_pending = 0U;
    APP_Stabilizer_SetServoCalibrationCandidateArmLock(0U);
}

static void app_control_servocal_set_event(const char *event,
                                           const char *reason)
{
    control_servocal_last_event = (event != NULL) ? event : "status";
    control_servocal_last_reason = (reason != NULL) ? reason : "none";
}

/*
 * Observe the service generation both during boot and at runtime.  Only the
 * first confirmed generation selects the active sensor transform.  A dirty
 * service blob is only pending: it must not replace the last Flash-confirmed
 * code until the background save has completed and dirty returns to zero.
 * Later confirmed changes update the persisted view but never overwrite a
 * temporary APPLY; active changes exclusively through IMUFRAME commands.
 */
static void app_control_imuframe_sync_param(void)
{
    APP_FlightCalibration calibration;
    APP_FlightCalibrationSnapshot active_snapshot;
    uint8_t blob[sizeof(APP_FlightCalibration)];
    APP_FlightCalibrationDecodeStatus decode_status;
    SVC_ParamStatus status;
    uint32_t generation;
    uint32_t size = 0U;
    uint8_t dirty;
    uint8_t orientation_code;

    if (SVC_Param_IsReady() == 0U) {
        return;
    }

    generation = SVC_Param_GetGeneration();
    dirty = SVC_Param_IsDirty();
    if ((control_imuframe_generation_valid != 0U) &&
        (generation == control_imuframe_param_generation) &&
        !((control_imuframe_last_dirty != 0U) && (dirty == 0U))) {
        return;
    }

    APP_FlightCalibration_Defaults(&calibration);
    memset(blob, 0, sizeof(blob));
    status = SVC_Param_GetBlob(blob, sizeof(blob), &size);
    decode_status = (status == SVC_PARAM_STATUS_OK) ?
        APP_FlightCalibration_Decode(blob, size, &calibration) :
        APP_FLIGHT_CAL_DECODE_INVALID;
    if (decode_status == APP_FLIGHT_CAL_DECODE_INVALID) {
        /* Invalid/no record is a safe, explicit no-calibration snapshot. */
        APP_FlightCalibration_Defaults(&calibration);
    }
    orientation_code = calibration.orientation_code;

    control_imuframe_param_generation = generation;
    control_imuframe_generation_valid = 1U;
    control_imuframe_last_dirty = dirty;

    if (dirty != 0U) {
        return;
    }

    /* This is the Flash-confirmed publication point; RAM preview is separate. */
    if (APP_FlightCalibration_PublishConfirmed(&calibration) == 0U) {
        return;
    }

    control_imucal_confirmed = calibration;
    control_imucal_confirmed_valid = 1U;
    if (APP_FlightCalibration_ReadActive(&active_snapshot) != 0U) {
        control_imucal_confirmed_generation = active_snapshot.generation;
    }

    if (control_imucal_commit_pending != 0U) {
        if (memcmp(&calibration,
                   &control_imucal_pending_record,
                   sizeof(calibration)) == 0) {
            app_control_imucal_set_event("committed", "none");
        } else {
            app_control_imucal_set_event("commit_failed", "record_mismatch");
        }
        app_control_imucal_clear_candidate();
    } else if (control_imucal_applied != 0U) {
        /* A separately confirmed Param change invalidates the RAM preview. */
        app_control_imucal_set_event("reverted", "persisted_changed");
        app_control_imucal_clear_candidate();
    }

    if (control_servocal_commit_pending != 0U) {
        if (memcmp(&calibration,
                   &control_servocal_pending_record,
                   sizeof(calibration)) == 0) {
            app_control_servocal_set_event("committed", "none");
        } else {
            app_control_servocal_set_event("commit_failed", "record_mismatch");
        }
        app_control_servocal_clear_preview();
    } else if (control_servocal_applied != 0U) {
        app_control_servocal_set_event("reverted", "persisted_changed");
        app_control_servocal_clear_preview();
    }

    control_imuframe_confirmed_code = orientation_code;
    control_imuframe_pending_code = orientation_code;
    control_imuframe_pending_valid = 0U;

    if (control_imuframe_boot_selection_pending != 0U) {
        (void)APP_Sensor_SetFluOrientationCode(orientation_code);
        control_imuframe_boot_selection_pending = 0U;
    }
}

static void app_control_report_imucal(void)
{
    APP_FlightCalibrationSnapshot snapshot;
    APP_FirmwareIdentity firmware_identity;
    DRV_IMU_Calibration imu_calibration;
    const APP_FlightCalibrationV1Candidate *candidate =
        &control_imucal_upload.candidate;
    uint8_t active_orientation = APP_Sensor_GetFluOrientation();
    uint8_t effective_mask;
    uint8_t candidate_ready =
        (control_imucal_upload.state == APP_FLIGHT_CAL_UPLOAD_READY) ? 1U : 0U;

    if ((APP_FlightCalibration_ReadActive(&snapshot) == 0U) ||
        (APP_FirmwareIdentity_Get(&firmware_identity) == 0U)) {
        app_control_queue_proto_text(
            APP_PROTO_MSG_IMU_CAL,
            "IMUCAL valid=0 source=active_snapshot unavailable\r\n");
        return;
    }
    effective_mask = APP_FlightCalibration_BuildImuCalibration(
        &snapshot.calibration, active_orientation, &imu_calibration);
    app_control_queue_proto_text(
        APP_PROTO_MSG_IMU_CAL,
        "IMUCAL event=%s reason=%s transfer=%s received=%lu expected=%lu "
        "candidate=%u applied=%u commit_pending=%u dirty=%u arm_lock=%u "
        "request=%lu\r\n",
        (control_imucal_last_event != NULL) ?
            control_imucal_last_event : "status",
        (control_imucal_last_reason != NULL) ?
            control_imucal_last_reason : "none",
        APP_FlightCalibration_UploadStateText(control_imucal_upload.state),
        (unsigned long)control_imucal_upload.received_size,
        (unsigned long)control_imucal_upload.expected_size,
        (unsigned int)candidate_ready,
        (unsigned int)control_imucal_applied,
        (unsigned int)control_imucal_commit_pending,
        (unsigned int)SVC_Param_IsDirty(),
        (unsigned int)APP_Stabilizer_IsImuCalibrationCandidateArmLocked(),
        (unsigned long)control_imucal_last_request);
    app_control_queue_proto_text(
        APP_PROTO_MSG_IMU_CAL,
        "IMUCAL generations active=%lu persisted=%lu record=%lu base=%lu "
        "active_mask=0x%02X persisted_mask=0x%02X candidate_mask=0x%02X "
        "active_orientation=%u persisted_orientation=%u candidate_orientation=%u\r\n",
        (unsigned long)snapshot.generation,
        (unsigned long)control_imucal_confirmed_generation,
        (unsigned long)((control_imucal_confirmed_valid != 0U) ?
            control_imucal_confirmed.calibration_generation : 0U),
        (unsigned long)((candidate_ready != 0U) ?
            candidate->base_generation : 0U),
        (unsigned int)snapshot.calibration.valid_mask,
        (unsigned int)((control_imucal_confirmed_valid != 0U) ?
            control_imucal_confirmed.valid_mask : 0U),
        (unsigned int)((candidate_ready != 0U) ? candidate->valid_mask : 0U),
        (unsigned int)active_orientation,
        (unsigned int)((control_imucal_confirmed_valid != 0U) ?
            control_imucal_confirmed.orientation_code :
            APP_SENSOR_FLU_ORIENTATION_LEGACY),
        (unsigned int)((candidate_ready != 0U) ?
            candidate->orientation_code : APP_SENSOR_FLU_ORIENTATION_LEGACY));
    app_control_queue_proto_text(
        APP_PROTO_MSG_IMU_CAL,
        "IMUCAL valid=1 source=active_snapshot schema=%u contract=%u "
        "cal_generation=%lu record_generation=%lu valid_mask=0x%02X "
        "effective_mask=0x%02X orientation=%u active_orientation=%u "
        "dirty=%u\r\n",
        (unsigned int)snapshot.calibration.schema,
        (unsigned int)snapshot.calibration.frame_contract,
        (unsigned long)snapshot.generation,
        (unsigned long)snapshot.calibration.calibration_generation,
        (unsigned int)snapshot.calibration.valid_mask,
        (unsigned int)effective_mask,
        (unsigned int)snapshot.calibration.orientation_code,
        (unsigned int)active_orientation,
        (unsigned int)SVC_Param_IsDirty());
    app_control_queue_proto_text(
        APP_PROTO_MSG_IMU_CAL,
        "IMUCAL identity firmware_crc32=0x%08lX image_bytes=%lu\r\n",
        (unsigned long)firmware_identity.image_crc32,
        (unsigned long)firmware_identity.image_size);
    app_control_queue_proto_text(
        APP_PROTO_MSG_IMU_CAL,
        "IMUCAL accel bias_ug=%ld,%ld,%ld "
        "matrix_ppm=%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld\r\n",
        (long)(snapshot.calibration.accel_bias[0] * 1000000.0f),
        (long)(snapshot.calibration.accel_bias[1] * 1000000.0f),
        (long)(snapshot.calibration.accel_bias[2] * 1000000.0f),
        (long)(snapshot.calibration.accel_correction[0][0] * 1000000.0f),
        (long)(snapshot.calibration.accel_correction[0][1] * 1000000.0f),
        (long)(snapshot.calibration.accel_correction[0][2] * 1000000.0f),
        (long)(snapshot.calibration.accel_correction[1][0] * 1000000.0f),
        (long)(snapshot.calibration.accel_correction[1][1] * 1000000.0f),
        (long)(snapshot.calibration.accel_correction[1][2] * 1000000.0f),
        (long)(snapshot.calibration.accel_correction[2][0] * 1000000.0f),
        (long)(snapshot.calibration.accel_correction[2][1] * 1000000.0f),
        (long)(snapshot.calibration.accel_correction[2][2] * 1000000.0f));
    app_control_queue_proto_text(
        APP_PROTO_MSG_IMU_CAL,
        "IMUCAL gyro residual_bias_mdps=%ld,%ld,%ld "
        "temp_slope_udps_per_c=%ld,%ld,%ld reference_temp_cdeg=%ld\r\n",
        (long)(snapshot.calibration.gyro_bias_ref[0] * 1000.0f),
        (long)(snapshot.calibration.gyro_bias_ref[1] * 1000.0f),
        (long)(snapshot.calibration.gyro_bias_ref[2] * 1000.0f),
        (long)(snapshot.calibration.gyro_temp_slope[0] * 1000000.0f),
        (long)(snapshot.calibration.gyro_temp_slope[1] * 1000000.0f),
        (long)(snapshot.calibration.gyro_temp_slope[2] * 1000000.0f),
        (long)(snapshot.calibration.reference_temp_c * 100.0f));
}

static uint8_t app_control_parse_hex_u32(const char *text, uint32_t *value)
{
    char *end_ptr;
    unsigned long parsed;
    size_t digits;

    if ((text == NULL) || (value == NULL) || (*text == '\0')) {
        return 0U;
    }
    if ((text[0] == '0') && ((text[1] == 'x') || (text[1] == 'X'))) {
        text += 2;
    }
    digits = strlen(text);
    if ((digits == 0U) || (digits > 8U)) {
        return 0U;
    }
    parsed = strtoul(text, &end_ptr, 16);
    if ((end_ptr == text) || (*end_ptr != '\0')) {
        return 0U;
    }
    *value = (uint32_t)parsed;
    return 1U;
}

static const char *app_control_imucal_candidate_context_error(void)
{
    const APP_FlightCalibrationV1Candidate *candidate =
        &control_imucal_upload.candidate;

    if (SVC_Param_IsDirty() != 0U) {
        return "param_dirty";
    }
    if ((control_imucal_confirmed_valid == 0U) ||
        (control_imucal_upload.state != APP_FLIGHT_CAL_UPLOAD_READY)) {
        return "candidate_not_ready";
    }
    if (candidate->base_generation != control_imucal_confirmed_generation) {
        return "base_generation_mismatch";
    }
    if (((control_imucal_confirmed.valid_mask &
          APP_FLIGHT_CAL_VALID_ORIENTATION) == 0U) ||
        (candidate->orientation_code !=
         control_imucal_confirmed.orientation_code) ||
        (candidate->orientation_code != APP_Sensor_GetFluOrientation())) {
        return "orientation_mismatch";
    }
    return NULL;
}

static const char *app_control_imucal_safety(
    StabilizerValidationImuSnapshot *snapshot,
    uint8_t require_sequence_progress)
{
    uint64_t now_us;

    memset(snapshot, 0, sizeof(*snapshot));
    if (APP_Stabilizer_ReadValidationImuSnapshot(snapshot) == 0U) {
        return "snapshot_invalid";
    }
    now_us = SVC_Timestamp_Us();
    if ((snapshot->timestamp_us == 0ULL) ||
        (now_us < snapshot->timestamp_us) ||
        ((now_us - snapshot->timestamp_us) >
         APP_CONTROL_IMUCAL_SNAPSHOT_MAX_AGE_US)) {
        return "snapshot_stale";
    }
    if (snapshot->armed != 0U) {
        return "armed";
    }
    if ((snapshot->esc_pulse_us[0] > APP_CONTROL_IMUCAL_ESC_SAFE_MAX_US) ||
        (snapshot->esc_pulse_us[1] > APP_CONTROL_IMUCAL_ESC_SAFE_MAX_US)) {
        return "esc_high";
    }
    if ((require_sequence_progress != 0U) &&
        (APP_Boot_HasSequenceAdvanced(snapshot->sequence,
                                      control_imucal_apply_sequence) == 0U)) {
        return "sequence_stalled";
    }
    return NULL;
}

static void app_control_imucal_transfer_result(
    const char *event,
    APP_FlightCalibrationTransferStatus status)
{
    const char *reason = APP_FlightCalibration_TransferStatusText(status);

    app_control_imucal_set_event(event,
        (status == APP_FLIGHT_CAL_TRANSFER_OK) ? "none" : reason);
    if ((status != APP_FLIGHT_CAL_TRANSFER_OK) &&
        (control_imucal_applied == 0U) &&
        (control_imucal_commit_pending == 0U) &&
        (control_imucal_upload.state == APP_FLIGHT_CAL_UPLOAD_EMPTY)) {
        APP_Stabilizer_SetImuCalibrationCandidateArmLock(0U);
    }
    app_control_report_imucal();
}

static void app_control_handle_imucal(char **tokens, uint32_t count)
{
    APP_FlightCalibrationTransferStatus transfer_status;
    StabilizerValidationImuSnapshot safety_snapshot;
    APP_FlightCalibrationSnapshot active_snapshot;
    APP_FlightCalibration current_record;
    APP_FlightCalibrationDecodeStatus decode_status;
    uint8_t blob[sizeof(APP_FlightCalibration)];
    uint8_t encoded[sizeof(APP_FlightCalibration)];
    uint32_t blob_size = 0U;
    uint32_t encoded_size;
    uint32_t size;
    uint32_t crc32;
    uint32_t offset;
    const char *size_text;
    const char *crc_text;
    const char *offset_text;
    const char *hex_text;
    const char *context_error;
    const char *safety_error;
    SVC_ParamStatus param_status;
    uint32_t now_ms = HAL_GetTick();

    if ((tokens == NULL) || (count == 0U)) {
        return;
    }
    app_control_imuframe_sync_param();

    if (strcmp(tokens[0], "IMUCAL?") == 0) {
        if (count != 1U) {
            APP_Control_QueueText("ERR usage IMUCAL?\r\n");
            return;
        }
        app_control_imucal_set_event("status", "none");
        app_control_report_imucal();
        return;
    }
    if ((count < 2U) || (strcmp(tokens[0], "IMUCAL") != 0)) {
        APP_Control_QueueText(
            "ERR usage IMUCAL BEGIN|DATA|END|APPLY|REVERT|COMMIT\r\n");
        return;
    }
    if ((control_servocal_applied != 0U) ||
        (control_servocal_commit_pending != 0U)) {
        app_control_imucal_set_event("rejected", "servocal_busy");
        app_control_report_imucal();
        return;
    }

    if (strcmp(tokens[1], "BEGIN") == 0) {
        size_text = app_control_token_value(tokens, count, "size");
        crc_text = app_control_token_value(tokens, count, "crc");
        if ((count != 4U) || (size_text == NULL) || (crc_text == NULL) ||
            (app_control_parse_u32(size_text, &size) == 0U) ||
            (app_control_parse_hex_u32(crc_text, &crc32) == 0U)) {
            APP_Control_QueueText(
                "ERR usage IMUCAL BEGIN size=<n> crc=<8hex>\r\n");
            return;
        }
        if ((control_imucal_applied != 0U) ||
            (control_imucal_commit_pending != 0U)) {
            app_control_imucal_set_event("begin_rejected", "candidate_applied");
            app_control_report_imucal();
            return;
        }
        if (SVC_Param_IsDirty() != 0U) {
            app_control_imucal_set_event("begin_rejected", "param_dirty");
            app_control_report_imucal();
            return;
        }
        /* Publish the hard arm lock before the candidate state becomes live. */
        APP_Stabilizer_SetImuCalibrationCandidateArmLock(1U);
        transfer_status = APP_FlightCalibration_UploadBegin(
            &control_imucal_upload, size, crc32, now_ms);
        app_control_imucal_transfer_result("begin", transfer_status);
        return;
    }

    if (strcmp(tokens[1], "DATA") == 0) {
        offset_text = app_control_token_value(tokens, count, "offset");
        hex_text = app_control_token_value(tokens, count, "hex");
        if ((count != 4U) || (offset_text == NULL) || (hex_text == NULL) ||
            (app_control_parse_u32(offset_text, &offset) == 0U)) {
            APP_Control_QueueText(
                "ERR usage IMUCAL DATA offset=<n> hex=<max64hex>\r\n");
            return;
        }
        if (SVC_Param_IsDirty() != 0U) {
            APP_FlightCalibration_UploadReset(&control_imucal_upload);
            APP_Stabilizer_SetImuCalibrationCandidateArmLock(0U);
            app_control_imucal_set_event("data_rejected", "param_dirty");
            app_control_report_imucal();
            return;
        }
        transfer_status = APP_FlightCalibration_UploadDataHex(
            &control_imucal_upload, offset, hex_text, now_ms);
        app_control_imucal_transfer_result("data", transfer_status);
        return;
    }

    if (strcmp(tokens[1], "END") == 0) {
        if (count != 2U) {
            APP_Control_QueueText("ERR usage IMUCAL END\r\n");
            return;
        }
        transfer_status = APP_FlightCalibration_UploadEnd(
            &control_imucal_upload, now_ms);
        if (transfer_status != APP_FLIGHT_CAL_TRANSFER_OK) {
            app_control_imucal_transfer_result("end", transfer_status);
            return;
        }
        context_error = app_control_imucal_candidate_context_error();
        if (context_error != NULL) {
            APP_FlightCalibration_UploadReset(&control_imucal_upload);
            APP_Stabilizer_SetImuCalibrationCandidateArmLock(0U);
            app_control_imucal_set_event("end_rejected", context_error);
            app_control_report_imucal();
            return;
        }
        app_control_imucal_set_event("ready", "none");
        app_control_report_imucal();
        return;
    }

    if (strcmp(tokens[1], "APPLY") == 0) {
        /*
         * This command is transport plus target-side safety only. Host code
         * must already have accepted validate_candidate_for_application(); a
         * successful APPLY is never evidence that V1 metrology passed.
         */
        if (count != 2U) {
            APP_Control_QueueText("ERR usage IMUCAL APPLY\r\n");
            return;
        }
        if ((control_imucal_applied != 0U) ||
            (control_imucal_commit_pending != 0U)) {
            app_control_imucal_set_event("apply_rejected", "bad_state");
            app_control_report_imucal();
            return;
        }
        context_error = app_control_imucal_candidate_context_error();
        if (context_error != NULL) {
            app_control_imucal_set_event("apply_rejected", context_error);
            app_control_report_imucal();
            return;
        }
        safety_error = app_control_imucal_safety(&safety_snapshot, 0U);
        if (safety_error != NULL) {
            app_control_imucal_set_event("apply_rejected", safety_error);
            app_control_report_imucal();
            return;
        }
        if (APP_FlightCalibration_MergeV1Candidate(
                &control_imucal_confirmed,
                &control_imucal_upload.candidate,
                &control_imucal_preview) == 0U) {
            app_control_imucal_set_event("apply_rejected", "merge_invalid");
            app_control_report_imucal();
            return;
        }
        if ((APP_FlightCalibration_PublishPreview(
                 &control_imucal_preview) == 0U) ||
            (APP_FlightCalibration_ReadActive(&active_snapshot) == 0U)) {
            app_control_imucal_set_event("apply_rejected", "publish_failed");
            app_control_report_imucal();
            return;
        }
        control_imucal_preview_generation = active_snapshot.generation;
        control_imucal_apply_sequence = safety_snapshot.sequence;
        control_imucal_applied = 1U;
        APP_Stabilizer_SetImuCalibrationCandidateArmLock(1U);
        app_control_imucal_set_event("applied", "none");
        app_control_report_imucal();
        return;
    }

    if (strcmp(tokens[1], "REVERT") == 0) {
        if (count != 2U) {
            APP_Control_QueueText("ERR usage IMUCAL REVERT\r\n");
            return;
        }
        if (SVC_Param_IsDirty() != 0U) {
            app_control_imucal_set_event("revert_rejected", "param_dirty");
            app_control_report_imucal();
            return;
        }
        safety_error = app_control_imucal_safety(&safety_snapshot, 0U);
        if (safety_error != NULL) {
            app_control_imucal_set_event("revert_rejected", safety_error);
            app_control_report_imucal();
            return;
        }
        if ((control_imucal_confirmed_valid == 0U) ||
            (APP_FlightCalibration_PublishPreview(
                 &control_imucal_confirmed) == 0U) ||
            (APP_FlightCalibration_ReadActive(&active_snapshot) == 0U)) {
            app_control_imucal_set_event("revert_rejected", "publish_failed");
            app_control_report_imucal();
            return;
        }
        control_imucal_confirmed_generation = active_snapshot.generation;
        app_control_imucal_clear_candidate();
        app_control_imucal_set_event("reverted", "none");
        app_control_report_imucal();
        return;
    }

    if (strcmp(tokens[1], "COMMIT") == 0) {
        if (count != 2U) {
            APP_Control_QueueText("ERR usage IMUCAL COMMIT\r\n");
            return;
        }
        if ((control_imucal_applied == 0U) ||
            (control_imucal_commit_pending != 0U) ||
            (control_imucal_upload.state != APP_FLIGHT_CAL_UPLOAD_READY)) {
            app_control_imucal_set_event("commit_rejected", "not_applied");
            app_control_report_imucal();
            return;
        }
        context_error = app_control_imucal_candidate_context_error();
        if (context_error != NULL) {
            app_control_imucal_set_event("commit_rejected", context_error);
            app_control_report_imucal();
            return;
        }
        safety_error = app_control_imucal_safety(&safety_snapshot, 1U);
        if (safety_error != NULL) {
            app_control_imucal_set_event("commit_rejected", safety_error);
            app_control_report_imucal();
            return;
        }
        if ((APP_FlightCalibration_ReadActive(&active_snapshot) == 0U) ||
            (active_snapshot.generation != control_imucal_preview_generation) ||
            (memcmp(&active_snapshot.calibration,
                    &control_imucal_preview,
                    sizeof(control_imucal_preview)) != 0) ||
            (safety_snapshot.calibration_generation !=
             control_imucal_preview_generation)) {
            app_control_imucal_set_event("commit_rejected", "preview_not_observed");
            app_control_report_imucal();
            return;
        }
        memset(blob, 0, sizeof(blob));
        param_status = SVC_Param_GetBlob(blob, sizeof(blob), &blob_size);
        decode_status = (param_status == SVC_PARAM_STATUS_OK) ?
            APP_FlightCalibration_Decode(blob, blob_size, &current_record) :
            APP_FLIGHT_CAL_DECODE_INVALID;
        if ((decode_status == APP_FLIGHT_CAL_DECODE_INVALID) ||
            (memcmp(&current_record,
                    &control_imucal_confirmed,
                    sizeof(current_record)) != 0) ||
            (APP_FlightCalibration_MergeV1Candidate(
                 &current_record,
                 &control_imucal_upload.candidate,
                 &control_imucal_pending_record) == 0U)) {
            app_control_imucal_set_event("commit_rejected", "persisted_base_changed");
            app_control_report_imucal();
            return;
        }
        encoded_size = APP_FlightCalibration_Encode(
            &control_imucal_pending_record, encoded, sizeof(encoded));
        if (encoded_size == 0U) {
            app_control_imucal_set_event("commit_rejected", "encode_failed");
            app_control_report_imucal();
            return;
        }
        param_status = SVC_Param_SetBlob(encoded, encoded_size);
        if (param_status != SVC_PARAM_STATUS_OK) {
            app_control_imucal_set_event("commit_rejected", "set_blob_failed");
            app_control_report_imucal();
            return;
        }
        control_imucal_commit_pending = 1U;
        control_imucal_last_request = SVC_Param_RequestSaveBlob();
        app_control_imucal_set_event("commit_queued",
            (control_imucal_last_request != 0U) ? "none" : "queue_retry");
        app_control_report_imucal();
        return;
    }

    APP_Control_QueueText(
        "ERR usage IMUCAL BEGIN|DATA|END|APPLY|REVERT|COMMIT\r\n");
}

static void app_control_service_imucal(void)
{
    if ((control_imucal_applied == 0U) &&
        (control_imucal_commit_pending == 0U) &&
        (APP_FlightCalibration_UploadExpire(&control_imucal_upload,
                                             HAL_GetTick()) != 0U)) {
        APP_Stabilizer_SetImuCalibrationCandidateArmLock(0U);
        app_control_imucal_set_event("expired", "timeout");
    }
    if ((control_imucal_commit_pending != 0U) &&
        (SVC_Param_IsDirty() != 0U) &&
        (control_imucal_last_request == 0U)) {
        control_imucal_last_request = SVC_Param_RequestSaveBlob();
    }
}

static void app_control_report_servocal_record(
    const char *scope,
    const APP_FlightCalibration *record,
    uint32_t runtime_generation)
{
    DRV_COAX_CTRL_ServoCalibration calibration;
    uint8_t valid = 0U;

    DRV_COAX_CTRL_GetDefaultServoCalibration(&calibration);
    if (record != NULL) {
        valid = APP_FlightCalibration_BuildServoMechanical(
            record, &calibration);
    }
    app_control_queue_proto_text(
        APP_PROTO_MSG_SERVO_CAL,
        "SERVOCAL scope=%s runtime_gen=%lu record_gen=%lu valid=%u "
        "alpha_center=%u alpha_min=%u alpha_max=%u alpha_sign=%d "
        "beta_center=%u beta_min=%u beta_max=%u beta_sign=%d\r\n",
        (scope != NULL) ? scope : "unknown",
        (unsigned long)runtime_generation,
        (unsigned long)((record != NULL) ?
            record->calibration_generation : 0U),
        (unsigned int)valid,
        calibration.center_us[DRV_COAX_CTRL_SERVO_ALPHA_INDEX],
        calibration.min_us[DRV_COAX_CTRL_SERVO_ALPHA_INDEX],
        calibration.max_us[DRV_COAX_CTRL_SERVO_ALPHA_INDEX],
        (int)calibration.pulse_sign[DRV_COAX_CTRL_SERVO_ALPHA_INDEX],
        calibration.center_us[DRV_COAX_CTRL_SERVO_BETA_INDEX],
        calibration.min_us[DRV_COAX_CTRL_SERVO_BETA_INDEX],
        calibration.max_us[DRV_COAX_CTRL_SERVO_BETA_INDEX],
        (int)calibration.pulse_sign[DRV_COAX_CTRL_SERVO_BETA_INDEX]);
}

static void app_control_report_servocal(void)
{
    APP_FlightCalibrationSnapshot active;

    app_control_imuframe_sync_param();
    memset(&active, 0, sizeof(active));
    (void)APP_FlightCalibration_ReadActive(&active);
    app_control_queue_proto_text(
        APP_PROTO_MSG_SERVO_CAL,
        "SERVOCAL event=%s reason=%s applied=%u commit_pending=%u "
        "dirty=%u arm_lock=%u request=%lu\r\n",
        (control_servocal_last_event != NULL) ?
            control_servocal_last_event : "status",
        (control_servocal_last_reason != NULL) ?
            control_servocal_last_reason : "none",
        (unsigned int)control_servocal_applied,
        (unsigned int)control_servocal_commit_pending,
        (unsigned int)SVC_Param_IsDirty(),
        (unsigned int)APP_Stabilizer_IsServoCalibrationCandidateArmLocked(),
        (unsigned long)control_servocal_last_request);
    app_control_report_servocal_record(
        "active", &active.calibration, active.generation);
    app_control_report_servocal_record(
        "persisted",
        (control_imucal_confirmed_valid != 0U) ?
            &control_imucal_confirmed : NULL,
        control_imucal_confirmed_generation);
}

static uint8_t app_control_parse_servocal(
    char **tokens,
    uint32_t count,
    DRV_COAX_CTRL_ServoCalibration *calibration)
{
    static const char *const center_keys[DRV_COAX_CTRL_SERVO_COUNT] = {
        "ac", "bc"
    };
    static const char *const min_keys[DRV_COAX_CTRL_SERVO_COUNT] = {
        "an", "bn"
    };
    static const char *const max_keys[DRV_COAX_CTRL_SERVO_COUNT] = {
        "ax", "bx"
    };
    static const char *const sign_keys[DRV_COAX_CTRL_SERVO_COUNT] = {
        "as", "bs"
    };
    uint32_t index;

    if ((tokens == NULL) || (calibration == NULL)) {
        return 0U;
    }
    for (index = 0U; index < DRV_COAX_CTRL_SERVO_COUNT; ++index) {
        uint32_t center;
        uint32_t minimum;
        uint32_t maximum;
        int32_t sign;
        const char *center_text = app_control_token_value(
            tokens, count, center_keys[index]);
        const char *min_text = app_control_token_value(
            tokens, count, min_keys[index]);
        const char *max_text = app_control_token_value(
            tokens, count, max_keys[index]);
        const char *sign_text = app_control_token_value(
            tokens, count, sign_keys[index]);
        if ((app_control_parse_u32(center_text, &center) == 0U) ||
            (app_control_parse_u32(min_text, &minimum) == 0U) ||
            (app_control_parse_u32(max_text, &maximum) == 0U) ||
            (app_control_parse_i32(sign_text, &sign) == 0U) ||
            (center > 65535U) || (minimum > 65535U) ||
            (maximum > 65535U) || (sign < -128) || (sign > 127)) {
            return 0U;
        }
        calibration->center_us[index] = (uint16_t)center;
        calibration->min_us[index] = (uint16_t)minimum;
        calibration->max_us[index] = (uint16_t)maximum;
        calibration->pulse_sign[index] = (int8_t)sign;
    }
    return DRV_COAX_CTRL_ValidateServoCalibration(calibration);
}

static void app_control_handle_servocal(char **tokens, uint32_t count)
{
    APP_FlightCalibrationSnapshot active;
    DRV_COAX_CTRL_ServoCalibration servo_calibration;
    StabilizerValidationImuSnapshot safety;
    const char *safety_error;
    uint8_t encoded[sizeof(APP_FlightCalibration)];
    uint32_t encoded_size;
    SVC_ParamStatus param_status;

    if ((tokens == NULL) || (count == 0U)) {
        return;
    }
    app_control_imuframe_sync_param();
    if (strcmp(tokens[0], "SERVOCAL?") == 0) {
        app_control_servocal_set_event("status", "none");
        app_control_report_servocal();
        return;
    }
    if ((count < 2U) || (strcmp(tokens[0], "SERVOCAL") != 0)) {
        APP_Control_QueueText(
            "ERR usage SERVOCAL? | SERVOCAL APPLY ac= an= ax= as= bc= bn= bx= bs= | REVERT | COMMIT\r\n");
        return;
    }
    if ((control_imucal_applied != 0U) ||
        (control_imucal_commit_pending != 0U) ||
        (control_imucal_upload.state != APP_FLIGHT_CAL_UPLOAD_EMPTY)) {
        app_control_servocal_set_event("rejected", "imucal_busy");
        app_control_report_servocal();
        return;
    }
    if (APP_Sensor_GetFluOrientation() != control_imuframe_confirmed_code) {
        app_control_servocal_set_event("rejected", "imuframe_preview_active");
        app_control_report_servocal();
        return;
    }

    if (strcmp(tokens[1], "APPLY") == 0) {
        if ((control_servocal_applied != 0U) ||
            (control_servocal_commit_pending != 0U) ||
            (SVC_Param_IsDirty() != 0U) ||
            (control_imucal_confirmed_valid == 0U) ||
            (APP_Acceptance_IsActive() != 0U) ||
            (app_control_parse_servocal(
                 tokens, count, &servo_calibration) == 0U)) {
            app_control_servocal_set_event("apply_rejected", "bad_state_or_values");
            app_control_report_servocal();
            return;
        }
        safety_error = app_control_imucal_safety(&safety, 0U);
        if (safety_error != NULL) {
            app_control_servocal_set_event("apply_rejected", safety_error);
            app_control_report_servocal();
            return;
        }
        control_servocal_preview = control_imucal_confirmed;
        if (APP_FlightCalibration_UpdateServoMechanical(
                &control_servocal_preview, &servo_calibration) == 0U) {
            app_control_servocal_set_event("apply_rejected", "merge_invalid");
            app_control_report_servocal();
            return;
        }
        APP_Stabilizer_SetServoCalibrationCandidateArmLock(1U);
        if ((APP_FlightCalibration_PublishPreview(
                 &control_servocal_preview) == 0U) ||
            (APP_FlightCalibration_ReadActive(&active) == 0U)) {
            APP_Stabilizer_SetServoCalibrationCandidateArmLock(0U);
            app_control_servocal_set_event("apply_rejected", "publish_failed");
            app_control_report_servocal();
            return;
        }
        control_servocal_preview_generation = active.generation;
        control_servocal_last_request = 0U;
        control_servocal_applied = 1U;
        app_control_servocal_set_event("applied", "none");
        app_control_report_servocal();
        return;
    }

    if (strcmp(tokens[1], "REVERT") == 0) {
        safety_error = app_control_imucal_safety(&safety, 0U);
        if ((count != 2U) || (control_servocal_applied == 0U) ||
            (control_servocal_commit_pending != 0U) ||
            (SVC_Param_IsDirty() != 0U) || (safety_error != NULL) ||
            (APP_FlightCalibration_PublishPreview(
                 &control_imucal_confirmed) == 0U)) {
            app_control_servocal_set_event(
                "revert_rejected",
                (safety_error != NULL) ? safety_error : "bad_state");
            app_control_report_servocal();
            return;
        }
        app_control_servocal_clear_preview();
        app_control_servocal_set_event("reverted", "none");
        app_control_report_servocal();
        return;
    }

    if (strcmp(tokens[1], "COMMIT") == 0) {
        safety_error = app_control_imucal_safety(&safety, 0U);
        if ((count != 2U) || (control_servocal_applied == 0U) ||
            (control_servocal_commit_pending != 0U) ||
            (SVC_Param_IsDirty() != 0U) || (safety_error != NULL) ||
            (APP_FlightCalibration_ReadActive(&active) == 0U) ||
            (active.generation != control_servocal_preview_generation) ||
            (memcmp(&active.calibration, &control_servocal_preview,
                    sizeof(active.calibration)) != 0)) {
            app_control_servocal_set_event(
                "commit_rejected",
                (safety_error != NULL) ? safety_error : "bad_state");
            app_control_report_servocal();
            return;
        }
        control_servocal_pending_record = control_imucal_confirmed;
        (void)APP_FlightCalibration_BuildServoMechanical(
            &control_servocal_preview, &servo_calibration);
        if (APP_FlightCalibration_UpdateServoMechanical(
                &control_servocal_pending_record,
                &servo_calibration) == 0U) {
            app_control_servocal_set_event("commit_rejected", "merge_invalid");
            app_control_report_servocal();
            return;
        }
        encoded_size = APP_FlightCalibration_Encode(
            &control_servocal_pending_record, encoded, sizeof(encoded));
        if (encoded_size == 0U) {
            app_control_servocal_set_event("commit_rejected", "encode_failed");
            app_control_report_servocal();
            return;
        }
        param_status = SVC_Param_SetBlob(encoded, encoded_size);
        if (param_status != SVC_PARAM_STATUS_OK) {
            app_control_servocal_set_event("commit_rejected", "param_failed");
            app_control_report_servocal();
            return;
        }
        control_servocal_commit_pending = 1U;
        control_servocal_last_request = SVC_Param_RequestSaveBlob();
        app_control_servocal_set_event(
            "commit_queued",
            (control_servocal_last_request != 0U) ? "none" : "queue_retry");
        app_control_report_servocal();
        return;
    }

    APP_Control_QueueText(
        "ERR usage SERVOCAL? | SERVOCAL APPLY ac= an= ax= as= bc= bn= bx= bs= | REVERT | COMMIT\r\n");
}

static void app_control_service_servocal(void)
{
    if ((control_servocal_commit_pending != 0U) &&
        (SVC_Param_IsDirty() != 0U) &&
        (control_servocal_last_request == 0U)) {
        control_servocal_last_request = SVC_Param_RequestSaveBlob();
    }
}

static int32_t app_control_acceptance_milli(float value)
{
    if ((!isfinite(value)) || (value > 2147483.0f)) return 2147483647L;
    if (value < -2147483.0f) return (-2147483647L - 1L);
    return (int32_t)(value * 1000.0f);
}

static void app_control_report_acceptance(void)
{
    APP_AcceptanceLeaseStatus lease;
    APP_AcceptanceSnapshot sample;
    APP_FirmwareIdentity identity;
    uint8_t sample_valid = APP_Acceptance_ReadSnapshot(&sample);
    uint8_t identity_valid = APP_FirmwareIdentity_Get(&identity);
    APP_Acceptance_GetLeaseStatus(&lease);
    app_control_queue_proto_text(
        APP_PROTO_MSG_ACCEPTANCE,
        "ACCEPT active=%u lease=%lu issued_ms=%lu expires_ms=%lu lease_ms=%u "
        "stage=%s props=%u sample=%u seq=%lu\r\n",
        (unsigned int)lease.active, (unsigned long)lease.lease_id,
        (unsigned long)lease.issued_ms, (unsigned long)lease.expires_ms,
        (unsigned int)APP_ACCEPTANCE_LEASE_MS,
        APP_Acceptance_StageText(lease.stage),
        (unsigned int)lease.props_removed, (unsigned int)sample_valid,
        (unsigned long)((sample_valid != 0U) ? sample.sequence : 0U));
    if (sample_valid == 0U) return;
    app_control_queue_proto_text(
        APP_PROTO_MSG_ACCEPTANCE,
        "ACCEPT context fw=0x%08lX contract=1 orientation=%u calgen=%lu "
        "valid=0x%02X esc=%u,%u link=%u failsafe=%u failsafe_ms=%lu mode=safe\r\n",
        (unsigned long)((identity_valid != 0U) ? identity.image_crc32 : 0U),
        (unsigned int)sample.orientation_code,
        (unsigned long)sample.calibration_generation,
        (unsigned int)sample.calibration_valid_mask,
        (unsigned int)sample.esc_ccr[0], (unsigned int)sample.esc_ccr[1],
        (unsigned int)sample.link_present,
        (unsigned int)sample.failsafe_active,
        (unsigned long)sample.failsafe_elapsed_ms);
    app_control_queue_proto_text(
        APP_PROTO_MSG_ACCEPTANCE,
        "ACCEPT motion rc=%u,%u,%u nav_mmps=%ld,%ld angle_mdeg=%ld,%ld "
        "rate_mdps=%ld,%ld moment_u=%ld,%ld\r\n",
        sample.rc_us[0], sample.rc_us[1], sample.rc_us[2],
        (long)app_control_acceptance_milli(sample.nav_velocity_m_s[0]),
        (long)app_control_acceptance_milli(sample.nav_velocity_m_s[1]),
        (long)app_control_acceptance_milli(sample.angle_deg[0]),
        (long)app_control_acceptance_milli(sample.angle_deg[1]),
        (long)app_control_acceptance_milli(sample.rate_dps[0]),
        (long)app_control_acceptance_milli(sample.rate_dps[1]),
        (long)app_control_acceptance_milli(sample.moment_n_m[0]),
        (long)app_control_acceptance_milli(sample.moment_n_m[1]));
    app_control_queue_proto_text(
        APP_PROTO_MSG_ACCEPTANCE,
        "ACCEPT control restoring_u=%ld,%ld damping_u=%ld,%ld\r\n",
        (long)app_control_acceptance_milli(sample.restoring_moment_n_m[0]),
        (long)app_control_acceptance_milli(sample.restoring_moment_n_m[1]),
        (long)app_control_acceptance_milli(sample.damping_moment_n_m[0]),
        (long)app_control_acceptance_milli(sample.damping_moment_n_m[1]));
    app_control_queue_proto_text(
        APP_PROTO_MSG_ACCEPTANCE,
        "ACCEPT servo center=%u,%u cmd=%u,%u sent=%u,%u feedback=%u,%u "
        "fb_valid=0x%02X fb_age=%u,%u\r\n",
        sample.servo_center_us[0], sample.servo_center_us[1],
        sample.servo_command_us[0], sample.servo_command_us[1],
        sample.servo_sent_us[0], sample.servo_sent_us[1],
        sample.servo_feedback_us[0], sample.servo_feedback_us[1],
        sample.servo_feedback_valid_mask,
        sample.servo_feedback_age_ms[0], sample.servo_feedback_age_ms[1]);
}

static void app_control_handle_acceptance(char **tokens, uint32_t count)
{
    APP_AcceptanceLeaseStatus lease;
    APP_AcceptanceStage stage;
    APP_FlightCalibrationSnapshot calibration;
    StabilizerValidationImuSnapshot safety;
    uint32_t lease_id;
    uint32_t now_ms = HAL_GetTick();
    uint64_t now_us;
    const char *value;

    if ((count == 1U) && (strcmp(tokens[0], "ACCEPT?") == 0)) {
        app_control_report_acceptance();
        return;
    }
    if ((count < 3U) || (strcmp(tokens[0], "ACCEPT") != 0) ||
        (strcmp(tokens[1], "V2") != 0)) {
        APP_Control_QueueText("ERR usage ACCEPT? | ACCEPT V2 START|KEEPALIVE|STAGE|STOP\r\n");
        return;
    }
    if (strcmp(tokens[2], "START") == 0) {
        value = app_control_token_value(tokens, count, "props");
        if ((value == NULL) || (strcmp(value, "1") != 0) ||
            (SVC_Param_IsDirty() != 0U) ||
            (control_imucal_applied != 0U) ||
            (control_imucal_commit_pending != 0U) ||
            (APP_Stabilizer_IsImuCalibrationCandidateArmLocked() != 0U) ||
            (APP_Stabilizer_IsServoCalibrationCandidateArmLocked() != 0U) ||
            (APP_FlightCalibration_ReadActive(&calibration) == 0U) ||
            ((calibration.calibration.valid_mask &
              (APP_FLIGHT_CAL_VALID_ORIENTATION | APP_FLIGHT_CAL_VALID_ACCEL |
               APP_FLIGHT_CAL_VALID_GYRO |
               APP_FLIGHT_CAL_VALID_SERVO_MECHANICAL)) !=
              (APP_FLIGHT_CAL_VALID_ORIENTATION | APP_FLIGHT_CAL_VALID_ACCEL |
               APP_FLIGHT_CAL_VALID_GYRO |
               APP_FLIGHT_CAL_VALID_SERVO_MECHANICAL)) ||
            (calibration.calibration.orientation_code !=
             APP_Sensor_GetFluOrientation()) ||
            (APP_Stabilizer_ReadValidationImuSnapshot(&safety) == 0U)) {
            APP_Control_QueueText("ACCEPT event=start_rejected reason=prerequisite\r\n");
            return;
        }
        now_us = SVC_Timestamp_Us();
        if ((safety.armed != 0U) || (safety.timestamp_us == 0ULL) ||
            (now_us < safety.timestamp_us) ||
            ((now_us - safety.timestamp_us) > 100000ULL) ||
            (APP_Acceptance_Start(now_ms, 1U, &lease_id) == 0U)) {
            APP_Control_QueueText("ACCEPT event=start_rejected reason=unsafe\r\n");
            return;
        }
        APP_Control_QueueText("ACCEPT event=started lease=%lu lease_ms=%u esc=0,0\r\n",
                              (unsigned long)lease_id,
                              (unsigned int)APP_ACCEPTANCE_LEASE_MS);
        app_control_report_acceptance();
        return;
    }
    value = app_control_token_value(tokens, count, "lease");
    if ((value == NULL) || (app_control_parse_u32(value, &lease_id) == 0U)) {
        APP_Control_QueueText("ACCEPT event=rejected reason=bad_lease\r\n");
        return;
    }
    APP_Acceptance_GetLeaseStatus(&lease);
    if ((lease.active == 0U) || (lease.lease_id != lease_id)) {
        APP_Control_QueueText("ACCEPT event=rejected reason=lease_mismatch\r\n");
        return;
    }
    if (strcmp(tokens[2], "KEEPALIVE") == 0) {
        APP_Control_QueueText("ACCEPT event=keepalive ok=%u lease=%lu\r\n",
            (unsigned int)APP_Acceptance_Keepalive(now_ms, lease_id),
            (unsigned long)lease_id);
        return;
    } else if (strcmp(tokens[2], "STAGE") == 0) {
        value = app_control_token_value(tokens, count, "name");
        if ((APP_Acceptance_Keepalive(now_ms, lease_id) == 0U) ||
            (APP_Acceptance_ParseStage(value, &stage) == 0U) ||
            (APP_Acceptance_SetStage(stage) == 0U)) {
            APP_Control_QueueText("ACCEPT event=stage_rejected reason=invalid\r\n");
            return;
        }
        APP_Control_QueueText("ACCEPT event=stage ok=1 name=%s lease=%lu\r\n",
                              APP_Acceptance_StageText(stage),
                              (unsigned long)lease_id);
    } else if (strcmp(tokens[2], "STOP") == 0) {
        APP_Acceptance_Stop();
        APP_Control_QueueText("ACCEPT event=stopped lease=%lu esc=0,0\r\n",
                              (unsigned long)lease_id);
    } else {
        APP_Control_QueueText("ERR usage ACCEPT V2 START|KEEPALIVE|STAGE|STOP\r\n");
        return;
    }
    app_control_report_acceptance();
}

static uint8_t app_control_send_boot_scheduled(void)
{
    APP_UART_TxMessage tx_message;
    APP_UART_TxMessage dropped;
    int written;

    if ((uartTxQueueHandle == 0) || (APP_IMU_Capture_IsExportActive() != 0U)) {
        return 0U;
    }
    tx_message.function = APP_PROTO_MSG_BOOT_STATUS;
    written = snprintf(
        tx_message.text,
        sizeof(tx_message.text),
        "BOOT mode=dfu state=scheduled delay_ms=%u transport=usb_reenumerate\r\n",
        (unsigned int)APP_BOOT_DFU_SCHEDULE_DELAY_MS);
    if ((written <= 0) || ((uint32_t)written >= sizeof(tx_message.text))) {
        return 0U;
    }
    tx_message.length = (uint16_t)written;

    /* Scheduling is forbidden until USB reports transmit completion. */
    if (APP_USB_CDC_Write((const uint8_t *)tx_message.text,
                          tx_message.length,
                          100U) == 0U) {
        return 0U;
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
    return 1U;
}

static uint8_t app_control_imuframe_is_safe(void)
{
    StabilizerValidationImuSnapshot snapshot;

    memset(&snapshot, 0, sizeof(snapshot));
    if (APP_Stabilizer_ReadValidationImuSnapshot(&snapshot) == 0U) {
        return 0U;
    }
    if (snapshot.armed != 0U) {
        return 0U;
    }
    if ((snapshot.esc_pulse_us[0] > 1100U) ||
        (snapshot.esc_pulse_us[1] > 1100U)) {
        return 0U;
    }
    return 1U;
}

static void app_control_report_imuframe(const char *event, uint32_t request_id)
{
    uint8_t active_code;
    uint8_t dirty;
    uint8_t pending_code;
    uint8_t temporary;
    const char *frame;

    app_control_imuframe_sync_param();
    active_code = APP_Sensor_GetFluOrientation();
    dirty = SVC_Param_IsDirty();
    pending_code = (control_imuframe_pending_valid != 0U) ?
                   control_imuframe_pending_code :
                   APP_SENSOR_FLU_ORIENTATION_LEGACY;
    temporary = (active_code != control_imuframe_confirmed_code) ? 1U : 0U;

    if (active_code >= APP_SENSOR_FLU_ORIENTATION_COUNT) {
        frame = "legacy_intermediate";
    } else if (temporary != 0U) {
        frame = "canonical_flu_candidate";
    } else {
        frame = "canonical_flu_persisted";
    }

    app_control_queue_proto_text(
        APP_PROTO_MSG_TEXT_LINE,
        "IMUFRAME event=%s base=legacy_intermediate_v1 "
        "active=%s active_code=%u persisted=%s persisted_code=%u "
        "pending_code=%u pending_valid=%u temporary=%u dirty=%u "
        "frame=%s arm_lock=%u request=%lu\r\n",
        (event != NULL) ? event : "status",
        APP_Sensor_GetFluOrientationDescriptorForCode(active_code),
        (unsigned int)active_code,
        APP_Sensor_GetFluOrientationDescriptorForCode(
            control_imuframe_confirmed_code),
        (unsigned int)control_imuframe_confirmed_code,
        (unsigned int)pending_code,
        (unsigned int)control_imuframe_pending_valid,
        (unsigned int)temporary,
        (unsigned int)dirty,
        frame,
        (unsigned int)APP_Stabilizer_IsImuFrameArmLocked(),
        (unsigned long)request_id);
}

static void app_control_handle_imuframe(char **tokens, uint32_t count)
{
    APP_FlightCalibration calibration;
    APP_FlightCalibrationDecodeStatus decode_status;
    uint8_t blob[sizeof(APP_FlightCalibration)];
    uint8_t encoded[sizeof(APP_FlightCalibration)];
    uint32_t blob_size = 0U;
    uint32_t encoded_size;
    SVC_ParamStatus status;
    uint8_t active_code;

    if ((tokens == NULL) || (count == 0U)) {
        return;
    }

    app_control_imuframe_sync_param();

    if (strcmp(tokens[0], "IMUFRAME?") == 0) {
        app_control_report_imuframe("status", control_imuframe_last_request);
        return;
    }

    if ((APP_Stabilizer_IsImuCalibrationCandidateArmLocked() != 0U) ||
        (APP_Stabilizer_IsServoCalibrationCandidateArmLocked() != 0U)) {
        app_control_report_imuframe("imucal_busy", 0U);
        return;
    }

    if ((count >= 2U) && (strcmp(tokens[1], "APPLY") == 0)) {
        if (app_control_imuframe_is_safe() == 0U) {
            app_control_report_imuframe("safety_blocked", 0U);
            return;
        }
        if (SVC_Param_IsDirty() != 0U) {
            app_control_report_imuframe("apply_busy", 0U);
            return;
        }
        if ((count != 3U) ||
            (APP_Sensor_SetFluOrientation(tokens[2]) == 0U)) {
            app_control_report_imuframe("invalid_descriptor", 0U);
            return;
        }
        control_imuframe_boot_selection_pending = 0U;
        control_imuframe_last_request = 0U;
        app_control_report_imuframe("applied", 0U);
        return;
    }

    if ((count >= 2U) && (strcmp(tokens[1], "REVERT") == 0)) {
        if (app_control_imuframe_is_safe() == 0U) {
            app_control_report_imuframe("safety_blocked", 0U);
            return;
        }
        if (SVC_Param_IsReady() == 0U) {
            app_control_report_imuframe("param_not_ready", 0U);
            return;
        }
        if (SVC_Param_IsDirty() != 0U) {
            app_control_report_imuframe("revert_busy", 0U);
            return;
        }
        if ((count != 2U) ||
            (APP_Sensor_SetFluOrientationCode(
                 control_imuframe_confirmed_code) == 0U)) {
            app_control_report_imuframe("revert_failed", 0U);
            return;
        }
        control_imuframe_boot_selection_pending = 0U;
        control_imuframe_last_request = 0U;
        app_control_report_imuframe("reverted", 0U);
        return;
    }

    if ((count >= 2U) && (strcmp(tokens[1], "COMMIT") == 0)) {
        if (app_control_imuframe_is_safe() == 0U) {
            app_control_report_imuframe("safety_blocked", 0U);
            return;
        }
        if (count != 2U) {
            app_control_report_imuframe("invalid_usage", 0U);
            return;
        }
        if (SVC_Param_IsReady() == 0U) {
            app_control_report_imuframe("param_not_ready", 0U);
            return;
        }

        active_code = APP_Sensor_GetFluOrientation();
        if (SVC_Param_IsDirty() != 0U) {
            if ((control_imuframe_pending_valid == 0U) ||
                (active_code != control_imuframe_pending_code)) {
                app_control_report_imuframe("commit_busy", 0U);
                return;
            }
            control_imuframe_last_request = SVC_Param_RequestSaveBlob();
            if (control_imuframe_last_request == 0U) {
                app_control_report_imuframe("commit_queue_failed", 0U);
                return;
            }
            app_control_report_imuframe("commit_queued",
                                        control_imuframe_last_request);
            return;
        }

        memset(blob, 0, sizeof(blob));
        status = SVC_Param_GetBlob(blob, sizeof(blob), &blob_size);
        if ((status == SVC_PARAM_STATUS_OK) && (blob_size == 0U)) {
            APP_FlightCalibration_Defaults(&calibration);
        } else if (status == SVC_PARAM_STATUS_OK) {
            decode_status = APP_FlightCalibration_Decode(
                blob, blob_size, &calibration);
            if (decode_status == APP_FLIGHT_CAL_DECODE_INVALID) {
                app_control_report_imuframe("commit_decode_failed", 0U);
                return;
            }
        } else if (status == SVC_PARAM_STATUS_NO_VALID_RECORD) {
            APP_FlightCalibration_Defaults(&calibration);
        } else {
            app_control_report_imuframe("commit_read_failed", 0U);
            return;
        }

        if (APP_FlightCalibration_UpdateOrientation(&calibration,
                                                    active_code) == 0U) {
            app_control_report_imuframe("commit_failed", 0U);
            return;
        }
        encoded_size = APP_FlightCalibration_Encode(
            &calibration, encoded, sizeof(encoded));
        if (encoded_size == 0U) {
            app_control_report_imuframe("commit_encode_failed", 0U);
            return;
        }
        status = SVC_Param_SetBlob(encoded, encoded_size);
        if (status != SVC_PARAM_STATUS_OK) {
            app_control_report_imuframe("commit_failed", 0U);
            return;
        }

        control_imuframe_pending_code = active_code;
        control_imuframe_pending_valid = 1U;
        control_imuframe_boot_selection_pending = 0U;
        control_imuframe_last_request = SVC_Param_RequestSaveBlob();
        if (control_imuframe_last_request == 0U) {
            app_control_report_imuframe("commit_queue_failed", 0U);
            return;
        }
        app_control_report_imuframe("commit_queued",
                                    control_imuframe_last_request);
        return;
    }

    app_control_report_imuframe("invalid_usage", 0U);
}

static const char *app_control_boot_state_name(APP_BootState state)
{
    switch (state) {
    case APP_BOOT_STATE_WAITING_USB_REPLY:
        return "waiting_usb_reply";
    case APP_BOOT_STATE_SCHEDULED:
        return "scheduled";
    case APP_BOOT_STATE_ENTERING:
        return "entering";
    case APP_BOOT_STATE_IDLE:
    default:
        return "idle";
    }
}

static const char *app_control_boot_safety_name(APP_BootSafety safety)
{
    switch (safety) {
    case APP_BOOT_SAFETY_NO_VALID_SNAPSHOT:
        return "snapshot_invalid";
    case APP_BOOT_SAFETY_ARMED:
        return "armed";
    case APP_BOOT_SAFETY_ESC_HIGH:
        return "esc_high";
    case APP_BOOT_SAFETY_SNAPSHOT_STALE:
        return "snapshot_stale";
    case APP_BOOT_SAFETY_OK:
    default:
        return "ok";
    }
}

static void app_control_report_boot(void)
{
    APP_BootStatus status;

    APP_Boot_GetStatus(&status);
    app_control_queue_proto_text(
        APP_PROTO_MSG_BOOT_STATUS,
        "BOOT mode=dfu state=%s target=rom_usb_dfu addr=0x%08lX delay_ms=%u "
        "remain_ms=%lu valid=%u armed=%u esc=%u,%u safe=%u reason=%s "
        "age_us=%lu seq=%lu request_seq=%lu vector=%u confirm=BOOT_DFU_CONFIRM\r\n",
        app_control_boot_state_name(status.state),
        (unsigned long)APP_BOOT_ROM_DFU_VECTOR_ADDRESS,
        (unsigned int)APP_BOOT_DFU_SCHEDULE_DELAY_MS,
        (unsigned long)status.remaining_ms,
        (unsigned int)status.snapshot_valid,
        (unsigned int)status.armed,
        (unsigned int)status.esc_pulse_us[0],
        (unsigned int)status.esc_pulse_us[1],
        (unsigned int)(status.safety == APP_BOOT_SAFETY_OK),
        app_control_boot_safety_name(status.safety),
        (unsigned long)status.snapshot_age_us,
        (unsigned long)status.snapshot_sequence,
        (unsigned long)status.request_sequence,
        (unsigned int)status.vector_valid);
}

static void app_control_handle_boot(char **tokens, uint32_t count)
{
    APP_BootRequestResult result;
    const char *reason;

    if ((tokens == NULL) || (count == 0U)) {
        return;
    }

    if (strcmp(tokens[0], "BOOT?") == 0) {
        if (count != 1U) {
            APP_Control_QueueText("ERR usage BOOT?\r\n");
            return;
        }
        app_control_report_boot();
        return;
    }

    /* Exact three-token confirmation prevents a generic BOOT/DFU click. */
    if ((count != 3U) ||
        (strcmp(tokens[1], "DFU") != 0) ||
        (strcmp(tokens[2], "CONFIRM") != 0)) {
        APP_Control_QueueText("ERR usage BOOT DFU CONFIRM\r\n");
        return;
    }

    result = APP_Boot_RequestDfu();
    if (result == APP_BOOT_REQUEST_READY_FOR_USB) {
        if (app_control_send_boot_scheduled() == 0U) {
            APP_Boot_CancelDfuRequest();
            app_control_queue_proto_text(
                APP_PROTO_MSG_BOOT_STATUS,
                "BOOT mode=dfu state=refused reason=usb_confirmation_failed\r\n");
            return;
        }
        if (APP_Boot_ConfirmDfuScheduled() == 0U) {
            APP_Boot_CancelDfuRequest();
            app_control_queue_proto_text(
                APP_PROTO_MSG_BOOT_STATUS,
                "BOOT mode=dfu state=cancelled reason=schedule_state_failed\r\n");
        }
        return;
    }

    switch (result) {
    case APP_BOOT_REQUEST_ALREADY_PENDING:
        reason = "already_pending";
        break;
    case APP_BOOT_REQUEST_NO_VALID_SNAPSHOT:
        reason = "snapshot_invalid";
        break;
    case APP_BOOT_REQUEST_ARMED:
        reason = "armed";
        break;
    case APP_BOOT_REQUEST_ESC_HIGH:
        reason = "esc_high";
        break;
    case APP_BOOT_REQUEST_SNAPSHOT_STALE:
        reason = "snapshot_stale";
        break;
    case APP_BOOT_REQUEST_VECTOR_INVALID:
    default:
        reason = "rom_vector_invalid";
        break;
    }
    app_control_queue_proto_text(APP_PROTO_MSG_BOOT_STATUS,
                                 "BOOT mode=dfu state=refused reason=%s\r\n",
                                 reason);
}

static void app_control_service_boot(void)
{
    APP_BootEvent event = APP_Boot_Tick();
    const char *reason;

    if (event == APP_BOOT_EVENT_NONE) {
        return;
    }

    switch (event) {
    case APP_BOOT_EVENT_CANCELLED_NO_SNAPSHOT:
        reason = "snapshot_invalid";
        break;
    case APP_BOOT_EVENT_CANCELLED_ARMED:
        reason = "armed";
        break;
    case APP_BOOT_EVENT_CANCELLED_ESC_HIGH:
        reason = "esc_high";
        break;
    case APP_BOOT_EVENT_CANCELLED_SNAPSHOT_STALE:
        reason = "snapshot_stale";
        break;
    case APP_BOOT_EVENT_CANCELLED_SEQUENCE_STALLED:
        reason = "snapshot_sequence_stalled";
        break;
    case APP_BOOT_EVENT_ESC_DISABLE_FAILED:
        reason = "esc_disable_failed";
        break;
    case APP_BOOT_EVENT_MAGIC_WRITE_FAILED:
        reason = "reset_magic_write_failed";
        break;
    case APP_BOOT_EVENT_VECTOR_INVALID:
    default:
        reason = "rom_vector_invalid";
        break;
    }

    app_control_queue_proto_text(APP_PROTO_MSG_BOOT_STATUS,
                                 "BOOT mode=dfu state=cancelled reason=%s\r\n",
                                 reason);
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

/*
 * Full-rate raw IMU capture control. Used to record undecimated pre-filter
 * samples for vibration spectrum analysis; see App/Inc/app_imu_capture.h.
 */
static void app_control_handle_imu_capture(char **tokens, uint32_t count)
{
    APP_IMU_CaptureStatus status;
    APP_IMU_CaptureCommandStatus cmd_status;

    if ((tokens == NULL) || (count == 0U)) {
        return;
    }

    if (strcmp(tokens[0], "IMUCAP?") == 0) {
        APP_IMU_Capture_GetStatus(&status);
        APP_Control_QueueText("IMUCAP state=%s stored=%lu capacity=%lu "
                              "requested=%lu dropped=%lu export_sent=%lu "
                              "session=%lu sample_bytes=%u "
                              "accel_range=%uG gyro_range=%udps "
                              "accel_aaf=%uHz gyro_aaf=%uHz\r\n",
                              APP_IMU_Capture_StateText(status.state),
                              (unsigned long)status.stored,
                              (unsigned long)status.capacity,
                              (unsigned long)status.requested,
                              (unsigned long)status.dropped,
                              (unsigned long)status.export_sent,
                              (unsigned long)status.session_id,
                              (unsigned int)sizeof(APP_IMU_CaptureSample),
                              (unsigned int)status.accel_range_g,
                              (unsigned int)status.gyro_range_dps,
                              (unsigned int)status.accel_aaf_hz,
                              (unsigned int)status.gyro_aaf_hz);
        return;
    }

    if ((count >= 2U) && (strcmp(tokens[1], "START") == 0)) {
        uint32_t samples = 0U; /* 0 selects the full buffer */

        if ((count >= 3U) &&
            (app_control_parse_u32(tokens[2], &samples) == 0U)) {
            APP_Control_QueueText("ERR usage IMUCAP START [samples]\r\n");
            return;
        }

        cmd_status = APP_IMU_Capture_Start(samples);
        APP_IMU_Capture_GetStatus(&status);
        APP_Control_QueueText("IMUCAP START %s requested=%lu\r\n",
                              APP_IMU_Capture_CommandStatusText(cmd_status),
                              (unsigned long)status.requested);
        return;
    }

    if ((count >= 2U) && (strcmp(tokens[1], "STOP") == 0)) {
        cmd_status = APP_IMU_Capture_Stop();
        APP_IMU_Capture_GetStatus(&status);
        APP_Control_QueueText("IMUCAP STOP %s stored=%lu\r\n",
                              APP_IMU_Capture_CommandStatusText(cmd_status),
                              (unsigned long)status.stored);
        return;
    }

    if ((count >= 2U) && (strcmp(tokens[1], "DUMP") == 0)) {
        cmd_status = APP_IMU_Capture_StartDump();
        if (cmd_status != APP_IMU_CAPTURE_CMD_OK) {
            APP_Control_QueueText("IMUCAP ERROR start %s\r\n",
                                  APP_IMU_Capture_CommandStatusText(cmd_status));
            return;
        }
        APP_IMU_Capture_GetStatus(&status);
        /* Host reads this line to size the binary stream that follows. */
        APP_Control_QueueText("IMUCAP DUMP ok samples=%lu sample_bytes=%u\r\n",
                              (unsigned long)status.stored,
                              (unsigned int)sizeof(APP_IMU_CaptureSample));
        return;
    }

    if ((count >= 2U) && (strcmp(tokens[1], "CANCEL") == 0)) {
        cmd_status = APP_IMU_Capture_CancelDump();
        APP_Control_QueueText("IMUCAP CANCEL %s\r\n",
                              APP_IMU_Capture_CommandStatusText(cmd_status));
        return;
    }

    APP_Control_QueueText("ERR usage IMUCAP? | IMUCAP START [samples] | "
                          "IMUCAP STOP | IMUCAP DUMP | IMUCAP CANCEL\r\n");
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
    DRV_COAX_CTRL_ServoCalibration servo_calibration;

    if (config == NULL) {
        return;
    }

    memset(config, 0, sizeof(*config));
    DRV_COAX_CTRL_GetServoCalibration(&servo_calibration);
    config->servo[0].id = 1U;
    config->servo[0].pulse_us =
        servo_calibration.center_us[DRV_COAX_CTRL_SERVO_ALPHA_INDEX];
    config->servo[0].time_ms = 500U;
    config->servo[0].mode = 1U;
    config->servo[0].enabled = 1U;
    config->servo[1].id = 2U;
    config->servo[1].pulse_us =
        servo_calibration.center_us[DRV_COAX_CTRL_SERVO_BETA_INDEX];
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
    DRV_COAX_CTRL_ServoCalibration calibration;
    uint16_t min_us;
    uint16_t max_us;

    DRV_COAX_CTRL_GetServoCalibration(&calibration);
    min_us = calibration.min_us[index];
    max_us = calibration.max_us[index];

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

/*
 * UI 与内部表示的符号换算。
 *
 * 历史上部分增益内部存负值，这里再翻一次让界面显示正数——于是"符号"这件事
 * 在 UI 层和控制层各有一套约定，两者不一致时极难发现（界面显示 -0.600 而
 * 内部实际是 +0.600 之类）。现在增益内部一律为正，UI 直接显示内部值，不再
 * 做任何翻转：只保留这个函数作为单一换算入口，恒返回 +1。
 *
 * 极性的唯一真相在 Driver/Src/drv_coax_ctrl.c 的"极性约定"块和
 * Core/Src/freertos.c 的摇杆映射常数，由 tests/test_coax_sign_convention.py
 * 锁定。
 */
static float app_control_ui_sign_for_param(const char *name)
{
    (void)name;
    return 1.0f;
}

static float app_control_param_to_ui_value(const char *name, float internal_value)
{
    return internal_value * app_control_ui_sign_for_param(name);
}

static float app_control_param_from_ui_value(const char *name, float ui_value)
{
    return ui_value * app_control_ui_sign_for_param(name);
}

static void app_control_report_coax_param(const char *name, float value)
{
    char value_text[24];

    app_control_format_float(app_control_param_to_ui_value(name, value),
                             value_text,
                             (uint32_t)sizeof(value_text));
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

static void app_control_capture_coax_tunables(APP_ControlCoaxTunableParams *out)
{
    DRV_COAX_CTRL_Params params;

    if (out == NULL) {
        return;
    }

    DRV_COAX_CTRL_GetParams(&params);
    out->pos_x_kp = params.pos_x_kp;
    out->pos_y_kp = params.pos_y_kp;
    out->pos_z_kp = params.pos_z_kp;
    out->pos_z_ki = params.pos_z_ki;
    out->vel_x_kd = params.vel_x_kd;
    out->vel_y_kd = params.vel_y_kd;
    out->vel_z_kd = params.vel_z_kd;
    out->vel_loop_enable = params.vel_loop_enable;
    out->vel_loop_x_kp = params.vel_loop_x_kp;
    out->vel_loop_x_ki = params.vel_loop_x_ki;
    out->vel_loop_x_kd = params.vel_loop_x_kd;
    out->vel_loop_y_kp = params.vel_loop_y_kp;
    out->vel_loop_y_ki = params.vel_loop_y_ki;
    out->vel_loop_y_kd = params.vel_loop_y_kd;
    out->roll_angle_kp = params.roll_angle_kp;
    out->pitch_angle_kp = params.pitch_angle_kp;
    out->roll_rate_kd = params.roll_rate_kd;
    out->pitch_rate_kd = params.pitch_rate_kd;
    out->tilt_limit_rad = params.tilt_limit_rad;
    out->yaw_angle_kp = params.yaw_angle_kp;
    out->yaw_rate_kd = params.yaw_rate_kd;
}

static void app_control_apply_coax_tunables(const APP_ControlCoaxTunableParams *in)
{
    DRV_COAX_CTRL_Params params;

    if (in == NULL) {
        return;
    }

    DRV_COAX_CTRL_GetDefaultParams(&params);
    params.pos_x_kp = in->pos_x_kp;
    params.pos_y_kp = in->pos_y_kp;
    params.pos_z_kp = in->pos_z_kp;
    params.pos_z_ki = in->pos_z_ki;
    params.vel_x_kd = in->vel_x_kd;
    params.vel_y_kd = in->vel_y_kd;
    params.vel_z_kd = in->vel_z_kd;
    params.vel_loop_enable = in->vel_loop_enable;
    params.vel_loop_x_kp = in->vel_loop_x_kp;
    params.vel_loop_x_ki = in->vel_loop_x_ki;
    params.vel_loop_x_kd = in->vel_loop_x_kd;
    params.vel_loop_y_kp = in->vel_loop_y_kp;
    params.vel_loop_y_ki = in->vel_loop_y_ki;
    params.vel_loop_y_kd = in->vel_loop_y_kd;
    params.roll_angle_kp = in->roll_angle_kp;
    params.pitch_angle_kp = in->pitch_angle_kp;
    params.roll_rate_kd = in->roll_rate_kd;
    params.pitch_rate_kd = in->pitch_rate_kd;
    params.tilt_limit_rad = in->tilt_limit_rad;
    if ((fabsf(params.tilt_limit_rad - APP_CONTROL_TILT_LIMIT_LEGACY_18_RAD) <=
         APP_CONTROL_TILT_LIMIT_LEGACY_EPS_RAD) ||
        (fabsf(params.tilt_limit_rad - APP_CONTROL_TILT_LIMIT_LEGACY_25_RAD) <=
         APP_CONTROL_TILT_LIMIT_LEGACY_EPS_RAD)) {
        params.tilt_limit_rad = APP_CONTROL_TILT_LIMIT_DEFAULT_RAD;
    }
    params.yaw_angle_kp = in->yaw_angle_kp;
    params.yaw_rate_kd = in->yaw_rate_kd;
    DRV_COAX_CTRL_SetParams(&params);
}

static void app_control_apply_coax_tunables_v15(
    const APP_ControlCoaxTunableParamsV15 *in)
{
    DRV_COAX_CTRL_Params params;

    if (in == NULL) {
        return;
    }

    DRV_COAX_CTRL_GetDefaultParams(&params);
    params.pos_x_kp = in->pos_x_kp;
    params.pos_y_kp = in->pos_y_kp;
    params.pos_z_kp = in->pos_z_kp;
    params.vel_x_kd = in->vel_x_kd;
    params.vel_y_kd = in->vel_y_kd;
    params.vel_z_kd = in->vel_z_kd;
    params.vel_loop_enable = in->vel_loop_enable;
    params.vel_loop_x_kp = in->vel_loop_x_kp;
    params.vel_loop_x_ki = in->vel_loop_x_ki;
    params.vel_loop_x_kd = in->vel_loop_x_kd;
    params.vel_loop_y_kp = in->vel_loop_y_kp;
    params.vel_loop_y_ki = in->vel_loop_y_ki;
    params.vel_loop_y_kd = in->vel_loop_y_kd;
    params.roll_angle_kp = in->roll_angle_kp;
    params.pitch_angle_kp = in->pitch_angle_kp;
    params.roll_rate_kd = in->roll_rate_kd;
    params.pitch_rate_kd = in->pitch_rate_kd;
    params.tilt_limit_rad = in->tilt_limit_rad;
    if ((fabsf(params.tilt_limit_rad - APP_CONTROL_TILT_LIMIT_LEGACY_18_RAD) <=
         APP_CONTROL_TILT_LIMIT_LEGACY_EPS_RAD) ||
        (fabsf(params.tilt_limit_rad - APP_CONTROL_TILT_LIMIT_LEGACY_25_RAD) <=
         APP_CONTROL_TILT_LIMIT_LEGACY_EPS_RAD)) {
        params.tilt_limit_rad = APP_CONTROL_TILT_LIMIT_DEFAULT_RAD;
    }
    params.yaw_angle_kp = in->yaw_angle_kp;
    params.yaw_rate_kd = in->yaw_rate_kd;
    DRV_COAX_CTRL_SetParams(&params);
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
    StabilizerValidationImuSnapshot snapshot;
    APP_FirmwareIdentity firmware_identity;
    uint8_t snapshot_valid;
    uint8_t firmware_identity_valid;

    APP_IMU_GetStatus(&imu_status);
    snapshot_valid = APP_Stabilizer_ReadValidationImuSnapshot(&snapshot);
    firmware_identity_valid = APP_FirmwareIdentity_Get(&firmware_identity);
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
    if (snapshot_valid == 0U) {
        app_control_queue_proto_text(
            APP_PROTO_MSG_IMU_SCALED,
            "IMU sample valid=0 source=stabilizer_snapshot unavailable\r\n");
    } else if (firmware_identity_valid == 0U) {
        app_control_queue_proto_text(
            APP_PROTO_MSG_IMU_SCALED,
            "IMU sample valid=0 source=firmware_identity unavailable\r\n");
    } else {
        int32_t accel_mg[3];
        int32_t gyro_mdps[3];
        int32_t attitude_cdeg[3];
        int32_t temperature_cdeg;
        int32_t acceleration_error_cdeg;
        int32_t acceleration_recovery_trigger_milli;
        uint32_t timestamp_ms;
        uint8_t fusion_flags = 0U;
        const char *snapshot_frame;

        accel_mg[0] = (int32_t)(snapshot.accel_g[0] * 1000.0f);
        accel_mg[1] = (int32_t)(snapshot.accel_g[1] * 1000.0f);
        accel_mg[2] = (int32_t)(snapshot.accel_g[2] * 1000.0f);
        gyro_mdps[0] = (int32_t)(snapshot.gyro_dps[0] * 1000.0f);
        gyro_mdps[1] = (int32_t)(snapshot.gyro_dps[1] * 1000.0f);
        gyro_mdps[2] = (int32_t)(snapshot.gyro_dps[2] * 1000.0f);
        attitude_cdeg[0] = (int32_t)(snapshot.roll_deg * 100.0f);
        attitude_cdeg[1] = (int32_t)(snapshot.pitch_deg * 100.0f);
        attitude_cdeg[2] = (int32_t)(snapshot.yaw_deg * 100.0f);
        temperature_cdeg = (int32_t)(snapshot.temperature_c * 100.0f);
        acceleration_error_cdeg =
            (int32_t)(snapshot.fusion_acceleration_error_deg * 100.0f);
        acceleration_recovery_trigger_milli =
            (int32_t)(snapshot.fusion_acceleration_recovery_trigger * 1000.0f);
        timestamp_ms = (uint32_t)(snapshot.timestamp_us / 1000ULL);
        if (snapshot.fusion_accelerometer_ignored != 0U) {
            fusion_flags |= APP_IMU_CAPTURE_FUSION_ACCEL_IGNORED;
        }
        if (snapshot.fusion_accel_norm_rejected != 0U) {
            fusion_flags |= APP_IMU_CAPTURE_FUSION_NORM_REJECTED;
        }
        if (snapshot.fusion_acceleration_recovery != 0U) {
            fusion_flags |= APP_IMU_CAPTURE_FUSION_ACCEL_RECOVERY;
        }
        if (snapshot.fusion_angular_rate_recovery != 0U) {
            fusion_flags |= APP_IMU_CAPTURE_FUSION_RATE_RECOVERY;
        }

        /* Derive provenance from this exact seqlock snapshot, not from the
         * independently mutable global orientation selection.  Protocol
         * values are frame=legacy_intermediate or frame=canonical_flu_ram. */
        snapshot_frame =
            (snapshot.imu_frame_orientation_code <
             APP_SENSOR_FLU_ORIENTATION_COUNT) ?
            "canonical_flu_ram" : "legacy_intermediate";

        /*
         * Keep each line below APP_UART_TX_TEXT_SIZE even at maximum integer
         * width.  The first line is provenance/state; the second is the
         * coherent sample and Fusion diagnostics from the same snapshot.
         */
        app_control_queue_proto_text(
            APP_PROTO_MSG_IMU_SCALED,
            "IMU sample valid=1 source=stabilizer_snapshot frame=%s units=mg_mdps_cdeg contract=%u migration=0x%02lX orientation=%u ts_ms=%lu seq=%lu bias=%u armed=%u m1=%u m2=%u temp_cdeg=%ld\r\n",
            snapshot_frame,
            (unsigned int)DRV_FRAME_CONTRACT_VERSION,
            (unsigned long)DRV_FRAME_RUNTIME_MIGRATION_DONE_MASK,
            (unsigned int)snapshot.imu_frame_orientation_code,
            (unsigned long)timestamp_ms,
            (unsigned long)snapshot.sequence,
            (unsigned int)snapshot.gyro_bias_ready,
            (unsigned int)snapshot.armed,
            (unsigned int)snapshot.esc_pulse_us[0],
            (unsigned int)snapshot.esc_pulse_us[1],
            (long)temperature_cdeg);
        app_control_queue_proto_text(
            APP_PROTO_MSG_IMU_SCALED,
            "IMU sample seq=%lu ax=%ld ay=%ld az=%ld gx=%ld gy=%ld gz=%ld roll=%ld pitch=%ld yaw=%ld fusion_flags=0x%02X ferr_cdeg=%ld ftrig_milli=%ld fcorr=%lu\r\n",
            (unsigned long)snapshot.sequence,
            (long)accel_mg[0],
            (long)accel_mg[1],
            (long)accel_mg[2],
            (long)gyro_mdps[0],
            (long)gyro_mdps[1],
            (long)gyro_mdps[2],
            (long)attitude_cdeg[0],
            (long)attitude_cdeg[1],
            (long)attitude_cdeg[2],
            (unsigned int)fusion_flags,
            (long)acceleration_error_cdeg,
            (long)acceleration_recovery_trigger_milli,
            (unsigned long)snapshot.fusion_accel_correction_count);
        app_control_queue_proto_text(
            APP_PROTO_MSG_IMU_SCALED,
            "IMU calibration cal_generation=%lu valid_mask=0x%02X firmware_crc32=0x%08lX\r\n",
            (unsigned long)snapshot.calibration_generation,
            (unsigned int)snapshot.calibration_valid_mask,
            (unsigned long)firmware_identity.image_crc32);
        /*
         * 采样链健康单独一行：上位机据此拒绝在降级状态下采集标定证据，并把
         * "为什么不能解锁"显示成真实原因，而不是笼统的 IMU 未就绪。
         */
        app_control_queue_proto_text(
            APP_PROTO_MSG_IMU_SCALED,
            "IMU health level=%u rate_hz=%u fault=%u fault_ever=%u\r\n",
            (unsigned int)snapshot.imu_health_level,
            (unsigned int)snapshot.imu_sample_rate_hz,
            (unsigned int)snapshot.imu_health_fault_active,
            (unsigned int)snapshot.imu_health_fault_ever);
    }
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
    app_control_report_usb_cdc_stats();
    app_control_report_wifi();
}

/*
 * USB CDC 的 TX 丢弃必须能被看见：文本镜像忽略返回值，一旦 tx_dropped 开始涨，就
 * 说明上位机收到的是残缺的多行回复（例如 IMU? 少一行导致快照永远凑不齐）。
 */
static void app_control_report_usb_cdc_stats(void)
{
    app_control_queue_proto_text(APP_PROTO_MSG_UART_STATS,
                                 "USBCDC tx_sent=%lu tx_dropped=%lu\r\n",
                                 (unsigned long)APP_USB_CDC_GetTxSent(),
                                 (unsigned long)APP_USB_CDC_GetTxDropped());
}

/* ---------------------------------------------------------------------------
 * 遥控通道映射与端点标定
 * ------------------------------------------------------------------------ */

static void app_control_apply_rc_config(const APP_RcConfig *config)
{
    if ((config == NULL) || (APP_RcConfig_Validate(config) == 0U)) {
        APP_RcConfig_Defaults(&control_rc_config);
    } else {
        control_rc_config = *config;
    }
    control_rc_config_dirty = 0U;
    (void)APP_RcConfig_PublishActive(&control_rc_config);
}

static void app_control_report_rc_map(const char *state)
{
    uint8_t function;

    app_control_queue_proto_text(
        APP_PROTO_MSG_RC_MAP,
        "RCMAP state=%s funcs=%u channels=%u deadband_us=%u calibrated=%u "
        "dirty=%u generation=%lu valid=%u\r\n",
        state,
        (unsigned int)APP_RC_FUNC_COUNT,
        (unsigned int)CRSF_CHANNEL_COUNT,
        (unsigned int)control_rc_config.deadband_us,
        (unsigned int)control_rc_config.calibrated,
        (unsigned int)control_rc_config_dirty,
        (unsigned long)APP_RcConfig_GetActiveGeneration(),
        (unsigned int)APP_RcConfig_Validate(&control_rc_config));
    for (function = 0U; function < APP_RC_FUNC_COUNT; ++function) {
        const APP_RcFunctionMap *map = &control_rc_config.function[function];

        app_control_queue_proto_text(
            APP_PROTO_MSG_RC_MAP,
            "RCMAP func=%s id=%u ch=%d rev=%u min=%u mid=%u max=%u\r\n",
            APP_RcConfig_FunctionName(function),
            (unsigned int)function,
            (map->channel == APP_RC_CHANNEL_UNBOUND) ? -1 : (int)map->channel,
            (unsigned int)map->reversed,
            (unsigned int)map->min_us,
            (unsigned int)map->mid_us,
            (unsigned int)map->max_us);
    }
}

static void app_control_report_rc_live(void)
{
    uint16_t channels[CRSF_CHANNEL_COUNT];
    APP_RcInputs inputs;
    const DRV_ELRS_LinkStats *link;
    uint32_t now_ms = HAL_GetTick();
    uint8_t fresh;

    APP_ELRS_GetChannels(channels);
    APP_RcConfig_Resolve(&control_rc_config, channels, &inputs);
    link = APP_ELRS_GetLinkStats();
    fresh = APP_ELRS_IsRcFresh(now_ms, APP_CONTROL_RC_FRESH_TIMEOUT_MS);

    /*
     * 两行拆分是为了每行都留在 APP_UART_TX_TEXT_SIZE 以内：16 路各 4 位数字
     * 加分隔符已经接近上限，链路统计只能另起一行。
     */
    app_control_queue_proto_text(
        APP_PROTO_MSG_RC_LIVE,
        "RC us=%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\r\n",
        (unsigned int)channels[0], (unsigned int)channels[1],
        (unsigned int)channels[2], (unsigned int)channels[3],
        (unsigned int)channels[4], (unsigned int)channels[5],
        (unsigned int)channels[6], (unsigned int)channels[7],
        (unsigned int)channels[8], (unsigned int)channels[9],
        (unsigned int)channels[10], (unsigned int)channels[11],
        (unsigned int)channels[12], (unsigned int)channels[13],
        (unsigned int)channels[14], (unsigned int)channels[15]);
    app_control_queue_proto_text(
        APP_PROTO_MSG_RC_LIVE,
        "RC link fresh=%u frames=%lu crc_err=%lu fps_x10=%lu lq=%u rssi=%u "
        "snr=%d age_ms=%lu armed=%u bound=0x%02X\r\n",
        (unsigned int)fresh,
        (unsigned long)APP_ELRS_GetRcFrames(),
        (unsigned long)APP_ELRS_GetCrcErrors(),
        (unsigned long)DRV_ELRS_GetFpsX10(),
        (unsigned int)((link != NULL) ? link->uplink_lq : 0U),
        (unsigned int)((link != NULL) ? link->uplink_rssi_1 : 0U),
        (int)((link != NULL) ? link->uplink_snr : 0),
        (unsigned long)(now_ms - APP_ELRS_GetLastRcMs()),
        (unsigned int)APP_Stabilizer_IsArmed(),
        (unsigned int)inputs.bound_mask);
    /* 归一化值单独一行：上位机据此画摇杆十字，无需自己复现标定公式。 */
    app_control_queue_proto_text(
        APP_PROTO_MSG_RC_LIVE,
        "RC norm roll=%d pitch=%d throttle=%d yaw=%d arm=%d mode=%d thr01=%d\r\n",
        (int)(inputs.norm[APP_RC_FUNC_ROLL] * 1000.0f),
        (int)(inputs.norm[APP_RC_FUNC_PITCH] * 1000.0f),
        (int)(inputs.norm[APP_RC_FUNC_THROTTLE] * 1000.0f),
        (int)(inputs.norm[APP_RC_FUNC_YAW] * 1000.0f),
        (int)(inputs.norm[APP_RC_FUNC_ARM] * 1000.0f),
        (int)(inputs.norm[APP_RC_FUNC_MODE] * 1000.0f),
        (int)(inputs.throttle_01 * 1000.0f));
}

/*
 * 改映射等于改"哪根杆是油门"。解锁状态下改一次就可能让电机响应错通道，
 * 所以所有写操作都要求飞控 disarmed，和 IMUFRAME 的安全门同源。
 */
static uint8_t app_control_rc_write_allowed(void)
{
    return (APP_Stabilizer_IsArmed() == 0U) ? 1U : 0U;
}

static void app_control_handle_rc_map(char *tokens[], uint32_t count)
{
    APP_RcConfig candidate;
    uint8_t function;
    long channel;
    long reversed;
    long min_us;
    long mid_us;
    long max_us;
    long deadband;

    if ((count == 1U) || (strcmp(tokens[0], "RCMAP?") == 0)) {
        app_control_report_rc_map("status");
        return;
    }

    if (strcmp(tokens[1], "SET") == 0) {
        if (count != 8U) {
            app_control_report_rc_map("invalid_usage");
            return;
        }
        if (app_control_rc_write_allowed() == 0U) {
            app_control_report_rc_map("armed_blocked");
            return;
        }
        function = APP_RcConfig_FunctionFromName(tokens[2]);
        if (function >= APP_RC_FUNC_COUNT) {
            app_control_report_rc_map("bad_function");
            return;
        }
        channel  = strtol(tokens[3], NULL, 0);
        reversed = strtol(tokens[4], NULL, 0);
        min_us   = strtol(tokens[5], NULL, 0);
        mid_us   = strtol(tokens[6], NULL, 0);
        max_us   = strtol(tokens[7], NULL, 0);

        candidate = control_rc_config;
        candidate.function[function].channel =
            (channel < 0) ? APP_RC_CHANNEL_UNBOUND : (uint8_t)channel;
        candidate.function[function].reversed = (reversed != 0) ? 1U : 0U;
        candidate.function[function].min_us = (uint16_t)min_us;
        candidate.function[function].mid_us = (uint16_t)mid_us;
        candidate.function[function].max_us = (uint16_t)max_us;
        if (APP_RcConfig_Validate(&candidate) == 0U) {
            app_control_report_rc_map("rejected");
            return;
        }
        control_rc_config = candidate;
        control_rc_config_dirty = 1U;
        (void)APP_RcConfig_PublishActive(&control_rc_config);
        app_control_report_rc_map("applied_ram");
        return;
    }

    if (strcmp(tokens[1], "DEADBAND") == 0) {
        if (count != 3U) {
            app_control_report_rc_map("invalid_usage");
            return;
        }
        if (app_control_rc_write_allowed() == 0U) {
            app_control_report_rc_map("armed_blocked");
            return;
        }
        deadband = strtol(tokens[2], NULL, 0);
        candidate = control_rc_config;
        candidate.deadband_us = (uint16_t)((deadband < 0) ? 0 : deadband);
        if (APP_RcConfig_Validate(&candidate) == 0U) {
            app_control_report_rc_map("rejected");
            return;
        }
        control_rc_config = candidate;
        control_rc_config_dirty = 1U;
        (void)APP_RcConfig_PublishActive(&control_rc_config);
        app_control_report_rc_map("applied_ram");
        return;
    }

    if (strcmp(tokens[1], "CALIBRATED") == 0) {
        if ((count != 3U) || (app_control_rc_write_allowed() == 0U)) {
            app_control_report_rc_map(
                (count != 3U) ? "invalid_usage" : "armed_blocked");
            return;
        }
        control_rc_config.calibrated =
            (strtol(tokens[2], NULL, 0) != 0) ? 1U : 0U;
        control_rc_config_dirty = 1U;
        (void)APP_RcConfig_PublishActive(&control_rc_config);
        app_control_report_rc_map("applied_ram");
        return;
    }

    if (strcmp(tokens[1], "RESET") == 0) {
        if (app_control_rc_write_allowed() == 0U) {
            app_control_report_rc_map("armed_blocked");
            return;
        }
        APP_RcConfig_Defaults(&control_rc_config);
        control_rc_config_dirty = 1U;
        (void)APP_RcConfig_PublishActive(&control_rc_config);
        app_control_report_rc_map("reset_ram");
        return;
    }

    if (strcmp(tokens[1], "COMMIT") == 0) {
        APP_FlashService_Status save_status;

        if (app_control_rc_write_allowed() == 0U) {
            app_control_report_rc_map("armed_blocked");
            return;
        }
        if (APP_RcConfig_Validate(&control_rc_config) == 0U) {
            app_control_report_rc_map("rejected");
            return;
        }
        save_status = app_control_save_config();
        if (save_status != APP_FLASH_SERVICE_OK) {
            control_config.last_flash_status = (uint8_t)save_status;
            app_control_report_rc_map("commit_failed");
            return;
        }
        control_config.last_flash_status = (uint8_t)save_status;
        control_config.loaded_from_flash = 1U;
        control_config.flash_valid = 1U;
        control_rc_config_dirty = 0U;
        app_control_report_rc_map("committed");
        return;
    }

    app_control_report_rc_map("invalid_usage");
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
    app_control_report_usb_cdc_stats();
}

static APP_FlashService_Status app_control_load_config(void)
{
    APP_ControlFlashRecord record;
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
        (record.size == (sizeof(record.config) + sizeof(record.coax_tunables) +
                         sizeof(record.rc_config)))) {
        checksum = app_control_checksum((const uint8_t *)&record.config,
                                        record.size);
        if (checksum != record.checksum) {
            return APP_FLASH_SERVICE_ERROR;
        }
        control_config = record.config;
        app_control_apply_coax_tunables(&record.coax_tunables);
        app_control_apply_rc_config(&record.rc_config);
    } else if ((record.version == APP_CONTROL_CFG_VERSION_V16) &&
               (record.size == (sizeof(record.config) +
                                sizeof(record.coax_tunables)))) {
        APP_ControlFlashRecordV16 legacy_record;

        status = APP_FlashService_ReadData(APP_CONTROL_CFG_ADDRESS,
                                           (uint8_t *)&legacy_record,
                                           sizeof(legacy_record));
        if (status != APP_FLASH_SERVICE_OK) {
            return status;
        }
        checksum = app_control_checksum((const uint8_t *)&legacy_record.config,
                                        legacy_record.size);
        if (checksum != legacy_record.checksum) {
            return APP_FLASH_SERVICE_ERROR;
        }
        control_config = legacy_record.config;
        app_control_apply_coax_tunables(&legacy_record.coax_tunables);
        /* V16 没有遥控映射，装出厂默认 —— 与旧固件写死的 CH1..CH6 完全一致。 */
        app_control_apply_rc_config(NULL);
    } else if ((record.version == APP_CONTROL_CFG_VERSION_V15) &&
               (record.size == (sizeof(record.config) +
                                sizeof(APP_ControlCoaxTunableParamsV15)))) {
        APP_ControlFlashRecordV15 legacy_record;

        status = APP_FlashService_ReadData(APP_CONTROL_CFG_ADDRESS,
                                           (uint8_t *)&legacy_record,
                                           sizeof(legacy_record));
        if (status != APP_FLASH_SERVICE_OK) {
            return status;
        }

        checksum = app_control_checksum((const uint8_t *)&legacy_record.config,
                                        legacy_record.size);
        if (checksum != legacy_record.checksum) {
            return APP_FLASH_SERVICE_ERROR;
        }
        control_config = legacy_record.config;
        app_control_apply_coax_tunables_v15(&legacy_record.coax_tunables);
        app_control_apply_rc_config(NULL);
    } else {
        return APP_FLASH_SERVICE_BAD_ID;
    }
    control_config.loaded_from_flash = 1U;
    control_config.flash_valid = 1U;
    return APP_FLASH_SERVICE_OK;
}

static APP_FlashService_Status app_control_save_config(void)
{
    APP_ControlFlashRecord record;
    APP_FlashService_Status status;

    memset(&record, 0xFF, sizeof(record));
    record.magic = APP_CONTROL_CFG_MAGIC;
    record.version = APP_CONTROL_CFG_VERSION;
    record.size = (uint16_t)(sizeof(record.config) + sizeof(record.coax_tunables) +
                             sizeof(record.rc_config));
    record.config = control_config;
    record.config.loaded_from_flash = 1U;
    record.config.flash_valid = 1U;
    app_control_capture_coax_tunables(&record.coax_tunables);
    record.rc_config = control_rc_config;
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

    if (strcmp(tokens[1], "JOG") == 0) {
        /* 保持型地面点动，解析与回复归 app_servo_jog.c（通信上下文）。 */
        APP_ServoJog_HandleCommand(tokens, count, HAL_GetTick());
        return;
    }

    if (strcmp(tokens[1], "FB") == 0) {
        uint32_t duration_ms;
        uint32_t timeout_ms = 10U;

        if (count < 3U) {
            APP_Control_QueueText("ERR usage SERVO FB START|SWEEP|STEP|STATUS|STOP\r\n");
            return;
        }
        if (strcmp(tokens[2], "STATUS") == 0) {
            APP_ServoFeedbackBench_ReportStatus(HAL_GetTick());
            return;
        }
        if (strcmp(tokens[2], "STOP") == 0) {
            APP_ServoFeedbackBench_Stop("command", HAL_GetTick());
            return;
        }
        if (strcmp(tokens[2], "START") == 0) {
            uint32_t rate_hz;

            if ((count < 5U) ||
                (app_control_parse_u32(tokens[3], &rate_hz) == 0U) ||
                (app_control_parse_u32(tokens[4], &duration_ms) == 0U) ||
                ((count >= 6U) &&
                 (app_control_parse_u32(tokens[5], &timeout_ms) == 0U))) {
                APP_Control_QueueText(
                    "ERR usage SERVO FB START rate_hz duration_ms [timeout_ms]\r\n");
                return;
            }
            if (APP_ServoFeedbackBench_Start(rate_hz,
                                             duration_ms,
                                             timeout_ms,
                                             HAL_GetTick()) == 0U) {
                APP_Control_QueueText(
                    "ERR servo_fb start range rate=1..200 duration=1000..60000 timeout=2..100 or active\r\n");
            }
            return;
        }
        if (strcmp(tokens[2], "SWEEP") == 0) {
            if ((count < 4U) ||
                (app_control_parse_u32(tokens[3], &duration_ms) == 0U) ||
                ((count >= 5U) &&
                 (app_control_parse_u32(tokens[4], &timeout_ms) == 0U))) {
                APP_Control_QueueText(
                    "ERR usage SERVO FB SWEEP duration_ms [timeout_ms]\r\n");
                return;
            }
            if (APP_ServoFeedbackBench_StartSweep(duration_ms,
                                                  timeout_ms,
                                                  HAL_GetTick()) == 0U) {
                APP_Control_QueueText(
                    "ERR servo_fb sweep duration=1000..60000 timeout=2..100 or active\r\n");
            }
            return;
        }
        if (strcmp(tokens[2], "STEP") == 0) {
            uint32_t servo_index;
            uint32_t delta_us;
            uint32_t rate_hz = 100U;
            uint32_t hold_ms = 600U;

            if ((count < 5U) ||
                (app_control_parse_u32(tokens[3], &servo_index) == 0U) ||
                (app_control_parse_u32(tokens[4], &delta_us) == 0U) ||
                ((count >= 6U) &&
                 (app_control_parse_u32(tokens[5], &rate_hz) == 0U)) ||
                ((count >= 7U) &&
                 (app_control_parse_u32(tokens[6], &hold_ms) == 0U)) ||
                ((count >= 8U) &&
                 (app_control_parse_u32(tokens[7], &timeout_ms) == 0U))) {
                APP_Control_QueueText(
                    "ERR usage SERVO FB STEP index delta_us [rate_hz] [hold_ms] [timeout_ms]\r\n");
                return;
            }
            if (APP_ServoFeedbackBench_StartStep(servo_index,
                                                 delta_us,
                                                 rate_hz,
                                                 hold_ms,
                                                 timeout_ms,
                                                 HAL_GetTick()) == 0U) {
                APP_Control_QueueText(
                    "ERR servo_fb step index=0..1 delta=20..200 rate=1..100 hold=300..5000 samples<=256 timeout=2..100 or active\r\n");
            }
            return;
        }

        APP_Control_QueueText("ERR unknown servo fb subcmd %s\r\n", tokens[2]);
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
        APP_Control_QueueText("ERR usage IDENT ARM|DISARM|STOP|ATT|STEP|DOUBLET|PRBS|CENTER|APPLY|?\r\n");
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

    if (strcmp(tokens[1], "ATT") == 0) {
        if (count < 3U) {
            APP_Control_QueueText("ERR usage IDENT ATT PRBS roll|pitch amp_mdeg=<v> bit_ms=<v> duration_ms=<v> [seed=<v>]\r\n");
            return;
        }
        if ((strcmp(tokens[2], "?") == 0) ||
            (strcmp(tokens[2], "STATUS") == 0)) {
            APP_Ident_ReportStatus();
            return;
        }
        if (strcmp(tokens[2], "STOP") == 0) {
            APP_IdentAtt_Stop("command");
            return;
        }
        if (strcmp(tokens[2], "PRBS") == 0) {
            uint32_t bit_ms;
            uint32_t duration_ms;
            uint32_t seed = 1U;
            int32_t amp_mdeg;
            const char *amp_text;
            const char *bit_text;
            const char *duration_text;
            const char *seed_text;

            if (count < 4U) {
                APP_Control_QueueText("ERR usage IDENT ATT PRBS roll|pitch amp_mdeg=<v> bit_ms=<v> duration_ms=<v> [seed=<v>]\r\n");
                return;
            }
            amp_text = app_control_token_value(tokens, count, "amp_mdeg");
            bit_text = app_control_token_value(tokens, count, "bit_ms");
            duration_text = app_control_token_value(tokens, count, "duration_ms");
            seed_text = app_control_token_value(tokens, count, "seed");
            if ((seed_text != NULL) &&
                (app_control_parse_u32(seed_text, &seed) == 0U)) {
                APP_Control_QueueText("ERR ident att seed\r\n");
                return;
            }
            if ((amp_text == NULL) || (bit_text == NULL) ||
                (duration_text == NULL) ||
                (app_control_parse_i32(amp_text, &amp_mdeg) == 0U) ||
                (app_control_parse_u32(bit_text, &bit_ms) == 0U) ||
                (app_control_parse_u32(duration_text, &duration_ms) == 0U)) {
                APP_Control_QueueText("ERR usage IDENT ATT PRBS roll|pitch amp_mdeg=<v> bit_ms=<v> duration_ms=<v> [seed=<v>]\r\n");
                return;
            }
            (void)APP_IdentAtt_StartPrbs(tokens[3],
                                         amp_mdeg,
                                         bit_ms,
                                         duration_ms,
                                         seed);
            return;
        }
        APP_Control_QueueText("ERR unknown ident att subcmd %s\r\n", tokens[2]);
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
        APP_Control_QueueText("ERR usage FLOW TX hex... | FLOW RX [max] | FLOW XCV rx_len hex...\r\n");
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

static void app_control_report_flow(void)
{
    StabilizerFlowCompensationSnapshot snapshot;

    APP_OpticalFlow_Report();
    memset(&snapshot, 0, sizeof(snapshot));
    if (APP_Stabilizer_ReadFlowCompensationSnapshot(&snapshot) == 0U) {
        APP_Control_QueueText(
            "FLOW comp valid=0 export=canonical_flu reason=no_snapshot\r\n");
        return;
    }
    APP_Control_QueueText(
        "FLOW comp valid=%u sample_ms=%lu contract=%u orientation=%u "
        "source=controller_legacy_x_forward_y_right export=canonical_flu "
        "sensor_vx_mm_s=%ld sensor_vy_mm_s=%ld "
        "corr_vx_mm_s=%ld corr_vy_mm_s=%ld\r\n",
        (unsigned int)snapshot.valid,
        (unsigned long)snapshot.sample_ms,
        (unsigned int)snapshot.frame_contract,
        (unsigned int)snapshot.orientation_code,
        (long)app_control_acceptance_milli(
            snapshot.sensor_velocity_flu_m_s[0]),
        (long)app_control_acceptance_milli(
            snapshot.sensor_velocity_flu_m_s[1]),
        (long)app_control_acceptance_milli(
            snapshot.corrected_velocity_flu_m_s[0]),
        (long)app_control_acceptance_milli(
            snapshot.corrected_velocity_flu_m_s[1]));
    APP_Control_QueueText(
        "FLOW comp_terms sample_ms=%lu export=canonical_flu "
        "opt_rot_vx_mm_s=%ld opt_rot_vy_mm_s=%ld "
        "offset_rot_vx_mm_s=%ld offset_rot_vy_mm_s=%ld\r\n",
        (unsigned long)snapshot.sample_ms,
        (long)app_control_acceptance_milli(
            snapshot.optical_rot_comp_flu_m_s[0]),
        (long)app_control_acceptance_milli(
            snapshot.optical_rot_comp_flu_m_s[1]),
        (long)app_control_acceptance_milli(
            snapshot.offset_rot_comp_flu_m_s[0]),
        (long)app_control_acceptance_milli(
            snapshot.offset_rot_comp_flu_m_s[1]));
}

static void app_control_handle_pid(char **tokens, uint32_t count)
{
    const char *kp_text;
    const char *kd_text;
    float kp;
    float kd;
    const char *kp_name;
    const char *kd_name;
    uint8_t axis_has_kp = 0U;

    if ((count == 1U) || ((count >= 2U) && (strcmp(tokens[1], "?") == 0)) ||
        ((count >= 2U) && (strcmp(tokens[1], "GET") == 0))) {
        app_control_report_pid_legacy();
        return;
    }

    if ((count < 3U) || (strcmp(tokens[1], "SET") != 0)) {
        APP_Control_QueueText("ERR usage PID SET roll|pitch|yaw [kp=<float>] [kd=<float>]\r\n");
        return;
    }

    if (strcmp(tokens[2], "roll") == 0) {
        kp_name = "coax.roll_angle_kp";
        kd_name = "coax.roll_rate_kd";
        axis_has_kp = 1U;
    } else if (strcmp(tokens[2], "pitch") == 0) {
        kp_name = "coax.pitch_angle_kp";
        kd_name = "coax.pitch_rate_kd";
        axis_has_kp = 1U;
    } else if (strcmp(tokens[2], "yaw") == 0) {
        kp_name = "coax.yaw_angle_kp";
        kd_name = "coax.yaw_rate_kd";
        axis_has_kp = 1U;
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
        if (axis_has_kp == 0U) {
            APP_Control_QueueText("ERR pid kp unused for %s\r\n", tokens[2]);
            return;
        }
        if (app_control_parse_f32(kp_text, &kp) == 0U) {
            APP_Control_QueueText("ERR usage PID SET roll|pitch|yaw [kp=<float>] [kd=<float>]\r\n");
            return;
        }
        kp = app_control_param_from_ui_value(kp_name, kp);
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
        kd = app_control_param_from_ui_value(kd_name, kd);
        if (DRV_COAX_CTRL_SetParam(kd_name, kd) != 0U) {
            app_control_schedule_flash_autosave();
        }
    }

    APP_Control_QueueText("OK pid axis=%s target=coax\r\n", tokens[2]);
    if (kp_name != NULL) {
        app_control_report_coax_param_by_name(kp_name);
    }
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
        { "roll_angle_kp",  "coax.roll_angle_kp"  },
        { "pitch_angle_kp", "coax.pitch_angle_kp" },
        { "roll_rate_kd",   "coax.roll_rate_kd"   },
        { "pitch_rate_kd",  "coax.pitch_rate_kd"  },
        { "yaw_angle_kp",   "coax.yaw_angle_kp"   },
        { "yaw_rate_kd",    "coax.yaw_rate_kd"    },
        { "pos_x_kp",       "coax.pos_x_kp"       },
        { "pos_y_kp",       "coax.pos_y_kp"       },
        { "pos_z_kp",       "coax.pos_z_kp"       },
        { "pos_z_ki",       "coax.pos_z_ki"       },
        { "vel_x_kd",       "coax.vel_x_kd"       },
        { "vel_y_kd",       "coax.vel_y_kd"       },
        { "vel_z_kd",       "coax.vel_z_kd"       },
        { "vel_loop_enable", "coax.vel_loop_enable" },
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
            if ((value <= 0.0f) || (value > APP_CONTROL_TILT_LIMIT_MAX_DEG)) {
                APP_Control_QueueText("ERR angle range\r\n");
                return 1U;
            }
            value *= APP_CONTROL_DEG_TO_RAD;
        }

        value = app_control_param_from_ui_value(map[map_index].param_name, value);
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

        value = app_control_param_from_ui_value(name, value);
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
    kp = app_control_param_to_ui_value("coax.roll_angle_kp", kp);
    kd = app_control_param_to_ui_value("coax.roll_rate_kd", kd);
    app_control_format_float(kp, kp_text, (uint32_t)sizeof(kp_text));
    app_control_format_float(kd, kd_text, (uint32_t)sizeof(kd_text));
    app_control_queue_proto_text(APP_PROTO_MSG_PID_RECORD,
                                 "PID axis=roll kp=%s ki=0 kd=%s source=coax\r\n",
                                 kp_text,
                                 kd_text);

    (void)DRV_COAX_CTRL_GetParam("coax.pitch_angle_kp", &kp);
    (void)DRV_COAX_CTRL_GetParam("coax.pitch_rate_kd", &kd);
    kp = app_control_param_to_ui_value("coax.pitch_angle_kp", kp);
    kd = app_control_param_to_ui_value("coax.pitch_rate_kd", kd);
    app_control_format_float(kp, kp_text, (uint32_t)sizeof(kp_text));
    app_control_format_float(kd, kd_text, (uint32_t)sizeof(kd_text));
    app_control_queue_proto_text(APP_PROTO_MSG_PID_RECORD,
                                 "PID axis=pitch kp=%s ki=0 kd=%s source=coax\r\n",
                                 kp_text,
                                 kd_text);

    (void)DRV_COAX_CTRL_GetParam("coax.yaw_angle_kp", &kp);
    (void)DRV_COAX_CTRL_GetParam("coax.yaw_rate_kd", &kd);
    kp = app_control_param_to_ui_value("coax.yaw_angle_kp", kp);
    kd = app_control_param_to_ui_value("coax.yaw_rate_kd", kd);
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
        APP_Control_QueueText("ERR usage PARAM SET coax.<param> value\r\n");
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
        APP_Control_QueueText("ERR usage PARAM SET coax.<param> value\r\n");
        return;
    }

    value = app_control_param_from_ui_value(name, value);
    if (DRV_COAX_CTRL_SetParam(name, value) == 0U) {
        APP_Control_QueueText("ERR param target %s\r\n", name);
        return;
    }
    app_control_schedule_flash_autosave();

    app_control_format_float(app_control_param_to_ui_value(name, value),
                             formatted,
                             (uint32_t)sizeof(formatted));
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

    APP_Boot_Init();
    APP_Acceptance_Init();
    APP_FlightCalibration_ResetActive();
    APP_FlightCalibration_UploadReset(&control_imucal_upload);
    APP_FlightCalibration_Defaults(&control_imucal_confirmed);
    memset(&control_imucal_preview, 0, sizeof(control_imucal_preview));
    memset(&control_imucal_pending_record, 0,
           sizeof(control_imucal_pending_record));
    control_imucal_confirmed_generation = 0U;
    control_imucal_preview_generation = 0U;
    control_imucal_apply_sequence = 0U;
    control_imucal_last_request = 0U;
    control_imucal_confirmed_valid = 0U;
    control_imucal_applied = 0U;
    control_imucal_commit_pending = 0U;
    app_control_imucal_set_event("init", "none");
    APP_Stabilizer_SetImuCalibrationCandidateArmLock(0U);
    memset(&control_servocal_preview, 0,
           sizeof(control_servocal_preview));
    memset(&control_servocal_pending_record, 0,
           sizeof(control_servocal_pending_record));
    control_servocal_preview_generation = 0U;
    control_servocal_last_request = 0U;
    control_servocal_applied = 0U;
    control_servocal_commit_pending = 0U;
    app_control_servocal_set_event("init", "none");
    APP_Stabilizer_SetServoCalibrationCandidateArmLock(0U);
    app_control_defaults(&control_config);
    control_imuframe_confirmed_code = APP_SENSOR_FLU_ORIENTATION_LEGACY;
    control_imuframe_pending_code = APP_SENSOR_FLU_ORIENTATION_LEGACY;
    control_imuframe_pending_valid = 0U;
    control_imuframe_generation_valid = 0U;
    control_imuframe_last_dirty = 0U;
    control_imuframe_boot_selection_pending = 1U;
    control_imuframe_param_generation = 0U;
    control_imuframe_last_request = 0U;
    app_control_imuframe_sync_param();
    control_wifi_reset_pending = 0U;
    control_wifi_reset_deadline_ms = 0U;
    APP_Ident_Init();
    APP_ServoFeedback_Init();
    APP_ServoFeedbackBench_Init();
    /* Flash 记录无效时也要有一份可用的映射，否则控制环只能退回默认且无从上报。 */
    app_control_apply_rc_config(NULL);
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

/* 补发 500Hz 舵机手势标定状态机缓存的事件文本（见 app_servo_cal.c 的通告注释）。 */
static void app_control_service_servo_cal_notice(void)
{
    char notice[64];

    if (APP_ServoCal_TakeNotice(notice, (uint16_t)sizeof(notice)) != 0U) {
        APP_Control_QueueText("%s", notice);
    }
}

/* 补发地面点动模块在控制环上下文缓存的事件文本（见 app_servo_jog.c）。 */
static void app_control_service_servo_jog_notice(void)
{
    char notice[64];

    if (APP_ServoJog_TakeNotice(notice, (uint16_t)sizeof(notice)) != 0U) {
        APP_Control_QueueText("%s", notice);
    }
}

static void app_control_tick_common(uint8_t emit_heartbeat)
{
    app_control_service_servo_cal_notice();
    app_control_service_servo_jog_notice();
    app_control_service_boot();
    app_control_imuframe_sync_param();
    app_control_service_imucal();
    app_control_service_servocal();
    APP_Acceptance_Service(HAL_GetTick());
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

/*
 * TELEM —— 遥测通道 schema 查询。
 *
 *   TELEM?              -> 一行表头（版本/通道数/速率/分页大小/指纹）
 *   TELEM CH from=<n>   -> 至多 APP_TELEM_PAGE_SIZE 条通道行 + 一行页脚
 *
 * 分页而非一次性回全表：uartTxQueue 深度 32 且满时丢最旧的一条，28 条通道
 * 一次推进队列在慢链路上会静默丢掉开头几条，而 schema 丢一条就会让上位机
 * 建错表。分页把单次回包压到 7 条，并且天然可重试。
 */
static void app_control_handle_telem(char **tokens, uint32_t count)
{
    const char *from_text;
    uint32_t    from;

    if (tokens == NULL) {
        return;
    }

    if (strcmp(tokens[0], "TELEM?") == 0) {
        if (count != 1U) {
            APP_Control_QueueText("ERR usage TELEM?\r\n");
            return;
        }
        APP_Telemetry_ReportHeader();
        return;
    }

    if ((count < 2U) || (strcmp(tokens[1], "CH") != 0)) {
        APP_Control_QueueText("ERR usage TELEM CH from=<n>\r\n");
        return;
    }

    from_text = app_control_token_value(tokens, count, "from");
    if ((count != 3U) || (from_text == NULL) ||
        (app_control_parse_u32(from_text, &from) == 0U)) {
        APP_Control_QueueText("ERR usage TELEM CH from=<n>\r\n");
        return;
    }

    APP_Telemetry_ReportPage(from);
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
    } else if ((strcmp(tokens[0], "BOOT?") == 0) ||
               (strcmp(tokens[0], "BOOT") == 0)) {
        app_control_handle_boot(tokens, count);
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
    } else if ((strcmp(tokens[0], "IMUCAL?") == 0) ||
               (strcmp(tokens[0], "IMUCAL") == 0)) {
        app_control_handle_imucal(tokens, count);
    } else if ((strcmp(tokens[0], "SERVOCAL?") == 0) ||
               (strcmp(tokens[0], "SERVOCAL") == 0)) {
        app_control_handle_servocal(tokens, count);
    } else if ((strcmp(tokens[0], "ACCEPT?") == 0) ||
               (strcmp(tokens[0], "ACCEPT") == 0)) {
        app_control_handle_acceptance(tokens, count);
    } else if ((strcmp(tokens[0], "IMUFRAME?") == 0) ||
               (strcmp(tokens[0], "IMUFRAME") == 0)) {
        app_control_handle_imuframe(tokens, count);
    } else if (strcmp(tokens[0], "RC?") == 0) {
        app_control_report_rc_live();
    } else if ((strcmp(tokens[0], "RCMAP?") == 0) ||
               (strcmp(tokens[0], "RCMAP") == 0)) {
        app_control_handle_rc_map(tokens, count);
    } else if (strcmp(tokens[0], "FLOW?") == 0) {
        app_control_report_flow();
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
    } else if ((strcmp(tokens[0], "IMUCAP?") == 0) ||
               (strcmp(tokens[0], "IMUCAP") == 0)) {
        app_control_handle_imu_capture(tokens, count);
    } else if ((strcmp(tokens[0], "TELEM?") == 0) ||
               (strcmp(tokens[0], "TELEM") == 0)) {
        app_control_handle_telem(tokens, count);
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

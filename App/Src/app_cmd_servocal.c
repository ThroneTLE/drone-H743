#include "app_control.h"
#include "app_control_internal.h"

#include "app_acceptance.h"
#include "app_flight_calibration.h"
#include "app_proto.h"
#include "app_sensor.h"
#include "app_stabilizer.h"
#include "drv_coax_ctrl.h"
#include "svc_param.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define app_control_imuframe_sync_param app_control_internal_imuframe_sync_param
#define app_control_imucal_safety app_control_internal_imucal_safety
#define control_imucal_confirmed \
    (*(const APP_FlightCalibration *)app_control_internal_imucal_confirmed_record())
#define control_imucal_confirmed_generation \
    app_control_internal_imucal_confirmed_generation()
#define control_imucal_confirmed_valid app_control_internal_imucal_confirmed_valid()
#define control_imucal_upload \
    ((APP_FlightCalibrationUpload){ \
        .state = (APP_FlightCalibrationUploadState) \
            app_control_internal_imucal_upload_state() \
    })
#define control_imucal_applied app_control_internal_imucal_applied()
#define control_imucal_commit_pending app_control_internal_imucal_commit_pending()
#define control_imuframe_confirmed_code \
    app_control_internal_imuframe_confirmed_code()

static APP_FlightCalibration control_servocal_preview;
static APP_FlightCalibration control_servocal_pending_record;
static uint32_t control_servocal_preview_generation;
static uint32_t control_servocal_last_request;
static uint8_t control_servocal_applied;
static uint8_t control_servocal_commit_pending;
static const char *control_servocal_last_event;
static const char *control_servocal_last_reason;

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

void app_control_handle_servocal(char **tokens, uint32_t count)
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

void app_control_service_servocal(void)
{
    if ((control_servocal_commit_pending != 0U) &&
        (SVC_Param_IsDirty() != 0U) &&
        (control_servocal_last_request == 0U)) {
        control_servocal_last_request = SVC_Param_RequestSaveBlob();
    }
}

void app_cmd_servocal_init(void)
{
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
}

void app_cmd_servocal_on_persisted(const void *record_ptr)
{
#define record ((const APP_FlightCalibration *)record_ptr)
#define calibration (*record)
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
#undef calibration
#undef record
}

uint8_t app_cmd_servocal_is_busy(void)
{
    return ((control_servocal_applied != 0U) ||
            (control_servocal_commit_pending != 0U)) ? 1U : 0U;
}

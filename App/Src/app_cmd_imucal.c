#include "app_control.h"
#include "app_control_internal.h"

#include "app_firmware_identity.h"
#include "app_flight_calibration.h"
#include "app_proto.h"
#include "app_sensor.h"
#include "app_stabilizer.h"
#include "svc_param.h"

#include "svc_timestamp.h"
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define app_control_imuframe_sync_param app_control_internal_imuframe_sync_param
#define app_control_imucal_safety app_control_internal_imucal_safety
#define control_imucal_confirmed \
    (*(const APP_FlightCalibration *)app_control_internal_imucal_confirmed_record())
#define control_imucal_confirmed_generation \
    (*app_control_internal_imucal_confirmed_generation_slot())
#define control_imucal_confirmed_valid app_control_internal_imucal_confirmed_valid()
#define control_imucal_apply_sequence \
    (*app_control_internal_imucal_apply_sequence_slot())

static APP_FlightCalibrationUpload control_imucal_upload;
static APP_FlightCalibration control_imucal_preview;
static APP_FlightCalibration control_imucal_pending_record;
static uint32_t control_imucal_preview_generation;
static uint32_t control_imucal_last_request;
static uint8_t control_imucal_applied;
static uint8_t control_imucal_commit_pending;
static const char *control_imucal_last_event;
static const char *control_imucal_last_reason;

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

void app_control_handle_imucal(char **tokens, uint32_t count)
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
    uint32_t now_ms = SVC_Timestamp_Ms();

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
    if (app_cmd_servocal_is_busy() != 0U) {
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

void app_control_service_imucal(void)
{
    if ((control_imucal_applied == 0U) &&
        (control_imucal_commit_pending == 0U) &&
        (APP_FlightCalibration_UploadExpire(&control_imucal_upload,
                                             SVC_Timestamp_Ms()) != 0U)) {
        APP_Stabilizer_SetImuCalibrationCandidateArmLock(0U);
        app_control_imucal_set_event("expired", "timeout");
    }
    if ((control_imucal_commit_pending != 0U) &&
        (SVC_Param_IsDirty() != 0U) &&
        (control_imucal_last_request == 0U)) {
        control_imucal_last_request = SVC_Param_RequestSaveBlob();
    }
}

void *app_cmd_imucal_upload_slot(void)
{
    return &control_imucal_upload;
}

void app_cmd_imucal_clear_candidate(void)
{
    app_control_imucal_clear_candidate();
}

void app_cmd_imucal_set_event(const char *event, const char *reason)
{
    app_control_imucal_set_event(event, reason);
}

void *app_cmd_imucal_preview_slot(void)
{
    return &control_imucal_preview;
}

void *app_cmd_imucal_pending_record_slot(void)
{
    return &control_imucal_pending_record;
}

uint32_t *app_cmd_imucal_preview_generation_slot(void)
{
    return &control_imucal_preview_generation;
}

uint32_t *app_cmd_imucal_last_request_slot(void)
{
    return &control_imucal_last_request;
}

uint8_t *app_cmd_imucal_applied_slot(void)
{
    return &control_imucal_applied;
}

uint8_t *app_cmd_imucal_commit_pending_slot(void)
{
    return &control_imucal_commit_pending;
}

const char **app_cmd_imucal_last_event_slot(void)
{
    return &control_imucal_last_event;
}

const char **app_cmd_imucal_last_reason_slot(void)
{
    return &control_imucal_last_reason;
}

uint8_t app_control_internal_imucal_upload_state(void)
{
    return (uint8_t)control_imucal_upload.state;
}

uint8_t app_control_internal_imucal_applied(void)
{
    return control_imucal_applied;
}

uint8_t app_control_internal_imucal_commit_pending(void)
{
    return control_imucal_commit_pending;
}

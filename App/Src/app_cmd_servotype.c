#include "app_control.h"
#include "app_control_internal.h"

#include "app_acceptance.h"
#include "app_flight_calibration.h"
#include "app_proto.h"
#include "app_servo_cal.h"
#include "app_servo_type.h"
#include "app_stabilizer.h"
#include "svc_param.h"

#include <stdint.h>
#include <string.h>

#define control_imucal_confirmed \
    (*(const APP_FlightCalibration *)app_control_internal_imucal_confirmed_record())
#define control_imucal_confirmed_valid app_control_internal_imucal_confirmed_valid()

static APP_ServoType control_servotype_persisted;
static APP_ServoType control_servotype_preview;
static APP_FlightCalibration control_servotype_pending_record;
static uint32_t control_servotype_record_generation;
static uint32_t control_servotype_last_request;
static uint8_t control_servotype_persisted_explicit;
static uint8_t control_servotype_applied;
static uint8_t control_servotype_commit_pending;
static const char *control_servotype_state;

static void app_servotype_set_state(const char *state)
{
    control_servotype_state = (state != NULL) ? state : "status";
}

static uint8_t app_servotype_dirty(void)
{
    return ((control_servotype_applied != 0U) ||
            (control_servotype_commit_pending != 0U)) ? 1U : 0U;
}

static void app_servotype_report(void)
{
    APP_ServoType active = APP_ServoType_GetActive();

    app_control_queue_proto_text(
        APP_PROTO_MSG_SERVO_TYPE,
        "SERVOTYPE state=%s active=%s persisted=%s dirty=%u valid=%u "
        "explicit=%u generation=%lu record_generation=%lu request=%lu\r\n",
        (control_servotype_state != NULL) ? control_servotype_state : "status",
        APP_ServoType_Name(active),
        APP_ServoType_Name(control_servotype_persisted),
        (unsigned int)app_servotype_dirty(),
        (unsigned int)APP_ServoType_IsValid(active),
        (unsigned int)control_servotype_persisted_explicit,
        (unsigned long)APP_ServoType_GetGeneration(),
        (unsigned long)control_servotype_record_generation,
        (unsigned long)control_servotype_last_request);
}

static uint8_t app_servotype_transaction_available(void)
{
    if ((APP_Stabilizer_IsArmed() != 0U) ||
        (APP_Acceptance_IsActive() != 0U) ||
        (APP_ServoCal_IsActive() != 0U) ||
        (app_cmd_servocal_is_busy() != 0U) ||
        (app_control_internal_imucal_applied() != 0U) ||
        (app_control_internal_imucal_commit_pending() != 0U) ||
        (app_control_internal_imucal_upload_state() != 0U) ||
        (SVC_Param_IsDirty() != 0U)) {
        return 0U;
    }
    return 1U;
}

void app_cmd_servotype_init(void)
{
    control_servotype_persisted = APP_SERVO_TYPE_BUS;
    control_servotype_preview = APP_SERVO_TYPE_BUS;
    memset(&control_servotype_pending_record, 0,
           sizeof(control_servotype_pending_record));
    control_servotype_record_generation = 0U;
    control_servotype_last_request = 0U;
    control_servotype_persisted_explicit = 0U;
    control_servotype_applied = 0U;
    control_servotype_commit_pending = 0U;
    app_servotype_set_state("init");
    APP_ServoType_ResetActive();
}

void app_cmd_servotype_on_persisted(const void *record_ptr)
{
    const APP_FlightCalibration *record =
        (const APP_FlightCalibration *)record_ptr;
    APP_ServoType persisted = APP_SERVO_TYPE_BUS;
    uint8_t explicit_value = APP_FlightCalibration_BuildServoType(
        record, &persisted);

    control_servotype_persisted = persisted;
    control_servotype_persisted_explicit = explicit_value;
    control_servotype_record_generation =
        (record != NULL) ? record->calibration_generation : 0U;

    if (control_servotype_commit_pending != 0U) {
        if ((record != NULL) &&
            (memcmp(record, &control_servotype_pending_record,
                    sizeof(*record)) == 0)) {
            app_servotype_set_state("committed");
        } else {
            app_servotype_set_state("commit_failed");
            (void)APP_ServoType_PublishActive(persisted);
        }
        control_servotype_applied = 0U;
        control_servotype_commit_pending = 0U;
        control_servotype_last_request = 0U;
        memset(&control_servotype_pending_record, 0,
               sizeof(control_servotype_pending_record));
        return;
    }

    if (control_servotype_applied != 0U) {
        app_servotype_set_state("reverted_persisted_changed");
        control_servotype_applied = 0U;
    } else {
        app_servotype_set_state("loaded");
    }
    control_servotype_preview = persisted;
    (void)APP_ServoType_PublishActive(persisted);
}

void app_control_handle_servotype(char **tokens, uint32_t count)
{
    APP_ServoType requested;
    const char *type_text;
    uint8_t encoded[sizeof(APP_FlightCalibration)];
    uint32_t encoded_size;
    SVC_ParamStatus param_status;

    if ((tokens == NULL) || (count == 0U)) {
        return;
    }
    app_control_internal_imuframe_sync_param();

    if ((count == 1U) && (strcmp(tokens[0], "SERVOTYPE?") == 0)) {
        app_servotype_set_state("status");
        app_servotype_report();
        return;
    }
    if ((count < 2U) || (strcmp(tokens[0], "SERVOTYPE") != 0)) {
        APP_Control_QueueText(
            "ERR usage SERVOTYPE? | SERVOTYPE APPLY type=bus|pwm | "
            "SERVOTYPE REVERT | SERVOTYPE COMMIT\r\n");
        return;
    }

    if (APP_Stabilizer_IsArmed() != 0U) {
        app_servotype_set_state("armed_blocked");
        app_servotype_report();
        return;
    }

    if (strcmp(tokens[1], "APPLY") == 0) {
        type_text = app_control_token_value(tokens, count, "type");
        if ((count != 3U) ||
            (APP_ServoType_FromName(type_text, &requested) == 0U) ||
            (control_servotype_applied != 0U) ||
            (control_servotype_commit_pending != 0U) ||
            (control_imucal_confirmed_valid == 0U) ||
            (app_servotype_transaction_available() == 0U)) {
            app_servotype_set_state("apply_rejected");
            app_servotype_report();
            return;
        }
        control_servotype_preview = requested;
        control_servotype_applied = 1U;
        control_servotype_last_request = 0U;
        (void)APP_ServoType_PublishActive(requested);
        app_servotype_set_state("applied");
        app_servotype_report();
        return;
    }

    if (strcmp(tokens[1], "REVERT") == 0) {
        if ((count != 2U) || (control_servotype_applied == 0U) ||
            (control_servotype_commit_pending != 0U)) {
            app_servotype_set_state("revert_rejected");
            app_servotype_report();
            return;
        }
        control_servotype_preview = control_servotype_persisted;
        control_servotype_applied = 0U;
        (void)APP_ServoType_PublishActive(control_servotype_persisted);
        app_servotype_set_state("reverted");
        app_servotype_report();
        return;
    }

    if (strcmp(tokens[1], "COMMIT") == 0) {
        if ((count != 2U) || (control_servotype_applied == 0U) ||
            (control_servotype_commit_pending != 0U) ||
            (control_imucal_confirmed_valid == 0U) ||
            (app_servotype_transaction_available() == 0U)) {
            app_servotype_set_state("commit_rejected");
            app_servotype_report();
            return;
        }
        control_servotype_pending_record = control_imucal_confirmed;
        if (APP_FlightCalibration_UpdateServoType(
                &control_servotype_pending_record,
                control_servotype_preview) == 0U) {
            app_servotype_set_state("commit_rejected");
            app_servotype_report();
            return;
        }
        encoded_size = APP_FlightCalibration_Encode(
            &control_servotype_pending_record, encoded, sizeof(encoded));
        if (encoded_size == 0U) {
            app_servotype_set_state("commit_rejected");
            app_servotype_report();
            return;
        }
        param_status = SVC_Param_SetBlob(encoded, encoded_size);
        if (param_status != SVC_PARAM_STATUS_OK) {
            app_servotype_set_state("commit_rejected");
            app_servotype_report();
            return;
        }
        control_servotype_commit_pending = 1U;
        control_servotype_last_request = SVC_Param_RequestSaveBlob();
        app_servotype_set_state(
            (control_servotype_last_request != 0U) ?
                "commit_queued" : "commit_queue_retry");
        app_servotype_report();
        return;
    }

    APP_Control_QueueText(
        "ERR usage SERVOTYPE? | SERVOTYPE APPLY type=bus|pwm | "
        "SERVOTYPE REVERT | SERVOTYPE COMMIT\r\n");
}

void app_control_service_servotype(void)
{
    if ((control_servotype_commit_pending != 0U) &&
        (SVC_Param_IsDirty() != 0U) &&
        (control_servotype_last_request == 0U)) {
        control_servotype_last_request = SVC_Param_RequestSaveBlob();
    }
}

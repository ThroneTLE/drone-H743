#include "app_control.h"
#include "app_control_internal.h"

#include "app_acceptance.h"
#include "app_battery.h"
#include "app_ident.h"
#include "app_prop_spin.h"
#include "app_servo_cal.h"
#include "app_stabilizer.h"
#include "app_sysid.h"
#include "app_thrust_bench.h"
#include "drv_prop_map.h"
#include "svc_timestamp.h"

#include <string.h>

static const char *block_reason(uint8_t check_battery)
{
    APP_BatterySnapshot battery;

    if (APP_Stabilizer_IsArmed()) { return "armed"; }
    if (!DRV_PropMap_IsCalibrated()) { return "mapping_unconfirmed"; }
    if (APP_Acceptance_IsActive()) { return "acceptance_active"; }
    if (APP_ServoCal_IsActive()) { return "servo_cal_active"; }
    if (APP_Ident_IsRunning()) { return "ident_running"; }
    if (APP_SysId_IsRunning()) { return "sysid_running"; }
    if (APP_PropSpin_IsActive()) { return "prop_spin_active"; }
    if (check_battery != 0U) {
        APP_Battery_GetSnapshot(&battery);
        if (!battery.state.valid) { return "battery_not_ready"; }
        if (!battery.can_arm) { return "battery_low"; }
    }
    return NULL;
}

static void reject(uint32_t now, const char *reason)
{
    if (APP_ThrustBench_IsActive()) {
        APP_ThrustBench_Close(now, APP_THRUST_BENCH_STOP_REJECTED);
    }
    APP_Control_QueueText("TBENCH state=rejected reason=%s\r\n", reason);
}

static uint8_t parse_percent_x100(const char *text, uint16_t *out)
{
    float value;
    if ((text == NULL) || (out == NULL) ||
        !app_control_parse_f32(text, &value) || (value < 0.0f) || (value > 100.0f)) {
        return 0U;
    }
    *out = (uint16_t)(value * 100.0f + 0.5f);
    return 1U;
}

uint8_t app_control_handle_thrust_bench(char **tokens, uint32_t count)
{
    uint32_t now = SVC_Timestamp_Ms();
    if ((tokens == NULL) || (count == 0U)) { return 0U; }
    if (strcmp(tokens[0], "TBENCH?") == 0) {
        uint32_t nonce;
        if ((count != 2U) || !app_control_parse_u32(tokens[1], &nonce)) {
            reject(now, "bad_nonce");
        } else if (!APP_ThrustBench_SendSnapshot(nonce)) {
            reject(now, "tx_busy");
        }
        return 1U;
    }
    if (strcmp(tokens[0], "TBENCH") != 0) { return 0U; }
    if (count < 2U) { reject(now, "bad_usage"); return 1U; }
    if (strcmp(tokens[1], "STOP") == 0) {
        APP_ThrustBench_Close(now, APP_THRUST_BENCH_STOP_REQUEST);
        APP_Control_QueueText("TBENCH state=idle active=0 max_pct=0 stop=request\r\n");
        return 1U;
    }
    if (strcmp(tokens[1], "ARM") == 0) {
        const char *confirm = app_control_token_value(tokens, count, "confirm");
        const char *max_text = app_control_token_value(tokens, count, "max_pct");
        const char *request_text = app_control_token_value(tokens, count, "request_id");
        const char *blocked;
        uint32_t max_percent, request_id;
        if ((confirm == NULL) || strcmp(confirm, "bench") != 0) {
            reject(now, "need_confirm"); return 1U;
        }
        if ((max_text == NULL) || !app_control_parse_u32(max_text, &max_percent) ||
            (max_percent < 1U) || (max_percent > 100U)) {
            reject(now, "bad_max_pct"); return 1U;
        }
        if ((request_text == NULL) || !app_control_parse_u32(request_text, &request_id) ||
            (request_id == 0U)) {
            reject(now, "bad_request_id"); return 1U;
        }
        if (APP_ThrustBench_IsActive()) { reject(now, "already_open"); return 1U; }
        blocked = block_reason(1U);
        if (blocked != NULL) { reject(now, blocked); return 1U; }
        if (!APP_ThrustBench_Open(now, (uint8_t)max_percent, request_id)) {
            reject(now, "open_failed"); return 1U;
        }
        APP_Control_QueueText(
            "TBENCH state=armed active=1 max_pct=%lu request_id=%lu token=%lu reason=-\r\n",
            (unsigned long)max_percent, (unsigned long)request_id,
            (unsigned long)request_id);
        return 1U;
    }
    if (strcmp(tokens[1], "SET") == 0) {
        const char *blocked = block_reason(0U);
        uint16_t upper, lower;
        uint32_t request_id, token;
        if (!APP_ThrustBench_IsActive()) { reject(now, "not_open"); return 1U; }
        if (blocked != NULL) {
            APP_ThrustBench_Close(now, APP_THRUST_BENCH_STOP_INHIBIT);
            APP_Control_QueueText("TBENCH state=rejected reason=%s\r\n", blocked);
            return 1U;
        }
        if (!parse_percent_x100(app_control_token_value(tokens, count, "upper_pct"), &upper) ||
            !parse_percent_x100(app_control_token_value(tokens, count, "lower_pct"), &lower) ||
            !app_control_parse_u32(app_control_token_value(tokens, count, "request_id"), &request_id) ||
            !app_control_parse_u32(app_control_token_value(tokens, count, "token"), &token) ||
            (request_id == 0U) || (token == 0U)) {
            reject(now, "bad_percent"); return 1U;
        }
        if (!APP_ThrustBench_Set(now, upper, lower, token, request_id)) {
            APP_Control_QueueText("TBENCH state=rejected request_id=%lu reason=token_or_limit\r\n",
                                  (unsigned long)request_id); return 1U;
        }
        APP_Control_QueueText(
            "TBENCH state=set active=1 upper_pct=%u.%02u lower_pct=%u.%02u age_ms=0 request_id=%lu token=%lu\r\n",
            (unsigned)(upper / 100U), (unsigned)(upper % 100U),
            (unsigned)(lower / 100U), (unsigned)(lower % 100U),
            (unsigned long)request_id, (unsigned long)token);
        return 1U;
    }
    reject(now, "bad_usage");
    return 1U;
}

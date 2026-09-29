#include "app_acceptance.h"
#include "app_battery.h"
#include "app_control.h"
#include "app_control_internal.h"
#include "app_ident.h"
#include "app_prop_spin.h"
#include "app_servo_cal.h"
#include "app_stabilizer.h"
#include "app_sysid.h"
#include "app_thrust_bench.h"
#include "drv_prop_map.h"
#include "svc_timestamp.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint8_t bench_active = 1U;
static uint8_t close_reason;
static char reply[160];

uint32_t SVC_Timestamp_Ms(void) { return 123U; }
uint8_t APP_Stabilizer_IsArmed(void) { return 0U; }
uint8_t DRV_PropMap_IsCalibrated(void) { return 1U; }
uint8_t APP_Acceptance_IsActive(void) { return 0U; }
uint8_t APP_ServoCal_IsActive(void) { return 0U; }
uint8_t APP_Ident_IsRunning(void) { return 0U; }
uint8_t APP_SysId_IsRunning(void) { return 0U; }
uint8_t APP_PropSpin_IsActive(void) { return 0U; }
void APP_Battery_GetSnapshot(APP_BatterySnapshot *out)
{
    memset(out, 0, sizeof(*out)); out->state.valid = 1U; out->can_arm = 1U;
}
void APP_Control_QueueText(const char *format, ...)
{
    va_list args; va_start(args, format);
    (void)vsnprintf(reply, sizeof(reply), format, args); va_end(args);
}
const char *app_control_token_value(char **tokens, uint32_t count, const char *key)
{
    const size_t length = strlen(key);
    for (uint32_t i = 0U; i < count; ++i) {
        if ((strncmp(tokens[i], key, length) == 0) && tokens[i][length] == '=') {
            return &tokens[i][length + 1U];
        }
    }
    return NULL;
}
uint8_t app_control_parse_u32(const char *text, uint32_t *value)
{
    char *end; unsigned long parsed;
    if ((text == NULL) || (value == NULL)) { return 0U; }
    parsed = strtoul(text, &end, 10);
    if ((end == text) || (*end != '\0')) { return 0U; }
    *value = (uint32_t)parsed; return 1U;
}
uint8_t app_control_parse_f32(const char *text, float *value)
{
    char *end; float parsed;
    if ((text == NULL) || (value == NULL)) { return 0U; }
    parsed = strtof(text, &end);
    if ((end == text) || (*end != '\0') || !isfinite(parsed)) { return 0U; }
    *value = parsed; return 1U;
}
uint8_t APP_ThrustBench_IsActive(void) { return bench_active; }
void APP_ThrustBench_Close(uint32_t now_ms, uint8_t reason)
{
    (void)now_ms; bench_active = 0U; close_reason = reason;
}
uint8_t APP_ThrustBench_SendSnapshot(uint32_t nonce) { (void)nonce; return 0U; }
uint8_t APP_ThrustBench_Open(uint32_t now_ms, uint8_t max_percent, uint32_t token)
{ (void)now_ms; (void)max_percent; (void)token; return 0U; }
uint8_t APP_ThrustBench_Set(uint32_t now_ms, uint16_t upper, uint16_t lower,
                           uint32_t token, uint32_t request_id)
{ (void)now_ms; (void)upper; (void)lower; (void)token; (void)request_id; return 0U; }

int main(void)
{
    char query[] = "TBENCH?"; char nonce[] = "7";
    char *tokens[] = {query, nonce};
    if (!app_control_handle_thrust_bench(tokens, 2U)) { return 2; }
    if (bench_active || close_reason != APP_THRUST_BENCH_STOP_REJECTED) { return 3; }
    if (strstr(reply, "reason=tx_busy") == NULL) { return 4; }
    return 0;
}

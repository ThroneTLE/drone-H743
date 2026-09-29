#include "app_thrust_bench.h"

#include "app_battery.h"
#include "app_current.h"
#include "app_diag_binary.h"
#include "app_proto.h"
#include "app_stabilizer.h"
#include "bsp_critical.h"
#include "bsp_dshot_rx.h"
#include "bsp_esc_protocol.h"
#include "bsp_pwm.h"
#include "drv_prop_map.h"
#include "svc_timestamp.h"

#include <limits.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

#define TBENCH_WIRE_VERSION 1U
#define TBENCH_PAYLOAD_BYTES 68U
#define TBENCH_FLAG_ERPM_1             (1U << 0)
#define TBENCH_FLAG_ERPM_2             (1U << 1)
#define TBENCH_FLAG_VOLTAGE            (1U << 2)
#define TBENCH_FLAG_TOTAL_CURRENT      (1U << 3)
#define TBENCH_FLAG_ESC_CURRENT_1      (1U << 4)
#define TBENCH_FLAG_ESC_CURRENT_2      (1U << 5)
#define TBENCH_FLAG_CURRENT_CALIBRATED (1U << 6)
#define TBENCH_FLAG_ACTIVE             (1U << 7)
#define TBENCH_FLAG_ARMED              (1U << 8)
#define TBENCH_FLAG_NOT_SPINNING_1     (1U << 9)
#define TBENCH_FLAG_NOT_SPINNING_2     (1U << 10)
#define TBENCH_SOURCE_FRESH_MS 250U

static volatile APP_ThrustBenchOutput bench_output;
static volatile uint32_t bench_last_heartbeat_ms;
static volatile uint8_t bench_last_stop_reason;
static volatile uint32_t bench_last_request_id;

static void close_locked(uint8_t reason)
{
    memset((void *)&bench_output, 0, sizeof(bench_output));
    bench_last_stop_reason = reason;
}

static uint16_t percent_to_pulse(uint16_t percent_x100)
{
    const uint32_t span = BSP_PWM_ESC_MAX_US - BSP_PWM_ESC_MIN_US;
    return (uint16_t)(BSP_PWM_ESC_MIN_US +
        (((uint32_t)percent_x100 * span + 5000U) / 10000U));
}

void APP_ThrustBench_Reset(void)
{
    uint32_t lock = BSP_Critical_Enter();
    memset((void *)&bench_output, 0, sizeof(bench_output));
    bench_last_heartbeat_ms = 0U;
    bench_last_stop_reason = (uint8_t)APP_THRUST_BENCH_STOP_NONE;
    bench_last_request_id = 0U;
    BSP_Critical_Exit(lock);
}

uint8_t APP_ThrustBench_Open(uint32_t now_ms, uint8_t max_percent,
                            uint32_t window_token)
{
    uint32_t lock;
    if ((max_percent == 0U) || (max_percent > 100U) || (window_token == 0U) ||
        (DRV_PropMap_IsCalibrated() == 0U) ||
        (DRV_PropMap_EscChannelForRole(DRV_PROP_ROLE_UPPER) == 0U) ||
        (DRV_PropMap_EscChannelForRole(DRV_PROP_ROLE_LOWER) == 0U)) {
        return 0U;
    }
    lock = BSP_Critical_Enter();
    if ((bench_output.active != 0U) ||
        ((int32_t)(window_token - bench_last_request_id) <= 0)) {
        BSP_Critical_Exit(lock);
        return 0U;
    }
    memset((void *)&bench_output, 0, sizeof(bench_output));
    bench_output.pulse_us[0] = BSP_PWM_ESC_MIN_US;
    bench_output.pulse_us[1] = BSP_PWM_ESC_MIN_US;
    bench_output.max_percent = max_percent;
    bench_output.window_token = window_token;
    bench_output.last_request_id = window_token;
    bench_last_request_id = window_token;
    bench_output.map_generation = DRV_PropMap_GetActiveGeneration();
    bench_output.upper_channel = DRV_PropMap_EscChannelForRole(DRV_PROP_ROLE_UPPER);
    bench_output.lower_channel = DRV_PropMap_EscChannelForRole(DRV_PROP_ROLE_LOWER);
    bench_last_heartbeat_ms = now_ms;
    bench_last_stop_reason = (uint8_t)APP_THRUST_BENCH_STOP_NONE;
    bench_output.active = 1U; /* publish only after both zero targets are ready */
    BSP_Critical_Exit(lock);
    return 1U;
}

uint8_t APP_ThrustBench_Set(uint32_t now_ms,
                           uint16_t upper_percent_x100,
                           uint16_t lower_percent_x100,
                           uint32_t window_token,
                           uint32_t request_id)
{
    uint8_t upper_channel = DRV_PropMap_EscChannelForRole(DRV_PROP_ROLE_UPPER);
    uint8_t lower_channel = DRV_PropMap_EscChannelForRole(DRV_PROP_ROLE_LOWER);
    uint32_t map_generation = DRV_PropMap_GetActiveGeneration();
    uint32_t lock = BSP_Critical_Enter();
    if ((bench_output.active == 0U) ||
        (window_token == 0U) || (window_token != bench_output.window_token) ||
        (request_id == 0U) ||
        ((int32_t)(request_id - bench_last_request_id) <= 0) ||
        ((uint32_t)(now_ms - bench_last_heartbeat_ms) >= APP_THRUST_BENCH_TIMEOUT_MS) ||
        (map_generation != bench_output.map_generation) ||
        (upper_channel != bench_output.upper_channel) ||
        (lower_channel != bench_output.lower_channel) ||
        (upper_channel == 0U) ||
        (lower_channel == 0U) || (upper_channel == lower_channel) ||
        (upper_percent_x100 > (uint16_t)(bench_output.max_percent * 100U)) ||
        (lower_percent_x100 > (uint16_t)(bench_output.max_percent * 100U))) {
        close_locked((uint8_t)APP_THRUST_BENCH_STOP_REJECTED);
        BSP_Critical_Exit(lock);
        return 0U;
    }
    bench_output.percent_x100[upper_channel - 1U] = upper_percent_x100;
    bench_output.percent_x100[lower_channel - 1U] = lower_percent_x100;
    bench_output.pulse_us[upper_channel - 1U] = percent_to_pulse(upper_percent_x100);
    bench_output.pulse_us[lower_channel - 1U] = percent_to_pulse(lower_percent_x100);
    bench_output.last_request_id = request_id;
    bench_last_request_id = request_id;
    bench_last_heartbeat_ms = now_ms; /* publish both targets/order before renewing */
    BSP_Critical_Exit(lock);
    return 1U;
}

void APP_ThrustBench_Close(uint32_t now_ms, uint8_t reason)
{
    uint32_t lock;
    (void)now_ms;
    lock = BSP_Critical_Enter();
    close_locked(reason);
    BSP_Critical_Exit(lock);
}

void APP_ThrustBench_Step(uint32_t now_ms, uint8_t inhibit)
{
    const uint32_t map_generation = DRV_PropMap_GetActiveGeneration();
    const uint8_t upper_channel = DRV_PropMap_EscChannelForRole(DRV_PROP_ROLE_UPPER);
    const uint8_t lower_channel = DRV_PropMap_EscChannelForRole(DRV_PROP_ROLE_LOWER);
    uint32_t lock = BSP_Critical_Enter();
    if (bench_output.active != 0U) {
        if ((inhibit != 0U) ||
            (map_generation != bench_output.map_generation) ||
            (upper_channel != bench_output.upper_channel) ||
            (lower_channel != bench_output.lower_channel)) {
            close_locked((uint8_t)APP_THRUST_BENCH_STOP_INHIBIT);
        } else if ((uint32_t)(now_ms - bench_last_heartbeat_ms) >=
                   APP_THRUST_BENCH_TIMEOUT_MS) {
            close_locked((uint8_t)APP_THRUST_BENCH_STOP_HEARTBEAT);
        }
    }
    BSP_Critical_Exit(lock);
}

uint8_t APP_ThrustBench_IsActive(void) { return bench_output.active; }

void APP_ThrustBench_GetOutput(APP_ThrustBenchOutput *out)
{
    uint32_t lock;
    if (out == NULL) { return; }
    lock = BSP_Critical_Enter();
    *out = bench_output;
    BSP_Critical_Exit(lock);
}

uint32_t APP_ThrustBench_HeartbeatAgeMs(uint32_t now_ms)
{
    return (bench_output.active != 0U) ?
        (uint32_t)(now_ms - bench_last_heartbeat_ms) : 0U;
}

uint8_t APP_ThrustBench_LastStopReason(void) { return bench_last_stop_reason; }

const char *APP_ThrustBench_StopReasonName(uint8_t reason)
{
    switch ((APP_ThrustBenchStopReason)reason) {
    case APP_THRUST_BENCH_STOP_NONE: return "none";
    case APP_THRUST_BENCH_STOP_REQUEST: return "request";
    case APP_THRUST_BENCH_STOP_HEARTBEAT: return "heartbeat_lost";
    case APP_THRUST_BENCH_STOP_INHIBIT: return "inhibited";
    case APP_THRUST_BENCH_STOP_REJECTED: return "rejected";
    case APP_THRUST_BENCH_STOP_OUTPUT_ERROR: return "output_error";
    default: return "unknown";
    }
}

uint32_t APP_ThrustBench_LastRequestId(void) { return bench_last_request_id; }

static void put_u16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)value; p[1] = (uint8_t)(value >> 8U);
}

static void put_u32(uint8_t *p, uint32_t value)
{
    p[0] = (uint8_t)value; p[1] = (uint8_t)(value >> 8U);
    p[2] = (uint8_t)(value >> 16U); p[3] = (uint8_t)(value >> 24U);
}

uint8_t APP_ThrustBench_SendSnapshot(uint32_t nonce)
{
    uint8_t payload[TBENCH_PAYLOAD_BYTES];
    BSP_DShotRxSnapshot rx;
    APP_BatterySnapshot battery;
    APP_CurrentSnapshot current;
    APP_ThrustBenchOutput output;
    uint32_t now;
    uint16_t flags = 0U;
    int32_t current_ma = INT32_MIN;
    uint8_t upper = DRV_PropMap_EscChannelForRole(DRV_PROP_ROLE_UPPER);
    uint8_t lower = DRV_PropMap_EscChannelForRole(DRV_PROP_ROLE_LOWER);
    uint16_t actual_command_us[2];
    uint32_t voltage_age;
    uint32_t current_age;

    memset(payload, 0, sizeof(payload));
    BSP_DShotRx_GetSnapshot(&rx);
    APP_Battery_GetSnapshot(&battery);
    APP_Current_GetSnapshot(&current);
    APP_ThrustBench_GetOutput(&output);
    BSP_PWM_GetEscPulses(actual_command_us);
    /* BSP DShot timestamps and SVC_Timestamp_Ms both use HAL_GetTick.  Read the
     * common clock after all snapshots so a 500 Hz harvest cannot look newer
     * than this frame and wrap its age to nearly 49 days. */
    now = SVC_Timestamp_Ms();
    voltage_age = battery.state.samples ?
        (uint32_t)(now - battery.state.sample_ms) : UINT32_MAX;
    current_age = current.samples ?
        (uint32_t)(now - current.sample_ms) : UINT32_MAX;
    for (uint32_t i = 0U; i < 2U; ++i) {
        const uint32_t erpm_age = (uint32_t)(now - rx.sample_ms[i]);
        const uint32_t current_age = (uint32_t)(now - rx.current_sample_ms[i]);
        if (rx.available && rx.valid[i] && erpm_age <= APP_THRUST_BENCH_ERPM_FRESH_MS) {
            flags |= (uint16_t)(TBENCH_FLAG_ERPM_1 << i);
            if (rx.not_spinning[i]) { flags |= (uint16_t)(TBENCH_FLAG_NOT_SPINNING_1 << i); }
        }
        if (rx.available && rx.current_valid[i] &&
            current_age <= APP_THRUST_BENCH_EDT_CURRENT_FRESH_MS) {
            flags |= (uint16_t)(TBENCH_FLAG_ESC_CURRENT_1 << i);
        }
    }
    if ((battery.state.valid != 0U) && (voltage_age <= TBENCH_SOURCE_FRESH_MS)) {
        flags |= TBENCH_FLAG_VOLTAGE;
    }
    if ((current.reading.valid != 0U) && isfinite(current.reading.current_a) &&
        (current_age <= TBENCH_SOURCE_FRESH_MS) &&
        (current.reading.current_a >= -2147483.0f) &&
        (current.reading.current_a <= 2147483.0f)) {
        current_ma = (int32_t)(current.reading.current_a * 1000.0f);
        flags |= TBENCH_FLAG_TOTAL_CURRENT;
    }
    if (current.reading.calibrated != 0U) { flags |= TBENCH_FLAG_CURRENT_CALIBRATED; }
    if (output.active != 0U) { flags |= TBENCH_FLAG_ACTIVE; }
    if (APP_Stabilizer_IsArmed() != 0U) { flags |= TBENCH_FLAG_ARMED; }

    payload[0] = TBENCH_WIRE_VERSION; payload[1] = TBENCH_PAYLOAD_BYTES;
    put_u16(&payload[2], flags); put_u32(&payload[4], nonce); put_u32(&payload[8], now);
    payload[12] = (uint8_t)BSP_ESC_PROTOCOL; payload[13] = upper;
    payload[14] = lower; payload[15] = output.max_percent;
    /* Final staged command at the BSP arbitration seam, not a physical-output
     * acknowledgement.  It observes RC/flight/SYSID as well as TBENCH. */
    put_u16(&payload[16], actual_command_us[0]);
    put_u16(&payload[18], actual_command_us[1]);
    put_u32(&payload[20], rx.erpm[0]); put_u32(&payload[24], rx.erpm[1]);
    put_u32(&payload[28], rx.valid[0] ? (uint32_t)(now - rx.sample_ms[0]) : UINT32_MAX);
    put_u32(&payload[32], rx.valid[1] ? (uint32_t)(now - rx.sample_ms[1]) : UINT32_MAX);
    put_u32(&payload[36], (flags & TBENCH_FLAG_VOLTAGE) ?
        battery.state.voltage_mv : UINT32_MAX);
    put_u32(&payload[40], (uint32_t)current_ma);
    put_u32(&payload[44], voltage_age);
    put_u32(&payload[48], current_age);
    put_u16(&payload[52], rx.current_a[0]); put_u16(&payload[54], rx.current_a[1]);
    put_u32(&payload[56], rx.current_valid[0] ?
        (uint32_t)(now - rx.current_sample_ms[0]) : UINT32_MAX);
    put_u32(&payload[60], rx.current_valid[1] ?
        (uint32_t)(now - rx.current_sample_ms[1]) : UINT32_MAX);
    put_u32(&payload[64], APP_ThrustBench_LastRequestId());
    return APP_Diag_SendBinary(APP_PROTO_MSG_THRUST_BENCH, payload, sizeof(payload));
}

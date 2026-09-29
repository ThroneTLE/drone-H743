#include "app_battery.h"
#include "app_current.h"
#include "app_diag_binary.h"
#include "app_proto.h"
#include "app_stabilizer.h"
#include "app_thrust_bench.h"
#include "bsp_critical.h"
#include "bsp_dshot_rx.h"
#include "drv_prop_map.h"
#include "svc_timestamp.h"

#include <stdio.h>
#include <string.h>

static uint8_t captured[128];
static uint16_t captured_length;
static uint32_t map_generation = 3U;

uint32_t BSP_Critical_Enter(void) { return 0U; }
void BSP_Critical_Exit(uint32_t state) { (void)state; }
void BSP_Critical_MemoryBarrier(void) { }
uint32_t SVC_Timestamp_Ms(void) { return 0x12345678U; }
uint8_t APP_Stabilizer_IsArmed(void) { return 0U; }
uint8_t DRV_PropMap_IsCalibrated(void) { return 1U; }
uint32_t DRV_PropMap_GetActiveGeneration(void) { return map_generation; }
uint8_t DRV_PropMap_EscChannelForRole(uint8_t role)
{
    return (role == DRV_PROP_ROLE_UPPER) ? 1U :
           (role == DRV_PROP_ROLE_LOWER) ? 2U : 0U;
}
void BSP_DShotRx_GetSnapshot(BSP_DShotRxSnapshot *out)
{
    memset(out, 0, sizeof(*out));
    out->available = 1U;
    out->valid[0] = 1U; out->valid[1] = 1U;
    out->erpm[0] = 123456U; out->not_spinning[1] = 1U;
    out->sample_ms[0] = 0x12345674U; out->sample_ms[1] = 0x12345670U;
    out->current_valid[0] = 1U; out->current_a[0] = 13U;
    out->current_sample_ms[0] = 0x123455B0U;
}
void APP_Battery_GetSnapshot(APP_BatterySnapshot *out)
{
    memset(out, 0, sizeof(*out)); out->state.valid = 1U;
    out->state.voltage_mv = 11100U; out->state.samples = 1U;
    out->state.sample_ms = 0x12345671U; out->age_ms = 7U;
}
void APP_Current_GetSnapshot(APP_CurrentSnapshot *out)
{
    memset(out, 0, sizeof(*out)); out->reading.valid = 1U;
    out->reading.current_a = 12.5f; out->sample_ms = 0x1234566FU;
    out->samples = 1U; out->age_ms = 9U;
}
void BSP_PWM_GetEscPulses(uint16_t out[2])
{
    out[0] = 1184U; out[1] = 1268U;
}
uint8_t APP_Diag_SendBinary(uint16_t function, const uint8_t *payload,
                            uint16_t length)
{
    if (function != APP_PROTO_MSG_THRUST_BENCH) { return 0U; }
    return APP_Proto_BuildFrame(APP_PROTO_DIR_FROM_FC, function, payload, length,
                                captured, sizeof(captured), &captured_length);
}

int main(int argc, char **argv)
{
    APP_ThrustBenchOutput output;
    APP_ThrustBench_Reset();
    if ((argc == 2) && (strcmp(argv[1], "timeout") == 0)) {
        if (!APP_ThrustBench_Open(1000U, 20U, 7U) ||
            !APP_ThrustBench_Set(1000U, 1000U, 2000U, 7U, 8U)) { return 3; }
        APP_ThrustBench_Step(1299U, 0U); APP_ThrustBench_GetOutput(&output);
        if (!output.active) { return 4; }
        APP_ThrustBench_Step(1300U, 0U); APP_ThrustBench_GetOutput(&output);
        return output.active || APP_ThrustBench_LastStopReason() !=
            APP_THRUST_BENCH_STOP_HEARTBEAT;
    }
    if ((argc == 2) && (strcmp(argv[1], "inhibit") == 0)) {
        if (!APP_ThrustBench_Open(1000U, 20U, 7U)) { return 5; }
        APP_ThrustBench_Step(1000U, 1U); APP_ThrustBench_GetOutput(&output);
        return output.active || APP_ThrustBench_LastStopReason() !=
            APP_THRUST_BENCH_STOP_INHIBIT;
    }
    if ((argc == 2) && (strcmp(argv[1], "replay") == 0)) {
        if (!APP_ThrustBench_Open(1000U, 20U, 7U) ||
            !APP_ThrustBench_Set(1000U, 1000U, 2000U, 7U, 8U)) { return 6; }
        /* duplicate request closes; neither the old SET nor old ARM may revive it */
        if (APP_ThrustBench_Set(1100U, 1000U, 2000U, 7U, 8U)) { return 7; }
        if (APP_ThrustBench_IsActive() ||
            APP_ThrustBench_Open(1200U, 20U, 7U) ||
            APP_ThrustBench_Set(1200U, 1000U, 2000U, 7U, 9U)) { return 8; }
        return APP_ThrustBench_LastStopReason() != APP_THRUST_BENCH_STOP_REJECTED;
    }
    if ((argc == 2) && (strcmp(argv[1], "late_set") == 0)) {
        if (!APP_ThrustBench_Open(1000U, 20U, 7U) ||
            APP_ThrustBench_Set(1300U, 1000U, 2000U, 7U, 8U)) { return 9; }
        return APP_ThrustBench_IsActive();
    }
    if ((argc == 2) && (strcmp(argv[1], "map_change") == 0)) {
        if (!APP_ThrustBench_Open(1000U, 20U, 7U)) { return 10; }
        map_generation++;
        APP_ThrustBench_Step(1001U, 0U);
        return APP_ThrustBench_IsActive();
    }
    if ((argc == 2) && (strcmp(argv[1], "multi_window_replay") == 0)) {
        if (!APP_ThrustBench_Open(1000U, 20U, 7U)) { return 11; }
        APP_ThrustBench_Close(1001U, APP_THRUST_BENCH_STOP_REQUEST);
        if (!APP_ThrustBench_Open(1002U, 20U, 9U)) { return 12; }
        APP_ThrustBench_Close(1003U, APP_THRUST_BENCH_STOP_REQUEST);
        if (APP_ThrustBench_Open(1004U, 20U, 7U) ||
            APP_ThrustBench_LastRequestId() != 9U) { return 13; }
        return 0;
    }
    if ((argc == 2) && (strcmp(argv[1], "serial_wrap") == 0)) {
        if (!APP_ThrustBench_Open(1000U, 20U, 0x7fffffffU) ||
            !APP_ThrustBench_Set(1001U, 0U, 0U, 0x7fffffffU, 0xfffffffeU) ||
            !APP_ThrustBench_Set(1002U, 0U, 0U, 0x7fffffffU, 0xffffffffU)) {
            return 14;
        }
        APP_ThrustBench_Close(1003U, APP_THRUST_BENCH_STOP_REQUEST);
        if (!APP_ThrustBench_Open(1004U, 20U, 1U)) { return 15; }
        return APP_ThrustBench_LastRequestId() != 1U;
    }
    if (!APP_ThrustBench_Open(SVC_Timestamp_Ms(), 20U, 7U) ||
        !APP_ThrustBench_Set(SVC_Timestamp_Ms(), 1000U, 2000U, 7U, 8U) ||
        !APP_ThrustBench_SendSnapshot(0x78563412U)) { return 2; }
    for (uint16_t i = 0U; i < captured_length; ++i) { printf("%02x", captured[i]); }
    putchar('\n');
    return 0;
}

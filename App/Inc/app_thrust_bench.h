#ifndef APP_THRUST_BENCH_H
#define APP_THRUST_BENCH_H

#include <stdint.h>

#define APP_THRUST_BENCH_TIMEOUT_MS 300U
#define APP_THRUST_BENCH_CHANNEL_COUNT 2U
#define APP_THRUST_BENCH_ERPM_FRESH_MS 100U
#define APP_THRUST_BENCH_EDT_CURRENT_FRESH_MS 1000U

typedef enum {
    APP_THRUST_BENCH_STOP_NONE = 0U,
    APP_THRUST_BENCH_STOP_REQUEST,
    APP_THRUST_BENCH_STOP_HEARTBEAT,
    APP_THRUST_BENCH_STOP_INHIBIT,
    APP_THRUST_BENCH_STOP_REJECTED,
    APP_THRUST_BENCH_STOP_OUTPUT_ERROR
} APP_ThrustBenchStopReason;

typedef struct {
    uint16_t pulse_us[APP_THRUST_BENCH_CHANNEL_COUNT]; /* ESC channel order */
    uint16_t percent_x100[APP_THRUST_BENCH_CHANNEL_COUNT];
    uint32_t window_token;
    uint32_t last_request_id;
    uint32_t map_generation;
    uint8_t upper_channel;
    uint8_t lower_channel;
    uint8_t active;
    uint8_t max_percent;
} APP_ThrustBenchOutput;

void APP_ThrustBench_Reset(void);
uint8_t APP_ThrustBench_Open(uint32_t now_ms, uint8_t max_percent,
                            uint32_t window_token);
uint8_t APP_ThrustBench_Set(uint32_t now_ms,
                           uint16_t upper_percent_x100,
                           uint16_t lower_percent_x100,
                           uint32_t window_token,
                           uint32_t request_id);
void APP_ThrustBench_Close(uint32_t now_ms, uint8_t reason);
void APP_ThrustBench_Step(uint32_t now_ms, uint8_t inhibit);
uint8_t APP_ThrustBench_IsActive(void);
void APP_ThrustBench_GetOutput(APP_ThrustBenchOutput *out);
uint32_t APP_ThrustBench_HeartbeatAgeMs(uint32_t now_ms);
uint8_t APP_ThrustBench_LastStopReason(void);
uint32_t APP_ThrustBench_LastRequestId(void);
const char *APP_ThrustBench_StopReasonName(uint8_t reason);
uint8_t APP_ThrustBench_SendSnapshot(uint32_t nonce);

#endif

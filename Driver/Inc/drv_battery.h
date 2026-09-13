#ifndef DRV_BATTERY_H
#define DRV_BATTERY_H
#include <stdint.h>
#define DRV_BATTERY_STALE_MS 250U
typedef struct { uint8_t cells; uint16_t low_cell_mv, recover_cell_mv; } DRV_BatteryConfig;
typedef struct {
    DRV_BatteryConfig config;
    uint32_t raw, voltage_mv, sample_ms, samples, errors;
    uint8_t valid, low, saturated, adc_status;
} DRV_BatteryState;
void DRV_Battery_Init(DRV_BatteryState *state);
uint8_t DRV_Battery_Configure(DRV_BatteryState *state, DRV_BatteryConfig config);
void DRV_Battery_Update(DRV_BatteryState *state, uint32_t raw, uint8_t adc_status, uint32_t now_ms);
uint8_t DRV_Battery_IsFresh(const DRV_BatteryState *state, uint32_t now_ms);
uint8_t DRV_Battery_CanArm(const DRV_BatteryState *state, uint32_t now_ms);
#endif

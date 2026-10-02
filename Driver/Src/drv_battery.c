/* Pure voltage conversion and pre-arm policy. No I/O, allocation, or flight action. */
#include "drv_battery.h"
#include <string.h>

void DRV_Battery_Init(DRV_BatteryState *state)
{
    memset(state,0,sizeof(*state));
    state->config=(DRV_BatteryConfig){3U,DRV_BATTERY_DEFAULT_LOW_CELL_MV,DRV_BATTERY_DEFAULT_RECOVER_CELL_MV};
    state->low=1U;state->adc_status=1U;
}
uint8_t DRV_Battery_Configure(DRV_BatteryState *state, DRV_BatteryConfig config)
{
    if (state==0 || config.cells<1U || config.cells>12U || config.low_cell_mv<2500U ||
        config.low_cell_mv>4100U || config.recover_cell_mv<=config.low_cell_mv ||
        config.recover_cell_mv>4400U) { return 0U; }
    state->config=config;
    state->low=(!state->valid || state->voltage_mv<(uint32_t)config.cells*config.recover_cell_mv);
    return 1U;
}
void DRV_Battery_Update(DRV_BatteryState *state,uint32_t raw,uint8_t adc_status,uint32_t now_ms)
{
    state->adc_status=adc_status;
    if (adc_status!=0U || raw>65535U) {
        state->adc_status=adc_status?adc_status:3U;
        state->valid=0U;state->errors++;return;
    }
    state->raw=raw;state->sample_ms=now_ms;state->samples++;
    state->saturated=(raw==65535U);
    /* MicoAir743v2 nominal divider=21.12, reference=3.3V, 16-bit ADC.
     * Integer mV, nearest rounding; 64-bit intermediate avoids overflow. */
    state->voltage_mv=(uint32_t)(((uint64_t)raw*69696U+32767U)/65535U);
    state->valid=(state->saturated==0U);
    if (!state->valid) { state->low=1U;return; }
    uint32_t threshold=(uint32_t)state->config.cells*
        (state->low?state->config.recover_cell_mv:state->config.low_cell_mv);
    state->low=state->voltage_mv<threshold;
}
uint8_t DRV_Battery_IsFresh(const DRV_BatteryState *state,uint32_t now_ms)
{
    return state->valid && state->samples && (uint32_t)(now_ms-state->sample_ms)<=DRV_BATTERY_STALE_MS;
}
uint8_t DRV_Battery_CanArm(const DRV_BatteryState *state,uint32_t now_ms)
{
    return DRV_Battery_IsFresh(state,now_ms) && !state->low;
}

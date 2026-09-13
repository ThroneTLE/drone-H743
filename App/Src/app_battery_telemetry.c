/* Battery telemetry is current data, never queued history or a fabricated percentage. */
#include "app_battery.h"
#include "app_current.h"
#include "app_elrs.h"
#include "svc_timestamp.h"
#include "bsp_critical.h"
#include <math.h>
static uint32_t last_ms;
static volatile uint32_t accepted,rejected;
static uint8_t attempted;
void APP_Battery_GetTxStats(uint32_t *ok,uint32_t *failed)
{
    uint32_t lock=BSP_Critical_Enter();
    if (ok) { *ok=accepted; } if (failed) { *failed=rejected; }
    BSP_Critical_Exit(lock);
}
void APP_Battery_TelemetryStep(void)
{
    uint32_t now=SVC_Timestamp_Ms();
    if (attempted && (uint32_t)(now-last_ms)<500U) { return; }
    attempted=1U;last_ms=now;
    APP_BatterySnapshot battery;APP_CurrentSnapshot current;
    APP_Battery_GetSnapshot(&battery);APP_Current_GetSnapshot(&current);
    uint16_t voltage=UINT16_MAX,amperes=UINT16_MAX;
    if (battery.state.valid) { voltage=(uint16_t)((battery.state.voltage_mv+50U)/100U); }
    if (current.reading.valid && isfinite(current.reading.current_a) && current.reading.current_a>=0.0f) {
        float value=current.reading.current_a*10.0f+0.5f;
        amperes=(uint16_t)(value<32766.0f?value:32766.0f);
    }
    /* Existing CRSF and EdgeTX use decivolts/deciamps. All-FF means unavailable
     * to EdgeTX; no capacity/SOC model exists, so do not send invented zeros. */
    if (APP_ELRS_SendTelemetryBattery(voltage,amperes,0xFFFFFFU,0xFFU)) { accepted++; }
    else { rejected++; }
}

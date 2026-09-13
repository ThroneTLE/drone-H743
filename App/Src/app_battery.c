#include "app_battery.h"
#include "app_current.h"
#include "app_stabilizer.h"
#include "bsp_current.h"
#include "bsp_critical.h"
#include "svc_timestamp.h"
static DRV_BatteryState state={.config={3U,3500U,3600U},.low=1U,.adc_status=1U};
static uint32_t last_sequence;
static uint8_t initialized;

void APP_Battery_Init(void)
{
    uint32_t lock=BSP_Critical_Enter();DRV_Battery_Init(&state);
    last_sequence=UINT32_MAX;initialized=1U;BSP_Critical_Exit(lock);
}
void APP_Battery_Step(void)
{
    BSP_VoltageSample sample;BSP_Current_GetVoltageSample(&sample);
    if (sample.sequence==last_sequence) { return; }
    last_sequence=sample.sequence;
    APP_CurrentSnapshot current;APP_Current_GetSnapshot(&current);
    uint32_t lock=BSP_Critical_Enter();
    DRV_Battery_Update(&state,sample.raw,(uint8_t)sample.status,current.sample_ms);
    BSP_Critical_Exit(lock);
}
void APP_Battery_GetSnapshot(APP_BatterySnapshot *out)
{
    if (!out) { return; }
    uint32_t lock=BSP_Critical_Enter(),now=SVC_Timestamp_Ms();
    out->state=state;out->age_ms=state.samples?(uint32_t)(now-state.sample_ms):UINT32_MAX;
    out->can_arm=DRV_Battery_CanArm(&state,now);
    out->state.valid=DRV_Battery_IsFresh(&state,now);
    BSP_Critical_Exit(lock);
}
uint8_t APP_Battery_CanArm(void)
{
    APP_BatterySnapshot snapshot;APP_Battery_GetSnapshot(&snapshot);return snapshot.can_arm;
}
uint8_t APP_Battery_Configure(DRV_BatteryConfig config)
{
    uint32_t lock=BSP_Critical_Enter();
    uint8_t result=(!initialized || APP_Stabilizer_IsArmed())?0U:DRV_Battery_Configure(&state,config);
    BSP_Critical_Exit(lock);return result;
}

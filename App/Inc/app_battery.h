#ifndef APP_BATTERY_H
#define APP_BATTERY_H
#include "drv_battery.h"
typedef struct { DRV_BatteryState state; uint32_t age_ms; uint8_t can_arm; } APP_BatterySnapshot;
void APP_Battery_Init(void);
void APP_Battery_Step(void); /* messageTask, immediately after APP_Current_Step */
void APP_Battery_GetSnapshot(APP_BatterySnapshot *out);
uint8_t APP_Battery_CanArm(void); /* cached data only; no ADC/transport/locks that wait */
uint8_t APP_Battery_Configure(DRV_BatteryConfig config); /* disarmed, RAM only */
uint8_t APP_Battery_Command(char **tokens,uint32_t count);
void APP_Battery_TelemetryStep(void); /* messageTask, nonblocking DMA, 2 Hz */
void APP_Battery_GetTxStats(uint32_t *accepted,uint32_t *rejected);
void APP_Battery_PublishLedWarning(void); /* LED task; amber eight-pulse warning */
uint8_t APP_Battery_SendReport(uint32_t nonce);
#endif

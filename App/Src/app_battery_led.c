/* Dedicated warning keeps the legacy configurable LED binding IDs unchanged. */
#include "app_battery.h"
#include "app_led.h"
#include "svc_led.h"
void APP_Battery_PublishLedWarning(void)
{
    static const DRV_RgbPattern warning={.color={255U,110U,0U},.effect=DRV_RGB_EFFECT_PULSES,
        .on_ms=160U,.off_ms=160U,.gap_ms=760U,.count=APP_LED_ARM_BLOCK_BATTERY};
    SVC_Led_Publish(SVC_LED_SOURCE_ARMED,0);
    SVC_Led_Publish(SVC_LED_SOURCE_STATUS,0);
    SVC_Led_Publish(SVC_LED_SOURCE_BLOCKED,&warning);
}

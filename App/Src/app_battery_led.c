/* Dedicated warning keeps the legacy configurable LED binding IDs unchanged. */
#include "app_battery.h"
#include "app_led.h"
#include "svc_led.h"
void APP_Battery_PublishLedWarning(void)
{
    static const DRV_RgbPattern warning={.color={255U,110U,0U},.effect=DRV_RGB_EFFECT_PULSES,
        .on_ms=160U,.off_ms=160U,.gap_ms=760U,.count=APP_LED_ARM_BLOCK_BATTERY};
    /* 不碰 SVC_LED_SOURCE_ARMED：那一路归调用方（app_led_publish_arm）管，而且
     * ARMED 优先级高于 BLOCKED——在这里清掉它，就等于让低压告警盖掉"电机带电"
     * 的常亮红灯。本函数只负责把告警放到 BLOCKED，并让出就绪指示。 */
    SVC_Led_Publish(SVC_LED_SOURCE_STATUS,0);
    SVC_Led_Publish(SVC_LED_SOURCE_BLOCKED,&warning);
}

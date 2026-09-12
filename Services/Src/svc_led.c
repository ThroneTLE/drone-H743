#include "svc_led.h"

#include <stddef.h>

typedef struct {
    DRV_RgbPattern pattern;
    uint8_t        active;
    uint8_t        timed;       /* 1 = 到点自动撤销 */
    uint32_t       expires_ms;
} SVC_LedSlot;

static SVC_LedSlot   led_slots[SVC_LED_SOURCE_COUNT];
static SVC_LedSink   led_sink;
static DRV_RgbDither led_dither;
static SVC_LedSource led_active_source = SVC_LED_SOURCE_COUNT;
static DRV_RgbColor  led_active_color;

void SVC_Led_Init(SVC_LedSink sink)
{
    uint8_t index;

    led_sink = sink;
    for (index = 0U; index < (uint8_t)SVC_LED_SOURCE_COUNT; index++) {
        led_slots[index].active = 0U;
        led_slots[index].timed = 0U;
    }
    DRV_RgbLed_DitherReset(&led_dither);
    led_active_source = SVC_LED_SOURCE_COUNT;
    led_active_color.r = 0U;
    led_active_color.g = 0U;
    led_active_color.b = 0U;
    if (led_sink != NULL) {
        led_sink(0U);
    }
}

void SVC_Led_Publish(SVC_LedSource source, const DRV_RgbPattern *pattern)
{
    if (source >= SVC_LED_SOURCE_COUNT) {
        return;
    }
    if (pattern == NULL) {
        led_slots[source].active = 0U;
        led_slots[source].timed = 0U;
        return;
    }
    led_slots[source].pattern = *pattern;
    led_slots[source].active = 1U;
    led_slots[source].timed = 0U;
}

void SVC_Led_PublishFor(SVC_LedSource source, const DRV_RgbPattern *pattern,
                        uint32_t hold_ms, uint32_t now_ms)
{
    if (source >= SVC_LED_SOURCE_COUNT) {
        return;
    }
    SVC_Led_Publish(source, pattern);
    if ((pattern != NULL) && (hold_ms != 0U)) {
        led_slots[source].timed = 1U;
        led_slots[source].expires_ms = now_ms + hold_ms;
    }
}

static void led_expire(uint32_t now_ms)
{
    uint8_t index;

    for (index = 0U; index < (uint8_t)SVC_LED_SOURCE_COUNT; index++) {
        if ((led_slots[index].active == 0U) || (led_slots[index].timed == 0U)) {
            continue;
        }
        /* 相减再比较，而不是 now >= expires：毫秒计数会回绕，直接比大小会让
         * 回绕前后那一次发布卡住约 49 天。 */
        if ((uint32_t)(now_ms - led_slots[index].expires_ms) < 0x80000000U) {
            led_slots[index].active = 0U;
            led_slots[index].timed = 0U;
        }
    }
}

void SVC_Led_Tick(uint32_t now_ms)
{
    uint8_t index;
    uint8_t bits;

    led_expire(now_ms);

    led_active_source = SVC_LED_SOURCE_COUNT;
    for (index = 0U; index < (uint8_t)SVC_LED_SOURCE_COUNT; index++) {
        if (led_slots[index].active != 0U) {
            led_active_source = (SVC_LedSource)index;
            break;      /* 枚举顺序即优先级，第一个有话说的就是赢家 */
        }
    }

    if (led_active_source >= SVC_LED_SOURCE_COUNT) {
        led_active_color.r = 0U;
        led_active_color.g = 0U;
        led_active_color.b = 0U;
    } else {
        led_active_color = DRV_RgbLed_Sample(&led_slots[led_active_source].pattern,
                                             now_ms);
    }

    bits = DRV_RgbLed_Modulate(&led_dither, led_active_color);
    if (led_sink != NULL) {
        led_sink(bits);
    }
}

SVC_LedSource SVC_Led_ActiveSource(void)
{
    return led_active_source;
}

DRV_RgbColor SVC_Led_ActiveColor(void)
{
    return led_active_color;
}

/*
 * LED 命令族：让状态灯的颜色与节奏可以在线试，而不是改一次编译一次烧一次。
 *
 * 两个用途，都不是锦上添花：
 *
 *   1. **核对接线**。三根脚哪根是红、极性是高有效还是低有效，只有点亮才知道。
 *      2026-09-12 就是靠"蓝灯大部分时间亮、偶尔闪一下"（被拒 1 下那条图案的
 *      反相）才发现本板是共阳接法——在此之前所有图案都反着，测试全绿。
 *   2. **调手感**。颜色与呼吸周期要调到顺眼为止，而"顺眼"只能用眼睛判。
 *
 * 点名一律带自动到期（默认 5 s）：调试时占着灯不还，之后所有真实状态都被盖住，
 * 而这种"灯不动了"最容易被当成固件死了。
 */

#include "app_control.h"
#include "app_control_internal.h"

#include "app_led.h"
#include "bsp_rgb_led.h"
#include "svc_led.h"

#include <stddef.h>
#include <string.h>

#define APP_CMD_LED_DEFAULT_HOLD_MS 5000U

static const char *led_source_name(uint8_t source)
{
    switch ((SVC_LedSource)source) {
    case SVC_LED_SOURCE_IDENTIFY:    return "identify";
    case SVC_LED_SOURCE_ARMED:       return "armed";
    case SVC_LED_SOURCE_CALIBRATION: return "calibration";
    case SVC_LED_SOURCE_BLOCKED:     return "blocked";
    case SVC_LED_SOURCE_WARNING:     return "warning";
    case SVC_LED_SOURCE_STATUS:      return "status";
    case SVC_LED_SOURCE_HEARTBEAT:   return "heartbeat";
    default:                         return "idle";
    }
}

static void led_report(void)
{
    APP_LED_Debug debug;

    APP_LED_GetDebug(&debug);
    APP_Control_QueueText(
        "LED src=%s r=%u g=%u b=%u armed=%u known=%u block=%u servo_cal=%u ticks=%lu\r\n",
        led_source_name(debug.source),
        (unsigned int)debug.color.r,
        (unsigned int)debug.color.g,
        (unsigned int)debug.color.b,
        (unsigned int)debug.armed,
        (unsigned int)debug.arm_published,
        (unsigned int)debug.block_reason,
        (unsigned int)debug.servo_cal,
        (unsigned long)debug.ticks);
    APP_Control_QueueText("LED wiring r=%s g=%s b=%s active_low=%u\r\n",
                          BSP_RgbLed_PinName(0U),
                          BSP_RgbLed_PinName(1U),
                          BSP_RgbLed_PinName(2U),
                          (unsigned int)debug.active_low);
}

/* 取一个 0..255 的通道值。越界一律判失败而不是钳住：命令行打错数字时，
 * 静默钳住会让人以为灯已经按他想的亮了。 */
static uint8_t led_parse_channel(const char *text, uint8_t *out)
{
    uint32_t value;

    if ((app_control_parse_u32(text, &value) == 0U) || (value > 255U)) {
        return 0U;
    }
    *out = (uint8_t)value;
    return 1U;
}

static uint8_t led_parse_color(char **tokens, uint32_t first, uint32_t count,
                               DRV_RgbColor *color)
{
    if ((first + 2U) >= count) {
        return 0U;
    }
    return (uint8_t)((led_parse_channel(tokens[first], &color->r) != 0U) &&
                     (led_parse_channel(tokens[first + 1U], &color->g) != 0U) &&
                     (led_parse_channel(tokens[first + 2U], &color->b) != 0U));
}

static uint32_t led_parse_hold(char **tokens, uint32_t index, uint32_t count)
{
    uint32_t hold;

    if ((index >= count) || (app_control_parse_u32(tokens[index], &hold) == 0U)) {
        return APP_CMD_LED_DEFAULT_HOLD_MS;
    }
    return hold;       /* 0 = 一直占着，直到 LED AUTO */
}

uint8_t app_control_handle_led(char **tokens, uint32_t count)
{
    DRV_RgbPattern pattern;
    DRV_RgbColor color;

    if ((count == 0U) || (tokens == NULL)) {
        return 0U;
    }
    if (strcmp(tokens[0], "LED?") == 0) {
        led_report();
        return 1U;
    }
    if (strcmp(tokens[0], "LED") != 0) {
        return 0U;
    }
    if (count < 2U) {
        APP_Control_QueueText(
            "ERR usage: LED? | LED AUTO | LED RGB r g b [ms] | "
            "LED BLINK r g b on_ms off_ms [ms] | LED BREATHE r g b period_ms [ms]\r\n");
        return 1U;
    }

    if (strcmp(tokens[1], "AUTO") == 0) {
        /* 撤销点名任何时候都允许——它只会让灯回到真实状态。 */
        APP_LED_Identify(NULL, 0U);
        APP_Control_QueueText("OK LED AUTO\r\n");
        return 1U;
    }

    /*
     * 解锁后拒绝点名。策略层每拍都会撤销 IDENTIFY（见 app_led_publish_arm），
     * 所以此刻发下去也是白发；这里明说一句，免得操作者对着一盏不听话的灯排查。
     */
    {
        APP_LED_Debug debug;

        APP_LED_GetDebug(&debug);
        if (debug.armed != 0U) {
            APP_Control_QueueText("ERR LED identify refused: armed\r\n");
            return 1U;
        }
    }

    memset(&pattern, 0, sizeof(pattern));

    if (strcmp(tokens[1], "RGB") == 0) {
        if (led_parse_color(tokens, 2U, count, &color) == 0U) {
            APP_Control_QueueText("ERR LED RGB needs r g b in 0..255\r\n");
            return 1U;
        }
        APP_LED_Identify(&color, led_parse_hold(tokens, 5U, count));
        APP_Control_QueueText("OK LED RGB %u %u %u\r\n",
                              (unsigned int)color.r, (unsigned int)color.g,
                              (unsigned int)color.b);
        return 1U;
    }

    if (strcmp(tokens[1], "BLINK") == 0) {
        uint32_t on_ms;
        uint32_t off_ms;

        if ((led_parse_color(tokens, 2U, count, &color) == 0U) || (count < 7U) ||
            (app_control_parse_u32(tokens[5], &on_ms) == 0U) ||
            (app_control_parse_u32(tokens[6], &off_ms) == 0U) ||
            (on_ms > 60000U) || (off_ms > 60000U)) {
            APP_Control_QueueText("ERR LED BLINK needs r g b on_ms off_ms\r\n");
            return 1U;
        }
        pattern.color = color;
        pattern.effect = DRV_RGB_EFFECT_BLINK;
        pattern.on_ms = (uint16_t)on_ms;
        pattern.off_ms = (uint16_t)off_ms;
        APP_LED_IdentifyPattern(&pattern, led_parse_hold(tokens, 7U, count));
        APP_Control_QueueText("OK LED BLINK %lu/%lu\r\n",
                              (unsigned long)on_ms, (unsigned long)off_ms);
        return 1U;
    }

    if (strcmp(tokens[1], "BREATHE") == 0) {
        uint32_t period_ms;

        if ((led_parse_color(tokens, 2U, count, &color) == 0U) || (count < 6U) ||
            (app_control_parse_u32(tokens[5], &period_ms) == 0U) ||
            (period_ms < 100U) || (period_ms > 60000U)) {
            APP_Control_QueueText("ERR LED BREATHE needs r g b period_ms(100..60000)\r\n");
            return 1U;
        }
        pattern.color = color;
        pattern.effect = DRV_RGB_EFFECT_BREATHE;
        pattern.period_ms = (uint16_t)period_ms;
        APP_LED_IdentifyPattern(&pattern, led_parse_hold(tokens, 6U, count));
        APP_Control_QueueText("OK LED BREATHE %lu\r\n", (unsigned long)period_ms);
        return 1U;
    }

    APP_Control_QueueText("ERR LED unknown op %s\r\n", tokens[1]);
    return 1U;
}

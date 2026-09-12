#include "drv_rgb_led.h"

#include <stddef.h>

/* Σ-Δ 的模。取 255 而不是 256，是为了让亮度 255 正好恒亮：
 * acc += 255 每拍都 >= 255，永远输出 1。用 256 的话满亮度也会漏掉一拍。 */
#define DRV_RGB_DITHER_SCALE 255U

static DRV_RgbColor rgb_scale(DRV_RgbColor color, uint32_t level)
{
    DRV_RgbColor out;

    out.r = (uint8_t)(((uint32_t)color.r * level) / 255U);
    out.g = (uint8_t)(((uint32_t)color.g * level) / 255U);
    out.b = (uint8_t)(((uint32_t)color.b * level) / 255U);
    return out;
}

static const DRV_RgbColor rgb_black = {0U, 0U, 0U};

/*
 * 眼睛对亮度的反应接近平方关系，所以线性扫的呼吸看起来会"一直很亮、突然暗一下"。
 * 先算三角波再平方，亮的那一半才不会糊成一片。一次乘法，没必要上查表。
 */
static uint32_t rgb_gamma(uint32_t linear)
{
    return (linear * linear) / 255U;
}

static uint32_t rgb_breathe_level(const DRV_RgbPattern *pattern, uint32_t now_ms)
{
    uint32_t period = (pattern->period_ms != 0U) ? (uint32_t)pattern->period_ms : 2000U;
    uint32_t half = period / 2U;
    uint32_t phase = now_ms % period;
    uint32_t triangle;
    uint32_t dim = pattern->dim;

    if (half == 0U) {
        return 255U;
    }
    /* 前半个周期由暗到亮，后半个周期原路返回——两半共用同一条曲线，
     * 亮度在顶点和谷底都不会跳。 */
    triangle = (phase < half) ? ((phase * 255U) / half)
                              : (((period - phase) * 255U) / half);
    triangle = rgb_gamma(triangle);
    if (dim >= 255U) {
        return 255U;
    }
    /* dim 是"最暗处还留多少"，避免全灭时看起来像灯坏了。 */
    return dim + ((triangle * (255U - dim)) / 255U);
}

static uint8_t rgb_pulses_on(const DRV_RgbPattern *pattern, uint32_t now_ms)
{
    uint32_t on_ms = (pattern->on_ms != 0U) ? (uint32_t)pattern->on_ms : 1U;
    uint32_t off_ms = (pattern->off_ms != 0U) ? (uint32_t)pattern->off_ms : 1U;
    uint32_t slot = on_ms + off_ms;
    uint32_t burst;
    uint32_t cycle;
    uint32_t phase;

    if (pattern->count == 0U) {
        return 0U;
    }
    burst = (uint32_t)pattern->count * slot;
    cycle = burst + (uint32_t)pattern->gap_ms;
    if (cycle == 0U) {
        return 0U;
    }
    phase = now_ms % cycle;
    if (phase >= burst) {
        return 0U;          /* 组间停顿：数灯的人靠这段黑来断句 */
    }
    return ((phase % slot) < on_ms) ? 1U : 0U;
}

DRV_RgbColor DRV_RgbLed_Sample(const DRV_RgbPattern *pattern, uint32_t now_ms)
{
    if (pattern == NULL) {
        return rgb_black;
    }

    switch (pattern->effect) {
    case DRV_RGB_EFFECT_SOLID:
        return pattern->color;

    case DRV_RGB_EFFECT_BLINK: {
        uint32_t on_ms = (pattern->on_ms != 0U) ? (uint32_t)pattern->on_ms : 1U;
        uint32_t off_ms = (pattern->off_ms != 0U) ? (uint32_t)pattern->off_ms : 1U;
        uint32_t phase = now_ms % (on_ms + off_ms);

        return (phase < on_ms) ? pattern->color : rgb_black;
    }

    case DRV_RGB_EFFECT_PULSES:
        return (rgb_pulses_on(pattern, now_ms) != 0U) ? pattern->color : rgb_black;

    case DRV_RGB_EFFECT_BREATHE:
        return rgb_scale(pattern->color, rgb_breathe_level(pattern, now_ms));

    case DRV_RGB_EFFECT_OFF:
    default:
        return rgb_black;
    }
}

void DRV_RgbLed_DitherReset(DRV_RgbDither *state)
{
    if (state == NULL) {
        return;
    }
    state->acc[0] = 0U;
    state->acc[1] = 0U;
    state->acc[2] = 0U;
}

uint8_t DRV_RgbLed_Modulate(DRV_RgbDither *state, DRV_RgbColor color)
{
    const uint8_t level[3] = {color.r, color.g, color.b};
    uint8_t bits = 0U;
    uint8_t index;

    if (state == NULL) {
        return 0U;
    }
    for (index = 0U; index < 3U; index++) {
        if (level[index] == 0U) {
            /* 亮度归零时把余量也清掉。不清的话，灯灭之后累加器里剩的那点余量
             * 会在下次点亮的第一拍提前触发，表现为一次多余的闪。 */
            state->acc[index] = 0U;
            continue;
        }
        state->acc[index] = (uint16_t)(state->acc[index] + level[index]);
        if (state->acc[index] >= DRV_RGB_DITHER_SCALE) {
            state->acc[index] = (uint16_t)(state->acc[index] - DRV_RGB_DITHER_SCALE);
            bits |= (uint8_t)(1U << index);
        }
    }
    return bits;
}

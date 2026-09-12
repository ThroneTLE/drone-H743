#include "bsp_rgb_led.h"

#include "bsp_gpio.h"
#include "drv_rgb_led.h"

/*
 * MicoAir743v2 板载三色 LED。
 *
 * **共阳接法：写 0 点亮。** 2026-09-12 实机确认——原来按"写 1 点亮"驱动时，
 * 蓝灯表现为"大部分时间亮、偶尔黑一下"，正好是解锁被拒那条"闪 1 下"图案的反相。
 * 极性错了不会让任何测试变红、也不会让灯不亮，只会让每一个图案都反过来读，
 * 所以这一条必须写在绑定里并配一条机检（tests/test_led_service.py）。
 *
 * 引脚出处：doc/micoair743v2/vendor/ardupilot-hwdef.dat
 *   PE3 LED_RED / PE2 LED_GREEN / PE4 LED_BLUE
 */
#define BSP_RGB_LED_ACTIVE_LOW 1U

static const BSP_GPIO_Pin rgb_pins[3] = {
    BSP_GPIO_PE3,   /* R */
    BSP_GPIO_PE2,   /* G */
    BSP_GPIO_PE4,   /* B */
};

static const char *const rgb_pin_names[3] = {"PE3", "PE2", "PE4"};

void BSP_RgbLed_Init(void)
{
    BSP_RgbLed_WriteBits(0U);
}

void BSP_RgbLed_WriteBits(uint8_t bits)
{
    uint8_t index;

    for (index = 0U; index < 3U; index++) {
        uint8_t on = ((bits >> index) & 1U);
#if (BSP_RGB_LED_ACTIVE_LOW != 0U)
        BSP_GPIO_Write(rgb_pins[index], (uint8_t)(on ^ 1U));
#else
        BSP_GPIO_Write(rgb_pins[index], on);
#endif
    }
}

const char *BSP_RgbLed_PinName(uint8_t channel)
{
    if (channel >= 3U) {
        return "-";
    }
    return rgb_pin_names[channel];
}

uint8_t BSP_RgbLed_IsActiveLow(void)
{
    return (uint8_t)BSP_RGB_LED_ACTIVE_LOW;
}

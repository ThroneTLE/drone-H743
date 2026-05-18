#include "bsp_pwm.h"

#include "tim.h"

static uint16_t esc_pulses_us[BSP_PWM_ESC_CHANNEL_COUNT] = {
    BSP_PWM_ESC_NEUTRAL_US,
    BSP_PWM_ESC_NEUTRAL_US,
    BSP_PWM_ESC_NEUTRAL_US,
    BSP_PWM_ESC_NEUTRAL_US
};
static uint8_t start_status[BSP_PWM_ESC_CHANNEL_COUNT];

static uint8_t pwm_started;

static uint32_t pwm_tim_channel(uint32_t channel)
{
    switch (channel) {
    case 1U:
        return TIM_CHANNEL_1;
    case 2U:
        return TIM_CHANNEL_2;
    case 3U:
        return TIM_CHANNEL_3;
    case 4U:
        return TIM_CHANNEL_4;
    default:
        return 0U;
    }
}

BSP_PWM_Status BSP_PWM_Init(void)
{
    uint32_t channel;

    for (channel = 1U; channel <= BSP_PWM_ESC_CHANNEL_COUNT; channel++) {
        HAL_StatusTypeDef status = HAL_TIM_PWM_Start(&htim2, pwm_tim_channel(channel));
        start_status[channel - 1U] = (uint8_t)status;
        if (status != HAL_OK) {
            return BSP_PWM_ERROR;
        }
    }

    pwm_started = 1U;
    for (channel = 1U; channel <= BSP_PWM_ESC_CHANNEL_COUNT; channel++) {
        (void)BSP_PWM_SetEscPulse(channel, BSP_PWM_ESC_NEUTRAL_US);
    }

    return BSP_PWM_OK;
}

BSP_PWM_Status BSP_PWM_SetEscPulse(uint32_t channel, uint16_t pulse_us)
{
    uint32_t tim_channel = pwm_tim_channel(channel);

    if ((channel == 0U) ||
        (channel > BSP_PWM_ESC_CHANNEL_COUNT) ||
        (pulse_us < BSP_PWM_ESC_MIN_US) ||
        (pulse_us > BSP_PWM_ESC_MAX_US)) {
        return BSP_PWM_INVALID_PARAM;
    }

    if (pwm_started == 0U) {
        BSP_PWM_Status init_status = BSP_PWM_Init();
        if (init_status != BSP_PWM_OK) {
            return init_status;
        }
    }

    esc_pulses_us[channel - 1U] = pulse_us;
    __HAL_TIM_SET_COMPARE(&htim2, tim_channel, pulse_us);
    return BSP_PWM_OK;
}

BSP_PWM_Status BSP_PWM_SetEscPercent(uint32_t channel, uint32_t percent)
{
    if (percent > BSP_PWM_ESC_MAX_PERCENT) {
        return BSP_PWM_INVALID_PARAM;
    }

    return BSP_PWM_SetEscPulse(channel, BSP_PWM_PercentToPulse(percent));
}

uint16_t BSP_PWM_PercentToPulse(uint32_t percent)
{
    uint32_t span = BSP_PWM_ESC_MAX_US - BSP_PWM_ESC_MIN_US;

    if (percent > BSP_PWM_ESC_MAX_PERCENT) {
        percent = BSP_PWM_ESC_MAX_PERCENT;
    }

    return (uint16_t)(BSP_PWM_ESC_MIN_US +
                      ((percent * span) / BSP_PWM_ESC_MAX_PERCENT));
}

uint16_t BSP_PWM_GetEscPulse(uint32_t channel)
{
    if ((channel == 0U) || (channel > BSP_PWM_ESC_CHANNEL_COUNT)) {
        return 0U;
    }

    return esc_pulses_us[channel - 1U];
}

uint8_t BSP_PWM_GetStartStatus(uint32_t channel)
{
    if ((channel == 0U) || (channel > BSP_PWM_ESC_CHANNEL_COUNT)) {
        return (uint8_t)HAL_ERROR;
    }

    return start_status[channel - 1U];
}

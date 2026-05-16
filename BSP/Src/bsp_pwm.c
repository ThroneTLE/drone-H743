#include "bsp_pwm.h"

#include "tim.h"

#define BSP_PWM_ESC_CHANNEL_COUNT 2U

static uint16_t esc_pulses_us[BSP_PWM_ESC_CHANNEL_COUNT] = {
    BSP_PWM_ESC_NEUTRAL_US,
    BSP_PWM_ESC_NEUTRAL_US
};

static uint8_t pwm_started;

static uint32_t pwm_tim_channel(uint32_t channel)
{
    switch (channel) {
    case 1U:
        return TIM_CHANNEL_1;
    case 2U:
        return TIM_CHANNEL_2;
    default:
        return 0U;
    }
}

BSP_PWM_Status BSP_PWM_Init(void)
{
    HAL_StatusTypeDef st1;
    HAL_StatusTypeDef st2;

    st1 = HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    st2 = HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    if ((st1 != HAL_OK) || (st2 != HAL_OK)) {
        return BSP_PWM_ERROR;
    }

    pwm_started = 1U;
    (void)BSP_PWM_SetEscPulse(1U, BSP_PWM_ESC_NEUTRAL_US);
    (void)BSP_PWM_SetEscPulse(2U, BSP_PWM_ESC_NEUTRAL_US);
    return BSP_PWM_OK;
}

BSP_PWM_Status BSP_PWM_SetEscPulse(uint32_t channel, uint16_t pulse_us)
{
    uint32_t tim_channel = pwm_tim_channel(channel);

    if ((tim_channel == 0U) ||
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

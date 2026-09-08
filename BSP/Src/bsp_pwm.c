#include "bsp_pwm.h"

#include "tim.h"

static uint16_t esc_pulses_us[BSP_PWM_ESC_CHANNEL_COUNT] = {
    BSP_PWM_ESC_NEUTRAL_US,
    BSP_PWM_ESC_NEUTRAL_US
};
static uint16_t servo_pulses_us[BSP_PWM_SERVO_CHANNEL_COUNT] = {
    BSP_PWM_SERVO_CENTER_US,
    BSP_PWM_SERVO_CENTER_US
};
static uint8_t start_status[BSP_PWM_TIM_CHANNEL_COUNT];

static uint8_t pwm_started;

/*
 * 一个帧里必须留得下最长脉冲，否则占空比会撞到 100%、输出退化成常高电平。
 * ESC：2500 us 帧装 1940 us 脉冲，余量 560 us。舵机：20000 us 帧装 2500 us。
 */
_Static_assert(BSP_PWM_ESC_FRAME_US > BSP_PWM_ESC_MAX_US,
               "ESC frame too short for the maximum pulse");
_Static_assert(BSP_PWM_SERVO_FRAME_US > BSP_PWM_SERVO_MAX_US,
               "servo frame too short for the maximum pulse");
_Static_assert((BSP_PWM_ESC_CHANNEL_COUNT + BSP_PWM_SERVO_CHANNEL_COUNT) ==
                   BSP_PWM_TIM_CHANNEL_COUNT,
               "start_status must cover every driven channel");

/* ESC 两路在 TIM5 上（PA0/PA1）。 */
static uint32_t pwm_esc_tim_channel(uint32_t channel)
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

/* 舵机两路留在 TIM2 上（PA2/PA3）。 */
static uint32_t pwm_servo_tim_channel(uint32_t channel)
{
    switch (channel) {
    case 1U:
        return TIM_CHANNEL_3;
    case 2U:
        return TIM_CHANNEL_4;
    default:
        return 0U;
    }
}

BSP_PWM_Status BSP_PWM_Init(void)
{
    uint32_t channel;

    /*
     * 帧率归 BSP，不靠 CubeMX 的 Period 初值（见 bsp_pwm.h）。时基 1 MHz，
     * 所以 ARR = 帧长(us) - 1。必须在 PWM_Start 之前写，此时计数器还在 0，
     * 不会出现"计数值已超过新 ARR、要绕一整圈才回卷"的情况。
     */
    __HAL_TIM_SET_AUTORELOAD(&htim5, BSP_PWM_ESC_FRAME_US - 1U);
    __HAL_TIM_SET_AUTORELOAD(&htim2, BSP_PWM_SERVO_FRAME_US - 1U);

    for (channel = 1U; channel <= BSP_PWM_ESC_CHANNEL_COUNT; channel++) {
        HAL_StatusTypeDef status =
            HAL_TIM_PWM_Start(&htim5, pwm_esc_tim_channel(channel));
        start_status[channel - 1U] = (uint8_t)status;
        if (status != HAL_OK) {
            return BSP_PWM_ERROR;
        }
    }
    for (channel = 1U; channel <= BSP_PWM_SERVO_CHANNEL_COUNT; channel++) {
        HAL_StatusTypeDef status =
            HAL_TIM_PWM_Start(&htim2, pwm_servo_tim_channel(channel));
        /* 诊断口 BSP_PWM_GetStartStatus(1..4) 的含义不变：1/2=ESC，3/4=舵机。 */
        start_status[BSP_PWM_ESC_CHANNEL_COUNT + channel - 1U] = (uint8_t)status;
        if (status != HAL_OK) {
            return BSP_PWM_ERROR;
        }
    }

    pwm_started = 1U;
    for (channel = 1U; channel <= BSP_PWM_ESC_CHANNEL_COUNT; channel++) {
        (void)BSP_PWM_SetEscPulse(channel, BSP_PWM_ESC_NEUTRAL_US);
    }
    for (channel = 1U; channel <= BSP_PWM_SERVO_CHANNEL_COUNT; channel++) {
        (void)BSP_PWM_SetServoPulse(channel, BSP_PWM_SERVO_CENTER_US);
    }

    return BSP_PWM_OK;
}

BSP_PWM_Status BSP_PWM_SetEscPulse(uint32_t channel, uint16_t pulse_us)
{
    uint32_t tim_channel = pwm_esc_tim_channel(channel);

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
    __HAL_TIM_SET_COMPARE(&htim5, tim_channel, pulse_us);
    return BSP_PWM_OK;
}

BSP_PWM_Status BSP_PWM_DisableEsc(uint32_t channel)
{
    uint32_t tim_channel = pwm_esc_tim_channel(channel);

    if ((channel == 0U) || (channel > BSP_PWM_ESC_CHANNEL_COUNT)) {
        return BSP_PWM_INVALID_PARAM;
    }

    if (pwm_started == 0U) {
        BSP_PWM_Status init_status = BSP_PWM_Init();
        if (init_status != BSP_PWM_OK) {
            return init_status;
        }
    }

    esc_pulses_us[channel - 1U] = 0U;
    __HAL_TIM_SET_COMPARE(&htim5, tim_channel, 0U);
    return BSP_PWM_OK;
}

BSP_PWM_Status BSP_PWM_SetServoPulse(uint32_t channel, uint16_t pulse_us)
{
    uint32_t tim_channel = pwm_servo_tim_channel(channel);

    if ((channel == 0U) ||
        (channel > BSP_PWM_SERVO_CHANNEL_COUNT) ||
        (pulse_us < BSP_PWM_SERVO_MIN_US) ||
        (pulse_us > BSP_PWM_SERVO_MAX_US)) {
        return BSP_PWM_INVALID_PARAM;
    }

    if (pwm_started == 0U) {
        BSP_PWM_Status init_status = BSP_PWM_Init();
        if (init_status != BSP_PWM_OK) {
            return init_status;
        }
    }

    servo_pulses_us[channel - 1U] = pulse_us;
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

uint16_t BSP_PWM_GetServoPulse(uint32_t channel)
{
    if ((channel == 0U) || (channel > BSP_PWM_SERVO_CHANNEL_COUNT)) {
        return 0U;
    }

    return servo_pulses_us[channel - 1U];
}

uint8_t BSP_PWM_GetStartStatus(uint32_t channel)
{
    if ((channel == 0U) || (channel > BSP_PWM_TIM_CHANNEL_COUNT)) {
        return (uint8_t)HAL_ERROR;
    }

    return start_status[channel - 1U];
}

#include "bsp_pwm.h"
#include "bsp_dshot.h"
#include "bsp_esc_protocol.h"
#include "drv_dshot.h"

#include "tim.h"

#include <stddef.h>
#include <string.h>

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
_Static_assert(BSP_PWM_ESC_MIN_US == DRV_DSHOT_EQUIV_MIN_US &&
               BSP_PWM_ESC_MAX_US == DRV_DSHOT_EQUIV_MAX_US,
               "PWM command bounds and DShot mapping must remain identical");

/*
 * ESC 两路在 TIM1_CH1/CH2 上（PE9 / PE11，即板上的 MOTOR4 / MOTOR3 焊盘）。
 *
 * 老板子用的是 TIM5_CH1/CH2（PA0/PA1）。MicoAir743v2 上 PA0/PA1 是 UART4，
 * PA2/PA3 是 USART2，四个脚全被串口占了，所以 ESC 与舵机都必须搬到电机焊盘。
 * 分两个定时器这一点没变，理由见 bsp_pwm.h 顶部——ESC 要 400 Hz、舵机要 50 Hz，
 * 共用一个定时器就只能共用一个 ARR，会把 ESC 拖回舵机的帧率。
 *
 * TIM1 是高级定时器，输出还需要 MOE；HAL_TIM_PWM_Start() 内部已经处理，
 * 不必额外调用，但换定时器时这一点必须确认过，否则波形永远出不来。
 */
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

/* 舵机两路在 TIM4_CH1/CH2 上（PD12 / PD13，即板上的 MOTOR7 / MOTOR8 焊盘）。 */
static uint32_t pwm_servo_tim_channel(uint32_t channel)
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
    uint32_t channel;

    /*
     * 帧率归 BSP，不靠 CubeMX 的 Period 初值（见 bsp_pwm.h）。时基 1 MHz，
     * 所以 ARR = 帧长(us) - 1。必须在 PWM_Start 之前写，此时计数器还在 0，
     * 不会出现"计数值已超过新 ARR、要绕一整圈才回卷"的情况。
     */
#if BSP_ESC_PROTOCOL == BSP_ESC_PROTOCOL_PWM
    /* CubeMX supplies DShot timing; restore the PWM backend's 1 MHz counter. */
    htim1.Instance->PSC = 119U;
    __HAL_TIM_SET_AUTORELOAD(&htim1, BSP_PWM_ESC_FRAME_US - 1U);
    htim1.Instance->EGR = TIM_EGR_UG;
#endif
    __HAL_TIM_SET_AUTORELOAD(&htim4, BSP_PWM_SERVO_FRAME_US - 1U);

#if BSP_ESC_PROTOCOL == BSP_ESC_PROTOCOL_DSHOT300
    BSP_DShotStatus dshot_status = BSP_DShot_Init();
    start_status[0] = start_status[1] = (uint8_t)(dshot_status == BSP_DSHOT_OK ? HAL_OK : HAL_ERROR);
    if (dshot_status != BSP_DSHOT_OK) { return BSP_PWM_ERROR; }
#else
    for (channel = 1U; channel <= BSP_PWM_ESC_CHANNEL_COUNT; channel++) {
        HAL_StatusTypeDef status =
            HAL_TIM_PWM_Start(&htim1, pwm_esc_tim_channel(channel));
        start_status[channel - 1U] = (uint8_t)status;
        if (status != HAL_OK) {
            return BSP_PWM_ERROR;
        }
    }
#endif
    for (channel = 1U; channel <= BSP_PWM_SERVO_CHANNEL_COUNT; channel++) {
        HAL_StatusTypeDef status =
            HAL_TIM_PWM_Start(&htim4, pwm_servo_tim_channel(channel));
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
#if BSP_ESC_PROTOCOL == BSP_ESC_PROTOCOL_DSHOT300
        return BSP_PWM_ERROR; /* recovery must be explicit */
#else
        BSP_PWM_Status init_status = BSP_PWM_Init();
        if (init_status != BSP_PWM_OK) { return init_status; }
#endif
    }

#if BSP_ESC_PROTOCOL == BSP_ESC_PROTOCOL_DSHOT300
    BSP_DShotSnapshot snapshot;
    BSP_DShot_GetSnapshot(&snapshot);
    if (snapshot.fault) { return BSP_PWM_ERROR; }
#endif
    esc_pulses_us[channel - 1U] = pulse_us;
#if BSP_ESC_PROTOCOL == BSP_ESC_PROTOCOL_PWM
    __HAL_TIM_SET_COMPARE(&htim1, tim_channel, pulse_us);
#else
    (void)tim_channel;
#endif
    return BSP_PWM_OK;
}

BSP_PWM_Status BSP_PWM_DisableEsc(uint32_t channel)
{
    uint32_t tim_channel = pwm_esc_tim_channel(channel);

    if ((channel == 0U) || (channel > BSP_PWM_ESC_CHANNEL_COUNT)) {
        return BSP_PWM_INVALID_PARAM;
    }

    if (pwm_started == 0U) {
#if BSP_ESC_PROTOCOL == BSP_ESC_PROTOCOL_DSHOT300
        esc_pulses_us[channel - 1U] = 0U;
        return BSP_DShot_Disable((uint8_t)(1U << (channel - 1U))) == BSP_DSHOT_OK ? BSP_PWM_OK : BSP_PWM_ERROR;
#else
        BSP_PWM_Status init_status = BSP_PWM_Init();
        if (init_status != BSP_PWM_OK) { return init_status; }
#endif
    }

    esc_pulses_us[channel - 1U] = 0U;
#if BSP_ESC_PROTOCOL == BSP_ESC_PROTOCOL_PWM
    __HAL_TIM_SET_COMPARE(&htim1, tim_channel, 0U);
#else
    (void)tim_channel;
    if (BSP_DShot_Disable((uint8_t)(1U << (channel - 1U))) != BSP_DSHOT_OK) {
        return BSP_PWM_ERROR;
    }
#endif
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
    __HAL_TIM_SET_COMPARE(&htim4, tim_channel, pulse_us);
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

const char *BSP_PWM_EscProtocol(void)
{
#if BSP_ESC_PROTOCOL == BSP_ESC_PROTOCOL_DSHOT300
    return "DSHOT300";
#else
    return "PWM";
#endif
}

BSP_PWM_Status BSP_PWM_CommitEsc(void)
{
#if BSP_ESC_PROTOCOL == BSP_ESC_PROTOCOL_DSHOT300
    uint16_t codes[2] = {0U, 0U};
    uint8_t mask = 0U;
    for (uint32_t i = 0U; i < 2U; ++i) {
        if (esc_pulses_us[i] != 0U) {
            if (DRV_DShot_FromPulseUs(esc_pulses_us[i], &codes[i]) != DRV_DSHOT_OK) {
                return BSP_PWM_INVALID_PARAM;
            }
            mask |= (uint8_t)(1U << i);
        }
    }
    BSP_DShotStatus status = BSP_DShot_Submit(codes, mask);
    if (status == BSP_DSHOT_BUSY) { return BSP_PWM_BUSY; }
    return status == BSP_DSHOT_OK ? BSP_PWM_OK : BSP_PWM_ERROR;
#else
    return BSP_PWM_OK;
#endif
}

uint8_t BSP_PWM_GetStartStatus(uint32_t channel)
{
    if ((channel == 0U) || (channel > BSP_PWM_TIM_CHANNEL_COUNT)) {
        return (uint8_t)HAL_ERROR;
    }

    return start_status[channel - 1U];
}

/* ---------------------------------------------------------------- 诊断快照 */

static void pwm_fill_timer_debug(BSP_PWM_TimerDebug *out,
                                 const char *name,
                                 const TIM_TypeDef *tim)
{
    if (out == NULL) {
        return;
    }

    (void)memset(out, 0, sizeof(*out));
    out->name = (name != NULL) ? name : "?";
    if (tim == NULL) {
        return;
    }

    out->cr1    = tim->CR1;
    out->ccer   = tim->CCER;
    out->ccmr1  = tim->CCMR1;
    out->ccmr2  = tim->CCMR2;
    out->psc    = tim->PSC;
    out->arr    = tim->ARR;
    out->cnt    = tim->CNT;
    out->ccr[0] = tim->CCR1;
    out->ccr[1] = tim->CCR2;
    out->ccr[2] = tim->CCR3;
    out->ccr[3] = tim->CCR4;
}

void BSP_PWM_GetEscTimerDebug(BSP_PWM_TimerDebug *out)
{
    pwm_fill_timer_debug(out, "tim1", htim1.Instance);
}

void BSP_PWM_GetServoTimerDebug(BSP_PWM_TimerDebug *out)
{
    pwm_fill_timer_debug(out, "tim4", htim4.Instance);
}

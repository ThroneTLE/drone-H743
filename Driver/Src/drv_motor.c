#include "drv_motor.h"

#include "bsp_pwm.h"

static uint32_t motor_percent[DRV_MOTOR_COUNT];

static uint32_t motor_to_pwm_channel(uint32_t motor)
{
    switch (motor) {
    case DRV_MOTOR_ID_1:
        return 1U;
    case DRV_MOTOR_ID_2:
        return 2U;
    default:
        return 0U;
    }
}

static DRV_MOTOR_Status motor_from_pwm_status(BSP_PWM_Status status)
{
    switch (status) {
    case BSP_PWM_OK:
        return DRV_MOTOR_OK;
    case BSP_PWM_INVALID_PARAM:
        return DRV_MOTOR_INVALID_PARAM;
    default:
        return DRV_MOTOR_ERROR;
    }
}

DRV_MOTOR_Status DRV_Motor_SetPercent(uint32_t motor, uint32_t percent)
{
    uint32_t pwm_channel = motor_to_pwm_channel(motor);
    BSP_PWM_Status pwm_status;

    if (motor == DRV_MOTOR_ID_BOTH) {
        DRV_MOTOR_Status st1;
        DRV_MOTOR_Status st2;

        if (percent > DRV_MOTOR_PERCENT_MAX) {
            return DRV_MOTOR_INVALID_PARAM;
        }

        st1 = DRV_Motor_SetPercent(DRV_MOTOR_ID_1, percent);
        st2 = DRV_Motor_SetPercent(DRV_MOTOR_ID_2, percent);
        return ((st1 == DRV_MOTOR_OK) && (st2 == DRV_MOTOR_OK)) ? DRV_MOTOR_OK : DRV_MOTOR_ERROR;
    }

    if ((pwm_channel == 0U) || (percent > DRV_MOTOR_PERCENT_MAX)) {
        return DRV_MOTOR_INVALID_PARAM;
    }

    pwm_status = BSP_PWM_SetEscPercent(pwm_channel, percent);
    if (pwm_status == BSP_PWM_OK) {
        motor_percent[motor - 1U] = percent;
    }

    return motor_from_pwm_status(pwm_status);
}

DRV_MOTOR_Status DRV_Motor_Stop(uint32_t motor)
{
    return DRV_Motor_SetPercent(motor, 0U);
}

DRV_MOTOR_Status DRV_Motor_StopAll(void)
{
    DRV_MOTOR_Status st1 = DRV_Motor_Stop(DRV_MOTOR_ID_1);
    DRV_MOTOR_Status st2 = DRV_Motor_Stop(DRV_MOTOR_ID_2);

    return ((st1 == DRV_MOTOR_OK) && (st2 == DRV_MOTOR_OK)) ? DRV_MOTOR_OK : DRV_MOTOR_ERROR;
}

uint32_t DRV_Motor_GetPercent(uint32_t motor)
{
    if ((motor == 0U) || (motor > DRV_MOTOR_COUNT)) {
        return 0U;
    }

    return motor_percent[motor - 1U];
}

uint16_t DRV_Motor_GetPulse(uint32_t motor)
{
    uint32_t pwm_channel = motor_to_pwm_channel(motor);

    if (pwm_channel == 0U) {
        return 0U;
    }

    return BSP_PWM_GetEscPulse(pwm_channel);
}

uint16_t DRV_Motor_PercentToPulse(uint32_t percent)
{
    if (percent > DRV_MOTOR_PERCENT_MAX) {
        percent = DRV_MOTOR_PERCENT_MAX;
    }

    return BSP_PWM_PercentToPulse(percent);
}

DRV_MOTOR_Status DRV_Motor_ThrustToPwmPercent(uint32_t thrust_percent,
                                               uint32_t *motor1_percent,
                                               uint32_t *motor2_percent)
{
    if ((motor1_percent == 0) || (motor2_percent == 0)) {
        return DRV_MOTOR_INVALID_PARAM;
    }

    if (thrust_percent > DRV_MOTOR_PERCENT_MAX) {
        thrust_percent = DRV_MOTOR_PERCENT_MAX;
    }

    *motor1_percent = thrust_percent;
    *motor2_percent = thrust_percent;
    return DRV_MOTOR_OK;
}

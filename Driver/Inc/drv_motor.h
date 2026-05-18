#ifndef DRV_MOTOR_H
#define DRV_MOTOR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    DRV_MOTOR_OK = 0,
    DRV_MOTOR_ERROR,
    DRV_MOTOR_INVALID_PARAM
} DRV_MOTOR_Status;

#define DRV_MOTOR_ID_BOTH 0U
#define DRV_MOTOR_ID_1 1U
#define DRV_MOTOR_ID_2 2U
#define DRV_MOTOR_COUNT 2U
#define DRV_MOTOR_PERCENT_MAX 100U

DRV_MOTOR_Status DRV_Motor_SetPercent(uint32_t motor, uint32_t percent);
DRV_MOTOR_Status DRV_Motor_Stop(uint32_t motor);
DRV_MOTOR_Status DRV_Motor_StopAll(void);
uint32_t DRV_Motor_GetPercent(uint32_t motor);
uint16_t DRV_Motor_GetPulse(uint32_t motor);
uint16_t DRV_Motor_PercentToPulse(uint32_t percent);
DRV_MOTOR_Status DRV_Motor_ThrustToPwmPercent(uint32_t thrust_percent,
                                               uint32_t *motor1_percent,
                                               uint32_t *motor2_percent);

#ifdef __cplusplus
}
#endif

#endif

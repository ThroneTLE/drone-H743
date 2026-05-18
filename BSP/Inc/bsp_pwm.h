#ifndef BSP_PWM_H
#define BSP_PWM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BSP_PWM_OK = 0,
    BSP_PWM_ERROR,
    BSP_PWM_INVALID_PARAM
} BSP_PWM_Status;

#define BSP_PWM_ESC_MIN_US      1100U
#define BSP_PWM_ESC_MAX_US      1940U
#define BSP_PWM_ESC_NEUTRAL_US  1100U
#define BSP_PWM_ESC_MAX_PERCENT 100U
#define BSP_PWM_ESC_CHANNEL_COUNT 4U

BSP_PWM_Status BSP_PWM_Init(void);
BSP_PWM_Status BSP_PWM_SetEscPulse(uint32_t channel, uint16_t pulse_us);
BSP_PWM_Status BSP_PWM_SetEscPercent(uint32_t channel, uint32_t percent);
uint16_t BSP_PWM_PercentToPulse(uint32_t percent);
uint16_t BSP_PWM_GetEscPulse(uint32_t channel);
uint8_t BSP_PWM_GetStartStatus(uint32_t channel);

#ifdef __cplusplus
}
#endif

#endif

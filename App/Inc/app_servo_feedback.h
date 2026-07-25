#ifndef APP_SERVO_FEEDBACK_H
#define APP_SERVO_FEEDBACK_H

#include "drv_servo.h"

#include <stdint.h>

#define APP_SERVO_FEEDBACK_SLOT_COUNT       2U
#define APP_SERVO_FEEDBACK_ALPHA_VALID_MASK (1U << 0)
#define APP_SERVO_FEEDBACK_BETA_VALID_MASK  (1U << 1)

typedef struct {
    uint16_t position_us[APP_SERVO_FEEDBACK_SLOT_COUNT];
    uint16_t age_ms[APP_SERVO_FEEDBACK_SLOT_COUNT];
    uint16_t sample_sequence[APP_SERVO_FEEDBACK_SLOT_COUNT];
    uint8_t valid_mask;
} APP_ServoFeedbackLogSample;

void APP_ServoFeedback_Init(void);
void APP_ServoFeedback_Service(uint32_t now_ms,
                               const DRV_SERVO_MoveCmd moves[2],
                               uint8_t polling_enabled);
void APP_ServoFeedback_GetLogSample(uint32_t now_ms,
                                    APP_ServoFeedbackLogSample *sample);

#endif

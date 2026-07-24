#ifndef APP_SERVO_FEEDBACK_BENCH_H
#define APP_SERVO_FEEDBACK_BENCH_H

#include "drv_servo.h"

#include <stdint.h>

void APP_ServoFeedbackBench_Init(void);
uint8_t APP_ServoFeedbackBench_Start(uint32_t rate_hz,
                                     uint32_t duration_ms,
                                     uint32_t timeout_ms,
                                     uint32_t now_ms);
uint8_t APP_ServoFeedbackBench_StartSweep(uint32_t duration_ms,
                                          uint32_t timeout_ms,
                                          uint32_t now_ms);
void APP_ServoFeedbackBench_Stop(const char *reason, uint32_t now_ms);
void APP_ServoFeedbackBench_ReportStatus(uint32_t now_ms);
void APP_ServoFeedbackBench_Step(uint32_t now_ms,
                                const DRV_SERVO_MoveCmd moves[2]);
void APP_ServoFeedbackBench_RecordMoveResult(DRV_SERVO_Status status);
uint8_t APP_ServoFeedbackBench_IsActive(void);
uint8_t APP_ServoFeedbackBench_MoveRefreshDue(uint32_t now_ms,
                                              uint32_t last_move_ms);

#endif

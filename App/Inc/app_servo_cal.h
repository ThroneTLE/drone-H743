#ifndef APP_SERVO_CAL_H
#define APP_SERVO_CAL_H

#include <stdint.h>

typedef enum {
    APP_SERVO_CAL_STATE_IDLE = 0,
    APP_SERVO_CAL_STATE_RELEASED = 1,
    APP_SERVO_CAL_STATE_SAVE_LOCK_ACK = 2,
    APP_SERVO_CAL_STATE_ERROR = 3,
} APP_ServoCalState;

typedef enum {
    APP_SERVO_CAL_RESULT_NONE = 0,
    APP_SERVO_CAL_RESULT_RELEASED = 1,
    APP_SERVO_CAL_RESULT_SAVED = 2,
    APP_SERVO_CAL_RESULT_ERROR = 3,
} APP_ServoCalResult;

void APP_ServoCal_Init(void);
APP_ServoCalResult APP_ServoCal_Step(const uint16_t ch[16],
                                     uint8_t rc_link_ok,
                                     uint8_t rc_arm_switch_high,
                                     uint32_t now_ms);
uint8_t APP_ServoCal_IsActive(void);
APP_ServoCalState APP_ServoCal_GetState(void);
/* 取走控制环里缓存的最新事件文本（0 = 无事件）。只允许通信任务调用。 */
uint16_t APP_ServoCal_TakeNotice(char *out, uint16_t capacity);

#endif

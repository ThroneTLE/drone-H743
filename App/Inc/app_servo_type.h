#ifndef APP_SERVO_TYPE_H
#define APP_SERVO_TYPE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    APP_SERVO_TYPE_BUS = 0U,
    APP_SERVO_TYPE_PWM = 1U,
} APP_ServoType;

#define APP_SERVO_TYPE_IS_VALID(type) \
    (((type) == APP_SERVO_TYPE_BUS) || ((type) == APP_SERVO_TYPE_PWM))

/*
 * 没有显式设定时用的舵机类型（作者 2026-09-25：默认 PWM 舵机，接 MOTOR7/MOTOR8 焊盘，
 * 即 PD12/PD13）。FCAL 里显式写过类型的记录仍按记录；只有从未设定或记录无效时才用它。
 * 记录里的编码（0=bus，1=pwm）不变，旧记录照常读取。
 */
#define APP_SERVO_TYPE_DEFAULT APP_SERVO_TYPE_PWM

uint8_t APP_ServoType_IsValid(APP_ServoType type);
const char *APP_ServoType_Name(APP_ServoType type);
uint8_t APP_ServoType_FromName(const char *name, APP_ServoType *type);

void APP_ServoType_ResetActive(void);
uint8_t APP_ServoType_PublishActive(APP_ServoType type);
APP_ServoType APP_ServoType_GetActive(void);
uint32_t APP_ServoType_GetGeneration(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_SERVO_TYPE_H */

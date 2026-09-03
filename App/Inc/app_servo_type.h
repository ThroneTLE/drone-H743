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

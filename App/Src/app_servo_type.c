#include "app_servo_type.h"

#include <string.h>

static volatile uint32_t app_servo_type_generation;
static volatile uint8_t app_servo_type_initialized;
static volatile uint8_t app_servo_type_active;

uint8_t APP_ServoType_IsValid(APP_ServoType type)
{
    return APP_SERVO_TYPE_IS_VALID(type) ? 1U : 0U;
}

const char *APP_ServoType_Name(APP_ServoType type)
{
    if (type == APP_SERVO_TYPE_PWM) {
        return "pwm";
    }
    return "bus";
}

uint8_t APP_ServoType_FromName(const char *name, APP_ServoType *type)
{
    if ((name == NULL) || (type == NULL)) {
        return 0U;
    }
    if (strcmp(name, "bus") == 0) {
        *type = APP_SERVO_TYPE_BUS;
        return 1U;
    }
    if (strcmp(name, "pwm") == 0) {
        *type = APP_SERVO_TYPE_PWM;
        return 1U;
    }
    return 0U;
}

void APP_ServoType_ResetActive(void)
{
    app_servo_type_active = (uint8_t)APP_SERVO_TYPE_BUS;
    app_servo_type_generation = 0U;
    app_servo_type_initialized = 0U;
}

uint8_t APP_ServoType_PublishActive(APP_ServoType type)
{
    if (APP_ServoType_IsValid(type) == 0U) {
        return 0U;
    }
    if ((app_servo_type_initialized != 0U) &&
        (app_servo_type_active == (uint8_t)type)) {
        return 1U;
    }
    app_servo_type_active = (uint8_t)type;
    app_servo_type_generation++;
    if (app_servo_type_generation == 0U) {
        app_servo_type_generation = 1U;
    }
    app_servo_type_initialized = 1U;
    return 1U;
}

APP_ServoType APP_ServoType_GetActive(void)
{
    APP_ServoType type = (APP_ServoType)app_servo_type_active;

    if (app_servo_type_initialized == 0U) {
        return APP_SERVO_TYPE_BUS;
    }
    return (APP_ServoType_IsValid(type) != 0U) ? type : APP_SERVO_TYPE_BUS;
}

uint32_t APP_ServoType_GetGeneration(void)
{
    return app_servo_type_generation;
}

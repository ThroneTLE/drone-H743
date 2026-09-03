#include "app_servo_bus_guard.h"

#include <string.h>

uint8_t APP_ServoBusGuard_IsPwmMode(void)
{
    return (APP_ServoType_GetActive() == APP_SERVO_TYPE_PWM) ? 1U : 0U;
}

uint8_t APP_ServoBusGuard_IsBusOnlyCommand(const char *subcommand)
{
    static const char *const bus_only_commands[] = {
        "MOVE",
        "MOVEALL",
        "ANGLE",
        "ID",
        "SETID",
        "MODE",
        "ENABLE",
        "CMD",
        "RAW",
        "BAUDRATE",
        "FB",
    };

    if (subcommand == NULL) {
        return 0U;
    }

    for (uint32_t index = 0U;
         index < (sizeof(bus_only_commands) / sizeof(bus_only_commands[0]));
         ++index) {
        if (strcmp(subcommand, bus_only_commands[index]) == 0) {
            return 1U;
        }
    }
    return 0U;
}

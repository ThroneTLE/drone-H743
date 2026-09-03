#ifndef APP_SERVO_BUS_GUARD_H
#define APP_SERVO_BUS_GUARD_H

#include <stdint.h>

#include "app_servo_type.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Runtime selector shared by command and control-loop bus-only paths. */
uint8_t APP_ServoBusGuard_IsPwmMode(void);

/* Return non-zero for SERVO subcommands that require a bus servo. */
uint8_t APP_ServoBusGuard_IsBusOnlyCommand(const char *subcommand);

#ifdef __cplusplus
}
#endif

#endif /* APP_SERVO_BUS_GUARD_H */

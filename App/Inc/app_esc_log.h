#ifndef APP_ESC_LOG_H
#define APP_ESC_LOG_H
#include <stdint.h>

/* FlightLog V11 extension, little-endian 4B2H6I, captured at Observe time.
 * present=0 means no DShot backend. Counters are since explicit BSP Init.
 * Codes are last accepted values, not a response/acknowledgement from an ESC.
 */
typedef struct __attribute__((packed)) {
    uint8_t present, enabled_mask, busy, fault;
    uint16_t upper_code, lower_code;
    uint32_t submitted, completed, busy_rejected, errors, cancelled, timer_clock_hz;
} APP_EscLog;

APP_EscLog APP_EscLog_Capture(void);
#endif

#ifndef APP_CURRENT_FORMAT_H
#define APP_CURRENT_FORMAT_H

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* CURRENT diagnostic decimals without newlib-nano floating printf or heap.
 * Caller owns 16 bytes. Supports the existing 3/5 decimal wire fields.
 * Non-finite/out-of-range values remain explicit nan, never fabricated zero.
 */
static inline uint8_t APP_Current_FormatFixed(char out[16], float value, unsigned digits)
{
    uint32_t scale = digits == 5U ? 100000U : 1000U;
    double scaled = fabs((double)value) * (double)scale;
    if ((digits != 3U && digits != 5U) || !isfinite(scaled) ||
        scaled > (double)UINT32_MAX - 0.5) {
        memcpy(out, "nan", 4U);
        return 0U;
    }
    uint32_t rounded = (uint32_t)(scaled + 0.5);
    (void)snprintf(out, 16U, "%s%lu.%0*lu", signbit(value) ? "-" : "",
                   (unsigned long)(rounded / scale), (int)digits,
                   (unsigned long)(rounded % scale));
    return 1U;
}
#endif

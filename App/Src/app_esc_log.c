#include "app_esc_log.h"
#include "bsp_dshot.h"
#include "bsp_esc_protocol.h"

_Static_assert(sizeof(APP_EscLog) == 32U, "FlightLog DShot extension ABI");

APP_EscLog APP_EscLog_Capture(void)
{
    APP_EscLog out = {0};
#if BSP_ESC_PROTOCOL == BSP_ESC_PROTOCOL_DSHOT300
    BSP_DShotSnapshot s;
    BSP_DShot_GetSnapshot(&s);
    out.present = 1U;
    out.enabled_mask = s.enabled_mask;
    out.busy = s.busy;
    out.fault = s.fault;
    out.upper_code = s.code[0];
    out.lower_code = s.code[1];
    out.submitted = s.submitted;
    out.completed = s.completed;
    out.busy_rejected = s.busy_rejected;
    out.errors = s.errors;
    out.cancelled = s.cancelled;
    out.timer_clock_hz = s.timer_clock_hz;
#endif
    return out;
}

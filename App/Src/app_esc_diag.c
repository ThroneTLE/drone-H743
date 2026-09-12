#include "app_esc_diag.h"
#include "app_control.h"
#include "bsp_pwm.h"
#include "bsp_dshot.h"
#include "bsp_esc_protocol.h"

void APP_EscDiag_Report(void)
{
#if BSP_ESC_PROTOCOL == BSP_ESC_PROTOCOL_DSHOT300
    BSP_DShotSnapshot s = {0};
    BSP_DShot_GetSnapshot(&s);
    APP_Control_QueueText(
        "ESC protocol=%s command_unit=pwm_equivalent_us ack=unavailable code=%u,%u enabled=%u busy=%u fault=%u submitted=%lu completed=%lu busy_rejected=%lu errors=%lu cancelled=%lu\r\n",
        BSP_PWM_EscProtocol(), (unsigned)s.code[0], (unsigned)s.code[1],
        (unsigned)s.enabled_mask, (unsigned)s.busy, (unsigned)s.fault,
        (unsigned long)s.submitted, (unsigned long)s.completed,
        (unsigned long)s.busy_rejected, (unsigned long)s.errors, (unsigned long)s.cancelled);
#else
    APP_Control_QueueText("ESC protocol=PWM command_unit=us ack=unavailable\r\n");
#endif
}

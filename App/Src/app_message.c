#include "app_message.h"

#include "app_baro.h"
#include "app_current.h"
#include "app_battery.h"
#include "app_flash.h"
#include "app_messages.h"
#include "app_proto.h"
#include "app_tasks.h"
#include "app_thrust_lut.h"

#define APP_MESSAGE_STARTUP_REPORT_ENABLED 0U

void APP_Message_Task_Init(void)
{
    APP_Current_Init();
    APP_Battery_Init();
#if (APP_MESSAGE_STARTUP_REPORT_ENABLED != 0U)
    APP_Flash_ReportStartup();
    APP_Baro_ReportStartup();
#endif
}

void APP_Message_Task_Step(void)
{
    /* Current samples must not wait behind storage scans/locks. This existing
     * worker has no slow periodic jobs; it is separate from the control task.
     * One bounded ADC conversion, then sleep -- never replay missed samples. */
    APP_Current_Step();
    APP_Battery_Step();
    APP_ThrustLut_Step();
    APP_Battery_TelemetryStep();
    uint32_t ticks = (APP_CURRENT_PERIOD_MS * osKernelGetTickFreq() + 999U) / 1000U;
    (void)osDelay(ticks != 0U ? ticks : 1U);
}

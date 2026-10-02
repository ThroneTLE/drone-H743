#include "app_flight_log_nav.h"

#include "app_battery.h"
#include "drv_nav_ekf.h"
#include "svc_flow_nav.h"

#include <string.h>

void APP_FlightLogNav_Capture(APP_FlightLogNavTail *tail, uint16_t caller_flags)
{
    DRV_NAV_EKF_Diagnostics ekf;
    SVC_FLOW_NAV_State nav;
    APP_BatterySnapshot battery;
    uint16_t flags = (uint16_t)(caller_flags & APP_FLIGHT_LOG_NAV_CALLER_MASK);

    if (tail == NULL) {
        return;
    }

    memset(tail, 0, sizeof(*tail));
    memset(&ekf, 0, sizeof(ekf));
    memset(&nav, 0, sizeof(nav));
    memset(&battery, 0, sizeof(battery));
    SVC_FlowNav_GetEkfDiagnostics(&ekf);
    SVC_FlowNav_GetState(&nav);
    APP_Battery_GetSnapshot(&battery);

    memcpy(tail->ekf_accel_bias_m_s2, ekf.accel_bias_m_s2,
           sizeof(tail->ekf_accel_bias_m_s2));
    memcpy(tail->ekf_innovation_m_s, ekf.last_innovation_m_s,
           sizeof(tail->ekf_innovation_m_s));
    tail->ekf_nis = ekf.last_nis;
    tail->ekf_flow_update_count = ekf.flow_update_count;
    tail->ekf_flow_reject_count = ekf.flow_reject_count;
    tail->flow_filtered_x = nav.flow_vel_x_filtered;
    tail->flow_filtered_y = nav.flow_vel_y_filtered;

    if (nav.velocity_valid != 0U) {
        flags |= APP_FLIGHT_LOG_NAV_FLAG_VELOCITY_VALID;
    }
    if (nav.flow_filter_ready != 0U) {
        flags |= APP_FLIGHT_LOG_NAV_FLAG_FLOW_FILTER_READY;
    }
    if (ekf.initialized != 0U) {
        flags |= APP_FLIGHT_LOG_NAV_FLAG_EKF_INITIALIZED;
    }
    if (battery.state.valid != 0U) {
        flags |= APP_FLIGHT_LOG_NAV_FLAG_BATTERY_VALID;
        tail->battery_mv = (battery.state.voltage_mv > 65535UL) ?
            65535U : (uint16_t)battery.state.voltage_mv;
    }
    if (battery.state.low != 0U) {
        flags |= APP_FLIGHT_LOG_NAV_FLAG_BATTERY_LOW;
    }
    if (battery.can_arm != 0U) {
        flags |= APP_FLIGHT_LOG_NAV_FLAG_BATTERY_CAN_ARM;
    }
    if (battery.state.saturated != 0U) {
        flags |= APP_FLIGHT_LOG_NAV_FLAG_BATTERY_SATURATED;
    }
    tail->nav_flags = flags;
}

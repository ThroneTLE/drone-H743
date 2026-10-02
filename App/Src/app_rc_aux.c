/*
 * 遥控辅助开关（CH9 手动 IMU 归零），规则见 app_rc_aux.h。
 */
#include "app_rc_aux.h"

#include "app_control.h"
#include "app_sensor.h"
#include "app_stabilizer.h"

#include <stddef.h>

static APP_RcAuxState app_rc_aux_state;

uint8_t APP_RcAux_Step(APP_RcAuxState *state, uint32_t now_ms, uint16_t channel_us,
                       uint8_t link_ok, uint8_t armed)
{
    if (state == NULL) {
        return APP_RC_AUX_ACTION_NONE;
    }
    if ((state->holdoff_active != 0U) &&
        ((int32_t)(now_ms - state->holdoff_until_ms) >= 0)) {
        state->holdoff_active = 0U;
    }
    if (link_ok == 0U) {
        state->low_seen = 0U;
        return APP_RC_AUX_ACTION_NONE;
    }
    if ((channel_us > 0U) && (channel_us < APP_RC_AUX_LOW_US)) {
        state->low_seen = 1U;
        return APP_RC_AUX_ACTION_NONE;
    }
    if ((channel_us <= APP_RC_AUX_HIGH_US) || (state->low_seen == 0U)) {
        return APP_RC_AUX_ACTION_NONE;      /* 中间带（回差）或没见过低位 */
    }
    /* 上升沿：无论这次是否执行，都要先拨回低位才算下一次。 */
    state->low_seen = 0U;
    if (armed != 0U) {
        return APP_RC_AUX_ACTION_ARMED_BLOCKED;
    }
    if (state->holdoff_active != 0U) {
        return APP_RC_AUX_ACTION_NONE;
    }
    state->holdoff_active = 1U;
    state->holdoff_until_ms = now_ms + APP_RC_AUX_IMUZERO_HOLDOFF_MS;
    return APP_RC_AUX_ACTION_IMUZERO;
}

void APP_RcAux_Update(uint32_t now_ms, const uint16_t ch[16], uint8_t link_ok, uint8_t armed)
{
    uint8_t action;

    if (ch == NULL) {
        return;
    }
    action = APP_RcAux_Step(&app_rc_aux_state, now_ms, ch[APP_RC_AUX_IMUZERO_CHANNEL],
                            link_ok, armed);
    if (action == APP_RC_AUX_ACTION_IMUZERO) {
        APP_Sensor_RequestGyroRecal();
        APP_Stabilizer_RequestAttitudeRezero();
        APP_Control_QueueText("IMUZERO state=restarted keep_still_ms=3000 source=rc\r\n");
    } else if (action == APP_RC_AUX_ACTION_ARMED_BLOCKED) {
        APP_Control_QueueText("IMUZERO state=armed_blocked source=rc\r\n");
    }
}

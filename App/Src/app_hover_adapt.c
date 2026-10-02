/*
 * 悬停推力自适应接入控制器的策略层，规则见 app_hover_adapt.h。
 */
#include "app_hover_adapt.h"

#include "app_hover_thrust.h"
#include "drv_coax_ctrl.h"
#include "drv_hover_adapt.h"

#include <stddef.h>

static APP_HoverAdaptState hover_adapt_state;
static volatile uint8_t hover_adapt_enabled = 1U;
static volatile float hover_adapt_applied_mirror = 0.0f;

uint8_t APP_HoverAdapt_Step(APP_HoverAdaptState *state, const APP_HoverAdaptInput *in)
{
    uint8_t action = APP_HOVER_ADAPT_ACTION_NONE;
    float target;

    if ((state == NULL) || (in == NULL)) {
        return action;
    }
    if (!(in->cfg_n > 0.0f)) {
        state->applied_n = 0.0f;              /* 配置值都没有：不覆盖 */
        state->armed_prev = in->armed;
        return action;
    }
    if ((in->armed != 0U) && (state->armed_prev == 0U)) {
        /* 每次解锁重新学：请求学习器重置，已用值回配置值。 */
        action = APP_HOVER_ADAPT_ACTION_RESET_LEARNER;
        state->wait_reset = 1U;
        state->applied_n = in->cfg_n;
    }
    state->armed_prev = in->armed;
    if ((state->wait_reset != 0U) && (in->samples == 0U)) {
        state->wait_reset = 0U;
    }
    if (in->armed == 0U) {
        state->applied_n = in->cfg_n;
        return action;
    }
    if (in->bypass != 0U) {
        if (!(state->applied_n > 0.0f)) {
            state->applied_n = in->cfg_n;
        }
        return action;
    }
    target = DRV_HoverAdapt_Target(in->enabled,
                                   (uint8_t)((in->converged != 0U) && (state->wait_reset == 0U)),
                                   in->est_n, in->cfg_n, state->applied_n);
    state->applied_n = DRV_HoverAdapt_Slew(state->applied_n, target, in->dt_s);
    return action;
}

void APP_HoverAdapt_Update(float dt_s, uint8_t armed, uint8_t bypass)
{
    APP_HoverThrustSnapshot snap;
    APP_HoverAdaptInput in;

    in.dt_s = dt_s;
    in.armed = armed;
    in.bypass = bypass;
    in.enabled = hover_adapt_enabled;
    in.cfg_n = DRV_COAX_CTRL_ConfiguredHoverThrustN();
    in.converged = 0U;
    in.samples = 0U;
    in.est_n = 0.0f;
    if (APP_HoverThrust_GetSnapshot(&snap) != 0U) {
        in.converged = snap.est.converged;
        in.samples = snap.est.samples;
        in.est_n = snap.est.est_n;
    } else {
        in.samples = 1U;                      /* 读不到快照：不当作"已重置" */
    }
    if (APP_HoverAdapt_Step(&hover_adapt_state, &in) == APP_HOVER_ADAPT_ACTION_RESET_LEARNER) {
        APP_HoverThrust_RequestReset();
    }
    DRV_COAX_CTRL_SetHoverThrustAdapt(hover_adapt_state.applied_n);
    hover_adapt_applied_mirror = hover_adapt_state.applied_n;
}

void APP_HoverAdapt_SetEnabled(uint8_t enabled)
{
    hover_adapt_enabled = (enabled != 0U) ? 1U : 0U;
}

uint8_t APP_HoverAdapt_IsEnabled(void)
{
    return hover_adapt_enabled;
}

float APP_HoverAdapt_GetApplied(void)
{
    return hover_adapt_applied_mirror;
}

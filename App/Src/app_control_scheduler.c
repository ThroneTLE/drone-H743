#include "app_control_scheduler.h"

#include <stddef.h>
#include <string.h>

#define APP_CONTROL_SCHED_MIN_DT_US   250ULL
#define APP_CONTROL_SCHED_MAX_DT_US 100000ULL

static float scheduler_dt(uint64_t now_us,
                          uint64_t previous_us,
                          uint64_t nominal_us,
                          uint8_t *fault)
{
    uint64_t elapsed_us;

    if ((previous_us == 0ULL) || (now_us <= previous_us)) {
        *fault = 1U;
        elapsed_us = nominal_us;
    } else {
        elapsed_us = now_us - previous_us;
    }
    if (elapsed_us < APP_CONTROL_SCHED_MIN_DT_US) {
        *fault = 1U;
        elapsed_us = APP_CONTROL_SCHED_MIN_DT_US;
    } else if (elapsed_us > APP_CONTROL_SCHED_MAX_DT_US) {
        *fault = 1U;
        elapsed_us = APP_CONTROL_SCHED_MAX_DT_US;
    }
    return (float)elapsed_us * 1.0e-6f;
}

static uint8_t scheduler_due(uint64_t now_us,
                             uint64_t previous_us,
                             uint64_t period_us)
{
    if (previous_us == 0ULL) {
        return 1U;
    }
    if (now_us <= previous_us) {
        return 1U;
    }
    return ((now_us - previous_us) >= period_us) ? 1U : 0U;
}

void APP_ControlScheduler_Reset(APP_ControlSchedulerState *state)
{
    if (state != NULL) {
        memset(state, 0, sizeof(*state));
    }
}

void APP_ControlScheduler_Step(APP_ControlSchedulerState *state,
                               uint64_t now_us,
                               uint64_t navigation_sample_us,
                               uint8_t navigation_valid,
                               APP_ControlSchedule *schedule)
{
    uint8_t new_navigation_sample;

    if ((state == NULL) || (schedule == NULL)) {
        return;
    }
    memset(schedule, 0, sizeof(*schedule));

    if ((state->initialized != 0U) &&
        ((now_us == 0ULL) || (now_us <= state->last_now_us))) {
        schedule->timestamp_fault = 1U;
        return;
    }

    if ((navigation_valid != 0U) && (navigation_sample_us > now_us)) {
        schedule->timestamp_fault = 1U;
        navigation_valid = 0U;
    }
    new_navigation_sample =
        ((navigation_valid != 0U) && (navigation_sample_us != 0ULL) &&
         ((state->last_nav_sample_us == 0ULL) ||
          (navigation_sample_us > state->last_nav_sample_us))) ? 1U : 0U;
    schedule->navigation_sample_new = new_navigation_sample;

    schedule->rate_due = scheduler_due(now_us, state->last_rate_us,
                                        APP_CONTROL_SCHED_RATE_PERIOD_US);
    schedule->attitude_due = scheduler_due(
        now_us, state->last_attitude_us, APP_CONTROL_SCHED_ATTITUDE_PERIOD_US);
    schedule->velocity_due =
        ((new_navigation_sample != 0U) &&
         (scheduler_due(now_us, state->last_velocity_us,
                        APP_CONTROL_SCHED_VELOCITY_PERIOD_US) != 0U)) ? 1U : 0U;
    schedule->position_due =
        ((new_navigation_sample != 0U) &&
         (scheduler_due(now_us, state->last_position_us,
                        APP_CONTROL_SCHED_POSITION_PERIOD_US) != 0U)) ? 1U : 0U;

    if (schedule->rate_due != 0U) {
        schedule->rate_dt_s = scheduler_dt(
            now_us, state->last_rate_us, APP_CONTROL_SCHED_RATE_PERIOD_US,
            &schedule->timestamp_fault);
        state->last_rate_us = now_us;
    }
    if (schedule->attitude_due != 0U) {
        schedule->attitude_dt_s = scheduler_dt(
            now_us, state->last_attitude_us,
            APP_CONTROL_SCHED_ATTITUDE_PERIOD_US, &schedule->timestamp_fault);
        state->last_attitude_us = now_us;
    }
    if (schedule->velocity_due != 0U) {
        schedule->velocity_dt_s = scheduler_dt(
            now_us, state->last_velocity_us,
            APP_CONTROL_SCHED_VELOCITY_PERIOD_US, &schedule->timestamp_fault);
        state->last_velocity_us = now_us;
    }
    if (schedule->position_due != 0U) {
        schedule->position_dt_s = scheduler_dt(
            now_us, state->last_position_us,
            APP_CONTROL_SCHED_POSITION_PERIOD_US, &schedule->timestamp_fault);
        state->last_position_us = now_us;
    }

    if (new_navigation_sample != 0U) {
        state->last_nav_sample_us = navigation_sample_us;
    }
    state->last_now_us = now_us;
    state->initialized = 1U;
}

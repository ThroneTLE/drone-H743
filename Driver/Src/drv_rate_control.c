#include "drv_rate_control.h"

#include <float.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

/* Real timestamp deltas are required.  These bounds prevent a bad timestamp
 * from creating a derivative or integral impulse. */
#define DRV_RATE_CONTROL_DT_MIN_SEC 1.0e-4f
#define DRV_RATE_CONTROL_DT_MAX_SEC 0.5f

static uint8_t rate_is_finite(float value)
{
    return isfinite(value) ? 1U : 0U;
}

static float rate_clamp(float value, float lower, float upper)
{
    if (value < lower) {
        return lower;
    }
    if (value > upper) {
        return upper;
    }
    return value;
}

static uint8_t rate_valid_vector(const float values[DRV_RATE_CONTROL_AXIS_COUNT])
{
    for (uint32_t axis = 0U; axis < DRV_RATE_CONTROL_AXIS_COUNT; ++axis) {
        if (rate_is_finite(values[axis]) == 0U) {
            return 0U;
        }
    }
    return 1U;
}

static uint8_t rate_valid_params(const DRV_RateControl_Params *params)
{
    if (params == NULL) {
        return 0U;
    }
    for (uint32_t axis = 0U; axis < DRV_RATE_CONTROL_AXIS_COUNT; ++axis) {
        if ((rate_is_finite(params->kp[axis]) == 0U) || (params->kp[axis] < 0.0f) ||
            (rate_is_finite(params->ki[axis]) == 0U) || (params->ki[axis] < 0.0f) ||
            (rate_is_finite(params->kd[axis]) == 0U) || (params->kd[axis] < 0.0f) ||
            (rate_is_finite(params->integrator_limit[axis]) == 0U) ||
            (params->integrator_limit[axis] < 0.0f) ||
            (rate_is_finite(params->large_error_threshold[axis]) == 0U) ||
            (params->large_error_threshold[axis] < 0.0f) ||
            (rate_is_finite(params->large_error_scale[axis]) == 0U) ||
            (params->large_error_scale[axis] < 0.0f) ||
            (params->large_error_scale[axis] > 1.0f) ||
            (rate_is_finite(params->ff_gain[axis]) == 0U) ||
            (params->ff_gain[axis] < 0.0f)) {
            return 0U;
        }
    }
    return (rate_is_finite(params->alpha_lpf_cutoff_rad_s) != 0U) &&
           (params->alpha_lpf_cutoff_rad_s >= 0.0f);
}

static float rate_filter_alpha(float cutoff_rad_s, float dt_s)
{
    if (cutoff_rad_s <= 0.0f) {
        /* Zero is the explicit no-filter setting. */
        return 1.0f;
    }
    return rate_clamp(1.0f - expf(-cutoff_rad_s * dt_s), 0.0f, 1.0f);
}

static float rate_gyro_cross(const float inertia[DRV_RATE_CONTROL_AXIS_COUNT],
                             const float omega[DRV_RATE_CONTROL_AXIS_COUNT],
                             uint32_t axis)
{
    /* omega x (J omega), for diagonal J. */
    if (axis == 0U) {
        return (inertia[2U] - inertia[1U]) * omega[1U] * omega[2U];
    }
    if (axis == 1U) {
        return (inertia[0U] - inertia[2U]) * omega[2U] * omega[0U];
    }
    return (inertia[1U] - inertia[0U]) * omega[0U] * omega[1U];
}

static void rate_clear_output(DRV_RateControl_Output *output,
                              const DRV_RateControl_State *state)
{
    memset(output, 0, sizeof(*output));
    if (state != NULL) {
        for (uint32_t axis = 0U; axis < DRV_RATE_CONTROL_AXIS_COUNT; ++axis) {
            output->filtered_alpha[axis] = state->filtered_alpha[axis];
        }
    }
}

static void rate_output_terms(const DRV_RateControl_Params *params,
                              const DRV_RateControl_Input *input,
                              DRV_RateControl_Output *output,
                              uint32_t axis)
{
    const float error = input->omega_sp[axis] - input->omega[axis];
    const float p_term = params->kp[axis] * error;
    const float d_term = params->kd[axis] * output->filtered_alpha[axis];
    const float ff_term = params->ff_gain[axis] * input->inertia[axis] *
                          input->alpha_ff[axis];
    const float gyroscopic = rate_gyro_cross(input->inertia, input->omega, axis);

    output->error[axis] = error;
    output->p_term[axis] = p_term;
    output->d_term[axis] = d_term;
    output->ff_term[axis] = ff_term;
    output->moment_unsat[axis] = p_term + output->i_term[axis] - d_term +
                                 ff_term + gyroscopic;
    output->moment_cmd[axis] = output->moment_unsat[axis];
}

static void rate_apply_moment_limit(const DRV_RateControl_Input *input,
                                    DRV_RateControl_Output *output,
                                    uint32_t axis)
{
    float positive = input->saturation_positive[axis];
    float negative = input->saturation_negative[axis];

    /* Non-ordered bounds mean no allocator feedback. */
    if ((rate_is_finite(positive) == 0U) ||
        (rate_is_finite(negative) == 0U) || (positive <= negative)) {
        positive = FLT_MAX;
        negative = -FLT_MAX;
    }
    if (output->moment_cmd[axis] >= positive) {
        output->moment_cmd[axis] = positive;
        output->saturated_pos[axis] = 1U;
    } else if (output->moment_cmd[axis] <= negative) {
        output->moment_cmd[axis] = negative;
        output->saturated_neg[axis] = 1U;
    }
}

void DRV_RateControl_InitState(DRV_RateControl_State *state)
{
    if (state != NULL) {
        memset(state, 0, sizeof(*state));
    }
}

uint8_t DRV_RateControl_Step(const DRV_RateControl_Params *params,
                             DRV_RateControl_State *state,
                             const DRV_RateControl_Input *input,
                             DRV_RateControl_Output *output)
{
    uint8_t dt_valid;
    uint8_t measurement_valid;
    float dt_s;

    if ((params == NULL) || (state == NULL) || (input == NULL) ||
        (output == NULL)) {
        return 0U;
    }
    if (input->integrator_reset != 0U) {
        DRV_RateControl_InitState(state);
    }
    rate_clear_output(output, state);

    if ((rate_valid_params(params) == 0U) ||
        (rate_valid_vector(input->omega) == 0U) ||
        (rate_valid_vector(input->omega_sp) == 0U) ||
        (rate_valid_vector(input->alpha_ff) == 0U) ||
        (rate_valid_vector(input->inertia) == 0U) ||
        (rate_valid_vector(input->saturation_positive) == 0U) ||
        (rate_valid_vector(input->saturation_negative) == 0U) ||
        (rate_is_finite(input->dt_s) == 0U)) {
        return 0U;
    }
    for (uint32_t axis = 0U; axis < DRV_RATE_CONTROL_AXIS_COUNT; ++axis) {
        if (input->inertia[axis] <= 0.0f) {
            return 0U;
        }
    }

    dt_s = input->dt_s;
    dt_valid = (dt_s >= DRV_RATE_CONTROL_DT_MIN_SEC &&
                dt_s <= DRV_RATE_CONTROL_DT_MAX_SEC) ? 1U : 0U;
    measurement_valid = (input->measurement_valid != 0U) ? 1U : 0U;

    if ((dt_valid == 0U) && (state->initialized != 0U)) {
        /* A bad timestamp invalidates the finite difference itself.  Clear
         * the derivative before forming this output, rather than carrying a
         * stale damping impulse through a timing fault. */
        for (uint32_t axis = 0U; axis < DRV_RATE_CONTROL_AXIS_COUNT; ++axis) {
            state->last_omega[axis] = input->omega[axis];
            state->filtered_alpha[axis] = 0.0f;
        }
    }

    if ((state->initialized == 0U) && (measurement_valid != 0U) &&
        (dt_valid != 0U)) {
        /* There is no derivative sample on the first valid observation. */
        for (uint32_t axis = 0U; axis < DRV_RATE_CONTROL_AXIS_COUNT; ++axis) {
            state->last_omega[axis] = input->omega[axis];
            state->filtered_alpha[axis] = 0.0f;
        }
        state->initialized = 1U;
    }

    if ((measurement_valid != 0U) && (dt_valid != 0U) &&
        (state->initialized != 0U)) {
        for (uint32_t axis = 0U; axis < DRV_RATE_CONTROL_AXIS_COUNT; ++axis) {
            const float raw_alpha = (input->omega[axis] - state->last_omega[axis]) /
                                    dt_s;
            const float alpha = rate_filter_alpha(
                params->alpha_lpf_cutoff_rad_s, dt_s);
            state->filtered_alpha[axis] +=
                alpha * (raw_alpha - state->filtered_alpha[axis]);
            output->filtered_alpha[axis] = state->filtered_alpha[axis];
        }
    }

    for (uint32_t axis = 0U; axis < DRV_RATE_CONTROL_AXIS_COUNT; ++axis) {
        float i_candidate = state->integrator[axis];
        float i_delta = 0.0f;
        float positive = input->saturation_positive[axis];
        float negative = input->saturation_negative[axis];
        const uint8_t can_integrate = (measurement_valid != 0U) &&
                                      (dt_valid != 0U) &&
                                      (input->integrator_enable != 0U) &&
                                      (input->integrator_freeze == 0U) &&
                                      (input->integrator_reset == 0U);

        if ((positive <= negative) || (rate_is_finite(positive) == 0U) ||
            (rate_is_finite(negative) == 0U)) {
            positive = FLT_MAX;
            negative = -FLT_MAX;
        }

        if (can_integrate != 0U) {
            float scale = 1.0f;
            const float error = input->omega_sp[axis] - input->omega[axis];
            if ((params->large_error_threshold[axis] > 0.0f) &&
                (fabsf(error) > params->large_error_threshold[axis])) {
                scale = rate_clamp(params->large_error_scale[axis], 0.0f, 1.0f);
            }
            i_delta = params->ki[axis] * error * dt_s * scale;
            i_candidate = rate_clamp(state->integrator[axis] + i_delta,
                                     -params->integrator_limit[axis],
                                     params->integrator_limit[axis]);
            if (((input->saturation_positive_active[axis] != 0U) &&
                 (i_delta > 0.0f)) ||
                ((input->saturation_negative_active[axis] != 0U) &&
                 (i_delta < 0.0f))) {
                i_delta = 0.0f;
                i_candidate = state->integrator[axis];
            }
        }

        output->filtered_alpha[axis] = state->filtered_alpha[axis];
        output->i_term[axis] = i_candidate;
        rate_output_terms(params, input, output, axis);
        {
            const float candidate_moment = output->moment_cmd[axis];
            const float current_moment = candidate_moment -
                                         (i_candidate - state->integrator[axis]);
            const float error = input->omega_sp[axis] - input->omega[axis];
            const uint8_t pushes_positive = (i_delta > 0.0f) && (error > 0.0f);
            const uint8_t pushes_negative = (i_delta < 0.0f) && (error < 0.0f);
            if ((can_integrate != 0U) && (pushes_positive != 0U) &&
                (current_moment < positive) && (candidate_moment > positive)) {
                /* Enter the actuator boundary exactly, avoiding one-cycle
                 * integral overshoot when dt is relatively large. */
                output->i_term[axis] = i_candidate - (candidate_moment - positive);
            } else if ((can_integrate != 0U) && (pushes_negative != 0U) &&
                       (current_moment > negative) && (candidate_moment < negative)) {
                output->i_term[axis] = i_candidate - (candidate_moment - negative);
            } else if ((can_integrate != 0U) &&
                       (((current_moment >= positive) && (pushes_positive != 0U)) ||
                        ((current_moment <= negative) && (pushes_negative != 0U)))) {
                output->i_term[axis] = state->integrator[axis];
            }
        }
        state->integrator[axis] = output->i_term[axis];
        rate_output_terms(params, input, output, axis);
        rate_apply_moment_limit(input, output, axis);

        if ((measurement_valid != 0U) && (dt_valid != 0U)) {
            state->last_omega[axis] = input->omega[axis];
        } else if (state->initialized != 0U) {
            /* Rebase after a bad gap so recovery has no derivative impulse. */
            state->last_omega[axis] = input->omega[axis];
            state->filtered_alpha[axis] = 0.0f;
            output->filtered_alpha[axis] = 0.0f;
        }
    }
    return 1U;
}

uint8_t DRV_RateControl_Evaluate(const DRV_RateControl_Params *params,
                                 const DRV_RateControl_State *state,
                                 const DRV_RateControl_Input *input,
                                 DRV_RateControl_Output *output)
{
    if ((params == NULL) || (state == NULL) || (input == NULL) ||
        (output == NULL) || (rate_valid_params(params) == 0U) ||
        (rate_valid_vector(input->omega) == 0U) ||
        (rate_valid_vector(input->omega_sp) == 0U) ||
        (rate_valid_vector(input->alpha_ff) == 0U) ||
        (rate_valid_vector(input->inertia) == 0U)) {
        return 0U;
    }
    rate_clear_output(output, state);
    for (uint32_t axis = 0U; axis < DRV_RATE_CONTROL_AXIS_COUNT; ++axis) {
        if (input->inertia[axis] <= 0.0f) {
            return 0U;
        }
        output->filtered_alpha[axis] = state->filtered_alpha[axis];
        output->i_term[axis] = state->integrator[axis];
        rate_output_terms(params, input, output, axis);
        rate_apply_moment_limit(input, output, axis);
    }
    return 1U;
}

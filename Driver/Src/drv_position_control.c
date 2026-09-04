/* Copyright (c) 2026 drone-H743 contributors. */

#include "drv_position_control.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define DRV_POSITION_CONTROL_PI 3.14159265358979323846f
#define DRV_POSITION_CONTROL_EPSILON 1.0e-6f

static uint8_t drv_position_control_is_finite(float value)
{
    return isfinite(value) ? 1U : 0U;
}

static uint8_t drv_position_control_valid_nonnegative(float value)
{
    return (drv_position_control_is_finite(value) != 0U) && (value >= 0.0f);
}

static uint8_t drv_position_control_valid_params(
    const DRV_POSITION_CONTROL_Params *params)
{
    uint32_t axis;

    if (params == NULL) {
        return 0U;
    }

    for (axis = 0U; axis < DRV_POSITION_CONTROL_AXIS_COUNT; ++axis) {
        if ((drv_position_control_valid_nonnegative(params->pos_kp[axis]) == 0U) ||
            (drv_position_control_valid_nonnegative(params->vel_kp[axis]) == 0U) ||
            (drv_position_control_valid_nonnegative(params->vel_ki[axis]) == 0U) ||
            (drv_position_control_valid_nonnegative(params->vel_kd[axis]) == 0U) ||
            (drv_position_control_valid_nonnegative(
                params->vel_integrator_limit[axis]) == 0U)) {
            return 0U;
        }
    }

    return (drv_position_control_valid_nonnegative(params->xy_speed_limit_m_s) != 0U) &&
           (drv_position_control_valid_nonnegative(params->z_speed_limit_up_m_s) != 0U) &&
           (drv_position_control_valid_nonnegative(params->z_speed_limit_down_m_s) != 0U) &&
           (drv_position_control_valid_nonnegative(params->xy_accel_limit_m_s2) != 0U) &&
           (drv_position_control_valid_nonnegative(params->z_accel_limit_up_m_s2) != 0U) &&
           (drv_position_control_valid_nonnegative(params->z_accel_limit_down_m_s2) != 0U) &&
           (drv_position_control_valid_nonnegative(params->accel_lpf_cutoff_hz) != 0U);
}

static uint8_t drv_position_control_valid_vector(
    const float vector[DRV_POSITION_CONTROL_AXIS_COUNT])
{
    uint32_t axis;

    for (axis = 0U; axis < DRV_POSITION_CONTROL_AXIS_COUNT; ++axis) {
        if (drv_position_control_is_finite(vector[axis]) == 0U) {
            return 0U;
        }
    }
    return 1U;
}

static float drv_position_control_clamp(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static float drv_position_control_xy_norm(const float vector[DRV_POSITION_CONTROL_AXIS_COUNT])
{
    return sqrtf((vector[0U] * vector[0U]) + (vector[1U] * vector[1U]));
}

static void drv_position_control_clear_feedback(
    DRV_POSITION_CONTROL_SaturationFeedback *feedback)
{
    uint32_t axis;

    for (axis = 0U; axis < DRV_POSITION_CONTROL_AXIS_COUNT; ++axis) {
        feedback->pos_limit[axis] = 0U;
        feedback->neg_limit[axis] = 0U;
    }
    feedback->thrust_saturated = 0U;
    feedback->tilt_saturated = 0U;
    feedback->horizontal_scale = 1.0f;
}

static void drv_position_control_mark_axis_clipping(
    float before,
    float after,
    uint32_t axis,
    DRV_POSITION_CONTROL_SaturationFeedback *feedback)
{
    if (after < (before - DRV_POSITION_CONTROL_EPSILON)) {
        if (before > 0.0f) {
            feedback->pos_limit[axis] = 1U;
        }
    } else if (after > (before + DRV_POSITION_CONTROL_EPSILON)) {
        if (before < 0.0f) {
            feedback->neg_limit[axis] = 1U;
        }
    }
}

static void drv_position_control_limit_xy(
    float limit,
    const float input[DRV_POSITION_CONTROL_AXIS_COUNT],
    float output[DRV_POSITION_CONTROL_AXIS_COUNT],
    DRV_POSITION_CONTROL_SaturationFeedback *feedback)
{
    const float norm = drv_position_control_xy_norm(input);

    if ((limit <= 0.0f) || (norm <= limit)) {
        return;
    }

    {
        const float scale = limit / norm;
        output[0U] = input[0U] * scale;
        output[1U] = input[1U] * scale;
        feedback->tilt_saturated = 1U;
        feedback->horizontal_scale = scale;
        drv_position_control_mark_axis_clipping(input[0U], output[0U], 0U, feedback);
        drv_position_control_mark_axis_clipping(input[1U], output[1U], 1U, feedback);
    }
}

static void drv_position_control_limit_z(
    float up_limit,
    float down_limit,
    const float input[DRV_POSITION_CONTROL_AXIS_COUNT],
    float output[DRV_POSITION_CONTROL_AXIS_COUNT],
    DRV_POSITION_CONTROL_SaturationFeedback *feedback)
{
    /* In the legacy frame, numeric +Z is down: negative is upward. */
    if ((up_limit > 0.0f) && (output[2U] < -up_limit)) {
        output[2U] = -up_limit;
        feedback->thrust_saturated = 1U;
        feedback->neg_limit[2U] = 1U;
    } else if ((down_limit > 0.0f) && (output[2U] > down_limit)) {
        output[2U] = down_limit;
        feedback->thrust_saturated = 1U;
        feedback->pos_limit[2U] = 1U;
    }

    (void)input;
}

static void drv_position_control_limit_velocity(
    const DRV_POSITION_CONTROL_Params *params,
    const float input[DRV_POSITION_CONTROL_AXIS_COUNT],
    float output[DRV_POSITION_CONTROL_AXIS_COUNT],
    DRV_POSITION_CONTROL_SaturationFeedback *feedback)
{
    memcpy(output, input, sizeof(float) * DRV_POSITION_CONTROL_AXIS_COUNT);
    drv_position_control_clear_feedback(feedback);
    drv_position_control_limit_xy(params->xy_speed_limit_m_s, input, output, feedback);
    drv_position_control_limit_z(params->z_speed_limit_up_m_s,
                                 params->z_speed_limit_down_m_s,
                                 input, output, feedback);
}

static void drv_position_control_limit_acceleration(
    const DRV_POSITION_CONTROL_Params *params,
    const float input[DRV_POSITION_CONTROL_AXIS_COUNT],
    float output[DRV_POSITION_CONTROL_AXIS_COUNT],
    DRV_POSITION_CONTROL_SaturationFeedback *feedback)
{
    memcpy(output, input, sizeof(float) * DRV_POSITION_CONTROL_AXIS_COUNT);
    drv_position_control_clear_feedback(feedback);
    drv_position_control_limit_xy(params->xy_accel_limit_m_s2, input, output, feedback);
    drv_position_control_limit_z(params->z_accel_limit_up_m_s2,
                                 params->z_accel_limit_down_m_s2,
                                 input, output, feedback);
}

static float drv_position_control_dt(float requested_dt, uint32_t *flags)
{
    if ((drv_position_control_is_finite(requested_dt) == 0U) ||
        (requested_dt <= 0.0f)) {
        *flags |= DRV_POSITION_CONTROL_FLAG_DT_INVALID;
        return 0.0f;
    }

    if (requested_dt > DRV_POSITION_CONTROL_MAX_DT_SEC) {
        *flags |= DRV_POSITION_CONTROL_FLAG_DT_CLIPPED;
        return DRV_POSITION_CONTROL_MAX_DT_SEC;
    }

    return requested_dt;
}

static float drv_position_control_lpf_alpha(float cutoff_hz, float dt_sec)
{
    if ((cutoff_hz <= 0.0f) || (dt_sec <= 0.0f)) {
        return 1.0f;
    }

    return drv_position_control_clamp(
        1.0f - expf(-2.0f * DRV_POSITION_CONTROL_PI * cutoff_hz * dt_sec),
        0.0f, 1.0f);
}

static void drv_position_control_zero_position_output(
    DRV_POSITION_CONTROL_PositionOutput *output)
{
    memset(output, 0, sizeof(*output));
    output->sat.horizontal_scale = 1.0f;
}

static void drv_position_control_zero_velocity_output(
    DRV_POSITION_CONTROL_VelocityOutput *output)
{
    memset(output, 0, sizeof(*output));
    output->sat.horizontal_scale = 1.0f;
}

void DRV_POSITION_CONTROL_ResetState(DRV_POSITION_CONTROL_State *state)
{
    if (state != NULL) {
        memset(state, 0, sizeof(*state));
    }
}

void DRV_POSITION_CONTROL_PositionStep(
    const DRV_POSITION_CONTROL_Params *params,
    const DRV_POSITION_CONTROL_PositionInput *input,
    DRV_POSITION_CONTROL_PositionOutput *output)
{
    float raw_velocity[DRV_POSITION_CONTROL_AXIS_COUNT];
    uint32_t flags = 0U;
    uint32_t axis;

    if (output == NULL) {
        return;
    }
    drv_position_control_zero_position_output(output);

    if ((params == NULL) || (input == NULL) ||
        (drv_position_control_valid_params(params) == 0U)) {
        output->flags = DRV_POSITION_CONTROL_FLAG_INPUT_INVALID;
        return;
    }

    (void)drv_position_control_dt(input->dt_sec, &flags);
    if ((flags & DRV_POSITION_CONTROL_FLAG_DT_INVALID) != 0U) {
        output->flags = flags;
        return;
    }

    if (input->position_bypass != 0U) {
        if (drv_position_control_valid_vector(input->direct_velocity_m_s) == 0U) {
            output->flags = DRV_POSITION_CONTROL_FLAG_INPUT_INVALID;
            return;
        }
        memcpy(raw_velocity, input->direct_velocity_m_s, sizeof(raw_velocity));
    } else {
        if ((input->measurement_valid == 0U) ||
            (drv_position_control_valid_vector(input->position_sp_m) == 0U) ||
            (drv_position_control_valid_vector(input->position_meas_m) == 0U) ||
            (drv_position_control_valid_vector(input->velocity_ff_m_s) == 0U)) {
            output->flags = flags | DRV_POSITION_CONTROL_FLAG_MEAS_INVALID;
            return;
        }
        for (axis = 0U; axis < DRV_POSITION_CONTROL_AXIS_COUNT; ++axis) {
            raw_velocity[axis] = input->velocity_ff_m_s[axis] +
                params->pos_kp[axis] *
                (input->position_sp_m[axis] - input->position_meas_m[axis]);
        }
    }

    drv_position_control_limit_velocity(params, raw_velocity,
                                         output->velocity_sp_m_s, &output->sat);
    output->flags = flags;
}

void DRV_POSITION_CONTROL_VelocityStep(
    const DRV_POSITION_CONTROL_Params *params,
    DRV_POSITION_CONTROL_State *state,
    const DRV_POSITION_CONTROL_VelocityInput *input,
    DRV_POSITION_CONTROL_VelocityOutput *output)
{
    float dt_sec;
    float candidate_i[DRV_POSITION_CONTROL_AXIS_COUNT];
    float candidate_unsat[DRV_POSITION_CONTROL_AXIS_COUNT];
    float limited_candidate[DRV_POSITION_CONTROL_AXIS_COUNT];
    float final_unsat[DRV_POSITION_CONTROL_AXIS_COUNT];
    float final_sat[DRV_POSITION_CONTROL_AXIS_COUNT];
    DRV_POSITION_CONTROL_SaturationFeedback candidate_feedback;
    uint32_t flags = 0U;
    uint32_t axis;
    uint8_t can_integrate;

    if (output == NULL) {
        return;
    }
    drv_position_control_zero_velocity_output(output);

    if ((params == NULL) || (state == NULL) || (input == NULL) ||
        (drv_position_control_valid_params(params) == 0U) ||
        (drv_position_control_valid_vector(input->velocity_sp_m_s) == 0U) ||
        (drv_position_control_valid_vector(input->velocity_meas_m_s) == 0U) ||
        (drv_position_control_valid_vector(input->accel_ff_m_s2) == 0U) ||
        ((input->measurement_valid != 0U) &&
         (drv_position_control_valid_vector(input->measured_accel_m_s2) == 0U))) {
        output->flags = DRV_POSITION_CONTROL_FLAG_INPUT_INVALID;
        return;
    }

    dt_sec = drv_position_control_dt(input->dt_sec, &flags);

    if (input->integrator_reset != 0U) {
        DRV_POSITION_CONTROL_ResetState(state);
        flags |= DRV_POSITION_CONTROL_FLAG_I_RESET;
    }

    can_integrate = ((input->measurement_valid != 0U) &&
                     (input->integrator_enable != 0U) &&
                     (input->integrator_freeze == 0U) &&
                     (input->integrator_reset == 0U) &&
                     (dt_sec > 0.0f)) ? 1U : 0U;
    if (can_integrate == 0U) {
        flags |= DRV_POSITION_CONTROL_FLAG_I_FREEZE;
    }
    if (input->measurement_valid == 0U) {
        flags |= DRV_POSITION_CONTROL_FLAG_MEAS_INVALID;
    }

    /* Update the measurement LPF only with a valid measurement and dt. */
    if ((input->measurement_valid != 0U) && (dt_sec > 0.0f)) {
        const float alpha = drv_position_control_lpf_alpha(
            params->accel_lpf_cutoff_hz, dt_sec);
        for (axis = 0U; axis < DRV_POSITION_CONTROL_AXIS_COUNT; ++axis) {
            if ((state->accel_lpf_initialized == 0U) || (alpha >= 1.0f)) {
                state->accel_lpf_m_s2[axis] = input->measured_accel_m_s2[axis];
            } else {
                state->accel_lpf_m_s2[axis] += alpha *
                    (input->measured_accel_m_s2[axis] - state->accel_lpf_m_s2[axis]);
            }
        }
        state->accel_lpf_initialized = 1U;
    }

    for (axis = 0U; axis < DRV_POSITION_CONTROL_AXIS_COUNT; ++axis) {
        output->error_m_s[axis] = input->velocity_sp_m_s[axis] -
                                  input->velocity_meas_m_s[axis];
        output->p_term_m_s2[axis] = params->vel_kp[axis] * output->error_m_s[axis];
        output->i_term_m_s2[axis] = state->velocity_integrator_m_s2[axis];
        output->d_term_m_s2[axis] = -params->vel_kd[axis] *
                                    state->accel_lpf_m_s2[axis];
        output->ff_term_m_s2[axis] = input->accel_ff_m_s2[axis];

        candidate_i[axis] = output->i_term_m_s2[axis];
        if (can_integrate != 0U) {
            candidate_i[axis] += params->vel_ki[axis] *
                output->error_m_s[axis] * dt_sec;
            candidate_i[axis] = drv_position_control_clamp(
                candidate_i[axis], -params->vel_integrator_limit[axis],
                params->vel_integrator_limit[axis]);
        }

        candidate_unsat[axis] = output->ff_term_m_s2[axis] +
                                output->p_term_m_s2[axis] +
                                candidate_i[axis] + output->d_term_m_s2[axis];
    }

    drv_position_control_limit_acceleration(params, candidate_unsat,
                                            limited_candidate,
                                            &candidate_feedback);

    for (axis = 0U; axis < DRV_POSITION_CONTROL_AXIS_COUNT; ++axis) {
        const float delta_i = candidate_i[axis] -
                              state->velocity_integrator_m_s2[axis];
        const uint8_t downstream_positive =
            (input->downstream_saturation.pos_limit[axis] != 0U) ||
            (((axis < 2U) &&
              ((input->downstream_saturation.tilt_saturated != 0U) ||
               ((input->downstream_saturation.horizontal_scale > 0.0f) &&
                (input->downstream_saturation.horizontal_scale < 0.999f)))) &&
             (delta_i > 0.0f)) ||
            (((axis == 2U) &&
              (input->downstream_saturation.thrust_saturated != 0U)) &&
             (delta_i > 0.0f));
        const uint8_t downstream_negative =
            (input->downstream_saturation.neg_limit[axis] != 0U) ||
            (((axis < 2U) &&
              ((input->downstream_saturation.tilt_saturated != 0U) ||
               ((input->downstream_saturation.horizontal_scale > 0.0f) &&
                (input->downstream_saturation.horizontal_scale < 0.999f)))) &&
             (delta_i < 0.0f)) ||
            (((axis == 2U) &&
              (input->downstream_saturation.thrust_saturated != 0U)) &&
             (delta_i < 0.0f));
        const uint8_t driving_deeper =
            (((candidate_feedback.pos_limit[axis] != 0U) ||
              (downstream_positive != 0U)) && (delta_i > 0.0f)) ||
            (((candidate_feedback.neg_limit[axis] != 0U) ||
              (downstream_negative != 0U)) && (delta_i < 0.0f));

        if ((can_integrate != 0U) && (driving_deeper == 0U)) {
            state->velocity_integrator_m_s2[axis] = candidate_i[axis];
        } else if ((can_integrate != 0U) && (driving_deeper != 0U)) {
            flags |= DRV_POSITION_CONTROL_FLAG_I_FREEZE;
        }

        output->i_term_m_s2[axis] = state->velocity_integrator_m_s2[axis];
        final_unsat[axis] = output->ff_term_m_s2[axis] +
                            output->p_term_m_s2[axis] +
                            output->i_term_m_s2[axis] + output->d_term_m_s2[axis];
    }

    drv_position_control_limit_acceleration(params, final_unsat, final_sat,
                                            &output->sat);
    for (axis = 0U; axis < DRV_POSITION_CONTROL_AXIS_COUNT; ++axis) {
        output->accel_unsat_m_s2[axis] = final_unsat[axis];
        output->accel_sat_m_s2[axis] = final_sat[axis];
    }
    output->flags = flags;
}

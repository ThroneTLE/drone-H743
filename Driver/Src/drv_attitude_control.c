#include "drv_attitude_control.h"

#include <math.h>

#include <stddef.h>

static uint8_t drv_attitude_control_is_finite(float value)
{
    return isfinite(value) ? 1U : 0U;
}

static uint8_t drv_attitude_control_valid_matrix(const float matrix[DRV_ATTITUDE_CONTROL_AXIS_COUNT][DRV_ATTITUDE_CONTROL_AXIS_COUNT])
{
    float determinant;

    for (uint32_t row = 0U; row < DRV_ATTITUDE_CONTROL_AXIS_COUNT; ++row) {
        for (uint32_t col = 0U; col < DRV_ATTITUDE_CONTROL_AXIS_COUNT; ++col) {
            if (drv_attitude_control_is_finite(matrix[row][col]) == 0U) {
                return 0U;
            }
        }
    }

    for (uint32_t row = 0U; row < DRV_ATTITUDE_CONTROL_AXIS_COUNT; ++row) {
        for (uint32_t other = row; other < DRV_ATTITUDE_CONTROL_AXIS_COUNT; ++other) {
            float dot = 0.0f;
            for (uint32_t col = 0U; col < DRV_ATTITUDE_CONTROL_AXIS_COUNT; ++col) {
                dot += matrix[row][col] * matrix[other][col];
            }
            if (fabsf(dot - ((row == other) ? 1.0f : 0.0f)) > 1.0e-3f) {
                return 0U;
            }
        }
    }
    determinant =
        matrix[0U][0U] * ((matrix[1U][1U] * matrix[2U][2U]) -
                           (matrix[1U][2U] * matrix[2U][1U])) -
        matrix[0U][1U] * ((matrix[1U][0U] * matrix[2U][2U]) -
                           (matrix[1U][2U] * matrix[2U][0U])) +
        matrix[0U][2U] * ((matrix[1U][0U] * matrix[2U][1U]) -
                           (matrix[1U][1U] * matrix[2U][0U]));
    if (fabsf(determinant - 1.0f) > 1.0e-3f) {
        return 0U;
    }

    return 1U;
}

static uint8_t drv_attitude_control_valid_vector(const float value[DRV_ATTITUDE_CONTROL_AXIS_COUNT])
{
    for (uint32_t axis = 0U; axis < DRV_ATTITUDE_CONTROL_AXIS_COUNT; ++axis) {
        if (drv_attitude_control_is_finite(value[axis]) == 0U) {
            return 0U;
        }
    }

    return 1U;
}

static uint8_t drv_attitude_control_valid_params(const DRV_AttitudeControl_Params *params)
{
    for (uint32_t axis = 0U; axis < DRV_ATTITUDE_CONTROL_AXIS_COUNT; ++axis) {
        if ((drv_attitude_control_is_finite(params->att_kp[axis]) == 0U) ||
            (params->att_kp[axis] < 0.0f)) {
            return 0U;
        }
        if (drv_attitude_control_is_finite(params->rate_limit_rad_s[axis]) == 0U ||
            (params->rate_limit_rad_s[axis] <= 0.0f)) {
            return 0U;
        }
    }

    return 1U;
}

static void drv_attitude_control_transpose3(const float input[DRV_ATTITUDE_CONTROL_AXIS_COUNT][DRV_ATTITUDE_CONTROL_AXIS_COUNT],
                                            float output[DRV_ATTITUDE_CONTROL_AXIS_COUNT][DRV_ATTITUDE_CONTROL_AXIS_COUNT])
{
    for (uint32_t row = 0U; row < DRV_ATTITUDE_CONTROL_AXIS_COUNT; ++row) {
        for (uint32_t col = 0U; col < DRV_ATTITUDE_CONTROL_AXIS_COUNT; ++col) {
            output[row][col] = input[col][row];
        }
    }
}

static void drv_attitude_control_mul3(const float left[DRV_ATTITUDE_CONTROL_AXIS_COUNT][DRV_ATTITUDE_CONTROL_AXIS_COUNT],
                                     const float right[DRV_ATTITUDE_CONTROL_AXIS_COUNT][DRV_ATTITUDE_CONTROL_AXIS_COUNT],
                                     float output[DRV_ATTITUDE_CONTROL_AXIS_COUNT][DRV_ATTITUDE_CONTROL_AXIS_COUNT])
{
    float product[DRV_ATTITUDE_CONTROL_AXIS_COUNT][DRV_ATTITUDE_CONTROL_AXIS_COUNT];

    for (uint32_t row = 0U; row < DRV_ATTITUDE_CONTROL_AXIS_COUNT; ++row) {
        for (uint32_t col = 0U; col < DRV_ATTITUDE_CONTROL_AXIS_COUNT; ++col) {
            product[row][col] =
                (left[row][0U] * right[0U][col]) +
                (left[row][1U] * right[1U][col]) +
                (left[row][2U] * right[2U][col]);
        }
    }

    for (uint32_t row = 0U; row < DRV_ATTITUDE_CONTROL_AXIS_COUNT; ++row) {
        for (uint32_t col = 0U; col < DRV_ATTITUDE_CONTROL_AXIS_COUNT; ++col) {
            output[row][col] = product[row][col];
        }
    }
}

static void drv_attitude_control_mul3_vec(const float left[DRV_ATTITUDE_CONTROL_AXIS_COUNT][DRV_ATTITUDE_CONTROL_AXIS_COUNT],
                                         const float right[DRV_ATTITUDE_CONTROL_AXIS_COUNT],
                                         float output[DRV_ATTITUDE_CONTROL_AXIS_COUNT])
{
    for (uint32_t row = 0U; row < DRV_ATTITUDE_CONTROL_AXIS_COUNT; ++row) {
        output[row] = (left[row][0U] * right[0U]) +
                      (left[row][1U] * right[1U]) +
                      (left[row][2U] * right[2U]);
    }
}

static float drv_attitude_control_clamp_f32(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

uint8_t DRV_AttitudeControl_Step(const DRV_AttitudeControl_Params *params,
                                 const DRV_AttitudeControl_Input *input,
                                 DRV_AttitudeControl_Output *output)
{
    float actual_t[DRV_ATTITUDE_CONTROL_AXIS_COUNT][DRV_ATTITUDE_CONTROL_AXIS_COUNT];
    float desired_t[DRV_ATTITUDE_CONTROL_AXIS_COUNT][DRV_ATTITUDE_CONTROL_AXIS_COUNT];
    float desired_t_actual[DRV_ATTITUDE_CONTROL_AXIS_COUNT][DRV_ATTITUDE_CONTROL_AXIS_COUNT];
    float actual_t_desired[DRV_ATTITUDE_CONTROL_AXIS_COUNT][DRV_ATTITUDE_CONTROL_AXIS_COUNT];
    float omega_tmp[DRV_ATTITUDE_CONTROL_AXIS_COUNT];

    if ((params == NULL) || (input == NULL) || (output == NULL)) {
        return 0U;
    }

    if (drv_attitude_control_valid_params(params) == 0U ||
        (drv_attitude_control_valid_matrix(input->actual_rotation) == 0U) ||
        (drv_attitude_control_valid_matrix(input->desired_rotation) == 0U) ||
        (drv_attitude_control_valid_vector(input->desired_rate_in_desired_frame) == 0U)) {
        for (uint32_t axis = 0U; axis < DRV_ATTITUDE_CONTROL_AXIS_COUNT; ++axis) {
            output->attitude_error[axis] = 0.0f;
            output->omega_ff[axis] = 0.0f;
            output->omega_sp[axis] = 0.0f;
            output->rate_saturated[axis] = 0U;
        }
        return 0U;
    }

    drv_attitude_control_transpose3(input->actual_rotation, actual_t);
    drv_attitude_control_transpose3(input->desired_rotation, desired_t);

    drv_attitude_control_mul3(desired_t, input->actual_rotation, desired_t_actual);
    drv_attitude_control_mul3(actual_t, input->desired_rotation, actual_t_desired);

    /*
     * e_R = 0.5 * (R_d^T R - R^T R_d)^
     * vee
     */
    output->attitude_error[0U] =
        0.5f * (desired_t_actual[2U][1U] - actual_t_desired[2U][1U]);
    output->attitude_error[1U] =
        0.5f * (desired_t_actual[0U][2U] - actual_t_desired[0U][2U]);
    output->attitude_error[2U] =
        0.5f * (desired_t_actual[1U][0U] - actual_t_desired[1U][0U]);

    /*
     * omega_ff = R^T R_d omega_d, where omega_d is given in desired body frame.
     */
    drv_attitude_control_mul3_vec(input->desired_rotation,
                                 input->desired_rate_in_desired_frame,
                                 omega_tmp);
    drv_attitude_control_mul3_vec(actual_t, omega_tmp, output->omega_ff);

    for (uint32_t axis = 0U; axis < DRV_ATTITUDE_CONTROL_AXIS_COUNT; ++axis) {
        /* e_R is actual-minus-desired, therefore this is negative feedback. */
        const float candidate = output->omega_ff[axis] -
                               (params->att_kp[axis] * output->attitude_error[axis]);

        if (fabsf(candidate) > params->rate_limit_rad_s[axis]) {
            output->rate_saturated[axis] = 1U;
            output->omega_sp[axis] =
                drv_attitude_control_clamp_f32(candidate,
                                              -params->rate_limit_rad_s[axis],
                                              params->rate_limit_rad_s[axis]);
        } else {
            output->rate_saturated[axis] = 0U;
            output->omega_sp[axis] = candidate;
        }
    }

    return 1U;
}

#include "drv_coax_ctrl.h"

#include "bsp_pwm.h"
#include "drv_airframe_model.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define DRV_COAX_CTRL_TILT_LIMIT_RAD 0.314159f
#define DRV_COAX_CTRL_PI 3.141592654f
#define DRV_COAX_CTRL_SERVO_TRAVEL_RAD \
    (DRV_COAX_CTRL_SERVO_TRAVEL_DEG * DRV_COAX_CTRL_PI / 180.0f)
#define DRV_COAX_CTRL_SERVO_LIMIT_RAD \
    (DRV_COAX_CTRL_SERVO_LIMIT_DEG * DRV_COAX_CTRL_PI / 180.0f)
#define DRV_COAX_CTRL_SERVO_ALPHA_SIGN    (1.0f)
#define DRV_COAX_CTRL_SERVO_BETA_SIGN     (1.0f)
#define DRV_COAX_CTRL_FORCE_FRAME_ROLL_SIGN  (-1.0f)
#define DRV_COAX_CTRL_FORCE_FRAME_PITCH_SIGN (1.0f)
#define DRV_COAX_CTRL_FORCE_EPS_N          1.0e-4f
#define DRV_COAX_CTRL_RATE_SCALE_EPS       1.0e-6f
#define DRV_COAX_CTRL_PROP9047_YAW_M_PER_N 0.0001f
#define DRV_COAX_CTRL_SINGLE_MAX_THRUST_N 10.2f
#define DRV_COAX_CTRL_THRUST_TABLE_POINTS  21U
#define DRV_COAX_CTRL_GRAMS_PER_NEWTON     101.971621f
#define DRV_COAX_CTRL_BALANCE_ITERATIONS   2U
#define DRV_COAX_CTRL_ROLL_EFFECTIVENESS   0.581f
#define DRV_COAX_CTRL_PITCH_EFFECTIVENESS  0.569f
#define DRV_COAX_CTRL_ROLL_MOMENT_SIGN     (1.0f)
#define DRV_COAX_CTRL_PITCH_MOMENT_SIGN    (1.0f)
#define DRV_COAX_CTRL_VEL_INTEGRAL_LIMIT_M 4.0f
#define DRV_COAX_CTRL_HORIZONTAL_ACCEL_LIMIT_M_S2 2.0f
#define DRV_COAX_CTRL_ATTITUDE_PROTECT_START_RAD 0.209440f
#define DRV_COAX_CTRL_ATTITUDE_PROTECT_END_RAD   0.436332f
#define DRV_COAX_CTRL_MOMENT_PROTECT_START       0.75f
#define DRV_COAX_CTRL_MOMENT_PROTECT_END         1.00f
#define DRV_COAX_CTRL_THRUST_PROTECT_START       0.92f
#define DRV_COAX_CTRL_THRUST_PROTECT_END         1.00f

typedef struct {
    const char *name;
    uint16_t offset;
} DRV_COAX_CTRL_ParamEntry;

typedef struct {
    float velocity_integral_m[2];
} DRV_COAX_CTRL_State;

typedef struct {
    float desired_force_local_n[3];
    float desired_force_body_n[3];
    float thrust_frame_r[3][3];
    float desired_body_r[3][3];
    float attitude_error[3];
    float attitude_error_angle_rad;
    float rate_error_rad_s[3];
    float moment_cmd_n_m[3];
    float alpha_rad;
    float beta_rad;
    float total_force_n;
    float raw_total_force_n;
    float moment_utilization;
    float thrust_utilization;
} DRV_COAX_CTRL_BalanceSolution;

static uint8_t coax_ctrl_initialized;
static DRV_COAX_CTRL_Params coax_ctrl_params;
static DRV_COAX_CTRL_Debug coax_ctrl_last_debug;
static DRV_COAX_CTRL_State coax_ctrl_state;

#define DRV_COAX_CTRL_PARAM_ENTRY(field) \
    { "coax." #field, (uint16_t)offsetof(DRV_COAX_CTRL_Params, field) }

static const DRV_COAX_CTRL_ParamEntry coax_ctrl_param_table[] = {
    DRV_COAX_CTRL_PARAM_ENTRY(pos_x_kp),
    DRV_COAX_CTRL_PARAM_ENTRY(pos_y_kp),
    DRV_COAX_CTRL_PARAM_ENTRY(pos_z_kp),
    DRV_COAX_CTRL_PARAM_ENTRY(vel_x_kd),
    DRV_COAX_CTRL_PARAM_ENTRY(vel_y_kd),
    DRV_COAX_CTRL_PARAM_ENTRY(vel_z_kd),
    DRV_COAX_CTRL_PARAM_ENTRY(vel_loop_enable),
    DRV_COAX_CTRL_PARAM_ENTRY(vel_loop_x_kp),
    DRV_COAX_CTRL_PARAM_ENTRY(vel_loop_x_ki),
    DRV_COAX_CTRL_PARAM_ENTRY(vel_loop_x_kd),
    DRV_COAX_CTRL_PARAM_ENTRY(vel_loop_y_kp),
    DRV_COAX_CTRL_PARAM_ENTRY(vel_loop_y_ki),
    DRV_COAX_CTRL_PARAM_ENTRY(vel_loop_y_kd),
    DRV_COAX_CTRL_PARAM_ENTRY(roll_angle_kp),
    DRV_COAX_CTRL_PARAM_ENTRY(pitch_angle_kp),
    DRV_COAX_CTRL_PARAM_ENTRY(roll_rate_kd),
    DRV_COAX_CTRL_PARAM_ENTRY(pitch_rate_kd),
    DRV_COAX_CTRL_PARAM_ENTRY(tilt_limit_rad),
    DRV_COAX_CTRL_PARAM_ENTRY(yaw_angle_kp),
    DRV_COAX_CTRL_PARAM_ENTRY(yaw_rate_kd),
};

static const uint32_t coax_ctrl_param_count =
    sizeof(coax_ctrl_param_table) / sizeof(coax_ctrl_param_table[0]);

static const uint16_t coax_ctrl_dual_pwm_us[DRV_COAX_CTRL_THRUST_TABLE_POINTS] = {
    1100U, 1142U, 1184U, 1226U, 1268U, 1310U, 1352U, 1394U,
    1436U, 1478U, 1520U, 1562U, 1604U, 1646U, 1688U, 1730U,
    1772U, 1814U, 1856U, 1898U, 1940U,
};

static const float coax_ctrl_dual_thrust_g[DRV_COAX_CTRL_THRUST_TABLE_POINTS] = {
    0.000f, 5.069f, 25.589f, 60.655f, 106.361f, 165.084f,
    216.696f, 287.758f, 386.724f, 501.680f, 624.697f, 725.173f,
    828.680f, 923.574f, 981.674f, 1114.845f, 1256.137f, 1366.352f,
    1466.668f, 1541.404f, 1595.342f,
};

static float coax_ctrl_clamp_f32(float value, float lo, float hi)
{
    if (value < lo) { return lo; }
    if (value > hi) { return hi; }
    return value;
}

static uint16_t coax_ctrl_clamp_u16(int32_t value, uint16_t lo, uint16_t hi)
{
    if (value < (int32_t)lo) { return lo; }
    if (value > (int32_t)hi) { return hi; }
    return (uint16_t)value;
}

static float coax_ctrl_wrap_pi(float angle_rad)
{
    while (angle_rad > DRV_COAX_CTRL_PI) {
        angle_rad -= 2.0f * DRV_COAX_CTRL_PI;
    }
    while (angle_rad < -DRV_COAX_CTRL_PI) {
        angle_rad += 2.0f * DRV_COAX_CTRL_PI;
    }
    return angle_rad;
}

/*
 * Controller inputs use the existing local frame: X forward, Y right, Z down
 * with altitude represented as z = -height. Keep the application, RC, and gain
 * polarities intact; only this force-frame projection adapts the measured roll
 * convention so vertical thrust maps to the physical roll servo direction.
 */
static void coax_ctrl_local_down_to_body(const DRV_COAX_CTRL_AttitudeInput *attitude,
                                         const float local_down[3],
                                         float body[3])
{
    const float phi =
        DRV_COAX_CTRL_FORCE_FRAME_ROLL_SIGN * attitude->roll_rad;
    const float theta =
        DRV_COAX_CTRL_FORCE_FRAME_PITCH_SIGN * attitude->pitch_rad;
    const float psi = attitude->yaw_rad;
    const float cphi = cosf(phi);
    const float sphi = sinf(phi);
    const float ctheta = cosf(theta);
    const float stheta = sinf(theta);
    const float cpsi = cosf(psi);
    const float spsi = sinf(psi);

    body[0] = (ctheta * cpsi * local_down[0]) +
              (ctheta * spsi * local_down[1]) -
              (stheta * local_down[2]);
    body[1] = ((sphi * stheta * cpsi - cphi * spsi) * local_down[0]) +
              ((sphi * stheta * spsi + cphi * cpsi) * local_down[1]) +
              (sphi * ctheta * local_down[2]);
    body[2] = ((cphi * stheta * cpsi + sphi * spsi) * local_down[0]) +
              ((cphi * stheta * spsi - sphi * cpsi) * local_down[1]) +
              (cphi * ctheta * local_down[2]);
}

static void coax_ctrl_cross3(const float a[3], const float b[3], float out[3])
{
    out[0] = (a[1] * b[2]) - (a[2] * b[1]);
    out[1] = (a[2] * b[0]) - (a[0] * b[2]);
    out[2] = (a[0] * b[1]) - (a[1] * b[0]);
}

static float coax_ctrl_norm3(const float value[3])
{
    return sqrtf((value[0] * value[0]) +
                 (value[1] * value[1]) +
                 (value[2] * value[2]));
}

static void coax_ctrl_normalize3(float value[3])
{
    const float norm = coax_ctrl_norm3(value);

    if (norm > DRV_COAX_CTRL_RATE_SCALE_EPS) {
        value[0] /= norm;
        value[1] /= norm;
        value[2] /= norm;
    }
}

static void coax_ctrl_matrix_transpose(const float input[3][3],
                                       float output[3][3])
{
    for (uint32_t row = 0U; row < 3U; ++row) {
        for (uint32_t col = 0U; col < 3U; ++col) {
            output[row][col] = input[col][row];
        }
    }
}

static void coax_ctrl_matrix_multiply(const float left[3][3],
                                      const float right[3][3],
                                      float output[3][3])
{
    float product[3][3];

    for (uint32_t row = 0U; row < 3U; ++row) {
        for (uint32_t col = 0U; col < 3U; ++col) {
            product[row][col] = 0.0f;
            for (uint32_t index = 0U; index < 3U; ++index) {
                product[row][col] += left[row][index] * right[index][col];
            }
        }
    }
    memcpy(output, product, sizeof(product));
}

static void coax_ctrl_attitude_matrix(
    const DRV_COAX_CTRL_AttitudeInput *attitude,
    float rotation[3][3])
{
    const float phi =
        DRV_COAX_CTRL_FORCE_FRAME_ROLL_SIGN * attitude->roll_rad;
    const float theta =
        DRV_COAX_CTRL_FORCE_FRAME_PITCH_SIGN * attitude->pitch_rad;
    const float psi = attitude->yaw_rad;
    const float cphi = cosf(phi);
    const float sphi = sinf(phi);
    const float ctheta = cosf(theta);
    const float stheta = sinf(theta);
    const float cpsi = cosf(psi);
    const float spsi = sinf(psi);

    rotation[0][0] = ctheta * cpsi;
    rotation[0][1] = (sphi * stheta * cpsi) - (cphi * spsi);
    rotation[0][2] = (cphi * stheta * cpsi) + (sphi * spsi);
    rotation[1][0] = ctheta * spsi;
    rotation[1][1] = (sphi * stheta * spsi) + (cphi * cpsi);
    rotation[1][2] = (cphi * stheta * spsi) - (sphi * cpsi);
    rotation[2][0] = -stheta;
    rotation[2][1] = sphi * ctheta;
    rotation[2][2] = cphi * ctheta;
}

static void coax_ctrl_gimbal_matrix(float alpha_rad,
                                    float beta_rad,
                                    float rotation[3][3])
{
    const float ca = cosf(alpha_rad);
    const float sa = sinf(alpha_rad);
    const float cb = cosf(beta_rad);
    const float sb = sinf(beta_rad);

    /* Rg = Ry(alpha) * Rx(beta). Its third column is thrust direction. */
    rotation[0][0] = ca;
    rotation[0][1] = sa * sb;
    rotation[0][2] = sa * cb;
    rotation[1][0] = 0.0f;
    rotation[1][1] = cb;
    rotation[1][2] = -sb;
    rotation[2][0] = -sa;
    rotation[2][1] = ca * sb;
    rotation[2][2] = ca * cb;
}

static void coax_ctrl_build_thrust_frame(const float force_local_n[3],
                                         float yaw_rad,
                                         float rotation[3][3])
{
    float b3[3] = {
        force_local_n[0],
        force_local_n[1],
        force_local_n[2],
    };
    const float heading[3] = { cosf(yaw_rad), sinf(yaw_rad), 0.0f };
    float b1[3];
    float b2[3];

    coax_ctrl_normalize3(b3);
    coax_ctrl_cross3(b3, heading, b2);
    if (coax_ctrl_norm3(b2) <= DRV_COAX_CTRL_RATE_SCALE_EPS) {
        const float fallback[3] = { -sinf(yaw_rad), cosf(yaw_rad), 0.0f };
        b2[0] = fallback[0];
        b2[1] = fallback[1];
        b2[2] = fallback[2];
    }
    coax_ctrl_normalize3(b2);
    coax_ctrl_cross3(b2, b3, b1);
    coax_ctrl_normalize3(b1);

    for (uint32_t row = 0U; row < 3U; ++row) {
        rotation[row][0] = b1[row];
        rotation[row][1] = b2[row];
        rotation[row][2] = b3[row];
    }
}

static void coax_ctrl_attitude_error(const float desired[3][3],
                                     const float actual[3][3],
                                     float error[3],
                                     float *error_angle_rad)
{
    float desired_t[3][3];
    float actual_t[3][3];
    float desired_t_actual[3][3];
    float actual_t_desired[3][3];

    coax_ctrl_matrix_transpose(desired, desired_t);
    coax_ctrl_matrix_transpose(actual, actual_t);
    coax_ctrl_matrix_multiply(desired_t, actual, desired_t_actual);
    coax_ctrl_matrix_multiply(actual_t, desired, actual_t_desired);

    error[0] = 0.5f * (desired_t_actual[2][1] - actual_t_desired[2][1]);
    error[1] = 0.5f * (desired_t_actual[0][2] - actual_t_desired[0][2]);
    error[2] = 0.5f * (desired_t_actual[1][0] - actual_t_desired[1][0]);
    if (error_angle_rad != NULL) {
        const float cos_angle = 0.5f *
            (desired_t_actual[0][0] + desired_t_actual[1][1] +
             desired_t_actual[2][2] - 1.0f);
        *error_angle_rad = acosf(coax_ctrl_clamp_f32(cos_angle, -1.0f, 1.0f));
    }
}

static void coax_ctrl_rotation_to_rpy(const float rotation[3][3],
                                      float rpy_rad[3])
{
    const float sin_pitch = coax_ctrl_clamp_f32(-rotation[2][0], -1.0f, 1.0f);

    rpy_rad[0] = atan2f(rotation[2][1], rotation[2][2]);
    rpy_rad[1] = asinf(sin_pitch);
    rpy_rad[2] = atan2f(rotation[1][0], rotation[0][0]);
}

static float coax_ctrl_protection_scale(float value, float start, float end)
{
    if (value <= start) {
        return 1.0f;
    }
    if (value >= end) {
        return 0.0f;
    }
    return (end - value) / (end - start);
}

static float *coax_ctrl_param_ptr(DRV_COAX_CTRL_Params *params,
                                  const DRV_COAX_CTRL_ParamEntry *entry)
{
    return (float *)((uint8_t *)params + entry->offset);
}

static const DRV_COAX_CTRL_ParamEntry *coax_ctrl_find_param(const char *name)
{
    if (name == NULL) {
        return NULL;
    }

    for (uint32_t i = 0U; i < coax_ctrl_param_count; ++i) {
        if (strcmp(name, coax_ctrl_param_table[i].name) == 0) {
            return &coax_ctrl_param_table[i];
        }
    }

    return NULL;
}

static uint8_t coax_ctrl_param_value_valid(const DRV_COAX_CTRL_ParamEntry *entry,
                                           float value)
{
    if ((entry == NULL) || !isfinite(value)) {
        return 0U;
    }

    if (fabsf(value) > 2000.0f) {
        return 0U;
    }

    if (entry->offset == offsetof(DRV_COAX_CTRL_Params, tilt_limit_rad)) {
        return ((value > 0.0f) &&
                (value <= DRV_COAX_CTRL_TILT_LIMIT_RAD)) ? 1U : 0U;
    }

    if (entry->offset == offsetof(DRV_COAX_CTRL_Params, vel_loop_enable)) {
        return (value >= 0.0f) ? 1U : 0U;
    }

    return 1U;
}

static uint8_t coax_ctrl_params_valid(const DRV_COAX_CTRL_Params *params)
{
    if (params == NULL) {
        return 0U;
    }

    for (uint32_t i = 0U; i < coax_ctrl_param_count; ++i) {
        if (coax_ctrl_param_value_valid(&coax_ctrl_param_table[i],
                                        *coax_ctrl_param_ptr((DRV_COAX_CTRL_Params *)params,
                                                             &coax_ctrl_param_table[i])) == 0U) {
            return 0U;
        }
    }

    return 1U;
}

static void coax_ctrl_apply_fixed_model_params(DRV_COAX_CTRL_Params *params)
{
    if (params == NULL) {
        return;
    }

    params->mass_kg = DRV_AIRFRAME_MASS_KG;
    params->gravity_m_s2 = DRV_AIRFRAME_GRAVITY_M_S2;
    params->pitch_tilt_lever_arm_m = DRV_AIRFRAME_PITCH_THRUST_LEVER_ARM_M;
    params->roll_tilt_lever_arm_m = DRV_AIRFRAME_ROLL_THRUST_LEVER_ARM_M;
    params->yaw_inertia = DRV_AIRFRAME_IZZ_KGM2;
    params->motor_single_max_thrust_n = DRV_COAX_CTRL_SINGLE_MAX_THRUST_N;
    params->yaw_torque_upper_m_per_n = DRV_COAX_CTRL_PROP9047_YAW_M_PER_N;
    params->yaw_torque_lower_m_per_n = DRV_COAX_CTRL_PROP9047_YAW_M_PER_N;
}

static void coax_ctrl_compute_accel_cmd(
    const DRV_COAX_CTRL_AttitudeInput *attitude,
    const DRV_COAX_CTRL_Reference *reference,
    const float velocity_integral_m[2],
    float horizontal_scale,
    DRV_COAX_CTRL_Debug *debug)
{
    float horizontal_norm;

    debug->pos_p_m_s2[0] = -coax_ctrl_params.vel_loop_x_kp *
                              (attitude->vx_m_s - reference->vx_m_s);
    debug->pos_p_m_s2[1] = -coax_ctrl_params.vel_loop_y_kp *
                              (attitude->vy_m_s - reference->vy_m_s);
    debug->pos_p_m_s2[2] =
        coax_ctrl_params.pos_z_kp * (reference->z_m - attitude->z_m);

    debug->vel_d_m_s2[0] =
        -coax_ctrl_params.vel_loop_x_ki * velocity_integral_m[0];
    debug->vel_d_m_s2[1] =
        -coax_ctrl_params.vel_loop_y_ki * velocity_integral_m[1];
    debug->vel_d_m_s2[2] =
        coax_ctrl_params.vel_z_kd * (reference->vz_m_s - attitude->vz_m_s);

    if (coax_ctrl_params.vel_loop_enable >= 0.5f) {
        debug->accel_out_m_s2[0] = reference->ax_m_s2 +
                                     debug->pos_p_m_s2[0] +
                                     debug->vel_d_m_s2[0];
        debug->accel_out_m_s2[1] = reference->ay_m_s2 +
                                     debug->pos_p_m_s2[1] +
                                     debug->vel_d_m_s2[1];
    } else {
        debug->pos_p_m_s2[0] = 0.0f;
        debug->pos_p_m_s2[1] = 0.0f;
        debug->vel_d_m_s2[0] = 0.0f;
        debug->vel_d_m_s2[1] = 0.0f;
        debug->accel_out_m_s2[0] = reference->ax_m_s2;
        debug->accel_out_m_s2[1] = reference->ay_m_s2;
    }

    horizontal_norm = sqrtf((debug->accel_out_m_s2[0] *
                             debug->accel_out_m_s2[0]) +
                            (debug->accel_out_m_s2[1] *
                             debug->accel_out_m_s2[1]));
    if (horizontal_norm > DRV_COAX_CTRL_HORIZONTAL_ACCEL_LIMIT_M_S2) {
        const float limit_scale =
            DRV_COAX_CTRL_HORIZONTAL_ACCEL_LIMIT_M_S2 / horizontal_norm;
        debug->accel_out_m_s2[0] *= limit_scale;
        debug->accel_out_m_s2[1] *= limit_scale;
        debug->pos_p_m_s2[0] *= limit_scale;
        debug->pos_p_m_s2[1] *= limit_scale;
        debug->vel_d_m_s2[0] *= limit_scale;
        debug->vel_d_m_s2[1] *= limit_scale;
    }

    debug->accel_out_m_s2[0] *= horizontal_scale;
    debug->accel_out_m_s2[1] *= horizontal_scale;
    debug->pos_p_m_s2[0] *= horizontal_scale;
    debug->pos_p_m_s2[1] *= horizontal_scale;
    debug->vel_d_m_s2[0] *= horizontal_scale;
    debug->vel_d_m_s2[1] *= horizontal_scale;
    debug->accel_out_m_s2[2] =
        debug->pos_p_m_s2[2] + debug->vel_d_m_s2[2] + reference->az_m_s2;
}

static void coax_ctrl_compute_balance_solution(
    const DRV_COAX_CTRL_AttitudeInput *attitude,
    const DRV_COAX_CTRL_Reference *reference,
    DRV_COAX_CTRL_Debug *debug,
    DRV_COAX_CTRL_BalanceSolution *solution)
{
    float actual_r[3][3];
    float actual_t[3][3];
    float gimbal_r[3][3];
    float gimbal_t[3][3];
    float actual_t_desired[3][3];
    float desired_omega[3] = { 0.0f, 0.0f, reference->yaw_rate_rad_s };
    float desired_omega_actual[3];
    const float actual_omega[3] = {
        DRV_COAX_CTRL_FORCE_FRAME_ROLL_SIGN * attitude->gyro_x_rad_s,
        DRV_COAX_CTRL_FORCE_FRAME_PITCH_SIGN * attitude->gyro_y_rad_s,
        attitude->gyro_z_rad_s,
    };
    const float kr_roll = -coax_ctrl_params.roll_angle_kp;
    const float kr_pitch = -coax_ctrl_params.pitch_angle_kp;
    const float kw_roll = -coax_ctrl_params.roll_rate_kd;
    const float kw_pitch = -coax_ctrl_params.pitch_rate_kd;
    const float sin_tilt_limit = sinf(coax_ctrl_params.tilt_limit_rad);
    float force_scale = 1.0f;

    memset(solution, 0, sizeof(*solution));
    solution->desired_force_local_n[0] =
        coax_ctrl_params.mass_kg * debug->accel_out_m_s2[0];
    solution->desired_force_local_n[1] =
        coax_ctrl_params.mass_kg * debug->accel_out_m_s2[1];
    solution->desired_force_local_n[2] =
        coax_ctrl_params.mass_kg *
        (coax_ctrl_params.gravity_m_s2 - debug->accel_out_m_s2[2]);
    if (solution->desired_force_local_n[2] < DRV_COAX_CTRL_FORCE_EPS_N) {
        solution->desired_force_local_n[2] = DRV_COAX_CTRL_FORCE_EPS_N;
    }

    solution->raw_total_force_n =
        coax_ctrl_norm3(solution->desired_force_local_n);
    solution->thrust_utilization =
        solution->raw_total_force_n / DRV_AIRFRAME_MAX_TOTAL_FORCE_N;
    if (solution->raw_total_force_n > DRV_AIRFRAME_MAX_TOTAL_FORCE_N) {
        force_scale = DRV_AIRFRAME_MAX_TOTAL_FORCE_N /
                      solution->raw_total_force_n;
        for (uint32_t axis = 0U; axis < 3U; ++axis) {
            solution->desired_force_local_n[axis] *= force_scale;
        }
    }
    solution->total_force_n = coax_ctrl_norm3(solution->desired_force_local_n);

    coax_ctrl_attitude_matrix(attitude, actual_r);
    coax_ctrl_matrix_transpose(actual_r, actual_t);
    coax_ctrl_build_thrust_frame(solution->desired_force_local_n,
                                 reference->yaw_rad,
                                 solution->thrust_frame_r);

    for (uint32_t iteration = 0U;
         iteration < DRV_COAX_CTRL_BALANCE_ITERATIONS;
         ++iteration) {
        float gyro_momentum_cross[3];
        float roll_capacity;
        float pitch_capacity;
        float beta_argument;
        float alpha_argument;

        coax_ctrl_gimbal_matrix(solution->alpha_rad,
                                solution->beta_rad,
                                gimbal_r);
        coax_ctrl_matrix_transpose(gimbal_r, gimbal_t);
        coax_ctrl_matrix_multiply(solution->thrust_frame_r,
                                  gimbal_t,
                                  solution->desired_body_r);
        coax_ctrl_attitude_error(solution->desired_body_r,
                                 actual_r,
                                 solution->attitude_error,
                                 &solution->attitude_error_angle_rad);
        coax_ctrl_matrix_multiply(actual_t,
                                  solution->desired_body_r,
                                  actual_t_desired);
        for (uint32_t row = 0U; row < 3U; ++row) {
            desired_omega_actual[row] =
                (actual_t_desired[row][0] * desired_omega[0]) +
                (actual_t_desired[row][1] * desired_omega[1]) +
                (actual_t_desired[row][2] * desired_omega[2]);
            solution->rate_error_rad_s[row] =
                actual_omega[row] - desired_omega_actual[row];
        }

        /* omega x (J * omega) for the diagonal airframe inertia model. */
        gyro_momentum_cross[0] =
            (DRV_AIRFRAME_IZZ_KGM2 - DRV_AIRFRAME_IYY_KGM2) *
            actual_omega[1] * actual_omega[2];
        gyro_momentum_cross[1] =
            (DRV_AIRFRAME_IXX_KGM2 - DRV_AIRFRAME_IZZ_KGM2) *
            actual_omega[2] * actual_omega[0];
        gyro_momentum_cross[2] =
            (DRV_AIRFRAME_IYY_KGM2 - DRV_AIRFRAME_IXX_KGM2) *
            actual_omega[0] * actual_omega[1];

        solution->moment_cmd_n_m[0] =
            (-kr_roll * solution->attitude_error[0]) -
            (kw_roll * solution->rate_error_rad_s[0]) +
            gyro_momentum_cross[0];
        solution->moment_cmd_n_m[1] =
            (-kr_pitch * solution->attitude_error[1]) -
            (kw_pitch * solution->rate_error_rad_s[1]) +
            gyro_momentum_cross[1];
        solution->moment_cmd_n_m[2] = gyro_momentum_cross[2];

        roll_capacity = DRV_COAX_CTRL_ROLL_EFFECTIVENESS *
                        coax_ctrl_params.roll_tilt_lever_arm_m *
                        solution->total_force_n;
        if (roll_capacity < DRV_COAX_CTRL_RATE_SCALE_EPS) {
            roll_capacity = DRV_COAX_CTRL_RATE_SCALE_EPS;
        }
        beta_argument = solution->moment_cmd_n_m[0] /
                        (DRV_COAX_CTRL_ROLL_MOMENT_SIGN * roll_capacity);
        solution->beta_rad = asinf(coax_ctrl_clamp_f32(beta_argument,
                                                       -sin_tilt_limit,
                                                        sin_tilt_limit));

        pitch_capacity = DRV_COAX_CTRL_PITCH_EFFECTIVENESS *
                         coax_ctrl_params.pitch_tilt_lever_arm_m *
                         solution->total_force_n * cosf(solution->beta_rad);
        if (pitch_capacity < DRV_COAX_CTRL_RATE_SCALE_EPS) {
            pitch_capacity = DRV_COAX_CTRL_RATE_SCALE_EPS;
        }
        alpha_argument = solution->moment_cmd_n_m[1] /
                         (DRV_COAX_CTRL_PITCH_MOMENT_SIGN * pitch_capacity);
        solution->alpha_rad = asinf(coax_ctrl_clamp_f32(alpha_argument,
                                                        -sin_tilt_limit,
                                                         sin_tilt_limit));

        solution->moment_utilization = fmaxf(
            fabsf(beta_argument) / sin_tilt_limit,
            fabsf(alpha_argument) / sin_tilt_limit);
    }

    coax_ctrl_local_down_to_body(attitude,
                                 solution->desired_force_local_n,
                                 solution->desired_force_body_n);
    debug->force_cmd_n[0] = solution->desired_force_body_n[0];
    debug->force_cmd_n[1] = solution->desired_force_body_n[1];
    debug->force_cmd_n[2] = solution->desired_force_body_n[2];

    debug->tilt_ff_rad[0] =
        atan2f(solution->desired_force_body_n[0],
               solution->desired_force_body_n[2]);
    debug->tilt_ff_rad[1] =
        -atan2f(solution->desired_force_body_n[1] *
                cosf(debug->tilt_ff_rad[0]),
                solution->desired_force_body_n[2]);

    {
        float roll_capacity = DRV_COAX_CTRL_ROLL_EFFECTIVENESS *
                              coax_ctrl_params.roll_tilt_lever_arm_m *
                              solution->total_force_n;
        float pitch_capacity = DRV_COAX_CTRL_PITCH_EFFECTIVENESS *
                               coax_ctrl_params.pitch_tilt_lever_arm_m *
                               solution->total_force_n *
                               cosf(solution->beta_rad);

        if (roll_capacity < DRV_COAX_CTRL_RATE_SCALE_EPS) {
            roll_capacity = DRV_COAX_CTRL_RATE_SCALE_EPS;
        }
        if (pitch_capacity < DRV_COAX_CTRL_RATE_SCALE_EPS) {
            pitch_capacity = DRV_COAX_CTRL_RATE_SCALE_EPS;
        }
        debug->tilt_angle_p_rad[0] = asinf(coax_ctrl_clamp_f32(
            (-kr_pitch * solution->attitude_error[1]) /
                (DRV_COAX_CTRL_PITCH_MOMENT_SIGN * pitch_capacity),
            -sin_tilt_limit,
             sin_tilt_limit));
        debug->tilt_angle_p_rad[1] = asinf(coax_ctrl_clamp_f32(
            (-kr_roll * solution->attitude_error[0]) /
                (DRV_COAX_CTRL_ROLL_MOMENT_SIGN * roll_capacity),
            -sin_tilt_limit,
             sin_tilt_limit));
        debug->tilt_rate_d_rad[0] = asinf(coax_ctrl_clamp_f32(
            (-kw_pitch * solution->rate_error_rad_s[1]) /
                (DRV_COAX_CTRL_PITCH_MOMENT_SIGN * pitch_capacity),
            -sin_tilt_limit,
             sin_tilt_limit));
        debug->tilt_rate_d_rad[1] = asinf(coax_ctrl_clamp_f32(
            (-kw_roll * solution->rate_error_rad_s[0]) /
                (DRV_COAX_CTRL_ROLL_MOMENT_SIGN * roll_capacity),
            -sin_tilt_limit,
             sin_tilt_limit));
    }

    debug->tilt_out_rad[0] = solution->alpha_rad;
    debug->tilt_out_rad[1] = solution->beta_rad;
    coax_ctrl_rotation_to_rpy(solution->desired_body_r,
                              debug->desired_attitude_rpy_rad);
    memcpy(debug->attitude_error,
           solution->attitude_error,
           sizeof(debug->attitude_error));
    memcpy(debug->rate_error_rad_s,
           solution->rate_error_rad_s,
           sizeof(debug->rate_error_rad_s));
    memcpy(debug->moment_cmd_n_m,
           solution->moment_cmd_n_m,
           sizeof(debug->moment_cmd_n_m));
    debug->total_force_n = solution->total_force_n;
    debug->moment_utilization = solution->moment_utilization;
    debug->thrust_utilization = solution->thrust_utilization;
}

static float coax_ctrl_balance_protection_scale(
    const DRV_COAX_CTRL_Reference *reference,
    const DRV_COAX_CTRL_BalanceSolution *solution,
    uint32_t *flags)
{
    float scale = 1.0f;
    float candidate;

    *flags = 0U;
    if ((coax_ctrl_params.vel_loop_enable >= 0.5f) &&
        (reference->horizontal_velocity_valid == 0U)) {
        *flags |= DRV_COAX_CTRL_PROTECT_VELOCITY_INVALID;
        scale = 0.0f;
    }

    candidate = coax_ctrl_protection_scale(
        solution->attitude_error_angle_rad,
        DRV_COAX_CTRL_ATTITUDE_PROTECT_START_RAD,
        DRV_COAX_CTRL_ATTITUDE_PROTECT_END_RAD);
    if (candidate < 1.0f) {
        *flags |= DRV_COAX_CTRL_PROTECT_ATTITUDE;
        scale = fminf(scale, candidate);
    }

    candidate = coax_ctrl_protection_scale(
        solution->moment_utilization,
        DRV_COAX_CTRL_MOMENT_PROTECT_START,
        DRV_COAX_CTRL_MOMENT_PROTECT_END);
    if (candidate < 1.0f) {
        *flags |= DRV_COAX_CTRL_PROTECT_MOMENT;
        scale = fminf(scale, candidate);
    }

    candidate = coax_ctrl_protection_scale(
        solution->thrust_utilization,
        DRV_COAX_CTRL_THRUST_PROTECT_START,
        DRV_COAX_CTRL_THRUST_PROTECT_END);
    if (candidate < 1.0f) {
        *flags |= DRV_COAX_CTRL_PROTECT_THRUST;
        scale = fminf(scale, candidate);
    }

    return scale;
}

static void coax_ctrl_compute_balance_command(
    const DRV_COAX_CTRL_AttitudeInput *attitude,
    const DRV_COAX_CTRL_Reference *reference,
    DRV_COAX_CTRL_Debug *debug,
    DRV_COAX_CTRL_BalanceSolution *solution)
{
    float candidate_integral_m[2] = {
        coax_ctrl_state.velocity_integral_m[0],
        coax_ctrl_state.velocity_integral_m[1],
    };
    float dt_sec = reference->dt_sec;
    float horizontal_scale;

    if ((dt_sec <= 0.0f) || (dt_sec > 0.2f)) {
        dt_sec = 0.02f;
    }

    if (coax_ctrl_params.vel_loop_enable < 0.5f) {
        candidate_integral_m[0] = 0.0f;
        candidate_integral_m[1] = 0.0f;
        coax_ctrl_state.velocity_integral_m[0] = 0.0f;
        coax_ctrl_state.velocity_integral_m[1] = 0.0f;
    } else if (reference->horizontal_velocity_valid != 0U) {
        candidate_integral_m[0] = coax_ctrl_clamp_f32(
            candidate_integral_m[0] +
            (attitude->vx_m_s - reference->vx_m_s) * dt_sec,
            -DRV_COAX_CTRL_VEL_INTEGRAL_LIMIT_M,
             DRV_COAX_CTRL_VEL_INTEGRAL_LIMIT_M);
        candidate_integral_m[1] = coax_ctrl_clamp_f32(
            candidate_integral_m[1] +
            (attitude->vy_m_s - reference->vy_m_s) * dt_sec,
            -DRV_COAX_CTRL_VEL_INTEGRAL_LIMIT_M,
             DRV_COAX_CTRL_VEL_INTEGRAL_LIMIT_M);
    }

    coax_ctrl_compute_accel_cmd(attitude,
                                reference,
                                candidate_integral_m,
                                1.0f,
                                debug);
    coax_ctrl_compute_balance_solution(attitude, reference, debug, solution);
    horizontal_scale = coax_ctrl_balance_protection_scale(reference,
                                                           solution,
                                                           &debug->protection_flags);

    if (horizontal_scale >= 0.999f) {
        coax_ctrl_state.velocity_integral_m[0] = candidate_integral_m[0];
        coax_ctrl_state.velocity_integral_m[1] = candidate_integral_m[1];
    } else {
        coax_ctrl_compute_accel_cmd(attitude,
                                    reference,
                                    coax_ctrl_state.velocity_integral_m,
                                    horizontal_scale,
                                    debug);
        coax_ctrl_compute_balance_solution(attitude, reference, debug, solution);
    }

    debug->horizontal_command_scale = horizontal_scale;
    debug->velocity_integral_m[0] = coax_ctrl_state.velocity_integral_m[0];
    debug->velocity_integral_m[1] = coax_ctrl_state.velocity_integral_m[1];
}

static float coax_ctrl_compute_yaw_torque_cmd(
    const DRV_COAX_CTRL_AttitudeInput *attitude,
    const DRV_COAX_CTRL_Reference *reference,
    DRV_COAX_CTRL_Debug *debug)
{
    const float yaw_err = coax_ctrl_wrap_pi(reference->yaw_rad - attitude->yaw_rad);

    debug->yaw_angle_p_rad_s = coax_ctrl_params.yaw_angle_kp * yaw_err;
    debug->yaw_rate_d_rad_s =
        coax_ctrl_params.yaw_rate_kd *
        (reference->yaw_rate_rad_s - attitude->gyro_z_rad_s);

    return coax_ctrl_params.yaw_inertia *
           (reference->yaw_accel_rad_s2 +
            debug->yaw_angle_p_rad_s +
            debug->yaw_rate_d_rad_s);
}

static void coax_ctrl_allocate_motor_thrust(float total_force_n,
                                            float yaw_torque_cmd,
                                            float *upper_n,
                                            float *lower_n)
{
    const float ku = coax_ctrl_params.yaw_torque_upper_m_per_n;
    const float kl = coax_ctrl_params.yaw_torque_lower_m_per_n;
    float denom = ku + kl;

    if (denom < DRV_COAX_CTRL_RATE_SCALE_EPS) {
        denom = DRV_COAX_CTRL_RATE_SCALE_EPS;
    }

    *upper_n = (kl * total_force_n - yaw_torque_cmd) / denom;
    *lower_n = (ku * total_force_n + yaw_torque_cmd) / denom;

    *upper_n = coax_ctrl_clamp_f32(*upper_n, 0.0f,
                                   coax_ctrl_params.motor_single_max_thrust_n);
    *lower_n = coax_ctrl_clamp_f32(*lower_n, 0.0f,
                                   coax_ctrl_params.motor_single_max_thrust_n);
}

void DRV_COAX_CTRL_Init(void)
{
    if (coax_ctrl_initialized == 0U) {
        DRV_COAX_CTRL_ResetParams();
        coax_ctrl_initialized = 1U;
    }
}

void DRV_COAX_CTRL_ResetState(void)
{
    memset(&coax_ctrl_state, 0, sizeof(coax_ctrl_state));
    memset(&coax_ctrl_last_debug, 0, sizeof(coax_ctrl_last_debug));
}

void DRV_COAX_CTRL_GetDefaultParams(DRV_COAX_CTRL_Params *params)
{
    if (params == NULL) {
        return;
    }

    params->pos_x_kp = 2.2f;
    params->pos_y_kp = 2.2f;
    params->pos_z_kp = 3.8f;
    params->vel_x_kd = 0.0f;
    params->vel_y_kd = 0.0f;
    params->vel_z_kd = 0.0f;
    params->vel_loop_enable = 1.0f;
    params->vel_loop_x_kp = 0.50f;
    params->vel_loop_x_ki = 0.0625f;
    params->vel_loop_x_kd = 0.0f;
    params->vel_loop_y_kp = 0.50f;
    params->vel_loop_y_ki = 0.0625f;
    params->vel_loop_y_kd = 0.0f;
    params->mass_kg = DRV_AIRFRAME_MASS_KG;
    params->gravity_m_s2 = DRV_AIRFRAME_GRAVITY_M_S2;
    params->pitch_tilt_lever_arm_m = DRV_AIRFRAME_PITCH_THRUST_LEVER_ARM_M;
    params->roll_tilt_lever_arm_m = DRV_AIRFRAME_ROLL_THRUST_LEVER_ARM_M;
    /* Stored signs preserve the existing positive UI convention. */
    params->roll_angle_kp = -0.0671f;
    params->pitch_angle_kp = -0.0660f;
    params->roll_rate_kd = -0.1104f;
    params->pitch_rate_kd = -0.1138f;
    params->tilt_limit_rad = DRV_COAX_CTRL_TILT_LIMIT_RAD;
    params->yaw_angle_kp = -1.0f;
    params->yaw_rate_kd = -0.15f;
    coax_ctrl_apply_fixed_model_params(params);
}

void DRV_COAX_CTRL_ResetParams(void)
{
    DRV_COAX_CTRL_GetDefaultParams(&coax_ctrl_params);
    DRV_COAX_CTRL_ResetState();
}

void DRV_COAX_CTRL_GetParams(DRV_COAX_CTRL_Params *params)
{
    if (params == NULL) {
        return;
    }

    DRV_COAX_CTRL_Init();
    *params = coax_ctrl_params;
}

void DRV_COAX_CTRL_SetParams(const DRV_COAX_CTRL_Params *params)
{
    if (params == NULL) {
        return;
    }

    DRV_COAX_CTRL_Init();
    DRV_COAX_CTRL_Params candidate = *params;
    coax_ctrl_apply_fixed_model_params(&candidate);
    if (coax_ctrl_params_valid(&candidate) != 0U) {
        coax_ctrl_params = candidate;
        DRV_COAX_CTRL_ResetState();
    } else {
        DRV_COAX_CTRL_ResetParams();
    }
}

uint32_t DRV_COAX_CTRL_ParamCount(void)
{
    return coax_ctrl_param_count;
}

const char *DRV_COAX_CTRL_ParamName(uint32_t index)
{
    return (index < coax_ctrl_param_count) ? coax_ctrl_param_table[index].name : NULL;
}

uint8_t DRV_COAX_CTRL_GetParam(const char *name, float *value)
{
    const DRV_COAX_CTRL_ParamEntry *entry = coax_ctrl_find_param(name);

    if ((entry == NULL) || (value == NULL)) {
        return 0U;
    }

    DRV_COAX_CTRL_Init();
    *value = *coax_ctrl_param_ptr(&coax_ctrl_params, entry);
    return 1U;
}

uint8_t DRV_COAX_CTRL_SetParam(const char *name, float value)
{
    const DRV_COAX_CTRL_ParamEntry *entry = coax_ctrl_find_param(name);
    DRV_COAX_CTRL_Params candidate;

    if (coax_ctrl_param_value_valid(entry, value) == 0U) {
        return 0U;
    }

    DRV_COAX_CTRL_Init();
    candidate = coax_ctrl_params;
    *coax_ctrl_param_ptr(&candidate, entry) = value;
    if (coax_ctrl_params_valid(&candidate) == 0U) {
        return 0U;
    }

    coax_ctrl_params = candidate;
    if ((entry->offset == offsetof(DRV_COAX_CTRL_Params, vel_loop_enable)) ||
        (entry->offset == offsetof(DRV_COAX_CTRL_Params, vel_loop_x_kp)) ||
        (entry->offset == offsetof(DRV_COAX_CTRL_Params, vel_loop_x_ki)) ||
        (entry->offset == offsetof(DRV_COAX_CTRL_Params, vel_loop_x_kd)) ||
        (entry->offset == offsetof(DRV_COAX_CTRL_Params, vel_loop_y_kp)) ||
        (entry->offset == offsetof(DRV_COAX_CTRL_Params, vel_loop_y_ki)) ||
        (entry->offset == offsetof(DRV_COAX_CTRL_Params, vel_loop_y_kd))) {
        DRV_COAX_CTRL_ResetState();
    }
    return 1U;
}

static uint16_t coax_ctrl_tilt_rad_to_servo_pulse(float tilt_rad,
                                                  uint16_t center_us,
                                                  uint16_t min_us,
                                                  uint16_t max_us)
{
    const float servo_span_us = (float)(DRV_COAX_CTRL_SERVO_PHYSICAL_MAX_US -
                                        DRV_COAX_CTRL_SERVO_PHYSICAL_MIN_US);
    const float servo_us_per_rad = servo_span_us / DRV_COAX_CTRL_SERVO_TRAVEL_RAD;
    DRV_COAX_CTRL_Init();
    const float tilt = coax_ctrl_clamp_f32(tilt_rad,
                                          -coax_ctrl_params.tilt_limit_rad,
                                           coax_ctrl_params.tilt_limit_rad);
    const float pulse_f = (float)center_us + tilt * servo_us_per_rad;
    const int32_t pulse_i =
        (int32_t)(pulse_f + ((pulse_f >= 0.0f) ? 0.5f : -0.5f));

    return coax_ctrl_clamp_u16(pulse_i, min_us, max_us);
}

uint16_t DRV_COAX_CTRL_AlphaTiltRadToServoPulse(float tilt_rad)
{
    return coax_ctrl_tilt_rad_to_servo_pulse(tilt_rad,
                                             DRV_COAX_CTRL_SERVO_ALPHA_CENTER_US,
                                             DRV_COAX_CTRL_SERVO_ALPHA_MIN_US,
                                             DRV_COAX_CTRL_SERVO_ALPHA_MAX_US);
}

uint16_t DRV_COAX_CTRL_BetaTiltRadToServoPulse(float tilt_rad)
{
    return coax_ctrl_tilt_rad_to_servo_pulse(tilt_rad,
                                             DRV_COAX_CTRL_SERVO_BETA_CENTER_US,
                                             DRV_COAX_CTRL_SERVO_BETA_MIN_US,
                                             DRV_COAX_CTRL_SERVO_BETA_MAX_US);
}

static void coax_ctrl_body_tilt_to_servo_tilts(float body_x_tilt_rad,
                                               float body_y_tilt_rad,
                                               float *servo_alpha_tilt_rad,
                                               float *servo_beta_tilt_rad)
{
    /*
     * The tilt-servo module is mounted 90 degrees CCW in top view.
     * Keep controller coordinates as X-forward/Y-right:
     *   servo 1 / alpha is now the left-right physical axis,
     *   servo 2 / beta is now the front-back physical axis.
     */
    if (servo_alpha_tilt_rad != NULL) {
        *servo_alpha_tilt_rad = -body_y_tilt_rad;
    }
    if (servo_beta_tilt_rad != NULL) {
        *servo_beta_tilt_rad = -body_x_tilt_rad;
    }
}

void DRV_COAX_CTRL_BodyTiltRadToServoPulses(float body_x_tilt_rad,
                                            float body_y_tilt_rad,
                                            uint16_t *servo_alpha_us,
                                            uint16_t *servo_beta_us)
{
    float servo_alpha_tilt_rad = 0.0f;
    float servo_beta_tilt_rad = 0.0f;

    coax_ctrl_body_tilt_to_servo_tilts(body_x_tilt_rad,
                                       body_y_tilt_rad,
                                       &servo_alpha_tilt_rad,
                                       &servo_beta_tilt_rad);
    if (servo_alpha_us != NULL) {
        *servo_alpha_us = DRV_COAX_CTRL_AlphaTiltRadToServoPulse(
            servo_alpha_tilt_rad * DRV_COAX_CTRL_SERVO_ALPHA_SIGN);
    }
    if (servo_beta_us != NULL) {
        *servo_beta_us = DRV_COAX_CTRL_BetaTiltRadToServoPulse(
            servo_beta_tilt_rad * DRV_COAX_CTRL_SERVO_BETA_SIGN);
    }
}

uint16_t DRV_COAX_CTRL_ThrustToMotorPulse(float thrust_n)
{
    float thrust_g;

    DRV_COAX_CTRL_Init();

    thrust_n = coax_ctrl_clamp_f32(thrust_n, 0.0f,
                                   coax_ctrl_params.motor_single_max_thrust_n);
    thrust_g = thrust_n * DRV_COAX_CTRL_GRAMS_PER_NEWTON * 2.0f;

    if (thrust_g <= coax_ctrl_dual_thrust_g[0]) {
        return coax_ctrl_dual_pwm_us[0];
    }

    for (uint32_t i = 1U; i < DRV_COAX_CTRL_THRUST_TABLE_POINTS; ++i) {
        if (thrust_g <= coax_ctrl_dual_thrust_g[i]) {
            const float left_g = coax_ctrl_dual_thrust_g[i - 1U];
            const float right_g = coax_ctrl_dual_thrust_g[i];
            const float left_pwm = (float)coax_ctrl_dual_pwm_us[i - 1U];
            const float right_pwm = (float)coax_ctrl_dual_pwm_us[i];
            const float ratio =
                (right_g > left_g) ? ((thrust_g - left_g) / (right_g - left_g)) : 0.0f;
            const float pulse_f = left_pwm + ratio * (right_pwm - left_pwm);
            const int32_t pulse_i = (int32_t)(pulse_f + 0.5f);
            return coax_ctrl_clamp_u16(pulse_i,
                                       BSP_PWM_ESC_MIN_US,
                                       BSP_PWM_ESC_MAX_US);
        }
    }

    return BSP_PWM_ESC_MAX_US;
}

void DRV_COAX_CTRL_Run(const DRV_COAX_CTRL_AttitudeInput *attitude,
                       const DRV_COAX_CTRL_Reference *reference,
                       DRV_COAX_CTRL_Output *output)
{
    DRV_COAX_CTRL_Debug debug;
    DRV_COAX_CTRL_BalanceSolution solution;
    float yaw_torque_cmd;
    float thrust_upper_n;
    float thrust_lower_n;

    if ((attitude == NULL) || (reference == NULL) || (output == NULL)) {
        return;
    }

    DRV_COAX_CTRL_Init();
    memset(&debug, 0, sizeof(debug));
    memset(output, 0, sizeof(*output));

    coax_ctrl_compute_balance_command(attitude, reference, &debug, &solution);
    yaw_torque_cmd =
        coax_ctrl_compute_yaw_torque_cmd(attitude, reference, &debug);
    debug.moment_cmd_n_m[2] = yaw_torque_cmd;
    coax_ctrl_allocate_motor_thrust(debug.total_force_n,
                                    yaw_torque_cmd,
                                    &thrust_upper_n,
                                    &thrust_lower_n);

    output->thrust_upper_n = thrust_upper_n;
    output->thrust_lower_n = thrust_lower_n;
    output->alpha_rad = solution.alpha_rad;
    output->beta_rad = solution.beta_rad;
    output->motor_upper_us = DRV_COAX_CTRL_ThrustToMotorPulse(thrust_upper_n);
    output->motor_lower_us = DRV_COAX_CTRL_ThrustToMotorPulse(thrust_lower_n);
    DRV_COAX_CTRL_BodyTiltRadToServoPulses(output->alpha_rad,
                                           output->beta_rad,
                                           &output->servo_alpha_us,
                                           &output->servo_beta_us);

    debug.motor_thrust_cmd_n[0] = output->thrust_upper_n;
    debug.motor_thrust_cmd_n[1] = output->thrust_lower_n;
    debug.motor_cmd_us[0] = (float)output->motor_upper_us;
    debug.motor_cmd_us[1] = (float)output->motor_lower_us;
    debug.yaw_torque_cmd =
        (coax_ctrl_params.yaw_torque_lower_m_per_n * output->thrust_lower_n) -
        (coax_ctrl_params.yaw_torque_upper_m_per_n * output->thrust_upper_n);

    coax_ctrl_last_debug = debug;
}

void DRV_COAX_CTRL_GetLastDebug(DRV_COAX_CTRL_Debug *debug)
{
    if (debug == NULL) {
        return;
    }

    *debug = coax_ctrl_last_debug;
}

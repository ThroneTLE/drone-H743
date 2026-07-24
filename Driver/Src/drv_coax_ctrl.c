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

typedef struct {
    const char *name;
    uint16_t offset;
} DRV_COAX_CTRL_ParamEntry;

static uint8_t coax_ctrl_initialized;
static DRV_COAX_CTRL_Params coax_ctrl_params;
static DRV_COAX_CTRL_Debug coax_ctrl_last_debug;

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

static void coax_ctrl_compute_force_cmd(
    const DRV_COAX_CTRL_AttitudeInput *attitude,
    const DRV_COAX_CTRL_Reference *reference,
    DRV_COAX_CTRL_Debug *debug)
{
    const float vel_local_down[3] = {
        attitude->vx_m_s,
        attitude->vy_m_s,
        attitude->vz_m_s,
    };
    const float omega_b[3] = {
        attitude->gyro_x_rad_s,
        attitude->gyro_y_rad_s,
        attitude->gyro_z_rad_s,
    };
    float thrust_accel_local_down[3];
    float thrust_accel_b[3];
    float vel_b[3];
    float omega_cross_vel_b[3];

    debug->pos_p_m_s2[0] =
        coax_ctrl_params.pos_x_kp * (reference->x_m - attitude->x_m);
    debug->pos_p_m_s2[1] =
        coax_ctrl_params.pos_y_kp * (reference->y_m - attitude->y_m);
    debug->pos_p_m_s2[2] =
        coax_ctrl_params.pos_z_kp * (reference->z_m - attitude->z_m);

    debug->vel_d_m_s2[0] =
        coax_ctrl_params.vel_x_kd * (reference->vx_m_s - attitude->vx_m_s);
    debug->vel_d_m_s2[1] =
        coax_ctrl_params.vel_y_kd * (reference->vy_m_s - attitude->vy_m_s);
    debug->vel_d_m_s2[2] =
        coax_ctrl_params.vel_z_kd * (reference->vz_m_s - attitude->vz_m_s);

    debug->accel_out_m_s2[0] =
        debug->pos_p_m_s2[0] + debug->vel_d_m_s2[0] + reference->ax_m_s2;
    debug->accel_out_m_s2[1] =
        debug->pos_p_m_s2[1] + debug->vel_d_m_s2[1] + reference->ay_m_s2;
    debug->accel_out_m_s2[2] =
        debug->pos_p_m_s2[2] + debug->vel_d_m_s2[2] + reference->az_m_s2;

    thrust_accel_local_down[0] = debug->accel_out_m_s2[0];
    thrust_accel_local_down[1] = debug->accel_out_m_s2[1];
    thrust_accel_local_down[2] =
        coax_ctrl_params.gravity_m_s2 - debug->accel_out_m_s2[2];
    coax_ctrl_local_down_to_body(attitude, thrust_accel_local_down, thrust_accel_b);
    coax_ctrl_local_down_to_body(attitude, vel_local_down, vel_b);
    coax_ctrl_cross3(omega_b, vel_b, omega_cross_vel_b);

    debug->force_cmd_n[0] =
        coax_ctrl_params.mass_kg * (thrust_accel_b[0] - omega_cross_vel_b[0]);
    debug->force_cmd_n[1] =
        coax_ctrl_params.mass_kg * (thrust_accel_b[1] - omega_cross_vel_b[1]);
    debug->force_cmd_n[2] =
        coax_ctrl_params.mass_kg * (thrust_accel_b[2] - omega_cross_vel_b[2]);
    if (debug->force_cmd_n[2] < DRV_COAX_CTRL_FORCE_EPS_N) {
        debug->force_cmd_n[2] = DRV_COAX_CTRL_FORCE_EPS_N;
    }
}

static void coax_ctrl_compute_tilt_from_force(
    const DRV_COAX_CTRL_AttitudeInput *attitude,
    DRV_COAX_CTRL_Debug *debug,
    float *alpha_rad,
    float *beta_rad)
{
    const float force_z = debug->force_cmd_n[2];
    const float alpha_ff_rad = atan2f(debug->force_cmd_n[0], force_z);
    float pitch_rate_scale =
        debug->total_force_n * coax_ctrl_params.pitch_tilt_lever_arm_m;
    float roll_rate_scale =
        debug->total_force_n * coax_ctrl_params.roll_tilt_lever_arm_m;
    float beta_ff_rad;

    if (pitch_rate_scale < DRV_COAX_CTRL_RATE_SCALE_EPS) {
        pitch_rate_scale = DRV_COAX_CTRL_RATE_SCALE_EPS;
    }
    if (roll_rate_scale < DRV_COAX_CTRL_RATE_SCALE_EPS) {
        roll_rate_scale = DRV_COAX_CTRL_RATE_SCALE_EPS;
    }

    debug->tilt_ff_rad[0] = alpha_ff_rad;
    debug->tilt_angle_p_rad[0] =
        -coax_ctrl_params.pitch_angle_kp * attitude->pitch_rad;
    debug->tilt_rate_d_rad[0] =
        (coax_ctrl_params.pitch_rate_kd * attitude->gyro_y_rad_s) /
        pitch_rate_scale;

    *alpha_rad = coax_ctrl_clamp_f32(alpha_ff_rad +
                                     debug->tilt_angle_p_rad[0] +
                                     debug->tilt_rate_d_rad[0],
                                     -coax_ctrl_params.tilt_limit_rad,
                                      coax_ctrl_params.tilt_limit_rad);

    beta_ff_rad =
        -atan2f(debug->force_cmd_n[1] * cosf(*alpha_rad), force_z);
    debug->tilt_ff_rad[1] = beta_ff_rad;
    debug->tilt_angle_p_rad[1] =
        -coax_ctrl_params.roll_angle_kp * attitude->roll_rad;
    debug->tilt_rate_d_rad[1] =
        (coax_ctrl_params.roll_rate_kd * attitude->gyro_x_rad_s) /
        roll_rate_scale;

    *beta_rad = coax_ctrl_clamp_f32(beta_ff_rad +
                                    debug->tilt_angle_p_rad[1] +
                                    debug->tilt_rate_d_rad[1],
                                    -coax_ctrl_params.tilt_limit_rad,
                                     coax_ctrl_params.tilt_limit_rad);

    debug->tilt_out_rad[0] = *alpha_rad;
    debug->tilt_out_rad[1] = *beta_rad;
}

static float coax_ctrl_compute_total_force(float alpha_rad, float beta_rad,
                                           const DRV_COAX_CTRL_Debug *debug)
{
    float denom = cosf(alpha_rad) * cosf(beta_rad);
    float total_force_n;

    if (denom < 0.05f) {
        denom = 0.05f;
    }

    total_force_n = debug->force_cmd_n[2] / denom;
    return total_force_n;
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
    params->vel_loop_x_kp = 4.95f;
    params->vel_loop_x_ki = 0.0f;
    params->vel_loop_x_kd = 0.0f;
    params->vel_loop_y_kp = 4.81f;
    params->vel_loop_y_ki = 0.0f;
    params->vel_loop_y_kd = 0.0f;
    params->mass_kg = DRV_AIRFRAME_MASS_KG;
    params->gravity_m_s2 = DRV_AIRFRAME_GRAVITY_M_S2;
    params->pitch_tilt_lever_arm_m = DRV_AIRFRAME_PITCH_THRUST_LEVER_ARM_M;
    params->roll_tilt_lever_arm_m = DRV_AIRFRAME_ROLL_THRUST_LEVER_ARM_M;
    params->roll_angle_kp = 0.0f;
    params->pitch_angle_kp = 0.0f;
    params->roll_rate_kd = -0.5f;
    params->pitch_rate_kd = -0.5f;
    params->tilt_limit_rad = DRV_COAX_CTRL_TILT_LIMIT_RAD;
    params->yaw_angle_kp = -1.0f;
    params->yaw_rate_kd = -0.15f;
    coax_ctrl_apply_fixed_model_params(params);
}

void DRV_COAX_CTRL_ResetParams(void)
{
    DRV_COAX_CTRL_GetDefaultParams(&coax_ctrl_params);
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
    float alpha_rad;
    float beta_rad;
    float yaw_torque_cmd;
    float thrust_upper_n;
    float thrust_lower_n;

    if ((attitude == NULL) || (reference == NULL) || (output == NULL)) {
        return;
    }

    DRV_COAX_CTRL_Init();
    memset(&debug, 0, sizeof(debug));
    memset(output, 0, sizeof(*output));

    coax_ctrl_compute_force_cmd(attitude, reference, &debug);
    debug.total_force_n =
        coax_ctrl_compute_total_force(0.0f, 0.0f, &debug);
    coax_ctrl_compute_tilt_from_force(attitude, &debug, &alpha_rad, &beta_rad);
    debug.total_force_n =
        coax_ctrl_compute_total_force(alpha_rad, beta_rad, &debug);
    yaw_torque_cmd =
        coax_ctrl_compute_yaw_torque_cmd(attitude, reference, &debug);
    coax_ctrl_allocate_motor_thrust(debug.total_force_n,
                                    yaw_torque_cmd,
                                    &thrust_upper_n,
                                    &thrust_lower_n);

    output->thrust_upper_n = thrust_upper_n;
    output->thrust_lower_n = thrust_lower_n;
    output->alpha_rad = alpha_rad;
    output->beta_rad = beta_rad;
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

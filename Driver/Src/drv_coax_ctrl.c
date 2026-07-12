#include "drv_coax_ctrl.h"

#include "bsp_pwm.h"
#include "coax_tiltrotor_controller_codegen.h"
#include "coax_tiltrotor_controller_codegen_initialize.h"
#include "drv_airframe_model.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define DRV_COAX_CTRL_TILT_LIMIT_RAD 0.523599f
#define DRV_COAX_CTRL_PI 3.141592654f
#define DRV_COAX_CTRL_SERVO_TRAVEL_RAD \
    (DRV_COAX_CTRL_SERVO_TRAVEL_DEG * DRV_COAX_CTRL_PI / 180.0f)
#define DRV_COAX_CTRL_SERVO_LIMIT_RAD \
    (DRV_COAX_CTRL_SERVO_LIMIT_DEG * DRV_COAX_CTRL_PI / 180.0f)
#define DRV_COAX_CTRL_SERVO_ALPHA_SIGN    (1.0f)
#define DRV_COAX_CTRL_SERVO_BETA_SIGN     (1.0f)
#define DRV_COAX_CTRL_MOTOR_OMEGA_MAX_RAD_S 900.0f

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
    DRV_COAX_CTRL_PARAM_ENTRY(rotation_error_gain),
    DRV_COAX_CTRL_PARAM_ENTRY(accel_xy_limit_m_s2),
    DRV_COAX_CTRL_PARAM_ENTRY(accel_z_limit_m_s2),
    DRV_COAX_CTRL_PARAM_ENTRY(vel_loop_enable),
    DRV_COAX_CTRL_PARAM_ENTRY(vel_loop_x_kp),
    DRV_COAX_CTRL_PARAM_ENTRY(vel_loop_x_ki),
    DRV_COAX_CTRL_PARAM_ENTRY(vel_loop_x_kd),
    DRV_COAX_CTRL_PARAM_ENTRY(vel_loop_y_kp),
    DRV_COAX_CTRL_PARAM_ENTRY(vel_loop_y_ki),
    DRV_COAX_CTRL_PARAM_ENTRY(vel_loop_y_kd),
    DRV_COAX_CTRL_PARAM_ENTRY(vel_loop_output_limit_m_s2),
    DRV_COAX_CTRL_PARAM_ENTRY(vel_loop_i_limit_m_s2),
    DRV_COAX_CTRL_PARAM_ENTRY(mass_kg),
    DRV_COAX_CTRL_PARAM_ENTRY(gravity_m_s2),
    DRV_COAX_CTRL_PARAM_ENTRY(min_total_force_n),
    DRV_COAX_CTRL_PARAM_ENTRY(max_total_force_n),
    DRV_COAX_CTRL_PARAM_ENTRY(tilt_lever_arm_m),
    DRV_COAX_CTRL_PARAM_ENTRY(roll_angle_kp),
    DRV_COAX_CTRL_PARAM_ENTRY(roll_rate_kd),
    DRV_COAX_CTRL_PARAM_ENTRY(pitch_angle_kp),
    DRV_COAX_CTRL_PARAM_ENTRY(pitch_rate_kd),
    DRV_COAX_CTRL_PARAM_ENTRY(tilt_limit_rad),
    DRV_COAX_CTRL_PARAM_ENTRY(yaw_angle_kp),
    DRV_COAX_CTRL_PARAM_ENTRY(yaw_rate_kd),
    DRV_COAX_CTRL_PARAM_ENTRY(yaw_rate_limit_rad_s),
    DRV_COAX_CTRL_PARAM_ENTRY(yaw_inertia),
    DRV_COAX_CTRL_PARAM_ENTRY(thrust_coeff_n_per_rad2),
    DRV_COAX_CTRL_PARAM_ENTRY(yaw_torque_coeff_n_m_per_rad2),
    DRV_COAX_CTRL_PARAM_ENTRY(motor_omega_max_rad_s),
};

static const uint32_t coax_ctrl_param_count =
    sizeof(coax_ctrl_param_table) / sizeof(coax_ctrl_param_table[0]);

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

static void coax_ctrl_compute_pure_damping_tilt(
    const DRV_COAX_CTRL_AttitudeInput *attitude,
    const DRV_COAX_CTRL_Reference *reference,
    float *alpha_rad,
    float *beta_rad,
    DRV_COAX_CTRL_Debug *debug)
{
    float acc_x_m_s2;
    float acc_y_m_s2;
    float vertical_acc_m_s2;
    float rate_torque_scale;
    float pitch_rate_d_rad;
    float roll_rate_d_rad;
    float pitch_ff_rad;
    float roll_ff_rad;

    acc_x_m_s2 = reference->ax_m_s2 -
                 coax_ctrl_params.vel_x_kd *
                 (reference->vx_m_s - attitude->vx_m_s);
    acc_y_m_s2 = reference->ay_m_s2 -
                 coax_ctrl_params.vel_y_kd *
                 (reference->vy_m_s - attitude->vy_m_s);

    acc_x_m_s2 = coax_ctrl_clamp_f32(acc_x_m_s2,
                                     -coax_ctrl_params.accel_xy_limit_m_s2,
                                      coax_ctrl_params.accel_xy_limit_m_s2);
    acc_y_m_s2 = coax_ctrl_clamp_f32(acc_y_m_s2,
                                     -coax_ctrl_params.accel_xy_limit_m_s2,
                                      coax_ctrl_params.accel_xy_limit_m_s2);

    vertical_acc_m_s2 = coax_ctrl_params.gravity_m_s2 - reference->az_m_s2;
    if (vertical_acc_m_s2 < 1.0e-3f) {
        vertical_acc_m_s2 = 1.0e-3f;
    }

    rate_torque_scale = coax_ctrl_params.mass_kg *
                        vertical_acc_m_s2 *
                        coax_ctrl_params.tilt_lever_arm_m;
    if (rate_torque_scale < 1.0e-6f) {
        rate_torque_scale = 1.0e-6f;
    }

    pitch_ff_rad = atan2f(acc_x_m_s2, vertical_acc_m_s2);
    roll_ff_rad = atan2f(acc_y_m_s2, vertical_acc_m_s2);
    pitch_rate_d_rad =
        (coax_ctrl_params.pitch_rate_kd * attitude->gyro_y_rad_s) /
        rate_torque_scale;
    roll_rate_d_rad =
        (coax_ctrl_params.roll_rate_kd * attitude->gyro_x_rad_s) /
        rate_torque_scale;

    *alpha_rad = pitch_ff_rad + pitch_rate_d_rad;
    *beta_rad = roll_ff_rad + roll_rate_d_rad;

    *alpha_rad = coax_ctrl_clamp_f32(*alpha_rad,
                                     -coax_ctrl_params.tilt_limit_rad,
                                      coax_ctrl_params.tilt_limit_rad);
    *beta_rad = coax_ctrl_clamp_f32(*beta_rad,
                                    -coax_ctrl_params.tilt_limit_rad,
                                     coax_ctrl_params.tilt_limit_rad);

    if (debug != NULL) {
        debug->tilt_ff_rad[0] = pitch_ff_rad;
        debug->tilt_ff_rad[1] = roll_ff_rad;
        debug->tilt_rate_d_rad[0] = pitch_rate_d_rad;
        debug->tilt_rate_d_rad[1] = roll_rate_d_rad;
        debug->tilt_out_rad[0] = *alpha_rad;
        debug->tilt_out_rad[1] = *beta_rad;
    }
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

    if ((entry->offset == offsetof(DRV_COAX_CTRL_Params, mass_kg)) ||
        (entry->offset == offsetof(DRV_COAX_CTRL_Params, gravity_m_s2)) ||
        (entry->offset == offsetof(DRV_COAX_CTRL_Params, min_total_force_n)) ||
        (entry->offset == offsetof(DRV_COAX_CTRL_Params, max_total_force_n)) ||
        (entry->offset == offsetof(DRV_COAX_CTRL_Params, tilt_lever_arm_m)) ||
        (entry->offset == offsetof(DRV_COAX_CTRL_Params, tilt_limit_rad)) ||
        (entry->offset == offsetof(DRV_COAX_CTRL_Params, yaw_inertia)) ||
        (entry->offset == offsetof(DRV_COAX_CTRL_Params, thrust_coeff_n_per_rad2)) ||
        (entry->offset == offsetof(DRV_COAX_CTRL_Params, yaw_torque_coeff_n_m_per_rad2)) ||
        (entry->offset == offsetof(DRV_COAX_CTRL_Params, motor_omega_max_rad_s))) {
        return (value > 0.0f) ? 1U : 0U;
    }

    if ((entry->offset == offsetof(DRV_COAX_CTRL_Params, accel_xy_limit_m_s2)) ||
        (entry->offset == offsetof(DRV_COAX_CTRL_Params, accel_z_limit_m_s2)) ||
        (entry->offset == offsetof(DRV_COAX_CTRL_Params, vel_loop_enable)) ||
        (entry->offset == offsetof(DRV_COAX_CTRL_Params, vel_loop_output_limit_m_s2)) ||
        (entry->offset == offsetof(DRV_COAX_CTRL_Params, vel_loop_i_limit_m_s2)) ||
        (entry->offset == offsetof(DRV_COAX_CTRL_Params, yaw_rate_limit_rad_s))) {
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

    return (params->min_total_force_n <= params->max_total_force_n) ? 1U : 0U;
}

static void coax_ctrl_fill_rotation_zyx(float roll_rad,
                                        float pitch_rad,
                                        float yaw_rad,
                                        float state[DRV_COAX_CTRL_STATE_LEN])
{
    const float cr = cosf(roll_rad);
    const float sr = sinf(roll_rad);
    const float cp = cosf(pitch_rad);
    const float sp = sinf(pitch_rad);
    const float cy = cosf(yaw_rad);
    const float sy = sinf(yaw_rad);

    state[6]  = cy * cp;
    state[7]  = sy * cp;
    state[8]  = -sp;
    state[9]  = cy * sp * sr - sy * cr;
    state[10] = sy * sp * sr + cy * cr;
    state[11] = cp * sr;
    state[12] = cy * sp * cr + sy * sr;
    state[13] = sy * sp * cr - cy * sr;
    state[14] = cp * cr;
}

void DRV_COAX_CTRL_Init(void)
{
    if (coax_ctrl_initialized == 0U) {
        DRV_COAX_CTRL_ResetParams();
        coax_tiltrotor_controller_codegen_initialize();
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
    params->rotation_error_gain = 0.5f;
    params->accel_xy_limit_m_s2 = 5.66f;
    params->accel_z_limit_m_s2 = 2.5f;
    params->vel_loop_enable = 1.0f;
    params->vel_loop_x_kp = 4.95f;
    params->vel_loop_x_ki = 0.0f;
    params->vel_loop_x_kd = 0.0f;
    params->vel_loop_y_kp = 4.81f;
    params->vel_loop_y_ki = 0.0f;
    params->vel_loop_y_kd = 0.0f;
    params->vel_loop_output_limit_m_s2 = 2.91f;
    params->vel_loop_i_limit_m_s2 = 0.0f;
    params->mass_kg = DRV_AIRFRAME_MASS_KG;
    params->gravity_m_s2 = DRV_AIRFRAME_GRAVITY_M_S2;
    params->min_total_force_n = DRV_AIRFRAME_WEIGHT_N;
    params->max_total_force_n = DRV_AIRFRAME_MAX_TOTAL_FORCE_N;
    params->tilt_lever_arm_m = 0.18f;
    params->roll_angle_kp = 0.0f;
    params->roll_rate_kd = -0.5f;
    params->pitch_angle_kp = 0.0f;
    params->pitch_rate_kd = -0.5f;

    params->tilt_limit_rad = DRV_COAX_CTRL_TILT_LIMIT_RAD;
    params->yaw_angle_kp = 1.0f;
    params->yaw_rate_kd = 0.15f;
    params->yaw_rate_limit_rad_s = 1.04719758f;
    params->yaw_inertia = 0.52f;
    params->thrust_coeff_n_per_rad2 = 3.0e-5f;
    params->yaw_torque_coeff_n_m_per_rad2 = 1.5e-6f;
    params->motor_omega_max_rad_s = DRV_COAX_CTRL_MOTOR_OMEGA_MAX_RAD_S;
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
    if (coax_ctrl_params_valid(params) != 0U) {
        coax_ctrl_params = *params;
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
    const float pulse_f = (float)center_us +
                          tilt * servo_us_per_rad;
    const int32_t pulse_i = (int32_t)(pulse_f + ((pulse_f >= 0.0f) ? 0.5f : -0.5f));

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

uint16_t DRV_COAX_CTRL_OmegaToMotorPulse(float omega_rad_s)
{
    DRV_COAX_CTRL_Init();
    float omega = coax_ctrl_clamp_f32(omega_rad_s,
                                     0.0f,
                                     coax_ctrl_params.motor_omega_max_rad_s);
    float ratio = omega / coax_ctrl_params.motor_omega_max_rad_s;
    float pulse_f = (float)BSP_PWM_ESC_MIN_US +
                    ratio * (float)(BSP_PWM_ESC_MAX_US - BSP_PWM_ESC_MIN_US);
    int32_t pulse_i = (int32_t)(pulse_f + 0.5f);

    return coax_ctrl_clamp_u16(pulse_i, BSP_PWM_ESC_MIN_US, BSP_PWM_ESC_MAX_US);
}

void DRV_COAX_CTRL_Run(const DRV_COAX_CTRL_AttitudeInput *attitude,
                       const DRV_COAX_CTRL_Reference *reference,
                       DRV_COAX_CTRL_Output *output)
{
    float x_rb[DRV_COAX_CTRL_STATE_LEN];
    float ref_cmd[DRV_COAX_CTRL_REF_LEN];
    float cmd[DRV_COAX_CTRL_CMD_LEN];
    float pos_ref_x;
    float pos_ref_y;
    float pos_ref_z;
    float alpha_rad;
    float beta_rad;
    DRV_COAX_CTRL_Debug debug;

    if ((attitude == NULL) || (reference == NULL) || (output == NULL)) {
        return;
    }

    DRV_COAX_CTRL_Init();
    memset(&debug, 0, sizeof(debug));

    memset(x_rb, 0, sizeof(x_rb));
    x_rb[0] = attitude->x_m;
    x_rb[1] = attitude->y_m;
    x_rb[2] = attitude->z_m;
    x_rb[3] = attitude->vx_m_s;
    x_rb[4] = attitude->vy_m_s;
    x_rb[5] = attitude->vz_m_s;
    coax_ctrl_fill_rotation_zyx(attitude->roll_rad,
                                attitude->pitch_rad,
                                attitude->yaw_rad,
                                x_rb);
    x_rb[15] = attitude->gyro_x_rad_s;
    x_rb[16] = attitude->gyro_y_rad_s;
    x_rb[17] = attitude->gyro_z_rad_s;

    pos_ref_x = reference->x_m;
    pos_ref_y = reference->y_m;
    pos_ref_z = reference->z_m;
    if (fabsf(coax_ctrl_params.pos_x_kp) > 1.0e-6f) {
        pos_ref_x += (coax_ctrl_params.vel_x_kd * reference->vx_m_s +
                      reference->ax_m_s2) / coax_ctrl_params.pos_x_kp;
    }
    if (fabsf(coax_ctrl_params.pos_y_kp) > 1.0e-6f) {
        pos_ref_y += (coax_ctrl_params.vel_y_kd * reference->vy_m_s +
                      reference->ay_m_s2) / coax_ctrl_params.pos_y_kp;
    }
    if (fabsf(coax_ctrl_params.pos_z_kp) > 1.0e-6f) {
        pos_ref_z += (coax_ctrl_params.vel_z_kd * reference->vz_m_s +
                      reference->az_m_s2) / coax_ctrl_params.pos_z_kp;
    }

    ref_cmd[0] = pos_ref_x;
    ref_cmd[1] = pos_ref_y;
    ref_cmd[2] = pos_ref_z;
    ref_cmd[3] = reference->yaw_rad;

    debug.pos_p_m_s2[0] = coax_ctrl_params.pos_x_kp * (pos_ref_x - attitude->x_m);
    debug.pos_p_m_s2[1] = coax_ctrl_params.pos_y_kp * (pos_ref_y - attitude->y_m);
    debug.pos_p_m_s2[2] = coax_ctrl_params.pos_z_kp * (pos_ref_z - attitude->z_m);
    debug.vel_d_m_s2[0] = -coax_ctrl_params.vel_x_kd * attitude->vx_m_s;
    debug.vel_d_m_s2[1] = -coax_ctrl_params.vel_y_kd * attitude->vy_m_s;
    debug.vel_d_m_s2[2] = -coax_ctrl_params.vel_z_kd * attitude->vz_m_s;
    debug.accel_out_m_s2[0] =
        coax_ctrl_clamp_f32(debug.pos_p_m_s2[0] + debug.vel_d_m_s2[0],
                            -coax_ctrl_params.accel_xy_limit_m_s2,
                             coax_ctrl_params.accel_xy_limit_m_s2);
    debug.accel_out_m_s2[1] =
        coax_ctrl_clamp_f32(debug.pos_p_m_s2[1] + debug.vel_d_m_s2[1],
                            -coax_ctrl_params.accel_xy_limit_m_s2,
                             coax_ctrl_params.accel_xy_limit_m_s2);
    debug.accel_out_m_s2[2] =
        coax_ctrl_clamp_f32(debug.pos_p_m_s2[2] + debug.vel_d_m_s2[2],
                            -coax_ctrl_params.accel_z_limit_m_s2,
                             coax_ctrl_params.accel_z_limit_m_s2);
    {
        float yaw_err = reference->yaw_rad - attitude->yaw_rad;

        while (yaw_err > DRV_COAX_CTRL_PI) {
            yaw_err -= 2.0f * DRV_COAX_CTRL_PI;
        }
        while (yaw_err < -DRV_COAX_CTRL_PI) {
            yaw_err += 2.0f * DRV_COAX_CTRL_PI;
        }
        debug.yaw_angle_p_rad_s =
            coax_ctrl_clamp_f32(coax_ctrl_params.yaw_angle_kp * yaw_err,
                                -coax_ctrl_params.yaw_rate_limit_rad_s,
                                 coax_ctrl_params.yaw_rate_limit_rad_s);
        debug.yaw_rate_d_rad_s =
            -coax_ctrl_params.yaw_rate_kd * attitude->gyro_z_rad_s;
    }

    coax_tiltrotor_controller_codegen(x_rb, ref_cmd, &coax_ctrl_params, cmd);
    coax_ctrl_compute_pure_damping_tilt(attitude, reference,
                                        &alpha_rad, &beta_rad, &debug);

    output->omega_upper = cmd[0];
    output->omega_lower = cmd[1];
    output->alpha_rad = alpha_rad;
    output->beta_rad = beta_rad;
    debug.omega_cmd_rad_s[0] = output->omega_upper;
    debug.omega_cmd_rad_s[1] = output->omega_lower;
    debug.total_force_n = coax_ctrl_params.thrust_coeff_n_per_rad2 *
        ((output->omega_upper * output->omega_upper) +
         (output->omega_lower * output->omega_lower));
    debug.yaw_torque_cmd = coax_ctrl_params.yaw_torque_coeff_n_m_per_rad2 *
        ((output->omega_lower * output->omega_lower) -
         (output->omega_upper * output->omega_upper));
    coax_ctrl_last_debug = debug;
    output->servo_alpha_us = DRV_COAX_CTRL_AlphaTiltRadToServoPulse(
        output->alpha_rad * DRV_COAX_CTRL_SERVO_ALPHA_SIGN);
    output->servo_beta_us = DRV_COAX_CTRL_BetaTiltRadToServoPulse(
        output->beta_rad * DRV_COAX_CTRL_SERVO_BETA_SIGN);
}

void DRV_COAX_CTRL_GetLastDebug(DRV_COAX_CTRL_Debug *debug)
{
    if (debug == NULL) {
        return;
    }

    *debug = coax_ctrl_last_debug;
}

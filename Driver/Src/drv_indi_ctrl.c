#include "drv_indi_ctrl.h"

#include "drv_airframe_model.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define DRV_INDI_DEFAULT_DT_SEC                 0.002f
#define DRV_INDI_MIN_DT_SEC                     0.00025f
#define DRV_INDI_MAX_DT_SEC                     0.020f
#define DRV_INDI_MIN_FORCE_N                    0.10f
#define DRV_INDI_MIN_LEVER_ARM_M                0.001f
#define DRV_INDI_MIN_INERTIA_KG_M2              1.0e-6f
#define DRV_INDI_MIN_EFFECTIVENESS_RAD_S2_RAD   1.0e-3f

static float indi_clamp_f32(float value, float lo, float hi)
{
    if (value < lo) { return lo; }
    if (value > hi) { return hi; }
    return value;
}

static float indi_nonzero_sign(float value)
{
    return (value < 0.0f) ? -1.0f : 1.0f;
}

static float indi_dt(float dt_sec)
{
    if ((dt_sec >= DRV_INDI_MIN_DT_SEC) &&
        (dt_sec <= DRV_INDI_MAX_DT_SEC)) {
        return dt_sec;
    }
    return DRV_INDI_DEFAULT_DT_SEC;
}

static uint8_t indi_config_finite(const DRV_INDI_Config *config)
{
    return (isfinite(config->enable) &&
            isfinite(config->roll_inertia_kg_m2) &&
            isfinite(config->pitch_inertia_kg_m2) &&
            isfinite(config->roll_attitude_kp_rad_s2_per_rad) &&
            isfinite(config->pitch_attitude_kp_rad_s2_per_rad) &&
            isfinite(config->roll_rate_kd_rad_s2_per_rad_s) &&
            isfinite(config->pitch_rate_kd_rad_s2_per_rad_s) &&
            isfinite(config->angular_accel_lpf_alpha) &&
            isfinite(config->correction_limit_rad) &&
            isfinite(config->increment_limit_rad) &&
            isfinite(config->correction_leak_hz) &&
            isfinite(config->roll_effectiveness_sign) &&
            isfinite(config->pitch_effectiveness_sign)) ? 1U : 0U;
}

static uint8_t indi_input_finite(const DRV_INDI_Input *input)
{
    /* dt_sec is intentionally omitted: indi_dt() safely falls back to 2 ms. */
    return (isfinite(input->roll_rad) &&
            isfinite(input->pitch_rad) &&
            isfinite(input->roll_ref_rad) &&
            isfinite(input->pitch_ref_rad) &&
            isfinite(input->gyro_x_rad_s) &&
            isfinite(input->gyro_y_rad_s) &&
            isfinite(input->base_alpha_rad) &&
            isfinite(input->base_beta_rad) &&
            isfinite(input->total_force_n) &&
            isfinite(input->tilt_lever_arm_m)) ? 1U : 0U;
}

void DRV_INDI_DefaultConfig(DRV_INDI_Config *config)
{
    if (config == NULL) {
        return;
    }

    config->enable = 1.0f;
    config->roll_inertia_kg_m2 = DRV_AIRFRAME_IXX_KGM2;
    config->pitch_inertia_kg_m2 = DRV_AIRFRAME_IYY_KGM2;
    config->roll_attitude_kp_rad_s2_per_rad = 16.0f;
    config->pitch_attitude_kp_rad_s2_per_rad = 16.0f;
    config->roll_rate_kd_rad_s2_per_rad_s = 6.4f;
    config->pitch_rate_kd_rad_s2_per_rad_s = 6.4f;
    config->angular_accel_lpf_alpha = 0.85f;
    config->correction_limit_rad = 0.174532925f;
    config->increment_limit_rad = 0.017453293f;
    config->correction_leak_hz = 0.5f;
    config->roll_effectiveness_sign = 1.0f;
    config->pitch_effectiveness_sign = 1.0f;
}

void DRV_INDI_Reset(DRV_INDI_State *state)
{
    if (state == NULL) {
        return;
    }
    memset(state, 0, sizeof(*state));
}

void DRV_INDI_Step(DRV_INDI_State *state,
                   const DRV_INDI_Config *config,
                   const DRV_INDI_Input *input,
                   DRV_INDI_Output *output)
{
    float dt;
    float alpha;
    float force_n;
    float lever_arm_m;
    float roll_inertia;
    float pitch_inertia;
    float roll_effectiveness;
    float pitch_effectiveness;
    float roll_virtual_accel;
    float pitch_virtual_accel;
    float roll_increment;
    float pitch_increment;
    float correction_limit;
    float increment_limit;
    float leak_factor;

    if ((state == NULL) || (config == NULL) ||
        (input == NULL) || (output == NULL)) {
        return;
    }

    memset(output, 0, sizeof(*output));
    output->alpha_rad = isfinite(input->base_alpha_rad) ?
                        input->base_alpha_rad : 0.0f;
    output->beta_rad = isfinite(input->base_beta_rad) ?
                       input->base_beta_rad : 0.0f;

    /*
     * A single non-finite sensor/model value must not poison the differentiator
     * or the accumulated correction. Fall back to the finite base command and
     * require one clean sample to initialize again.
     */
    if ((indi_config_finite(config) == 0U) ||
        (indi_input_finite(input) == 0U)) {
        DRV_INDI_Reset(state);
        return;
    }

    if (config->enable < 0.5f) {
        DRV_INDI_Reset(state);
        return;
    }

    dt = indi_dt(input->dt_sec);
    alpha = indi_clamp_f32(config->angular_accel_lpf_alpha, 0.0f, 1.0f);
    correction_limit = fabsf(config->correction_limit_rad);
    increment_limit = fabsf(config->increment_limit_rad);

    if (state->initialized == 0U) {
        state->previous_rate_rad_s[0] = input->gyro_x_rad_s;
        state->previous_rate_rad_s[1] = input->gyro_y_rad_s;
        state->initialized = 1U;
    } else {
        const float raw_roll_accel =
            (input->gyro_x_rad_s - state->previous_rate_rad_s[0]) / dt;
        const float raw_pitch_accel =
            (input->gyro_y_rad_s - state->previous_rate_rad_s[1]) / dt;

        state->angular_accel_rad_s2[0] =
            alpha * state->angular_accel_rad_s2[0] +
            (1.0f - alpha) * raw_roll_accel;
        state->angular_accel_rad_s2[1] =
            alpha * state->angular_accel_rad_s2[1] +
            (1.0f - alpha) * raw_pitch_accel;
        state->previous_rate_rad_s[0] = input->gyro_x_rad_s;
        state->previous_rate_rad_s[1] = input->gyro_y_rad_s;
    }

    roll_virtual_accel =
        config->roll_attitude_kp_rad_s2_per_rad *
        (input->roll_ref_rad - input->roll_rad) -
        config->roll_rate_kd_rad_s2_per_rad_s * input->gyro_x_rad_s;
    pitch_virtual_accel =
        config->pitch_attitude_kp_rad_s2_per_rad *
        (input->pitch_ref_rad - input->pitch_rad) -
        config->pitch_rate_kd_rad_s2_per_rad_s * input->gyro_y_rad_s;

    force_n = fmaxf(fabsf(input->total_force_n), DRV_INDI_MIN_FORCE_N);
    lever_arm_m = fmaxf(fabsf(input->tilt_lever_arm_m),
                        DRV_INDI_MIN_LEVER_ARM_M);
    roll_inertia = fmaxf(fabsf(config->roll_inertia_kg_m2),
                         DRV_INDI_MIN_INERTIA_KG_M2);
    pitch_inertia = fmaxf(fabsf(config->pitch_inertia_kg_m2),
                          DRV_INDI_MIN_INERTIA_KG_M2);
    roll_effectiveness = indi_nonzero_sign(config->roll_effectiveness_sign) *
                         force_n * lever_arm_m / roll_inertia;
    pitch_effectiveness = indi_nonzero_sign(config->pitch_effectiveness_sign) *
                          force_n * lever_arm_m / pitch_inertia;

    if (fabsf(roll_effectiveness) < DRV_INDI_MIN_EFFECTIVENESS_RAD_S2_RAD) {
        roll_effectiveness =
            indi_nonzero_sign(roll_effectiveness) *
            DRV_INDI_MIN_EFFECTIVENESS_RAD_S2_RAD;
    }
    if (fabsf(pitch_effectiveness) < DRV_INDI_MIN_EFFECTIVENESS_RAD_S2_RAD) {
        pitch_effectiveness =
            indi_nonzero_sign(pitch_effectiveness) *
            DRV_INDI_MIN_EFFECTIVENESS_RAD_S2_RAD;
    }

    roll_increment =
        (roll_virtual_accel - state->angular_accel_rad_s2[0]) /
        roll_effectiveness;
    pitch_increment =
        (pitch_virtual_accel - state->angular_accel_rad_s2[1]) /
        pitch_effectiveness;
    roll_increment = indi_clamp_f32(roll_increment,
                                    -increment_limit,
                                     increment_limit);
    pitch_increment = indi_clamp_f32(pitch_increment,
                                     -increment_limit,
                                      increment_limit);

    leak_factor = indi_clamp_f32(1.0f -
                                 fabsf(config->correction_leak_hz) * dt,
                                 0.0f,
                                 1.0f);
    state->correction_rad[0] =
        indi_clamp_f32(state->correction_rad[0] * leak_factor + roll_increment,
                       -correction_limit,
                        correction_limit);
    state->correction_rad[1] =
        indi_clamp_f32(state->correction_rad[1] * leak_factor + pitch_increment,
                       -correction_limit,
                        correction_limit);

    output->alpha_rad = input->base_alpha_rad + state->correction_rad[1];
    output->beta_rad = input->base_beta_rad + state->correction_rad[0];
    output->angular_accel_rad_s2[0] = state->angular_accel_rad_s2[0];
    output->angular_accel_rad_s2[1] = state->angular_accel_rad_s2[1];
    output->virtual_accel_rad_s2[0] = roll_virtual_accel;
    output->virtual_accel_rad_s2[1] = pitch_virtual_accel;
    output->effectiveness_rad_s2_per_rad[0] = roll_effectiveness;
    output->effectiveness_rad_s2_per_rad[1] = pitch_effectiveness;
    output->correction_rad[0] = state->correction_rad[0];
    output->correction_rad[1] = state->correction_rad[1];
    output->active = 1U;
}

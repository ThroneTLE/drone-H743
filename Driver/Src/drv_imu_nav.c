#include "drv_imu_nav.h"

#include <math.h>
#include <string.h>

#define IMU_NAV_GRAVITY_REF_ALPHA 0.995f
#define IMU_NAV_MIN_ACCEL_NORM_G  0.10f
#define IMU_NAV_MIN_AXIS_NORM     1.0e-4f

static float imu_nav_clamp_f32(float value, float lo, float hi)
{
    if (value < lo) { return lo; }
    if (value > hi) { return hi; }
    return value;
}

static float imu_nav_dot3(const float a[3], const float b[3])
{
    return (a[0] * b[0]) + (a[1] * b[1]) + (a[2] * b[2]);
}

static void imu_nav_cross3(const float a[3], const float b[3], float out[3])
{
    out[0] = (a[1] * b[2]) - (a[2] * b[1]);
    out[1] = (a[2] * b[0]) - (a[0] * b[2]);
    out[2] = (a[0] * b[1]) - (a[1] * b[0]);
}

static uint8_t imu_nav_normalize3(float v[3])
{
    const float norm = sqrtf(imu_nav_dot3(v, v));
    if (norm <= IMU_NAV_MIN_AXIS_NORM) {
        return 0U;
    }

    v[0] /= norm;
    v[1] /= norm;
    v[2] /= norm;
    return 1U;
}

static float imu_nav_dt(float dt_sec)
{
    if ((dt_sec > 0.0f) && (dt_sec <= 0.02f)) {
        return dt_sec;
    }
    return 0.001f;
}

static void imu_nav_update_level_reference(DRV_IMU_NAV_State *state,
                                           const DRV_IMU_NAV_Input *input)
{
    float z_body[3];
    float norm_g;

    if ((state == NULL) || (input == NULL)) {
        return;
    }

    /*
     * Specific force points up when the airframe is at rest, so the raw
     * accelerometer vector is already the local-level up axis in body
     * coordinates.  (Before the seam 2 migration this was negated to build a
     * down axis for a (forward, right, down) frame.)
     */
    z_body[0] = input->accel_x_g;
    z_body[1] = input->accel_y_g;
    z_body[2] = input->accel_z_g;
    norm_g = sqrtf(imu_nav_dot3(z_body, z_body));
    if (norm_g < IMU_NAV_MIN_ACCEL_NORM_G) {
        return;
    }

    z_body[0] /= norm_g;
    z_body[1] /= norm_g;
    z_body[2] /= norm_g;

    if (state->level_ready == 0U) {
        state->level_z_body_unit[0] = z_body[0];
        state->level_z_body_unit[1] = z_body[1];
        state->level_z_body_unit[2] = z_body[2];
        state->level_ready = 1U;
        return;
    }

    state->level_z_body_unit[0] =
        (IMU_NAV_GRAVITY_REF_ALPHA * state->level_z_body_unit[0]) +
        ((1.0f - IMU_NAV_GRAVITY_REF_ALPHA) * z_body[0]);
    state->level_z_body_unit[1] =
        (IMU_NAV_GRAVITY_REF_ALPHA * state->level_z_body_unit[1]) +
        ((1.0f - IMU_NAV_GRAVITY_REF_ALPHA) * z_body[1]);
    state->level_z_body_unit[2] =
        (IMU_NAV_GRAVITY_REF_ALPHA * state->level_z_body_unit[2]) +
        ((1.0f - IMU_NAV_GRAVITY_REF_ALPHA) * z_body[2]);

    if (imu_nav_normalize3(state->level_z_body_unit) == 0U) {
        state->level_ready = 0U;
    }
}

static void imu_nav_acc_body_to_nav(const DRV_IMU_NAV_Input *input,
                                    const float level_z_body_unit[3],
                                    float acc_nav_m_s2[3])
{
    const float gravity = (input->gravity_m_s2 > 0.0f) ? input->gravity_m_s2 : 9.80665f;
    const float acc_body_m_s2[3] = {
        input->accel_x_g * gravity,
        input->accel_y_g * gravity,
        input->accel_z_g * gravity,
    };
    float z_body[3] = {
        level_z_body_unit[0],
        level_z_body_unit[1],
        level_z_body_unit[2],
    };
    float x_body[3] = {1.0f, 0.0f, 0.0f};
    float y_body[3];
    float x_dot_z;

    /*
     * Degenerate fallback: assume the airframe is level, i.e. up is body +Z.
     * This constant was already +Z before the seam 2 migration, when the axis
     * it fed was supposed to be *down*; the vertical channel then produced
     * +2g instead of 0 for a level sample.  Migrating the frame makes the
     * fallback consistent with the axis it fills in.  Reached only when the
     * smoothed gravity reference fails to normalise.
     */
    if (imu_nav_normalize3(z_body) == 0U) {
        z_body[0] = 0.0f;
        z_body[1] = 0.0f;
        z_body[2] = 1.0f;
    }

    x_dot_z = imu_nav_dot3(x_body, z_body);
    x_body[0] -= x_dot_z * z_body[0];
    x_body[1] -= x_dot_z * z_body[1];
    x_body[2] -= x_dot_z * z_body[2];
    if (imu_nav_normalize3(x_body) == 0U) {
        x_body[0] = 0.0f;
        x_body[1] = 1.0f;
        x_body[2] = 0.0f;
        x_dot_z = imu_nav_dot3(x_body, z_body);
        x_body[0] -= x_dot_z * z_body[0];
        x_body[1] -= x_dot_z * z_body[1];
        x_body[2] -= x_dot_z * z_body[2];
        (void)imu_nav_normalize3(x_body);
    }

    /* z is up and x is forward, so cross(z, x) is left: the frame is FLU. */
    imu_nav_cross3(z_body, x_body, y_body);

    /*
     * Accelerometers measure specific force.  At rest, body specific force is
     * +g along up, so dot(z_up_body, f_body) is +g and -gravity gives 0.
     */
    acc_nav_m_s2[0] = imu_nav_dot3(x_body, acc_body_m_s2);
    acc_nav_m_s2[1] = imu_nav_dot3(y_body, acc_body_m_s2);
    acc_nav_m_s2[2] = imu_nav_dot3(z_body, acc_body_m_s2) - gravity;
}

void DRV_IMU_NAV_Reset(DRV_IMU_NAV_State *state)
{
    if (state == NULL) {
        return;
    }

    memset(state, 0, sizeof(*state));
}

void DRV_IMU_NAV_CaptureBias(DRV_IMU_NAV_State *state,
                             const DRV_IMU_NAV_Input *input)
{
    if ((state == NULL) || (input == NULL)) {
        return;
    }

    imu_nav_update_level_reference(state, input);
    imu_nav_acc_body_to_nav(input, state->level_z_body_unit, state->acc_bias_nav_m_s2);
    state->bias_ready = 1U;
}

void DRV_IMU_NAV_Update(DRV_IMU_NAV_State *state,
                        const DRV_IMU_NAV_Input *input)
{
    float raw_nav[3];
    float dt_sec;
    float alpha;
    float leak;

    if ((state == NULL) || (input == NULL)) {
        return;
    }

    dt_sec = imu_nav_dt(input->dt_sec);
    alpha = imu_nav_clamp_f32(input->accel_lpf_alpha, 0.0f, 1.0f);
    leak = imu_nav_clamp_f32(input->velocity_leak_rate_hz, 0.0f, 5.0f);

    imu_nav_update_level_reference(state, input);
    imu_nav_acc_body_to_nav(input, state->level_z_body_unit, raw_nav);

    for (uint32_t axis = 0U; axis < 3U; ++axis) {
        float acc = raw_nav[axis];
        if (state->bias_ready != 0U) {
            acc -= state->acc_bias_nav_m_s2[axis];
        }

        state->acc_nav_m_s2[axis] =
            alpha * state->acc_nav_m_s2[axis] + (1.0f - alpha) * acc;
        state->vel_m_s[axis] += state->acc_nav_m_s2[axis] * dt_sec;
        state->vel_m_s[axis] -= state->vel_m_s[axis] * leak * dt_sec;
    }

    state->accel_lpf_alpha = alpha;
    state->velocity_leak_rate_hz = leak;
}

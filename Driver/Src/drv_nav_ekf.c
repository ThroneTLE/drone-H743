#include "drv_nav_ekf.h"

#include <math.h>
#include <string.h>

#define NAV_EKF_STATE_LEN 4U
#define NAV_EKF_MIN_DT_SEC 1.0e-5f
#define NAV_EKF_MIN_VARIANCE 1.0e-8f
#define NAV_EKF_MAX_VARIANCE 1.0e6f
#define NAV_EKF_MIN_FLOW_NOISE_M_S 0.02f

static float nav_ekf_clamp_f32(float value, float lo, float hi)
{
    if (value < lo) { return lo; }
    if (value > hi) { return hi; }
    return value;
}

static float nav_ekf_square(float value)
{
    return value * value;
}

static float nav_ekf_dt(const DRV_NAV_EKF_State *state, float dt_sec)
{
    float max_dt = 0.02f;

    if (state != NULL && state->config.max_dt_sec > 0.0f) {
        max_dt = state->config.max_dt_sec;
    }

    if ((dt_sec >= NAV_EKF_MIN_DT_SEC) && (dt_sec <= max_dt)) {
        return dt_sec;
    }
    return 0.001f;
}

static void nav_ekf_symmetrize_and_bound(DRV_NAV_EKF_State *state)
{
    for (uint32_t i = 0U; i < NAV_EKF_STATE_LEN; ++i) {
        for (uint32_t j = i + 1U; j < NAV_EKF_STATE_LEN; ++j) {
            const float sym = 0.5f *
                (state->covariance[i][j] + state->covariance[j][i]);
            state->covariance[i][j] = sym;
            state->covariance[j][i] = sym;
        }
    }

    for (uint32_t i = 0U; i < NAV_EKF_STATE_LEN; ++i) {
        state->covariance[i][i] =
            nav_ekf_clamp_f32(state->covariance[i][i],
                              NAV_EKF_MIN_VARIANCE,
                              NAV_EKF_MAX_VARIANCE);
    }
}

void DRV_NAV_EKF_DefaultConfig(DRV_NAV_EKF_Config *config)
{
    if (config == NULL) {
        return;
    }

    config->process_accel_noise_m_s2 = 1.5f;
    config->bias_random_walk_m_s3 = 0.08f;
    config->flow_noise_m_s = 0.25f;
    config->flow_gate_nis = 9.21f;
    config->initial_velocity_variance = 0.25f;
    config->initial_bias_variance = 0.50f;
    config->max_dt_sec = 0.02f;
    config->max_velocity_m_s = 2.50f;
    /* 默认不泄漏：泄漏不在协方差模型里，光流正常时它只会把速度往 0 拉。
     * 2026-10-01 钢尺实测原为 1.5 Hz 时导航位移只有真值 0.75~0.8；光流丢失的
     * 收敛由 svc_flow_nav 的 decay 分支负责，不靠这里。机制保留供 A/B。 */
    config->predict_leak_hz = 0.0f;
}

void DRV_NAV_EKF_Reset(DRV_NAV_EKF_State *state,
                       const DRV_NAV_EKF_Config *config)
{
    DRV_NAV_EKF_Config effective_config;

    if (state == NULL) {
        return;
    }

    if (config == NULL) {
        DRV_NAV_EKF_DefaultConfig(&effective_config);
    } else {
        effective_config = *config;
    }

    memset(state, 0, sizeof(*state));
    state->config = effective_config;
    state->covariance[0][0] =
        nav_ekf_clamp_f32(effective_config.initial_velocity_variance,
                          NAV_EKF_MIN_VARIANCE,
                          NAV_EKF_MAX_VARIANCE);
    state->covariance[1][1] = state->covariance[0][0];
    state->covariance[2][2] =
        nav_ekf_clamp_f32(effective_config.initial_bias_variance,
                          NAV_EKF_MIN_VARIANCE,
                          NAV_EKF_MAX_VARIANCE);
    state->covariance[3][3] = state->covariance[2][2];
    state->last_gate_nis = effective_config.flow_gate_nis;
    state->last_flow_noise_m_s = effective_config.flow_noise_m_s;
    state->initialized = 1U;
}

void DRV_NAV_EKF_Predict(DRV_NAV_EKF_State *state,
                         float acc_x_m_s2,
                         float acc_y_m_s2,
                         float dt_sec)
{
    float p[4][4];
    float fp[4][4];
    float p_new[4][4];
    float q_vel;
    float q_bias;
    float dt;

    if (state == NULL) {
        return;
    }

    if (state->initialized == 0U) {
        DRV_NAV_EKF_Reset(state, NULL);
    }

    dt = nav_ekf_dt(state, dt_sec);
    memcpy(p, state->covariance, sizeof(p));

    state->vel_m_s[0] += (acc_x_m_s2 - state->accel_bias_m_s2[0]) * dt;
    state->vel_m_s[1] += (acc_y_m_s2 - state->accel_bias_m_s2[1]) * dt;
    if (state->config.predict_leak_hz > 0.0f) {
        float leak = 1.0f - (state->config.predict_leak_hz * dt);
        leak = nav_ekf_clamp_f32(leak, 0.0f, 1.0f);
        state->vel_m_s[0] *= leak;
        state->vel_m_s[1] *= leak;
    }
    if (state->config.max_velocity_m_s > 0.0f) {
        state->vel_m_s[0] =
            nav_ekf_clamp_f32(state->vel_m_s[0],
                              -state->config.max_velocity_m_s,
                               state->config.max_velocity_m_s);
        state->vel_m_s[1] =
            nav_ekf_clamp_f32(state->vel_m_s[1],
                              -state->config.max_velocity_m_s,
                               state->config.max_velocity_m_s);
    }

    for (uint32_t j = 0U; j < NAV_EKF_STATE_LEN; ++j) {
        fp[0][j] = p[0][j] - dt * p[2][j];
        fp[1][j] = p[1][j] - dt * p[3][j];
        fp[2][j] = p[2][j];
        fp[3][j] = p[3][j];
    }

    for (uint32_t i = 0U; i < NAV_EKF_STATE_LEN; ++i) {
        p_new[i][0] = fp[i][0] - dt * fp[i][2];
        p_new[i][1] = fp[i][1] - dt * fp[i][3];
        p_new[i][2] = fp[i][2];
        p_new[i][3] = fp[i][3];
    }

    q_vel = nav_ekf_square(state->config.process_accel_noise_m_s2 * dt);
    q_bias = nav_ekf_square(state->config.bias_random_walk_m_s3) * dt;
    p_new[0][0] += q_vel;
    p_new[1][1] += q_vel;
    p_new[2][2] += q_bias;
    p_new[3][3] += q_bias;

    memcpy(state->covariance, p_new, sizeof(state->covariance));
    nav_ekf_symmetrize_and_bound(state);
    state->last_dt_sec = dt;
    state->predict_count++;
}

uint8_t DRV_NAV_EKF_FuseFlow(DRV_NAV_EKF_State *state,
                             float flow_vx_m_s,
                             float flow_vy_m_s,
                             uint8_t flow_valid,
                             uint32_t flow_sample_ms,
                             float flow_noise_m_s)
{
    float innovation[2];
    float s00;
    float s01;
    float s10;
    float s11;
    float det;
    float inv_s00;
    float inv_s01;
    float inv_s10;
    float inv_s11;
    float nis;
    float k[4][2];
    float p_old[4][4];
    float p_new[4][4];
    float noise;
    float r;

    if (state == NULL) {
        return 0U;
    }

    if ((state->initialized == 0U) || (flow_valid == 0U) ||
        (flow_sample_ms == 0U) ||
        (flow_sample_ms == state->last_flow_sample_ms)) {
        state->flow_skip_count++;
        return 0U;
    }

    state->last_flow_sample_ms = flow_sample_ms;
    noise = (flow_noise_m_s > 0.0f) ? flow_noise_m_s :
            state->config.flow_noise_m_s;
    noise = nav_ekf_clamp_f32(noise, NAV_EKF_MIN_FLOW_NOISE_M_S, 5.0f);
    r = nav_ekf_square(noise);

    innovation[0] = flow_vx_m_s - state->vel_m_s[0];
    innovation[1] = flow_vy_m_s - state->vel_m_s[1];

    s00 = state->covariance[0][0] + r;
    s01 = state->covariance[0][1];
    s10 = state->covariance[1][0];
    s11 = state->covariance[1][1] + r;
    det = (s00 * s11) - (s01 * s10);
    if (fabsf(det) < 1.0e-12f) {
        state->flow_reject_count++;
        return 0U;
    }

    inv_s00 = s11 / det;
    inv_s01 = -s01 / det;
    inv_s10 = -s10 / det;
    inv_s11 = s00 / det;
    nis = innovation[0] * (inv_s00 * innovation[0] +
                           inv_s01 * innovation[1]) +
          innovation[1] * (inv_s10 * innovation[0] +
                           inv_s11 * innovation[1]);

    state->last_innovation_m_s[0] = innovation[0];
    state->last_innovation_m_s[1] = innovation[1];
    state->last_nis = nis;
    state->last_gate_nis = state->config.flow_gate_nis;
    state->last_flow_noise_m_s = noise;

    if ((state->config.flow_gate_nis > 0.0f) &&
        (nis > state->config.flow_gate_nis)) {
        state->flow_reject_count++;
        return 0U;
    }

    memcpy(p_old, state->covariance, sizeof(p_old));
    for (uint32_t i = 0U; i < NAV_EKF_STATE_LEN; ++i) {
        k[i][0] = (p_old[i][0] * inv_s00) + (p_old[i][1] * inv_s10);
        k[i][1] = (p_old[i][0] * inv_s01) + (p_old[i][1] * inv_s11);
    }

    state->vel_m_s[0] += k[0][0] * innovation[0] + k[0][1] * innovation[1];
    state->vel_m_s[1] += k[1][0] * innovation[0] + k[1][1] * innovation[1];
    state->accel_bias_m_s2[0] +=
        k[2][0] * innovation[0] + k[2][1] * innovation[1];
    state->accel_bias_m_s2[1] +=
        k[3][0] * innovation[0] + k[3][1] * innovation[1];

    for (uint32_t i = 0U; i < NAV_EKF_STATE_LEN; ++i) {
        for (uint32_t j = 0U; j < NAV_EKF_STATE_LEN; ++j) {
            p_new[i][j] = p_old[i][j] -
                          k[i][0] * p_old[0][j] -
                          k[i][1] * p_old[1][j];
        }
    }

    memcpy(state->covariance, p_new, sizeof(state->covariance));
    nav_ekf_symmetrize_and_bound(state);
    state->flow_update_count++;
    state->last_flow_update_ms = flow_sample_ms;
    return 1U;
}

void DRV_NAV_EKF_GetVelocity(const DRV_NAV_EKF_State *state,
                             float *vx_m_s,
                             float *vy_m_s)
{
    if (state == NULL) {
        return;
    }

    if (vx_m_s != NULL) {
        *vx_m_s = state->vel_m_s[0];
    }
    if (vy_m_s != NULL) {
        *vy_m_s = state->vel_m_s[1];
    }
}

void DRV_NAV_EKF_GetDiagnostics(const DRV_NAV_EKF_State *state,
                                DRV_NAV_EKF_Diagnostics *diagnostics)
{
    if ((state == NULL) || (diagnostics == NULL)) {
        return;
    }

    memset(diagnostics, 0, sizeof(*diagnostics));
    diagnostics->vel_m_s[0] = state->vel_m_s[0];
    diagnostics->vel_m_s[1] = state->vel_m_s[1];
    diagnostics->accel_bias_m_s2[0] = state->accel_bias_m_s2[0];
    diagnostics->accel_bias_m_s2[1] = state->accel_bias_m_s2[1];
    for (uint32_t i = 0U; i < NAV_EKF_STATE_LEN; ++i) {
        diagnostics->covariance_diag[i] = state->covariance[i][i];
    }
    diagnostics->predict_count = state->predict_count;
    diagnostics->flow_update_count = state->flow_update_count;
    diagnostics->flow_reject_count = state->flow_reject_count;
    diagnostics->flow_skip_count = state->flow_skip_count;
    diagnostics->last_flow_update_ms = state->last_flow_update_ms;
    diagnostics->last_dt_sec = state->last_dt_sec;
    diagnostics->last_flow_noise_m_s = state->last_flow_noise_m_s;
    diagnostics->last_innovation_m_s[0] = state->last_innovation_m_s[0];
    diagnostics->last_innovation_m_s[1] = state->last_innovation_m_s[1];
    diagnostics->last_nis = state->last_nis;
    diagnostics->last_gate_nis = state->last_gate_nis;
    diagnostics->initialized = state->initialized;
}

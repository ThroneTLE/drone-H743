#include "drv_hover_thrust_est.h"

#include <math.h>
#include <string.h>

#define HOVEREST_FALLBACK_INIT_N 10.0f

static float hover_clamp(float value, float low, float high)
{
    if (value < low) {
        return low;
    }
    return (value > high) ? high : value;
}

static uint8_t hover_finite(float value)
{
    return (isfinite(value) != 0) ? 1U : 0U;
}

void DRV_HoverEst_DefaultParams(DRV_HoverEstParams *params)
{
    if (params == NULL) {
        return;
    }
    params->init_std_n = DRV_HOVEREST_INIT_STD_DEFAULT_N;
    params->process_std_n_sqrt_s = DRV_HOVEREST_PROCESS_STD_DEFAULT;
    params->meas_std_init = DRV_HOVEREST_MEAS_STD_INIT_DEFAULT;
    params->meas_std_min = DRV_HOVEREST_MEAS_STD_MIN_DEFAULT;
    params->meas_std_max = DRV_HOVEREST_MEAS_STD_MAX_DEFAULT;
    params->noise_alpha = DRV_HOVEREST_NOISE_ALPHA_DEFAULT;
    params->gate_sigma = DRV_HOVEREST_GATE_SIGMA_DEFAULT;
    params->reject_inflate = (uint16_t)DRV_HOVEREST_REJECT_INFLATE_DEFAULT;
    params->inflate_std_n = DRV_HOVEREST_INFLATE_STD_DEFAULT_N;
    params->thrust_lag_s = DRV_HOVEREST_THRUST_LAG_DEFAULT_S;
    params->accel_lpf_s = DRV_HOVEREST_ACCEL_LPF_DEFAULT_S;
    params->converged_std_n = DRV_HOVEREST_CONV_STD_DEFAULT_N;
    params->converged_samples = DRV_HOVEREST_CONV_SAMPLES_DEFAULT;
}

uint8_t DRV_HoverEst_ParamsValid(const DRV_HoverEstParams *params)
{
    if (params == NULL) {
        return 0U;
    }
    if ((hover_finite(params->init_std_n) == 0U) || (params->init_std_n <= 0.0f) ||
        (params->init_std_n > 20.0f)) {
        return 0U;
    }
    if ((hover_finite(params->process_std_n_sqrt_s) == 0U) ||
        (params->process_std_n_sqrt_s < 0.0f) || (params->process_std_n_sqrt_s > 2.0f)) {
        return 0U;
    }
    if ((hover_finite(params->meas_std_min) == 0U) || (hover_finite(params->meas_std_max) == 0U) ||
        (hover_finite(params->meas_std_init) == 0U) || (params->meas_std_min <= 0.0f) ||
        (params->meas_std_max < params->meas_std_min) ||
        (params->meas_std_init < params->meas_std_min) ||
        (params->meas_std_init > params->meas_std_max)) {
        return 0U;
    }
    if ((hover_finite(params->noise_alpha) == 0U) || (params->noise_alpha < 0.0f) ||
        (params->noise_alpha > 1.0f)) {
        return 0U;
    }
    if ((hover_finite(params->gate_sigma) == 0U) || (params->gate_sigma < 1.0f) ||
        (params->gate_sigma > 20.0f)) {
        return 0U;
    }
    if ((hover_finite(params->inflate_std_n) == 0U) || (params->inflate_std_n <= 0.0f) ||
        (params->inflate_std_n > 20.0f)) {
        return 0U;
    }
    if ((hover_finite(params->thrust_lag_s) == 0U) || (params->thrust_lag_s < 0.0f) ||
        (params->thrust_lag_s > 1.0f) || (hover_finite(params->accel_lpf_s) == 0U) ||
        (params->accel_lpf_s < 0.0f) || (params->accel_lpf_s > 1.0f)) {
        return 0U;
    }
    if ((hover_finite(params->converged_std_n) == 0U) || (params->converged_std_n <= 0.0f)) {
        return 0U;
    }
    return 1U;
}

static void hover_reset_state(DRV_HoverEst *est)
{
    est->h_n = est->init_n;
    est->p_n2 = est->params.init_std_n * est->params.init_std_n;
    est->r_var = est->params.meas_std_init * est->params.meas_std_init;
    est->thrust_f_n = 0.0f;
    est->accel_f_m_s2 = 0.0f;
    est->last_innov_m_s2 = 0.0f;
    est->samples = 0U;
    est->rejected = 0U;
    est->reject_run = 0U;
    est->filt_primed = 0U;
    est->learning = 0U;
}

uint8_t DRV_HoverEst_Init(DRV_HoverEst *est, const DRV_HoverEstParams *params, float init_n)
{
    uint8_t ok = 1U;

    if (est == NULL) {
        return 0U;
    }
    memset(est, 0, sizeof(*est));
    if (params == NULL) {
        DRV_HoverEst_DefaultParams(&est->params);
    } else if (DRV_HoverEst_ParamsValid(params) != 0U) {
        est->params = *params;
    } else {
        DRV_HoverEst_DefaultParams(&est->params);
        ok = 0U;
    }
    if ((hover_finite(init_n) == 0U) || (init_n < DRV_HOVEREST_H_MIN_N) ||
        (init_n > DRV_HOVEREST_H_MAX_N)) {
        init_n = HOVEREST_FALLBACK_INIT_N;
        ok = 0U;
    }
    est->init_n = init_n;
    hover_reset_state(est);
    return ok;
}

void DRV_HoverEst_Reset(DRV_HoverEst *est)
{
    if (est != NULL) {
        hover_reset_state(est);
    }
}

/* 一阶低通/滞后：tau = 0 直通。 */
static float hover_lag(float state, float input, float dt_s, float tau_s)
{
    if (tau_s <= 0.0f) {
        return input;
    }
    return state + ((dt_s / (tau_s + dt_s)) * (input - state));
}

DRV_HoverEstResult DRV_HoverEst_Step(DRV_HoverEst *est, const DRV_HoverEstInput *in)
{
    float cos_tilt;
    float thrust_up;
    float g;
    float h;
    float ratio;
    float h_gain;       /* H = ∂a/∂h */
    float innov;
    float s_var;
    float gain;
    float p_prior;

    if ((est == NULL) || (in == NULL)) {
        return DRV_HOVEREST_IGNORED;
    }
    est->learning = 0U;
    if ((hover_finite(in->dt_s) == 0U) || (in->dt_s <= 0.0f) ||
        (hover_finite(in->thrust_n) == 0U) || (in->thrust_n < 0.0f) ||
        (hover_finite(in->a_up_m_s2) == 0U) || (hover_finite(in->roll_rad) == 0U) ||
        (hover_finite(in->pitch_rad) == 0U) || (hover_finite(in->gravity_m_s2) == 0U) ||
        (in->gravity_m_s2 <= 1.0f)) {
        est->filt_primed = 0U;       /* 输入断档：下一个合法拍重新起步，别带着旧值滤 */
        return DRV_HOVEREST_IGNORED;
    }
    cos_tilt = cosf(in->roll_rad) * cosf(in->pitch_rad);
    if (cos_tilt <= 0.05f) {
        est->filt_primed = 0U;
        return DRV_HOVEREST_IGNORED;
    }
    thrust_up = in->thrust_n * cos_tilt;

    if ((est->filt_primed == 0U) || (in->dt_s > DRV_HOVEREST_DT_MAX_S)) {
        est->thrust_f_n = thrust_up;
        est->accel_f_m_s2 = in->a_up_m_s2;
        est->filt_primed = 1U;
        return DRV_HOVEREST_HELD;    /* 起步拍不学：滤波器刚被这一拍种下，还没有历史 */
    }
    est->thrust_f_n = hover_lag(est->thrust_f_n, thrust_up, in->dt_s, est->params.thrust_lag_s);
    est->accel_f_m_s2 = hover_lag(est->accel_f_m_s2, in->a_up_m_s2, in->dt_s,
                                  est->params.accel_lpf_s);
    if (in->learn == 0U) {
        return DRV_HOVEREST_HELD;
    }
    est->learning = 1U;

    /* 预测：只在学的拍推进。 */
    est->p_n2 += est->params.process_std_n_sqrt_s * est->params.process_std_n_sqrt_s * in->dt_s;

    g = in->gravity_m_s2;
    h = est->h_n;
    ratio = est->thrust_f_n / h;
    h_gain = -g * ratio / h;
    innov = est->accel_f_m_s2 - (g * (ratio - 1.0f));
    s_var = (h_gain * h_gain * est->p_n2) + est->r_var;
    est->last_innov_m_s2 = innov;

    if ((innov * innov) > (est->params.gate_sigma * est->params.gate_sigma * s_var)) {
        est->rejected++;
        if (est->reject_run < 0xFFFFU) {
            est->reject_run++;
        }
        if (est->reject_run >= est->params.reject_inflate) {
            const float inflate = est->params.inflate_std_n * est->params.inflate_std_n;

            if (est->p_n2 < inflate) {
                est->p_n2 = inflate;
            }
            est->reject_run = 0U;
        }
        return DRV_HOVEREST_REJECTED;
    }
    est->reject_run = 0U;

    p_prior = est->p_n2;
    gain = (p_prior * h_gain) / s_var;
    est->h_n = hover_clamp(h + (gain * innov), DRV_HOVEREST_H_MIN_N, DRV_HOVEREST_H_MAX_N);
    est->p_n2 = p_prior - (gain * h_gain * p_prior);
    if (est->p_n2 < DRV_HOVEREST_P_MIN_N2) {
        est->p_n2 = DRV_HOVEREST_P_MIN_N2;
    }
    {
        /* 测量方差自适应：新息² 里扣掉状态不确定度贡献的部分，剩下的归测量噪声。 */
        const float excess = (innov * innov) - (h_gain * h_gain * p_prior);
        const float r_min = est->params.meas_std_min * est->params.meas_std_min;
        const float r_max = est->params.meas_std_max * est->params.meas_std_max;
        const float target = (excess > r_min) ? excess : r_min;

        est->r_var = hover_clamp(((1.0f - est->params.noise_alpha) * est->r_var) +
                                 (est->params.noise_alpha * target), r_min, r_max);
    }
    est->samples++;
    return DRV_HOVEREST_ACCEPTED;
}

void DRV_HoverEst_GetOutput(const DRV_HoverEst *est, DRV_HoverEstOutput *out)
{
    if ((est == NULL) || (out == NULL)) {
        return;
    }
    out->est_n = est->h_n;
    out->std_n = sqrtf(est->p_n2);
    out->init_n = est->init_n;
    out->innov_m_s2 = est->last_innov_m_s2;
    out->meas_std_m_s2 = sqrtf(est->r_var);
    out->samples = est->samples;
    out->rejected = est->rejected;
    out->converged = ((est->samples >= est->params.converged_samples) &&
                      (sqrtf(est->p_n2) <= est->params.converged_std_n)) ? 1U : 0U;
    out->learning = est->learning;
}

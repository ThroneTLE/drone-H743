#include "drv_z_estimator.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

/* 模型、增益推导与延迟补偿的说明都在头文件里，这里只写实现细节。 */

#define ZEST_SINCE_MEAS_CAP_S  10.0f   /* 只需判"超没超 timeout"，封顶防 float 长期累加失真 */

static float zest_clampf(float value, float lo, float hi)
{
    if (value < lo) {
        return lo;
    }
    if (value > hi) {
        return hi;
    }
    return value;
}

static uint8_t zest_in_range(float value, float lo, float hi)
{
    return (isfinite(value) && (value >= lo) && (value <= hi)) ? 1U : 0U;
}

static uint8_t zest_valid(const DRV_ZEst *est)
{
    return ((est->initialized != 0U) &&
            (est->since_meas_s <= est->params.timeout_s)) ? 1U : 0U;
}

static uint8_t zest_older(uint8_t index)
{
    return (uint8_t)((index + DRV_ZEST_HISTORY - 1U) % DRV_ZEST_HISTORY);
}

void DRV_ZEst_DefaultParams(DRV_ZEstParams *params)
{
    if (params == NULL) {
        return;
    }
    params->tau_s = DRV_ZEST_TAU_DEFAULT_S;
    params->delay_s = DRV_ZEST_DELAY_DEFAULT_S;
    params->gate_m = DRV_ZEST_GATE_DEFAULT_M;
    params->reject_reset = DRV_ZEST_REJECT_RESET_DEFAULT;
    params->timeout_s = DRV_ZEST_TIMEOUT_DEFAULT_S;
}

uint8_t DRV_ZEst_ParamsValid(const DRV_ZEstParams *params)
{
    if (params == NULL) {
        return 0U;
    }
    return ((zest_in_range(params->tau_s, DRV_ZEST_TAU_MIN_S, DRV_ZEST_TAU_MAX_S) != 0U) &&
            (zest_in_range(params->delay_s, 0.0f, DRV_ZEST_DELAY_MAX_S) != 0U) &&
            (zest_in_range(params->gate_m, DRV_ZEST_GATE_MIN_M, DRV_ZEST_GATE_MAX_M) != 0U) &&
            (params->reject_reset >= 1U) &&
            (zest_in_range(params->timeout_s, DRV_ZEST_TIMEOUT_MIN_S,
                           DRV_ZEST_TIMEOUT_MAX_S) != 0U)) ? 1U : 0U;
}

uint8_t DRV_ZEst_Init(DRV_ZEst *est, const DRV_ZEstParams *params)
{
    uint8_t ok = 1U;

    if (est == NULL) {
        return 0U;
    }
    memset(est, 0, sizeof(*est));
    if (params == NULL) {
        DRV_ZEst_DefaultParams(&est->params);
    } else if (DRV_ZEst_ParamsValid(params) != 0U) {
        est->params = *params;
    } else {
        DRV_ZEst_DefaultParams(&est->params);
        ok = 0U;
    }
    return ok;
}

uint8_t DRV_ZEst_SetParams(DRV_ZEst *est, const DRV_ZEstParams *params)
{
    if ((est == NULL) || (DRV_ZEst_ParamsValid(params) == 0U)) {
        return 0U;
    }
    est->params = *params;
    return 1U;
}

void DRV_ZEst_Reset(DRV_ZEst *est)
{
    DRV_ZEstParams keep;

    if (est == NULL) {
        return;
    }
    keep = est->params;
    memset(est, 0, sizeof(*est));
    est->params = keep;
}

void DRV_ZEst_Predict(DRV_ZEst *est, float dt_s, float a_up_m_s2, uint8_t accel_valid)
{
    float a;

    if (est == NULL) {
        return;
    }
    est->accel_valid = ((accel_valid != 0U) && isfinite(a_up_m_s2)) ? 1U : 0U;
    est->accel_raw_m_s2 = (est->accel_valid != 0U) ? a_up_m_s2 : 0.0f;
    if (!isfinite(dt_s) || (dt_s <= 0.0f)) {
        return;                                   /* dt = 0：只记加速度，不推进 */
    }
    if (dt_s > DRV_ZEST_DT_MAX_S) {
        dt_s = DRV_ZEST_DT_MAX_S;
    }

    if (est->initialized != 0U) {
        a = (est->accel_valid != 0U) ? (est->accel_raw_m_s2 - est->bias_m_s2) : 0.0f;
        est->z_m += (est->vz_m_s * dt_s) + (0.5f * a * dt_s * dt_s);
        est->vz_m_s += a * dt_s;
    }
    if (est->since_meas_s < ZEST_SINCE_MEAS_CAP_S) {
        est->since_meas_s += dt_s;
    }

    est->head = (uint8_t)((est->head + 1U) % DRV_ZEST_HISTORY);
    est->hist_z[est->head] = est->z_m;
    est->hist_dt[est->head] = dt_s;
    if (est->count < DRV_ZEST_HISTORY) {
        est->count++;
    }
}

/*
 * 历史里"lag_s 以前"的高度：从最新一条往回按 dt 累加、相邻两条线性插值；覆盖不到就取
 * 最老一条。*lag_used 回写实际用到的时间差（覆盖不到时比 lag_s 短）。
 */
static float zest_history_at(const DRV_ZEst *est, float lag_s, float *lag_used)
{
    uint8_t index = est->head;
    float age = 0.0f;

    if ((est->count == 0U) || (lag_s <= 0.0f)) {
        *lag_used = 0.0f;
        return est->z_m;
    }
    for (uint8_t n = 1U; n < est->count; ++n) {
        const float dt = est->hist_dt[index];
        const uint8_t older = zest_older(index);

        if ((age + dt) >= lag_s) {
            const float frac = (dt > 0.0f) ? ((lag_s - age) / dt) : 0.0f;
            *lag_used = lag_s;
            return est->hist_z[index] + (frac * (est->hist_z[older] - est->hist_z[index]));
        }
        age += dt;
        index = older;
    }
    *lag_used = age;
    return est->hist_z[index];
}

/*
 * 在延迟时刻 t_d 施加的修正 (dz, dv, db) 沿模型传播到每一条历史：
 * 离 t_d 为 τ 的那条平移 dz + dv·τ − ½·db·τ²（零偏 +db 让之后的 a 少 db）。
 * 比 t_d 还老的条目 τ < 0，同一公式即沿模型往回的延拓，只在样本年龄抖动时才会被查到。
 */
static void zest_shift_history(DRV_ZEst *est, float lag_s, float dz, float dv, float db)
{
    uint8_t index = est->head;
    float age = 0.0f;

    for (uint8_t n = 0U; n < est->count; ++n) {
        const float tau = lag_s - age;

        est->hist_z[index] += dz + (dv * tau) - (0.5f * db * tau * tau);
        age += est->hist_dt[index];
        index = zest_older(index);
    }
}

static void zest_start_at(DRV_ZEst *est, float height_m, float age_s)
{
    est->z_m = height_m;
    est->vz_m_s = 0.0f;
    for (uint8_t n = 0U; n < DRV_ZEST_HISTORY; ++n) {
        est->hist_z[n] = height_m;
    }
    est->since_meas_s = age_s;
    est->initialized = 1U;
    est->reject_run = 0U;
    est->last_innovation_m = 0.0f;
}

DRV_ZEstMeasResult DRV_ZEst_Correct(DRV_ZEst *est, float height_m, float age_s)
{
    float lag_s;
    float z_then;
    float e;
    float t_s;
    float p;
    float q;
    float dz;
    float dv;
    float db;
    float bias_new;

    if ((est == NULL) || !isfinite(height_m) || !isfinite(age_s)) {
        return DRV_ZEST_MEAS_IGNORED;
    }
    if (age_s < 0.0f) {
        age_s = 0.0f;
    }
    if (age_s > est->params.timeout_s) {
        return DRV_ZEST_MEAS_IGNORED;             /* 到手时已过期 */
    }

    if (zest_valid(est) == 0U) {
        zest_start_at(est, height_m, age_s);
        est->accepted_count++;
        return DRV_ZEST_MEAS_INIT;
    }

    z_then = zest_history_at(est, est->params.delay_s + age_s, &lag_s);
    e = height_m - z_then;
    est->last_innovation_m = e;

    if (fabsf(e) > est->params.gate_m) {
        est->rejected_count++;
        est->reject_run++;
        if (est->reject_run < est->params.reject_reset) {
            return DRV_ZEST_MEAS_REJECTED;
        }
        /* 连续拒收：读数确实跳了（地形台阶等）。只平移高度，速度与零偏不动。 */
        zest_shift_history(est, lag_s, e, 0.0f, 0.0f);
        est->z_m += e;
        est->since_meas_s = age_s;
        est->reject_run = 0U;
        est->reset_count++;
        return DRV_ZEST_MEAS_RESET;
    }

    /* 与上一个被接收样本的采样间隔：since_meas 从上一个样本的采样时刻算起。 */
    t_s = zest_clampf(est->since_meas_s - age_s, DRV_ZEST_MEAS_T_MIN_S, DRV_ZEST_MEAS_T_MAX_S);
    p = expf(-t_s / est->params.tau_s);
    q = 1.0f - p;
    dz = q * (1.0f + p + (p * p)) * e;            /* (1 − p³)·e */
    dv = 1.5f * q * q * (1.0f + p) / t_s * e;
    db = -(q * q * q) / (t_s * t_s) * e;
    bias_new = zest_clampf(est->bias_m_s2 + db, -DRV_ZEST_BIAS_LIMIT_M_S2,
                           DRV_ZEST_BIAS_LIMIT_M_S2);
    db = bias_new - est->bias_m_s2;

    zest_shift_history(est, lag_s, dz, dv, db);
    est->z_m += dz + (dv * lag_s) - (0.5f * db * lag_s * lag_s);
    est->vz_m_s += dv - (db * lag_s);
    est->bias_m_s2 = bias_new;
    est->since_meas_s = age_s;
    est->reject_run = 0U;
    est->accepted_count++;
    return DRV_ZEST_MEAS_ACCEPTED;
}

DRV_ZEstMeasResult DRV_ZEst_Tick(DRV_ZEst *est, const DRV_ZEstTickInput *in)
{
    float dt_s = 0.0f;
    int32_t age_ms;

    if ((est == NULL) || (in == NULL)) {
        return DRV_ZEST_MEAS_IGNORED;
    }
    if ((est->tick_primed != 0U) && (in->now_us > est->tick_last_us)) {
        dt_s = (float)(in->now_us - est->tick_last_us) * 1.0e-6f;
    }
    est->tick_last_us = in->now_us;
    est->tick_primed = 1U;
    DRV_ZEst_Predict(est, dt_s, in->a_up_m_s2, in->accel_valid);

    if ((in->range_valid == 0U) || (in->range_sample_ms == 0U) ||
        (in->range_sample_ms == est->tick_last_sample_ms)) {
        return DRV_ZEST_MEAS_NONE;
    }
    est->tick_last_sample_ms = in->range_sample_ms;
    /* 同一个 ms 钟相减；样本时间戳比 now_ms 还新（取时刻之间刚到）按年龄 0 处理。 */
    age_ms = (int32_t)(in->now_ms - in->range_sample_ms);
    return DRV_ZEst_Correct(est, in->range_m, (age_ms > 0) ? ((float)age_ms * 1.0e-3f) : 0.0f);
}

void DRV_ZEst_GetOutput(const DRV_ZEst *est, DRV_ZEstOutput *out)
{
    if (out == NULL) {
        return;
    }
    memset(out, 0, sizeof(*out));
    if (est == NULL) {
        return;
    }
    out->z_m = est->z_m;
    out->vz_m_s = est->vz_m_s;
    out->bias_m_s2 = est->bias_m_s2;
    out->accel_valid = est->accel_valid;
    out->accel_m_s2 = (est->accel_valid != 0U) ? (est->accel_raw_m_s2 - est->bias_m_s2) : 0.0f;
    out->innovation_m = est->last_innovation_m;
    out->valid = zest_valid(est);
}

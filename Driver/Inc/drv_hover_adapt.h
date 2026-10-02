#ifndef DRV_HOVER_ADAPT_H
#define DRV_HOVER_ADAPT_H

#include <math.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 悬停推力自适应接入控制器的纯算术（与 drv_att_reference.h 一样写成头文件模块）。
 *
 * 作者 2026-10-01："让自适应学习出现效果，然后可以配合积分适当补偿"，并确认做法：
 * "收敛之后才用；限制在初值上下 15% 以内，慢慢过渡；每次解锁重新学"（同 PX4 MPC_USE_HTE）。
 *
 *   目标值：未启用 → 配置值；学习器已收敛 → 估计值夹到 配置值 ×(1 ± BAND)；
 *           未收敛 → 保持当前已用值（不因学习器暂时不稳而跳回）。
 *   过渡：  已用值向目标以 ≤ SLEW 的速率移动。
 *   无扰：  控制器合推力 T = H·(1 + a/g)（H 悬停推力、a 竖直加速度指令）。H 由 H0 换成 H1 时，
 *           竖直速度积分加 Δa = (g + a)·(H0/H1 − 1)，当拍 T 不变；之后积分只管快变残差，
 *           慢变的电池掉压由 H 吸收（同 PX4 PositionControl::updateHoverThrust）。
 *
 * 单位：推力 N（推力查补表口径，与 coax.hover_thrust_n 同口径）、加速度 m/s²（竖直向上为正）、时间 s。
 */

#define DRV_HOVER_ADAPT_BAND_FRAC   0.15f   /* 只在配置值 ±15% 内用学到的值 */
#define DRV_HOVER_ADAPT_SLEW_N_S    0.5f    /* 已用值的最大变化率 */

static inline float DRV_HoverAdapt_Target(uint8_t enabled, uint8_t converged,
                                          float est_n, float cfg_n, float held_n)
{
    float lo;
    float hi;

    if ((enabled == 0U) || !(cfg_n > 0.0f)) {
        return cfg_n;
    }
    if ((converged == 0U) || (isfinite(est_n) == 0)) {
        return (held_n > 0.0f) ? held_n : cfg_n;
    }
    lo = cfg_n * (1.0f - DRV_HOVER_ADAPT_BAND_FRAC);
    hi = cfg_n * (1.0f + DRV_HOVER_ADAPT_BAND_FRAC);
    return (est_n < lo) ? lo : ((est_n > hi) ? hi : est_n);
}

static inline float DRV_HoverAdapt_Slew(float applied_n, float target_n, float dt_s)
{
    float step;

    if (!(applied_n > 0.0f)) {
        return target_n;
    }
    if (!(dt_s > 0.0f)) {
        return applied_n;
    }
    step = DRV_HOVER_ADAPT_SLEW_N_S * dt_s;
    if (target_n > applied_n + step) {
        return applied_n + step;
    }
    if (target_n < applied_n - step) {
        return applied_n - step;
    }
    return target_n;
}

/* 悬停推力 old_n → new_n 时竖直速度积分应加的量，使 T = H·(1 + a/g) 当拍不变。 */
static inline float DRV_HoverAdapt_IntegratorShift(float old_n, float new_n,
                                                   float gravity_m_s2, float accel_cmd_m_s2)
{
    if (!(old_n > 0.0f) || !(new_n > 0.0f) || (isfinite(accel_cmd_m_s2) == 0)) {
        return 0.0f;
    }
    return (gravity_m_s2 + accel_cmd_m_s2) * (old_n / new_n - 1.0f);
}

#ifdef __cplusplus
}
#endif

#endif /* DRV_HOVER_ADAPT_H */

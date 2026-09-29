#ifndef DRV_ATT_REFERENCE_H
#define DRV_ATT_REFERENCE_H

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 姿态参考模型（横滚/俯仰）：把阶跃式的角度目标整形成一条对象跟得上的轨迹，
 * 同时给出前馈。纯算术：无寄存器、无 RTOS、无文件级可变量，状态在调用者的结构体里。
 * 与 drv_moment_notch.h 一样写成头文件模块（理由见那里的文末说明）。
 *
 * ──────────────── 做什么 ────────────────
 *
 * 临界阻尼二阶参考（每轴独立）：
 *     θ̈_ref = ωr²·(θ_cmd − θ_ref) − 2ωr·θ̇_ref
 * 离散化与设计脚本 attitude_design_compare.sim() 逐字相同（半隐式 Euler）：
 *     rdd = ωr²·(cmd − r) − 2ωr·rd;  rd += rdd·dt;  r += rd·dt
 * 输出三样，用途不同、时间也不同：
 *     angle = θ_ref(t − Td)   角度环反馈的目标
 *     rate  = θ̇_ref(t − Td)   角度环的角速度前馈
 *     accel = θ̈_ref(t)        速率环力矩前馈 α_ff（**不延后**）
 * 反馈用延后的参考：力矩前馈要经过对象本来就有的约 Td 的滞后（纯延迟 + 舵机）才变成
 * 姿态，拿 t 时刻的参考去比 t 时刻的姿态，反馈会去"追"这段本来就该有的滞后。
 *
 * ──────────────── 契约（D4-3） ────────────────
 *
 * 角度 rad、角速度 rad/s、角加速度 rad/s²，轴 0 = 横滚、1 = 俯仰，规范 FLU 欧拉角
 * （drv_frame_contract.h）；本模块不改坐标、不改符号。dt 来自调用者的真实时间戳差
 * （D2-3），夹到 [0, DT_MAX]；dt = 0 表示"本拍不推进"，输出保持上一拍。
 * 延迟 Td 按时间在历史里线性插值查找（调度抖动下仍是时间意义的 Td）；历史覆盖不到 Td 时
 * 取最老的一条并置 delay_truncated（历史容量按 250 Hz 飞控与 500 Hz 光杆两种拍率留足）。
 *
 * 对齐：Reset 之后的第一次 Evaluate 以调用者给的 align_angle（通常是当前实测角）为起点、
 * 速度与加速度为 0；对齐点之前的历史视为一直停在对齐角，所以开头 Td 内的延后参考就是
 * 对齐角——不会带着旧状态冲出去。
 *
 * 同一拍可以 Evaluate 多次（飞控保护缩放会同拍重算），Commit 才推进状态。
 */

#define DRV_ATT_REF_AXES            2U        /* 0 = roll, 1 = pitch */
#define DRV_ATT_REF_HISTORY         64U       /* 1 kHz 下 64 ms，500 Hz 下 128 ms，250 Hz 下 256 ms */
#define DRV_ATT_REF_WR_MIN_RAD_S    0.5f      /* 非 0 时的下限；0 = 关 */
/* 上限保证 ωr·dt ≤ 0.6（dt ≤ DT_MAX），半隐式 Euler 在 ωr·dt < 0.83 内稳定。 */
#define DRV_ATT_REF_WR_MAX_RAD_S    30.0f
#define DRV_ATT_REF_DELAY_MAX_MS    80.0f     /* 参数 att_ref_delay_ms 的上限 */
#define DRV_ATT_REF_DT_MAX_S        0.020f

/* 一拍的结果（待提交）。 */
typedef struct {
    float angle[DRV_ATT_REF_AXES];        /* θ_ref(t) */
    float rate[DRV_ATT_REF_AXES];         /* θ̇_ref(t) */
    float accel[DRV_ATT_REF_AXES];        /* 本拍用的 θ̈_ref（推进前状态算出） */
    float base_angle[DRV_ATT_REF_AXES];   /* aligned=1 时的对齐角 */
    float dt_s;
    uint8_t aligned;                      /* 本拍从对齐点起步 */
} DRV_AttRefStep;

typedef struct {
    DRV_AttRefStep last;                                  /* 最新已提交的一拍 */
    float hist_angle[DRV_ATT_REF_HISTORY][DRV_ATT_REF_AXES];
    float hist_rate[DRV_ATT_REF_HISTORY][DRV_ATT_REF_AXES];
    float hist_dt[DRV_ATT_REF_HISTORY];                   /* 该条与更老一条的时间差 */
    uint8_t head;                                         /* 最新一条的下标 */
    uint8_t count;                                        /* 有效条数 */
    uint8_t initialized;
} DRV_AttRef;

typedef struct {
    float angle[DRV_ATT_REF_AXES];   /* θ_ref(t − Td) */
    float rate[DRV_ATT_REF_AXES];    /* θ̇_ref(t − Td) */
    float accel[DRV_ATT_REF_AXES];   /* θ̈_ref(t) */
    uint8_t delay_truncated;         /* 历史不够 Td，按最老一条给出 */
} DRV_AttRefOutput;

/* 0 = 关；否则 [WR_MIN, WR_MAX]。 */
static inline uint8_t DRV_AttRef_BandwidthValid(float wr_rad_s)
{
    return (isfinite(wr_rad_s) &&
            ((wr_rad_s == 0.0f) ||
             ((wr_rad_s >= DRV_ATT_REF_WR_MIN_RAD_S) && (wr_rad_s <= DRV_ATT_REF_WR_MAX_RAD_S))))
        ? 1U : 0U;
}

/* 延后 Td 以毫秒校验（参数单位），[0, DELAY_MAX_MS]。 */
static inline uint8_t DRV_AttRef_DelayValid(float delay_ms)
{
    return (isfinite(delay_ms) && (delay_ms >= 0.0f) && (delay_ms <= DRV_ATT_REF_DELAY_MAX_MS))
        ? 1U : 0U;
}

/* 下次 Evaluate 重新对齐。 */
static inline void DRV_AttRef_Reset(DRV_AttRef *r)
{
    if (r != NULL) {
        r->initialized = 0U;
        r->count = 0U;
        r->head = 0U;
    }
}

/*
 * 在"最新一拍（step，时间 0）→ 已提交历史 → 对齐角"这条倒序时间线上取 t − Td 的值。
 * initialized = 0 时历史只有对齐点一条（step.base_angle，速度 0，时间无限久远）。
 */
static inline void DRV_AttRef_Lookup(const DRV_AttRef *r, const DRV_AttRefStep *step,
                                     float delay_s, DRV_AttRefOutput *out)
{
    float t_newer = 0.0f;
    float dt_newer = step->dt_s;
    const float *a_newer = step->angle;
    const float *v_newer = step->rate;
    uint32_t available;
    uint32_t index;

    out->delay_truncated = 0U;
    for (uint32_t axis = 0U; axis < DRV_ATT_REF_AXES; ++axis) {
        out->angle[axis] = step->angle[axis];
        out->rate[axis] = step->rate[axis];
    }
    if (!(delay_s > 0.0f)) {
        return;
    }
    if (r->initialized == 0U) {
        /* 对齐点之前一直停在对齐角：t − Td 早于对齐点就给对齐角。 */
        const float frac = (dt_newer > 0.0f) ? (delay_s / dt_newer) : 1.0f;
        for (uint32_t axis = 0U; axis < DRV_ATT_REF_AXES; ++axis) {
            const float base = step->aligned ? step->base_angle[axis] : step->angle[axis];
            if (frac >= 1.0f) {
                out->angle[axis] = base;
                out->rate[axis] = 0.0f;
            } else {
                out->angle[axis] = step->angle[axis] + ((base - step->angle[axis]) * frac);
                out->rate[axis] = step->rate[axis] * (1.0f - frac);
            }
        }
        return;
    }
    available = r->count;
    index = r->head;
    for (uint32_t n = 0U; n < available; ++n) {
        const float t_older = t_newer + dt_newer;
        const float *a_older = r->hist_angle[index];
        const float *v_older = r->hist_rate[index];

        if (t_older >= delay_s) {
            const float span = t_older - t_newer;
            const float frac = (span > 0.0f) ? ((delay_s - t_newer) / span) : 1.0f;
            for (uint32_t axis = 0U; axis < DRV_ATT_REF_AXES; ++axis) {
                out->angle[axis] = a_newer[axis] + ((a_older[axis] - a_newer[axis]) * frac);
                out->rate[axis] = v_newer[axis] + ((v_older[axis] - v_newer[axis]) * frac);
            }
            return;
        }
        t_newer = t_older;
        dt_newer = r->hist_dt[index];
        a_newer = a_older;
        v_newer = v_older;
        index = (index == 0U) ? (DRV_ATT_REF_HISTORY - 1U) : (index - 1U);
    }
    /* 翻到头也没够 Td：最老那条。历史未满时最老那条就是对齐点，属于正常起步。 */
    for (uint32_t axis = 0U; axis < DRV_ATT_REF_AXES; ++axis) {
        out->angle[axis] = a_newer[axis];
        out->rate[axis] = v_newer[axis];
    }
    out->delay_truncated = (r->count >= DRV_ATT_REF_HISTORY) ? 1U : 0U;
}

/*
 * 算一拍（不提交）。cmd 是本拍的角度目标，align_angle 只在需要对齐时使用，
 * dt_s 是本拍推进的时间（0 = 不推进，输出保持上一拍的状态与加速度）。
 */
static inline void DRV_AttRef_Evaluate(const DRV_AttRef *r, float wr_rad_s, float delay_s,
                                       const float cmd[DRV_ATT_REF_AXES],
                                       const float align_angle[DRV_ATT_REF_AXES],
                                       float dt_s, DRV_AttRefStep *step,
                                       DRV_AttRefOutput *out)
{
    const float dt = (isfinite(dt_s) && (dt_s > 0.0f))
        ? ((dt_s < DRV_ATT_REF_DT_MAX_S) ? dt_s : DRV_ATT_REF_DT_MAX_S) : 0.0f;
    const float w2 = wr_rad_s * wr_rad_s;
    const float two_w = 2.0f * wr_rad_s;

    if ((r == NULL) || (cmd == NULL) || (align_angle == NULL) || (step == NULL) ||
        (out == NULL)) {
        return;
    }
    memset(out, 0, sizeof(*out));
    if (r->initialized != 0U) {
        *step = r->last;
        step->aligned = 0U;
    } else {
        memset(step, 0, sizeof(*step));
        for (uint32_t axis = 0U; axis < DRV_ATT_REF_AXES; ++axis) {
            step->angle[axis] = align_angle[axis];
            step->base_angle[axis] = align_angle[axis];
        }
        step->aligned = 1U;
    }
    step->dt_s = dt;
    if (dt > 0.0f) {
        for (uint32_t axis = 0U; axis < DRV_ATT_REF_AXES; ++axis) {
            const float target = isfinite(cmd[axis]) ? cmd[axis] : step->angle[axis];
            const float rdd = (w2 * (target - step->angle[axis])) - (two_w * step->rate[axis]);

            step->accel[axis] = rdd;
            step->rate[axis] += rdd * dt;
            step->angle[axis] += step->rate[axis] * dt;
        }
    }
    DRV_AttRef_Lookup(r, step, delay_s, out);
    for (uint32_t axis = 0U; axis < DRV_ATT_REF_AXES; ++axis) {
        out->accel[axis] = step->accel[axis];
    }
}

/* 提交 Evaluate 算出的那一拍。dt = 0 的拍不占历史。 */
static inline void DRV_AttRef_Commit(DRV_AttRef *r, const DRV_AttRefStep *step)
{
    if ((r == NULL) || (step == NULL)) {
        return;
    }
    if (r->initialized == 0U) {
        /* 对齐点作为最老的一条：速度 0，开头 Td 内的延后参考就是它。 */
        r->head = 0U;
        for (uint32_t axis = 0U; axis < DRV_ATT_REF_AXES; ++axis) {
            r->hist_angle[0][axis] = step->aligned ? step->base_angle[axis] : step->angle[axis];
            r->hist_rate[0][axis] = 0.0f;
        }
        r->hist_dt[0] = 0.0f;
        r->count = 1U;
        r->initialized = 1U;
    }
    if (step->dt_s > 0.0f) {
        r->head = (uint8_t)((r->head + 1U) % DRV_ATT_REF_HISTORY);
        for (uint32_t axis = 0U; axis < DRV_ATT_REF_AXES; ++axis) {
            r->hist_angle[r->head][axis] = step->angle[axis];
            r->hist_rate[r->head][axis] = step->rate[axis];
        }
        r->hist_dt[r->head] = step->dt_s;
        if (r->count < DRV_ATT_REF_HISTORY) {
            r->count++;
        }
    }
    r->last = *step;
    r->last.aligned = 0U;
}

#ifdef __cplusplus
}
#endif

#endif /* DRV_ATT_REFERENCE_H */

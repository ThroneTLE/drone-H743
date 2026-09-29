#ifndef DRV_MOMENT_NOTCH_H
#define DRV_MOMENT_NOTCH_H

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "drv_rate_control.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 速率环力矩出口陷波（横滚/俯仰）。纯算术：无寄存器、无 RTOS、无文件级可变量，
 * 状态在调用者传入的结构体里。整个模块写在头文件里（static inline），原因见文末。
 *
 * ──────────────── 为什么要它 ────────────────
 *
 * 光杆辨识的飞行对象 P(s) = e^{−sT}·S(s)·(κ + ρs²)/(I·s)。舵机甩动的反作用 ρs²
 * 让回路增益在约 2 Hz 反谐振之上重新抬头，3–8 Hz 的裕度对模型误差很敏感。在反馈
 * 力矩上压一个 6 Hz、−20 dB 的部分深度陷波后，辨识不确定度的最坏角点仍满足
 * GM ≥ 6 dB / PM ≥ 45°（data/analysis/sysid-rig-params/2026-09-28/scripts/
 * attitude_filter_search.py、attitude_design_compare.py）。
 *
 * 第二级（coax.rate_out_notch2_*，默认关）串在第一级之后、滤同一个信号：2026-09-28
 * 台架舵机单独扫频，舵机指令 → 机体角速度在 8–18 Hz 比模型高 5–11 dB（舵机高阶动态
 * + 结构），kp 一加大就在约 15 Hz 把舵机抖起来；约 16 Hz、Q≈1 的第二级能压住它、
 * 代价小（hf_filter_study.py）。两级是同一个模块的两个实例，深度都是 −20 dB。
 *
 * ──────────────── 信号契约（D4-3） ────────────────
 *
 * 连续原型 N(s) = (s² + d·ω0/Q·s + ω0²)/(s² + ω0/Q·s + ω0²)，d = 0.1（−20 dB）。
 * 直流增益恰为 1：配平力矩与积分稳态不受影响。输入输出都是 N·m，轴 0 = 横滚、
 * 1 = 俯仰（规范 FLU 机体系，drv_frame_contract.h），滤波不改坐标、不改符号。
 * 离散化用双线性变换并预畸变到 f0（f0 处深度精确为 d），与
 * attitude_design_compare.py 的 notch_biquad 是同一公式。
 *
 * 只滤反馈部分（P + I − D）：力矩前馈 ff_gain·I·α_ff 与陀螺耦合项原样叠回、不过
 * 陷波——参考模型给出的 I·θ̈_ref 恰好落在陷波附近的频段，滤它等于把前馈削掉
 * （设计脚本 sim() 也是先滤反馈再加前馈）。
 *
 * ──────────────── 时间与状态 ────────────────
 *
 * 系数按**本拍真实 dt** 现算（解耦规范 D2-3：Δt 来自时间戳差，不写死名义周期）；
 * dt 与参数都没变就沿用上一组。控制拍在 2–3 ms 之间抖，按名义 500 Hz 固定系数的话，
 * 3 ms 那一拍的实际陷波中心会漂到约 4 Hz。
 * 直接型 I（DF1）存的是真实的输入输出历史，换系数不会注入台阶（与 drv_rpm_notch.h
 * 选 DF1 的理由相同）。
 *
 * 同一拍可以 Apply 多次（控制器的保护缩放会在同一拍按缩放后的指令重算一遍），
 * 只有 Commit 才推进历史；不推进的拍调 Discard。
 *
 * 关闭（f0 = 0）、参数非法、dt 超界或 f0 ≥ 0.45/dt 时 Apply **逐位直通**，历史作废；
 * 再打开后的第一拍以当时的输入为直流稳态起步（输出 = 输入），不会砸出台阶。
 *
 * 为什么是头文件模块：飞控和光杆辨识两处都用它，而宿主上有二十多个 harness 与
 * 上位机仿真（tools/sim_xz）按固定源文件清单编译 drv_coax_ctrl.c / app_sysid.c。
 * 放进 .c 就得同时改所有清单，漏一个就是链接失败；与 drv_frame_contract.h 的
 * static inline 同一做法。
 */

#define DRV_MOMENT_NOTCH_AXES              2U       /* 0 = roll, 1 = pitch */
#define DRV_MOMENT_NOTCH_DEPTH             0.1f     /* 中心处增益，−20 dB */
#define DRV_MOMENT_NOTCH_HZ_MIN            1.0f     /* 非 0 时的下限；0 = 关 */
#define DRV_MOMENT_NOTCH_HZ_MAX            100.0f
#define DRV_MOMENT_NOTCH_Q_MIN             0.3f
#define DRV_MOMENT_NOTCH_Q_MAX             10.0f
#define DRV_MOMENT_NOTCH_NYQUIST_FRACTION  0.45f    /* f0 必须 < 0.45/dt，否则直通 */
#define DRV_MOMENT_NOTCH_DT_MIN_S          1.0e-4f
#define DRV_MOMENT_NOTCH_DT_MAX_S          0.05f
#define DRV_MOMENT_NOTCH_FLUSH_ABS         1.0e-20f
#define DRV_MOMENT_NOTCH_TWO_PI            6.28318530717958647692f

/* 归一化到 a0 = 1：y = b0·x + b1·x1 + b2·x2 − a1·y1 − a2·y2（本型 a1 == b1）。 */
typedef struct {
    float b0, b1, b2, a1, a2;
} DRV_MomentNotchCoef;

typedef struct {
    DRV_MomentNotchCoef coef;
    float coef_hz, coef_q, coef_dt_s;       /* 当前系数对应的参数；active=0 时无意义 */
    float x1[DRV_MOMENT_NOTCH_AXES], x2[DRV_MOMENT_NOTCH_AXES];
    float y1[DRV_MOMENT_NOTCH_AXES], y2[DRV_MOMENT_NOTCH_AXES];
    float pending_x[DRV_MOMENT_NOTCH_AXES], pending_y[DRV_MOMENT_NOTCH_AXES];
    uint8_t active;                          /* 系数有效，陷波生效 */
    uint8_t primed[DRV_MOMENT_NOTCH_AXES];   /* 历史里是真实样本 */
    uint8_t pending[DRV_MOMENT_NOTCH_AXES];  /* 本拍 Apply 过、等待 Commit */
} DRV_MomentNotch;

/* 0 = 关；否则 [HZ_MIN, HZ_MAX]。 */
static inline uint8_t DRV_MomentNotch_FrequencyValid(float f0_hz)
{
    return (isfinite(f0_hz) &&
            ((f0_hz == 0.0f) ||
             ((f0_hz >= DRV_MOMENT_NOTCH_HZ_MIN) && (f0_hz <= DRV_MOMENT_NOTCH_HZ_MAX))))
        ? 1U : 0U;
}

static inline uint8_t DRV_MomentNotch_QValid(float q)
{
    return (isfinite(q) && (q >= DRV_MOMENT_NOTCH_Q_MIN) && (q <= DRV_MOMENT_NOTCH_Q_MAX))
        ? 1U : 0U;
}

/*
 * 按 f0、Q、深度 d 与采样间隔 dt 设计系数（双线性 + 预畸变到 f0）：
 *   ω0 = 2π·f0，K = ω0 / tan(ω0·dt/2)
 *   b = [K² + d·(ω0/Q)·K + ω0², 2(ω0² − K²), K² − d·(ω0/Q)·K + ω0²] / a0
 *   a = [1, 2(ω0² − K²)/a0, (K² − (ω0/Q)·K + ω0²)/a0]，a0 = K² + (ω0/Q)·K + ω0²
 * 任何输入非法或 f0 ≥ 0.45/dt 返回 0，c 原样不动。
 */
static inline uint8_t DRV_MomentNotch_Design(DRV_MomentNotchCoef *c, float f0_hz,
                                             float q, float depth, float dt_s)
{
    float w0, k, kk, w2, bw, a0;
    DRV_MomentNotchCoef out;

    if ((c == NULL) || (DRV_MomentNotch_QValid(q) == 0U) || !isfinite(f0_hz) ||
        !(f0_hz > 0.0f) || !isfinite(depth) || (depth < 0.0f) || (depth > 1.0f) ||
        !isfinite(dt_s) || (dt_s < DRV_MOMENT_NOTCH_DT_MIN_S) ||
        (dt_s > DRV_MOMENT_NOTCH_DT_MAX_S) ||
        !((f0_hz * dt_s) < DRV_MOMENT_NOTCH_NYQUIST_FRACTION)) {
        return 0U;
    }
    w0 = DRV_MOMENT_NOTCH_TWO_PI * f0_hz;
    k = w0 / tanf(0.5f * w0 * dt_s);
    kk = k * k;
    w2 = w0 * w0;
    bw = (w0 / q) * k;
    a0 = kk + bw + w2;
    out.b0 = (kk + (depth * bw) + w2) / a0;
    out.b1 = (2.0f * (w2 - kk)) / a0;
    out.b2 = (kk - (depth * bw) + w2) / a0;
    out.a1 = out.b1;
    out.a2 = (kk - bw + w2) / a0;
    if (!isfinite(out.b0) || !isfinite(out.b1) || !isfinite(out.b2) || !isfinite(out.a2)) {
        return 0U;
    }
    *c = out;
    return 1U;
}

/* 全部作废：直通，下次生效时从稳态起步。 */
static inline void DRV_MomentNotch_Reset(DRV_MomentNotch *n)
{
    if (n != NULL) {
        memset(n, 0, sizeof(*n));
    }
}

/*
 * 每个**推进拍**（速率环 Step 拍）先调一次：按本拍 dt 与参数刷新系数，只有参数或
 * dt 变了才重算。f0 = 0 是"关"。返回 1 = 本拍陷波生效，0 = 直通。
 */
static inline uint8_t DRV_MomentNotch_Configure(DRV_MomentNotch *n, float f0_hz,
                                                float q, float dt_s)
{
    DRV_MomentNotchCoef coef;

    if (n == NULL) {
        return 0U;
    }
    if ((n->active != 0U) && (f0_hz == n->coef_hz) && (q == n->coef_q) &&
        (dt_s == n->coef_dt_s)) {
        return 1U;
    }
    if ((DRV_MomentNotch_FrequencyValid(f0_hz) == 0U) || !(f0_hz > 0.0f) ||
        (DRV_MomentNotch_Design(&coef, f0_hz, q, DRV_MOMENT_NOTCH_DEPTH, dt_s) == 0U)) {
        DRV_MomentNotch_Reset(n);
        return 0U;
    }
    if (n->active == 0U) {
        /* 从直通切进来：历史里不是这一段的样本，第一拍按稳态起步。 */
        memset(n->primed, 0, sizeof(n->primed));
        memset(n->pending, 0, sizeof(n->pending));
    }
    n->coef = coef;
    n->coef_hz = f0_hz;
    n->coef_q = q;
    n->coef_dt_s = dt_s;
    n->active = 1U;
    return 1U;
}

/*
 * 一个轴一个样本：只算输出、记为待提交，不动历史（同拍可重算）。
 * 未生效或输入非有限时原样返回 x（逐位直通）。
 */
static inline float DRV_MomentNotch_Apply(DRV_MomentNotch *n, uint32_t axis, float x)
{
    float y;

    if ((n == NULL) || (axis >= DRV_MOMENT_NOTCH_AXES)) {
        return x;
    }
    n->pending[axis] = 0U;
    if ((n->active == 0U) || !isfinite(x)) {
        return x;
    }
    if (n->primed[axis] == 0U) {
        y = x;   /* 历史按"一直是 x"的直流稳态处理：输出等于输入 */
    } else {
        y = (n->coef.b0 * x) + (n->coef.b1 * n->x1[axis]) + (n->coef.b2 * n->x2[axis]) -
            (n->coef.a1 * n->y1[axis]) - (n->coef.a2 * n->y2[axis]);
        if (!isfinite(y)) {
            /* 历史被坏值污染：这一拍直通，下一次从稳态重新起步。 */
            n->primed[axis] = 0U;
            return x;
        }
        if (fabsf(y) < DRV_MOMENT_NOTCH_FLUSH_ABS) {
            y = 0.0f;
        }
    }
    n->pending_x[axis] = x;
    n->pending_y[axis] = y;
    n->pending[axis] = 1U;
    return y;
}

/* 推进拍末尾：把本拍最后一次 Apply 的样本推进历史。 */
static inline void DRV_MomentNotch_Commit(DRV_MomentNotch *n)
{
    if (n == NULL) {
        return;
    }
    for (uint32_t axis = 0U; axis < DRV_MOMENT_NOTCH_AXES; ++axis) {
        if (n->pending[axis] == 0U) {
            continue;
        }
        if (n->primed[axis] == 0U) {
            n->x2[axis] = n->pending_x[axis];
            n->x1[axis] = n->pending_x[axis];
            n->y2[axis] = n->pending_y[axis];
            n->y1[axis] = n->pending_y[axis];
            n->primed[axis] = 1U;
        } else {
            n->x2[axis] = n->x1[axis];
            n->x1[axis] = n->pending_x[axis];
            n->y2[axis] = n->y1[axis];
            n->y1[axis] = n->pending_y[axis];
        }
        n->pending[axis] = 0U;
    }
}

/* 不推进的拍（速率环只 Evaluate）：丢掉本拍的待提交样本。 */
static inline void DRV_MomentNotch_Discard(DRV_MomentNotch *n)
{
    if (n != NULL) {
        memset(n->pending, 0, sizeof(n->pending));
    }
}

/*
 * 接到速率环输出上（飞控与光杆辨识共用这一处）：横滚/俯仰只滤 P + I − D，
 * 先过 first、再过 second（两级串联，滤的是同一个反馈信号），前馈与陀螺耦合项
 * 原样叠回；然后按同一组力矩上下限重新钳位、重算饱和标志——规则与
 * drv_rate_control.c 的 rate_apply_moment_limit 相同（上下限非有限或不成序视为无限）。
 *
 * 哪一级为 NULL 或未生效，那一级就是逐位直通（Apply 原样返回输入），所以第二级关着
 * 时结果与只有一级时逐位相同；两级都未生效时什么都不改（逐位不变）。
 *
 * 与抗积分饱和的关系：速率环本拍的积分钳位看的是陷波前的力矩；陷波直流增益为 1，
 * 稳态饱和判断一致，瞬态差异由重算后的饱和标志经下一拍的
 * saturation_*_active 反馈给积分器（与分配器饱和同一条路）。
 */
static inline void DRV_MomentNotch_ApplyCascadeToRateOutput(DRV_MomentNotch *first,
                                                            DRV_MomentNotch *second,
                                                            const DRV_RateControl_Input *input,
                                                            DRV_RateControl_Output *output)
{
    const uint8_t first_on = ((first != NULL) && (first->active != 0U)) ? 1U : 0U;
    const uint8_t second_on = ((second != NULL) && (second->active != 0U)) ? 1U : 0U;

    if ((input == NULL) || (output == NULL) || ((first_on == 0U) && (second_on == 0U))) {
        return;
    }
    for (uint32_t axis = 0U; axis < DRV_MOMENT_NOTCH_AXES; ++axis) {
        const float feedback = output->p_term[axis] + output->i_term[axis] -
                               output->d_term[axis];
        const float bypass = output->moment_unsat[axis] - feedback;
        float positive = input->saturation_positive[axis];
        float negative = input->saturation_negative[axis];
        float filtered = DRV_MomentNotch_Apply(first, axis, feedback);

        filtered = DRV_MomentNotch_Apply(second, axis, filtered);
        output->moment_unsat[axis] = filtered + bypass;
        output->moment_cmd[axis] = output->moment_unsat[axis];
        output->saturated_pos[axis] = 0U;
        output->saturated_neg[axis] = 0U;
        if (!isfinite(positive) || !isfinite(negative) || (positive <= negative)) {
            continue;
        }
        if (output->moment_cmd[axis] >= positive) {
            output->moment_cmd[axis] = positive;
            output->saturated_pos[axis] = 1U;
        } else if (output->moment_cmd[axis] <= negative) {
            output->moment_cmd[axis] = negative;
            output->saturated_neg[axis] = 1U;
        }
    }
}

/* 只有一级：等同于第二级为 NULL。 */
static inline void DRV_MomentNotch_ApplyToRateOutput(DRV_MomentNotch *n,
                                                     const DRV_RateControl_Input *input,
                                                     DRV_RateControl_Output *output)
{
    DRV_MomentNotch_ApplyCascadeToRateOutput(n, NULL, input, output);
}

#ifdef __cplusplus
}
#endif

#endif /* DRV_MOMENT_NOTCH_H */

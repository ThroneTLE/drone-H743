/*
 * 按转速跟踪的陀螺陷波。全部是纯算术；能在 PC 上判对错，因此不允许出现任何硬件依赖。
 * 设计理由与信号契约见 Driver/Inc/drv_rpm_notch.h。
 */
#include "drv_rpm_notch.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define RPM_NOTCH_TWO_PI       6.28318530717958647692f
#define RPM_NOTCH_MAX_DT_S     0.02f
/* 权重爬到目标时的容差：20 次累加 0.05f 可能差一个 ulp，差这点就直接落到目标上。 */
#define RPM_NOTCH_RAMP_SNAP    1.0001f
/* |y| 超过它（含 inf），或 y 是 NaN，就当状态已坏。陀螺量级离它差 36 个数量级。 */
#define RPM_NOTCH_FINITE_MAX   3.0e38f

static float notch_clamp01(float value)
{
    return (value < 0.0f) ? 0.0f : ((value > 1.0f) ? 1.0f : value);
}

uint8_t DRV_Notch_Design(DRV_NotchCoef *c, float f0_hz, float q, float fs_hz)
{
    float w0, cos_w0, sin_w0, alpha, k;
    DRV_NotchCoef out;

    if ((c == NULL) || (isfinite(f0_hz) == 0) || (isfinite(q) == 0) || (isfinite(fs_hz) == 0)) {
        return 0U;
    }
    if (!(fs_hz > 0.0f) || !(f0_hz > 0.0f) || !(f0_hz < (0.5f * fs_hz)) ||
        (q < DRV_RPM_NOTCH_Q_MIN) || (q > DRV_RPM_NOTCH_Q_MAX)) {
        return 0U;
    }
    w0 = RPM_NOTCH_TWO_PI * f0_hz / fs_hz;
    cos_w0 = cosf(w0);
    /* w0 ∈ (0, π)，sin 恒正：一次 cosf 加一次开方，省掉 sinf。 */
    sin_w0 = sqrtf(fmaxf(0.0f, 1.0f - (cos_w0 * cos_w0)));
    alpha = sin_w0 / (2.0f * q);
    k = 1.0f / (1.0f + alpha);
    out.b0 = k;
    out.b1 = -2.0f * cos_w0 * k;
    out.a2 = (1.0f - alpha) * k;
    if ((isfinite(out.b0) == 0) || (isfinite(out.b1) == 0) || (isfinite(out.a2) == 0)) {
        return 0U;
    }
    *c = out;
    return 1U;
}

void DRV_Notch_Reset(DRV_NotchState *s, float x)
{
    if (s == NULL) {
        return;
    }
    s->x1 = x;
    s->x2 = x;
    s->y1 = x;
    s->y2 = x;
}

float DRV_Notch_Step(DRV_NotchState *s, const DRV_NotchCoef *c, float x)
{
    float y;

    if ((s == NULL) || (c == NULL) || (isfinite(x) == 0)) {
        return x;
    }
    /* y = b0·x + b1·x1 + b2·x2 − a1·y1 − a2·y2，其中 b2 = b0、a1 = b1。 */
    y = (c->b0 * (x + s->x2)) + (c->b1 * (s->x1 - s->y1)) - (c->a2 * s->y2);
    if (isfinite(y) == 0) {
        /* 状态被坏值污染：就地回到以当前输入为直流的稳态，本拍原样输出。 */
        DRV_Notch_Reset(s, x);
        return x;
    }
    if (fabsf(y) < DRV_RPM_NOTCH_FLUSH_ABS) {
        y = 0.0f;
    }
    s->x2 = s->x1;
    s->x1 = x;
    s->y2 = s->y1;
    s->y1 = y;
    return y;
}

void DRV_RpmNotch_DefaultConfig(DRV_RpmNotchConfig *cfg)
{
    if (cfg == NULL) {
        return;
    }
    cfg->harmonic_mask = 0x01U;
    cfg->q = 3.0f;
    cfg->min_hz = 50.0f;
    cfg->fade_hz = 20.0f;
    cfg->ramp_s = 0.020f;
    cfg->slew_hz_per_s = 3000.0f;
    cfg->watchdog_s = 0.040f;
}

uint8_t DRV_RpmNotch_ConfigValid(const DRV_RpmNotchConfig *cfg)
{
    if (cfg == NULL) {
        return 0U;
    }
    if ((cfg->harmonic_mask == 0U) || (cfg->harmonic_mask > 0x07U)) {
        return 0U;
    }
    if ((isfinite(cfg->q) == 0) || (cfg->q < DRV_RPM_NOTCH_Q_MIN) || (cfg->q > DRV_RPM_NOTCH_Q_MAX)) {
        return 0U;
    }
    if ((isfinite(cfg->min_hz) == 0) || (cfg->min_hz < 0.0f) ||
        (isfinite(cfg->fade_hz) == 0) || !(cfg->fade_hz > 0.0f)) {
        return 0U;
    }
    if ((isfinite(cfg->ramp_s) == 0) || !(cfg->ramp_s > 0.0f) || (cfg->ramp_s > 1.0f) ||
        (isfinite(cfg->slew_hz_per_s) == 0) || !(cfg->slew_hz_per_s > 0.0f) ||
        (isfinite(cfg->watchdog_s) == 0) || !(cfg->watchdog_s > 0.0f) || (cfg->watchdog_s > 1.0f)) {
        return 0U;
    }
    return 1U;
}

uint8_t DRV_RpmNotch_Init(DRV_RpmNotch *n, const DRV_RpmNotchConfig *cfg)
{
    if (n == NULL) {
        return 0U;
    }
    memset(n, 0, sizeof(*n));
    if (DRV_RpmNotch_ConfigValid(cfg) == 0U) {
        /* 配置留全 0：谐波掩码为 0，SetFs 也会把 fs 保持为 0，整组永远直通。 */
        return 0U;
    }
    n->cfg = *cfg;
    return 1U;
}

void DRV_RpmNotch_SetFs(DRV_RpmNotch *n, float fs_hz)
{
    if (n == NULL) {
        return;
    }
    /* 掩码为 0 = Init 拒收了配置（结构保持全 0）：整组直通。每拍都调，不重跑整套校验。 */
    if ((isfinite(fs_hz) == 0) || !(fs_hz > 0.0f) || (n->cfg.harmonic_mask == 0U)) {
        n->fs_hz = 0.0f;   /* 采样率未知：Apply 直通 */
        return;
    }
    n->fs_hz = fs_hz;
    if ((n->coef_fs_hz > 0.0f) &&
        (fabsf(fs_hz - n->coef_fs_hz) <= (DRV_RPM_NOTCH_RECOMPUTE_FS_REL * n->coef_fs_hz))) {
        return;   /* 0.1% 以内的估计抖动不值得重算系数 */
    }
    n->coef_fs_hz = fs_hz;
    n->coef_dirty = (uint8_t)((1U << DRV_RPM_NOTCH_MOTORS) - 1U);
    n->ramp_step = 1.0f / (n->cfg.ramp_s * fs_hz);
    n->watchdog_samples = (uint32_t)((n->cfg.watchdog_s * fs_hz) + 0.5f);
}

static uint8_t rpm_notch_motor_active(const DRV_RpmNotch *n, uint8_t motor)
{
    const uint32_t base = (uint32_t)motor * DRV_RPM_NOTCH_HARMONICS;

    for (uint32_t h = 0U; h < DRV_RPM_NOTCH_HARMONICS; ++h) {
        if (n->slot[base + h].active != 0U) {
            return 1U;
        }
    }
    return 0U;
}

/* 频率 → 目标权重：低端 min..min+fade 线性淡入，高端 0.40..0.45·fs 线性淡出。 */
static float rpm_notch_target_weight(const DRV_RpmNotch *n, float f_hz)
{
    const float fs = n->coef_fs_hz;
    const float low = notch_clamp01((f_hz - n->cfg.min_hz) / n->cfg.fade_hz);
    const float span = (DRV_RPM_NOTCH_TOP_FADE_END - DRV_RPM_NOTCH_TOP_FADE_START) * fs;
    const float high = notch_clamp01(((DRV_RPM_NOTCH_TOP_FADE_END * fs) - f_hz) / span);

    return low * high;
}

void DRV_RpmNotch_SetMotor(DRV_RpmNotch *n, uint8_t motor, float fundamental_hz, uint8_t fresh, float dt_s)
{
    uint32_t base;
    uint8_t dirty;
    float hz = fundamental_hz;

    if ((n == NULL) || (motor >= DRV_RPM_NOTCH_MOTORS)) {
        return;
    }
    n->samples_since_update = 0U;
    n->watchdog_tripped = 0U;
    base = (uint32_t)motor * DRV_RPM_NOTCH_HARMONICS;
    if ((fresh == 0U) || (isfinite(hz) == 0) || !(hz > 0.0f) || !(n->coef_fs_hz > 0.0f)) {
        for (uint32_t h = 0U; h < DRV_RPM_NOTCH_HARMONICS; ++h) {
            n->slot[base + h].target = 0.0f;   /* 中心保持：淡出沿用最后一个中心 */
        }
        return;
    }
    if ((rpm_notch_motor_active(n, motor) != 0U) && (n->motor_hz_valid[motor] != 0U)) {
        const float dt = (isfinite(dt_s) == 0) ? 0.0f :
                         ((dt_s < 0.0f) ? 0.0f : ((dt_s > RPM_NOTCH_MAX_DT_S) ? RPM_NOTCH_MAX_DT_S : dt_s));
        const float step = n->cfg.slew_hz_per_s * dt;
        const float delta = hz - n->motor_hz[motor];

        if (delta > step) {
            hz = n->motor_hz[motor] + step;
            n->slew_clamp_count++;
        } else if (delta < -step) {
            hz = n->motor_hz[motor] - step;
            n->slew_clamp_count++;
        }
    }
    n->motor_hz[motor] = hz;
    n->motor_hz_valid[motor] = 1U;
    dirty = (uint8_t)((n->coef_dirty >> motor) & 1U);
    n->coef_dirty = (uint8_t)(n->coef_dirty & (uint8_t)~(1U << motor));

    for (uint32_t h = 0U; h < DRV_RPM_NOTCH_HARMONICS; ++h) {
        DRV_RpmNotchSlot *slot = &n->slot[base + h];
        const float f = (float)(h + 1U) * hz;
        float target;

        if ((n->cfg.harmonic_mask & (1U << h)) == 0U) {
            slot->target = 0.0f;
            continue;
        }
        target = rpm_notch_target_weight(n, f);
        if ((slot->coef_valid == 0U) || (fabsf(f - slot->center_hz) > DRV_RPM_NOTCH_RECOMPUTE_HZ) ||
            (dirty != 0U)) {
            if (DRV_Notch_Design(&slot->coef, f, n->cfg.q, n->coef_fs_hz) != 0U) {
                slot->center_hz = f;
                slot->coef_valid = 1U;
            } else {
                target = 0.0f;   /* 设计失败：旧系数留着淡出，从未有效的槽就一直不启用 */
            }
        }
        slot->target = target;
    }
}

/*
 * 一个槽走一步（三轴）：必要时启用、权重爬一格、DF1、按权重混合、淡完即停。
 * DF1 的式子与 DRV_Notch_Step 相同，这里按槽内联三轴：Debug 是 -O0，每轴一次函数
 * 调用本身就要十几条指令，而这是每个 IMU 样本都走的路径。
 */
static void rpm_notch_run_slot(DRV_RpmNotchSlot *slot, float ramp_step, float x[DRV_RPM_NOTCH_AXES])
{
    float gap, b0, b1, a2, w;

    if (slot->active == 0U) {
        /* 启用：以级联到这一级的当前值为直流稳态起步，权重从 0 爬升盖住陷波自己的振铃。 */
        for (uint32_t a = 0U; a < DRV_RPM_NOTCH_AXES; ++a) {
            DRV_Notch_Reset(&slot->state[a], x[a]);
        }
        slot->active = 1U;
        slot->weight = 0.0f;
    }
    gap = slot->target - slot->weight;
    if (fabsf(gap) <= (ramp_step * RPM_NOTCH_RAMP_SNAP)) {
        slot->weight = slot->target;
    } else {
        slot->weight += (gap > 0.0f) ? ramp_step : -ramp_step;
    }
    b0 = slot->coef.b0;
    b1 = slot->coef.b1;
    a2 = slot->coef.a2;
    w = slot->weight;
    for (uint32_t a = 0U; a < DRV_RPM_NOTCH_AXES; ++a) {
        DRV_NotchState *st = &slot->state[a];
        const float xa = x[a];
        float y = (b0 * (xa + st->x2)) + (b1 * (st->x1 - st->y1)) - (a2 * st->y2);

        if (!(fabsf(y) <= RPM_NOTCH_FINITE_MAX)) {
            DRV_Notch_Reset(st, xa);   /* 状态被坏值污染（NaN 也落在这里）：回稳态，本拍不混 */
            continue;
        }
        if (fabsf(y) < DRV_RPM_NOTCH_FLUSH_ABS) {
            y = 0.0f;
        }
        st->x2 = st->x1;
        st->x1 = xa;
        st->y2 = st->y1;
        st->y1 = y;
        x[a] = xa + (w * (y - xa));
    }
    if ((slot->weight == 0.0f) && (slot->target == 0.0f)) {
        slot->active = 0U;
    }
}

void DRV_RpmNotch_Apply(DRV_RpmNotch *n, const float in[3], float out[3])
{
    float x[DRV_RPM_NOTCH_AXES];

    if ((in == NULL) || (out == NULL)) {
        return;
    }
    for (uint32_t a = 0U; a < DRV_RPM_NOTCH_AXES; ++a) {
        x[a] = in[a];
    }
    if ((n == NULL) || !(n->fs_hz > 0.0f)) {
        for (uint32_t a = 0U; a < DRV_RPM_NOTCH_AXES; ++a) {
            out[a] = x[a];
        }
        return;
    }
    if ((isfinite(x[0]) == 0) || (isfinite(x[1]) == 0) || (isfinite(x[2]) == 0)) {
        /* 本拍原样放行（让上游的非有限判据照常看到它），下一个有限样本处回到稳态。 */
        n->nonfinite_count++;
        n->reset_pending = 1U;
        for (uint32_t a = 0U; a < DRV_RPM_NOTCH_AXES; ++a) {
            out[a] = x[a];
        }
        return;
    }
    if (n->reset_pending != 0U) {
        for (uint32_t s = 0U; s < DRV_RPM_NOTCH_SLOTS; ++s) {
            if (n->slot[s].active == 0U) {
                continue;
            }
            for (uint32_t a = 0U; a < DRV_RPM_NOTCH_AXES; ++a) {
                DRV_Notch_Reset(&n->slot[s].state[a], x[a]);
            }
        }
        n->reset_pending = 0U;
        n->reset_count++;
    }
    n->samples_since_update++;
    if (n->samples_since_update > n->watchdog_samples) {
        /* 控制拍停了（转速不再更新）：所有目标归零，按正常淡出退场。 */
        if (n->watchdog_tripped == 0U) {
            n->watchdog_tripped = 1U;
            n->watchdog_count++;
        }
        for (uint32_t s = 0U; s < DRV_RPM_NOTCH_SLOTS; ++s) {
            n->slot[s].target = 0.0f;
        }
    }

    for (uint32_t s = 0U; s < DRV_RPM_NOTCH_SLOTS; ++s) {
        if ((n->slot[s].active != 0U) ||
            ((n->slot[s].target > 0.0f) && (n->slot[s].coef_valid != 0U))) {
            rpm_notch_run_slot(&n->slot[s], n->ramp_step, x);
        }   /* 其余不活动的槽不步进，状态也不碰 */
    }
    for (uint32_t a = 0U; a < DRV_RPM_NOTCH_AXES; ++a) {
        out[a] = x[a];
    }
}

void DRV_RpmNotch_Reset(DRV_RpmNotch *n)
{
    if (n != NULL) {
        n->reset_pending = 1U;
    }
}

float DRV_RpmNotch_SlotWeight(const DRV_RpmNotch *n, uint8_t motor, uint8_t harmonic)
{
    const DRV_RpmNotchSlot *slot;

    if ((n == NULL) || (motor >= DRV_RPM_NOTCH_MOTORS) || (harmonic == 0U) ||
        (harmonic > DRV_RPM_NOTCH_HARMONICS)) {
        return 0.0f;
    }
    slot = &n->slot[((uint32_t)motor * DRV_RPM_NOTCH_HARMONICS) + (uint32_t)harmonic - 1U];
    return (slot->active != 0U) ? slot->weight : 0.0f;
}

uint8_t DRV_RpmNotch_ActiveSlots(const DRV_RpmNotch *n)
{
    uint8_t count = 0U;

    if (n == NULL) {
        return 0U;
    }
    for (uint32_t s = 0U; s < DRV_RPM_NOTCH_SLOTS; ++s) {
        if (n->slot[s].active != 0U) {
            count++;
        }
    }
    return count;
}

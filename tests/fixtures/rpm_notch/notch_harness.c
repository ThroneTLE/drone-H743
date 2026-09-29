/*
 * 宿主装置（算法层）：把 Driver/Src/drv_rpm_notch.c 编成 DLL 给 ctypes 调。
 *
 * 只做三件事：
 *   - 批量运行器：一次把整段输入按"每个样本 Apply、指定样本上先 SetMotor"跑完，
 *     免得 Python 逐样本穿 ctypes（实录重放是几万次调用）；
 *   - 同一个陷波分别走公开的单步参照（DRV_Notch_Step）与固件热路径（Apply 里内联的 DF1），
 *     让频响/直流/相位判据落在真正每个样本都跑的那份代码上；
 *   - 只有 C 里才看得见的检查：状态里有没有次正规数（fpclassify）。
 * 滤波器本身一行不替：判据要能在真代码上失败才有意义。
 */

#include "drv_rpm_notch.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

static DRV_RpmNotch bank;

uint8_t harness_bank_init(uint8_t mask, float q, float min_hz, float fade_hz)
{
    DRV_RpmNotchConfig cfg;

    DRV_RpmNotch_DefaultConfig(&cfg);
    cfg.harmonic_mask = mask;
    cfg.q = q;
    cfg.min_hz = min_hz;
    cfg.fade_hz = fade_hz;
    return DRV_RpmNotch_Init(&bank, &cfg);
}

void harness_bank_set_fs(float fs_hz) { DRV_RpmNotch_SetFs(&bank, fs_hz); }
void harness_bank_set_motor(uint8_t motor, float hz, uint8_t fresh, float dt_s)
{
    DRV_RpmNotch_SetMotor(&bank, motor, hz, fresh, dt_s);
}
void harness_bank_apply(const float *in, float *out) { DRV_RpmNotch_Apply(&bank, in, out); }
void harness_bank_reset(void) { DRV_RpmNotch_Reset(&bank); }
float harness_slot_weight(uint8_t motor, uint8_t harmonic)
{
    return DRV_RpmNotch_SlotWeight(&bank, motor, harmonic);
}
uint8_t harness_active_slots(void) { return DRV_RpmNotch_ActiveSlots(&bank); }
float harness_slot_center(uint32_t slot) { return bank.slot[slot].center_hz; }
float harness_slot_target(uint32_t slot) { return bank.slot[slot].target; }
uint8_t harness_slot_active(uint32_t slot) { return bank.slot[slot].active; }
uint8_t harness_slot_coef_valid(uint32_t slot) { return bank.slot[slot].coef_valid; }
void harness_slot_coef(uint32_t slot, float *out3)
{
    out3[0] = bank.slot[slot].coef.b0;
    out3[1] = bank.slot[slot].coef.b1;
    out3[2] = bank.slot[slot].coef.a2;
}
void harness_slot_state(uint32_t slot, uint32_t axis, float *out4)
{
    out4[0] = bank.slot[slot].state[axis].x1;
    out4[1] = bank.slot[slot].state[axis].x2;
    out4[2] = bank.slot[slot].state[axis].y1;
    out4[3] = bank.slot[slot].state[axis].y2;
}
float harness_motor_hz(uint8_t motor) { return bank.motor_hz[motor]; }

/* 0 slew 1 nonfinite 2 watchdog 3 reset */
uint32_t harness_counter(uint32_t which)
{
    switch (which) {
    case 0: return bank.slew_clamp_count;
    case 1: return bank.nonfinite_count;
    case 2: return bank.watchdog_count;
    default: return bank.reset_count;
    }
}

/*
 * 批量运行。第 i 个样本：tick[i] 非 0 时先对两个电机 SetMotor(hz[2i+m], fresh[2i+m], dt[i])，
 * 再 Apply(in[3i..], out[3i..])。weights 非空时记下每个样本之后 6 个槽的权重（活动才计）。
 */
void harness_run(uint32_t n, const float *in, float *out, const uint8_t *tick, const float *hz,
                 const uint8_t *fresh, const float *dt, float *weights)
{
    for (uint32_t i = 0U; i < n; ++i) {
        if ((tick != NULL) && (tick[i] != 0U)) {
            for (uint8_t m = 0U; m < DRV_RPM_NOTCH_MOTORS; ++m) {
                DRV_RpmNotch_SetMotor(&bank, m, hz[(2U * i) + m], fresh[(2U * i) + m], dt[i]);
            }
        }
        DRV_RpmNotch_Apply(&bank, &in[3U * i], &out[3U * i]);
        if (weights != NULL) {
            for (uint32_t s = 0U; s < DRV_RPM_NOTCH_SLOTS; ++s) {
                weights[(DRV_RPM_NOTCH_SLOTS * i) + s] =
                    (bank.slot[s].active != 0U) ? bank.slot[s].weight : 0.0f;
            }
        }
    }
}

/* 单个陷波从零状态跑一段（冲激响应 → 频响）。 */
uint8_t harness_single(float f0, float q, float fs, uint32_t n, const float *in, float *out)
{
    DRV_NotchCoef c;
    DRV_NotchState s;

    if (DRV_Notch_Design(&c, f0, q, fs) == 0U) {
        return 0U;
    }
    memset(&s, 0, sizeof(s));
    for (uint32_t i = 0U; i < n; ++i) {
        out[i] = DRV_Notch_Step(&s, &c, in[i]);
    }
    return 1U;
}

/* 同上，但每个样本用 centers[i] 重设计（中心变化 < 0.1 Hz 也照样重设计：这里测的是 DF1 本身）。 */
uint8_t harness_single_varying(const float *centers, float q, float fs, uint32_t n,
                               const float *in, float *out)
{
    DRV_NotchCoef c;
    DRV_NotchState s;

    if (DRV_Notch_Design(&c, centers[0], q, fs) == 0U) {
        return 0U;
    }
    DRV_Notch_Reset(&s, in[0]);
    for (uint32_t i = 0U; i < n; ++i) {
        if (DRV_Notch_Design(&c, centers[i], q, fs) == 0U) {
            return 0U;
        }
        out[i] = DRV_Notch_Step(&s, &c, in[i]);
    }
    return 1U;
}

/*
 * 同一个陷波，但走固件真正每个样本都跑的路径（DRV_RpmNotch_Apply 里内联的那份 DF1）：
 * 电机 1 的 1x 槽中心 f0、电机 2 过期；先喂 lead 个零让权重爬满（零输入下状态保持为 0），
 * 再把 in[] 三轴相同喂进去，out 取轴 0。控制拍每 2 个样本一次（看门狗不触发；中心不动，
 * 系数逐位不变）。返回 0 = 配置被拒或权重没爬到 1。
 */
uint8_t harness_bank_response(float f0, float q, float fs, uint32_t lead, uint32_t n,
                              const float *in, float *out)
{
    const float zero[3] = { 0.0f, 0.0f, 0.0f };
    float x[3], y[3];

    if (harness_bank_init(0x01U, q, 20.0f, 5.0f) == 0U) {
        return 0U;
    }
    DRV_RpmNotch_SetFs(&bank, fs);
    for (uint32_t i = 0U; i < (lead + n); ++i) {
        if ((i % 2U) == 0U) {
            DRV_RpmNotch_SetMotor(&bank, 0U, f0, 1U, 2.0f / fs);
            DRV_RpmNotch_SetMotor(&bank, 1U, 0.0f, 0U, 2.0f / fs);
        }
        if (i < lead) {
            DRV_RpmNotch_Apply(&bank, zero, y);
            continue;
        }
        if ((i == lead) && (DRV_RpmNotch_SlotWeight(&bank, 0U, 1U) != 1.0f)) {
            return 0U;
        }
        x[0] = in[i - lead];
        x[1] = x[0];
        x[2] = x[0];
        DRV_RpmNotch_Apply(&bank, x, y);
        out[i - lead] = y[0];
    }
    return 1U;
}

/* 设计失败时 c 不能被改：先填哨兵值再调。返回 (ok, 哨兵是否完好)。 */
uint8_t harness_design_untouched(float f0, float q, float fs, uint8_t *ok)
{
    DRV_NotchCoef c = { 123.0f, 456.0f, 789.0f };

    *ok = DRV_Notch_Design(&c, f0, q, fs);
    return ((c.b0 == 123.0f) && (c.b1 == 456.0f) && (c.a2 == 789.0f)) ? 1U : 0U;
}

uint8_t harness_design(float f0, float q, float fs, float *out3)
{
    DRV_NotchCoef c;

    if (DRV_Notch_Design(&c, f0, q, fs) == 0U) {
        return 0U;
    }
    out3[0] = c.b0;
    out3[1] = c.b1;
    out3[2] = c.a2;
    return 1U;
}

/* 任何槽任何轴的状态里有次正规数就返回 1。 */
uint8_t harness_bank_has_subnormal(void)
{
    for (uint32_t s = 0U; s < DRV_RPM_NOTCH_SLOTS; ++s) {
        for (uint32_t a = 0U; a < DRV_RPM_NOTCH_AXES; ++a) {
            const DRV_NotchState *st = &bank.slot[s].state[a];
            if ((fpclassify(st->x1) == FP_SUBNORMAL) || (fpclassify(st->x2) == FP_SUBNORMAL) ||
                (fpclassify(st->y1) == FP_SUBNORMAL) || (fpclassify(st->y2) == FP_SUBNORMAL)) {
                return 1U;
            }
        }
    }
    return 0U;
}

/*
 * 冲击之后连续 seconds 秒零输入，途中任何一步出现次正规状态或次正规输出都返回 1。
 * 陷波保持在 center（tick 每 2 个样本一次，保持新鲜），fs 固定。
 */
uint8_t harness_zero_tail_has_subnormal(float fs, float center, float seconds)
{
    const uint32_t n = (uint32_t)(fs * seconds);
    float in[3] = { 1.0f, -1.0f, 0.5f };
    float out[3];

    for (uint32_t i = 0U; i < n; ++i) {
        if ((i % 2U) == 0U) {
            DRV_RpmNotch_SetMotor(&bank, 0U, center, 1U, 2.0f / fs);
            DRV_RpmNotch_SetMotor(&bank, 1U, center * 1.02f, 1U, 2.0f / fs);
        }
        DRV_RpmNotch_Apply(&bank, in, out);
        in[0] = 0.0f;
        in[1] = 0.0f;
        in[2] = 0.0f;
        for (uint32_t a = 0U; a < 3U; ++a) {
            if (fpclassify(out[a]) == FP_SUBNORMAL) {
                return 1U;
            }
        }
        if (harness_bank_has_subnormal() != 0U) {
            return 1U;
        }
    }
    return 0U;
}

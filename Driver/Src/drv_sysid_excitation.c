/*
 * 辨识激励剖面。纯算术 + 纯函数，宿主可逐样本判对错。
 */
#include "drv_sysid_excitation.h"

#include <math.h>
#include <stddef.h>

static uint8_t exc_finite(float value)
{
    return (isfinite(value) != 0) ? 1U : 0U;
}

/*
 * 把"上一段电平 -> 本段电平"的过渡摊在 ramp_ms 上，返回该时刻的值与斜率。
 * 方波类剖面全部经过这里，所以 α_ff 处处有界（见头文件）。
 */
static void exc_ramped_level(float previous, float current,
                             uint32_t since_edge_ms, uint32_t ramp_ms,
                             float *value, float *slope)
{
    if ((since_edge_ms >= ramp_ms) || (ramp_ms == 0U)) {
        *value = current;
        *slope = 0.0f;
        return;
    }
    const float fraction = (float)since_edge_ms / (float)ramp_ms;
    *value = previous + ((current - previous) * fraction);
    *slope = (current - previous) / ((float)ramp_ms * 0.001f);
}

/*
 * 31 位 LFSR（x^31 + x^28 + 1），第 index 位的输出。
 *
 * 两处不是随手写的：
 *
 * 1. **种子先散开再用**。像 1、7 这种小种子高 30 位全是 0，直接跑 LFSR 会有长达
 *    30 拍的"只左移、反馈恒为 0"的暖机期；而我们一趟激励也就几十位，于是
 *    seed=1 和 seed=7 会给出**完全相同**的序列。种子形同虚设，两次实验以为换了
 *    激励其实没换——这种错在数据里是看不出来的。先乘 Knuth 常数填满寄存器。
 * 2. **取最高位而不是最低位**。左移式 LFSR 的最低位是刚灌进去的反馈位，
 *    暖机期内恒为 0；最高位才携带完整状态。
 */
static uint8_t exc_prbs_bit(uint32_t seed, uint32_t index)
{
    uint32_t state = ((seed == 0U) ? 1U : seed) * 2654435761U;

    state &= 0x7FFFFFFFU;
    if (state == 0U) {
        state = 1U;
    }
    for (uint32_t i = 0U; i < index; ++i) {
        const uint32_t feedback = ((state >> 30U) ^ (state >> 27U)) & 1U;
        state = ((state << 1U) | feedback) & 0x7FFFFFFFU;
    }
    return (uint8_t)((state >> 30U) & 1U);
}

DRV_SysIdExcStatus DRV_SysIdExcitation_Validate(const DRV_SysIdExcitation *spec)
{
    if (spec == NULL) {
        return DRV_SYSID_EXC_INVALID;
    }
    if (spec->profile >= (uint8_t)DRV_SYSID_PROFILE_COUNT) {
        return DRV_SYSID_EXC_INVALID;
    }
    if ((exc_finite(spec->amplitude_rad_s) == 0U) ||
        (spec->amplitude_rad_s <= 0.0f) ||
        (spec->amplitude_rad_s > DRV_SYSID_EXC_MAX_RATE_RAD_S)) {
        return DRV_SYSID_EXC_INVALID;
    }
    if ((spec->ramp_ms < DRV_SYSID_EXC_MIN_RAMP_MS) ||
        (spec->duration_ms == 0U) ||
        (spec->duration_ms > DRV_SYSID_EXC_MAX_DURATION_MS)) {
        return DRV_SYSID_EXC_INVALID;
    }

    switch ((DRV_SysIdProfile)spec->profile) {
    case DRV_SYSID_PROFILE_STEP:
        /* 斜坡上、保持、斜坡回，平台不能短于一个斜坡否则根本到不了幅值。 */
        if (spec->hold_ms < spec->ramp_ms) {
            return DRV_SYSID_EXC_INVALID;
        }
        break;
    case DRV_SYSID_PROFILE_DOUBLET:
        if ((spec->hold_ms < spec->ramp_ms) || (spec->repeat == 0U) ||
            (spec->repeat > DRV_SYSID_EXC_MAX_REPEAT)) {
            return DRV_SYSID_EXC_INVALID;
        }
        break;
    case DRV_SYSID_PROFILE_CHIRP:
        if ((exc_finite(spec->chirp_f0_hz) == 0U) ||
            (exc_finite(spec->chirp_f1_hz) == 0U) ||
            (spec->chirp_f0_hz <= 0.0f) || (spec->chirp_f1_hz <= 0.0f) ||
            (spec->chirp_f1_hz <= spec->chirp_f0_hz) ||
            (spec->chirp_f1_hz > 100.0f)) {
            return DRV_SYSID_EXC_INVALID;
        }
        break;
    case DRV_SYSID_PROFILE_PRBS:
        if ((spec->prbs_bit_ms < spec->ramp_ms) || (spec->prbs_bit_ms == 0U)) {
            return DRV_SYSID_EXC_INVALID;
        }
        break;
    default:
        return DRV_SYSID_EXC_INVALID;
    }
    return DRV_SYSID_EXC_OK;
}

DRV_SysIdExcStatus DRV_SysIdExcitation_TotalMs(const DRV_SysIdExcitation *spec,
                                               uint32_t *out_ms)
{
    if ((out_ms == NULL) ||
        (DRV_SysIdExcitation_Validate(spec) != DRV_SYSID_EXC_OK)) {
        return DRV_SYSID_EXC_INVALID;
    }
    switch ((DRV_SysIdProfile)spec->profile) {
    case DRV_SYSID_PROFILE_STEP:
        /* 上斜坡 + 平台 + 回零斜坡 */
        *out_ms = spec->ramp_ms + spec->hold_ms + spec->ramp_ms;
        break;
    case DRV_SYSID_PROFILE_DOUBLET: {
        const uint32_t one_pair = spec->hold_ms * 2U;
        const uint32_t total = one_pair * spec->repeat;
        *out_ms = (total < spec->duration_ms) ? total : spec->duration_ms;
        break;
    }
    case DRV_SYSID_PROFILE_CHIRP:
    case DRV_SYSID_PROFILE_PRBS:
    default:
        *out_ms = spec->duration_ms;
        break;
    }
    if (*out_ms > spec->duration_ms) {
        *out_ms = spec->duration_ms;
    }
    return DRV_SYSID_EXC_OK;
}

DRV_SysIdExcStatus DRV_SysIdExcitation_Eval(const DRV_SysIdExcitation *spec,
                                            uint32_t t_ms,
                                            DRV_SysIdExcSample *out)
{
    uint32_t total_ms = 0U;

    if ((out == NULL) ||
        (DRV_SysIdExcitation_TotalMs(spec, &total_ms) != DRV_SYSID_EXC_OK)) {
        return DRV_SYSID_EXC_INVALID;
    }
    out->omega_sp_rad_s = 0.0f;
    out->alpha_ff_rad_s2 = 0.0f;
    out->finished = 0U;

    if (t_ms >= total_ms) {
        out->finished = 1U;
        return DRV_SYSID_EXC_OK;
    }

    switch ((DRV_SysIdProfile)spec->profile) {
    case DRV_SYSID_PROFILE_STEP: {
        const uint32_t ramp = spec->ramp_ms;
        if (t_ms < ramp) {
            exc_ramped_level(0.0f, spec->amplitude_rad_s, t_ms, ramp,
                             &out->omega_sp_rad_s, &out->alpha_ff_rad_s2);
        } else if (t_ms < (ramp + spec->hold_ms)) {
            out->omega_sp_rad_s = spec->amplitude_rad_s;
            out->alpha_ff_rad_s2 = 0.0f;
        } else {
            exc_ramped_level(spec->amplitude_rad_s, 0.0f,
                             t_ms - (ramp + spec->hold_ms), ramp,
                             &out->omega_sp_rad_s, &out->alpha_ff_rad_s2);
        }
        break;
    }
    case DRV_SYSID_PROFILE_DOUBLET: {
        const uint32_t half = spec->hold_ms;
        const uint32_t segment = t_ms / half;
        const uint32_t since_edge = t_ms - (segment * half);
        /* 偶数段为正、奇数段为负；第 0 段之前视为 0。 */
        const float current = ((segment % 2U) == 0U) ? spec->amplitude_rad_s
                                                     : -spec->amplitude_rad_s;
        const float previous = (segment == 0U)
            ? 0.0f
            : (((segment - 1U) % 2U) == 0U ? spec->amplitude_rad_s
                                           : -spec->amplitude_rad_s);
        exc_ramped_level(previous, current, since_edge, spec->ramp_ms,
                         &out->omega_sp_rad_s, &out->alpha_ff_rad_s2);
        break;
    }
    case DRV_SYSID_PROFILE_CHIRP: {
        /*
         * 线性扫频：f(t) = f0 + (f1-f0)·t/T，相位是它的积分
         *     φ(t) = 2π (f0·t + (f1-f0)·t²/(2T))
         * α_ff 用解析导数 A·cos(φ)·φ̇ 而不是差分——差分会在高频段引入相位误差，
         * 而相位正是这一段要辨的东西。
         */
        const float seconds = (float)t_ms * 0.001f;
        const float span = (float)total_ms * 0.001f;
        const float sweep = (spec->chirp_f1_hz - spec->chirp_f0_hz) / span;
        const float phase = 6.2831853071795864769f *
            ((spec->chirp_f0_hz * seconds) + (0.5f * sweep * seconds * seconds));
        const float frequency = spec->chirp_f0_hz + (sweep * seconds);
        const float phase_rate = 6.2831853071795864769f * frequency;
        out->omega_sp_rad_s = spec->amplitude_rad_s * sinf(phase);
        out->alpha_ff_rad_s2 = spec->amplitude_rad_s * cosf(phase) * phase_rate;
        break;
    }
    case DRV_SYSID_PROFILE_PRBS: {
        const uint32_t bit_ms = spec->prbs_bit_ms;
        const uint32_t index = t_ms / bit_ms;
        const uint32_t since_edge = t_ms - (index * bit_ms);
        const float current = (exc_prbs_bit(spec->prbs_seed, index) != 0U)
            ? spec->amplitude_rad_s : -spec->amplitude_rad_s;
        const float previous = (index == 0U)
            ? 0.0f
            : ((exc_prbs_bit(spec->prbs_seed, index - 1U) != 0U)
                   ? spec->amplitude_rad_s : -spec->amplitude_rad_s);
        exc_ramped_level(previous, current, since_edge, spec->ramp_ms,
                         &out->omega_sp_rad_s, &out->alpha_ff_rad_s2);
        break;
    }
    default:
        return DRV_SYSID_EXC_INVALID;
    }
    return DRV_SYSID_EXC_OK;
}

/*
 * app_cmd_rpmnotch.c —— `RPMNOTCH` 命令族与辨识溯源行。策略与状态在 app_rpm_notch.c。
 *
 *   RPMNOTCH | RPMNOTCH ?                 4 行状态
 *   RPMNOTCH ON | OFF                     A/B 开关
 *   RPMNOTCH SET POLES <1..30> | HARM <1..7> | Q <1.5..10> | MINHZ <20..200> | FADEHZ <5..100>
 *                                         （MINHZ+FADEHZ <= 300）
 *   RPMNOTCH DEFAULTS                     可调量回编译期默认（开关不动）
 *   RPMNOTCH CLEAR                        计数与耗时最大值清零（任何时候都可以）
 *
 * ON/OFF/SET/DEFAULTS 只在未解锁、且辨识没占着台架时收（照 THRUSTLUT MODE 的先例）：
 * 飞行中切换控制用陀螺等于在回路里换一个滤波器；辨识一轮跑到一半换，那一轮的溯源
 * （SYSID NOTCH 行）就不再代表整轮。阶段一只在 RAM，不写 Flash。
 *
 * 大小写敏感，同 THRUSTLUT。printf 只用整数（newlib-nano 不带 %f）。
 * 挂在 app_cmd_fallback.c 的兜底链上，不碰只减不增的 app_control.c。
 */

#include "app_rpm_notch.h"

#include "app_control.h"
#include "app_control_internal.h"
#include "app_stabilizer.h"
#include "app_sysid.h"

#include <stddef.h>
#include <string.h>

#define RPMNOTCH_USAGE \
    "RPMNOTCH ?|ON|OFF|DEFAULTS|CLEAR|SET POLES|HARM|Q|MINHZ|FADEHZ <v>"

static unsigned long rpmnotch_scaled(float value, float scale)
{
    const float scaled = value * scale;

    return (scaled > 0.0f) ? (unsigned long)(scaled + 0.5f) : 0UL;
}

static unsigned long rpmnotch_avg_x100(uint32_t sum, uint32_t count)
{
    return (count == 0U) ? 0UL : (unsigned long)(((uint64_t)sum * 100ULL) / (uint64_t)count);
}

static const char *rpmnotch_src(const APP_RpmNotchStatus *s)
{
    return (s->motors.available != 0U) ? "bidir" : "unavailable";
}

static void rpmnotch_report_status(void)
{
    APP_RpmNotchStatus s;

    APP_RpmNotch_GetStatus(&s);
    APP_Control_QueueText(
        "RPMNOTCH cfg en=%u state=%s src=%s pp=%u harm=%u q_x100=%u min_hz=%u fade_hz=%u "
        "fs_nom=%u fs_x10=%lu active=%u\r\n",
        (unsigned int)s.cfg.enable, APP_RpmNotch_StateName(s.state), rpmnotch_src(&s),
        (unsigned int)s.cfg.pole_pairs, (unsigned int)s.cfg.harmonic_mask,
        (unsigned int)rpmnotch_scaled(s.cfg.q, 100.0f), (unsigned int)rpmnotch_scaled(s.cfg.min_hz, 1.0f),
        (unsigned int)rpmnotch_scaled(s.cfg.fade_hz, 1.0f), (unsigned int)s.fs_nominal_hz,
        rpmnotch_scaled(s.fs_hz, 10.0f), (unsigned int)s.active_slots);
    APP_Control_QueueText(
        "RPMNOTCH esc ch1_erpm=%lu ch1_hz_x10=%lu ch1_age_ms=%lu ch1_w_x100=%u "
        "ch2_erpm=%lu ch2_hz_x10=%lu ch2_age_ms=%lu ch2_w_x100=%u\r\n",
        (unsigned long)s.motors.erpm[0], rpmnotch_scaled(s.motors.hz[0], 10.0f),
        (unsigned long)s.motors.age_ms[0], (unsigned int)rpmnotch_scaled(s.weight_base[0], 100.0f),
        (unsigned long)s.motors.erpm[1], rpmnotch_scaled(s.motors.hz[1], 10.0f),
        (unsigned long)s.motors.age_ms[1], (unsigned int)rpmnotch_scaled(s.weight_base[1], 100.0f));
    APP_Control_QueueText(
        "RPMNOTCH count stale=%lu gap1=%lu reset=%lu nonfinite=%lu slewclamp=%lu reject=%lu wdog=%lu\r\n",
        (unsigned long)s.stale, (unsigned long)s.gap1, (unsigned long)s.reset,
        (unsigned long)s.nonfinite, (unsigned long)s.slewclamp, (unsigned long)s.reject,
        (unsigned long)s.wdog);
    APP_Control_QueueText(
        "RPMNOTCH time samples=%lu spin=%lu tracked=%lu apply_us_avg_x100=%lu apply_us_max=%lu "
        "tick_us_avg_x100=%lu tick_us_max=%lu\r\n",
        (unsigned long)s.samples, (unsigned long)s.spin, (unsigned long)s.tracked,
        rpmnotch_avg_x100(s.apply_us_sum, s.apply_count), (unsigned long)s.apply_us_max,
        rpmnotch_avg_x100(s.tick_us_sum, s.tick_count), (unsigned long)s.tick_us_max);
}

static uint8_t rpmnotch_reject(const char *reason)
{
    APP_Control_QueueText("RPMNOTCH event=rejected reason=%s\r\n", reason);
    return 1U;
}

static uint8_t rpmnotch_usage(void)
{
    APP_Control_QueueText("RPMNOTCH event=rejected reason=usage usage=" RPMNOTCH_USAGE "\r\n");
    return 1U;
}

static uint8_t rpmnotch_range(const char *key)
{
    APP_Control_QueueText("RPMNOTCH event=rejected reason=range key=%s\r\n", key);
    return 1U;
}

/* SET <key> <value>：解析失败回 usage，范围不对回 range。返回 1 = 已回复（拒绝），0 = cfg 已改好。 */
static uint8_t rpmnotch_apply_set(const char *key, const char *text, APP_RpmNotchConfig *cfg)
{
    uint32_t u = 0U;
    float f = 0.0f;

    if ((strcmp(key, "POLES") == 0) || (strcmp(key, "HARM") == 0)) {
        const uint8_t poles = (strcmp(key, "POLES") == 0) ? 1U : 0U;
        const uint32_t hi = (poles != 0U) ? APP_RPM_NOTCH_POLE_PAIRS_MAX : 0x07U;

        if (app_control_parse_u32(text, &u) == 0U) {
            return rpmnotch_usage();
        }
        if ((u < 1U) || (u > hi)) {
            return rpmnotch_range(key);
        }
        if (poles != 0U) {
            cfg->pole_pairs = (uint8_t)u;
        } else {
            cfg->harmonic_mask = (uint8_t)u;
        }
        return 0U;
    }
    if ((strcmp(key, "Q") != 0) && (strcmp(key, "MINHZ") != 0) && (strcmp(key, "FADEHZ") != 0)) {
        return rpmnotch_usage();
    }
    if (app_control_parse_f32(text, &f) == 0U) {
        return rpmnotch_usage();
    }
    if (strcmp(key, "Q") == 0) {
        if ((f < APP_RPM_NOTCH_Q_LO) || (f > APP_RPM_NOTCH_Q_HI)) {
            return rpmnotch_range(key);
        }
        cfg->q = f;
    } else if (strcmp(key, "MINHZ") == 0) {
        if ((f < APP_RPM_NOTCH_MIN_HZ_LO) || (f > APP_RPM_NOTCH_MIN_HZ_HI)) {
            return rpmnotch_range(key);
        }
        cfg->min_hz = f;
    } else {
        if ((f < APP_RPM_NOTCH_FADE_HZ_LO) || (f > APP_RPM_NOTCH_FADE_HZ_HI)) {
            return rpmnotch_range(key);
        }
        cfg->fade_hz = f;
    }
    return 0U;
}

uint8_t APP_RpmNotch_Command(char **tokens, uint32_t count)
{
    APP_RpmNotchConfig cfg;
    const char *sub;
    uint8_t change;

    if ((tokens == NULL) || (count == 0U) || (strcmp(tokens[0], "RPMNOTCH") != 0)) {
        return 0U;
    }
    if ((count == 1U) || ((count == 2U) && (strcmp(tokens[1], "?") == 0))) {
        rpmnotch_report_status();
        return 1U;
    }
    sub = tokens[1];
    if ((count == 2U) && (strcmp(sub, "CLEAR") == 0)) {
        APP_RpmNotch_ClearStats();   /* 控制拍下一拍清；计数只有那边写 */
        APP_Control_QueueText("RPMNOTCH event=cleared\r\n");
        return 1U;
    }
    change = (((count == 2U) && ((strcmp(sub, "ON") == 0) || (strcmp(sub, "OFF") == 0) ||
                                 (strcmp(sub, "DEFAULTS") == 0))) ||
              ((count == 4U) && (strcmp(sub, "SET") == 0))) ? 1U : 0U;
    if (change == 0U) {
        return rpmnotch_usage();
    }
    if (APP_Stabilizer_IsArmed() != 0U) {
        return rpmnotch_reject("armed");
    }
    if (APP_SysId_IsEngaged() != 0U) {
        return rpmnotch_reject("sysid");
    }
    APP_RpmNotch_GetConfig(&cfg);
    if (strcmp(sub, "ON") == 0) {
        cfg.enable = 1U;
    } else if (strcmp(sub, "OFF") == 0) {
        cfg.enable = 0U;
    } else if (strcmp(sub, "DEFAULTS") == 0) {
        const uint8_t enable = cfg.enable;

        APP_RpmNotch_DefaultConfig(&cfg);
        cfg.enable = enable;
    } else if (rpmnotch_apply_set(tokens[2], tokens[3], &cfg) != 0U) {
        return 1U;
    }
    if (APP_RpmNotch_RequestConfig(&cfg) == 0U) {
        return rpmnotch_range("MINHZ+FADEHZ");   /* 单项都在范围内，只剩组合上限 */
    }
    rpmnotch_report_status();
    return 1U;
}

void APP_RpmNotch_ReportProvenance(uint16_t run_id)
{
    APP_RpmNotchStatus s;

    APP_RpmNotch_GetStatus(&s);
    APP_Control_QueueText(
        "SYSID NOTCH run=%u en=%u state=%s src=%s pp=%u harm=%u q_x100=%u min_hz=%u fade_hz=%u "
        "fs_x10=%lu\r\n",
        (unsigned int)run_id, (unsigned int)s.cfg.enable, APP_RpmNotch_StateName(s.state),
        rpmnotch_src(&s), (unsigned int)s.cfg.pole_pairs, (unsigned int)s.cfg.harmonic_mask,
        (unsigned int)rpmnotch_scaled(s.cfg.q, 100.0f), (unsigned int)rpmnotch_scaled(s.cfg.min_hz, 1.0f),
        (unsigned int)rpmnotch_scaled(s.cfg.fade_hz, 1.0f), rpmnotch_scaled(s.fs_hz, 10.0f));
}

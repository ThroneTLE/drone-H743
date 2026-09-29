/*
 * 推力查补表接入飞控（作者 2026-09-23 要求）。
 *
 * 1) 启动时把推力台查补表装进双桨控制器的“推力 -> 脉宽”换算（DRV_COAX_CTRL_SetThrustMap）。
 *    控制器给出单桨推力 T；与旧曲线同一口径，按“两桨同油门时合推力 = 2T”查同油门线，
 *    得到等效油门 e，再按电量电压补偿成油门% = e × 12 / 电量电压，最后按推力台同一
 *    线性关系换成脉宽（1100 + % × 8.4 us）。
 * 2) messageTask 每 20 ms 估一次电量电压：带载电压 + k·Σ(eRPM/1e4)³，低通 2 s。
 *    电压或任一路 eRPM 不新鲜时保持上一次估计（不能把压降误当成掉电）；从未有过
 *    可信估计时为 0，换算按 12 V 计，即不补偿。
 * 3) 差速（偏航）时上下桨一起换算（作者 2026-09-24 要求）：先逐桨查同油门线（旧口径），
 *    再保持两桨油门差、整体平移，使二维油门表的合推力正好等于指令，偏航不再带出高度变化。
 *    无差速时平移为 0，与逐桨换算完全相同。推力台“差速实测验证”用同一算法做新旧 A/B。
 * 4) THRUSTLUT 命令：只读查询（与上位机对拍）和 A/B 切换 PAIR/LUT/LEGACY（仅未解锁时）。
 */

#include "app_thrust_lut.h"

#include "app_battery.h"
#include "app_control.h"
#include "app_control_internal.h"
#include "app_rpm_notch.h"
#include "app_stabilizer.h"
#include "bsp_dshot_rx.h"
#include "bsp_pwm.h"
#include "drv_coax_ctrl.h"
#include "drv_thrust_lut.h"
#include "svc_timestamp.h"

#include <stddef.h>
#include <string.h>

#define THRUST_LUT_GRAMS_PER_NEWTON 101.971621f
#define THRUST_LUT_VOLTAGE_FRESH_MS 500U
#define THRUST_LUT_ERPM_FRESH_MS    100U
#define THRUST_LUT_FILTER_TAU_S     2.0f
/*
 * 双向 DShot 回包偶有错帧过了 4 位校验、解出离谱的 eRPM（2026-09-28 杆上一轮约 2 s 旋转里
 * 陷波拒收 20 帧 >150000）。电量电压的压降项是 eRPM 的三次方，一帧 50 万就把滤波后的
 * 电量电压顶高几伏、越出可用范围：光杆辨识报 thrust_stale，飞行时查表按上限电压给油门、
 * 推力偏小。与陷波同一个物理上限（KV1300×12.6 V×7 对极约 11.5 万）：本拍跳过，不更新
 * 也不判过期；错帧连续 500 ms 才因不新鲜而过期。
 */
#define THRUST_LUT_ERPM_MAX         APP_RPM_NOTCH_ERPM_MAX

static volatile float thrust_lut_charge_v;
static volatile float thrust_lut_last_shift;
static uint32_t thrust_lut_last_ms;
static volatile uint32_t thrust_lut_erpm_skips;   /* 因 eRPM 超上限跳过的拍数 */
/* volatile：控制拍（高优先级）随时读，本任务写；只在真正过期时从 1 变 0。 */
static volatile uint8_t thrust_lut_fresh;

static uint16_t thrust_lut_percent_to_pulse(float percent)
{
    const float span = (float)(BSP_PWM_ESC_MAX_US - BSP_PWM_ESC_MIN_US);
    return (uint16_t)((float)BSP_PWM_ESC_MIN_US + percent * span / 100.0f + 0.5f);
}

uint16_t APP_ThrustLut_PulseForMotorThrust(float thrust_n, float charge_v)
{
    const float total_g = 2.0f * thrust_n * THRUST_LUT_GRAMS_PER_NEWTON;
    float effective;

    if (!(total_g > 0.0f)) {
        return BSP_PWM_ESC_MIN_US;
    }
    effective = DRV_ThrustLut_BalancedEffectiveForThrust(total_g);
    return thrust_lut_percent_to_pulse(DRV_ThrustLut_PercentForEffective(effective, charge_v));
}

float APP_ThrustLut_TotalThrustForPulse(uint16_t pulse_us, float charge_v)
{
    const float span = (float)(BSP_PWM_ESC_MAX_US - BSP_PWM_ESC_MIN_US);
    float percent;

    if (pulse_us <= BSP_PWM_ESC_MIN_US) {
        return 0.0f;
    }
    percent = (float)(pulse_us - BSP_PWM_ESC_MIN_US) * 100.0f / span;
    return DRV_ThrustLut_BalancedThrustForEffective(DRV_ThrustLut_EffectiveForPercent(percent, charge_v)) /
           THRUST_LUT_GRAMS_PER_NEWTON;
}

float APP_ThrustLut_PulsesForPair(float upper_n, float lower_n, float charge_v,
                                  uint16_t *upper_us, uint16_t *lower_us)
{
    const float e_upper = DRV_ThrustLut_BalancedEffectiveForThrust(2.0f * upper_n * THRUST_LUT_GRAMS_PER_NEWTON);
    const float e_lower = DRV_ThrustLut_BalancedEffectiveForThrust(2.0f * lower_n * THRUST_LUT_GRAMS_PER_NEWTON);
    const float shift = DRV_ThrustLut_PairShift(e_upper, e_lower, (upper_n + lower_n) * THRUST_LUT_GRAMS_PER_NEWTON,
                                                DRV_ThrustLut_EffectiveForPercent(100.0f, charge_v));

    /* 平移只在两桨都在实测范围内时非 0，所以零推力的桨仍是怠速脉宽。 */
    *upper_us = (upper_n > 0.0f) ?
        thrust_lut_percent_to_pulse(DRV_ThrustLut_PercentForEffective(e_upper + shift, charge_v)) : BSP_PWM_ESC_MIN_US;
    *lower_us = (lower_n > 0.0f) ?
        thrust_lut_percent_to_pulse(DRV_ThrustLut_PercentForEffective(e_lower + shift, charge_v)) : BSP_PWM_ESC_MIN_US;
    return shift;
}

static uint16_t thrust_lut_map_pulse(float thrust_n)
{
    return APP_ThrustLut_PulseForMotorThrust(thrust_n, thrust_lut_charge_v);
}

static float thrust_lut_map_thrust(uint16_t pulse_us)
{
    return APP_ThrustLut_TotalThrustForPulse(pulse_us, thrust_lut_charge_v);
}

static void thrust_lut_map_pair(float upper_n, float lower_n, uint16_t *upper_us, uint16_t *lower_us)
{
    thrust_lut_last_shift = APP_ThrustLut_PulsesForPair(upper_n, lower_n, thrust_lut_charge_v, upper_us, lower_us);
}

static const DRV_COAX_CTRL_ThrustMap thrust_lut_map = {
    thrust_lut_map_pulse,
    thrust_lut_map_thrust,
    NULL,
};

static const DRV_COAX_CTRL_ThrustMap thrust_lut_pair_map = {
    thrust_lut_map_pulse,
    thrust_lut_map_thrust,
    thrust_lut_map_pair,
};

void APP_ThrustLut_Init(void)
{
    thrust_lut_charge_v = 0.0f;
    thrust_lut_last_shift = 0.0f;
    thrust_lut_fresh = 0U;
    DRV_COAX_CTRL_SetThrustMap(&thrust_lut_pair_map);
}

static const char *thrust_lut_mode_name(void)
{
    const DRV_COAX_CTRL_ThrustMap *map = DRV_COAX_CTRL_GetThrustMap();

    if (map == &thrust_lut_pair_map) { return "pair"; }
    if (map == &thrust_lut_map) { return "lut"; }
    return "legacy";
}

float APP_ThrustLut_ChargeVoltage(void)
{
    return thrust_lut_charge_v;
}

uint8_t APP_ThrustLut_IsFresh(void)
{
    return thrust_lut_fresh &&
           (uint32_t)(SVC_Timestamp_Ms() - thrust_lut_last_ms) <= THRUST_LUT_VOLTAGE_FRESH_MS &&
           thrust_lut_charge_v >= DRV_ThrustLut_Table.charge_usable_v[0] &&
           thrust_lut_charge_v <= DRV_ThrustLut_Table.charge_usable_v[1];
}

void APP_ThrustLut_Step(void)
{
    APP_BatterySnapshot battery;
    BSP_DShotRxSnapshot rx;
    float erpm[2];
    float raw;
    float charge;
    uint32_t now;

    APP_Battery_GetSnapshot(&battery);
    BSP_DShotRx_GetSnapshot(&rx);
    now = SVC_Timestamp_Ms();
    /*
     * 不在开头先清 0 再在末尾置 1：控制拍抢占在这段中间会读到一个瞬时的"不新鲜"，
     * 光杆辨识就以 thrust_stale 中止（2026-09-27 审查确认）。只在真过期的分支清。
     */
    if ((battery.state.valid == 0U) || (battery.age_ms > THRUST_LUT_VOLTAGE_FRESH_MS) ||
        (rx.available == 0U)) {
        thrust_lut_fresh = 0U;
        return;
    }
    for (uint32_t i = 0U; i < 2U; ++i) {
        if ((rx.valid[i] == 0U) || ((uint32_t)(now - rx.sample_ms[i]) > THRUST_LUT_ERPM_FRESH_MS)) {
            thrust_lut_fresh = 0U;
            return;
        }
        if ((rx.not_spinning[i] == 0U) && (rx.erpm[i] > THRUST_LUT_ERPM_MAX)) {
            thrust_lut_erpm_skips++;
            return;
        }
        erpm[i] = (rx.not_spinning[i] != 0U) ? 0.0f : (float)rx.erpm[i];
    }
    raw = DRV_ThrustLut_ChargeVoltage((float)battery.state.voltage_mv / 1000.0f, erpm[0], erpm[1]);
    charge = thrust_lut_charge_v;
    if (!(charge > 0.0f)) {
        charge = raw;
    } else {
        float dt_s = (float)(uint32_t)(now - thrust_lut_last_ms) / 1000.0f;
        if (dt_s > 1.0f) { dt_s = 1.0f; }
        charge += (raw - charge) * dt_s / (THRUST_LUT_FILTER_TAU_S + dt_s);
    }
    thrust_lut_last_ms = now;
    thrust_lut_charge_v = charge;
    thrust_lut_fresh = 1U;
}

static long thrust_lut_scaled(float value, float scale)
{
    const float scaled = value * scale;
    return (long)(scaled + ((scaled >= 0.0f) ? 0.5f : -0.5f));
}

static void thrust_lut_report_status(void)
{
    const DRV_ThrustLutTable *t = &DRV_ThrustLut_Table;
    const float charge = thrust_lut_charge_v;

    APP_Control_QueueText(
        "THRUSTLUT event=status mode=%s model=%s charge_mv=%ld state=%s usable_mv=%ld-%ld measured_mv=%ld-%ld diag=%u "
        "last_shift_x100=%ld erpm_skip=%lu\r\n",
        thrust_lut_mode_name(),
        t->model_id,
        thrust_lut_scaled(charge, 1000.0f),
        !(charge > 0.0f) ? "unknown" : ((thrust_lut_fresh != 0U) ? "tracking" : "holding"),
        thrust_lut_scaled(t->charge_usable_v[0], 1000.0f),
        thrust_lut_scaled(t->charge_usable_v[1], 1000.0f),
        thrust_lut_scaled(t->charge_measured_v[0], 1000.0f),
        thrust_lut_scaled(t->charge_measured_v[1], 1000.0f),
        (unsigned int)t->diag_count,
        thrust_lut_scaled(thrust_lut_last_shift, 100.0f),
        (unsigned long)thrust_lut_erpm_skips);
}

uint8_t app_control_handle_thrust_lut(char **tokens, uint32_t count)
{
    float a = 0.0f;
    float b = 0.0f;
    float c = 0.0f;

    if ((tokens == NULL) || (count < 2U) || (strcmp(tokens[0], "THRUSTLUT") != 0)) {
        return 0U;
    }
    if ((count == 2U) && (strcmp(tokens[1], "?") == 0)) {
        thrust_lut_report_status();
        return 1U;
    }
    /* THRUSTLUT MAP <双桨合推力 g> [电量电压 V]：与飞行同一条换算，不驱动电机。 */
    if ((strcmp(tokens[1], "MAP") == 0) && ((count == 3U) || (count == 4U)) &&
        app_control_parse_f32(tokens[2], &a) && ((count == 3U) || app_control_parse_f32(tokens[3], &b))) {
        const float charge = (count == 4U) ? b : thrust_lut_charge_v;
        const float effective = DRV_ThrustLut_BalancedEffectiveForThrust(a);
        const float percent = DRV_ThrustLut_PercentForEffective(effective, charge);
        const uint16_t pulse = APP_ThrustLut_PulseForMotorThrust(a / (2.0f * THRUST_LUT_GRAMS_PER_NEWTON), charge);

        APP_Control_QueueText(
            "THRUSTLUT event=map total_dg=%ld charge_mv=%ld effective_x100=%ld percent_x100=%ld pulse=%u back_dg=%ld\r\n",
            thrust_lut_scaled(a, 10.0f), thrust_lut_scaled(charge, 1000.0f),
            thrust_lut_scaled(effective, 100.0f), thrust_lut_scaled(percent, 100.0f), (unsigned int)pulse,
            thrust_lut_scaled(APP_ThrustLut_TotalThrustForPulse(pulse, charge) * THRUST_LUT_GRAMS_PER_NEWTON, 10.0f));
        return 1U;
    }
    /* THRUSTLUT PAIR <上桨推力 g> <下桨推力 g> [电量电压 V]：控制器给的逐桨推力（同油门一半口径）→ 两路脉宽。 */
    if ((strcmp(tokens[1], "PAIR") == 0) && ((count == 4U) || (count == 5U)) &&
        app_control_parse_f32(tokens[2], &a) && app_control_parse_f32(tokens[3], &b) &&
        ((count == 4U) || app_control_parse_f32(tokens[4], &c))) {
        const float charge = (count == 5U) ? c : thrust_lut_charge_v;
        uint16_t upper_us = 0U;
        uint16_t lower_us = 0U;
        const float shift = APP_ThrustLut_PulsesForPair(a / THRUST_LUT_GRAMS_PER_NEWTON, b / THRUST_LUT_GRAMS_PER_NEWTON,
                                                        charge, &upper_us, &lower_us);

        APP_Control_QueueText(
            "THRUSTLUT event=pair upper_dg=%ld lower_dg=%ld charge_mv=%ld shift_x100=%ld upper_us=%u lower_us=%u\r\n",
            thrust_lut_scaled(a, 10.0f), thrust_lut_scaled(b, 10.0f), thrust_lut_scaled(charge, 1000.0f),
            thrust_lut_scaled(shift, 100.0f), (unsigned int)upper_us, (unsigned int)lower_us);
        return 1U;
    }
    /* THRUSTLUT SPEED <上桨 eRPM> <下桨 eRPM>：转速表查推力。 */
    if ((strcmp(tokens[1], "SPEED") == 0) && (count == 4U) &&
        app_control_parse_f32(tokens[2], &a) && app_control_parse_f32(tokens[3], &b)) {
        APP_Control_QueueText("THRUSTLUT event=speed upper=%ld lower=%ld thrust_dg=%ld\r\n",
                              thrust_lut_scaled(a, 1.0f), thrust_lut_scaled(b, 1.0f),
                              thrust_lut_scaled(DRV_ThrustLut_ThrustFromSpeed(a, b), 10.0f));
        return 1U;
    }
    /* THRUSTLUT MODE PAIR|LUT|LEGACY：A/B 对照，仅未解锁时；重启回到 PAIR。 */
    if ((strcmp(tokens[1], "MODE") == 0) && (count == 3U) &&
        ((strcmp(tokens[2], "PAIR") == 0) || (strcmp(tokens[2], "LUT") == 0) ||
         (strcmp(tokens[2], "LEGACY") == 0))) {
        if (APP_Stabilizer_IsArmed() != 0U) {
            APP_Control_QueueText("THRUSTLUT event=rejected reason=armed\r\n");
            return 1U;
        }
        DRV_COAX_CTRL_SetThrustMap((strcmp(tokens[2], "PAIR") == 0) ? &thrust_lut_pair_map :
                                   (strcmp(tokens[2], "LUT") == 0) ? &thrust_lut_map : NULL);
        thrust_lut_report_status();
        return 1U;
    }
    APP_Control_QueueText(
        "THRUSTLUT event=rejected reason=usage usage=THRUSTLUT ?|MAP <g> [V]|PAIR <g> <g> [V]|SPEED <erpm> <erpm>|MODE PAIR|LUT|LEGACY\r\n");
    return 1U;
}

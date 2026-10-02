/*
 * app_sysid_yaw.c —— 吊绳偏航辨识（SYSID MODE YAW）。
 *
 * 设计与口径写在 App/Inc/app_sysid_yaw.h，两侧契约 doc/sysid-yaw-contract.md。安全门、采样网格、
 * 油门出口都是 app_sysid.c 已有的那一套，本文件只算"这一拍在哪个阶段、上下桨各给多少"。
 * 结构照 app_sysid_xy.c：阶段推进、软停（推力 1 s 线性降到遥控油门再报理由）、配置面。
 * 与 XY 的区别：没有姿态链（舵机全程中位）、没有光流；出力是上下桨**不同**的脉宽。
 *
 * 状态放 AXI SRAM（NOLOAD，首次调用时显式清），不占 DTCM。配置只在命令任务里改（且只在空闲时），
 * 本轮用的那份在开跑时锁存。
 */

#include "app_sysid_yaw.h"

#include "app_control.h"

#include "drv_airframe_params.h"
#include "drv_coax_ctrl.h"
#include "drv_prop_map.h"
#include "drv_rate_control.h"
#include "bsp_pwm.h"   /* 只取电调脉宽量程常量做封顶；写电调仍只在稳定环 */

#include <math.h>
#include <stdio.h>
#include <string.h>

#define YAW_PI 3.14159265f
/* 抗饱和上限的下限，同生产（DRV_COAX_CTRL_RATE_SCALE_EPS）：上限为 0 时不让环拿到非法区间。 */
#define YAW_MOMENT_LIMIT_EPS 1.0e-6f
/* 横滚/俯仰轴在本模式不用：给个有序的非零上下限即可（输入恒 0，输出恒 0）。 */
#define YAW_UNUSED_AXIS_LIMIT_N_M 3.0f

typedef struct {
    APP_SysIdYawConfig config;          /* 命令面改的那份；thrust_mn = 0 表示用默认值 */

    /* 本轮（APP_SysIdYaw_Begin 锁存） */
    APP_SysIdYawConfig run;             /* thrust_mn 已解析成真值 */
    DRV_SysIdExcitation spec;
    float max_pct;
    float thrust_n;                     /* 总推力 F */
    float k_m_per_n;                    /* (k_upper + k_lower)/2 */
    float izz_kgm2;
    float single_max_n;
    float twist_rad;
    float polarity;                     /* 偏航极性 P，开跑时锁存（记录 az 用） */

    /* 本拍量 */
    float psi_rad;                      /* 陀螺 z 积分偏航（开跑清零，跨轮保留给诊断） */
    float rate_sp_rad_s;                /* r（记录 vz_sp） */
    float moment_nm;                    /* M，饱和前（记录 torque） */
    float diff_n;                       /* ΔT = P·(T_lower − T_upper)，分配后（记录 az） */
    float record_thrust_n;              /* 记录的总推力 */
    DRV_RateControl_State rate_state;
    uint8_t alloc_sat_pos;              /* 上一拍分配器撞限且 M > 0 / M < 0：喂角速度环冻结积分 */
    uint8_t alloc_sat_neg;
    uint16_t idle_pulse_us;
    uint16_t ramp_from_upper_us;
    uint16_t ramp_from_lower_us;
    uint16_t upper_us;
    uint16_t lower_us;

    char refusal[80];                   /* 带数值的拒绝理由（Precheck 返回它；放 AXI SRAM 不占 DTCM） */

    /* 软停：soft_reason 非空 = 正在软停，走完报它 */
    const char *soft_reason;
    uint16_t soft_from_upper_us;
    uint16_t soft_from_lower_us;
    uint32_t soft_start_ms;
} SysIdYawContext;

#if defined(__GNUC__) && defined(__arm__)
__attribute__((section(".ram_d1_noinit"), aligned(32)))
#endif
static SysIdYawContext yaw;
static uint8_t yaw_initialised;

static const char *const yaw_inject_names[APP_SYSID_YAW_INJECT_COUNT] = { "diff", "rate" };

static void yaw_init(void)
{
    if (yaw_initialised != 0U) {
        return;
    }
    yaw_initialised = 1U;
    memset(&yaw, 0, sizeof(yaw));
    yaw.config.inject = (uint8_t)APP_SYSID_YAW_INJECT_DIFF;
    yaw.config.thrust_mn = 0U;   /* 0 = 默认 0.5×悬停推力，读取时才解析（机体参数上电后才有） */
    yaw.config.twist_deg = APP_SYSID_YAW_TWIST_DEG_DEFAULT;
}

static float yaw_finite_or(float value, float fallback)
{
    return (isfinite(value) != 0) ? value : fallback;
}

/* 悬停推力：coax.hover_thrust_n 开着取它（EffectiveMass×g），否则机重。机体无效返回 0。 */
static float yaw_hover_thrust_n(void)
{
    const DRV_Airframe_Params *airframe = DRV_Airframe_Get();

    if ((airframe == NULL) || (isfinite(airframe->mass_kg) == 0) || !(airframe->mass_kg > 0.0f) ||
        (isfinite(airframe->gravity_m_s2) == 0) || !(airframe->gravity_m_s2 > 0.0f)) {
        return 0.0f;
    }
    return DRV_COAX_CTRL_EffectiveMassKg(airframe->mass_kg) * airframe->gravity_m_s2;
}

/* 机重 [N]（lift 判据用；不看悬停学习值，机体不会因为学习值变轻而更容易被提起）。 */
static float yaw_weight_n(void)
{
    const DRV_Airframe_Params *airframe = DRV_Airframe_Get();

    if ((airframe == NULL) || (isfinite(airframe->mass_kg) == 0) || !(airframe->mass_kg > 0.0f) ||
        (isfinite(airframe->gravity_m_s2) == 0) || !(airframe->gravity_m_s2 > 0.0f)) {
        return 0.0f;
    }
    return airframe->mass_kg * airframe->gravity_m_s2;
}

static uint16_t yaw_default_thrust_mn(void)
{
    const float hover_n = yaw_hover_thrust_n();
    float mn = APP_SYSID_YAW_DEFAULT_HOVER_FRACTION * hover_n * 1000.0f;

    if (!(mn >= (float)APP_SYSID_YAW_THRUST_MN_MIN)) {
        mn = (float)APP_SYSID_YAW_THRUST_MN_MIN;
    }
    if (mn > (float)APP_SYSID_YAW_THRUST_MN_MAX) {
        mn = (float)APP_SYSID_YAW_THRUST_MN_MAX;
    }
    return (uint16_t)lroundf(mn);
}

static uint16_t yaw_resolved_thrust_mn(const APP_SysIdYawConfig *config)
{
    return (config->thrust_mn != 0U) ? config->thrust_mn : yaw_default_thrust_mn();
}

/* ------------------------------------------------------------------ 配置面 */

void APP_SysIdYaw_GetConfig(APP_SysIdYawConfig *out)
{
    yaw_init();
    if (out != NULL) {
        *out = yaw.config;
        out->thrust_mn = yaw_resolved_thrust_mn(&yaw.config);
    }
}

const char *APP_SysIdYaw_SetConfig(const APP_SysIdYawConfig *config)
{
    const float weight_n = yaw_weight_n();
    const DRV_Airframe_Params *airframe = DRV_Airframe_Get();

    yaw_init();
    if ((config == NULL) || (config->inject >= (uint8_t)APP_SYSID_YAW_INJECT_COUNT) ||
        (config->thrust_mn < APP_SYSID_YAW_THRUST_MN_MIN) ||
        (config->twist_deg < APP_SYSID_YAW_TWIST_DEG_MIN) ||
        (config->twist_deg > APP_SYSID_YAW_TWIST_DEG_MAX) || !(weight_n > 0.0f)) {
        return "range";
    }
    /* thrust > 0.8×机重 会把机体提起来、绳不再绷紧：先于机体最大推力判，给出更有用的理由。 */
    if (((float)config->thrust_mn * 0.001f) > (APP_SYSID_YAW_LIFT_FRACTION * weight_n)) {
        return "lift";
    }
    if (((float)config->thrust_mn * 0.001f) > airframe->max_total_force_n) {
        return "range";
    }
    yaw.config = *config;
    return NULL;
}

const char *APP_SysIdYaw_InjectName(uint8_t inject)
{
    return (inject < (uint8_t)APP_SYSID_YAW_INJECT_COUNT) ? yaw_inject_names[inject] : "?";
}

uint8_t APP_SysIdYaw_InjectFromName(const char *name, uint8_t *out)
{
    if ((name == NULL) || (out == NULL)) {
        return 0U;
    }
    for (uint8_t i = 0U; i < (uint8_t)APP_SYSID_YAW_INJECT_COUNT; ++i) {
        if (strcmp(name, yaw_inject_names[i]) == 0) {
            *out = i;
            return 1U;
        }
    }
    return 0U;
}

static const char *yaw_control_name(uint8_t inject)
{
    return (inject == (uint8_t)APP_SYSID_YAW_INJECT_DIFF) ? "openloop" : "closed_loop";
}

void APP_SysIdYaw_ReportConfig(void)
{
    yaw_init();
    APP_Control_QueueText("SYSID YAW inject=%s thrust_mn=%u twist_deg=%u control=%s\r\n",
                          APP_SysIdYaw_InjectName(yaw.config.inject),
                          (unsigned int)yaw_resolved_thrust_mn(&yaw.config),
                          (unsigned int)yaw.config.twist_deg,
                          yaw_control_name(yaw.config.inject));
}

float APP_SysIdYaw_ThrustN(void)
{
    yaw_init();
    return (float)yaw_resolved_thrust_mn(&yaw.config) * 0.001f;
}

static float yaw_amplitude_max(uint8_t inject, float thrust_n, float single_max_n)
{
    float limit;

    if (inject != (uint8_t)APP_SYSID_YAW_INJECT_DIFF) {
        return APP_SYSID_YAW_RATE_AMP_MAX_RAD_S;
    }
    /* 两桨各 F/2 ± ΔT/2 都要落在 [0, T单max]：|ΔT| ≤ F 且 ≤ 2·(T单max − F/2)，再留 20% 余量（0.8·F）。 */
    limit = fminf(APP_SYSID_YAW_DIFF_FRACTION * thrust_n, 2.0f * (single_max_n - (0.5f * thrust_n)));
    return (limit > 0.0f) ? limit : 0.0f;
}

float APP_SysIdYaw_AmplitudeMax(void)
{
    yaw_init();
    return yaw_amplitude_max(yaw.config.inject, APP_SysIdYaw_ThrustN(),
                             DRV_COAX_CTRL_SingleMaxThrustN());
}

/* ------------------------------------------------------------------ 开跑检查与锁存 */

const char *APP_SysIdYaw_Precheck(const DRV_SysIdExcitation *spec)
{
    const DRV_Airframe_Params *airframe;
    const float amplitude = (spec != NULL) ? spec->amplitude_rad_s : 0.0f;
    float single_max_n;
    float thrust_n;
    float max_amp;
    float ku = 0.0f;
    float kl = 0.0f;
    uint32_t milli;

    yaw_init();
    airframe = DRV_Airframe_Get();
    if ((airframe == NULL) || !(yaw_weight_n() > 0.0f) ||
        !(airframe->izz_kgm2 > 0.0f) || (isfinite(airframe->izz_kgm2) == 0)) {
        return "airframe invalid";
    }
    if ((DRV_PropMap_YawTorquePolarity() == 0.0f) ||
        (DRV_PropMap_EscChannelForRole((uint8_t)DRV_PROP_ROLE_UPPER) == 0U) ||
        (DRV_PropMap_EscChannelForRole((uint8_t)DRV_PROP_ROLE_LOWER) == 0U)) {
        return "prop map uncalibrated (yaw polarity)";
    }
    DRV_COAX_CTRL_GetYawTorqueCoefficients(&ku, &kl);
    if (!((ku + kl) > 0.0f)) {
        return "yaw torque coefficient invalid";
    }
    thrust_n = APP_SysIdYaw_ThrustN();
    if (thrust_n > (APP_SYSID_YAW_LIFT_FRACTION * yaw_weight_n())) {
        return "thrust over lift limit (thrust_mn>0.8*weight)";
    }
    if (thrust_n < APP_SYSID_MIN_THRUST_N) {
        return "thrust too low (thrust_mn<2000)";
    }
    single_max_n = DRV_COAX_CTRL_SingleMaxThrustN();
    if ((thrust_n > airframe->max_total_force_n) || (thrust_n > (2.0f * single_max_n))) {
        return "thrust over max";
    }
    max_amp = yaw_amplitude_max(yaw.config.inject, thrust_n, single_max_n);
    if ((isfinite(amplitude) == 0) || !(amplitude > 0.0f) || !(amplitude <= max_amp)) {
        milli = (uint32_t)lroundf(max_amp * 1000.0f);
        (void)snprintf(yaw.refusal, sizeof(yaw.refusal), "amp over limit (inject=%s max=%lu.%03lu)",
                       APP_SysIdYaw_InjectName(yaw.config.inject),
                       (unsigned long)(milli / 1000UL), (unsigned long)(milli % 1000UL));
        return yaw.refusal;
    }
    return NULL;
}

void APP_SysIdYaw_Begin(const DRV_SysIdExcitation *spec, float max_pct)
{
    const DRV_Airframe_Params *airframe = DRV_Airframe_Get();
    float ku = 0.0f;
    float kl = 0.0f;

    yaw_init();
    yaw.run = yaw.config;
    yaw.run.thrust_mn = yaw_resolved_thrust_mn(&yaw.config);
    if (spec != NULL) {
        yaw.spec = *spec;
    }
    yaw.max_pct = max_pct;
    yaw.thrust_n = (float)yaw.run.thrust_mn * 0.001f;
    DRV_COAX_CTRL_GetYawTorqueCoefficients(&ku, &kl);
    yaw.k_m_per_n = 0.5f * (ku + kl);
    yaw.izz_kgm2 = (airframe != NULL) ? airframe->izz_kgm2 : 0.0f;
    yaw.single_max_n = DRV_COAX_CTRL_SingleMaxThrustN();
    yaw.twist_rad = (float)yaw.run.twist_deg * (YAW_PI / 180.0f);
    yaw.polarity = DRV_PropMap_YawTorquePolarity();
    yaw.psi_rad = 0.0f;
    yaw.rate_sp_rad_s = 0.0f;
    yaw.moment_nm = 0.0f;
    yaw.diff_n = 0.0f;
    yaw.record_thrust_n = 0.0f;
    DRV_RateControl_InitState(&yaw.rate_state);
    yaw.alloc_sat_pos = 0U;
    yaw.alloc_sat_neg = 0U;
    yaw.idle_pulse_us = 0U;
    yaw.ramp_from_upper_us = 0U;
    yaw.ramp_from_lower_us = 0U;
    yaw.upper_us = 0U;
    yaw.lower_us = 0U;
    yaw.soft_reason = NULL;
}

void APP_SysIdYaw_ReportStart(uint16_t run_id)
{
    yaw_init();
    APP_Control_QueueText(
        "SYSID YAWSTART run=%u yaw_inject=%s yaw_thrust_mn=%lu yaw_k_um_per_n=%ld "
        "yaw_izz_ugm2=%ld yaw_single_max_mn=%ld yaw_twist_deg=%u\r\n",
        (unsigned int)run_id, APP_SysIdYaw_InjectName(yaw.run.inject),
        (unsigned long)yaw.run.thrust_mn, (long)lroundf(yaw.k_m_per_n * 1.0e6f),
        (long)lroundf(yaw.izz_kgm2 * 1.0e6f), (long)lroundf(yaw.single_max_n * 1000.0f),
        (unsigned int)yaw.run.twist_deg);
}

/* ------------------------------------------------------------------ 出力小工具 */

static uint16_t yaw_cap_us(void)
{
    const float span = (float)(BSP_PWM_ESC_MAX_US - BSP_PWM_ESC_MIN_US);

    return (uint16_t)((float)BSP_PWM_ESC_MIN_US + yaw.max_pct * span / 100.0f + 0.5f);
}

/* 上/下桨推力 → 脉宽：同生产分配后的换算（有 pulses_for_pair 用它，否则逐桨查表），再按最高油门 % 封顶。 */
static void yaw_pulses_for(float upper_n, float lower_n, uint16_t *upper_us, uint16_t *lower_us,
                           uint8_t *capped)
{
    const DRV_COAX_CTRL_ThrustMap *map = DRV_COAX_CTRL_GetThrustMap();
    const uint16_t cap_us = yaw_cap_us();
    uint16_t up;
    uint16_t lo;

    if ((map != NULL) && (map->pulses_for_pair != NULL)) {
        up = 0U;
        lo = 0U;
        map->pulses_for_pair(upper_n, lower_n, &up, &lo);
        up = (up < BSP_PWM_ESC_MIN_US) ? (uint16_t)BSP_PWM_ESC_MIN_US :
             ((up > BSP_PWM_ESC_MAX_US) ? (uint16_t)BSP_PWM_ESC_MAX_US : up);
        lo = (lo < BSP_PWM_ESC_MIN_US) ? (uint16_t)BSP_PWM_ESC_MIN_US :
             ((lo > BSP_PWM_ESC_MAX_US) ? (uint16_t)BSP_PWM_ESC_MAX_US : lo);
    } else {
        up = DRV_COAX_CTRL_ThrustToMotorPulse(upper_n);
        lo = DRV_COAX_CTRL_ThrustToMotorPulse(lower_n);
    }
    *capped = ((up > cap_us) || (lo > cap_us)) ? 1U : 0U;
    *upper_us = (up > cap_us) ? cap_us : up;
    *lower_us = (lo > cap_us) ? cap_us : lo;
}

static uint16_t yaw_lerp_pulse(uint16_t from, uint16_t to, uint32_t t_ms, uint32_t span_ms)
{
    const int32_t delta = (int32_t)to - (int32_t)from;

    if ((span_ms == 0U) || (t_ms >= span_ms)) {
        return to;
    }
    return (uint16_t)((int32_t)from + (delta * (int32_t)t_ms) / (int32_t)span_ms);
}

/* 出口：记下本拍脉宽；推力 < 最小推力时不判推力门（RAMP_UP 早段）。 */
static void yaw_finish_output(APP_SysIdPhase phase, uint16_t upper_us, uint16_t lower_us,
                              uint8_t capped, float thrust_n, APP_SysIdYawOutput *out)
{
    yaw.upper_us = upper_us;
    yaw.lower_us = lower_us;
    out->phase = phase;
    out->upper_us = upper_us;
    out->lower_us = lower_us;
    out->capped = capped;
    out->thrust_n = thrust_n;
    yaw.record_thrust_n = thrust_n;
    if (thrust_n >= APP_SYSID_MIN_THRUST_N) {
        out->check_thrust = 1U;
    }
}

static uint16_t yaw_mean_pulse(uint16_t upper_us, uint16_t lower_us)
{
    return (uint16_t)(((uint32_t)upper_us + (uint32_t)lower_us) / 2U);
}

/* 两桨同脉宽段（起升/回落）的合推力读回：取两路平均。 */
static float yaw_readback_thrust_n(uint16_t upper_us, uint16_t lower_us)
{
    return yaw_finite_or(DRV_COAX_CTRL_MotorPulseToTotalThrust(yaw_mean_pulse(upper_us, lower_us)),
                         0.0f);
}

/*
 * 偏航角速度环：生产同一个 DRV_RateControl_Step，只让 z 轴有输入（x/y 的 omega、omega_sp 恒 0，
 * 输出与陀螺耦合项都是 0）。参数每拍从 DRV_COAX_CTRL_GetParams() 的 .rate 取。
 * 返回 0 = 控制器拒绝输入（参数或测量非法），调用方中止。
 */
static uint8_t yaw_run_rate_loop(const APP_SysIdObserve *obs, float rate_sp_rad_s, float dt_s,
                                 float *moment_unsat_nm, float *moment_cmd_nm)
{
    const DRV_Airframe_Params *airframe = DRV_Airframe_Get();
    DRV_COAX_CTRL_Params params;
    DRV_RateControl_Input in;
    DRV_RateControl_Output out;
    const float limit_nm = fmaxf(DRV_COAX_CTRL_YawLimitMomentNm(yaw.thrust_n), YAW_MOMENT_LIMIT_EPS);

    DRV_COAX_CTRL_GetParams(&params);
    memset(&in, 0, sizeof(in));
    in.omega[2] = obs->gyro_ctrl_rad_s[2];
    in.omega_sp[2] = rate_sp_rad_s;
    in.inertia[0] = airframe->ixx_kgm2;
    in.inertia[1] = airframe->iyy_kgm2;
    in.inertia[2] = airframe->izz_kgm2;
    in.dt_s = dt_s;
    in.saturation_positive[0] = YAW_UNUSED_AXIS_LIMIT_N_M;
    in.saturation_positive[1] = YAW_UNUSED_AXIS_LIMIT_N_M;
    in.saturation_positive[2] = limit_nm;
    in.saturation_negative[0] = -YAW_UNUSED_AXIS_LIMIT_N_M;
    in.saturation_negative[1] = -YAW_UNUSED_AXIS_LIMIT_N_M;
    in.saturation_negative[2] = -limit_nm;
    in.measurement_valid = 1U;
    in.integrator_enable = 1U;
    in.saturation_positive_active[2] = yaw.alloc_sat_pos;
    in.saturation_negative_active[2] = yaw.alloc_sat_neg;
    if (DRV_RateControl_Step(&params.rate, &yaw.rate_state, &in, &out) == 0U) {
        return 0U;
    }
    *moment_unsat_nm = out.moment_unsat[2];
    *moment_cmd_nm = out.moment_cmd[2];
    return 1U;
}

/*
 * SETTLE / PREROLL / EXCITE 共用的出力：由 shape（SETTLE/PREROLL 为 0）得 ΔT 或 r →
 * 偏航力矩 M → 同生产的差动分配 → 脉宽。M（饱和前）、ΔT、r 记在 yaw.* 里给记录用。
 * 返回 0 = 控制器拒绝输入。
 */
static uint8_t yaw_drive(const APP_SysIdObserve *obs, float dt_s, float shape, uint8_t excite,
                         APP_SysIdPhase phase, APP_SysIdYawOutput *out)
{
    const float amplitude = yaw.spec.amplitude_rad_s;
    float moment_unsat_nm;
    float moment_cmd_nm;
    float upper_n = 0.0f;
    float lower_n = 0.0f;
    uint16_t upper_us = 0U;
    uint16_t lower_us = 0U;
    uint8_t saturated = 0U;
    uint8_t capped = 0U;

    if (yaw.run.inject == (uint8_t)APP_SYSID_YAW_INJECT_RATE) {
        yaw.rate_sp_rad_s = (excite != 0U) ? (amplitude * shape) : 0.0f;
        if (yaw_run_rate_loop(obs, yaw.rate_sp_rad_s, dt_s, &moment_unsat_nm,
                              &moment_cmd_nm) == 0U) {
            return 0U;
        }
    } else {
        /* 开环：M = k·ΔT，不经任何限幅（幅值上限在开跑前已按分配边界核过）。 */
        yaw.rate_sp_rad_s = 0.0f;
        moment_unsat_nm = yaw.k_m_per_n * ((excite != 0U) ? (amplitude * shape) : 0.0f);
        moment_cmd_nm = moment_unsat_nm;
    }
    if ((isfinite(moment_unsat_nm) == 0) || (isfinite(moment_cmd_nm) == 0)) {
        return 0U;
    }
    DRV_COAX_CTRL_AllocateYawPair(yaw.thrust_n, moment_cmd_nm, &upper_n, &lower_n, &saturated);
    /* 同生产：分配器这一拍撞了限，下一拍告诉角速度环往哪个方向别再积分。 */
    yaw.alloc_sat_pos = ((saturated != 0U) && (moment_cmd_nm > 0.0f)) ? 1U : 0U;
    yaw.alloc_sat_neg = ((saturated != 0U) && (moment_cmd_nm < 0.0f)) ? 1U : 0U;
    yaw_pulses_for(upper_n, lower_n, &upper_us, &lower_us, &capped);
    yaw.moment_nm = moment_unsat_nm;
    yaw.diff_n = yaw.polarity * (lower_n - upper_n);
    yaw_finish_output(phase, upper_us, lower_us, capped, upper_n + lower_n, out);
    return 1U;
}

/* ------------------------------------------------------------------ 阶段推进与门 */

static APP_SysIdPhase yaw_advance(const APP_SysIdObserve *obs, APP_SysIdPhase phase,
                                  uint32_t *phase_ms, APP_SysIdYawOutput *out)
{
    APP_SysIdPhase next = phase;

    switch (phase) {
    case APP_SYSID_PHASE_IDLE:
        yaw.idle_pulse_us = obs->throttle_us;   /* 杆在最低 = 怠速：起升起点 */
        yaw.upper_us = obs->throttle_us;
        yaw.lower_us = obs->throttle_us;
        next = APP_SYSID_PHASE_RAMP_UP;
        break;
    case APP_SYSID_PHASE_RAMP_UP:
        if (*phase_ms >= APP_SYSID_YAW_RAMP_UP_MS) {
            DRV_RateControl_InitState(&yaw.rate_state);   /* 角速度环从稳定段起跑 */
            yaw.alloc_sat_pos = 0U;
            yaw.alloc_sat_neg = 0U;
            next = APP_SYSID_PHASE_SETTLE;
        }
        break;
    case APP_SYSID_PHASE_SETTLE:
        if (*phase_ms >= APP_SYSID_YAW_SETTLE_MS) {
            next = APP_SYSID_PHASE_PREROLL;
        }
        break;
    case APP_SYSID_PHASE_PREROLL:
        if (*phase_ms >= APP_SYSID_YAW_PREROLL_MS) {
            next = APP_SYSID_PHASE_EXCITE;
        }
        break;
    case APP_SYSID_PHASE_EXCITE:
        break;   /* 剖面跑完才切，在出力那一步判 */
    case APP_SYSID_PHASE_RAMP_DOWN:
        if (*phase_ms >= APP_SYSID_YAW_RAMP_DOWN_MS) {
            out->finished = 1U;
        }
        break;
    default:
        out->abort_reason = "phase";
        break;
    }
    if (next != phase) {
        *phase_ms = 0U;
    }
    return next;
}

/* 超速/绞角门：RAMP_DOWN 之外全程都判（RAMP_DOWN 时 ΔT 已清零，没有激励在驱动）。 */
static const char *yaw_gate(APP_SysIdPhase phase, const APP_SysIdObserve *obs)
{
    if (phase == APP_SYSID_PHASE_RAMP_DOWN) {
        return NULL;
    }
    if (fabsf(obs->gyro_rad_s[2]) > APP_SYSID_YAW_OVERSPEED_RAD_S) {
        return "yaw_overspeed";
    }
    if (fabsf(yaw.psi_rad) > yaw.twist_rad) {
        return "yaw_twist";
    }
    return NULL;
}

/* ------------------------------------------------------------------ 软停 */

static void yaw_soft_begin(const APP_SysIdObserve *obs, const char *reason)
{
    yaw.soft_reason = reason;
    /* ΔT 立刻清零：两桨从当前平均脉宽起一起线性回落，不各自从原脉宽走（那样差速要 1 s 才消掉）。 */
    yaw.soft_from_upper_us = yaw_mean_pulse(yaw.upper_us, yaw.lower_us);
    yaw.soft_from_lower_us = yaw.soft_from_upper_us;
    yaw.soft_start_ms = obs->now_ms;
}

/* 软停的一拍（阶段报 ramp_down）：ΔT 与 r 清零、环停，推力 1 s 线性降到遥控油门；走完报进入时的理由。 */
static void yaw_soft_step(const APP_SysIdObserve *obs, APP_SysIdYawOutput *out)
{
    const uint32_t elapsed = (uint32_t)(obs->now_ms - yaw.soft_start_ms);
    uint16_t upper_us;
    uint16_t lower_us;

    yaw.rate_sp_rad_s = 0.0f;
    yaw.moment_nm = 0.0f;
    yaw.diff_n = 0.0f;
    if (elapsed >= APP_SYSID_YAW_RAMP_DOWN_MS) {
        out->abort_reason = yaw.soft_reason;
        return;
    }
    upper_us = yaw_lerp_pulse(yaw.soft_from_upper_us, obs->throttle_us, elapsed,
                              APP_SYSID_YAW_RAMP_DOWN_MS);
    lower_us = yaw_lerp_pulse(yaw.soft_from_lower_us, obs->throttle_us, elapsed,
                              APP_SYSID_YAW_RAMP_DOWN_MS);
    yaw_finish_output(APP_SYSID_PHASE_RAMP_DOWN, upper_us, lower_us, 0U,
                      yaw_readback_thrust_n(upper_us, lower_us), out);
}

/* ------------------------------------------------------------------ 一拍 */

static void yaw_step_run(const APP_SysIdObserve *obs, APP_SysIdPhase phase, uint32_t phase_ms,
                         float dt_s, APP_SysIdYawOutput *out)
{
    float shape = 0.0f;
    uint8_t excite = 0U;

    phase = yaw_advance(obs, phase, &phase_ms, out);
    if (out->abort_reason != NULL) {
        return;
    }
    if (phase == APP_SYSID_PHASE_EXCITE) {
        DRV_SysIdExcSample excitation;

        if (DRV_SysIdExcitation_Eval(&yaw.spec, phase_ms, &excitation) != DRV_SYSID_EXC_OK) {
            out->abort_reason = "excitation";
            return;
        }
        if (excitation.finished != 0U) {
            yaw.ramp_from_upper_us = yaw_mean_pulse(yaw.upper_us, yaw.lower_us);   /* 末拍 ΔT 本就是 0 */
            yaw.ramp_from_lower_us = yaw.ramp_from_upper_us;
            phase = APP_SYSID_PHASE_RAMP_DOWN;
            phase_ms = 0U;
        } else {
            /* 现有剖面按单位幅值归一化（Validate 保证 amp > 0），再乘本注入类型的幅值。 */
            shape = yaw_finite_or(excitation.omega_sp_rad_s / yaw.spec.amplitude_rad_s, 0.0f);
            shape = fmaxf(-1.0f, fminf(1.0f, shape));
            excite = 1U;
        }
    }
    out->abort_reason = yaw_gate(phase, obs);
    if (out->abort_reason != NULL) {
        return;
    }

    switch (phase) {
    case APP_SYSID_PHASE_RAMP_UP: {
        uint16_t target_upper_us = 0U;
        uint16_t target_lower_us = 0U;
        uint8_t capped = 0U;
        uint16_t upper_us;
        uint16_t lower_us;

        yaw.rate_sp_rad_s = 0.0f;
        yaw.moment_nm = 0.0f;
        yaw.diff_n = 0.0f;
        yaw_pulses_for(0.5f * yaw.thrust_n, 0.5f * yaw.thrust_n, &target_upper_us,
                       &target_lower_us, &capped);
        upper_us = yaw_lerp_pulse(yaw.idle_pulse_us, target_upper_us, phase_ms,
                                  APP_SYSID_YAW_RAMP_UP_MS);
        lower_us = yaw_lerp_pulse(yaw.idle_pulse_us, target_lower_us, phase_ms,
                                  APP_SYSID_YAW_RAMP_UP_MS);
        yaw_finish_output(phase, upper_us, lower_us, capped,
                          yaw_readback_thrust_n(upper_us, lower_us), out);
        break;
    }
    case APP_SYSID_PHASE_SETTLE:
    case APP_SYSID_PHASE_PREROLL:
    case APP_SYSID_PHASE_EXCITE:
        if (yaw_drive(obs, dt_s, shape, excite, phase, out) == 0U) {
            out->abort_reason = "controller";
        }
        break;
    case APP_SYSID_PHASE_RAMP_DOWN: {
        /* 回落终点同现有自动油门：本拍遥控器给的脉宽（杆在最低即怠速）。 */
        const uint16_t upper_us = yaw_lerp_pulse(yaw.ramp_from_upper_us, obs->throttle_us,
                                                 phase_ms, APP_SYSID_YAW_RAMP_DOWN_MS);
        const uint16_t lower_us = yaw_lerp_pulse(yaw.ramp_from_lower_us, obs->throttle_us,
                                                 phase_ms, APP_SYSID_YAW_RAMP_DOWN_MS);

        yaw.rate_sp_rad_s = 0.0f;
        yaw.moment_nm = 0.0f;
        yaw.diff_n = 0.0f;
        yaw_finish_output(phase, upper_us, lower_us, 0U, yaw_readback_thrust_n(upper_us, lower_us),
                          out);
        break;
    }
    default:
        out->abort_reason = "phase";
        break;
    }
}

void APP_SysIdYaw_Step(const APP_SysIdObserve *obs, APP_SysIdPhase phase, uint32_t phase_ms,
                       float dt_s, APP_SysIdYawOutput *out)
{
    yaw_init();
    if ((obs == NULL) || (out == NULL)) {
        return;
    }
    memset(out, 0, sizeof(*out));
    if ((isfinite(obs->gyro_rad_s[2]) != 0) && (isfinite(dt_s) != 0) && (dt_s > 0.0f)) {
        yaw.psi_rad += obs->gyro_rad_s[2] * dt_s;
    }
    if (yaw.soft_reason == NULL) {
        yaw_step_run(obs, phase, phase_ms, dt_s, out);
        if (out->abort_reason == NULL) {
            return;
        }
        yaw_soft_begin(obs, out->abort_reason);
        memset(out, 0, sizeof(*out));
    }
    yaw_soft_step(obs, out);
}

/* ------------------------------------------------------------------ 记录 */

void APP_SysIdYaw_FillSample(DRV_SysIdSample *sample, const APP_SysIdObserve *obs)
{
    yaw_init();
    if ((sample == NULL) || (obs == NULL)) {
        return;
    }
    sample->thrust_n = yaw.record_thrust_n;
    sample->torque_n_m = yaw.moment_nm;
    sample->height_m = yaw.psi_rad;
    sample->height_raw_m = 0.0f;
    sample->height_sp_m = 0.0f;
    sample->vz_m_s = obs->gyro_rad_s[2];     /* 原始陀螺 z，未陷波 */
    sample->vz_sp_m_s = yaw.rate_sp_rad_s;
    sample->az_m_s2 = yaw.diff_n;
    sample->vbat_v = obs->vbat_v;
}

float APP_SysIdYaw_GetPsiRad(void)
{
    yaw_init();
    return yaw.psi_rad;
}

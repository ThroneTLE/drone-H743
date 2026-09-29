/*
 * app_sysid_alt.c —— 光杆台架高度辨识（SYSID MODE ALT）的序列、高度环与注入。
 *
 * 设计与口径写在 App/Inc/app_sysid_alt.h。安全门、姿态链、采样网格、油门出口都是
 * app_sysid.c 已有的那一套，本文件只算"这一拍在哪个阶段、电机给多少、参考是多少"。
 *
 * 状态放 AXI SRAM（NOLOAD，首次调用时显式清）：DTCM 已被辨识采样环占去一大块。
 * 配置只在命令任务里改（且只在空闲时），本轮用的那份在开跑时锁存；控制拍只写
 * 运行状态与"最近测高"两项，命令任务只读后者。
 */

#include "app_sysid_alt.h"

#include "app_control.h"
#include "app_control_scheduler.h"

#include "drv_airframe_params.h"
#include "drv_coax_ctrl.h"
#include "drv_position_control.h"
#include "bsp_pwm.h"   /* 只取电调脉宽量程常量做封顶；写电调仍只在稳定环 */

#include <math.h>
#include <string.h>

typedef struct {
    APP_SysIdAltConfig config;          /* 命令面改的那份 */
    volatile uint8_t live_valid;        /* 最近一拍测高是否有效 */
    volatile float live_height_m;       /* 最近一次**有效**的测高 */

    /* 本轮（APP_SysIdAlt_Begin 锁存） */
    APP_SysIdAltConfig run;
    DRV_SysIdExcitation spec;
    float mass_kg;
    float gravity_m_s2;
    float max_pct;
    float h0_m;
    float z_hold_m;
    float z_land_m;
    float z_ref_m;           /* 爬升/回落段的斜坡参考 */
    float z_sp_m;            /* 本拍高度参考（记录 height_sp） */
    float vz_sp_m_s;         /* 位置环给速度环的参考，含前馈（记录 vz_sp） */
    float v_inj_int_m;       /* vel 注入的 ∫v_inj dt */
    float accel_cmd_m_s2;    /* 速度环最近一次输出 a_z，两次速度环更新之间保持 */
    DRV_POSITION_CONTROL_State loop;
    uint32_t last_position_us;
    uint32_t last_velocity_us;
    uint32_t last_height_ms; /* 最近一拍测高有效的时刻 */
    uint16_t idle_pulse_us;
    uint16_t ramp_from_us;
    uint16_t pulse_us;
    uint8_t loop_started;    /* 高度环已接管；0 = 下一次更新先复位积分 */
    uint8_t capped;          /* 上一拍被封顶：喂速度环的下游饱和，同生产 */
} SysIdAltContext;

#if defined(__GNUC__) && defined(__arm__)
__attribute__((section(".ram_d1_noinit"), aligned(32)))
#endif
static SysIdAltContext alt;
static uint8_t alt_initialised;

static const char *const alt_inject_names[APP_SYSID_ALT_INJECT_COUNT] = { "force", "vel", "pos" };

static void alt_init(void)
{
    if (alt_initialised != 0U) {
        return;
    }
    alt_initialised = 1U;
    memset(&alt, 0, sizeof(alt));
    alt.config.inject = (uint8_t)APP_SYSID_ALT_INJECT_FORCE;
    alt.config.mass_g = 0U;
    alt.config.win_mm = APP_SYSID_ALT_WIN_MM_DEFAULT;
    alt.config.lift_mm = APP_SYSID_ALT_LIFT_MM_DEFAULT;
}

static float alt_finite_or(float value, float fallback)
{
    return (isfinite(value) != 0) ? value : fallback;
}

static uint8_t alt_config_valid(const APP_SysIdAltConfig *config)
{
    return ((config != NULL) &&
            (config->inject < (uint8_t)APP_SYSID_ALT_INJECT_COUNT) &&
            ((config->mass_g == 0U) ||
             ((config->mass_g >= APP_SYSID_ALT_MASS_G_MIN) &&
              (config->mass_g <= APP_SYSID_ALT_MASS_G_MAX))) &&
            (config->win_mm >= APP_SYSID_ALT_WIN_MM_MIN) &&
            (config->win_mm <= APP_SYSID_ALT_WIN_MM_MAX) &&
            (config->lift_mm >= APP_SYSID_ALT_LIFT_MM_MIN) &&
            (config->lift_mm <= APP_SYSID_ALT_LIFT_MM_MAX)) ? 1U : 0U;
}

/* m_run：配置了就用台架质量，0 取机体质量（airframe 只读，不改）。 */
static float alt_run_mass_kg(const APP_SysIdAltConfig *config)
{
    const DRV_Airframe_Params *airframe = DRV_Airframe_Get();

    if (config->mass_g != 0U) {
        return (float)config->mass_g / 1000.0f;
    }
    return (airframe != NULL) ? airframe->mass_kg : 0.0f;
}

/* ------------------------------------------------------------------ 配置面 */

void APP_SysIdAlt_GetConfig(APP_SysIdAltConfig *out)
{
    alt_init();
    if (out != NULL) {
        *out = alt.config;
    }
}

uint8_t APP_SysIdAlt_SetConfig(const APP_SysIdAltConfig *config)
{
    alt_init();
    if (alt_config_valid(config) == 0U) {
        return 0U;
    }
    alt.config = *config;
    return 1U;
}

const char *APP_SysIdAlt_InjectName(uint8_t inject)
{
    return (inject < (uint8_t)APP_SYSID_ALT_INJECT_COUNT) ? alt_inject_names[inject] : "?";
}

uint8_t APP_SysIdAlt_InjectFromName(const char *name, uint8_t *out)
{
    if ((name == NULL) || (out == NULL)) {
        return 0U;
    }
    for (uint8_t i = 0U; i < (uint8_t)APP_SYSID_ALT_INJECT_COUNT; ++i) {
        if (strcmp(name, alt_inject_names[i]) == 0) {
            *out = i;
            return 1U;
        }
    }
    return 0U;
}

void APP_SysIdAlt_ReportConfig(void)
{
    alt_init();
    APP_Control_QueueText("SYSID ALT inject=%s mass_g=%u win_mm=%u lift_mm=%u\r\n",
                          APP_SysIdAlt_InjectName(alt.config.inject),
                          (unsigned int)alt.config.mass_g,
                          (unsigned int)alt.config.win_mm,
                          (unsigned int)alt.config.lift_mm);
}

float APP_SysIdAlt_VerticalAccel(float accel_x_g, float accel_y_g, float accel_z_g,
                                 float roll_rad, float pitch_rad, float gravity_m_s2)
{
    const float cr = cosf(roll_rad);
    const float sr = sinf(roll_rad);
    const float cp = cosf(pitch_rad);
    const float sp = sinf(pitch_rad);
    /* Ry(pitch)·Rx(roll) 的第三行：机体比力在竖直方向的分量（静止水平时 = 1 g）。 */
    const float up_g = (-sp * accel_x_g) + ((cp * sr) * accel_y_g) + ((cp * cr) * accel_z_g);

    return (up_g - 1.0f) * gravity_m_s2;
}

/* ------------------------------------------------------------------ 运行面 */

void APP_SysIdAlt_Observe(const APP_SysIdObserve *obs)
{
    alt_init();
    if (obs == NULL) {
        return;
    }
    if ((obs->height_valid != 0U) && (isfinite(obs->height_m) != 0)) {
        alt.live_height_m = obs->height_m;
        alt.live_valid = 1U;
    } else {
        alt.live_valid = 0U;
    }
}

const char *APP_SysIdAlt_Precheck(float throttle_target_n, float amplitude)
{
    static const float amp_max[APP_SYSID_ALT_INJECT_COUNT] = {
        APP_SYSID_ALT_FORCE_AMP_MAX_N, APP_SYSID_ALT_VEL_AMP_MAX_M_S, APP_SYSID_ALT_POS_AMP_MAX_M,
    };
    static const char *const amp_refusal[APP_SYSID_ALT_INJECT_COUNT] = {
        "amp over limit: inject=force amp<=3 N",
        "amp over limit: inject=vel amp<=0.3 m/s",
        "amp over limit: inject=pos amp<=0.15 m",
    };
    float mass_kg;

    alt_init();
    if (!(throttle_target_n > 0.0f)) {
        /* 起升、高度环、回落都要本模块给油门；遥控器手动给油门没法跑这一套。 */
        return "needs auto throttle (SYSID THROTTLE target_n>0)";
    }
    if (alt_config_valid(&alt.config) == 0U) {
        return "config invalid";
    }
    if ((isfinite(amplitude) == 0) || !(amplitude > 0.0f) ||
        (amplitude > amp_max[alt.config.inject])) {
        return amp_refusal[alt.config.inject];
    }
    mass_kg = alt_run_mass_kg(&alt.config);
    if ((isfinite(mass_kg) == 0) || !(mass_kg > 0.0f)) {
        return "mass unknown (set SYSID ALT mass_g= or airframe mass)";
    }
    if (alt.live_valid == 0U) {
        return "height invalid (TOF)";
    }
    return NULL;
}

static void alt_loop_reset(void)
{
    DRV_POSITION_CONTROL_ResetState(&alt.loop);
    alt.loop_started = 0U;
    alt.accel_cmd_m_s2 = 0.0f;
    alt.vz_sp_m_s = 0.0f;
    alt.last_position_us = 0U;
    alt.last_velocity_us = 0U;
}

void APP_SysIdAlt_Begin(const DRV_SysIdExcitation *spec, float max_pct)
{
    const DRV_Airframe_Params *airframe = DRV_Airframe_Get();

    alt_init();
    alt.run = alt.config;
    if (spec != NULL) {
        alt.spec = *spec;
    }
    alt.max_pct = max_pct;
    alt.mass_kg = alt_run_mass_kg(&alt.run);
    alt.gravity_m_s2 = (airframe != NULL) ? airframe->gravity_m_s2 : 9.81f;
    alt.h0_m = alt.live_height_m;
    alt.z_hold_m = alt.h0_m + ((float)alt.run.lift_mm * 0.001f);
    alt.z_land_m = alt.h0_m + APP_SYSID_ALT_LAND_ABOVE_M;
    alt.z_ref_m = alt.h0_m;
    alt.z_sp_m = alt.h0_m;
    alt.v_inj_int_m = 0.0f;
    alt.idle_pulse_us = 0U;
    alt.ramp_from_us = 0U;
    alt.pulse_us = 0U;
    alt.capped = 0U;
    alt_loop_reset();
}

void APP_SysIdAlt_ReportStart(uint16_t run_id)
{
    alt_init();
    APP_Control_QueueText(
        "SYSID ALTSTART run=%u alt_inject=%s alt_mass_g=%ld alt_win_mm=%u alt_lift_mm=%u "
        "alt_h0_mm=%ld\r\n",
        (unsigned int)run_id, APP_SysIdAlt_InjectName(alt.run.inject),
        (long)lroundf(alt.mass_kg * 1000.0f), (unsigned int)alt.run.win_mm,
        (unsigned int)alt.run.lift_mm, (long)lroundf(alt.h0_m * 1000.0f));
}

/* 与 app_sysid.c 的 sysid_target_pulse 同一口径：单桨 = 合推力/2 查表，再按最高油门 % 封顶。 */
static uint16_t alt_pulse_for(float total_n, uint8_t *capped)
{
    const float span = (float)(BSP_PWM_ESC_MAX_US - BSP_PWM_ESC_MIN_US);
    const uint16_t cap_us = (uint16_t)((float)BSP_PWM_ESC_MIN_US +
                                       alt.max_pct * span / 100.0f + 0.5f);
    const uint16_t pulse = DRV_COAX_CTRL_ThrustToMotorPulse(0.5f * total_n);

    *capped = (pulse > cap_us) ? 1U : 0U;
    return (pulse > cap_us) ? cap_us : pulse;
}

static uint16_t alt_lerp_pulse(uint16_t from, uint16_t to, uint32_t t_ms, uint32_t span_ms)
{
    const int32_t delta = (int32_t)to - (int32_t)from;

    if ((span_ms == 0U) || (t_ms >= span_ms)) {
        return to;
    }
    return (uint16_t)((int32_t)from + (delta * (int32_t)t_ms) / (int32_t)span_ms);
}

static float alt_due_dt(uint32_t now_us, uint32_t last_us, uint32_t period_us, uint8_t first)
{
    if (first != 0U) {
        return (float)period_us * 1.0e-6f;
    }
    return ((uint32_t)(now_us - last_us) >= period_us) ?
           ((float)(uint32_t)(now_us - last_us) * 1.0e-6f) : 0.0f;
}

/*
 * 生产高度环的 z 通道：位置 P（50 Hz）→ 速度 PI(D)（100 Hz），参数就是在飞那一份。
 * 测高短暂无效时不更新、保持上一次的 a_z（连续无效超时由测高门中止）。返回 0 = 控制器
 * 拒绝输入（参数或测量非法），调用方中止。
 */
static uint8_t alt_run_loop(const APP_SysIdObserve *obs, float v_ff_m_s)
{
    const uint32_t position_period_us = (uint32_t)APP_CONTROL_SCHED_POSITION_PERIOD_US;
    const uint32_t velocity_period_us = (uint32_t)APP_CONTROL_SCHED_VELOCITY_PERIOD_US;
    const uint8_t first = (alt.loop_started == 0U) ? 1U : 0U;
    DRV_COAX_CTRL_Params params;
    float dt_s;

    if ((obs->height_valid == 0U) || (isfinite(obs->height_m) == 0) ||
        (isfinite(obs->vz_m_s) == 0)) {
        return 1U;
    }
    DRV_COAX_CTRL_GetParams(&params);

    dt_s = alt_due_dt(obs->now_us, alt.last_position_us, position_period_us, first);
    if (dt_s > 0.0f) {
        DRV_POSITION_CONTROL_PositionInput input;
        DRV_POSITION_CONTROL_PositionOutput output;

        memset(&input, 0, sizeof(input));
        input.position_sp_m[2] = alt.z_sp_m;
        input.position_meas_m[2] = obs->height_m;
        input.velocity_ff_m_s[2] = v_ff_m_s;
        input.dt_sec = dt_s;
        input.measurement_valid = 1U;
        DRV_POSITION_CONTROL_PositionStep(&params.position, &input, &output);
        if ((output.flags & (DRV_POSITION_CONTROL_FLAG_INPUT_INVALID |
                             DRV_POSITION_CONTROL_FLAG_MEAS_INVALID |
                             DRV_POSITION_CONTROL_FLAG_DT_INVALID)) != 0U) {
            return 0U;
        }
        alt.vz_sp_m_s = output.velocity_sp_m_s[2];
        alt.last_position_us = obs->now_us;
    }

    dt_s = alt_due_dt(obs->now_us, alt.last_velocity_us, velocity_period_us, first);
    if (dt_s > 0.0f) {
        DRV_POSITION_CONTROL_VelocityInput input;
        DRV_POSITION_CONTROL_VelocityOutput output;

        memset(&input, 0, sizeof(input));
        input.velocity_sp_m_s[2] = alt.vz_sp_m_s;
        input.velocity_meas_m_s[2] = obs->vz_m_s;
        input.measured_accel_m_s2[2] = alt_finite_or(obs->az_m_s2, 0.0f);
        input.dt_sec = dt_s;
        input.measurement_valid = 1U;
        input.integrator_enable = 1U;
        input.integrator_reset = first;
        /* 同生产（drv_coax_ctrl 的 translation_saturation）：推力顶住且还要往上加时冻结积分。 */
        input.downstream_saturation.horizontal_scale = 1.0f;
        input.downstream_saturation.thrust_saturated = alt.capped;
        input.downstream_saturation.pos_limit[2] =
            ((alt.capped != 0U) && (alt.accel_cmd_m_s2 > 0.0f)) ? 1U : 0U;
        DRV_POSITION_CONTROL_VelocityStep(&params.position, &alt.loop, &input, &output);
        if ((output.flags & DRV_POSITION_CONTROL_FLAG_INPUT_INVALID) != 0U) {
            return 0U;
        }
        alt.accel_cmd_m_s2 = output.accel_sat_m_s2[2];
        alt.last_velocity_us = obs->now_us;
    }
    alt.loop_started = 1U;
    return 1U;
}

/* ① 阶段推进。条件满足本拍就切，返回新阶段并把 *phase_ms 清零（本拍按新阶段出力）。 */
static APP_SysIdPhase alt_advance(const APP_SysIdObserve *obs, APP_SysIdPhase phase,
                                  uint32_t *phase_ms, APP_SysIdAltOutput *out)
{
    APP_SysIdPhase next = phase;

    switch (phase) {
    case APP_SYSID_PHASE_IDLE:
        alt.idle_pulse_us = obs->throttle_us;   /* 杆在最低 = 怠速：起升起点 */
        alt.pulse_us = obs->throttle_us;
        alt.last_height_ms = obs->now_ms;
        next = APP_SYSID_PHASE_RAMP_UP;
        break;
    case APP_SYSID_PHASE_RAMP_UP:
        if (*phase_ms >= APP_SYSID_ALT_RAMP_UP_MS) {
            alt_loop_reset();
            alt.z_ref_m = alt.h0_m;
            next = APP_SYSID_PHASE_CLIMB;
        }
        break;
    case APP_SYSID_PHASE_CLIMB:
        if (alt.z_ref_m >= alt.z_hold_m) {
            next = APP_SYSID_PHASE_SETTLE;
        }
        break;
    case APP_SYSID_PHASE_SETTLE:
        if (*phase_ms >= APP_SYSID_ALT_SETTLE_MS) {
            next = APP_SYSID_PHASE_PREROLL;
        }
        break;
    case APP_SYSID_PHASE_PREROLL:
        if (*phase_ms >= APP_SYSID_ALT_PREROLL_MS) {
            alt.v_inj_int_m = 0.0f;
            next = APP_SYSID_PHASE_EXCITE;
        }
        break;
    case APP_SYSID_PHASE_EXCITE:
        break;   /* 剖面跑完才切，在出力那一步判 */
    case APP_SYSID_PHASE_DESCEND:
        if (alt.z_ref_m <= alt.z_land_m) {
            alt.ramp_from_us = alt.pulse_us;
            next = APP_SYSID_PHASE_RAMP_DOWN;
        }
        break;
    case APP_SYSID_PHASE_RAMP_DOWN:
        if (*phase_ms >= APP_SYSID_ALT_RAMP_DOWN_MS) {
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

/* ② 测高门：起升到回落之前都判。返回中止理由或 NULL。 */
static const char *alt_height_gate(const APP_SysIdObserve *obs)
{
    if ((obs->height_valid != 0U) && (isfinite(obs->height_m) != 0)) {
        alt.last_height_ms = obs->now_ms;
        if ((obs->height_m < (alt.h0_m - APP_SYSID_ALT_BELOW_START_M)) ||
            (obs->height_m > (alt.z_hold_m + ((float)alt.run.win_mm * 0.001f)))) {
            return "height_window";
        }
        return NULL;
    }
    if ((uint32_t)(obs->now_ms - alt.last_height_ms) > APP_SYSID_ALT_HEIGHT_TIMEOUT_MS) {
        return "height_invalid";
    }
    return NULL;
}

void APP_SysIdAlt_Step(const APP_SysIdObserve *obs, APP_SysIdPhase phase, uint32_t phase_ms,
                       float dt_s, APP_SysIdAltOutput *out)
{
    float amplitude;
    float shape = 0.0f;
    float v_ff_m_s = 0.0f;
    float force_inj_n = 0.0f;
    uint16_t pulse;
    uint8_t capped = 0U;
    uint8_t loop = 0U;

    alt_init();
    if ((obs == NULL) || (out == NULL)) {
        return;
    }
    memset(out, 0, sizeof(*out));
    amplitude = alt.spec.amplitude_rad_s;
    phase = alt_advance(obs, phase, &phase_ms, out);
    pulse = alt.pulse_us;
    if (out->abort_reason != NULL) {
        return;
    }
    if (phase == APP_SYSID_PHASE_EXCITE) {
        DRV_SysIdExcSample excitation;

        if (DRV_SysIdExcitation_Eval(&alt.spec, phase_ms, &excitation) != DRV_SYSID_EXC_OK) {
            out->abort_reason = "excitation";
            return;
        }
        if (excitation.finished != 0U) {
            alt.z_ref_m = alt.z_sp_m;   /* 从激励结束时的参考开始回落（vel 注入可能已偏离 z_hold） */
            phase = APP_SYSID_PHASE_DESCEND;
            phase_ms = 0U;
        } else {
            /* 现有剖面按单位幅值归一化（Validate 保证 amp > 0），再乘本注入类型的幅值。 */
            shape = alt_finite_or(excitation.omega_sp_rad_s / amplitude, 0.0f);
            shape = fmaxf(-1.0f, fminf(1.0f, shape));
        }
    }
    if (phase != APP_SYSID_PHASE_RAMP_DOWN) {
        out->abort_reason = alt_height_gate(obs);
        if (out->abort_reason != NULL) {
            return;
        }
    }

    /* ③ 参考与注入 */
    switch (phase) {
    case APP_SYSID_PHASE_RAMP_UP:
        pulse = alt_lerp_pulse(alt.idle_pulse_us,
                               alt_pulse_for(APP_SYSID_ALT_RAMP_UP_FRACTION * alt.mass_kg *
                                             alt.gravity_m_s2, &capped),
                               phase_ms, APP_SYSID_ALT_RAMP_UP_MS);
        alt.z_sp_m = alt.h0_m;
        alt.vz_sp_m_s = 0.0f;
        break;
    case APP_SYSID_PHASE_CLIMB:
        alt.z_ref_m = fminf(alt.z_ref_m + (APP_SYSID_ALT_REF_SPEED_M_S * dt_s), alt.z_hold_m);
        v_ff_m_s = (alt.z_ref_m < alt.z_hold_m) ? APP_SYSID_ALT_REF_SPEED_M_S : 0.0f;
        alt.z_sp_m = alt.z_ref_m;
        loop = 1U;
        break;
    case APP_SYSID_PHASE_SETTLE:
    case APP_SYSID_PHASE_PREROLL:
        alt.z_sp_m = alt.z_hold_m;
        loop = 1U;
        break;
    case APP_SYSID_PHASE_EXCITE:
        alt.z_sp_m = alt.z_hold_m;
        if (alt.run.inject == (uint8_t)APP_SYSID_ALT_INJECT_FORCE) {
            force_inj_n = amplitude * shape;
        } else if (alt.run.inject == (uint8_t)APP_SYSID_ALT_INJECT_VEL) {
            v_ff_m_s = amplitude * shape;
            alt.v_inj_int_m += v_ff_m_s * dt_s;
            alt.z_sp_m = alt.z_hold_m + alt.v_inj_int_m;
        } else {
            alt.z_sp_m = alt.z_hold_m + (amplitude * shape);
        }
        loop = 1U;
        break;
    case APP_SYSID_PHASE_DESCEND:
        alt.z_ref_m = fmaxf(alt.z_ref_m - (APP_SYSID_ALT_REF_SPEED_M_S * dt_s), alt.z_land_m);
        v_ff_m_s = (alt.z_ref_m > alt.z_land_m) ? -APP_SYSID_ALT_REF_SPEED_M_S : 0.0f;
        alt.z_sp_m = alt.z_ref_m;
        loop = 1U;
        break;
    case APP_SYSID_PHASE_RAMP_DOWN:
        /* 回落终点同现有自动油门：本拍遥控器给的脉宽（杆在最低即怠速）。 */
        pulse = alt_lerp_pulse(alt.ramp_from_us, obs->throttle_us, phase_ms,
                               APP_SYSID_ALT_RAMP_DOWN_MS);
        alt.vz_sp_m_s = 0.0f;
        break;
    default:
        out->abort_reason = "phase";
        return;
    }

    /* ④ 高度环 → 合推力（+ force 注入）→ 查补表 → 封顶 */
    if (loop != 0U) {
        float total_n;

        if (alt_run_loop(obs, v_ff_m_s) == 0U) {
            out->abort_reason = "controller";
            return;
        }
        total_n = (alt.mass_kg * (alt.gravity_m_s2 + alt.accel_cmd_m_s2)) + force_inj_n;
        if (isfinite(total_n) == 0) {
            out->abort_reason = "controller";
            return;
        }
        pulse = alt_pulse_for(total_n, &capped);
        out->hold_attitude = 1U;
        out->check_thrust = 1U;
    }
    alt.pulse_us = pulse;
    alt.capped = capped;
    out->phase = phase;
    out->motor_pulse_us = pulse;
    out->capped = capped;
    out->thrust_n = alt_finite_or(DRV_COAX_CTRL_MotorPulseToTotalThrust(pulse), 0.0f);
}

void APP_SysIdAlt_FillSample(DRV_SysIdSample *sample, const APP_SysIdObserve *obs)
{
    const uint8_t valid = ((obs != NULL) && (obs->height_valid != 0U)) ? 1U : 0U;

    alt_init();
    if ((sample == NULL) || (obs == NULL)) {
        return;
    }
    sample->height_m = (valid != 0U) ? obs->height_m : 0.0f;
    sample->height_raw_m = (valid != 0U) ? obs->height_raw_m : 0.0f;
    sample->height_sp_m = alt.z_sp_m;
    sample->vz_m_s = (valid != 0U) ? obs->vz_m_s : 0.0f;
    sample->vz_sp_m_s = alt.vz_sp_m_s;
    sample->az_m_s2 = obs->az_m_s2;
    sample->vbat_v = obs->vbat_v;
}

void APP_SysIdAlt_GetLive(float *height_m, uint8_t *valid, float *height_sp_m)
{
    alt_init();
    if (height_m != NULL) {
        *height_m = alt.live_height_m;
    }
    if (valid != NULL) {
        *valid = alt.live_valid;
    }
    if (height_sp_m != NULL) {
        *height_sp_m = alt.z_sp_m;
    }
}

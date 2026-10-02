/*
 * app_sysid_xy.c —— 水平槽 XY 速度/位置辨识（SYSID MODE XY）。
 *
 * 设计与口径写在 App/Inc/app_sysid_xy.h，两侧契约 doc/sysid-xy-contract.md。安全门、姿态链、
 * 采样网格、油门出口都是 app_sysid.c 已有的那一套，本文件只算"这一拍在哪个阶段、电机给多少、
 * 倾角偏置是多少"。结构照 app_sysid_alt.c，但 XY 没有测高可看：软停按时间线性降推力。
 *
 * 状态放 AXI SRAM（NOLOAD，首次调用时显式清），不占 DTCM。配置只在命令任务里改（且只在空闲时），
 * 本轮用的那份在开跑时锁存；控制拍只写运行状态与"最近光流"几项，命令任务只读后者。
 */

#include "app_sysid_xy.h"

#include "app_control.h"
#include "app_control_scheduler.h"

#include "drv_airframe_params.h"
#include "drv_coax_ctrl.h"
#include "drv_position_control.h"
#include "bsp_pwm.h"   /* 只取电调脉宽量程常量做封顶；写电调仍只在稳定环 */

#include <math.h>
#include <stdio.h>
#include <string.h>

#define XY_LUT_CAP_TOL_N 0.10f

/* 软停只清倾角、靠动摩擦滑停；0.2 m/s² 取实测台架动摩擦的偏保守值（真实减速更大）。 */
#define XY_COAST_DECEL_M_S2 0.2f
/* 速度幅值上限：vel 注入幅值上限 0.3 m/s 之上留余量；方向设反闭环正反馈时靠它和出窗门兜底。 */
#define XY_MAX_SPEED_M_S 0.4f
/* 偏航只监视不控制（XY 模式偏航开环）：积分偏航超 15 度软停。 */
#define XY_YAW_LIMIT_RAD (15.0f * 3.14159265f / 180.0f)
/* 光流缺口容忍：连续无效不超过此时长，保持上一拍输出不软停。 */
#define XY_FLOW_GAP_TOL_MS 200
/* pos 注入幅值不得超过窗口的此比例（留出过冲余量）。 */
#define XY_POS_AMP_WIN_FRACTION 0.7f

typedef struct {
    APP_SysIdXyConfig config;           /* 命令面改的那份 */
    volatile uint8_t live_flow_ok;      /* 最近一拍光流速度/位置新鲜有效 */
    volatile uint8_t live_height_ok;    /* 最近一拍测距新鲜有效 */
    volatile float live_x_m;            /* 最近一次光流位置（世界/机体系，未投影） */
    volatile float live_y_m;
    volatile float live_vx_m_s;
    volatile float live_vy_m_s;
    volatile float live_psi_rad;        /* 空闲时用的杆方位角 */
    volatile uint32_t live_now_ms;      /* 最近一拍时刻与光流样本 token：开跑拒绝时的诊断 */
    volatile uint32_t live_flow_sample_ms;
    volatile uint8_t live_flow_raw_valid;
    volatile uint8_t live_height_raw_valid;
    char refusal[80];                   /* 带诊断的拒绝理由（Precheck 返回它） */

    /* 本轮（APP_SysIdXy_Begin 锁存） */
    APP_SysIdXyConfig run;
    DRV_SysIdExcitation spec;
    float mass_kg;
    float gravity_m_s2;
    float max_pct;
    float target_n;
    float psi_rad;
    float ux, uy;                       /* 沿槽方向 u = (−sin ψ, cos ψ) */
    float nx, ny;                       /* 杆轴方向 n = (cos ψ, sin ψ) */
    float x0_m, y0_m;                   /* 起点：开跑那一刻的光流位置（跨轮保留，THR? 行相对它） */

    /* 本拍投影量（相对起点）与参考 */
    uint8_t flow_fresh;                 /* 本拍光流/测距新鲜有效 */
    float p_u_m, p_perp_m, v_u_m_s, v_perp_m_s;
    float yaw_rad;                      /* 偏航监视：开跑清零，陀螺 z 积分（跨轮保留给 THR?） */
    uint32_t last_fresh_ms;             /* 最近一次光流新鲜的时刻（缺口计时） */
    uint8_t gap_timing;                 /* last_fresh_ms 有效 */
    uint32_t now_ms_last;               /* 本拍时刻（gate 算缺口用） */
    float p_sp_m;                       /* 沿 u 的位置参考（记录 height_sp；tilt 为 0） */
    float v_sp_m_s;                     /* 速度环参考，含前馈（记录 vz_sp；tilt 为 0） */
    float a_u_m_s2;                     /* 沿 u 的期望加速度（记录 az） */
    float v_inj_int_m;                  /* vel 注入的 ∫v_inj dt */
    float trim_a_m_s2;                  /* tilt：激励前按住开跑点的环输出，激励时作开环基准 */
    float bias_rad[2];                  /* 本拍目标倾角偏置 [0] 横滚 [1] 俯仰 */
    DRV_POSITION_CONTROL_State loop;
    uint32_t last_position_us;
    uint32_t last_velocity_us;
    uint16_t idle_pulse_us;
    uint16_t ramp_from_us;
    uint16_t pulse_us;
    uint8_t loop_started;
    uint8_t tilt_sat;                   /* 上一拍倾角被夹住：喂速度环的下游饱和，同生产 */

    /* 软停：soft_reason 非空 = 正在软停，走完报它 */
    const char *soft_reason;
    uint16_t soft_from_us;
    uint32_t soft_start_ms;
} SysIdXyContext;

#if defined(__GNUC__) && defined(__arm__)
__attribute__((section(".ram_d1_noinit"), aligned(32)))
#endif
static SysIdXyContext xy;
static uint8_t xy_initialised;

static const char *const xy_inject_names[APP_SYSID_XY_INJECT_COUNT] = { "tilt", "vel", "pos" };

static void xy_init(void)
{
    if (xy_initialised != 0U) {
        return;
    }
    xy_initialised = 1U;
    memset(&xy, 0, sizeof(xy));
    xy.config.inject = (uint8_t)APP_SYSID_XY_INJECT_TILT;
    xy.config.win_mm = APP_SYSID_XY_WIN_MM_DEFAULT;
    xy.config.mass_g = 0U;
    xy.ux = 0.0f;
    xy.uy = 1.0f;
    xy.nx = 1.0f;
}

static float xy_finite_or(float value, float fallback)
{
    return (isfinite(value) != 0) ? value : fallback;
}

static uint8_t xy_config_valid(const APP_SysIdXyConfig *config)
{
    return ((config != NULL) &&
            (config->inject < (uint8_t)APP_SYSID_XY_INJECT_COUNT) &&
            ((config->mass_g == 0U) ||
             ((config->mass_g >= APP_SYSID_XY_MASS_G_MIN) &&
              (config->mass_g <= APP_SYSID_XY_MASS_G_MAX))) &&
            (config->win_mm >= APP_SYSID_XY_WIN_MM_MIN) &&
            (config->win_mm <= APP_SYSID_XY_WIN_MM_MAX)) ? 1U : 0U;
}

/* m_run：配置了就用台架质量，0 取机体质量（airframe 只读，不改）。 */
static float xy_run_mass_kg(const APP_SysIdXyConfig *config)
{
    const DRV_Airframe_Params *airframe = DRV_Airframe_Get();

    if (config->mass_g != 0U) {
        return (float)config->mass_g / 1000.0f;
    }
    return (airframe != NULL) ? airframe->mass_kg : 0.0f;
}

static uint8_t xy_is_tilt(void)
{
    return (xy.run.inject == (uint8_t)APP_SYSID_XY_INJECT_TILT) ? 1U : 0U;
}

/* ------------------------------------------------------------------ 配置面 */

void APP_SysIdXy_GetConfig(APP_SysIdXyConfig *out)
{
    xy_init();
    if (out != NULL) {
        *out = xy.config;
    }
}

uint8_t APP_SysIdXy_SetConfig(const APP_SysIdXyConfig *config)
{
    xy_init();
    if (xy_config_valid(config) == 0U) {
        return 0U;
    }
    xy.config = *config;
    return 1U;
}

const char *APP_SysIdXy_InjectName(uint8_t inject)
{
    return (inject < (uint8_t)APP_SYSID_XY_INJECT_COUNT) ? xy_inject_names[inject] : "?";
}

uint8_t APP_SysIdXy_InjectFromName(const char *name, uint8_t *out)
{
    if ((name == NULL) || (out == NULL)) {
        return 0U;
    }
    for (uint8_t i = 0U; i < (uint8_t)APP_SYSID_XY_INJECT_COUNT; ++i) {
        if (strcmp(name, xy_inject_names[i]) == 0) {
            *out = i;
            return 1U;
        }
    }
    return 0U;
}

static const char *xy_control_name(uint8_t inject)
{
    return (inject == (uint8_t)APP_SYSID_XY_INJECT_TILT) ? "openloop" : "closed_loop";
}

void APP_SysIdXy_ReportConfig(void)
{
    xy_init();
    APP_Control_QueueText("SYSID XY inject=%s win_mm=%u mass_g=%u control=%s\r\n",
                          APP_SysIdXy_InjectName(xy.config.inject),
                          (unsigned int)xy.config.win_mm,
                          (unsigned int)xy.config.mass_g,
                          xy_control_name(xy.config.inject));
}

/* ------------------------------------------------------------------ 光流观测 */

static uint8_t xy_flow_fresh(const APP_SysIdObserve *obs)
{
    return ((obs->flow_valid != 0U) && (obs->flow_sample_ms != 0U) &&
            (isfinite(obs->flow_vel_m_s[0]) != 0) && (isfinite(obs->flow_vel_m_s[1]) != 0) &&
            (isfinite(obs->flow_pos_m[0]) != 0) && (isfinite(obs->flow_pos_m[1]) != 0) &&
            ((int32_t)(obs->now_ms - obs->flow_sample_ms) <=
             (int32_t)APP_SYSID_XY_FLOW_TIMEOUT_MS)) ? 1U : 0U;
}

static uint8_t xy_height_fresh(const APP_SysIdObserve *obs)
{
    return ((obs->height_valid != 0U) && (obs->height_sample_ms != 0U) &&
            (isfinite(obs->height_m) != 0) &&
            ((int32_t)(obs->now_ms - obs->height_sample_ms) <=
             (int32_t)APP_SYSID_XY_FLOW_TIMEOUT_MS)) ? 1U : 0U;
}

void APP_SysIdXy_Observe(const APP_SysIdObserve *obs, float azimuth_rad)
{
    xy_init();
    if (obs == NULL) {
        return;
    }
    xy.live_psi_rad = azimuth_rad;
    xy.live_now_ms = obs->now_ms;
    xy.live_flow_sample_ms = obs->flow_sample_ms;
    xy.live_flow_raw_valid = obs->flow_valid;
    xy.live_height_raw_valid = obs->height_valid;
    xy.live_height_ok = xy_height_fresh(obs);
    xy.live_flow_ok = xy_flow_fresh(obs);
    if (xy_flow_fresh(obs) != 0U) {
        xy.live_x_m = obs->flow_pos_m[0];
        xy.live_y_m = obs->flow_pos_m[1];
        xy.live_vx_m_s = obs->flow_vel_m_s[0];
        xy.live_vy_m_s = obs->flow_vel_m_s[1];
    }
}

const char *APP_SysIdXy_Precheck(float throttle_target_n, const DRV_SysIdExcitation *spec)
{
    static const char *const amp_refusal[APP_SYSID_XY_INJECT_COUNT] = {
        "amp over limit: inject=tilt amp<=0.10 rad",
        "amp over limit: inject=vel amp<=0.3 m/s",
        "amp over limit: inject=pos amp<=0.15 m",
    };
    static const float amp_max[APP_SYSID_XY_INJECT_COUNT] = {
        APP_SYSID_XY_TILT_AMP_MAX_RAD, APP_SYSID_XY_VEL_AMP_MAX_M_S, APP_SYSID_XY_POS_AMP_MAX_M,
    };
    const DRV_Airframe_Params *airframe;
    const float amplitude = (spec != NULL) ? spec->amplitude_rad_s : 0.0f;
    float mass_kg;

    xy_init();
    if (!(throttle_target_n > 0.0f)) {
        return "needs auto throttle (SYSID THROTTLE target_n>0)";
    }
    if (xy_config_valid(&xy.config) == 0U) {
        return "config invalid";
    }
    airframe = DRV_Airframe_Get();
    if ((airframe == NULL) || (isfinite(airframe->gravity_m_s2) == 0) ||
        !(airframe->gravity_m_s2 > 0.0f)) {
        return "airframe gravity invalid";
    }
    if (throttle_target_n > airframe->max_total_force_n) {
        return "target over max thrust";
    }
    if ((isfinite(amplitude) == 0) || !(amplitude > 0.0f) ||
        (amplitude > amp_max[xy.config.inject])) {
        return amp_refusal[xy.config.inject];
    }
    if ((xy.config.inject == (uint8_t)APP_SYSID_XY_INJECT_POS) &&
        (amplitude > ((float)xy.config.win_mm * 0.001f * XY_POS_AMP_WIN_FRACTION))) {
        return "amp over window: inject=pos amp<=0.7*win";
    }
    mass_kg = xy_run_mass_kg(&xy.config);
    if ((isfinite(mass_kg) == 0) || !(mass_kg > 0.0f)) {
        return "mass unknown (set SYSID XY mass_g= or airframe mass)";
    }
    if (xy.live_flow_ok == 0U) {
        /* vel_valid 取 flow_valid（= 光流速度有效且测距有效）；age_ms = -1 表示从未出过样本。 */
        const int32_t age_ms = (xy.live_flow_sample_ms != 0U) ?
                               (int32_t)(xy.live_now_ms - xy.live_flow_sample_ms) : -1;

        (void)snprintf(xy.refusal, sizeof(xy.refusal),
                       "flow invalid (vel_valid=%u height_valid=%u age_ms=%ld)",
                       (unsigned int)((xy.live_flow_raw_valid != 0U) ? 1U : 0U),
                       (unsigned int)((xy.live_height_raw_valid != 0U) ? 1U : 0U),
                       (long)age_ms);
        return xy.refusal;
    }
    if (xy.live_height_ok == 0U) {
        return "height invalid (TOF)";
    }
    return NULL;
}

/* 光流位置/速度投影到 u（相对起点）与 n。光流不新鲜时保持上一拍的值，由光流门中止。 */
static void xy_project(const APP_SysIdObserve *obs)
{
    xy.flow_fresh = xy_flow_fresh(obs);
    if (xy.flow_fresh != 0U) {
        const float dx = obs->flow_pos_m[0] - xy.x0_m;
        const float dy = obs->flow_pos_m[1] - xy.y0_m;

        xy.p_u_m = (dx * xy.ux) + (dy * xy.uy);
        xy.p_perp_m = (dx * xy.nx) + (dy * xy.ny);
        xy.v_u_m_s = (obs->flow_vel_m_s[0] * xy.ux) + (obs->flow_vel_m_s[1] * xy.uy);
        xy.v_perp_m_s = (obs->flow_vel_m_s[0] * xy.nx) + (obs->flow_vel_m_s[1] * xy.ny);
        xy.last_fresh_ms = obs->now_ms;
        xy.gap_timing = 1U;
    } else if (xy.gap_timing == 0U) {
        xy.last_fresh_ms = obs->now_ms;   /* 开跑后第一拍就无效：缺口从这一拍算起 */
        xy.gap_timing = 1U;
    }
}

static void xy_loop_reset(void)
{
    DRV_POSITION_CONTROL_ResetState(&xy.loop);
    xy.loop_started = 0U;
    xy.a_u_m_s2 = 0.0f;
    xy.v_sp_m_s = 0.0f;
    xy.last_position_us = 0U;
    xy.last_velocity_us = 0U;
}

void APP_SysIdXy_Begin(const DRV_SysIdExcitation *spec, float target_n, float max_pct,
                       float azimuth_rad)
{
    const DRV_Airframe_Params *airframe = DRV_Airframe_Get();

    xy_init();
    xy.run = xy.config;
    if (spec != NULL) {
        xy.spec = *spec;
    }
    xy.max_pct = max_pct;
    xy.target_n = target_n;
    xy.mass_kg = xy_run_mass_kg(&xy.run);
    xy.gravity_m_s2 = (airframe != NULL) ? airframe->gravity_m_s2 : 9.81f;
    xy.psi_rad = azimuth_rad;
    xy.ux = -sinf(azimuth_rad);
    xy.uy = cosf(azimuth_rad);
    xy.nx = cosf(azimuth_rad);
    xy.ny = sinf(azimuth_rad);
    xy.x0_m = xy.live_x_m;
    xy.y0_m = xy.live_y_m;
    xy.p_u_m = 0.0f;
    xy.p_perp_m = 0.0f;
    xy.v_u_m_s = 0.0f;
    xy.v_perp_m_s = 0.0f;
    xy.yaw_rad = 0.0f;
    xy.gap_timing = 0U;
    xy.last_fresh_ms = 0U;
    xy.p_sp_m = 0.0f;
    xy.v_inj_int_m = 0.0f;
    xy.trim_a_m_s2 = 0.0f;
    xy.bias_rad[0] = 0.0f;
    xy.bias_rad[1] = 0.0f;
    xy.idle_pulse_us = 0U;
    xy.ramp_from_us = 0U;
    xy.pulse_us = 0U;
    xy.tilt_sat = 0U;
    xy.soft_reason = NULL;
    xy_loop_reset();
}

void APP_SysIdXy_ReportStart(uint16_t run_id)
{
    xy_init();
    APP_Control_QueueText(
        "SYSID XYSTART run=%u xy_inject=%s xy_win_mm=%u xy_mass_g=%ld xy_psi_mrad=%ld "
        "xy_target_cn=%ld xy_x0_mm=%ld xy_y0_mm=%ld\r\n",
        (unsigned int)run_id, APP_SysIdXy_InjectName(xy.run.inject),
        (unsigned int)xy.run.win_mm, (long)lroundf(xy.mass_kg * 1000.0f),
        (long)lroundf(xy.psi_rad * 1000.0f), (long)lroundf(xy.target_n * 100.0f),
        (long)lroundf(xy.x0_m * 1000.0f), (long)lroundf(xy.y0_m * 1000.0f));
}

/* ------------------------------------------------------------------ 出力小工具 */

/* 与 app_sysid.c 的 sysid_target_pulse 同一口径：单桨 = 合推力/2 查表，再按最高油门 % 封顶。 */
static uint16_t xy_pulse_for(float total_n, uint8_t *capped)
{
    const float span = (float)(BSP_PWM_ESC_MAX_US - BSP_PWM_ESC_MIN_US);
    const uint16_t cap_us = (uint16_t)((float)BSP_PWM_ESC_MIN_US +
                                       xy.max_pct * span / 100.0f + 0.5f);
    const uint16_t pulse = DRV_COAX_CTRL_ThrustToMotorPulse(0.5f * total_n);
    const uint16_t limited = (pulse > cap_us) ? cap_us : pulse;
    const float achieved_n = DRV_COAX_CTRL_MotorPulseToTotalThrust(limited);

    *capped = ((pulse > cap_us) || (isfinite(achieved_n) == 0) ||
               ((achieved_n + XY_LUT_CAP_TOL_N) < total_n)) ? 1U : 0U;
    return limited;
}

static uint16_t xy_lerp_pulse(uint16_t from, uint16_t to, uint32_t t_ms, uint32_t span_ms)
{
    const int32_t delta = (int32_t)to - (int32_t)from;

    if ((span_ms == 0U) || (t_ms >= span_ms)) {
        return to;
    }
    return (uint16_t)((int32_t)from + (delta * (int32_t)t_ms) / (int32_t)span_ms);
}

static float xy_due_dt(uint32_t now_us, uint32_t last_us, uint32_t period_us, uint8_t first)
{
    if (first != 0U) {
        return (float)period_us * 1.0e-6f;
    }
    return ((uint32_t)(now_us - last_us) >= period_us) ?
           ((float)(uint32_t)(now_us - last_us) * 1.0e-6f) : 0.0f;
}

static void xy_finish_output(APP_SysIdPhase phase, uint16_t pulse, uint8_t capped,
                             APP_SysIdXyOutput *out)
{
    xy.pulse_us = pulse;
    out->phase = phase;
    out->motor_pulse_us = pulse;
    out->capped = capped;
    out->thrust_n = xy_finite_or(DRV_COAX_CTRL_MotorPulseToTotalThrust(pulse), 0.0f);
    out->tilt_bias_rad[0] = xy.bias_rad[0];
    out->tilt_bias_rad[1] = xy.bias_rad[1];
    /* 预升/回落段也保持姿态（同 ALT）：推力一上来机体就可能绕杆扭。 */
    if (out->thrust_n >= APP_SYSID_MIN_THRUST_N) {
        out->hold_attitude = 1U;
        out->check_thrust = 1U;
    }
}

/*
 * 沿 u 的期望水平加速度 → 目标倾角偏置（横滚、俯仰），生产同一公式（DRV_COAX_CTRL_TiltFromForce），
 * 各自夹在 coax.tilt_limit_rad 内。返回 1 = 有夹限（喂速度环的饱和反馈）。
 */
static uint8_t xy_tilt_from_accel(float accel_u_m_s2, float bias_rad[2])
{
    DRV_COAX_CTRL_Params params;
    const float mass_eff_kg = DRV_COAX_CTRL_EffectiveMassKg(xy.mass_kg);
    float force_n[3];
    float roll_rad = 0.0f;
    float pitch_rad = 0.0f;
    float limit_rad;
    uint8_t clamped = 0U;

    DRV_COAX_CTRL_GetParams(&params);
    limit_rad = fmaxf(params.tilt_limit_rad, 0.0f);
    force_n[0] = mass_eff_kg * accel_u_m_s2 * xy.ux;
    force_n[1] = mass_eff_kg * accel_u_m_s2 * xy.uy;
    force_n[2] = mass_eff_kg * xy.gravity_m_s2;
    DRV_COAX_CTRL_TiltFromForce(force_n, &roll_rad, &pitch_rad);
    if (fabsf(roll_rad) > limit_rad) {
        roll_rad = copysignf(limit_rad, roll_rad);
        clamped = 1U;
    }
    if (fabsf(pitch_rad) > limit_rad) {
        pitch_rad = copysignf(limit_rad, pitch_rad);
        clamped = 1U;
    }
    bias_rad[0] = roll_rad;
    bias_rad[1] = pitch_rad;
    return clamped;
}

/*
 * 生产位置/速度环沿 u 的一维（x 通道，其余轴输入清零）：位置 P（50 Hz）→ 速度 PI(D)（100 Hz），
 * 参数每拍从 DRV_COAX_CTRL_GetParams() 的 .position 取，SYSID PARAM/PARAM SET 改的值下一拍生效。
 * 返回 0 = 控制器拒绝输入（参数或测量非法），调用方中止。
 */
static uint8_t xy_run_loop(const APP_SysIdObserve *obs, float v_ff_m_s)
{
    const uint32_t position_period_us = (uint32_t)APP_CONTROL_SCHED_POSITION_PERIOD_US;
    const uint32_t velocity_period_us = (uint32_t)APP_CONTROL_SCHED_VELOCITY_PERIOD_US;
    const uint8_t first = (xy.loop_started == 0U) ? 1U : 0U;
    DRV_COAX_CTRL_Params params;
    float dt_s;

    if (xy.flow_fresh == 0U) {
        /* 光流缺口：环不更新（保持上一拍输出、积分不累加），并把节拍基准推到现在，
         * 恢复后第一拍按一个正常周期算，不会把缺口时长当 dt 一次积进去。 */
        if (first == 0U) {
            xy.last_position_us = obs->now_us;
            xy.last_velocity_us = obs->now_us;
        }
        return 1U;
    }
    if ((isfinite(xy.p_u_m) == 0) || (isfinite(xy.v_u_m_s) == 0)) {
        return 1U;
    }
    DRV_COAX_CTRL_GetParams(&params);

    dt_s = xy_due_dt(obs->now_us, xy.last_position_us, position_period_us, first);
    if (dt_s > 0.0f) {
        DRV_POSITION_CONTROL_PositionInput input;
        DRV_POSITION_CONTROL_PositionOutput output;

        memset(&input, 0, sizeof(input));
        input.position_sp_m[0] = xy.p_sp_m;
        input.position_meas_m[0] = xy.p_u_m;
        input.velocity_ff_m_s[0] = v_ff_m_s;
        input.dt_sec = dt_s;
        input.measurement_valid = 1U;
        DRV_POSITION_CONTROL_PositionStep(&params.position, &input, &output);
        if ((output.flags & (DRV_POSITION_CONTROL_FLAG_INPUT_INVALID |
                             DRV_POSITION_CONTROL_FLAG_MEAS_INVALID |
                             DRV_POSITION_CONTROL_FLAG_DT_INVALID)) != 0U) {
            return 0U;
        }
        xy.v_sp_m_s = output.velocity_sp_m_s[0];
        xy.last_position_us = obs->now_us;
    }

    dt_s = xy_due_dt(obs->now_us, xy.last_velocity_us, velocity_period_us, first);
    if (dt_s > 0.0f) {
        DRV_POSITION_CONTROL_VelocityInput input;
        DRV_POSITION_CONTROL_VelocityOutput output;

        memset(&input, 0, sizeof(input));
        input.velocity_sp_m_s[0] = xy.v_sp_m_s;
        input.velocity_meas_m_s[0] = xy.v_u_m_s;
        input.dt_sec = dt_s;
        input.measurement_valid = 1U;
        input.integrator_enable = 1U;
        input.integrator_reset = first;
        /* 同生产（drv_coax_ctrl 的 translation_saturation）：倾角顶住且还要往同方向加时冻结积分。 */
        input.downstream_saturation.horizontal_scale = 1.0f;
        input.downstream_saturation.tilt_saturated = xy.tilt_sat;
        input.downstream_saturation.pos_limit[0] =
            ((xy.tilt_sat != 0U) && (xy.a_u_m_s2 > 0.0f)) ? 1U : 0U;
        input.downstream_saturation.neg_limit[0] =
            ((xy.tilt_sat != 0U) && (xy.a_u_m_s2 < 0.0f)) ? 1U : 0U;
        DRV_POSITION_CONTROL_VelocityStep(&params.position, &xy.loop, &input, &output);
        if ((output.flags & DRV_POSITION_CONTROL_FLAG_INPUT_INVALID) != 0U) {
            return 0U;
        }
        xy.a_u_m_s2 = output.accel_sat_m_s2[0];
        xy.last_velocity_us = obs->now_us;
    }
    xy.loop_started = 1U;
    return 1U;
}

/* ------------------------------------------------------------------ 阶段推进与门 */

static APP_SysIdPhase xy_advance(const APP_SysIdObserve *obs, APP_SysIdPhase phase,
                                 uint32_t *phase_ms, APP_SysIdXyOutput *out)
{
    APP_SysIdPhase next = phase;

    switch (phase) {
    case APP_SYSID_PHASE_IDLE:
        xy.idle_pulse_us = obs->throttle_us;   /* 杆在最低 = 怠速：起升起点 */
        xy.pulse_us = obs->throttle_us;
        next = APP_SYSID_PHASE_RAMP_UP;
        break;
    case APP_SYSID_PHASE_RAMP_UP:
        if (*phase_ms >= APP_SYSID_XY_RAMP_UP_MS) {
            xy_loop_reset();
            next = APP_SYSID_PHASE_SETTLE;
        }
        break;
    case APP_SYSID_PHASE_SETTLE:
        if (*phase_ms >= APP_SYSID_XY_SETTLE_MS) {
            next = APP_SYSID_PHASE_PREROLL;
        }
        break;
    case APP_SYSID_PHASE_PREROLL:
        if (*phase_ms >= APP_SYSID_XY_PREROLL_MS) {
            xy.v_inj_int_m = 0.0f;
            next = APP_SYSID_PHASE_EXCITE;
        }
        break;
    case APP_SYSID_PHASE_EXCITE:
        break;   /* 剖面跑完才切，在出力那一步判 */
    case APP_SYSID_PHASE_RAMP_DOWN:
        if (*phase_ms >= APP_SYSID_XY_RAMP_DOWN_MS) {
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

/*
 * 出窗/超速/偏航/光流门：RAMP_DOWN 之外全程都判。
 * 出窗：d = hypot(p_u, p_perp)（相对开跑点，旋转不变——机体可在槽里偏航、杆方位也可能设错）
 * 加预测停车距离 s²/(2·a_coast) 超过窗口即软停：软停只清倾角，滑停靠动摩擦。
 */
static const char *xy_gate(APP_SysIdPhase phase)
{
    float d_m;
    float s_m_s;

    if (phase == APP_SYSID_PHASE_RAMP_DOWN) {
        return NULL;
    }
    if ((xy.flow_fresh == 0U) &&
        ((int32_t)(xy.now_ms_last - xy.last_fresh_ms) > XY_FLOW_GAP_TOL_MS)) {
        return "xy_flow_invalid";
    }
    d_m = hypotf(xy.p_u_m, xy.p_perp_m);
    s_m_s = hypotf(xy.v_u_m_s, xy.v_perp_m_s);
    if (s_m_s > XY_MAX_SPEED_M_S) {
        return "xy_overspeed";
    }
    if ((d_m + ((s_m_s * s_m_s) / (2.0f * XY_COAST_DECEL_M_S2))) >
        ((float)xy.run.win_mm * 0.001f)) {
        return "xy_window";
    }
    if (fabsf(xy.yaw_rad) > XY_YAW_LIMIT_RAD) {
        return "xy_yaw_limit";
    }
    return NULL;
}

/* ------------------------------------------------------------------ 软停 */

/* 推力还不到 0.5·m·g（RAMP_UP 早段）返回 0，调用方照原样当拍交还。 */
static uint8_t xy_soft_begin(const APP_SysIdObserve *obs, const char *reason)
{
    const float thrust_n = xy_finite_or(DRV_COAX_CTRL_MotorPulseToTotalThrust(xy.pulse_us), 0.0f);

    if (thrust_n < (APP_SYSID_XY_SOFT_SKIP_FRACTION * xy.mass_kg * xy.gravity_m_s2)) {
        return 0U;
    }
    xy.soft_reason = reason;
    xy.soft_from_us = xy.pulse_us;
    xy.soft_start_ms = obs->now_ms;
    return 1U;
}

/* 软停的一拍（阶段报 ramp_down）：倾角偏置清零，推力 1 s 线性降到遥控油门；走完报进入时的理由。 */
static void xy_soft_step(const APP_SysIdObserve *obs, APP_SysIdXyOutput *out)
{
    const uint32_t elapsed = (uint32_t)(obs->now_ms - xy.soft_start_ms);

    xy.bias_rad[0] = 0.0f;
    xy.bias_rad[1] = 0.0f;
    xy.p_sp_m = 0.0f;
    xy.v_sp_m_s = 0.0f;
    xy.a_u_m_s2 = 0.0f;
    if (elapsed >= APP_SYSID_XY_RAMP_DOWN_MS) {
        out->abort_reason = xy.soft_reason;
        return;
    }
    xy_finish_output(APP_SYSID_PHASE_RAMP_DOWN,
                     xy_lerp_pulse(xy.soft_from_us, obs->throttle_us, elapsed,
                                   APP_SYSID_XY_RAMP_DOWN_MS),
                     0U, out);
}

uint8_t APP_SysIdXy_BeginSoftStop(const APP_SysIdObserve *obs, const char *reason)
{
    xy_init();
    if ((obs == NULL) || (reason == NULL)) {
        return 0U;
    }
    return (xy.soft_reason != NULL) ? 1U : xy_soft_begin(obs, reason);
}

/* ------------------------------------------------------------------ 一拍 */

static void xy_step_run(const APP_SysIdObserve *obs, APP_SysIdPhase phase, uint32_t phase_ms,
                        float dt_s, APP_SysIdXyOutput *out)
{
    const float amplitude = xy.spec.amplitude_rad_s;
    float shape = 0.0f;
    float v_ff_m_s = 0.0f;
    uint16_t pulse;
    uint8_t capped = 0U;
    uint8_t loop = 0U;

    phase = xy_advance(obs, phase, &phase_ms, out);
    pulse = xy.pulse_us;
    if (out->abort_reason != NULL) {
        return;
    }
    if (phase == APP_SYSID_PHASE_EXCITE) {
        DRV_SysIdExcSample excitation;

        if (DRV_SysIdExcitation_Eval(&xy.spec, phase_ms, &excitation) != DRV_SYSID_EXC_OK) {
            out->abort_reason = "excitation";
            return;
        }
        if (excitation.finished != 0U) {
            xy.ramp_from_us = xy.pulse_us;
            phase = APP_SYSID_PHASE_RAMP_DOWN;
            phase_ms = 0U;
        } else {
            /* 现有剖面按单位幅值归一化（Validate 保证 amp > 0），再乘本注入类型的幅值。 */
            shape = xy_finite_or(excitation.omega_sp_rad_s / amplitude, 0.0f);
            shape = fmaxf(-1.0f, fminf(1.0f, shape));
        }
    }
    out->abort_reason = xy_gate(phase);
    if (out->abort_reason != NULL) {
        return;
    }

    /* 位置/速度环的输出在两次更新之间要保持，所以只有不跑环的阶段才清 v_sp / a_u。 */
    xy.p_sp_m = 0.0f;
    switch (phase) {
    case APP_SYSID_PHASE_RAMP_UP:
        pulse = xy_lerp_pulse(xy.idle_pulse_us, xy_pulse_for(xy.target_n, &capped),
                              phase_ms, APP_SYSID_XY_RAMP_UP_MS);
        xy.v_sp_m_s = 0.0f;
        xy.a_u_m_s2 = 0.0f;
        break;
    case APP_SYSID_PHASE_SETTLE:
    case APP_SYSID_PHASE_PREROLL:
        pulse = xy_pulse_for(xy.target_n, &capped);
        /*
         * tilt 也跑环，把机体按在开跑点：杆式台架上"水平"并不等于零水平力（悬挂/舵机配平/轻微坡度），
         * 浮起推力下摩擦又很小，不按住机体会在激励前自己溜走（2026-10-01 台架：静止段溜 8~11 cm，
         * 激励只剩微小响应）。按住所需的环输出就是要抵消的偏置，激励时锁成开环基准。
         * 只按速度（位置参考跟着实测走）：静摩擦卡住时位置误差会让积分一直涨，锁进基准就成了
         * 积分饱和而不是偏置；只按速度则一停住误差即为 0，积分不再增长。
         */
        if (xy_is_tilt()) {
            xy.p_sp_m = xy.p_u_m;
        }
        loop = 1U;
        break;
    case APP_SYSID_PHASE_EXCITE:
        pulse = xy_pulse_for(xy.target_n, &capped);
        if (xy.run.inject == (uint8_t)APP_SYSID_XY_INJECT_VEL) {
            v_ff_m_s = amplitude * shape;
            xy.v_inj_int_m += v_ff_m_s * dt_s;
            xy.p_sp_m = xy.v_inj_int_m;
            loop = 1U;
        } else if (xy.run.inject == (uint8_t)APP_SYSID_XY_INJECT_POS) {
            xy.p_sp_m = amplitude * shape;
            loop = 1U;
        } else {
            /* tilt：直接给沿 u 的倾角 θ，等效 a_u = g·tan θ；不跑位置/速度环。 */
            DRV_COAX_CTRL_Params params;
            float limit_rad;
            float theta_rad;

            DRV_COAX_CTRL_GetParams(&params);
            limit_rad = fmaxf(params.tilt_limit_rad, 0.0f);
            theta_rad = fmaxf(-limit_rad, fminf(limit_rad, amplitude * shape));
            xy.v_sp_m_s = 0.0f;
            /* 开环：激励前按住机体的环输出作基准，叠加 g·tan θ；激励期间环不再更新。 */
            xy.a_u_m_s2 = xy.trim_a_m_s2 + xy.gravity_m_s2 * tanf(theta_rad);
        }
        break;
    case APP_SYSID_PHASE_RAMP_DOWN:
        /* 回落终点同现有自动油门：本拍遥控器给的脉宽（杆在最低即怠速）。 */
        pulse = xy_lerp_pulse(xy.ramp_from_us, obs->throttle_us, phase_ms,
                              APP_SYSID_XY_RAMP_DOWN_MS);
        xy.v_sp_m_s = 0.0f;
        xy.a_u_m_s2 = 0.0f;
        break;
    default:
        out->abort_reason = "phase";
        return;
    }

    if (loop != 0U) {
        if (xy_run_loop(obs, v_ff_m_s) == 0U) {
            out->abort_reason = "controller";
            return;
        }
        if (xy_is_tilt() && (phase == APP_SYSID_PHASE_PREROLL)) {
            xy.trim_a_m_s2 = xy.a_u_m_s2;   /* 预备段末拍的值进入激励 */
        }
    }
    if (isfinite(xy.a_u_m_s2) == 0) {
        out->abort_reason = "controller";
        return;
    }
    if ((phase == APP_SYSID_PHASE_RAMP_UP) || (phase == APP_SYSID_PHASE_RAMP_DOWN)) {
        xy.bias_rad[0] = 0.0f;
        xy.bias_rad[1] = 0.0f;
        xy.tilt_sat = 0U;
    } else {
        xy.tilt_sat = xy_tilt_from_accel(xy.a_u_m_s2, xy.bias_rad);
    }
    xy_finish_output(phase, pulse, capped, out);
}

void APP_SysIdXy_Step(const APP_SysIdObserve *obs, APP_SysIdPhase phase, uint32_t phase_ms,
                      float dt_s, APP_SysIdXyOutput *out)
{
    xy_init();
    if ((obs == NULL) || (out == NULL)) {
        return;
    }
    memset(out, 0, sizeof(*out));
    xy.now_ms_last = obs->now_ms;
    xy_project(obs);
    if ((isfinite(obs->gyro_rad_s[2]) != 0) && (isfinite(dt_s) != 0) && (dt_s > 0.0f)) {
        xy.yaw_rad += obs->gyro_rad_s[2] * dt_s;   /* 只监视：XY 偏航开环，没有执行通路 */
    }
    if (xy.soft_reason == NULL) {
        xy_step_run(obs, phase, phase_ms, dt_s, out);
        if (out->abort_reason == NULL) {
            return;
        }
        if (xy_soft_begin(obs, out->abort_reason) == 0U) {
            return;
        }
        memset(out, 0, sizeof(*out));
    }
    xy_soft_step(obs, out);
}

/* ------------------------------------------------------------------ 记录与状态行 */

void APP_SysIdXy_FillSample(DRV_SysIdSample *sample, const APP_SysIdObserve *obs)
{
    const uint8_t valid = (xy.flow_fresh != 0U) ? 1U : 0U;

    xy_init();
    if ((sample == NULL) || (obs == NULL)) {
        return;
    }
    sample->height_m = (valid != 0U) ? xy.p_u_m : 0.0f;
    sample->height_raw_m = (valid != 0U) ? xy.p_perp_m : 0.0f;
    sample->height_sp_m = xy.p_sp_m;
    sample->vz_m_s = (valid != 0U) ? xy.v_u_m_s : 0.0f;
    sample->vz_sp_m_s = xy.v_sp_m_s;
    sample->az_m_s2 = xy.a_u_m_s2;
    sample->vbat_v = obs->vbat_v;
}

void APP_SysIdXy_GetLive(float *pos_m, float *vel_m_s, uint8_t *ok)
{
    /* 台架设置在跑的时候改不了，空闲时取当前杆方位角：都是 live_psi_rad。 */
    const float psi = xy.live_psi_rad;
    const float ux = -sinf(psi);
    const float uy = cosf(psi);

    xy_init();
    if (pos_m != NULL) {
        *pos_m = ((xy.live_x_m - xy.x0_m) * ux) + ((xy.live_y_m - xy.y0_m) * uy);
    }
    if (vel_m_s != NULL) {
        *vel_m_s = (xy.live_vx_m_s * ux) + (xy.live_vy_m_s * uy);
    }
    if (ok != NULL) {
        *ok = ((xy.live_flow_ok != 0U) && (xy.live_height_ok != 0U)) ? 1U : 0U;
    }
}

float APP_SysIdXy_GetYawRad(void)
{
    xy_init();
    return xy.yaw_rad;
}

void APP_SysIdXy_GetTiltBias(float *roll_rad, float *pitch_rad, float *accel_u_m_s2)
{
    xy_init();
    if (roll_rad != NULL) {
        *roll_rad = xy.bias_rad[0];
    }
    if (pitch_rad != NULL) {
        *pitch_rad = xy.bias_rad[1];
    }
    if (accel_u_m_s2 != NULL) {
        *accel_u_m_s2 = xy.a_u_m_s2;
    }
}

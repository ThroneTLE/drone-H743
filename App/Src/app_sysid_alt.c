/*
 * app_sysid_alt.c —— 光杆台架高度辨识（SYSID MODE ALT）：离地/滑落阈值实验与闭环候选验证。
 *
 * 设计与口径写在 App/Inc/app_sysid_alt.h。安全门、姿态链、采样网格、油门出口都是
 * app_sysid.c 已有的那一套，本文件只算"这一拍在哪个阶段、电机给多少、参考是多少"。
 *
 * 状态放 AXI SRAM（NOLOAD，首次调用时显式清）：DTCM 已被辨识采样环占去一大块。
 * 配置只在命令任务里改（且只在空闲时），本轮用的那份在开跑时锁存；控制拍只写
 * 运行状态与"最近测高"几项，命令任务只读后者。
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
    volatile float live_height_m;       /* 最近一次**有效**的测高（单样本） */
    volatile uint32_t live_sample_ms;   /* 独立 Z 样本 token，不能以控制拍续命 */
    /* 最近 8 个新测距样本（按 token 去重）的环形缓冲：所有高度判据都用它的平均。 */
    float ring_m[APP_SYSID_ALT_AVG_SAMPLES];
    uint8_t ring_head;
    volatile uint8_t ring_count;
    uint32_t ring_token_ms;
    /* 每个新样本进来后的平均值与 token（再往前 8 个）：算平均高度的斜率 = 上下运动速度。 */
    float avg_hist_m[APP_SYSID_ALT_AVG_SAMPLES + 1U];
    uint32_t avg_tok_ms[APP_SYSID_ALT_AVG_SAMPLES + 1U];
    uint8_t avg_head;
    uint8_t avg_count;

    /* 本轮（APP_SysIdAlt_Begin 锁存） */
    APP_SysIdAltConfig run;
    DRV_SysIdExcitation spec;
    float mass_kg;
    float gravity_m_s2;
    float max_pct;
    float target_n;          /* break：离地搜索上限，不是悬停真值 */
    float h0_m;              /* 开跑时的平均高度 */
    float bottom_m;          /* 物理槽底/顶：本轮绝对 TOF 读数，不从 h0 重置 */
    float top_m;
    float upper_m;           /* vel/pos：min(bottom + lift + win, top − 40 mm)；break：top − 10 mm */
    float z_hold_m;
    float z_land_m;
    float z_ref_m;           /* 爬升/回落段的斜坡参考 */
    float z_sp_m;            /* 本拍高度参考（记录 height_sp；break 为 0 = 不适用） */
    float vz_sp_m_s;         /* 位置环给速度环的参考，含前馈（记录 vz_sp） */
    float v_inj_int_m;       /* vel 注入的 ∫v_inj dt */
    float accel_cmd_m_s2;    /* 速度环最近一次输出 a_z，两次速度环更新之间保持 */
    DRV_POSITION_CONTROL_State loop;
    uint32_t last_position_us;
    uint32_t last_velocity_us;
    uint32_t last_height_ms; /* 最近一个新鲜测距样本的 token */
    uint16_t idle_pulse_us;
    uint16_t ramp_from_us;
    uint16_t pulse_us;
    uint8_t loop_started;    /* 高度环已接管；0 = 下一次更新先复位积分 */
    uint8_t capped;          /* 上一拍被封顶：喂速度环的下游饱和，同生产 */

    /* break 阈值实验 */
    float rate_n_s;          /* 慢升/慢降速率 = EXC amp */
    float f_start_n;         /* 0.9·m_run·g */
    float f_floor_n;         /* 0.8·m_run·g */
    float cmd_n;             /* 本阶段当前合推力指令 */
    float last_cmd_n;        /* 上一拍已下发的指令：观测反映的是它 */
    float f_hold_n;          /* 制停停住时的指令 = 慢降起点 */
    float f_det_n;           /* 离地判定时的指令 F_det */
    float stop_first_n;      /* 制停第一档降多少（离地时按上升速度估） */
    float h_base_m;          /* 进慢升时的平均高度（槽底基线） */
    float h_stuck_m;         /* 制停停住时的平均高度 */
    float hist_h_m[2];       /* 制停：100 ms 前、200 ms 前的平均高度 */
    float fast_h_m;
    float land_h_m;
    uint32_t ceil_ms;        /* 慢升指令到搜索上限的时刻，0 = 还没到 */
    uint32_t last_step_ms;
    uint32_t hist_ms;
    uint32_t fast_ms;
    uint32_t land_ms;
    uint32_t brake_until_ms;
    uint32_t brake_step_ms;  /* 制停中上一次 height_brake 降档的时刻 */
    uint8_t settle_first;    /* 制停第一拍：保持离地判定时的指令，PHASE 行报的就是它 */
    uint8_t brake_stepped;   /* brake_step_ms 有效 */

    /* 软着陆：soft_reason 非空 = 正在软着陆，结束时报它 */
    const char *soft_reason;
    float soft_cmd_n;
    float soft_cap_n;        /* 进入时的推力：往回加不超过它 */
    uint32_t soft_start_ms;
    uint32_t soft_tick_ms;
    uint32_t soft_final_ms;  /* 转入线性降到怠速的时刻 */
    uint32_t soft_span_ms;   /* 0 = 还在按高度慢降 */
    float thrust_avg_n;      /* 最近约 0.5 s 下发推力的平均（0 = 还没有）：软着陆的起点 */
} SysIdAltContext;

#if defined(__GNUC__) && defined(__arm__)
__attribute__((section(".ram_d1_noinit"), aligned(32)))
#endif
static SysIdAltContext alt;
static uint8_t alt_initialised;

static const char *const alt_inject_names[APP_SYSID_ALT_INJECT_COUNT] = { "break", "vel", "pos" };

static void alt_init(void)
{
    if (alt_initialised != 0U) {
        return;
    }
    alt_initialised = 1U;
    memset(&alt, 0, sizeof(alt));
    alt.config.inject = (uint8_t)APP_SYSID_ALT_INJECT_BREAK;
    alt.config.mass_g = 0U;
    alt.config.win_mm = APP_SYSID_ALT_WIN_MM_DEFAULT;
    alt.config.lift_mm = APP_SYSID_ALT_LIFT_MM_DEFAULT;
}

static uint8_t alt_is_break(void)
{
    return (alt.run.inject == (uint8_t)APP_SYSID_ALT_INJECT_BREAK) ? 1U : 0U;
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
            (config->lift_mm <= APP_SYSID_ALT_LIFT_MM_MAX) &&
            (config->bottom_mm <= APP_SYSID_ALT_ENDPOINT_MAX_MM) &&
            (config->top_mm <= APP_SYSID_ALT_ENDPOINT_MAX_MM)) ? 1U : 0U;
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

/* 最近若干新测距样本的平均（没有样本时返回单样本值）。 */
static float alt_avg_m(void)
{
    float sum = 0.0f;
    const uint8_t count = alt.ring_count;

    if (count == 0U) {
        return alt.live_height_m;
    }
    for (uint8_t i = 0U; i < count; ++i) {
        sum += alt.ring_m[i];
    }
    return sum / (float)count;
}

/* 8 个样本攒满才用平均值做判断：断档清空后头几个样本太吵（单样本 σ 3–8 mm）。 */
static uint8_t alt_avg_ready(void)
{
    return (alt.ring_count >= APP_SYSID_ALT_AVG_SAMPLES) ? 1U : 0U;
}

/* 平均高度的斜率 [m/s]（向上为正）：最新平均与 8 个样本之前的平均之差；样本不够时 0。 */
static float alt_rate_m_s(void)
{
    const uint8_t n = APP_SYSID_ALT_AVG_SAMPLES + 1U;
    uint8_t newest;
    uint8_t oldest;
    uint32_t dt_ms;

    if (alt.avg_count < n) {
        return 0.0f;
    }
    newest = (uint8_t)((alt.avg_head + n - 1U) % n);
    oldest = alt.avg_head;
    dt_ms = (uint32_t)(alt.avg_tok_ms[newest] - alt.avg_tok_ms[oldest]);
    if (dt_ms == 0U) {
        return 0.0f;
    }
    return (alt.avg_hist_m[newest] - alt.avg_hist_m[oldest]) * 1000.0f / (float)dt_ms;
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

static const char *alt_control_name(uint8_t inject)
{
    return (inject == (uint8_t)APP_SYSID_ALT_INJECT_BREAK) ? "breakaway" : "closed_loop";
}

void APP_SysIdAlt_ReportConfig(void)
{
    alt_init();
    APP_Control_QueueText("SYSID ALT inject=%s mass_g=%u win_mm=%u lift_mm=%u bottom_mm=%u "
                          "top_mm=%u control=%s\r\n",
                          APP_SysIdAlt_InjectName(alt.config.inject),
                          (unsigned int)alt.config.mass_g,
                          (unsigned int)alt.config.win_mm,
                          (unsigned int)alt.config.lift_mm,
                          (unsigned int)alt.config.bottom_mm,
                          (unsigned int)alt.config.top_mm,
                          alt_control_name(alt.config.inject));
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
    if ((obs->height_valid != 0U) && (obs->height_sample_ms != 0U) &&
        (isfinite(obs->height_m) != 0) &&
        ((uint32_t)(obs->now_ms - obs->height_sample_ms) <= APP_SYSID_ALT_HEIGHT_TIMEOUT_MS)) {
        alt.live_height_m = obs->height_m;
        alt.live_sample_ms = obs->height_sample_ms;
        alt.live_valid = 1U;
        if ((alt.ring_count != 0U) &&
            ((uint32_t)(obs->height_sample_ms - alt.ring_token_ms) > APP_SYSID_ALT_HEIGHT_TIMEOUT_MS)) {
            alt.ring_count = 0U;     /* 两个样本隔了 100 ms 以上（或时间倒退）：旧样本不再算 */
            alt.ring_head = 0U;
            alt.avg_count = 0U;
            alt.avg_head = 0U;
        }
        if ((alt.ring_count == 0U) || (obs->height_sample_ms != alt.ring_token_ms)) {
            alt.ring_m[alt.ring_head] = obs->height_m;
            alt.ring_head = (uint8_t)((alt.ring_head + 1U) % APP_SYSID_ALT_AVG_SAMPLES);
            if (alt.ring_count < APP_SYSID_ALT_AVG_SAMPLES) {
                alt.ring_count++;
            }
            alt.ring_token_ms = obs->height_sample_ms;
            alt.avg_hist_m[alt.avg_head] = alt_avg_m();
            alt.avg_tok_ms[alt.avg_head] = obs->height_sample_ms;
            alt.avg_head = (uint8_t)((alt.avg_head + 1U) % (APP_SYSID_ALT_AVG_SAMPLES + 1U));
            if (alt.avg_count < (APP_SYSID_ALT_AVG_SAMPLES + 1U)) {
                alt.avg_count++;
            }
        }
    } else {
        alt.live_valid = 0U;
        if ((uint32_t)(obs->now_ms - alt.ring_token_ms) > APP_SYSID_ALT_HEIGHT_TIMEOUT_MS) {
            alt.ring_count = 0U;     /* 断过就重新攒，不把很久以前的样本混进平均 */
            alt.ring_head = 0U;
            alt.avg_count = 0U;
            alt.avg_head = 0U;
        }
    }
}

const char *APP_SysIdAlt_Precheck(float throttle_target_n, const DRV_SysIdExcitation *spec)
{
    static const float amp_max[APP_SYSID_ALT_INJECT_COUNT] = {
        APP_SYSID_ALT_BREAK_RATE_MAX_N_S, APP_SYSID_ALT_VEL_AMP_MAX_M_S,
        APP_SYSID_ALT_POS_AMP_MAX_M,
    };
    static const char *const amp_refusal[APP_SYSID_ALT_INJECT_COUNT] = {
        "amp over limit: inject=break amp<=1 N/s",
        "amp over limit: inject=vel amp<=0.3 m/s",
        "amp over limit: inject=pos amp<=0.15 m",
    };
    float mass_kg;
    float weight_n;
    const DRV_Airframe_Params *airframe;
    const float amplitude = (spec != NULL) ? spec->amplitude_rad_s : 0.0f;

    alt_init();
    if (!(throttle_target_n > 0.0f)) {
        /* 起升、找阈值、高度环、回落都要本模块给油门；遥控器手动给油门没法跑这一套。 */
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
    airframe = DRV_Airframe_Get();
    if ((airframe == NULL) || (isfinite(airframe->gravity_m_s2) == 0) ||
        !(airframe->gravity_m_s2 > 0.0f)) {
        return "airframe gravity invalid";
    }
    weight_n = mass_kg * airframe->gravity_m_s2;
    if (alt.config.inject == (uint8_t)APP_SYSID_ALT_INJECT_BREAK) {
        /* 上限低于重力一定离不了地；高过重力 5 N 则一旦离地冲得太猛。 */
        if (throttle_target_n < weight_n) {
            return "break target below run weight";
        }
        if (throttle_target_n > (weight_n + APP_SYSID_ALT_BREAK_TARGET_EXCESS_MAX_N)) {
            return "break target over run weight + 5 N";
        }
    }
    if ((alt.config.bottom_mm == 0U) || (alt.config.top_mm == 0U)) {
        return "slot_endpoints_missing";
    }
    if ((alt.config.top_mm <= alt.config.bottom_mm) ||
        (((uint32_t)alt.config.top_mm - alt.config.bottom_mm) < APP_SYSID_ALT_TRAVEL_MIN_MM) ||
        (((uint32_t)alt.config.top_mm - alt.config.bottom_mm) > APP_SYSID_ALT_TRAVEL_MAX_MM)) {
        return "slot_travel";
    }
    if ((alt.live_valid == 0U) || (alt.ring_count < APP_SYSID_ALT_AVG_SAMPLES)) {
        return "height invalid (TOF)";
    }
    if (fabsf(alt_avg_m() - ((float)alt.config.bottom_mm * 0.001f)) >
        APP_SYSID_ALT_START_BOTTOM_TOL_M) {
        return "slot_start";
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

void APP_SysIdAlt_Begin(const DRV_SysIdExcitation *spec, float target_n, float max_pct)
{
    const DRV_Airframe_Params *airframe = DRV_Airframe_Get();
    float weight_n;

    alt_init();
    alt.run = alt.config;
    if (spec != NULL) {
        alt.spec = *spec;
    }
    alt.max_pct = max_pct;
    alt.target_n = target_n;
    alt.mass_kg = alt_run_mass_kg(&alt.run);
    alt.gravity_m_s2 = (airframe != NULL) ? airframe->gravity_m_s2 : 9.81f;
    alt.h0_m = alt_avg_m();
    alt.bottom_m = (float)alt.run.bottom_mm * 0.001f;
    alt.top_m = (float)alt.run.top_mm * 0.001f;
    alt.z_hold_m = alt.bottom_m + ((float)alt.run.lift_mm * 0.001f);
    alt.upper_m = alt_is_break() ? (alt.top_m - APP_SYSID_ALT_BREAK_TOP_MARGIN_M) :
                  fminf(alt.z_hold_m + ((float)alt.run.win_mm * 0.001f),
                        alt.top_m - APP_SYSID_ALT_TOP_MARGIN_M);
    alt.z_land_m = alt.bottom_m + APP_SYSID_ALT_LAND_ABOVE_M;
    alt.z_ref_m = alt.h0_m;
    alt.z_sp_m = alt_is_break() ? 0.0f : alt.h0_m;
    alt.v_inj_int_m = 0.0f;
    alt.idle_pulse_us = 0U;
    alt.ramp_from_us = 0U;
    alt.pulse_us = 0U;
    alt.last_height_ms = alt.live_sample_ms;
    alt.capped = 0U;
    alt_loop_reset();

    weight_n = alt.mass_kg * alt.gravity_m_s2;
    alt.rate_n_s = alt.spec.amplitude_rad_s;
    alt.f_start_n = APP_SYSID_ALT_RAMP_UP_FRACTION * weight_n;
    alt.f_floor_n = APP_SYSID_ALT_BREAK_FLOOR_FRACTION * weight_n;
    alt.cmd_n = 0.0f;
    alt.last_cmd_n = 0.0f;
    alt.h_base_m = alt.h0_m;
    alt.h_stuck_m = alt.h0_m;
    alt.fast_h_m = alt.h0_m;
    alt.land_h_m = alt.h0_m;
    alt.ceil_ms = 0U;
    alt.last_step_ms = 0U;
    alt.hist_ms = 0U;
    alt.fast_ms = 0U;
    alt.land_ms = 0U;
    alt.brake_until_ms = 0U;
    alt.brake_step_ms = 0U;
    alt.settle_first = 0U;
    alt.brake_stepped = 0U;
    alt.soft_reason = NULL;
    alt.soft_span_ms = 0U;
    alt.thrust_avg_n = 0.0f;
}

void APP_SysIdAlt_ReportStart(uint16_t run_id)
{
    alt_init();
    APP_Control_QueueText(
        "SYSID ALTSTART run=%u alt_inject=%s alt_mass_g=%ld alt_win_mm=%u alt_lift_mm=%u "
        "alt_h0_mm=%ld alt_control=%s alt_target_cn=%ld alt_bottom_mm=%u alt_top_mm=%u\r\n",
        (unsigned int)run_id, APP_SysIdAlt_InjectName(alt.run.inject),
        (long)lroundf(alt.mass_kg * 1000.0f), (unsigned int)alt.run.win_mm,
        (unsigned int)alt.run.lift_mm, (long)lroundf(alt.h0_m * 1000.0f),
        alt_control_name(alt.run.inject), (long)lroundf(alt.target_n * 100.0f),
        (unsigned int)alt.run.bottom_mm, (unsigned int)alt.run.top_mm);
}

/*
 * 与 app_sysid.c 的 sysid_target_pulse 同一口径：单桨 = 合推力/2 查表，再按最高油门 % 封顶。
 * 查补表到端点/平台也会削顶：读回的推力比要的少 0.1 N 以上同样算封顶，免得记录误报未饱和。
 */
static uint16_t alt_pulse_for(float total_n, uint8_t *capped)
{
    const float span = (float)(BSP_PWM_ESC_MAX_US - BSP_PWM_ESC_MIN_US);
    const uint16_t cap_us = (uint16_t)((float)BSP_PWM_ESC_MIN_US +
                                       alt.max_pct * span / 100.0f + 0.5f);
    const uint16_t pulse = DRV_COAX_CTRL_ThrustToMotorPulse(0.5f * total_n);
    const uint16_t limited = (pulse > cap_us) ? cap_us : pulse;
    const float achieved_n = DRV_COAX_CTRL_MotorPulseToTotalThrust(limited);

    *capped = ((pulse > cap_us) || (isfinite(achieved_n) == 0) ||
               ((achieved_n + APP_SYSID_ALT_LUT_CAP_TOL_N) < total_n)) ? 1U : 0U;
    return limited;
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

static uint8_t alt_height_fresh(const APP_SysIdObserve *obs)
{
    return ((obs->height_valid != 0U) && (obs->height_sample_ms != 0U) &&
            (isfinite(obs->height_m) != 0) &&
            (isfinite(obs->vz_m_s) != 0) &&
            ((uint32_t)(obs->now_ms - obs->height_sample_ms) <=
             APP_SYSID_ALT_HEIGHT_TIMEOUT_MS)) ? 1U : 0U;
}

/* RAMP_DOWN 走完：平均高度回到槽底以上 20 mm 以内才算完成（所有注入）。 */
static void alt_check_landing(uint32_t phase_ms, APP_SysIdAltOutput *out)
{
    if ((phase_ms < APP_SYSID_ALT_RAMP_DOWN_MS) || (alt_avg_ready() == 0U)) {
        return;                      /* 平均值没攒满就再等几拍（已在怠速；断档由测高门管） */
    }
    if (alt_avg_m() <= (alt.bottom_m + APP_SYSID_ALT_LAND_CONFIRM_M)) {
        out->finished = 1U;
    } else {
        out->abort_reason = "landing_unconfirmed";
    }
}

/* ------------------------------------------------------------------ break：找阈值 */

static void alt_break_enter_settle(uint32_t now_ms)
{
    /* 离地时多出的净推力 ≈ m·v²/(2z)（从静止匀加速升了 z、速度 v）；第一档降它 + 余量。 */
    const float v = fmaxf(alt_rate_m_s(), 0.0f);
    const float z = fmaxf(alt_avg_m() - alt.h_base_m, APP_SYSID_ALT_BREAK_MOVE_M);
    const float excess_n = alt.mass_kg * v * v / (2.0f * z);

    alt.stop_first_n = fminf(fmaxf(excess_n + APP_SYSID_ALT_BREAK_STOP_MARGIN_N,
                                   APP_SYSID_ALT_BREAK_STOP_FIRST_MIN_N),
                             APP_SYSID_ALT_BREAK_STOP_FIRST_MAX_N);
    alt.cmd_n = alt.last_cmd_n;     /* 离地判定的观测来自上一拍已发的指令 */
    alt.f_det_n = alt.cmd_n;
    alt.settle_first = 1U;
    alt.last_step_ms = now_ms;
    alt.hist_ms = now_ms;
    alt.hist_h_m[0] = alt_avg_m();
    alt.hist_h_m[1] = alt.hist_h_m[0];
}

/* 制停中 height_brake：预测会冲过上沿，立刻再降一档（不中止）。第一拍保持 F_det（PHASE 行报它），不动。 */
static void alt_break_brake(uint32_t now_ms)
{
    if ((alt.settle_first != 0U) ||
        ((alt.brake_stepped != 0U) &&
         ((uint32_t)(now_ms - alt.brake_step_ms) < APP_SYSID_ALT_BREAK_BRAKE_STEP_MS))) {
        return;
    }
    alt.cmd_n = fmaxf(alt.cmd_n - APP_SYSID_ALT_BREAK_STOP_STEP_N, fminf(alt.f_floor_n, alt.cmd_n));
    alt.brake_step_ms = now_ms;
    alt.brake_stepped = 1U;
    alt.last_step_ms = now_ms;
}

static APP_SysIdPhase alt_advance_break(const APP_SysIdObserve *obs, APP_SysIdPhase phase,
                                        uint32_t *phase_ms, APP_SysIdAltOutput *out)
{
    APP_SysIdPhase next = phase;
    const uint32_t now = obs->now_ms;
    const float h = alt_avg_m();

    if ((alt_avg_ready() == 0U) && (phase != APP_SYSID_PHASE_IDLE) &&
        (phase != APP_SYSID_PHASE_RAMP_DOWN)) {
        /* 平均值还没攒满：不判离地/停住/滑落/落地，保持本阶段（断档由测高门兜底）。 */
        if ((phase == APP_SYSID_PHASE_SETTLE) && (*phase_ms >= APP_SYSID_ALT_BREAK_STOP_MAX_MS)) {
            out->abort_reason = "stop_timeout";
        } else if ((phase == APP_SYSID_PHASE_DESCEND) &&
                   (*phase_ms >= APP_SYSID_ALT_BREAK_DESCEND_MAX_MS)) {
            out->abort_reason = "descent_timeout";
        }
        return phase;
    }
    switch (phase) {
    case APP_SYSID_PHASE_IDLE:
        alt.idle_pulse_us = obs->throttle_us;
        alt.pulse_us = obs->throttle_us;
        alt.last_height_ms = obs->height_sample_ms;
        next = APP_SYSID_PHASE_RAMP_UP;
        break;
    case APP_SYSID_PHASE_RAMP_UP:
        /* 预升段不判离地：2026-09-30 实测推力升到 1.4～6 N（重力 11.6 N）时测距就涨 10～25 mm、
         * 推力一撤又回来——杆的弹性形变或桨下洗/振动干扰，不是离地。基线等进慢升时再取。 */
        if (*phase_ms >= APP_SYSID_ALT_RAMP_UP_MS) {
            alt.h_base_m = h;
            alt.ceil_ms = 0U;
            next = APP_SYSID_PHASE_CLIMB;
        }
        break;
    case APP_SYSID_PHASE_CLIMB:
        /* 离地 = 比慢升起点高 10 mm 且正以 >0.03 m/s 上升：杆的弹性形变跟着推力慢慢变（约 1 mm/s），
         * 测距漂移来回晃，两样同时满足才是真的动起来了。 */
        if ((h >= (alt.h_base_m + APP_SYSID_ALT_BREAK_MOVE_M)) &&
            (alt_rate_m_s() >= APP_SYSID_ALT_BREAK_MOVE_RATE_M_S)) {
            alt_break_enter_settle(now);
            next = APP_SYSID_PHASE_SETTLE;
        } else if ((alt.ceil_ms != 0U) &&
                   ((uint32_t)(now - alt.ceil_ms) >= APP_SYSID_ALT_BREAK_CEIL_HOLD_MS)) {
            out->abort_reason = "no_liftoff";
        }
        break;
    case APP_SYSID_PHASE_SETTLE:
        if (*phase_ms >= APP_SYSID_ALT_BREAK_STOP_MAX_MS) {
            out->abort_reason = "stop_timeout";
            break;
        }
        if ((uint32_t)(now - alt.hist_ms) >= 100U) {
            const float rise = h - alt.hist_h_m[0];                 /* 这 100 ms */
            const float rise_before = alt.hist_h_m[0] - alt.hist_h_m[1];
            const float net_200ms = h - alt.hist_h_m[1];
            const uint32_t since_step = (uint32_t)(now - alt.last_step_ms);

            alt.hist_h_m[1] = alt.hist_h_m[0];
            alt.hist_h_m[0] = h;
            alt.hist_ms = now;
            if ((rise > APP_SYSID_ALT_BREAK_RISE_M) &&
                (rise >= (rise_before - APP_SYSID_ALT_BREAK_DECEL_SLACK_M)) &&
                (since_step >= APP_SYSID_ALT_BREAK_CHECK_MS)) {
                /* 此刻还在往上走：再降一档；到下限还在升说明推力表/配置离谱，停下交还。 */
                if ((alt.cmd_n - APP_SYSID_ALT_BREAK_STOP_STEP_N) < alt.f_floor_n) {
                    out->abort_reason = "stop_failed";
                    break;
                }
                alt.cmd_n -= APP_SYSID_ALT_BREAK_STOP_STEP_N;
                alt.last_step_ms = now;
            } else if ((rise < -APP_SYSID_ALT_BREAK_FAST_DOWN_M) &&
                       (since_step >= APP_SYSID_ALT_BREAK_CHECK_MS) &&
                       (h > (alt.h_base_m + APP_SYSID_ALT_BREAK_AT_BOTTOM_M))) {
                /* 降过头、往回掉得快（>0.2 m/s）：加回一档接住，停在半路再慢降找滑落；不回到离地那一档。 */
                alt.cmd_n = fminf(alt.cmd_n + APP_SYSID_ALT_BREAK_STOP_STEP_N,
                                  alt.f_det_n - APP_SYSID_ALT_BREAK_STOP_FIRST_MIN_N);
                alt.last_step_ms = now;
            } else if ((fabsf(net_200ms) <= APP_SYSID_ALT_BREAK_STILL_M) &&
                       (since_step >= APP_SYSID_ALT_BREAK_QUIET_MS)) {
                if (h <= (alt.h_base_m + APP_SYSID_ALT_BREAK_AT_BOTTOM_M)) {
                    alt.ramp_from_us = alt.pulse_us;   /* 已滑回槽底：本轮没有下行阈值 */
                    next = APP_SYSID_PHASE_RAMP_DOWN;
                } else {
                    alt.h_stuck_m = h;
                    alt.f_hold_n = alt.cmd_n;
                    next = APP_SYSID_PHASE_EXCITE;
                }
            }
        }
        break;
    case APP_SYSID_PHASE_EXCITE: {
        /* 停得离槽底不到 25 mm + 20 mm 时滑 25 mm 就到底了：要求的下降量取小，最少 10 mm。 */
        const float drop = fmaxf(fminf(APP_SYSID_ALT_BREAK_MOVE_M,
                                       alt.h_stuck_m - alt.h_base_m - APP_SYSID_ALT_BREAK_AT_BOTTOM_M),
                                 0.010f);

        if ((h <= (alt.h_stuck_m - drop)) &&
            (alt_rate_m_s() <= -APP_SYSID_ALT_BREAK_MOVE_RATE_M_S)) {
            alt.cmd_n = alt.last_cmd_n;             /* F_slide：观测来自上一拍的指令 */
            alt.fast_ms = now;
            alt.fast_h_m = h;
            alt.land_ms = now;
            alt.land_h_m = h;
            alt.brake_until_ms = now;
            next = APP_SYSID_PHASE_DESCEND;
        }
        break;
    }
    case APP_SYSID_PHASE_DESCEND:
        if (*phase_ms >= APP_SYSID_ALT_BREAK_DESCEND_MAX_MS) {
            out->abort_reason = "descent_timeout";
            break;
        }
        if ((uint32_t)(now - alt.fast_ms) >= 100U) {
            if ((h - alt.fast_h_m) < -APP_SYSID_ALT_BREAK_FAST_DOWN_M) {
                alt.brake_until_ms = now + APP_SYSID_ALT_BREAK_BRAKE_MS;   /* 掉得太快：短刹车 */
            }
            alt.fast_h_m = h;
            alt.fast_ms = now;
        }
        if ((uint32_t)(now - alt.land_ms) >= 200U) {
            const uint8_t still = (fabsf(h - alt.land_h_m) <= APP_SYSID_ALT_BREAK_LAND_STILL_M) ?
                                  1U : 0U;

            alt.land_h_m = h;
            alt.land_ms = now;
            if ((still != 0U) && (h <= (alt.h_base_m + APP_SYSID_ALT_BREAK_AT_BOTTOM_M))) {
                alt.ramp_from_us = alt.pulse_us;
                next = APP_SYSID_PHASE_RAMP_DOWN;
            } else if ((still != 0U) &&
                       ((alt.cmd_n - APP_SYSID_ALT_BREAK_STOP_STEP_N) >= alt.f_floor_n)) {
                alt.cmd_n -= APP_SYSID_ALT_BREAK_STOP_STEP_N;   /* 半路又卡住：再降一档让它滑下去 */
            }
        }
        break;
    case APP_SYSID_PHASE_RAMP_DOWN:
        alt_check_landing(*phase_ms, out);
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
 * break 这一拍的合推力指令（阶段已推进好）；RAMP_UP/RAMP_DOWN 由调用方按脉宽插值。
 * 返回负数 = 慢降到下限仍未滑落（no_slide）。
 */
static float alt_break_command(APP_SysIdPhase phase, uint32_t phase_ms, uint32_t now_ms)
{
    const float t_s = (float)phase_ms * 0.001f;
    float total_n = alt.cmd_n;

    switch (phase) {
    case APP_SYSID_PHASE_CLIMB:
        total_n = alt.f_start_n + (alt.rate_n_s * t_s);
        if (total_n >= alt.target_n) {
            total_n = alt.target_n;
            if (alt.ceil_ms == 0U) {
                alt.ceil_ms = (now_ms != 0U) ? now_ms : 1U;
            }
        }
        alt.cmd_n = total_n;
        break;
    case APP_SYSID_PHASE_SETTLE:
        if (alt.settle_first != 0U) {
            alt.settle_first = 0U;               /* 这一拍保持 F_det，PHASE 行报它 */
            total_n = alt.cmd_n;
            /* 下限只防降过头，绝不往上抬（2026-09-30 误判离地后曾把 1.6 N 抬到 8.1 N）。 */
            alt.cmd_n = fmaxf(alt.cmd_n - alt.stop_first_n, fminf(alt.f_floor_n, alt.cmd_n));
        }
        break;
    case APP_SYSID_PHASE_EXCITE:
        total_n = alt.f_hold_n - (alt.rate_n_s * t_s);   /* 进入那一拍即 F_hold，PHASE 行报它 */
        if (total_n <= alt.f_floor_n) {
            return -1.0f;
        }
        alt.cmd_n = total_n;
        break;
    case APP_SYSID_PHASE_DESCEND:
        total_n = alt.cmd_n +
                  (((int32_t)(alt.brake_until_ms - now_ms) > 0) ? APP_SYSID_ALT_BREAK_BRAKE_N : 0.0f);
        break;
    default:
        break;
    }
    return total_n;
}

/* ------------------------------------------------------------------ vel/pos：闭环 */

/* ① 阶段推进。条件满足本拍就切，返回新阶段并把 *phase_ms 清零（本拍按新阶段出力）。 */
static APP_SysIdPhase alt_advance(const APP_SysIdObserve *obs, APP_SysIdPhase phase,
                                  uint32_t *phase_ms, APP_SysIdAltOutput *out)
{
    APP_SysIdPhase next = phase;

    if (alt_is_break() != 0U) {
        return alt_advance_break(obs, phase, phase_ms, out);
    }

    switch (phase) {
    case APP_SYSID_PHASE_IDLE:
        alt.idle_pulse_us = obs->throttle_us;   /* 杆在最低 = 怠速：起升起点 */
        alt.pulse_us = obs->throttle_us;
        alt.last_height_ms = obs->height_sample_ms;
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
        alt_check_landing(*phase_ms, out);
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

/* ② 测高门：全程都判（含回落）。高度一律用 8 样本平均；返回中止理由或 NULL。 */
static const char *alt_height_gate(const APP_SysIdObserve *obs, APP_SysIdPhase phase)
{
    if (alt_height_fresh(obs) != 0U) {
        const float h = alt_avg_m();

        alt.last_height_ms = obs->height_sample_ms;
        if (alt_avg_ready() == 0U) {
            return NULL;             /* 平均值没攒满不判窗；持续没有新样本由下面的超时门管 */
        }
        if ((h < (alt.bottom_m - APP_SYSID_ALT_BELOW_BOTTOM_M)) || (h > alt.upper_m)) {
            return "height_window";
        }
        /* 提前制动门只管 break（没有高度环兜着）；vel/pos 有生产高度环，只判硬窗。break 慢降找滑落、
         * 滑回槽底时推力只降不升，冲不上去，不判它（停在高处时测距漂移会被算成上升）。 */
        if ((alt_is_break() != 0U) && (alt_rate_m_s() > 0.0f) &&
            (phase != APP_SYSID_PHASE_EXCITE) && (phase != APP_SYSID_PHASE_DESCEND)) {
            const float up_v = alt_rate_m_s();
            const float age_s = (float)(uint32_t)(obs->now_ms - obs->height_sample_ms) * 0.001f;
            const float stopping = APP_SYSID_ALT_BRAKE_MARGIN_M +
                up_v * (APP_SYSID_ALT_BRAKE_DELAY_S + age_s) +
                (up_v * up_v) / (2.0f * APP_SYSID_ALT_BRAKE_DECEL_M_S2);
            if ((h + stopping) >= alt.upper_m) {
                return "height_brake";
            }
        }
        return NULL;
    }
    if ((uint32_t)(obs->now_ms - alt.last_height_ms) > APP_SYSID_ALT_HEIGHT_TIMEOUT_MS) {
        return "height_invalid";
    }
    return NULL;
}

static void alt_finish_output(APP_SysIdPhase phase, uint16_t pulse, uint8_t capped,
                              APP_SysIdAltOutput *out)
{
    alt.pulse_us = pulse;
    alt.capped = capped;
    out->phase = phase;
    out->motor_pulse_us = pulse;
    out->capped = capped;
    out->thrust_n = alt_finite_or(DRV_COAX_CTRL_MotorPulseToTotalThrust(pulse), 0.0f);
}

/* break 的出力：高度/vz 只参与判据与保护，绝不经过 Z PID。 */
static void alt_break_output(const APP_SysIdObserve *obs, APP_SysIdPhase phase, uint32_t phase_ms,
                             APP_SysIdAltOutput *out)
{
    uint16_t pulse;
    uint8_t capped = 0U;
    float total_n;

    alt.z_sp_m = 0.0f;            /* height_sp/vz_sp = 0 表示不适用 */
    alt.vz_sp_m_s = 0.0f;
    switch (phase) {
    case APP_SYSID_PHASE_RAMP_UP:
        pulse = alt_lerp_pulse(alt.idle_pulse_us, alt_pulse_for(alt.f_start_n, &capped),
                               phase_ms, APP_SYSID_ALT_RAMP_UP_MS);
        alt.last_cmd_n = alt_finite_or(DRV_COAX_CTRL_MotorPulseToTotalThrust(pulse), 0.0f);
        alt_finish_output(phase, pulse, capped, out);
        break;
    case APP_SYSID_PHASE_RAMP_DOWN:
        pulse = alt_lerp_pulse(alt.ramp_from_us, obs->throttle_us, phase_ms,
                               APP_SYSID_ALT_RAMP_DOWN_MS);
        alt_finish_output(phase, pulse, 0U, out);
        break;
    default:
        total_n = alt_break_command(phase, phase_ms, obs->now_ms);
        if (total_n < 0.0f) {
            out->abort_reason = "no_slide";
            return;
        }
        pulse = alt_pulse_for(total_n, &capped);
        alt.last_cmd_n = total_n;
        alt_finish_output(phase, pulse, capped, out);
        break;
    }
    if (out->thrust_n >= APP_SYSID_MIN_THRUST_N) {
        out->hold_attitude = 1U;
        out->check_thrust = 1U;
    }
}

/* ------------------------------------------------------------------ 软着陆 */

/* 本模块要中止时先软着陆。推力还不到 0.5·m·g（机体必压在槽底）返回 0，调用方照原样当拍中止。 */
static uint8_t alt_soft_begin(const APP_SysIdObserve *obs, const char *reason)
{
    const float thrust_n = alt_finite_or(DRV_COAX_CTRL_MotorPulseToTotalThrust(alt.pulse_us), 0.0f);
    const float avg_n = (alt.thrust_avg_n > 0.0f) ? alt.thrust_avg_n : thrust_n;

    if (thrust_n < (APP_SYSID_ALT_SOFT_SKIP_FRACTION * alt.mass_kg * alt.gravity_m_s2)) {
        return 0U;
    }
    alt.soft_reason = reason;
    /* 从"当前"与"近 0.5 s 平均"中小的那个起降：高度环振荡时当前值可能正顶在上限——2026-09-30
     * alt_053452 从 16 N 起降，机体先被推到槽顶附近。往回加的上限取两者中大的。 */
    alt.soft_cmd_n = fminf(thrust_n, avg_n);
    alt.soft_cap_n = fmaxf(thrust_n, avg_n);
    alt.soft_start_ms = obs->now_ms;
    alt.soft_tick_ms = obs->now_ms;
    alt.soft_span_ms = 0U;
    return 1U;
}

/* 软着陆的一拍（阶段报 ramp_down）：按平均高度斜率慢降回槽底，再线性降到怠速；走完报进入时的理由。 */
static void alt_soft_step(const APP_SysIdObserve *obs, APP_SysIdAltOutput *out)
{
    const uint32_t now = obs->now_ms;
    uint8_t capped = 0U;
    uint16_t pulse;

    alt.z_sp_m = 0.0f;
    alt.vz_sp_m_s = 0.0f;
    if (alt_height_fresh(obs) != 0U) {
        alt.last_height_ms = obs->height_sample_ms;
    }
    if (alt.soft_span_ms == 0U) {
        const float dt_s = (float)(uint32_t)(now - alt.soft_tick_ms) * 0.001f;
        const float idle_n = alt_finite_or(DRV_COAX_CTRL_MotorPulseToTotalThrust(obs->throttle_us), 0.0f);
        uint32_t span = 0U;

        alt.soft_tick_ms = now;
        if ((uint32_t)(now - alt.last_height_ms) > APP_SYSID_ALT_HEIGHT_TIMEOUT_MS) {
            span = APP_SYSID_ALT_SOFT_BLIND_MS;
        } else if (alt_avg_ready() != 0U) {      /* 平均值在重新攒（刚断过档）的几拍：保持 */
            const float v = alt_rate_m_s();

            if (v < -APP_SYSID_ALT_SOFT_FAST_M_S) {
                alt.soft_cmd_n = fminf(alt.soft_cmd_n + (APP_SYSID_ALT_SOFT_BRAKE_N_S * dt_s),
                                       alt.soft_cap_n);
            } else if (v > APP_SYSID_ALT_SOFT_STILL_M_S) {
                alt.soft_cmd_n -= APP_SYSID_ALT_SOFT_STOP_N_S * dt_s;
            } else if (v >= -APP_SYSID_ALT_SOFT_STILL_M_S) {
                alt.soft_cmd_n -= APP_SYSID_ALT_SOFT_DOWN_N_S * dt_s;
            }                                    /* 缓降：保持 */
            if ((alt_avg_m() <= (alt.bottom_m + APP_SYSID_ALT_SOFT_AT_BOTTOM_M)) &&
                (v >= -APP_SYSID_ALT_SOFT_FAST_M_S)) {
                span = APP_SYSID_ALT_RAMP_DOWN_MS;
            }
        }
        if ((span == 0U) && ((alt.soft_cmd_n <= idle_n) ||
                             ((uint32_t)(now - alt.soft_start_ms) >= APP_SYSID_ALT_SOFT_MAX_MS))) {
            span = APP_SYSID_ALT_RAMP_DOWN_MS;
        }
        pulse = alt_pulse_for(fmaxf(alt.soft_cmd_n, idle_n), &capped);
        if (span != 0U) {
            alt.soft_span_ms = span;
            alt.soft_final_ms = now;
            alt.ramp_from_us = pulse;
        }
    } else {
        const uint32_t elapsed = (uint32_t)(now - alt.soft_final_ms);

        if (elapsed >= alt.soft_span_ms) {
            out->abort_reason = alt.soft_reason;
            return;
        }
        pulse = alt_lerp_pulse(alt.ramp_from_us, obs->throttle_us, elapsed, alt.soft_span_ms);
    }
    alt_finish_output(APP_SYSID_PHASE_RAMP_DOWN, pulse, capped, out);
    if (out->thrust_n >= APP_SYSID_MIN_THRUST_N) {
        out->hold_attitude = 1U;
        out->check_thrust = 1U;
    }
}

uint8_t APP_SysIdAlt_BeginSoftLanding(const APP_SysIdObserve *obs, const char *reason)
{
    alt_init();
    if ((obs == NULL) || (reason == NULL)) {
        return 0U;
    }
    return (alt.soft_reason != NULL) ? 1U : alt_soft_begin(obs, reason);
}

static void alt_step_run(const APP_SysIdObserve *obs, APP_SysIdPhase phase, uint32_t phase_ms,
                         float dt_s, APP_SysIdAltOutput *out);

void APP_SysIdAlt_Step(const APP_SysIdObserve *obs, APP_SysIdPhase phase, uint32_t phase_ms,
                       float dt_s, APP_SysIdAltOutput *out)
{
    alt_init();
    if ((obs == NULL) || (out == NULL)) {
        return;
    }
    memset(out, 0, sizeof(*out));
    if (alt.soft_reason == NULL) {
        alt_step_run(obs, phase, phase_ms, dt_s, out);
        if (out->abort_reason == NULL) {
            const float k = fminf(fmaxf(dt_s, 0.0f) / APP_SYSID_ALT_SOFT_AVG_TAU_S, 1.0f);

            alt.thrust_avg_n = (alt.thrust_avg_n > 0.0f) ?
                               (alt.thrust_avg_n + (k * (out->thrust_n - alt.thrust_avg_n))) :
                               out->thrust_n;
            return;
        }
        if (alt_soft_begin(obs, out->abort_reason) == 0U) {
            return;
        }
        memset(out, 0, sizeof(*out));
    }
    alt_soft_step(obs, out);
}

static void alt_step_run(const APP_SysIdObserve *obs, APP_SysIdPhase phase, uint32_t phase_ms,
                         float dt_s, APP_SysIdAltOutput *out)
{
    const float amplitude = alt.spec.amplitude_rad_s;
    float shape = 0.0f;
    float v_ff_m_s = 0.0f;
    uint16_t pulse;
    uint8_t capped = 0U;
    uint8_t loop = 0U;

    phase = alt_advance(obs, phase, &phase_ms, out);
    pulse = alt.pulse_us;
    if (out->abort_reason != NULL) {
        return;
    }
    if ((alt_is_break() == 0U) && (phase == APP_SYSID_PHASE_EXCITE)) {
        DRV_SysIdExcSample excitation;

        if (DRV_SysIdExcitation_Eval(&alt.spec, phase_ms, &excitation) != DRV_SYSID_EXC_OK) {
            out->abort_reason = "excitation";
            return;
        }
        if (excitation.finished != 0U) {
            alt.z_ref_m = alt.z_sp_m;   /* 闭环 vel 注入可能已偏离 z_hold */
            phase = APP_SYSID_PHASE_DESCEND;
            phase_ms = 0U;
        } else {
            /* 现有剖面按单位幅值归一化（Validate 保证 amp > 0），再乘本注入类型的幅值。 */
            shape = alt_finite_or(excitation.omega_sp_rad_s / amplitude, 0.0f);
            shape = fmaxf(-1.0f, fminf(1.0f, shape));
        }
    }
    out->abort_reason = alt_height_gate(obs, phase);
    if ((out->abort_reason != NULL) && (alt_is_break() != 0U) &&
        (phase == APP_SYSID_PHASE_SETTLE) && (strcmp(out->abort_reason, "height_brake") == 0)) {
        alt_break_brake(obs->now_ms);
        out->abort_reason = NULL;
    }
    if (out->abort_reason != NULL) {
        return;
    }
    if (alt_is_break() != 0U) {
        alt_break_output(obs, phase, phase_ms, out);
        return;
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
        if (alt.run.inject == (uint8_t)APP_SYSID_ALT_INJECT_VEL) {
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

    /* ④ vel/pos：生产高度环 → 合推力 → 查补表 → 封顶 */
    if (loop != 0U) {
        float total_n;

        if (alt_run_loop(obs, v_ff_m_s) == 0U) {
            out->abort_reason = "controller";
            return;
        }
        /* 同生产：coax.hover_thrust_n > 0 时按悬停推力换算有效质量，否则 m_run。 */
        total_n = DRV_COAX_CTRL_EffectiveMassKg(alt.mass_kg) *
                  (alt.gravity_m_s2 + alt.accel_cmd_m_s2);
        if (isfinite(total_n) == 0) {
            out->abort_reason = "controller";
            return;
        }
        pulse = alt_pulse_for(total_n, &capped);
        out->hold_attitude = 1U;
        out->check_thrust = 1U;
    }
    alt_finish_output(phase, pulse, capped, out);
    /* 预升/回落段也保持姿态（同 break）：2026-09-30 pos 轮预升 1.5 s 舵机回中，推力升到 10 N 时
     * 机体绕杆扭到 −4～−5°，进慢升才开始扶正——作者看到"起飞时往一边偏然后才扶正"。 */
    if (out->thrust_n >= APP_SYSID_MIN_THRUST_N) {
        out->hold_attitude = 1U;
        out->check_thrust = 1U;
    }
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
        *height_m = alt_avg_m();     /* 上位机「记为槽底/槽顶」用平均值，不用单个噪声样本 */
    }
    if (valid != NULL) {
        *valid = alt.live_valid;
    }
    if (height_sp_m != NULL) {
        *height_sp_m = alt.z_sp_m;
    }
}

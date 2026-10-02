#include "svc_flow_nav.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

/*
 * R-M5-5：这一份实现是"搬移 + 汇总"，不是重写。
 *   - 高度 LPF / 垂直速度 LPF / 中值窗 / 质量门 / 速度合理性门：原样自
 *     App/Src/app_optical_flow.c 搬来；
 *   - EKF 桥接 / 衰减 / 零速判定 / 质量自适应量测噪声：原样自
 *     App/Src/app_stabilizer.c 搬来；
 *   - 位置与累计位移积分：本次新增，用传感器自己的时间轴。
 * 数值一个没改，改的只是这些数学归谁所有。
 */

typedef struct {
    /* --- 高度 --- */
    float    height_m;
    float    height_raw_m;
    uint8_t  height_valid;
    float    vertical_velocity_m_s;
    uint32_t height_sample_ms;
    float    previous_height_m;
    uint32_t previous_height_sample_ms;
    float    height_jump_candidate_m;   /* 被门拦下的跳变样本（等确认） */
    uint8_t  height_jump_count;         /* 连续一致的跳变样本数 */
    uint32_t height_jump_since_ms;      /* 候选高度第一次出现的时刻 */

    /* --- 传感器系速度 --- */
    int16_t  flow_vel_x_window[SVC_FLOW_NAV_MEDIAN_WINDOW];
    int16_t  flow_vel_y_window[SVC_FLOW_NAV_MEDIAN_WINDOW];
    uint8_t  flow_median_count;
    uint8_t  flow_median_pos;
    int16_t  filtered_flow_vel_x;
    int16_t  filtered_flow_vel_y;
    uint8_t  flow_filter_ready;
    float    vx_m_s;
    float    vy_m_s;
    uint8_t  velocity_valid;
    uint32_t velocity_sample_ms;
    uint32_t velocity_reject_count;
    uint32_t processed_flow_ms;
    uint32_t last_good_ms;
    float    last_accept_vx_m_s;
    float    last_accept_vy_m_s;

    /* --- 积分时间基准（传感器自己的时钟） --- */
    uint32_t previous_accepted_sensor_time_ms;
    uint32_t pending_integration_dt_us;
    uint32_t last_integration_dt_us;
    uint32_t integrated_step_count;

    /* --- 水平速度估计 --- */
    float    vel_m_s[2];
    DRV_NAV_EKF_State ekf;
    DRV_NAV_EKF_Diagnostics diagnostics;
    uint8_t  zero_flow_count;

    /* --- 位置与累计位移 --- */
    float    position_m[2];
    float    displacement_m[2];
    float    diag_raw_m[2];      /* 诊断里程计：原始计数×0.01×高度×dt 累计 */
    float    diag_filt_m[2];     /* 诊断里程计：中值滤波后同一换算累计 */
    uint32_t diag_steps;
} SVC_FlowNav_Context;

static SVC_FlowNav_Context flow_nav_ctx;

static float flow_nav_square(float value)
{
    return value * value;
}

static float flow_nav_clamp(float value, float lo, float hi)
{
    if (value < lo) {
        return lo;
    }
    if (value > hi) {
        return hi;
    }
    return value;
}

/* ------------------------------------------------------------------ */
/* 传感器系：中值窗 / 高度 / 质量门                                     */
/* ------------------------------------------------------------------ */

static void flow_nav_reset_median_filter(void)
{
    memset(flow_nav_ctx.flow_vel_x_window, 0,
           sizeof(flow_nav_ctx.flow_vel_x_window));
    memset(flow_nav_ctx.flow_vel_y_window, 0,
           sizeof(flow_nav_ctx.flow_vel_y_window));
    flow_nav_ctx.flow_median_count = 0U;
    flow_nav_ctx.flow_median_pos = 0U;
    flow_nav_ctx.filtered_flow_vel_x = 0;
    flow_nav_ctx.filtered_flow_vel_y = 0;
    flow_nav_ctx.flow_filter_ready = 0U;
}

static void flow_nav_clear_velocity_sample(void)
{
    flow_nav_ctx.vx_m_s = 0.0f;
    flow_nav_ctx.vy_m_s = 0.0f;
    flow_nav_ctx.velocity_valid = 0U;
    flow_nav_ctx.velocity_sample_ms = 0U;
    flow_nav_ctx.last_good_ms = 0U;
    flow_nav_ctx.last_accept_vx_m_s = 0.0f;
    flow_nav_ctx.last_accept_vy_m_s = 0.0f;
    flow_nav_ctx.previous_accepted_sensor_time_ms = 0U;
    flow_nav_ctx.pending_integration_dt_us = 0U;
}

static void flow_nav_mark_velocity_invalid(void)
{
    flow_nav_ctx.velocity_valid = 0U;
    flow_nav_ctx.velocity_sample_ms = 0U;
    /*
     * 速度一旦作废，积分的时间锚点必须跟着丢掉。留着它的话，下一帧有效样本
     * 会把这中间整段无效时间当成一次正常步长积进去，凭空造出路程。
     */
    flow_nav_ctx.previous_accepted_sensor_time_ms = 0U;
    flow_nav_ctx.pending_integration_dt_us = 0U;
}

static void flow_nav_reject_velocity_sample(void)
{
    flow_nav_ctx.velocity_reject_count++;
    flow_nav_mark_velocity_invalid();
}

static int16_t flow_nav_median_i16(const int16_t *values, uint8_t count)
{
    int16_t sorted[SVC_FLOW_NAV_MEDIAN_WINDOW];

    if ((values == NULL) || (count == 0U)) {
        return 0;
    }
    if (count > SVC_FLOW_NAV_MEDIAN_WINDOW) {
        count = SVC_FLOW_NAV_MEDIAN_WINDOW;
    }

    for (uint8_t i = 0U; i < count; ++i) {
        sorted[i] = values[i];
    }
    for (uint8_t i = 1U; i < count; ++i) {
        int16_t key = sorted[i];
        uint8_t j = i;
        while ((j > 0U) && (sorted[j - 1U] > key)) {
            sorted[j] = sorted[j - 1U];
            j--;
        }
        sorted[j] = key;
    }

    return sorted[count / 2U];
}

/*
 * 窗口滤波输出：先取中值，偏离中值超过 SPIKE 的帧按中值算（剔毛刺），再取平均（四舍五入）。
 * 不直接输出中值：MTF-02P 每 10 ms 只报整像素位移（20 计数步进），低速时多数帧是 0，中值会把
 * 速度往 0 取整。2026-10-01 逐帧抓取（FLOWCAP）实测：慢推中值积分只剩原始的 73%、快推 95%，
 * 悬停 5 cm/s 以下的慢漂几乎全被抹成 0；平均对整像素数据无偏，延迟与中值相同（约 20 ms）。
 */
static int16_t flow_nav_gated_mean_i16(const int16_t *values, uint8_t count)
{
    int32_t median;
    int32_t sum = 0;

    if ((values == NULL) || (count == 0U)) {
        return 0;
    }
    if (count > SVC_FLOW_NAV_MEDIAN_WINDOW) {
        count = SVC_FLOW_NAV_MEDIAN_WINDOW;
    }
    median = flow_nav_median_i16(values, count);
    for (uint8_t i = 0U; i < count; ++i) {
        int32_t value = values[i];
        if ((value - median > SVC_FLOW_NAV_SPIKE_COUNTS) ||
            (median - value > SVC_FLOW_NAV_SPIKE_COUNTS)) {
            value = median;
        }
        sum += value;
    }
    return (int16_t)((sum >= 0) ? ((sum + (int32_t)(count / 2U)) / (int32_t)count)
                                : -((-sum + (int32_t)(count / 2U)) / (int32_t)count));
}

static void flow_nav_update_median_filter(const SVC_FLOW_NAV_Sample *sample)
{
    if (sample == NULL) {
        flow_nav_reset_median_filter();
        return;
    }

    flow_nav_ctx.flow_vel_x_window[flow_nav_ctx.flow_median_pos] =
        sample->flow_vel_x;
    flow_nav_ctx.flow_vel_y_window[flow_nav_ctx.flow_median_pos] =
        sample->flow_vel_y;
    flow_nav_ctx.flow_median_pos++;
    if (flow_nav_ctx.flow_median_pos >= SVC_FLOW_NAV_MEDIAN_WINDOW) {
        flow_nav_ctx.flow_median_pos = 0U;
    }
    if (flow_nav_ctx.flow_median_count < SVC_FLOW_NAV_MEDIAN_WINDOW) {
        flow_nav_ctx.flow_median_count++;
    }

    flow_nav_ctx.filtered_flow_vel_x =
        flow_nav_gated_mean_i16(flow_nav_ctx.flow_vel_x_window,
                                flow_nav_ctx.flow_median_count);
    flow_nav_ctx.filtered_flow_vel_y =
        flow_nav_gated_mean_i16(flow_nav_ctx.flow_vel_y_window,
                                flow_nav_ctx.flow_median_count);
    flow_nav_ctx.flow_filter_ready =
        (flow_nav_ctx.flow_median_count >= SVC_FLOW_NAV_MEDIAN_MIN_SAMPLES) ?
        1U : 0U;
}

static uint8_t flow_nav_velocity_plausible(float vx_m_s, float vy_m_s)
{
    float speed_sq = flow_nav_square(vx_m_s) + flow_nav_square(vy_m_s);
    float dvx;
    float dvy;
    float step_sq;

    if (speed_sq > flow_nav_square(SVC_FLOW_NAV_MAX_SPEED_M_S)) {
        return 0U;
    }

    if (flow_nav_ctx.last_good_ms == 0U) {
        return 1U;
    }

    dvx = vx_m_s - flow_nav_ctx.last_accept_vx_m_s;
    dvy = vy_m_s - flow_nav_ctx.last_accept_vy_m_s;
    step_sq = flow_nav_square(dvx) + flow_nav_square(dvy);
    return (step_sq <= flow_nav_square(SVC_FLOW_NAV_MAX_SPEED_STEP_M_S)) ?
           1U : 0U;
}

static void flow_nav_update_height(const SVC_FLOW_NAV_Sample *sample)
{
    float raw_height_m;
    float filtered_height_m;

    if ((sample == NULL) || (sample->distance_valid == 0U) ||
        (sample->distance_mm == 0UL) ||
        (sample->distance_received_ms == 0UL) ||
        (sample->distance_received_ms ==
         flow_nav_ctx.previous_height_sample_ms)) {
        return;
    }

    raw_height_m = (float)sample->distance_mm * 0.001f;
    if ((raw_height_m <= 0.0f) || (raw_height_m > SVC_FLOW_NAV_MAX_HEIGHT_M)) {
        return;
    }
    if ((flow_nav_ctx.previous_height_sample_ms != 0U) &&
        ((sample->distance_received_ms - flow_nav_ctx.previous_height_sample_ms) <=
         SVC_FLOW_NAV_HEIGHT_GATE_HOLD_MS) &&
        (fabsf(raw_height_m - flow_nav_ctx.height_m) >
         SVC_FLOW_NAV_MAX_HEIGHT_STEP_M)) {
        /* 跳变门（参照最近一次接受的高度，不看 height_valid，见头文件）。 */
        if ((flow_nav_ctx.height_jump_count != 0U) &&
            (fabsf(raw_height_m - flow_nav_ctx.height_jump_candidate_m) <=
             SVC_FLOW_NAV_MAX_HEIGHT_STEP_M)) {
            flow_nav_ctx.height_jump_count++;
        } else {
            flow_nav_ctx.height_jump_count = 1U;
            flow_nav_ctx.height_jump_since_ms = sample->distance_received_ms;
        }
        flow_nav_ctx.height_jump_candidate_m = raw_height_m;
        if ((flow_nav_ctx.height_jump_count < SVC_FLOW_NAV_HEIGHT_JUMP_CONFIRM) ||
            ((sample->distance_received_ms - flow_nav_ctx.height_jump_since_ms) <
             SVC_FLOW_NAV_HEIGHT_JUMP_CONFIRM_MS)) {
            return;
        }
        /* 真跳变已确认：从新高度重新起步，不让低通与微分把台阶拖成一串假垂直速度。 */
        flow_nav_ctx.height_jump_count = 0U;
        flow_nav_ctx.previous_height_sample_ms = 0U;
    } else {
        flow_nav_ctx.height_jump_count = 0U;
    }

    if (flow_nav_ctx.previous_height_sample_ms == 0U) {
        filtered_height_m = raw_height_m;
        flow_nav_ctx.vertical_velocity_m_s = 0.0f;
    } else {
        uint32_t dt_ms = sample->distance_received_ms -
                         flow_nav_ctx.previous_height_sample_ms;
        filtered_height_m = flow_nav_ctx.height_m +
            SVC_FLOW_NAV_HEIGHT_LPF_ALPHA *
            (raw_height_m - flow_nav_ctx.height_m);
        if ((dt_ms >= 2U) && (dt_ms <= SVC_FLOW_NAV_TIMEOUT_MS)) {
            float vz_m_s = (filtered_height_m -
                            flow_nav_ctx.previous_height_m) /
                           ((float)dt_ms * 0.001f);
            vz_m_s = flow_nav_clamp(vz_m_s,
                                    -SVC_FLOW_NAV_MAX_VERTICAL_VEL_M_S,
                                     SVC_FLOW_NAV_MAX_VERTICAL_VEL_M_S);
            flow_nav_ctx.vertical_velocity_m_s +=
                SVC_FLOW_NAV_VELOCITY_LPF_ALPHA *
                (vz_m_s - flow_nav_ctx.vertical_velocity_m_s);
        }
    }

    flow_nav_ctx.height_raw_m = raw_height_m;
    flow_nav_ctx.height_m = filtered_height_m;
    flow_nav_ctx.height_valid = 1U;
    flow_nav_ctx.height_sample_ms = sample->distance_received_ms;
    flow_nav_ctx.previous_height_m = flow_nav_ctx.height_m;
    flow_nav_ctx.previous_height_sample_ms = sample->distance_received_ms;
}

static uint8_t flow_nav_sample_usable(const SVC_FLOW_NAV_Sample *sample,
                                      uint32_t now_ms)
{
    if ((sample == NULL) ||
        (sample->frame_valid == 0U) ||
        (sample->distance_valid == 0U) ||
        (sample->flow_valid == 0U) ||
        (sample->distance_received_ms == 0UL) ||
        (sample->flow_received_ms == 0UL) ||
        ((now_ms - sample->distance_received_ms) > SVC_FLOW_NAV_TIMEOUT_MS) ||
        ((now_ms - sample->flow_received_ms) > SVC_FLOW_NAV_TIMEOUT_MS) ||
        (sample->flow_quality < SVC_FLOW_NAV_MIN_QUALITY) ||
        (flow_nav_ctx.height_valid == 0U)) {
        return 0U;
    }

    return 1U;
}

/*
 * 取本次积分的步长，全部来自传感器自己的时钟：
 *   首选 time_ms 与上一次被接受样本的 time_ms 之差（对 App 跳帧免疫）；
 *   time_ms 不可用（首帧、回绕、传感器没给）时退回 Driver 统计的
 *   sample_interval_us——它本身也是由 time_ms 差分算出来的。
 * 拿控制环节拍或上位机轮询周期充数是明确禁止的。
 */
static uint32_t flow_nav_integration_dt_us(const SVC_FLOW_NAV_Sample *sample)
{
    uint32_t dt_us = 0UL;

    if ((sample->sensor_time_ms != 0UL) &&
        (flow_nav_ctx.previous_accepted_sensor_time_ms != 0UL) &&
        (sample->sensor_time_ms >
         flow_nav_ctx.previous_accepted_sensor_time_ms)) {
        dt_us = (sample->sensor_time_ms -
                 flow_nav_ctx.previous_accepted_sensor_time_ms) * 1000UL;
    } else if (flow_nav_ctx.previous_accepted_sensor_time_ms != 0UL) {
        dt_us = (uint32_t)sample->sample_interval_us;
    } else {
        return 0UL;
    }

    if ((dt_us < SVC_FLOW_NAV_MIN_INTEGRATION_DT_US) ||
        (dt_us > SVC_FLOW_NAV_MAX_INTEGRATION_DT_US)) {
        return 0UL;
    }

    return dt_us;
}

/* ------------------------------------------------------------------ */
/* 水平速度估计（EKF）                                                  */
/* ------------------------------------------------------------------ */

static void flow_nav_estimator_reset(void)
{
    DRV_NAV_EKF_Config config;

    flow_nav_ctx.zero_flow_count = 0U;
    flow_nav_ctx.vel_m_s[0] = 0.0f;
    flow_nav_ctx.vel_m_s[1] = 0.0f;

    DRV_NAV_EKF_DefaultConfig(&config);
    config.flow_noise_m_s = SVC_FLOW_NAV_EKF_FLOW_NOISE_M_S;
    config.flow_gate_nis = 0.0f;
    DRV_NAV_EKF_Reset(&flow_nav_ctx.ekf, &config);
    DRV_NAV_EKF_GetDiagnostics(&flow_nav_ctx.ekf, &flow_nav_ctx.diagnostics);
}

static void flow_nav_estimator_zero_horizontal(void)
{
    flow_nav_ctx.vel_m_s[0] = 0.0f;
    flow_nav_ctx.vel_m_s[1] = 0.0f;
    flow_nav_ctx.zero_flow_count = 0U;
    flow_nav_ctx.ekf.vel_m_s[0] = 0.0f;
    flow_nav_ctx.ekf.vel_m_s[1] = 0.0f;
    flow_nav_ctx.ekf.accel_bias_m_s2[0] = 0.0f;
    flow_nav_ctx.ekf.accel_bias_m_s2[1] = 0.0f;
}

/*
 * 零速钳位只清速度，保留已学到的加速度计零偏。零偏是慢变量，静止恰恰是它最好学的
 * 时候；连它一起清，|零偏| 落在 ZERO_ACCEL 门限以下时会被每 80 ms 抹一次、永远学
 * 不到，静止也按 ~1 cm/s 漂（2026-10-01 仿真：0.17 m/s² 零偏 20 s 漂 19 cm）。
 */
static void flow_nav_estimator_zero_velocity(void)
{
    flow_nav_ctx.vel_m_s[0] = 0.0f;
    flow_nav_ctx.vel_m_s[1] = 0.0f;
    flow_nav_ctx.zero_flow_count = 0U;
    flow_nav_ctx.ekf.vel_m_s[0] = 0.0f;
    flow_nav_ctx.ekf.vel_m_s[1] = 0.0f;
}

static float flow_nav_noise_from_quality(uint8_t quality)
{
    float quality_norm;
    float weak;
    /*
     * 锚点用 QUALITY_NOISE_LOW 而不是硬拒门：拒门放宽只应该让原本被丢弃的帧
     * "以最低信任度进来"，不应该顺带抬高对 q>=80 那批数据的信任度。
     */
    const float q_min = (float)SVC_FLOW_NAV_QUALITY_NOISE_LOW;
    const float q_high = (float)SVC_FLOW_NAV_EKF_FLOW_QUALITY_HIGH;

    if (quality <= SVC_FLOW_NAV_QUALITY_NOISE_LOW) {
        return SVC_FLOW_NAV_EKF_FLOW_NOISE_MAX_M_S;
    }
    if (quality >= SVC_FLOW_NAV_EKF_FLOW_QUALITY_HIGH) {
        return SVC_FLOW_NAV_EKF_FLOW_NOISE_MIN_M_S;
    }

    quality_norm = ((float)quality - q_min) / (q_high - q_min);
    weak = 1.0f - flow_nav_clamp(quality_norm, 0.0f, 1.0f);
    return SVC_FLOW_NAV_EKF_FLOW_NOISE_MIN_M_S +
           (SVC_FLOW_NAV_EKF_FLOW_NOISE_MAX_M_S -
            SVC_FLOW_NAV_EKF_FLOW_NOISE_MIN_M_S) * weak * weak;
}

static uint8_t flow_nav_control_velocity_plausible(float flow_vx_m_s,
                                                   float flow_vy_m_s,
                                                   uint32_t flow_sample_ms)
{
    float speed_sq;

    if (flow_sample_ms == 0U) {
        return 0U;
    }

    speed_sq = (flow_vx_m_s * flow_vx_m_s) + (flow_vy_m_s * flow_vy_m_s);
    if (speed_sq >
        (SVC_FLOW_NAV_EKF_CONTROL_MAX_SPEED_M_S *
         SVC_FLOW_NAV_EKF_CONTROL_MAX_SPEED_M_S)) {
        return 0U;
    }

    if ((flow_nav_ctx.diagnostics.last_flow_update_ms != 0U) &&
        (flow_sample_ms > flow_nav_ctx.diagnostics.last_flow_update_ms)) {
        const float dt_sec =
            (float)(flow_sample_ms -
                    flow_nav_ctx.diagnostics.last_flow_update_ms) * 0.001f;
        float max_step_m_s = SVC_FLOW_NAV_FLOW_ONLY_MAX_ACCEL_M_S2 * dt_sec;
        const float dvx = flow_vx_m_s - flow_nav_ctx.vel_m_s[0];
        const float dvy = flow_vy_m_s - flow_nav_ctx.vel_m_s[1];
        const float step_sq = (dvx * dvx) + (dvy * dvy);

        if (max_step_m_s < SVC_FLOW_NAV_FLOW_ONLY_MIN_STEP_M_S) {
            max_step_m_s = SVC_FLOW_NAV_FLOW_ONLY_MIN_STEP_M_S;
        }
        if (step_sq > (max_step_m_s * max_step_m_s)) {
            return 0U;
        }
    }

    return 1U;
}

/*
 * 位置 / 累计位移积分。只在光流量测真正被 EKF 采纳的那一拍推进，步长取该样本
 * 在传感器时间轴上跨过的时长。光流丢失时估计器只是在按 decay 收敛，没有新的
 * 量测也就没有新的路程，这时候继续积分等于把衰减曲线当成真实移动。
 */
static void flow_nav_integrate_position(void)
{
    uint32_t dt_us = flow_nav_ctx.pending_integration_dt_us;
    float dt_sec;
    float dx_m;
    float dy_m;

    flow_nav_ctx.pending_integration_dt_us = 0U;
    if ((dt_us < SVC_FLOW_NAV_MIN_INTEGRATION_DT_US) ||
        (dt_us > SVC_FLOW_NAV_MAX_INTEGRATION_DT_US)) {
        return;
    }

    dt_sec = (float)dt_us * 1.0e-6f;
    dx_m = flow_nav_ctx.vel_m_s[0] * dt_sec;
    dy_m = flow_nav_ctx.vel_m_s[1] * dt_sec;

    flow_nav_ctx.displacement_m[0] += dx_m;
    flow_nav_ctx.displacement_m[1] += dy_m;
    flow_nav_ctx.position_m[0] =
        flow_nav_clamp(flow_nav_ctx.position_m[0] + dx_m,
                       -SVC_FLOW_NAV_POSITION_LIMIT_M,
                        SVC_FLOW_NAV_POSITION_LIMIT_M);
    flow_nav_ctx.position_m[1] =
        flow_nav_clamp(flow_nav_ctx.position_m[1] + dy_m,
                       -SVC_FLOW_NAV_POSITION_LIMIT_M,
                        SVC_FLOW_NAV_POSITION_LIMIT_M);
    flow_nav_ctx.last_integration_dt_us = dt_us;
    flow_nav_ctx.integrated_step_count++;
}

/* ------------------------------------------------------------------ */
/* 公共 API                                                             */
/* ------------------------------------------------------------------ */

void SVC_FlowNav_Init(void)
{
    memset(&flow_nav_ctx, 0, sizeof(flow_nav_ctx));
    flow_nav_estimator_reset();
}

void SVC_FlowNav_Reset(void)
{
    flow_nav_ctx.height_m = 0.0f;
    flow_nav_ctx.height_raw_m = 0.0f;
    flow_nav_ctx.height_valid = 0U;
    flow_nav_ctx.vertical_velocity_m_s = 0.0f;
    flow_nav_ctx.height_sample_ms = 0U;
    flow_nav_ctx.previous_height_m = 0.0f;
    flow_nav_ctx.previous_height_sample_ms = 0U;
    flow_nav_ctx.height_jump_candidate_m = 0.0f;
    flow_nav_ctx.height_jump_count = 0U;
    flow_nav_ctx.height_jump_since_ms = 0U;
    flow_nav_reset_median_filter();
    flow_nav_clear_velocity_sample();
    flow_nav_ctx.processed_flow_ms = 0U;
    flow_nav_ctx.velocity_reject_count = 0U;
    /* 传感器重来一遍，里程计的连续性也就断了，跟着清。 */
    SVC_FlowNav_ResetPosition();
    SVC_FlowNav_ResetDisplacement();
}

void SVC_FlowNav_Age(uint32_t now_ms)
{
    if (flow_nav_ctx.height_sample_ms == 0U) {
        return;
    }
    if ((now_ms - flow_nav_ctx.height_sample_ms) <= SVC_FLOW_NAV_TIMEOUT_MS) {
        return;
    }

    flow_nav_ctx.height_valid = 0U;
    flow_nav_ctx.vertical_velocity_m_s = 0.0f;
    flow_nav_mark_velocity_invalid();
    if ((flow_nav_ctx.last_good_ms == 0U) ||
        ((now_ms - flow_nav_ctx.last_good_ms) > SVC_FLOW_NAV_FILTER_RESET_MS)) {
        flow_nav_reset_median_filter();
    }
}

SVC_FLOW_NAV_SampleResult SVC_FlowNav_PushSample(
    const SVC_FLOW_NAV_Sample *sample, uint32_t now_ms)
{
    float sensor_vx_m_s;
    float sensor_vy_m_s;
    uint32_t dt_us;

    if (sample == NULL) {
        return SVC_FLOW_NAV_SAMPLE_STALE;
    }

    flow_nav_update_height(sample);

    if ((sample->flow_received_ms == 0UL) ||
        (sample->flow_received_ms == flow_nav_ctx.processed_flow_ms)) {
        /* 本拍没有新的光流样本：上一帧还新鲜就继续用，过期就作废。 */
        if ((flow_nav_ctx.last_good_ms != 0U) &&
            ((now_ms - flow_nav_ctx.last_good_ms) <= SVC_FLOW_NAV_TIMEOUT_MS)) {
            flow_nav_ctx.velocity_valid = 1U;
            return SVC_FLOW_NAV_SAMPLE_HOLD;
        }
        flow_nav_ctx.velocity_valid = 0U;
        return SVC_FLOW_NAV_SAMPLE_STALE;
    }

    flow_nav_ctx.processed_flow_ms = sample->flow_received_ms;

    if (flow_nav_sample_usable(sample, now_ms) == 0U) {
        flow_nav_reject_velocity_sample();
        if ((flow_nav_ctx.last_good_ms == 0U) ||
            ((now_ms - flow_nav_ctx.last_good_ms) >
             SVC_FLOW_NAV_FILTER_RESET_MS)) {
            flow_nav_reset_median_filter();
        }
        return SVC_FLOW_NAV_SAMPLE_REJECTED;
    }

    flow_nav_update_median_filter(sample);
    if (flow_nav_ctx.flow_filter_ready == 0U) {
        flow_nav_mark_velocity_invalid();
        return SVC_FLOW_NAV_SAMPLE_WARMUP;
    }

    /*
     * 单位换算：v = 计数 * 0.01 * 高度。
     * 输入的 flow_vel_x/y 已经是规范 FLU 计数（方言适配在
     * app_optical_flow.c 的采集边界完成），本 Service 不旋转、不换轴。
     */
    sensor_vx_m_s = (float)flow_nav_ctx.filtered_flow_vel_x * 0.01f *
                    flow_nav_ctx.height_m;
    sensor_vy_m_s = (float)flow_nav_ctx.filtered_flow_vel_y * 0.01f *
                    flow_nav_ctx.height_m;

    if (flow_nav_velocity_plausible(sensor_vx_m_s, sensor_vy_m_s) == 0U) {
        flow_nav_reject_velocity_sample();
        if ((flow_nav_ctx.last_good_ms == 0U) ||
            ((now_ms - flow_nav_ctx.last_good_ms) >
             SVC_FLOW_NAV_FILTER_RESET_MS)) {
            flow_nav_reset_median_filter();
        }
        return SVC_FLOW_NAV_SAMPLE_REJECTED;
    }

    dt_us = flow_nav_integration_dt_us(sample);
    if (dt_us != 0UL) {
        const float dt_s = (float)dt_us * 1.0e-6f;
        flow_nav_ctx.diag_raw_m[0] +=
            (float)sample->flow_vel_x * 0.01f * flow_nav_ctx.height_m * dt_s;
        flow_nav_ctx.diag_raw_m[1] +=
            (float)sample->flow_vel_y * 0.01f * flow_nav_ctx.height_m * dt_s;
        flow_nav_ctx.diag_filt_m[0] += sensor_vx_m_s * dt_s;
        flow_nav_ctx.diag_filt_m[1] += sensor_vy_m_s * dt_s;
        flow_nav_ctx.diag_steps++;
    }
    flow_nav_ctx.vx_m_s = sensor_vx_m_s;
    flow_nav_ctx.vy_m_s = sensor_vy_m_s;
    flow_nav_ctx.velocity_valid = 1U;
    flow_nav_ctx.velocity_sample_ms = sample->flow_received_ms;
    flow_nav_ctx.last_good_ms = sample->flow_received_ms;
    flow_nav_ctx.last_accept_vx_m_s = sensor_vx_m_s;
    flow_nav_ctx.last_accept_vy_m_s = sensor_vy_m_s;
    flow_nav_ctx.previous_accepted_sensor_time_ms = sample->sensor_time_ms;
    /*
     * 控制环消费得比传感器快，正常情况下这里累加的就是单个样本的步长；万一
     * 两拍之间来了多帧，也把它们跨过的时间一起交出去，不丢步长。
     */
    if (dt_us != 0UL) {
        uint32_t pending = flow_nav_ctx.pending_integration_dt_us + dt_us;
        flow_nav_ctx.pending_integration_dt_us =
            (pending > SVC_FLOW_NAV_MAX_INTEGRATION_DT_US) ? 0U : pending;
    }

    return SVC_FLOW_NAV_SAMPLE_ACCEPTED;
}

uint8_t SVC_FlowNav_GetHeight(float *height_m,
                              float *vertical_velocity_m_s,
                              uint32_t *sample_ms,
                              uint32_t now_ms)
{
    if ((flow_nav_ctx.height_valid == 0U) ||
        (flow_nav_ctx.height_sample_ms == 0U) ||
        ((now_ms - flow_nav_ctx.height_sample_ms) > SVC_FLOW_NAV_TIMEOUT_MS)) {
        return 0U;
    }

    if (height_m != NULL) {
        *height_m = flow_nav_ctx.height_m;
    }
    if (vertical_velocity_m_s != NULL) {
        *vertical_velocity_m_s = flow_nav_ctx.vertical_velocity_m_s;
    }
    if (sample_ms != NULL) {
        *sample_ms = flow_nav_ctx.height_sample_ms;
    }
    return 1U;
}

uint8_t SVC_FlowNav_GetSensorVelocity(float *vx_m_s,
                                      float *vy_m_s,
                                      uint32_t *sample_ms,
                                      uint32_t now_ms)
{
    if ((flow_nav_ctx.velocity_valid == 0U) ||
        (flow_nav_ctx.velocity_sample_ms == 0U) ||
        ((now_ms - flow_nav_ctx.velocity_sample_ms) >
         SVC_FLOW_NAV_TIMEOUT_MS)) {
        return 0U;
    }

    if (vx_m_s != NULL) {
        *vx_m_s = flow_nav_ctx.vx_m_s;
    }
    if (vy_m_s != NULL) {
        *vy_m_s = flow_nav_ctx.vy_m_s;
    }
    if (sample_ms != NULL) {
        *sample_ms = flow_nav_ctx.velocity_sample_ms;
    }
    return 1U;
}

void SVC_FlowNav_GetState(SVC_FLOW_NAV_State *state)
{
    if (state == NULL) {
        return;
    }

    memset(state, 0, sizeof(*state));
    state->height_valid = flow_nav_ctx.height_valid;
    state->height_m = flow_nav_ctx.height_m;
    state->height_raw_m = flow_nav_ctx.height_raw_m;
    state->vertical_velocity_m_s = flow_nav_ctx.vertical_velocity_m_s;
    state->height_filter_alpha = SVC_FLOW_NAV_HEIGHT_LPF_ALPHA;
    state->height_sample_ms = flow_nav_ctx.height_sample_ms;
    state->velocity_valid = flow_nav_ctx.velocity_valid;
    state->vx_m_s = flow_nav_ctx.vx_m_s;
    state->vy_m_s = flow_nav_ctx.vy_m_s;
    state->velocity_sample_ms = flow_nav_ctx.velocity_sample_ms;
    state->velocity_reject_count = flow_nav_ctx.velocity_reject_count;
    state->flow_vel_x_filtered = flow_nav_ctx.filtered_flow_vel_x;
    state->flow_vel_y_filtered = flow_nav_ctx.filtered_flow_vel_y;
    state->flow_filter_ready = flow_nav_ctx.flow_filter_ready;
}

uint32_t SVC_FlowNav_GetLastGoodMs(void)
{
    return flow_nav_ctx.last_good_ms;
}

uint8_t SVC_FlowNav_Fuse(const SVC_FLOW_NAV_FuseInput *input)
{
    uint8_t flow_accepted;
    uint8_t imu_bridge_ok = 0U;
    uint32_t flow_age_ms = 0xFFFFFFFFUL;
    uint8_t new_flow_sample = 0U;
    float flow_noise_m_s;
    float dt_sec;

    if (input == NULL) {
        return 0U;
    }

    dt_sec = (input->dt_sec > 0.0f) ? input->dt_sec :
             SVC_FLOW_NAV_DEFAULT_DT_SEC;

    if (flow_nav_ctx.diagnostics.last_flow_update_ms != 0U) {
        flow_age_ms = input->now_ms -
                      flow_nav_ctx.diagnostics.last_flow_update_ms;
    }
    new_flow_sample =
        ((input->flow_valid != 0U) &&
         (input->flow_sample_ms != 0U) &&
         (input->flow_sample_ms != flow_nav_ctx.ekf.last_flow_sample_ms)) ?
        1U : 0U;

    if ((new_flow_sample == 0U) &&
        (flow_nav_ctx.diagnostics.last_flow_update_ms != 0U) &&
        (flow_age_ms > SVC_FLOW_NAV_EKF_FLOW_STALE_RESET_MS)) {
        flow_nav_estimator_zero_horizontal();
        DRV_NAV_EKF_GetDiagnostics(&flow_nav_ctx.ekf,
                                   &flow_nav_ctx.diagnostics);
        return 0U;
    }

    if ((flow_nav_ctx.diagnostics.flow_update_count != 0U) &&
        (flow_nav_ctx.diagnostics.last_flow_update_ms != 0U) &&
        (flow_age_ms <= SVC_FLOW_NAV_EKF_IMU_BRIDGE_TIMEOUT_MS)) {
        imu_bridge_ok = 1U;
    }

    if (imu_bridge_ok != 0U) {
        /*
         * 机头对齐系是随偏航转动的系，速度分量的导数不等于加速度：
         *     dv/dt|分量 = a - ω × v,  ω = (0, 0, ω_z)
         * 展开 ω × v = (-ω_z*v_y, +ω_z*v_x, 0)，故按下式换成等效加速度。
         * 用当前状态估计，不是上一拍的量。
         *
         * 这一项留给 EKF 自己做是错的：drv_nav_ekf 被明确定义为 X/Y 互不串
         * 扰的纯数值 2D KF（tests/test_flu_seam2_navigation_frame.py 钉住），
         * 而这里恰恰是一个把两轴耦合起来的项——它属于坐标系语义，归本 Service。
         */
        const float omega_z = input->yaw_rate_rad_s;
        const float accel_x_eff = input->accel_x_m_s2 +
                                  (omega_z * flow_nav_ctx.ekf.vel_m_s[1]);
        const float accel_y_eff = input->accel_y_m_s2 -
                                  (omega_z * flow_nav_ctx.ekf.vel_m_s[0]);

        DRV_NAV_EKF_Predict(&flow_nav_ctx.ekf,
                            accel_x_eff,
                            accel_y_eff,
                            dt_sec);
    } else {
        float decay_hz = SVC_FLOW_NAV_EKF_FLOW_LOST_DECAY_HZ;
        float decay;
        if ((flow_nav_ctx.diagnostics.last_flow_update_ms != 0U) &&
            (flow_age_ms > SVC_FLOW_NAV_EKF_FLOW_SOFT_HOLD_MS)) {
            decay_hz = SVC_FLOW_NAV_EKF_FLOW_STALE_DECAY_HZ;
        }
        decay = 1.0f - (decay_hz * dt_sec);
        decay = flow_nav_clamp(decay, 0.0f, 1.0f);
        flow_nav_ctx.ekf.vel_m_s[0] *= decay;
        flow_nav_ctx.ekf.vel_m_s[1] *= decay;
        flow_nav_ctx.ekf.accel_bias_m_s2[0] = 0.0f;
        flow_nav_ctx.ekf.accel_bias_m_s2[1] = 0.0f;
    }

    flow_noise_m_s = flow_nav_noise_from_quality(input->flow_quality);
    if ((new_flow_sample != 0U) &&
        (flow_nav_control_velocity_plausible(input->flow_vx_m_s,
                                             input->flow_vy_m_s,
                                             input->flow_sample_ms) == 0U)) {
        flow_nav_ctx.zero_flow_count = 0U;
        flow_nav_ctx.ekf.flow_skip_count++;
        flow_nav_ctx.ekf.last_flow_sample_ms = input->flow_sample_ms;
        DRV_NAV_EKF_GetDiagnostics(&flow_nav_ctx.ekf,
                                   &flow_nav_ctx.diagnostics);
        flow_nav_ctx.vel_m_s[0] = flow_nav_ctx.diagnostics.vel_m_s[0];
        flow_nav_ctx.vel_m_s[1] = flow_nav_ctx.diagnostics.vel_m_s[1];
        /* 这一帧被判为野值，它跨过的时间不能算成路程。 */
        flow_nav_ctx.pending_integration_dt_us = 0U;
        return 0U;
    }

    flow_accepted = DRV_NAV_EKF_FuseFlow(&flow_nav_ctx.ekf,
                                         input->flow_vx_m_s,
                                         input->flow_vy_m_s,
                                         input->flow_valid,
                                         input->flow_sample_ms,
                                         flow_noise_m_s);
    DRV_NAV_EKF_GetDiagnostics(&flow_nav_ctx.ekf, &flow_nav_ctx.diagnostics);
    if ((flow_accepted != 0U) &&
        (((input->flow_vx_m_s * input->flow_vx_m_s) +
          (input->flow_vy_m_s * input->flow_vy_m_s)) <=
         (SVC_FLOW_NAV_EKF_ZERO_FLOW_SPEED_M_S *
          SVC_FLOW_NAV_EKF_ZERO_FLOW_SPEED_M_S)) &&
        (((input->accel_x_m_s2 * input->accel_x_m_s2) +
          (input->accel_y_m_s2 * input->accel_y_m_s2)) <=
         (SVC_FLOW_NAV_EKF_ZERO_ACCEL_M_S2 *
          SVC_FLOW_NAV_EKF_ZERO_ACCEL_M_S2))) {
        if (flow_nav_ctx.zero_flow_count <
            SVC_FLOW_NAV_EKF_ZERO_FLOW_COUNT) {
            flow_nav_ctx.zero_flow_count++;
        }
        if (flow_nav_ctx.zero_flow_count >=
            SVC_FLOW_NAV_EKF_ZERO_FLOW_COUNT) {
            flow_nav_estimator_zero_velocity();
            DRV_NAV_EKF_GetDiagnostics(&flow_nav_ctx.ekf,
                                       &flow_nav_ctx.diagnostics);
        }
    } else if (new_flow_sample != 0U) {
        flow_nav_ctx.zero_flow_count = 0U;
    }
    flow_nav_ctx.vel_m_s[0] = flow_nav_ctx.diagnostics.vel_m_s[0];
    flow_nav_ctx.vel_m_s[1] = flow_nav_ctx.diagnostics.vel_m_s[1];

    if (flow_accepted != 0U) {
        flow_nav_integrate_position();
    }

    return flow_accepted;
}

void SVC_FlowNav_GetVelocity(float *vx_m_s, float *vy_m_s)
{
    if (vx_m_s != NULL) {
        *vx_m_s = flow_nav_ctx.vel_m_s[0];
    }
    if (vy_m_s != NULL) {
        *vy_m_s = flow_nav_ctx.vel_m_s[1];
    }
}

void SVC_FlowNav_GetPosition(float *x_m, float *y_m)
{
    if (x_m != NULL) {
        *x_m = flow_nav_ctx.position_m[0];
    }
    if (y_m != NULL) {
        *y_m = flow_nav_ctx.position_m[1];
    }
}

void SVC_FlowNav_GetDisplacement(float *dx_m, float *dy_m)
{
    if (dx_m != NULL) {
        *dx_m = flow_nav_ctx.displacement_m[0];
    }
    if (dy_m != NULL) {
        *dy_m = flow_nav_ctx.displacement_m[1];
    }
}

void SVC_FlowNav_GetDiagOdometer(float raw_m[2], float filt_m[2], uint32_t *steps)
{
    if (raw_m != NULL) {
        raw_m[0] = flow_nav_ctx.diag_raw_m[0];
        raw_m[1] = flow_nav_ctx.diag_raw_m[1];
    }
    if (filt_m != NULL) {
        filt_m[0] = flow_nav_ctx.diag_filt_m[0];
        filt_m[1] = flow_nav_ctx.diag_filt_m[1];
    }
    if (steps != NULL) {
        *steps = flow_nav_ctx.diag_steps;
    }
}

uint32_t SVC_FlowNav_GetIntegratedStepCount(void)
{
    return flow_nav_ctx.integrated_step_count;
}

uint32_t SVC_FlowNav_GetLastIntegrationDtUs(void)
{
    return flow_nav_ctx.last_integration_dt_us;
}

void SVC_FlowNav_ResetEstimator(void)
{
    flow_nav_estimator_reset();
    SVC_FlowNav_ResetPosition();
}

void SVC_FlowNav_ResetPosition(void)
{
    /*
     * 只清控制用的位置状态，不碰累计位移。
     *
     * 稳定环在低油门直通分支里每个控制周期都调一次 ResetEstimator，位置本来就该
     * 在那里归零（外环重新以当前点为原点）。但累计位移是里程计，被这条每拍都走的
     * 路径连带清掉就永远累不起来——地面上盯着这个数只会看到恒 0。
     * 里程计只在传感器重新初始化或显式请求时清零。
     */
    flow_nav_ctx.position_m[0] = 0.0f;
    flow_nav_ctx.position_m[1] = 0.0f;
}

void SVC_FlowNav_ResetDisplacement(void)
{
    flow_nav_ctx.displacement_m[0] = 0.0f;
    flow_nav_ctx.displacement_m[1] = 0.0f;
    flow_nav_ctx.integrated_step_count = 0U;
}

void SVC_FlowNav_GetEkfDiagnostics(DRV_NAV_EKF_Diagnostics *diagnostics)
{
    if (diagnostics == NULL) {
        return;
    }

    *diagnostics = flow_nav_ctx.diagnostics;
}

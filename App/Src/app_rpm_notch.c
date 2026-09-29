/*
 * app_rpm_notch.c —— 按转速跟踪的陀螺陷波：策略层。设计意图写在 App/Inc/app_rpm_notch.h。
 *
 * 这里补三处"为什么这么写"：
 *
 * 1. **按 IMU 样本滤，不按控制拍滤。** IMU 样本在芯片时钟上等间隔，固定系数才名副
 *    其实；控制拍在 2～3 ms 之间抖，按 500 Hz 设计的陷波真实中心会在 72～107 Hz 间漂。
 *    系数每个控制拍按最新转速重算一次，同一个任务里用，不存在竞争。
 *
 * 2. **转速新鲜度按有符号差算。** 控制拍的 now 取在提交之前，提交里收割回包时打的
 *    sample_ms 可能是 now+1；无符号相减会回绕成约 4e9 被判过期。所以 Tick 先取快照、
 *    后取时间，年龄再做一次有符号保护。
 *
 * 3. **采样率用实测值。** 名义 ODR 只是种子；芯片振荡器 3% 的误差会 1:1 挪走中心
 *    （107.5 Hz 上就是 3.2 Hz，Q5 下只剩 −10 dB）。IMU 时间戳来自晶振驱动的定时器，
 *    按样本间隔做 EMA 就把这项误差消掉；估计偏离名义 5% 以上判 fs_bad，陷波淡出直通。
 */

#include "app_rpm_notch.h"

#include "bsp_critical.h"
#include "bsp_dshot_rx.h"
#include "bsp_imu_rate.h"
#include "drv_dshot_telemetry.h"
#include "drv_rpm_notch.h"
#include "svc_timestamp.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define RPM_NOTCH_AGE_NEVER_MS   9999U
#define RPM_NOTCH_TICK_DT_MAX_S  0.02f
#define RPM_NOTCH_FS_OK_REL      0.04f     /* fs_bad 的恢复门：回到 4% 以内才解除（迟滞） */
#define RPM_NOTCH_EMA_GAIN       (1.0f / (float)(1UL << APP_RPM_NOTCH_FS_EMA_SHIFT))
#define RPM_NOTCH_READ_RETRIES   8U
#define RPM_NOTCH_DT_RESET_US    1000000ULL   /* 超过 1 s 的间隔直接按复位处理（也免得 64 位转浮点） */

typedef enum {
    RPM_NOTCH_SAMPLE_RESET = 0,
    RPM_NOTCH_SAMPLE_NORMAL,
    RPM_NOTCH_SAMPLE_GAP1
} RpmNotchSampleKind;

/* 只有 StabilizerTask 读写；其它任务只经发布副本与待生效配置交互。 */
typedef struct {
    DRV_RpmNotch bank;
    APP_RpmNotchConfig cfg;          /* 生效中的配置 */
    uint32_t applied_seq;
    uint32_t applied_clear_seq;
    APP_RpmNotchState state;
    uint8_t engaged;                 /* idle/tracking/fs_bad：ApplySample 真的过滤 */
    uint8_t fs_bad;
    uint16_t fs_nominal_hz;
    float nominal_dt_us;
    float ema_dt_us;                 /* 实测样本间隔 EMA；0 = 未知 */
    float fs_est_hz;
    float period_us;                 /* ApplySample 分类用的 P，Tick 里刷新（避免每样本一次除法） */
    uint64_t last_ts_us;
    uint8_t has_last_ts;
    uint8_t reset_request;
    float prev_in[3];
    uint64_t last_tick_us;
    uint8_t has_tick;
    uint32_t accept_erpm[2], accept_ms[2], reject_ms[2];
    uint8_t has_accept[2], has_reject[2];
    uint8_t was_fresh_spinning[2];   /* 上一拍新鲜且在转：据此数 stale */
    uint8_t usable[2];               /* 本拍这一路的转速可以喂给陷波 */
    uint8_t base_h;                  /* 掩码里最低的谐波（1..3）：权重与 spin/tracked 都看它 */
    uint8_t full_band[2];            /* 可用且最低谐波在全权重频段：spin/tracked 统计 */
    APP_RpmNotchMotors motors;
    /* 滤波组重建（改配置、重新接入）前把它的计数并进来，状态里报累计值。 */
    uint32_t nonfinite_base, slewclamp_base, wdog_base;
    uint32_t stale, gap1, reset, reject, samples, spin, tracked;
    uint32_t apply_us_sum, apply_count, apply_us_max, tick_us_sum, tick_count, tick_us_max;
} RpmNotchContext;

static RpmNotchContext rpm_notch;
/*
 * 发布副本用双缓冲 + 序号（不关中断）：控制拍写"另一份"、写完才把序号 +1；读者按序号取
 * 那份，拷完**序号没动**才收。序号动过就重拷：再下一次发布写的正是读者这份，而它写的
 * 过程中序号只前进了 1——"前进不到 2 就收"会收下半新半旧的拷贝。序号没动时写者最多
 * 正在写另一份，读者这份一定完整。
 * 整份在 -O0 下拷一遍要两微秒多，不该压在 1 kHz 控制环的抖动上；命令任务与控制拍同优先级、
 * 会互相时间片抢占，所以也不用单缓冲序号锁（写者被切走在半路，读者就一直读不到完整的）。
 */
static APP_RpmNotchStatus rpm_notch_status_buf[2];
static uint32_t rpm_notch_status_cfg_seq[2];          /* 各份副本对应的配置序号 */
static volatile uint32_t rpm_notch_status_seq;
static APP_RpmNotchConfig rpm_notch_pending = {
    APP_RPM_NOTCH_DEFAULT_ENABLE, APP_RPM_NOTCH_DEFAULT_POLE_PAIRS, APP_RPM_NOTCH_DEFAULT_HARMONICS,
    APP_RPM_NOTCH_DEFAULT_Q, APP_RPM_NOTCH_DEFAULT_MIN_HZ, APP_RPM_NOTCH_DEFAULT_FADE_HZ
};
static volatile uint32_t rpm_notch_pending_seq;      /* 命令任务每提交一次加 1 */
static volatile uint32_t rpm_notch_clear_seq;

/* ------------------------------------------------------------------ 配置 */

void APP_RpmNotch_DefaultConfig(APP_RpmNotchConfig *cfg)
{
    if (cfg == NULL) {
        return;
    }
    cfg->enable = APP_RPM_NOTCH_DEFAULT_ENABLE;
    cfg->pole_pairs = APP_RPM_NOTCH_DEFAULT_POLE_PAIRS;
    cfg->harmonic_mask = APP_RPM_NOTCH_DEFAULT_HARMONICS;
    cfg->q = APP_RPM_NOTCH_DEFAULT_Q;
    cfg->min_hz = APP_RPM_NOTCH_DEFAULT_MIN_HZ;
    cfg->fade_hz = APP_RPM_NOTCH_DEFAULT_FADE_HZ;
}

uint8_t APP_RpmNotch_ConfigValid(const APP_RpmNotchConfig *cfg)
{
    if (cfg == NULL) {
        return 0U;
    }
    /* 极对数 0 = 旁路（合法但陷波不启用）；命令面只收 1..30。 */
    if ((cfg->enable > 1U) || (cfg->pole_pairs > APP_RPM_NOTCH_POLE_PAIRS_MAX) ||
        (cfg->harmonic_mask == 0U) || (cfg->harmonic_mask > 0x07U)) {
        return 0U;
    }
    if ((isfinite(cfg->q) == 0) || (cfg->q < APP_RPM_NOTCH_Q_LO) || (cfg->q > APP_RPM_NOTCH_Q_HI) ||
        (isfinite(cfg->min_hz) == 0) || (cfg->min_hz < APP_RPM_NOTCH_MIN_HZ_LO) ||
        (cfg->min_hz > APP_RPM_NOTCH_MIN_HZ_HI) ||
        (isfinite(cfg->fade_hz) == 0) || (cfg->fade_hz < APP_RPM_NOTCH_FADE_HZ_LO) ||
        (cfg->fade_hz > APP_RPM_NOTCH_FADE_HZ_HI) ||
        ((cfg->min_hz + cfg->fade_hz) > APP_RPM_NOTCH_MIN_PLUS_FADE_MAX)) {
        return 0U;
    }
    return 1U;
}

uint8_t APP_RpmNotch_RequestConfig(const APP_RpmNotchConfig *cfg)
{
    uint32_t lock;

    if (APP_RpmNotch_ConfigValid(cfg) == 0U) {
        return 0U;
    }
    lock = BSP_Critical_Enter();
    rpm_notch_pending = *cfg;
    rpm_notch_pending_seq = rpm_notch_pending_seq + 1U;
    BSP_Critical_Exit(lock);
    return 1U;
}

void APP_RpmNotch_GetConfig(APP_RpmNotchConfig *out)
{
    uint32_t lock;

    if (out == NULL) {
        return;
    }
    /* 待生效的那份就是最新请求；生效后两者相同。 */
    lock = BSP_Critical_Enter();
    *out = rpm_notch_pending;
    BSP_Critical_Exit(lock);
}

void APP_RpmNotch_ClearStats(void)
{
    uint32_t lock = BSP_Critical_Enter();

    rpm_notch_clear_seq = rpm_notch_clear_seq + 1U;
    BSP_Critical_Exit(lock);
}

const char *APP_RpmNotch_StateName(APP_RpmNotchState state)
{
    switch (state) {
    case APP_RPM_NOTCH_OFF:         return "off";
    case APP_RPM_NOTCH_UNAVAILABLE: return "unavailable";
    case APP_RPM_NOTCH_FS_UNKNOWN:  return "fs_unknown";
    case APP_RPM_NOTCH_FS_BAD:      return "fs_bad";
    case APP_RPM_NOTCH_IDLE:        return "idle";
    case APP_RPM_NOTCH_TRACKING:    return "tracking";
    default:                        return "unknown";
    }
}

/* ------------------------------------------------------------------ 内部 */

/*
 * 重建滤波组：Q/谐波/频段取自生效配置，爬升/限速/看门狗取驱动默认。计数先并进累计值。
 * 顺带记下掩码里最低的谐波（掩码已校验为 1..7）：HARM 2/4/6 没有 1x，权重和跟踪统计
 * 若死盯 1x 槽，陷波明明在滤也会报 0。
 */
static void rpm_notch_bank_init(void)
{
    const uint8_t mask = rpm_notch.cfg.harmonic_mask;
    DRV_RpmNotchConfig drv;

    rpm_notch.base_h = ((mask & 0x01U) != 0U) ? 1U : (((mask & 0x02U) != 0U) ? 2U : 3U);

    rpm_notch.nonfinite_base += rpm_notch.bank.nonfinite_count;
    rpm_notch.slewclamp_base += rpm_notch.bank.slew_clamp_count;
    rpm_notch.wdog_base += rpm_notch.bank.watchdog_count;
    DRV_RpmNotch_DefaultConfig(&drv);
    drv.harmonic_mask = rpm_notch.cfg.harmonic_mask;
    drv.q = rpm_notch.cfg.q;
    drv.min_hz = rpm_notch.cfg.min_hz;
    drv.fade_hz = rpm_notch.cfg.fade_hz;
    (void)DRV_RpmNotch_Init(&rpm_notch.bank, &drv);
}

static void rpm_notch_clear_counters(void)
{
    rpm_notch.bank.nonfinite_count = 0U;
    rpm_notch.bank.slew_clamp_count = 0U;
    rpm_notch.bank.watchdog_count = 0U;
    rpm_notch.nonfinite_base = 0U;
    rpm_notch.slewclamp_base = 0U;
    rpm_notch.wdog_base = 0U;
    rpm_notch.stale = 0U;
    rpm_notch.gap1 = 0U;
    rpm_notch.reset = 0U;
    rpm_notch.reject = 0U;
    rpm_notch.samples = 0U;
    rpm_notch.spin = 0U;
    rpm_notch.tracked = 0U;
    rpm_notch.apply_us_sum = 0U;
    rpm_notch.apply_count = 0U;
    rpm_notch.apply_us_max = 0U;
    rpm_notch.tick_us_sum = 0U;
    rpm_notch.tick_count = 0U;
    rpm_notch.tick_us_max = 0U;
}

/* 配置 + 输入 → 状态（不含 tracking：那要看这一拍的目标权重）。命令回复的预测也用它。 */
static APP_RpmNotchState rpm_notch_base_state(const APP_RpmNotchConfig *cfg, uint8_t available,
                                              uint16_t nominal_hz, uint8_t fs_bad)
{
    if (cfg->enable == 0U) {
        return APP_RPM_NOTCH_OFF;
    }
    if (available == 0U) {
        return APP_RPM_NOTCH_UNAVAILABLE;
    }
    if (nominal_hz == 0U) {
        return APP_RPM_NOTCH_FS_UNKNOWN;
    }
    return (fs_bad != 0U) ? APP_RPM_NOTCH_FS_BAD : APP_RPM_NOTCH_IDLE;
}

/*
 * 一路电调的转速判定。新鲜 = 构建有回传、这一路拿到过回包、年龄 <= 20 ms；
 * 电调明确回报"未旋转"是新鲜的有效回包（目标权重立即归零）。超过 150k 的 eRPM
 * 按坏帧拒收，最后一次接受的值在它自己的 20 ms 内继续顶用。
 */
static void rpm_notch_update_motor(uint32_t m, const BSP_DShotRxSnapshot *rx, uint32_t now_ms)
{
    const int32_t diff = (int32_t)(now_ms - rx->sample_ms[m]);
    const uint32_t age = (diff < 0) ? 0U : (uint32_t)diff;
    const uint8_t fresh = ((rx->available != 0U) && (rx->valid[m] != 0U) &&
                           (age <= APP_RPM_NOTCH_STALE_MS)) ? 1U : 0U;
    const uint8_t spinning = ((fresh != 0U) && (rx->not_spinning[m] == 0U)) ? 1U : 0U;
    const float full_hz = rpm_notch.cfg.min_hz + rpm_notch.cfg.fade_hz;
    uint32_t erpm = 0U;
    uint8_t usable = 0U;
    float hz = 0.0f, base_hz;

    if (spinning != 0U) {
        if (rx->erpm[m] > APP_RPM_NOTCH_ERPM_MAX) {
            if ((rpm_notch.has_reject[m] == 0U) || (rpm_notch.reject_ms[m] != rx->sample_ms[m])) {
                rpm_notch.reject++;   /* 同一帧跨拍只数一次 */
                rpm_notch.reject_ms[m] = rx->sample_ms[m];
                rpm_notch.has_reject[m] = 1U;
            }
            if ((rpm_notch.has_accept[m] != 0U) &&
                ((int32_t)(now_ms - rpm_notch.accept_ms[m]) <= (int32_t)APP_RPM_NOTCH_STALE_MS)) {
                erpm = rpm_notch.accept_erpm[m];
                usable = 1U;
            }
        } else {
            rpm_notch.accept_erpm[m] = rx->erpm[m];
            rpm_notch.accept_ms[m] = rx->sample_ms[m];
            rpm_notch.has_accept[m] = 1U;
            erpm = rx->erpm[m];
            usable = 1U;
        }
    }
    if ((rpm_notch.was_fresh_spinning[m] != 0U) && (fresh == 0U)) {
        rpm_notch.stale++;
    }
    rpm_notch.was_fresh_spinning[m] = spinning;
    if (usable != 0U) {
        hz = (float)DRV_DShotTelem_MechanicalRpm(erpm, rpm_notch.cfg.pole_pairs) / 60.0f;
    }
    rpm_notch.usable[m] = usable;
    /* 最低谐波落在全权重频段（低端 min+fade 起、高端 0.40·fs 止）才算"该满权重地滤"。 */
    base_hz = (float)rpm_notch.base_h * hz;
    rpm_notch.full_band[m] = ((usable != 0U) && (base_hz >= full_hz) &&
                              (!(rpm_notch.fs_est_hz > 0.0f) ||
                               (base_hz <= (DRV_RPM_NOTCH_TOP_FADE_START * rpm_notch.fs_est_hz)))) ? 1U : 0U;
    rpm_notch.motors.erpm[m] = (usable != 0U) ? erpm : 0U;
    rpm_notch.motors.hz[m] = hz;
    rpm_notch.motors.age_ms[m] = ((rx->available == 0U) || (rx->valid[m] == 0U)) ? RPM_NOTCH_AGE_NEVER_MS :
                                 ((age > RPM_NOTCH_AGE_NEVER_MS) ? RPM_NOTCH_AGE_NEVER_MS : age);
    rpm_notch.motors.fresh[m] = fresh;
    rpm_notch.motors.spinning[m] = spinning;
}

static void rpm_notch_publish(void)
{
    const uint32_t next = rpm_notch_status_seq + 1U;
    APP_RpmNotchStatus *s = &rpm_notch_status_buf[next & 1U];
    const uint8_t engaged = rpm_notch.engaged;

    s->cfg = rpm_notch.cfg;
    s->motors = rpm_notch.motors;
    s->state = rpm_notch.state;
    s->fs_nominal_hz = rpm_notch.fs_nominal_hz;
    s->fs_hz = rpm_notch.fs_est_hz;
    for (uint8_t m = 0U; m < 2U; ++m) {
        s->weight_base[m] = (engaged != 0U) ? DRV_RpmNotch_SlotWeight(&rpm_notch.bank, m, rpm_notch.base_h) : 0.0f;
    }
    s->active_slots = (engaged != 0U) ? DRV_RpmNotch_ActiveSlots(&rpm_notch.bank) : 0U;
    s->stale = rpm_notch.stale;
    s->gap1 = rpm_notch.gap1;
    s->reset = rpm_notch.reset;
    s->nonfinite = rpm_notch.nonfinite_base + rpm_notch.bank.nonfinite_count;
    s->slewclamp = rpm_notch.slewclamp_base + rpm_notch.bank.slew_clamp_count;
    s->reject = rpm_notch.reject;
    s->wdog = rpm_notch.wdog_base + rpm_notch.bank.watchdog_count;
    s->samples = rpm_notch.samples;
    s->spin = rpm_notch.spin;
    s->tracked = rpm_notch.tracked;
    s->apply_us_sum = rpm_notch.apply_us_sum;
    s->apply_count = rpm_notch.apply_count;
    s->apply_us_max = rpm_notch.apply_us_max;
    s->tick_us_sum = rpm_notch.tick_us_sum;
    s->tick_count = rpm_notch.tick_count;
    s->tick_us_max = rpm_notch.tick_us_max;
    rpm_notch_status_cfg_seq[next & 1U] = rpm_notch.applied_seq;
    BSP_Critical_MemoryBarrier();
    rpm_notch_status_seq = next;
}

/* 取一份完整的发布副本（任意任务）。拷的途中控制拍发布过就重拷（2～3 ms 一次，拷两微秒，极少重拷）。 */
static void rpm_notch_read_status(APP_RpmNotchStatus *out, uint32_t *cfg_seq)
{
    uint32_t lock;

    for (uint32_t attempt = 0U; attempt < RPM_NOTCH_READ_RETRIES; ++attempt) {
        const uint32_t before = rpm_notch_status_seq;

        BSP_Critical_MemoryBarrier();
        *out = rpm_notch_status_buf[before & 1U];
        *cfg_seq = rpm_notch_status_cfg_seq[before & 1U];
        BSP_Critical_MemoryBarrier();
        if (rpm_notch_status_seq == before) {
            return;
        }
    }
    /* 兜底（实际碰不到）：关中断时写者不会前进；它若停在半路，写的也是另一份。 */
    lock = BSP_Critical_Enter();
    *out = rpm_notch_status_buf[rpm_notch_status_seq & 1U];
    *cfg_seq = rpm_notch_status_cfg_seq[rpm_notch_status_seq & 1U];
    BSP_Critical_Exit(lock);
}

static void rpm_notch_note_time(uint64_t t0, uint32_t *sum, uint32_t *count, uint32_t *max)
{
    const uint64_t t1 = SVC_Timestamp_Us();
    const uint32_t elapsed = (t1 > t0) ? (uint32_t)(t1 - t0) : 0U;

    *sum += elapsed;
    *count += 1U;
    if (elapsed > *max) {
        *max = elapsed;
    }
}

/* ------------------------------------------------------------------ 运行面 */

void APP_RpmNotch_Init(void)
{
    uint32_t lock;

    memset(&rpm_notch, 0, sizeof(rpm_notch));
    lock = BSP_Critical_Enter();
    APP_RpmNotch_DefaultConfig(&rpm_notch_pending);   /* 阶段一只在 RAM：上电回到编译期默认 */
    rpm_notch.cfg = rpm_notch_pending;
    rpm_notch.applied_seq = rpm_notch_pending_seq;
    rpm_notch.applied_clear_seq = rpm_notch_clear_seq;
    BSP_Critical_Exit(lock);
    rpm_notch_bank_init();
    rpm_notch.state = APP_RPM_NOTCH_OFF;
    rpm_notch_publish();
}

void APP_RpmNotch_ResetState(void)
{
    rpm_notch.reset_request = 1U;
}

void APP_RpmNotch_Tick(void)
{
    const uint64_t t0 = SVC_Timestamp_Us();
    BSP_DShotRxSnapshot rx;
    APP_RpmNotchState state;
    uint32_t lock, seq, clear, now_ms;
    uint16_t nominal;
    float dt_s = 0.0f;
    uint8_t engaged;

    /* 1. 命令任务交来的配置 / 清零请求：本函数是生效配置的唯一写者。 */
    lock = BSP_Critical_Enter();
    seq = rpm_notch_pending_seq;
    clear = rpm_notch_clear_seq;
    if (seq != rpm_notch.applied_seq) {
        rpm_notch.cfg = rpm_notch_pending;
    }
    BSP_Critical_Exit(lock);
    if (seq != rpm_notch.applied_seq) {
        rpm_notch.applied_seq = seq;
        rpm_notch_bank_init();
        rpm_notch.engaged = 0U;
    }
    if (clear != rpm_notch.applied_clear_seq) {
        rpm_notch.applied_clear_seq = clear;
        rpm_notch_clear_counters();
    }

    /* 2. 名义 ODR：换了芯片/档位就重新播种估计，滤波组回稳态。 */
    nominal = BSP_IMU_GetGyroOdrHz();
    if (nominal != rpm_notch.fs_nominal_hz) {
        rpm_notch.fs_nominal_hz = nominal;
        rpm_notch.nominal_dt_us = (nominal != 0U) ? (1.0e6f / (float)nominal) : 0.0f;
        rpm_notch.ema_dt_us = rpm_notch.nominal_dt_us;
        rpm_notch.fs_bad = 0U;
        DRV_RpmNotch_Reset(&rpm_notch.bank);
    }
    rpm_notch.fs_est_hz = (rpm_notch.ema_dt_us > 0.0f) ? (1.0e6f / rpm_notch.ema_dt_us) : 0.0f;
    rpm_notch.period_us = rpm_notch.ema_dt_us;
    if ((nominal != 0U) && (rpm_notch.fs_est_hz > 0.0f)) {
        const float rel = fabsf((rpm_notch.fs_est_hz / (float)nominal) - 1.0f);

        if (rel > APP_RPM_NOTCH_FS_BAD_REL) {
            rpm_notch.fs_bad = 1U;
        } else if (rel < RPM_NOTCH_FS_OK_REL) {
            rpm_notch.fs_bad = 0U;
        }
    }

    /* 3. 转速：先取快照、后取时间（见文件头第 2 条）。 */
    BSP_DShotRx_GetSnapshot(&rx);
    now_ms = SVC_Timestamp_Ms();
    for (uint32_t m = 0U; m < 2U; ++m) {
        rpm_notch_update_motor(m, &rx, now_ms);
    }
    rpm_notch.motors.available = rx.available;

    if (rpm_notch.has_tick != 0U) {
        const uint64_t elapsed_us = (t0 > rpm_notch.last_tick_us) ? (t0 - rpm_notch.last_tick_us) : 0U;

        dt_s = (elapsed_us >= (uint64_t)(RPM_NOTCH_TICK_DT_MAX_S * 1.0e6f)) ? RPM_NOTCH_TICK_DT_MAX_S :
               ((float)(uint32_t)elapsed_us * 1.0e-6f);
    }
    rpm_notch.last_tick_us = t0;
    rpm_notch.has_tick = 1U;

    /* 4. 状态与滤波组。fs_bad 时照样跑，只是所有目标归零，让权重淡出。 */
    state = rpm_notch_base_state(&rpm_notch.cfg, rx.available, nominal, rpm_notch.fs_bad);
    engaged = ((state == APP_RPM_NOTCH_IDLE) || (state == APP_RPM_NOTCH_FS_BAD)) ? 1U : 0U;
    if ((engaged != 0U) && (rpm_notch.engaged == 0U)) {
        rpm_notch_bank_init();   /* 重新接入：不带着上次断开时的状态与权重 */
    }
    rpm_notch.engaged = engaged;
    if (engaged != 0U) {
        uint8_t targeted = 0U;

        DRV_RpmNotch_SetFs(&rpm_notch.bank, rpm_notch.fs_est_hz);
        for (uint8_t m = 0U; m < 2U; ++m) {
            const uint8_t feed = ((rpm_notch.usable[m] != 0U) && (state != APP_RPM_NOTCH_FS_BAD) &&
                                  (rpm_notch.cfg.pole_pairs > 0U)) ? 1U : 0U;

            DRV_RpmNotch_SetMotor(&rpm_notch.bank, m, rpm_notch.motors.hz[m], feed, dt_s);
        }
        for (uint32_t s = 0U; s < DRV_RPM_NOTCH_SLOTS; ++s) {
            if ((rpm_notch.bank.slot[s].target > 0.0f) || (rpm_notch.bank.slot[s].active != 0U)) {
                targeted = 1U;
            }
        }
        if ((state == APP_RPM_NOTCH_IDLE) && (targeted != 0U)) {
            state = APP_RPM_NOTCH_TRACKING;
        }
    }
    rpm_notch.state = state;

    /* 5. 发布（关着也发：转速与计数照样看得到）。 */
    rpm_notch_publish();
    rpm_notch_note_time(t0, &rpm_notch.tick_us_sum, &rpm_notch.tick_count, &rpm_notch.tick_us_max);
}

void APP_RpmNotch_ApplySample(const float in_rad_s[3], float out_rad_s[3], uint64_t timestamp_us)
{
    const uint64_t t0 = SVC_Timestamp_Us();
    RpmNotchSampleKind kind = RPM_NOTCH_SAMPLE_RESET;

    if ((in_rad_s == NULL) || (out_rad_s == NULL)) {
        return;
    }
    /* 样本间隔分类：P 是估计周期。一个 1.5～2.5 P 的缺口按丢一个样本补中点。 */
    if ((rpm_notch.has_last_ts != 0U) && (rpm_notch.reset_request == 0U) &&
        (timestamp_us > rpm_notch.last_ts_us) &&
        ((timestamp_us - rpm_notch.last_ts_us) < RPM_NOTCH_DT_RESET_US)) {
        const float dt_us = (float)(uint32_t)(timestamp_us - rpm_notch.last_ts_us);
        const float p = rpm_notch.period_us;

        if (!(p > 0.0f)) {
            kind = RPM_NOTCH_SAMPLE_NORMAL;   /* 周期未知：此时也不会过滤 */
        } else if ((dt_us > (0.5f * p)) && (dt_us <= (1.5f * p))) {
            kind = RPM_NOTCH_SAMPLE_NORMAL;
            if ((rpm_notch.nominal_dt_us > 0.0f) &&
                (fabsf(dt_us - rpm_notch.nominal_dt_us) <=
                 (APP_RPM_NOTCH_FS_ACCEPT_REL * rpm_notch.nominal_dt_us))) {
                rpm_notch.ema_dt_us += (dt_us - rpm_notch.ema_dt_us) * RPM_NOTCH_EMA_GAIN;
            }
        } else if ((dt_us > (1.5f * p)) && (dt_us <= (2.5f * p))) {
            kind = RPM_NOTCH_SAMPLE_GAP1;
        }
    }
    rpm_notch.last_ts_us = timestamp_us;
    rpm_notch.has_last_ts = 1U;
    rpm_notch.reset_request = 0U;
    if (kind == RPM_NOTCH_SAMPLE_RESET) {
        rpm_notch.reset++;
    } else if (kind == RPM_NOTCH_SAMPLE_GAP1) {
        rpm_notch.gap1++;
    }

    if (rpm_notch.engaged == 0U) {
        /* 关闭/不可用：逐位拷贝，和没有这个模块时一模一样。 */
        out_rad_s[0] = in_rad_s[0];
        out_rad_s[1] = in_rad_s[1];
        out_rad_s[2] = in_rad_s[2];
    } else {
        float in[3] = { in_rad_s[0], in_rad_s[1], in_rad_s[2] };

        if (kind == RPM_NOTCH_SAMPLE_RESET) {
            DRV_RpmNotch_Reset(&rpm_notch.bank);
        } else if (kind == RPM_NOTCH_SAMPLE_GAP1) {
            /* 丢的那个样本用中点补一步，保持在采样网格上；输出丢弃。 */
            float mid[3], scratch[3];

            for (uint32_t a = 0U; a < 3U; ++a) {
                mid[a] = 0.5f * (rpm_notch.prev_in[a] + in[a]);
            }
            DRV_RpmNotch_Apply(&rpm_notch.bank, mid, scratch);
        }
        DRV_RpmNotch_Apply(&rpm_notch.bank, in, out_rad_s);
    }
    rpm_notch.prev_in[0] = in_rad_s[0];
    rpm_notch.prev_in[1] = in_rad_s[1];
    rpm_notch.prev_in[2] = in_rad_s[2];

    rpm_notch.samples++;
    if ((rpm_notch.full_band[0] != 0U) || (rpm_notch.full_band[1] != 0U)) {
        uint8_t all_full = rpm_notch.engaged;

        rpm_notch.spin++;
        for (uint8_t m = 0U; m < 2U; ++m) {
            if ((rpm_notch.full_band[m] != 0U) &&
                (DRV_RpmNotch_SlotWeight(&rpm_notch.bank, m, rpm_notch.base_h) < 1.0f)) {
                all_full = 0U;
            }
        }
        if (all_full != 0U) {
            rpm_notch.tracked++;
        }
    }
    rpm_notch_note_time(t0, &rpm_notch.apply_us_sum, &rpm_notch.apply_count, &rpm_notch.apply_us_max);
}

/* ------------------------------------------------------------------ 读出 */

void APP_RpmNotch_GetStatus(APP_RpmNotchStatus *out)
{
    APP_RpmNotchConfig pending;
    uint32_t lock, seq, published;

    if (out == NULL) {
        return;
    }
    rpm_notch_read_status(out, &published);
    lock = BSP_Critical_Enter();
    pending = rpm_notch_pending;
    seq = rpm_notch_pending_seq;
    BSP_Critical_Exit(lock);
    if (seq != published) {
        /*
         * 命令刚提交、控制拍还没取用（最多一拍）：报请求的配置与它将进入的状态，
         * 免得 ON 的回复里还写着 state=off。重新接入的滤波组从零权重起步，所以是 idle。
         */
        out->cfg = pending;
        out->state = rpm_notch_base_state(&pending, out->motors.available, out->fs_nominal_hz,
                                          (out->state == APP_RPM_NOTCH_FS_BAD) ? 1U : 0U);
        out->weight_base[0] = 0.0f;
        out->weight_base[1] = 0.0f;
        out->active_slots = 0U;
    }
}

void APP_RpmNotch_GetMotors(APP_RpmNotchMotors *out)
{
    APP_RpmNotchStatus status;
    uint32_t published;

    if (out == NULL) {
        return;
    }
    rpm_notch_read_status(&status, &published);
    *out = status.motors;
}

#ifndef APP_RPM_NOTCH_H
#define APP_RPM_NOTCH_H

#include <stdint.h>

/*
 * 按电调转速跟踪的陀螺陷波：策略层（作者 2026-09-27 决定）。
 *
 * 解决什么：杆上悬停时 gx/gy 上有约 107.5 Hz 的桨叶振动线（约 0.08 rad/s rms），
 * 经角速度环 P 漏进舵机指令，再被 50 Hz 舵机帧混叠到 7～8 Hz——正落在回路
 * −180° 穿越（约 9.5 Hz）附近。陷波在每个 IMU 样本上把这条线从**控制用**的陀螺里
 * 挖掉；Fusion、alpha/光流导航、IMUCAP、IMU?、vofa、飞行日志 imu、辨识记录仍是原始陀螺。
 *
 * 分工（decoupling-spec）：滤波算法在 Driver/drv_rpm_notch.c；这里只管
 *   - 转速新不新鲜（双向 DShot 回包快照，20 ms 内算新鲜，年龄按有符号差算）；
 *   - 极对数、谐波、Q 等配置的跨任务交接（命令任务写待生效配置，控制拍取用）；
 *   - 采样率估计（名义 ODR 作种子，按 IMU 时间戳 EMA 精修）、丢样与复位处理；
 *   - 计数与耗时，供 `RPMNOTCH ?`。
 *
 * 运行在 StabilizerTask：ApplySample 每个 IMU 样本一次，Tick 每个控制拍一次（提交之后）。
 * **本模块不许包含 app_control.h、不许调 APP_Control_QueueText**（控制拍不打字）；
 * 文字回复全在 app_cmd_rpmnotch.c，由命令任务出。
 *
 * 非双向 DShot 构建（CMake 默认 DSHOT300）没有转速：状态报 unavailable，逐位直通，
 * 不挡解锁。阶段一只在 RAM：重启回到下面的编译期默认值。
 */

#define APP_RPM_NOTCH_DEFAULT_ENABLE      1U      /* 2026-09-28: ON at boot after the rig A/B (107 Hz prop line -37 dB) */
#define APP_RPM_NOTCH_DEFAULT_POLE_PAIRS  7U      /* AEO CRM2413-KV1300; unverified until H1 */
#define APP_RPM_NOTCH_DEFAULT_HARMONICS   0x01U
#define APP_RPM_NOTCH_DEFAULT_Q           3.0f
#define APP_RPM_NOTCH_DEFAULT_MIN_HZ      50.0f
#define APP_RPM_NOTCH_DEFAULT_FADE_HZ     20.0f
#define APP_RPM_NOTCH_STALE_MS            20U
#define APP_RPM_NOTCH_ERPM_MAX            150000U
#define APP_RPM_NOTCH_FS_ACCEPT_REL       0.25f   /* dt within ±25% of nominal feeds the fs EMA */
#define APP_RPM_NOTCH_FS_EMA_SHIFT        10U     /* alpha = 1/1024 */
#define APP_RPM_NOTCH_FS_BAD_REL          0.05f   /* |fs_est/nominal-1| > 5% -> fs_bad (recover below 4%) */

/* 命令面接受的范围（MINHZ+FADEHZ 另要求 <= 300）。 */
#define APP_RPM_NOTCH_POLE_PAIRS_MAX      30U
#define APP_RPM_NOTCH_Q_LO                1.5f    /* 与 DRV_RPM_NOTCH_Q_MIN/MAX 同值 */
#define APP_RPM_NOTCH_Q_HI                10.0f
#define APP_RPM_NOTCH_MIN_HZ_LO           20.0f
#define APP_RPM_NOTCH_MIN_HZ_HI           200.0f
#define APP_RPM_NOTCH_FADE_HZ_LO          5.0f
#define APP_RPM_NOTCH_FADE_HZ_HI          100.0f
#define APP_RPM_NOTCH_MIN_PLUS_FADE_MAX   300.0f

typedef struct { uint8_t enable, pole_pairs, harmonic_mask; float q, min_hz, fade_hz; } APP_RpmNotchConfig;
typedef enum { APP_RPM_NOTCH_OFF=0, APP_RPM_NOTCH_UNAVAILABLE, APP_RPM_NOTCH_FS_UNKNOWN,
               APP_RPM_NOTCH_FS_BAD, APP_RPM_NOTCH_IDLE, APP_RPM_NOTCH_TRACKING } APP_RpmNotchState;
typedef struct {                     /* index = ESC channel-1 (rx index), role-agnostic */
    uint32_t erpm[2];                /* accepted eRPM if fresh && spinning, else 0 */
    float    hz[2];                  /* MechanicalRpm(erpm,pp)/60 */
    uint32_t age_ms[2];              /* capped at 9999; 9999 = never */
    uint8_t  fresh[2], spinning[2], available;
} APP_RpmNotchMotors;
/*
 * weight_base：掩码里最低那个谐波（默认掩码下就是 1x）的当前权重。spin/tracked 也按它算：
 * spin = 至少一路可用、且这个谐波落在全权重频段 [min+fade, 0.40·fs] 的样本；
 * tracked = 其中每一路这样的电机权重都已是 1 的样本。
 */
typedef struct {
    APP_RpmNotchConfig cfg; APP_RpmNotchMotors motors; APP_RpmNotchState state;
    uint16_t fs_nominal_hz; float fs_hz; float weight_base[2]; uint8_t active_slots;
    uint32_t stale, gap1, reset, nonfinite, slewclamp, reject, wdog, samples, spin, tracked;
    uint32_t apply_us_sum, apply_count, apply_us_max, tick_us_sum, tick_count, tick_us_max;
} APP_RpmNotchStatus;

void    APP_RpmNotch_Init(void);
void    APP_RpmNotch_ApplySample(const float in_rad_s[3], float out_rad_s[3], uint64_t timestamp_us); /* StabilizerTask, per IMU sample */
void    APP_RpmNotch_ResetState(void);          /* frame/calibration reset: next sample is treated as first */
void    APP_RpmNotch_Tick(void);                /* StabilizerTask, once per control tick after commit */
void    APP_RpmNotch_GetMotors(APP_RpmNotchMotors *out);   /* same snapshot as GetStatus, any task */
void    APP_RpmNotch_GetStatus(APP_RpmNotchStatus *out);   /* double-buffered snapshot, any task */
void    APP_RpmNotch_GetConfig(APP_RpmNotchConfig *out);   /* pending config if any, else active */
void    APP_RpmNotch_DefaultConfig(APP_RpmNotchConfig *cfg);
uint8_t APP_RpmNotch_ConfigValid(const APP_RpmNotchConfig *cfg);
uint8_t APP_RpmNotch_RequestConfig(const APP_RpmNotchConfig *cfg); /* command task; applied at next Tick */
void    APP_RpmNotch_ClearStats(void);
/* 状态名（off/unavailable/fs_unknown/fs_bad/idle/tracking），命令面与溯源行共用。 */
const char *APP_RpmNotch_StateName(APP_RpmNotchState state);
/* implemented in app_cmd_rpmnotch.c */
uint8_t APP_RpmNotch_Command(char **tokens, uint32_t count);
void    APP_RpmNotch_ReportProvenance(uint16_t run_id);

#endif /* APP_RPM_NOTCH_H */

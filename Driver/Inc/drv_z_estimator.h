#ifndef DRV_Z_ESTIMATOR_H
#define DRV_Z_ESTIMATOR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 竖直通道估计器：高度 z / 垂直速度 vz / 竖直加速度零偏 b 的三阶互补滤波
 * （等价于三个极点重合的定常 α-β-γ 滤波器）。纯算术：无寄存器、无 RTOS、无 I/O、
 * 无文件级可变量，状态在调用者传入的结构体里（解耦规范 D3-3），宿主可直接单测。
 *
 * ──────────────── 为什么要它 ────────────────
 *
 * 生产 vz 全来自测距：svc_flow_nav 先对高度做 EMA（α=0.35），再差分、再对速度做 EMA
 * （α=0.2）。2026-09-30 槽式台架实录里它相对 IMU 积分速度滞后约 92 ms、槽底静止噪声约
 * ±0.1 m/s，而电机 t63≈65 ms——这段滞后卡住了高度环带宽。IMU 竖直加速度没有这段滞后
 * 但积分会漂，测距不漂但慢且有噪声：高频听 IMU、低频听测距，加速度零偏由测距慢慢校出来。
 *
 * ──────────────── 模型与离散化 ────────────────
 *
 * 预测（每个控制拍，Δt = 真实时间戳差，D2-3）：
 *     a  = a_up − b          （加速度无效的拍 a = 0，匀速外推）
 *     z += vz·Δt + ½·a·Δt²;  vz += a·Δt
 * 校正（每个新测距样本）：
 *     e  = h_meas − ẑ(t_s − D)       ẑ(·) 查历史，t_s = 样本到达时刻，D = 测距延迟
 *     p  = exp(−T/τ)，T = 与上一个被接收样本的采样间隔
 *     δz = (1 − p³)·e
 *     δv = 1.5·(1 − p)²·(1 + p)/T · e
 *     δb = −(1 − p)³/T² · e
 * 这组增益让"样本间隔为 T 的离散误差系统"三个极点恰好都在 p（推导与数值核对见
 * tests/test_z_estimator.py）；T ≪ τ 时退化为连续三阶互补 3/τ、3/τ²、1/τ³
 * （ArduPilot AP_InertialNav 同形）。测距到样率会抖（约 11 ms，偶有 20 ms），按每个样本
 * 自己的 T 现算，不假定固定间隔。
 *
 * ──────────────── 延迟补偿 ────────────────
 *
 * 测距读数对应的是 D 以前的高度。新息用历史里"那个时刻的估计"算，修正量视作在延迟时刻
 * t_d 施加，再按模型传播到现在加到当前状态（L = 现在 − t_d）：
 *     z += δz + δv·L − ½·δb·L²;  vz += δv − δb·L;  b += δb
 * 历史里每一条也按同一公式平移（代入它自己离 t_d 的时间差），下一次查历史时已包含这次
 * 修正——否则 D 内相继到来的几个样本会把同一份误差重复修好几遍，D 一大就振荡。
 * 线性系统下这与"在延迟时刻跑滤波器、再用缓存的加速度推到现在"完全等价，所以误差
 * 动态与不带延迟时一样，稳定性不随 D 变差。
 *
 * ──────────────── 门限、重置与有效性 ────────────────
 *
 * |e| > gate 的样本拒收（单个尖刺）；连续拒收 reject_reset 个则认定地形/读数确实跳了，
 * 把高度（连同历史）直接平移到测量值，速度与零偏不动。距上一个被接收样本的采样时刻
 * 超过 timeout 即标无效；无效状态下来的第一个样本不过门限，以它为起点重新起步
 * （z = 测量值、vz = 0、零偏保留——零偏是 IMU 的性质，与测距断没断无关）。
 *
 * ──────────────── 契约（D4-3） ────────────────
 *
 * z：测距高度 [m]，正值 = 离地更高（与 svc_flow_nav 的 height_raw_m 同口径，未做倾角/
 * 杆臂补偿）。vz [m/s]、a_up / accel [m/s²]：竖直向上为正，a_up 已去掉重力。
 * 时间：Predict 的 Δt 与 Correct 的样本年龄都是秒；Tick 用两个钟——now_us 只用来算 Δt
 * （svc_timestamp 的 µs 时基），now_ms 与 range_sample_ms 必须是同一个 ms 钟（测距样本
 * 时间戳所在的 HAL tick），两者只相减、不跨钟比较。
 *
 * 默认参数的定标依据见 data/analysis/sysid-rig-params/2026-09-30/z_estimator_replay.txt
 * （把本文件编成宿主库回放 09-30 ALT 实录）。
 */

#define DRV_ZEST_HISTORY            64U       /* 500 Hz 下 128 ms，1 kHz 下 64 ms */
#define DRV_ZEST_DT_MAX_S           0.05f     /* 单拍 Δt 上限；更长的空档按此截断 */
#define DRV_ZEST_MEAS_T_MIN_S       0.002f    /* 样本间隔 T 的下限（同拍重复样本不至于除零） */
#define DRV_ZEST_MEAS_T_MAX_S       0.05f     /* 样本间隔 T 的上限（空档后第一个样本不致一步修过头） */
#define DRV_ZEST_BIAS_LIMIT_M_S2    3.0f      /* 零偏估计的饱和，防发散 */

/* 参数范围：越界的整组拒收（SetParams/Init 返回 0）。 */
#define DRV_ZEST_TAU_MIN_S          0.05f
#define DRV_ZEST_TAU_MAX_S          5.0f
#define DRV_ZEST_DELAY_MAX_S        0.08f     /* 须 ≤ 历史覆盖（500 Hz 下 128 ms）减样本年龄 */
#define DRV_ZEST_GATE_MIN_M         0.01f
#define DRV_ZEST_GATE_MAX_M         2.0f
#define DRV_ZEST_TIMEOUT_MIN_S      0.02f
#define DRV_ZEST_TIMEOUT_MAX_S      1.0f

/*
 * 默认值（2026-09-30 槽式台架 8 轮 ALT 实录回放，数字见 z_estimator_replay.txt）：
 * τ   0.25 s：槽底静止时 az 在 0.3–3 Hz 只有约 0.036 m/s² RMS，同频段 TOF 二阶导却有
 *            0.18–0.25 m/s²——静止时 IMU 比 TOF 干净得多；τ 从 0.1 加到 0.25，槽底 vz 标准差
 *            0.027 → 0.012 m/s、运动窗 RMS 0.069 → 0.057 m/s，再大收益变小（0.4 时 0.0115 /
 *            0.055），而 az 在运动中有 0.3–0.5 m/s² 的慢误差，τ 越大 vz 被它带偏越久。
 * D   30 ms：快速运动样本的新息 RMS 在 D = 30–40 ms 最小；IMU/TOF 相关较好的两轮（041426、
 *            042331）在加速度层面 TOF 落后 IMU 31 / 22 ms。
 * 门限 0.04 m：8 轮干净样本的新息最大 10.5–16.8 mm（RMS 约 3.5 mm），041445 t≈12.8 s 的
 *            TOF 尖刺（IMU 同期无对应加速度）新息 42 / 75 / 51 mm，0.04 拒掉这三个。
 * 连续 5 个拒收（约 55 ms）才认定读数真跳了；超时 0.1 s 与 SVC_FLOW_NAV_TIMEOUT_MS 一致。
 */
#define DRV_ZEST_TAU_DEFAULT_S           0.25f
#define DRV_ZEST_DELAY_DEFAULT_S         0.030f
#define DRV_ZEST_GATE_DEFAULT_M          0.04f
#define DRV_ZEST_REJECT_RESET_DEFAULT    5U
#define DRV_ZEST_TIMEOUT_DEFAULT_S       0.10f

typedef struct {
    float   tau_s;          /* 互补滤波时间常数：三个极点都在 −1/τ */
    float   delay_s;        /* 测距读数相对 IMU 的延迟 D */
    float   gate_m;         /* 新息门限 */
    uint8_t reject_reset;   /* 连续拒收这么多个就重置到测量值（≥ 1） */
    float   timeout_s;      /* 这么久没有被接收的测距就标无效 */
} DRV_ZEstParams;

typedef enum {
    DRV_ZEST_MEAS_NONE = 0,     /* 本拍没有新样本（仅 Tick 返回） */
    DRV_ZEST_MEAS_ACCEPTED,
    DRV_ZEST_MEAS_INIT,         /* 无效状态下的第一个样本：以它起步 */
    DRV_ZEST_MEAS_REJECTED,     /* 过门限，拒收 */
    DRV_ZEST_MEAS_RESET,        /* 连续拒收到上限：高度平移到测量值 */
    DRV_ZEST_MEAS_IGNORED       /* 输入非法（非有限、样本已过期） */
} DRV_ZEstMeasResult;

typedef struct {
    DRV_ZEstParams params;
    float z_m;                           /* 当前时刻高度 */
    float vz_m_s;                        /* 当前时刻垂直速度 */
    float bias_m_s2;                     /* a_up 的零偏估计 */
    float accel_raw_m_s2;                /* 最近一拍送进来的 a_up */
    float since_meas_s;                  /* 距上一个被接收样本的采样时刻 */
    float last_innovation_m;             /* 最近一个样本的新息（含被拒收的） */
    float hist_z[DRV_ZEST_HISTORY];      /* 每拍预测后的高度 */
    float hist_dt[DRV_ZEST_HISTORY];     /* 该条与更老一条的时间差 */
    uint64_t tick_last_us;               /* Tick 簿记：上一拍 µs 时刻 */
    uint32_t tick_last_sample_ms;        /* Tick 簿记：上一次处理的样本时间戳 */
    uint32_t accepted_count;
    uint32_t rejected_count;
    uint32_t reset_count;
    uint8_t head;                        /* 最新一条的下标 */
    uint8_t count;                       /* 有效条数 */
    uint8_t initialized;                 /* 至少起步过一次 */
    uint8_t reject_run;                  /* 当前连续拒收数 */
    uint8_t accel_valid;                 /* 最近一拍的 a_up 有效 */
    uint8_t tick_primed;
} DRV_ZEst;

typedef struct {
    float z_m;
    float vz_m_s;
    float accel_m_s2;       /* a_up − b，供速度环 D 项 */
    float bias_m_s2;
    float innovation_m;
    uint8_t valid;          /* 起步过且 since_meas ≤ timeout */
    uint8_t accel_valid;    /* 最近一拍的 a_up 有效 */
} DRV_ZEstOutput;

/* 每个控制拍的一组输入（Tick 用）。 */
typedef struct {
    uint64_t now_us;            /* µs 时基，只用来算 Δt */
    uint32_t now_ms;            /* 与 range_sample_ms 同一个 ms 钟 */
    float    a_up_m_s2;         /* 竖直运动加速度，向上为正、已去 g */
    uint8_t  accel_valid;
    uint8_t  range_valid;       /* 测距服务报有效 */
    float    range_m;           /* 未平滑的原始测距高度 */
    uint32_t range_sample_ms;   /* 该样本的到达时刻；变了才算新样本，0 = 没有 */
} DRV_ZEstTickInput;

void    DRV_ZEst_DefaultParams(DRV_ZEstParams *params);
uint8_t DRV_ZEst_ParamsValid(const DRV_ZEstParams *params);
/* params 为 NULL 用默认值；非法时装默认值并返回 0。状态全部清零（无效）。 */
uint8_t DRV_ZEst_Init(DRV_ZEst *est, const DRV_ZEstParams *params);
/* 换参数不动状态；非法返回 0，原参数不动。 */
uint8_t DRV_ZEst_SetParams(DRV_ZEst *est, const DRV_ZEstParams *params);
/* 清状态（含零偏与 Tick 簿记），保留参数。 */
void    DRV_ZEst_Reset(DRV_ZEst *est);

/* dt_s ≤ 0 或非有限：本拍不推进。dt 截到 DT_MAX。 */
void    DRV_ZEst_Predict(DRV_ZEst *est, float dt_s, float a_up_m_s2, uint8_t accel_valid);
/* age_s：样本到达至今（≥ 0）；D + age 超出历史覆盖时按最老一条算。 */
DRV_ZEstMeasResult DRV_ZEst_Correct(DRV_ZEst *est, float height_m, float age_s);

/*
 * 一拍：按 now_us 差值预测，range_sample_ms 变了（且非 0、range_valid）就用
 * age = now_ms − range_sample_ms 校正。第一拍只起时钟不推进。
 */
DRV_ZEstMeasResult DRV_ZEst_Tick(DRV_ZEst *est, const DRV_ZEstTickInput *in);
void    DRV_ZEst_GetOutput(const DRV_ZEst *est, DRV_ZEstOutput *out);

#ifdef __cplusplus
}
#endif

#endif /* DRV_Z_ESTIMATOR_H */

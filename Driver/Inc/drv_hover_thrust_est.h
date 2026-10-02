#ifndef DRV_HOVER_THRUST_EST_H
#define DRV_HOVER_THRUST_EST_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 悬停推力在线估计器：一维卡尔曼，状态 h = "托住机体所需的合推力" [N]，推力查补表口径
 * （与 coax.hover_thrust_n、DRV_COAX_CTRL_EffectiveMassKg 同口径）。思路取自 PX4
 * mc_hover_thrust_estimator（ZeroOrderHoverThrustEkf）。纯算术：无寄存器、无 RTOS、无 I/O、
 * 无文件级可变量，状态在调用者传入的结构体里（解耦规范 D3-3），宿主可直接单测。
 *
 * 这一版只学、只读：估计值不回写 coax.hover_thrust_n，也不进任何控制器。
 *
 * ──────────────── 为什么要它 ────────────────
 *
 * 高度环的合推力是用 coax.hover_thrust_n 归一化的（有效质量 = hover/g）。槽式台架上托住
 * 机体要约 14.25 N（表值），而 1184 g 的重力只有 11.6 N；多出来的 2.35 N 是台架特有的
 * （槽壁摩擦、线束、机身挡下洗……）还是真飞也有，只有真飞才知道。与其让人去量，不如让
 * 飞控自己边飞边学，作者随时读出来看。
 *
 * ──────────────── 测量模型 ────────────────
 *
 * 竖直运动加速度（已去重力，向上为正）a_up 与实际下发的合推力竖直分量 T_up 之间：
 *     a_up = g · (T_up / h − 1)
 * 推力刚好等于悬停推力时 a_up = 0；h 未知时，每个样本都是对 h 的一次带噪观测：
 *     h ≈ T_up / (1 + a_up/g)，H = ∂a/∂h = −g·T_up/h²（按当前估计线性化）。
 * T_up = T · cos(roll) · cos(pitch)：两个桨的倾转（≤ 5°，cos ≈ 0.996）不计。
 *
 * 两处预处理，都是为了让"推力"和"加速度"落在同一个时间轴、同一个带宽上：
 *   - 推力过一阶滞后 thrust_lag_s：下发的是指令推力，电机实际推力 t63≈65 ms 才跟上，
 *     不滤的话每次油门阶跃都在新息里留下一大段假误差；
 *   - 加速度过一阶低通 accel_lpf_s：250 Hz 的 IMU 加速度有振动噪声，滤掉它并把样本从"每拍
 *     一个"变成"几十毫秒一个独立样本"。模型对 (T, a) 是线性的（g 常数），两边各滤各的
 *     不改变 a = g(T/h − 1) 的关系，只要两个滤波器的时间常数取一致的量级。
 *
 * ──────────────── 滤波器 ────────────────
 *
 *   预测：P += q²·Δt        （只在"正在学"的拍推进；不学的拍状态原样保持，不让 P 在地面
 *                              空转里无限长大）
 *   校正：S = H²P + R，K = P·H/S，h += K·新息，P −= K·H·P
 *   过程噪声 q（process_std_n_sqrt_s）：h 的随机游走强度。悬停推力随电池掉电（查补表已带
 *          电压补偿，残余漂移慢）、桨叶脏污、重心移动而慢变。取 0.03 N/√s：飞 60 s 的随机
 *          游走 1σ 约 0.23 N（约 1.6% h），跟得上一块电池 10 分钟里 1–2 N 的慢漂，又不会被
 *          单次机动带偏。PX4 用 0.0036/√s 相对归一化推力 ≈0.5，量级同为 ~0.7%/√s。
 *   测量噪声 R：自适应（PX4 同法）。R ← (1−α)R + α·max(新息² − H²P_先验, R_min)，夹到
 *          [R_min, R_max]。运动中新息变大 R 就跟着变大，估计自动"放慢"；静止时 R 缩回去。
 *
 * ──────────────── 新息门限与失锁恢复 ────────────────
 *
 * 新息² > gate_sigma² · S 的样本拒收（撞击、碰槽壁的尖峰加速度）。拒收不更新 h/P/R。
 * 但纯门限有个失锁陷阱：初值错很多且 P 已收得很小时，真实偏差引起的新息也会被拒收，估计
 * 卡死。所以连续拒收 reject_inflate 个后把 P 抬回 inflate_std_n²（"我可能错了"），下一拍
 * 起重新允许大新息进来。
 *
 * ──────────────── 收敛判据 ────────────────
 *
 * 已接收样本 ≥ converged_samples 且 std ≤ converged_std_n。注意 std 是卡尔曼 P 的平方根，
 * 采样相关（低通后相邻样本并不独立）会让它偏乐观，只当"学得差不多了"的信号，不是精度保证。
 *
 * ──────────────── 契约（D4-3） ────────────────
 *
 * 推力 [N]（查补表口径）、加速度 [m/s²]（竖直向上为正、已去重力）、Δt [s]、角度 [rad]。
 * "什么时候该学"不在本模块判断，由调用者用 learn 标志给（见 App/Src/app_hover_thrust.c）：
 * learn = 0 的拍只推进两个预处理滤波器，不动 h/P/R。
 *
 * 默认参数定标依据见 data/analysis/sysid-rig-params/2026-09-30/hover_est_replay.txt
 * （把本文件编成宿主库回放 09-30 ALT 实录）。
 */

#define DRV_HOVEREST_DT_MAX_S        0.05f     /* 单拍 Δt 上限；更长的空档当作断档，滤波器重新起步 */
#define DRV_HOVEREST_H_MIN_N         1.0f      /* 与 DRV_COAX_CTRL_HOVER_THRUST_MIN_N 同 */
#define DRV_HOVEREST_H_MAX_N         40.0f     /* 与 DRV_COAX_CTRL_HOVER_THRUST_MAX_N 同 */
#define DRV_HOVEREST_P_MIN_N2        1.0e-6f   /* 方差下限，防退化 */

/* 默认值。数字来源见上面的滤波器说明与 hover_est_replay.txt。 */
#define DRV_HOVEREST_INIT_STD_DEFAULT_N        3.0f
#define DRV_HOVEREST_PROCESS_STD_DEFAULT       0.03f     /* N/√s */
#define DRV_HOVEREST_MEAS_STD_INIT_DEFAULT     1.0f      /* m/s² */
#define DRV_HOVEREST_MEAS_STD_MIN_DEFAULT      0.15f
#define DRV_HOVEREST_MEAS_STD_MAX_DEFAULT      5.0f
#define DRV_HOVEREST_NOISE_ALPHA_DEFAULT       0.02f
#define DRV_HOVEREST_GATE_SIGMA_DEFAULT        3.5f
#define DRV_HOVEREST_REJECT_INFLATE_DEFAULT    50U
#define DRV_HOVEREST_INFLATE_STD_DEFAULT_N     1.5f
#define DRV_HOVEREST_THRUST_LAG_DEFAULT_S      0.065f
#define DRV_HOVEREST_ACCEL_LPF_DEFAULT_S       0.065f
#define DRV_HOVEREST_CONV_STD_DEFAULT_N        0.30f
#define DRV_HOVEREST_CONV_SAMPLES_DEFAULT      500U

typedef struct {
    float    init_std_n;          /* 初值的不确定度 σ */
    float    process_std_n_sqrt_s;/* h 随机游走强度 q [N/√s] */
    float    meas_std_init;       /* 测量噪声初值 σ [m/s²] */
    float    meas_std_min;        /* 自适应下限 */
    float    meas_std_max;        /* 自适应上限 */
    float    noise_alpha;         /* R 自适应的每样本混合系数 */
    float    gate_sigma;          /* 新息门限（倍 √S） */
    uint16_t reject_inflate;      /* 连续拒收这么多个就把 P 抬回 inflate_std_n² */
    float    inflate_std_n;
    float    thrust_lag_s;        /* 推力预滤波的一阶滞后；0 = 不滤 */
    float    accel_lpf_s;         /* 加速度预滤波的一阶时间常数；0 = 不滤 */
    float    converged_std_n;
    uint32_t converged_samples;
} DRV_HoverEstParams;

typedef enum {
    DRV_HOVEREST_HELD = 0,        /* learn = 0：只推进预处理滤波器 */
    DRV_HOVEREST_ACCEPTED,
    DRV_HOVEREST_REJECTED,        /* 新息过门限 */
    DRV_HOVEREST_IGNORED          /* 输入非法（非有限、Δt 非正、倾角 ≥ 90°） */
} DRV_HoverEstResult;

typedef struct {
    DRV_HoverEstParams params;
    float    init_n;              /* Init/Reset 时的初值 */
    float    h_n;                 /* 当前估计 */
    float    p_n2;                /* 估计方差 */
    float    r_var;               /* 自适应测量方差 [(m/s²)²] */
    float    thrust_f_n;          /* 预滤波后的竖直推力 */
    float    accel_f_m_s2;        /* 预滤波后的竖直加速度 */
    float    last_innov_m_s2;     /* 最近一个被评估样本的新息（含被拒收的） */
    uint32_t samples;             /* 累计接收的样本数 */
    uint32_t rejected;            /* 累计拒收数 */
    uint16_t reject_run;          /* 当前连续拒收数 */
    uint8_t  filt_primed;         /* 预处理滤波器已起步 */
    uint8_t  learning;            /* 最近一拍 learn = 1 且输入合法 */
} DRV_HoverEst;

typedef struct {
    float    est_n;
    float    std_n;
    float    init_n;
    float    innov_m_s2;
    float    meas_std_m_s2;       /* 自适应测量噪声当前 σ */
    uint32_t samples;
    uint32_t rejected;
    uint8_t  converged;
    uint8_t  learning;
} DRV_HoverEstOutput;

/* 每拍输入。 */
typedef struct {
    float   dt_s;
    float   thrust_n;             /* 实际下发的合推力（查补表口径），未乘倾角 */
    float   roll_rad;
    float   pitch_rad;
    float   a_up_m_s2;            /* 竖直运动加速度，向上为正、已去重力 */
    float   gravity_m_s2;
    uint8_t learn;                /* 调用者判定"此刻的样本可用来学" */
} DRV_HoverEstInput;

void    DRV_HoverEst_DefaultParams(DRV_HoverEstParams *params);
uint8_t DRV_HoverEst_ParamsValid(const DRV_HoverEstParams *params);
/*
 * params 为 NULL 用默认值；非法时装默认值并返回 0。init_n 越出 [H_MIN, H_MAX] 或非有限
 * 也返回 0，此时状态置为"未初始化的默认 10 N"，调用者不应使用。
 */
uint8_t DRV_HoverEst_Init(DRV_HoverEst *est, const DRV_HoverEstParams *params, float init_n);
/* 估计回到 Init 时的初值与 init_std，样本计数与预处理滤波器清零；参数保留。 */
void    DRV_HoverEst_Reset(DRV_HoverEst *est);
DRV_HoverEstResult DRV_HoverEst_Step(DRV_HoverEst *est, const DRV_HoverEstInput *in);
void    DRV_HoverEst_GetOutput(const DRV_HoverEst *est, DRV_HoverEstOutput *out);

#ifdef __cplusplus
}
#endif

#endif /* DRV_HOVER_THRUST_EST_H */

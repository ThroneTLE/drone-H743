#ifndef DRV_SYSID_EXCITATION_H
#define DRV_SYSID_EXCITATION_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 辨识激励剖面发生器：**时间的纯函数**，没有内部状态。
 *
 * `Eval(spec, t_ms)` 只依赖入参，同样的 (spec, t_ms) 永远得到同样的输出。
 * 这不是洁癖，是三个具体好处：
 *   - 宿主可以逐样本和固件对拍（`tests/test_sysid_excitation.py`）；
 *   - 上位机能在**不连飞控**的情况下画出将要发出的波形；
 *   - 中途丢一拍不会让序列跑偏——第 k 拍算的永远是第 k 拍该有的值。
 *
 * ──────────────── 输出的是角速度，不是舵机量 ────────────────
 *
 * 本模块给出绕台架杆轴的**期望角速度** ω_sp(t) 和它的导数 α_ff(t)。
 * 由角速度到舵机脉宽那一段是：
 *     τ_ff = I_n · α_ff  →  τ_body = τ_n · n（drv_sysid_rig）
 *     →  DRV_COAX_CTRL_SolveBodyTiltFromMoment（真实控制器的反解）
 *     →  DRV_COAX_CTRL_BodyTiltRadToServoPulses
 * 一行新的力学都不写，全部复用已在飞的那套 C。
 *
 * 这也是"多次高频执行收敛出惯量"的机制所在：用**假定惯量** I_est 算前馈，
 * 忽略阻尼/延迟时，期望角加速度除以实测角加速度才是 I_true/I_est，时移就是总延迟。
 * 实际辨识还必须检查执行器饱和、推力标定误差及模型适用性。
 *
 * ──────────────── 为什么每个剖面都有斜坡 ────────────────
 *
 * 方波的导数是冲激，而冲激力矩是做不出来的：前馈会要求一个无穷大的倾角。
 * 所以 doublet/PRBS 的每一次翻转都按 `ramp_ms` 线性过渡，α_ff 因此处处有界。
 * ramp_ms = 0 会被 Validate 拒绝，不是"取消斜坡"。
 *
 * 单位：角速度 rad/s，角加速度 rad/s²，时间 ms，频率 Hz。
 */

typedef enum {
    DRV_SYSID_EXC_OK = 0,
    DRV_SYSID_EXC_INVALID
} DRV_SysIdExcStatus;

typedef enum {
    DRV_SYSID_PROFILE_STEP = 0,    /* 单向阶跃：斜坡上去、保持、斜坡回零 */
    DRV_SYSID_PROFILE_DOUBLET = 1, /* 正负交替方波对 */
    DRV_SYSID_PROFILE_CHIRP = 2,   /* 线性扫频正弦 */
    DRV_SYSID_PROFILE_PRBS = 3,    /* 31 位 LFSR 伪随机方波 */
    DRV_SYSID_PROFILE_COUNT = 4
} DRV_SysIdProfile;

/* 安全上限。超出由 Validate 拒绝，不裁剪——裁剪会让人以为发出去的就是填的值。 */
#define DRV_SYSID_EXC_MAX_RATE_RAD_S    5.0f      /* 约 286 deg/s */
#define DRV_SYSID_EXC_MAX_DURATION_MS   30000U
#define DRV_SYSID_EXC_MIN_RAMP_MS       5U
#define DRV_SYSID_EXC_MAX_REPEAT        20U

typedef struct {
    uint8_t  profile;            /* DRV_SysIdProfile */
    float    amplitude_rad_s;    /* 峰值角速度，> 0 */
    uint32_t duration_ms;        /* 总时长；doublet 由 hold/repeat 推出后取二者较小 */
    uint32_t hold_ms;            /* step 的平台时长 / doublet 的半周期 */
    uint32_t repeat;             /* doublet 的对数 */
    uint32_t ramp_ms;            /* 每次电平过渡的斜坡时长，> 0 */
    float    chirp_f0_hz;        /* chirp 起始频率 */
    float    chirp_f1_hz;        /* chirp 终止频率 */
    uint32_t prbs_bit_ms;        /* PRBS 每位时长 */
    uint32_t prbs_seed;          /* LFSR 种子；0 视为 1 */
} DRV_SysIdExcitation;

typedef struct {
    float   omega_sp_rad_s;   /* 绕杆轴的期望角速度 */
    float   alpha_ff_rad_s2;  /* 其时间导数，喂给前馈 */
    uint8_t finished;         /* 1 = 剖面已走完，调用方应结束本趟 */
} DRV_SysIdExcSample;

/* 参数体检。返回 INVALID 的 spec 绝不允许被 Eval 或下发。 */
DRV_SysIdExcStatus DRV_SysIdExcitation_Validate(const DRV_SysIdExcitation *spec);

/* 剖面总时长 [ms]。Validate 通过才有意义。 */
DRV_SysIdExcStatus DRV_SysIdExcitation_TotalMs(const DRV_SysIdExcitation *spec,
                                               uint32_t *out_ms);

/* 时间的纯函数。t_ms 超过总时长时输出零并置 finished。 */
DRV_SysIdExcStatus DRV_SysIdExcitation_Eval(const DRV_SysIdExcitation *spec,
                                            uint32_t t_ms,
                                            DRV_SysIdExcSample *out);

#ifdef __cplusplus
}
#endif
#endif

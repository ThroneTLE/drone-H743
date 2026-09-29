#ifndef DRV_RPM_NOTCH_H
#define DRV_RPM_NOTCH_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 按电调转速跟踪的陀螺陷波（RPM notch）：**纯算法**，没有寄存器、没有 RTOS、
 * 没有文件级可变量。策略（转速新不新鲜、极对数、采样率估计）在 App/Src/app_rpm_notch.c。
 *
 * ──────────────── 信号契约（D4-3） ────────────────
 *
 * 输入与输出都是角速度 rad/s，坐标系任意但固定（本工程接的是规范 FLU 机体系，
 * 见 drv_frame_contract.h）。三轴用同一组系数：每个陷波直流增益恰为 1、线性时不变，
 * 所以它与零偏扣除、V0/V1 线性映射可交换，**永远不改变坐标系或符号**。
 * 频率单位 Hz，时间单位 s。
 *
 * ──────────────── 结构 ────────────────
 *
 * 每个电机 3 个槽（1x/2x/3x 谐波），两个电机共 6 个槽，按固定顺序级联：
 *     slot = motor*3 + (h-1)，motor = 电调通道 - 1
 * 系数由槽的中心频率决定、三轴共用；状态逐轴。每个槽的输出按权重与输入混合
 * （Betaflight 式 y = x + w·(N(x) − x)），权重在 ramp_s 内线性爬升/回落，
 * 所以开、关、换中心都不会在输出上砸出台阶。
 *
 * **直接型 I（DF1）而不是转置 II 型。** DF1 存的是真实的历史输入输出；
 * 每组系数的直流增益都恰为 1，稳态下 x1 == y1，换系数不注入任何东西。
 * TDF2 的状态依赖系数，每次换中心都会注入一个与变化率成正比的台阶。
 *
 * **没有活动槽时输出逐位等于输入**（memcpy 意义上），这是"关掉就和以前一模一样"的保证。
 */

#define DRV_RPM_NOTCH_AXES        3U
#define DRV_RPM_NOTCH_MOTORS      2U   /* index = ESC channel - 1 */
#define DRV_RPM_NOTCH_HARMONICS   3U
#define DRV_RPM_NOTCH_SLOTS       (DRV_RPM_NOTCH_MOTORS * DRV_RPM_NOTCH_HARMONICS) /* slot = motor*3 + (h-1) */
#define DRV_RPM_NOTCH_Q_MIN            1.5f
#define DRV_RPM_NOTCH_Q_MAX            10.0f
#define DRV_RPM_NOTCH_TOP_FADE_START   0.40f   /* x fs: weight 1 below */
#define DRV_RPM_NOTCH_TOP_FADE_END     0.45f   /* x fs: weight 0 above */
#define DRV_RPM_NOTCH_RECOMPUTE_HZ     0.1f
#define DRV_RPM_NOTCH_RECOMPUTE_FS_REL 0.001f
#define DRV_RPM_NOTCH_FLUSH_ABS        1.0e-20f

/* RBJ notch normalised by a0: b2 == b0 and a1 == b1 by construction, so only 3 values are stored. */
typedef struct { float b0, b1, a2; } DRV_NotchCoef;
typedef struct { float x1, x2, y1, y2; } DRV_NotchState;

typedef struct {
    uint8_t harmonic_mask;   /* bit0=1x bit1=2x bit2=3x, 1..7 */
    float   q;               /* constant Q, [1.5, 10] */
    float   min_hz;          /* weight 0 at/below */
    float   fade_hz;         /* weight 1 at min_hz+fade_hz; > 0 */
    float   ramp_s;          /* weight slew 0->1 time, default 0.020 */
    float   slew_hz_per_s;   /* fundamental slew limit, default 3000 */
    float   watchdog_s;      /* Apply without SetMotor longer than this -> all targets 0, default 0.040 */
} DRV_RpmNotchConfig;

typedef struct {
    DRV_NotchCoef  coef;
    DRV_NotchState state[DRV_RPM_NOTCH_AXES];
    float   center_hz, weight, target;
    uint8_t coef_valid, active;
} DRV_RpmNotchSlot;

typedef struct {
    DRV_RpmNotchConfig cfg;
    DRV_RpmNotchSlot   slot[DRV_RPM_NOTCH_SLOTS];
    float    fs_hz, coef_fs_hz, ramp_step;   /* ramp_step = 1/(ramp_s*fs) */
    uint32_t watchdog_samples, samples_since_update;
    float    motor_hz[DRV_RPM_NOTCH_MOTORS];
    /* coef_dirty 按电机置位（bit m）：fs 变了，该电机下一次 SetMotor 必须重算系数。 */
    uint8_t  motor_hz_valid[DRV_RPM_NOTCH_MOTORS], coef_dirty, reset_pending, watchdog_tripped;
    uint32_t slew_clamp_count, nonfinite_count, watchdog_count, reset_count;
} DRV_RpmNotch;

/*
 * 设计一个陷波：w0 = 2π·f0/fs，α = sin w0/(2q)，k = 1/(1+α)，
 * b0 = k，b1 = −2·cos w0·k，a2 = (1−α)·k。
 * 返回 1 = 成功。f0<=0、f0>=0.5fs、q 不在 [1.5,10]、fs<=0、任何输入或结果非有限都返回 0，
 * 此时 c 原样不动。
 */
uint8_t DRV_Notch_Design(DRV_NotchCoef *c, float f0_hz, float q, float fs_hz);
void    DRV_Notch_Reset(DRV_NotchState *s, float x);            /* x1=x2=y1=y2=x (DC steady state) */
/* DF1 一步。输出 |y| < 1e-20 冲成 0（不留次正规数）；非有限输入原样返回、状态不动。 */
float   DRV_Notch_Step(DRV_NotchState *s, const DRV_NotchCoef *c, float x);

void    DRV_RpmNotch_DefaultConfig(DRV_RpmNotchConfig *cfg);
uint8_t DRV_RpmNotch_ConfigValid(const DRV_RpmNotchConfig *cfg);
uint8_t DRV_RpmNotch_Init(DRV_RpmNotch *n, const DRV_RpmNotchConfig *cfg); /* invalid cfg -> 0, bank bypasses */
void    DRV_RpmNotch_SetFs(DRV_RpmNotch *n, float fs_hz);       /* 0 = unknown -> Apply passthrough */
/*
 * 每个控制拍每个电机一次。fundamental_hz 是该电机的机械转频（1x），dt_s 是距上一拍的时间。
 * fresh=0（或频率非有限/非正）：该电机所有槽目标权重置 0，中心保持（淡出用最后的中心）。
 * 槽仍在工作时，中心变化被限到 slew_hz_per_s·dt（防 4 bit CRC 漏检的坏帧拽走陷波）；
 * 全部淡出后重新捕获不限速。
 */
void    DRV_RpmNotch_SetMotor(DRV_RpmNotch *n, uint8_t motor, float fundamental_hz, uint8_t fresh, float dt_s);
/* 每个 IMU 样本一次，in/out 可以是同一数组。 */
void    DRV_RpmNotch_Apply(DRV_RpmNotch *n, const float in[3], float out[3]);
void    DRV_RpmNotch_Reset(DRV_RpmNotch *n);                    /* next Apply resets active slots to steady state at its input */
float   DRV_RpmNotch_SlotWeight(const DRV_RpmNotch *n, uint8_t motor, uint8_t harmonic); /* harmonic 1..3 */
uint8_t DRV_RpmNotch_ActiveSlots(const DRV_RpmNotch *n);

#ifdef __cplusplus
}
#endif

#endif /* DRV_RPM_NOTCH_H */

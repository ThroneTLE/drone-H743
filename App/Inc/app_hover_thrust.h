#ifndef APP_HOVER_THRUST_H
#define APP_HOVER_THRUST_H

#include <stdint.h>

#include "drv_hover_thrust_est.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 悬停推力在线学习的"何时学"策略层（算法在 Driver/Src/drv_hover_thrust_est.c）。
 *
 * 估计值不回写 coax.hover_thrust_n；2026-10-01 起经 app_hover_adapt.c 接进控制器（收敛才用、±15%、慢过渡、
 * 每次解锁重学，`HOVER ADAPT OFF` 可关）。读取用 `HOVER?`
 * （App/Src/app_cmd_hover.c），重置用 `HOVER RESET`。
 *
 * ──────────────── 什么时候学 ────────────────
 *
 * 机体压在地面或槽底时，推力小于重力而加速度 ≈ 0，模型 a = g(T/h − 1) 会把 h 学得严重偏低。
 * 所以以下六条必须**同时**满足才学，任何一条不满足这一拍就只推进预处理滤波器：
 *   bit0 已解锁（且遥控链路正常）——电机不受飞控控制时没有"下发推力"可言；
 *   bit1 本拍有实际下发的推力；
 *   bit2 推力没被封顶/饱和——封顶时控制器要的比给的多，这拍不是稳态托举；
 *   bit3 倾角 < 20°——倾角大时 T·cosφcosθ 近似与舵机倾转的误差都放大；
 *   bit4 加速度有效（IMU 新鲜）；
 *   bit5 机体已离开支撑面。
 *
 * "离开支撑面"怎么判：仓库里没有现成的着陆/离地判据（app_stabilizer.c 只有开机后首个
 * 有效测距锁存的 height_origin_m，且只在生产的稳定混控分支里才设，ALT/SYSID 走的直通分支
 * 里没有；ALT 的槽底读数 bottom_mm 只在它自己的模块里）。所以这里用两条路径都拿得到的
 * 原始测距自己判：
 *   - 支撑面高度 ground = 未解锁时对测距的滑动平均；解锁后只允许往下更新（槽底/地面
 *     不会自己升高，往下是纠正）。每次上锁重新锁存，落在另一个台面上不受上一次影响。
 *     解锁时还没锁存过（开机即解锁）就以首个有效读数起步。
 *   - 测距 > ground + 5 cm 且持续 0.2 s 才算离开；回落到 ground + 4 cm 以内立即算回到
 *     支撑面。5 cm 的依据：槽式台架实录里"槽底静止"的读数在推力作用下会上抬 3–4 cm
 *     （0.447 → 0.48–0.486 m，机体仍压在槽里），5 cm 恰好在它上面。
 * 找不到支撑面高度（测距从未有效）时永不判为离开——宁可不学也不学歪。
 *
 * 初值：coax.hover_thrust_n > 0 用它，否则用机体 m·g（airframe 有效时）。airframe 还没
 * 就绪时先不初始化，就绪后的第一拍才起步。
 */

#define APP_HOVER_LIFT_ON_M        0.05f
#define APP_HOVER_LIFT_OFF_M       0.04f
#define APP_HOVER_LIFT_HOLD_MS     200U
#define APP_HOVER_TILT_MAX_RAD     0.3491f   /* 20° */
#define APP_HOVER_THRUST_MIN_N     2.0f      /* 低于它没有可学的东西（怠速） */
#define APP_HOVER_GROUND_ALPHA     0.02f     /* 未解锁时支撑面高度的滑动平均系数（每拍） */
#define APP_HOVER_RANGE_ALPHA      0.2f      /* 解锁后往下追踪前对测距的一阶平滑（压尖刺） */

/* gate_mask 各位 */
#define APP_HOVER_GATE_ARMED       0x01U
#define APP_HOVER_GATE_THRUST      0x02U
#define APP_HOVER_GATE_UNSAT       0x04U
#define APP_HOVER_GATE_TILT        0x08U
#define APP_HOVER_GATE_ACCEL       0x10U
#define APP_HOVER_GATE_AIRBORNE    0x20U
#define APP_HOVER_GATE_ALL         0x3FU

typedef struct {
    uint32_t now_ms;
    float    dt_s;
    uint8_t  armed;              /* 已解锁且遥控链路正常 */
    uint8_t  thrust_valid;       /* 本拍有实际下发的合推力 */
    float    thrust_n;           /* 实际下发的合推力（查补表口径） */
    uint8_t  saturated;          /* 推力被封顶/饱和 */
    float    roll_rad;
    float    pitch_rad;
    float    a_up_m_s2;
    uint8_t  accel_valid;
    uint8_t  range_valid;
    float    range_m;            /* 未平滑的原始测距高度 */
    float    gravity_m_s2;
    float    hover_cfg_n;        /* coax.hover_thrust_n，0 = 未设 */
    float    mass_kg;            /* 机体质量（airframe 无效时 0） */
} APP_HoverThrustInput;

typedef struct {
    DRV_HoverEstOutput est;
    uint8_t  initialized;
    uint8_t  gate_mask;          /* 最近一拍各条件是否满足（APP_HOVER_GATE_*） */
    uint8_t  airborne;
    uint8_t  ground_valid;
    float    ground_m;
    float    above_ground_m;     /* 最近一次有效测距相对支撑面的高度 */
} APP_HoverThrustSnapshot;

/* 清全部状态（含支撑面锁存），下一拍按当时的初值来源重新起步。 */
void APP_HoverThrust_Init(void);
/* 每个控制拍一次。写者只有控制环。 */
void APP_HoverThrust_Step(const APP_HoverThrustInput *in);
/* 任意任务可调：读一份一致的快照（seqlock）。返回 0 = 尚未初始化或读取被写者反复打断。 */
uint8_t APP_HoverThrust_GetSnapshot(APP_HoverThrustSnapshot *out);
/* 任意任务可调：请求重置估计（回到初值），由控制环下一拍执行。 */
void APP_HoverThrust_RequestReset(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_HOVER_THRUST_H */

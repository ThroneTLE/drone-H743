#ifndef APP_SYSID_YAW_H
#define APP_SYSID_YAW_H

#include <stdint.h>

#include "app_sysid.h"
#include "drv_sysid_excitation.h"
#include "drv_sysid_record.h"

/*
 * 吊绳偏航辨识（SYSID MODE YAW = 6）。两侧契约：doc/sysid-yaw-contract.md。
 *
 * ──────────────── 台架 ────────────────
 *
 * 机体用绳从上方吊住，推力始终小于机重（绳一直绷紧），绳顶最好有转环。横滚/俯仰由吊挂摆约束，
 * 所以本模式舵机全程中位、不跑横滚俯仰环；只用上下桨差速产生偏航力矩。总推力 F 由配置给
 * （thrust_mn，默认 0.5×悬停推力），不借用 SYSID THROTTLE 的 target_n（只借它的"最高油门 %"封顶）。
 *
 * ──────────────── 一轮的样子 ────────────────
 *
 * RAMP_UP（1.5 s，两桨由怠速线性升到各 F/2，ΔT = 0）→ SETTLE（2 s）→ PREROLL（0.5 s）→
 * EXCITE（激励剖面）→ RAMP_DOWN（1 s 线性降到遥控油门，走完 finished）。舵机全程中位。
 *
 * 注入（EXC 的 amp 在本模式下按注入类型解释单位，shape = 激励剖面按单位幅值归一化并夹到 ±1）：
 *   diff：开环，辨对象。偏航力矩指令 M = k·ΔT，ΔT = amp[N]·shape，k = (k_upper + k_lower)/2
 *         （coax.yaw_torque_*_m_per_n，现 0.005 m、从未实测）。ΔT 上限 = min(0.8·F, 2·(T单max − F/2))。
 *   rate：闭环，验整定。偏航角速度参考 r = amp[rad/s]·shape（≤ 2.0）；M 由生产偏航角速度环算
 *         （DRV_RateControl_Step 只跑 z 轴：x/y 轴 omega = omega_sp = 0，故 x/y 输出与陀螺耦合项恒 0，
 *         参数 coax.rate_yaw_kp/ki/kd/i_limit_n_m/ff 与生产同一份，每拍现取，SYSID PARAM 改的下一拍生效；
 *         测量取 gyro_ctrl（偏航不在出口陷波之内，陷波只作用于横滚/俯仰），抗饱和上限取
 *         DRV_COAX_CTRL_YawLimitMomentNm(F)，分配器上一拍撞限时同生产冻结积分）。
 *         SETTLE/PREROLL 段 r = 0，环已在跑（只保持不转）。
 * 两种注入都走同一个分配 DRV_COAX_CTRL_AllocateYawPair（含偏航极性与单桨上限钳位）→ 推力映射换脉宽
 * （注入了 pulses_for_pair 用它，否则逐桨 DRV_COAX_CTRL_ThrustToMotorPulse），再被"最高油门 %"封顶。
 *
 * ──────────────── 安全 ────────────────
 *
 * 软停（本模块自己的中止）：ΔT 与 r 清零、环停，推力 1 s 线性降到遥控油门，走完再报理由。触发：
 *   yaw_overspeed：|gyro_z| > 8.0 rad/s；yaw_twist：开跑起陀螺 z 积分偏航角 |ψ| > twist_deg（绳会绞）；
 *   controller / excitation / phase。RAMP_DOWN 段不再判速度与绞角。
 * 遥控失联、上锁、推油门杆、IMU 失效、推力来源失效、STOP 命令由运行层当拍交还。
 *
 * ──────────────── 记录（沿用 ALT/XY 的 7 个尾字段，批头 DRV_SYSID_FLAG_YAW） ────────────────
 *   height = ψ（陀螺 z 积分，开跑清零）  height_raw = 0  height_sp = 0
 *   vz = 原始陀螺 z  vz_sp = r（diff 为 0）  az = ΔT = P·(T_lower − T_upper)（分配后、P 为偏航极性，
 *   正值对应正 M）  vbat。torque = M（饱和前），thrust = 分配用总推力 F；angle/angle_sp/servo_tilt/
 *   tilt_x/tilt_y 恒 0。
 */

typedef enum {
    APP_SYSID_YAW_INJECT_DIFF = 0,
    APP_SYSID_YAW_INJECT_RATE,
    APP_SYSID_YAW_INJECT_COUNT
} APP_SysIdYawInject;

/* `SYSID YAW` 的配置：只在空闲时收、只存 RAM，上电回默认（diff / 0.5×悬停推力 / 720）。 */
typedef struct {
    uint8_t  inject;      /* APP_SysIdYawInject */
    uint16_t thrust_mn;   /* 总推力 [mN]；Get 时 0（未设）已解析成默认值 */
    uint16_t twist_deg;   /* 绞绳上限 [deg] */
} APP_SysIdYawConfig;

#define APP_SYSID_YAW_THRUST_MN_MIN      2000U
#define APP_SYSID_YAW_THRUST_MN_MAX     65535U
#define APP_SYSID_YAW_TWIST_DEG_MIN        90U
#define APP_SYSID_YAW_TWIST_DEG_MAX      1440U
#define APP_SYSID_YAW_TWIST_DEG_DEFAULT   720U
#define APP_SYSID_YAW_LIFT_FRACTION       0.8f   /* thrust > 0.8×机重 会把机体提起来：配置时就拒 */
#define APP_SYSID_YAW_DEFAULT_HOVER_FRACTION 0.5f
#define APP_SYSID_YAW_DIFF_FRACTION       0.8f   /* ΔT ≤ 0.8·F */
#define APP_SYSID_YAW_RATE_AMP_MAX_RAD_S  2.0f
#define APP_SYSID_YAW_OVERSPEED_RAD_S     8.0f  /* 2026-10-01 首轮吊绳：作者"停止阈值太低了我正常激励都会触发"（实测效能约模型 3.7 倍） */

#define APP_SYSID_YAW_RAMP_UP_MS        1500U
#define APP_SYSID_YAW_SETTLE_MS         2000U
#define APP_SYSID_YAW_PREROLL_MS         500U
#define APP_SYSID_YAW_RAMP_DOWN_MS      1000U

/* ---------------------------------------------------------------- 配置面（命令任务） */

void APP_SysIdYaw_GetConfig(APP_SysIdYawConfig *out);
/* NULL = 已生效；否则整条不生效并回拒绝理由 "range" / "lift"。是否在跑由调用方判。 */
const char *APP_SysIdYaw_SetConfig(const APP_SysIdYawConfig *config);
const char *APP_SysIdYaw_InjectName(uint8_t inject);
uint8_t APP_SysIdYaw_InjectFromName(const char *name, uint8_t *out);
/* 回显一行：SYSID YAW inject= thrust_mn= twist_deg= control=openloop|closed_loop */
void APP_SysIdYaw_ReportConfig(void);

/* 本轮用的总推力 [N]（开跑前 = 当前配置，开跑后 = 锁存值）。 */
float APP_SysIdYaw_ThrustN(void);
/* 当前注入类型下的幅值上限（diff：N，rate：rad/s）；推力不合理时 0。 */
float APP_SysIdYaw_AmplitudeMax(void);

/* ---------------------------------------------------------------- 运行面（只由 app_sysid.c 调） */

/* 开跑前检查；NULL = 可开跑，否则回拒绝理由（ERR sysid yaw <理由>）。 */
const char *APP_SysIdYaw_Precheck(const DRV_SysIdExcitation *spec);
/* 开跑：锁存配置、剖面、油门封顶、k、Izz、T单max，清偏航角与环状态。 */
void APP_SysIdYaw_Begin(const DRV_SysIdExcitation *spec, float max_pct);
/* 紧跟 SYSID start 的溯源行 SYSID YAWSTART。 */
void APP_SysIdYaw_ReportStart(uint16_t run_id);

typedef struct {
    APP_SysIdPhase phase;       /* 本拍所处阶段；与传入的不同 = 本拍切换 */
    uint16_t upper_us;          /* 上/下桨脉宽，已按最高油门 % 封顶 */
    uint16_t lower_us;
    float    thrust_n;          /* 分配用总推力 F（RAMP 段为脉宽读回的合推力） */
    uint8_t  capped;
    uint8_t  check_thrust;      /* 推力 >= APP_SYSID_MIN_THRUST_N 才判推力门 */
    uint8_t  finished;          /* RAMP_DOWN 走完：本轮完成 */
    const char *abort_reason;   /* yaw_overspeed / yaw_twist / controller / excitation / phase；软停走完才非空 */
} APP_SysIdYawOutput;

/* 一个控制拍：偏航角积分 → 阶段推进 → 速度/绞角门 → ΔT 或角速度环 → 分配 → 脉宽。 */
void APP_SysIdYaw_Step(const APP_SysIdObserve *obs, APP_SysIdPhase phase, uint32_t phase_ms,
                       float dt_s, APP_SysIdYawOutput *out);
/* 把本拍的记录量填进样本（gz 等通用量已由运行层填好）。 */
void APP_SysIdYaw_FillSample(DRV_SysIdSample *sample, const APP_SysIdObserve *obs);
/* 偏航监视：本轮（或最近一轮）陀螺 z 积分偏航 [rad]，诊断/测试用。 */
float APP_SysIdYaw_GetPsiRad(void);

#endif /* APP_SYSID_YAW_H */

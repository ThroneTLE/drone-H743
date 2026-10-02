#ifndef APP_SYSID_XY_H
#define APP_SYSID_XY_H

#include <stdint.h>

#include "app_sysid.h"
#include "drv_sysid_excitation.h"
#include "drv_sysid_record.h"

/*
 * 水平槽 XY 速度/位置辨识（SYSID MODE XY = 5，R-XYID-1）。两侧契约：doc/sysid-xy-contract.md。
 *
 * ──────────────── 台架与方向 ────────────────
 *
 * 碳杆（杆轴方位角 ψ = sysid.rig.azimuth_rad）的两端落在水平槽里；推力托住机体自重，绕杆倾斜时
 * 推力的水平分量推着机体沿槽平移。平移方向 u = 水平且垂直于杆 = (−sin ψ, cos ψ)（FLU 机体系 x、y
 * 分量，台架上机体不偏航，机体系即世界系）。杆轴单位向量 n = (cos ψ, sin ψ)，垂直分量按 n 取号。
 * 光流位置/速度（SVC_FlowNav_GetPosition/GetVelocity）投影到 u 上，位置相对开跑那一刻。
 *
 * 符号提醒：绕杆轴 n 的正角（记录字段 angle = roll·cosψ + pitch·sinψ）使推力偏向 −u，
 * 即 angle > 0 对应沿 u 的加速度 < 0；angle_sp（目标倾角偏置在 n 上的分量）同一口径。
 *
 * ──────────────── 一轮的样子 ────────────────
 *
 * RAMP_UP（1.5 s 由怠速线性升到 target_n）→ SETTLE（2 s）→ PREROLL（0.5 s）→ EXCITE（激励剖面）→
 * RAMP_DOWN（1 s 降到遥控油门，走完 finished）。推力恒为 target_n（RAMP 段除外）。
 * 姿态：与 ALT 相同的两轴保持（目标 = 开跑第一拍的姿态 att0），本模块只给叠在上面的
 * 目标倾角偏置（横滚、俯仰）。
 *
 * 倾角偏置由沿 u 的期望水平加速度 a_u 换算：F = m_eff·(a_u·u_x, a_u·u_y, g)，
 * m_eff = DRV_COAX_CTRL_EffectiveMassKg(m_run)，再用生产同一个纯函数
 * DRV_COAX_CTRL_TiltFromForce 得横滚/俯仰，各自夹在 coax.tilt_limit_rad 内。
 *
 * 注入（EXC 的 amp 在本模式下按注入类型解释单位，shape = 激励剖面按单位幅值归一化并夹到 ±1）：
 *   tilt：开环。θ = amp[rad]·shape，a_u = g·tan θ；不跑位置/速度环。SETTLE/PREROLL 段 θ = 0。≤ 0.10 rad。
 *   vel ：闭环。p_sp = ∫v_inj dt，速度前馈 v_inj = amp[m/s]·shape。≤ 0.30 m/s。
 *   pos ：闭环。p_sp = amp[m]·shape，无前馈。≤ 0.15 m 且 ≤ 0.7·win。
 * 闭环用生产 DRV_POSITION_CONTROL_PositionStep/VelocityStep（沿 u 一维，只用 x 通道：参数取
 * coax.pos_x_kp / vel_x_kp / vel_x_ki / vel_x_kd / vel_x_i_limit_m_s2 / accel_xy_max_m_s2 /
 * pos_xy_vel_max_m_s，y、z 通道输入清零）。位置 50 Hz、速度 100 Hz（同生产节拍），倾角夹住时把
 * 饱和反馈喂回速度环（冻结积分）。与生产的口径差异：速度环 D 项的测量加速度取 0（台架没有
 * 沿 u 的加速度计量；vel_x_kd 默认 0 时无差别）。coax.vel_loop_enable 不看。
 *
 * ──────────────── 安全 ────────────────
 *
 * 软停（本模块自己的中止）：倾角偏置清零、位置/速度环停，姿态回 att0，推力 1 s 线性降到遥控油门，
 * 走完再报理由。触发：d + s²/(2·0.2) > win（xy_window，d = 相对起点位移模长 hypot(p_u, p_perp)，
 * s = 速度模长；旋转不变）；s > 0.4 m/s（xy_overspeed）；陀螺 z 积分偏航 > 15 度（xy_yaw_limit）；
 * 光流无效或样本超 200 ms 的缺口持续超过 200 ms（xy_flow_invalid；缺口内环保持上一拍输出）；controller / excitation / phase；运行层的杆轴残差/角度超限也走软停。
 * 遥控失联、上锁、推油门杆、IMU 失效、推力来源失效、STOP 命令由运行层当拍交还。
 * 推力 < 0.5·m_run·g（RAMP_UP 早段）时本模块的中止也当拍交还。
 * RAMP_DOWN 段不再判窗与光流（倾角偏置已清零，推力在降）。
 *
 * ──────────────── 记录（沿用 ALT 的 7 个尾字段） ────────────────
 *   height = p_u（沿 u，相对起点）  height_raw = p_perp（沿 n，相对起点）  height_sp = p_sp_u
 *   vz = v_u  vz_sp = v_sp_u（含前馈）  az = a_u  vbat；tilt 注入的 height_sp / vz_sp 为 0，az = g·tan θ。
 *   angle_sp = 目标倾角偏置在杆轴 n 上的分量；批头带 DRV_SYSID_FLAG_XY。
 */

typedef enum {
    APP_SYSID_XY_INJECT_TILT = 0,
    APP_SYSID_XY_INJECT_VEL,
    APP_SYSID_XY_INJECT_POS,
    APP_SYSID_XY_INJECT_COUNT
} APP_SysIdXyInject;

/* `SYSID XY` 的配置：只在空闲时收、只存 RAM，上电回默认（tilt / 150 / 0）。 */
typedef struct {
    uint8_t  inject;   /* APP_SysIdXyInject */
    uint16_t win_mm;   /* 出窗余量：|p_u − p0| > win 软停 */
    uint16_t mass_g;   /* 随动质量 [g]；0 = 取机体质量 */
} APP_SysIdXyConfig;

#define APP_SYSID_XY_WIN_MM_DEFAULT   150U
#define APP_SYSID_XY_WIN_MM_MIN        30U
#define APP_SYSID_XY_WIN_MM_MAX       400U
#define APP_SYSID_XY_MASS_G_MIN       500U
#define APP_SYSID_XY_MASS_G_MAX      3000U

#define APP_SYSID_XY_TILT_AMP_MAX_RAD  0.10f
#define APP_SYSID_XY_VEL_AMP_MAX_M_S   0.30f
#define APP_SYSID_XY_POS_AMP_MAX_M     0.15f

#define APP_SYSID_XY_RAMP_UP_MS        1500U
#define APP_SYSID_XY_SETTLE_MS         2000U
#define APP_SYSID_XY_PREROLL_MS         500U
#define APP_SYSID_XY_RAMP_DOWN_MS      1000U
#define APP_SYSID_XY_FLOW_TIMEOUT_MS    200U   /* 光流/测距样本的最长年龄 */
#define APP_SYSID_XY_SOFT_SKIP_FRACTION 0.5f   /* 推力不到 0.5·m_run·g：软停直接当拍交还 */

/* ---------------------------------------------------------------- 配置面（命令任务） */

void    APP_SysIdXy_GetConfig(APP_SysIdXyConfig *out);
/* 只做范围体检；是否在跑由调用方判。0 = 越界，原配置不动。 */
uint8_t APP_SysIdXy_SetConfig(const APP_SysIdXyConfig *config);
const char *APP_SysIdXy_InjectName(uint8_t inject);
uint8_t APP_SysIdXy_InjectFromName(const char *name, uint8_t *out);
/* 回显一行：SYSID XY inject= win_mm= mass_g= control=openloop|closed_loop */
void    APP_SysIdXy_ReportConfig(void);

/* ---------------------------------------------------------------- 运行面（只由 app_sysid.c 调） */

/* 每个控制拍（跑不跑都调）：记下最近的光流位置/速度与测距有效性，供开跑检查、起点与 THR? 行。 */
void APP_SysIdXy_Observe(const APP_SysIdObserve *obs, float azimuth_rad);
/* 开跑前检查；NULL = 可开跑，否则回拒绝理由（ERR sysid xy <理由>）。 */
const char *APP_SysIdXy_Precheck(float throttle_target_n, const DRV_SysIdExcitation *spec);
/* 开跑：锁存配置、剖面、油门上限、m_run、杆方位角与起点（开跑那一刻的光流位置）。 */
void APP_SysIdXy_Begin(const DRV_SysIdExcitation *spec, float target_n, float max_pct,
                       float azimuth_rad);
/* 紧跟 SYSID start 的溯源行 SYSID XYSTART。 */
void APP_SysIdXy_ReportStart(uint16_t run_id);

typedef struct {
    APP_SysIdPhase phase;       /* 本拍所处阶段；与传入的不同 = 本拍切换 */
    uint16_t motor_pulse_us;    /* 两路电机共用脉宽，已按最高油门 % 封顶 */
    float    thrust_n;          /* 该脉宽经推力查补表读回的合推力 */
    uint8_t  capped;
    uint8_t  hold_attitude;     /* 1 = 姿态链保持 att0 + tilt_bias */
    uint8_t  check_thrust;
    uint8_t  finished;          /* RAMP_DOWN 走完：本轮完成 */
    float    tilt_bias_rad[2];  /* 叠在 att0 上的目标倾角偏置 [0] 横滚 [1] 俯仰 */
    const char *abort_reason;   /* xy_window / xy_overspeed / xy_yaw_limit / xy_flow_invalid / controller / excitation / phase */
} APP_SysIdXyOutput;

/* 一个控制拍：光流投影 → 阶段推进 → 光流/出窗门 → 倾角偏置（开环或位置/速度环）→ 脉宽。 */
void APP_SysIdXy_Step(const APP_SysIdObserve *obs, APP_SysIdPhase phase, uint32_t phase_ms,
                      float dt_s, APP_SysIdXyOutput *out);
/* 运行层要中止（残差/角度）时先软停，本拍接着调 Step 即按软停出力。
 * 返回 0 = 推力还不到 0.5·m·g，调用方照原样当拍中止；已在软停返回 1。 */
uint8_t APP_SysIdXy_BeginSoftStop(const APP_SysIdObserve *obs, const char *reason);
/* 把记录尾部 7 个字段填进样本（光流无效的那一拍 height/height_raw/vz 记 0）。 */
void APP_SysIdXy_FillSample(DRV_SysIdSample *sample, const APP_SysIdObserve *obs);
/* THR? 行：沿 u 相对起点的位置 [m]、速度 [m/s]、光流+测距是否新鲜有效。 */
void APP_SysIdXy_GetLive(float *pos_m, float *vel_m_s, uint8_t *ok);
/* 偏航监视：本轮（或最近一轮）陀螺 z 积分出的偏航 [rad]，开跑清零；THR? 行 xy_yaw_mrad。 */
float APP_SysIdXy_GetYawRad(void);
/* 本轮最近一拍的倾角偏置（横滚/俯仰）与沿 u 的期望加速度，诊断/仿真用。 */
void APP_SysIdXy_GetTiltBias(float *roll_rad, float *pitch_rad, float *accel_u_m_s2);

#endif /* APP_SYSID_XY_H */

#ifndef APP_SYSID_ALT_H
#define APP_SYSID_ALT_H

#include <stdint.h>

#include "app_sysid.h"
#include "drv_sysid_excitation.h"
#include "drv_sysid_record.h"

/*
 * 光杆台架高度辨识（SYSID MODE ALT = 4，PIPELINE R-ALTID-1）：序列、高度环、三种注入。
 *
 * ──────────────── 台架与一轮的样子 ────────────────
 *
 * 碳杆（杆轴 −45°）的轴装在竖直槽里，机体连杆一起上下动，仍可绕杆轴转。测高只有光流
 * 模块的 TOF。解锁且油门杆最低时 START，之后：
 *
 *   RAMP_UP   1.5 s 开环把推力从怠速升到 0.9·m_run·g（机体还压在槽底）
 *   CLIMB     高度环接管：参考 z_sp 从开跑高度 h0 以 0.1 m/s 爬到 z_hold = h0 + lift_mm
 *   SETTLE    2 s 保持 z_hold
 *   PREROLL   0.5 s 零激励
 *   EXCITE    按 SYSID EXC 的剖面注入（时长 = 剖面总时长）
 *   DESCEND   z_sp 以 0.1 m/s 回到 h0 + 10 mm
 *   RAMP_DOWN 1 s 推力回怠速 → 本轮完成
 *
 * 全程按采样网格记录（含起升与回落，便于看起飞与摩擦）。姿态由运行层的姿态链保持：
 * 与 ANGLE 同一条生产控制路径，目标恒为杆轴角 0（起升/回落两段舵机回中）。
 *
 * ──────────────── 高度环：生产代码、生产参数 ────────────────
 *
 * 位置 P → 速度 PI(D) 就是在飞那一份 DRV_POSITION_CONTROL_PositionStep / VelocityStep（z 通道），
 * 参数每拍从 DRV_COAX_CTRL_GetParams() 的 .position 取——coax.pos_z_kp、coax.vel_z_kp/ki/kd、
 * coax.vel_z_i_limit_m_s2、coax.pos_z_vel_up/down_max_m_s、coax.accel_z_up/down_max_m_s2、
 * coax.accel_lpf_cutoff_hz，SYSID PARAM 试用的值下一拍就生效。本模块不另写 PI，只持有自己的
 * 状态实例。节拍取生产周期：位置 50 Hz、速度 100 Hz（APP_CONTROL_SCHED_*_PERIOD_US）；
 * 生产另要求光流速度新样本，台架上光流速度常无效，这里只看测距有效。
 * 与生产的两处口径差异（都写在这里，免得拿台架结论套飞行时对不上）：
 *   - 速度环 D 项的"测量加速度"用 IMU 竖直加速度 az（生产用 vz 差分）。vel_z_kd 默认 0，
 *     此时两者没有区别；
 *   - coax.vel_loop_enable 不看：本模式就是来量这条环的。
 * 合推力 F = m_run·(g + a_z)，m_run = mass_g>0 ? mass_g/1000 : 机体质量（台架随动质量只作
 * 本轮参数，不改 airframe），再加 force 注入，经推力查补表换成脉宽，被"最高油门 %"封顶。
 * 封顶时记录的 thrust 是封顶后实际下发脉宽对应的推力，批头带 DRV_SYSID_FLAG_THRUST_CAPPED；
 * 同生产一样，封顶且 a_z>0 时把"正向饱和"喂回速度环，积分不再往上顶。
 *
 * ──────────────── 注入（EXC 的 amp 在本模式下按注入类型解释单位） ────────────────
 *
 * shape(t) = 激励发生器按单位幅值归一化的剖面（ω_sp/amp，夹到 ±1，同 SERVO 模式做法）：
 *   force：F += amp[N]·shape——加在高度环算出的 F 之后、查补表之前，开环叠加；
 *   vel  ：v_inj = amp[m/s]·shape 作速度前馈，z_sp = z_hold + ∫v_inj dt（跟匀速移动的参考，
 *          同速上升/下降所需推力之差即摩擦）；
 *   pos  ：z_sp = z_hold + amp[m]·shape（阶跃验证）。
 * 幅值上限 force 3 N、vel 0.3 m/s、pos 0.15 m，超出拒绝开跑并回理由（不裁剪）。
 *
 * ──────────────── 安全门（中止，不限幅） ────────────────
 *
 * 本模块只判高度：TOF 连续无效超过 100 ms（height_invalid）；高度低于 h0 − 20 mm 或高于
 * z_hold + win_mm（height_window）。遥控/解锁/油门杆/IMU/杆轴残差/角度/推力来源仍由运行层
 * 那道门判，而且排在高度之前——链路断了之后高度当然也会出窗，先报根本原因。
 *
 * ──────────────── 竖直运动加速度 az 的来源 ────────────────
 *
 * 稳定环的导航系加速度只有水平两轴（acc_nav_m_s2[0..1]），没有 z。所以 az 取机体比力
 * （已标定、FLU、单位 g）按 roll/pitch 旋到竖直再减 g：与水平两轴同一个旋转
 * Ry(pitch)·Rx(roll)（stabilizer_compensated_imu_accel_level_xy），取它的第三行。
 * 没扣 IMU 杆臂的旋转加速度（姿态保持时角速度小），也不做倾转权重衰减。
 */

typedef enum {
    APP_SYSID_ALT_INJECT_FORCE = 0,
    APP_SYSID_ALT_INJECT_VEL,
    APP_SYSID_ALT_INJECT_POS,
    APP_SYSID_ALT_INJECT_COUNT
} APP_SysIdAltInject;

/* `SYSID ALT` 的配置：只在空闲时收、只存 RAM，上电回默认（force / 0 / 60 / 60）。 */
typedef struct {
    uint8_t  inject;    /* APP_SysIdAltInject */
    uint16_t mass_g;    /* 垂直移动质量 [g]；0 = 取机体质量 */
    uint16_t win_mm;    /* 出窗余量：高度 > z_hold + win_mm 即中止 */
    uint16_t lift_mm;   /* 抬升高度：z_hold = h0 + lift_mm */
} APP_SysIdAltConfig;

/*
 * 默认值按作者的槽式台架定（2026-09-29）：槽的总行程 160 mm，槽底起抬升 60 mm 悬停，
 * 超过 120 mm（z_hold + 60 mm）即中止，离槽顶留 40 mm。范围仍按契约放宽，换台架时改命令即可。
 */
#define APP_SYSID_ALT_MASS_G_MIN       500U
#define APP_SYSID_ALT_MASS_G_MAX      3000U
#define APP_SYSID_ALT_WIN_MM_DEFAULT    60U
#define APP_SYSID_ALT_WIN_MM_MIN        30U
#define APP_SYSID_ALT_WIN_MM_MAX       400U
#define APP_SYSID_ALT_LIFT_MM_DEFAULT   60U
#define APP_SYSID_ALT_LIFT_MM_MIN       30U
#define APP_SYSID_ALT_LIFT_MM_MAX      300U

#define APP_SYSID_ALT_FORCE_AMP_MAX_N  3.0f
#define APP_SYSID_ALT_VEL_AMP_MAX_M_S  0.3f
#define APP_SYSID_ALT_POS_AMP_MAX_M    0.15f

#define APP_SYSID_ALT_RAMP_UP_MS        1500U
#define APP_SYSID_ALT_RAMP_UP_FRACTION  0.9f
#define APP_SYSID_ALT_SETTLE_MS         2000U
#define APP_SYSID_ALT_PREROLL_MS         500U
#define APP_SYSID_ALT_RAMP_DOWN_MS      1000U
#define APP_SYSID_ALT_REF_SPEED_M_S     0.1f
#define APP_SYSID_ALT_LAND_ABOVE_M      0.010f
#define APP_SYSID_ALT_BELOW_START_M     0.020f
#define APP_SYSID_ALT_HEIGHT_TIMEOUT_MS  100U

/* ---------------------------------------------------------------- 配置面（命令任务） */

void    APP_SysIdAlt_GetConfig(APP_SysIdAltConfig *out);
/* 只做范围体检；是否在跑由调用方判（运行中拒收是命令面的事）。0 = 越界，原配置不动。 */
uint8_t APP_SysIdAlt_SetConfig(const APP_SysIdAltConfig *config);
const char *APP_SysIdAlt_InjectName(uint8_t inject);
uint8_t APP_SysIdAlt_InjectFromName(const char *name, uint8_t *out);
/* 回显一行：SYSID ALT inject=<force|vel|pos> mass_g=<g> win_mm=<mm> lift_mm=<mm> */
void    APP_SysIdAlt_ReportConfig(void);

/* 竖直运动加速度 [m/s²]，向上为正、已去 g（来源见文件头）。纯函数，稳定环填观测时调。 */
float APP_SysIdAlt_VerticalAccel(float accel_x_g, float accel_y_g, float accel_z_g,
                                 float roll_rad, float pitch_rad, float gravity_m_s2);

/* ---------------------------------------------------------------- 运行面（只由 app_sysid.c 调） */

/* 每个控制拍（跑不跑都调）：记下最近的测高，供开跑检查、h0 与状态行。 */
void APP_SysIdAlt_Observe(const APP_SysIdObserve *obs);
/* 开跑前检查：NULL = 可以开跑，否则是回给上位机的理由（运行层加 "ERR sysid alt " 前缀）。 */
const char *APP_SysIdAlt_Precheck(float throttle_target_n, float amplitude);
/* 开跑：锁存本轮配置、剖面、最高油门 %、m_run 与 h0，复位高度环。运行层在发布 RUNNING 前调。 */
void APP_SysIdAlt_Begin(const DRV_SysIdExcitation *spec, float max_pct);
/* 紧跟 SYSID start 的溯源行：SYSID ALTSTART run= alt_inject= alt_mass_g= alt_win_mm= alt_lift_mm= alt_h0_mm= */
void APP_SysIdAlt_ReportStart(uint16_t run_id);

typedef struct {
    APP_SysIdPhase phase;       /* 本拍所处阶段；与传入的不同 = 本拍切换 */
    uint16_t motor_pulse_us;    /* 两路电机共用脉宽，已按最高油门 % 封顶 */
    float    thrust_n;          /* 该脉宽经推力查补表读回的合推力（记录的 thrust） */
    uint8_t  capped;            /* 本拍被封顶 */
    uint8_t  hold_attitude;     /* 1 = 姿态链保持杆轴角 0；0 = 舵机回中（起升/回落） */
    uint8_t  check_thrust;      /* 1 = 推力来源与推力下限照判（高度环接管期间） */
    uint8_t  finished;          /* 回落走完：本轮完成 */
    const char *abort_reason;   /* height_invalid / height_window / excitation / controller / phase */
} APP_SysIdAltOutput;

/* 一个控制拍：阶段推进（条件满足本拍就切、按新阶段出力）→ 高度门 → 参考与注入 → 高度环 → 脉宽。 */
void APP_SysIdAlt_Step(const APP_SysIdObserve *obs, APP_SysIdPhase phase, uint32_t phase_ms,
                       float dt_s, APP_SysIdAltOutput *out);
/* 把记录尾部 7 个高度字段填进样本（测高无效的那一拍 height/height_raw/vz 记 0）。 */
void APP_SysIdAlt_FillSample(DRV_SysIdSample *sample, const APP_SysIdObserve *obs);
/* 状态行：最近一拍的测高 [m] 与是否有效、本轮当前高度参考 [m]。 */
void APP_SysIdAlt_GetLive(float *height_m, uint8_t *valid, float *height_sp_m);

#endif /* APP_SYSID_ALT_H */

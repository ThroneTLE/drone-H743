#ifndef APP_SYSID_ALT_H
#define APP_SYSID_ALT_H

#include <stdint.h>

#include "app_sysid.h"
#include "drv_sysid_excitation.h"
#include "drv_sysid_record.h"

/*
 * 光杆台架高度辨识（SYSID MODE ALT = 4，PIPELINE R-ALTID-1）。
 *
 * ──────────────── 台架与一轮的样子 ────────────────
 *
 * 碳杆（杆轴 −45°）的轴装在竖直槽里，机体连杆一起上下动，仍可绕杆轴转。测高只有光流
 * 模块的 TOF。槽底/槽顶的 TOF 绝对读数由上位机配置（bottom_mm/top_mm），开跑时机体必须在
 * 槽底附近。解锁且油门杆最低时 START，之后：
 *
 * break（离地/滑落阈值，2026-09-30 作者批准，替代 Codex 的开环保持 + 短脉冲）：
 *   RAMP_UP   1.5 s 从怠速线性升到 F_start = 0.9·m_run·g（机体还压在槽底）。这一段不判离地：
 *             实测推力一加测距就涨 10～25 mm（杆弹性形变/桨下洗），不是离地。
 *   CLIMB     慢升找离地：F_start + r·t，封顶 target_n；平均高度比进慢升时高 25 mm 且
 *             正以 >0.10 m/s 上升才算离地（门槛按实测测距漂移定，见常量处）。
 *   SETTLE    制停：第一拍保持离地判定时的指令（PHASE 行报它 = F_up 粗值），之后按离地时的
 *             上升速度估出多出的净推力，降"估值 + 0.5 N"（0.6～1.5 N）；
 *             每 100 ms 比一次上升量：还在升（>12 mm/100 ms）且没明显减速、距上次降档 ≥200 ms
 *             才再降 0.4 N——已在减速就等它自己停，免得降过头滑回去；200 ms 内净变化 ≤12 mm
 *             且距上次降档 ≥800 ms 算停稳：已回槽底直接 RAMP_DOWN，否则进 EXCITE。
 *   EXCITE    慢降找滑落：F_hold − r·t；平均高度比停住时低 25 mm 且正以 >0.10 m/s 下降即滑落。
 *   DESCEND   保持滑落时的指令滑回槽底，下降过快时短刹车；到底且停稳 → RAMP_DOWN。
 *   RAMP_DOWN 1 s 降到遥控器脉宽（杆在最低即怠速），落地确认后本轮完成。
 * 上行阈值 F_up ≈ m·g + Fs、下行阈值 F_down ≈ m·g − Fs（LUT 合推力指令）；精确起点由离线
 * 分析（tools/sysid/breakaway.py）从整段记录找，固件只做粗判切阶段。r = EXC 幅值 [N/s]。
 * vel/pos（闭环候选 PID 验证）：RAMP_UP → CLIMB → SETTLE → PREROLL → EXCITE → DESCEND →
 * RAMP_DOWN；Z 位置/速度环只在这两类注入里使用。
 *
 * 全程按采样网格记录（含起升与回落，便于看起飞与摩擦）。姿态由运行层的姿态链保持：
 * 与 ANGLE 同一条生产控制路径，目标恒为杆轴角 0。推力达到可靠反解下限后保持杆轴姿态；
 * 更低推力时舵机回中。
 *
 * ──────────────── 测高：8 样本平均 ────────────────
 *
 * 槽底静止、电机在转时 TOF 单样本噪声 σ 3–8 mm、峰峰 15–37 mm（2026-09-29 实录），单样本
 * 判离地/出窗会被噪声误触发。所以所有判据用最近 8 个**新**测距样本（按样本 token 去重，
 * 约 100 ms）的平均 h_avg；h0 = 开跑时的 h_avg。速度（制动门、制停第一档）用 h_avg 在最近 8 个
 * 样本上的斜率（槽底静止时噪声约 0.02 m/s；飞控 vz 在槽底静止时就有 ±0.1 m/s）。
 *
 * ──────────────── vel/pos 的高度环：生产代码、生产参数 ────────────────
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
 * 本轮参数，不改 airframe）；coax.hover_thrust_n > 0 时同生产换成 hover_thrust_n/g·(g + a_z)
 * （DRV_COAX_CTRL_EffectiveMassKg）。经推力查补表换成脉宽，被"最高油门 %"封顶。
 * 封顶（含查补表自身削顶）时记录的 thrust 是实际下发脉宽对应的推力，批头带
 * DRV_SYSID_FLAG_THRUST_CAPPED；同生产一样，封顶且 a_z>0 时把"正向饱和"喂回速度环。
 *
 * ──────────────── 注入（EXC 的 amp 在本模式下按注入类型解释单位） ────────────────
 *
 *   break：amp = 慢升/慢降速率 [N/s]，≤ 1 N/s；剖面不用。target_n 是离地搜索上限，
 *          必须在 [m_run·g, m_run·g + 5 N]。
 *   vel  ：v_inj = amp[m/s]·shape 作速度前馈，z_sp = z_hold + ∫v_inj dt；≤ 0.3 m/s。
 *   pos  ：z_sp = z_hold + amp[m]·shape（阶跃验证）；≤ 0.15 m。
 * shape(t) = 激励发生器按单位幅值归一化的剖面（ω_sp/amp，夹到 ±1）。超限拒绝开跑（不裁剪）。
 *
 * ──────────────── 安全门（中止，不限幅） ────────────────
 *
 * 测距样本 token 100 ms 不更新（height_invalid）；h_avg 低于 bottom−30 mm 或高于 upper
 * （height_window）：vel/pos 的 upper = min(bottom + lift + win, top − 40 mm)，break 的 upper = top − 10 mm
 * （2026-09-30 作者：顶到槽顶止挡不危险，别在离槽顶 6 cm 处就停）。break 上行时另按 h_avg、其斜率、
 * 80 ms 响应迟滞与 1 m/s² 保守减速距离预测会冲过 upper（height_brake）：制停中就立刻再降一档推力
 * （不中止），其他阶段中止。RAMP_DOWN 结束时 h_avg 不在 bottom+20 mm 以内（landing_unconfirmed）。
 * break 另有 no_liftoff / stop_timeout / stop_failed / no_slide / descent_timeout。
 * 本模块自己的中止先软着陆（见下面 SOFT_*）：按平均高度斜率慢降回槽底、再 1 s 降到怠速，最后才报中止。
 * 运行层的杆轴残差（ALT 至少 1.5 rad/s）与角度超限也走软着陆。遥控/解锁/油门杆/IMU/推力来源仍由
 * 运行层判、优先回根本原因，与 STOP 命令一样当拍把电机交还遥控器油门（杆在最低 = 怠速）。
 *
 * ──────────────── 竖直运动加速度 az 的来源 ────────────────
 *
 * 稳定环的导航系加速度只有水平两轴（acc_nav_m_s2[0..1]），没有 z。所以 az 取机体比力
 * （已标定、FLU、单位 g）按 roll/pitch 旋到竖直再减 g：与水平两轴同一个旋转
 * Ry(pitch)·Rx(roll)（stabilizer_compensated_imu_accel_level_xy），取它的第三行。
 * 没扣 IMU 杆臂的旋转加速度（姿态保持时角速度小），也不做倾转权重衰减。
 */

typedef enum {
    APP_SYSID_ALT_INJECT_BREAK = 0,
    APP_SYSID_ALT_INJECT_VEL,
    APP_SYSID_ALT_INJECT_POS,
    APP_SYSID_ALT_INJECT_COUNT
} APP_SysIdAltInject;

/* `SYSID ALT` 的配置：只在空闲时收、只存 RAM，上电回默认（break / 0 / 60 / 60，端点 0 = 未配置）。 */
typedef struct {
    uint8_t  inject;    /* APP_SysIdAltInject */
    uint16_t mass_g;    /* 垂直移动质量 [g]；0 = 取机体质量 */
    uint16_t win_mm;    /* 出窗余量（vel/pos）：upper = min(bottom + lift + win, top − 40 mm) */
    uint16_t lift_mm;   /* vel/pos 的保持高度（槽底以上）；break 两项都不用 */
    uint16_t bottom_mm; /* TOF 物理槽底绝对读数 [mm]，0 = 未配置 */
    uint16_t top_mm;    /* TOF 物理槽顶绝对读数 [mm]，0 = 未配置 */
} APP_SysIdAltConfig;

/*
 * 默认值按作者的槽式台架定（2026-09-29）：槽的总行程 160 mm，上端保护窗 = 槽底 + 120 mm
 * （lift 60 + win 60）与槽顶 − 40 mm 取小。换台架时按实测机械行程配置端点。
 */
#define APP_SYSID_ALT_MASS_G_MIN       500U
#define APP_SYSID_ALT_MASS_G_MAX      3000U
#define APP_SYSID_ALT_WIN_MM_DEFAULT    60U
#define APP_SYSID_ALT_WIN_MM_MIN        30U
#define APP_SYSID_ALT_WIN_MM_MAX       400U
#define APP_SYSID_ALT_LIFT_MM_DEFAULT   60U
#define APP_SYSID_ALT_LIFT_MM_MIN       30U
#define APP_SYSID_ALT_LIFT_MM_MAX      300U
#define APP_SYSID_ALT_ENDPOINT_MAX_MM 4000U
#define APP_SYSID_ALT_TRAVEL_MIN_MM    135U
#define APP_SYSID_ALT_TRAVEL_MAX_MM    185U

#define APP_SYSID_ALT_BREAK_RATE_MAX_N_S 1.0f
/* 2026-09-30 实测：杆轴在槽里滑起来要推力表读数约 14.5～15 N（机体 + 碳杆重力 11.6 N），
 * 槽不卡——随动部件或下洗损失让有效重力比称的大；+3 N 推不到，放到 +5 N。 */
#define APP_SYSID_ALT_BREAK_TARGET_EXCESS_MAX_N 5.0f
#define APP_SYSID_ALT_VEL_AMP_MAX_M_S  0.3f
#define APP_SYSID_ALT_POS_AMP_MAX_M    0.15f

#define APP_SYSID_ALT_RAMP_UP_MS        1500U
#define APP_SYSID_ALT_RAMP_UP_FRACTION  0.9f
#define APP_SYSID_ALT_SETTLE_MS         2000U
#define APP_SYSID_ALT_PREROLL_MS         500U
#define APP_SYSID_ALT_RAMP_DOWN_MS      1000U
#define APP_SYSID_ALT_REF_SPEED_M_S     0.1f
#define APP_SYSID_ALT_LAND_ABOVE_M      0.010f
#define APP_SYSID_ALT_HEIGHT_TIMEOUT_MS  100U

/* 测高平均与硬窗 */
#define APP_SYSID_ALT_AVG_SAMPLES          8U
#define APP_SYSID_ALT_START_BOTTOM_TOL_M   0.020f
#define APP_SYSID_ALT_BELOW_BOTTOM_M       0.030f
#define APP_SYSID_ALT_TOP_MARGIN_M         0.040f
#define APP_SYSID_ALT_BREAK_TOP_MARGIN_M   0.010f  /* break 没有 lift/win：上沿只离槽顶这么多 */
#define APP_SYSID_ALT_LAND_CONFIRM_M       0.020f
/* 上行提前制动门（h_avg + 余量 + v·(迟滞+样本年龄) + v²/2a ≥ upper 即中止） */
#define APP_SYSID_ALT_BRAKE_MARGIN_M       0.020f
#define APP_SYSID_ALT_BRAKE_DELAY_S        0.08f
#define APP_SYSID_ALT_BRAKE_DECEL_M_S2     1.0f
#define APP_SYSID_ALT_LUT_CAP_TOL_N        0.10f

/* break：慢升/慢降找阈值 */
/* 2026-09-30 实测：杆轴没动的慢升段里，8 样本平均高度最大偏离 10～14 mm、8 样本斜率最大 ±0.1 m/s
 * （推力接近重力时测距慢漂：桨下洗 + 杆弹性形变）。门槛都按这个定。 */
#define APP_SYSID_ALT_BREAK_MOVE_M         0.025f  /* 平均高度动了这么多才算离地/滑落 */
#define APP_SYSID_ALT_BREAK_MOVE_RATE_M_S  0.10f   /* 同时平均高度斜率要超过它 */
#define APP_SYSID_ALT_BREAK_CEIL_HOLD_MS   1000U   /* 升到搜索上限后再等这么久仍未离地 → no_liftoff */
/* 制停第一档：静摩擦 > 动摩擦时离地会"蹦"。按离地时的上升速度 v 与已升高度 z 估多出的净推力
 * m·v²/(2z)（v 取平均高度斜率，偏滞后），第一档降"估值 + 0.5 N"，夹在 0.6～1.5 N。 */
#define APP_SYSID_ALT_BREAK_STOP_FIRST_MIN_N 0.6f
#define APP_SYSID_ALT_BREAK_STOP_FIRST_MAX_N 1.5f
#define APP_SYSID_ALT_BREAK_STOP_MARGIN_N    0.5f
#define APP_SYSID_ALT_BREAK_STOP_STEP_N    0.4f    /* 之后每一档降多少推力 */
#define APP_SYSID_ALT_BREAK_CHECK_MS        200U   /* 制停时两次降档至少隔这么久 */
#define APP_SYSID_ALT_BREAK_STILL_M        0.012f  /* 200 ms 内平均高度净变化不超过它算"停住" */
/* 100 ms 内平均高度还升了这么多（>0.12 m/s）才算"还在升"。2026-09-30 实机降第一档后以约 0.1 m/s 匀速上爬、
 * 够不着它：交给提前制动门在上沿前每 100 ms 降一档。改成"200 ms 净升 12 mm"的仿真里降过头滑回槽底、
 * 测不到滑落的轮次多了近一倍（data/analysis/sysid-rig-params/2026-09-30/soft_landing_replay_031818.txt）。 */
#define APP_SYSID_ALT_BREAK_RISE_M         0.012f
#define APP_SYSID_ALT_BREAK_DECEL_SLACK_M  0.004f  /* 这 100 ms 比上个 100 ms 少升超过它才算"明显在减速"（噪声约 3 mm） */
#define APP_SYSID_ALT_BREAK_QUIET_MS        800U   /* 距上次降推力至少这么久才判"停住" */
#define APP_SYSID_ALT_BREAK_STOP_MAX_MS    4000U
#define APP_SYSID_ALT_BREAK_FLOOR_FRACTION 0.7f    /* 制停/慢降的推力下限 = 0.7·m_run·g（静摩擦大时滑落阈值很低） */
#define APP_SYSID_ALT_BREAK_AT_BOTTOM_M    0.020f  /* 平均高度离基线不超过它算"在槽底" */
#define APP_SYSID_ALT_BREAK_FAST_DOWN_M    0.020f  /* 100 ms 内平均高度降超过它（>0.2 m/s）就刹车 */
#define APP_SYSID_ALT_BREAK_BRAKE_N        0.4f
#define APP_SYSID_ALT_BREAK_BRAKE_MS        200U
#define APP_SYSID_ALT_BREAK_LAND_STILL_M   0.010f  /* 200 ms 内平均高度变化不超过它算落稳 */
#define APP_SYSID_ALT_BREAK_DESCEND_MAX_MS 4000U
#define APP_SYSID_ALT_BREAK_BRAKE_STEP_MS   100U   /* 制停中 height_brake 两次降档至少隔这么久 */

/* 软着陆（本模块自己的中止）。2026-09-30 height_brake 当拍从 14.3 N 掉到怠速，机体从槽底以上约 8 cm
 * 摔回槽底、弹跳。现在按 8 样本平均高度的斜率 v 调推力：还在往上走降得快，停着慢慢降，缓降时保持，
 * 降得太快就往回加（不超过进入时的推力）；回到槽底附近或推力降到怠速再 1 s 降到怠速，才报中止。 */
#define APP_SYSID_ALT_SOFT_SKIP_FRACTION   0.5f    /* 推力不到 0.5·m_run·g：机体必压在槽底，直接中止 */
#define APP_SYSID_ALT_SOFT_AVG_TAU_S       0.5f    /* 起降推力 = min(当前, 近 0.5 s 一阶平均)，往回加上限取大者 */
#define APP_SYSID_ALT_SOFT_STOP_N_S        3.0f    /* v > +0.03 m/s（还在升）时每秒降这么多 */
#define APP_SYSID_ALT_SOFT_DOWN_N_S        1.5f    /* |v| ≤ 0.03 m/s（停着）时每秒降这么多 */
#define APP_SYSID_ALT_SOFT_BRAKE_N_S       6.0f    /* v < −0.08 m/s（降太快）时每秒加回这么多 */
#define APP_SYSID_ALT_SOFT_STILL_M_S       0.03f
#define APP_SYSID_ALT_SOFT_FAST_M_S        0.08f
#define APP_SYSID_ALT_SOFT_AT_BOTTOM_M     0.035f  /* 槽底读数以上这么多以内算回到槽底（含杆被顶直的 10～25 mm） */
#define APP_SYSID_ALT_SOFT_MAX_MS          8000U   /* 超时也转入 1 s 降到怠速 */
#define APP_SYSID_ALT_SOFT_BLIND_MS        2000U   /* 测高断了看不到高度：按时间 2 s 线性降到怠速 */
/* 槽式台架上杆两端各在一条槽里，绕"水平且垂直于杆"的轴本来就能转（两轴都由姿态环保持）。2026-09-30 两轮
 * 离地前后杆一端卡住、歪到约 3.5° 后突然松开，绕它的角速度超过单轴台架的残差门 0.52 rad/s，当拍断电摔下。
 * ALT 的残差门至少放宽到这么多，残差/角度超限都走软着陆（遥控/上锁/IMU 等仍当拍交还）。 */
#define APP_SYSID_ALT_RESIDUAL_MIN_RAD_S   1.5f

/* ---------------------------------------------------------------- 配置面（命令任务） */

void    APP_SysIdAlt_GetConfig(APP_SysIdAltConfig *out);
/* 只做范围体检；是否在跑由调用方判（运行中拒收是命令面的事）。0 = 越界，原配置不动。 */
uint8_t APP_SysIdAlt_SetConfig(const APP_SysIdAltConfig *config);
const char *APP_SysIdAlt_InjectName(uint8_t inject);
uint8_t APP_SysIdAlt_InjectFromName(const char *name, uint8_t *out);
/* 回显一行，含端点与 control=breakaway（break）或 closed_loop（vel/pos）。 */
void    APP_SysIdAlt_ReportConfig(void);

/* 竖直运动加速度 [m/s²]，向上为正、已去 g（来源见文件头）。纯函数，稳定环填观测时调。 */
float APP_SysIdAlt_VerticalAccel(float accel_x_g, float accel_y_g, float accel_z_g,
                                 float roll_rad, float pitch_rad, float gravity_m_s2);

/* ---------------------------------------------------------------- 运行面（只由 app_sysid.c 调） */

/* 每个控制拍（跑不跑都调）：记下最近的测高（含 8 样本平均），供开跑检查、h0 与状态行。 */
void APP_SysIdAlt_Observe(const APP_SysIdObserve *obs);
/* 开跑前检查（端点、开跑在槽底、break 的搜索上限与速率）；NULL = 可开跑，否则回拒绝理由。 */
const char *APP_SysIdAlt_Precheck(float throttle_target_n, const DRV_SysIdExcitation *spec);
/* 开跑：锁存配置、剖面、搜索上限、油门上限、m_run 与 h0。运行层在发布 RUNNING 前调。 */
void APP_SysIdAlt_Begin(const DRV_SysIdExcitation *spec, float target_n, float max_pct);
/* 紧跟 SYSID start 的溯源行，含 alt_control、alt_target_cn（break 搜索上限）与端点。 */
void APP_SysIdAlt_ReportStart(uint16_t run_id);

typedef struct {
    APP_SysIdPhase phase;       /* 本拍所处阶段；与传入的不同 = 本拍切换 */
    uint16_t motor_pulse_us;    /* 两路电机共用脉宽，已按最高油门 % 封顶 */
    float    thrust_n;          /* 该脉宽经推力查补表读回的合推力（记录的 thrust） */
    uint8_t  capped;            /* 本拍被封顶 */
    uint8_t  hold_attitude;     /* 1 = 姿态链保持杆轴角 0；低于反解推力下限才回中 */
    uint8_t  check_thrust;      /* 1 = 推力来源与推力下限照判 */
    uint8_t  finished;          /* 回落走完且落地确认：本轮完成 */
    const char *abort_reason;   /* 见文件头「安全门」；另有 excitation / controller / phase */
} APP_SysIdAltOutput;

/* 一个控制拍：阶段推进 → 高度门 → break 直接推力或 vel/pos 高度环 → 脉宽。 */
void APP_SysIdAlt_Step(const APP_SysIdObserve *obs, APP_SysIdPhase phase, uint32_t phase_ms,
                       float dt_s, APP_SysIdAltOutput *out);
/* 运行层要中止（残差/角度）时先软着陆，本拍接着调 Step 即按软着陆出力，走完报 reason。
 * 返回 0 = 推力还不到 0.5·m·g（机体必在槽底），调用方照原样当拍中止；已在软着陆返回 1。 */
uint8_t APP_SysIdAlt_BeginSoftLanding(const APP_SysIdObserve *obs, const char *reason);
/* 把记录尾部 7 个高度字段填进样本（测高无效的那一拍 height/height_raw/vz 记 0）。 */
void APP_SysIdAlt_FillSample(DRV_SysIdSample *sample, const APP_SysIdObserve *obs);
/* 状态行：最近一拍的测高 [m] 与是否有效、本轮当前高度参考 [m]。 */
void APP_SysIdAlt_GetLive(float *height_m, uint8_t *valid, float *height_sp_m);

#endif /* APP_SYSID_ALT_H */

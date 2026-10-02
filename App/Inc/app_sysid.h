#ifndef APP_SYSID_H
#define APP_SYSID_H

#include <stddef.h>
#include <stdint.h>

#include "drv_sysid_excitation.h"
#include "drv_sysid_record.h"
#include "drv_sysid_rig.h"

/*
 * 内环系统辨识的运行层：状态机、安全门、500 Hz 执行、采样环形缓冲。
 *
 * ──────────────── 这个模块做的那一件事 ────────────────
 *
 * 给一条**期望角速度剖面** ω_sp(t)，在每个控制拍上把它变成舵机脉宽，并把
 * "发出去的"和"量回来的"同时记下来。整条链是：
 *
 *     ω_sp, α_ff = DRV_SysIdExcitation_Eval(spec, t)      —— 纯函数，无状态
 *     τ_n        = I_est · α_ff                            —— 假定惯量
 *     τ_body     = DRV_SysIdRig_MomentAboutAxis(τ_n)       —— 打在杆轴上
 *     倾角        = DRV_COAX_CTRL_SolveBodyTiltFromMoment  —— **在飞的那个反解器**
 *     脉宽        = DRV_COAX_CTRL_BodyTiltRadToServoPulses
 *
 * 一行新的力学都不写。这不是为了省事：辨识如果自己再实现一遍力矩分配，辨出来
 * 的就是"辨识程序的模型"而不是"飞控的模型"，拿去整定 PID 会系统性地偏。
 *
 * 前馈只是初始激励。惯量估计依赖有效推力标定、已知力臂、无饱和及充分激励；
 * 接近初始惯量不代表验证通过。RATE/ANGLE 模式复用生产控制器验证候选参数。
 *
 * ──────────────── 油门归属（作者 2026-09-26 授权） ────────────────
 *
 * 解锁永远归飞手；**解锁且油门杆在最低**时点开始，本模块才接管油门：
 * 升油门 → 稳定 → 激励 → 降油门。目标是合推力（N），经分配器里装的同一张推力表
 * （DRV_COAX_CTRL_ThrustToMotorPulse，与读回推力同源）换成脉宽，再被"最高油门 %"封顶。本模块只算脉宽（`APP_SysId_GetMotorPulse`），
 * 真正写电调的仍是稳定环里"已解锁 + 链路正常"那一支——上锁、失联走的都是
 * 别的分支，辨识接不到电机。飞手推油门杆、上锁、失联、点停止、任何安全门
 * 触发，都立刻把油门交还遥控器（杆在最低即怠速）。target_n=0 为旧的手动模式：
 * 推力由遥控器给，本模块只读回。
 *
 * 源码里仍不许出现 DRV_Motor / BSP_PWM_SetEscPulse（`tests/test_sysid_runtime_contract.py`）。
 *
 * ──────────────── 舵机单独辨识（SERVO，模式 3，2026-09-27） ────────────────
 *
 * 电机不转、推力为零时，杆上的机体只会被舵机的反作用力矩带动（"尾巴摇狗"）。
 * 一趟就能单独量出舵机动态、死区时间、反作用惯量和齿隙，推力不在回路里。
 *
 *   - 开跑要求**未解锁**（电机转不起来的前提），解锁中拒绝；跑的过程中飞手
 *     一解锁立即中止（rc_arm）。自动油门设置在本模式下忽略，电机一拍都不碰：
 *     `APP_SysId_GetMotorPulse` 恒返回 0。
 *   - 阶段：500 ms 零激励前导（照常采样，同一采样网格）→ 激励 → 结束。
 *   - 倾转指令 = servo_tilt · shape(t) · d，shape 是现有激励发生器按单位幅值
 *     归一化的剖面（ω_sp/amp），幅值由 EXC 的 servo_tilt_mrad 给，夹在控制器
 *     倾角上限内（记录的就是夹过的值）。舵机脉宽若再被标定行程 min/max 夹住
 *     （顶到端点），记录就不再是舵机真走的量：中止，理由 actuator_saturated。
 *     方向 d 取"有推力时 FF 为绕杆轴正力矩会下发的倾转方向"：
 *         d = (sinψ·sgn(L_pitch), cosψ·sgn(L_roll))       （机体倾转 X, Y）
 *     推导：τ_roll = L_roll·T·sin(tilt_y)，τ_pitch = L_pitch·T·sin(tilt_x)·cos(tilt_y)，
 *     杆轴 n = (cosψ, sinψ)，FF 下发的力矩 = τ_n·n，逐轴取号即得 d。
 *   - 记录里的 servo_tilt 字段对所有模式都按同一个投影算：
 *         servo_tilt = tilt_x·sinψ·sgn(L_pitch) + tilt_y·cosψ·sgn(L_roll)
 *     SERVO 下它就是标量指令本身；torque/thrust/omega_sp/alpha_ff/angle_sp 恒为 0。
 *
 * ──────────────── 高度辨识（ALT，模式 4，R-ALTID-1，2026-09-29） ────────────────
 *
 * 杆轴装在竖直槽里，机体连杆一起上下动。force 开环对象辨识、vel/pos 生产 Z 高度环
 * 候选验证的序列都在 app_sysid_alt.h/.c；本模块只把它接到已有的
 * 安全门、姿态链（与 ANGLE 同一条，目标恒为 0）、采样网格和油门出口上。force 使用
 * target_n 作离底搜索合推力上限，离底后锁存已下发的 LUT 推力作短激励基线；vel/pos
 * 保留生产 Z 闭环。全程采样，含起升与回落。
 *
 * 上/下桨 eRPM 在采样拍从双向 DShot 回包快照里取（与推力查补表同一份数据、同一
 * 新鲜度），桨位由桨叶标定（drv_prop_map）按角色查通道。它只进记录、不进任何
 * 判据，所以不占稳定环填的观测结构。
 *
 * 不写 Flash。整定出来的候选增益由上位机经 `SYSID PARAM` 只写 RAM 实时验证，
 * 是否落盘是作者事后的单独动作。
 */

typedef enum {
    APP_SYSID_STATE_IDLE = 0,
    APP_SYSID_STATE_RUNNING,
    APP_SYSID_STATE_DONE,
    APP_SYSID_STATE_ABORTED
} APP_SysIdState;

/* 一轮内的阶段。手动油门模式直接从 EXCITE 开始、跑完即结束。
 * SERVO 模式走 PREROLL → EXCITE → DONE；ALT 走 RAMP_UP → CLIMB → SETTLE → PREROLL →
 * EXCITE → DESCEND → RAMP_DOWN。枚举值只追加，不重排。 */
typedef enum {
    APP_SYSID_PHASE_IDLE = 0,
    APP_SYSID_PHASE_RAMP_UP,
    APP_SYSID_PHASE_SETTLE,
    APP_SYSID_PHASE_EXCITE,
    APP_SYSID_PHASE_RAMP_DOWN,
    APP_SYSID_PHASE_PREROLL,
    APP_SYSID_PHASE_DONE,
    APP_SYSID_PHASE_CLIMB,      /* ALT：高度参考从开跑高度匀速爬到保持高度 */
    APP_SYSID_PHASE_DESCEND     /* ALT：激励后高度参考匀速回到开跑高度上方 10 mm */
} APP_SysIdPhase;

#define APP_SYSID_RAMP_UP_MS   1500U
#define APP_SYSID_SETTLE_MS    1500U
/* 稳定段最后这一段照常采样（零激励前导）：拟合要丢掉开头约 0.5 s 的起始瞬态，
 * 让它落在前导上而不是吃掉激励（2026-09-27 两轮实录跨轮验证的结论）。 */
#define APP_SYSID_PREROLL_MS    500U
#define APP_SYSID_RAMP_DOWN_MS 1000U
#define APP_SYSID_THROTTLE_PCT_MIN 10.0f
#define APP_SYSID_THROTTLE_PCT_MAX 95.0f

typedef enum {
    APP_SYSID_FEEDFORWARD = 0,
    APP_SYSID_RATE,
    APP_SYSID_ANGLE,
    APP_SYSID_SERVO,         /* 3：电机不转，只动舵机（见文件头） */
    APP_SYSID_ALT,           /* 4：光杆台架高度辨识（见文件头与 app_sysid_alt.h） */
    APP_SYSID_XY,            /* 5：水平槽 XY 速度/位置辨识（app_sysid_xy.h） */
    APP_SYSID_YAW            /* 6：吊绳偏航辨识（app_sysid_yaw.h）：上下桨差速，舵机全程中位 */
} APP_SysIdMode;
uint8_t APP_SysId_SetMode(APP_SysIdMode mode, float angle_amplitude_rad);

/* SERVO 模式的倾转幅值 [rad]：默认 87 mrad（5°），允许 10..262 mrad。仅空闲时可改。 */
#define APP_SYSID_SERVO_TILT_DEFAULT_RAD 0.087f
#define APP_SYSID_SERVO_TILT_MIN_RAD     0.010f
#define APP_SYSID_SERVO_TILT_MAX_RAD     0.262f
uint8_t APP_SysId_SetServoTilt(float rad);
float   APP_SysId_GetServoTilt(void);
APP_SysIdMode APP_SysId_GetMode(void);
/* Keep centered bench ownership through STOP/abort/completion until RC disarms. */
uint8_t APP_SysId_IsEngaged(void);
uint8_t APP_SysId_NeedsService(void);
uint8_t APP_SysId_Discard(void); /* explicit cancellation of stopped-run transmission */
void APP_SysId_Hold(void); /* center-only standby; RC still owns throttle */
/* Target port binds the stream to the START command's link (not a later command). */
uint8_t APP_SysId_PortBegin(uint32_t rate_hz);
uint8_t APP_SysId_PortSend(const uint8_t *payload, uint16_t length);

/* 自动油门：target_n=0 关闭（遥控器给油门）；否则 [MIN_THRUST, 机体最大合推力]。仅空闲时可改。 */
uint8_t APP_SysId_SetThrottle(float target_n, float max_pct);
void    APP_SysId_GetThrottle(float *target_n, float *max_pct);
/* 本拍辨识是否接管电机；是则给出两路共用的脉宽。只由稳定环的"已解锁+链路正常"分支调用。
 * SERVO 模式恒返回 0。 */
uint8_t APP_SysId_GetMotorPulse(uint16_t *pulse_us);
/* 同上，但分上/下桨给：YAW 模式两桨脉宽不同（差速出偏航力矩），其余模式两者逐位相同（= GetMotorPulse）。 */
uint8_t APP_SysId_GetMotorPulsePair(uint16_t *upper_us, uint16_t *lower_us);
APP_SysIdPhase APP_SysId_GetPhase(void);
/* ALT/XY 台架模式本轮的姿态保持目标 att0（控制角坐标，rad）；诊断与主机测试用。 */
void APP_SysId_GetBenchAtt0(float *roll_rad, float *pitch_rad);

/* 一个控制拍交给辨识的全部外界信息。调用方（稳定环）负责填，本模块不自己取数。 */
typedef struct {
    uint32_t now_ms;
    uint32_t now_us;          /* SVC_Timestamp_Us() 低 32 位；样本时间戳用它 */
    float    gyro_rad_s[3];   /* 规范 FLU 机体角速度 */
    float    roll_rad;        /* 融合姿态，仅用于限位与交叉校验 */
    float    pitch_rad;
    uint16_t throttle_us;     /* 遥控器给的电机脉宽；自动油门时是起升/回落的基准 */
    uint8_t  rc_link_ok;
    uint8_t  rc_armed;
    uint8_t  imu_valid;
    uint8_t  actuator_inhibit; /* acceptance/calibration/TBENCH/legacy IDENT owns output */
    uint8_t  thrust_valid; /* current LUT voltage/eRPM provenance is fresh/in range */
    uint8_t  rc_throttle_low; /* 油门杆在解锁用的"最低"档内；自动油门期间离开即交还 */
    /*
     * 控制用陀螺（转速陷波之后，app_rpm_notch.h）：只喂 RATE/ANGLE 的闭环 PID，与在飞
     * 控制器同一口径。记录、杆轴残差门、SERVO 模式仍用上面的原始 gyro_rad_s——
     * 记录与历史实录连续，残差门看的是真实运动。陷波关时两者逐位相等。
     */
    float    gyro_ctrl_rad_s[3];
    /*
     * 高度辨识（ALT）用，稳定环每拍都填；非 ALT 轮本模块一概不读（记录里那 7 个字段填 0）。
     * height_m 是 SVC_FlowNav 滤波后的 TOF 高度——生产 relative_height 减上电原点之前的
     * 那个量（ALT 全程以开跑高度为基准，原点常数不影响控制）；height_raw_m 是滤波前的
     * 原始测距；vz_m_s 与生产高度环同源。height_sample_ms 是独立测距样本的 HAL ms
     * 更新 token（不是当前控制拍时刻），用来判断新样本及 100 ms 新鲜度。
     * az_m_s2 的来源见 APP_SysIdAlt_VerticalAccel。
     * vbat_v 取电池快照（不新鲜时 0）。
     */
    uint8_t  height_valid;
    uint32_t height_sample_ms;
    float    height_m;
    float    height_raw_m;
    float    vz_m_s;
    float    az_m_s2;
    float    vbat_v;
    /*
     * 水平槽 XY 辨识用（app_sysid_xy.h），稳定环每拍只读填写，非 XY 轮本模块一概不读。
     * flow_valid = 光流速度有效且测距有效（导航位置有效的同一判据）；flow_sample_ms 是光流速度
     * 样本的 HAL ms 更新 token（不是控制拍时刻），用来判断 200 ms 新鲜度；速度取 EKF 融合后的
     * 水平速度、位置取其限幅累计（SVC_FlowNav_GetVelocity/GetPosition），规范 FLU（x 前 / y 左）。
     */
    uint8_t  flow_valid;
    uint32_t flow_sample_ms;
    float    flow_vel_m_s[2];
    float    flow_pos_m[2];
    /*
     * 重力水平在 roll_rad/pitch_rad 这套坐标里的位置（= −开机姿态零点）。roll_rad/pitch_rad
     * 是扣过开机零点的控制角，开机时机体挂在台架上就把悬挂姿态当成了 0。XY 绕杆轴的保持目标
     * 要用真水平，否则推力一直偏一个悬挂角（2026-09-30 台架：+2.2°，浮起推力下自己溜 11 cm）。
     * level_valid=0（零点未就绪）时 XY 退回开跑姿态。
     */
    uint8_t  level_valid;
    float    level_roll_rad;
    float    level_pitch_rad;
} APP_SysIdObserve;

/* 采样率档位。500 Hz 是控制拍本身，再高没有意义（没有新样本）。 */
#define APP_SYSID_RATE_MIN_HZ   50U
#define APP_SYSID_RATE_MAX_HZ   500U
#define APP_SYSID_RATE_DEFAULT_HZ 250U

/*
 * 绕杆轴的角度硬限。激励目标是 ±15°，这里留 5° 余量。
 * 超限 abort 而不是限幅：限幅会让一段被削顶的数据看起来仍然"跑完了"。
 */
#define APP_SYSID_ANGLE_LIMIT_DEFAULT_RAD 0.3490658504f  /* 20 deg */
#define APP_SYSID_ANGLE_LIMIT_MAX_RAD     0.5235987756f  /* 30 deg */

/*
 * 轴向残差门 [rad/s]。理想台架上 ω 必须平行于杆轴，垂直分量恒为 0。
 * 超限说明杆不在填的方位角上、机体没夹紧、或轴向标定错了——这一趟数据作废。
 */
#define APP_SYSID_RESIDUAL_LIMIT_DEFAULT_RAD_S 0.5f

/*
 * 开跑所需的最小总推力 [N]。低于它反解会解出超限倾角（τ/(l·F) 的分母太小），
 * 而且没有推力的倾转本来也产生不了力矩，那一趟只会记下一串零。
 */
#define APP_SYSID_MIN_THRUST_N 2.0f

/* 采样环形缓冲的条数。500 Hz 下约 256 ms，够遥测任务 40 Hz 拍的排空节奏用。 */
#define APP_SYSID_RING_SAMPLES 128U

void APP_SysId_Init(void);

/* ---------------------------------------------------------------- 配置面 */

uint8_t APP_SysId_SetRig(const DRV_SysIdRig *rig);
void    APP_SysId_GetRig(DRV_SysIdRig *out);

uint8_t APP_SysId_SetExcitation(const DRV_SysIdExcitation *spec);
void    APP_SysId_GetExcitation(DRV_SysIdExcitation *out);

/* 前馈用的假定惯量 [kg·m²]。为 0 时开跑会从 airframe.ixx_kgm2 取。 */
uint8_t APP_SysId_SetInertia(float kg_m2);
float   APP_SysId_GetInertia(void);

uint8_t APP_SysId_SetSampleRate(uint32_t hz);
uint32_t APP_SysId_GetSampleRate(void);

uint8_t APP_SysId_SetAngleLimit(float rad);
float   APP_SysId_GetAngleLimit(void);

uint8_t APP_SysId_SetResidualLimit(float rad_s);
float   APP_SysId_GetResidualLimit(void);

/* ---------------------------------------------------------------- 运行面 */

/*
 * 开跑。返回 0 表示被拒绝，拒绝理由已经用文本回出去了——不要静默失败，
 * "按了没反应"是这类台架程序最常见的坑。
 */
uint8_t APP_SysId_Start(void);
void    APP_SysId_Stop(const char *reason);

uint8_t APP_SysId_IsRunning(void);
APP_SysIdState APP_SysId_GetState(void);
const char *APP_SysId_GetLastReason(void);
uint16_t APP_SysId_GetRunId(void);

/* 控制拍的一次推进：安全门 -> 激励 -> 反解 -> 采样入环。 */
void APP_SysId_Update(const APP_SysIdObserve *obs);

/* 本拍要下发的舵机脉宽。未在跑时给出中位。 */
void APP_SysId_GetServoTargets(uint16_t *alpha_us, uint16_t *beta_us);

void APP_SysId_ReportStatus(void);
void APP_SysId_ReportThrottle(void);   /* 只报 SYSID THR 一行 */
void APP_SysId_ReportSchema(void);

/* ---------------------------------------------------------------- 发送面 */

/*
 * 取走一批样本并打包成 `APP_PROTO_MSG_SYSID_BATCH` 的 payload。
 * 返回 0 表示这一拍没有成批的数据可发（不是错误）。
 *
 * **一批绝不跨越采样间断**：环里丢过样本时本批就在断点处截断，下一批带
 * `DRV_SYSID_FLAG_GAP`。因为线上时间戳是 `base + k·dt` 的压缩形式，
 * 跨断点拼一批会让丢掉的那几拍在主机上表现成"时间被压缩了"，
 * 而时移正是这套辨识要测的东西——那种错会直接变成一个假的延迟值。
 *
 * 批头的模式标志（RATE/ANGLE/SERVO）是**开跑时锁存**的本轮模式，不是排空时的
 * 当前模式：跑完、尾批还没发完时 `SYSID MODE` 照样能改。
 */
uint8_t APP_SysId_PopBatch(uint8_t *out, size_t capacity, size_t *out_len);

/* 挂在遥测任务拍上的排空。本身不采样，只负责把环里的东西发出去。 */
void APP_SysId_StreamTick(void);

/* 丢样计数（环满）。诊断用，`SYSID?` 会报。 */
uint32_t APP_SysId_GetDroppedSamples(void);

/*
 * 命令面（`SYSID ...` 与搬出来的 `IDENT ...`）声明在 App/Inc/app_control_internal.h，
 * 和其余 `app_control_handle_*` 放在一处——`app_control.c` 本来就只包含那一个头，
 * 在它里面为这件事再加一行 #include 就是往只减不增的文件里加行。
 */

#endif /* APP_SYSID_H */

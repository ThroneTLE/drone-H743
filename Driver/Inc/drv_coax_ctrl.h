#ifndef DRV_COAX_CTRL_H
#define DRV_COAX_CTRL_H

#include <stdint.h>
#include "drv_attitude_control.h"
#include "drv_position_control.h"
#include "drv_rate_control.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DRV_COAX_CTRL_SERVO_ALPHA_CENTER_US 1500U
#define DRV_COAX_CTRL_SERVO_BETA_CENTER_US  1500U
#define DRV_COAX_CTRL_SERVO_PHYSICAL_MIN_US  500U
#define DRV_COAX_CTRL_SERVO_PHYSICAL_MAX_US 2500U
#define DRV_COAX_CTRL_SERVO_LIMIT_DELTA_US  1000U
#define DRV_COAX_CTRL_SERVO_CENTER_MIN_US(center_us) \
    (((center_us) > (DRV_COAX_CTRL_SERVO_PHYSICAL_MIN_US + \
                     DRV_COAX_CTRL_SERVO_LIMIT_DELTA_US)) ? \
     ((center_us) - DRV_COAX_CTRL_SERVO_LIMIT_DELTA_US) : \
     DRV_COAX_CTRL_SERVO_PHYSICAL_MIN_US)
#define DRV_COAX_CTRL_SERVO_CENTER_MAX_US(center_us) \
    ((((center_us) + DRV_COAX_CTRL_SERVO_LIMIT_DELTA_US) < \
      DRV_COAX_CTRL_SERVO_PHYSICAL_MAX_US) ? \
     ((center_us) + DRV_COAX_CTRL_SERVO_LIMIT_DELTA_US) : \
     DRV_COAX_CTRL_SERVO_PHYSICAL_MAX_US)
#define DRV_COAX_CTRL_SERVO_ALPHA_MIN_US \
    DRV_COAX_CTRL_SERVO_CENTER_MIN_US(DRV_COAX_CTRL_SERVO_ALPHA_CENTER_US)
#define DRV_COAX_CTRL_SERVO_ALPHA_MAX_US \
    DRV_COAX_CTRL_SERVO_CENTER_MAX_US(DRV_COAX_CTRL_SERVO_ALPHA_CENTER_US)
#define DRV_COAX_CTRL_SERVO_BETA_MIN_US \
    DRV_COAX_CTRL_SERVO_CENTER_MIN_US(DRV_COAX_CTRL_SERVO_BETA_CENTER_US)
#define DRV_COAX_CTRL_SERVO_BETA_MAX_US \
    DRV_COAX_CTRL_SERVO_CENTER_MAX_US(DRV_COAX_CTRL_SERVO_BETA_CENTER_US)
#define DRV_COAX_CTRL_SERVO_TRAVEL_DEG       180.0f
#define DRV_COAX_CTRL_SERVO_LIMIT_DEG         90.0f
#define DRV_COAX_CTRL_SERVO_COUNT               2U
#define DRV_COAX_CTRL_SERVO_ALPHA_INDEX         0U
#define DRV_COAX_CTRL_SERVO_BETA_INDEX          1U
#define DRV_COAX_CTRL_SERVO_MIN_CAL_SPAN_US    50U

#define DRV_COAX_CTRL_PROTECT_VELOCITY_INVALID (1UL << 0)
#define DRV_COAX_CTRL_PROTECT_ATTITUDE         (1UL << 1)
#define DRV_COAX_CTRL_PROTECT_MOMENT           (1UL << 2)
#define DRV_COAX_CTRL_PROTECT_THRUST           (1UL << 3)

typedef struct {
    /*
     * Paper p_d, p_d_dot and p_d_ddot references in the local controller
     * frame: canonical FLU (Driver/Inc/drv_frame_contract.h) -- +X forward,
     * +Y left, +Z up.
     *
     * R-F6-2 (2026-09-06): migrated from the legacy local-level frame
     * (+X forward, +Y right, +Z down, z = -height above ground).  The +Y
     * migration negates the legacy right-positive navigation Y at the named
     * App seam2->seam3 boundary.  +Z also flips sign (height above ground is
     * now +z directly, no negation).  See
     * doc/req-rf6-2-controller-flu-migration.md section 3 and
     * DRV_COAX_CTRL_AttitudeInput for the full seam 3 frame map.
     */
    float x_m;
    float y_m;
    float z_m;
    float vx_m_s;
    float vy_m_s;
    float vz_m_s;
    float ax_m_s2;
    float ay_m_s2;
    float az_m_s2;
    float dt_sec;
    uint8_t horizontal_velocity_valid;
    uint8_t navigation_position_valid;
    uint8_t navigation_velocity_valid;
    uint8_t position_control_bypass;
    uint8_t direct_attitude_target_valid;
    uint8_t manual_total_force_valid;
    float target_roll_rad;
    float target_pitch_rad;
    float manual_total_force_n;
    /* Paper psi_d, psi_d_dot and psi_d_ddot references. */
    float yaw_rad;
    float yaw_rate_rad_s;
    float yaw_accel_rad_s2;
} DRV_COAX_CTRL_Reference;

typedef struct {
    /*
     * Paper p, p_dot and attitude feedback after App-layer sensor mounting
     * correction.  Canonical FLU throughout (Driver/Inc/drv_frame_contract.h)
     * -- +X forward, +Y left, +Z up; +roll right wing down, +pitch nose down,
     * +yaw nose left.  roll_rad/pitch_rad/yaw_rad/gyro_*_rad_s were already
     * canonical FLU before R-F6-2 (seam 0/1).  x_m/y_m/z_m/vx_m_s/vy_m_s/
     * vz_m_s must carry the SAME frame as DRV_COAX_CTRL_Reference: they feed
     * the same position/velocity error computation
     * (coax_ctrl_compute_accel_cmd), and migrating one side without the
     * other makes that error physically inconsistent.
     *
     * R-F6-2 (2026-09-06): the driver used to re-derive its own "force
     * frame" from this attitude via DRV_COAX_CTRL_FORCE_FRAME_ROLL_SIGN /
     * _PITCH_SIGN and DRV_COAX_CTRL_RATE_FRAME_ROLL_SIGN / _PITCH_SIGN,
     * because the position/velocity side of this same struct (and of
     * DRV_COAX_CTRL_Reference) still used the legacy local-level frame.
     * That per-axis sign is not a similarity transform on the SO(3) attitude
     * error -- measured 2026-09-06: flipping just the roll constant moved
     * beta by up to 67.5 mrad against a 0.8 mrad servo tolerance (see
     * tests/test_flu_seam3_force_frame_derivation.py) -- so it did not
     * "cancel"; it was a real, load-bearing correction for a real frame
     * mismatch.  Once both structs are genuinely FLU throughout, the
     * standard ZYX Euler rotation matrix (coax_ctrl_rpy_matrix) is already
     * correct for FLU's own right-handed axes with no extra sign, because
     * FLU's roll/pitch/yaw signs are themselves defined by the right-hand
     * rule about FLU's own X/Y/Z (see drv_frame_contract.h).  The four
     * constants are therefore deleted, not re-valued.
     *
     * The +Y question is pinned by the 2026-08-30 left_y recording: seam2's
     * navigation output is right-positive and is negated at the named App
     * boundary before entering this FLU interface.  See
     * doc/req-rf6-2-controller-flu-migration.md section 3.
     */
    float x_m;
    float y_m;
    float z_m;
    float vx_m_s;
    float vy_m_s;
    float vz_m_s;
    float roll_rad;
    float pitch_rad;
    float yaw_rad;
    float gyro_x_rad_s;
    float gyro_y_rad_s;
    float gyro_z_rad_s;
    /* Measured local acceleration for the velocity-loop D term [m/s^2]. */
    float accel_m_s2[3];
    uint8_t acceleration_valid;
} DRV_COAX_CTRL_AttitudeInput;

typedef struct {
    uint8_t position_update;
    uint8_t velocity_update;
    uint8_t attitude_update;
    uint8_t rate_update;
    uint8_t integrator_enable;
    uint8_t integrator_freeze;
    uint8_t integrator_reset;
    float position_dt_s;
    float velocity_dt_s;
    float attitude_dt_s;
    float rate_dt_s;
} DRV_COAX_CTRL_Schedule;

typedef struct {
    float thrust_upper_n;
    float thrust_lower_n;
    float alpha_rad;
    float beta_rad;
    uint16_t motor_upper_us;
    uint16_t motor_lower_us;
    uint16_t servo_alpha_us;
    uint16_t servo_beta_us;
    float moment_achieved_n_m[3];
    uint8_t saturation_positive[3];
    uint8_t saturation_negative[3];
    uint8_t thrust_saturated;
    uint8_t tilt_saturated;
    uint8_t yaw_differential_saturated;
} DRV_COAX_CTRL_Output;

typedef struct {
    float pos_p_m_s2[3];
    float pos_z_i_m_s2;
    float vel_d_m_s2[3];
    float accel_out_m_s2[3];
    float force_cmd_n[3];
    float target_attitude_rp_rad[2];
    float tilt_angle_p_rad[2];
    float tilt_rate_d_rad[2];
    float tilt_out_rad[2];
    float yaw_angle_p_rad_s;
    float yaw_rate_d_rad_s;
    float yaw_torque_cmd;
    float total_force_n;
    float motor_thrust_cmd_n[2];
    float motor_cmd_us[2];
    float desired_attitude_rpy_rad[3];
    float attitude_error[3];
    float rate_error_rad_s[3];
    float moment_cmd_n_m[3];
    float moment_achieved_n_m[3];
    float position_sp_m[3];
    float position_m[3];
    float position_error_m[3];
    float velocity_ff_m_s[3];
    float velocity_sp_m_s[3];
    float velocity_m_s[3];
    float velocity_error_m_s[3];
    float velocity_p_m_s2[3];
    float velocity_i_m_s2[3];
    float velocity_d_m_s2[3];
    float velocity_ff_m_s2[3];
    float accel_unsat_m_s2[3];
    float omega_ff_rad_s[3];
    float omega_sp_rad_s[3];
    float omega_rad_s[3];
    float rate_limit_rad_s[3];
    float rate_p_n_m[3];
    float rate_i_n_m[3];
    float rate_d_n_m[3];
    float rate_ff_n_m[3];
    uint8_t saturation_positive[3];
    uint8_t saturation_negative[3];
    uint8_t thrust_saturated;
    uint8_t tilt_saturated;
    uint8_t yaw_differential_saturated;
    float horizontal_command_scale;
    float moment_utilization;
    float thrust_utilization;
    uint32_t protection_flags;
} DRV_COAX_CTRL_Debug;

typedef struct {
    DRV_POSITION_CONTROL_Params position;
    DRV_AttitudeControl_Params attitude;
    DRV_RateControl_Params rate;
    float vel_loop_enable;
    float mass_kg;
    float gravity_m_s2;
    /*
     * 倾转力臂 [m]，**带符号的几何量**（2026-09-27 起）：
     *     roll_tilt_lever_arm_m  = cg_z_m − servo1_axis_z_m
     *     pitch_tilt_lever_arm_m = cg_z_m − servo2_axis_z_m
     * 即 −r_z，舵机转轴在重心下方时为正；倾转力矩 = 力臂 × T × sin(倾角)，
     * 极性已含在符号里。不是可调参数：每拍由机体模型重算（2026-09-27 当晚
     * 板上几何按部件表重心算出两轴都是 +0.035442，默认 N·m 增益按它换算；
     * 作者随后实测重心 −0.01 m，力臂约 0.12 m——机体模型一改，横滚/俯仰
     * N·m 增益须重新辨识推导）。机体模型无效或 |力臂| 小于
     * DRV_AIRFRAME_TILT_AXIS_MIN_LEVER_M 时倾转反解回中、正向力矩记 0。
     * 2026-09-27 之前这里存的是 0.145 m 的输入力臂，
     * 再乘 0.569/0.581 的经验系数——解读旧飞行日志（v6~v9 参数快照）时别混用。
     */
    float pitch_tilt_lever_arm_m;
    float roll_tilt_lever_arm_m;
    float tilt_limit_rad;
    float yaw_inertia;
    float motor_single_max_thrust_n;
    float yaw_torque_upper_m_per_n;
    float yaw_torque_lower_m_per_n;
    /*
     * 横滚/俯仰的指令整形与力矩出口陷波（2026-09-28，光杆辨识模型设计），默认全关：
     *   rate_out_notch_hz  速率环反馈力矩出口陷波中心 [Hz]，0 = 关（drv_moment_notch.h）
     *   rate_out_notch_q   陷波 Q
     *   rate_out_notch2_hz 第二级出口陷波中心 [Hz]，0 = 关；串在第一级之后、滤同一信号
     *   rate_out_notch2_q  第二级 Q（范围与深度同第一级）
     *   att_ref_wr_rad_s   姿态参考模型固有频率 [rad/s]，0 = 关（drv_att_reference.h）
     *   att_ref_delay_ms   角度环反馈用参考的延后 Td [ms]
     * 三个开关都为 0 时控制律与加入它们之前逐位相同。参考模型开启时力矩前馈走
     * rate.ff_gain × I × θ̈_ref；力矩已是真实 N·m，台架启用时 ff 倍率应为 1。
     */
    float rate_out_notch_hz;
    float rate_out_notch_q;
    float rate_out_notch2_hz;
    float rate_out_notch2_q;
    float att_ref_wr_rad_s;
    float att_ref_delay_ms;
} DRV_COAX_CTRL_Params;

/*
 * Runtime mechanical adapter.  pulse_sign is +1 when a positive mechanism
 * tilt increases pulse width and -1 when installation reverses that motion.
 * This polarity lives after controller/body-frame allocation; it must never
 * be compensated with negative control gains.
 */
typedef struct {
    uint16_t center_us[DRV_COAX_CTRL_SERVO_COUNT];
    uint16_t min_us[DRV_COAX_CTRL_SERVO_COUNT];
    uint16_t max_us[DRV_COAX_CTRL_SERVO_COUNT];
    int8_t pulse_sign[DRV_COAX_CTRL_SERVO_COUNT];
} DRV_COAX_CTRL_ServoCalibration;

void DRV_COAX_CTRL_Init(void);
void DRV_COAX_CTRL_ResetState(void);

void DRV_COAX_CTRL_Run(const DRV_COAX_CTRL_AttitudeInput *attitude,
                       const DRV_COAX_CTRL_Reference *reference,
                       DRV_COAX_CTRL_Output *output);
void DRV_COAX_CTRL_RunScheduled(const DRV_COAX_CTRL_AttitudeInput *attitude,
                                const DRV_COAX_CTRL_Reference *reference,
                                const DRV_COAX_CTRL_Schedule *schedule,
                                DRV_COAX_CTRL_Output *output);
void DRV_COAX_CTRL_GetLastDebug(DRV_COAX_CTRL_Debug *debug);

void DRV_COAX_CTRL_GetDefaultParams(DRV_COAX_CTRL_Params *params);
void DRV_COAX_CTRL_ResetParams(void);
void DRV_COAX_CTRL_GetParams(DRV_COAX_CTRL_Params *params);
void DRV_COAX_CTRL_SetParams(const DRV_COAX_CTRL_Params *params);
uint32_t DRV_COAX_CTRL_ParamCount(void);
const char *DRV_COAX_CTRL_ParamName(uint32_t index);
uint8_t DRV_COAX_CTRL_GetParam(const char *name, float *value);
uint8_t DRV_COAX_CTRL_SetParam(const char *name, float value);

void DRV_COAX_CTRL_GetDefaultServoCalibration(
    DRV_COAX_CTRL_ServoCalibration *calibration);
uint8_t DRV_COAX_CTRL_ValidateServoCalibration(
    const DRV_COAX_CTRL_ServoCalibration *calibration);
void DRV_COAX_CTRL_ResetServoCalibration(void);
void DRV_COAX_CTRL_GetServoCalibration(
    DRV_COAX_CTRL_ServoCalibration *calibration);
uint8_t DRV_COAX_CTRL_SetServoCalibration(
    const DRV_COAX_CTRL_ServoCalibration *calibration);

uint16_t DRV_COAX_CTRL_AlphaTiltRadToServoPulse(float tilt_rad);
uint16_t DRV_COAX_CTRL_BetaTiltRadToServoPulse(float tilt_rad);
void DRV_COAX_CTRL_BodyTiltRadToServoPulses(float body_x_tilt_rad,
                                            float body_y_tilt_rad,
                                            uint16_t *servo_alpha_us,
                                            uint16_t *servo_beta_us);
uint16_t DRV_COAX_CTRL_ThrustToMotorPulse(float thrust_n);
float DRV_COAX_CTRL_MotorPulseToTotalThrust(uint16_t pulse_us);

/*
 * 推力 <-> 电机脉宽的换算可由上层注入：App 启动时装推力台查补表（带电池电压补偿，
 * 见 app_thrust_lut.c）。未注入或传 NULL 时用本文件内置的 21 点旧曲线，供 A/B 对照。
 * 两个回调都在控制节拍里调用，不得阻塞；只在未解锁时切换。
 */
typedef struct {
    uint16_t (*pulse_for_motor_thrust)(float thrust_n); /* 单桨推力（N），已夹到 [0, 单桨上限] */
    float (*total_thrust_for_pulse)(uint16_t pulse_us); /* 两桨同脉宽时的合推力（N） */
    /*
     * 可选：上下桨一起换算。差速时两桨合推力不等于各自按同油门曲线之和（共轴互扰），
     * 提供它的实现可按二维表让合推力正好等于 upper_n + lower_n。NULL 时逐桨换算。
     */
    void (*pulses_for_pair)(float upper_n, float lower_n, uint16_t *upper_us, uint16_t *lower_us);
} DRV_COAX_CTRL_ThrustMap;

void DRV_COAX_CTRL_SetThrustMap(const DRV_COAX_CTRL_ThrustMap *map);
const DRV_COAX_CTRL_ThrustMap *DRV_COAX_CTRL_GetThrustMap(void);

/*
 * 期望机体力矩 -> 机体倾转角。**这是分配器内部那两个反解器的公开出口**，
 * 不是另写一份：内部直接调 coax_ctrl_solve_roll/pitch_tilt_from_moment，
 * 因此系统辨识算出来的倾角与在飞的控制律逐位一致。
 *
 * 输出可以直接喂给 DRV_COAX_CTRL_BodyTiltRadToServoPulses。
 * moment_n_m 是规范 FLU 机体系力矩 [N·m]；只用 X（roll）与 Y（pitch）两个分量，
 * 偏航靠上下桨差速、不由倾转产生，Z 分量被忽略。
 * 倾角上限取自当前生效的 `coax.tilt_limit_rad`，调用方不能绕过它。
 *
 * total_force_n 必须为正：力矩来自"推力 × 力臂 × sin(倾角)"，推力为零时
 * 任何倾角都产生不了力矩。此时返回零倾角并报 0——**不要**把它当成"可以随便倾"，
 * 反解器在零推力下会收敛到限位，那是最危险的一种输出。
 * 机体模型无效（DRV_Airframe_IsValid() == 0）或 |力臂| 小于
 * DRV_AIRFRAME_TILT_AXIS_MIN_LEVER_M 时同样返回零倾角并报 0：那时力臂的方向
 * 都不可信，给出的倾角可能正好是反的。
 */
uint8_t DRV_COAX_CTRL_SolveBodyTiltFromMoment(const float moment_n_m[3],
                                              float total_force_n,
                                              float *body_x_tilt_rad,
                                              float *body_y_tilt_rad);

/* Command-based moment estimate after pulse quantisation and mechanical limits.
 * FLU N*m; excludes unmeasured actuator lag. Same forward map as RunScheduled.
 * 机体模型无效或 |力臂| 过小时写 0 力矩并返回 0（与上面的反解同一判据）。 */
uint8_t DRV_COAX_CTRL_MomentFromServoPulses(float total_force_n,
    uint16_t alpha_us, uint16_t beta_us, float moment[3]);

#ifdef __cplusplus
}
#endif

#endif

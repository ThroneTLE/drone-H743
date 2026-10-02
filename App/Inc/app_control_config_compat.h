#ifndef APP_CONTROL_CONFIG_COMPAT_H
#define APP_CONTROL_CONFIG_COMPAT_H

#include <stdint.h>

/* CFG V19: every stored name matches its physical role and SI unit. */
typedef struct {
    float pos_x_kp;
    float pos_y_kp;
    float pos_z_kp;
    float pos_xy_vel_max_m_s;
    float pos_z_vel_up_max_m_s;
    float pos_z_vel_down_max_m_s;
    float vel_x_kp;
    float vel_y_kp;
    float vel_z_kp;
    float vel_x_ki;
    float vel_y_ki;
    float vel_z_ki;
    float vel_x_kd;
    float vel_y_kd;
    float vel_z_kd;
    float vel_x_i_limit_m_s2;
    float vel_y_i_limit_m_s2;
    float vel_z_i_limit_m_s2;
    float accel_lpf_cutoff_hz;
    float accel_xy_max_m_s2;
    float accel_z_up_max_m_s2;
    float accel_z_down_max_m_s2;
    float att_roll_kp;
    float att_pitch_kp;
    float att_yaw_kp;
    float roll_rate_limit_rad_s;
    float pitch_rate_limit_rad_s;
    float yaw_rate_limit_rad_s;
    float rate_roll_kp;
    float rate_pitch_kp;
    float rate_yaw_kp;
    float rate_roll_ki;
    float rate_pitch_ki;
    float rate_yaw_ki;
    float rate_roll_kd;
    float rate_pitch_kd;
    float rate_yaw_kd;
    float rate_roll_i_limit_n_m;
    float rate_pitch_i_limit_n_m;
    float rate_yaw_i_limit_n_m;
    float angular_accel_lpf_cutoff_hz;
    float rate_roll_ff;
    float rate_pitch_ff;
    float rate_yaw_ff;
    float tilt_limit_rad;
    float vel_loop_enable;
} APP_ControlCoaxTunableParams;

/*
 * CFG V24：横滚/俯仰的指令整形与力矩出口陷波（coax.rate_out_notch_* / coax.att_ref_*），
 * 在记录里单独成块、排在磁力计块之后。**不并进上面的增益块**：增益块同时是飞行日志
 * 扇区头的参数快照（app_flight_log.c），改它的布局等于改日志格式。
 * 布局冻结：以后加字段另起一块或升版本，不改这四个的顺序。
 * v23 及更早的记录里没有这一块，读取器显式落回默认（两个开关为 0 = 关）。
 *
 * CFG V25：块尾追加第二级出口陷波（coax.rate_out_notch2_*），前四个字段原位不动。
 * v24 的块按下面冻结的 V24 类型读，第二级落回默认（关、Q 1.0），见
 * APP_ControlConfigCompat_ShapingV24ToCurrent。
 */
typedef struct {
    float rate_out_notch_hz;
    float rate_out_notch_q;
    float att_ref_wr_rad_s;
    float att_ref_delay_ms;
    float rate_out_notch2_hz;
    float rate_out_notch2_q;
} APP_ControlCoaxShapingParams;

/*
 * 竖直通道块（CFG V28，R-ALTID-1）：coax.hover_thrust_n（悬停推力，推力查补表口径 N，0 = 关）
 * 与 coax.z_vel_fusion（竖直速度融合 IMU 加速度，0/1）。v27 及更早没有这一块，读取器落回驱动
 * 默认（悬停推力 0、融合开）。布局冻结，以后加字段另起一块。
 */
typedef struct {
    float hover_thrust_n;
    float z_vel_fusion;
} APP_ControlZChannelParams;

/*
 * 飞行限幅块（CFG V29）：coax.alt_max_m、coax.manual_tilt_max_rad、coax.yaw_stick_rate_rad_s。
 * 作者 2026-10-01："飞机限速/加速度等参数让我可以在上位机可以随意配置"。v28 及更早没有这一块，
 * 读取器落回驱动默认（0.40 m、20°、1.0472 rad/s，即原写死值）。布局冻结，以后加字段另起一块。
 */
typedef struct {
    float alt_max_m;
    float manual_tilt_max_rad;
    float yaw_stick_rate_rad_s;
} APP_ControlFlightLimitParams;

/* V24 = 当前整形块减掉第二级陷波。冻结它是为了让 v24 记录按原字节布局校验。 */
typedef struct {
    float rate_out_notch_hz;
    float rate_out_notch_q;
    float att_ref_wr_rad_s;
    float att_ref_delay_ms;
} APP_ControlCoaxShapingParamsV24;

/* V18 = pre-cascade layout. Frozen for backward-compatible reads only. */
typedef struct {
    float pos_x_kp;
    float pos_y_kp;
    float pos_z_kp;
    float pos_z_ki;
    float vel_x_kd;
    float vel_y_kd;
    float vel_z_kd;
    float vel_loop_enable;
    float roll_angle_kp;
    float pitch_angle_kp;
    float roll_rate_kd;
    float pitch_rate_kd;
    float tilt_limit_rad;
    float yaw_angle_kp;
    float yaw_rate_kd;
} APP_ControlCoaxTunableParamsV18;

/* V17 additionally carried six never-connected horizontal loop gains. */
typedef struct {
    float pos_x_kp;
    float pos_y_kp;
    float pos_z_kp;
    float pos_z_ki;
    float vel_x_kd;
    float vel_y_kd;
    float vel_z_kd;
    float vel_loop_enable;
    float vel_loop_x_kp;
    float vel_loop_x_ki;
    float vel_loop_x_kd;
    float vel_loop_y_kp;
    float vel_loop_y_ki;
    float vel_loop_y_kd;
    float roll_angle_kp;
    float pitch_angle_kp;
    float roll_rate_kd;
    float pitch_rate_kd;
    float tilt_limit_rad;
    float yaw_angle_kp;
    float yaw_rate_kd;
} APP_ControlCoaxTunableParamsV17;

/* V15 predates pos_z_ki and rc_config. */
typedef struct {
    float pos_x_kp;
    float pos_y_kp;
    float pos_z_kp;
    float vel_x_kd;
    float vel_y_kd;
    float vel_z_kd;
    float vel_loop_enable;
    float vel_loop_x_kp;
    float vel_loop_x_ki;
    float vel_loop_x_kd;
    float vel_loop_y_kp;
    float vel_loop_y_ki;
    float vel_loop_y_kd;
    float roll_angle_kp;
    float pitch_angle_kp;
    float roll_rate_kd;
    float pitch_rate_kd;
    float tilt_limit_rad;
    float yaw_angle_kp;
    float yaw_rate_kd;
} APP_ControlCoaxTunableParamsV15;

/*
 * v19 的调参块类型与当前完全相同（v20 只是在记录**后面**追加了机体模型块），
 * 所以这一步是纯拷贝。仍然走 converter 这个接口，是为了让迁移链保持一条直线——
 * 给 v19 开特例分支的话，下一个加版本的人就得先读懂两套写法。
 */
uint8_t APP_ControlConfigCompat_CurrentPassthrough(
    const APP_ControlCoaxTunableParams *legacy,
    APP_ControlCoaxTunableParams *current);

uint8_t APP_ControlConfigCompat_V18ToCurrent(
    const APP_ControlCoaxTunableParamsV18 *legacy,
    APP_ControlCoaxTunableParams *current);
uint8_t APP_ControlConfigCompat_V17ToCurrent(
    const APP_ControlCoaxTunableParamsV17 *legacy,
    APP_ControlCoaxTunableParams *current);
uint8_t APP_ControlConfigCompat_V15ToCurrent(
    const APP_ControlCoaxTunableParamsV15 *legacy,
    APP_ControlCoaxTunableParams *current);

/*
 * v24 整形块 → 当前：前四个字段原样拷贝，第二级陷波落回代码默认（notch2_hz = 0 关、
 * notch2_q = 1.0）。与增益块迁移一样是确定的函数，不读运行时的值。
 */
uint8_t APP_ControlConfigCompat_ShapingV24ToCurrent(
    const APP_ControlCoaxShapingParamsV24 *legacy,
    APP_ControlCoaxShapingParams *current);

/*
 * CFG V20～V25 的机体块 = DRV_Airframe_Params 追加光流安装两项（V26，R-FLOWMOUNT-1）
 * 之前的布局：36 个 float，顺序与当前结构体的前 36 项逐一相同（从 board_mass_g 到
 * derived_auto，含两个 retired_ 字段）。当前结构体就是它 + 尾部两项，所以这里只冻结
 * 大小、不重抄字段名——字段名只有一份，app_control_config_store.c 用静态断言钉住
 * "它恰好是当前结构体的前缀"，并在那里做 v25 → 当前的转换（安装两项落回 0/0）。
 * 转换不放在本模块：本模块不碰机体模型的任何东西（见 tests/test_control_config_v19.py）。
 */
#define APP_CONTROL_AIRFRAME_V25_FLOAT_COUNT 36U

typedef struct {
    float field[APP_CONTROL_AIRFRAME_V25_FLOAT_COUNT];
} APP_ControlAirframeParamsV25;

#endif /* APP_CONTROL_CONFIG_COMPAT_H */

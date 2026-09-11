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

#endif /* APP_CONTROL_CONFIG_COMPAT_H */

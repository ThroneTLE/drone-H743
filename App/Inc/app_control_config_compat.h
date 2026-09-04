#ifndef APP_CONTROL_CONFIG_COMPAT_H
#define APP_CONTROL_CONFIG_COMPAT_H

#include <stdint.h>

/*
 * 仅描述 CFG 记录里可调同轴参数的稳定布局。硬件模型参数不进入 CFG，
 * 仍由 drv_airframe_model.h 提供。V17 类型被冻结，只用于读取旧记录。
 */
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
} APP_ControlCoaxTunableParams;

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

/* 丢弃 V17 中从未接入控制律的六个 vel_loop_x/y_* 字段。 */
uint8_t APP_ControlConfigCompat_V17ToCurrent(
    const APP_ControlCoaxTunableParamsV17 *legacy,
    APP_ControlCoaxTunableParams *current);

#endif /* APP_CONTROL_CONFIG_COMPAT_H */

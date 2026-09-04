#include "app_control_config_compat.h"

#include <string.h>

uint8_t APP_ControlConfigCompat_V17ToCurrent(
    const APP_ControlCoaxTunableParamsV17 *legacy,
    APP_ControlCoaxTunableParams *current)
{
    if ((legacy == NULL) || (current == NULL)) {
        return 0U;
    }

    memset(current, 0, sizeof(*current));
    current->pos_x_kp = legacy->pos_x_kp;
    current->pos_y_kp = legacy->pos_y_kp;
    current->pos_z_kp = legacy->pos_z_kp;
    current->pos_z_ki = legacy->pos_z_ki;
    current->vel_x_kd = legacy->vel_x_kd;
    current->vel_y_kd = legacy->vel_y_kd;
    current->vel_z_kd = legacy->vel_z_kd;
    current->vel_loop_enable = legacy->vel_loop_enable;
    current->roll_angle_kp = legacy->roll_angle_kp;
    current->pitch_angle_kp = legacy->pitch_angle_kp;
    current->roll_rate_kd = legacy->roll_rate_kd;
    current->pitch_rate_kd = legacy->pitch_rate_kd;
    current->tilt_limit_rad = legacy->tilt_limit_rad;
    current->yaw_angle_kp = legacy->yaw_angle_kp;
    current->yaw_rate_kd = legacy->yaw_rate_kd;
    return 1U;
}

#include "app_control_config_compat.h"

#include "drv_airframe_model.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define APP_CONTROL_COMPAT_EPS                    1.0e-6f
#define APP_CONTROL_COMPAT_XY_VEL_MAX_M_S         0.40f
#define APP_CONTROL_COMPAT_Z_VEL_MAX_M_S          0.30f
#define APP_CONTROL_COMPAT_XY_ACCEL_MAX_M_S2      3.70f
#define APP_CONTROL_COMPAT_Z_ACCEL_MAX_M_S2       3.70f
#define APP_CONTROL_COMPAT_ACCEL_LPF_HZ           20.0f
/*
 * V18 的配置块里没有这个字段，所以这里不是"迁移历史值"，而是给缺失字段挑一个
 * 落点——按惯例应当落在当前默认上。2026-09-07 随 drv_coax_ctrl.c 的默认值一起
 * 由 30 Hz 改为 3 Hz：30 Hz 在 50 Hz 执行器出口下会让任何非零 rate.kd 发散，
 * 留在迁移路径里就是个陷阱（推导见 drv_coax_ctrl.c 中 alpha_lpf_cutoff_rad_s
 * 默认值上方）。
 */
#define APP_CONTROL_COMPAT_ANGULAR_ACCEL_LPF_HZ    3.0f
#define APP_CONTROL_COMPAT_ROLL_PITCH_RATE_RAD_S  3.49065850f
#define APP_CONTROL_COMPAT_YAW_RATE_RAD_S          1.04719758f

static float compat_positive(float value)
{
    return fabsf(value);
}

static float compat_ratio(float numerator, float denominator, float fallback)
{
    denominator = compat_positive(denominator);
    return (denominator > APP_CONTROL_COMPAT_EPS) ?
        (compat_positive(numerator) / denominator) : fallback;
}

static void compat_common_defaults(APP_ControlCoaxTunableParams *current)
{
    memset(current, 0, sizeof(*current));
    current->pos_xy_vel_max_m_s = APP_CONTROL_COMPAT_XY_VEL_MAX_M_S;
    current->pos_z_vel_up_max_m_s = APP_CONTROL_COMPAT_Z_VEL_MAX_M_S;
    current->pos_z_vel_down_max_m_s = APP_CONTROL_COMPAT_Z_VEL_MAX_M_S;
    current->vel_x_i_limit_m_s2 = 1.50f;
    current->vel_y_i_limit_m_s2 = 1.50f;
    current->vel_z_i_limit_m_s2 = 1.50f;
    current->accel_lpf_cutoff_hz = APP_CONTROL_COMPAT_ACCEL_LPF_HZ;
    current->accel_xy_max_m_s2 = APP_CONTROL_COMPAT_XY_ACCEL_MAX_M_S2;
    current->accel_z_up_max_m_s2 = APP_CONTROL_COMPAT_Z_ACCEL_MAX_M_S2;
    current->accel_z_down_max_m_s2 = APP_CONTROL_COMPAT_Z_ACCEL_MAX_M_S2;
    current->roll_rate_limit_rad_s = APP_CONTROL_COMPAT_ROLL_PITCH_RATE_RAD_S;
    current->pitch_rate_limit_rad_s = APP_CONTROL_COMPAT_ROLL_PITCH_RATE_RAD_S;
    current->yaw_rate_limit_rad_s = APP_CONTROL_COMPAT_YAW_RATE_RAD_S;
    current->rate_roll_i_limit_n_m = 0.010f;
    current->rate_pitch_i_limit_n_m = 0.010f;
    current->rate_yaw_i_limit_n_m = 0.00020f;
    current->angular_accel_lpf_cutoff_hz =
        APP_CONTROL_COMPAT_ANGULAR_ACCEL_LPF_HZ;
    current->rate_roll_ff = 1.0f;
    current->rate_pitch_ff = 1.0f;
    current->rate_yaw_ff = 1.0f;
}

uint8_t APP_ControlConfigCompat_V18ToCurrent(
    const APP_ControlCoaxTunableParamsV18 *legacy,
    APP_ControlCoaxTunableParams *current)
{
    if ((legacy == NULL) || (current == NULL)) {
        return 0U;
    }

    compat_common_defaults(current);
    current->vel_x_kp = compat_positive(legacy->vel_x_kd);
    current->vel_y_kp = compat_positive(legacy->vel_y_kd);
    current->pos_x_kp = compat_ratio(legacy->pos_x_kp,
                                     current->vel_x_kp, 0.0f);
    current->pos_y_kp = compat_ratio(legacy->pos_y_kp,
                                     current->vel_y_kp, 0.0f);

    /* V18 vel_z_kd is normally zero: this factorization is not exact. */
    current->vel_z_kp = (compat_positive(legacy->vel_z_kd) >
                          APP_CONTROL_COMPAT_EPS) ?
        compat_positive(legacy->vel_z_kd) : 1.0f;
    current->pos_z_kp = compat_ratio(legacy->pos_z_kp,
                                     current->vel_z_kp,
                                     compat_positive(legacy->pos_z_kp));

    /* New I/D defaults are zero; legacy pos_z_ki has different units. */
    current->rate_roll_kp = compat_positive(legacy->roll_rate_kd);
    current->rate_pitch_kp = compat_positive(legacy->pitch_rate_kd);
    current->rate_yaw_kp = DRV_AIRFRAME_IZZ_KGM2 *
                           compat_positive(legacy->yaw_rate_kd);
    current->att_roll_kp = compat_ratio(legacy->roll_angle_kp,
                                        current->rate_roll_kp, 0.0f);
    current->att_pitch_kp = compat_ratio(legacy->pitch_angle_kp,
                                         current->rate_pitch_kp, 0.0f);
    current->att_yaw_kp = compat_ratio(legacy->yaw_angle_kp,
                                       legacy->yaw_rate_kd, 0.0f);
    current->tilt_limit_rad = compat_positive(legacy->tilt_limit_rad);
    current->vel_loop_enable = legacy->vel_loop_enable;
    return 1U;
}

uint8_t APP_ControlConfigCompat_V17ToCurrent(
    const APP_ControlCoaxTunableParamsV17 *legacy,
    APP_ControlCoaxTunableParams *current)
{
    APP_ControlCoaxTunableParamsV18 v18;

    if ((legacy == NULL) || (current == NULL)) {
        return 0U;
    }
    memset(&v18, 0, sizeof(v18));
    memcpy(&v18, legacy, offsetof(APP_ControlCoaxTunableParamsV17,
                                  vel_loop_x_kp));
    v18.roll_angle_kp = legacy->roll_angle_kp;
    v18.pitch_angle_kp = legacy->pitch_angle_kp;
    v18.roll_rate_kd = legacy->roll_rate_kd;
    v18.pitch_rate_kd = legacy->pitch_rate_kd;
    v18.tilt_limit_rad = legacy->tilt_limit_rad;
    v18.yaw_angle_kp = legacy->yaw_angle_kp;
    v18.yaw_rate_kd = legacy->yaw_rate_kd;
    return APP_ControlConfigCompat_V18ToCurrent(&v18, current);
}

uint8_t APP_ControlConfigCompat_V15ToCurrent(
    const APP_ControlCoaxTunableParamsV15 *legacy,
    APP_ControlCoaxTunableParams *current)
{
    APP_ControlCoaxTunableParamsV17 v17;

    if ((legacy == NULL) || (current == NULL)) {
        return 0U;
    }
    memset(&v17, 0, sizeof(v17));
    v17.pos_x_kp = legacy->pos_x_kp;
    v17.pos_y_kp = legacy->pos_y_kp;
    v17.pos_z_kp = legacy->pos_z_kp;
    v17.vel_x_kd = legacy->vel_x_kd;
    v17.vel_y_kd = legacy->vel_y_kd;
    v17.vel_z_kd = legacy->vel_z_kd;
    v17.vel_loop_enable = legacy->vel_loop_enable;
    v17.roll_angle_kp = legacy->roll_angle_kp;
    v17.pitch_angle_kp = legacy->pitch_angle_kp;
    v17.roll_rate_kd = legacy->roll_rate_kd;
    v17.pitch_rate_kd = legacy->pitch_rate_kd;
    v17.tilt_limit_rad = legacy->tilt_limit_rad;
    v17.yaw_angle_kp = legacy->yaw_angle_kp;
    v17.yaw_rate_kd = legacy->yaw_rate_kd;
    return APP_ControlConfigCompat_V17ToCurrent(&v17, current);
}

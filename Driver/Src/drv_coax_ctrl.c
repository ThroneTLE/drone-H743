#include "drv_coax_ctrl.h"

#include "bsp_pwm.h"
#include "drv_airframe_model.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define DRV_COAX_CTRL_TILT_LIMIT_RAD 0.4886922f
#define DRV_COAX_CTRL_PI 3.141592654f
#define DRV_COAX_CTRL_SERVO_TRAVEL_RAD \
    (DRV_COAX_CTRL_SERVO_TRAVEL_DEG * DRV_COAX_CTRL_PI / 180.0f)
#define DRV_COAX_CTRL_SERVO_LIMIT_RAD \
    (DRV_COAX_CTRL_SERVO_LIMIT_DEG * DRV_COAX_CTRL_PI / 180.0f)
/* ════════════════════════════════════════════════════════════════════════ */
/*  极性约定（唯一声明处）                                                   */
/*                                                                        */
/*  从传感器到舵机这条链路上曾经散落着 8 个互相独立的符号开关，2^8 = 256    */
/*  种组合里只有少数是自洽的，而且它们的效果会互相掩盖：负增益在数学上等价  */
/*  于翻转符号，所以极性错误可以被"把增益调成负的"吸收掉——飞机看起来能自稳， */
/*  但摇杆方向是反的。这正是极性问题反复出现的原因：自稳只验证了"负反馈"    */
/*  一个条件，不验证绝对方向。                                              */
/*                                                                        */
/*  因此本文件把所有符号集中在这里，每一项都写明物理含义，并由              */
/*  tests/test_coax_sign_convention.py 逐条锁定。增益一律为正值，负反馈由   */
/*  控制律的 -K_R*e_R - K_w*e_w 结构保证——极性错了飞机会立刻发散，而不是    */
/*  悄悄反向工作。                                                          */
/*                                                                        */
/*  姿态约定：本层拿到的角度**口径由姿态融合的 convention 决定**，而它按    */
/*  APP_Sensor_IsFluOrientationActive() 在 NED / NWU 之间切换：              */
/*    roll_rad  > 0  →  机身右侧下沉   （两种口径相同，迁移不改它）         */
/*    pitch_rad > 0  →  legacy(NED)=机头上仰；FLU(NWU)=机头**下俯**         */
/*    gyro_x    > 0  →  正 roll 方向的角速率                               */
/*    gyro_y    > 0  →  正 pitch 方向的角速率                              */
/*  可执行证据见 tests/test_flu_seam1_estimator_frame.py。                  */
/*                                                                        */
/*  R-F6-2（2026-09-06，工单见                                              */
/*  doc/req-rf6-2-controller-flu-migration.md）：位置/速度侧（Reference /   */
/*  AttitudeInput 的 x_m/y_m/z_m、vx/vy/vz_m_s）已migrate到规范 FLU。       */
/*  第 3 节定性结论：**local Y 从原始光流读数起就一直是左正**（拆桨向右平移 */
/*  实测原始 flow_vy 为负），四处"local Y 是机体右"的注释全部是错的，Y 不   */
/*  需要任何数值改动；只有 Z 真的从下正翻成了上正。姿态/角速率本来就已是    */
/*  规范 FLU，因此力坐标系不再需要任何符号补偿——下面这条曾经的                */
/*  DRV_COAX_CTRL_FORCE_FRAME_ROLL_SIGN / _PITCH_SIGN /                     */
/*  DRV_COAX_CTRL_RATE_FRAME_ROLL_SIGN / _PITCH_SIGN 四个常量已删除，        */
/*  coax_ctrl_rpy_matrix 直接吃 FLU 角度，不加任何符号：标准 ZYX 欧拉矩阵    */
/*  公式对任何右手系都成立，FLU 的 roll/pitch/yaw 定义本来就是绕 FLU 自己    */
/*  的 X/Y/Z 的右手旋转（见 drv_frame_contract.h），无需额外补偿。          */
/* ════════════════════════════════════════════════════════════════════════ */

#define DRV_COAX_CTRL_FORCE_EPS_N          1.0e-4f
#define DRV_COAX_CTRL_RATE_SCALE_EPS       1.0e-6f
#define DRV_COAX_CTRL_SERVO_ANGLE_TOL_RAD  8.0e-4f
#define DRV_COAX_CTRL_PROP9047_YAW_M_PER_N 0.0001f
#define DRV_COAX_CTRL_SINGLE_MAX_THRUST_N 10.2f
#define DRV_COAX_CTRL_THRUST_TABLE_POINTS  21U
#define DRV_COAX_CTRL_GRAMS_PER_NEWTON     101.971621f
#define DRV_COAX_CTRL_ROLL_EFFECTIVENESS   0.581f
#define DRV_COAX_CTRL_PITCH_EFFECTIVENESS  0.569f
#define DRV_COAX_CTRL_ROLL_MOMENT_SIGN     (-1.0f)
#define DRV_COAX_CTRL_PITCH_MOMENT_SIGN    (-1.0f)
#define DRV_COAX_CTRL_HORIZONTAL_ACCEL_LIMIT_M_S2 3.70f
#define DRV_COAX_CTRL_VEL_D_ACCEL_LIMIT_M_S2 3.70f
#define DRV_COAX_CTRL_POS_Z_I_ACCEL_LIMIT_M_S2 1.50f
#define DRV_COAX_CTRL_DT_MAX_S 0.05f
#define DRV_COAX_CTRL_ATTITUDE_PROTECT_START_RAD 0.436332f
#define DRV_COAX_CTRL_ATTITUDE_PROTECT_END_RAD   0.785398f
#define DRV_COAX_CTRL_MOMENT_PROTECT_START       0.95f
#define DRV_COAX_CTRL_MOMENT_PROTECT_END         1.15f
#define DRV_COAX_CTRL_THRUST_PROTECT_START       1.05f
#define DRV_COAX_CTRL_THRUST_PROTECT_END         1.25f

typedef struct {
    const char *name;
    uint16_t offset;
} DRV_COAX_CTRL_ParamEntry;

typedef struct {
    DRV_POSITION_CONTROL_State position;
    DRV_POSITION_CONTROL_PositionOutput position_output;
    DRV_POSITION_CONTROL_VelocityOutput velocity_output;
    DRV_AttitudeControl_Output attitude_output;
    DRV_RateControl_State rate;
    DRV_RateControl_Output rate_output;
    DRV_POSITION_CONTROL_SaturationFeedback translation_saturation;
    uint8_t moment_saturation_positive[3];
    uint8_t moment_saturation_negative[3];
} DRV_COAX_CTRL_State;

typedef struct {
    float desired_force_local_n[3];
    float desired_force_body_n[3];
    float desired_body_r[3][3];
    float attitude_error[3];
    float attitude_error_angle_rad;
    float attitude_tilt_error_rad;
    float rate_error_rad_s[3];
    float moment_cmd_n_m[3];
    float alpha_rad;
    float beta_rad;
    float total_force_n;
    float raw_total_force_n;
    float moment_utilization;
    float thrust_utilization;
} DRV_COAX_CTRL_BalanceSolution;

static uint8_t coax_ctrl_initialized;
static DRV_COAX_CTRL_Params coax_ctrl_params;
static DRV_COAX_CTRL_ServoCalibration coax_ctrl_servo_calibration;
static DRV_COAX_CTRL_Debug coax_ctrl_last_debug;
static DRV_COAX_CTRL_State coax_ctrl_state;

#define DRV_COAX_CTRL_PARAM_ENTRY(field) \
    { "coax." #field, (uint16_t)offsetof(DRV_COAX_CTRL_Params, field) }

#define DRV_COAX_CTRL_NAMED_PARAM_ENTRY(name, field) \
    { "coax." name, (uint16_t)offsetof(DRV_COAX_CTRL_Params, field) }

static const DRV_COAX_CTRL_ParamEntry coax_ctrl_param_table[] = {
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("pos_x_kp", position.pos_kp[0]),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("pos_y_kp", position.pos_kp[1]),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("pos_z_kp", position.pos_kp[2]),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("pos_xy_vel_max_m_s", position.xy_speed_limit_m_s),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("pos_z_vel_up_max_m_s", position.z_speed_limit_up_m_s),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("pos_z_vel_down_max_m_s", position.z_speed_limit_down_m_s),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("vel_x_kp", position.vel_kp[0]),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("vel_y_kp", position.vel_kp[1]),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("vel_z_kp", position.vel_kp[2]),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("vel_x_ki", position.vel_ki[0]),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("vel_y_ki", position.vel_ki[1]),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("vel_z_ki", position.vel_ki[2]),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("vel_x_kd", position.vel_kd[0]),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("vel_y_kd", position.vel_kd[1]),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("vel_z_kd", position.vel_kd[2]),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("vel_x_i_limit_m_s2", position.vel_integrator_limit[0]),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("vel_y_i_limit_m_s2", position.vel_integrator_limit[1]),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("vel_z_i_limit_m_s2", position.vel_integrator_limit[2]),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("accel_lpf_cutoff_hz", position.accel_lpf_cutoff_hz),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("accel_xy_max_m_s2", position.xy_accel_limit_m_s2),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("accel_z_up_max_m_s2", position.z_accel_limit_up_m_s2),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("accel_z_down_max_m_s2", position.z_accel_limit_down_m_s2),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("att_roll_kp", attitude.att_kp[0]),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("att_pitch_kp", attitude.att_kp[1]),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("att_yaw_kp", attitude.att_kp[2]),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("roll_rate_limit_rad_s", attitude.rate_limit_rad_s[0]),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("pitch_rate_limit_rad_s", attitude.rate_limit_rad_s[1]),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("yaw_rate_limit_rad_s", attitude.rate_limit_rad_s[2]),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("rate_roll_kp", rate.kp[0]),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("rate_pitch_kp", rate.kp[1]),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("rate_yaw_kp", rate.kp[2]),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("rate_roll_ki", rate.ki[0]),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("rate_pitch_ki", rate.ki[1]),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("rate_yaw_ki", rate.ki[2]),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("rate_roll_kd", rate.kd[0]),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("rate_pitch_kd", rate.kd[1]),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("rate_yaw_kd", rate.kd[2]),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("rate_roll_i_limit_n_m", rate.integrator_limit[0]),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("rate_pitch_i_limit_n_m", rate.integrator_limit[1]),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("rate_yaw_i_limit_n_m", rate.integrator_limit[2]),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("angular_accel_lpf_cutoff_rad_s", rate.alpha_lpf_cutoff_rad_s),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("rate_roll_ff", rate.ff_gain[0]),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("rate_pitch_ff", rate.ff_gain[1]),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("rate_yaw_ff", rate.ff_gain[2]),
    DRV_COAX_CTRL_PARAM_ENTRY(vel_loop_enable),
    DRV_COAX_CTRL_PARAM_ENTRY(tilt_limit_rad),
};

static const uint32_t coax_ctrl_param_count =
    sizeof(coax_ctrl_param_table) / sizeof(coax_ctrl_param_table[0]);

static const uint16_t coax_ctrl_dual_pwm_us[DRV_COAX_CTRL_THRUST_TABLE_POINTS] = {
    1100U, 1142U, 1184U, 1226U, 1268U, 1310U, 1352U, 1394U,
    1436U, 1478U, 1520U, 1562U, 1604U, 1646U, 1688U, 1730U,
    1772U, 1814U, 1856U, 1898U, 1940U,
};

static const float coax_ctrl_dual_thrust_g[DRV_COAX_CTRL_THRUST_TABLE_POINTS] = {
    0.000f, 5.069f, 25.589f, 60.655f, 106.361f, 165.084f,
    216.696f, 287.758f, 386.724f, 501.680f, 624.697f, 725.173f,
    828.680f, 923.574f, 981.674f, 1114.845f, 1256.137f, 1366.352f,
    1466.668f, 1541.404f, 1595.342f,
};

static float coax_ctrl_clamp_f32(float value, float lo, float hi)
{
    if (value < lo) { return lo; }
    if (value > hi) { return hi; }
    return value;
}

static uint16_t coax_ctrl_clamp_u16(int32_t value, uint16_t lo, uint16_t hi)
{
    if (value < (int32_t)lo) { return lo; }
    if (value > (int32_t)hi) { return hi; }
    return (uint16_t)value;
}

/*
 * R-F6-2 (2026-09-06): controller inputs use the canonical FLU local frame
 * (X forward, Y left, Z up).  This is R^T(roll, pitch, yaw), the standard
 * ZYX Euler rotation matrix transposed -- it needs no per-axis sign
 * adaptation because FLU's own roll/pitch/yaw are already defined as
 * right-hand rotations about FLU's own X/Y/Z (drv_frame_contract.h).
 */
static void coax_ctrl_local_down_to_body(const DRV_COAX_CTRL_AttitudeInput *attitude,
                                         const float local_down[3],
                                         float body[3])
{
    const float phi = attitude->roll_rad;
    const float theta = attitude->pitch_rad;
    const float psi = attitude->yaw_rad;
    const float cphi = cosf(phi);
    const float sphi = sinf(phi);
    const float ctheta = cosf(theta);
    const float stheta = sinf(theta);
    const float cpsi = cosf(psi);
    const float spsi = sinf(psi);

    body[0] = (ctheta * cpsi * local_down[0]) +
              (ctheta * spsi * local_down[1]) -
              (stheta * local_down[2]);
    body[1] = ((sphi * stheta * cpsi - cphi * spsi) * local_down[0]) +
              ((sphi * stheta * spsi + cphi * cpsi) * local_down[1]) +
              (sphi * ctheta * local_down[2]);
    body[2] = ((cphi * stheta * cpsi + sphi * spsi) * local_down[0]) +
              ((cphi * stheta * spsi - sphi * cpsi) * local_down[1]) +
              (cphi * ctheta * local_down[2]);
}

static float coax_ctrl_norm3(const float value[3])
{
    return sqrtf((value[0] * value[0]) +
                 (value[1] * value[1]) +
                 (value[2] * value[2]));
}

static float coax_ctrl_roll_moment_from_tilt(float total_force_n,
                                             float beta_rad)
{
    return DRV_COAX_CTRL_ROLL_MOMENT_SIGN *
           DRV_COAX_CTRL_ROLL_EFFECTIVENESS *
           coax_ctrl_params.roll_tilt_lever_arm_m *
           total_force_n *
           sinf(beta_rad);
}

static float coax_ctrl_pitch_moment_from_tilt(float total_force_n,
                                              float alpha_rad,
                                              float beta_rad)
{
    return DRV_COAX_CTRL_PITCH_MOMENT_SIGN *
           DRV_COAX_CTRL_PITCH_EFFECTIVENESS *
           coax_ctrl_params.pitch_tilt_lever_arm_m *
           total_force_n *
           sinf(alpha_rad) *
           cosf(beta_rad);
}

static float coax_ctrl_solve_roll_tilt_from_moment(float moment_n_m,
                                                   float total_force_n,
                                                   float tilt_limit_rad)
{
    float lo = -tilt_limit_rad;
    float hi = tilt_limit_rad;
    float moment_lo = coax_ctrl_roll_moment_from_tilt(total_force_n, lo);
    float moment_hi = coax_ctrl_roll_moment_from_tilt(total_force_n, hi);
    const float min_moment = fminf(moment_lo, moment_hi);
    const float max_moment = fmaxf(moment_lo, moment_hi);
    float target = moment_n_m;

    if (target < min_moment) {
        target = min_moment;
    } else if (target > max_moment) {
        target = max_moment;
    }

    for (uint32_t i = 0U; i < 18U; ++i) {
        const float mid = 0.5f * (lo + hi);
        const float moment_mid =
            coax_ctrl_roll_moment_from_tilt(total_force_n, mid);
        const uint8_t increasing = (moment_hi > moment_lo) ? 1U : 0U;

        if (((increasing != 0U) && (moment_mid < target)) ||
            ((increasing == 0U) && (moment_mid > target))) {
            lo = mid;
            moment_lo = moment_mid;
        } else {
            hi = mid;
            moment_hi = moment_mid;
        }
    }

    return 0.5f * (lo + hi);
}

static float coax_ctrl_solve_pitch_tilt_from_moment(float moment_n_m,
                                                    float total_force_n,
                                                    float beta_rad,
                                                    float tilt_limit_rad)
{
    float lo = -tilt_limit_rad;
    float hi = tilt_limit_rad;
    float moment_lo =
        coax_ctrl_pitch_moment_from_tilt(total_force_n, lo, beta_rad);
    float moment_hi =
        coax_ctrl_pitch_moment_from_tilt(total_force_n, hi, beta_rad);
    const float min_moment = fminf(moment_lo, moment_hi);
    const float max_moment = fmaxf(moment_lo, moment_hi);
    float target = moment_n_m;

    if (target < min_moment) {
        target = min_moment;
    } else if (target > max_moment) {
        target = max_moment;
    }

    for (uint32_t i = 0U; i < 18U; ++i) {
        const float mid = 0.5f * (lo + hi);
        const float moment_mid =
            coax_ctrl_pitch_moment_from_tilt(total_force_n, mid, beta_rad);
        const uint8_t increasing = (moment_hi > moment_lo) ? 1U : 0U;

        if (((increasing != 0U) && (moment_mid < target)) ||
            ((increasing == 0U) && (moment_mid > target))) {
            lo = mid;
            moment_lo = moment_mid;
        } else {
            hi = mid;
            moment_hi = moment_mid;
        }
    }

    return 0.5f * (lo + hi);
}

static void coax_ctrl_matrix_transpose(const float input[3][3],
                                       float output[3][3])
{
    for (uint32_t row = 0U; row < 3U; ++row) {
        for (uint32_t col = 0U; col < 3U; ++col) {
            output[row][col] = input[col][row];
        }
    }
}

static void coax_ctrl_matrix_multiply(const float left[3][3],
                                      const float right[3][3],
                                      float output[3][3])
{
    float product[3][3];

    for (uint32_t row = 0U; row < 3U; ++row) {
        for (uint32_t col = 0U; col < 3U; ++col) {
            product[row][col] = 0.0f;
            for (uint32_t index = 0U; index < 3U; ++index) {
                product[row][col] += left[row][index] * right[index][col];
            }
        }
    }
    memcpy(output, product, sizeof(product));
}

static void coax_ctrl_rpy_matrix(float roll_rad,
                                 float pitch_rad,
                                 float yaw_rad,
                                 float rotation[3][3])
{
    const float cr = cosf(roll_rad);
    const float sr = sinf(roll_rad);
    const float cp = cosf(pitch_rad);
    const float sp = sinf(pitch_rad);
    const float cy = cosf(yaw_rad);
    const float sy = sinf(yaw_rad);

    rotation[0][0] = cp * cy;
    rotation[0][1] = (sr * sp * cy) - (cr * sy);
    rotation[0][2] = (cr * sp * cy) + (sr * sy);
    rotation[1][0] = cp * sy;
    rotation[1][1] = (sr * sp * sy) + (cr * cy);
    rotation[1][2] = (cr * sp * sy) - (sr * cy);
    rotation[2][0] = -sp;
    rotation[2][1] = sr * cp;
    rotation[2][2] = cr * cp;
}

static void coax_ctrl_attitude_matrix(
    const DRV_COAX_CTRL_AttitudeInput *attitude,
    float rotation[3][3])
{
    coax_ctrl_rpy_matrix(attitude->roll_rad, attitude->pitch_rad,
                        attitude->yaw_rad, rotation);
}

static void coax_ctrl_attitude_error(const float desired[3][3],
                                     const float actual[3][3],
                                     float error[3],
                                     float *error_angle_rad,
                                     float *tilt_error_rad)
{
    float desired_t[3][3];
    float actual_t[3][3];
    float desired_t_actual[3][3];
    float actual_t_desired[3][3];

    coax_ctrl_matrix_transpose(desired, desired_t);
    coax_ctrl_matrix_transpose(actual, actual_t);
    coax_ctrl_matrix_multiply(desired_t, actual, desired_t_actual);
    coax_ctrl_matrix_multiply(actual_t, desired, actual_t_desired);

    error[0] = 0.5f * (desired_t_actual[2][1] - actual_t_desired[2][1]);
    error[1] = 0.5f * (desired_t_actual[0][2] - actual_t_desired[0][2]);
    error[2] = 0.5f * (desired_t_actual[1][0] - actual_t_desired[1][0]);
    if (error_angle_rad != NULL) {
        const float cos_angle = 0.5f *
            (desired_t_actual[0][0] + desired_t_actual[1][1] +
             desired_t_actual[2][2] - 1.0f);
        *error_angle_rad = acosf(coax_ctrl_clamp_f32(cos_angle, -1.0f, 1.0f));
    }
    if (tilt_error_rad != NULL) {
        const float z_axis_dot =
            (desired[0][2] * actual[0][2]) +
            (desired[1][2] * actual[1][2]) +
            (desired[2][2] * actual[2][2]);
        *tilt_error_rad = acosf(coax_ctrl_clamp_f32(z_axis_dot, -1.0f, 1.0f));
    }
}

static void coax_ctrl_rotation_to_rpy(const float rotation[3][3],
                                      float rpy_rad[3])
{
    const float sin_pitch = coax_ctrl_clamp_f32(-rotation[2][0], -1.0f, 1.0f);

    rpy_rad[0] = atan2f(rotation[2][1], rotation[2][2]);
    rpy_rad[1] = asinf(sin_pitch);
    rpy_rad[2] = atan2f(rotation[1][0], rotation[0][0]);
}

/*
 * 差动推力能提供的偏航力矩上限，与 coax_ctrl_allocate_motor_thrust 的
 * 钳位边界严格对偶：任一路推力越界都会让指令被静默削掉。
 */
static float coax_ctrl_yaw_limit_moment(float total_force_n)
{
    const float ku = coax_ctrl_params.yaw_torque_upper_m_per_n;
    const float kl = coax_ctrl_params.yaw_torque_lower_m_per_n;
    const float span = (ku + kl) * coax_ctrl_params.motor_single_max_thrust_n;
    float limit = fminf(kl * total_force_n, ku * total_force_n);

    limit = fminf(limit, span - (ku * total_force_n));
    limit = fminf(limit, span - (kl * total_force_n));
    return (limit > 0.0f) ? limit : 0.0f;
}

static float coax_ctrl_protection_scale(float value, float start, float end)
{
    if (value <= start) {
        return 1.0f;
    }
    if (value >= end) {
        return 0.0f;
    }
    return (end - value) / (end - start);
}

static float *coax_ctrl_param_ptr(DRV_COAX_CTRL_Params *params,
                                  const DRV_COAX_CTRL_ParamEntry *entry)
{
    return (float *)((uint8_t *)params + entry->offset);
}

static const DRV_COAX_CTRL_ParamEntry *coax_ctrl_find_param(const char *name)
{
    if (name == NULL) {
        return NULL;
    }

    for (uint32_t i = 0U; i < coax_ctrl_param_count; ++i) {
        if (strcmp(name, coax_ctrl_param_table[i].name) == 0) {
            return &coax_ctrl_param_table[i];
        }
    }

    return NULL;
}

static uint8_t coax_ctrl_param_value_valid(const DRV_COAX_CTRL_ParamEntry *entry,
                                           float value)
{
    if ((entry == NULL) || !isfinite(value)) {
        return 0U;
    }

    if (fabsf(value) > 2000.0f) {
        return 0U;
    }

    if (entry->offset == offsetof(DRV_COAX_CTRL_Params, tilt_limit_rad)) {
        return ((value > 0.0f) &&
                (value <= DRV_COAX_CTRL_TILT_LIMIT_RAD)) ? 1U : 0U;
    }

    if (entry->offset == offsetof(DRV_COAX_CTRL_Params, vel_loop_enable)) {
        return ((value >= 0.0f) && (value <= 1.0f)) ? 1U : 0U;
    }
    if ((entry->offset == offsetof(DRV_COAX_CTRL_Params,
                                   attitude.rate_limit_rad_s[0])) ||
        (entry->offset == offsetof(DRV_COAX_CTRL_Params,
                                   attitude.rate_limit_rad_s[1])) ||
        (entry->offset == offsetof(DRV_COAX_CTRL_Params,
                                   attitude.rate_limit_rad_s[2]))) {
        return (value > 0.0f) ? 1U : 0U;
    }
    return (value >= 0.0f) ? 1U : 0U;
}

static uint8_t coax_ctrl_params_valid(const DRV_COAX_CTRL_Params *params)
{
    if (params == NULL) {
        return 0U;
    }

    for (uint32_t i = 0U; i < coax_ctrl_param_count; ++i) {
        if (coax_ctrl_param_value_valid(&coax_ctrl_param_table[i],
                                        *coax_ctrl_param_ptr((DRV_COAX_CTRL_Params *)params,
                                                             &coax_ctrl_param_table[i])) == 0U) {
            return 0U;
        }
    }

    return 1U;
}

static void coax_ctrl_apply_fixed_model_params(DRV_COAX_CTRL_Params *params)
{
    if (params == NULL) {
        return;
    }

    params->mass_kg = DRV_AIRFRAME_MASS_KG;
    params->gravity_m_s2 = DRV_AIRFRAME_GRAVITY_M_S2;
    params->pitch_tilt_lever_arm_m = DRV_AIRFRAME_PITCH_THRUST_LEVER_ARM_M;
    params->roll_tilt_lever_arm_m = DRV_AIRFRAME_ROLL_THRUST_LEVER_ARM_M;
    params->yaw_inertia = DRV_AIRFRAME_IZZ_KGM2;
    params->motor_single_max_thrust_n = DRV_COAX_CTRL_SINGLE_MAX_THRUST_N;
    params->yaw_torque_upper_m_per_n = DRV_COAX_CTRL_PROP9047_YAW_M_PER_N;
    params->yaw_torque_lower_m_per_n = DRV_COAX_CTRL_PROP9047_YAW_M_PER_N;
}

static void coax_ctrl_compute_accel_cmd(
    const DRV_COAX_CTRL_AttitudeInput *attitude,
    const DRV_COAX_CTRL_Reference *reference,
    const DRV_COAX_CTRL_Schedule *schedule,
    float horizontal_scale,
    DRV_COAX_CTRL_Debug *debug)
{
    DRV_POSITION_CONTROL_PositionInput position_input;
    DRV_POSITION_CONTROL_VelocityInput velocity_input;
    const float position_sp[3] = { reference->x_m, reference->y_m, reference->z_m };
    const float position_meas[3] = { attitude->x_m, attitude->y_m, attitude->z_m };
    const float velocity_ff[3] = { reference->vx_m_s, reference->vy_m_s,
                                   reference->vz_m_s };
    const float velocity_meas[3] = { attitude->vx_m_s, attitude->vy_m_s,
                                     attitude->vz_m_s };
    const float accel_ff[3] = { reference->ax_m_s2, reference->ay_m_s2,
                                reference->az_m_s2 };
    const uint8_t translation_bypass =
        ((reference->manual_total_force_valid != 0U) ||
         (reference->direct_attitude_target_valid != 0U) ||
         (coax_ctrl_params.vel_loop_enable < 0.5f)) ? 1U : 0U;

    memset(&position_input, 0, sizeof(position_input));
    memset(&velocity_input, 0, sizeof(velocity_input));

    if ((translation_bypass == 0U) && (schedule->position_update != 0U)) {
        position_input.position_sp_m[0] = reference->x_m;
        position_input.position_sp_m[1] = reference->y_m;
        position_input.position_sp_m[2] = reference->z_m;
        position_input.position_meas_m[0] = attitude->x_m;
        position_input.position_meas_m[1] = attitude->y_m;
        position_input.position_meas_m[2] = attitude->z_m;
        position_input.direct_velocity_m_s[0] = reference->vx_m_s;
        position_input.direct_velocity_m_s[1] = reference->vy_m_s;
        position_input.direct_velocity_m_s[2] = reference->vz_m_s;
        memcpy(position_input.velocity_ff_m_s, velocity_ff,
               sizeof(position_input.velocity_ff_m_s));
        position_input.dt_sec = schedule->position_dt_s;
        position_input.measurement_valid = reference->navigation_position_valid;
        position_input.position_bypass = reference->position_control_bypass;
        DRV_POSITION_CONTROL_PositionStep(&coax_ctrl_params.position,
                                          &position_input,
                                          &coax_ctrl_state.position_output);
    }

    if ((translation_bypass == 0U) && (schedule->velocity_update != 0U)) {
        memcpy(velocity_input.velocity_sp_m_s,
               coax_ctrl_state.position_output.velocity_sp_m_s,
               sizeof(velocity_input.velocity_sp_m_s));
        velocity_input.velocity_meas_m_s[0] = attitude->vx_m_s;
        velocity_input.velocity_meas_m_s[1] = attitude->vy_m_s;
        velocity_input.velocity_meas_m_s[2] = attitude->vz_m_s;
        velocity_input.accel_ff_m_s2[0] = reference->ax_m_s2;
        velocity_input.accel_ff_m_s2[1] = reference->ay_m_s2;
        velocity_input.accel_ff_m_s2[2] = reference->az_m_s2;
        memcpy(velocity_input.measured_accel_m_s2,
               attitude->accel_m_s2,
               sizeof(velocity_input.measured_accel_m_s2));
        velocity_input.dt_sec = schedule->velocity_dt_s;
        velocity_input.measurement_valid =
            (reference->navigation_velocity_valid != 0U) &&
            (attitude->acceleration_valid != 0U);
        velocity_input.integrator_enable = schedule->integrator_enable;
        velocity_input.integrator_freeze = schedule->integrator_freeze;
        velocity_input.integrator_reset = schedule->integrator_reset;
        velocity_input.downstream_saturation =
            coax_ctrl_state.translation_saturation;
        DRV_POSITION_CONTROL_VelocityStep(&coax_ctrl_params.position,
                                          &coax_ctrl_state.position,
                                          &velocity_input,
                                          &coax_ctrl_state.velocity_output);
    }

    for (uint32_t axis = 0U; axis < 3U; ++axis) {
        debug->position_sp_m[axis] = position_sp[axis];
        debug->position_m[axis] = position_meas[axis];
        debug->position_error_m[axis] =
            debug->position_sp_m[axis] - debug->position_m[axis];
        debug->velocity_ff_m_s[axis] = velocity_ff[axis];
        debug->velocity_sp_m_s[axis] =
            coax_ctrl_state.position_output.velocity_sp_m_s[axis];
        debug->velocity_m_s[axis] = velocity_meas[axis];
        debug->velocity_error_m_s[axis] =
            coax_ctrl_state.velocity_output.error_m_s[axis];
        debug->velocity_p_m_s2[axis] =
            coax_ctrl_state.velocity_output.p_term_m_s2[axis];
        debug->velocity_i_m_s2[axis] =
            coax_ctrl_state.velocity_output.i_term_m_s2[axis];
        debug->velocity_d_m_s2[axis] =
            coax_ctrl_state.velocity_output.d_term_m_s2[axis];
        debug->velocity_ff_m_s2[axis] =
            coax_ctrl_state.velocity_output.ff_term_m_s2[axis];
        debug->accel_unsat_m_s2[axis] =
            coax_ctrl_state.velocity_output.accel_unsat_m_s2[axis];
        debug->accel_out_m_s2[axis] =
            coax_ctrl_state.velocity_output.accel_sat_m_s2[axis];
        debug->pos_p_m_s2[axis] =
            coax_ctrl_params.position.vel_kp[axis] *
            coax_ctrl_params.position.pos_kp[axis] *
            debug->position_error_m[axis];
        debug->vel_d_m_s2[axis] = debug->velocity_p_m_s2[axis];
    }

    debug->pos_z_i_m_s2 = debug->velocity_i_m_s2[2];
    if (translation_bypass != 0U) {
        memset(&coax_ctrl_state.position_output, 0,
               sizeof(coax_ctrl_state.position_output));
        memset(&coax_ctrl_state.velocity_output, 0,
               sizeof(coax_ctrl_state.velocity_output));
        for (uint32_t axis = 0U; axis < 3U; ++axis) {
            debug->pos_p_m_s2[axis] = 0.0f;
            debug->vel_d_m_s2[axis] = 0.0f;
            debug->velocity_error_m_s[axis] = 0.0f;
            debug->velocity_p_m_s2[axis] = 0.0f;
            debug->velocity_i_m_s2[axis] = 0.0f;
            debug->velocity_d_m_s2[axis] = 0.0f;
            debug->velocity_ff_m_s2[axis] = accel_ff[axis];
            debug->accel_out_m_s2[axis] = accel_ff[axis];
            debug->accel_unsat_m_s2[axis] = debug->accel_out_m_s2[axis];
        }
        if ((reference->manual_total_force_valid != 0U) ||
            (reference->direct_attitude_target_valid != 0U)) {
            debug->accel_out_m_s2[0] = 0.0f;
            debug->accel_out_m_s2[1] = 0.0f;
            debug->accel_unsat_m_s2[0] = 0.0f;
            debug->accel_unsat_m_s2[1] = 0.0f;
        }
    }

    if (horizontal_scale < 0.999f) {
        debug->accel_out_m_s2[0] *= horizontal_scale;
        debug->accel_out_m_s2[1] *= horizontal_scale;
    }
}

static void coax_ctrl_compute_balance_solution(
    const DRV_COAX_CTRL_AttitudeInput *attitude,
    const DRV_COAX_CTRL_Reference *reference,
    const DRV_COAX_CTRL_Schedule *schedule,
    DRV_COAX_CTRL_Debug *debug,
    DRV_COAX_CTRL_BalanceSolution *solution)
{
    float actual_r[3][3];
    DRV_AttitudeControl_Input attitude_input;
    DRV_RateControl_Input rate_input;
    float error_for_angle[3];
    const float actual_omega[3] = {
        attitude->gyro_x_rad_s,
        attitude->gyro_y_rad_s,
        attitude->gyro_z_rad_s,
    };
    float force_scale = 1.0f;
    float target_pitch_rad;
    float target_roll_rad;
    float roll_limit_moment_n_m;
    float pitch_limit_moment_n_m;
    float yaw_limit_moment_n_m;
    float roll_utilization;
    float pitch_utilization;
    float yaw_utilization;

    memset(solution, 0, sizeof(*solution));
    solution->desired_force_local_n[0] =
        coax_ctrl_params.mass_kg * debug->accel_out_m_s2[0];
    solution->desired_force_local_n[1] =
        coax_ctrl_params.mass_kg * debug->accel_out_m_s2[1];
    if (reference->manual_total_force_valid != 0U) {
        solution->desired_force_local_n[2] =
            coax_ctrl_clamp_f32(reference->manual_total_force_n,
                                DRV_COAX_CTRL_FORCE_EPS_N,
                                DRV_AIRFRAME_MAX_TOTAL_FORCE_N);
    } else {
        /*
         * R-F6-2: Z is now up-positive, so Newton's second law along Z gives
         * F_thrust = m*(g + a_up) -- more commanded upward acceleration
         * means more thrust, not less.  (Legacy down-positive Z used
         * F_thrust = m*(g - a_down); this is the same physics, opposite
         * sign convention.)
         */
        solution->desired_force_local_n[2] =
            coax_ctrl_params.mass_kg *
            (coax_ctrl_params.gravity_m_s2 + debug->accel_out_m_s2[2]);
        if (solution->desired_force_local_n[2] < DRV_COAX_CTRL_FORCE_EPS_N) {
            solution->desired_force_local_n[2] = DRV_COAX_CTRL_FORCE_EPS_N;
        }
    }

    solution->raw_total_force_n =
        coax_ctrl_norm3(solution->desired_force_local_n);
    solution->thrust_utilization =
        solution->raw_total_force_n / DRV_AIRFRAME_MAX_TOTAL_FORCE_N;
    if (solution->raw_total_force_n > DRV_AIRFRAME_MAX_TOTAL_FORCE_N) {
        force_scale = DRV_AIRFRAME_MAX_TOTAL_FORCE_N /
                      solution->raw_total_force_n;
        for (uint32_t axis = 0U; axis < 3U; ++axis) {
            solution->desired_force_local_n[axis] *= force_scale;
        }
    }
    solution->total_force_n = coax_ctrl_norm3(solution->desired_force_local_n);

    /*
     * Horizontal outer loop owns acceleration only. The requested force vector
     * is converted once into a target body attitude; servos are not commanded
     * directly from velocity or position terms.
     */
    if (reference->direct_attitude_target_valid != 0U) {
        target_roll_rad =
            coax_ctrl_clamp_f32(reference->target_roll_rad,
                                -coax_ctrl_params.tilt_limit_rad,
                                 coax_ctrl_params.tilt_limit_rad);
        target_pitch_rad =
            coax_ctrl_clamp_f32(reference->target_pitch_rad,
                                -coax_ctrl_params.tilt_limit_rad,
                                 coax_ctrl_params.tilt_limit_rad);
    } else {
        target_pitch_rad =
            atan2f(solution->desired_force_local_n[0],
                   solution->desired_force_local_n[2]);
        target_roll_rad =
            -atan2f(solution->desired_force_local_n[1] * cosf(target_pitch_rad),
                    solution->desired_force_local_n[2]);
    }
    coax_ctrl_attitude_matrix(attitude, actual_r);
    coax_ctrl_rpy_matrix(target_roll_rad,
                         target_pitch_rad,
                         reference->yaw_rad,
                         solution->desired_body_r);
    coax_ctrl_attitude_error(solution->desired_body_r,
                             actual_r,
                             error_for_angle,
                             &solution->attitude_error_angle_rad,
                             &solution->attitude_tilt_error_rad);
    memset(&attitude_input, 0, sizeof(attitude_input));
    memcpy(attitude_input.actual_rotation, actual_r, sizeof(actual_r));
    memcpy(attitude_input.desired_rotation, solution->desired_body_r,
           sizeof(solution->desired_body_r));
    attitude_input.desired_rate_in_desired_frame[2] =
        reference->yaw_rate_rad_s;
    if (schedule->attitude_update != 0U) {
        (void)DRV_AttitudeControl_Step(&coax_ctrl_params.attitude,
                                       &attitude_input,
                                       &coax_ctrl_state.attitude_output);
    }

    roll_limit_moment_n_m = fabsf(coax_ctrl_roll_moment_from_tilt(
        solution->total_force_n, coax_ctrl_params.tilt_limit_rad));
    pitch_limit_moment_n_m = fabsf(coax_ctrl_pitch_moment_from_tilt(
        solution->total_force_n, coax_ctrl_params.tilt_limit_rad, 0.0f));
    yaw_limit_moment_n_m = coax_ctrl_yaw_limit_moment(solution->total_force_n);
    roll_limit_moment_n_m = fmaxf(roll_limit_moment_n_m,
                                  DRV_COAX_CTRL_RATE_SCALE_EPS);
    pitch_limit_moment_n_m = fmaxf(pitch_limit_moment_n_m,
                                   DRV_COAX_CTRL_RATE_SCALE_EPS);
    yaw_limit_moment_n_m = fmaxf(yaw_limit_moment_n_m,
                                 DRV_COAX_CTRL_RATE_SCALE_EPS);

    memset(&rate_input, 0, sizeof(rate_input));
    memcpy(rate_input.omega, actual_omega, sizeof(actual_omega));
    memcpy(rate_input.omega_sp, coax_ctrl_state.attitude_output.omega_sp,
           sizeof(rate_input.omega_sp));
    rate_input.alpha_ff[2] = reference->yaw_accel_rad_s2;
    rate_input.inertia[0] = DRV_AIRFRAME_IXX_KGM2;
    rate_input.inertia[1] = DRV_AIRFRAME_IYY_KGM2;
    rate_input.inertia[2] = DRV_AIRFRAME_IZZ_KGM2;
    rate_input.dt_s = schedule->rate_dt_s;
    rate_input.saturation_positive[0] = roll_limit_moment_n_m;
    rate_input.saturation_positive[1] = pitch_limit_moment_n_m;
    rate_input.saturation_positive[2] = yaw_limit_moment_n_m;
    rate_input.saturation_negative[0] = -roll_limit_moment_n_m;
    rate_input.saturation_negative[1] = -pitch_limit_moment_n_m;
    rate_input.saturation_negative[2] = -yaw_limit_moment_n_m;
    rate_input.measurement_valid = 1U;
    rate_input.integrator_enable = schedule->integrator_enable;
    rate_input.integrator_freeze = schedule->integrator_freeze;
    rate_input.integrator_reset = schedule->integrator_reset;
    memcpy(rate_input.saturation_positive_active,
           coax_ctrl_state.moment_saturation_positive,
           sizeof(rate_input.saturation_positive_active));
    memcpy(rate_input.saturation_negative_active,
           coax_ctrl_state.moment_saturation_negative,
           sizeof(rate_input.saturation_negative_active));
    if (schedule->rate_update != 0U) {
        (void)DRV_RateControl_Step(&coax_ctrl_params.rate,
                                   &coax_ctrl_state.rate,
                                   &rate_input,
                                   &coax_ctrl_state.rate_output);
    } else {
        (void)DRV_RateControl_Evaluate(&coax_ctrl_params.rate,
                                       &coax_ctrl_state.rate,
                                       &rate_input,
                                       &coax_ctrl_state.rate_output);
    }

    memcpy(solution->attitude_error,
           coax_ctrl_state.attitude_output.attitude_error,
           sizeof(solution->attitude_error));
    memcpy(solution->rate_error_rad_s,
           coax_ctrl_state.rate_output.error,
           sizeof(solution->rate_error_rad_s));
    memcpy(solution->moment_cmd_n_m,
           coax_ctrl_state.rate_output.moment_unsat,
           sizeof(solution->moment_cmd_n_m));
    debug->yaw_angle_p_rad_s =
        -coax_ctrl_params.attitude.att_kp[2] * solution->attitude_error[2];
    debug->yaw_rate_d_rad_s = solution->rate_error_rad_s[2];

    solution->beta_rad = coax_ctrl_solve_roll_tilt_from_moment(
        solution->moment_cmd_n_m[0],
        solution->total_force_n,
        coax_ctrl_params.tilt_limit_rad);

    solution->alpha_rad = coax_ctrl_solve_pitch_tilt_from_moment(
        solution->moment_cmd_n_m[1],
        solution->total_force_n,
        solution->beta_rad,
        coax_ctrl_params.tilt_limit_rad);

    roll_limit_moment_n_m = fabsf(coax_ctrl_roll_moment_from_tilt(
        solution->total_force_n,
        coax_ctrl_params.tilt_limit_rad));
    pitch_limit_moment_n_m = fabsf(coax_ctrl_pitch_moment_from_tilt(
        solution->total_force_n,
        coax_ctrl_params.tilt_limit_rad,
        solution->beta_rad));
    if (roll_limit_moment_n_m < DRV_COAX_CTRL_RATE_SCALE_EPS) {
        roll_limit_moment_n_m = DRV_COAX_CTRL_RATE_SCALE_EPS;
    }
    if (pitch_limit_moment_n_m < DRV_COAX_CTRL_RATE_SCALE_EPS) {
        pitch_limit_moment_n_m = DRV_COAX_CTRL_RATE_SCALE_EPS;
    }
    /*
     * 偏航的可行力矩上限由差动推力分配决定，不是倾转限位。分配式
     *   upper = (kl*F - Mz)/(ku+kl)，lower = (ku*F + Mz)/(ku+kl)
     * 要求两路都落在 [0, T_max]，解出四个边界，取最紧的一个：
     *   |Mz| <= min(kl*F, ku*F, (ku+kl)*T_max - ku*F, (ku+kl)*T_max - kl*F)
     * 悬停 F=13.4N、T_max=10.2N、ku=kl=1e-4 时约束来自上桨推力上限，
     * 上限只有 7e-4 N*m —— 偏航权限很紧，所以它必须参与保护缩放，
     * 否则分配环节的 clamp 会静默削掉指令而保护层毫无感知。
     */
    yaw_limit_moment_n_m = coax_ctrl_yaw_limit_moment(solution->total_force_n);
    if (yaw_limit_moment_n_m < DRV_COAX_CTRL_RATE_SCALE_EPS) {
        yaw_limit_moment_n_m = DRV_COAX_CTRL_RATE_SCALE_EPS;
    }
    roll_utilization =
        fabsf(solution->moment_cmd_n_m[0]) / roll_limit_moment_n_m;
    pitch_utilization =
        fabsf(solution->moment_cmd_n_m[1]) / pitch_limit_moment_n_m;
    yaw_utilization =
        fabsf(solution->moment_cmd_n_m[2]) / yaw_limit_moment_n_m;
    solution->moment_utilization = fmaxf(fmaxf(roll_utilization,
                                               pitch_utilization),
                                         yaw_utilization);

    coax_ctrl_local_down_to_body(attitude,
                                 solution->desired_force_local_n,
                                 solution->desired_force_body_n);
    debug->force_cmd_n[0] = solution->desired_force_body_n[0];
    debug->force_cmd_n[1] = solution->desired_force_body_n[1];
    debug->force_cmd_n[2] = solution->desired_force_body_n[2];

    debug->target_attitude_rp_rad[0] = target_roll_rad;
    debug->target_attitude_rp_rad[1] = target_pitch_rad;

    debug->tilt_angle_p_rad[0] = coax_ctrl_solve_pitch_tilt_from_moment(
        -coax_ctrl_params.rate.kp[1] *
         coax_ctrl_params.attitude.att_kp[1] * solution->attitude_error[1],
        solution->total_force_n,
        solution->beta_rad,
        coax_ctrl_params.tilt_limit_rad);
    debug->tilt_angle_p_rad[1] = coax_ctrl_solve_roll_tilt_from_moment(
        -coax_ctrl_params.rate.kp[0] *
         coax_ctrl_params.attitude.att_kp[0] * solution->attitude_error[0],
        solution->total_force_n,
        coax_ctrl_params.tilt_limit_rad);
    debug->tilt_rate_d_rad[0] = coax_ctrl_solve_pitch_tilt_from_moment(
        coax_ctrl_state.rate_output.p_term[1] +
        (coax_ctrl_params.rate.kp[1] *
         coax_ctrl_params.attitude.att_kp[1] * solution->attitude_error[1]),
        solution->total_force_n,
        solution->beta_rad,
        coax_ctrl_params.tilt_limit_rad);
    debug->tilt_rate_d_rad[1] = coax_ctrl_solve_roll_tilt_from_moment(
        coax_ctrl_state.rate_output.p_term[0] +
        (coax_ctrl_params.rate.kp[0] *
         coax_ctrl_params.attitude.att_kp[0] * solution->attitude_error[0]),
        solution->total_force_n,
        coax_ctrl_params.tilt_limit_rad);

    debug->tilt_out_rad[0] = solution->alpha_rad;
    debug->tilt_out_rad[1] = solution->beta_rad;
    coax_ctrl_rotation_to_rpy(solution->desired_body_r,
                              debug->desired_attitude_rpy_rad);
    memcpy(debug->attitude_error,
           solution->attitude_error,
           sizeof(debug->attitude_error));
    memcpy(debug->rate_error_rad_s,
           solution->rate_error_rad_s,
           sizeof(debug->rate_error_rad_s));
    memcpy(debug->moment_cmd_n_m,
           solution->moment_cmd_n_m,
           sizeof(debug->moment_cmd_n_m));
    memcpy(debug->omega_ff_rad_s,
           coax_ctrl_state.attitude_output.omega_ff,
           sizeof(debug->omega_ff_rad_s));
    memcpy(debug->omega_sp_rad_s,
           coax_ctrl_state.attitude_output.omega_sp,
           sizeof(debug->omega_sp_rad_s));
    memcpy(debug->omega_rad_s, actual_omega, sizeof(debug->omega_rad_s));
    memcpy(debug->rate_limit_rad_s,
           coax_ctrl_params.attitude.rate_limit_rad_s,
           sizeof(debug->rate_limit_rad_s));
    memcpy(debug->rate_p_n_m, coax_ctrl_state.rate_output.p_term,
           sizeof(debug->rate_p_n_m));
    memcpy(debug->rate_i_n_m, coax_ctrl_state.rate_output.i_term,
           sizeof(debug->rate_i_n_m));
    memcpy(debug->rate_d_n_m, coax_ctrl_state.rate_output.d_term,
           sizeof(debug->rate_d_n_m));
    memcpy(debug->rate_ff_n_m, coax_ctrl_state.rate_output.ff_term,
           sizeof(debug->rate_ff_n_m));
    debug->total_force_n = solution->total_force_n;
    debug->moment_utilization = solution->moment_utilization;
    debug->thrust_utilization = solution->thrust_utilization;
}

static float coax_ctrl_balance_protection_scale(
    const DRV_COAX_CTRL_Reference *reference,
    const DRV_COAX_CTRL_BalanceSolution *solution,
    uint32_t *flags)
{
    float scale = 1.0f;
    float candidate;

    *flags = 0U;
    if ((reference->direct_attitude_target_valid == 0U) &&
        (coax_ctrl_params.vel_loop_enable >= 0.5f) &&
        (reference->horizontal_velocity_valid == 0U)) {
        *flags |= DRV_COAX_CTRL_PROTECT_VELOCITY_INVALID;
        scale = 0.0f;
    }

    candidate = coax_ctrl_protection_scale(
        solution->attitude_tilt_error_rad,
        DRV_COAX_CTRL_ATTITUDE_PROTECT_START_RAD,
        DRV_COAX_CTRL_ATTITUDE_PROTECT_END_RAD);
    if (candidate < 1.0f) {
        *flags |= DRV_COAX_CTRL_PROTECT_ATTITUDE;
        scale = fminf(scale, candidate);
    }

    candidate = coax_ctrl_protection_scale(
        solution->moment_utilization,
        DRV_COAX_CTRL_MOMENT_PROTECT_START,
        DRV_COAX_CTRL_MOMENT_PROTECT_END);
    if (candidate < 1.0f) {
        *flags |= DRV_COAX_CTRL_PROTECT_MOMENT;
        scale = fminf(scale, candidate);
    }

    candidate = coax_ctrl_protection_scale(
        solution->thrust_utilization,
        DRV_COAX_CTRL_THRUST_PROTECT_START,
        DRV_COAX_CTRL_THRUST_PROTECT_END);
    if (candidate < 1.0f) {
        *flags |= DRV_COAX_CTRL_PROTECT_THRUST;
        scale = fminf(scale, candidate);
    }

    return scale;
}

static void coax_ctrl_compute_balance_command(
    const DRV_COAX_CTRL_AttitudeInput *attitude,
    const DRV_COAX_CTRL_Reference *reference,
    const DRV_COAX_CTRL_Schedule *schedule,
    DRV_COAX_CTRL_Debug *debug,
    DRV_COAX_CTRL_BalanceSolution *solution)
{
    float horizontal_scale;
    DRV_COAX_CTRL_Schedule recalc_schedule;

    coax_ctrl_compute_accel_cmd(attitude,
                                reference,
                                schedule,
                                1.0f,
                                debug);
    coax_ctrl_compute_balance_solution(attitude, reference, schedule,
                                       debug, solution);
    horizontal_scale = coax_ctrl_balance_protection_scale(reference,
                                                           solution,
                                                           &debug->protection_flags);

    /* 保护缩放介入时按缩放后的加速度指令重算一遍，让力矩与实际下发一致。 */
    if (horizontal_scale < 0.999f) {
        recalc_schedule = *schedule;
        recalc_schedule.position_update = 0U;
        recalc_schedule.velocity_update = 0U;
        recalc_schedule.rate_update = 0U;
        coax_ctrl_compute_accel_cmd(attitude,
                                    reference,
                                    &recalc_schedule,
                                    horizontal_scale,
                                    debug);
        coax_ctrl_compute_balance_solution(attitude, reference,
                                           &recalc_schedule, debug, solution);
    }

    debug->horizontal_command_scale = horizontal_scale;
    coax_ctrl_state.translation_saturation.horizontal_scale = horizontal_scale;
    coax_ctrl_state.translation_saturation.tilt_saturated =
        (horizontal_scale < 0.999f) ? 1U : 0U;
}

static void coax_ctrl_allocate_motor_thrust(float total_force_n,
                                            float yaw_torque_cmd,
                                            float *upper_n,
                                            float *lower_n,
                                            uint8_t *upper_saturated,
                                            uint8_t *lower_saturated)
{
    const float ku = coax_ctrl_params.yaw_torque_upper_m_per_n;
    const float kl = coax_ctrl_params.yaw_torque_lower_m_per_n;
    float denom = ku + kl;

    if (denom < DRV_COAX_CTRL_RATE_SCALE_EPS) {
        denom = DRV_COAX_CTRL_RATE_SCALE_EPS;
    }

    const float upper_raw = (kl * total_force_n - yaw_torque_cmd) / denom;
    const float lower_raw = (ku * total_force_n + yaw_torque_cmd) / denom;

    *upper_n = upper_raw;
    *lower_n = lower_raw;

    *upper_n = coax_ctrl_clamp_f32(*upper_n, 0.0f,
                                   coax_ctrl_params.motor_single_max_thrust_n);
    *lower_n = coax_ctrl_clamp_f32(*lower_n, 0.0f,
                                   coax_ctrl_params.motor_single_max_thrust_n);
    if (upper_saturated != NULL) {
        *upper_saturated = (fabsf(*upper_n - upper_raw) >
                            DRV_COAX_CTRL_RATE_SCALE_EPS) ? 1U : 0U;
    }
    if (lower_saturated != NULL) {
        *lower_saturated = (fabsf(*lower_n - lower_raw) >
                            DRV_COAX_CTRL_RATE_SCALE_EPS) ? 1U : 0U;
    }
}

void DRV_COAX_CTRL_Init(void)
{
    if (coax_ctrl_initialized == 0U) {
        DRV_COAX_CTRL_GetDefaultParams(&coax_ctrl_params);
        DRV_COAX_CTRL_GetDefaultServoCalibration(
            &coax_ctrl_servo_calibration);
        DRV_COAX_CTRL_ResetState();
        coax_ctrl_initialized = 1U;
    }
}

void DRV_COAX_CTRL_ResetState(void)
{
    memset(&coax_ctrl_state, 0, sizeof(coax_ctrl_state));
    memset(&coax_ctrl_last_debug, 0, sizeof(coax_ctrl_last_debug));
}

void DRV_COAX_CTRL_GetDefaultParams(DRV_COAX_CTRL_Params *params)
{
    if (params == NULL) {
        return;
    }

    memset(params, 0, sizeof(*params));
    params->position.pos_kp[0] = 0.375f;
    params->position.pos_kp[1] = 0.375f;
    params->position.pos_kp[2] = 3.8f;
    params->position.vel_kp[0] = 0.80f;
    params->position.vel_kp[1] = 0.80f;
    params->position.vel_kp[2] = 1.0f;
    params->position.xy_speed_limit_m_s = 0.40f;
    params->position.z_speed_limit_up_m_s = 0.30f;
    params->position.z_speed_limit_down_m_s = 0.30f;
    params->position.vel_integrator_limit[0] = 1.50f;
    params->position.vel_integrator_limit[1] = 1.50f;
    params->position.vel_integrator_limit[2] = 1.50f;
    params->position.xy_accel_limit_m_s2 = 3.70f;
    params->position.z_accel_limit_up_m_s2 = 3.70f;
    params->position.z_accel_limit_down_m_s2 = 3.70f;
    params->position.accel_lpf_cutoff_hz = 20.0f;
    params->vel_loop_enable = 1.0f;
    params->attitude.att_kp[0] = 0.0671f / 0.1104f;
    params->attitude.att_kp[1] = 0.0660f / 0.1138f;
    params->attitude.att_kp[2] = 1.0f / 0.15f;
    params->attitude.rate_limit_rad_s[0] = 3.49065850f;
    params->attitude.rate_limit_rad_s[1] = 3.49065850f;
    params->attitude.rate_limit_rad_s[2] = 1.04719758f;
    params->rate.kp[0] = 0.1104f;
    params->rate.kp[1] = 0.1138f;
    params->rate.kp[2] = DRV_AIRFRAME_IZZ_KGM2 * 0.15f;
    params->rate.integrator_limit[0] = 0.010f;
    params->rate.integrator_limit[1] = 0.010f;
    params->rate.integrator_limit[2] = 0.00020f;
    params->rate.alpha_lpf_cutoff_rad_s = 188.495559f;
    for (uint32_t axis = 0U; axis < 3U; ++axis) {
        params->rate.large_error_threshold[axis] = 1.5f;
        params->rate.large_error_scale[axis] = 0.0f;
        params->rate.ff_gain[axis] = 1.0f;
    }
    params->tilt_limit_rad = DRV_COAX_CTRL_TILT_LIMIT_RAD;
    coax_ctrl_apply_fixed_model_params(params);
}

void DRV_COAX_CTRL_ResetParams(void)
{
    DRV_COAX_CTRL_GetDefaultParams(&coax_ctrl_params);
    DRV_COAX_CTRL_ResetState();
}

void DRV_COAX_CTRL_GetParams(DRV_COAX_CTRL_Params *params)
{
    if (params == NULL) {
        return;
    }

    DRV_COAX_CTRL_Init();
    *params = coax_ctrl_params;
}

static uint8_t coax_ctrl_try_set_params(const DRV_COAX_CTRL_Params *params)
{
    if (params == NULL) {
        return 0U;
    }

    DRV_COAX_CTRL_Init();
    DRV_COAX_CTRL_Params candidate = *params;
    coax_ctrl_apply_fixed_model_params(&candidate);
    if (coax_ctrl_params_valid(&candidate) != 0U) {
        coax_ctrl_params = candidate;
        DRV_COAX_CTRL_ResetState();
        return 1U;
    }
    return 0U;
}

void DRV_COAX_CTRL_SetParams(const DRV_COAX_CTRL_Params *params)
{
    (void)coax_ctrl_try_set_params(params);
}

uint32_t DRV_COAX_CTRL_ParamCount(void)
{
    return coax_ctrl_param_count;
}

const char *DRV_COAX_CTRL_ParamName(uint32_t index)
{
    return (index < coax_ctrl_param_count) ? coax_ctrl_param_table[index].name : NULL;
}

uint8_t DRV_COAX_CTRL_GetParam(const char *name, float *value)
{
    const DRV_COAX_CTRL_ParamEntry *entry = coax_ctrl_find_param(name);

    if ((name == NULL) || (value == NULL)) {
        return 0U;
    }

    DRV_COAX_CTRL_Init();
    if (strcmp(name, "coax.roll_angle_kp") == 0) {
        *value = coax_ctrl_params.rate.kp[0] *
                 coax_ctrl_params.attitude.att_kp[0];
        return 1U;
    }
    if (strcmp(name, "coax.pitch_angle_kp") == 0) {
        *value = coax_ctrl_params.rate.kp[1] *
                 coax_ctrl_params.attitude.att_kp[1];
        return 1U;
    }
    if (strcmp(name, "coax.roll_rate_kd") == 0) {
        *value = coax_ctrl_params.rate.kp[0];
        return 1U;
    }
    if (strcmp(name, "coax.pitch_rate_kd") == 0) {
        *value = coax_ctrl_params.rate.kp[1];
        return 1U;
    }
    if (strcmp(name, "coax.yaw_angle_kp") == 0) {
        *value = (coax_ctrl_params.rate.kp[2] *
                  coax_ctrl_params.attitude.att_kp[2]) /
                 coax_ctrl_params.yaw_inertia;
        return 1U;
    }
    if (strcmp(name, "coax.yaw_rate_kd") == 0) {
        *value = coax_ctrl_params.rate.kp[2] /
                 coax_ctrl_params.yaw_inertia;
        return 1U;
    }
    if (entry == NULL) {
        return 0U;
    }
    *value = *coax_ctrl_param_ptr(&coax_ctrl_params, entry);
    return 1U;
}

uint8_t DRV_COAX_CTRL_SetParam(const char *name, float value)
{
    const DRV_COAX_CTRL_ParamEntry *entry = coax_ctrl_find_param(name);
    DRV_COAX_CTRL_Params candidate;

    if ((name == NULL) || !isfinite(value) || (value < 0.0f)) {
        return 0U;
    }
    DRV_COAX_CTRL_Init();
    candidate = coax_ctrl_params;
    if (strcmp(name, "coax.roll_angle_kp") == 0) {
        if ((candidate.rate.kp[0] <= DRV_COAX_CTRL_RATE_SCALE_EPS) &&
            (value > 0.0f)) return 0U;
        candidate.attitude.att_kp[0] = value / fmaxf(candidate.rate.kp[0],
                                                     DRV_COAX_CTRL_RATE_SCALE_EPS);
        return coax_ctrl_try_set_params(&candidate);
    }
    if (strcmp(name, "coax.pitch_angle_kp") == 0) {
        if ((candidate.rate.kp[1] <= DRV_COAX_CTRL_RATE_SCALE_EPS) &&
            (value > 0.0f)) return 0U;
        candidate.attitude.att_kp[1] = value / fmaxf(candidate.rate.kp[1],
                                                     DRV_COAX_CTRL_RATE_SCALE_EPS);
        return coax_ctrl_try_set_params(&candidate);
    }
    if ((strcmp(name, "coax.roll_rate_kd") == 0) ||
        (strcmp(name, "coax.pitch_rate_kd") == 0)) {
        const uint32_t axis = (name[5] == 'r') ? 0U : 1U;
        const float kr = candidate.rate.kp[axis] *
                         candidate.attitude.att_kp[axis];
        if ((value <= DRV_COAX_CTRL_RATE_SCALE_EPS) &&
            (kr > DRV_COAX_CTRL_RATE_SCALE_EPS)) return 0U;
        candidate.rate.kp[axis] = value;
        candidate.attitude.att_kp[axis] =
            kr / fmaxf(value, DRV_COAX_CTRL_RATE_SCALE_EPS);
        return coax_ctrl_try_set_params(&candidate);
    }
    if (strcmp(name, "coax.yaw_angle_kp") == 0) {
        if ((candidate.rate.kp[2] <= DRV_COAX_CTRL_RATE_SCALE_EPS) &&
            (value > 0.0f)) return 0U;
        candidate.attitude.att_kp[2] =
            (candidate.yaw_inertia * value) /
            fmaxf(candidate.rate.kp[2], DRV_COAX_CTRL_RATE_SCALE_EPS);
        return coax_ctrl_try_set_params(&candidate);
    }
    if (strcmp(name, "coax.yaw_rate_kd") == 0) {
        const float kr = candidate.rate.kp[2] *
                         candidate.attitude.att_kp[2];
        if (((candidate.yaw_inertia * value) <= DRV_COAX_CTRL_RATE_SCALE_EPS) &&
            (kr > DRV_COAX_CTRL_RATE_SCALE_EPS)) return 0U;
        candidate.rate.kp[2] = candidate.yaw_inertia * value;
        candidate.attitude.att_kp[2] =
            kr / fmaxf(candidate.rate.kp[2], DRV_COAX_CTRL_RATE_SCALE_EPS);
        return coax_ctrl_try_set_params(&candidate);
    }
    if (coax_ctrl_param_value_valid(entry, value) == 0U) {
        return 0U;
    }

    *coax_ctrl_param_ptr(&candidate, entry) = value;
    if (coax_ctrl_params_valid(&candidate) == 0U) {
        return 0U;
    }

    coax_ctrl_params = candidate;
    DRV_COAX_CTRL_ResetState();
    return 1U;
}

void DRV_COAX_CTRL_GetDefaultServoCalibration(
    DRV_COAX_CTRL_ServoCalibration *calibration)
{
    if (calibration == NULL) {
        return;
    }
    calibration->center_us[DRV_COAX_CTRL_SERVO_ALPHA_INDEX] =
        DRV_COAX_CTRL_SERVO_ALPHA_CENTER_US;
    calibration->center_us[DRV_COAX_CTRL_SERVO_BETA_INDEX] =
        DRV_COAX_CTRL_SERVO_BETA_CENTER_US;
    calibration->min_us[DRV_COAX_CTRL_SERVO_ALPHA_INDEX] =
        DRV_COAX_CTRL_SERVO_ALPHA_MIN_US;
    calibration->min_us[DRV_COAX_CTRL_SERVO_BETA_INDEX] =
        DRV_COAX_CTRL_SERVO_BETA_MIN_US;
    calibration->max_us[DRV_COAX_CTRL_SERVO_ALPHA_INDEX] =
        DRV_COAX_CTRL_SERVO_ALPHA_MAX_US;
    calibration->max_us[DRV_COAX_CTRL_SERVO_BETA_INDEX] =
        DRV_COAX_CTRL_SERVO_BETA_MAX_US;
    calibration->pulse_sign[DRV_COAX_CTRL_SERVO_ALPHA_INDEX] = 1;
    calibration->pulse_sign[DRV_COAX_CTRL_SERVO_BETA_INDEX] = 1;
}

uint8_t DRV_COAX_CTRL_ValidateServoCalibration(
    const DRV_COAX_CTRL_ServoCalibration *calibration)
{
    uint32_t index;

    if (calibration == NULL) {
        return 0U;
    }
    for (index = 0U; index < DRV_COAX_CTRL_SERVO_COUNT; ++index) {
        const uint16_t center = calibration->center_us[index];
        const uint16_t minimum = calibration->min_us[index];
        const uint16_t maximum = calibration->max_us[index];
        if ((minimum < DRV_COAX_CTRL_SERVO_PHYSICAL_MIN_US) ||
            (maximum > DRV_COAX_CTRL_SERVO_PHYSICAL_MAX_US) ||
            (minimum >= center) || (center >= maximum) ||
            ((uint16_t)(center - minimum) <
             DRV_COAX_CTRL_SERVO_MIN_CAL_SPAN_US) ||
            ((uint16_t)(maximum - center) <
             DRV_COAX_CTRL_SERVO_MIN_CAL_SPAN_US) ||
            ((calibration->pulse_sign[index] != 1) &&
             (calibration->pulse_sign[index] != -1))) {
            return 0U;
        }
    }
    return 1U;
}

void DRV_COAX_CTRL_ResetServoCalibration(void)
{
    DRV_COAX_CTRL_GetDefaultServoCalibration(&coax_ctrl_servo_calibration);
    DRV_COAX_CTRL_ResetState();
}

void DRV_COAX_CTRL_GetServoCalibration(
    DRV_COAX_CTRL_ServoCalibration *calibration)
{
    if (calibration == NULL) {
        return;
    }
    DRV_COAX_CTRL_Init();
    *calibration = coax_ctrl_servo_calibration;
}

uint8_t DRV_COAX_CTRL_SetServoCalibration(
    const DRV_COAX_CTRL_ServoCalibration *calibration)
{
    if (DRV_COAX_CTRL_ValidateServoCalibration(calibration) == 0U) {
        return 0U;
    }
    DRV_COAX_CTRL_Init();
    coax_ctrl_servo_calibration = *calibration;
    DRV_COAX_CTRL_ResetState();
    return 1U;
}

static uint16_t coax_ctrl_tilt_rad_to_servo_pulse(float tilt_rad,
                                                  uint16_t center_us,
                                                  uint16_t min_us,
                                                  uint16_t max_us)
{
    const float servo_span_us = (float)(DRV_COAX_CTRL_SERVO_PHYSICAL_MAX_US -
                                        DRV_COAX_CTRL_SERVO_PHYSICAL_MIN_US);
    const float servo_us_per_rad = servo_span_us / DRV_COAX_CTRL_SERVO_TRAVEL_RAD;
    DRV_COAX_CTRL_Init();
    const float tilt = coax_ctrl_clamp_f32(tilt_rad,
                                          -coax_ctrl_params.tilt_limit_rad,
                                           coax_ctrl_params.tilt_limit_rad);
    const float pulse_f = (float)center_us + tilt * servo_us_per_rad;
    const int32_t pulse_i =
        (int32_t)(pulse_f + ((pulse_f >= 0.0f) ? 0.5f : -0.5f));

    return coax_ctrl_clamp_u16(pulse_i, min_us, max_us);
}

uint16_t DRV_COAX_CTRL_AlphaTiltRadToServoPulse(float tilt_rad)
{
    DRV_COAX_CTRL_Init();
    return coax_ctrl_tilt_rad_to_servo_pulse(tilt_rad,
        coax_ctrl_servo_calibration.center_us[DRV_COAX_CTRL_SERVO_ALPHA_INDEX],
        coax_ctrl_servo_calibration.min_us[DRV_COAX_CTRL_SERVO_ALPHA_INDEX],
        coax_ctrl_servo_calibration.max_us[DRV_COAX_CTRL_SERVO_ALPHA_INDEX]);
}

uint16_t DRV_COAX_CTRL_BetaTiltRadToServoPulse(float tilt_rad)
{
    DRV_COAX_CTRL_Init();
    return coax_ctrl_tilt_rad_to_servo_pulse(tilt_rad,
        coax_ctrl_servo_calibration.center_us[DRV_COAX_CTRL_SERVO_BETA_INDEX],
        coax_ctrl_servo_calibration.min_us[DRV_COAX_CTRL_SERVO_BETA_INDEX],
        coax_ctrl_servo_calibration.max_us[DRV_COAX_CTRL_SERVO_BETA_INDEX]);
}

static void coax_ctrl_body_tilt_to_servo_tilts(float body_x_tilt_rad,
                                               float body_y_tilt_rad,
                                               float *servo_alpha_tilt_rad,
                                               float *servo_beta_tilt_rad)
{
    /*
     * The tilt-servo module is mounted 90 degrees CCW in top view.
     * Keep controller coordinates as X-forward/Y-right:
     *   servo 1 / alpha is now the left-right physical axis,
     *   servo 2 / beta is now the front-back physical axis.
     */
    if (servo_alpha_tilt_rad != NULL) {
        *servo_alpha_tilt_rad = -body_y_tilt_rad;
    }
    if (servo_beta_tilt_rad != NULL) {
        *servo_beta_tilt_rad = -body_x_tilt_rad;
    }
}

void DRV_COAX_CTRL_BodyTiltRadToServoPulses(float body_x_tilt_rad,
                                            float body_y_tilt_rad,
                                            uint16_t *servo_alpha_us,
                                            uint16_t *servo_beta_us)
{
    float servo_alpha_tilt_rad = 0.0f;
    float servo_beta_tilt_rad = 0.0f;

    coax_ctrl_body_tilt_to_servo_tilts(body_x_tilt_rad,
                                       body_y_tilt_rad,
                                       &servo_alpha_tilt_rad,
                                       &servo_beta_tilt_rad);
    if (servo_alpha_us != NULL) {
        *servo_alpha_us = DRV_COAX_CTRL_AlphaTiltRadToServoPulse(
            servo_alpha_tilt_rad *
            (float)coax_ctrl_servo_calibration.pulse_sign[
                DRV_COAX_CTRL_SERVO_ALPHA_INDEX]);
    }
    if (servo_beta_us != NULL) {
        *servo_beta_us = DRV_COAX_CTRL_BetaTiltRadToServoPulse(
            servo_beta_tilt_rad *
            (float)coax_ctrl_servo_calibration.pulse_sign[
                DRV_COAX_CTRL_SERVO_BETA_INDEX]);
    }
}

static void coax_ctrl_servo_pulses_to_body_tilts(uint16_t servo_alpha_us,
                                                  uint16_t servo_beta_us,
                                                  float *body_x_tilt_rad,
                                                  float *body_y_tilt_rad)
{
    const float servo_span_us =
        (float)(DRV_COAX_CTRL_SERVO_PHYSICAL_MAX_US -
                DRV_COAX_CTRL_SERVO_PHYSICAL_MIN_US);
    const float rad_per_us = DRV_COAX_CTRL_SERVO_TRAVEL_RAD / servo_span_us;
    const float servo_alpha_tilt =
        ((float)servo_alpha_us -
         (float)coax_ctrl_servo_calibration.center_us[
             DRV_COAX_CTRL_SERVO_ALPHA_INDEX]) * rad_per_us *
        (float)coax_ctrl_servo_calibration.pulse_sign[
            DRV_COAX_CTRL_SERVO_ALPHA_INDEX];
    const float servo_beta_tilt =
        ((float)servo_beta_us -
         (float)coax_ctrl_servo_calibration.center_us[
             DRV_COAX_CTRL_SERVO_BETA_INDEX]) * rad_per_us *
        (float)coax_ctrl_servo_calibration.pulse_sign[
            DRV_COAX_CTRL_SERVO_BETA_INDEX];

    if (body_x_tilt_rad != NULL) {
        *body_x_tilt_rad = -servo_beta_tilt;
    }
    if (body_y_tilt_rad != NULL) {
        *body_y_tilt_rad = -servo_alpha_tilt;
    }
}

uint16_t DRV_COAX_CTRL_ThrustToMotorPulse(float thrust_n)
{
    float thrust_g;

    DRV_COAX_CTRL_Init();

    thrust_n = coax_ctrl_clamp_f32(thrust_n, 0.0f,
                                   coax_ctrl_params.motor_single_max_thrust_n);
    thrust_g = thrust_n * DRV_COAX_CTRL_GRAMS_PER_NEWTON * 2.0f;

    if (thrust_g <= coax_ctrl_dual_thrust_g[0]) {
        return coax_ctrl_dual_pwm_us[0];
    }

    for (uint32_t i = 1U; i < DRV_COAX_CTRL_THRUST_TABLE_POINTS; ++i) {
        if (thrust_g <= coax_ctrl_dual_thrust_g[i]) {
            const float left_g = coax_ctrl_dual_thrust_g[i - 1U];
            const float right_g = coax_ctrl_dual_thrust_g[i];
            const float left_pwm = (float)coax_ctrl_dual_pwm_us[i - 1U];
            const float right_pwm = (float)coax_ctrl_dual_pwm_us[i];
            const float ratio =
                (right_g > left_g) ? ((thrust_g - left_g) / (right_g - left_g)) : 0.0f;
            const float pulse_f = left_pwm + ratio * (right_pwm - left_pwm);
            const int32_t pulse_i = (int32_t)(pulse_f + 0.5f);
            return coax_ctrl_clamp_u16(pulse_i,
                                       BSP_PWM_ESC_MIN_US,
                                       BSP_PWM_ESC_MAX_US);
        }
    }

    return BSP_PWM_ESC_MAX_US;
}

float DRV_COAX_CTRL_MotorPulseToTotalThrust(uint16_t pulse_us)
{
    float thrust_g;

    DRV_COAX_CTRL_Init();

    if (pulse_us <= coax_ctrl_dual_pwm_us[0]) {
        return 0.0f;
    }

    for (uint32_t i = 1U; i < DRV_COAX_CTRL_THRUST_TABLE_POINTS; ++i) {
        if (pulse_us <= coax_ctrl_dual_pwm_us[i]) {
            const float left_pwm = (float)coax_ctrl_dual_pwm_us[i - 1U];
            const float right_pwm = (float)coax_ctrl_dual_pwm_us[i];
            const float ratio = ((float)pulse_us - left_pwm) /
                                (right_pwm - left_pwm);

            thrust_g = coax_ctrl_dual_thrust_g[i - 1U] +
                       ratio * (coax_ctrl_dual_thrust_g[i] -
                                coax_ctrl_dual_thrust_g[i - 1U]);
            return coax_ctrl_clamp_f32(
                thrust_g / DRV_COAX_CTRL_GRAMS_PER_NEWTON,
                0.0f,
                2.0f * coax_ctrl_params.motor_single_max_thrust_n);
        }
    }

    return coax_ctrl_dual_thrust_g[DRV_COAX_CTRL_THRUST_TABLE_POINTS - 1U] /
           DRV_COAX_CTRL_GRAMS_PER_NEWTON;
}

void DRV_COAX_CTRL_RunScheduled(const DRV_COAX_CTRL_AttitudeInput *attitude,
                                const DRV_COAX_CTRL_Reference *reference,
                                const DRV_COAX_CTRL_Schedule *schedule,
                                DRV_COAX_CTRL_Output *output)
{
    DRV_COAX_CTRL_Debug debug;
    DRV_COAX_CTRL_BalanceSolution solution;
    float yaw_torque_cmd;
    float thrust_upper_n;
    float thrust_lower_n;
    float requested_alpha_rad;
    float requested_beta_rad;
    float achieved_total_force_n;
    uint8_t upper_motor_saturated = 0U;
    uint8_t lower_motor_saturated = 0U;

    if ((attitude == NULL) || (reference == NULL) || (schedule == NULL) ||
        (output == NULL)) {
        return;
    }

    DRV_COAX_CTRL_Init();
    if (schedule->integrator_reset != 0U) {
        DRV_COAX_CTRL_ResetState();
    }
    memset(&debug, 0, sizeof(debug));
    memset(output, 0, sizeof(*output));

    coax_ctrl_compute_balance_command(attitude, reference, schedule,
                                      &debug, &solution);
    /* 三轴力矩同出一套 SO(3) 控制律；分配器才是分叉点（倾转 vs 差动推力）。 */
    yaw_torque_cmd = solution.moment_cmd_n_m[2];
    coax_ctrl_allocate_motor_thrust(debug.total_force_n,
                                    yaw_torque_cmd,
                                    &thrust_upper_n,
                                    &thrust_lower_n,
                                    &upper_motor_saturated,
                                    &lower_motor_saturated);

    output->thrust_upper_n = thrust_upper_n;
    output->thrust_lower_n = thrust_lower_n;
    requested_alpha_rad = solution.alpha_rad;
    requested_beta_rad = solution.beta_rad;
    output->motor_upper_us = DRV_COAX_CTRL_ThrustToMotorPulse(thrust_upper_n);
    output->motor_lower_us = DRV_COAX_CTRL_ThrustToMotorPulse(thrust_lower_n);
    DRV_COAX_CTRL_BodyTiltRadToServoPulses(requested_alpha_rad,
                                           requested_beta_rad,
                                           &output->servo_alpha_us,
                                           &output->servo_beta_us);
    coax_ctrl_servo_pulses_to_body_tilts(output->servo_alpha_us,
                                         output->servo_beta_us,
                                         &output->alpha_rad,
                                         &output->beta_rad);
    debug.tilt_out_rad[0] = output->alpha_rad;
    debug.tilt_out_rad[1] = output->beta_rad;

    debug.motor_thrust_cmd_n[0] = output->thrust_upper_n;
    debug.motor_thrust_cmd_n[1] = output->thrust_lower_n;
    debug.motor_cmd_us[0] = (float)output->motor_upper_us;
    debug.motor_cmd_us[1] = (float)output->motor_lower_us;
    debug.yaw_torque_cmd =
        (coax_ctrl_params.yaw_torque_lower_m_per_n * output->thrust_lower_n) -
        (coax_ctrl_params.yaw_torque_upper_m_per_n * output->thrust_upper_n);

    achieved_total_force_n = output->thrust_upper_n + output->thrust_lower_n;
    output->moment_achieved_n_m[0] = coax_ctrl_roll_moment_from_tilt(
        achieved_total_force_n, output->beta_rad);
    output->moment_achieved_n_m[1] = coax_ctrl_pitch_moment_from_tilt(
        achieved_total_force_n, output->alpha_rad, output->beta_rad);
    output->moment_achieved_n_m[2] = debug.yaw_torque_cmd;
    for (uint32_t axis = 0U; axis < 3U; ++axis) {
        uint8_t constrained =
            coax_ctrl_state.rate_output.saturated_pos[axis] ||
            coax_ctrl_state.rate_output.saturated_neg[axis];
        if ((axis == 0U) &&
            (fabsf(requested_beta_rad - output->beta_rad) >
             DRV_COAX_CTRL_SERVO_ANGLE_TOL_RAD)) constrained = 1U;
        if ((axis == 1U) &&
            (fabsf(requested_alpha_rad - output->alpha_rad) >
             DRV_COAX_CTRL_SERVO_ANGLE_TOL_RAD)) constrained = 1U;
        if ((axis == 2U) &&
            ((upper_motor_saturated != 0U) ||
             (lower_motor_saturated != 0U))) constrained = 1U;
        if ((constrained != 0U) && (solution.moment_cmd_n_m[axis] > 0.0f)) {
            output->saturation_positive[axis] = 1U;
        } else if ((constrained != 0U) &&
                   (solution.moment_cmd_n_m[axis] < 0.0f)) {
            output->saturation_negative[axis] = 1U;
        }
        coax_ctrl_state.moment_saturation_positive[axis] =
            output->saturation_positive[axis];
        coax_ctrl_state.moment_saturation_negative[axis] =
            output->saturation_negative[axis];
    }
    output->tilt_saturated =
        output->saturation_positive[0] || output->saturation_negative[0] ||
        output->saturation_positive[1] || output->saturation_negative[1] ||
        (debug.horizontal_command_scale < 0.999f);
    output->yaw_differential_saturated =
        output->saturation_positive[2] || output->saturation_negative[2];
    output->thrust_saturated =
        (solution.raw_total_force_n > DRV_AIRFRAME_MAX_TOTAL_FORCE_N) ||
        (fabsf((output->thrust_upper_n + output->thrust_lower_n) -
               debug.total_force_n) > DRV_COAX_CTRL_RATE_SCALE_EPS);
    memcpy(debug.moment_achieved_n_m, output->moment_achieved_n_m,
           sizeof(debug.moment_achieved_n_m));
    memcpy(debug.saturation_positive, output->saturation_positive,
           sizeof(debug.saturation_positive));
    memcpy(debug.saturation_negative, output->saturation_negative,
           sizeof(debug.saturation_negative));
    debug.thrust_saturated = output->thrust_saturated;
    debug.tilt_saturated = output->tilt_saturated;
    debug.yaw_differential_saturated = output->yaw_differential_saturated;
    coax_ctrl_state.translation_saturation.thrust_saturated =
        output->thrust_saturated;
    coax_ctrl_state.translation_saturation.tilt_saturated =
        output->tilt_saturated;
    memset(coax_ctrl_state.translation_saturation.pos_limit, 0,
           sizeof(coax_ctrl_state.translation_saturation.pos_limit));
    memset(coax_ctrl_state.translation_saturation.neg_limit, 0,
           sizeof(coax_ctrl_state.translation_saturation.neg_limit));
    for (uint32_t axis = 0U; axis < 3U; ++axis) {
        const uint8_t limited = (axis < 2U) ? output->tilt_saturated :
                                             output->thrust_saturated;
        if ((limited != 0U) && (debug.accel_out_m_s2[axis] > 0.0f)) {
            coax_ctrl_state.translation_saturation.pos_limit[axis] = 1U;
        } else if ((limited != 0U) && (debug.accel_out_m_s2[axis] < 0.0f)) {
            coax_ctrl_state.translation_saturation.neg_limit[axis] = 1U;
        }
    }

    coax_ctrl_last_debug = debug;
}

void DRV_COAX_CTRL_Run(const DRV_COAX_CTRL_AttitudeInput *attitude,
                       const DRV_COAX_CTRL_Reference *reference,
                       DRV_COAX_CTRL_Output *output)
{
    DRV_COAX_CTRL_Schedule schedule;

    if (reference == NULL) {
        return;
    }
    memset(&schedule, 0, sizeof(schedule));
    schedule.position_update = 1U;
    schedule.velocity_update = 1U;
    schedule.attitude_update = 1U;
    schedule.rate_update = 1U;
    schedule.integrator_enable = 1U;
    schedule.position_dt_s = reference->dt_sec;
    schedule.velocity_dt_s = reference->dt_sec;
    schedule.attitude_dt_s = reference->dt_sec;
    schedule.rate_dt_s = reference->dt_sec;
    DRV_COAX_CTRL_RunScheduled(attitude, reference, &schedule, output);
}

void DRV_COAX_CTRL_GetLastDebug(DRV_COAX_CTRL_Debug *debug)
{
    if (debug == NULL) {
        return;
    }

    *debug = coax_ctrl_last_debug;
}

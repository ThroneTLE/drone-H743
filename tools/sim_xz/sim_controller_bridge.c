#include "sim_controller_bridge.h"

#include "drv_coax_ctrl.h"
#include "drv_airframe_params.h"
#include "drv_prop_map.h"
#include "drv_servo_actuator_model.h"
#include "app_control_scheduler.h"

#include <string.h>

static APP_ControlSchedulerState sim_scheduler;
static uint64_t sim_time_us;
static uint64_t sim_navigation_token;

/*
 * 被仿真的那架飞机。
 *
 * 固件里**没有**机体数据：真机的唯一来源是上位机写进 Flash 的那一份
 * （drv_airframe_params.h）。仿真器没有 Flash，所以它必须自己说出"我在仿谁"——
 * 下面这组就是它的被控对象定义，写在这里正合适：改仿真的飞机只改这里，
 * 不会碰到真机的任何一个数。
 *
 * 数值逐位复刻改造前 drv_airframe_model.h 的内容，**包括那处矛盾**：四个部件
 * 质量加起来是 754.6 g，而整机质量写的是 1.3670 kg。所以这里用手动派生档
 * （derived_auto = 0），把派生值原样钉住。这不是在维护那个矛盾，而是为了让
 * "机体模型改成运行时取值"这次改动在仿真里**逐位不改变行为**——真机的正确
 * 数值要拿秤重新量，从上位机写进去。例外是两个舵机转轴（2026-09-27 起决定
 * 倾转力矩），理由写在字段旁边。
 */
static uint8_t sim_airframe_loaded;

static void sim_controller_load_airframe(void)
{
    DRV_Airframe_Params airframe;

    if (sim_airframe_loaded != 0U) {
        return;
    }
    sim_airframe_loaded = 1U;

    memset(&airframe, 0, sizeof(airframe));

    airframe.board_mass_g            = 75.0f;
    airframe.battery_mass_g          = 232.0f;
    airframe.base_mass_g             = 99.0f;
    airframe.servo_motor_mass_g      = 348.6f;
    airframe.board_cg_z_m            = 0.0f;
    airframe.battery_cg_z_m          = 0.109f;
    airframe.base_cg_z_m             = -0.117f;
    airframe.servo_motor_cg_z_m      = -0.244f;

    airframe.imu_z_m                 = 0.0f;
    airframe.prop_plane_d_m          = 0.2500f;
    airframe.roll_axis_to_prop_plane_m  = 0.1450f;
    airframe.pitch_axis_to_prop_plane_m = 0.1050f;
    /*
     * 2026-09-27：倾转力臂改为几何量 重心 z − 舵机转轴 z（控制律与下面的被控
     * 对象共用这一个数，没有经验系数）。转轴取作者 2026-09-27 实测的 −0.13 m，
     * 不再用旧头文件的 −0.161/−0.215：那两个是旧机体的量，而控制律的默认增益
     * 已按 2026-09-27 几何换算，被仿的飞机得是那架飞机。力臂 = −0.0946 − (−0.13)
     * = 0.0354 m（真机按部件表重心 −0.094558 算是 0.035442 m）。
     */
    airframe.servo1_axis_z_m         = -0.13f;
    airframe.servo2_axis_z_m         = -0.13f;
    airframe.thrust_point_z_m        = -0.2955f;
    airframe.tether_attach_z_m       = 0.1563f;
    airframe.tether_rope_m           = 0.6400f;

    airframe.ixx_kgm2                = 0.051f;
    airframe.iyy_kgm2                = 0.051f;
    airframe.izz_kgm2                = 0.005f;
    airframe.gravity_m_s2            = 9.81f;
    airframe.max_total_thrust_g      = 1595.342f;
    airframe.servo_deg_per_us        = 0.090f;

    /* 手动档：下面这些就是改造前头文件里的字面值，不由部件表推算。 */
    airframe.derived_auto            = 0.0f;
    airframe.mass_kg                 = 1.3670f;
    airframe.cg_z_m                  = -0.0946f;
    airframe.weight_n                = 13.410270f;
    airframe.thrust_point_to_cg_z_m  = -0.2955f - (-0.0946f);
    airframe.tether_attach_to_cg_m   = 0.2509f;
    airframe.tether_rod_to_cg_m      = 0.8909f;
    airframe.max_total_force_n       = 15.644959f;
    airframe.hover_thrust_percent    = 85.716236f;
    airframe.servo_us_per_deg        = 11.111111f;

    DRV_Airframe_SetParams(&airframe);

    /*
     * 被仿真的那架飞机的接线：通道 1 = 上桨（俯视逆时针），通道 2 = 下桨
     * （俯视顺时针）。2026-09-13 之前这是 airframe.lower_rotor_spin_sense = -1
     * 加上写死的通道顺序，推出的偏航极性是 +1；这里逐位复刻它，所以仿真轨迹
     * 一个数都不变。真机的这一组要靠上位机通电标定，仿真器没有 Flash，
     * 只能自己说出"我在仿谁"。
     */
    {
        DRV_PropMap prop;

        DRV_PropMap_Defaults(&prop);
        prop.channel[0].role = (uint8_t)DRV_PROP_ROLE_UPPER;
        prop.channel[0].spin_sense = DRV_PROP_SPIN_CCW;
        prop.channel[1].role = (uint8_t)DRV_PROP_ROLE_LOWER;
        prop.channel[1].spin_sense = DRV_PROP_SPIN_CW;
        prop.calibrated = 1U;
        (void)DRV_PropMap_PublishActive(&prop);
    }
}

void sim_controller_reset(void)
{
    sim_controller_load_airframe();
    DRV_COAX_CTRL_ResetState();
    APP_ControlScheduler_Reset(&sim_scheduler);
    sim_time_us = 0ULL;
    sim_navigation_token = 0ULL;
}

void sim_controller_reset_params(void)
{
    /* 机体模型必须先于默认增益装好：偏航默认增益按 I_zz 缩放。 */
    sim_controller_load_airframe();
    DRV_COAX_CTRL_ResetParams();
    sim_controller_reset();
}

static DRV_COAX_CTRL_Params sim_controller_params(void)
{
    DRV_COAX_CTRL_Params params;
    memset(&params, 0, sizeof(params));
    sim_controller_load_airframe();
    DRV_COAX_CTRL_GetParams(&params);
    return params;
}

float sim_controller_mass_kg(void)
{
    return sim_controller_params().mass_kg;
}

float sim_controller_gravity_m_s2(void)
{
    return sim_controller_params().gravity_m_s2;
}

float sim_controller_pitch_inertia_kgm2(void)
{
    sim_controller_load_airframe();
    return DRV_Airframe_Get()->iyy_kgm2;
}

/*
 * 被控对象的俯仰力臂：直接取控制律正在用的那个带符号几何力臂
 * （重心 z − 2 号舵机转轴 z）。被控对象与控制器用同一个数、同一个公式
 * τ = 力臂 × T × sin(倾角)——没有第二个"有效系数"可以让两边悄悄对不上。
 */
float sim_controller_pitch_lever_arm_m(void)
{
    return sim_controller_params().pitch_tilt_lever_arm_m;
}

float sim_controller_tilt_tau_s(void)
{
    return DRV_AIRFRAME_SERVO_BETA_ACTUATOR_TAU_INCREASE_S;
}

float sim_controller_tilt_gain(void)
{
    return DRV_AIRFRAME_SERVO_BETA_ACTUATOR_GAIN;
}

float sim_controller_tilt_delay_s(void)
{
    return DRV_AIRFRAME_SERVO_BETA_ACTUATOR_DELAY_S;
}

float sim_controller_tilt_tau_decrease_s(void)
{
    return DRV_AIRFRAME_SERVO_BETA_ACTUATOR_TAU_DECREASE_S;
}

float sim_controller_max_total_thrust_n(void)
{
    sim_controller_load_airframe();
    return DRV_Airframe_Get()->max_total_force_n;
}

float sim_controller_servo_limit_rad(void)
{
    return DRV_COAX_CTRL_SERVO_LIMIT_DEG * 0.017453292519943295f;
}

float sim_controller_pitch_pulse_direction(void)
{
    uint16_t a0, b0, a1, b1;
    DRV_COAX_CTRL_BodyTiltRadToServoPulses(0.0f, 0.0f, &a0, &b0);
    DRV_COAX_CTRL_BodyTiltRadToServoPulses(0.01f, 0.0f, &a1, &b1);
    return b1 >= b0 ? 1.0f : -1.0f;
}

uint32_t sim_controller_param_count(void)
{
    return DRV_COAX_CTRL_ParamCount();
}

const char *sim_controller_param_name(uint32_t index)
{
    return DRV_COAX_CTRL_ParamName(index);
}

uint8_t sim_controller_get_param(const char *name, float *value)
{
    return DRV_COAX_CTRL_GetParam(name, value);
}

uint8_t sim_controller_set_param(const char *name, float value)
{
    return DRV_COAX_CTRL_SetParam(name, value);
}

void sim_controller_step(const SimControllerInput *input,
                         SimControllerOutput *output)
{
    sim_controller_load_airframe();

    DRV_COAX_CTRL_AttitudeInput attitude = {0};
    DRV_COAX_CTRL_Reference reference = {0};
    DRV_COAX_CTRL_Schedule schedule = {0};
    DRV_COAX_CTRL_Output controller_output = {0};
    DRV_COAX_CTRL_Debug debug = {0};

    if ((input == NULL) || (output == NULL)) {
        return;
    }
    attitude.x_m = input->x_m;
    attitude.z_m = input->z_m;
    attitude.vx_m_s = input->vx_m_s;
    attitude.vz_m_s = input->vz_m_s;
    attitude.pitch_rad = input->pitch_rad;
    attitude.gyro_y_rad_s = input->pitch_rate_rad_s;
    memcpy(attitude.accel_m_s2, input->accel_m_s2, sizeof(attitude.accel_m_s2));
    attitude.acceleration_valid = input->acceleration_valid;

    reference.x_m = input->target_x_m;
    reference.z_m = input->target_z_m;
    reference.vx_m_s = input->direct_velocity_x_m_s;
    reference.navigation_position_valid = 1U;
    reference.navigation_velocity_valid = 1U;
    reference.horizontal_velocity_valid = 1U;
    reference.position_control_bypass = input->position_control_bypass;
    reference.direct_attitude_target_valid = input->direct_attitude_target_valid;
    reference.target_pitch_rad = input->target_pitch_rad;
    reference.manual_total_force_valid = input->manual_total_force_valid;
    reference.manual_total_force_n = input->manual_total_force_n;

    APP_ControlSchedule scheduled = {0};
    sim_time_us += (uint64_t)(input->dt_s * 1000000.0f);
    sim_navigation_token++;
    APP_ControlScheduler_Step(&sim_scheduler, sim_time_us, sim_navigation_token,
                              1U, &scheduled);
    schedule.position_update = scheduled.position_due;
    schedule.velocity_update = scheduled.velocity_due;
    schedule.attitude_update = scheduled.attitude_due;
    schedule.rate_update = scheduled.rate_due;
    schedule.integrator_enable = 1U;
    schedule.integrator_reset = input->integrator_reset;
    schedule.position_dt_s = scheduled.position_dt_s;
    schedule.velocity_dt_s = scheduled.velocity_dt_s;
    schedule.attitude_dt_s = scheduled.attitude_dt_s;
    schedule.rate_dt_s = scheduled.rate_dt_s;

    DRV_COAX_CTRL_RunScheduled(&attitude, &reference, &schedule,
                               &controller_output);
    APP_ControlScheduler_Commit(&sim_scheduler, sim_time_us,
                                sim_navigation_token, &scheduled);
    DRV_COAX_CTRL_GetLastDebug(&debug);
    memset(output, 0, sizeof(*output));
    /* The motor table describes two rotors at the same pulse: divide by two
     * to obtain each rotor's static achievable thrust after PWM quantization.
     * The plant then applies actuator dynamics, rather than using ideal force. */
    output->thrust_upper_n = 0.5f * DRV_COAX_CTRL_MotorPulseToTotalThrust(controller_output.motor_upper_us);
    output->thrust_lower_n = 0.5f * DRV_COAX_CTRL_MotorPulseToTotalThrust(controller_output.motor_lower_us);
    output->pitch_tilt_rad = controller_output.alpha_rad;
    output->roll_tilt_rad = controller_output.beta_rad;
    memcpy(output->moment_achieved_n_m, controller_output.moment_achieved_n_m,
           sizeof(output->moment_achieved_n_m));
    memcpy(output->force_cmd_n, debug.force_cmd_n, sizeof(output->force_cmd_n));
    memcpy(output->velocity_sp_m_s, debug.velocity_sp_m_s,
           sizeof(output->velocity_sp_m_s));
    memcpy(output->omega_sp_rad_s, debug.omega_sp_rad_s,
           sizeof(output->omega_sp_rad_s));
}

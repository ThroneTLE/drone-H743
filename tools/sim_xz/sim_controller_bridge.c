#include "sim_controller_bridge.h"

#include "drv_coax_ctrl.h"
#include "drv_airframe_model.h"
#include "app_control_scheduler.h"

#include <string.h>

static APP_ControlSchedulerState sim_scheduler;
static uint64_t sim_time_us;
static uint64_t sim_navigation_token;

void sim_controller_reset(void)
{
    DRV_COAX_CTRL_ResetState();
    APP_ControlScheduler_Reset(&sim_scheduler);
    sim_time_us = 0ULL;
    sim_navigation_token = 0ULL;
}

void sim_controller_reset_params(void)
{
    DRV_COAX_CTRL_ResetParams();
    sim_controller_reset();
}

static DRV_COAX_CTRL_Params sim_controller_params(void)
{
    DRV_COAX_CTRL_Params params;
    memset(&params, 0, sizeof(params));
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
    return DRV_AIRFRAME_IYY_KGM2;
}

float sim_controller_pitch_lever_arm_m(void)
{
    return sim_controller_params().pitch_tilt_lever_arm_m;
}

float sim_controller_tilt_tau_s(void)
{
    return DRV_AIRFRAME_SERVO_BETA_ACTUATOR_TAU_INCREASE_S;
}

float sim_controller_pitch_effectiveness(void)
{
    return SIM_PITCH_EFFECTIVENESS;
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
    return DRV_AIRFRAME_MAX_TOTAL_FORCE_N;
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

#ifndef SIM_CONTROLLER_BRIDGE_H
#define SIM_CONTROLLER_BRIDGE_H

#include <stdint.h>

typedef struct {
    float x_m;
    float z_m;
    float vx_m_s;
    float vz_m_s;
    float pitch_rad;
    float pitch_rate_rad_s;
    float target_x_m;
    float target_z_m;
    float direct_velocity_x_m_s;
    float target_pitch_rad;
    float accel_m_s2[3];
    float manual_total_force_n;
    float dt_s;
    uint8_t position_control_bypass;
    uint8_t direct_attitude_target_valid;
    uint8_t integrator_reset;
    uint8_t acceleration_valid;
    uint8_t manual_total_force_valid;
} SimControllerInput;

typedef struct {
    float thrust_upper_n;
    float thrust_lower_n;
    float pitch_tilt_rad;
    float roll_tilt_rad;
    float moment_achieved_n_m[3];
    float force_cmd_n[3];
    float velocity_sp_m_s[3];
    float omega_sp_rad_s[3];
} SimControllerOutput;

void sim_controller_reset(void);
void sim_controller_reset_params(void);
float sim_controller_mass_kg(void);
float sim_controller_gravity_m_s2(void);
float sim_controller_pitch_inertia_kgm2(void);
float sim_controller_pitch_lever_arm_m(void);
float sim_controller_tilt_tau_s(void);
float sim_controller_tilt_gain(void);
float sim_controller_pitch_effectiveness(void);
float sim_controller_tilt_delay_s(void);
float sim_controller_tilt_tau_decrease_s(void);
float sim_controller_max_total_thrust_n(void);
float sim_controller_servo_limit_rad(void);
float sim_controller_pitch_pulse_direction(void);
uint32_t sim_controller_param_count(void);
const char *sim_controller_param_name(uint32_t index);
uint8_t sim_controller_get_param(const char *name, float *value);
uint8_t sim_controller_set_param(const char *name, float value);
void sim_controller_step(const SimControllerInput *input,
                         SimControllerOutput *output);

#endif

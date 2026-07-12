#ifndef DRV_COAX_CTRL_H
#define DRV_COAX_CTRL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DRV_COAX_CTRL_STATE_LEN 18U
#define DRV_COAX_CTRL_REF_LEN   4U
#define DRV_COAX_CTRL_CMD_LEN   4U

#define DRV_COAX_CTRL_SERVO_ALPHA_CENTER_US 1441U
#define DRV_COAX_CTRL_SERVO_BETA_CENTER_US  1877U
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

typedef struct {
    float x_m;
    float y_m;
    float z_m;
    float vx_m_s;
    float vy_m_s;
    float vz_m_s;
    float ax_m_s2;
    float ay_m_s2;
    float az_m_s2;
    float yaw_rad;
} DRV_COAX_CTRL_Reference;

typedef struct {
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
} DRV_COAX_CTRL_AttitudeInput;

typedef struct {
    float omega_upper;
    float omega_lower;
    float alpha_rad;
    float beta_rad;
    uint16_t servo_alpha_us;
    uint16_t servo_beta_us;
} DRV_COAX_CTRL_Output;

typedef struct {
    float pos_p_m_s2[3];
    float vel_d_m_s2[3];
    float accel_out_m_s2[3];
    float tilt_ff_rad[2];
    float tilt_rate_d_rad[2];
    float tilt_out_rad[2];
    float yaw_angle_p_rad_s;
    float yaw_rate_d_rad_s;
    float yaw_torque_cmd;
    float total_force_n;
    float omega_cmd_rad_s[2];
} DRV_COAX_CTRL_Debug;

typedef struct {
    float pos_x_kp;
    float pos_y_kp;
    float pos_z_kp;
    float vel_x_kd;
    float vel_y_kd;
    float vel_z_kd;
    float rotation_error_gain;
    float accel_xy_limit_m_s2;
    float accel_z_limit_m_s2;
    float vel_loop_enable;
    float vel_loop_x_kp;
    float vel_loop_x_ki;
    float vel_loop_x_kd;
    float vel_loop_y_kp;
    float vel_loop_y_ki;
    float vel_loop_y_kd;
    float vel_loop_output_limit_m_s2;
    float vel_loop_i_limit_m_s2;
    float mass_kg;
    float gravity_m_s2;
    float min_total_force_n;
    float max_total_force_n;
    float tilt_lever_arm_m;
    float roll_angle_kp;
    float roll_rate_kd;
    float pitch_angle_kp;
    float pitch_rate_kd;
    float tilt_limit_rad;
    float yaw_angle_kp;
    float yaw_rate_kd;
    float yaw_rate_limit_rad_s;
    float yaw_inertia;
    float thrust_coeff_n_per_rad2;
    float yaw_torque_coeff_n_m_per_rad2;
    float motor_omega_max_rad_s;
} DRV_COAX_CTRL_Params;

void DRV_COAX_CTRL_Init(void);

void DRV_COAX_CTRL_Run(const DRV_COAX_CTRL_AttitudeInput *attitude,
                       const DRV_COAX_CTRL_Reference *reference,
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

uint16_t DRV_COAX_CTRL_AlphaTiltRadToServoPulse(float tilt_rad);
uint16_t DRV_COAX_CTRL_BetaTiltRadToServoPulse(float tilt_rad);
uint16_t DRV_COAX_CTRL_OmegaToMotorPulse(float omega_rad_s);

#ifdef __cplusplus
}
#endif

#endif

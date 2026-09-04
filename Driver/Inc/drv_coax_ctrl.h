#ifndef DRV_COAX_CTRL_H
#define DRV_COAX_CTRL_H

#include <stdint.h>
#include "drv_attitude_control.h"
#include "drv_position_control.h"
#include "drv_rate_control.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DRV_COAX_CTRL_SERVO_ALPHA_CENTER_US 1500U
#define DRV_COAX_CTRL_SERVO_BETA_CENTER_US  1500U
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
#define DRV_COAX_CTRL_SERVO_COUNT               2U
#define DRV_COAX_CTRL_SERVO_ALPHA_INDEX         0U
#define DRV_COAX_CTRL_SERVO_BETA_INDEX          1U
#define DRV_COAX_CTRL_SERVO_MIN_CAL_SPAN_US    50U

#define DRV_COAX_CTRL_PROTECT_VELOCITY_INVALID (1UL << 0)
#define DRV_COAX_CTRL_PROTECT_ATTITUDE         (1UL << 1)
#define DRV_COAX_CTRL_PROTECT_MOMENT           (1UL << 2)
#define DRV_COAX_CTRL_PROTECT_THRUST           (1UL << 3)

typedef struct {
    /*
     * Paper p_d, p_d_dot and p_d_ddot references in the local controller
     * frame. X is forward, Y is right. Z follows the existing altitude
     * convention:
     * range height above ground is exposed to the controller as z = -height.
     *
     * This is the legacy local-level frame and is deliberately NOT the
     * canonical FLU body frame; the attitude feedback below arrives in FLU
     * instead.  See DRV_COAX_CTRL_AttitudeInput for the full seam 3 frame map.
     */
    float x_m;
    float y_m;
    float z_m;
    float vx_m_s;
    float vy_m_s;
    float vz_m_s;
    float ax_m_s2;
    float ay_m_s2;
    float az_m_s2;
    float dt_sec;
    uint8_t horizontal_velocity_valid;
    uint8_t navigation_position_valid;
    uint8_t navigation_velocity_valid;
    uint8_t position_control_bypass;
    uint8_t direct_attitude_target_valid;
    uint8_t manual_total_force_valid;
    float target_roll_rad;
    float target_pitch_rad;
    float manual_total_force_n;
    /* Paper psi_d, psi_d_dot and psi_d_ddot references. */
    float yaw_rad;
    float yaw_rate_rad_s;
    float yaw_accel_rad_s2;
} DRV_COAX_CTRL_Reference;

typedef struct {
    /*
     * Paper p, p_dot and attitude feedback after App-layer sensor mounting
     * correction.
     *
     * Frames actually delivered by the runtime (seam 3 audit, 2026-08-30):
     *   roll_rad / pitch_rad / yaw_rad, gyro_*_rad_s
     *       canonical FLU body frame of Driver/Inc/drv_frame_contract.h --
     *       +X forward, +Y left, +Z up; +roll right wing down, +pitch nose
     *       down, +yaw nose left.  Passed in unconverted.
     *   x_m / y_m / z_m, vx_m_s / vy_m_s / vz_m_s
     *       still the legacy (forward, right, down) local-level frame; the
     *       App layer negates the altitude channel on the way in.
     *
     * The conversion from the delivered attitude into the control law's own
     * force frame is NOT done by the caller.  It lives in this driver as
     * DRV_COAX_CTRL_FORCE_FRAME_ROLL_SIGN / _PITCH_SIGN (attitude) and
     * DRV_COAX_CTRL_RATE_FRAME_ROLL_SIGN / _PITCH_SIGN (body rates).  The
     * force-frame sign is applied to the measured and the target attitude
     * alike, so it cancels in the attitude error and never reaches stick
     * direction.  Keep polarity in those named constants; never fold it into
     * a gain.
     *
     * Whether those constants suit this airframe is settled by M6 props-off
     * direction verification, not by host tests.
     */
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
    /* Measured local acceleration for the velocity-loop D term [m/s^2]. */
    float accel_m_s2[3];
    uint8_t acceleration_valid;
} DRV_COAX_CTRL_AttitudeInput;

typedef struct {
    uint8_t position_update;
    uint8_t velocity_update;
    uint8_t attitude_update;
    uint8_t rate_update;
    uint8_t integrator_enable;
    uint8_t integrator_freeze;
    uint8_t integrator_reset;
    float position_dt_s;
    float velocity_dt_s;
    float attitude_dt_s;
    float rate_dt_s;
} DRV_COAX_CTRL_Schedule;

typedef struct {
    float thrust_upper_n;
    float thrust_lower_n;
    float alpha_rad;
    float beta_rad;
    uint16_t motor_upper_us;
    uint16_t motor_lower_us;
    uint16_t servo_alpha_us;
    uint16_t servo_beta_us;
    float moment_achieved_n_m[3];
    uint8_t saturation_positive[3];
    uint8_t saturation_negative[3];
    uint8_t thrust_saturated;
    uint8_t tilt_saturated;
    uint8_t yaw_differential_saturated;
} DRV_COAX_CTRL_Output;

typedef struct {
    float pos_p_m_s2[3];
    float pos_z_i_m_s2;
    float vel_d_m_s2[3];
    float accel_out_m_s2[3];
    float force_cmd_n[3];
    float target_attitude_rp_rad[2];
    float tilt_angle_p_rad[2];
    float tilt_rate_d_rad[2];
    float tilt_out_rad[2];
    float yaw_angle_p_rad_s;
    float yaw_rate_d_rad_s;
    float yaw_torque_cmd;
    float total_force_n;
    float motor_thrust_cmd_n[2];
    float motor_cmd_us[2];
    float desired_attitude_rpy_rad[3];
    float attitude_error[3];
    float rate_error_rad_s[3];
    float moment_cmd_n_m[3];
    float moment_achieved_n_m[3];
    float position_sp_m[3];
    float position_m[3];
    float position_error_m[3];
    float velocity_ff_m_s[3];
    float velocity_sp_m_s[3];
    float velocity_m_s[3];
    float velocity_error_m_s[3];
    float velocity_p_m_s2[3];
    float velocity_i_m_s2[3];
    float velocity_d_m_s2[3];
    float velocity_ff_m_s2[3];
    float accel_unsat_m_s2[3];
    float omega_ff_rad_s[3];
    float omega_sp_rad_s[3];
    float omega_rad_s[3];
    float rate_limit_rad_s[3];
    float rate_p_n_m[3];
    float rate_i_n_m[3];
    float rate_d_n_m[3];
    float rate_ff_n_m[3];
    uint8_t saturation_positive[3];
    uint8_t saturation_negative[3];
    uint8_t thrust_saturated;
    uint8_t tilt_saturated;
    uint8_t yaw_differential_saturated;
    float horizontal_command_scale;
    float moment_utilization;
    float thrust_utilization;
    uint32_t protection_flags;
} DRV_COAX_CTRL_Debug;

typedef struct {
    DRV_POSITION_CONTROL_Params position;
    DRV_AttitudeControl_Params attitude;
    DRV_RateControl_Params rate;
    float vel_loop_enable;
    float mass_kg;
    float gravity_m_s2;
    float pitch_tilt_lever_arm_m;
    float roll_tilt_lever_arm_m;
    float tilt_limit_rad;
    float yaw_inertia;
    float motor_single_max_thrust_n;
    float yaw_torque_upper_m_per_n;
    float yaw_torque_lower_m_per_n;
} DRV_COAX_CTRL_Params;

/*
 * Runtime mechanical adapter.  pulse_sign is +1 when a positive mechanism
 * tilt increases pulse width and -1 when installation reverses that motion.
 * This polarity lives after controller/body-frame allocation; it must never
 * be compensated with negative control gains.
 */
typedef struct {
    uint16_t center_us[DRV_COAX_CTRL_SERVO_COUNT];
    uint16_t min_us[DRV_COAX_CTRL_SERVO_COUNT];
    uint16_t max_us[DRV_COAX_CTRL_SERVO_COUNT];
    int8_t pulse_sign[DRV_COAX_CTRL_SERVO_COUNT];
} DRV_COAX_CTRL_ServoCalibration;

void DRV_COAX_CTRL_Init(void);
void DRV_COAX_CTRL_ResetState(void);

void DRV_COAX_CTRL_Run(const DRV_COAX_CTRL_AttitudeInput *attitude,
                       const DRV_COAX_CTRL_Reference *reference,
                       DRV_COAX_CTRL_Output *output);
void DRV_COAX_CTRL_RunScheduled(const DRV_COAX_CTRL_AttitudeInput *attitude,
                                const DRV_COAX_CTRL_Reference *reference,
                                const DRV_COAX_CTRL_Schedule *schedule,
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

void DRV_COAX_CTRL_GetDefaultServoCalibration(
    DRV_COAX_CTRL_ServoCalibration *calibration);
uint8_t DRV_COAX_CTRL_ValidateServoCalibration(
    const DRV_COAX_CTRL_ServoCalibration *calibration);
void DRV_COAX_CTRL_ResetServoCalibration(void);
void DRV_COAX_CTRL_GetServoCalibration(
    DRV_COAX_CTRL_ServoCalibration *calibration);
uint8_t DRV_COAX_CTRL_SetServoCalibration(
    const DRV_COAX_CTRL_ServoCalibration *calibration);

uint16_t DRV_COAX_CTRL_AlphaTiltRadToServoPulse(float tilt_rad);
uint16_t DRV_COAX_CTRL_BetaTiltRadToServoPulse(float tilt_rad);
void DRV_COAX_CTRL_BodyTiltRadToServoPulses(float body_x_tilt_rad,
                                            float body_y_tilt_rad,
                                            uint16_t *servo_alpha_us,
                                            uint16_t *servo_beta_us);
uint16_t DRV_COAX_CTRL_ThrustToMotorPulse(float thrust_n);
float DRV_COAX_CTRL_MotorPulseToTotalThrust(uint16_t pulse_us);

#ifdef __cplusplus
}
#endif

#endif

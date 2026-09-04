#ifndef DRV_ATTITUDE_CONTROL_H
#define DRV_ATTITUDE_CONTROL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DRV_ATTITUDE_CONTROL_AXIS_COUNT 3U

/**
 * Pure attitude-step parameters.
 *
 * att_kp: attitude feedback gain K_R/K_omega [rad/s per rad].  The
 *         negative-feedback command is omega_sp = omega_ff - att_kp*e_R.
 * rate_limit_rad_s: per-axis set-point saturation for |omega_sp| (rad/s).
 */
typedef struct {
    float att_kp[DRV_ATTITUDE_CONTROL_AXIS_COUNT];
    float rate_limit_rad_s[DRV_ATTITUDE_CONTROL_AXIS_COUNT];
} DRV_AttitudeControl_Params;

/**
 * Inputs for the pure SO(3) attitude step.
 *
 * actual_rotation and desired_rotation are body-to-world rotation matrices in
 * the same named controller attitude frame.  The caller owns the explicit
 * legacy/FLU adapter boundary; this module never changes signs or frames. The
 * desired_rate_in_desired_frame is omega_d in the desired body frame; the
 * SO(3) error is dimensionless.
 */
typedef struct {
    float actual_rotation[DRV_ATTITUDE_CONTROL_AXIS_COUNT][DRV_ATTITUDE_CONTROL_AXIS_COUNT];
    float desired_rotation[DRV_ATTITUDE_CONTROL_AXIS_COUNT][DRV_ATTITUDE_CONTROL_AXIS_COUNT];
    float desired_rate_in_desired_frame[DRV_ATTITUDE_CONTROL_AXIS_COUNT];
} DRV_AttitudeControl_Input;

/**
 * Outputs of the attitude step.
 */
typedef struct {
    /* e_R = vee(Rd^T R - R^T Rd)/2: actual-minus-desired rotation error. */
    float attitude_error[DRV_ATTITUDE_CONTROL_AXIS_COUNT];
    float omega_ff[DRV_ATTITUDE_CONTROL_AXIS_COUNT];
    /* Negative-feedback desired body rate, before the rate controller. */
    float omega_sp[DRV_ATTITUDE_CONTROL_AXIS_COUNT];
    uint8_t rate_saturated[DRV_ATTITUDE_CONTROL_AXIS_COUNT];
} DRV_AttitudeControl_Output;

uint8_t DRV_AttitudeControl_Step(
    const DRV_AttitudeControl_Params *params,
    const DRV_AttitudeControl_Input *input,
    DRV_AttitudeControl_Output *output);

#ifdef __cplusplus
}
#endif

#endif  /* DRV_ATTITUDE_CONTROL_H */

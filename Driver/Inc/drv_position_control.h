/* Copyright (c) 2026 drone-H743 contributors. */
#ifndef DRV_POSITION_CONTROL_H
#define DRV_POSITION_CONTROL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Pure translational position-P and velocity-PID controller. */
#define DRV_POSITION_CONTROL_AXIS_COUNT 3U
#define DRV_POSITION_CONTROL_MAX_DT_SEC 0.05f

/*
 * Public frame and timing contract:
 *
 * The controller uses the canonical FLU local frame (drv_frame_contract.h):
 * +X forward, +Y left, +Z up.  Position is metres, velocity is metres per
 * second, acceleration is metres per second squared, and dt_sec is seconds.
 * These are SI control quantities; no HAL, RTOS, I/O, or frame conversion is
 * performed here.
 *
 * R-F6-2 (2026-09-06): legacy +Y right and +Z down inputs are converted to
 * +Y left and +Z up before entering this module.  See
 * doc/req-rf6-2-controller-flu-migration.md section 3.
 *
 * The position and velocity steps are independent.  A caller may run the
 * position step at a slower rate and feed its velocity_sp to the velocity
 * step at a faster rate.  PositionStep does not integrate or retain state.
 *
 * A velocity dt <= 0 or non-finite is invalid: no integrator or LPF update is
 * made, while the finite P/I/D/FF terms are still reported.  A dt above
 * DRV_POSITION_CONTROL_MAX_DT_SEC is clipped to that value and reported.
 * measurement_valid=0 freezes both the integrator and acceleration LPF;
 * integrator_freeze or integrator_enable=0 freezes only the integrator.
 * integrator_reset clears both state elements before the step.  Invalid input
 * and invalid parameters leave state unchanged and return zero-valued output.
 */
typedef struct {
    float pos_kp[DRV_POSITION_CONTROL_AXIS_COUNT];
    float vel_kp[DRV_POSITION_CONTROL_AXIS_COUNT];
    float vel_ki[DRV_POSITION_CONTROL_AXIS_COUNT];
    float vel_kd[DRV_POSITION_CONTROL_AXIS_COUNT];

    /* Position velocity limits: XY vector norm; Z numeric + is up. */
    float xy_speed_limit_m_s;
    float z_speed_limit_up_m_s;   /* limit for positive Z (upward) */
    float z_speed_limit_down_m_s; /* limit for negative Z (downward) */

    /* Velocity-PID acceleration limits: XY vector norm; Z numeric + is up. */
    float vel_integrator_limit[DRV_POSITION_CONTROL_AXIS_COUNT];
    float xy_accel_limit_m_s2;
    float z_accel_limit_up_m_s2;   /* limit for positive Z (upward) */
    float z_accel_limit_down_m_s2; /* limit for negative Z (downward) */

    /* First-order measured-acceleration LPF cutoff in Hz; <=0 bypasses it. */
    float accel_lpf_cutoff_hz;
} DRV_POSITION_CONTROL_Params;

typedef struct {
    float velocity_integrator_m_s2[DRV_POSITION_CONTROL_AXIS_COUNT];
    float accel_lpf_m_s2[DRV_POSITION_CONTROL_AXIS_COUNT];
    uint8_t accel_lpf_initialized;
} DRV_POSITION_CONTROL_State;

/* Saturation is reported in numeric axis directions (+/- X, +/- Y, +/- Z). */
typedef struct {
    uint8_t pos_limit[DRV_POSITION_CONTROL_AXIS_COUNT];
    uint8_t neg_limit[DRV_POSITION_CONTROL_AXIS_COUNT];
    uint8_t thrust_saturated;
    uint8_t tilt_saturated;
    /* XY output/input norm ratio; 1.0 means no horizontal scaling. */
    float horizontal_scale;
} DRV_POSITION_CONTROL_SaturationFeedback;

typedef struct {
    float position_sp_m[DRV_POSITION_CONTROL_AXIS_COUNT];
    float position_meas_m[DRV_POSITION_CONTROL_AXIS_COUNT];
    float velocity_ff_m_s[DRV_POSITION_CONTROL_AXIS_COUNT];
    float direct_velocity_m_s[DRV_POSITION_CONTROL_AXIS_COUNT];
    float dt_sec;
    uint8_t measurement_valid;
    /* Non-zero selects direct_velocity_m_s and bypasses position P. */
    uint8_t position_bypass;
} DRV_POSITION_CONTROL_PositionInput;

typedef struct {
    float velocity_sp_m_s[DRV_POSITION_CONTROL_AXIS_COUNT];
    DRV_POSITION_CONTROL_SaturationFeedback sat;
    uint32_t flags;
} DRV_POSITION_CONTROL_PositionOutput;

typedef struct {
    float velocity_sp_m_s[DRV_POSITION_CONTROL_AXIS_COUNT];
    float velocity_meas_m_s[DRV_POSITION_CONTROL_AXIS_COUNT];
    float accel_ff_m_s2[DRV_POSITION_CONTROL_AXIS_COUNT];
    float measured_accel_m_s2[DRV_POSITION_CONTROL_AXIS_COUNT];
    float dt_sec;
    uint8_t measurement_valid;
    uint8_t integrator_enable;
    uint8_t integrator_freeze;
    uint8_t integrator_reset;
    /* Achieved actuator feedback from the preceding allocation step. */
    DRV_POSITION_CONTROL_SaturationFeedback downstream_saturation;
} DRV_POSITION_CONTROL_VelocityInput;

typedef struct {
    float error_m_s[DRV_POSITION_CONTROL_AXIS_COUNT];
    float p_term_m_s2[DRV_POSITION_CONTROL_AXIS_COUNT];
    float i_term_m_s2[DRV_POSITION_CONTROL_AXIS_COUNT];
    float d_term_m_s2[DRV_POSITION_CONTROL_AXIS_COUNT];
    float ff_term_m_s2[DRV_POSITION_CONTROL_AXIS_COUNT];
    float accel_unsat_m_s2[DRV_POSITION_CONTROL_AXIS_COUNT];
    float accel_sat_m_s2[DRV_POSITION_CONTROL_AXIS_COUNT];
    DRV_POSITION_CONTROL_SaturationFeedback sat;
    uint32_t flags;
} DRV_POSITION_CONTROL_VelocityOutput;

#define DRV_POSITION_CONTROL_FLAG_INPUT_INVALID ((uint32_t)(1U << 0))
#define DRV_POSITION_CONTROL_FLAG_DT_INVALID ((uint32_t)(1U << 1))
#define DRV_POSITION_CONTROL_FLAG_DT_CLIPPED ((uint32_t)(1U << 2))
#define DRV_POSITION_CONTROL_FLAG_MEAS_INVALID ((uint32_t)(1U << 3))
#define DRV_POSITION_CONTROL_FLAG_I_FREEZE ((uint32_t)(1U << 4))
#define DRV_POSITION_CONTROL_FLAG_I_RESET ((uint32_t)(1U << 5))

void DRV_POSITION_CONTROL_ResetState(DRV_POSITION_CONTROL_State *state);

void DRV_POSITION_CONTROL_PositionStep(
    const DRV_POSITION_CONTROL_Params *params,
    const DRV_POSITION_CONTROL_PositionInput *input,
    DRV_POSITION_CONTROL_PositionOutput *output);

void DRV_POSITION_CONTROL_VelocityStep(
    const DRV_POSITION_CONTROL_Params *params,
    DRV_POSITION_CONTROL_State *state,
    const DRV_POSITION_CONTROL_VelocityInput *input,
    DRV_POSITION_CONTROL_VelocityOutput *output);

#ifdef __cplusplus
}
#endif

#endif /* DRV_POSITION_CONTROL_H */

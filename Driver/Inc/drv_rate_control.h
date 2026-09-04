#ifndef DRV_RATE_CONTROL_H
#define DRV_RATE_CONTROL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DRV_RATE_CONTROL_AXIS_COUNT 3U

/**
 * Rate-controller parameters (diagonal-inertia formulation).
 * All gains use SI units and are non-negative.  A non-positive LPF cutoff
 * disables filtering (the measured finite difference is used directly).
 */
typedef struct {
    float kp[DRV_RATE_CONTROL_AXIS_COUNT];
    float ki[DRV_RATE_CONTROL_AXIS_COUNT];
    float kd[DRV_RATE_CONTROL_AXIS_COUNT];
    float integrator_limit[DRV_RATE_CONTROL_AXIS_COUNT];
    float alpha_lpf_cutoff_rad_s;
    float large_error_threshold[DRV_RATE_CONTROL_AXIS_COUNT];
    float large_error_scale[DRV_RATE_CONTROL_AXIS_COUNT];
    float ff_gain[DRV_RATE_CONTROL_AXIS_COUNT];
} DRV_RateControl_Params;

/**
 * Persistent internal state. Kept outside the module for pure-function style.
 * `last_omega` is the previous valid measured body rate (rad/s).
 */
typedef struct {
    float integrator[DRV_RATE_CONTROL_AXIS_COUNT];
    float last_omega[DRV_RATE_CONTROL_AXIS_COUNT];
    float filtered_alpha[DRV_RATE_CONTROL_AXIS_COUNT];
    uint8_t initialized;
} DRV_RateControl_State;

/**
 * Inputs for one rate update.  omega/omega_sp are canonical FLU body-frame rad/s;
 * alpha_ff is desired body angular acceleration in rad/s^2; inertia is the
 * positive diagonal J=[Jxx,Jyy,Jzz] in kg*m^2.  saturation_positive and
 * saturation_negative are per-axis moment bounds in N*m, with positive >
 * negative.  The rate error is omega_sp - omega (desired-minus-measured).
 */
typedef struct {
    float omega[DRV_RATE_CONTROL_AXIS_COUNT];
    float omega_sp[DRV_RATE_CONTROL_AXIS_COUNT];
    float alpha_ff[DRV_RATE_CONTROL_AXIS_COUNT];
    float inertia[DRV_RATE_CONTROL_AXIS_COUNT];
    float dt_s;
    float saturation_positive[DRV_RATE_CONTROL_AXIS_COUNT];
    float saturation_negative[DRV_RATE_CONTROL_AXIS_COUNT];
    uint8_t measurement_valid;
    uint8_t integrator_enable;
    uint8_t integrator_freeze;
    uint8_t integrator_reset;
    /* Actual previous-cycle allocator saturation, per numeric direction. */
    uint8_t saturation_positive_active[DRV_RATE_CONTROL_AXIS_COUNT];
    uint8_t saturation_negative_active[DRV_RATE_CONTROL_AXIS_COUNT];
} DRV_RateControl_Input;

/**
 * Rate-step outputs, including decomposed terms for tests.  The command is
 * M = Kp*e + I - Kd*alpha_filtered + ff_gain*J*alpha_ff
 *     + omega x (J*omega).
 */
typedef struct {
    float error[DRV_RATE_CONTROL_AXIS_COUNT];
    float p_term[DRV_RATE_CONTROL_AXIS_COUNT];
    float i_term[DRV_RATE_CONTROL_AXIS_COUNT];
    float d_term[DRV_RATE_CONTROL_AXIS_COUNT];
    float ff_term[DRV_RATE_CONTROL_AXIS_COUNT];
    float filtered_alpha[DRV_RATE_CONTROL_AXIS_COUNT];
    float moment_unsat[DRV_RATE_CONTROL_AXIS_COUNT];
    float moment_cmd[DRV_RATE_CONTROL_AXIS_COUNT];
    uint8_t saturated_pos[DRV_RATE_CONTROL_AXIS_COUNT];
    uint8_t saturated_neg[DRV_RATE_CONTROL_AXIS_COUNT];
} DRV_RateControl_Output;

void DRV_RateControl_InitState(DRV_RateControl_State *state);

uint8_t DRV_RateControl_Step(
    const DRV_RateControl_Params *params,
    DRV_RateControl_State *state,
    const DRV_RateControl_Input *input,
    DRV_RateControl_Output *output);

/* Re-evaluate with held I/D state; does not mutate state or integrate. */
uint8_t DRV_RateControl_Evaluate(
    const DRV_RateControl_Params *params,
    const DRV_RateControl_State *state,
    const DRV_RateControl_Input *input,
    DRV_RateControl_Output *output);

#ifdef __cplusplus
}
#endif

#endif  /* DRV_RATE_CONTROL_H */

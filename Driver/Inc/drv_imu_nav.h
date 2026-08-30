#ifndef DRV_IMU_NAV_H
#define DRV_IMU_NAV_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Strapdown velocity state.
 *
 * Navigation frame: local-level FLU -- +X forward, +Y left, +Z up -- built
 * each update from the smoothed gravity reference, with heading left free
 * (this estimator has no heading source, so +X is the body forward axis
 * projected into the level plane, not North).  Body inputs are the canonical
 * FLU body frame of Driver/Inc/drv_frame_contract.h.
 *
 * acc_bias_nav_m_s2 / acc_nav_m_s2 / vel_m_s are all expressed in that
 * navigation frame.
 *
 * level_z_body_unit is the navigation *up* axis expressed in body
 * coordinates; at rest it equals the normalised specific force.  Before the
 * seam 2 migration this frame was (forward, right, down) and the field held
 * the down axis instead.
 */
typedef struct {
    float acc_bias_nav_m_s2[3];
    float acc_nav_m_s2[3];
    float vel_m_s[3];
    float level_z_body_unit[3];
    float accel_lpf_alpha;
    float velocity_leak_rate_hz;
    uint8_t bias_ready;
    uint8_t level_ready;
} DRV_IMU_NAV_State;

typedef struct {
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    float roll_rad;
    float pitch_rad;
    float yaw_rad;
    float dt_sec;
    float gravity_m_s2;
    float accel_lpf_alpha;
    float velocity_leak_rate_hz;
} DRV_IMU_NAV_Input;

void DRV_IMU_NAV_Reset(DRV_IMU_NAV_State *state);
void DRV_IMU_NAV_CaptureBias(DRV_IMU_NAV_State *state,
                             const DRV_IMU_NAV_Input *input);
void DRV_IMU_NAV_Update(DRV_IMU_NAV_State *state,
                        const DRV_IMU_NAV_Input *input);

#ifdef __cplusplus
}
#endif

#endif

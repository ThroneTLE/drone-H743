#ifndef DRV_INDI_CTRL_H
#define DRV_INDI_CTRL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float enable;
    float roll_inertia_kg_m2;
    float pitch_inertia_kg_m2;
    float roll_attitude_kp_rad_s2_per_rad;
    float pitch_attitude_kp_rad_s2_per_rad;
    float roll_rate_kd_rad_s2_per_rad_s;
    float pitch_rate_kd_rad_s2_per_rad_s;
    float angular_accel_lpf_alpha;
    float correction_limit_rad;
    float increment_limit_rad;
    float correction_leak_hz;
    float roll_effectiveness_sign;
    float pitch_effectiveness_sign;
} DRV_INDI_Config;

typedef struct {
    float roll_rad;
    float pitch_rad;
    float roll_ref_rad;
    float pitch_ref_rad;
    float gyro_x_rad_s;
    float gyro_y_rad_s;
    float base_alpha_rad;
    float base_beta_rad;
    float total_force_n;
    float tilt_lever_arm_m;
    float dt_sec;
} DRV_INDI_Input;

typedef struct {
    float alpha_rad;
    float beta_rad;
    float angular_accel_rad_s2[2];
    float virtual_accel_rad_s2[2];
    float effectiveness_rad_s2_per_rad[2];
    float correction_rad[2];
    uint8_t active;
} DRV_INDI_Output;

typedef struct {
    float previous_rate_rad_s[2];
    float angular_accel_rad_s2[2];
    float correction_rad[2];
    uint8_t initialized;
} DRV_INDI_State;

void DRV_INDI_DefaultConfig(DRV_INDI_Config *config);
void DRV_INDI_Reset(DRV_INDI_State *state);
void DRV_INDI_Step(DRV_INDI_State *state,
                   const DRV_INDI_Config *config,
                   const DRV_INDI_Input *input,
                   DRV_INDI_Output *output);

#ifdef __cplusplus
}
#endif

#endif

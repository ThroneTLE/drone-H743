#ifndef DRV_ATTITUDE_FUSION_H
#define DRV_ATTITUDE_FUSION_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Attitude-only fusion input in the aircraft's compensated NED contract.
 * Gyroscope units are degrees per second and accelerometer units are g.
 */
typedef struct {
    uint64_t time_us;
    float gyroscope_dps[3];
    float accelerometer_g[3];
    float dt_s;
} DRV_AttitudeFusionInput;

typedef struct {
    uint64_t time_us;
    float quaternion[4];
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float acceleration_error_deg;
    float acceleration_recovery_trigger;
    float sample_period_s;
    uint32_t sample_count;
    uint32_t accel_correction_count;
    uint8_t initialized;
    uint8_t startup;
    uint8_t accelerometer_ignored;
    uint8_t acceleration_recovery;
    uint8_t angular_rate_recovery;
    uint8_t accel_norm_rejected;
} DRV_AttitudeFusionOutput;

void DRV_AttitudeFusion_Init(void);
uint8_t DRV_AttitudeFusion_Update(
    const DRV_AttitudeFusionInput *input,
    DRV_AttitudeFusionOutput *output);
void DRV_AttitudeFusion_GetOutput(DRV_AttitudeFusionOutput *output);

#ifdef __cplusplus
}
#endif

#endif /* DRV_ATTITUDE_FUSION_H */

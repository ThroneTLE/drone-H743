#ifndef DRV_ATTITUDE_FUSION_H
#define DRV_ATTITUDE_FUSION_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    DRV_ATTITUDE_FUSION_CONVENTION_NED = 0U,
    DRV_ATTITUDE_FUSION_CONVENTION_NWU = 1U,
} DRV_AttitudeFusionConvention;

/*
 * Attitude-only fusion input.  The caller must identify the convention at
 * initialisation and provide both gyroscope and accelerometer in that same
 * proper body frame.  Units are degrees/second and g.
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
void DRV_AttitudeFusion_InitForConvention(
    DRV_AttitudeFusionConvention convention);
uint8_t DRV_AttitudeFusion_Update(
    const DRV_AttitudeFusionInput *input,
    DRV_AttitudeFusionOutput *output);
void DRV_AttitudeFusion_GetOutput(DRV_AttitudeFusionOutput *output);

#ifdef __cplusplus
}
#endif

#endif /* DRV_ATTITUDE_FUSION_H */

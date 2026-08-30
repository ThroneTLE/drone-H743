#ifndef DRV_ATTITUDE_FUSION_H
#define DRV_ATTITUDE_FUSION_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Estimator convention.  Each value fixes a body frame and a navigation frame
 * together; they are not interchangeable and must match the frame the caller
 * actually samples in.
 *
 *   NWU  body FLU  (+X forward, +Y left,  +Z up)
 *        nav  NWU  (North, West, Up)
 *        Canonical pairing of Driver/Inc/drv_frame_contract.h; selected by the
 *        runtime once a V0 orientation candidate is active.
 *
 *   NED  body FRD  (+X forward, +Y right, +Z down)
 *        nav  NED  (North, East, Down)
 *        Legacy fallback used only while no V0 candidate is persisted.
 */
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

/*
 * Attitude output.
 *
 * quaternion: body-to-navigation rotation, component order (w, x, y, z).
 *   Rotating a vector expressed in the body frame by this quaternion yields
 *   the same vector expressed in the navigation frame selected at
 *   initialisation.  Level and stationary is the identity quaternion.
 *
 * roll_deg / pitch_deg / yaw_deg: Euler decomposition of that same rotation.
 *   Under the NWU convention the signs are the canonical ones of
 *   drv_frame_contract.h -- positive roll is right wing down, positive pitch
 *   is nose down, positive yaw is nose left.  Verified end to end against the
 *   real estimator in tests/test_flu_seam1_estimator_frame.py.
 */
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

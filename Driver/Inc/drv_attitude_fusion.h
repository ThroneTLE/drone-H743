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

    /*
     * Magnetometer, optional. Body FLU per drv_frame_contract.h
     * (DRV_FRAME_MAGNETOMETER_UNIT_IS_MILLIGAUSS), already mounting-rotated
     * (Services/Inc/svc_mag.h) and hard-/soft-iron corrected
     * (Driver/Inc/drv_mag_calibration.h) by the caller. Units are
     * milligauss.
     *
     * magnetometer_valid must be the caller's precomputed AND of the first
     * three of the four magnetometer fusion-gate conditions: calibrated,
     * axis-verification usable for fusion (SVC_MAG_AxisUsableForFusion()),
     * and sample freshness. This driver never contacts persistence or
     * Services -- it does not know how calibration state or axis provenance
     * are stored -- and applies only the fourth, physics-based gate:
     * field-magnitude plausibility, via DRV_MAG_FieldMagnitude_InRange()
     * (Driver/Inc/drv_mag_calibration.h), the magnetometer analogue of the
     * accelerometer specific-force norm gate in drv_attitude_fusion.c.
     *
     * When magnetometer_valid is 0, or the fourth gate rejects the sample,
     * this driver takes the exact same FusionAhrsUpdateNoMagnetometer()
     * path it always has -- the default/uncalibrated/axis-unverified state
     * must produce attitude output bit-identical to a magnetometer-free
     * build.
     */
    float magnetometer_mgauss[3];
    uint8_t magnetometer_valid;
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

    /*
     * Magnetometer observability -- distinguishes "no magnetometer this
     * tick" from "magnetometer present but this sample was rejected"
     * (diagnostics must be able to tell those apart, see C4 of the
     * magnetometer-fusion task).
     *
     *   magnetometer_subsystem_enabled: mirrors input->magnetometer_valid
     *     for this update (the caller's calibrated/axis-verified/fresh
     *     judgement, gate conditions 1-3).
     *   magnetometer_field_rejected: this driver's own field-magnitude gate
     *     (condition 4) rejected the sample; only meaningful when
     *     subsystem_enabled is set.
     *   magnetometer_used: the sample was actually fed to
     *     FusionAhrsUpdate() this tick, i.e.
     *     subsystem_enabled && !field_rejected.
     *   magnetometer_ignored / magnetic_recovery / magnetic_error_deg /
     *     magnetic_recovery_trigger: the Fusion library's own internal
     *     magnetic-rejection state (mirrors the acceleration_* fields
     *     above); only meaningful when magnetometer_used is set.
     */
    uint8_t magnetometer_subsystem_enabled;
    uint8_t magnetometer_field_rejected;
    uint8_t magnetometer_used;
    uint8_t magnetometer_ignored;
    uint8_t magnetic_recovery;
    float magnetic_error_deg;
    float magnetic_recovery_trigger;
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

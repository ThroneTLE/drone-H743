#ifndef DRV_IMU_CALIBRATION_H
#define DRV_IMU_CALIBRATION_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Portable, frame-agnostic IMU metrology correction.
 *
 * The caller owns coordinate-frame selection.  Both vectors supplied here
 * must already be expressed in the same final body frame.  For this project
 * that means V0 has already produced canonical FLU before this V1 correction
 * is called.
 */
#define DRV_IMU_CAL_VALID_ACCEL      0x01U
#define DRV_IMU_CAL_VALID_GYRO       0x02U
#define DRV_IMU_CAL_VALID_GYRO_TEMP  0x04U
#define DRV_IMU_CAL_VALID_MASK       0x07U

typedef struct {
    uint8_t valid_mask;
    float accel_bias_g[3];
    float accel_correction[3][3];
    /* Residual bias after the application's per-boot stationary gyro trim. */
    float gyro_bias_ref_dps[3];
    float gyro_correction[3][3];
    float gyro_temp_slope_dps_per_c[3];
    float reference_temp_c;
} DRV_IMU_Calibration;

/*
 * Apply in place, independently for accel and gyro:
 *
 *   corrected = M * (value - bias(T))
 *
 * accel bias is temperature independent.  Gyro bias(T) is
 * gyro_bias_ref + gyro_temp_slope * (temperature-reference_temp) when the
 * GYRO_TEMP bit is valid.  GYRO_TEMP without GYRO is intentionally ignored.
 *
 * The return value is the subset of DRV_IMU_CAL_VALID_* actually applied.
 * With no valid bits this function is an exact no-op (it does not rewrite the
 * input floats).  A malformed enabled branch is ignored without disturbing a
 * separately valid branch.
 */
uint8_t DRV_IMU_Calibration_Apply(
    const DRV_IMU_Calibration *calibration,
    float temperature_c,
    float accel_g[3],
    float gyro_dps[3]);

#ifdef __cplusplus
}
#endif

#endif /* DRV_IMU_CALIBRATION_H */

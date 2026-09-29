#ifndef DRV_MAG_CALIBRATION_H
#define DRV_MAG_CALIBRATION_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Portable, frame-agnostic magnetometer hard-iron/soft-iron correction.
 *
 * This module is pure math: all state lives in the caller-owned
 * DRV_MAG_Calibration struct, there is no file-level mutable state, no I/O,
 * no RTOS call, and no HAL dependency. Form mirrors
 * Driver/Inc/drv_imu_calibration.h.
 *
 * Scope boundary: this header does not define, and must not be used to
 * infer, the chip-axis-to-FLU mounting rotation. That mapping is a separate
 * contract owned by Driver/Inc/drv_frame_contract.h (this file intentionally
 * does not include it or depend on it). axis_verified below only records
 * whether the mounting mapping owned by that other contract has been
 * physically confirmed; it does not compute or apply that mapping.
 *
 * Fitting hard_iron_bias_mgauss and soft_iron_matrix is done offline on the
 * host (ellipsoid fit against a logged rotation capture) -- exactly like the
 * six-face accelerometer calibration precedent
 * (Driver/Inc/drv_imu_calibration.h). Firmware only ever applies
 * caller-supplied coefficients; it never fits them.
 *
 *   corrected_mgauss = M * (raw_mgauss - b)
 *
 * Uncalibrated state (per the project's magnetometer-calibration contract):
 * b = {0,0,0}, M = identity, calibrated = 0. Applying that state is an exact
 * identity transform -- DRV_MAG_Calibration_Apply() does not special-case
 * calibrated == 0, the math alone reproduces the input bit-for-bit.
 */

/*
 * Axis-mounting provenance for hard_iron_bias_mgauss/soft_iron_matrix below.
 * A frame default/derivation (for example a vendor-datasheet
 * chip-to-FRD-to-FLU guess) is NOT verification: it must be recorded as
 * UNVERIFIED until a physical axis-verification procedure confirms it.
 * Consumers must treat axis_verified == DRV_MAG_CAL_AXIS_UNVERIFIED as
 * "mounting orientation unknown", never as a usable default.
 */
#define DRV_MAG_CAL_AXIS_UNVERIFIED 0U
#define DRV_MAG_CAL_AXIS_VERIFIED   1U

typedef struct {
    /*
     * 0 = uncalibrated (host has not fit hard/soft-iron coefficients yet).
     * Consumers (fusion, diagnostics) must treat calibrated == 0 identically
     * to "no magnetometer available", regardless of what
     * hard_iron_bias_mgauss/soft_iron_matrix happen to contain.
     */
    uint8_t calibrated;

    /* DRV_MAG_CAL_AXIS_UNVERIFIED or DRV_MAG_CAL_AXIS_VERIFIED; see above. */
    uint8_t axis_verified;

    /*
     * The frame contract version (DRV_FRAME_CONTRACT_VERSION, owned by
     * drv_frame_contract.h) captured at the moment axis_verified was last
     * set to DRV_MAG_CAL_AXIS_VERIFIED. The owning layer (persistence/App)
     * must invalidate axis_verified the instant the live contract version no
     * longer matches this field. This driver only carries the value and
     * never compares it against anything, because it must not depend on
     * drv_frame_contract.h.
     */
    uint32_t frame_contract_version;

    /* Hard-iron offset b, milligauss. */
    float hard_iron_bias_mgauss[3];

    /* Soft-iron correction M, dimensionless. Identity when calibrated == 0. */
    float soft_iron_matrix[3][3];
} DRV_MAG_Calibration;

typedef enum {
    DRV_MAG_CAL_VALID = 0,
    DRV_MAG_CAL_INVALID_ARGS = 1,        /* NULL pointer */
    DRV_MAG_CAL_INVALID_NONFINITE = 2,   /* NaN/Inf in bias, matrix, or input/output vector */
    DRV_MAG_CAL_INVALID_DETERMINANT = 3, /* det(M) <= 0: singular or a reflection, not a physical fit */
    DRV_MAG_CAL_INVALID_SCALE = 4,       /* det(M) outside [DRV_MAG_CAL_MATRIX_DET_MIN, DRV_MAG_CAL_MATRIX_DET_MAX] */
} DRV_MAG_CalibrationStatus;

/*
 * det(M) is a fast, basis-independent proxy for "is this a physically
 * plausible soft-iron fit". A clean sensor and a proper mounting rotation
 * keep each axis gain within roughly a factor of 2-3 of unity, so a
 * well-formed fit has det(M) close to 1. These bounds are one full order of
 * magnitude on either side of 1 and exist to reject a corrupted/garbage
 * matrix -- they are not a tight calibration-quality check, that judgment
 * belongs to the host-side fit, not to this runtime gate.
 */
#define DRV_MAG_CAL_MATRIX_DET_MIN 0.1f
#define DRV_MAG_CAL_MATRIX_DET_MAX 10.0f

/*
 * Numeric sanity of the coefficients alone (NaN/Inf, degenerate/reflected or
 * wildly-scaled soft-iron matrix). Deliberately does NOT look at calibrated
 * or axis_verified -- those are provenance/gating bits, not numeric
 * well-formedness. A corrupted-but-"calibrated" record must be reported
 * invalid here, never silently treated as identical to uncalibrated.
 */
DRV_MAG_CalibrationStatus DRV_MAG_Calibration_Validate(
    const DRV_MAG_Calibration *calibration);

/*
 * corrected_mgauss = M * (raw_mgauss - b). raw_mgauss and corrected_mgauss
 * may point at the same array (in-place use is supported).
 *
 * On any DRV_MAG_CAL_INVALID_* return, corrected_mgauss is left completely
 * untouched -- this function never falls back to copying raw_mgauss through
 * as if it were an identity transform, because that would make a broken
 * calibration record indistinguishable from a genuinely uncalibrated one.
 */
DRV_MAG_CalibrationStatus DRV_MAG_Calibration_Apply(
    const DRV_MAG_Calibration *calibration,
    const float raw_mgauss[3],
    float corrected_mgauss[3]);

/*
 * Field-strength gate for fusion admission (magnetic-interference rejection,
 * the fourth and last magnetometer fusion-gate condition; the other three --
 * calibrated, axis_verified, sample freshness -- are decided above this
 * driver, closer to the fusion call site).
 *
 * Earth's total field intensity is on the order of 250-650 mGauss at the
 * surface: roughly 250-350 mGauss near the geomagnetic equator/South
 * Atlantic Anomaly, rising to 550-650 mGauss at high geomagnetic latitude
 * (IGRF/WMM reference field tables). The bounds below sit outside that
 * nominal band by a wide margin on both sides -- about a fifth below the
 * lowest equatorial reading, and a quarter above the highest polar reading
 * -- so ordinary calibration residual, sensor noise, and worldwide operating
 * location cannot false-reject. A sample outside [MIN, MAX] is either a
 * dead/saturated sensor or magnetic interference (motor phase current,
 * ferrous airframe hardware), which is exactly what this gate exists to
 * catch before such a value is allowed to steer yaw.
 */
#define DRV_MAG_FIELD_MIN_MGAUSS 200.0f
#define DRV_MAG_FIELD_MAX_MGAUSS 800.0f

/*
 * 1 when the vector's magnitude falls within [DRV_MAG_FIELD_MIN_MGAUSS,
 * DRV_MAG_FIELD_MAX_MGAUSS]; 0 otherwise, including NULL/non-finite input
 * (fail closed -- a sample that cannot be gated must never be treated as
 * passing the gate).
 */
uint8_t DRV_MAG_FieldMagnitude_InRange(const float field_mgauss[3]);

#ifdef __cplusplus
}
#endif

#endif /* DRV_MAG_CALIBRATION_H */

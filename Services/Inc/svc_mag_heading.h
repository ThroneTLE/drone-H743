#ifndef SVC_MAG_HEADING_H
#define SVC_MAG_HEADING_H

/*
 * Near-level relative-heading aid for the on-board magnetometer.
 *
 * Input and output vectors are body FLU, in milligauss. This is deliberately
 * separate from the full 3-D hard/soft-iron calibration: it applies only an
 * XY hard-iron centre from a level yaw sweep. The state is caller-owned and
 * the module has no HAL, RTOS, I/O, or persistent configuration dependency.
 * The caller must check sensor freshness and the physical mounting-axis
 * verification before calling Update. No data from this module may be used
 * as a full 3-D calibration claim.
 *
 * The READY output is direction-only: the aligned XY rescaled to
 * SVC_MAG_HEADING_FUSION_FIELD_MGAUSS and Z = 0 (the raw Z is not trusted;
 * see the note at the output in svc_mag_heading.c).
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SVC_MAG_HEADING_Z_WINDOW 10U
#define SVC_MAG_HEADING_MAX_TILT_DEG 5.0f
/* Inside the AHRS 200..800 mG field gate; the AHRS uses direction only. */
#define SVC_MAG_HEADING_FUSION_FIELD_MGAUSS 500.0f

typedef struct {
    float bias_x_mgauss;
    float bias_y_mgauss;
    float radius_xy_mgauss; /* diagnostic provenance; XY bias alone sets heading */
    uint32_t generation;    /* changes on config edit or persisted reload */
    uint32_t frame_contract_version;
    uint8_t axis_verified;  /* separate from persistent 3-D MAGFRAME */
    uint8_t enabled;
} SVC_MAG_HeadingConfig;

typedef struct {
    uint64_t timestamp_us;  /* successful magnetic sample, not control tick */
    float raw_flu_mgauss[3];
    float roll_deg;
    float pitch_deg;
    float yaw_deg;       /* current gyro/AHRS yaw, for initial relative alignment */
} SVC_MAG_HeadingInput;

typedef enum {
    SVC_MAG_HEADING_DISABLED = 0,
    SVC_MAG_HEADING_REPEATED,
    SVC_MAG_HEADING_INVALID,
    SVC_MAG_HEADING_TILT,
    SVC_MAG_HEADING_WARMUP,
    SVC_MAG_HEADING_READY
} SVC_MAG_HeadingResult;

typedef struct {
    uint64_t last_timestamp_us;
    uint32_t generation;
    float z_window_mgauss[SVC_MAG_HEADING_Z_WINDOW];
    float last_z_mean_mgauss;
    float xy_rotation_rad; /* fixed virtual-north rotation after first READY */
    uint8_t z_count;
    uint8_t z_next;
    uint8_t heading_aligned;
    uint32_t ready_count;
    uint32_t tilt_count;
} SVC_MAG_HeadingState;

void SVC_MAG_HeadingReset(SVC_MAG_HeadingState *state);
uint8_t SVC_MAG_HeadingConfigValid(const SVC_MAG_HeadingConfig *config);

/*
 * Returns READY only once for a fresh sample after ten near-level samples.
 * On first READY, the corrected XY direction is rotated to agree with the
 * caller's current yaw: the magnetometer subsequently corrects relative
 * drift instead of pulling the vehicle toward geographic magnetic north.
 * Output is a virtual body-FLU magnetic vector with XY bias removed and Z
 * averaged over ten recent samples. When tilt exceeds
 * the 5-degree near-level budget, the Z window is cleared and must warm up
 * again. The caller still applies the existing field-strength and Fusion
 * magnetic-innovation gates before treating the observation as accepted.
 */
SVC_MAG_HeadingResult SVC_MAG_HeadingUpdate(
    const SVC_MAG_HeadingConfig *config,
    SVC_MAG_HeadingState *state,
    const SVC_MAG_HeadingInput *input,
    float corrected_flu_mgauss[3]);

#ifdef __cplusplus
}
#endif

#endif

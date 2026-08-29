#ifndef APP_FLIGHT_CALIBRATION_H
#define APP_FLIGHT_CALIBRATION_H

#include <stdint.h>

#include "drv_coax_ctrl.h"
#include "drv_imu_calibration.h"

#ifdef __cplusplus
extern "C" {
#endif

#define APP_FLIGHT_CAL_MAGIC  0x4643414CUL /* "FCAL" */
#define APP_FLIGHT_CAL_SCHEMA 1U

#define APP_FLIGHT_CAL_VALID_ORIENTATION (1U << 0)
#define APP_FLIGHT_CAL_VALID_ACCEL       (1U << 1)
#define APP_FLIGHT_CAL_VALID_GYRO        (1U << 2)
#define APP_FLIGHT_CAL_VALID_GYRO_TEMP   (1U << 3)
#define APP_FLIGHT_CAL_VALID_SERVO_MECHANICAL (1U << 4)
#define APP_FLIGHT_CAL_VALID_MASK_SUPPORTED 0x1FU
#define APP_FLIGHT_CAL_V1_VALID_MASK_SUPPORTED 0x0FU

#define APP_FLIGHT_CAL_V1_CANDIDATE_MAGIC  0x31564349UL /* "ICV1" */
#define APP_FLIGHT_CAL_V1_CANDIDATE_SCHEMA 1U
#define APP_FLIGHT_CAL_V1_UPLOAD_MAX_BYTES 128U
#define APP_FLIGHT_CAL_V1_CHUNK_MAX_BYTES  32U
#define APP_FLIGHT_CAL_V1_UPLOAD_TIMEOUT_MS 5000U

typedef struct {
    uint32_t magic;
    uint16_t schema;
    uint16_t size;
    uint16_t frame_contract;
    uint8_t orientation_code;
    uint8_t valid_mask;
    uint32_t calibration_generation;
    /* V1 parameters are expressed in canonical FLU after the V0 rotation. */
    float accel_bias[3];
    float accel_correction[3][3];
    /* Residual after the per-boot stationary gyro trim, not chip raw bias. */
    float gyro_bias_ref[3];
    float gyro_correction[3][3];
    float gyro_temp_slope[3];
    float reference_temp_c;
    /* Reserved V2 navigation/controller/RC/actuator mapping selectors. */
    uint8_t v2_navigation_mapping;
    uint8_t v2_controller_mapping;
    uint8_t v2_rc_mapping;
    uint8_t v2_actuator_mapping;
    /* Mechanical adapter after controller allocation; record stays 160 B. */
    uint16_t servo_center_us[DRV_COAX_CTRL_SERVO_COUNT];
    uint16_t servo_min_us[DRV_COAX_CTRL_SERVO_COUNT];
    uint16_t servo_max_us[DRV_COAX_CTRL_SERVO_COUNT];
    int8_t servo_pulse_sign[DRV_COAX_CTRL_SERVO_COUNT];
    uint8_t servo_reserved[2];
    uint32_t v2_reserved[3];
} APP_FlightCalibration;

/* Immutable reader view published by the text/control task. */
typedef struct {
    APP_FlightCalibration calibration;
    /* Runtime epoch; changes whenever the effective confirmed record changes. */
    uint32_t generation;
} APP_FlightCalibrationSnapshot;

/*
 * Compact wire payload for V1 only. It deliberately contains no V2 fields;
 * COMMIT merges these fields into the currently confirmed whole FCAL record.
 * The BEGIN command carries CRC32 for these exact 128 little-endian bytes.
 */
typedef struct {
    uint32_t magic;
    uint16_t schema;
    uint16_t size;
    uint32_t base_generation;
    uint16_t frame_contract;
    uint8_t orientation_code;
    uint8_t valid_mask;
    float accel_bias[3];
    float accel_correction[3][3];
    float gyro_bias_ref[3];
    float gyro_correction[3][3];
    float gyro_temp_slope[3];
    float reference_temp_c;
} APP_FlightCalibrationV1Candidate;

typedef enum {
    APP_FLIGHT_CAL_UPLOAD_EMPTY = 0U,
    APP_FLIGHT_CAL_UPLOAD_RECEIVING = 1U,
    APP_FLIGHT_CAL_UPLOAD_READY = 2U,
} APP_FlightCalibrationUploadState;

typedef enum {
    APP_FLIGHT_CAL_TRANSFER_OK = 0U,
    APP_FLIGHT_CAL_TRANSFER_BAD_STATE,
    APP_FLIGHT_CAL_TRANSFER_OVERLAP,
    APP_FLIGHT_CAL_TRANSFER_GAP,
    APP_FLIGHT_CAL_TRANSFER_OVERSIZE,
    APP_FLIGHT_CAL_TRANSFER_BAD_HEX,
    APP_FLIGHT_CAL_TRANSFER_INCOMPLETE,
    APP_FLIGHT_CAL_TRANSFER_BAD_CRC,
    APP_FLIGHT_CAL_TRANSFER_TIMEOUT,
    APP_FLIGHT_CAL_TRANSFER_BAD_MAGIC,
    APP_FLIGHT_CAL_TRANSFER_BAD_SCHEMA,
    APP_FLIGHT_CAL_TRANSFER_BAD_SIZE,
    APP_FLIGHT_CAL_TRANSFER_BAD_FRAME,
    APP_FLIGHT_CAL_TRANSFER_BAD_ORIENTATION,
    APP_FLIGHT_CAL_TRANSFER_BAD_VALID_MASK,
    APP_FLIGHT_CAL_TRANSFER_NONFINITE,
    APP_FLIGHT_CAL_TRANSFER_BAD_RANGE,
    APP_FLIGHT_CAL_TRANSFER_BAD_MATRIX,
} APP_FlightCalibrationTransferStatus;

typedef struct {
    uint8_t payload[APP_FLIGHT_CAL_V1_UPLOAD_MAX_BYTES];
    APP_FlightCalibrationV1Candidate candidate;
    uint32_t expected_size;
    uint32_t expected_crc32;
    uint32_t received_size;
    uint32_t last_activity_ms;
    APP_FlightCalibrationUploadState state;
} APP_FlightCalibrationUpload;

typedef enum {
    APP_FLIGHT_CAL_DECODE_INVALID = 0U,
    APP_FLIGHT_CAL_DECODE_CURRENT = 1U,
    APP_FLIGHT_CAL_DECODE_MIGRATED_IMUF_V1 = 2U,
} APP_FlightCalibrationDecodeStatus;

void APP_FlightCalibration_Defaults(APP_FlightCalibration *calibration);
uint8_t APP_FlightCalibration_Validate(
    const APP_FlightCalibration *calibration);
uint32_t APP_FlightCalibration_Encode(
    const APP_FlightCalibration *calibration,
    uint8_t *output,
    uint32_t capacity);
APP_FlightCalibrationDecodeStatus APP_FlightCalibration_Decode(
    const uint8_t *data,
    uint32_t size,
    APP_FlightCalibration *calibration);
uint8_t APP_FlightCalibration_UpdateOrientation(
    APP_FlightCalibration *calibration,
    uint8_t orientation_code);

/*
 * Runtime publication boundary.  ResetActive installs safe defaults
 * (valid_mask=0).  Only the Param owner may call PublishConfirmed, and only
 * after dirty=0 proves that the decoded FCAL record is Flash-confirmed.
 * Readers never receive a pointer to mutable storage; ReadActive copies a
 * coherent odd/even-seqlock snapshot.
 */
void APP_FlightCalibration_ResetActive(void);
uint8_t APP_FlightCalibration_PublishConfirmed(
    const APP_FlightCalibration *calibration);
/* RAM-only publication; always advances the runtime generation/reset seam. */
uint8_t APP_FlightCalibration_PublishPreview(
    const APP_FlightCalibration *calibration);
uint8_t APP_FlightCalibration_ReadActive(
    APP_FlightCalibrationSnapshot *snapshot);
uint8_t APP_FlightCalibration_ReadActiveGeneration(uint32_t *generation);
uint32_t APP_FlightCalibration_GetActiveGeneration(void);

/*
 * Bind a confirmed FCAL record to the exact V0 orientation used for a frame.
 * V1 coefficients are disabled if orientation provenance is absent/mismatched.
 * The returned mask remains in APP_FLIGHT_CAL_VALID_* bit positions.
 */
uint8_t APP_FlightCalibration_BuildImuCalibration(
    const APP_FlightCalibration *calibration,
    uint8_t active_orientation_code,
    DRV_IMU_Calibration *imu_calibration);

uint8_t APP_FlightCalibration_UpdateServoMechanical(
    APP_FlightCalibration *calibration,
    const DRV_COAX_CTRL_ServoCalibration *servo_calibration);
uint8_t APP_FlightCalibration_BuildServoMechanical(
    const APP_FlightCalibration *calibration,
    DRV_COAX_CTRL_ServoCalibration *servo_calibration);

uint32_t APP_FlightCalibration_Crc32(const uint8_t *data, uint32_t size);
void APP_FlightCalibration_UploadReset(APP_FlightCalibrationUpload *upload);
uint8_t APP_FlightCalibration_UploadExpire(APP_FlightCalibrationUpload *upload,
                                           uint32_t now_ms);
APP_FlightCalibrationTransferStatus APP_FlightCalibration_UploadBegin(
    APP_FlightCalibrationUpload *upload,
    uint32_t size,
    uint32_t crc32,
    uint32_t now_ms);
APP_FlightCalibrationTransferStatus APP_FlightCalibration_UploadDataHex(
    APP_FlightCalibrationUpload *upload,
    uint32_t offset,
    const char *hex,
    uint32_t now_ms);
APP_FlightCalibrationTransferStatus APP_FlightCalibration_UploadEnd(
    APP_FlightCalibrationUpload *upload,
    uint32_t now_ms);
APP_FlightCalibrationTransferStatus APP_FlightCalibration_ValidateV1Candidate(
    const APP_FlightCalibrationV1Candidate *candidate);
uint8_t APP_FlightCalibration_MergeV1Candidate(
    const APP_FlightCalibration *base,
    const APP_FlightCalibrationV1Candidate *candidate,
    APP_FlightCalibration *merged);
const char *APP_FlightCalibration_UploadStateText(
    APP_FlightCalibrationUploadState state);
const char *APP_FlightCalibration_TransferStatusText(
    APP_FlightCalibrationTransferStatus status);

#ifdef __cplusplus
}
#endif

#endif /* APP_FLIGHT_CALIBRATION_H */

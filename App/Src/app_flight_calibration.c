#include "app_flight_calibration.h"

#include "app_sensor.h"
#include "drv_frame_contract.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define APP_FLIGHT_CAL_LEGACY_IMUF_MAGIC  0x494D5546UL
#define APP_FLIGHT_CAL_LEGACY_IMUF_SCHEMA 1U
#define APP_FLIGHT_CAL_MATRIX_DIAG_MIN    0.85f
#define APP_FLIGHT_CAL_MATRIX_DIAG_MAX    1.15f
#define APP_FLIGHT_CAL_MATRIX_OFFDIAG_MAX 0.10f
#define APP_FLIGHT_CAL_MATRIX_DET_MIN     0.60f
#define APP_FLIGHT_CAL_MATRIX_DET_MAX     1.55f
#define APP_FLIGHT_CAL_MATRIX_COND_MAX    1.25f
#define APP_FLIGHT_CAL_SNAPSHOT_RETRIES   8U
#define APP_FLIGHT_CAL_ACCEL_BIAS_ABS_MAX_G       0.50f
#define APP_FLIGHT_CAL_GYRO_BIAS_ABS_MAX_DPS     20.0f
#define APP_FLIGHT_CAL_GYRO_TEMP_SLOPE_ABS_MAX   2.0f
#define APP_FLIGHT_CAL_REFERENCE_TEMP_MIN_C      (-40.0f)
#define APP_FLIGHT_CAL_REFERENCE_TEMP_MAX_C      85.0f

typedef struct {
    uint32_t magic;
    uint16_t schema;
    uint16_t base_contract;
    uint8_t orientation_code;
    uint8_t reserved[3];
} APP_FlightCalibrationLegacyImufV1;

_Static_assert(sizeof(APP_FlightCalibrationLegacyImufV1) == 12U,
               "legacy IMUF v1 decoder layout changed");
_Static_assert(sizeof(APP_FlightCalibration) == 160U,
               "FCAL schema size changed; bump schema before editing");
_Static_assert(sizeof(APP_FlightCalibrationV1Candidate) == 128U,
               "V1 candidate wire ABI must remain 128 bytes");
_Static_assert(sizeof(APP_FlightCalibrationV1Candidate) <=
                   APP_FLIGHT_CAL_V1_UPLOAD_MAX_BYTES,
               "V1 candidate exceeds bounded upload buffer");

/*
 * One control-task writer, many real-time readers.  The public API copies an
 * immutable value; no caller can retain a pointer into this storage.
 */
static volatile uint32_t app_flight_cal_active_seqlock;
static volatile uint32_t app_flight_cal_active_generation;
static volatile uint8_t app_flight_cal_active_initialized;
static volatile APP_FlightCalibration app_flight_cal_active;

static void app_flight_cal_memory_barrier(void)
{
#if defined(__GNUC__) || defined(__clang__)
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
#else
    /* Supported firmware and host-test compilers take the branch above. */
#endif
}

static void app_flight_cal_copy_from_volatile(
    APP_FlightCalibration *destination,
    const volatile APP_FlightCalibration *source)
{
    uint8_t *out = (uint8_t *)destination;
    const volatile uint8_t *in = (const volatile uint8_t *)source;
    uint32_t index;

    for (index = 0U; index < sizeof(*destination); ++index) {
        out[index] = in[index];
    }
}

static void app_flight_cal_copy_to_volatile(
    volatile APP_FlightCalibration *destination,
    const APP_FlightCalibration *source)
{
    volatile uint8_t *out = (volatile uint8_t *)destination;
    const uint8_t *in = (const uint8_t *)source;
    uint32_t index;

    for (index = 0U; index < sizeof(*source); ++index) {
        out[index] = in[index];
    }
}

static uint8_t app_flight_cal_orientation_valid(uint8_t code)
{
    return ((code < APP_SENSOR_FLU_ORIENTATION_COUNT) ||
            (code == APP_SENSOR_FLU_ORIENTATION_LEGACY)) ? 1U : 0U;
}

static float app_flight_cal_det3(const float matrix[3][3])
{
    return matrix[0][0] *
               (matrix[1][1] * matrix[2][2] -
                matrix[1][2] * matrix[2][1]) -
           matrix[0][1] *
               (matrix[1][0] * matrix[2][2] -
                matrix[1][2] * matrix[2][0]) +
           matrix[0][2] *
               (matrix[1][0] * matrix[2][1] -
                matrix[1][1] * matrix[2][0]);
}

static float app_flight_cal_norm1(const float matrix[3][3])
{
    float maximum = 0.0f;
    uint32_t column;

    for (column = 0U; column < 3U; ++column) {
        float sum = fabsf(matrix[0][column]) +
                    fabsf(matrix[1][column]) +
                    fabsf(matrix[2][column]);
        if (sum > maximum) {
            maximum = sum;
        }
    }
    return maximum;
}

static uint8_t app_flight_cal_matrix_valid(const float matrix[3][3])
{
    float adjugate[3][3];
    float determinant;
    float condition;
    uint32_t row;
    uint32_t column;

    for (row = 0U; row < 3U; ++row) {
        for (column = 0U; column < 3U; ++column) {
            float value = matrix[row][column];
            if (!isfinite(value)) {
                return 0U;
            }
            if (((row == column) &&
                 ((value < APP_FLIGHT_CAL_MATRIX_DIAG_MIN) ||
                  (value > APP_FLIGHT_CAL_MATRIX_DIAG_MAX))) ||
                ((row != column) &&
                 (fabsf(value) > APP_FLIGHT_CAL_MATRIX_OFFDIAG_MAX))) {
                return 0U;
            }
        }
    }

    determinant = app_flight_cal_det3(matrix);
    if ((!isfinite(determinant)) ||
        (determinant < APP_FLIGHT_CAL_MATRIX_DET_MIN) ||
        (determinant > APP_FLIGHT_CAL_MATRIX_DET_MAX)) {
        return 0U;
    }

    adjugate[0][0] = matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1];
    adjugate[0][1] = matrix[0][2] * matrix[2][1] - matrix[0][1] * matrix[2][2];
    adjugate[0][2] = matrix[0][1] * matrix[1][2] - matrix[0][2] * matrix[1][1];
    adjugate[1][0] = matrix[1][2] * matrix[2][0] - matrix[1][0] * matrix[2][2];
    adjugate[1][1] = matrix[0][0] * matrix[2][2] - matrix[0][2] * matrix[2][0];
    adjugate[1][2] = matrix[0][2] * matrix[1][0] - matrix[0][0] * matrix[1][2];
    adjugate[2][0] = matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0];
    adjugate[2][1] = matrix[0][1] * matrix[2][0] - matrix[0][0] * matrix[2][1];
    adjugate[2][2] = matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0];
    condition = app_flight_cal_norm1(matrix) *
                app_flight_cal_norm1(adjugate) / determinant;
    return (isfinite(condition) &&
            (condition <= APP_FLIGHT_CAL_MATRIX_COND_MAX)) ? 1U : 0U;
}

static uint8_t app_flight_cal_vector_finite(const float vector[3])
{
    return (isfinite(vector[0]) && isfinite(vector[1]) &&
            isfinite(vector[2])) ? 1U : 0U;
}

static uint8_t app_flight_cal_vector_abs_bounded(const float vector[3],
                                                  float maximum)
{
    return ((fabsf(vector[0]) <= maximum) &&
            (fabsf(vector[1]) <= maximum) &&
            (fabsf(vector[2]) <= maximum)) ? 1U : 0U;
}

static uint8_t app_flight_cal_physical_ranges_valid(
    const float accel_bias[3],
    const float gyro_bias_ref[3],
    const float gyro_temp_slope[3],
    float reference_temp_c)
{
    return ((app_flight_cal_vector_abs_bounded(
                 accel_bias, APP_FLIGHT_CAL_ACCEL_BIAS_ABS_MAX_G) != 0U) &&
            (app_flight_cal_vector_abs_bounded(
                 gyro_bias_ref, APP_FLIGHT_CAL_GYRO_BIAS_ABS_MAX_DPS) != 0U) &&
            (app_flight_cal_vector_abs_bounded(
                 gyro_temp_slope,
                 APP_FLIGHT_CAL_GYRO_TEMP_SLOPE_ABS_MAX) != 0U) &&
            (reference_temp_c >= APP_FLIGHT_CAL_REFERENCE_TEMP_MIN_C) &&
            (reference_temp_c <= APP_FLIGHT_CAL_REFERENCE_TEMP_MAX_C)) ? 1U : 0U;
}

static uint8_t app_flight_cal_matrix_finite(const float matrix[3][3])
{
    uint32_t row;
    uint32_t column;

    for (row = 0U; row < 3U; ++row) {
        for (column = 0U; column < 3U; ++column) {
            if (!isfinite(matrix[row][column])) {
                return 0U;
            }
        }
    }
    return 1U;
}

/*
 * 与 Driver/Src/drv_coax_ctrl.c 的 DRV_COAX_CTRL_GetDefault/Validate-
 * ServoCalibration 判据逐条锁步（本文件参与 host 单测编译，不能链接整个
 * 驱动，故保留本地副本）。改任何一侧判据都必须同步另一侧，并由
 * tests/test_coax_ctrl_contract.py 的 lockstep 用例强制校验。
 */
static void app_flight_cal_servo_defaults(
    DRV_COAX_CTRL_ServoCalibration *calibration)
{
    if (calibration == NULL) {
        return;
    }
    calibration->center_us[0] = DRV_COAX_CTRL_SERVO_ALPHA_CENTER_US;
    calibration->center_us[1] = DRV_COAX_CTRL_SERVO_BETA_CENTER_US;
    calibration->min_us[0] = DRV_COAX_CTRL_SERVO_ALPHA_MIN_US;
    calibration->min_us[1] = DRV_COAX_CTRL_SERVO_BETA_MIN_US;
    calibration->max_us[0] = DRV_COAX_CTRL_SERVO_ALPHA_MAX_US;
    calibration->max_us[1] = DRV_COAX_CTRL_SERVO_BETA_MAX_US;
    calibration->pulse_sign[0] = 1;
    calibration->pulse_sign[1] = 1;
}

static uint8_t app_flight_cal_servo_valid(
    const DRV_COAX_CTRL_ServoCalibration *calibration)
{
    uint32_t index;

    if (calibration == NULL) {
        return 0U;
    }
    for (index = 0U; index < DRV_COAX_CTRL_SERVO_COUNT; ++index) {
        const uint16_t center = calibration->center_us[index];
        const uint16_t minimum = calibration->min_us[index];
        const uint16_t maximum = calibration->max_us[index];
        if ((minimum < DRV_COAX_CTRL_SERVO_PHYSICAL_MIN_US) ||
            (maximum > DRV_COAX_CTRL_SERVO_PHYSICAL_MAX_US) ||
            (minimum >= center) || (center >= maximum) ||
            ((uint16_t)(center - minimum) <
             DRV_COAX_CTRL_SERVO_MIN_CAL_SPAN_US) ||
            ((uint16_t)(maximum - center) <
             DRV_COAX_CTRL_SERVO_MIN_CAL_SPAN_US) ||
            ((calibration->pulse_sign[index] != 1) &&
             (calibration->pulse_sign[index] != -1))) {
            return 0U;
        }
    }
    return 1U;
}

void APP_FlightCalibration_Defaults(APP_FlightCalibration *calibration)
{
    uint32_t axis;
    DRV_COAX_CTRL_ServoCalibration servo_defaults;

    if (calibration == NULL) {
        return;
    }
    memset(calibration, 0, sizeof(*calibration));
    calibration->magic = APP_FLIGHT_CAL_MAGIC;
    calibration->schema = APP_FLIGHT_CAL_SCHEMA;
    calibration->size = (uint16_t)sizeof(*calibration);
    calibration->frame_contract = DRV_FRAME_CONTRACT_VERSION;
    calibration->orientation_code = APP_SENSOR_FLU_ORIENTATION_LEGACY;
    calibration->reference_temp_c = 25.0f;
    for (axis = 0U; axis < 3U; ++axis) {
        calibration->accel_correction[axis][axis] = 1.0f;
        calibration->gyro_correction[axis][axis] = 1.0f;
    }
    app_flight_cal_servo_defaults(&servo_defaults);
    memcpy(calibration->servo_center_us, servo_defaults.center_us,
           sizeof(calibration->servo_center_us));
    memcpy(calibration->servo_min_us, servo_defaults.min_us,
           sizeof(calibration->servo_min_us));
    memcpy(calibration->servo_max_us, servo_defaults.max_us,
           sizeof(calibration->servo_max_us));
    memcpy(calibration->servo_pulse_sign, servo_defaults.pulse_sign,
           sizeof(calibration->servo_pulse_sign));
}

uint8_t APP_FlightCalibration_Validate(
    const APP_FlightCalibration *calibration)
{
    DRV_COAX_CTRL_ServoCalibration servo_calibration;

    if (calibration != NULL) {
        memcpy(servo_calibration.center_us, calibration->servo_center_us,
               sizeof(servo_calibration.center_us));
        memcpy(servo_calibration.min_us, calibration->servo_min_us,
               sizeof(servo_calibration.min_us));
        memcpy(servo_calibration.max_us, calibration->servo_max_us,
               sizeof(servo_calibration.max_us));
        memcpy(servo_calibration.pulse_sign, calibration->servo_pulse_sign,
               sizeof(servo_calibration.pulse_sign));
    }
    if ((calibration == NULL) ||
        (calibration->magic != APP_FLIGHT_CAL_MAGIC) ||
        (calibration->schema != APP_FLIGHT_CAL_SCHEMA) ||
        (calibration->size != sizeof(*calibration)) ||
        (calibration->frame_contract != DRV_FRAME_CONTRACT_VERSION) ||
        (app_flight_cal_orientation_valid(calibration->orientation_code) == 0U) ||
        ((calibration->valid_mask &
          (uint8_t)~APP_FLIGHT_CAL_VALID_MASK_SUPPORTED) != 0U) ||
        (app_flight_cal_vector_finite(calibration->accel_bias) == 0U) ||
        (app_flight_cal_vector_finite(calibration->gyro_bias_ref) == 0U) ||
        (app_flight_cal_vector_finite(calibration->gyro_temp_slope) == 0U) ||
        (!isfinite(calibration->reference_temp_c)) ||
        (app_flight_cal_physical_ranges_valid(
             calibration->accel_bias,
             calibration->gyro_bias_ref,
             calibration->gyro_temp_slope,
             calibration->reference_temp_c) == 0U) ||
        (app_flight_cal_matrix_valid(calibration->accel_correction) == 0U) ||
        (app_flight_cal_matrix_valid(calibration->gyro_correction) == 0U) ||
        (((calibration->valid_mask &
           APP_FLIGHT_CAL_VALID_SERVO_MECHANICAL) != 0U) &&
         (app_flight_cal_servo_valid(&servo_calibration) == 0U)) ||
        (((calibration->valid_mask &
           APP_FLIGHT_CAL_VALID_SERVO_TYPE) != 0U) &&
         !APP_SERVO_TYPE_IS_VALID(
             (APP_ServoType)calibration->v2_actuator_mapping))) {
        return 0U;
    }
    return 1U;
}

uint32_t APP_FlightCalibration_Encode(
    const APP_FlightCalibration *calibration,
    uint8_t *output,
    uint32_t capacity)
{
    if ((output == NULL) || (capacity < sizeof(*calibration)) ||
        (APP_FlightCalibration_Validate(calibration) == 0U)) {
        return 0U;
    }
    memcpy(output, calibration, sizeof(*calibration));
    return (uint32_t)sizeof(*calibration);
}

APP_FlightCalibrationDecodeStatus APP_FlightCalibration_Decode(
    const uint8_t *data,
    uint32_t size,
    APP_FlightCalibration *calibration)
{
    APP_FlightCalibrationLegacyImufV1 legacy;

    if ((data == NULL) || (calibration == NULL)) {
        return APP_FLIGHT_CAL_DECODE_INVALID;
    }
    if (size == sizeof(*calibration)) {
        memcpy(calibration, data, sizeof(*calibration));
        return (APP_FlightCalibration_Validate(calibration) != 0U) ?
               APP_FLIGHT_CAL_DECODE_CURRENT :
               APP_FLIGHT_CAL_DECODE_INVALID;
    }
    if (size != sizeof(legacy)) {
        return APP_FLIGHT_CAL_DECODE_INVALID;
    }

    memcpy(&legacy, data, sizeof(legacy));
    if ((legacy.magic != APP_FLIGHT_CAL_LEGACY_IMUF_MAGIC) ||
        (legacy.schema != APP_FLIGHT_CAL_LEGACY_IMUF_SCHEMA) ||
        (legacy.base_contract != DRV_FRAME_CONTRACT_VERSION) ||
        (app_flight_cal_orientation_valid(legacy.orientation_code) == 0U)) {
        return APP_FLIGHT_CAL_DECODE_INVALID;
    }

    APP_FlightCalibration_Defaults(calibration);
    calibration->orientation_code = legacy.orientation_code;
    if (legacy.orientation_code < APP_SENSOR_FLU_ORIENTATION_COUNT) {
        calibration->valid_mask |= APP_FLIGHT_CAL_VALID_ORIENTATION;
    }
    calibration->calibration_generation = 1U;
    return APP_FLIGHT_CAL_DECODE_MIGRATED_IMUF_V1;
}

uint8_t APP_FlightCalibration_UpdateOrientation(
    APP_FlightCalibration *calibration,
    uint8_t orientation_code)
{
    if ((APP_FlightCalibration_Validate(calibration) == 0U) ||
        (app_flight_cal_orientation_valid(orientation_code) == 0U)) {
        return 0U;
    }
    if (calibration->orientation_code != orientation_code) {
        calibration->valid_mask &=
            (uint8_t)~(APP_FLIGHT_CAL_VALID_ACCEL |
                       APP_FLIGHT_CAL_VALID_GYRO |
                       APP_FLIGHT_CAL_VALID_GYRO_TEMP);
    }
    calibration->orientation_code = orientation_code;
    if (orientation_code < APP_SENSOR_FLU_ORIENTATION_COUNT) {
        calibration->valid_mask |= APP_FLIGHT_CAL_VALID_ORIENTATION;
    } else {
        calibration->valid_mask &= (uint8_t)~APP_FLIGHT_CAL_VALID_ORIENTATION;
    }
    ++calibration->calibration_generation;
    if (calibration->calibration_generation == 0U) {
        calibration->calibration_generation = 1U;
    }
    return 1U;
}

void APP_FlightCalibration_ResetActive(void)
{
    APP_FlightCalibration defaults;

    APP_FlightCalibration_Defaults(&defaults);
    app_flight_cal_active_seqlock++;
    app_flight_cal_memory_barrier();
    app_flight_cal_copy_to_volatile(&app_flight_cal_active, &defaults);
    app_flight_cal_active_generation = 0U;
    app_flight_cal_active_initialized = 1U;
    app_flight_cal_memory_barrier();
    app_flight_cal_active_seqlock++;
}

uint8_t APP_FlightCalibration_ReadActive(
    APP_FlightCalibrationSnapshot *snapshot)
{
    uint32_t before;
    uint32_t after;
    uint32_t attempt;

    if (snapshot == NULL) {
        return 0U;
    }
    if (app_flight_cal_active_initialized == 0U) {
        APP_FlightCalibration_Defaults(&snapshot->calibration);
        snapshot->generation = 0U;
        return 1U;
    }

    for (attempt = 0U; attempt < APP_FLIGHT_CAL_SNAPSHOT_RETRIES; ++attempt) {
        before = app_flight_cal_active_seqlock;
        if ((before & 1U) != 0U) {
            continue;
        }
        app_flight_cal_memory_barrier();
        app_flight_cal_copy_from_volatile(&snapshot->calibration,
                                          &app_flight_cal_active);
        snapshot->generation = app_flight_cal_active_generation;
        app_flight_cal_memory_barrier();
        after = app_flight_cal_active_seqlock;
        if ((before == after) && ((after & 1U) == 0U)) {
            return 1U;
        }
    }
    return 0U;
}

static uint8_t app_flight_cal_publish(
    const APP_FlightCalibration *calibration,
    uint8_t force_generation_change)
{
    APP_FlightCalibrationSnapshot current;
    APP_FlightCalibration defaults;
    uint32_t next_generation;
    uint8_t initialized;

    if (APP_FlightCalibration_Validate(calibration) == 0U) {
        return 0U;
    }

    initialized = app_flight_cal_active_initialized;
    if (APP_FlightCalibration_ReadActive(&current) == 0U) {
        return 0U;
    }
    if ((force_generation_change == 0U) &&
        (initialized != 0U) &&
        (memcmp(&current.calibration, calibration,
                sizeof(*calibration)) == 0)) {
        return 1U;
    }

    next_generation = calibration->calibration_generation;
    if (initialized == 0U) {
        APP_FlightCalibration_Defaults(&defaults);
        if ((next_generation == 0U) &&
            (memcmp(&defaults, calibration, sizeof(defaults)) != 0)) {
            next_generation = 1U;
        }
    } else if (next_generation <= current.generation) {
        next_generation = current.generation + 1U;
        if (next_generation == 0U) {
            next_generation = 1U;
        }
    }

    app_flight_cal_active_seqlock++;
    app_flight_cal_memory_barrier();
    app_flight_cal_copy_to_volatile(&app_flight_cal_active, calibration);
    app_flight_cal_active_generation = next_generation;
    app_flight_cal_active_initialized = 1U;
    app_flight_cal_memory_barrier();
    app_flight_cal_active_seqlock++;
    return 1U;
}

uint8_t APP_FlightCalibration_PublishConfirmed(
    const APP_FlightCalibration *calibration)
{
    return app_flight_cal_publish(calibration, 0U);
}

uint8_t APP_FlightCalibration_PublishPreview(
    const APP_FlightCalibration *calibration)
{
    return app_flight_cal_publish(calibration, 1U);
}

uint8_t APP_FlightCalibration_ReadActiveGeneration(uint32_t *generation)
{
    uint32_t before;
    uint32_t after;
    uint32_t current_generation;
    uint32_t attempt;

    if (generation == NULL) {
        return 0U;
    }
    if (app_flight_cal_active_initialized == 0U) {
        *generation = 0U;
        return 1U;
    }
    for (attempt = 0U; attempt < APP_FLIGHT_CAL_SNAPSHOT_RETRIES; ++attempt) {
        before = app_flight_cal_active_seqlock;
        if ((before & 1U) != 0U) {
            continue;
        }
        app_flight_cal_memory_barrier();
        current_generation = app_flight_cal_active_generation;
        app_flight_cal_memory_barrier();
        after = app_flight_cal_active_seqlock;
        if ((before == after) && ((after & 1U) == 0U)) {
            *generation = current_generation;
            return 1U;
        }
    }
    return 0U;
}

uint32_t APP_FlightCalibration_GetActiveGeneration(void)
{
    uint32_t generation = 0U;

    (void)APP_FlightCalibration_ReadActiveGeneration(&generation);
    return generation;
}

uint8_t APP_FlightCalibration_BuildImuCalibration(
    const APP_FlightCalibration *calibration,
    uint8_t active_orientation_code,
    DRV_IMU_Calibration *imu_calibration)
{
    uint8_t effective_mask;

    if (imu_calibration == NULL) {
        return 0U;
    }
    memset(imu_calibration, 0, sizeof(*imu_calibration));
    if ((APP_FlightCalibration_Validate(calibration) == 0U) ||
        ((calibration->valid_mask & APP_FLIGHT_CAL_VALID_ORIENTATION) == 0U) ||
        (calibration->orientation_code != active_orientation_code)) {
        return 0U;
    }

    effective_mask = calibration->valid_mask;
    if ((effective_mask & APP_FLIGHT_CAL_VALID_GYRO) == 0U) {
        effective_mask &= (uint8_t)~APP_FLIGHT_CAL_VALID_GYRO_TEMP;
    }
    if ((effective_mask & APP_FLIGHT_CAL_VALID_ACCEL) != 0U) {
        imu_calibration->valid_mask |= DRV_IMU_CAL_VALID_ACCEL;
    }
    if ((effective_mask & APP_FLIGHT_CAL_VALID_GYRO) != 0U) {
        imu_calibration->valid_mask |= DRV_IMU_CAL_VALID_GYRO;
    }
    if ((effective_mask & APP_FLIGHT_CAL_VALID_GYRO_TEMP) != 0U) {
        imu_calibration->valid_mask |= DRV_IMU_CAL_VALID_GYRO_TEMP;
    }

    memcpy(imu_calibration->accel_bias_g,
           calibration->accel_bias,
           sizeof(imu_calibration->accel_bias_g));
    memcpy(imu_calibration->accel_correction,
           calibration->accel_correction,
           sizeof(imu_calibration->accel_correction));
    memcpy(imu_calibration->gyro_bias_ref_dps,
           calibration->gyro_bias_ref,
           sizeof(imu_calibration->gyro_bias_ref_dps));
    memcpy(imu_calibration->gyro_correction,
           calibration->gyro_correction,
           sizeof(imu_calibration->gyro_correction));
    memcpy(imu_calibration->gyro_temp_slope_dps_per_c,
           calibration->gyro_temp_slope,
           sizeof(imu_calibration->gyro_temp_slope_dps_per_c));
    imu_calibration->reference_temp_c = calibration->reference_temp_c;
    return effective_mask;
}

uint8_t APP_FlightCalibration_UpdateServoMechanical(
    APP_FlightCalibration *calibration,
    const DRV_COAX_CTRL_ServoCalibration *servo_calibration)
{
    if ((APP_FlightCalibration_Validate(calibration) == 0U) ||
        (app_flight_cal_servo_valid(servo_calibration) == 0U)) {
        return 0U;
    }
    memcpy(calibration->servo_center_us, servo_calibration->center_us,
           sizeof(calibration->servo_center_us));
    memcpy(calibration->servo_min_us, servo_calibration->min_us,
           sizeof(calibration->servo_min_us));
    memcpy(calibration->servo_max_us, servo_calibration->max_us,
           sizeof(calibration->servo_max_us));
    memcpy(calibration->servo_pulse_sign, servo_calibration->pulse_sign,
           sizeof(calibration->servo_pulse_sign));
    calibration->valid_mask |= APP_FLIGHT_CAL_VALID_SERVO_MECHANICAL;
    ++calibration->calibration_generation;
    if (calibration->calibration_generation == 0U) {
        calibration->calibration_generation = 1U;
    }
    return APP_FlightCalibration_Validate(calibration);
}

uint8_t APP_FlightCalibration_BuildServoMechanical(
    const APP_FlightCalibration *calibration,
    DRV_COAX_CTRL_ServoCalibration *servo_calibration)
{
    if (servo_calibration == NULL) {
        return 0U;
    }
    app_flight_cal_servo_defaults(servo_calibration);
    if ((APP_FlightCalibration_Validate(calibration) == 0U) ||
        ((calibration->valid_mask &
          APP_FLIGHT_CAL_VALID_SERVO_MECHANICAL) == 0U)) {
        return 0U;
    }
    memcpy(servo_calibration->center_us, calibration->servo_center_us,
           sizeof(servo_calibration->center_us));
    memcpy(servo_calibration->min_us, calibration->servo_min_us,
           sizeof(servo_calibration->min_us));
    memcpy(servo_calibration->max_us, calibration->servo_max_us,
           sizeof(servo_calibration->max_us));
    memcpy(servo_calibration->pulse_sign, calibration->servo_pulse_sign,
           sizeof(servo_calibration->pulse_sign));
    return app_flight_cal_servo_valid(servo_calibration);
}

uint8_t APP_FlightCalibration_UpdateServoType(
    APP_FlightCalibration *calibration,
    APP_ServoType servo_type)
{
    if ((APP_FlightCalibration_Validate(calibration) == 0U) ||
        !APP_SERVO_TYPE_IS_VALID(servo_type)) {
        return 0U;
    }
    calibration->v2_actuator_mapping = (uint8_t)servo_type;
    calibration->valid_mask |= APP_FLIGHT_CAL_VALID_SERVO_TYPE;
    ++calibration->calibration_generation;
    if (calibration->calibration_generation == 0U) {
        calibration->calibration_generation = 1U;
    }
    return APP_FlightCalibration_Validate(calibration);
}

uint8_t APP_FlightCalibration_BuildServoType(
    const APP_FlightCalibration *calibration,
    APP_ServoType *servo_type)
{
    if (servo_type == NULL) {
        return 0U;
    }
    *servo_type = APP_SERVO_TYPE_BUS;
    if ((APP_FlightCalibration_Validate(calibration) == 0U) ||
        ((calibration->valid_mask & APP_FLIGHT_CAL_VALID_SERVO_TYPE) == 0U)) {
        return 0U;
    }
    *servo_type = (APP_ServoType)calibration->v2_actuator_mapping;
    return 1U;
}

uint32_t APP_FlightCalibration_Crc32(const uint8_t *data, uint32_t size)
{
    uint32_t crc = 0xFFFFFFFFUL;
    uint32_t index;
    uint32_t bit;

    if ((data == NULL) && (size != 0U)) {
        return 0U;
    }
    for (index = 0U; index < size; ++index) {
        crc ^= (uint32_t)data[index];
        for (bit = 0U; bit < 8U; ++bit) {
            crc = ((crc & 1UL) != 0UL) ?
                  ((crc >> 1) ^ 0xEDB88320UL) : (crc >> 1);
        }
    }
    return ~crc;
}

void APP_FlightCalibration_UploadReset(APP_FlightCalibrationUpload *upload)
{
    if (upload != NULL) {
        memset(upload, 0, sizeof(*upload));
    }
}

uint8_t APP_FlightCalibration_UploadExpire(APP_FlightCalibrationUpload *upload,
                                           uint32_t now_ms)
{
    if ((upload == NULL) || (upload->state == APP_FLIGHT_CAL_UPLOAD_EMPTY)) {
        return 0U;
    }
    if ((uint32_t)(now_ms - upload->last_activity_ms) <
        APP_FLIGHT_CAL_V1_UPLOAD_TIMEOUT_MS) {
        return 0U;
    }
    APP_FlightCalibration_UploadReset(upload);
    return 1U;
}

static APP_FlightCalibrationTransferStatus app_flight_cal_upload_fail(
    APP_FlightCalibrationUpload *upload,
    APP_FlightCalibrationTransferStatus status)
{
    APP_FlightCalibration_UploadReset(upload);
    return status;
}

APP_FlightCalibrationTransferStatus APP_FlightCalibration_UploadBegin(
    APP_FlightCalibrationUpload *upload,
    uint32_t size,
    uint32_t crc32,
    uint32_t now_ms)
{
    if (upload == NULL) {
        return APP_FLIGHT_CAL_TRANSFER_BAD_STATE;
    }
    if (upload->state == APP_FLIGHT_CAL_UPLOAD_RECEIVING) {
        return app_flight_cal_upload_fail(
            upload, APP_FLIGHT_CAL_TRANSFER_OVERLAP);
    }
    if (size > APP_FLIGHT_CAL_V1_UPLOAD_MAX_BYTES) {
        return app_flight_cal_upload_fail(
            upload, APP_FLIGHT_CAL_TRANSFER_OVERSIZE);
    }
    if (size != sizeof(APP_FlightCalibrationV1Candidate)) {
        return app_flight_cal_upload_fail(
            upload, APP_FLIGHT_CAL_TRANSFER_BAD_SIZE);
    }

    APP_FlightCalibration_UploadReset(upload);
    upload->expected_size = size;
    upload->expected_crc32 = crc32;
    upload->last_activity_ms = now_ms;
    upload->state = APP_FLIGHT_CAL_UPLOAD_RECEIVING;
    return APP_FLIGHT_CAL_TRANSFER_OK;
}

static int8_t app_flight_cal_hex_nibble(char character)
{
    if ((character >= '0') && (character <= '9')) {
        return (int8_t)(character - '0');
    }
    if ((character >= 'a') && (character <= 'f')) {
        return (int8_t)(character - 'a' + 10);
    }
    if ((character >= 'A') && (character <= 'F')) {
        return (int8_t)(character - 'A' + 10);
    }
    return -1;
}

APP_FlightCalibrationTransferStatus APP_FlightCalibration_UploadDataHex(
    APP_FlightCalibrationUpload *upload,
    uint32_t offset,
    const char *hex,
    uint32_t now_ms)
{
    size_t hex_size;
    uint32_t byte_count;
    uint32_t index;

    if ((upload == NULL) ||
        (upload->state != APP_FLIGHT_CAL_UPLOAD_RECEIVING)) {
        return APP_FLIGHT_CAL_TRANSFER_BAD_STATE;
    }
    if (APP_FlightCalibration_UploadExpire(upload, now_ms) != 0U) {
        return APP_FLIGHT_CAL_TRANSFER_TIMEOUT;
    }
    if (offset < upload->received_size) {
        return app_flight_cal_upload_fail(
            upload, APP_FLIGHT_CAL_TRANSFER_OVERLAP);
    }
    if (offset > upload->received_size) {
        return app_flight_cal_upload_fail(
            upload, APP_FLIGHT_CAL_TRANSFER_GAP);
    }
    if (hex == NULL) {
        return app_flight_cal_upload_fail(
            upload, APP_FLIGHT_CAL_TRANSFER_BAD_HEX);
    }
    hex_size = strlen(hex);
    if ((hex_size == 0U) || ((hex_size & 1U) != 0U)) {
        return app_flight_cal_upload_fail(
            upload, APP_FLIGHT_CAL_TRANSFER_BAD_HEX);
    }
    byte_count = (uint32_t)(hex_size / 2U);
    if ((byte_count > APP_FLIGHT_CAL_V1_CHUNK_MAX_BYTES) ||
        ((upload->received_size + byte_count) > upload->expected_size)) {
        return app_flight_cal_upload_fail(
            upload, APP_FLIGHT_CAL_TRANSFER_OVERSIZE);
    }

    for (index = 0U; index < byte_count; ++index) {
        int8_t high = app_flight_cal_hex_nibble(hex[index * 2U]);
        int8_t low = app_flight_cal_hex_nibble(hex[index * 2U + 1U]);
        if ((high < 0) || (low < 0)) {
            return app_flight_cal_upload_fail(
                upload, APP_FLIGHT_CAL_TRANSFER_BAD_HEX);
        }
        upload->payload[upload->received_size + index] =
            (uint8_t)(((uint8_t)high << 4U) | (uint8_t)low);
    }
    upload->received_size += byte_count;
    upload->last_activity_ms = now_ms;
    return APP_FLIGHT_CAL_TRANSFER_OK;
}

APP_FlightCalibrationTransferStatus APP_FlightCalibration_ValidateV1Candidate(
    const APP_FlightCalibrationV1Candidate *candidate)
{
    uint8_t v1_valid;

    if (candidate == NULL) {
        return APP_FLIGHT_CAL_TRANSFER_BAD_STATE;
    }
    if (candidate->magic != APP_FLIGHT_CAL_V1_CANDIDATE_MAGIC) {
        return APP_FLIGHT_CAL_TRANSFER_BAD_MAGIC;
    }
    if (candidate->schema != APP_FLIGHT_CAL_V1_CANDIDATE_SCHEMA) {
        return APP_FLIGHT_CAL_TRANSFER_BAD_SCHEMA;
    }
    if (candidate->size != sizeof(*candidate)) {
        return APP_FLIGHT_CAL_TRANSFER_BAD_SIZE;
    }
    if (candidate->frame_contract != DRV_FRAME_CONTRACT_VERSION) {
        return APP_FLIGHT_CAL_TRANSFER_BAD_FRAME;
    }
    if (candidate->orientation_code >= APP_SENSOR_FLU_ORIENTATION_COUNT) {
        return APP_FLIGHT_CAL_TRANSFER_BAD_ORIENTATION;
    }
    if (((candidate->valid_mask &
          (uint8_t)~APP_FLIGHT_CAL_V1_VALID_MASK_SUPPORTED) != 0U) ||
        ((candidate->valid_mask & APP_FLIGHT_CAL_VALID_ORIENTATION) == 0U)) {
        return APP_FLIGHT_CAL_TRANSFER_BAD_VALID_MASK;
    }
    v1_valid = candidate->valid_mask &
               (APP_FLIGHT_CAL_VALID_ACCEL |
                APP_FLIGHT_CAL_VALID_GYRO |
                APP_FLIGHT_CAL_VALID_GYRO_TEMP);
    if ((v1_valid == 0U) ||
        (((v1_valid & APP_FLIGHT_CAL_VALID_GYRO_TEMP) != 0U) &&
         ((v1_valid & APP_FLIGHT_CAL_VALID_GYRO) == 0U))) {
        return APP_FLIGHT_CAL_TRANSFER_BAD_VALID_MASK;
    }
    if ((app_flight_cal_vector_finite(candidate->accel_bias) == 0U) ||
        (app_flight_cal_vector_finite(candidate->gyro_bias_ref) == 0U) ||
        (app_flight_cal_vector_finite(candidate->gyro_temp_slope) == 0U) ||
        (app_flight_cal_matrix_finite(candidate->accel_correction) == 0U) ||
        (app_flight_cal_matrix_finite(candidate->gyro_correction) == 0U) ||
        (!isfinite(candidate->reference_temp_c))) {
        return APP_FLIGHT_CAL_TRANSFER_NONFINITE;
    }
    if ((app_flight_cal_matrix_valid(candidate->accel_correction) == 0U) ||
        (app_flight_cal_matrix_valid(candidate->gyro_correction) == 0U)) {
        return APP_FLIGHT_CAL_TRANSFER_BAD_MATRIX;
    }
    if (app_flight_cal_physical_ranges_valid(
            candidate->accel_bias,
            candidate->gyro_bias_ref,
            candidate->gyro_temp_slope,
            candidate->reference_temp_c) == 0U) {
        return APP_FLIGHT_CAL_TRANSFER_BAD_RANGE;
    }
    return APP_FLIGHT_CAL_TRANSFER_OK;
}

APP_FlightCalibrationTransferStatus APP_FlightCalibration_UploadEnd(
    APP_FlightCalibrationUpload *upload,
    uint32_t now_ms)
{
    APP_FlightCalibrationTransferStatus validation;

    if ((upload == NULL) ||
        (upload->state != APP_FLIGHT_CAL_UPLOAD_RECEIVING)) {
        return APP_FLIGHT_CAL_TRANSFER_BAD_STATE;
    }
    if (APP_FlightCalibration_UploadExpire(upload, now_ms) != 0U) {
        return APP_FLIGHT_CAL_TRANSFER_TIMEOUT;
    }
    if (upload->received_size != upload->expected_size) {
        return app_flight_cal_upload_fail(
            upload, APP_FLIGHT_CAL_TRANSFER_INCOMPLETE);
    }
    if (APP_FlightCalibration_Crc32(upload->payload,
                                    upload->expected_size) !=
        upload->expected_crc32) {
        return app_flight_cal_upload_fail(
            upload, APP_FLIGHT_CAL_TRANSFER_BAD_CRC);
    }

    memcpy(&upload->candidate, upload->payload, sizeof(upload->candidate));
    validation = APP_FlightCalibration_ValidateV1Candidate(
        &upload->candidate);
    if (validation != APP_FLIGHT_CAL_TRANSFER_OK) {
        return app_flight_cal_upload_fail(upload, validation);
    }
    upload->last_activity_ms = now_ms;
    upload->state = APP_FLIGHT_CAL_UPLOAD_READY;
    return APP_FLIGHT_CAL_TRANSFER_OK;
}

uint8_t APP_FlightCalibration_MergeV1Candidate(
    const APP_FlightCalibration *base,
    const APP_FlightCalibrationV1Candidate *candidate,
    APP_FlightCalibration *merged)
{
    uint32_t next_generation;

    if ((merged == NULL) ||
        (APP_FlightCalibration_Validate(base) == 0U) ||
        (APP_FlightCalibration_ValidateV1Candidate(candidate) !=
         APP_FLIGHT_CAL_TRANSFER_OK) ||
        ((base->valid_mask & APP_FLIGHT_CAL_VALID_ORIENTATION) == 0U) ||
        (base->orientation_code != candidate->orientation_code)) {
        return 0U;
    }

    *merged = *base;
    merged->valid_mask &=
        (uint8_t)~(APP_FLIGHT_CAL_VALID_ACCEL |
                   APP_FLIGHT_CAL_VALID_GYRO |
                   APP_FLIGHT_CAL_VALID_GYRO_TEMP);
    merged->valid_mask |= candidate->valid_mask &
        (APP_FLIGHT_CAL_VALID_ACCEL |
         APP_FLIGHT_CAL_VALID_GYRO |
         APP_FLIGHT_CAL_VALID_GYRO_TEMP);
    memcpy(merged->accel_bias, candidate->accel_bias,
           sizeof(merged->accel_bias));
    memcpy(merged->accel_correction, candidate->accel_correction,
           sizeof(merged->accel_correction));
    memcpy(merged->gyro_bias_ref, candidate->gyro_bias_ref,
           sizeof(merged->gyro_bias_ref));
    memcpy(merged->gyro_correction, candidate->gyro_correction,
           sizeof(merged->gyro_correction));
    memcpy(merged->gyro_temp_slope, candidate->gyro_temp_slope,
           sizeof(merged->gyro_temp_slope));
    merged->reference_temp_c = candidate->reference_temp_c;
    next_generation = candidate->base_generation + 1U;
    merged->calibration_generation =
        (next_generation != 0U) ? next_generation : 1U;
    return APP_FlightCalibration_Validate(merged);
}

const char *APP_FlightCalibration_UploadStateText(
    APP_FlightCalibrationUploadState state)
{
    switch (state) {
    case APP_FLIGHT_CAL_UPLOAD_RECEIVING: return "receiving";
    case APP_FLIGHT_CAL_UPLOAD_READY:     return "ready";
    case APP_FLIGHT_CAL_UPLOAD_EMPTY:
    default:                              return "empty";
    }
}

const char *APP_FlightCalibration_TransferStatusText(
    APP_FlightCalibrationTransferStatus status)
{
    switch (status) {
    case APP_FLIGHT_CAL_TRANSFER_OK:              return "ok";
    case APP_FLIGHT_CAL_TRANSFER_BAD_STATE:       return "bad_state";
    case APP_FLIGHT_CAL_TRANSFER_OVERLAP:         return "overlap";
    case APP_FLIGHT_CAL_TRANSFER_GAP:             return "gap";
    case APP_FLIGHT_CAL_TRANSFER_OVERSIZE:        return "oversize";
    case APP_FLIGHT_CAL_TRANSFER_BAD_HEX:         return "bad_hex";
    case APP_FLIGHT_CAL_TRANSFER_INCOMPLETE:      return "incomplete";
    case APP_FLIGHT_CAL_TRANSFER_BAD_CRC:         return "bad_crc";
    case APP_FLIGHT_CAL_TRANSFER_TIMEOUT:         return "timeout";
    case APP_FLIGHT_CAL_TRANSFER_BAD_MAGIC:       return "bad_magic";
    case APP_FLIGHT_CAL_TRANSFER_BAD_SCHEMA:      return "bad_schema";
    case APP_FLIGHT_CAL_TRANSFER_BAD_SIZE:        return "bad_size";
    case APP_FLIGHT_CAL_TRANSFER_BAD_FRAME:       return "bad_frame";
    case APP_FLIGHT_CAL_TRANSFER_BAD_ORIENTATION: return "bad_orientation";
    case APP_FLIGHT_CAL_TRANSFER_BAD_VALID_MASK:  return "bad_valid_mask";
    case APP_FLIGHT_CAL_TRANSFER_NONFINITE:       return "nonfinite";
    case APP_FLIGHT_CAL_TRANSFER_BAD_RANGE:       return "bad_range";
    case APP_FLIGHT_CAL_TRANSFER_BAD_MATRIX:      return "bad_matrix";
    default:                                      return "invalid";
    }
}

#include "drv_imu_calibration.h"

#include <math.h>
#include <stddef.h>

static uint8_t drv_imu_cal_vector_finite(const float vector[3])
{
    return ((vector != NULL) &&
            isfinite(vector[0]) &&
            isfinite(vector[1]) &&
            isfinite(vector[2])) ? 1U : 0U;
}

static uint8_t drv_imu_cal_matrix_finite(const float matrix[3][3])
{
    uint32_t row;
    uint32_t column;

    if (matrix == NULL) {
        return 0U;
    }
    for (row = 0U; row < 3U; ++row) {
        for (column = 0U; column < 3U; ++column) {
            if (!isfinite(matrix[row][column])) {
                return 0U;
            }
        }
    }
    return 1U;
}

static uint8_t drv_imu_cal_correct_vector(
    const float matrix[3][3],
    const float value[3],
    const float bias[3],
    float corrected[3])
{
    float centered[3];
    uint32_t row;

    if ((drv_imu_cal_matrix_finite(matrix) == 0U) ||
        (drv_imu_cal_vector_finite(value) == 0U) ||
        (drv_imu_cal_vector_finite(bias) == 0U) ||
        (corrected == NULL)) {
        return 0U;
    }

    centered[0] = value[0] - bias[0];
    centered[1] = value[1] - bias[1];
    centered[2] = value[2] - bias[2];
    for (row = 0U; row < 3U; ++row) {
        corrected[row] = matrix[row][0] * centered[0] +
                         matrix[row][1] * centered[1] +
                         matrix[row][2] * centered[2];
        if (!isfinite(corrected[row])) {
            return 0U;
        }
    }
    return 1U;
}

uint8_t DRV_IMU_Calibration_Apply(
    const DRV_IMU_Calibration *calibration,
    float temperature_c,
    float accel_g[3],
    float gyro_dps[3])
{
    float corrected[3];
    float gyro_bias[3];
    uint8_t applied = 0U;
    uint8_t enabled;
    uint32_t axis;

    if (calibration == NULL) {
        return 0U;
    }
    enabled = calibration->valid_mask & DRV_IMU_CAL_VALID_MASK;
    if (enabled == 0U) {
        /* Deliberately do not even assign the input values back to themselves. */
        return 0U;
    }

    if (((enabled & DRV_IMU_CAL_VALID_ACCEL) != 0U) &&
        (drv_imu_cal_correct_vector(calibration->accel_correction,
                                    accel_g,
                                    calibration->accel_bias_g,
                                    corrected) != 0U)) {
        accel_g[0] = corrected[0];
        accel_g[1] = corrected[1];
        accel_g[2] = corrected[2];
        applied |= DRV_IMU_CAL_VALID_ACCEL;
    }

    if ((enabled & DRV_IMU_CAL_VALID_GYRO) != 0U) {
        uint8_t gyro_bias_valid =
            drv_imu_cal_vector_finite(calibration->gyro_bias_ref_dps);

        if (gyro_bias_valid != 0U) {
            gyro_bias[0] = calibration->gyro_bias_ref_dps[0];
            gyro_bias[1] = calibration->gyro_bias_ref_dps[1];
            gyro_bias[2] = calibration->gyro_bias_ref_dps[2];
        }
        if ((gyro_bias_valid != 0U) &&
            ((enabled & DRV_IMU_CAL_VALID_GYRO_TEMP) != 0U)) {
            if ((!isfinite(temperature_c)) ||
                (!isfinite(calibration->reference_temp_c)) ||
                (drv_imu_cal_vector_finite(
                     calibration->gyro_temp_slope_dps_per_c) == 0U)) {
                gyro_bias_valid = 0U;
            } else {
                const float delta_temp =
                    temperature_c - calibration->reference_temp_c;
                for (axis = 0U; axis < 3U; ++axis) {
                    gyro_bias[axis] +=
                        calibration->gyro_temp_slope_dps_per_c[axis] *
                        delta_temp;
                }
            }
        }

        if ((gyro_bias_valid != 0U) &&
            (drv_imu_cal_correct_vector(calibration->gyro_correction,
                                        gyro_dps,
                                        gyro_bias,
                                        corrected) != 0U)) {
            gyro_dps[0] = corrected[0];
            gyro_dps[1] = corrected[1];
            gyro_dps[2] = corrected[2];
            applied |= DRV_IMU_CAL_VALID_GYRO;
            if ((enabled & DRV_IMU_CAL_VALID_GYRO_TEMP) != 0U) {
                applied |= DRV_IMU_CAL_VALID_GYRO_TEMP;
            }
        }
    }

    return applied;
}

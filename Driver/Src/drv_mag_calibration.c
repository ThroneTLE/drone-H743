#include "drv_mag_calibration.h"

#include <math.h>
#include <stddef.h>

static uint8_t drv_mag_cal_vector_finite(const float vector[3])
{
    return ((vector != NULL) &&
            isfinite(vector[0]) &&
            isfinite(vector[1]) &&
            isfinite(vector[2])) ? 1U : 0U;
}

static uint8_t drv_mag_cal_matrix_finite(const float matrix[3][3])
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

static float drv_mag_cal_determinant(const float matrix[3][3])
{
    return (matrix[0][0] * ((matrix[1][1] * matrix[2][2]) -
                            (matrix[1][2] * matrix[2][1]))) -
           (matrix[0][1] * ((matrix[1][0] * matrix[2][2]) -
                            (matrix[1][2] * matrix[2][0]))) +
           (matrix[0][2] * ((matrix[1][0] * matrix[2][1]) -
                            (matrix[1][1] * matrix[2][0])));
}

DRV_MAG_CalibrationStatus DRV_MAG_Calibration_Validate(
    const DRV_MAG_Calibration *calibration)
{
    float determinant;

    if (calibration == NULL) {
        return DRV_MAG_CAL_INVALID_ARGS;
    }
    if ((drv_mag_cal_vector_finite(calibration->hard_iron_bias_mgauss) == 0U) ||
        (drv_mag_cal_matrix_finite(calibration->soft_iron_matrix) == 0U)) {
        return DRV_MAG_CAL_INVALID_NONFINITE;
    }

    determinant = drv_mag_cal_determinant(calibration->soft_iron_matrix);
    if (!(determinant > 0.0f)) {
        /* Covers zero, negative, and NaN (NaN > 0.0f is false) determinants. */
        return DRV_MAG_CAL_INVALID_DETERMINANT;
    }
    if ((determinant < DRV_MAG_CAL_MATRIX_DET_MIN) ||
        (determinant > DRV_MAG_CAL_MATRIX_DET_MAX)) {
        return DRV_MAG_CAL_INVALID_SCALE;
    }
    return DRV_MAG_CAL_VALID;
}

DRV_MAG_CalibrationStatus DRV_MAG_Calibration_Apply(
    const DRV_MAG_Calibration *calibration,
    const float raw_mgauss[3],
    float corrected_mgauss[3])
{
    DRV_MAG_CalibrationStatus status;
    float centered[3];
    float result[3];
    uint32_t row;

    if ((raw_mgauss == NULL) || (corrected_mgauss == NULL)) {
        return DRV_MAG_CAL_INVALID_ARGS;
    }
    if (drv_mag_cal_vector_finite(raw_mgauss) == 0U) {
        return DRV_MAG_CAL_INVALID_NONFINITE;
    }

    status = DRV_MAG_Calibration_Validate(calibration);
    if (status != DRV_MAG_CAL_VALID) {
        /* Coefficients are unusable: leave corrected_mgauss untouched rather
         * than degrading to a passthrough that would look like a genuinely
         * uncalibrated (identity) result. */
        return status;
    }

    centered[0] = raw_mgauss[0] - calibration->hard_iron_bias_mgauss[0];
    centered[1] = raw_mgauss[1] - calibration->hard_iron_bias_mgauss[1];
    centered[2] = raw_mgauss[2] - calibration->hard_iron_bias_mgauss[2];

    for (row = 0U; row < 3U; ++row) {
        result[row] =
            (calibration->soft_iron_matrix[row][0] * centered[0]) +
            (calibration->soft_iron_matrix[row][1] * centered[1]) +
            (calibration->soft_iron_matrix[row][2] * centered[2]);
    }
    if (drv_mag_cal_vector_finite(result) == 0U) {
        return DRV_MAG_CAL_INVALID_NONFINITE;
    }

    corrected_mgauss[0] = result[0];
    corrected_mgauss[1] = result[1];
    corrected_mgauss[2] = result[2];
    return DRV_MAG_CAL_VALID;
}

uint8_t DRV_MAG_FieldMagnitude_InRange(const float field_mgauss[3])
{
    float magnitude;

    if (drv_mag_cal_vector_finite(field_mgauss) == 0U) {
        return 0U;
    }
    magnitude = sqrtf((field_mgauss[0] * field_mgauss[0]) +
                      (field_mgauss[1] * field_mgauss[1]) +
                      (field_mgauss[2] * field_mgauss[2]));
    if (!isfinite(magnitude)) {
        return 0U;
    }
    return ((magnitude >= DRV_MAG_FIELD_MIN_MGAUSS) &&
            (magnitude <= DRV_MAG_FIELD_MAX_MGAUSS)) ? 1U : 0U;
}

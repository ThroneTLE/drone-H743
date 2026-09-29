"""Host-side tests for the Driver-layer magnetometer calibration math.

Compiles the real drv_mag_calibration.c with host gcc (no ADC/board access,
no HAL, no RTOS) and runs it, mirroring tests/test_imu_calibration_runtime.py
and tests/test_current_driver.py.
"""
from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]

HARNESS = r"""
#include "drv_mag_calibration.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define CHECK(condition, code) do { if (!(condition)) { \
    fprintf(stderr, "line %d, code %d: %s\n", __LINE__, (code), #condition); \
    return (code); } } while (0)

static int closef(float a, float b, float tol)
{
    return fabsf(a - b) < tol;
}

static void identity3(float m[3][3])
{
    memset(m, 0, 9U * sizeof(float));
    m[0][0] = 1.0f;
    m[1][1] = 1.0f;
    m[2][2] = 1.0f;
}

static void mat3_mul(const float a[3][3], const float b[3][3], float out[3][3])
{
    int i, j, k;
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            float sum = 0.0f;
            for (k = 0; k < 3; k++) {
                sum += a[i][k] * b[k][j];
            }
            out[i][j] = sum;
        }
    }
}

static void mat3_transpose(const float a[3][3], float out[3][3])
{
    int i, j;
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            out[i][j] = a[j][i];
        }
    }
}

static void mat3_vec(const float a[3][3], const float v[3], float out[3])
{
    int i;
    for (i = 0; i < 3; i++) {
        out[i] = (a[i][0] * v[0]) + (a[i][1] * v[1]) + (a[i][2] * v[2]);
    }
}

static void rot_z(float angle, float out[3][3])
{
    float c = cosf(angle);
    float s = sinf(angle);
    identity3(out);
    out[0][0] = c;  out[0][1] = -s;
    out[1][0] = s;  out[1][1] = c;
}

static void rot_y(float angle, float out[3][3])
{
    float c = cosf(angle);
    float s = sinf(angle);
    identity3(out);
    out[0][0] = c;  out[0][2] = s;
    out[2][0] = -s; out[2][2] = c;
}

/* --- 1. Uncalibrated = identity transform, bit-for-bit. --- */
static int test_uncalibrated_identity(void)
{
    DRV_MAG_Calibration cal;
    float raw[3] = {123.456f, -78.9f, 300.125f};
    float corrected[3] = {0};
    DRV_MAG_CalibrationStatus status;

    memset(&cal, 0, sizeof(cal));
    identity3(cal.soft_iron_matrix);
    /* cal.calibrated == 0, cal.hard_iron_bias_mgauss == {0,0,0} already via memset */

    status = DRV_MAG_Calibration_Apply(&cal, raw, corrected);
    CHECK(status == DRV_MAG_CAL_VALID, 1);
    CHECK(corrected[0] == raw[0], 2);
    CHECK(corrected[1] == raw[1], 3);
    CHECK(corrected[2] == raw[2], 4);

    /* In-place aliasing must also be an exact identity. */
    memcpy(corrected, raw, sizeof(raw));
    status = DRV_MAG_Calibration_Apply(&cal, corrected, corrected);
    CHECK(status == DRV_MAG_CAL_VALID, 5);
    CHECK(corrected[0] == raw[0] && corrected[1] == raw[1] &&
          corrected[2] == raw[2], 6);
    return 0;
}

/* --- 2. Pure hard iron: known bias, corrected sphere is centred at origin. --- */
static int test_pure_hard_iron(void)
{
    DRV_MAG_Calibration cal;
    static const float directions[4][3] = {
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f},
        {0.5773503f, 0.5773503f, 0.5773503f},
    };
    const float bias[3] = {40.0f, -25.0f, 15.0f};
    const float field_mag = 450.0f;
    int idx;

    memset(&cal, 0, sizeof(cal));
    cal.calibrated = 1U;
    identity3(cal.soft_iron_matrix);
    cal.hard_iron_bias_mgauss[0] = bias[0];
    cal.hard_iron_bias_mgauss[1] = bias[1];
    cal.hard_iron_bias_mgauss[2] = bias[2];

    for (idx = 0; idx < 4; idx++) {
        float truev[3] = {
            directions[idx][0] * field_mag,
            directions[idx][1] * field_mag,
            directions[idx][2] * field_mag,
        };
        float raw[3] = {
            truev[0] + bias[0],
            truev[1] + bias[1],
            truev[2] + bias[2],
        };
        float corrected[3];
        float mag;
        DRV_MAG_CalibrationStatus status =
            DRV_MAG_Calibration_Apply(&cal, raw, corrected);

        CHECK(status == DRV_MAG_CAL_VALID, 10 + idx);
        CHECK(closef(corrected[0], truev[0], 0.01f) &&
              closef(corrected[1], truev[1], 0.01f) &&
              closef(corrected[2], truev[2], 0.01f), 20 + idx);
        mag = sqrtf((corrected[0] * corrected[0]) +
                    (corrected[1] * corrected[1]) +
                    (corrected[2] * corrected[2]));
        CHECK(closef(mag, field_mag, 0.05f), 30 + idx);
    }
    return 0;
}

/* --- 3. Hard iron + rotated soft iron: corrected magnitudes agree. --- */
static int test_hard_and_soft_iron_with_rotation(void)
{
    DRV_MAG_Calibration cal;
    float rz[3][3];
    float ry[3][3];
    float r[3][3];
    float rt[3][3];
    float diag_scale[3][3];
    float diag_inv_scale[3][3];
    float tmp[3][3];
    float a_matrix[3][3];
    float m_matrix[3][3];
    const float scale[3] = {2.0f, 0.5f, 1.5f};
    const float bias[3] = {40.0f, -25.0f, 15.0f};
    const float field_mag = 450.0f;
    static const float directions[6][3] = {
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f},
        {0.5773503f, 0.5773503f, 0.5773503f},
        {0.8f, -0.6f, 0.0f},
        {-0.2672612f, 0.5345225f, 0.8017837f},
    };
    int idx;

    /* R = Ry * Rz is a proper rotation (det = +1), used to build a rotated
     * (non axis-aligned) ellipsoid so the test actually exercises the
     * off-diagonal terms of the soft-iron matrix, not just per-axis scale. */
    rot_z(0.5f, rz);
    rot_y(0.7f, ry);
    mat3_mul(ry, rz, r);
    mat3_transpose(r, rt);

    identity3(diag_scale);
    diag_scale[0][0] = scale[0];
    diag_scale[1][1] = scale[1];
    diag_scale[2][2] = scale[2];
    identity3(diag_inv_scale);
    diag_inv_scale[0][0] = 1.0f / scale[0];
    diag_inv_scale[1][1] = 1.0f / scale[1];
    diag_inv_scale[2][2] = 1.0f / scale[2];

    /* A = R * diag(scale) * R^T models the sensor's raw-axis distortion;
     * M = R * diag(1/scale) * R^T = A^-1 because R is orthogonal. */
    mat3_mul(r, diag_scale, tmp);
    mat3_mul(tmp, rt, a_matrix);
    mat3_mul(r, diag_inv_scale, tmp);
    mat3_mul(tmp, rt, m_matrix);

    memset(&cal, 0, sizeof(cal));
    cal.calibrated = 1U;
    memcpy(cal.soft_iron_matrix, m_matrix, sizeof(m_matrix));
    cal.hard_iron_bias_mgauss[0] = bias[0];
    cal.hard_iron_bias_mgauss[1] = bias[1];
    cal.hard_iron_bias_mgauss[2] = bias[2];

    for (idx = 0; idx < 6; idx++) {
        float truev[3] = {
            directions[idx][0] * field_mag,
            directions[idx][1] * field_mag,
            directions[idx][2] * field_mag,
        };
        float raw[3];
        float corrected[3];
        float mag;
        DRV_MAG_CalibrationStatus status;

        mat3_vec(a_matrix, truev, raw);
        raw[0] += bias[0];
        raw[1] += bias[1];
        raw[2] += bias[2];

        status = DRV_MAG_Calibration_Apply(&cal, raw, corrected);
        CHECK(status == DRV_MAG_CAL_VALID, 40 + idx);
        CHECK(closef(corrected[0], truev[0], 0.5f) &&
              closef(corrected[1], truev[1], 0.5f) &&
              closef(corrected[2], truev[2], 0.5f), 50 + idx);
        mag = sqrtf((corrected[0] * corrected[0]) +
                    (corrected[1] * corrected[1]) +
                    (corrected[2] * corrected[2]));
        CHECK(closef(mag, field_mag, 0.5f), 60 + idx);
    }
    return 0;
}

/* --- 4. Invalid coefficients are rejected, never degraded to identity. --- */
static int test_invalid_coefficients_rejected(void)
{
    DRV_MAG_Calibration cal;
    float raw[3] = {100.0f, 50.0f, 200.0f};
    float sentinel[3] = {-999.0f, -999.0f, -999.0f};
    float corrected[3];
    DRV_MAG_CalibrationStatus status;

    /* NaN in the hard-iron bias. */
    memset(&cal, 0, sizeof(cal));
    cal.calibrated = 1U;
    identity3(cal.soft_iron_matrix);
    cal.hard_iron_bias_mgauss[0] = NAN;
    memcpy(corrected, sentinel, sizeof(sentinel));
    status = DRV_MAG_Calibration_Validate(&cal);
    CHECK(status == DRV_MAG_CAL_INVALID_NONFINITE, 70);
    status = DRV_MAG_Calibration_Apply(&cal, raw, corrected);
    CHECK(status == DRV_MAG_CAL_INVALID_NONFINITE, 71);
    CHECK(memcmp(corrected, sentinel, sizeof(sentinel)) == 0, 72);
    CHECK(!(corrected[0] == raw[0] && corrected[1] == raw[1] &&
            corrected[2] == raw[2]), 73);

    /* Inf in the soft-iron matrix. */
    memset(&cal, 0, sizeof(cal));
    cal.calibrated = 1U;
    identity3(cal.soft_iron_matrix);
    cal.soft_iron_matrix[1][2] = INFINITY;
    memcpy(corrected, sentinel, sizeof(sentinel));
    status = DRV_MAG_Calibration_Apply(&cal, raw, corrected);
    CHECK(status == DRV_MAG_CAL_INVALID_NONFINITE, 74);
    CHECK(memcmp(corrected, sentinel, sizeof(sentinel)) == 0, 75);

    /* Degenerate matrix: determinant exactly 0 (duplicated row). */
    memset(&cal, 0, sizeof(cal));
    cal.calibrated = 1U;
    cal.soft_iron_matrix[0][0] = 1.0f; cal.soft_iron_matrix[0][1] = 0.0f; cal.soft_iron_matrix[0][2] = 0.0f;
    cal.soft_iron_matrix[1][0] = 1.0f; cal.soft_iron_matrix[1][1] = 0.0f; cal.soft_iron_matrix[1][2] = 0.0f;
    cal.soft_iron_matrix[2][0] = 0.0f; cal.soft_iron_matrix[2][1] = 0.0f; cal.soft_iron_matrix[2][2] = 1.0f;
    memcpy(corrected, sentinel, sizeof(sentinel));
    status = DRV_MAG_Calibration_Validate(&cal);
    CHECK(status == DRV_MAG_CAL_INVALID_DETERMINANT, 76);
    status = DRV_MAG_Calibration_Apply(&cal, raw, corrected);
    CHECK(status == DRV_MAG_CAL_INVALID_DETERMINANT, 77);
    CHECK(memcmp(corrected, sentinel, sizeof(sentinel)) == 0, 78);

    /* Reflected matrix: determinant negative. */
    memset(&cal, 0, sizeof(cal));
    cal.calibrated = 1U;
    identity3(cal.soft_iron_matrix);
    cal.soft_iron_matrix[2][2] = -1.0f;
    status = DRV_MAG_Calibration_Validate(&cal);
    CHECK(status == DRV_MAG_CAL_INVALID_DETERMINANT, 79);

    /* Magnitude out of range: uniform 100x scale, det = 1e6. */
    memset(&cal, 0, sizeof(cal));
    cal.calibrated = 1U;
    identity3(cal.soft_iron_matrix);
    cal.soft_iron_matrix[0][0] = 100.0f;
    cal.soft_iron_matrix[1][1] = 100.0f;
    cal.soft_iron_matrix[2][2] = 100.0f;
    memcpy(corrected, sentinel, sizeof(sentinel));
    status = DRV_MAG_Calibration_Validate(&cal);
    CHECK(status == DRV_MAG_CAL_INVALID_SCALE, 80);
    status = DRV_MAG_Calibration_Apply(&cal, raw, corrected);
    CHECK(status == DRV_MAG_CAL_INVALID_SCALE, 81);
    CHECK(memcmp(corrected, sentinel, sizeof(sentinel)) == 0, 82);

    /* Magnitude out of range on the low side too: uniform 0.01x scale. */
    memset(&cal, 0, sizeof(cal));
    cal.calibrated = 1U;
    identity3(cal.soft_iron_matrix);
    cal.soft_iron_matrix[0][0] = 0.01f;
    cal.soft_iron_matrix[1][1] = 0.01f;
    cal.soft_iron_matrix[2][2] = 0.01f;
    status = DRV_MAG_Calibration_Validate(&cal);
    CHECK(status == DRV_MAG_CAL_INVALID_SCALE, 83);

    /* NULL calibration/pointers. */
    CHECK(DRV_MAG_Calibration_Validate(NULL) == DRV_MAG_CAL_INVALID_ARGS, 84);
    memset(&cal, 0, sizeof(cal));
    identity3(cal.soft_iron_matrix);
    CHECK(DRV_MAG_Calibration_Apply(&cal, NULL, corrected) ==
          DRV_MAG_CAL_INVALID_ARGS, 85);
    CHECK(DRV_MAG_Calibration_Apply(&cal, raw, NULL) ==
          DRV_MAG_CAL_INVALID_ARGS, 86);

    return 0;
}

/* --- 5. Field-strength gating. --- */
static int test_field_magnitude_gate(void)
{
    float in_range[3] = {300.0f, 300.0f, 150.0f}; /* magnitude 450 mGauss */
    float too_weak[3] = {1.0f, 1.0f, 1.0f};        /* dead/absent sensor */
    float too_strong[3] = {2000.0f, 0.0f, 0.0f};   /* near a motor phase */
    float non_finite[3] = {NAN, 0.0f, 0.0f};
    float at_min[3] = {DRV_MAG_FIELD_MIN_MGAUSS, 0.0f, 0.0f};
    float at_max[3] = {DRV_MAG_FIELD_MAX_MGAUSS, 0.0f, 0.0f};

    CHECK(DRV_MAG_FieldMagnitude_InRange(in_range) == 1U, 90);
    CHECK(DRV_MAG_FieldMagnitude_InRange(too_weak) == 0U, 91);
    CHECK(DRV_MAG_FieldMagnitude_InRange(too_strong) == 0U, 92);
    CHECK(DRV_MAG_FieldMagnitude_InRange(non_finite) == 0U, 93);
    CHECK(DRV_MAG_FieldMagnitude_InRange(NULL) == 0U, 94);
    CHECK(DRV_MAG_FieldMagnitude_InRange(at_min) == 1U, 95);
    CHECK(DRV_MAG_FieldMagnitude_InRange(at_max) == 1U, 96);
    return 0;
}

int main(void)
{
    int rc;
    if ((rc = test_uncalibrated_identity()) != 0) return rc;
    if ((rc = test_pure_hard_iron()) != 0) return rc;
    if ((rc = test_hard_and_soft_iron_with_rotation()) != 0) return rc;
    if ((rc = test_invalid_coefficients_rejected()) != 0) return rc;
    if ((rc = test_field_magnitude_gate()) != 0) return rc;
    puts("ok");
    return 0;
}
"""


def _compile_and_run(tmp_path: Path) -> str:
    compiler = shutil.which("gcc") or shutil.which("clang")
    if compiler is None:
        pytest.skip("host C compiler is unavailable")
    harness = tmp_path / "harness.c"
    executable = tmp_path / "harness.exe"
    harness.write_text(HARNESS, encoding="ascii")
    result = subprocess.run(
        [
            compiler,
            "-std=c11",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-O2",
            f"-I{ROOT / 'Driver' / 'Inc'}",
            str(ROOT / "Driver" / "Src" / "drv_mag_calibration.c"),
            str(harness),
            "-lm",
            "-o",
            str(executable),
        ],
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    result = subprocess.run(
        [str(executable)], capture_output=True, text=True
    )
    assert result.returncode == 0, result.stdout + result.stderr
    return result.stdout.strip()


def test_mag_calibration_math_on_real_c(tmp_path: Path) -> None:
    assert _compile_and_run(tmp_path) == "ok"


def test_mag_calibration_driver_has_no_hardware_dependency() -> None:
    header = (ROOT / "Driver" / "Inc" / "drv_mag_calibration.h").read_text(
        encoding="utf-8"
    )
    source = (ROOT / "Driver" / "Src" / "drv_mag_calibration.c").read_text(
        encoding="utf-8"
    )
    for banned in ("HAL_", "FreeRTOS", "osKernel", "cmsis_os", "#include \"bsp_",
                   "#include \"stm32", "xQueue", "xSemaphore"):
        assert banned not in header
        assert banned not in source
    # Boundary: must not take a dependency on the frame-contract or drv_mag
    # headers owned by other work orders (comments may still mention them by
    # name to explain the boundary, so check the #include directive only).
    assert '#include "drv_frame_contract.h"' not in header
    assert '#include "drv_frame_contract.h"' not in source
    assert '#include "drv_mag.h"' not in header
    assert '#include "drv_mag.h"' not in source

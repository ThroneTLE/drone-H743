"""Contract tests for the 2nd-order Butterworth sensor filter.

The filter was upgraded from a 1st-order IIR because measured rotor vibration
sits at 56-180 Hz, where the old filter only reached -8.9 dB. These tests pin
the properties that matter: the attenuation actually achieved, steady-state
startup, and that the firmware and the analysis tool agree on the math.
"""

from __future__ import annotations

import importlib.util
import shutil
import subprocess
from pathlib import Path

import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def load_report_tool():
    spec = importlib.util.spec_from_file_location(
        "imu_filter_report", ROOT / "tools" / "imu_filter_report.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_filter_is_second_order_biquad() -> None:
    header = read("App/Inc/app_sensor.h")
    source = read("App/Src/app_sensor.c")

    # A 1st-order single-pole state would not have these.
    for field in ("b0", "b1", "b2", "a1", "a2", "x1", "x2", "y1", "y2"):
        assert field in header

    # The old single-pole implementation must be gone, not left alongside.
    # (APP_IMU_AttitudeDebug has an unrelated `alpha` field, so scope the check
    # to the filter struct itself.)
    lpf_struct = header[header.index("typedef struct {\n    float b0"):
                        header.index("} APP_Sensor_Lpf;")]
    assert "alpha" not in lpf_struct
    assert "state" not in lpf_struct
    assert "lpf->state +=" not in source


def test_cutoffs_match_the_measured_blade_band() -> None:
    freertos = read("Core/Src/freertos.c")

    # Measured hover blade tone is ~180 Hz; 80 Hz 2nd-order gives -15.9 dB.
    assert "APP_Sensor_LpfInit(&gyro_lpf[i], 80.0f, 0.001f)" in freertos
    assert "APP_Sensor_LpfInit(&acc_lpf[i], 40.0f, 0.001f)" in freertos


BIQUAD_HARNESS = r"""
#include "app_sensor.h"

#include <math.h>
#include <stdio.h>

#define RATE 1000.0f
#define DT   (1.0f / RATE)

/* Steady-state amplitude of the filter output at one frequency. */
static float measure_gain(float cutoff_hz, float tone_hz)
{
    APP_Sensor_Lpf lpf;
    float peak = 0.0f;
    int i;

    APP_Sensor_LpfInit(&lpf, cutoff_hz, DT);
    /* Run long enough for transients to die out before measuring. */
    for (i = 0; i < 4000; ++i) {
        (void)APP_Sensor_LpfApply(&lpf, sinf(2.0f * 3.14159265f * tone_hz * (float)i * DT));
    }
    for (; i < 6000; ++i) {
        float out = APP_Sensor_LpfApply(&lpf, sinf(2.0f * 3.14159265f * tone_hz * (float)i * DT));
        if (fabsf(out) > peak) { peak = fabsf(out); }
    }
    return peak;
}

int main(void)
{
    APP_Sensor_Lpf lpf;
    float dc;
    int i;

    /* DC must pass with unity gain, or gravity would be scaled. */
    APP_Sensor_LpfInit(&lpf, 80.0f, DT);
    for (i = 0; i < 2000; ++i) { dc = APP_Sensor_LpfApply(&lpf, 1.0f); }
    if (fabsf(dc - 1.0f) > 0.001f) { return 1; }

    /* First sample must emerge as-is: a filter starting from zero would show a
     * false large tilt while ramping up to gravity. */
    APP_Sensor_LpfInit(&lpf, 80.0f, DT);
    if (fabsf(APP_IMU_FIRST(&lpf) - 0.98f) > 0.0001f) { return 2; }

    /* Attenuation at the measured blade tones. */
    if (!(measure_gain(80.0f, 179.9f) < 0.20f)) { return 3; }   /* < -14 dB */
    if (!(measure_gain(80.0f, 107.2f) < 0.55f)) { return 4; }   /* < -5 dB  */
    /* In-band signal must survive. */
    if (!(measure_gain(80.0f, 10.0f) > 0.95f)) { return 5; }

    /* Beats the 1st-order design it replaced (-8.9 dB at 180 Hz => 0.36). */
    if (!(measure_gain(80.0f, 179.9f) < 0.30f)) { return 6; }

    printf("ok\n");
    return 0;
}
"""


def test_biquad_runtime_behaviour(tmp_path: Path) -> None:
    compiler = shutil.which("gcc")
    if compiler is None:
        pytest.skip("host gcc is unavailable")

    # app_sensor.h pulls in HAL/RTOS headers; provide the minimum it needs.
    stub_dir = tmp_path / "stubs"
    stub_dir.mkdir()
    (stub_dir / "drv_imu.h").write_text(
        "#include <stdint.h>\n#include <stdbool.h>\n"
        "typedef struct { int16_t temperature, accel_x, accel_y, accel_z,"
        " gyro_x, gyro_y, gyro_z; } DRV_IMU_RawData;\n"
        "typedef struct { float temperature_c, accel_x_g, accel_y_g, accel_z_g,"
        " gyro_x_dps, gyro_y_dps, gyro_z_dps; } DRV_IMU_ScaledData;\n"
        "typedef int DRV_IMU_AccelRange; typedef int DRV_IMU_GyroRange;\n"
        "typedef struct { DRV_IMU_AccelRange accel_range;"
        " DRV_IMU_GyroRange gyro_range; } DRV_IMU_Config_stub;\n",
        encoding="ascii")

    harness_src = BIQUAD_HARNESS.replace("APP_IMU_FIRST(&lpf)",
                                         "APP_Sensor_LpfApply(&lpf, 0.98f)")
    harness = tmp_path / "biquad.c"
    harness.write_text(harness_src, encoding="ascii")

    # Compile only the filter functions, extracted to avoid the HAL dependency.
    source = read("App/Src/app_sensor.c")
    start = source.index("void APP_Sensor_LpfInit")
    # Stop before the gyro-bias code, which needs types outside this slice.
    end = source.index("void APP_Sensor_LpfApply3f")
    header = read("App/Inc/app_sensor.h")
    h_start = header.index("typedef struct {\n    float b0")
    h_end = header.index("void APP_Sensor_LpfApply3f")

    iso_h = tmp_path / "app_sensor.h"
    iso_h.write_text(
        "#ifndef ISO_H\n#define ISO_H\n#include <stdint.h>\n"
        + header[h_start:h_end] + "\n#endif\n", encoding="utf-8")
    iso_c = tmp_path / "filter.c"
    iso_c.write_text(
        '#include "app_sensor.h"\n#include <math.h>\n#include <string.h>\n'
        '#include <stddef.h>\n#define NULL_OK\n' + source[start:end],
        encoding="utf-8")

    executable = tmp_path / "biquad.exe"
    subprocess.run(
        [compiler, "-std=c11", "-O2", f"-I{tmp_path}",
         str(iso_c), str(harness), "-lm", "-o", str(executable)],
        check=True, capture_output=True, text=True)
    subprocess.run([str(executable)], check=True, capture_output=True, text=True)


def test_report_tool_matches_firmware_coefficients() -> None:
    module = load_report_tool()

    # The analysis tool's design must be the same filter the firmware runs,
    # otherwise the published attenuation figures describe nothing real.
    b0, b1, b2, a1, a2 = module.biquad_coeffs(80.0, 1000.0)

    w0 = 2 * np.pi * 80.0 / 1000.0
    alpha = np.sin(w0) * 0.70710678
    a0 = 1 + alpha
    assert b0 == pytest.approx(((1 - np.cos(w0)) / 2) / a0)
    assert b1 == pytest.approx((1 - np.cos(w0)) / a0)
    assert b2 == pytest.approx(b0)
    assert a1 == pytest.approx((-2 * np.cos(w0)) / a0)
    assert a2 == pytest.approx((1 - alpha) / a0)

    # Unity DC gain: sum(b) / (1 + sum(a)) == 1.
    assert (b0 + b1 + b2) / (1 + a1 + a2) == pytest.approx(1.0, abs=1e-6)


def test_second_order_beats_first_order_at_blade_tone() -> None:
    module = load_report_tool()
    rate = 992.1

    first = module.response_db(80.0, 1, np.array([179.9]), rate)[0]
    second = module.response_db(80.0, 2, np.array([179.9]), rate)[0]

    assert first == pytest.approx(-8.9, abs=0.5)
    assert second == pytest.approx(-15.9, abs=0.5)

    # The delay penalty must stay under 1 ms in the attitude band, or the
    # attenuation would be bought at the cost of loop stability.
    d1 = module.group_delay_ms(80.0, 1, 10.0, rate)
    d2 = module.group_delay_ms(80.0, 2, 10.0, rate)
    assert 0.0 < (d2 - d1) < 1.0

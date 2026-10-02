"""Run the real XY service with the real Fusion AHRS at fresh-mag cadence.

The artificial gyro bias is only a software integration probe. Flight drift
acceptance still needs synchronized physical captures under changed DCDC load.
"""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
HARNESS = r"""
#include "drv_attitude_fusion.h"
#include "svc_mag_heading.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static unsigned long long time_us;

static DRV_AttitudeFusionOutput tick(float gyro_z_dps, const float mag[3],
                                      unsigned char valid)
{
    DRV_AttitudeFusionInput input;
    DRV_AttitudeFusionOutput output;
    memset(&input, 0, sizeof(input));
    memset(&output, 0, sizeof(output));
    time_us += 2000ULL;
    input.time_us = time_us;
    input.dt_s = 0.002f;
    input.gyroscope_dps[2] = gyro_z_dps;
    input.accelerometer_g[2] = 1.0f;
    if (mag != NULL) {
        memcpy(input.magnetometer_mgauss, mag, sizeof(input.magnetometer_mgauss));
    }
    input.magnetometer_valid = valid;
    (void)DRV_AttitudeFusion_Update(&input, &output);
    return output;
}

static float trial(unsigned char with_mag, float *first_mag_change,
                   unsigned long *mag_ready_count)
{
    SVC_MAG_HeadingConfig config = {
        .bias_x_mgauss = -3.7253387f,
        .bias_y_mgauss = 135.43175f,
        .radius_xy_mgauss = 268.29245f,
        .generation = 1U,
        .axis_verified = 1U,
        .enabled = 1U,
    };
    SVC_MAG_HeadingState state;
    DRV_AttitudeFusionOutput output;
    float field[3] = {0.0f, 0.0f, 0.0f};
    int index;
    unsigned long long mag_timestamp = 0ULL;
    float initial_yaw;

    SVC_MAG_HeadingReset(&state);
    DRV_AttitudeFusion_InitForConvention(DRV_ATTITUDE_FUSION_CONVENTION_NWU);
    time_us = 0ULL;
    for (index = 0; index < 2000; ++index) {
        output = tick(0.0f, NULL, 0U); /* leave 3-second startup */
    }
    for (index = 0; index < 500; ++index) {
        output = tick(20.0f, NULL, 0U); /* 20-degree relative initial yaw */
    }
    for (index = 0; index < 250; ++index) {
        unsigned char mag_valid = 0U;
        if (index % 25 == 0 && with_mag != 0U) {
            SVC_MAG_HeadingInput sample;
            SVC_MAG_HeadingResult result;
            memset(&sample, 0, sizeof(sample));
            mag_timestamp += 50000ULL;
            sample.timestamp_us = mag_timestamp;
            sample.raw_flu_mgauss[0] = 265.0f;
            sample.raw_flu_mgauss[1] = 194.0f;
            sample.raw_flu_mgauss[2] = ((index / 25) & 1) ? 630.0f : 750.0f;
            sample.yaw_deg = output.yaw_deg;
            result = SVC_MAG_HeadingUpdate(&config, &state, &sample, field);
            if (result == SVC_MAG_HEADING_READY) {
                mag_valid = 1U;
                *first_mag_change = output.yaw_deg;
            }
        }
        output = tick(0.0f, field, mag_valid);
        if (mag_valid != 0U) {
            *first_mag_change = output.yaw_deg - *first_mag_change;
        }
    }
    initial_yaw = output.yaw_deg;
    for (index = 0; index < 60000; ++index) {
        unsigned char mag_valid = 0U;
        if (index % 25 == 0 && with_mag != 0U) {
            SVC_MAG_HeadingInput sample;
            memset(&sample, 0, sizeof(sample));
            mag_timestamp += 50000ULL;
            sample.timestamp_us = mag_timestamp;
            sample.raw_flu_mgauss[0] = 265.0f;
            sample.raw_flu_mgauss[1] = 194.0f;
            sample.raw_flu_mgauss[2] = ((index / 25) & 1) ? 630.0f : 750.0f;
            sample.yaw_deg = output.yaw_deg;
            mag_valid = (SVC_MAG_HeadingUpdate(&config, &state, &sample, field)
                         == SVC_MAG_HEADING_READY) ? 1U : 0U;
        }
        output = tick(0.05f, field, mag_valid);
    }
    *mag_ready_count = state.ready_count;
    return output.yaw_deg - initial_yaw;
}

int main(void)
{
    float snap_no_mag = 0.0f;
    float snap_mag = 0.0f;
    unsigned long count_no_mag = 0U;
    unsigned long count_mag = 0U;
    float no_mag_drift = trial(0U, &snap_no_mag, &count_no_mag);
    float mag_drift = trial(1U, &snap_mag, &count_mag);
    printf("no_mag_drift=%.6f mag_drift=%.6f first_mag_change=%.6f count=%lu\n",
           no_mag_drift, mag_drift, snap_mag, count_mag);
    return 0;
}
"""


def test_fresh_xy_mag_has_no_initial_north_snap_and_slowly_limits_bias(tmp_path: Path) -> None:
    gcc = shutil.which("gcc")
    if gcc is None:
        pytest.skip("host gcc unavailable")
    harness = tmp_path / "magxy_cadence.c"
    executable = tmp_path / "magxy_cadence.exe"
    harness.write_text(HARNESS, encoding="ascii")
    command = [
        gcc, "-std=c11", "-Wall", "-Wextra", "-Werror",
        f"-I{ROOT / 'Driver' / 'Inc'}",
        f"-I{ROOT / 'Services' / 'Inc'}",
        f"-I{ROOT / 'ThirdParty' / 'Fusion'}",
        str(ROOT / "Driver/Src/drv_attitude_fusion.c"),
        str(ROOT / "Driver/Src/drv_mag_calibration.c"),
        str(ROOT / "Services/Src/svc_mag_heading.c"),
        str(ROOT / "ThirdParty/Fusion/FusionAhrs.c"),
        str(harness), "-lm", "-o", str(executable),
    ]
    built = subprocess.run(command, capture_output=True, text=True)
    assert built.returncode == 0, built.stdout + built.stderr
    run = subprocess.run([str(executable)], capture_output=True, text=True)
    assert run.returncode == 0, run.stdout + run.stderr
    fields = dict(item.split("=") for item in run.stdout.strip().split())
    no_mag = float(fields["no_mag_drift"])
    with_mag = float(fields["mag_drift"])
    initial_change = float(fields["first_mag_change"])
    assert 5.0 < no_mag < 7.0
    assert abs(initial_change) < 0.05
    assert abs(with_mag) < abs(no_mag)
    assert int(fields["count"]) > 2000

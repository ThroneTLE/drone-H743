"""Contract tests for the ICM-42688 anti-alias filter and accelerometer range.

The AAF is an analogue filter ahead of the sampler, so it is the only mechanism
that can stop out-of-band rotor harmonics from folding into the attitude band.
It was previously left at the power-on default, and the accelerometer ran at
+-4 g where coaxial-rotor vibration clipped the int16 rail.
"""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def test_accel_range_is_16g_to_avoid_vibration_clipping() -> None:
    bsp = read("BSP/Src/bsp_imu.c")

    assert "DRV_IMU_ACCEL_RANGE_16G" in bsp
    assert "DRV_IMU_ACCEL_RANGE_4G" not in bsp


def test_scaling_is_derived_from_configured_range_not_hardcoded() -> None:
    sensor = read("App/Src/app_sensor.c")

    # A hardcoded divisor silently misscales attitude whenever BSP changes
    # the range, which is exactly how the +-4 g assumption survived.
    assert "APP_IMU_ACCEL_LSB_PER_G" not in sensor
    assert "DRV_IMU_AccelLsbPerG" in sensor
    assert "DRV_IMU_GyroLsbPerDps" in sensor


def test_aaf_registers_are_configured_in_both_user_banks() -> None:
    driver = read("Driver/Src/drv_imu.c")

    # Gyro AAF is in bank 1, accel AAF in bank 2.
    for token in (
        "ICM42688_REG_GYRO_CONFIG_STATIC3",
        "ICM42688_REG_GYRO_CONFIG_STATIC4",
        "ICM42688_REG_GYRO_CONFIG_STATIC5",
        "ICM42688_REG_ACCEL_CONFIG_STATIC2",
        "ICM42688_REG_ACCEL_CONFIG_STATIC3",
        "ICM42688_REG_ACCEL_CONFIG_STATIC4",
    ):
        assert token in driver

    assert "ICM42688_BANK1" in driver
    assert "ICM42688_BANK2" in driver
    # Bank 0 must be restored or subsequent data reads target the wrong bank.
    assert "restore_bank0" in driver


def test_board_requests_an_aaf_cutoff_below_nyquist() -> None:
    bsp = read("BSP/Src/bsp_imu.c")

    assert "accel_aaf_hz" in bsp
    assert "gyro_aaf_hz" in bsp
    # 1 kHz ODR gives a 500 Hz Nyquist; the cutoff must sit meaningfully below.
    assert "213U" in bsp


AAF_HARNESS = r"""
#include "drv_imu.h"

#include <stdio.h>

#define CHECK(condition, code) do { if (!(condition)) { return (code); } } while (0)

int main(void)
{
    unsigned short actual = 0U;

    /* An exact table entry must be honoured. */
    CHECK(DRV_IMU_AafSettingForCutoff(213U, &actual) != NULL, 1);
    CHECK(actual == 213U, 2);

    /* Between entries, snap down so the filter never passes more than asked. */
    (void)DRV_IMU_AafSettingForCutoff(250U, &actual);
    CHECK(actual == 213U, 3);

    /* Below the narrowest entry, fall back to the narrowest rather than zero. */
    (void)DRV_IMU_AafSettingForCutoff(10U, &actual);
    CHECK(actual == 42U, 4);

    /* A very wide request saturates at the widest supported cutoff. */
    (void)DRV_IMU_AafSettingForCutoff(9000U, &actual);
    CHECK(actual == 1962U, 5);

    return 0;
}
"""


def test_aaf_cutoff_selection_snaps_to_supported_values(tmp_path: Path) -> None:
    compiler = shutil.which("gcc")
    if compiler is None:
        pytest.skip("host gcc is unavailable")

    harness = tmp_path / "aaf_harness.c"
    executable = tmp_path / "aaf_harness.exe"
    harness.write_text(AAF_HARNESS, encoding="ascii")

    # Compile only the pure selection logic; the register writes need HAL.
    source = (ROOT / "Driver" / "Src" / "drv_imu.c").read_text(encoding="utf-8")
    start = source.index("typedef struct {\n    uint16_t cutoff_hz;")
    end = source.index("static void icm42688_delay_ms")
    isolated = tmp_path / "aaf_selection.c"
    isolated.write_text(
        "#include <stdint.h>\n#include <stddef.h>\n" + source[start:end],
        encoding="utf-8",
    )

    shim = tmp_path / "drv_imu.h"
    shim.write_text(
        "#include <stdint.h>\n#include <stddef.h>\n"
        "const void *DRV_IMU_AafSettingForCutoff(uint16_t desired_hz,\n"
        "                                        uint16_t *actual_hz);\n",
        encoding="ascii",
    )

    subprocess.run(
        [
            compiler,
            "-std=c11",
            "-Wall",
            "-Wextra",
            "-Werror",
            f"-I{tmp_path}",
            str(isolated),
            str(harness),
            "-o",
            str(executable),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    subprocess.run([str(executable)], check=True, capture_output=True, text=True)

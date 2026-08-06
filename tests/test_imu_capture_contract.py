"""Contract tests for the full-rate raw IMU capture path.

The capture exists to record undecimated pre-LPF samples for vibration spectrum
analysis. Two properties matter most and are easy to regress:

1. The producer hook must never block, because it runs in Sensor_Task at 1 kHz.
2. The on-wire sample layout must match the host decoder exactly.
"""

from __future__ import annotations

import importlib.util
import re
import shutil
import struct
import subprocess
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def load_host_tool():
    spec = importlib.util.spec_from_file_location(
        "imu_vibration_capture", ROOT / "tools" / "imu_vibration_capture.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_capture_buffer_lives_outside_dtcm() -> None:
    source = read("App/Src/app_imu_capture.c")

    # DTCM holds the RTOS stacks and is only 128 KB; the buffer must not land there.
    assert ".ram_d1_noinit" in source
    assert ".dma_buffer" not in source


def test_producer_hook_is_non_blocking() -> None:
    source = read("App/Src/app_imu_capture.c")
    push = source[source.index("void APP_IMU_Capture_Push") :
                  source.index("static int16_t imu_capture_saturate")]

    # Anything that waits, locks, or transmits would stall the 1 kHz sampler.
    for forbidden in ("osDelay", "osMutex", "osSemaphore", "APP_USB_CDC_Write",
                      "HAL_Delay", "while ("):
        assert forbidden not in push, f"{forbidden} must not appear in the sample hook"


def test_capture_hook_runs_before_filtering() -> None:
    source = read("Core/Src/freertos.c")

    push_at = source.index("APP_IMU_Capture_Push")
    lpf_at = source.index("APP_Sensor_LpfApply3f(gyro_lpf")
    scale_at = source.index("APP_IMU_RawToScaled(&raw, &scaled)")

    # Raw samples are the point: capture must precede scaling and the LPF.
    assert push_at < scale_at < lpf_at


def test_export_runs_off_the_sampling_path() -> None:
    source = read("Core/Src/freertos.c")

    # The blocking USB export must be driven from a low-priority task, never
    # from Sensor_Task where the samples are produced. Locate the export call
    # and confirm the enclosing task entry point is not Sensor_Task.
    export_at = source.index("APP_IMU_Capture_ExportStep")
    task_starts = [
        (source.index(sig + "(void *argument)\n{"), sig)
        for sig in ("void Sensor_Task", "void StabilizerTask", "void BackgroundTask")
        if (sig + "(void *argument)\n{") in source
    ]
    enclosing = max((pos, sig) for pos, sig in task_starts if pos < export_at)
    assert enclosing[1] != "void Sensor_Task", (
        f"ExportStep is called from {enclosing[1]}, which produces samples")
    assert enclosing[1] != "void StabilizerTask", (
        "ExportStep must not block the stabilizer loop")


def test_sample_layout_matches_host_decoder() -> None:
    module = load_host_tool()
    header = read("App/Inc/app_imu_capture.h")

    # Packed, or the compiler's tail padding silently desynchronises the host.
    assert "__attribute__((packed))" in header
    assert module.SAMPLE_SIZE == 46
    assert module.HEADER_SIZE == struct.calcsize(module.HEADER_FMT)


LAYOUT_HARNESS = r"""
#include "app_imu_capture.h"
#include <stdio.h>

int main(void)
{
    printf("%u %u\n",
           (unsigned int)sizeof(APP_IMU_CaptureSample),
           (unsigned int)sizeof(APP_IMU_CaptureBlockHeader));
    return 0;
}
"""


def test_firmware_struct_sizes_match_host(tmp_path: Path) -> None:
    compiler = shutil.which("gcc")
    if compiler is None:
        pytest.skip("host gcc is unavailable")

    module = load_host_tool()

    # Stub the HAL-dependent include so the header can compile standalone.
    stub = tmp_path / "drv_imu.h"
    stub.write_text(
        "#include <stdint.h>\n"
        "typedef struct { int16_t temperature, accel_x, accel_y, accel_z,"
        " gyro_x, gyro_y, gyro_z; } DRV_IMU_RawData;\n",
        encoding="ascii",
    )

    harness = tmp_path / "layout.c"
    harness.write_text(LAYOUT_HARNESS, encoding="ascii")
    executable = tmp_path / "layout.exe"

    subprocess.run(
        [compiler, "-std=c11", "-Wall", "-Wextra", "-Werror",
         f"-I{tmp_path}", f"-I{ROOT / 'App' / 'Inc'}",
         str(harness), "-o", str(executable)],
        check=True, capture_output=True, text=True,
    )
    result = subprocess.run([str(executable)], check=True,
                            capture_output=True, text=True)
    sample_size, header_size = (int(x) for x in result.stdout.split())

    assert sample_size == module.SAMPLE_SIZE
    assert header_size == module.HEADER_SIZE


def test_block_magic_cannot_appear_in_status_text() -> None:
    """The magic must not collide with the ASCII command responses.

    Regression: the original magic spelled "IMUC", which is exactly how
    "IMUCAP DUMP ok ..." starts. A host resynchronising on the magic locked onto
    that status line and parsed ASCII as header fields, reporting a bogus
    version/sample_size mismatch instead of reading the stream.
    """
    module = load_host_tool()
    magic = struct.pack("<I", module.CAPTURE_MAGIC)

    assert any(byte > 0x7F for byte in magic), (
        "magic must contain a non-ASCII byte so it cannot occur in status text")

    for line in (b"IMUCAP DUMP ok samples=6144 sample_bytes=46",
                 b"IMUCAP state=idle stored=0 capacity=6144",
                 b"IMUCAP START ok requested=6144",
                 b"ERR usage IMUCAP? | IMUCAP START [samples]"):
        assert magic not in line

    # Firmware and host must agree on the value.
    header = read("App/Inc/app_imu_capture.h")
    match = re.search(r"APP_IMU_CAPTURE_MAGIC\s+0[xX]([0-9A-Fa-f]+)UL", header)
    assert match, "magic literal not found in the firmware header"
    assert int(match.group(1), 16) == module.CAPTURE_MAGIC

    version = re.search(r"APP_IMU_CAPTURE_VERSION\s+(\d+)U", header)
    assert version and int(version.group(1)) == module.CAPTURE_VERSION


def test_reader_skips_implausible_headers_instead_of_aborting() -> None:
    source = (ROOT / "tools" / "imu_vibration_capture.py").read_text(encoding="utf-8")
    reader = source[source.index("def read_blocks") : source.index("def samples_to_rows")]

    # A false-positive magic match must resync, not kill the whole dump.
    assert "plausible" in reader
    assert "continue" in reader


def test_analysis_recovers_a_known_tone() -> None:
    module = load_host_tool()
    import numpy as np

    rate, count, tone_hz = 1000.0, 4096, 250.0
    t = np.arange(count) / rate
    samples = []
    for i in range(count):
        gyro_lsb = int(3000 * np.sin(2 * np.pi * tone_hz * t[i]))
        samples.append((i * 1000, 0, 0, 2048, gyro_lsb, 0, 0,
                        0, 0, 1000, 0, 0, 0,
                        0, 0, 0, 1500, 1500, 1500, 1500, 0, 3, 0))

    meta = {"accel_range_g": 16, "gyro_range_dps": 1000,
            "accel_aaf_hz": 213, "gyro_aaf_hz": 213}
    rows = module.samples_to_rows(samples, meta)
    report = module.analyse(rows, meta)

    assert abs(report["effective_rate_hz"] - rate) < 1.0
    peak = report["raw_spectra"]["gyro_x"]["peaks_hz"][0]["freq_hz"]
    assert abs(peak - tone_hz) < 1.0
    # A tone above the configured AAF must be called out, not silently accepted.
    assert any("AAF" in note for note in report["recommendation"]["notes"])


def test_clipping_is_reported() -> None:
    module = load_host_tool()

    # Samples pinned at the int16 rail, as +-4 g did under rotor vibration.
    samples = [(i * 1000, 32767, 0, 2048, 0, 0, 0,
                0, 0, 1000, 0, 0, 0, 0, 0, 0,
                1500, 1500, 1500, 1500, 0, 3, 0) for i in range(128)]
    meta = {"accel_range_g": 4, "gyro_range_dps": 1000,
            "accel_aaf_hz": 213, "gyro_aaf_hz": 213}
    report = module.analyse(module.samples_to_rows(samples, meta), meta)

    assert report["clipping"]["accel"]["clipped_pct"] > 30.0
    assert any("clipping" in note for note in report["recommendation"]["notes"])

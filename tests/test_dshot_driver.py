"""R-DSHOT-1 first phase: compile the real C driver and frozen PX4 encoder.

No DMA/register simulation here: hardware startup, preload and IRQ races belong
to the BSP integration phase after CubeMX regeneration. These tests prove only
the protocol words, waveform arrays and timer calculations.
"""
from __future__ import annotations

import hashlib
import re
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
FIXTURES = ROOT / "tests/fixtures/dshot_px4"
UPSTREAM_SHA256 = "fa428eec81cbfdf70d89684eb182fbb2d023ca64c8db4cb82133598d2c5fbbaa"


def upstream_function() -> str:
    raw = (FIXTURES / "upstream_dshot.c.txt").read_bytes()
    assert hashlib.sha256(raw).hexdigest() == UPSTREAM_SHA256
    source = raw.decode("utf-8")
    start = source.index("void dshot_motor_data_set(")
    opening = source.index("{", start)
    depth = 0
    for pos in range(opening, len(source)):
        depth += (source[pos] == "{") - (source[pos] == "}")
        if depth == 0:
            return source[start:pos + 1]
    raise AssertionError("frozen upstream function is incomplete")


REFERENCE_ADAPTER = r"""
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#define DSHOT_THROTTLE_POSITION 5u
#define DSHOT_TELEMETRY_POSITION 4u
#define NIBBLES_SIZE 4u
#define DSHOT_NUMBER_OF_NIBBLES 3u
#define ONE_MOTOR_DATA_SIZE 16u
#define MOTOR_PWM_BIT_0 7u
#define MOTOR_PWM_BIT_1 14u
typedef struct { uint8_t channel_count_including_gaps, lowest_timer_channel; } io_timers_channel_mapping_element_t;
static const struct { uint8_t timer_index, timer_channel; } timer_io_channels[2] = {{0, 1}, {0, 2}};
static const struct { bool initialized_channels[4]; } timer_configs[1] = {{{true, true, false, false}}};
static const struct { io_timers_channel_mapping_element_t element[1]; } io_timers_channel_mapping = {{{2, 1}}};
static const bool _bidirectional = false;
static uint32_t storage[34];
static uint32_t *dshot_output_buffer[1] = {storage};
"""


@pytest.fixture(scope="module")
def harness(tmp_path_factory):
    compiler = shutil.which("gcc")
    assert compiler, "host gcc required for the DShot contract"
    build = tmp_path_factory.mktemp("dshot-driver")
    reference = build / "px4_reference.c"
    reference.write_text(
        REFERENCE_ADAPTER + upstream_function() + r"""
void px4_reference(uint16_t first, uint16_t second, uint32_t out[34]) {
    memset(storage, 0, sizeof(storage));
    dshot_motor_data_set(0, first, false);
    dshot_motor_data_set(1, second, false);
    memcpy(out, storage, sizeof(storage));
}
""", encoding="utf-8")
    executable = build / "driver_harness.exe"
    result = subprocess.run([
        compiler, "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror", "-pedantic",
        "-I", str(ROOT / "Driver/Inc"), str(ROOT / "Driver/Src/drv_dshot.c"),
        str(reference), str(FIXTURES / "driver_harness.c"), "-o", str(executable),
    ], capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr
    return executable


@pytest.mark.parametrize("case", range(4), ids=["golden-encoding", "pulse-mapping", "px4-exhaustive", "timing-rejections"])
def test_real_driver(harness, case):
    result = subprocess.run([str(harness), str(case)], capture_output=True, text=True)
    print(result.stdout, end="")
    assert result.returncode == 0, result.stdout + result.stderr


def test_upstream_provenance_and_adapter_constants():
    source = (FIXTURES / "upstream_dshot.c.txt").read_text(encoding="utf-8")
    assert "void dshot_motor_data_set(" in upstream_function()
    for name, value in re.findall(r"#define (\w+) (\d+)u", REFERENCE_ADAPTER):
        assert re.search(rf"#define\s+{name}\s+{value}u\b", source), name
    port = (ROOT / "Driver/Src/drv_dshot.c").read_text(encoding="utf-8")
    assert "6ea3539157ca358c70a515878b77077af7d4611d" in port
    assert "Copyright (C) 2024 PX4 Development Team" in port
    assert "THIS SOFTWARE IS PROVIDED" in port


def test_driver_is_a_buildable_hardware_independent_module():
    source = (ROOT / "Driver/Src/drv_dshot.c").read_text(encoding="utf-8")
    header = (ROOT / "Driver/Inc/drv_dshot.h").read_text(encoding="utf-8")
    includes = re.findall(r'^#include\s+[<"]([^>"\n]+)', source + "\n" + header, re.M)
    assert set(includes) == {"drv_dshot.h", "stdint.h", "stddef.h", "string.h"}
    assert "Driver/Src/drv_dshot.c" in (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
    assert "HAL_" not in source
    assert "不是两个额外的 DShot 0 bit" in header

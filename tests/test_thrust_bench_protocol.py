from __future__ import annotations

import shutil
import struct
import subprocess
from pathlib import Path

import pytest

from tools.thrust_bench import fc_protocol as fc

ROOT = Path(__file__).resolve().parents[1]


def payload(*, flags=0, protocol=2, upper=1, lower=2, maximum=0,
            command=(0, 0), erpm=(0, 0), erpm_age=(0xFFFFFFFF, 0xFFFFFFFF),
            voltage_mv=0xFFFFFFFF, current_ma=-0x80000000,
            voltage_age=0xFFFFFFFF, current_age=0xFFFFFFFF,
            esc_current=(0, 0), esc_age=(0xFFFFFFFF, 0xFFFFFFFF),
            last_request_id=0) -> bytes:
    return fc._FRAME.pack(
        1, fc.PAYLOAD_BYTES, flags, 0x78563412, 0x12345678,
        protocol, upper, lower, maximum, *command, *erpm, *erpm_age,
        voltage_mv, current_ma, voltage_age, current_age,
        *esc_current, *esc_age, last_request_id,
    )


def test_v1_golden_payload_decodes_units_and_preserves_integer_ages() -> None:
    flags = (fc.FLAG_ERPM_1 | fc.FLAG_ERPM_2 | fc.FLAG_VOLTAGE |
             fc.FLAG_TOTAL_CURRENT | fc.FLAG_ESC_CURRENT_1 |
             fc.FLAG_BENCH_ACTIVE | fc.FLAG_NOT_SPINNING_2)
    raw = payload(flags=flags, maximum=20, command=(1184, 1268),
                  erpm=(123456, 0), erpm_age=(4, 8), voltage_mv=11100,
                  current_ma=12500, voltage_age=7, current_age=9,
                  esc_current=(13, 0), esc_age=(200, 0xFFFFFFFF),
                  last_request_id=42)
    snapshot = fc.decode_snapshot(raw)
    assert snapshot.nonce == 0x78563412
    assert snapshot.command_us == (1184, 1268)
    assert snapshot.erpm == (123456, 0)
    assert snapshot.erpm_age_ms == (4, 8)
    assert all(isinstance(value, int) for value in snapshot.erpm_age_ms)
    assert snapshot.voltage_v == 11.1
    assert snapshot.total_current_a == 12.5
    assert snapshot.esc_current_a == (13, None)
    assert snapshot.esc_current_age_ms == (200, None)
    assert snapshot.not_spinning == (False, True)
    assert snapshot.last_request_id == 42


@pytest.mark.parametrize("raw", [b"", b"x" * (fc.PAYLOAD_BYTES - 1),
                                  b"x" * (fc.PAYLOAD_BYTES + 1)])
def test_length_is_exact(raw: bytes) -> None:
    with pytest.raises(ValueError, match="length"):
        fc.decode_snapshot(raw)


def mutate(raw: bytes, offset: int, replacement: bytes) -> bytes:
    return raw[:offset] + replacement + raw[offset + len(replacement):]


@pytest.mark.parametrize("raw, message", [
    (mutate(payload(), 0, b"\x02"), "version"),
    (mutate(payload(), 1, b"\x3f"), "embedded length"),
    (mutate(payload(), 2, struct.pack("<H", 1 << 15)), "unknown"),
    (mutate(payload(), 12, b"\x03"), "protocol"),
    (mutate(payload(), 13, b"\x01\x01"), "mapping"),
    (payload(flags=fc.FLAG_BENCH_ACTIVE | fc.FLAG_ARMED, maximum=20,
             command=(1100, 1100)), "mutually exclusive"),
    (payload(flags=fc.FLAG_BENCH_ACTIVE, maximum=0, command=(1100, 1100)),
     "max percent"),
    (payload(flags=fc.FLAG_BENCH_ACTIVE, maximum=20, command=(1000, 1100)),
     "1100..1940"),
    (payload(flags=fc.FLAG_NOT_SPINNING_1), "not-spinning"),
    (payload(flags=fc.FLAG_ERPM_1, erpm=(0, 0), erpm_age=(1, 0)),
     "not-spinning"),
    (payload(flags=fc.FLAG_ERPM_1, erpm=(10, 0), erpm_age=(101, 0)), "stale"),
    (payload(flags=fc.FLAG_ESC_CURRENT_1, esc_age=(1001, 0)), "stale"),
])
def test_malformed_or_physically_contradictory_payloads_are_rejected(raw, message) -> None:
    with pytest.raises(ValueError, match=message):
        fc.decode_snapshot(raw)


def test_missing_values_remain_none_and_calibration_is_an_independent_fact() -> None:
    snapshot = fc.decode_snapshot(payload(flags=fc.FLAG_CURRENT_CALIBRATED))
    assert snapshot.current_calibrated
    assert snapshot.total_current_a is None
    assert snapshot.erpm == (None, None)


def test_command_helpers_match_the_firmware_text_contract() -> None:
    assert fc.snapshot_command(7) == "TBENCH? 7"
    assert fc.arm_command(20, request_id=41) == (
        "TBENCH ARM confirm=bench max_pct=20 request_id=41")
    assert fc.set_command(1.25, 20, request_id=42, window_token=41) == (
        "TBENCH SET upper_pct=1.25 lower_pct=20 request_id=42 token=41")
    assert fc.stop_command() == "TBENCH STOP"
    for bad in (-1, 101, True, 1.0):
        with pytest.raises(ValueError):
            fc.arm_command(bad, request_id=1)


@pytest.fixture()
def c_harness(tmp_path: Path) -> Path:
    compiler = shutil.which("gcc") or shutil.which("clang")
    if compiler is None:
        pytest.skip("host C compiler required")
    executable = tmp_path / "thrust_bench_harness.exe"
    include_dirs = [
        "App/Inc", "BSP/Inc", "Driver/Inc", "Services/Inc",
        "Drivers/CMSIS/RTOS2/Include", "Middlewares/Third_Party/FreeRTOS/Source/include",
        "Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F", "Core/Inc",
        "Drivers/STM32H7xx_HAL_Driver/Inc", "Drivers/CMSIS/Device/ST/STM32H7xx/Include",
        "Drivers/CMSIS/Include",
    ]
    command = [compiler, "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror", "-pedantic"]
    for directory in include_dirs:
        command.extend(("-I", str(ROOT / directory)))
    command.extend((str(ROOT / "App/Src/app_thrust_bench.c"),
                    str(ROOT / "App/Src/app_proto.c"),
                    str(ROOT / "tests/fixtures/thrust_bench/app_harness.c"),
                    "-o", str(executable)))
    built = subprocess.run(command, capture_output=True, text=True)
    assert built.returncode == 0, built.stdout + built.stderr
    return executable


def test_real_c_snapshot_encoder_matches_the_frozen_golden_frame(c_harness: Path) -> None:
    result = subprocess.run([str(c_harness)], capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr
    golden_payload = (
        "01449f04123456787856341201010214a004f40440e201000000000004"
        "000000080000005c2b0000d430000007000000090000000d000000"
        "c8000000ffffffff08000000"
    )
    golden = "24583e0035224400" + golden_payload + "eb"
    assert result.stdout.strip() == golden
    snapshot = fc.decode_snapshot(bytes.fromhex(golden_payload))
    assert snapshot.nonce == 0x78563412
    assert snapshot.command_us == (1184, 1268)
    assert snapshot.last_request_id == 8


@pytest.mark.parametrize("case", ["timeout", "inhibit", "replay", "late_set",
                                  "map_change", "multi_window_replay", "serial_wrap"])
def test_real_c_bench_window_closes_at_the_500hz_commit_gate(c_harness: Path,
                                                              case: str) -> None:
    result = subprocess.run([str(c_harness), case], capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr


def test_firmware_ack_text_and_float_set_parser_are_frozen() -> None:
    source = (ROOT / "App/Src/app_cmd_thrust_bench.c").read_text(encoding="utf-8")
    assert "TBENCH state=armed active=1 max_pct=%lu request_id=%lu token=%lu reason=-" in source
    assert "TBENCH state=set active=1 upper_pct=%u.%02u lower_pct=%u.%02u age_ms=0 request_id=%lu token=%lu" in source
    assert "TBENCH state=idle active=0 max_pct=0 stop=request" in source
    assert "app_control_parse_f32" in source


def test_tx_busy_closes_an_active_window_in_real_c(tmp_path: Path) -> None:
    compiler = shutil.which("gcc") or shutil.which("clang")
    if compiler is None:
        pytest.skip("host C compiler required")
    executable = tmp_path / "command_harness.exe"
    include_dirs = [
        "App/Inc", "BSP/Inc", "Driver/Inc", "Services/Inc",
        "Drivers/CMSIS/RTOS2/Include", "Middlewares/Third_Party/FreeRTOS/Source/include",
        "Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F", "Core/Inc",
        "Drivers/STM32H7xx_HAL_Driver/Inc", "Drivers/CMSIS/Device/ST/STM32H7xx/Include",
        "Drivers/CMSIS/Include",
    ]
    command = [compiler, "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror", "-pedantic"]
    for directory in include_dirs:
        command.extend(("-I", str(ROOT / directory)))
    command.extend((str(ROOT / "App/Src/app_cmd_thrust_bench.c"),
                    str(ROOT / "tests/fixtures/thrust_bench/command_harness.c"),
                    "-o", str(executable)))
    built = subprocess.run(command, capture_output=True, text=True)
    assert built.returncode == 0, built.stdout + built.stderr
    result = subprocess.run([str(executable)], capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr


def test_real_shared_float_parser_rejects_nan_and_infinity(tmp_path: Path) -> None:
    source = (ROOT / "App/Src/app_control.c").read_text(encoding="utf-8")
    start = source.index("uint8_t app_control_parse_f32(")
    opening = source.index("{", start)
    depth = 0
    end = opening
    for end in range(opening, len(source)):
        depth += (source[end] == "{") - (source[end] == "}")
        if depth == 0:
            break
    function = source[start:end + 1]
    harness = tmp_path / "parse_f32.c"
    harness.write_text(
        "#include <stdint.h>\n#include <stdlib.h>\n#include <math.h>\n" + function +
        '\nint main(void){float v=0;return app_control_parse_f32("nan",&v)||'
        'app_control_parse_f32("inf",&v)||app_control_parse_f32("-inf",&v)||'
        '!app_control_parse_f32("1.25",&v);}\n', encoding="utf-8")
    executable = tmp_path / "parse_f32.exe"
    built = subprocess.run([shutil.which("gcc"), "-std=c11", "-Wall", "-Wextra",
                            "-Werror", "-pedantic", str(harness), "-o", str(executable)],
                           capture_output=True, text=True)
    assert built.returncode == 0, built.stdout + built.stderr
    result = subprocess.run([str(executable)], capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr

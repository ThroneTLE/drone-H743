"""Wiring and D2-2 no-blocking contracts for the magnetometer fusion task.

Mirrors the source-scan idiom already used in
tests/test_control_loop_blocking_contract.py::test_no_blocking (a module
running in the 500Hz control loop must not call the blocking USB/UART text
path or anything else that can stall) and the brace-matching function-body
extraction helper used throughout that file, scoped here to the specific
functions this task touches rather than a whole-file scan -- unlike
app_servo_cal.c, app_stabilizer.c and app_mag.c are multi-purpose files with
plenty of legitimately-blocking code outside the 500Hz path.
"""
from __future__ import annotations

import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
STABILIZER = ROOT / "App" / "Src" / "app_stabilizer.c"
APP_MAG = ROOT / "App" / "Src" / "app_mag.c"
CMAKE = ROOT / "CMakeLists.txt"

# Anything on this list appearing inside the 500Hz stabilizer_imu_step() body
# (beyond the pre-existing calls already there) would violate D2-2: Flash,
# blocking text output, RTOS delays, or blocking bus I/O.
BLOCKING_SYMBOLS = (
    "APP_Control_QueueText(",
    "SVC_Param_",
    "APP_FlashService_",
    "APP_ControlConfigStore_",
    "osDelay(",
    "vTaskDelay(",
    "SVC_Timestamp_BusyWaitMs(",
    "BSP_MAG_Read(",
    "BSP_I2C_",
)


def _c_function_body(source: str, name: str) -> str:
    pattern = re.compile(
        rf"^(?:static\s+)?[A-Za-z_][A-Za-z0-9_ \*]*\b{re.escape(name)}\s*\(",
        re.MULTILINE,
    )
    matches = list(pattern.finditer(source))
    assert matches, f"{name} not found"
    match = matches[-1]
    brace = source.index("{", match.start())
    depth = 0
    for index in range(brace, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[brace : index + 1]
    raise AssertionError(f"unterminated C function: {name}")


def test_stabilizer_reads_the_mag_snapshot_with_the_three_app_side_gates() -> None:
    source = STABILIZER.read_text(encoding="utf-8")
    body = _c_function_body(source, "stabilizer_imu_step")

    assert "APP_MAG_GetSnapshot(&mag_snapshot)" in body
    # Gate 1 (calibrated), gate 2 (axis_verified) and gate 3 (freshness) --
    # the fourth (field magnitude) is Driver-layer, decided inside
    # DRV_AttitudeFusion_Update() via DRV_MAG_FieldMagnitude_InRange().
    assert "mag_snapshot.calibrated != 0U" in body
    assert "mag_snapshot.axis_verified != 0U" in body
    assert "mag_age_us <= APP_MAG_SNAPSHOT_MAX_AGE_US" in body
    assert "fusion_input.magnetometer_valid = 1U" in body
    # magnetometer_valid must default to 0 (struct is zero-initialised) and
    # only ever be set inside the gated branch -- never assigned
    # unconditionally.
    assert body.count("fusion_input.magnetometer_valid = 1U") == 1
    assert "fusion_input.magnetometer_valid = 0U" not in body


def test_stabilizer_imu_step_has_no_new_blocking_calls() -> None:
    source = STABILIZER.read_text(encoding="utf-8")
    body = _c_function_body(source, "stabilizer_imu_step")
    for symbol in BLOCKING_SYMBOLS:
        assert symbol not in body, (
            f"stabilizer_imu_step runs on the 500Hz control path; "
            f"{symbol} can block and must not appear there (D2-2)"
        )


def test_mag_snapshot_getter_is_a_pure_critical_section_copy() -> None:
    source = APP_MAG.read_text(encoding="utf-8")
    body = _c_function_body(source, "APP_MAG_GetSnapshot")

    assert "BSP_Critical_Enter()" in body
    assert "BSP_Critical_Exit(lock)" in body
    for symbol in BLOCKING_SYMBOLS + ("BSP_MAG_GetStatus(",):
        assert symbol not in body, (
            f"APP_MAG_GetSnapshot() is called from the 500Hz control path "
            f"via app_stabilizer.c; it must stay a pure critical-section "
            f"copy with no I/O ({symbol} found)"
        )


def test_mag_snapshot_production_applies_rotation_then_calibration() -> None:
    """Contract 4 pipeline order: svc_mag rotation, then drv_mag_calibration
    correction -- the host-side ellipsoid fit (tools/mag_cal_fit.py) must be
    captured in the same (already-FLU-rotated) frame the firmware applies
    its coefficients in."""
    source = APP_MAG.read_text(encoding="utf-8")
    body = _c_function_body(source, "app_mag_update_snapshot")

    rotate_at = body.index("SVC_MAG_RotateToFlu(")
    calibrate_at = body.index("DRV_MAG_Calibration_Apply(")
    assert rotate_at < calibrate_at


def test_cmake_lists_the_stage_and_upstream_new_sources() -> None:
    source = CMAKE.read_text(encoding="utf-8")
    for path in (
        "Driver/Src/drv_mag_calibration.c",
        "Services/Src/svc_mag.c",
        "App/Src/app_cmd_magcal.c",
    ):
        assert path in source, f"{path} must be built into the firmware target"


def test_app_control_c_was_not_touched_by_this_task() -> None:
    """Hard constraint: App/Src/app_control.c is only-decrease. This task's
    own contribution must not mention any of its new symbols there --
    new commands are wired through app_cmd_fallback.c instead."""
    source = (ROOT / "App" / "Src" / "app_control.c").read_text(encoding="utf-8")
    for needle in ("MAGCAL", "MAGFRAME", "app_cmd_magcal", "APP_MagCal_"):
        assert needle not in source


def test_new_command_family_is_wired_through_the_fallback_chain_not_app_control() -> None:
    fallback = (ROOT / "App" / "Src" / "app_cmd_fallback.c").read_text(
        encoding="utf-8"
    )
    assert "app_control_handle_magcal(tokens, count)" in fallback

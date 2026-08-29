"""IMU frame correction persistence and command-safety contract."""

from __future__ import annotations

import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CONTROL = ROOT / "App" / "Src" / "app_control.c"
PROTO = ROOT / "App" / "Inc" / "app_proto.h"


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def function_body(source: str, name: str) -> str:
    match = re.search(re.escape(name) + r"\s*\([^;{]*\)\s*\{", source)
    if match is None:
        raise AssertionError(f"missing function definition {name}")
    brace = source.index("{", match.start())
    depth = 0
    for index in range(brace, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[brace : index + 1]
    raise AssertionError(f"unterminated function {name}")


def dispatch_branch(source: str, command: str, next_command: str) -> str:
    start = source.index(f'}} else if (strcmp(tokens[0], "{command}") == 0)')
    end = source.index(
        f'}} else if (strcmp(tokens[0], "{next_command}") == 0)', start
    )
    return source[start:end]


def test_protocol_reserves_a_stable_imuframe_request_id() -> None:
    proto = read(PROTO)

    assert "#define APP_PROTO_REQ_IMU_FRAME      0x101EU" in proto


def test_orientation_lives_in_versioned_fcal_aggregate() -> None:
    source = read(CONTROL)

    assert '#include "svc_param.h"' in source
    assert '#include "app_flight_calibration.h"' in source
    assert "APP_ControlImuFrameParam" not in source
    assert "APP_FlightCalibration" in source

    sync = function_body(source, "static void app_control_imuframe_sync_param")
    assert "SVC_Param_GetGeneration()" in sync
    assert "SVC_Param_GetBlob(" in sync
    assert "APP_FlightCalibration_Decode" in sync
    assert "APP_Sensor_SetFluOrientationCode" in sync
    assert "control_imuframe_boot_selection_pending" in sync


def test_ascii_commands_support_safe_temporary_apply_revert_and_commit() -> None:
    source = read(CONTROL)
    handler = function_body(source, "static void app_control_handle_imuframe")
    safety = function_body(source, "static uint8_t app_control_imuframe_is_safe")

    assert 'strcmp(tokens[0], "IMUFRAME?") == 0' in source
    assert 'strcmp(tokens[0], "IMUFRAME") == 0' in source
    for operation in ("APPLY", "REVERT", "COMMIT"):
        assert f'strcmp(tokens[1], "{operation}")' in handler

    assert "APP_Stabilizer_ReadValidationImuSnapshot" in safety
    assert "snapshot.armed != 0U" in safety
    assert "snapshot.esc_pulse_us[0] > 1100U" in safety
    assert "snapshot.esc_pulse_us[1] > 1100U" in safety
    assert handler.count("app_control_imuframe_is_safe()") >= 3

    apply = handler[handler.index('strcmp(tokens[1], "APPLY")') :]
    assert "APP_Sensor_SetFluOrientation(tokens[2])" in apply
    apply = apply[: apply.index('strcmp(tokens[1], "REVERT")')]
    assert "SVC_Param_SetBlob" not in apply
    assert apply.index("SVC_Param_IsDirty()") < apply.index(
        "APP_Sensor_SetFluOrientation(tokens[2])"
    )

    commit = handler[handler.index('strcmp(tokens[1], "COMMIT")') :]
    assert "APP_Sensor_GetFluOrientation()" in commit
    assert "SVC_Param_SetBlob" in commit
    assert "SVC_Param_RequestSaveBlob()" in commit
    assert "APP_FlightCalibration_Decode" in commit
    assert "APP_FlightCalibration_UpdateOrientation" in commit
    assert "APP_FlightCalibration_Encode" in commit
    new_commit = commit[commit.index("memset(blob") :]
    assert new_commit.index("SVC_Param_SetBlob(encoded, encoded_size)") < new_commit.index(
        "SVC_Param_RequestSaveBlob()"
    )
    assert "control_imuframe_pending_code = active_code;" in commit
    assert "control_imuframe_pending_valid = 1U;" in commit
    assert "control_imuframe_confirmed_code = active_code;" not in commit

    revert = handler[handler.index('strcmp(tokens[1], "REVERT")') :]
    revert = revert[: revert.index('strcmp(tokens[1], "COMMIT")')]
    assert 'app_control_report_imuframe("revert_busy", 0U)' in revert
    assert "control_imuframe_confirmed_code" in revert
    assert revert.index("SVC_Param_IsDirty()") < revert.index(
        "APP_Sensor_SetFluOrientationCode"
    )


def test_status_is_machine_parseable_and_distinguishes_candidate_from_flash() -> None:
    source = read(CONTROL)
    report = function_body(source, "static void app_control_report_imuframe")

    expected_fields = (
        "event=%s",
        "base=legacy_intermediate_v1",
        "active=%s",
        "active_code=%u",
        "persisted=%s",
        "persisted_code=%u",
        "pending_code=%u",
        "pending_valid=%u",
        "temporary=%u",
        "dirty=%u",
        "frame=%s",
        "arm_lock=%u",
        "request=%lu",
    )
    for field in expected_fields:
        assert field in report
    assert "APP_Stabilizer_IsImuFrameArmLocked()" in report
    assert "APP_Sensor_GetFluOrientationDescriptorForCode" in report
    assert "control_imuframe_confirmed_code" in report
    assert "control_imuframe_pending_valid" in report
    for frame in (
        '"legacy_intermediate"',
        '"canonical_flu_candidate"',
        '"canonical_flu_persisted"',
    ):
        assert frame in report
    for event in ('"status"', '"applied"', '"reverted"', '"commit_queued"'):
        assert event in source

    worst_case = (
        "IMUFRAME event=commit_queue_failed base=legacy_intermediate_v1 "
        "active=-z,-y,-x active_code=255 persisted=-z,-y,-x "
        "persisted_code=255 pending_code=255 pending_valid=1 temporary=1 "
        "dirty=1 frame=canonical_flu_candidate arm_lock=1 "
        "request=4294967295\r\n"
    )
    assert len(worst_case.encode("ascii")) < 256


def test_legacy_config_commands_cannot_change_active_orientation() -> None:
    source = read(CONTROL)

    for command, next_command in (
        ("SAVE", "LOAD"),
        ("LOAD", "DEFAULTS"),
        ("DEFAULTS", "PARAM"),
    ):
        branch = dispatch_branch(source, command, next_command)
        assert "APP_Sensor_SetFluOrientation" not in branch
        assert "SVC_Param_" not in branch


def test_runtime_tick_observes_param_generation_without_overwriting_temp_apply() -> None:
    source = read(CONTROL)
    init = function_body(source, "void APP_Control_Init")
    tick = function_body(source, "static void app_control_tick_common")
    sync = function_body(source, "static void app_control_imuframe_sync_param")

    assert "app_control_imuframe_sync_param();" in init
    assert "app_control_imuframe_sync_param();" in tick
    assert re.search(
        r"if\s*\(control_imuframe_boot_selection_pending\s*!=\s*0U\).*?"
        r"APP_Sensor_SetFluOrientationCode",
        sync,
        re.DOTALL,
    )
    dirty_return = sync.index("if (dirty != 0U)")
    confirmed_assignment = sync.index(
        "control_imuframe_confirmed_code = orientation_code;"
    )
    assert dirty_return < confirmed_assignment
    assert "control_imuframe_last_dirty" in sync
    assert "control_imuframe_pending_valid = 0U;" in sync


def test_descriptor_lookup_has_one_owner_in_the_sensor_api() -> None:
    control = read(CONTROL)

    assert "control_imuframe_descriptors" not in control
    assert "APP_Sensor_GetFluOrientationDescriptorForCode" in control

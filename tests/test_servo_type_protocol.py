"""S7 servo output type persistence and command protocol contracts."""

from pathlib import Path
import shutil
import subprocess

import pytest


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def test_servo_type_reuses_reserved_fcal_byte_without_abi_growth() -> None:
    header = read("App/Inc/app_flight_calibration.h")
    source = read("App/Src/app_flight_calibration.c")

    assert "APP_FLIGHT_CAL_VALID_SERVO_TYPE" in header
    assert "APP_FLIGHT_CAL_VALID_MASK_SUPPORTED 0x3FU" in header
    assert "APP_FLIGHT_CAL_V1_VALID_MASK_SUPPORTED 0x0FU" in header
    assert "v2_actuator_mapping" in header
    assert "APP_FlightCalibration_UpdateServoType" in header
    assert "APP_FlightCalibration_BuildServoType" in header
    assert "sizeof(APP_FlightCalibration) == 160U" in source
    assert "APP_SERVO_TYPE_DEFAULT" in source


def test_servo_type_runtime_module_defaults_unset_records_to_pwm() -> None:
    """Author 2026-09-25: PWM servos (MOTOR7/8) unless FCAL explicitly says bus; record codes unchanged."""
    header = read("App/Inc/app_servo_type.h")
    source = read("App/Src/app_servo_type.c")

    assert "APP_SERVO_TYPE_BUS = 0U" in header
    assert "APP_SERVO_TYPE_PWM = 1U" in header
    assert "#define APP_SERVO_TYPE_DEFAULT APP_SERVO_TYPE_PWM" in header
    assert "APP_ServoType_ResetActive" in header
    assert "APP_ServoType_PublishActive" in header
    assert "APP_ServoType_GetActive" in header
    assert 'return "bus";' in source
    assert 'return "pwm";' in source


def test_servotype_protocol_is_single_value_apply_revert_commit() -> None:
    source = read("App/Src/app_cmd_servotype.c")
    internal = read("App/Inc/app_control_internal.h")
    control = read("App/Src/app_control.c")

    for command in (
        "SERVOTYPE?",
        "SERVOTYPE APPLY type=bus|pwm",
        "SERVOTYPE REVERT",
        "SERVOTYPE COMMIT",
    ):
        assert command in source
    for field in (
        "active=%s",
        "persisted=%s",
        "dirty=%u",
        "valid=%u",
        "explicit=%u",
        "generation=%lu",
        "record_generation=%lu",
        "request=%lu",
    ):
        assert field in source
    assert "app_control_handle_servotype" in internal
    assert 'strcmp(tokens[0], "SERVOTYPE?")' in control
    assert 'strcmp(tokens[0], "SERVOTYPE")' in control


def test_servotype_uses_param_blob_and_has_stable_protocol_ids() -> None:
    source = read("App/Src/app_cmd_servotype.c")
    proto = read("App/Inc/app_proto.h")
    cmake = read("CMakeLists.txt")
    caps = read("App/Src/app_cmd_system.c")

    assert "SVC_Param_SetBlob" in source
    assert "SVC_Param_RequestSaveBlob" in source
    assert "APP_FlightCalibration_Encode" in source
    assert "APP_PROTO_REQ_SERVOTYPE" in proto
    assert "#define APP_PROTO_REQ_SERVO_CAL      0x1024U" in proto
    assert "#define APP_PROTO_REQ_SERVOTYPE      0x1025U" in proto
    assert "APP_PROTO_MSG_SERVO_TYPE" in proto
    assert "App/Src/app_servo_type.c" in cmake
    assert "App/Src/app_cmd_servotype.c" in cmake
    assert "SERVOTYPE?" in caps


def test_servotype_write_path_is_disarmed_and_does_not_modify_servocal() -> None:
    source = read("App/Src/app_cmd_servotype.c")
    servocal = read("App/Src/app_cmd_servocal.c")

    assert "APP_Stabilizer_IsArmed()" in source
    assert "armed_blocked" in source
    assert "app_cmd_servocal_is_busy" in source
    assert "SERVOTYPE" not in servocal


def test_servo_type_fcal_round_trip_and_runtime_selection(tmp_path: Path) -> None:
    compiler = shutil.which("gcc") or shutil.which("clang")
    if compiler is None:
        pytest.skip("host C compiler is unavailable")
    (tmp_path / "app_sensor.h").write_text(
        "#define APP_SENSOR_FLU_ORIENTATION_COUNT 24U\n"
        "#define APP_SENSOR_FLU_ORIENTATION_LEGACY 255U\n",
        encoding="ascii",
    )
    (tmp_path / "drv_frame_contract.h").write_text(
        "#define DRV_FRAME_CONTRACT_VERSION 1U\n", encoding="ascii"
    )
    harness = tmp_path / "servo_type_round_trip.c"
    executable = tmp_path / "servo_type_round_trip.exe"
    harness.write_text(
        r'''
#include "app_flight_calibration.h"
#include "app_servo_type.h"
#include <assert.h>
#include <stdint.h>

int main(void) {
    APP_FlightCalibration source;
    APP_FlightCalibration decoded;
    APP_ServoType type = APP_SERVO_TYPE_PWM;
    uint8_t bytes[sizeof(APP_FlightCalibration)];

    APP_FlightCalibration_Defaults(&source);
    assert(sizeof(source) == 160U);
    type = APP_SERVO_TYPE_BUS;
    assert(APP_FlightCalibration_BuildServoType(&source, &type) == 0U);
    assert(type == APP_SERVO_TYPE_PWM);  /* never set: the PWM default */
    {
        APP_FlightCalibration bus_record = source;
        APP_ServoType bus_type = APP_SERVO_TYPE_PWM;
        assert(APP_FlightCalibration_UpdateServoType(&bus_record, APP_SERVO_TYPE_BUS) == 1U);
        assert(APP_FlightCalibration_BuildServoType(&bus_record, &bus_type) == 1U);
        assert(bus_type == APP_SERVO_TYPE_BUS);  /* an explicit bus record stays bus */
    }
    assert(APP_FlightCalibration_UpdateServoType(&source, APP_SERVO_TYPE_PWM) == 1U);
    assert((source.valid_mask & APP_FLIGHT_CAL_VALID_SERVO_TYPE) != 0U);
    assert(APP_FlightCalibration_BuildServoType(&source, &type) == 1U);
    assert(type == APP_SERVO_TYPE_PWM);
    assert(APP_FlightCalibration_Encode(&source, bytes, sizeof(bytes)) == 160U);
    assert(APP_FlightCalibration_Decode(bytes, sizeof(bytes), &decoded) ==
           APP_FLIGHT_CAL_DECODE_CURRENT);
    assert(APP_FlightCalibration_BuildServoType(&decoded, &type) == 1U);
    assert(type == APP_SERVO_TYPE_PWM);

    APP_ServoType_ResetActive();
    assert(APP_ServoType_GetActive() == APP_SERVO_TYPE_PWM);
    assert(APP_ServoType_PublishActive(APP_SERVO_TYPE_PWM) == 1U);
    assert(APP_ServoType_GetActive() == APP_SERVO_TYPE_PWM);
    assert(APP_ServoType_GetGeneration() == 1U);
    assert(APP_ServoType_FromName("bus", &type) == 1U);
    assert(type == APP_SERVO_TYPE_BUS);
    return 0;
}
''',
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
            f"-I{ROOT / 'App' / 'Inc'}",
            f"-I{ROOT / 'Driver' / 'Inc'}",
            str(ROOT / "App/Src/app_flight_calibration.c"),
            str(ROOT / "App/Src/app_servo_type.c"),
            str(harness),
            "-lm",
            "-o",
            str(executable),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    subprocess.run([str(executable)], check=True)

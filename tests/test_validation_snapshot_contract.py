from __future__ import annotations

import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
HEADER = ROOT / "App" / "Inc" / "app_stabilizer.h"
STABILIZER = ROOT / "App" / "Src" / "app_stabilizer.c"
CONTROL = ROOT / "App" / "Src" / "app_control.c"
FRAME_CONTRACT = ROOT / "Driver" / "Inc" / "drv_frame_contract.h"


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def function_body(source: str, signature: str) -> str:
    start = source.index(signature)
    brace = source.index("{", start)
    depth = 0
    for index in range(brace, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[brace + 1 : index]
    raise AssertionError(f"unterminated function: {signature}")


def test_public_snapshot_contains_the_read_only_validation_payload() -> None:
    header = read(HEADER)

    assert "typedef struct {" in header
    assert "} StabilizerValidationImuSnapshot;" in header
    for field in (
        "uint64_t timestamp_us;",
        "uint32_t sequence;",
        "uint32_t sample_count;",
        "float accel_g[3];",
        "float gyro_dps[3];",
        "float temperature_c;",
        "float roll_deg;",
        "float pitch_deg;",
        "float yaw_deg;",
        "float fusion_acceleration_error_deg;",
        "float fusion_acceleration_recovery_trigger;",
        "uint32_t fusion_accel_correction_count;",
        "uint16_t esc_pulse_us[2];",
        "uint8_t gyro_bias_ready;",
        "uint8_t fusion_accelerometer_ignored;",
        "uint8_t fusion_acceleration_recovery;",
        "uint8_t fusion_angular_rate_recovery;",
        "uint8_t fusion_accel_norm_rejected;",
        "uint8_t armed;",
    ):
        assert field in header
    assert "uint8_t APP_Stabilizer_ReadValidationImuSnapshot(" in header


def test_snapshot_is_published_after_the_legacy_pipeline_annotations() -> None:
    source = read(STABILIZER)
    publish = function_body(source, "static void stabilizer_validation_imu_publish(")
    imu_step = function_body(source, "static void stabilizer_imu_step(")

    assert "msg->base.type != APP_SENSOR_TYPE_IMU" in publish
    for assignment in (
        "next.timestamp_us = msg->base.timestamp_us;",
        "next.sequence = msg->base.sequence;",
        "next.sample_count = msg->base.sequence;",
        "next.accel_g[0] = msg->imu.accel_x_g;",
        "next.gyro_dps[0] = msg->imu.gyro_x_dps;",
        "next.roll_deg = msg->roll_deg;",
        "next.gyro_bias_ready = msg->gyro_bias_ready;",
        "msg->fusion_acceleration_error_deg",
        "msg->fusion_accelerometer_ignored",
        "next.armed = stabilizer_capture_armed;",
        "BSP_PWM_GetEscPulse(1U)",
        "BSP_PWM_GetEscPulse(2U)",
    ):
        assert assignment in publish

    annotate_at = imu_step.index("msg->yaw_deg   = ctx->yaw_control;")
    publish_at = imu_step.index("stabilizer_validation_imu_publish(msg,")
    queue_at = imu_step.index("osMessageQueuePut(s_vofa_q, msg")
    assert annotate_at < publish_at < queue_at


def test_snapshot_uses_a_bounded_dmb_seqlock_and_a_real_valid_gate() -> None:
    source = read(STABILIZER)
    reset = function_body(source, "static void stabilizer_validation_imu_reset(")
    publish = function_body(source, "static void stabilizer_validation_imu_publish(")
    reader = function_body(
        source, "uint8_t APP_Stabilizer_ReadValidationImuSnapshot("
    )

    assert "static volatile uint32_t stabilizer_validation_imu_seqlock;" in source
    assert "static volatile uint8_t stabilizer_validation_imu_valid;" in source
    assert reset.count("stabilizer_validation_imu_seqlock++;") == 2
    assert "stabilizer_validation_imu_valid = 0U;" in reset
    assert reset.count("__DMB();") >= 2
    assert publish.count("stabilizer_validation_imu_seqlock++;") == 2
    assert "stabilizer_validation_imu_valid = 1U;" in publish
    assert publish.count("__DMB();") >= 2
    assert "STABILIZER_VALIDATION_SNAPSHOT_READ_RETRIES" in reader
    assert "if ((before & 1U) != 0U)" in reader
    assert "(before == after) && ((after & 1U) == 0U)" in reader
    assert reader.count("__DMB();") >= 2
    assert reader.index("if (valid == 0U)") < reader.index("*out = current;")
    assert "return 1U;" in reader
    assert reader.rstrip().endswith("return 0U;")


def test_imu_query_keeps_hardware_state_but_never_emits_a_fake_zero_sample() -> None:
    source = read(CONTROL)
    report = function_body(source, "static void app_control_report_imu(void)")

    assert "APP_IMU_GetStatus(&imu_status);" in report
    assert "IMU ok=%u stage=%s stage_id=%u" in report
    assert "APP_Stabilizer_ReadValidationImuSnapshot(&snapshot)" in report
    assert "if (snapshot_valid == 0U)" in report
    assert (
        "IMU sample valid=0 source=stabilizer_snapshot unavailable" in report
    )
    unavailable = report[
        report.index("if (snapshot_valid == 0U)") : report.index("} else {")
    ]
    assert "ax=" not in unavailable
    assert "IMU scaled ax_mg=" not in report


def test_valid_imu_protocol_is_provenanced_and_remains_legacy() -> None:
    control = read(CONTROL)
    report = function_body(control, "static void app_control_report_imu(void)")
    frame_contract = read(FRAME_CONTRACT)

    assert '#include "drv_frame_contract.h"' in control
    for token in (
        "valid=1",
        "source=stabilizer_snapshot",
        "frame=legacy_intermediate",
        "units=mg_mdps_cdeg",
        "contract=%u",
        "migration=0x%02lX",
        "ts_ms=%lu",
        "seq=%lu",
        "bias=%u",
        "armed=%u",
        "m1=%u",
        "m2=%u",
        "fusion_flags=0x%02X",
        "ferr_cdeg=%ld",
    ):
        assert token in report
    assert "DRV_FRAME_CONTRACT_VERSION" in report
    assert "DRV_FRAME_RUNTIME_MIGRATION_DONE_MASK" in report
    assert report.count("seq=%lu") == 2
    assert re.search(
        r"^#define\s+DRV_FRAME_RUNTIME_MIGRATION_DONE_MASK\s+0U\s*$",
        frame_contract,
        re.MULTILINE,
    )
    assert "#define DRV_FRAME_RUNTIME_MIGRATION_DONE_MASK" not in control
    assert "#define DRV_FRAME_RUNTIME_MIGRATION_DONE_MASK" not in read(STABILIZER)
    assert "%llu" not in report


def test_validation_query_is_read_only_and_protocol_lines_fit_the_buffer() -> None:
    stabilizer = read(STABILIZER)
    control = read(CONTROL)
    publish = function_body(stabilizer, "static void stabilizer_validation_imu_publish(")
    report = function_body(control, "static void app_control_report_imu(void)")

    for forbidden in (
        "SVC_Param_",
        "APP_Flash",
        "APP_Background",
        "SetEscPulse",
        "DisableEsc",
        "DRV_Motor_Set",
    ):
        assert forbidden not in publish
        assert forbidden not in report

    provenance_worst_case = (
        "IMU sample valid=1 source=stabilizer_snapshot "
        "frame=legacy_intermediate units=mg_mdps_cdeg "
        "contract=4294967295 migration=0xFFFFFFFF "
        "ts_ms=4294967295 seq=4294967295 bias=1 armed=1 "
        "m1=65535 m2=65535 temp_cdeg=-2147483648\r\n"
    )
    data_worst_case = (
        "IMU sample seq=4294967295 "
        "ax=-2147483648 ay=-2147483648 az=-2147483648 "
        "gx=-2147483648 gy=-2147483648 gz=-2147483648 "
        "roll=-2147483648 pitch=-2147483648 yaw=-2147483648 "
        "fusion_flags=0xFF ferr_cdeg=-2147483648 "
        "ftrig_milli=-2147483648 fcorr=4294967295\r\n"
    )
    assert len(provenance_worst_case.encode("ascii")) < 256
    assert len(data_worst_case.encode("ascii")) < 256

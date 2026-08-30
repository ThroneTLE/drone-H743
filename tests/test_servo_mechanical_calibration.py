"""Persistent servo-mechanical calibration and flow-compensation evidence."""

from __future__ import annotations

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def test_servo_mechanical_fields_reuse_fcal_reserved_space_without_abi_growth() -> None:
    header = read("App/Inc/app_flight_calibration.h")
    source = read("App/Src/app_flight_calibration.c")
    assert "APP_FLIGHT_CAL_VALID_SERVO_MECHANICAL (1U << 4)" in header
    for field in (
        "servo_center_us", "servo_min_us", "servo_max_us", "servo_pulse_sign"
    ):
        assert field in header
    assert "uint32_t v2_reserved[3];" in header
    assert "sizeof(APP_FlightCalibration) == 160U" in source
    assert "APP_FlightCalibration_UpdateServoMechanical" in source
    assert "APP_FlightCalibration_BuildServoMechanical" in source


def test_servocal_protocol_has_preview_revert_commit_and_param_confirmation() -> None:
    control = read("App/Src/app_cmd_servocal.c")
    proto = read("App/Inc/app_proto.h")
    stabilizer = read("App/Src/app_stabilizer.c")
    assert "APP_PROTO_REQ_SERVO_CAL      0x1024U" in proto
    assert "APP_PROTO_MSG_SERVO_CAL         0x2225U" in proto
    for command in ("SERVOCAL?", 'strcmp(tokens[1], "APPLY")',
                    'strcmp(tokens[1], "REVERT")', 'strcmp(tokens[1], "COMMIT")'):
        assert command in control
    assert "APP_FlightCalibration_PublishPreview" in control
    assert "SVC_Param_SetBlob(encoded, encoded_size)" in control
    assert "SVC_Param_RequestSaveBlob" in control
    assert "control_servocal_pending_record" in control
    assert "record_mismatch" in control
    assert "APP_Stabilizer_SetServoCalibrationCandidateArmLock(1U)" in control
    assert "stabilizer_servo_calibration_candidate_arm_lock" in stabilizer


def test_runtime_consumers_use_the_same_servo_calibration() -> None:
    driver = read("Driver/Src/drv_coax_ctrl.c")
    stabilizer = read("App/Src/app_stabilizer.c")
    acceptance = read("App/Src/app_acceptance.c")
    ident = read("App/Src/app_ident.c")
    control = read("App/Src/app_control.c")
    assert "coax_ctrl_servo_calibration.center_us" in driver
    assert "coax_ctrl_servo_calibration.pulse_sign" in driver
    assert "APP_FlightCalibration_BuildServoMechanical" in stabilizer
    assert "DRV_COAX_CTRL_SetServoCalibration" in stabilizer
    for source in (acceptance, ident, control):
        assert "DRV_COAX_CTRL_GetServoCalibration" in source
    assert "APP_FLIGHT_CAL_VALID_SERVO_MECHANICAL" in control


def test_flow_compensation_snapshot_is_coherent_and_exported_as_flu() -> None:
    header = read("App/Inc/app_stabilizer.h")
    stabilizer = read("App/Src/app_stabilizer.c")
    flow_cmd = read("App/Src/app_cmd_flow.c")
    panel = read("tools/panel_lib/pages/flow_ranging.py")
    assert "StabilizerFlowCompensationSnapshot" in header
    assert "APP_Stabilizer_ReadFlowCompensationSnapshot" in header
    assert "stabilizer_flow_comp_seqlock" in stabilizer
    assert "__DMB();" in stabilizer
    assert "sensor_velocity_flu_m_s[1] = -debug->sensor_velocity_m_s[1]" in stabilizer
    assert "corrected_velocity_flu_m_s[1] = -debug->corrected_velocity_m_s[1]" in stabilizer
    assert "source=controller_legacy_x_forward_y_right export=canonical_flu" in flow_cmd
    assert "corr_vx_mm_s" in flow_cmd
    assert "vx_compensated_m_s" in panel
    assert 'self.flow_diag_values.get("export") != "canonical_flu"' in panel


def test_panel_only_marks_servo_target_written_after_persisted_match() -> None:
    mechanical = read("tools/panel_lib/pages/mechanical.py")
    assert "应用到 RAM" in mechanical
    assert "撤销 RAM 预览" in mechanical
    assert "写入参数 Flash" in mechanical
    assert "重启后核对" in mechanical
    assert '_mechanical_target_matches_local("persisted")' in mechanical
    assert '"target_parameters_written": persisted_match' in mechanical
    assert '"reboot_verification_passed": self.mechanical_reboot_verified' in mechanical

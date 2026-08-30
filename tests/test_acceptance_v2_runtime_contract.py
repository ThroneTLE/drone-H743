from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def test_v2a_lease_and_esc_fail_safe_are_target_owned() -> None:
    header = read("App/Inc/app_acceptance.h")
    source = read("App/Src/app_acceptance.c")
    control = read("App/Src/app_control.c")
    stabilizer = read("App/Src/app_stabilizer.c")

    assert "APP_ACCEPTANCE_LEASE_MS 500U" in header
    assert "now_ms + APP_ACCEPTANCE_LEASE_MS" in source
    assert "APP_Acceptance_Stop();" in source[source.index(
        "void APP_Acceptance_Service"):]
    assert "BSP_PWM_DisableEsc(1U)" in source
    assert "BSP_PWM_DisableEsc(2U)" in source
    assert "BSP_PWM_GetEscPulse(1U) != 0U" in source
    assert "APP_Acceptance_IsActive() != 0U" in stabilizer
    acceptance_output = stabilizer[stabilizer.index(
        "if (APP_Acceptance_IsActive() != 0U) {", stabilizer.index(
            "static void stabilizer_control_commit")):]
    assert acceptance_output.index("BSP_PWM_DisableEsc(1U)") < acceptance_output.index(
        "frame->servo_cal_active")
    assert "stabilizer_rc_arm_latched = 0U" in stabilizer
    assert "control_imucal_applied != 0U" in control
    assert "APP_Stabilizer_IsImuCalibrationCandidateArmLocked" in control


def test_v2a_stage_names_match_offline_engine_and_servo_delta_is_bounded() -> None:
    source = read("App/Src/app_acceptance.c")
    engine = read("tools/flight_acceptance_v2.py")
    for stage in (
        "rc_center", "rc_positive_roll", "rc_positive_pitch", "rc_positive_yaw",
        "nav_static", "nav_forward", "nav_left", "restore_positive_roll",
        "restore_positive_pitch", "servo_alpha_positive_50us",
        "servo_alpha_negative_50us", "servo_beta_positive_50us",
        "servo_beta_negative_50us", "failsafe",
    ):
        assert f'"{stage}"' in source
        assert stage in engine
    assert "APP_ACCEPTANCE_SERVO_DELTA_US    50U" in source
    assert "DRV_COAX_CTRL_GetServoCalibration(&calibration);" in source
    assert "calibration.pulse_sign[DRV_COAX_CTRL_SERVO_ALPHA_INDEX]" in source
    assert "APP_ACCEPTANCE_SERVO_CENTER_US" not in source


def test_v2a_protocol_is_explicit_and_panel_keeps_lease_alive() -> None:
    control = read("App/Src/app_control.c")
    panel = read("tools/panel_lib/pages/acceptance_v2.py")
    proto = read("App/Inc/app_proto.h")
    assert "APP_PROTO_REQ_ACCEPTANCE     0x1021U" in proto
    assert "APP_PROTO_MSG_ACCEPTANCE        0x2222U" in proto
    for word in ("START", "KEEPALIVE", "STAGE", "STOP"):
        assert f'strcmp(tokens[2], "{word}")' in control
    assert '"ACCEPT V2 START props=1"' in panel
    assert "ACCEPT V2 KEEPALIVE lease=" in panel
    assert "ACCEPT V2 STAGE name=" in panel
    assert "不构成自由飞行放行" in panel

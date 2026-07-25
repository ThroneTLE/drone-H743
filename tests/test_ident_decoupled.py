from __future__ import annotations

import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def load_panel_module():
    path = ROOT / "tools" / "drone_tcp_panel.py"
    spec = importlib.util.spec_from_file_location("drone_tcp_panel", path)
    assert spec is not None
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_ident_control_payload_and_decoupled_servo_takeover() -> None:
    app_aiwb2 = read("App/Src/app_aiwb2.c")
    freertos = read("Core/Src/freertos.c")
    ident = read("App/Src/app_ident.c")

    assert 'strcmp(line, "IDENT?") == 0' in app_aiwb2
    assert 'aiwb2_starts_with(line, "IDENT ")' in app_aiwb2
    assert "ident_running = APP_Ident_IsRunning();" in freertos
    assert "APP_Ident_GetServoTargets(&ident_alpha_us, &ident_beta_us);" in freertos
    assert "DRV_COAX_CTRL_Run(&attitude, &reference, &ctrl_out);" in freertos
    assert "BSP_PWM_SetEscPulse" not in ident
    assert "DRV_Motor" not in ident
    assert "if (ident_ctx.axis == APP_IDENT_AXIS_ROLL) {\n        alpha += offset;" in ident
    assert "} else {\n        beta += offset;" in ident


def test_ident_commands_exist_and_are_text_based() -> None:
    app_control = read("App/Src/app_control.c")
    ident_h = read("App/Inc/app_ident.h")

    assert "APP_Ident_StartStep" in ident_h
    assert "APP_Ident_StartDoublet" in ident_h
    assert "APP_Ident_StartPrbs" in ident_h
    assert "APP_IdentAtt_StartPrbs" in ident_h
    assert "IDENT STEP" in app_control
    assert "IDENT DOUBLET" in app_control
    assert "IDENT PRBS" in app_control
    assert "IDENT ATT PRBS" in app_control
    assert "IDENT APPLY" in app_control
    assert "IDENT CENTER" in app_control


def test_closed_loop_attitude_ident_injects_reference_accel_and_logs_it() -> None:
    freertos = read("Core/Src/freertos.c")
    ident = read("App/Src/app_ident.c")
    flog_h = read("App/Inc/app_flight_log.h")
    receiver = read("tools/flight_log_receive.py")

    assert "APP_IdentAtt_Update(now);" in freertos
    assert "APP_IdentAtt_Apply(&reference.ax_m_s2" in freertos
    assert "APP_IdentAtt_Observe(&ident_att_obs);" in freertos
    assert "flog_snapshot.ident_att = ident_att_log;" in freertos
    assert "APP_IDENT_ATT_PENDING_STABLE_MS" in ident
    assert "IDENT att armed" in ident
    assert "wait_rc_arm" in ident
    assert "ident_att_try_begin" in ident
    assert "IDENT att wait" in ident
    assert "ident_att_ctx.log.signal_m_s2 =" in ident
    assert "APP_IDENT_ATT_AXIS_ROLL" in ident
    assert "APP_IDENT_ATT_AXIS_PITCH" in ident
    assert "APP_IdentAttLog ident_att;" in flog_h
    assert '"ident_att_signal_m_s2"' in receiver


def test_attitude_ident_safe_start_accepts_unsettled_controller_quality() -> None:
    ident = read("App/Src/app_ident.c")
    freertos = read("Core/Src/freertos.c")

    assert "#define APP_IDENT_ATT_ATTITUDE_LIMIT_DEG  20.0f" in ident
    assert "#define APP_IDENT_ATT_GYRO_LIMIT_DPS      150.0f" in ident
    assert "#define APP_IDENT_ATT_PENDING_STABLE_MS   250U" in ident
    assert "#define APP_IDENT_ATT_PENDING_REPORT_MS   1000U" in ident

    ready_block = ident.split(
        "static const char *ident_att_ready_reason", 1
    )[1].split("static void ident_att_report_wait", 1)[0]
    assert "wait_tilt" not in ready_block
    assert "wait_protection" not in ready_block
    assert "tilt_out_rad" not in ready_block
    assert "protection_flags" not in ready_block

    assert "ident_att_obs.control_valid =\n            ((rc_use_stabilized_motor_mix != 0U) &&\n             (imu_control_valid != 0U) &&\n             (ident_running == 0U)) ? 1U : 0U;" in freertos
    observe_block = ident.split("void APP_IdentAtt_Observe", 1)[1].split(
        "void APP_Ident_Update", 1
    )[0]
    assert 'APP_IdentAtt_Stop("tilt_limit")' not in observe_block
    assert 'APP_IdentAtt_Stop("protection")' not in observe_block
    assert 'APP_IdentAtt_Stop("control_invalid")' in observe_block


def test_ident_sample_parser_and_step_fit() -> None:
    panel = load_panel_module()
    line = (
        "IDENT sample id=3 seq=12 t_ms=3456 axis=roll mode=step "
        "alpha_us=1540 beta_us=1500 roll=1.230 pitch=-0.120 "
        "gx=4.50 gy=-0.80 rc_arm=1 throttle_us=1180"
    )
    record = panel.ident_record_from_line(line)
    assert record is not None
    assert record["seq"] == 12
    assert record["roll"] == 1.23
    assert record["gy"] == -0.8

    samples = []
    for index in range(30):
        t_ms = index * 40
        alpha = 1500 if index < 3 else 1540
        response = 0.0 if index < 3 else 4.0 * (1.0 - pow(2.718281828, -((index - 3) * 0.04) / 0.25))
        samples.append(
            panel.ident_record_from_line(
                f"IDENT sample id=1 seq={index} t_ms={t_ms} axis=roll mode=step "
                f"alpha_us={alpha} beta_us=1500 roll={response:.3f} "
                "pitch=0.000 gx=0.00 gy=0.00 rc_arm=1 throttle_us=1180"
            )
        )
    fit = panel.fit_ident_step([sample for sample in samples if sample is not None], "roll")
    assert fit is not None
    assert fit["K"] > 0
    assert fit["tau"] > 0
    assert fit["kp"] > 0

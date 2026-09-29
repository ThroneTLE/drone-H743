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
    freertos = read("Core/Src/freertos.c") + read("App/Src/app_stabilizer.c")
    ident = read("App/Src/app_ident.c")

    assert 'strcmp(line, "IDENT?") == 0' in app_aiwb2
    assert 'aiwb2_starts_with(line, "IDENT ")' in app_aiwb2
    # 舵机接管这个缝现在由两套辨识共用：`ident_running` 表示"有辨识在占用执行器"，
    # 下游联锁（LED / 点桨 inhibit / ESC 直通）全挂在它上面，具体是哪一套只决定
    # 舵机目标从谁那儿取。老 ident 必须还在这条缝里。
    assert "(APP_Ident_IsRunning() != 0U) || (frame->sysid_running != 0U)" in freertos
    assert "APP_Ident_GetServoTargets(&ident_alpha_us, &ident_beta_us);" in freertos
    assert "DRV_COAX_CTRL_RunScheduled(&frame->attitude, &frame->reference," in freertos
    assert "BSP_PWM_SetEscPulse" not in ident
    assert "DRV_Motor" not in ident
    assert "if (ident_ctx.axis == APP_IDENT_AXIS_ROLL) {\n        alpha += offset *" in ident
    assert "} else {\n        beta += offset *" in ident
    assert "calibration.pulse_sign[DRV_COAX_CTRL_SERVO_ALPHA_INDEX]" in ident


def test_ident_commands_exist_and_are_text_based() -> None:
    """IDENT 的语法已经从 app_control.c 搬到 app_cmd_sysid.c。

    搬的理由是 app_control.c 只减不增，而这一组子命令只调 `APP_Ident_*` 的公开
    接口、和那个文件的静态状态没有牵连。`IDENT START`（电机阶梯）没搬——它读写
    app_control.c 的 ident_* 静态量并由它的服务拍推进。
    """
    app_control = read("App/Src/app_control.c")
    cmd_sysid = read("App/Src/app_cmd_sysid.c")
    ident_h = read("App/Inc/app_ident.h")

    assert "APP_Ident_StartStep" in ident_h
    assert "APP_Ident_StartDoublet" in ident_h
    assert "APP_Ident_StartPrbs" in ident_h
    assert "APP_IdentAtt_StartPrbs" in ident_h
    for usage in ("IDENT STEP", "IDENT DOUBLET", "IDENT PRBS",
                  "IDENT ATT PRBS", "IDENT APPLY", "IDENT CENTER"):
        assert usage in cmd_sysid
        assert usage not in app_control, f"{usage} 应当只留在 app_cmd_sysid.c"
    assert "IDENT START" in app_control
    assert "app_cmd_sysid_handle_ident(tokens, count)" in app_control


def test_closed_loop_attitude_ident_injects_reference_accel_and_logs_it() -> None:
    freertos = read("Core/Src/freertos.c") + read("App/Src/app_stabilizer.c")
    ident = read("App/Src/app_ident.c")
    flog_h = read("App/Inc/app_flight_log.h")
    receiver = read("tools/flight_log_receive.py")

    assert "APP_IdentAtt_Update(frame->now_ms);" in freertos
    assert "APP_IdentAtt_Apply(&frame->reference.ax_m_s2" in freertos
    assert "APP_IdentAtt_Observe(&ident_att_obs);" in freertos
    assert "flog_snapshot.ident_att = frame->ident_att_log;" in freertos
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
    freertos = read("Core/Src/freertos.c") + read("App/Src/app_stabilizer.c")

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

    assert "ident_att_obs.control_valid =\n      ((frame->rc_use_stabilized_motor_mix != 0U) &&\n       (frame->imu_control_valid != 0U) &&\n       (frame->ident_running == 0U)) ? 1U : 0U;" in freertos
    observe_block = ident.split("void APP_IdentAtt_Observe", 1)[1].split(
        "void APP_Ident_Update", 1
    )[0]
    assert 'APP_IdentAtt_Stop("tilt_limit")' not in observe_block
    assert 'APP_IdentAtt_Stop("protection")' not in observe_block
    assert 'APP_IdentAtt_Stop("control_invalid")' in observe_block


def test_the_threshold_crossing_fit_is_gone_and_what_replaced_it() -> None:
    """老那套 `fit_ident_step` 是阈值穿越法：找到响应越过某比例的时刻，然后
    `kp = 0.35/|K|` 拍一个增益。**没有回归、没有延迟估计、没有置信度**，
    而且 `_ident_apply_fit` 自己在界面上写着"不写入四环控制器"——整条链在末端
    是断的。2026-09-13 随整页删除（R-SYSID-1）。

    接替它的是 `tools/sysid/fit.py`：延迟是**拟合参数**，先用方程误差粗扫再做
    输出误差精修，并报出阻尼与偏心的相关系数。行为判据在
    `tests/test_sysid_host_core.py`，这里只钉住"老的确实没了、新的确实在"。
    """
    panel_source = read("tools/drone_tcp_panel.py")
    assert "fit_ident_step" not in panel_source
    assert "ident_record_from_line" not in panel_source
    assert "0.35" not in panel_source or "kp = 0.35" not in panel_source

    fit_source = read("tools/sysid/fit.py")
    assert "def fit_model(" in fit_source
    assert "delay_s" in fit_source
    # 延迟必须是被拟合出来的，不是事后拿阈值猜的。
    assert "_equation_error_scan(" in fit_source
    assert "least_squares(" in fit_source
    assert "damping_eccentricity_correlation" in fit_source

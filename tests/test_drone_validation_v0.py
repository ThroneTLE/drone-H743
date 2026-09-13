from __future__ import annotations

import ast
import hashlib
import os
from pathlib import Path
from types import SimpleNamespace
import tkinter as tk
import queue
import re
import subprocess
import sys

import pytest

from tools import drone_tcp_panel as panel
from tools.panel_lib import evidence as panel_evidence
from tools.panel_lib import transport as panel_transport
from tools.panel_lib.pages import validation_v0 as validation_v0_page


ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / "tools" / "drone_tcp_panel.py").read_text(encoding="utf-8")
THEME_SOURCE = (ROOT / "tools" / "panel_lib" / "theme.py").read_text(encoding="utf-8")
EVIDENCE_SOURCE = (
    ROOT / "tools" / "panel_lib" / "evidence.py"
).read_text(encoding="utf-8")
VALIDATION_V0_SOURCE = (
    ROOT / "tools" / "panel_lib" / "pages" / "validation_v0.py"
).read_text(encoding="utf-8")
V1_SOURCE = (
    ROOT / "tools" / "panel_lib" / "pages" / "v1_metrology.py"
).read_text(encoding="utf-8")
V2_SOURCE = (
    ROOT / "tools" / "panel_lib" / "pages" / "acceptance_v2.py"
).read_text(encoding="utf-8")
TRANSPORT_SOURCE = (ROOT / "tools" / "panel_lib" / "transport.py").read_text(
    encoding="utf-8"
)
INCREMENT10_PARENT = "69bf4158977c62f65a78f7dc4b61b8a9f80275b7"
# H2 receipt/generation repair changes only three snapshot methods below;
# their behavior is covered by test_panel_connection_reliability.py.
V0_PAGE_AST_SHA256 = {
    "_build_validation_stepper": "fc3cd984eb80f7d0dc833d3e488eefa858e54a2fa71d468e06ccbf59f863164e",
    "_validation_current_step": "4f85dc0baf49912ef1b050d7c0cfce42d5c49fd9ee74992d8bee34663224cd6a",
    "_validation_refresh_stepper": "18ef352f3ce1f3bb20c1ef7647d0a8d2099c4172812cf36f70968c51b91f9791",
    "_validation_recolor_stage_rows": "8e3bd0eb74fa22373c2bcc28119b59c37675302ba3ed1eaddfb950f55da910ae",
    "_validation_set_gate": "e6fdd2fa939cbbb2bb7b1a9468baa61ad5f83dfbd9fe9fa707f1dc2b45c5eeba",
    "_build_validation_page": "6bc6a9f42fb6d6823645a8287dcbaff8b48ea8905f362b12db3bdcb6dba0fe7f",
    "_validation_update_safety_text": "b39643683d08e58eb54249112b57ff979c6127fa6c2456c9b7c1a89b1c23428d",
    "_validation_refresh_readiness": "d83dec0d051628ada7edae01a7dc111044bb8b09191ceb9884307e7d225f7c56",
    "_validation_set_status": "5d3f430fa41769066c0073e5bb3195aa9f6dd0fe499ea3d793d3d96b0f78d7c0",
    "_validation_selected_stage": "f8396054884d5298861756f8e257114867cd5ded507381dbbbc97d70d82073d2",
    "_on_validation_stage_select": "e803cf911affe959b0c474f5ac432d3058d1b63cde7dafe5d875a94d085f3fa2",
    "_validation_refresh_stage_view": "0cf5087241d610e7929d413b568aa99e152d32876e1a776f78e3b91f6cd0a148",
    "_validation_refresh_candidate_summary": "c14cc75de6e77e3e6472cee2c05c5de3840764c1b615a4cd258733c08ab94702",
    "_validation_candidate_verification_passed": "e108ca6c5c7a1b9dbffd58f401aa6eddff392c6f3b2643650904b4f518c3dc46",
    "_validation_static_fusion_consistency": "a9556590cdcaf4483a282c5a3eaee279c4ba399f5851b10b3f07300ed4789654",
    "_validation_candidate_level_ready": "54fa8e55c0a0e05ba2d0c0344700709d243bac70dd5ae239727d523c3384f6ef",
    "_validation_refresh_orientation_controls": "541d9996dc19a36640019d17e8fcc216dad299b5836926d0fbf2764f37a54dbf",
    "_validation_apply_candidate": "821876a9537787ad60150402f4f439ad9e742e7e30d1eeed5b0f0f0b38f9dccf",
    "_validation_revert_candidate": "68a6b77ecd1db4c24d8169289ce33d7dd01c75997a15f7b05038f0b53ee67b9a",
    "_validation_begin_candidate_verification": "1b1724e2f787b21e6ebcb76bebb661d06a703203b7c28603de8ae6b9ce34a1ea",
    "_validation_commit_candidate": "8991f2ed784470b4953b37b14e21657b04ab73c57120c845b7eec503241c6943",
    "_validation_note_orientation_event": "e7fa4b43f6ae05530fe4223e7232d328aa117e4de61ea570582d4fa370c4020b",
    "_validation_send_orientation_command": "5327044fc17f29e8a3c95102d713471b553966a7e35f5ba0dee66250bc6df6d1",
    "_validation_poll_orientation_commit": "88b67c88586284a79062ebee4758d78dfcbc0a89ff41d1ff82b2e8711a314c11",
    "_validation_handle_imu_frame_line": "1a5780660cf75262692ecb29e2e71946dd03ae348e1aac60756785dcf83a569b",
    "_validation_stage_inputs": "1bdeda74c3608d5aa07ceca7a48e6495b4691a8f8ef43250ead985720c255f57",
    "_validation_rebuild_results": "6b20bd640b7dc205cdf090d7e8946b1b3ae3870efd19707b663fbc1b91e4432d",
    "_validation_skip_stage": "345abe34461b5744a8a52b458de922c70248bd73ad0fa92a97cfcf667a828526",
    "_validation_start_session": "28afa8f2ead337eeaebad37ef770aed10b84140d31cf8a9953ca84331db182dd",
    "_validation_stop_session": "39789aeb07d44f609fd6be28143cf4d823def77d3d94d7ba2eb92c775d24a6e4",
    "_validation_latest_is_safe": "ccf735fd3d98be6b758258cca478b6efe998e89f22bbaa607ad110cbe874542b",
    "_validation_begin_stage": "c1e266f583b0ea900d4d8c169cb25beb6b6428bdce644ba7bb636a82f9dfbaa0",
    "_validation_finish_stage": "0363a839cfee1fbeb75f80bd30815bbcee2c28c81f3b7ef8e5880cee812731fb",
    "_validation_abort_active_stage": "5f85ca116f87fed3c93e5ca7ebb3e64245dc056a47e5b5a0c56c65282239b753",
    "_validation_accept_imu_health": "54fa1881e4110c0bb3215faa47dfffe2db4e241d8ccdcb16ffed7edb43f38e70",
    "_validation_health_state": "293f628cce3d64a09a358643c707faaf760697de226cd7af2784d6ebc9b33e38",
    "_validation_target_restarted": "a9551e6878742199eff9abe747a59b84f03b88aafa769fcb8bad00993b7fe62a",
    "_validation_accept_imu_values": "ac7d5f6d70a623df9ea167f34406ece06eb7cf99efcf513564345d44d4c88861",
}
EVIDENCE_AST_SHA256 = {
    "_validation_live_safety_gate": "c8e337f0876111c36d368fa3423533eff107eb2d5a05cb1078466f87c2486c09",
    "_validation_write_orientation_audit": "c4c616951e83b4acf540a4cb5a09a055cad959e0090c615ef1f7a139d82a884d",
    "_validation_raw_row_from_sample": "a43979fe0c91674ffea284ed63ae4fce75d8209d336480a2bbaddef609744957",
    "_validation_autosave_session": "ae4133acac4b6ad61635468b0a7bef19073c14c27b2d395a7553662511f4c63a",
    "_validation_autosave_workflow": "707bbc06ec4e05238717bcb38969471a4056d58f831db801e84b86265825a765",
    "_validation_load_workflow": "cf6901a35ad33c6688f05d5898c14d4999149ccf5fd00257bbe97480af180980",
    "_validation_apply_loaded_session": "937350572cc444d50797cdea995ee88af3a4234a5aa3ff4af92f7d18d6be7de4",
    "_validation_apply_report_only": "7e0b3cf38a1e3afb2d879c37758aa5fa0bfe075277666e32756db8ae427eb282",
    "_validation_load_report_artifact": "4e3a7793cbad949e12a6aa5da44b792ebf2b4587803023705e6abb781a90f403",
    "_validation_refresh_history_choices": "33d13bacdddc7083364027b4696b864717afe9247e4811c6ef0325810b476c11",
    "_validation_load_artifact": "a8eda7eef265346fa3a4ad806ebc6978c61050676256c0e335df8514bf46c56b",
    "_validation_load_selected_history": "440b49b1c09875a777a0c4267ea17f4575728ce3fa1a0d8e3ebbefd8f8dab1e8",
    "_validation_load_latest_artifact": "8c25e9238055f3f0524a4987533c7d5732a8f83bedb85af8e6a41f2b62a809ba",
    "_validation_resume_session": "525acd5122abeec18606841af40965a2eb0c81065444e1710c40bed09783eb82",
    "_validation_write_report": "b9a75d336ef454d416ee549de1962a1dac7662cdb7faaec7b02eb9f3004d59e2",
    "_validation_open_report_dir": "ffd6c85d1c7be4a949c9a40c1bb32d07440c2c38519fd3e1db551ff9c5e76407",
    "_validation_command_allowed": "c71523aba05690a98dfc304ed95baff0a5389d58815b4a6ad3c65a959a8b3fc7",
    "_validation_guard_command": "407b2b565c631dec84635c988d0e3b1ad7d4a7d4683ba4ce35bb8d41ef8648bd",
}
VALIDATION_HELPER_AST_SHA256 = {
    "v0_workflow_guidance": "63c14f0d473be16f0d152de5b59a6b1d0127c95e82e65aa2b899a2b46c039984",
    "validation_history_artifacts": "cde67ba5a3412cb795138189ebec5c6609ac0311a2095be46c703acf7823fe66",
    "validation_sample_from_snapshot": "581d4a0d6bdcdbda759f0d289174fbdb3d902d2a9afb31fd5e58fdb1a363195f",
    "validation_samples_from_csv": "aa92f5ae6f2f51c3926d937baf7ba59270b12a94670d4c9bf7c1b707440dd3a0",
    "signed_permutation_descriptor": "b4e3ba261a1d8f3a1a6563d3c3c457389a2ca40543774f4e9145bc5eaf4b122f",
}


def function_body(source: str, signature: str) -> str:
    if source is SOURCE and ("_validation" in signature or "_build_validation" in signature):
        if signature in VALIDATION_V0_SOURCE:
            source = VALIDATION_V0_SOURCE
        elif signature in EVIDENCE_SOURCE:
            source = EVIDENCE_SOURCE
    start = source.index(signature)
    end = source.find("\n    def ", start + len(signature))
    return source[start:] if end < 0 else source[start:end]


def complete_snapshot() -> dict[str, str]:
    return {
        "valid": "1",
        "source": "stabilizer_snapshot",
        "frame": "legacy_intermediate",
        "units": "mg_mdps_cdeg",
        "contract": "1",
        "migration": "0x00",
        "ts_ms": "1250",
        "seq": "1234",
        "bias": "1",
        "armed": "0",
        "m1": "1000",
        "m2": "1000",
        "ax_mg": "100",
        "ay_mg": "-200",
        "az_mg": "975",
        "gx_mdps": "1500",
        "gy_mdps": "-2500",
        "gz_mdps": "500",
        "roll_cdeg": "123",
        "pitch_cdeg": "-456",
        "yaw_cdeg": "789",
    }


def test_snapshot_parser_requires_provenance_and_never_invents_zeroes() -> None:
    legacy_zero_line = panel.parse_kv(
        "IMU scaled ax_mg=0 ay_mg=0 az_mg=0 gx_mdps=0 gy_mdps=0 gz_mdps=0"
    )

    sample, reason = panel.validation_sample_from_snapshot(legacy_zero_line)

    assert sample is None
    assert reason is not None and reason.startswith("incomplete:")
    assert "source" in reason

    malformed = complete_snapshot()
    malformed.pop("ts_ms")
    malformed["ts_us"] = "1u"
    sample, reason = panel.validation_sample_from_snapshot(malformed)
    assert sample is None
    assert reason is not None and "ts_ms" in reason


def test_snapshot_parser_normalizes_target_units_without_claiming_flu() -> None:
    values = complete_snapshot()

    sample, reason = panel.validation_sample_from_snapshot(values)

    assert reason is None
    assert sample is not None
    assert sample.timestamp_s == pytest.approx(1.25)
    assert sample.accel_g == pytest.approx((0.1, -0.2, 0.975))
    assert sample.gyro_dps == pytest.approx((1.5, -2.5, 0.5))
    assert sample.attitude_deg == pytest.approx((1.23, -4.56, 7.89))
    assert values["frame"] == "legacy_intermediate"


def test_post_apply_gate_compares_canonical_accel_tilt_with_fusion() -> None:
    def sample(*, accel: tuple[float, float, float], roll: float, pitch: float):
        return panel.ImuSample(
            timestamp_s=0.0,
            accel_x_g=accel[0],
            accel_y_g=accel[1],
            accel_z_g=accel[2],
            gyro_x_dps=0.0,
            gyro_y_dps=0.0,
            gyro_z_dps=0.0,
            roll_deg=roll,
            pitch_deg=pitch,
            yaw_deg=0.0,
        )

    good = SimpleNamespace(
        validation_samples={
            panel.ValidationStage.LEVEL: [
                sample(accel=(0.0, 0.0, 1.0), roll=0.0, pitch=0.0)
                for _ in range(20)
            ],
            panel.ValidationStage.NOSE_UP: [
                sample(accel=(1.0, 0.0, 0.0), roll=0.0, pitch=-90.0)
                for _ in range(20)
            ],
        }
    )
    bad = SimpleNamespace(
        validation_samples={
            panel.ValidationStage.LEFT_SIDE_UP: [
                sample(accel=(0.0, 1.0, 0.0), roll=-90.0, pitch=0.0)
                for _ in range(20)
            ]
        }
    )

    level_status, level_error, _ = panel.DronePanel._validation_static_fusion_consistency(
        good, panel.ValidationStage.LEVEL
    )
    nose_status, nose_error, _ = panel.DronePanel._validation_static_fusion_consistency(
        good, panel.ValidationStage.NOSE_UP
    )
    bad_status, bad_error, _ = panel.DronePanel._validation_static_fusion_consistency(
        bad, panel.ValidationStage.LEFT_SIDE_UP
    )

    assert level_status is panel.ValidationStatus.PASS
    assert level_error == pytest.approx(0.0)
    assert nose_status is panel.ValidationStatus.PASS
    assert nose_error == pytest.approx(0.0)
    assert bad_status is panel.ValidationStatus.FAIL
    assert bad_error == pytest.approx(180.0)


def test_v0_session_command_gate_only_grants_exact_orientation_actions() -> None:
    active = SimpleNamespace(
        validation_session_active=True,
        validation_authorized_orientation_commands=set(),
        firmware_authorized_commands=set(),
    )
    inactive = SimpleNamespace(
        validation_session_active=False,
        firmware_authorized_commands=set(),
    )

    assert panel.DronePanel._validation_command_allowed(active, "IMU?")
    assert panel.DronePanel._validation_command_allowed(active, "STATUS?")
    assert panel.DronePanel._validation_command_allowed(active, "IMUCAP STOP")
    assert panel.DronePanel._validation_command_allowed(active, "IMUFRAME?")
    assert panel.DronePanel._validation_command_allowed(active, "IMUFRAME?")
    for command in (
        "SAVE",
        "LOAD",
        "DEFAULTS",
        "PARAM SET x 1",
        "PID SET roll 1 2",
        "SERVO MOVE 1 1600 500",
        "IDENT ARM",
        "IMUFRAME APPLY -x,-y,+z",
        "IMUFRAME REVERT",
        "IMUFRAME COMMIT",
        "BOOT DFU CONFIRM",
        "BOOT DFU",
        "UNKNOWN?",
    ):
        assert not panel.DronePanel._validation_command_allowed(active, command)

    active.validation_authorized_orientation_commands.add(
        "IMUFRAME APPLY -X,-Y,+Z"
    )
    assert panel.DronePanel._validation_command_allowed(
        active, "IMUFRAME APPLY -x,-y,+z"
    )
    assert not panel.DronePanel._validation_command_allowed(
        active, "IMUFRAME APPLY +x,+y,+z"
    )
    assert panel.DronePanel._validation_command_allowed(inactive, "SAVE")
    assert not panel.DronePanel._validation_command_allowed(inactive, "IMUFRAME COMMIT")
    assert not panel.DronePanel._validation_command_allowed(
        inactive, "IMUFRAME APPLY -x,-y,+z"
    )
    assert not panel.DronePanel._validation_command_allowed(inactive, "BOOT DFU CONFIRM")
    inactive.firmware_authorized_commands.add(panel.FIRMWARE_BOOT_COMMAND)
    assert panel.DronePanel._validation_command_allowed(inactive, " boot   dfu confirm ")
    assert not panel.DronePanel._validation_command_allowed(inactive, "BOOT DFU CONFIRM NOW")
    inactive.firmware_update_pending = True
    assert not panel.DronePanel._validation_command_allowed(inactive, "IMU?")
    fallback = function_body(SOURCE, "    def _fallback_proto_request(")
    assert "_validation_command_allowed(legacy_line)" in fallback


def test_v0_session_invalidates_transport_write_queues_at_the_send_boundary() -> None:
    rx: "queue.Queue[str]" = queue.Queue()
    tcp = panel.TcpTransport(rx)
    serial = panel.SerialTransport(rx)
    tcp._send_queue.put_nowait((tcp._send_generation, b"unsafe-tcp"))
    serial._send_queue.put_nowait((serial._send_generation, b"unsafe-serial"))

    tcp.cancel_pending_sends()
    serial.cancel_pending_sends()

    assert tcp._send_queue.empty()
    assert serial._send_queue.empty()
    assert tcp._send_generation == 1
    assert serial._send_generation == 1
    serial_source = (ROOT / "tools/panel_lib/serial_session.py").read_text(encoding="utf-8")
    assert (TRANSPORT_SOURCE + serial_source).count("generation != self._send_generation") == 2
    start = function_body(SOURCE, "    def _validation_start_session(")
    assert "transport.cancel_pending_sends()" in start
    assert start.index("transport.cancel_pending_sends()") < start.index(
        "self.validation_session_active = True"
    )


def test_validation_page_limits_target_writes_to_the_guarded_orientation_flow() -> None:
    body = function_body(SOURCE, "    def _build_validation_page(")

    assert "采集阶段只读" in body
    assert "只有下方 A/B/C 映射流程" in body
    assert "A · 临时应用到 RAM" in body
    assert "B · 清空旧样本并重采 6 步" in body
    assert "C · 复验通过后写入 Flash" in body
    assert "PROTO_REQ_IMU_FRAME" in body
    assert 'self.notebook.add(calibration, text="校准")' in (ROOT / "tools/panel_lib/shell.py").read_text(encoding="utf-8")
    assert 'self.calibration_notebook.add(validation_scroll, text="坐标系与极性")' in (ROOT / "tools/panel_lib/shell.py").read_text(encoding="utf-8")
    assert "AIRFRAME_CALIBRATION_DIR" in SOURCE
    for forbidden in ("PROTO_REQ_SAVE", "PROTO_REQ_LOAD", "PROTO_REQ_DEFAULTS", "PROTO_REQ_PARAM_SET", "PROTO_REQ_PID_SET", "PROTO_REQ_SERVO_MOVE"):
        assert forbidden not in body


def test_v0_standard_flow_has_six_required_and_three_legacy_optional_stages() -> None:
    required = [
        panel.ValidationStage.LEVEL,
        panel.ValidationStage.NOSE_UP,
        panel.ValidationStage.LEFT_SIDE_UP,
        panel.ValidationStage.POSITIVE_ROLL,
        panel.ValidationStage.POSITIVE_PITCH,
        panel.ValidationStage.POSITIVE_YAW,
    ]
    optional = [
        stage
        for stage, definition in panel.STAGE_DEFINITIONS.items()
        if not definition.required
    ]

    assert list(panel.VALIDATION_UI_STAGES) == required
    assert optional == [
        panel.ValidationStage.NOSE_DOWN,
        panel.ValidationStage.RIGHT_SIDE_UP,
        panel.ValidationStage.INVERTED,
    ]


def test_signed_permutation_descriptor_matches_the_measured_flu_candidate() -> None:
    assert panel.signed_permutation_descriptor(
        ((-1, 0, 0), (0, -1, 0), (0, 0, 1))
    ) == "-x,-y,+z"
    assert panel.signed_permutation_descriptor(
        ((-1, 0, 0), (0, 1, 0), (0, 0, 1))
    ) is None


def test_ui_hot_paths_are_bounded_and_render_is_throttled() -> None:
    drain = function_body(SOURCE, "    def _drain_rx(")
    append = function_body(SOURCE, "    def _append(")
    flush = function_body(SOURCE, "    def _flush_log(")
    imu_tick = function_body(SOURCE, "    def _imu_poll_tick(")
    ident = function_body(SOURCE, "    def _ident_handle_line(")

    assert "RX_DRAIN_BATCH_SIZE" in drain
    dispatcher = (ROOT / "tools/panel_lib/rx_dispatch.py").read_text(encoding="utf-8")
    assert "processed < batch_size" in dispatcher
    assert "RX_DRAIN_BUSY_MS" in drain
    assert "_pending_log_lines.append" in append
    assert 'batch = "".join(self._pending_log_lines)' in flush
    assert "IMU_RENDER_PERIOD_MS" in imu_tick
    assert "imu_tab_visible" in imu_tick
    assert "1_000_000_000" in ident
    assert "200_000_000" in ident
    assert "self.notebook.select() == str(self.ident_tab)" in ident


def test_firmware_update_only_hard_blocks_on_the_link() -> None:
    """解锁/油门由固件的 BOOT 处理器判定，主机只管命令送不送得出去。"""
    ok, reason = panel.firmware_update_link_gate(
        connected=True, serial_transport_selected=True
    )
    assert ok
    assert "USB CDC" in reason

    blocked, message = panel.firmware_update_link_gate(
        connected=False, serial_transport_selected=True
    )
    assert not blocked
    assert "未连接" in message

    wrong_transport, message = panel.firmware_update_link_gate(
        connected=True, serial_transport_selected=False
    )
    assert not wrong_transport
    assert "serial" in message


def test_snapshot_advisory_describes_but_never_blocks() -> None:
    values = complete_snapshot()
    level, text = panel.firmware_update_snapshot_advisory(values, sample_age_s=0.2)
    assert level == "ok"
    assert "armed=0" in text

    for update, expected in (({"armed": "1"}, "armed=1"), ({"m1": "1200"}, "m1=1200")):
        level, text = panel.firmware_update_snapshot_advisory(
            values | update, sample_age_s=0.2
        )
        assert level == "warn"
        assert expected in text

    # 没有快照不是拒绝理由——旧固件、遥测缺一行都会走到这里。
    for missing, age in (
        ({}, 0.2),
        (values, 2.0),
        (values, float("inf")),
        (values | {"source": "legacy"}, 0.2),
    ):
        level, _text = panel.firmware_update_snapshot_advisory(missing, sample_age_s=age)
        assert level == "unknown"


# --------------------------------------------------------------------------
# 目标重启后序号基线必须失效
#
# 2026-08-29 实测：飞控烧录后重启，seqlock 序号从 ~2049958 回到 ~11350。单调守卫
# 把之后的每一帧都当成重放丢弃，validation_latest_host_time 永远停在 0，升级页显示
# "安全快照已过期（inf）"，断开重连也救不回来——只能重启上位机。
# --------------------------------------------------------------------------

def restart_subject(previous_ms: int | None) -> SimpleNamespace:
    return SimpleNamespace(validation_latest_timestamp_ms=previous_ms)


def test_target_reboot_is_detected_from_the_board_uptime() -> None:
    detect = panel.DronePanel._validation_target_restarted
    assert detect(restart_subject(2_049_958), 11_350) is True
    assert detect(restart_subject(1000), 1001) is False
    assert detect(restart_subject(1000), 1000) is False
    # 第一帧没有基线，不能误判成重启。
    assert detect(restart_subject(None), 5) is False


def test_reboot_clears_the_sequence_baseline_before_the_monotonic_guard() -> None:
    body = function_body(SOURCE, "    def _validation_accept_imu_values(")
    reset = body.index("self.validation_latest_sequence = None")
    guard = body.index("sequence <= self.validation_latest_sequence")
    assert body.index("_validation_target_restarted(timestamp_ms)") < reset < guard
    assert "self.validation_latest_timestamp_ms = timestamp_ms" in body


def test_reconnect_paths_drop_the_sequence_baseline() -> None:
    """烧完必然重启；重连后不清基线，升级页会一直停在"状态未知"。"""
    for name in ("    def _stop(", "    def _firmware_auto_reconnect("):
        body = function_body(SOURCE, name)
        assert "self.validation_latest_sequence = None" in body
        assert "self.validation_latest_timestamp_ms = None" in body


def test_stale_snapshot_never_reaches_the_user_as_infs() -> None:
    _level, text = panel.firmware_update_snapshot_advisory(
        complete_snapshot(), sample_age_s=float("inf")
    )
    assert "inf" not in text


def test_firmware_usb_identity_policy_allows_only_application_cdc_by_default() -> None:
    allowed = {
        "vid": 0x0483,
        "pid": 0x5740,
        "description": "STM32 Virtual COM Port",
        "hwid": "USB VID:PID=0483:5740",
    }
    assert panel.serial_device_identity_policy(allowed)[0] == "allowed"
    for description in ("STLink Virtual COM Port", "USB-SERIAL CH340", "CP210x", "FTDI"):
        state, _reason = panel.serial_device_identity_policy(
            {"vid": 0x1234, "pid": 0x5678, "description": description}
        )
        assert state == "rejected"
    assert panel.serial_device_identity_policy(
        {"vid": 0x1234, "pid": 0x5678, "description": "Unknown CDC"}
    )[0] == "unknown"


def test_firmware_reenumeration_matches_the_same_application_cdc() -> None:
    previous = {
        "device": "COM31", "vid": 0x0483, "pid": 0x5740,
        "serial_number": "H743-UID-01", "location": "1-4",
        "description": "STM32 Virtual COM Port", "hwid": "USB VID:PID=0483:5740",
    }
    candidates = [
        {"device": "COM8", "vid": 0x1A86, "pid": 0x7523,
         "serial_number": "", "location": "1-2", "description": "CH340", "hwid": "CH340"},
        {"device": "COM44", "vid": 0x0483, "pid": 0x5740,
         "serial_number": "H743-UID-01", "location": "1-4",
         "description": "STM32 Virtual COM Port", "hwid": "USB VID:PID=0483:5740"},
    ]
    assert panel.select_reenumerated_application_port(
        "COM31", previous, candidates) == "COM44"

    unrelated = [dict(candidates[1], device="COM45", serial_number="OTHER", location="2-1")]
    assert panel.select_reenumerated_application_port(
        "COM31", previous, unrelated) is None


def test_firmware_waits_for_application_cdc_without_selecting_first_com() -> None:
    previous = {
        "device": "COM31", "vid": 0x0483, "pid": 0x5740,
        "serial_number": "UID", "location": "1-4",
        "description": "STM32 Virtual COM Port", "hwid": "USB VID:PID=0483:5740",
    }
    replies = iter((
        ({"device": "COM8", "vid": 0x1A86, "pid": 0x7523,
          "description": "CH340", "hwid": "CH340"},),
        ({"device": "COM42", "vid": 0x0483, "pid": 0x5740,
          "serial_number": "UID", "location": "1-4",
          "description": "STM32 Virtual COM Port", "hwid": "USB VID:PID=0483:5740"},),
    ))
    assert panel.wait_for_application_serial(
        "COM31", previous, timeout_s=1.0, poll_interval_s=0.01,
        enumerate_ports=lambda: next(replies),
    ) == "COM42"


def test_serial_transport_latches_port_and_changes_connection_generation(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    class FakePort:
        is_open = True

        def close(self) -> None:
            self.is_open = False

    class FakeSerialModule:
        @staticmethod
        def Serial(*_args: object, **_kwargs: object) -> FakePort:
            return FakePort()

    class NoopThread:
        def __init__(self, *args: object, **kwargs: object) -> None:
            del args, kwargs

        def start(self) -> None:
            return

    monkeypatch.setattr(panel_transport, "HAS_PYSERIAL", True)
    monkeypatch.setattr(panel_transport, "serial", FakeSerialModule)
    monkeypatch.setattr(panel_transport.threading, "Thread", NoopThread)
    transport = panel.SerialTransport(queue.Queue())
    initial = transport.connection_generation
    transport.start("COM31", 115200)
    connected_generation = transport.connection_generation
    assert transport.active_port == "COM31"
    assert connected_generation > initial
    transport.stop()
    assert transport.active_port is None
    assert transport.connection_generation > connected_generation


def test_running_dfu_wait_honors_target_cancelled_reason() -> None:
    status = SimpleNamespace(value="", set=lambda value: setattr(status, "value", value))
    cancel = __import__("threading").Event()
    logs: list[str] = []
    subject = SimpleNamespace(
        last_reply_rx=0.0,
        firmware_attempt_id=7,
        firmware_update_running=True,
        firmware_update_pending=False,
        firmware_target_abort_reason="",
        firmware_cancel_event=cancel,
        firmware_status_var=status,
        _firmware_record_log=lambda attempt, line: logs.append(f"{attempt}:{line}"),
    )
    panel.DronePanel._firmware_handle_boot_line(
        subject, "BOOT mode=dfu state=cancelled reason=armed"
    )
    assert cancel.is_set()
    assert "armed" in subject.firmware_target_abort_reason
    assert status.value == subject.firmware_target_abort_reason
    assert any("TARGET_ABORT" in line for line in logs)


def test_firmware_page_uses_one_shot_boot_authorization_and_background_worker() -> None:
    page = function_body(SOURCE, "    def _build_firmware_update_page(")
    # 烧录入口现在分成两段：_firmware_start_update 先编译，
    # _firmware_start_update_after_build 才做预检并发 BOOT。
    start = (
        function_body(SOURCE, "    def _firmware_start_update(self)")
        + function_body(SOURCE, "    def _firmware_start_update_after_build(")
    )
    sender = function_body(SOURCE, "    def _firmware_send_boot_command(")
    baseline = function_body(SOURCE, "    def _firmware_check_baseline_worker(")
    after_baseline = function_body(SOURCE, "    def _firmware_send_boot_after_baseline(")
    worker = function_body(SOURCE, "    def _firmware_update_worker(")
    cancel = function_body(SOURCE, "    def _firmware_cancel_update(")
    close = function_body(SOURCE, "    def _on_close(")

    assert "固件升级" in page
    assert "只接受 ELF/HEX" in page
    assert "FIRMWARE_BOOT_COMMAND" in start
    assert "transport.cancel_pending_sends()" in start
    assert "_firmware_check_baseline_worker" in start
    assert "list_usb_dfu_ports" in baseline
    assert "baseline_conflict" in baseline
    assert "_firmware_send_boot_command" not in start
    assert "_firmware_send_boot_command" in after_baseline
    assert "firmware_authorized_commands.discard" in sender
    assert "wait_for_usb_dfu" in worker
    assert "flash_firmware" in worker
    assert "wait_for_application_serial" in worker
    assert "_firmware_auto_reconnect" in SOURCE
    assert "result.download_verified" in SOURCE
    assert "写入并校验成功，但 ROM DFU 未自动启动应用" in SOURCE
    assert "threading.Thread" in SOURCE
    assert cancel.index("if self.firmware_programming") < cancel.index(
        "self.firmware_cancel_event.set()"
    )
    assert "program/verify 阶段不能从界面取消" in cancel
    assert close.index("if self.firmware_programming") < close.index(
        "self.firmware_cancel_event.set()"
    )


def relative_luminance(hex_color: str) -> float:
    """WCAG 相对亮度：sRGB 必须先做 gamma 解码，否则中间调会被高估。"""
    channels = []
    for offset in (1, 3, 5):
        c = int(hex_color[offset:offset + 2], 16) / 255.0
        channels.append(c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4)
    r, g, b = channels
    return 0.2126 * r + 0.7152 * g + 0.0722 * b


def contrast_ratio(fg: str, bg: str) -> float:
    a, b = relative_luminance(fg), relative_luminance(bg)
    lighter, darker = max(a, b), min(a, b)
    return (lighter + 0.05) / (darker + 0.05)


def test_panel_visual_hierarchy_uses_semantic_styles_and_guidance() -> None:
    configure = function_body(THEME_SOURCE, "    def _configure_style(")
    validation = function_body(SOURCE, "    def _build_validation_page(")
    v1 = function_body(V1_SOURCE, "    def _build_v1_page(")
    v2 = function_body(V2_SOURCE, "    def _build_v2_page(")
    firmware = function_body(SOURCE, "    def _build_firmware_update_page(")
    for style in (
        "Primary.TButton", "Secondary.TButton", "Warning.TButton",
        "Danger.TButton", "Guide.TLabel", "Eyebrow.TLabel",
    ):
        assert style in SOURCE
    assert 'style.theme_use("clam")' in configure
    assert "IMU FRAME  /  建立唯一坐标与极性" in validation
    assert "IMU CALIBRATION  /  修正传感器连续误差" in v1
    assert "GROUND ACCEPTANCE  /  验证控制链与执行方向" in v2
    assert "安全升级与自动重连" in firmware
    assert 'style="Warning.TButton"' in firmware


def test_theme_is_a_mid_dark_console_not_near_black() -> None:
    """深灰工程控制台：底色要暗但不能压成近黑，层级要能分辨。"""
    palette = panel.UI_PALETTE
    for key in ("canvas", "surface", "panel", "console"):
        lum = relative_luminance(palette[key])
        assert 0.004 < lum < 0.06, (key, lum)
    # canvas → surface → panel 必须逐级变亮，否则卡片和页面糊成一片。
    assert (
        relative_luminance(palette["canvas"])
        < relative_luminance(palette["surface"])
        < relative_luminance(palette["panel"])
        < relative_luminance(palette["raised"])
    )
    # 兼容别名必须指向同一个强调色，避免又长出第二套主色。
    assert palette["blue"] == palette["accent"]
    assert palette["navy"] == palette["panel"]


def test_theme_text_meets_wcag_aa_contrast() -> None:
    """字"看不清"不能只靠肉眼判断，这里用 WCAG AA (4.5:1) 卡住。"""
    palette = panel.UI_PALETTE
    for fg, bg in (
        ("ink", "surface"), ("ink", "panel"), ("ink", "canvas"),
        ("ink_dim", "surface"), ("ink_dim", "panel"), ("ink_dim", "console"),
        ("muted", "surface"), ("muted", "panel"),
        ("accent", "surface"), ("green", "panel"),
        ("amber", "panel"), ("red", "panel"),
    ):
        ratio = contrast_ratio(palette[fg], palette[bg])
        assert ratio >= 4.5, (fg, bg, round(ratio, 2))
    # 强调色实心按钮上的文字。
    assert contrast_ratio(palette["accent_ink"], palette["accent"]) >= 4.5


def test_panel_declares_dpi_awareness_before_creating_the_tk_root() -> None:
    """非 DPI-aware 进程会被 Windows 位图拉伸，高分屏上文字发虚。

    声明必须早于 super().__init__()——根窗口一旦建立就无法再改。
    """
    init = function_body(SOURCE, "    def __init__(self) -> None:")
    code = [
        line.strip() for line in init.splitlines()
        if line.strip() and not line.strip().startswith("#")
    ]
    awareness = next(i for i, line in enumerate(code) if "enable_hidpi_awareness()" in line)
    root = next(i for i, line in enumerate(code) if line == "super().__init__()")
    assert awareness < root
    assert "SetProcessDpiAwareness" in SOURCE


def test_scaling_follows_the_real_dpi_instead_of_shrinking_text() -> None:
    def scaling_for(dpi_scale: float, screen_height: int) -> float:
        calls: list[tuple] = []
        subject = SimpleNamespace(
            ui_dpi_scale=dpi_scale,
            tk=SimpleNamespace(call=lambda *args: calls.append(args) or "1.3333333"),
            winfo_screenheight=lambda: screen_height,
        )
        panel.DronePanel._configure_compact_scaling(subject)
        return subject.ui_scaling

    # 2K + 150% 系统缩放：点→像素必须按真实 DPI 放大，而不是压到 1.5 以下。
    assert scaling_for(1.5, 1440) == pytest.approx(1.3333333 * 1.5 * 0.95, rel=1e-3)
    # 100% 缩放的 1080p 仍略微收紧密度，但不会低于 1.0。
    assert scaling_for(1.0, 1080) == pytest.approx(1.3333333 * 0.90, rel=1e-3)
    assert scaling_for(1.0, 1080) >= 1.0


def test_action_buttons_carry_no_step_numbers() -> None:
    """序号进按钮是"幼稚感"的主要来源；顺序改由步骤条表达。

    小节标题（LabelFrame）仍可带 "1 · ..." 索引——工程手册就是这么编号的，
    这里只约束按钮本身。
    """
    offenders = [
        match.group(0)
        for match in re.finditer(r"ttk\.Button\((?:[^()]|\([^()]*\))*?\)", SOURCE, re.S)
        if re.search(r'text=f?"[1-9]\s', match.group(0))
    ]
    assert offenders == [], offenders


def test_v0_page_shows_a_stepper_and_indicator_dots_instead_of_glyph_soup() -> None:
    page = function_body(SOURCE, "    def _build_validation_page(")
    assert "_build_validation_stepper" in page
    assert panel.DronePanel.VALIDATION_STEPS == ("坐标发现", "RAM 复验", "写入 Flash")
    # 就绪条件改为指示灯 + 等宽数值，不再用带圈数字和 ✓/✗ 拼一行文字。
    assert "validation_gate_dots" in page
    readiness = function_body(SOURCE, "    def _validation_refresh_readiness(")
    for glyph in ("①", "②", "③", "④", "✓", "✗"):
        assert glyph not in readiness, glyph


def test_stepper_tracks_the_phase_text() -> None:
    def step_for(phase: str) -> int:
        subject = SimpleNamespace(
            validation_phase_var=SimpleNamespace(get=lambda: phase),
            VALIDATION_STEPS=panel.DronePanel.VALIDATION_STEPS,
        )
        return panel.DronePanel._validation_current_step(subject)

    assert step_for("阶段 1/3 · 已发现 FLU 映射候选，尚未应用") == 0
    assert step_for("阶段 2/3 · 正在验证 RAM 中的 FLU 映射") == 1
    assert step_for("阶段 3/3 · RAM 映射下的 6 步复验已经 PASS") == 2
    assert step_for("坐标系校准已完成 · FLU 映射已写入参数 Flash") == 3


def test_workflow_pages_are_scrollable_and_default_to_compact_dpi() -> None:
    init = function_body(SOURCE, "    def __init__(self) -> None:")
    build = (ROOT / "tools/panel_lib/shell.py").read_text(encoding="utf-8")
    assert "VerticalScrolledFrame" in SOURCE
    assert "_configure_compact_scaling" in init
    for page in (
        "validation_scroll", "metrology_scroll", "mechanical_scroll",
        "flow_range_scroll", "acceptance_v2_scroll", "vibration_scroll",
        "firmware_scroll",
    ):
        assert page in build


def test_v0_phase_guidance_explains_that_discovery_failures_require_recollection() -> None:
    phase, action = panel.v0_workflow_guidance(
        candidate_available=True,
        candidate_applied=False,
        verification_mode=False,
        candidate_verified=False,
        candidate_committed=False,
        runtime_ready=False,
    )
    assert "阶段 1/3" in phase
    assert "旧 FAIL" in action
    assert "A" in action

    phase, action = panel.v0_workflow_guidance(
        candidate_available=True,
        candidate_applied=True,
        verification_mode=False,
        candidate_verified=False,
        candidate_committed=False,
        runtime_ready=True,
    )
    assert "阶段 2/3" in phase
    assert "不会自动变成 PASS" in action
    assert "B" in action

    phase, action = panel.v0_workflow_guidance(
        candidate_available=True,
        candidate_applied=True,
        verification_mode=True,
        candidate_verified=True,
        candidate_committed=False,
        runtime_ready=True,
    )
    assert "阶段 3/3" in phase
    assert "C" in action


def test_v0_history_catalog_keeps_completed_reports_reachable(tmp_path: Path) -> None:
    day = tmp_path / "2026-08-28"
    day.mkdir()
    first = day / "flu_acceptance_v0_20260828_173121_report.json"
    first.write_text("{}", encoding="utf-8")
    second_report = day / "flu_acceptance_v0_20260828_173317_report.json"
    second_report.write_text("{}", encoding="utf-8")
    second_session = day / "flu_acceptance_v0_20260828_173317_session.json"
    second_session.write_text(
        '{"samples_by_stage":{"level":[{"timestamp_s":0}]}}', encoding="utf-8")

    catalog = panel.validation_history_artifacts(tmp_path)
    assert len(catalog) == 2
    assert catalog[0][1] == second_session
    assert "会话 + 报告" in catalog[0][0]
    assert catalog[1][1] == first


def test_orientation_receipts_are_not_clobbered_by_the_periodic_refresh() -> None:
    """写 Flash / 读取状态的提示必须留在 A/B/C 操作区，而不是被 0.5s 刷新盖掉。

    _validation_refresh_orientation_controls 每 0.5s 由就绪检查触发一次，并且
    无条件重写 validation_orientation_var；因此事件回执必须用独立变量。
    """
    refresh = function_body(SOURCE, "    def _validation_refresh_orientation_controls(")
    assert "validation_orientation_var.set" in refresh
    assert "validation_orientation_event_var" not in refresh
    assert "validation_orientation_target_var" not in refresh

    # 所有瞬时事件都必须走回执通道。
    for caller in (
        "    def _validation_commit_candidate(",
        "    def _validation_apply_candidate(",
        "    def _validation_poll_orientation_commit(",
    ):
        body = function_body(SOURCE, caller)
        assert "validation_orientation_var.set" not in body, caller

    page = function_body(SOURCE, "    def _build_validation_page(")
    assert "validation_orientation_event_var" in page
    assert "validation_orientation_target_var" in page


def test_imu_frame_reply_always_produces_a_visible_receipt() -> None:
    """点“读取飞机映射状态”后，即使状态没变也要看得见回包到达。"""
    handler = function_body(SOURCE, "    def _validation_handle_imu_frame_line(")
    assert "self.validation_orientation_target_var.set(" in handler
    assert "reported = False" in handler
    assert "if not reported:" in handler
    # 每条提前 return 的分支也必须先写过回执。
    assert handler.count("_validation_note_orientation_event") >= 5


def test_flash_commit_success_also_raises_a_modal() -> None:
    """页头状态条滚动后会移出视野，写 Flash 这种一次性里程碑必须弹窗确认。"""
    handler = function_body(SOURCE, "    def _validation_handle_imu_frame_line(")
    committed = handler[handler.index("first_commit and commit_was_pending"):]
    assert "messagebox.showinfo" in committed
    assert "flight_release" in committed


def test_v0_page_exposes_a_visible_history_selector_and_resume_control() -> None:
    """历史结果必须能主动选取，不能只靠"自动加载最新文件"碰运气。"""
    page = function_body(SOURCE, "    def _build_validation_page(")
    assert "validation_history_combo" in page
    assert "_validation_load_selected_history" in page
    assert "_validation_refresh_history_choices" in page
    assert "validation_resume_button" in page
    # 恢复按钮必须带明确的交互样式，否则又会像上次那样"看着像禁用"。
    resume = page[page.index("validation_resume_button = ttk.Button"):]
    resume = resume[: resume.index(".pack(")]
    assert 'style="Secondary.TButton"' in resume


def test_v1_page_relinquishes_usb_and_exposes_guarded_apply_commit() -> None:
    page = function_body(V1_SOURCE, "    def _build_v1_page(")
    start = function_body(V1_SOURCE, "    def _v1_start_capture(")
    worker = function_body(V1_SOURCE, "    def _v1_capture_worker(")
    drain = function_body(V1_SOURCE, "    def _v1_drain_events(")
    assert "IMU 零偏、比例与正交性校准" in V1_SOURCE
    assert "应用基础/完整候选到 RAM" in page
    assert "确认后写入参数 Flash" in page
    assert "_v1_apply_candidate" in page
    assert "_v1_commit_candidate" in page
    assert "state=tk.DISABLED" in page
    assert "self.serial_transport.stop()" in start
    assert "threading.Thread" in start
    assert "CaptureLink" in worker
    assert 'link.send("IMUCAP STOP")' in worker
    assert 'link.send("IMUCAP DUMP")' in worker
    assert "persist_v1_capture" in worker
    assert "self.serial_transport.start(port, baud)" in drain


def test_panel_builds_the_reordered_v0_layout_without_connecting(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    calibration_dir = tmp_path / "calibration" / "airframe"
    monkeypatch.setattr(panel_evidence, "AIRFRAME_CALIBRATION_DIR", calibration_dir)
    monkeypatch.setattr(panel, "FIRMWARE_UPDATE_DIR", tmp_path / "firmware_updates")
    dialog_messages: list[tuple[str, str]] = []
    monkeypatch.setattr(
        panel.messagebox,
        "showwarning",
        lambda title, message: dialog_messages.append((title, message)),
    )
    monkeypatch.setattr(
        panel.messagebox,
        "showinfo",
        lambda title, message: dialog_messages.append((title, message)),
    )
    monkeypatch.setattr(panel.messagebox, "askyesno", lambda *_args, **_kwargs: True)
    try:
        app = panel.DronePanel()
    except tk.TclError as exc:
        pytest.skip(f"Tk display unavailable: {exc}")
    try:
        app.update_idletasks()
        labels = [app.notebook.tab(tab_id, "text") for tab_id in app.notebook.tabs()]
        # R-T1-5：状态监视工作台成为默认首页（作者裁决），排在“总览”之前。
        assert labels[:5] == [
            "状态监视", "总览", "校准", "维护 · 固件升级", "传感器",
        ]
        calibration_labels = [
            app.calibration_notebook.tab(tab_id, "text")
            for tab_id in app.calibration_notebook.tabs()
        ]
        assert calibration_labels == [
            "坐标系与极性", "IMU 零偏与比例", "遥控器", "舵机机械中心与行程",
            "光流与测距", "无桨控制链验收", "振动检测与滤波",
        ]
        # R-S1-1：四个传感器页收进“传感器”分组，顶层不再平铺它们。
        sensor_labels = [
            app.sensor_notebook.tab(tab_id, "text")
            for tab_id in app.sensor_notebook.tabs()
        ]
        assert sensor_labels == ["气压计", "IMU 监视（旧链）", "GPS / 磁力计", "光流", "电流计", "电池电压"]
        assert "气压计" not in labels
        assert "IMU 监视（旧链）" not in labels
        assert "GPS / 磁力计" not in labels
        assert str(app.baro_tab.master) == str(app.sensor_notebook)
        assert str(app.imu_tab.master) == str(app.sensor_notebook)
        assert str(app.gps_tab.master) == str(app.sensor_notebook)
        assert str(app.flow_sensor_tab.master) == str(app.sensor_notebook)
        assert str(app.current_tab.master) == str(app.sensor_notebook)
        # 总览的“打开气压计页/姿态页/GPS 页”跨两层跳转，直接 select 子页会抛 TclError。
        app._open_gps_tab()
        assert app.notebook.select() == str(app.sensor_group_tab)
        assert app.sensor_notebook.select() == str(app.gps_tab)
        app._open_imu_tab()
        assert app.sensor_notebook.select() == str(app.imu_tab)
        app._open_baro_tab()
        assert app.sensor_notebook.select() == str(app.baro_tab)
        assert "诊断 / 命令" in labels
        assert app.firmware_image_var.get().endswith("build\\Debug\\drone-H743.elf")
        assert str(app.firmware_start_button["state"]) == "disabled"
        assert str(app.v1_start_button["state"]) == "disabled"
        # 面板会自动恢复最近一次 V1 会话，因此这里的文案取决于本机 data/ 内容；
        # 只断言"没有把采集当成已经在跑"，不绑定开发机上的历史数据。
        assert "采集中" not in app.v1_status_var.get()
        assert str(app.log_box) not in app._body_pane.panes()
        assert app.validation_stage_tree.exists("level")
        assert app.validation_stage_tree.exists("positive_yaw")
        assert tuple(app.validation_stage_tree.get_children()) == tuple(
            stage.value for stage in panel.VALIDATION_UI_STAGES
        )
        assert "可选，不阻止验收" in app.validation_firmware_hash_var.get()
        assert str(app.validation_begin_button["state"]) == "disabled"
        assert str(app.validation_session_button["style"]) == "Primary.TButton"
        assert str(app.validation_commit_button["style"]) == "Warning.TButton"
        assert str(app.v1_apply_button["style"]) == "Warning.TButton"
        assert str(app.firmware_start_button["style"]) == "Warning.TButton"
        assert isinstance(app.validation_tab, panel.VerticalScrolledFrame)
        assert isinstance(app.v1_tab, panel.VerticalScrolledFrame)
        assert isinstance(app.v2_tab, panel.VerticalScrolledFrame)
        assert isinstance(app.firmware_tab, panel.VerticalScrolledFrame)
        assert app.ui_scaling <= app.ui_native_scaling

        # The two target lines merge only when their sequence matches.
        app._transport_connected = lambda: True  # type: ignore[method-assign]
        app.validation_props_removed_var.set(True)
        app.validation_power_safe_var.set(True)
        app.validation_session_active = True
        app._update_imu_line(
            "IMU sample valid=1 source=stabilizer_snapshot frame=legacy_intermediate "
            "units=mg_mdps_cdeg contract=1 migration=0x00 ts_ms=1250 seq=1234 "
            "bias=1 armed=0 m1=1000 m2=1000 temp_cdeg=2500"
        )
        app._update_imu_line(
            "IMU sample seq=1234 ax=100 ay=-200 az=975 gx=1500 gy=-2500 gz=500 "
            "roll=123 pitch=-456 yaw=789 fusion_flags=0x00 ferr_cdeg=100 "
            "ftrig_milli=0 fcorr=10"
        )

        assert app.validation_samples[panel.ValidationStage.LEVEL] == []
        assert app.validation_latest_values["frame"] == "legacy_intermediate"
        assert str(app.validation_begin_button["state"]) == "normal"
        assert "准备完成" in app.validation_next_action_var.get()

        app._validation_begin_stage()
        app._update_imu_line(
            "IMU sample valid=1 source=stabilizer_snapshot frame=legacy_intermediate "
            "units=mg_mdps_cdeg contract=1 migration=0x00 ts_ms=1350 seq=1235 "
            "bias=1 armed=0 m1=1000 m2=1000 temp_cdeg=2500"
        )
        app._update_imu_line(
            "IMU sample seq=1235 ax=100 ay=-200 az=975 gx=1500 gy=-2500 gz=500 "
            "roll=123 pitch=-456 yaw=789 fusion_flags=0x00 ferr_cdeg=100 "
            "ftrig_milli=0 fcorr=11"
        )
        samples = app.validation_samples[panel.ValidationStage.LEVEL]
        assert len(samples) == 1
        assert samples[0].timestamp_s == pytest.approx(1.35)
        assert samples[0].accel_g == pytest.approx((0.1, -0.2, 0.975))
        assert len(app.validation_raw_rows) == 1
        # A later target sample becoming armed must terminate the same stage.
        app._update_imu_line(
            "IMU sample valid=1 source=stabilizer_snapshot frame=legacy_intermediate "
            "units=mg_mdps_cdeg contract=1 migration=0x00 ts_ms=1450 seq=1236 "
            "bias=1 armed=1 m1=1200 m2=1000 temp_cdeg=2500"
        )
        app._update_imu_line(
            "IMU sample seq=1236 ax=0 ay=0 az=1000 gx=0 gy=0 gz=0 "
            "roll=0 pitch=0 yaw=0 fusion_flags=0 ferr_cdeg=0 ftrig_milli=0 fcorr=2"
        )

        assert app.validation_active_stage is None
        assert panel.ValidationStage.LEVEL in app.validation_unsupported_stages
        assert len(app.validation_samples[panel.ValidationStage.LEVEL]) == 1
        assert app.validation_stage_tree.set("level", "status") == "FAIL"
        assert "armed=1" in app.validation_session_var.get()

        # The three reverse static poses stay in the file schema for legacy
        # evidence, but the standard workflow must not ask the user for them.
        legacy_optional_stages = {
            panel.ValidationStage.NOSE_DOWN,
            panel.ValidationStage.RIGHT_SIDE_UP,
            panel.ValidationStage.INVERTED,
        }
        assert all(
            not app.validation_stage_tree.exists(stage.value)
            for stage in legacy_optional_stages
        )
        app.validation_stage_tree.selection_set(panel.ValidationStage.NOSE_UP.value)
        app._on_validation_stage_select()

        # Every finished/aborted/skipped step is recoverable from a PC-side
        # session file.  Preserve all old reverse-pose markers even though the
        # current UI no longer presents those steps.  Loading is historical
        # review, not target state.
        app.validation_skipped_stages.update(legacy_optional_stages)
        app.validation_finished_stages.update(legacy_optional_stages)
        app._validation_autosave_session(force=True)
        session_path = app.validation_session_path
        assert session_path is not None and session_path.is_file()
        level_samples_before_reload = tuple(
            app.validation_samples[panel.ValidationStage.LEVEL]
        )
        app._validation_stop_session()
        app.validation_samples = {
            stage: [] for stage in panel.STAGE_DEFINITIONS
        }
        app.validation_stage_results.clear()
        app.validation_unsupported_stages.clear()
        app.validation_skipped_stages.clear()
        app.validation_finished_stages.clear()
        app.validation_session_path = None
        app.validation_loaded_report_path = None

        app._validation_load_latest_artifact()

        assert tuple(
            app.validation_samples[panel.ValidationStage.LEVEL]
        ) == level_samples_before_reload
        assert panel.ValidationStage.LEVEL in app.validation_unsupported_stages
        assert legacy_optional_stages <= app.validation_skipped_stages
        assert legacy_optional_stages <= app.validation_finished_stages
        assert not app.validation_session_active
        assert str(app.validation_resume_button["state"]) == "normal"
        historical_text = " ".join(
            (
                app.validation_session_var.get(),
                app.validation_report_var.get(),
                app.validation_next_action_var.get(),
            )
        )
        assert "历史" in historical_text

        # Continuing a loaded session must keep prior evidence intact.
        app._validation_resume_session()
        assert app.validation_session_active
        assert tuple(
            app.validation_samples[panel.ValidationStage.LEVEL]
        ) == level_samples_before_reload
        assert app.validation_session_path == session_path

        # A candidate is not applicable until target provenance is fresh, the
        # aircraft is safe, and both independent mapping confirmations are set.
        app._update_imu_line(
            "IMU sample valid=1 source=stabilizer_snapshot frame=legacy_intermediate "
            "units=mg_mdps_cdeg contract=1 migration=0x00 ts_ms=2200 seq=2200 "
            "bias=1 armed=0 m1=1000 m2=1000 temp_cdeg=2500"
        )
        app._update_imu_line(
            "IMU sample seq=2200 ax=0 ay=0 az=1000 gx=0 gy=0 gz=0 "
            "roll=0 pitch=0 yaw=0 fusion_flags=0 ferr_cdeg=0 ftrig_milli=0 fcorr=2"
        )
        app.validation_candidate_matrix = (
            (-1, 0, 0),
            (0, -1, 0),
            (0, 0, 1),
        )
        app.validation_candidate_descriptor = "-x,-y,+z"
        app.validation_candidate_confidence = 0.99
        app.validation_axis_labels_confirmed_var.set(False)
        app.validation_mapping_confirmed_var.set(False)
        app._validation_refresh_orientation_controls()
        assert str(app.validation_apply_button["state"]) == "disabled"

        app.validation_axis_labels_confirmed_var.set(True)
        app._validation_refresh_orientation_controls()
        assert str(app.validation_apply_button["state"]) == "disabled"
        app.validation_mapping_confirmed_var.set(True)
        app.validation_power_safe_var.set(False)
        app._validation_refresh_orientation_controls()
        assert str(app.validation_apply_button["state"]) == "disabled"
        app.validation_power_safe_var.set(True)
        app._validation_refresh_orientation_controls()
        assert str(app.validation_apply_button["state"]) == "normal"

        sent_orientation_commands: list[str] = []

        def record_orientation_command(
            _function: int,
            label: str,
            payload: str = "",
            expect_reply: bool = True,
        ) -> None:
            del expect_reply
            sent_orientation_commands.append(payload or label)

        app._send_proto = record_orientation_command  # type: ignore[method-assign]
        app._validation_apply_candidate()
        assert sent_orientation_commands[-1] == "IMUFRAME APPLY -x,-y,+z"
        assert app.validation_orientation_pending == "apply"
        assert app._validation_command_allowed("IMUFRAME APPLY -x,-y,+z")

        # The target's status response is authoritative: it confirms RAM state
        # and consumes the exact command grant so the diagnostics box cannot
        # replay it.
        app._handle_board_line(
            "IMUFRAME event=applied base=legacy_intermediate_v1 "
            "active=-x,-y,+z active_code=3 persisted=+x,+y,+z persisted_code=0 dirty=1"
        )
        assert app.validation_orientation_values["active"] == "-x,-y,+z"
        assert app.validation_candidate_applied
        assert not app.validation_candidate_committed
        assert app.validation_orientation_pending == ""
        assert not app._validation_command_allowed("IMUFRAME APPLY -x,-y,+z")
        assert str(app.validation_verify_button["state"]) == "normal"

        # B discards the pre-application samples.  C remains blocked until all
        # six visible stages have direct canonical PASS results.
        app._validation_begin_candidate_verification()
        assert app.validation_verification_mode
        assert all(not samples for samples in app.validation_samples.values())
        assert app.validation_finished_stages == set()
        app.validation_samples[panel.ValidationStage.LEVEL] = [
            level_samples_before_reload[0]
        ]
        workflow_session = app._validation_autosave_session(force=True)
        assert workflow_session is not None
        workflow_path = workflow_session.with_name(
            workflow_session.name.removesuffix("_session.json") + "_workflow.json"
        )
        assert workflow_path.is_file()
        workflow = app._validation_load_workflow(workflow_session)
        assert workflow is not None
        assert workflow["phase"] == "ram_verification"
        assert workflow["candidate_descriptor"] == "-x,-y,+z"
        app._update_imu_line(
            "IMU sample valid=1 source=stabilizer_snapshot frame=canonical_flu_ram "
            "units=mg_mdps_cdeg contract=1 migration=0x00 orientation=3 ts_ms=2300 seq=2300 "
            "bias=1 armed=0 m1=1000 m2=1000 temp_cdeg=2500"
        )
        app._update_imu_line(
            "IMU sample seq=2300 ax=0 ay=0 az=1000 gx=0 gy=0 gz=0 "
            "roll=0 pitch=0 yaw=0 fusion_flags=0 ferr_cdeg=0 ftrig_milli=0 fcorr=2"
        )
        pass_result = SimpleNamespace(
            status=panel.ValidationStatus.PASS,
            canonical_match_status=panel.ValidationStatus.PASS,
            gyro_direction_matches_flu=True,
            attitude_direction_matches_flu=True,
        )
        app._validation_static_fusion_consistency = (  # type: ignore[method-assign]
            lambda _stage: (panel.ValidationStatus.PASS, 0.0, "synthetic PASS")
        )
        for stage in panel.VALIDATION_UI_STAGES[:-1]:
            app.validation_finished_stages.add(stage)
            app.validation_stage_results[stage] = pass_result
        app._validation_refresh_orientation_controls()
        assert not app.validation_candidate_verified
        assert str(app.validation_commit_button["state"]) == "disabled"
        sent_before_blocked_commit = list(sent_orientation_commands)
        app._validation_commit_candidate()
        assert sent_orientation_commands == sent_before_blocked_commit
        assert any("6 个必要步骤必须全部 PASS" in message for _title, message in dialog_messages)

        final_stage = panel.VALIDATION_UI_STAGES[-1]
        app.validation_finished_stages.add(final_stage)
        app.validation_stage_results[final_stage] = pass_result
        app._validation_refresh_orientation_controls()
        assert app.validation_candidate_verified
        assert str(app.validation_commit_button["state"]) == "normal"

        app._validation_commit_candidate()
        assert sent_orientation_commands[-1] == "IMUFRAME COMMIT"
        assert app.validation_orientation_pending == "commit"
        assert app._validation_command_allowed("IMUFRAME COMMIT")
        app._handle_board_line(
            "IMUFRAME event=committed base=legacy_intermediate_v1 "
            "active=-x,-y,+z persisted=-x,-y,+z dirty=0"
        )
        assert app.validation_candidate_committed
        assert app.validation_orientation_pending == ""
        assert not app._validation_command_allowed("IMUFRAME COMMIT")
        assert "flight_release仍为false" in app.validation_session_var.get()
        assert str(app.validation_commit_button["state"]) == "disabled"
        assert app.validation_orientation_audit_path is not None
        audit_text = app.validation_orientation_audit_path.read_text(encoding="utf-8")
        assert '"flash_writes": 1' in audit_text
        assert '"flight_release": false' in audit_text
    finally:
        app.destroy()


def _python_class_methods(path: Path, class_name: str) -> dict[str, ast.FunctionDef]:
    module = ast.parse(path.read_text(encoding="utf-8"))
    owner = next(
        node for node in module.body
        if isinstance(node, ast.ClassDef) and node.name == class_name
    )
    return {
        node.name: node for node in owner.body if isinstance(node, ast.FunctionDef)
    }


def _python_class_assignments(path: Path, class_name: str) -> set[str]:
    module = ast.parse(path.read_text(encoding="utf-8"))
    owner = next(
        node for node in module.body
        if isinstance(node, ast.ClassDef) and node.name == class_name
    )
    names: set[str] = set()
    for node in owner.body:
        if isinstance(node, ast.Assign):
            names.update(
                target.id for target in node.targets if isinstance(target, ast.Name)
            )
    return names


def _python_top_functions(path: Path) -> dict[str, ast.FunctionDef]:
    return {
        node.name: node
        for node in ast.parse(path.read_text(encoding="utf-8")).body
        if isinstance(node, ast.FunctionDef)
    }


def _python_ast_sha256(node: ast.AST) -> str:
    return hashlib.sha256(ast.dump(node, include_attributes=False).encode()).hexdigest()


def test_s6_increment10_validation_and_evidence_ast_owners() -> None:
    legacy_path = ROOT / "tools" / "drone_tcp_panel.py"
    evidence_path = ROOT / "tools" / "panel_lib" / "evidence.py"
    validation_path = ROOT / "tools" / "panel_lib" / "pages" / "validation_v0.py"
    legacy = _python_class_methods(legacy_path, "DronePanel")
    evidence = _python_class_methods(evidence_path, "EvidenceMixin")
    validation = _python_class_methods(validation_path, "ValidationV0PageMixin")

    assert INCREMENT10_PARENT == "69bf4158977c62f65a78f7dc4b61b8a9f80275b7"
    assert set(validation) == set(V0_PAGE_AST_SHA256)
    assert set(evidence) == set(EVIDENCE_AST_SHA256)
    assert (set(validation) | set(evidence)).isdisjoint(legacy)
    assert {name: _python_ast_sha256(validation[name]) for name in V0_PAGE_AST_SHA256} == V0_PAGE_AST_SHA256
    assert {name: _python_ast_sha256(evidence[name]) for name in EVIDENCE_AST_SHA256} == EVIDENCE_AST_SHA256
    assert len(validation["_build_validation_page"].body) == 117

    helper_owners = {
        **_python_top_functions(validation_path),
        **_python_top_functions(evidence_path),
    }
    assert set(helper_owners) == set(VALIDATION_HELPER_AST_SHA256)
    assert {
        name: _python_ast_sha256(helper_owners[name])
        for name in VALIDATION_HELPER_AST_SHA256
    } == VALIDATION_HELPER_AST_SHA256

    assert _python_class_assignments(validation_path, "ValidationV0PageMixin") == {
        "VALIDATION_STEPS",
        "_STAGE_ROW_TAGS",
        "_GATE_INDICATORS",
        "_ORIENTATION_EVENT_STYLES",
        "IMU_HEALTH_LABELS",
    }
    assert panel.ValidationV0PageMixin is validation_v0_page.ValidationV0PageMixin
    assert panel.EvidenceMixin is panel_evidence.EvidenceMixin
    for owner, hashes in (
        (validation_v0_page.ValidationV0PageMixin, V0_PAGE_AST_SHA256),
        (panel_evidence.EvidenceMixin, EVIDENCE_AST_SHA256),
    ):
        for name in hashes:
            assert getattr(panel.DronePanel, name) is getattr(owner, name)
    for name in panel_evidence.__all__:
        assert getattr(panel, name) is getattr(panel_evidence, name), name
    for name in validation_v0_page.__all__:
        assert getattr(panel, name) is getattr(validation_v0_page, name), name
    assert panel.DronePanel.VALIDATION_STEPS == (
        "坐标发现", "RAM 复验", "写入 Flash"
    )
    assert all(
        validation_v0_page.UI_PALETTE[name] == panel.UI_PALETTE[name]
        for name in validation_v0_page.UI_PALETTE
    )

    for name in ("_validation_autosave_session", "_validation_autosave_workflow"):
        first = evidence[name].body[0]
        assert isinstance(first, ast.If)
        assert isinstance(first.test, ast.Attribute)
        assert first.test.attr == "validation_loaded_history"
        assert len(first.body) == 1 and isinstance(first.body[0], ast.Return)
        assert isinstance(first.body[0].value, ast.Constant)
        assert first.body[0].value.value is None

    env = os.environ.copy()
    env.pop("PYTHONPATH", None)
    result = subprocess.run(
        [
            sys.executable,
            "-c",
            (
                "import drone_tcp_panel as p; "
                "import panel_lib.evidence as e; "
                "import panel_lib.pages.validation_v0 as v; "
                "assert p.DronePanel._validation_autosave_session is "
                "e.EvidenceMixin._validation_autosave_session; "
                "assert p.DronePanel._build_validation_page is "
                "v.ValidationV0PageMixin._build_validation_page"
            ),
        ],
        cwd=ROOT / "tools",
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=20,
    )
    assert result.returncode == 0, result.stdout

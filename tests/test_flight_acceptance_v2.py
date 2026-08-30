from __future__ import annotations

import ast
import hashlib
import json
import math
import os
from dataclasses import FrozenInstanceError, replace
from pathlib import Path
import subprocess
import sys
import pytest

from tools import drone_tcp_panel as legacy_panel
from tools.flight_acceptance_v2 import (
    ACCEPTANCE_CAPTURE_SOURCE, ACCEPTANCE_PROVENANCE, AcceptanceStatus,
    PhysicalConfirmation, STAGE_DEFINITIONS, UNSUPPORTED_STAGES, V2Sample,
    V2Stage, V2Thresholds, build_v2_report, report_from_dict,
    report_from_json, report_to_dict, report_to_json,
    validate_report_for_application,
)
from tools.panel_lib.pages import acceptance_v2 as acceptance_v2_page


ROOT = Path(__file__).resolve().parents[1]
LEGACY_PANEL_PATH = ROOT / "tools" / "drone_tcp_panel.py"
V2_PAGE_PATH = ROOT / "tools" / "panel_lib" / "pages" / "acceptance_v2.py"
INCREMENT9_PARENT = "4d87d78f7497318737f3b3645f812ce3dd9977e7"
V2_PAGE_AST_SHA256 = {
    "_build_v2_page": "88c22660c867905d80525a96d22f1620bb8f021e2e989806dd03b28e859340f2",
    "_v2_start": "f462ccdc451e156315c59e34efe38fc8ca886902a0e322dccb0004856e81156e",
    "_v2_set_stage": "9c9f56d725688ceda0e73aa1a11edb0d0082addac16a13e24cd29a76c7300fe1",
    "_v2_stop": "f546e12abedd6833bf4e533f083974b139a6a4e02253916be273ed160a4f49d8",
    "_v2_tick": "a8c65b3513ef1ca1b278bc7d1ff97e639dc4f6d39161c2280f5d40930a5608e3",
    "_v2_handle_line": "f70b04a3dccf56bc38abc6f381da2035b3445155876f4437d7908a7faecc1446",
}


def sample(stage: V2Stage, sequence: int = 1, **overrides: object) -> V2Sample:
    values: dict[str, object] = dict(
        timestamp_s=sequence * 0.02, sequence=sequence, stage=stage,
        frame_id="FLU", frame_contract_version=1, orientation_code=3,
        calibration_generation=7, firmware_id="fw-test", session_id="session-v2-123",
        capture_source=ACCEPTANCE_CAPTURE_SOURCE, provenance=ACCEPTANCE_PROVENANCE,
        v0_record_id="v0-flu-persisted-3", v1_record_id="v1-room-temp-7",
        lease_id="lease-123", lease_issued_at_s=0.0, lease_expires_at_s=100.0,
        v0_persisted=True, v1_room_temp_valid=True, props_removed=True,
        acceptance_mode=True, lease_valid=True, esc_enabled=False, esc_ccr_1=0, esc_ccr_2=0,
    )
    if stage in {V2Stage.RC_CENTER, V2Stage.RC_POSITIVE_ROLL, V2Stage.RC_POSITIVE_PITCH, V2Stage.RC_POSITIVE_YAW}:
        values.update(rc_roll_us=1500.0, rc_pitch_us=1500.0, rc_yaw_us=1500.0)
        main = {V2Stage.RC_POSITIVE_ROLL: "rc_roll_us", V2Stage.RC_POSITIVE_PITCH: "rc_pitch_us", V2Stage.RC_POSITIVE_YAW: "rc_yaw_us"}.get(stage)
        if main: values[main] = 1750.0
    elif stage in {V2Stage.NAV_STATIC, V2Stage.NAV_FORWARD, V2Stage.NAV_LEFT}:
        values.update(nav_velocity_x_mps=0.01, nav_velocity_y_mps=-0.01)
        if stage is V2Stage.NAV_FORWARD: values.update(nav_velocity_x_mps=0.20, nav_velocity_y_mps=0.04)
        elif stage is V2Stage.NAV_LEFT: values.update(nav_velocity_x_mps=0.04, nav_velocity_y_mps=0.20)
    elif stage is V2Stage.RESTORE_POSITIVE_ROLL:
        values.update(roll_angle_deg=12.0, roll_rate_dps=4.0, restoring_roll_moment=-0.30, damping_roll_moment=-0.10)
    elif stage is V2Stage.RESTORE_POSITIVE_PITCH:
        values.update(pitch_angle_deg=12.0, pitch_rate_dps=4.0, restoring_pitch_moment=-0.30, damping_pitch_moment=-0.10)
    elif stage in {V2Stage.SERVO_ALPHA_POSITIVE, V2Stage.SERVO_ALPHA_NEGATIVE}:
        delta = 50.0 if stage is V2Stage.SERVO_ALPHA_POSITIVE else -50.0
        values.update(servo_alpha_center_us=1500.0, servo_alpha_command_us=1500.0 + delta, servo_alpha_feedback_us=1500.0 + delta, servo_alpha_feedback_valid=True)
    elif stage in {V2Stage.SERVO_BETA_POSITIVE, V2Stage.SERVO_BETA_NEGATIVE}:
        delta = 50.0 if stage is V2Stage.SERVO_BETA_POSITIVE else -50.0
        values.update(servo_beta_center_us=1500.0, servo_beta_command_us=1500.0 + delta, servo_beta_feedback_us=1500.0 + delta, servo_beta_feedback_valid=True)
    elif stage is V2Stage.FAILSAFE:
        values.update(link_present=False, failsafe_elapsed_ms=50.0, failsafe_active=True, control_mode="safe")
    values.update(overrides)
    return V2Sample(**values)  # type: ignore[arg-type]


def confirmations(**overrides: object) -> tuple[PhysicalConfirmation, ...]:
    result = []
    confirmation_times = {
        V2Stage.SERVO_ALPHA_POSITIVE: 3.8,
        V2Stage.SERVO_ALPHA_NEGATIVE: 4.2,
        V2Stage.SERVO_BETA_POSITIVE: 4.6,
        V2Stage.SERVO_BETA_NEGATIVE: 5.0,
    }
    for stage in (V2Stage.SERVO_ALPHA_POSITIVE, V2Stage.SERVO_ALPHA_NEGATIVE, V2Stage.SERVO_BETA_POSITIVE, V2Stage.SERVO_BETA_NEGATIVE):
        values: dict[str, object] = dict(
            stage=stage, confirmed=True,
            observed_direction=STAGE_DEFINITIONS[stage].expected_mechanical_direction,
            note="operator watched marked tilt vector with props removed", operator_id="operator-1",
            confirmed_at_s=confirmation_times[stage], session_id="session-v2-123", firmware_id="fw-test",
            calibration_generation=7, lease_id="lease-123", props_removed=True,
        )
        values.update(overrides); result.append(PhysicalConfirmation(**values))  # type: ignore[arg-type]
    return tuple(result)


def complete_samples(count: int = 20, step_s: float = 0.02) -> tuple[V2Sample, ...]:
    evidence: list[V2Sample] = []; sequence = 1
    for stage, definition in STAGE_DEFINITIONS.items():
        if not definition.required: continue
        for index in range(count):
            extra: dict[str, object] = {"timestamp_s": sequence * step_s}
            if stage is V2Stage.FAILSAFE:
                if index < 3: extra.update(link_present=True, failsafe_elapsed_ms=0.0, failsafe_active=False, control_mode="acceptance")
                elif index == 3: extra.update(link_present=False, failsafe_elapsed_ms=0.0, failsafe_active=False, control_mode="acceptance")
                else: extra.update(link_present=False, failsafe_elapsed_ms=(index - 3) * 50.0, failsafe_active=True, control_mode="safe")
            evidence.append(sample(stage, sequence, **extra)); sequence += 1
    return tuple(evidence)


def replace_stage(evidence: tuple[V2Sample, ...], stage: V2Stage, transform) -> tuple[V2Sample, ...]:
    result = []; index = 0
    for item in evidence:
        if item.stage is stage: result.append(transform(item, index)); index += 1
        else: result.append(item)
    return tuple(result)


def passing_report():
    return build_v2_report(complete_samples(), physical_confirmations=confirmations(), created_at="2026-08-28T20:00:00+08:00")


def test_complete_v2a_passes_but_never_releases_flight() -> None:
    report = passing_report()
    assert report.status is AcceptanceStatus.PASS and report.global_gate.status is AcceptanceStatus.PASS
    assert report.flight_release is False and report.evidence_only is True
    assert all(report.stage_result(stage).status is AcceptanceStatus.UNSUPPORTED for stage in UNSUPPORTED_STAGES)
    with pytest.raises(FrozenInstanceError): report.flight_release = True  # type: ignore[misc]


@pytest.mark.parametrize("kwargs", [
    {"rc_sign_fraction_min": math.inf}, {"nav_cross_rms_ratio_max": -1.0},
    {"servo_command_fraction_min": 1.1}, {"required_stage_min_samples": 0},
    {"restoring_angle_min_deg": 20.0, "restoring_angle_max_deg": 8.0},
    {"nav_positive_fraction_min": 0.5}, {"failsafe_latency_max_ms": 601.0},
    {"nav_cross_median_ratio_max": 0.9, "nav_cross_rms_ratio_max": 0.5},
])
def test_thresholds_strict(kwargs: dict[str, object]) -> None:
    with pytest.raises((TypeError, ValueError)): V2Thresholds(**kwargs)  # type: ignore[arg-type]


def test_thresholds_serialized_and_metric_integrity_protected() -> None:
    payload = report_to_dict(passing_report())
    assert payload["thresholds"]["failsafe_latency_max_ms"] == 600.0
    payload["stages"][0]["metrics"][0]["value"] = 999.0
    with pytest.raises(ValueError, match="integrity"): report_from_dict(payload)


def test_zero_sample_pass_semantically_impossible() -> None:
    report = passing_report(); zero = tuple(replace(item, sample_count=0) for item in report.stages)
    with pytest.raises(ValueError, match="nonzero evidence"):
        replace(report, sample_count=0, global_gate=replace(report.global_gate, sample_count=0), stages=zero, integrity_sha256="AUTO")


def test_sequence_timestamp_and_context_are_strict() -> None:
    evidence = list(complete_samples()); evidence[1] = replace(evidence[1], sequence=evidence[0].sequence)
    with pytest.raises(ValueError, match="strictly increasing"): build_v2_report(evidence, physical_confirmations=confirmations())
    with pytest.raises(ValueError, match="nonnegative"): sample(V2Stage.NAV_FORWARD, timestamp_s=-1.0)
    with pytest.raises(ValueError, match="capture_source"): sample(V2Stage.NAV_FORWARD, capture_source="guess")
    mixed = list(complete_samples()); mixed[-1] = replace(mixed[-1], session_id="other")
    with pytest.raises(ValueError, match="mixes"): build_v2_report(mixed, physical_confirmations=confirmations())


def test_lease_and_physical_confirmation_are_bound() -> None:
    with pytest.raises(ValueError, match="lease"): sample(V2Stage.NAV_FORWARD, timestamp_s=101.0)
    report = build_v2_report(complete_samples(), physical_confirmations=confirmations(lease_id="old"))
    assert report.status is AcceptanceStatus.FAIL and "physical_props_or_servo_confirmation" in report.global_gate.reasons
    report = build_v2_report(complete_samples(), physical_confirmations=confirmations(props_removed=False))
    assert report.global_gate.status is AcceptanceStatus.FAIL


def test_required_stage_needs_count_and_duration() -> None:
    assert build_v2_report(complete_samples(1), physical_confirmations=confirmations()).status is AcceptanceStatus.FAIL
    assert build_v2_report(complete_samples(step_s=0.0001), physical_confirmations=confirmations()).status is AcceptanceStatus.FAIL


def test_rc_and_servo_sign_consistency() -> None:
    evidence = replace_stage(complete_samples(), V2Stage.RC_POSITIVE_ROLL, lambda s, i: replace(s, rc_roll_us=1250.0) if i < 9 else s)
    evidence = replace_stage(evidence, V2Stage.SERVO_ALPHA_POSITIVE, lambda s, i: replace(s, servo_alpha_command_us=1450.0, servo_alpha_feedback_us=1450.0) if i < 9 else s)
    report = build_v2_report(evidence, physical_confirmations=confirmations())
    assert report.stage_result(V2Stage.RC_POSITIVE_ROLL).status is AcceptanceStatus.FAIL
    assert report.stage_result(V2Stage.SERVO_ALPHA_POSITIVE).status is AcceptanceStatus.FAIL


def test_nav_cross_cancellation_and_tiny_restoring_rejected() -> None:
    evidence = replace_stage(complete_samples(), V2Stage.NAV_FORWARD, lambda s, i: replace(s, nav_velocity_y_mps=100.0 if i % 2 == 0 else -100.0))
    report = build_v2_report(evidence, physical_confirmations=confirmations())
    assert report.stage_result(V2Stage.NAV_FORWARD).status is AcceptanceStatus.FAIL
    evidence = replace_stage(complete_samples(), V2Stage.RESTORE_POSITIVE_ROLL, lambda s, _i: replace(s, roll_rate_dps=1e-300, restoring_roll_moment=-1e-300, damping_roll_moment=-1e-300))
    assert build_v2_report(evidence, physical_confirmations=confirmations()).stage_result(V2Stage.RESTORE_POSITIVE_ROLL).status is AcceptanceStatus.FAIL


def test_failsafe_requires_baseline_chronology_and_persistence() -> None:
    evidence = replace_stage(complete_samples(), V2Stage.FAILSAFE, lambda s, i: replace(s, link_present=False, failsafe_elapsed_ms=float(i * 10), failsafe_active=(i == 0), control_mode="safe" if i == 0 else "manual"))
    assert build_v2_report(evidence, physical_confirmations=confirmations()).stage_result(V2Stage.FAILSAFE).status is AcceptanceStatus.FAIL
    with pytest.raises(ValueError, match="nonnegative"): sample(V2Stage.FAILSAFE, failsafe_elapsed_ms=-1.0)
    evidence = replace_stage(complete_samples(), V2Stage.FAILSAFE, lambda s, i: replace(s, failsafe_elapsed_ms=500.0 - i))
    assert build_v2_report(evidence, physical_confirmations=confirmations()).stage_result(V2Stage.FAILSAFE).status is AcceptanceStatus.FAIL


def test_strict_json_and_application_recompute() -> None:
    report = passing_report(); text = report_to_json(report)
    assert report_from_json(text) == report
    assert validate_report_for_application(report, complete_samples(), confirmations())
    changed = replace_stage(complete_samples(), V2Stage.NAV_FORWARD, lambda s, _i: replace(s, nav_velocity_x_mps=0.3))
    with pytest.raises(ValueError, match="does not exactly match"): validate_report_for_application(report, changed, confirmations())
    duplicate = text.replace('"schema": 2,', '"schema": 2, "schema": 2,', 1)
    with pytest.raises(ValueError, match="duplicate JSON key"): report_from_json(duplicate)
    payload = report_to_dict(report); payload["flight_release"] = True
    with pytest.raises(ValueError, match="unsafe"): report_from_dict(payload)
    payload = report_to_dict(report); payload["stages"][0]["metrics"][0]["value"] = float("nan")
    with pytest.raises(ValueError, match="non-finite"): report_from_json(json.dumps(payload, allow_nan=True))


def _page_methods(path: Path, class_name: str) -> dict[str, ast.FunctionDef]:
    module = ast.parse(path.read_text(encoding="utf-8"))
    owner = next(
        node for node in module.body
        if isinstance(node, ast.ClassDef) and node.name == class_name
    )
    return {
        node.name: node for node in owner.body if isinstance(node, ast.FunctionDef)
    }


def _page_ast_sha256(node: ast.AST) -> str:
    return hashlib.sha256(ast.dump(node, include_attributes=False).encode()).hexdigest()


def test_s6_increment9_v2_page_ast_owner_and_forwarding() -> None:
    owned = _page_methods(V2_PAGE_PATH, "AcceptanceV2PageMixin")
    legacy = _page_methods(LEGACY_PANEL_PATH, "DronePanel")

    assert INCREMENT9_PARENT == "4d87d78f7497318737f3b3645f812ce3dd9977e7"
    assert set(owned) == set(V2_PAGE_AST_SHA256)
    assert set(owned).isdisjoint(legacy)
    assert {name: _page_ast_sha256(owned[name]) for name in V2_PAGE_AST_SHA256} == V2_PAGE_AST_SHA256
    assert len(owned["_build_v2_page"].body) == 22
    assert legacy_panel.AcceptanceV2PageMixin is acceptance_v2_page.AcceptanceV2PageMixin
    for name in V2_PAGE_AST_SHA256:
        assert getattr(legacy_panel.DronePanel, name) is getattr(
            acceptance_v2_page.AcceptanceV2PageMixin, name
        )
    for name in (
        "FLIGHT_ACCEPTANCE_CALIBRATION_DIR", "dated_directory", "ensure_directory"
    ):
        assert getattr(acceptance_v2_page, name) is getattr(legacy_panel, name)

    env = os.environ.copy()
    env.pop("PYTHONPATH", None)
    result = subprocess.run(
        [
            sys.executable,
            "-c",
            (
                "import drone_tcp_panel as p; "
                "import panel_lib.pages.acceptance_v2 as v; "
                "assert p.DronePanel._v2_tick is v.AcceptanceV2PageMixin._v2_tick"
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

from __future__ import annotations

import ast
import hashlib
import math
import os
from pathlib import Path
import subprocess
import sys

import pytest

from tools import drone_tcp_panel as legacy_panel
from tools.ground_calibration import (
    FlowRangeSample,
    GroundCalibrationError,
    analyze_flow_axis,
    analyze_flow_zero,
    analyze_rotation_compensation,
    fit_range_two_point,
    validate_servo_geometry,
)
from tools.panel_lib.pages import servo_debug as servo_debug_page
from tools.panel_lib.pages import vibration as vibration_page


ROOT = Path(__file__).resolve().parents[1]
PANEL_SOURCE = (ROOT / "tools" / "drone_tcp_panel.py").read_text(encoding="utf-8")
MECHANICAL_PAGE_SOURCE = (
    ROOT / "tools" / "panel_lib" / "pages" / "mechanical.py"
).read_text(encoding="utf-8")
SERVO_DEBUG_PAGE_PATH = ROOT / "tools" / "panel_lib" / "pages" / "servo_debug.py"
VIBRATION_PAGE_PATH = ROOT / "tools" / "panel_lib" / "pages" / "vibration.py"
VIBRATION_PAGE_SOURCE = VIBRATION_PAGE_PATH.read_text(encoding="utf-8")

INCREMENT8_PARENT = "4eaf9f290d031a879f3baefa15f6e306ac9cbcf5"
VIBRATION_AST_SHA256 = {
    "_build_vibration_filter_page": "d6483841646aea2d7ae966623f75c441db8d11e01d983a95fa0a4db80e344106",
}
SERVO_DEBUG_AST_SHA256 = {
    "_build_servo_page": "d26d416f9a002efe30c3848cd717b123eac8ad698a13a1e723892309d06ea917",
    "_build_servo_tab": "47e4f39cf11240deea95976af1d20c84f196fc4a0574f432f3285474b3b08a3c",
    "_send_raw": "93c964e44ca39d44dadffe470117080adda5f25bd6e54ab3dd29b6d1b610ca2b",
    "_servo_values": "eb8e3632a5b07df9a26a46cc73ddd8c6ee40cc1c0ac2e0da8d514bff69b65484",
    "_servo_move": "4cc86c79336b760bb2b4ce8e8b297c1d2d8c13f36e86abcdcec8d594ba3f6d98",
    "_servo_mode": "3cd97ca97bb89bb6408c2a3fecf05c9b878f8cd42eb0daa74c98ca1efaca2c25",
    "_servo_enable": "886020ccc807b35540014783c354710fc6d42ccffde9e31f806f129942a99a54",
    "_servo_set_id": "464afb5be501ae2dbd2a9e56da8d4edf8db19ed6e2cb7dea79a8eb91cedcac37",
    "_servo_set_physical_id": "d1e72d59244db33f8aab98ea032ec37dee10ce655f71025542825d69e79563b4",
    "_servo_cmd": "96ea6e79bcb2f30ba523400b6c555344d335d100bb6cbdec00c5c34f39848177",
    "_servo_baud": "d5e442de8d889383f12f55f3f1cd85c89dae795e7fd8773cc06d5f3ea51e21cd",
    "_update_servo_ok_line": "e34aefe4ead010297387a1c03c366ce42be177ceb320461fab9fd403b521e6ff",
}


def function_body(name: str) -> str:
    if name == "_build_mechanical_calibration_page" or name.startswith("_mechanical_"):
        source = MECHANICAL_PAGE_SOURCE
    elif name == "_build_vibration_filter_page":
        source = VIBRATION_PAGE_SOURCE
    else:
        source = PANEL_SOURCE
    marker = f"    def {name}("
    start = source.index(marker)
    next_method = source.find("\n    def ", start + len(marker))
    return source[start:] if next_method < 0 else source[start:next_method]


def test_calibration_navigation_uses_function_names_instead_of_version_codes() -> None:
    build = function_body("_build_ui")
    assert 'self.notebook.add(calibration, text="校准")' in build
    for label in (
        "坐标系与极性", "IMU 零偏与比例", "遥控器", "舵机机械中心与行程",
        "光流与测距", "无桨控制链验收", "振动检测与滤波",
    ):
        assert label in build
    for opaque_label in ("坐标 V0", "IMU V1", "链路 V2A"):
        assert opaque_label not in build


def test_mechanical_page_is_guarded_and_only_claims_a_matching_target_readback() -> None:
    page = function_body("_build_mechanical_calibration_page")
    values = function_body("_mechanical_row_values")
    move = function_body("_mechanical_move")
    save = function_body("_mechanical_save_evidence")
    assert "已拆除全部桨叶" in page
    assert "validate_servo_geometry" in values
    assert "_validation_live_safety_gate" in move
    # 点动必须走保持型 SERVO JOG：一次性 SERVO MOVE 会被稳定环 500ms 强制刷新拉回。
    assert "SERVO JOG" in move
    assert "SERVO MOVE" not in move
    assert '_mechanical_target_matches_local("persisted")' in save
    assert '"target_parameters_written": persisted_match' in save
    assert '"flight_release": False' in save


def test_mechanical_page_jog_ux_covers_flu_guides_trim_and_release() -> None:
    page = function_body("_build_mechanical_calibration_page")
    nudge = function_body("_mechanical_nudge_center")
    stop = function_body("_mechanical_jog_stop")
    # 判向文字必须引用 FLU 契约方向，而不是含糊的"机构标记"自说自话。
    assert "向机体左侧(+Y)倾" in page
    assert "向机尾(−X)倾" in page
    assert "中点微调" in page
    assert "结束点动" in page
    # 中点微调与点动同一安全门；微调后中心仍满足两侧 ≥50 µs 几何约束。
    assert "_validation_live_safety_gate" in nudge
    assert "minimum + 50" in nudge and "maximum - 50" in nudge
    assert "SERVO JOG" in nudge
    assert '"SERVO JOG STOP"' in stop


def test_flow_page_covers_required_ground_checks_and_uses_compensated_velocity() -> None:
    stages = PANEL_SOURCE[PANEL_SOURCE.index("FLOW_CALIBRATION_STAGES = {"):
                          PANEL_SOURCE.index("DEFAULT_HOST =")]
    page = function_body("_build_flow_range_calibration_page")
    save = function_body("_flow_cal_save_report")
    for stage in ("static_zero", "forward_x", "left_y", "range_near", "range_far", "yaw_rotation"):
        assert stage in stages
    assert "补偿前/后 FLU 速度" in page
    assert '"target_parameters_written": False' in save


def test_vibration_page_is_a_non_actionable_placeholder_with_existing_filter_baseline() -> None:
    page = function_body("_build_vibration_filter_page")
    assert "1000 Hz" in page
    assert "80 Hz" in page
    assert "40 Hz" in page
    assert "不提供开始采集按钮" in page
    assert "command=" not in page


def sample(
    t: float,
    vx: float,
    vy: float,
    *,
    height: float | None = None,
    gyro: float | None = 0.0,
    vx_comp: float | None = None,
    vy_comp: float | None = None,
) -> FlowRangeSample:
    return FlowRangeSample(
        host_time_s=t,
        vx_m_s=vx,
        vy_m_s=vy,
        height_raw_m=height,
        gyro_z_dps=gyro,
        vx_compensated_m_s=vx_comp,
        vy_compensated_m_s=vy_comp,
    )


def test_static_flow_reports_zero_offset_and_noise() -> None:
    samples = [sample(i * 0.1, 0.01, -0.02) for i in range(10)]
    result = analyze_flow_zero(samples)
    assert result["sample_count"] == 10
    assert result["vx_mean_m_s"] == pytest.approx(0.01)
    assert result["vy_mean_m_s"] == pytest.approx(-0.02)
    assert result["speed_rms_m_s"] == pytest.approx(math.hypot(0.01, 0.02))


def test_positive_x_motion_checks_flu_sign_and_scale() -> None:
    samples = [sample(i * 0.1, 0.5, 0.02) for i in range(11)]
    result = analyze_flow_axis(samples, expected_axis="x", reference_distance_m=0.55)
    assert result["positive_sign_ok"] is True
    assert result["observed_distance_m"] == pytest.approx(0.5)
    assert result["axis_dominance_ratio"] == pytest.approx(25.0)
    assert result["diagnostic_scale"] == pytest.approx(1.1)


def test_negative_axis_motion_never_produces_a_writeable_scale() -> None:
    samples = [sample(i * 0.1, -0.4, 0.01) for i in range(11)]
    result = analyze_flow_axis(samples, expected_axis="x", reference_distance_m=0.4)
    assert result["positive_sign_ok"] is False
    assert result["diagnostic_scale"] is None


def test_two_point_range_fit_recovers_scale_and_offset() -> None:
    # true = 1.1 * measured + 0.02
    near = [sample(i * 0.1, 0.0, 0.0, height=0.254545) for i in range(6)]
    far = [sample(i * 0.1, 0.0, 0.0, height=0.890909) for i in range(6)]
    result = fit_range_two_point(
        near, far, near_reference_m=0.30, far_reference_m=1.00
    )
    assert result["range_scale"] == pytest.approx(1.1, rel=1e-5)
    assert result["range_offset_m"] == pytest.approx(0.02, abs=1e-5)


def test_rotation_requires_post_compensation_velocity() -> None:
    samples = [sample(i * 0.1, 0.1, -0.1, gyro=45.0) for i in range(6)]
    result = analyze_rotation_compensation(samples)
    assert result["supported"] is False
    assert "不能判定通过" in result["reason"]


def test_rotation_uses_compensated_velocity_when_target_exposes_it() -> None:
    samples = [
        sample(i * 0.1, 0.1, -0.1, gyro=45.0, vx_comp=0.01, vy_comp=-0.02)
        for i in range(6)
    ]
    result = analyze_rotation_compensation(samples)
    assert result["supported"] is True
    assert result["compensated_speed_rms_m_s"] == pytest.approx(math.hypot(0.01, 0.02))
    assert result["passed"] is True


def test_rotation_fails_when_motion_is_too_small_or_residual_is_too_large() -> None:
    samples = [
        sample(i * 0.1, 0.2, -0.2, gyro=5.0, vx_comp=0.10, vy_comp=-0.10)
        for i in range(6)
    ]
    result = analyze_rotation_compensation(samples)
    assert result["supported"] is True
    assert result["motion_ok"] is False
    assert result["residual_ok"] is False
    assert result["passed"] is False


@pytest.mark.parametrize(
    "values",
    [(1500, 1000, 2000), (1400, 900, 1850), (1600, 1300, 2100)],
)
def test_servo_geometry_accepts_ordered_safe_travel(values: tuple[int, int, int]) -> None:
    center, minimum, maximum = values
    validate_servo_geometry(center, minimum, maximum)


@pytest.mark.parametrize(
    "values",
    [(1500, 1500, 2000), (1500, 1000, 1500), (400, 300, 1000), (1500, 1490, 2000)],
)
def test_servo_geometry_rejects_unsafe_or_degenerate_travel(values: tuple[int, int, int]) -> None:
    center, minimum, maximum = values
    with pytest.raises(GroundCalibrationError):
        validate_servo_geometry(center, minimum, maximum)


def _class_methods(path: Path, class_name: str) -> dict[str, ast.FunctionDef]:
    module = ast.parse(path.read_text(encoding="utf-8"))
    owner = next(
        node for node in module.body
        if isinstance(node, ast.ClassDef) and node.name == class_name
    )
    return {
        node.name: node for node in owner.body if isinstance(node, ast.FunctionDef)
    }


def _ast_sha256(node: ast.AST) -> str:
    return hashlib.sha256(ast.dump(node, include_attributes=False).encode()).hexdigest()


def test_s6_increment8_page_owners_ast_forwarding() -> None:
    legacy = _class_methods(ROOT / "tools" / "drone_tcp_panel.py", "DronePanel")
    vibration = _class_methods(VIBRATION_PAGE_PATH, "VibrationPageMixin")
    servo_debug = _class_methods(SERVO_DEBUG_PAGE_PATH, "ServoDebugPageMixin")

    assert INCREMENT8_PARENT == "4eaf9f290d031a879f3baefa15f6e306ac9cbcf5"
    assert {name: _ast_sha256(vibration[name]) for name in VIBRATION_AST_SHA256} == VIBRATION_AST_SHA256
    assert {name: _ast_sha256(servo_debug[name]) for name in SERVO_DEBUG_AST_SHA256} == SERVO_DEBUG_AST_SHA256
    assert set(vibration) == set(VIBRATION_AST_SHA256)
    assert set(servo_debug) == set(SERVO_DEBUG_AST_SHA256)
    assert (set(vibration) | set(servo_debug)).isdisjoint(legacy)
    assert len(vibration["_build_vibration_filter_page"].body) == 10
    assert len(servo_debug["_build_servo_page"].body) == 8
    assert len(servo_debug["_build_servo_tab"].body) == 33
    assert legacy_panel.VibrationPageMixin is vibration_page.VibrationPageMixin
    assert legacy_panel.ServoDebugPageMixin is servo_debug_page.ServoDebugPageMixin
    for owner, hashes in (
        (vibration_page.VibrationPageMixin, VIBRATION_AST_SHA256),
        (servo_debug_page.ServoDebugPageMixin, SERVO_DEBUG_AST_SHA256),
    ):
        for name in hashes:
            assert getattr(legacy_panel.DronePanel, name) is getattr(owner, name)

    env = os.environ.copy()
    env.pop("PYTHONPATH", None)
    result = subprocess.run(
        [
            sys.executable,
            "-c",
            (
                "import drone_tcp_panel as p; "
                "import panel_lib.pages.servo_debug as s; "
                "import panel_lib.pages.vibration as v; "
                "assert p.DronePanel._servo_move is s.ServoDebugPageMixin._servo_move; "
                "assert p.DronePanel._build_vibration_filter_page is "
                "v.VibrationPageMixin._build_vibration_filter_page"
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

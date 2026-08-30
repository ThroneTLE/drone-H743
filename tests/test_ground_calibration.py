from __future__ import annotations

import math
from pathlib import Path

import pytest

from tools.ground_calibration import (
    FlowRangeSample,
    GroundCalibrationError,
    analyze_flow_axis,
    analyze_flow_zero,
    analyze_rotation_compensation,
    fit_range_two_point,
    validate_servo_geometry,
)


ROOT = Path(__file__).resolve().parents[1]
PANEL_SOURCE = (ROOT / "tools" / "drone_tcp_panel.py").read_text(encoding="utf-8")
MECHANICAL_PAGE_SOURCE = (
    ROOT / "tools" / "panel_lib" / "pages" / "mechanical.py"
).read_text(encoding="utf-8")


def function_body(name: str) -> str:
    source = (
        MECHANICAL_PAGE_SOURCE
        if name == "_build_mechanical_calibration_page" or name.startswith("_mechanical_")
        else PANEL_SOURCE
    )
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

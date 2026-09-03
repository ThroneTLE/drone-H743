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
FLOW_PAGE_SOURCE = (
    ROOT / "tools" / "panel_lib" / "pages" / "flow_ranging.py"
).read_text(encoding="utf-8")
FLOW_MONITOR_PAGE_SOURCE = (
    ROOT / "tools" / "panel_lib" / "pages" / "flow_monitor.py"
).read_text(encoding="utf-8")
SERVO_DEBUG_PAGE_PATH = ROOT / "tools" / "panel_lib" / "pages" / "servo_debug.py"
VIBRATION_PAGE_PATH = ROOT / "tools" / "panel_lib" / "pages" / "vibration.py"
VIBRATION_PAGE_SOURCE = VIBRATION_PAGE_PATH.read_text(encoding="utf-8")

INCREMENT8_PARENT = "4eaf9f290d031a879f3baefa15f6e306ac9cbcf5"
VIBRATION_AST_SHA256 = {
    "_build_vibration_filter_page": "d6483841646aea2d7ae966623f75c441db8d11e01d983a95fa0a4db80e344106",
}
SERVO_DEBUG_AST_SHA256 = {
    "_build_servo_page": "cdbca905951b7c69996a5f2c4093367c915ba16d76b45072cb605b249f7f8640",
    "_build_servo_tab": "8c939df106d33a9912c12315b036d8ec94b446f4057ee82f5c8b5c8076553345",
    "_servo_spin": "f7a1c49d31771e45a0e931da512d6a19fb5130701f5822fd1a29b6bef22e6733",
    "_refresh_servo_output_controls": "6f5c31f85e094d2cd5bc2c61626a44125db36241a8d08550fb8d5cd94f6c604b",
    "_send_raw": "bce3e02d68ff9b9ad4c69fa3b70c456fb18f203f0d0771471fab834663873b47",
    "_servo_values": "eb8e3632a5b07df9a26a46cc73ddd8c6ee40cc1c0ac2e0da8d514bff69b65484",
    # R-S7-7：PWM 调试页改走即时通路（NOW），三个方法随之重钉。
    "_servo_move_pwm_immediate": "f4bbc9011fc0b91bd29c0a216f9dd34a4cf47194e74ac8b3b19120802922eeec",
    "_servo_move": "0222f52f9f85c0e765f64fa88de8415165f0164d11fc35785302dc006e6d51a8",
    "_servo_move_all": "0111acb759e4d9cb51590a10a22ac483edf72477950db636e68329278a9e2c16",
    "_servo_mode": "605e3b441cf7618211518224f57c82dc257a916b550bd69ab1ac5f0f0cc9a429",
    "_servo_enable": "81171cf844bac2fedf76e0698446397ae18a9924552ce6a046d59cc75aa6e206",
    "_servo_set_id": "feecd52bcb00bc07d6af187071c26811cf227eb261cfc307510f22b0332def84",
    "_servo_set_physical_id": "0c80edcc0d69035d83f7a8930b428b597244aa3ac5577f4e08179a126d600a13",
    "_servo_cmd": "1af048dfd65fdb84e818357c3ce0e80f0ba01600fb5b7cf6ba90581f687a0547",
    "_servo_baud": "6275e9dd10a41cb0a0b442fd87012f630af638c6c73879c40337fee14c13c6dc",
    "_update_servo_ok_line": "e34aefe4ead010297387a1c03c366ce42be177ceb320461fab9fd403b521e6ff",
}


def function_body(name: str) -> str:
    if name == "_build_mechanical_calibration_page" or name.startswith("_mechanical_"):
        source = MECHANICAL_PAGE_SOURCE
    elif name == "_build_vibration_filter_page":
        source = VIBRATION_PAGE_SOURCE
    elif name.startswith(
        ("_flow_monitor_", "_build_sensor_flow_", "_build_flow_monitor_", "_update_flow_monitor_")
    ):
        source = FLOW_MONITOR_PAGE_SOURCE
    elif name.startswith(("_flow_", "_build_flow_", "_update_flow_", "_update_range_")):
        source = FLOW_PAGE_SOURCE
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


def test_sensor_group_collects_baro_imu_gps_flow_under_one_expandable_tab() -> None:
    """R-S1-1：四个传感器页收进顶层“传感器”分组，顶层不再平铺它们。"""

    build = function_body("_build_ui")
    assert 'self.notebook.add(sensors, text="传感器")' in build
    for label in ("气压计", "IMU 监视（旧链）", "GPS / 磁力计", "光流"):
        assert f'self.sensor_notebook.add(' in build
        assert f'text="{label}")' in build
    # 顶层必须彻底交出这三页，否则会出现两个入口。
    assert 'self.notebook.add(baro, text="气压计")' not in build
    assert 'self.notebook.add(imu, text="IMU 监视（旧链）")' not in build
    assert 'self.notebook.add(gps, text="GPS / 磁力计")' not in build
    # 容器归属也必须真的改到二级 Notebook 上，不能只改 .add() 调用点。
    for name in ("baro", "imu", "gps", "flow_sensor"):
        assert f"{name} = ttk.Frame(self.sensor_notebook," in build
    # “校准”分组及其子页与本次改动无关，必须原样保留。
    assert 'self.notebook.add(calibration, text="校准")' in build
    assert 'self.calibration_notebook.add(flow_range_scroll, text="光流与测距")' in build


def test_sensor_pages_stay_container_agnostic_and_navigation_follows_the_new_nesting() -> None:
    """页面实现只认 parent；跨层跳转与 IMU 轮询门必须跟着嵌套关系走。"""

    for name in ("_build_baro_page", "_build_imu_page", "_build_gps_page"):
        body = function_body(name)
        assert "self.notebook" not in body
        assert "sensor_notebook" not in body
    # 顶层 notebook.select(子页) 在嵌套后会抛 TclError，三个入口必须先选分组。
    select = function_body("_select_sensor_tab")
    assert "self.notebook.select(sensor_group_tab)" in select
    assert "self.sensor_notebook.select(tab)" in select
    for opener in ("_open_baro_tab", "_open_imu_tab", "_open_gps_tab"):
        body = function_body(opener)
        assert "self._select_sensor_tab(" in body
        assert "self.notebook.select(" not in body
    # 顶层 selection 只会等于“传感器”分组，IMU 可见性必须查二级 selection。
    tick = function_body("_imu_poll_tick")
    assert "self.sensor_notebook.select() == str(self.imu_tab)" in tick


def test_flow_sensor_tab_is_a_real_monitor_page_owned_by_its_own_module() -> None:
    """R-S1-2：占位页换成真实监控页，实现整体归 flow_monitor.py，不回流大文件。"""

    assert "_build_sensor_flow_placeholder_page" not in PANEL_SOURCE
    assert "建设中" not in FLOW_MONITOR_PAGE_SOURCE
    body = function_body("_build_sensor_flow_page")
    assert "PageTitle.TLabel" in body
    # 四项内容各有入口：质量/高度读数、速度曲线、可清零的累计位移。
    readouts = function_body("_build_flow_monitor_readouts")
    for key in ('"quality"', '"height"', '"velocity"', '"valid"'):
        assert key in readouts
    displacement = function_body("_build_flow_monitor_displacement")
    assert "_flow_monitor_reset_displacement" in displacement
    assert "_update_flow_monitor_velocity_plot" in FLOW_MONITOR_PAGE_SOURCE
    # 大文件只留装配点，页面实现不许再往里堆。
    for owned in ("_flow_monitor_handle_line", "_flow_monitor_integrate", "_flow_monitor_tick"):
        assert f"    def {owned}(" not in PANEL_SOURCE
        assert f"    def {owned}(" in FLOW_MONITOR_PAGE_SOURCE


def test_flow_monitor_poll_is_gated_by_its_own_tab_and_never_reuses_the_calibration_switch() -> None:
    """监控页必须自己发 FLOW?，且只在自己这一页被选中时发。"""

    tick = function_body("_imu_poll_tick")
    # R-T1-3 把这段门控整体搬进 panel_lib/pages/flow_monitor.py（面板那个 5500 行
    # 的文件只减不增）。断言跟着搬，语义不变。
    assert "self._flow_monitor_poll_tick(now)" in tick
    assert "flow_sensor_tab_visible" not in tick

    monitor = (ROOT / "tools" / "panel_lib" / "pages" / "flow_monitor.py").read_text(
        encoding="utf-8"
    )
    gate = monitor[monitor.index("def _flow_monitor_poll_tick"):]
    gate = gate[: gate.index("\n    def ", 1)]
    assert "self.sensor_notebook.select() == str(self.flow_sensor_tab)" in gate
    assert "now - self.flow_monitor_last_poll < FLOW_MONITOR_POLL_PERIOD_S" in gate
    assert 'self.transport.send_line("FLOW?")' in gate
    # 监控页不得挂在标定采集开关上；那条采集轮询仍原样留在面板里。
    body = gate[gate.index('"""', gate.index('"""') + 3) + 3:]
    assert "flow_cal_collecting" not in body
    assert 'if getattr(self, "flow_cal_collecting", False):' in tick


def test_flow_monitor_reset_clears_both_accumulators() -> None:
    """R-M5-5 之后固件也有里程计：归零要两份一起清。

    上位机那份直接清本地；固件那份只能由飞控自己清，所以发 FLOW ZERO——
    该命令只动累计位移，不碰控制位置与速度估计，因此不属于会扰动飞行的帧。
    """

    body = function_body("_flow_monitor_reset_displacement")
    assert "self.flow_monitor_dx_m = 0.0" in body
    assert "self.flow_monitor_track.clear()" in body
    assert 'self.transport.send_line("FLOW ZERO")' in body
    # 仍然只走安全门允许的通路，且不得改用会动控制状态的命令。
    assert '_validation_command_allowed("FLOW ZERO")' in body
    for forbidden in ("SERVO", "ARM", "PARAM SET"):
        assert forbidden not in body


def test_flow_monitor_integration_uses_real_elapsed_time_and_pauses_on_invalid_velocity() -> None:
    body = function_body("_flow_monitor_integrate")
    assert "dt = now - anchor[0]" in body
    assert "if not velocity_valid:" in body
    assert "self.flow_monitor_anchor = None" in body
    # 固定周期假设是禁止的：积分因子必须是实测 dt。
    assert "FLOW_MONITOR_MAX_INTEGRATION_DT_S" in body


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
    stages = FLOW_PAGE_SOURCE[FLOW_PAGE_SOURCE.index("FLOW_CALIBRATION_STAGES = {"):
                              FLOW_PAGE_SOURCE.index("class FlowRangingPageMixin")]
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
    assert len(servo_debug["_build_servo_page"].body) == 15
    assert len(servo_debug["_build_servo_tab"].body) == 42
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

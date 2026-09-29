"""台架刚度（挂砝码试验）：换算公式、前后一致性，以及它代替 m·g·d 之后拟合找回真值。

合成数据只用于单元测试：TWD 杆上模型（`fit.simulate_twd`，1 kHz 精确离散）+ 控制拍抖动的
250 Hz 抽样 + 陀螺噪声。台架是实测重心那组几何：杆在质心上方 d = 0.05 m，俯仰倾转轴在
质心下方 0.12 m，固件力臂也是 0.12 m（κ_几何 = 1）；回中刚度除了 m·g·d = 0.370 N·m/rad
还有 0.5 N·m/rad 的线缆刚度。只按 m·g·d 拟合时 I_杆、G、κ、k 同比例偏低到约 0.43——
2026-09-27 实测重心下 k ≈ 0.45 的一种解释；给了挂砝码实测的 K 就回到真值。
"""
from __future__ import annotations

import importlib.util
import math
import sys
from pathlib import Path

import pytest

np = pytest.importorskip("numpy")
pytest.importorskip("scipy")

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from sysid import fit, rig_stiffness  # noqa: E402
from sysid.excitation import PROFILE_DOUBLET, Excitation  # noqa: E402
sys.path.pop(0)

_spec = importlib.util.spec_from_file_location(
    "sysid_panel_core_stiffness", ROOT / "tools" / "panel_lib" / "pages" / "sysid" / "_core.py")
core = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(core)

MASS, GRAVITY = 0.7546, 9.81
PIVOT = 0.05
TILT_AXIS = -0.12
I_CG, I_EST = 0.04, 0.051
CABLE = 0.5
K_TRUE = MASS * GRAVITY * PIVOT + CABLE
SCALE = 1.0 - PIVOT / TILT_AXIS          # s(d)，κ = 1 时 G = s(d)
EXCITATION = Excitation(profile=PROFILE_DOUBLET, amplitude_rad_s=0.15, hold_ms=250, repeat=16,
                        ramp_ms=150)


# ---------------------------------------------------------------- 挂砝码试验本身


def test_the_weight_test_formula():
    """m_w·g·x / ((θ前 − θ后)/2)：100 g 挂在 0.1 m，前后各偏 2°。"""
    estimate = rig_stiffness.weight_test_stiffness(100, 0.1, 2.0, -2.0, 0.0)
    expected = 0.1 * rig_stiffness.GRAVITY_M_S2 * 0.1 / math.radians(2.0)
    assert estimate.stiffness_n_m_rad == pytest.approx(expected)
    assert estimate.front_n_m_rad == pytest.approx(expected)
    assert estimate.back_n_m_rad == pytest.approx(expected)
    assert estimate.consistent and estimate.notes == ()
    # 零点不在 0 也一样（前后一起用时零点误差对消）；距离的正负无所谓。
    shifted = rig_stiffness.weight_test_stiffness(100, -0.1, 5.0, 1.0, 3.0)
    assert shifted.stiffness_n_m_rad == pytest.approx(expected)


def test_front_and_back_disagreeing_is_flagged():
    estimate = rig_stiffness.weight_test_stiffness(100, 0.1, 3.0, -1.5, 0.0)
    assert not estimate.consistent and estimate.side_disagreement > 0.15
    assert any("前后单边刚度相差" in note for note in estimate.notes)
    swapped = rig_stiffness.weight_test_stiffness(100, 0.1, 2.0, -2.0, 5.0)
    assert not swapped.consistent and any("不在挂前、挂后两者之间" in n for n in swapped.notes)
    no_level = rig_stiffness.weight_test_stiffness(100, 0.1, 2.0, -2.0)
    assert no_level.consistent and math.isnan(no_level.front_n_m_rad)


def test_the_attitude_reading_is_projected_onto_the_rod():
    """斜杆 ψ = 45°：真刚度 1 N·m/rad，俯仰读数只有绕杆转角的 sin45°，换算后找回 1.0。"""
    moment = 0.05 * rig_stiffness.GRAVITY_M_S2 * 0.1
    theta = math.degrees(moment / 1.0)
    pitch = theta * math.sin(math.radians(45.0))
    estimate = rig_stiffness.weight_test_stiffness(50, 0.1, pitch, -pitch, 0.0, azimuth_deg=45.0)
    assert estimate.stiffness_n_m_rad == pytest.approx(1.0)
    assert estimate.reading == "俯仰" and estimate.projection == pytest.approx(math.sqrt(0.5))
    assert any("已按" in note and "换算" in note for note in estimate.notes)
    roll = rig_stiffness.weight_test_stiffness(50, 0.1, theta, -theta, 0.0, azimuth_deg=0.0)
    assert roll.stiffness_n_m_rad == pytest.approx(1.0) and roll.reading == "横滚"
    # 默认 ψ = 90°（俯仰杆）：读数就是绕杆转角，没有换算说明。
    plain = rig_stiffness.weight_test_stiffness(50, 0.1, theta, -theta, 0.0)
    assert plain.stiffness_n_m_rad == pytest.approx(1.0) and plain.notes == ()


@pytest.mark.parametrize("args, message", [
    ((0, 0.1, 2, -2), "砝码质量要填正数"),
    ((100, 0, 2, -2), "水平距离"),
    ((100, 0.1, 1, 1), "几乎一样"),
    (("x", 0.1, 2, -2), "要填数字"),
])
def test_bad_weight_test_inputs_are_refused_in_chinese(args, message):
    with pytest.raises(ValueError, match=message):
        rig_stiffness.weight_test_stiffness(*args)


def test_implied_pivot_and_extra_stiffness():
    assert rig_stiffness.implied_pivot_m(K_TRUE, MASS, GRAVITY) == pytest.approx(K_TRUE / (MASS * GRAVITY))
    assert rig_stiffness.extra_stiffness(K_TRUE, MASS, PIVOT, GRAVITY) == pytest.approx(CABLE)


# ---------------------------------------------------------------- 拟合找回真值


def torque_record(time_s: np.ndarray) -> np.ndarray:
    ms = np.floor(time_s * 1000.0).astype(int)
    total = EXCITATION.total_ms()
    return np.array([I_EST * EXCITATION.eval(int(m))[1] if 0 <= m < total else 0.0 for m in ms])


@pytest.fixture(scope="module")
def cable_rig():
    """(时间戳, 样本)：κ = 1 的几何固件，回中刚度 = m·g·d + 线缆。"""
    rng = np.random.default_rng(3)
    span = EXCITATION.total_ms() / 1000.0
    grid = np.arange(0.0, span, 0.001)
    torque = torque_record(grid)
    rate = fit.simulate_twd(grid, torque, inertia_rod=I_CG + MASS * PIVOT ** 2, gain=SCALE,
                            stiffness=K_TRUE, dead_time_s=0.037, servo_wn=33.0, servo_zeta=0.44,
                            reaction_couple=0.009, damping=0.01)
    ticks = np.cumsum(rng.uniform(0.0020, 0.0031, int(span / 0.002) + 10))
    stamps, due = [], 0.0
    for tick in ticks:
        if tick >= span - 0.002:
            break
        if tick >= due:
            stamps.append(tick)
            due += 0.004
    stamps = np.array(stamps)
    measured = np.interp(stamps, grid, rate) + 0.004 + 0.02 * rng.standard_normal(stamps.size)
    samples = [{"torque": float(u), "gx": 0.0, "gy": float(r), "angle": 0.0}
               for u, r in zip(np.interp(stamps, grid, torque), measured)]
    return stamps, samples


def fit_cable_rig(cable_rig, **kwargs):
    stamps, samples = cable_rig
    return core.fit_inner_loop(stamps, samples, azimuth_rad=math.pi / 2, mass_kg=MASS,
                               assumed_inertia_kg_m2=I_EST, thrust_point_to_cg_z_m=-0.2,
                               pivot_above_cg_m=PIVOT, roll_pivot_to_cg_z_m=TILT_AXIS,
                               pitch_pivot_to_cg_z_m=TILT_AXIS,
                               firmware_tilt_levers_m=(-TILT_AXIS, -TILT_AXIS), **kwargs)


def test_the_measured_stiffness_recovers_the_truth(cable_rig):
    result = fit_cable_rig(cable_rig, rig_stiffness_n_m_rad=K_TRUE)
    assert result.structure == "twd"
    assert result.inertia_rod_kg_m2 == pytest.approx(I_CG + MASS * PIVOT ** 2, rel=0.08)
    assert result.torque_model_scale == pytest.approx(1.0, rel=0.08)
    assert result.torque_model_geometric_scale == pytest.approx(1.0, rel=1e-6)
    assert result.thrust_servo_model_scale == pytest.approx(1.0, rel=0.08)
    assert result.tuning_inertia_kg_m2 == pytest.approx(I_CG, rel=0.10)
    assert result.natural_hz == pytest.approx(math.sqrt(K_TRUE / (I_CG + MASS * PIVOT ** 2))
                                              / (2 * math.pi), rel=0.05)
    assert result.rig_stiffness_n_m_rad == K_TRUE
    assert result.effective_pivot_m == pytest.approx(K_TRUE / (MASS * GRAVITY))
    assert result.extra_stiffness_n_m_rad == pytest.approx(CABLE)
    assert not any("推力×舵机模型比例" in b for b in result.tuning_blockers)
    assert any("等效杆高 d_eff" in note and "多出来的刚度" in note for note in result.notes)


def test_gravity_alone_makes_k_look_like_the_cable_share(cable_rig):
    """只按 m·g·d：I_杆 与 G 同比例偏低到 m·g·d/K ≈ 0.43，k 跟着低——拦下，理由是模型比例。"""
    result = fit_cable_rig(cable_rig)
    share = MASS * GRAVITY * PIVOT / K_TRUE
    assert result.inertia_rod_kg_m2 == pytest.approx((I_CG + MASS * PIVOT ** 2) * share, rel=0.08)
    assert result.thrust_servo_model_scale == pytest.approx(share, rel=0.08)
    assert any("推力×舵机模型比例" in b for b in result.tuning_blockers)
    assert math.isnan(result.rig_stiffness_n_m_rad)


def test_a_non_positive_stiffness_is_refused(cable_rig):
    with pytest.raises(ValueError, match="台架刚度必须是正的有限值"):
        fit_cable_rig(cable_rig, rig_stiffness_n_m_rad=0.0)


def test_the_legacy_rod_through_the_cg_ignores_the_stiffness(cable_rig):
    stamps, samples = cable_rig
    result = core.fit_inner_loop(stamps, samples, azimuth_rad=math.pi / 2, mass_kg=MASS,
                                 assumed_inertia_kg_m2=I_EST, pivot_above_cg_m=0.005,
                                 rig_stiffness_n_m_rad=K_TRUE)
    assert result.structure == "legacy"
    assert any("刚度不参与" in note for note in result.notes)


# ---------------------------------------------------------------- 页面：输入、换算、交给拟合、记住

from test_sysid_page import (  # noqa: E402,F401  页面夹具
    FakeFit, finished_ff_run, fresh_page, page, settings_file, wait_for,
)


def fill_weight_test(page, weight="100", distance="0.1", front="2", back="-2", level="0"):
    for variable, text in ((page.stiffness_weight_var, weight), (page.stiffness_distance_var, distance),
                           (page.stiffness_front_var, front), (page.stiffness_back_var, back),
                           (page.stiffness_level_var, level)):
        variable.set(text)


def test_the_preparation_page_shows_the_stiffness_as_you_type(page):
    page.psi_var.set("90")                   # 2026-09-27 的俯仰杆
    assert "没填" in page.stiffness_hint_var.get()
    fill_weight_test(page, back="")
    assert "没填全" in page.stiffness_hint_var.get()
    fill_weight_test(page)
    expected = 0.1 * rig_stiffness.GRAVITY_M_S2 * 0.1 / math.radians(2.0)
    assert f"K = {expected:.4f} N·m/rad" in page.stiffness_hint_var.get()
    assert "相差 0%" in page.stiffness_hint_var.get()
    assert page.stiffness_front_label_var.get() == "砝码挂前时的俯仰角 [deg]"
    assert page.stiffness_back_label_var.get() == "砝码挂后时的俯仰角 [deg]"
    fill_weight_test(page, front="3", back="-1.5")
    assert "前后单边刚度相差" in page.stiffness_hint_var.get()


@pytest.mark.parametrize("psi, reading, sides, projection", [
    ("45", "俯仰", ("右前", "左后"), math.sin(math.radians(45.0))),
    ("-45", "俯仰", ("左前", "右后"), math.sin(math.radians(45.0))),
    ("0", "横滚", ("左", "右"), 1.0),
])
def test_the_weight_test_follows_the_rod_azimuth(page, psi, reading, sides, projection):
    """绕杆转 θ 时姿态显示 横滚 θ·cosψ、俯仰 θ·sinψ：斜杆上拿俯仰读数直接当 θ，K 会大
    1/sin45° = 1.41 倍；横滚杆上砝码要挂左右、读横滚。标签与说明跟着 ψ 变，读数自动换算。"""
    page.psi_var.set(psi)
    fill_weight_test(page)
    true_k = 0.1 * rig_stiffness.GRAVITY_M_S2 * 0.1 / math.radians(2.0) * projection
    assert f"K = {true_k:.4f} N·m/rad" in page.stiffness_hint_var.get()
    assert page.rig_stiffness_n_m_rad() == pytest.approx(true_k)
    assert page.stiffness_front_label_var.get() == f"砝码挂{sides[0]}时的{reading}角 [deg]"
    assert page.stiffness_back_label_var.get() == f"砝码挂{sides[1]}时的{reading}角 [deg]"
    assert page.stiffness_level_label_var.get() == f"不挂砝码时的{reading}角 [deg]"
    howto = page.stiffness_howto_var.get()
    assert f"{sides[0]}、{sides[1]}各一次" in howto and f"读面板上的{reading}角" in howto
    assert ("0.71 倍" in howto) == (projection < 0.99)


def test_the_weight_test_reaches_the_fit_and_is_remembered(page, monkeypatch, tmp_path):
    page.psi_var.set("90")
    fill_weight_test(page)
    calls = finished_ff_run(page, monkeypatch, tmp_path)
    expected = 0.1 * rig_stiffness.GRAVITY_M_S2 * 0.1 / math.radians(2.0)
    assert calls[0]["rig_stiffness_n_m_rad"] == pytest.approx(expected)
    # 重新分析按这一轮记录的 ψ（90°）换算，不按「台架」里后来改成的 45°。
    page.psi_var.set("45")
    page.workflow.fit(manual=True)
    assert wait_for(page, lambda: page.workflow.job is None and len(calls) == 2)
    assert calls[1]["rig_stiffness_n_m_rad"] == pytest.approx(expected, rel=1e-6)
    import json
    saved = json.loads(settings_file().read_text(encoding="utf-8"))
    assert saved["stiffness_weight_g"] == 100 and saved["stiffness_front_deg"] == 2
    again = fresh_page(page)
    assert again.stiffness_distance_var.get() == "0.1" and again.stiffness_level_var.get() == "0"


def test_a_half_filled_weight_test_stops_the_analysis_with_a_reason(page, monkeypatch, tmp_path):
    fill_weight_test(page, distance="")
    from panel_lib.pages.sysid import analysis
    calls = []
    monkeypatch.setattr(analysis, "fit_inner_loop", lambda *a, **k: calls.append(k) or FakeFit())
    monkeypatch.setattr(page.workflow, "archive_directory", lambda: tmp_path / "run")
    from test_sysid_page import feed_full_run, start_run
    start_run(page)
    feed_full_run(page, n=100)
    page.handle_line("SYSID end run=3 state=done reason=complete dropped=0")
    assert wait_for(page, lambda: not page.workflow.saving and "没填全" in page.fit_var.get())
    assert calls == []


def test_the_stiffness_line_on_the_result_card():
    sys.path.insert(0, str(ROOT / "tools"))
    try:
        from panel_lib.pages.sysid.results import card_blocks
    finally:
        sys.path.pop(0)
    result = FakeFit()
    plain = card_blocks(result, pivot_m=0.05)["plant"]
    assert "台架刚度" not in plain
    object.__setattr__(result, "rig_stiffness_n_m_rad", K_TRUE)
    object.__setattr__(result, "effective_pivot_m", K_TRUE / (MASS * GRAVITY))
    object.__setattr__(result, "extra_stiffness_n_m_rad", CABLE)
    text = card_blocks(result, pivot_m=0.05)["plant"]
    assert f"K = {K_TRUE:.4f} N·m/rad" in text and "等效杆高 d_eff = 117.5 mm" in text
    assert "+0.5000 N·m/rad" in text

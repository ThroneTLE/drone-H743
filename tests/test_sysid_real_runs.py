"""实录回归：2026-09-27 同一台架的两轮双脉冲（A、B）与一轮扫频（C）。

数据（只读）：`data/identification/attitude/2026-09-27/` 下的
rod_012332_422504f4（A）、rod_012409_92da9975（B）——FF doublet，4 s；
rod_025620_3d68951c（C）——新固件，0.5 s 零力矩预滚 + 0.5→6 Hz 扫频 12 s，幅值 50 mrad/s。
都是俯仰杆（ψ ≈ 90°），杆高 d = 0.1346 m。作者实测俯仰、横滚倾转轴都在飞控板下方 0.13 m
（两轴相交），即倾转轴到质心 −0.0354 m，杆到倾转轴 0.17 m。三轮都是旧固件力矩模型
（力臂 0.145 × 有效系数），`firmware_tilt_levers` 从参数回显算出固件力臂。

钉的结论：
* TWD 结构（刚体单摆 + 二阶舵机 + 反作用力偶 + 纯延迟）、12 Hz/4 阶带内拟合、阻尼 c 放开。
  两轮双脉冲的整定惯量、舵机频率、TWD 零点、1 Hz 等效延迟一致；I_杆 与 κ 此消彼长。
* 几何核对：κ_几何 = 0.0354/0.0825 ≈ 0.43——固件力矩模型把俯仰操纵力矩高估约 2.3 倍，
  实测 κ 与之一致（A+B 联合 k ≈ 1.03），这是真实的辨识结论，不是测量错误。
* 扫频 C：模型无关频响直接量到 TWD 零点（约 2.76 Hz，+180° 相位翻转）；摆的阻尼只有
  扫过摆频才看得出来。但扫频低频段舵机倾角只有 1–3°（双脉冲约 10°），落在舵机回差里：
  等效增益 k ≈ 0.55、等效延迟偏大，且 6 Hz 的扫频终点没越过舵机转折频率，舵机二阶
  贴边界——所以 C 单轮与 A+B+C 联合都被拦下，这是正确的结果。
"""
from __future__ import annotations

import importlib.util
import json
import math
import sys
from pathlib import Path

import pytest

np = pytest.importorskip("numpy")
pytest.importorskip("scipy")

ROOT = Path(__file__).resolve().parents[1]
DAY = ROOT / "data" / "identification" / "attitude" / "2026-09-27"
RUNS = {"A": DAY / "rod_012332_422504f4", "B": DAY / "rod_012409_92da9975",
        "C": DAY / "rod_025620_3d68951c"}
#: 作者实测：两个倾转轴都在飞控板下方 0.13 m；飞控板到质心 −0.094558 m。
TILT_AXIS_TO_CG = -0.13 - (-0.094558)
MISSING_PITCH = "缺倾转轴高度（俯仰），κ 与整定惯量无法换算"
#: 新固件几何力矩模型的参数回显（servoN_axis_z_m 设为 −0.13 后）：合成的，用来测"几何"分支。
GEOMETRIC_ECHO = {"airframe.servo1_axis_z_m": "-0.130000", "airframe.servo2_axis_z_m": "-0.130000",
                  "airframe.cg_z_m": "-0.094558", "airframe.thrust_point_to_cg_z_m": "-0.200942"}

if not all((folder / "samples.csv").is_file() for folder in RUNS.values()):
    pytest.skip("实录不在本机", allow_module_level=True)

sys.path.insert(0, str(ROOT / "tools"))
from sysid import fit, tune  # noqa: E402
sys.path.pop(0)

_spec = importlib.util.spec_from_file_location(
    "sysid_panel_core_real_runs", ROOT / "tools" / "panel_lib" / "pages" / "sysid" / "_core.py")
core = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(core)


def echo(folder: Path) -> dict:
    return json.loads((folder / "conditions.json").read_text(encoding="utf-8"))["parameter_echo"]


def load(folder: Path, pitch_pivot: float | None = TILT_AXIS_TO_CG):
    """按页面的方式还原一轮：时间取相对首样本的秒数，样本是逐行字段字典。"""
    conditions = json.loads((folder / "conditions.json").read_text(encoding="utf-8"))
    raw = np.genfromtxt(folder / "samples.csv", delimiter=",", names=True)
    samples = [{name: float(row[name]) for name in raw.dtype.names} for row in raw]
    times = (raw["t_us"] - raw["t_us"][0]) * 1e-6
    parameters = conditions["parameter_echo"]
    options = dict(
        azimuth_rad=float(conditions["psi_mrad"]) * 1e-3,
        mass_kg=float(conditions["mass_mg"]) * 1e-6,
        assumed_inertia_kg_m2=float(conditions["I_ugm2"]) * 1e-6,
        thrust_point_to_cg_z_m=float(parameters["airframe.thrust_point_to_cg_z_m"]),
        pivot_above_cg_m=float(conditions["axis_off_um"]) * 1e-6,
        roll_pivot_to_cg_z_m=TILT_AXIS_TO_CG,
        pitch_pivot_to_cg_z_m=pitch_pivot,
        firmware_tilt_levers_m=core.firmware_tilt_levers(parameters))
    return times, samples, options


def flight_plant(result) -> tune.FlightPlant:
    return tune.FlightPlant(
        inertia_cg_kg_m2=result.inertia_kg_m2, torque_model_scale=result.torque_model_scale,
        dead_time_s=result.dead_time_s, servo_wn_rad_s=result.servo_wn_rad_s,
        servo_zeta=result.servo_zeta, reaction_couple_s2=result.reaction_couple_s2,
        feedback_notch=tune.DEPLOYED_RPM_NOTCH)   # 与 _core.synthesise_gains 同一对象（含转速陷波）


def relative_difference(a: float, b: float) -> float:
    return abs(a - b) / (0.5 * (abs(a) + abs(b)))


@pytest.fixture(scope="module")
def runs():
    return {name: load(folder) for name, folder in RUNS.items()}


@pytest.fixture(scope="module")
def single(runs):
    return {name: core.fit_inner_loop(times, samples, **options)
            for name, (times, samples, options) in runs.items()}


@pytest.fixture(scope="module")
def joint_ab(runs):
    return core.fit_inner_loop_multi([runs[name][:2] for name in "AB"], **runs["A"][2])


@pytest.fixture(scope="module")
def joint_abc(runs):
    return core.fit_inner_loop_multi([runs[name][:2] for name in "ABC"], **runs["A"][2])


# ---------------------------------------------------------------- 固件力矩模型的两个辅助函数


def test_the_recorded_runs_use_the_legacy_torque_model():
    for folder in RUNS.values():
        parameters = echo(folder)
        assert core.torque_model_signature(parameters) == "legacy"
        roll, pitch = core.firmware_tilt_levers(parameters)
        assert roll == pytest.approx(0.145 * 0.581)
        assert pitch == pytest.approx(0.145 * 0.569)


def test_the_geometric_torque_model_levers_come_from_the_axis_heights():
    assert core.torque_model_signature(GEOMETRIC_ECHO) == "geometric"
    assert core.firmware_tilt_levers(GEOMETRIC_ECHO) == pytest.approx((0.035442, 0.035442))


def test_the_helpers_say_unknown_or_none_when_they_cannot_tell():
    assert core.torque_model_signature({}) == "unknown"
    assert core.firmware_tilt_levers({}) is None
    partial = {key: value for key, value in GEOMETRIC_ECHO.items() if key != "airframe.cg_z_m"}
    assert core.torque_model_signature(partial) == "unknown"
    assert core.firmware_tilt_levers(partial) is None
    legacy = dict(echo(RUNS["A"]))
    assert core.firmware_tilt_levers({**legacy, "airframe.pitch_thrust_lever_arm_m": "0"}) is None
    assert core.firmware_tilt_levers({**legacy, "airframe.thrust_point_to_cg_z_m": "0"}) is None
    assert core.firmware_tilt_levers({**legacy, "airframe.roll_thrust_lever_arm_m": "x"}) is None
    assert core.firmware_tilt_levers({**GEOMETRIC_ECHO, "airframe.servo2_axis_z_m": "-0.094558"}) is None
    flipped = core.firmware_tilt_levers({**legacy, "airframe.thrust_point_to_cg_z_m": "0.2"})
    assert flipped[0] < 0.0 and flipped[1] < 0.0, "推力点在质心上方时旧固件极性取反"


def test_the_geometric_torque_model_predicts_kappa_one(runs):
    times, samples, options = runs["A"]
    result = core.fit_inner_loop(times, samples, **{
        **options, "firmware_tilt_levers_m": core.firmware_tilt_levers(GEOMETRIC_ECHO)})
    assert result.torque_model_geometric_scale == pytest.approx(1.0, rel=0.01)


# ---------------------------------------------------------------- 双脉冲 A、B


def test_the_two_doublet_runs_agree_on_what_tuning_needs(single):
    a, b = single["A"], single["B"]
    for result in (a, b):
        assert result.structure == "twd"
        assert result.tuning_damping_n_m_s == 0.0
        assert result.fit_percent >= fit.TWD_FIT_PERCENT_MIN
    assert relative_difference(a.tuning_inertia_kg_m2, b.tuning_inertia_kg_m2) < 0.10
    assert relative_difference(a.servo_wn_rad_s, b.servo_wn_rad_s) < 0.10
    assert relative_difference(a.rig_zero_hz, b.rig_zero_hz) < 0.10
    assert relative_difference(a.delay_s, b.delay_s) < 0.10


def test_the_rod_inertia_and_torque_scale_trade_off_between_runs(single):
    """单轮里 I_杆 与 κ 此消彼长，所以 σ 门槛作用在 I_cg/κ 上，k 的门槛也扣掉 G 的不确定度。"""
    a, b = single["A"], single["B"]
    assert relative_difference(a.inertia_rod_kg_m2, b.inertia_rod_kg_m2) > 0.10
    assert relative_difference(a.torque_model_scale, b.torque_model_scale) > 0.10
    assert not any("推力×舵机模型比例" in blocker for blocker in a.tuning_blockers)


def test_the_doublet_joint_fit_is_consistent_with_the_geometry(joint_ab, single):
    assert joint_ab.structure == "twd"
    assert min(joint_ab.fit_percent_runs) >= fit.TWD_FIT_PERCENT_MIN
    assert joint_ab.fit_percent == min(joint_ab.fit_percent_runs)
    assert joint_ab.samples == single["A"].samples + single["B"].samples
    assert joint_ab.tuning_blockers == ()
    assert joint_ab.tuning_inertia_uncertainty_pct < fit.TUNING_INERTIA_UNCERTAINTY_MAX_PCT
    assert joint_ab.torque_model_geometric_scale == pytest.approx(0.0354 / 0.0825, rel=0.01)
    assert 0.85 < joint_ab.thrust_servo_model_scale < 1.2
    assert any("固件力矩模型高估俯仰操纵力矩约 2.3 倍" in w for w in joint_ab.warnings)
    # 量级回归（实测几何）：整定惯量、纯延迟、等效延迟、舵机、TWD 零点。
    assert 0.09 < joint_ab.tuning_inertia_kg_m2 < 0.12
    assert 0.030 < joint_ab.dead_time_s < 0.045
    assert 0.055 < joint_ab.delay_s < 0.075
    assert 28.0 < joint_ab.servo_wn_rad_s < 42.0
    assert 2.6 < joint_ab.rig_zero_hz < 3.7


@pytest.mark.parametrize("which", ["A", "B", "AB"])
def test_the_synthesised_gains_are_safe_on_the_identified_plant(which, single, joint_ab, runs):
    result = joint_ab if which == "AB" else single[which]
    rate, attitude, commands = core.synthesise_gains(result, azimuth_rad=runs["A"][2]["azimuth_rad"])
    assert 0.15 <= rate.kp <= 0.40
    assert rate.feasible
    assert rate.gain_margin_db >= 6.0 - 1e-6
    assert rate.phase_margin_deg >= 45.0
    assert 0.0 < rate.crossover_hz <= tune.RATE_BANDWIDTH_MAX_HZ
    assert attitude.kp > 0.0
    assert commands and all(line.startswith("SYSID PARAM coax.") for line in commands)
    assert all("_pitch_" in line for line in commands), "俯仰杆只给俯仰参数"


@pytest.mark.parametrize("which", ["A", "B"])
def test_gains_from_the_rigid_reading_would_be_unsafe(which, single):
    result = single[which]
    rigid = tune.synthesise_rate(result.rigid_tuning_inertia_kg_m2, 0.0, result.rigid_delay_s)
    margins = tune.loop_margins(rigid.kp, rigid.ki, flight_plant(result))
    assert margins.gain_margin_db < 3.0
    assert any("刚体积分读数：被 6/10 Hz 谱线证伪" in note for note in result.notes)


def test_an_unmeasured_pitch_pivot_blocks_tuning():
    times, samples, options = load(RUNS["A"], pitch_pivot=None)
    result = core.fit_inner_loop(times, samples, **options)
    assert MISSING_PITCH in result.tuning_blockers


# ---------------------------------------------------------------- 扫频 C


def test_the_chirp_measures_the_twd_notch_model_free(single):
    c = single["C"]
    assert 2.6 < c.measured_zero_hz < 2.95
    assert 0.50 < c.measured_pendulum_hz < 0.65
    assert any("模型无关频响量到的TWD 零点" in w for w in c.warnings)
    for name in "AB":
        assert math.isnan(single[name].measured_zero_hz), "4 s 双脉冲读不出模型无关零点"


def test_the_chirp_identifies_pendulum_damping(single):
    """扫过摆频才看得见摆的阻尼（阻尼比约 0.14）；整定阻尼仍按 0。"""
    c = single["C"]
    zeta = c.damping_n_m_s / (2.0 * math.sqrt(0.7546 * 9.81 * 0.134558 * c.inertia_rod_kg_m2))
    assert 0.08 < zeta < 0.25
    assert c.tuning_damping_n_m_s == 0.0
    assert c.fit_percent >= fit.TWD_FIT_PERCENT_MIN


def test_the_small_amplitude_chirp_is_blocked_for_the_right_reasons(single, joint_ab):
    """扫频低频段舵机倾角只有 1–3°：等效增益低（k ≈ 0.55）、等效延迟大；
    6 Hz 的扫频终点没越过舵机转折频率，舵机二阶贴边界。"""
    c = single["C"]
    assert any("舵机二阶" in b for b in c.tuning_blockers)
    assert any("推力×舵机模型比例" in b for b in c.tuning_blockers)
    assert not any("带内拟合优度" in b for b in c.tuning_blockers)
    assert c.thrust_servo_model_scale < 0.7 < joint_ab.thrust_servo_model_scale
    assert c.delay_s > joint_ab.delay_s


def test_mixing_the_chirp_with_the_doublets_is_blocked(joint_abc):
    assert len(joint_abc.fit_percent_runs) == 3
    assert joint_abc.fit_percent == min(joint_abc.fit_percent_runs)
    assert any("带内拟合优度" in b for b in joint_abc.tuning_blockers)
    assert any("力矩 rms 相差" in w for w in joint_abc.warnings)
    assert joint_abc.measured_zero_hz == pytest.approx(joint_abc.rig_zero_hz, rel=0.15)


def test_the_real_runs_pass_the_page_time_axis_check(runs):
    for times, _samples, _options in runs.values():
        steps = np.diff(times)
        assert np.all(steps > 0.0)
        assert np.max(steps) <= 2.5 * np.median(steps)
        assert math.isclose(1.0 / float(np.median(steps)), 250.0, rel_tol=0.2)


# ---------------------------------------------------------------- 台架刚度（挂砝码实测）代替 m·g·d


def test_a_stiffer_rig_rescales_rod_inertia_kappa_and_k_together(runs, joint_ab):
    """同一份数据只认 K/I_杆 与 G/I_杆：K 换成 1.3·m·g·d，I_杆、G、κ、k 一起 ×1.3，
    整定惯量 (I_杆 − m·d²)/κ 跟着变；等效杆高与多出来的刚度照实报。"""
    options = dict(runs["A"][2])
    weight = options["mass_kg"] * 9.81 * options["pivot_above_cg_m"]
    stiff = core.fit_inner_loop_multi([runs[name][:2] for name in "AB"],
                                      rig_stiffness_n_m_rad=1.3 * weight, **options)
    assert stiff.inertia_rod_kg_m2 == pytest.approx(1.3 * joint_ab.inertia_rod_kg_m2, rel=0.03)
    assert stiff.torque_model_scale == pytest.approx(1.3 * joint_ab.torque_model_scale, rel=0.03)
    assert stiff.thrust_servo_model_scale == pytest.approx(1.3 * joint_ab.thrust_servo_model_scale,
                                                           rel=0.03)
    assert stiff.natural_hz == pytest.approx(joint_ab.natural_hz, rel=0.02)
    mass_d2 = options["mass_kg"] * options["pivot_above_cg_m"] ** 2
    expected_tuning = ((1.3 * joint_ab.inertia_rod_kg_m2 - mass_d2)
                       / (1.3 * joint_ab.torque_model_scale))
    assert stiff.tuning_inertia_kg_m2 == pytest.approx(expected_tuning, rel=0.05)
    assert stiff.effective_pivot_m == pytest.approx(1.3 * options["pivot_above_cg_m"])
    assert stiff.extra_stiffness_n_m_rad == pytest.approx(0.3 * weight)
    assert any("等效杆高 d_eff" in w for w in stiff.warnings)


# ---------------------------------------------------------------- 力矩单位：旧模型录的增益写进几何固件


@pytest.mark.parametrize("cg_z_m", [-0.094558, -0.01])
def test_converted_gains_keep_the_identified_margins_on_the_new_firmware(joint_ab, runs, cg_z_m):
    """A+B（旧力矩模型录制）合成的增益，按 L_now/L_src 换到几何固件（今晚重心 −0.094558、
    修正后 −0.01）：新固件单位里的飞行对象 κ、ρ 都 ×L_src/L_now，于是回路增益不变、裕度逐位
    相同；不换算的话今晚会硬 2.3 倍，增益裕度掉光。"""
    sys.path.insert(0, str(ROOT / "tools"))
    try:
        from panel_lib.pages.sysid import lever_units
    finally:
        sys.path.pop(0)
    rate, _attitude, commands = core.synthesise_gains(joint_ab, azimuth_rad=runs["A"][2]["azimuth_rad"])
    board = {"airframe.cg_z_m": f"{cg_z_m:.6f}", "airframe.servo1_axis_z_m": "-0.130000",
             "airframe.servo2_axis_z_m": "-0.130000"}
    source = lever_units.torque_record(echo(RUNS["A"]))
    pairs = [tuple(line.split()[2:4]) for line in commands]
    converted = dict(lever_units.convert(pairs, source, lever_units.torque_record(board))[0])
    ratio = (cg_z_m + 0.13) / (0.145 * 0.569)                       # L_now / L_src（俯仰）
    kp, ki = float(converted["coax.rate_pitch_kp"]), float(converted["coax.rate_pitch_ki"])
    assert kp == pytest.approx(rate.kp * ratio, rel=1e-4)
    old_plant = flight_plant(joint_ab)
    from dataclasses import replace
    new_plant = replace(old_plant, torque_model_scale=old_plant.torque_model_scale / ratio,
                        reaction_couple_s2=old_plant.reaction_couple_s2 / ratio)
    before = tune.loop_margins(rate.kp, rate.ki, old_plant)
    after = tune.loop_margins(kp, ki, new_plant)
    assert after.gain_margin_db == pytest.approx(before.gain_margin_db, abs=0.05)
    assert after.phase_margin_deg == pytest.approx(before.phase_margin_deg, abs=0.5)
    unconverted = tune.loop_margins(rate.kp, rate.ki, new_plant)
    if ratio < 1.0:
        assert unconverted.gain_margin_db < before.gain_margin_db - 6.0, "不换算就硬 1/ratio 倍"

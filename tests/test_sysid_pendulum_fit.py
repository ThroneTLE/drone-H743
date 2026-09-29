"""杆不过质心的单摆台架：量出杆高 d，用重力标定惯量，辨出力矩模型比例 κ。

实测杆高时默认结构是 TWD：刚体单摆 + 二阶舵机 + 舵机甩动的反作用力偶 ρ·u'' + 纯延迟，
在 12 Hz（4 阶零相位）以下、起步 0.5 s 之后拟合，c 固定 0；判据是带内拟合优度 ≥ 72%、
整定惯量 I_cg/κ 的不确定度 ≤ 8%，以及在飞行 TWD 对象上能否同时满足 6 dB / 45° 裕度。
4 Hz 刚体积分读数只作诊断基线。

台架：机体绕一根水平光杆转，杆在质心上方 d；舵机倾转轴在质心下方 h_roll / h_pitch，
桨盘中点在 h = −0.2009 m（只用于阻尼换算）。固件记录的 τ_rec 是分配器按模型算的绕质心
力矩，真实力矩 = κ·τ_rec，κ 未知；绕杆力矩 = κ·s(d)·τ_rec，
s(d) = cos²ψ·(h_roll − d)/h_roll + sin²ψ·(h_pitch − d)/h_pitch；重力给 −m·g·d·sin θ。
回归只给得出 G/I_杆、c/I_杆、m·g·d/I_杆 三个量，d 与 κ 混在一起，所以 d 必须量。
d 已知后重力是唯一已知的力矩，标定 I_杆；整定用 I_cg/κ（固件力矩单位）。

这里的真值数据由本文件自己合成，只用于单元测试——验证"拟合器能把已知参数找回来"，
不替代实机数据。合成尽量贴近固件：激励取 `sysid.excitation` 与固件逐样本一致的
doublet，前馈力矩 τ_rec = I_est·α_ff；控制拍间隔 2.0–3.1 ms 抖动，力矩在拍上保持；
记录按 250 Hz 在控制拍上抽样；数据窗就是激励总时长，没有静默尾段。
"""
from __future__ import annotations

import importlib.util
import json
import math
import sys
from dataclasses import asdict
from pathlib import Path

import pytest

np = pytest.importorskip("numpy")
pytest.importorskip("scipy")

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from sysid import fit, tune  # noqa: E402
from sysid.excitation import PROFILE_DOUBLET, Excitation  # noqa: E402
sys.path.pop(0)

# 直接按文件加载桥模块：不经过 Tk 页面包的 __init__，界面改动不会牵连这里。
_spec = importlib.util.spec_from_file_location(
    "sysid_panel_core_under_test", ROOT / "tools" / "panel_lib" / "pages" / "sysid" / "_core.py")
core = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(core)

MASS = 0.7546
GRAVITY = 9.81
H = -0.2009
#: 倾转轴到质心（与界面测试同一组数：尺量到飞控板 + 飞控到质心）。
ROLL_PIVOT = -0.165
PITCH_PIVOT = -0.145
AZIMUTH = math.pi / 4.0
#: 固件前馈用的假定惯量。
I_EST = 0.051
EXCITATION = Excitation(profile=PROFILE_DOUBLET, amplitude_rad_s=0.15, hold_ms=250,
                        repeat=8, ramp_ms=150)
TOTAL_MS = EXCITATION.total_ms()
#: 约 14 Hz 台架次要模态的注入幅值 [rad/s]（量级取自评审：低于 5 Hz 的贡献 < 0.013 rad/s）。
RIG_MODE_AMPLITUDE = 0.05
_ALPHA_FF = np.array([EXCITATION.eval(ms)[1] for ms in range(TOTAL_MS)])

#: 真值取评审给的量级（两轮实录的 TWD 拟合）：d 为实测杆高，舵机约 5.3 Hz，
#: 反作用力偶让杆上零点落在约 2.8 Hz。
TRUTH = dict(pivot=0.1346, inertia_cg=0.04, damping=0.0, delay_s=0.037,
             servo_wn=33.0, servo_zeta=0.44, rho=0.007)


def feedforward_torque(time_s: float) -> float:
    ms = int(math.floor(time_s * 1000.0))
    if ms < 0 or ms >= TOTAL_MS:
        return 0.0
    return I_EST * float(_ALPHA_FF[ms])


def true_scale(pivot: float, azimuth: float, roll_pivot: float | None,
               pitch_pivot: float | None) -> float:
    """真值几何：同一横向推力绕杆与绕质心的力矩之比，参考点是倾转轴。"""
    if roll_pivot is None and pitch_pivot is None:
        return 1.0
    total = 0.0
    for weight, height in ((math.cos(azimuth) ** 2, roll_pivot),
                           (math.sin(azimuth) ** 2, pitch_pivot)):
        if height is not None:
            total += weight * (height - pivot) / height
    return total


def firmware_record(*, kappa: float = 1.0, pivot: float = TRUTH["pivot"],
                    inertia_cg: float = TRUTH["inertia_cg"], damping: float = TRUTH["damping"],
                    delay_s: float = TRUTH["delay_s"],
                    servo_wn: float | None = TRUTH["servo_wn"],
                    servo_zeta: float = TRUTH["servo_zeta"], rho: float = TRUTH["rho"],
                    azimuth: float = AZIMUTH, roll_pivot: float | None = ROLL_PIVOT,
                    pitch_pivot: float | None = PITCH_PIVOT, noise: float = 0.03,
                    bias: float = 0.004, seed: int = 1, step: float = 0.0005,
                    torque_of=None, span_s: float | None = None):
    """真值（RK4 细步积分，sin θ 非线性）::

        舵机 u'' = ωs²(τ_拍(t − T) − u) − 2ζs·ωs·u'
        机体 I_杆·θ̈ = κ·s(d)·u + ρ·u'' − c·θ̇ − m·g·d·sin θ

    `servo_wn=None` 时没有舵机、没有反作用（u = τ_拍(t − T)），即刚体积分 + 延迟。
    `torque_of`/`span_s` 换激励（默认固件 doublet 前馈，数据窗 = 激励总时长）。
    """
    torque_of = feedforward_torque if torque_of is None else torque_of
    rng = np.random.default_rng(seed)
    span = TOTAL_MS / 1000.0 if span_s is None else span_s
    ticks = np.concatenate([[0.0], np.cumsum(rng.uniform(0.0020, 0.0031,
                                                         int(span / 0.002) + 10))])
    tick_torque = np.array([torque_of(x) for x in ticks])
    gain = kappa * true_scale(pivot, azimuth, roll_pivot, pitch_pivot)
    inertia_rod = inertia_cg + MASS * pivot * pivot

    def applied(time_s: float) -> float:
        index = int(np.searchsorted(ticks, time_s - delay_s, side="right")) - 1
        return float(tick_torque[index]) if index >= 0 else 0.0

    def derivative(time_s, state):
        angle, rate, servo, servo_rate = state
        command = applied(time_s)
        if servo_wn is None:
            servo, servo_accel = command, 0.0
        else:
            servo_accel = (servo_wn ** 2 * (command - servo)
                           - 2.0 * servo_zeta * servo_wn * servo_rate)
        accel = (gain * servo + rho * servo_accel - damping * rate
                 - MASS * GRAVITY * pivot * math.sin(angle)) / inertia_rod
        return np.array([rate, accel, servo_rate, servo_accel])

    count = int(round(span / step)) + 1
    times = np.arange(count) * step
    rates = np.zeros(count)
    state = np.zeros(4)
    for k in range(count):
        rates[k] = state[1]
        now = times[k]
        k1 = derivative(now, state)
        k2 = derivative(now + step / 2, state + step / 2 * k1)
        k3 = derivative(now + step / 2, state + step / 2 * k2)
        k4 = derivative(now + step, state + step * k3)
        state = state + step / 6 * (k1 + 2 * k2 + 2 * k3 + k4)

    # 250 Hz 记录：每 4 ms 到期后的第一个控制拍记一条。
    chosen, due = [], 0.0
    for index, tick in enumerate(ticks):
        if tick >= span:
            break
        if tick >= due - 1e-12:
            chosen.append(index)
            due += 0.004
    chosen = np.array(chosen)
    stamps = ticks[chosen]
    true_rate = np.interp(stamps, times, rates)
    # 各轴独立噪声 σ；投影到杆轴上仍是 σ。
    gx = true_rate * math.cos(azimuth) + bias + noise * rng.standard_normal(stamps.size)
    gy = true_rate * math.sin(azimuth) + bias + noise * rng.standard_normal(stamps.size)
    # 记录里的融合姿态故意弄脏：摆动中 IMU 的切向/向心加速度会把它带偏。
    fused = 0.05 + 0.3 * np.gradient(true_rate, stamps)
    samples = [{"torque": float(tick_torque[i]), "gx": float(a), "gy": float(b),
                "gz": 0.0, "angle": float(f), "thrust": 7.4}
               for i, a, b, f in zip(chosen, gx, gy, fused)]
    return stamps, samples, true_rate


#: 宽带扫频激励：0.5 s 零力矩预滚，然后 0.3→8 Hz 线性扫频 12 s，力矩幅值恒定。
CHIRP = dict(pre_roll_s=0.5, f0_hz=0.3, f1_hz=8.0, duration_s=12.0, amplitude_n_m=0.03)


#: 扫频真值里摆的绕杆阻尼（摆的阻尼比约 0.15，与 2026-09-27 扫频实录同量级）。
CHIRP_DAMPING = 0.07


def chirp_torque(time_s: float) -> float:
    local = time_s - CHIRP["pre_roll_s"]
    if local < 0.0 or local >= CHIRP["duration_s"]:
        return 0.0
    sweep = (CHIRP["f1_hz"] - CHIRP["f0_hz"]) / CHIRP["duration_s"]
    phase = 2 * math.pi * (CHIRP["f0_hz"] * local + 0.5 * sweep * local * local)
    return CHIRP["amplitude_n_m"] * math.sin(phase)


def fit_record(stamps, samples, **kwargs):
    options = dict(azimuth_rad=AZIMUTH, mass_kg=MASS, assumed_inertia_kg_m2=I_EST,
                   thrust_point_to_cg_z_m=H, pivot_above_cg_m=TRUTH["pivot"],
                   roll_pivot_to_cg_z_m=ROLL_PIVOT, pitch_pivot_to_cg_z_m=PITCH_PIVOT)
    options.update(kwargs)
    return core.fit_inner_loop(stamps, samples, **options)


@pytest.fixture(scope="module")
def nominal():
    stamps, samples, true_rate = firmware_record(seed=1)
    return fit_record(stamps, samples), stamps, samples, true_rate


# ---------------------------------------------------------------- 合成数据本身


def test_the_synthetic_record_looks_like_the_firmware(nominal):
    _result, stamps, samples, true_rate = nominal
    steps = np.diff(stamps)
    assert 0.001 < steps.min() and steps.max() < 0.0072
    assert stamps[-1] < TOTAL_MS / 1000.0, "数据窗就是激励总时长，没有静默尾段"
    assert samples[0]["torque"] != 0.0, "doublet 从 t=0 就在斜坡上：记录开始前的力矩要按 0 补"
    assert 0.1 < float(np.max(np.abs(true_rate))) < 0.5


# ---------------------------------------------------------------- 实测杆高：重力标定


@pytest.mark.parametrize("kappa", [0.8, 1.0, 1.2])
@pytest.mark.parametrize("seed, noise", [(1, 0.03), (2, 0.04)])
def test_the_twd_parameters_are_recovered(kappa, seed, noise):
    stamps, samples, _rate = firmware_record(kappa=kappa, seed=seed, noise=noise)
    result = fit_record(stamps, samples)
    assert result.structure == "twd"
    assert result.torque_model_scale == pytest.approx(kappa, rel=0.10)
    assert result.inertia_kg_m2 == pytest.approx(TRUTH["inertia_cg"], rel=0.15)
    assert result.tuning_inertia_kg_m2 == pytest.approx(TRUTH["inertia_cg"] / kappa, rel=0.12)
    # 真值纯延迟 37 ms，外加力矩在控制拍上保持的约半拍。
    assert result.dead_time_s == pytest.approx(TRUTH["delay_s"], abs=0.004)
    assert result.servo_wn_rad_s == pytest.approx(TRUTH["servo_wn"], rel=0.10)
    assert result.servo_zeta == pytest.approx(TRUTH["servo_zeta"], rel=0.15)
    assert result.reaction_couple_s2 == pytest.approx(TRUTH["rho"], rel=0.20)
    true_equivalent = fit.equivalent_delay_s(TRUTH["delay_s"], TRUTH["servo_wn"],
                                             TRUTH["servo_zeta"])
    assert result.delay_s == result.equivalent_delay_1hz_s
    assert result.delay_s == pytest.approx(true_equivalent, abs=0.005)
    assert result.fit_percent >= fit.TWD_FIT_PERCENT_MIN, "带内拟合优度"
    assert result.tuning_blockers == ()
    assert 0.0 < result.tuning_inertia_uncertainty_pct < 8.0
    # 双脉冲激励不到摆的共振：拟合的绕杆阻尼落在 0 附近（真值 0），整定阻尼恒为 0。
    assert 0.0 <= result.damping_n_m_s < 0.03 and result.tuning_damping_n_m_s == 0.0
    assert result.gain_margin_db >= 6.0 - 1e-6 and result.phase_margin_deg >= 45.0


def test_the_measured_pivot_is_an_input_not_a_fit(nominal):
    result, _stamps, _samples, _rate = nominal
    d = TRUTH["pivot"]
    assert result.pivot_above_cg_m == d
    assert result.eccentricity_m == d
    assert result.torque_scale == pytest.approx(true_scale(d, AZIMUTH, ROLL_PIVOT, PITCH_PIVOT))
    assert result.inertia_kg_m2 == pytest.approx(result.inertia_rod_kg_m2 - MASS * d * d)
    assert result.tuning_inertia_kg_m2 == pytest.approx(
        result.inertia_kg_m2 / result.torque_model_scale)
    truth_rod = TRUTH["inertia_cg"] + MASS * d * d
    truth_hz = math.sqrt(MASS * GRAVITY * d / truth_rod) / (2 * math.pi)
    assert result.natural_hz == pytest.approx(truth_hz, rel=0.05)
    assert result.tuning_blockers == ()
    assert result.fit_percent_runs == (result.fit_percent,)
    assert any("按 0（保守）" in w for w in result.warnings)
    assert any("15 Hz" in w for w in result.warnings)
    gain = result.torque_model_scale * result.torque_scale
    rho = result.reaction_couple_s2
    assert result.rig_zero_hz == pytest.approx(math.sqrt(gain / rho) / (2 * math.pi))
    assert result.flight_zero_hz == pytest.approx(
        math.sqrt(result.torque_model_scale / rho) / (2 * math.pi))


def test_the_rigid_reading_is_kept_only_as_a_baseline(nominal):
    """刚体积分读数照算、照报，但被 6/10 Hz 谱线证伪，不进整定。"""
    result, stamps, samples, _rate = nominal
    assert any("刚体积分读数：被 6/10 Hz 谱线证伪，不用于整定" in w for w in result.warnings)
    rigid = fit_record(stamps, samples, structure="rigid")
    assert rigid.structure == "rigid"
    assert "刚体积分结构只作对照，不用于整定" in rigid.tuning_blockers
    assert result.rigid_tuning_inertia_kg_m2 == pytest.approx(rigid.tuning_inertia_kg_m2)
    assert result.rigid_delay_s == pytest.approx(rigid.delay_s)
    assert result.rigid_fit_percent == pytest.approx(rigid.fit_percent)
    with pytest.raises(ValueError):
        fit_record(stamps, samples, structure="blackbox")


def test_pinning_the_pendulum_frequency_fixes_the_rod_inertia(nominal):
    result, stamps, samples, _rate = nominal
    truth_rod = TRUTH["inertia_cg"] + MASS * TRUTH["pivot"] ** 2
    frequency = math.sqrt(MASS * GRAVITY * TRUTH["pivot"] / truth_rod) / (2 * math.pi)
    pinned = fit_record(stamps, samples, pendulum_hz=frequency)
    assert pinned.inertia_rod_kg_m2 == pytest.approx(truth_rod, rel=1e-9)
    assert pinned.inertia_uncertainty_pct == 0.0
    assert pinned.torque_model_scale == pytest.approx(1.0, rel=0.10)
    assert pinned.tuning_inertia_kg_m2 == pytest.approx(result.tuning_inertia_kg_m2, rel=0.10)
    assert any("钉死" in w for w in pinned.warnings)
    with pytest.raises(ValueError):
        fit_record(stamps, samples, pendulum_hz=-1.0)


def test_a_one_centimetre_pivot_error_moves_the_tuning_inertia_only_a_little(nominal):
    """d 是量出来的，量错 1 cm 时整定惯量的变化要在可接受范围内。"""
    result, stamps, samples, _rate = nominal
    for wrong in (TRUTH["pivot"] - 0.01, TRUTH["pivot"] + 0.01):
        shifted = fit_record(stamps, samples, pivot_above_cg_m=wrong)
        change = shifted.tuning_inertia_kg_m2 / result.tuning_inertia_kg_m2 - 1.0
        assert abs(change) < 0.08
        assert shifted.tuning_blockers == ()


def test_the_old_keyword_is_an_alias_for_the_measured_pivot(nominal):
    result, stamps, samples, _rate = nominal
    again = core.fit_inner_loop(stamps, samples, azimuth_rad=AZIMUTH, mass_kg=MASS,
                                assumed_inertia_kg_m2=I_EST, thrust_point_to_cg_z_m=H,
                                pivot_above_cg_guess_m=TRUTH["pivot"],
                                roll_pivot_to_cg_z_m=ROLL_PIVOT,
                                pitch_pivot_to_cg_z_m=PITCH_PIVOT)
    assert again == result


#: 固件力臂与真实绕质心力臂一致（新固件的几何力矩模型）：κ_几何 = 1。
EXACT_LEVERS = (-ROLL_PIVOT, -PITCH_PIVOT)


def test_a_torque_model_consistent_with_geometry_passes():
    """κ 与"真实力臂/固件力臂"一致时 k = κ/κ_几何 ≈ 1，不拦。"""
    stamps, samples, _rate = firmware_record(kappa=1.0, seed=3)
    result = fit_record(stamps, samples, firmware_tilt_levers_m=EXACT_LEVERS)
    assert result.torque_model_geometric_scale == pytest.approx(1.0)
    assert result.thrust_servo_model_scale == pytest.approx(result.torque_model_scale)
    assert result.thrust_servo_model_scale == pytest.approx(1.0, rel=0.10)
    assert result.tuning_blockers == ()


def test_a_thrust_servo_model_far_from_geometry_blocks_tuning():
    """κ 只有几何预测的 0.4 倍：推力查补表、舵机标定或几何量错了，必须拦。"""
    stamps, samples, _rate = firmware_record(kappa=0.4, seed=3)
    result = fit_record(stamps, samples, firmware_tilt_levers_m=EXACT_LEVERS)
    assert result.torque_model_scale == pytest.approx(0.4, rel=0.10)
    assert result.thrust_servo_model_scale == pytest.approx(0.4, rel=0.10)
    assert any("推力×舵机模型比例" in b for b in result.tuning_blockers)


def test_a_firmware_model_that_overstates_authority_is_only_a_note():
    """固件力臂是真实的 2 倍：记录力矩被高估 2 倍，κ ≈ 0.5 是真的、不是测量错误。"""
    doubled = tuple(2.0 * lever for lever in EXACT_LEVERS)
    stamps, samples, _rate = firmware_record(kappa=0.5, seed=3)
    result = fit_record(stamps, samples, firmware_tilt_levers_m=doubled)
    assert result.torque_model_geometric_scale == pytest.approx(0.5)
    assert result.thrust_servo_model_scale == pytest.approx(1.0, rel=0.10)
    assert result.tuning_blockers == ()
    assert any("固件力矩模型高估" in w and "2.0 倍" in w and "二者一致" in w
               for w in result.warnings)


def test_a_reversed_firmware_polarity_blocks_tuning():
    stamps, samples, _rate = firmware_record(kappa=1.0, seed=3)
    reversed_levers = tuple(-lever for lever in EXACT_LEVERS)
    result = fit_record(stamps, samples, firmware_tilt_levers_m=reversed_levers)
    assert result.torque_model_geometric_scale < 0.0
    assert any("极性" in b for b in result.tuning_blockers)


def test_without_firmware_levers_kappa_only_gets_a_wide_sanity_range():
    stamps, samples, _rate = firmware_record(kappa=1.0, seed=3)
    result = fit_record(stamps, samples)
    assert math.isnan(result.torque_model_geometric_scale)
    assert any("宽范围" in w for w in result.warnings)
    assert result.tuning_blockers == ()
    stamps, samples, _rate = firmware_record(kappa=3.6, seed=3)
    result = fit_record(stamps, samples)
    assert any("不在 0.2–3" in b for b in result.tuning_blockers)


def test_the_geometric_gain_is_rod_to_axis_over_the_firmware_lever():
    d = TRUTH["pivot"]
    gain = fit.firmware_geometric_gain(d, math.pi / 2, None, PITCH_PIVOT, (1.0, 0.0825))
    assert gain == pytest.approx((d - PITCH_PIVOT) / 0.0825)
    mixed = fit.firmware_geometric_gain(d, AZIMUTH, ROLL_PIVOT, PITCH_PIVOT, (0.08, 0.09))
    assert mixed == pytest.approx(0.5 * (d - ROLL_PIVOT) / 0.08 + 0.5 * (d - PITCH_PIVOT) / 0.09)
    assert fit.firmware_geometric_gain(d, AZIMUTH, ROLL_PIVOT, PITCH_PIVOT, None) is None
    assert fit.firmware_geometric_gain(d, AZIMUTH, None, PITCH_PIVOT, (0.08, 0.09)) is None
    assert fit.firmware_geometric_gain(d, AZIMUTH, ROLL_PIVOT, PITCH_PIVOT, (0.0, 0.09)) is None


def test_the_disk_centre_is_not_needed_when_the_pivots_are_known(nominal):
    """桨盘中点只作缺倾转轴高度时的兜底：倾转轴高度齐全时它不影响任何结果。"""
    result, stamps, samples, _rate = nominal
    without = fit_record(stamps, samples, thrust_point_to_cg_z_m=None)
    assert without.torque_scale == result.torque_scale
    assert without.torque_model_scale == pytest.approx(result.torque_model_scale)
    assert without.tuning_inertia_kg_m2 == pytest.approx(result.tuning_inertia_kg_m2)
    assert without.tuning_blockers == ()


# ---------------------------------------------------------------- 力臂参考点：倾转轴


def test_the_lever_is_the_tilt_pivot_not_the_disk_centre():
    """倾转轴与桨盘中点不同高时，κ 必须按倾转轴换算；按桨盘中点这里会偏约 12%。"""
    stamps, samples, _rate = firmware_record(kappa=1.0, seed=5)
    right = fit_record(stamps, samples)
    assert right.torque_model_scale == pytest.approx(1.0, rel=0.10)
    assert right.tuning_blockers == ()
    disk_only = fit_record(stamps, samples, roll_pivot_to_cg_z_m=None,
                           pitch_pivot_to_cg_z_m=None)
    assert disk_only.torque_scale == pytest.approx((H - TRUTH["pivot"]) / H)
    assert disk_only.torque_model_scale == pytest.approx(
        right.torque_model_scale * right.torque_scale / disk_only.torque_scale, rel=1e-3)
    assert "缺倾转轴高度（横滚、俯仰），κ 与整定惯量无法换算" in disk_only.tuning_blockers
    assert any("不可信" in w for w in disk_only.warnings)


def test_a_pitch_rod_needs_only_the_pitch_pivot_height():
    pitch_rod = math.pi / 2
    stamps, samples, _rate = firmware_record(kappa=1.2, azimuth=pitch_rod, roll_pivot=None,
                                             pitch_pivot=PITCH_PIVOT, seed=6)
    enough = fit_record(stamps, samples, azimuth_rad=pitch_rod, roll_pivot_to_cg_z_m=None)
    assert enough.torque_scale == pytest.approx((PITCH_PIVOT - TRUTH["pivot"]) / PITCH_PIVOT)
    assert enough.torque_model_scale == pytest.approx(1.2, rel=0.10)
    assert enough.tuning_blockers == ()
    missing = fit_record(stamps, samples, azimuth_rad=pitch_rod, pitch_pivot_to_cg_z_m=None)
    assert "缺倾转轴高度（俯仰），κ 与整定惯量无法换算" in missing.tuning_blockers
    for value in (0.0, float("nan")):
        again = fit_record(stamps, samples, azimuth_rad=pitch_rod, pitch_pivot_to_cg_z_m=value)
        assert any("缺倾转轴高度（俯仰）" in b for b in again.tuning_blockers)


def test_the_45_degree_rod_mixes_both_tilt_axes():
    inverse, missing = fit.tilt_lever_inverse(AZIMUTH, ROLL_PIVOT, PITCH_PIVOT)
    assert missing == ()
    d = 0.3
    expected = 0.5 * (ROLL_PIVOT - d) / ROLL_PIVOT + 0.5 * (PITCH_PIVOT - d) / PITCH_PIVOT
    assert 1.0 - d * inverse == pytest.approx(expected)
    # 权重 < 0.02 的轴忽略：纯俯仰杆不需要横滚高度。
    inverse, missing = fit.tilt_lever_inverse(math.pi / 2, None, PITCH_PIVOT)
    assert missing == () and inverse == pytest.approx(1.0 / PITCH_PIVOT)
    inverse, missing = fit.tilt_lever_inverse(0.0, None, None)
    assert missing == ("横滚",)


def test_a_tilt_pivot_on_the_far_side_of_the_rod_is_refused(nominal):
    """倾转轴落在质心与杆之间时，绕杆与绕质心的力臂反号：这是量错了，不是一组能拟合的数。"""
    _result, stamps, samples, _rate = nominal
    with pytest.raises(ValueError):
        fit_record(stamps, samples, pitch_pivot_to_cg_z_m=0.08)


# ---------------------------------------------------------------- 按轴出参数


@pytest.mark.parametrize("azimuth, axes", [
    (math.pi / 2, {"pitch"}), (-math.pi / 2, {"pitch"}), (0.0, {"roll"}),
    (math.pi, {"roll"}), (math.pi / 4, {"roll", "pitch"}), (None, {"roll", "pitch"})])
def test_gains_are_written_only_for_the_identified_axes(nominal, azimuth, axes):
    result, _stamps, _samples, _rate = nominal
    rate, _attitude, commands = core.synthesise_gains(result, azimuth_rad=azimuth)
    assert commands and all(line.startswith("SYSID PARAM coax.") for line in commands)
    written = {axis for axis in ("roll", "pitch") if any(f"_{axis}_" in c for c in commands)}
    assert written == axes
    if azimuth is not None and len(axes) == 2:
        assert "斜杆辨识的是两轴混合" in rate.rationale


def flight_plant(result) -> tune.FlightPlant:
    return tune.FlightPlant(
        inertia_cg_kg_m2=result.inertia_kg_m2, torque_model_scale=result.torque_model_scale,
        dead_time_s=result.dead_time_s, servo_wn_rad_s=result.servo_wn_rad_s,
        servo_zeta=result.servo_zeta, reaction_couple_s2=result.reaction_couple_s2,
        feedback_notch=tune.DEPLOYED_RPM_NOTCH)   # 与 _core.synthesise_gains 同一对象（含转速陷波）


def test_tuning_is_loop_shaped_on_the_flight_twd_plant(nominal):
    result, _stamps, _samples, _rate = nominal
    # 双脉冲激励不到摆的共振：拟合的绕杆阻尼落在 0 附近（真值 0），整定阻尼恒为 0。
    assert 0.0 <= result.damping_n_m_s < 0.03 and result.tuning_damping_n_m_s == 0.0
    rate, attitude, commands = core.synthesise_gains(result)
    expected = tune.synthesise_rate_shaped(flight_plant(result))
    assert rate.kp == pytest.approx(expected.kp) and rate.ki == pytest.approx(expected.ki)
    assert rate.feasible
    assert rate.gain_margin_db >= 6.0 - 1e-6 and rate.phase_margin_deg >= 45.0
    assert rate.gain_margin_db == pytest.approx(result.gain_margin_db)
    assert rate.crossover_hz == pytest.approx(result.crossover_hz)
    assert rate.ki == pytest.approx(rate.kp * 2 * math.pi * rate.crossover_hz / 5, rel=0.02)
    assert attitude.bandwidth_hz == pytest.approx(rate.crossover_hz * tune.ATTITUDE_BANDWIDTH_RATIO)
    assert "κ+ρs²" in rate.rationale and "GM" in rate.rationale and "PM" in rate.rationale
    assert commands and all(line.startswith("SYSID PARAM ") for line in commands)
    assert not any("SAVE" in line.upper() for line in commands)


def test_gains_from_the_rigid_reading_are_unsafe_on_the_twd_plant(nominal):
    """这就是不用刚体读数整定的原因：它合成的 kp 在 TWD 对象上几乎没有增益裕度。"""
    result, _stamps, _samples, _rate = nominal
    rigid = tune.synthesise_rate(result.rigid_tuning_inertia_kg_m2, 0.0, result.rigid_delay_s)
    margins = tune.loop_margins(rigid.kp, rigid.ki, flight_plant(result))
    assert margins.gain_margin_db < 3.0
    shaped = core.synthesise_gains(result)[0]
    assert shaped.kp < rigid.kp


def test_the_result_round_trips_through_json(nominal):
    result, _stamps, _samples, _rate = nominal
    payload = json.loads(json.dumps(asdict(result)))
    for key in ("inertia_kg_m2", "inertia_rod_kg_m2", "pivot_above_cg_m", "eccentricity_m",
                "torque_scale", "damping_n_m_s", "delay_s", "fit_percent", "natural_hz",
                "damping_eccentricity_correlation", "separable", "samples",
                "torque_model_scale", "tuning_inertia_kg_m2", "tuning_damping_n_m_s",
                "inertia_uncertainty_pct", "tuning_blockers", "notes", "structure",
                "dead_time_s", "servo_wn_rad_s", "servo_zeta", "reaction_couple_s2",
                "rig_zero_hz", "flight_zero_hz", "equivalent_delay_1hz_s", "gain_margin_db",
                "phase_margin_deg", "crossover_hz", "rigid_tuning_inertia_kg_m2",
                "rigid_delay_s", "rigid_fit_percent"):
        assert key in payload
    assert payload["tuning_blockers"] == []


def test_the_recorded_fused_angle_does_not_steer_the_fit(nominal):
    result, stamps, samples, _rate = nominal
    garbage = [dict(s, angle=0.7 * math.sin(3.0 * k)) for k, s in enumerate(samples)]
    assert fit_record(stamps, garbage) == result


def test_control_tick_jitter_is_accepted_but_a_real_break_is_not(nominal):
    _result, stamps, samples, _rate = nominal
    median = float(np.median(np.diff(stamps)))
    one_missing = [k for k in range(len(stamps)) if k != 500]
    assert np.max(np.diff(stamps[one_missing])) < 2.5 * median
    fit_record(stamps[one_missing], [samples[k] for k in one_missing])
    broken = [k for k in range(len(stamps)) if k not in (500, 501)]
    assert np.max(np.diff(stamps[broken])) > 2.5 * median
    with pytest.raises(ValueError):
        fit_record(stamps[broken], [samples[k] for k in broken])


def test_a_rig_mode_above_the_band_does_not_move_the_tuning_numbers(nominal):
    """约 14 Hz 的台架/杆弹性次要模态不在模型里：12 Hz/4 阶带把它压下去，
    它只能拉低 15 Hz 诊断值，不能挪动整定惯量、延迟与裕度。"""
    result, stamps, samples, _rate = nominal
    mode = RIG_MODE_AMPLITUDE * np.sin(2 * math.pi * 14.0 * stamps) * np.exp(-0.3 * stamps)
    shaken = [dict(s, gx=s["gx"] + m * math.cos(AZIMUTH), gy=s["gy"] + m * math.sin(AZIMUTH))
              for s, m in zip(samples, mode)]
    again = fit_record(stamps, shaken)
    assert again.tuning_inertia_kg_m2 == pytest.approx(result.tuning_inertia_kg_m2, rel=0.05)
    assert again.dead_time_s == pytest.approx(result.dead_time_s, abs=0.003)
    assert again.fit_percent >= fit.TWD_FIT_PERCENT_MIN
    assert again.fit_percent_15hz < result.fit_percent_15hz - 1.0
    assert again.tuning_blockers == ()


# ---------------------------------------------------------------- 宽带扫频：阻尼与模型无关诊断


@pytest.fixture(scope="module")
def chirp_run():
    """线性执行器上的宽带扫频（0.3→8 Hz，过摆频与 TWD 零点），摆有真实阻尼。"""
    span = CHIRP["pre_roll_s"] + CHIRP["duration_s"]
    stamps, samples, _rate = firmware_record(torque_of=chirp_torque, span_s=span,
                                             damping=CHIRP_DAMPING, seed=11)
    return fit_record(stamps, samples, firmware_tilt_levers_m=EXACT_LEVERS), stamps, samples


def test_a_chirp_through_the_pendulum_resonance_identifies_its_damping(chirp_run):
    """扫频从摆频附近起步：c ≡ 0 时无阻尼共振吃掉整个代价；c 放开后能辨出来。"""
    result, _stamps, _samples = chirp_run
    assert result.damping_n_m_s == pytest.approx(CHIRP_DAMPING, rel=0.15)
    assert result.tuning_damping_n_m_s == 0.0
    assert result.torque_model_scale == pytest.approx(1.0, rel=0.10)
    assert result.dead_time_s == pytest.approx(TRUTH["delay_s"], abs=0.004)
    assert result.servo_wn_rad_s == pytest.approx(TRUTH["servo_wn"], rel=0.10)
    assert result.servo_zeta == pytest.approx(TRUTH["servo_zeta"], rel=0.15)
    assert result.fit_percent >= fit.TWD_FIT_PERCENT_MIN
    assert result.tuning_blockers == ()


def test_the_model_free_notch_and_pendulum_match_the_truth(chirp_run):
    result, _stamps, _samples = chirp_run
    gain = true_scale(TRUTH["pivot"], AZIMUTH, ROLL_PIVOT, PITCH_PIVOT)
    truth_zero = math.sqrt(gain / TRUTH["rho"]) / (2 * math.pi)
    truth_rod = TRUTH["inertia_cg"] + MASS * TRUTH["pivot"] ** 2
    truth_pendulum = math.sqrt(MASS * GRAVITY * TRUTH["pivot"] / truth_rod) / (2 * math.pi)
    assert result.measured_zero_hz == pytest.approx(truth_zero, rel=0.06)
    assert result.measured_pendulum_hz == pytest.approx(truth_pendulum, rel=0.03)
    assert result.rig_zero_hz == pytest.approx(result.measured_zero_hz, rel=0.10)
    assert any("模型无关频响量到的TWD 零点" in w for w in result.warnings)


def test_a_doublet_record_is_too_short_for_the_model_free_diagnostic(nominal):
    result, _stamps, _samples, _rate = nominal
    assert math.isnan(result.measured_zero_hz) and math.isnan(result.measured_pendulum_hz)


# ---------------------------------------------------------------- 多轮联合


def test_a_joint_fit_shares_one_set_of_parameters():
    records = [firmware_record(kappa=1.1, seed=seed, noise=noise)[:2]
               for seed, noise in ((7, 0.03), (8, 0.04))]
    options = dict(azimuth_rad=AZIMUTH, mass_kg=MASS, assumed_inertia_kg_m2=I_EST,
                   thrust_point_to_cg_z_m=H, pivot_above_cg_m=TRUTH["pivot"],
                   roll_pivot_to_cg_z_m=ROLL_PIVOT, pitch_pivot_to_cg_z_m=PITCH_PIVOT)
    result = core.fit_inner_loop_multi(records, **options)
    assert result.torque_model_scale == pytest.approx(1.1, rel=0.10)
    assert result.tuning_inertia_kg_m2 == pytest.approx(TRUTH["inertia_cg"] / 1.1, rel=0.10)
    assert len(result.fit_percent_runs) == 2
    assert min(result.fit_percent_runs) >= fit.TWD_FIT_PERCENT_MIN
    assert result.fit_percent == min(result.fit_percent_runs)
    assert result.tuning_blockers == ()
    assert any("2 轮联合拟合" in w for w in result.warnings)
    assert result.servo_wn_rad_s == pytest.approx(TRUTH["servo_wn"], rel=0.10)
    singles = [fit_record(*record) for record in records]
    assert result.samples == sum(single.samples for single in singles)
    assert result.tuning_inertia_uncertainty_pct < min(
        single.tuning_inertia_uncertainty_pct for single in singles)


def test_a_joint_fit_needs_a_measured_pivot(nominal):
    _result, stamps, samples, _rate = nominal
    with pytest.raises(ValueError):
        core.fit_inner_loop_multi([(stamps, samples)] * 2, azimuth_rad=AZIMUTH, mass_kg=MASS,
                                  assumed_inertia_kg_m2=I_EST)
    with pytest.raises(ValueError):
        core.fit_inner_loop_multi([], azimuth_rad=AZIMUTH, mass_kg=MASS,
                                  assumed_inertia_kg_m2=I_EST, pivot_above_cg_m=0.3)


# ---------------------------------------------------------------- 旧工况：杆过质心


def test_the_legacy_rod_through_the_cg_still_fits():
    stamps, samples, _rate = firmware_record(pivot=0.0, inertia_cg=0.019, damping=0.004,
                                             delay_s=0.03, servo_wn=None, rho=0.0,
                                             roll_pivot=None, pitch_pivot=None, seed=4)
    result = core.fit_inner_loop(stamps, samples, azimuth_rad=AZIMUTH, mass_kg=MASS,
                                 assumed_inertia_kg_m2=0.01)
    assert result.torque_model_scale == 1.0 and result.torque_scale == 1.0
    assert abs(result.pivot_above_cg_m) < 0.02
    assert result.inertia_kg_m2 == pytest.approx(0.019, rel=0.15)
    assert result.tuning_inertia_kg_m2 == result.inertia_kg_m2
    assert result.tuning_damping_n_m_s == result.damping_n_m_s
    assert result.delay_s == pytest.approx(0.03, abs=0.008)
    assert result.fit_percent >= 70.0
    assert result.tuning_blockers == ()
    assert any("未提供推力点位置，按杆过质心近似" in w for w in result.warnings)
    assert any("κ 按 1" in w for w in result.warnings)
    assert result.structure == "legacy"


def test_a_pendulum_fitted_as_if_the_rod_went_through_the_cg_is_blocked(nominal):
    """忘了填杆高：旧工况的残余偏心只放 ±5 cm，装不下 0.13 m 的单摆，必须拦下。"""
    _result, stamps, samples, _rate = nominal
    result = fit_record(stamps, samples, pivot_above_cg_m=None)
    assert result.tuning_blockers


# ---------------------------------------------------------------- 零件


def test_the_legacy_pivot_ratio_inverts_the_torque_scale():
    """旧工况 κ = 1 时回归给出 R = d/s，d = R·h/(h + R) 必须把它原样解回来。"""
    for d in (0.0, 0.01, 0.04, -0.03):
        scale = fit.torque_scale(d, H)
        assert scale == pytest.approx((H - d) / H)
        assert fit.pivot_from_ratio(d / scale, H) == pytest.approx(d, abs=1e-12)
    assert fit.torque_scale(0.3, None) == 1.0
    assert fit.pivot_bounds(H) == (-0.05, 0.05)
    assert fit.pivot_bounds(None) == (-0.05, 0.05)
    assert fit.pivot_bounds(0.03)[1] == pytest.approx(0.01)


def test_the_hand_discretised_model_matches_scipy_lsim():
    """FOH 离散化是手推的：拿 scipy 的连续时间仿真对拍。"""
    from scipy import signal

    fs = 250.0
    t = np.arange(0.0, 4.0, 1.0 / fs)
    torque = np.array([feedforward_torque(x) for x in t])
    inertia, damping, pivot, kappa = 0.108, 0.01, 0.3, 1.2
    gain = kappa * (H - pivot) / H
    ours = fit.simulate(np.array([inertia, damping, pivot, 0.0]), t, torque, MASS, GRAVITY,
                        thrust_point_to_cg_z_m=H, torque_model_scale=kappa)
    a = np.array([[0.0, 1.0], [-MASS * GRAVITY * pivot / inertia, -damping / inertia]])
    b = np.array([[0.0], [gain / inertia]])
    _, reference, _ = signal.lsim((a, b, np.array([[0.0, 1.0]]), np.array([[0.0]])),
                                  U=torque, T=t)
    assert np.max(np.abs(ours - reference)) < 1e-9 * max(1.0, np.max(np.abs(reference)))


def test_the_twd_simulation_matches_scipy_lsim():
    """n 状态 FOH + lfilter 的 TWD 仿真是手推的：拿 scipy 的连续时间仿真对拍。"""
    from scipy import signal

    fs = 250.0
    t = np.arange(0.0, 4.0, 1.0 / fs)
    torque = np.array([feedforward_torque(x) for x in t])
    inertia, gain, wn, zeta, rho = 0.054, 2.1, 33.0, 0.44, 0.007
    stiffness = MASS * GRAVITY * TRUTH["pivot"]
    ours = fit.simulate_twd(t, torque, inertia_rod=inertia, gain=gain, stiffness=stiffness,
                            dead_time_s=0.0, servo_wn=wn, servo_zeta=zeta, reaction_couple=rho)
    a, b, c = fit._twd_matrices(inertia, gain, stiffness, wn, zeta, rho)
    _, reference, _ = signal.lsim((a, b[:, None], c[None, :], np.array([[0.0]])),
                                  U=torque, T=t)
    assert np.max(np.abs(ours - reference)) < 1e-8 * max(1.0, np.max(np.abs(reference)))


def test_the_lowpass_adds_no_delay():
    fs = 250.0
    t = np.arange(0.0, 4.0, 1.0 / fs)
    wave = np.sin(2 * math.pi * 1.0 * t)
    filtered = fit.lowpass_zero_phase(wave, fs)
    middle = slice(100, -100)
    assert np.max(np.abs(filtered[middle] - wave[middle])) < 1e-3
    assert np.array_equal(filtered, fit.lowpass_zero_phase(wave, fs, order=2))
    slow = np.sin(2 * math.pi * 0.5 * t)
    band = fit.lowpass_zero_phase(slow, fs, fit.FIT_BAND_HZ, fit.FIT_BAND_ORDER)
    inner = slice(250, -250)       # 4 阶 4 Hz 的端点瞬态更长，多让开一点
    assert np.max(np.abs(band[inner] - slow[inner])) < 1e-3


def test_a_non_positive_cg_inertia_is_flagged():
    result = fit.ModelFit(
        inertia_kg_m2=-0.01, inertia_rod_kg_m2=0.06, pivot_above_cg_m=0.3,
        eccentricity_m=0.3, torque_scale=2.5, damping_n_m_s=0.01, delay_s=0.03,
        fit_percent=95.0, natural_hz=1.0, damping_eccentricity_correlation=0.1,
        separable=True, samples=900, torque_model_scale=1.0,
        tuning_inertia_kg_m2=-0.01, tuning_damping_n_m_s=0.002,
        inertia_uncertainty_pct=float("inf"), tuning_inertia_uncertainty_pct=float("inf"),
        fit_percent_15hz=60.0, tuning_blockers=("绕质心惯量不为正",))
    assert any("不为正" in w for w in result.warnings)


def test_a_zero_thrust_lever_is_refused(nominal):
    _result, stamps, samples, _rate = nominal
    with pytest.raises(ValueError):
        fit_record(stamps, samples, thrust_point_to_cg_z_m=0.0)

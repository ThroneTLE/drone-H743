"""舵机单独（电机不转、全程上锁）：拟合、折算与页面流程。

真值数据由本文件合成（RK4 细步积分，sin θ 非线性，控制拍 2.0–3.1 ms 抖动、250 Hz 在拍上
抽样），只用于单元测试；实机还没有舵机单独的记录。模型::

    I_杆·θ̈ + c·θ̇ + K·θ = −J·s̈ + H·s，  s̈ = ωs²·(u(t − T) − s) − 2ζs·ωs·ṡ

H·s 是倾转组件的静态重力偏心（组件质心不在舵机轴上），默认 0，专门的用例里加上。
u 是指令倾角（固件 v3 记录的 servo_tilt，正向 = 杆轴正向力矩）。J < 0 = 舵机加速把机体推向
推力力矩的同一方向（带桨时的 TWD 凹口）。回差放在指令侧（迟滞），用来验证幅值依赖的估计。
"""
from __future__ import annotations

import importlib.util
import json
import math
import struct
import sys
from pathlib import Path

import pytest

np = pytest.importorskip("numpy")
pytest.importorskip("scipy")

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from sysid import fit, servo_fit  # noqa: E402
sys.path.pop(0)

_spec = importlib.util.spec_from_file_location(
    "sysid_panel_core_servo", ROOT / "tools" / "panel_lib" / "pages" / "sysid" / "_core.py")
core = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(core)

from test_sysid_page import (  # noqa: E402,F401  页面夹具与报文工具
    FLAG_FIRST_BATCH, FLAG_LAST_BATCH, GEOMETRIC_START, feed, page, status_report, thr_line,
    wait_for,
)

MASS, GRAVITY, PIVOT = 0.7546, 9.81, 0.05
STIFFNESS = MASS * GRAVITY * PIVOT
TRUTH = dict(inertia_rod=0.045, reaction=-0.008, damping=0.01, delay=0.030, wn=38.0, zeta=0.5,
             pod_gravity=0.0)


def doublet(time_s: float, amplitude: float, hold: float = 0.25, pre: float = 0.5,
            pairs: int = 10) -> float:
    local = time_s - pre
    if local < 0.0 or local >= pairs * 2 * hold:
        return 0.0
    return amplitude if (local % (2 * hold)) < hold else -amplitude


def servo_record(amplitude: float = 0.087, *, backlash: float = 0.0, seed: int = 1,
                 noise: float = 0.02, truth: dict | None = None):
    """(时间戳 s, 指令倾角, 陀螺沿杆轴)：舵机单独的一轮，含 0.5 s 零激励前导。"""
    truth = {**TRUTH, **(truth or {})}
    rng = np.random.default_rng(seed)
    span = 0.5 + 10 * 0.5 + 1.0
    ticks = np.concatenate([[0.0], np.cumsum(rng.uniform(0.0020, 0.0031, int(span / 0.002) + 10))])
    raw = np.array([doublet(x, amplitude) for x in ticks])
    held = np.zeros_like(raw)
    level = 0.0
    for index, value in enumerate(raw):          # 指令侧回差：先走完 ±b 的空程才带动
        if value - level > backlash:
            level = value - backlash
        elif level - value > backlash:
            level = value + backlash
        held[index] = level

    def applied(time_s):
        index = int(np.searchsorted(ticks, time_s - truth["delay"], side="right")) - 1
        return float(held[index]) if index >= 0 else 0.0

    def derivative(time_s, state):
        angle, rate, servo, servo_rate = state
        servo_accel = (truth["wn"] ** 2 * (applied(time_s) - servo)
                       - 2.0 * truth["zeta"] * truth["wn"] * servo_rate)
        accel = (-truth["damping"] * rate - STIFFNESS * math.sin(angle)
                 - truth["reaction"] * servo_accel + truth["pod_gravity"] * servo) / truth["inertia_rod"]
        return np.array([rate, accel, servo_rate, servo_accel])

    step = 0.0005
    times = np.arange(int(round(span / step)) + 1) * step
    rates = np.zeros(times.size)
    state = np.zeros(4)
    for k, now in enumerate(times):
        rates[k] = state[1]
        k1 = derivative(now, state)
        k2 = derivative(now + step / 2, state + step / 2 * k1)
        k3 = derivative(now + step / 2, state + step / 2 * k2)
        k4 = derivative(now + step, state + step * k3)
        state = state + step / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    chosen, due = [], 0.0
    for index, tick in enumerate(ticks):
        if tick >= span:
            break
        if tick >= due - 1e-12:
            chosen.append(index)
            due += 0.004
    stamps = ticks[chosen]
    rate = np.interp(stamps, times, rates) + 0.003 + noise * rng.standard_normal(stamps.size)
    return stamps, raw[chosen], rate


def as_samples(stamps, tilt, rate, azimuth=math.pi / 2):
    return [{"gx": float(r * math.cos(azimuth)), "gy": float(r * math.sin(azimuth)), "gz": 0.0,
             "servo_tilt": float(u), "torque": 0.0, "thrust": 0.0, "erpm": 0.0, "erpm_lower": 0.0}
            for u, r in zip(tilt, rate)]


def fit_servo(amplitude=0.087, **kwargs):
    stamps, tilt, rate = servo_record(amplitude, **kwargs)
    return core.fit_servo_only(stamps, as_samples(stamps, tilt, rate), azimuth_rad=math.pi / 2,
                               mass_kg=MASS, assumed_inertia_kg_m2=0.04, pivot_above_cg_m=PIVOT)


@pytest.fixture(scope="module")
def nominal():
    return fit_servo()


# ---------------------------------------------------------------- 拟合


def test_the_reaction_servo_and_delay_are_recovered(nominal):
    result = nominal
    assert result.structure == "servo"
    assert result.reaction_inertia_kg_m2 == pytest.approx(TRUTH["reaction"], rel=0.08)
    assert result.inertia_rod_kg_m2 == pytest.approx(TRUTH["inertia_rod"], rel=0.08)
    assert result.stiffness_n_m_rad == pytest.approx(STIFFNESS)
    assert result.servo_wn_rad_s == pytest.approx(TRUTH["wn"], rel=0.05)
    assert result.servo_zeta == pytest.approx(TRUTH["zeta"], rel=0.10)
    # 真值纯延迟 30 ms，外加指令在控制拍上保持的约半拍。
    assert result.dead_time_s == pytest.approx(TRUTH["delay"], abs=0.003)
    assert result.fit_percent >= servo_fit.FIT_PERCENT_MIN and result.blockers == ()
    assert result.reaction_uncertainty_pct < 10.0
    assert result.amplitude_rad == pytest.approx(0.087)
    assert any("TWD 凹口" in note for note in result.notes), "J < 0 的含义要说出来"


def test_the_reaction_sign_is_free():
    result = fit_servo(truth={"reaction": 0.006})
    assert result.reaction_inertia_kg_m2 == pytest.approx(0.006, rel=0.10)
    assert any("实零点" in note for note in result.notes)


def test_a_measured_rig_stiffness_replaces_mgd():
    """线缆多出的刚度：K 按挂砝码实测给，I_杆 与 J 按它标定；只按 m·g·d 算会同比例偏低。"""
    extra = 0.3
    stamps, tilt, rate = servo_record(truth={})
    samples = as_samples(stamps, tilt, rate)
    options = dict(azimuth_rad=math.pi / 2, mass_kg=MASS, assumed_inertia_kg_m2=0.04,
                   pivot_above_cg_m=PIVOT)
    plain = core.fit_servo_only(stamps, samples, **options)
    scaled = core.fit_servo_only(stamps, samples, rig_stiffness_n_m_rad=STIFFNESS + extra, **options)
    ratio = (STIFFNESS + extra) / STIFFNESS
    assert scaled.inertia_rod_kg_m2 == pytest.approx(plain.inertia_rod_kg_m2 * ratio, rel=0.02)
    assert scaled.reaction_inertia_kg_m2 == pytest.approx(plain.reaction_inertia_kg_m2 * ratio, rel=0.02)


@pytest.mark.parametrize("pod_gravity", [0.35, -0.35])
def test_a_static_pod_gravity_term_does_not_bias_the_reaction(pod_gravity):
    """倾转组件质心不在舵机轴上：倾转 s 时绕杆多出 H·s（本机 servo_motor 348.6 g、质心在轴下
    0.114 m → 约 0.39 N·m/rad，与 m·g·d 同量级）。模型里没有这一项时它拽偏摆频，J 偏 −34%/+15%
    而报的不确定度只有 2–3%；单独拟合 H 之后 J、I_杆 回到真值，H 本身也找得回来。"""
    result = fit_servo(truth={"pod_gravity": pod_gravity})
    assert result.reaction_inertia_kg_m2 == pytest.approx(TRUTH["reaction"], rel=0.05)
    assert result.inertia_rod_kg_m2 == pytest.approx(TRUTH["inertia_rod"], rel=0.05)
    assert result.pod_gravity_n_m_rad == pytest.approx(pod_gravity, rel=0.12)
    assert result.blockers == () and result.reaction_uncertainty_pct < 5.0
    assert any("静态重力偏心 H" in note for note in result.notes)


def test_a_weak_pod_gravity_term_widens_the_uncertainty_instead_of_hiding_a_bias():
    """H 小（0.05 N·m/rad）时它与摆频分不太开：J 的误差要落在报出的不确定度之内，不许
    "误差 3 倍、不确定度照报几个百分点"。"""
    for seed in (1, 2):
        result = fit_servo(truth={"pod_gravity": 0.05}, seed=seed)
        error = abs(result.reaction_inertia_kg_m2 - TRUTH["reaction"])
        sigma = result.reaction_uncertainty_pct / 100.0 * abs(result.reaction_inertia_kg_m2)
        assert error <= 2.5 * sigma, (seed, result.reaction_inertia_kg_m2, sigma)
        assert result.reaction_inertia_kg_m2 == pytest.approx(TRUTH["reaction"], rel=0.25)


def test_the_simulator_carries_the_pod_gravity_term():
    t = np.arange(0, 4.0, 0.004)
    tilt = np.array([doublet(x, 0.087, pairs=6) for x in t])
    options = dict(inertia_rod=0.045, stiffness=STIFFNESS, damping=0.01, reaction_inertia=0.0,
                   dead_time_s=0.03, servo_wn=38.0, servo_zeta=0.5)
    static = servo_fit.simulate_servo_only(t, tilt, pod_gravity=0.35, **options)
    # H·s 等价于带桨 TWD 里的 G = H（u = s）、ρ = 0。
    twd = fit.simulate_twd(t, tilt, inertia_rod=0.045, gain=0.35, stiffness=STIFFNESS,
                           dead_time_s=0.03, servo_wn=38.0, servo_zeta=0.5, reaction_couple=0.0,
                           damping=0.01)
    assert np.max(np.abs(static)) > 0.05 and np.allclose(static, twd, atol=1e-9)


def test_the_predicted_couple_matches_the_twd_convention():
    """ρ_pred = −J/(|L|·T)：把 u = |L|·T·s 喂给带桨 TWD 模型（G = 0、ρ = ρ_pred），
    得到的角速度与舵机单独模型逐点相同——两边同一个符号约定，可以直接比。"""
    lever, thrust = 0.035442, 7.4
    rho = servo_fit.predicted_reaction_couple(TRUTH["reaction"], lever, thrust)
    assert rho == pytest.approx(0.008 / (lever * thrust))
    assert servo_fit.predicted_reaction_couple(TRUTH["reaction"], -lever, thrust) == rho
    t = np.arange(0, 4.0, 0.004)
    tilt = np.array([doublet(x, 0.087, pairs=6) for x in t])
    servo_only = servo_fit.simulate_servo_only(
        t, tilt, inertia_rod=0.045, stiffness=STIFFNESS, damping=0.01,
        reaction_inertia=TRUTH["reaction"], dead_time_s=0.03, servo_wn=38.0, servo_zeta=0.5)
    twd = fit.simulate_twd(t, lever * thrust * tilt, inertia_rod=0.045, gain=0.0,
                           stiffness=STIFFNESS, dead_time_s=0.03, servo_wn=38.0, servo_zeta=0.5,
                           reaction_couple=rho, damping=0.01)
    assert np.max(np.abs(servo_only)) > 0.05
    assert np.allclose(servo_only, twd, atol=1e-9)
    assert math.isnan(servo_fit.predicted_reaction_couple(-0.008, None, thrust))


def test_the_rod_axis_lever_weights_the_tilt_axes():
    assert servo_fit.rod_axis_lever((0.03, 0.04), math.pi / 2) == pytest.approx(0.04)
    assert servo_fit.rod_axis_lever((0.03, -0.04), math.pi / 4) == pytest.approx(0.035)
    assert servo_fit.rod_axis_lever(None, 0.0) is None


def test_backlash_shows_up_as_amplitude_dependent_reaction():
    """指令侧回差 ±0.01 rad：双脉冲幅值 A 的表观 J = J·(1 − b/A)，对 1/A 回归找回 J 与 b。"""
    backlash = 0.01
    runs = []
    for amplitude in (0.04, 0.087, 0.2):
        result = fit_servo(amplitude, backlash=backlash)
        expected = TRUTH["reaction"] * (1.0 - backlash / amplitude)
        assert result.reaction_inertia_kg_m2 == pytest.approx(expected, rel=0.08)
        runs.append((result.amplitude_rad, result.reaction_inertia_kg_m2, result.dead_time_s,
                     result.equivalent_delay_1hz_s))
    estimate = servo_fit.backlash_estimate(runs)
    assert estimate is not None and len(estimate.rows) == 3
    assert estimate.backlash_rad == pytest.approx(backlash, rel=0.25)
    assert estimate.reaction_limit_kg_m2 == pytest.approx(TRUTH["reaction"], rel=0.08)
    assert "描述函数" in estimate.notes[0]


def test_backlash_needs_two_distinct_amplitudes():
    assert servo_fit.backlash_estimate([(0.087, -0.008, 0.03, 0.05)]) is None
    assert servo_fit.backlash_estimate([(0.087, -0.008, 0.03, 0.05),
                                        (0.088, -0.0079, 0.03, 0.05)]) is None
    flat = servo_fit.backlash_estimate([(0.05, -0.008, 0.03, 0.05), (0.2, -0.008, 0.03, 0.05)])
    assert math.isnan(flat.backlash_rad) or flat.backlash_rad < 1e-3


def test_a_still_servo_or_missing_field_is_refused():
    stamps, tilt, rate = servo_record()
    with pytest.raises(ValueError, match="几乎没动"):
        servo_fit.fit_servo_only(*fit.uniform_signal(stamps, np.zeros_like(tilt)),
                                 fit.uniform_signal(stamps, rate)[1], stiffness_n_m_rad=STIFFNESS,
                                 inertia_rod_guess_kg_m2=0.04)
    samples = [{k: v for k, v in s.items() if k != "servo_tilt"}
               for s in as_samples(stamps, tilt, rate)]
    with pytest.raises(ValueError, match="servo_tilt"):
        core.fit_servo_only(stamps, samples, azimuth_rad=math.pi / 2, mass_kg=MASS,
                            assumed_inertia_kg_m2=0.04, pivot_above_cg_m=PIVOT)
    with pytest.raises(ValueError, match="回中刚度"):
        core.fit_servo_only(stamps, as_samples(stamps, tilt, rate), azimuth_rad=math.pi / 2,
                            mass_kg=MASS, assumed_inertia_kg_m2=0.04, pivot_above_cg_m=0.005)


# ---------------------------------------------------------------- 记录 v3 与解码


V3_FIELDS = (("gx", "i16", 0.001), ("gy", "i16", 0.001), ("omega_sp", "i16", 0.001),
             ("angle", "i16", 0.0001), ("angle_sp", "i16", 0.0001), ("torque", "i16", 0.0001),
             ("erpm", "u16", 4.0), ("erpm_lower", "u16", 4.0), ("servo_tilt", "i16", 0.0001))
V3_HASH = 0x5E7B0003
SCHEMA_V3_LINES = [f"SYSID SCHEMA ver=3 n={len(V3_FIELDS)} hash={V3_HASH:08X} rec={2 * len(V3_FIELDS)}"] + [
    f"SYSID FIELD idx={i} name={name} unit=x scale={scale:.9f} type={kind}"
    for i, (name, kind, scale) in enumerate(V3_FIELDS)]
FLAG_SERVO = 0x0080


def v3_frame(run, rows, *, flags, base_us, dt_us=4000):
    header = struct.pack("<BBHIIHH", 3, len(rows), run, V3_HASH, base_us, dt_us, flags)
    return header + b"".join(struct.pack("<6hHHh", *row) for row in rows)


def test_the_bridge_decodes_v3_frames_and_still_refuses_a_stale_table():
    schema = core.parse_schema_lines(SCHEMA_V3_LINES)
    frame = v3_frame(5, [(100, -200, 0, 0, 0, 0, 3000, 2500, 870)], flags=FLAG_FIRST_BATCH | FLAG_SERVO,
                     base_us=1000)
    batch = core.decode_batch(frame, schema)
    sample = batch.samples[0]
    assert sample["servo_tilt"] == pytest.approx(0.087) and sample["erpm"] == 12000.0
    assert sample["erpm_lower"] == 10000.0 and sample["gy"] == pytest.approx(-0.2)
    from dataclasses import replace
    with pytest.raises(core.SchemaMismatch):
        core.decode_batch(frame, replace(schema, hash=schema.hash ^ 0xFF))


# ---------------------------------------------------------------- 页面：开跑、收数、分析


def servo_report(expected, **report):
    """整份状态报告；v3 固件的 EXC 行末尾多一个 servo_tilt_mrad。"""
    lines = status_report(expected, armed=0, **report)
    tilt = expected.get("servo_tilt_mrad", 87)
    return [f"{line} servo_tilt_mrad={tilt}" if line.startswith("SYSID EXC ") else line
            for line in lines]


def drive_servo(page, *, tilt="5"):
    page.mode_var.set("SERVO")
    page.servo_tilt_var.set(tilt)
    page.psi_var.set("90")
    page.rod_to_fc_var.set("0.04")
    page.pitch_pivot_var.set("-0.13")
    page.handle_line(thr_line(armed=0))
    page.start_run()
    assert page.panel.transport.lines[-1] == "SYSID SCHEMA"
    feed(page, SCHEMA_V3_LINES)
    feed(page, [f"PARAM name={name} value={value}" for name, value in GEOMETRIC_START])
    for _ in range(12):
        awaiting = page.workflow.awaiting
        if not awaiting or awaiting == "SYSID START":
            break
        feed(page, servo_report(page.workflow.expected))


def test_servo_mode_start_keeps_motors_off_and_verifies_the_tilt(page):
    drive_servo(page)
    sent = page.panel.transport.lines
    assert sent[-1] == "SYSID START"
    exc = next(line for line in sent if line.startswith("SYSID EXC "))
    assert exc.endswith(" servo_tilt_mrad=87")
    assert "SYSID MODE SERVO 3" in sent
    assert not any(line.startswith("SYSID THROTTLE") for line in sent), "电机不转：不下发程序油门"
    assert any(line.startswith("SYSID RIG psi_deg=90 ") for line in sent)
    assert page.workflow.expected["mode"] == 3 and page.workflow.expected["servo_tilt_mrad"] == 87


def test_a_wrong_tilt_echo_cancels_the_start(page):
    page.mode_var.set("SERVO")
    page.rod_to_fc_var.set("0.04")
    page.handle_line(thr_line(armed=0))
    page.start_run()
    feed(page, SCHEMA_V3_LINES)
    feed(page, [f"PARAM name={name} value={value}" for name, value in GEOMETRIC_START])
    for _ in range(12):
        if not page.workflow.awaiting or page.workflow.awaiting == "SYSID START":
            break
        feed(page, servo_report({**page.workflow.expected, "servo_tilt_mrad": 50}))
    assert "SYSID START" not in page.panel.transport.lines
    assert "servo_tilt_mrad" in page.status_var.get()


def test_servo_mode_refuses_to_start_while_armed(page):
    page.mode_var.set("SERVO")
    page.rod_to_fc_var.set("0.04")
    page.handle_line(thr_line(armed=1))
    page.refresh_banner()
    assert page.banner_var.get() == "请先上锁"
    assert "保持上锁；解锁会立即停止" in page.banner_detail_var.get()
    page.start_run()
    assert page.panel.transport.lines == []
    assert "保持上锁" in page.banner_detail_var.get()
    page.handle_line(thr_line(armed=0))
    page.workflow.notice = ""
    page.refresh_banner()
    assert page.banner_var.get() == "就绪（舵机单独），可以开始"


@pytest.mark.parametrize("tilt", ["0.2", "20", "abc"])
def test_an_out_of_range_tilt_is_refused_before_sending(page, tilt):
    page.mode_var.set("SERVO")
    page.servo_tilt_var.set(tilt)
    page.rod_to_fc_var.set("0.04")
    page.start_run()
    assert page.panel.transport.lines == []
    assert "舵机摆幅" in page.banner_detail_var.get()


def test_the_firmware_refusal_and_the_rc_arm_abort_are_explained():
    sys.path.insert(0, str(ROOT / "tools"))
    try:
        from panel_lib.pages.sysid.reasons import explain_end, explain_error
    finally:
        sys.path.pop(0)
    assert "保持上锁" in explain_error("ERR sysid servo mode needs disarmed")
    what, step = explain_end("rc_arm", "3")
    assert "解锁" in what and "上锁" in step


def servo_rows(n):
    return [(int(300 * math.sin(i / 6)), int(300 * math.sin(i / 6)), 0, 0, 0, 0, 0, 0,
             int(870 if (i // 20) % 2 else -870)) for i in range(n)]


def start_servo_run(page):
    drive_servo(page)
    page.handle_line("SYSID start run=3 profile=1 amp_mrad_s=65 dur_ms=8000 rate_hz=250 "
                     "I=20000 ugm2 psi_mrad=1570 auto=0 target_cn=0")


def test_servo_frames_must_carry_the_servo_flag(page):
    start_servo_run(page)
    rows = servo_rows(10)
    page.accept(v3_frame(3, rows, flags=FLAG_FIRST_BATCH, base_us=1000))
    assert "采样模式与本轮快照不符" in page.status_var.get()


def test_a_finished_servo_run_is_saved_fitted_and_gives_no_gains(page, monkeypatch, tmp_path):
    from panel_lib.pages.sysid import analysis
    calls = []
    fake = servo_fit.ServoFit(
        reaction_inertia_kg_m2=-0.008, reaction_ratio=-0.18, pod_gravity_n_m_rad=0.35,
        inertia_rod_kg_m2=0.045, stiffness_n_m_rad=0.37, damping_n_m_s=0.01, natural_hz=0.46, dead_time_s=0.031,
        servo_wn_rad_s=38.0, servo_zeta=0.5, equivalent_delay_1hz_s=0.05, fit_percent=95.0,
        amplitude_rad=0.087, reaction_uncertainty_pct=4.0, samples=100)
    monkeypatch.setattr(analysis, "fit_servo_only", lambda *a, **k: calls.append(k) or fake)
    folder = tmp_path / "2026-09-27" / "rod_050000_servo"
    monkeypatch.setattr(page.workflow, "archive_directory", lambda: folder)
    start_servo_run(page)
    rows = servo_rows(100)
    page.accept(v3_frame(3, rows[:50], flags=FLAG_FIRST_BATCH | FLAG_SERVO, base_us=1000))
    page.accept(v3_frame(3, rows[50:], flags=FLAG_LAST_BATCH | FLAG_SERVO, base_us=201000))
    assert page.samples[0]["servo_tilt"] == pytest.approx(-0.087)
    page.handle_line("SYSID end run=3 state=done reason=complete dropped=0")
    assert wait_for(page, lambda: page.workflow.job is None and "反作用惯量" in page.fit_var.get())
    (kwargs,) = calls
    assert kwargs["azimuth_rad"] == pytest.approx(1.570)
    assert kwargs["pivot_above_cg_m"] == pytest.approx(0.075)        # 0.04 + 飞控到质心 0.035
    assert "rig_stiffness_n_m_rad" not in kwargs
    assert kwargs["pendulum_hz"] is None, "同组没有带桨拟合时不钉摆频"
    card = page.fit_var.get()
    # 本轮固件力臂 0.095 m（重心 0.045 − 转轴 −0.05），悬停推力 = 机重 9.81 N。
    rho = 0.008 / (0.095 * 9.80665)
    assert f"{rho:.4g} s²" in card and "ρ_pred = −J/(|L|·T)" in card
    assert "不给建议参数" in page.gain_var.get()
    saved = json.loads((folder / "fit_servo.json").read_text(encoding="utf-8"))
    assert saved["fit"]["reaction_inertia_kg_m2"] == -0.008
    assert saved["fit"]["pod_gravity_n_m_rad"] == 0.35, "H 存进结果，同组带桨轮核对时要用"
    assert "静态重力偏心 H = +0.3500 N·m/rad" in page.plant_var.get() + card
    assert (folder / "report_servo.md").exists()
    page.apply_to_ram()
    assert "没有候选参数" in page.status_var.get()
    assert not any(line.startswith(("SYSID PARAM", "PARAM SET")) for line in page.panel.transport.lines)


# ---------------------------------------------------------------- 同组对照与幅值依赖


def write_run(root, name, *, mode, echo, fit_name=None, fit_data=None, target_cn="740"):
    folder = root / "2026-09-27" / name
    folder.mkdir(parents=True)
    conditions = {"mode": mode, "psi_mrad": "1570", "axis_off_um": "50000", "mass_mg": "754600",
                  "target_cn": target_cn, "start": {"target_cn": target_cn},
                  "parameter_echo": echo, "end": {"state": "done", "reason": "complete"}}
    (folder / "conditions.json").write_text(json.dumps(conditions), encoding="utf-8")
    if fit_name:
        (folder / fit_name).write_text(json.dumps(fit_data), encoding="utf-8")
    return folder, conditions


TONIGHT = {"airframe.cg_z_m": "-0.094558", "airframe.servo1_axis_z_m": "-0.130000",
           "airframe.servo2_axis_z_m": "-0.130000", "airframe.weight_n": "7.400000",
           "airframe.servo_motor_mass_g": "348.600000", "airframe.servo_motor_cg_z_m": "-0.244000"}
LEGACY = {"airframe.roll_thrust_lever_arm_m": "0.145000",
          "airframe.pitch_thrust_lever_arm_m": "0.145000",
          "airframe.thrust_point_to_cg_z_m": "-0.200942", "airframe.weight_n": "7.400000"}


def test_the_card_compares_with_the_latest_twd_fit_and_reports_amplitude(tmp_path):
    sys.path.insert(0, str(ROOT / "tools"))
    try:
        from panel_lib.pages.sysid import servo_results
    finally:
        sys.path.pop(0)
    write_run(tmp_path, "rod_010000_twd", mode="0", echo=LEGACY, fit_name="fit.json",
              fit_data={"structure": "twd", "reaction_couple_s2": 0.012, "servo_wn_rad_s": 33.0,
                        "servo_zeta": 0.45, "dead_time_s": 0.037})
    for name, amplitude, reaction in (("rod_020000_s1", 0.04, -0.006), ("rod_021000_s2", 0.2, -0.0076)):
        write_run(tmp_path, name, mode="3", echo=TONIGHT, fit_name="fit_servo.json",
                  fit_data={"fit": {"amplitude_rad": amplitude, "reaction_inertia_kg_m2": reaction,
                                    "dead_time_s": 0.03, "equivalent_delay_1hz_s": 0.05,
                                    "stiffness_n_m_rad": 0.37}})
    folder, conditions = write_run(tmp_path, "rod_030000_s3", mode="3", echo=TONIGHT)
    result = fit_servo()
    outcome = servo_results.summarise(result, conditions, folder, azimuth_rad=math.pi / 2,
                                      root=tmp_path)
    reaction = result.reaction_inertia_kg_m2
    assert outcome.lever_m == pytest.approx(0.035442) and outcome.hover_thrust_n == 7.4
    assert outcome.predicted_rho_s2 == pytest.approx(-reaction / (0.035442 * 7.4))
    comparison = outcome.comparison
    assert comparison["folder"] == "rod_010000_twd"
    assert comparison["lever_m"] == pytest.approx(0.145 * 0.569) and comparison["thrust_n"] == 7.4
    assert comparison["predicted_rho_s2"] == pytest.approx(-reaction / (0.145 * 0.569 * 7.4))
    assert outcome.amplitude is not None and len(outcome.amplitude.rows) == 3
    text = servo_results.servo_card(outcome)
    assert "对照同组最近的带桨 TWD 拟合（rod_010000_twd）" in text and "只作交叉核对" in text
    assert f"静态重力偏心 H = {result.pod_gravity_n_m_rad:+.4f} N·m/rad" in text
    # 参考：servo_motor 348.6 g、质心 −0.244，俯仰杆（ψ = 90°）取舵机 2 转轴 −0.13 → r = 0.114 m。
    assert outcome.pod_reference == pytest.approx((0.3486, 0.114, 0.3486 * 9.80665 * 0.114))
    assert "servo_motor 组件 348.6 g、质心离舵机轴 0.114 m" in text
    assert "【幅值依赖（同组舵机单独轮）】" in text and "回差半宽" in text and "描述函数" in text
    assert "取同组带桨拟合" not in text, "没传钉死摆频就不能说钉了"


def test_the_pendulum_is_pinned_from_the_group_powered_fit(tmp_path):
    """舵机单独激不起杆摆：摆频取同组最近的带桨 TWD 拟合（2026-09-27 实机两轮 3° 自由拟合跑到
    0.80 / 1.55 Hz）；别组的、没有带桨拟合的都不钉。"""
    sys.path.insert(0, str(ROOT / "tools"))
    try:
        from panel_lib.pages.sysid import servo_results
    finally:
        sys.path.pop(0)
    folder, conditions = write_run(tmp_path, "rod_030000_s3", mode="3", echo=TONIGHT)
    assert servo_results.group_pendulum_hz(folder, conditions) is None
    other = tmp_path / "2026-09-27" / "rod_005000_other"
    other.mkdir(parents=True)
    (other / "conditions.json").write_text(json.dumps(
        {"mode": "0", "psi_mrad": "1570", "axis_off_um": "61800",
         "end": {"state": "done", "reason": "complete"}}), encoding="utf-8")
    (other / "fit.json").write_text(json.dumps(
        {"structure": "twd", "reaction_couple_s2": 0.003, "natural_hz": 0.9}), encoding="utf-8")
    assert servo_results.group_pendulum_hz(folder, conditions) is None, "杆高不同不是同组"
    write_run(tmp_path, "rod_010000_twd", mode="0", echo=LEGACY, fit_name="fit.json",
              fit_data={"structure": "twd", "reaction_couple_s2": 0.012, "natural_hz": 0.46})
    assert servo_results.group_pendulum_hz(folder, conditions) == (0.46, "rod_010000_twd")
    assert servo_results.group_pendulum_hz(None, conditions) is None

    stamps, tilt, rate = servo_record()
    pinned = core.fit_servo_only(stamps, as_samples(stamps, tilt, rate), azimuth_rad=math.pi / 2,
                                 mass_kg=MASS, assumed_inertia_kg_m2=0.04, pivot_above_cg_m=PIVOT,
                                 pendulum_hz=0.46)
    assert pinned.natural_hz == pytest.approx(0.46, rel=1e-6)
    assert pinned.inertia_rod_kg_m2 == pytest.approx(STIFFNESS / (2 * math.pi * 0.46) ** 2, rel=1e-6)
    assert any("钉死" in note for note in pinned.notes)
    outcome = servo_results.summarise(pinned, conditions, folder, azimuth_rad=math.pi / 2,
                                      root=tmp_path, pendulum_pin=(0.46, "rod_010000_twd"))
    assert "摆频 0.460 Hz（取同组带桨拟合 rod_010000_twd" in servo_results.servo_card(outcome)


def test_the_servo_settings_are_remembered(page):
    from test_sysid_page import fresh_page, settings_file
    drive_servo(page, tilt="8")
    saved = json.loads(settings_file().read_text(encoding="utf-8"))
    assert saved["mode"] == "SERVO" and saved["servo_tilt_deg"] == 8
    again = fresh_page(page)
    assert again.mode_var.get() == "SERVO" and again.servo_tilt_var.get() == "8"
    assert again.workflow.expected == {}


def test_the_archived_v3_csv_reads_back_with_the_new_fields(page, monkeypatch, tmp_path):
    from panel_lib.pages.sysid import analysis
    from panel_lib.pages.sysid.joint import read_samples
    monkeypatch.setattr(analysis, "fit_servo_only", lambda *a, **k: (_ for _ in ()).throw(ValueError("跳过")))
    folder = tmp_path / "2026-09-27" / "rod_060000_servo"
    monkeypatch.setattr(page.workflow, "archive_directory", lambda: folder)
    start_servo_run(page)
    rows = servo_rows(80)
    page.accept(v3_frame(3, rows[:40], flags=FLAG_FIRST_BATCH | FLAG_SERVO, base_us=1000))
    page.accept(v3_frame(3, rows[40:], flags=FLAG_LAST_BATCH | FLAG_SERVO, base_us=161000))
    page.handle_line("SYSID end run=3 state=done reason=complete dropped=0")
    assert wait_for(page, lambda: page.workflow.job is None and "分析失败" in page.fit_var.get())
    times, samples = read_samples(folder)
    assert len(samples) == 80 and times[1] == pytest.approx(0.004)
    assert {"servo_tilt", "erpm", "erpm_lower"} <= set(samples[0])
    assert samples[0]["servo_tilt"] == pytest.approx(-0.087)


def test_the_waveform_shows_real_erpm_when_present(page):
    if getattr(page, "figure", None) is None:
        pytest.skip("matplotlib 不可用")
    from test_sysid_page import drive_config
    drive_config(page, schema=SCHEMA_V3_LINES)
    page.handle_line("SYSID start run=3 profile=1 amp_mrad_s=65 dur_ms=8000 rate_hz=250 "
                     "I=20000 ugm2 psi_mrad=785 auto=1 target_cn=740")
    rows = [(100, 100, 100, 0, 0, 10, 10500, 9500, 0) for _ in range(20)]
    page.accept(v3_frame(3, rows, flags=FLAG_FIRST_BATCH, base_us=1000))
    text = page.erpm_var.get()
    assert "上桨 42000 eRPM（电频率 700.0 Hz，按 7 对极机械转频 100.0 Hz）" in text
    assert "下桨 38000 eRPM" in text
    # 2026-09-27：不再画转速副轴（量级不同、阶梯虚线糊满、副轴不跟主题），只留一行字。
    assert not hasattr(page, "erpm_axis")
    assert len(page.figure.axes) == 1
    page.clear_samples()
    page._draw(page.excitation())
    assert page.erpm_var.get() == ""


def test_the_measured_rate_is_drawn_without_prop_vibration(page):
    """主线画 15 Hz 以下：108 Hz 桨振动被中心滑动平均压掉，纵轴不被振动尖峰撑开。"""
    from panel_lib.pages.sysid.waveform import _moving_average
    fs = 250.0
    slow = [0.05 * math.sin(2 * math.pi * 0.5 * i / fs) for i in range(1000)]
    vib = [0.08 * math.sin(2 * math.pi * 108.0 * i / fs) for i in range(1000)]
    smooth = _moving_average([a + b for a, b in zip(slow, vib)], 9)
    resid = [s - a for s, a in zip(smooth[20:-20], slow[20:-20])]
    assert max(abs(r) for r in resid) < 0.012, "108 Hz 振动应被压到约一成以下"


def test_a_twd_fit_is_cross_checked_against_the_latest_servo_run(tmp_path):
    """带桨拟合只拿舵机单独轮作核对（不钉死）：J 按本次回中刚度折算，换成本轮固件单位的 ρ。"""
    sys.path.insert(0, str(ROOT / "tools"))
    try:
        from panel_lib.pages.sysid import servo_results
    finally:
        sys.path.pop(0)

    class TwdResult:
        structure = "twd"
        reaction_couple_s2 = 0.03
        rig_stiffness_n_m_rad = float("nan")
        pivot_above_cg_m = 0.05
        torque_model_scale = 0.5
        torque_scale = 1.2

    folder, conditions = write_run(tmp_path, "rod_030000_twd", mode="0", echo=TONIGHT)
    assert servo_results.servo_cross_check(TwdResult(), conditions, folder, azimuth_rad=math.pi / 2,
                                           root=tmp_path) == ""
    write_run(tmp_path, "rod_020000_s1", mode="3", echo=TONIGHT, fit_name="fit_servo.json",
              fit_data={"fit": {"amplitude_rad": 0.087, "reaction_inertia_kg_m2": -0.008,
                                "stiffness_n_m_rad": 0.37, "dead_time_s": 0.03,
                                "equivalent_delay_1hz_s": 0.05, "servo_wn_rad_s": 40.0,
                                "servo_zeta": 0.6, "pod_gravity_n_m_rad": 0.1}})
    text = servo_results.servo_cross_check(TwdResult(), conditions, folder, azimuth_rad=math.pi / 2,
                                           root=tmp_path)
    stiffness = 0.7546 * 9.81 * 0.05
    predicted = 0.008 * (stiffness / 0.37) / (0.035442 * 7.4)
    assert f"{predicted:.4g} s²" in text and "本次拟合 ρ = 0.03 s²" in text
    assert "rod_020000_s1" in text and "只作核对，没有钉进拟合" in text
    # 带桨轮里 H·s = H/(|L|·T)·u 被并进 G：报它占 G（κ·尺度 = 0.6）的比例。
    share = 0.1 * (stiffness / 0.37) / (0.035442 * 7.4)
    assert f"H/(|L|·T) = {share:+.3f}" in text and f"约 {100 * share / 0.6:+.0f}% 来自 H" in text


def test_servo_mode_needs_the_v3_record(page):
    """舵机单独要记录里的 servo_tilt：固件只报 v2 字段表时不开跑。"""
    from test_sysid_page import SCHEMA_V2_LINES
    page.mode_var.set("SERVO")
    page.rod_to_fc_var.set("0.04")
    page.handle_line(thr_line(armed=0))
    page.start_run()
    feed(page, SCHEMA_V2_LINES)
    feed(page, [f"PARAM name={name} value={value}" for name, value in GEOMETRIC_START])
    for _ in range(12):
        if not page.workflow.awaiting or page.workflow.awaiting == "SYSID START":
            break
        feed(page, servo_report(page.workflow.expected))
    assert "SYSID START" not in page.panel.transport.lines
    assert "SYSID 记录 v3" in page.banner_detail_var.get()

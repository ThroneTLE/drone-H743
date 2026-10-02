"""吊绳偏航辨识（YAW）分析：用已知 b、d、s、τ 生成合成记录，核对拟合误差在合理范围。

契约 `doc/sysid-yaw-contract.md`：记录尾字段沿用 ALT（height=ψ、vz=ω、vz_sp=r、az=ΔT），
`torque`=偏航力矩指令 M、`thrust`=总推力 F；conditions["yaw"] 是 `SYSID YAWSTART` 的字段。
"""
from __future__ import annotations

import json
import math
import random
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from sysid import yaw_analysis  # noqa: E402

K = 0.005                 # 模型 k [N·m/N]
IZZ_MODEL = 0.0125        # 模型 Izz [kg·m²]
T_MAX = 9.0               # 单桨可用最大推力 [N]
THRUST = 5.6              # 总推力 F [N]
HOVER = 11.2


def yaw_conditions(inject="diff", **changes):
    yaw = {"run": "7", "yaw_inject": inject, "yaw_thrust_mn": "5600",
           "yaw_k_um_per_n": "5000", "yaw_izz_ugm2": str(int(IZZ_MODEL * 1e6)),
           "yaw_single_max_mn": "9000", "yaw_twist_deg": "720"}
    yaw.update(changes)
    return {"mode": "6", "yaw": yaw, "yaw_request": {"inject": inject, "control":
            "openloop" if inject == "diff" else "closed_loop"},
            "parameter_echo": {"coax.hover_thrust_n": f"{HOVER}"}}


def doublet(t, amp, start=1.0, hold=1.2, repeat=3, ramp=0.1):
    """doublet：start 后 +amp 保持 hold、−amp 保持 hold，重复 repeat 对，过渡斜坡 ramp。"""
    if t < start or t >= start + 2 * hold * repeat:
        return 0.0
    u = (t - start) % (2 * hold)
    sign_pos = u < hold
    local = u if sign_pos else u - hold
    level = min(1.0, local / ramp)
    return amp * level * (1.0 if sign_pos else -1.0)


def diff_run(*, b=60.0, d=0.8, s=1.5, tau=0.045, c=0.0, amp_dt=1.5, dt=0.004, seconds=11.0,
             noise=0.0, seed=1, gyro_bias=0.0):
    """开环 diff：ΔT 给 M = k·ΔT，ω̇ = b·M(t−τ) − d·ω − s·ψ + c；返回 (时间 s, 样本)。"""
    rng = random.Random(seed)
    count = int(seconds / dt)
    times = [i * dt for i in range(count)]
    cmd = [K * doublet(t, amp_dt) for t in times]          # M 指令
    lag = int(round(tau / dt))
    omega = psi = 0.0
    rows = []
    for i, t in enumerate(times):
        delayed = cmd[max(0, i - lag)]
        alpha = b * delayed - d * omega - s * psi + c
        omega += alpha * dt
        psi += omega * dt
        rows.append(dict(height=psi, height_raw=0.0, height_sp=0.0,
                         vz=omega + gyro_bias + rng.gauss(0.0, noise), vz_sp=0.0,
                         az=cmd[i] / K, torque=cmd[i], thrust=THRUST, vbat=12.3,
                         erpm=3000.0, erpm_lower=3100.0))
    return times, rows


def rate_run(*, tau=0.08, amp=0.8, dt=0.004, seconds=9.0, noise=0.0, limit=None, seed=2):
    """闭环 rate：ω 一阶跟随 r（时间常数 tau），M 随跟踪误差变化（可设饱和线 limit）。"""
    rng = random.Random(seed)
    times = [i * dt for i in range(int(seconds / dt))]
    omega, rows = 0.0, []
    for t in times:
        ref = doublet(t, amp, start=1.0, hold=1.5, repeat=2, ramp=0.15)
        err = ref - omega
        moment = 0.01 * err
        if limit is not None:
            moment = max(-limit * 1.3, min(limit * 1.3, moment * 60.0))
        omega += err / tau * dt
        rows.append(dict(height=0.0, height_raw=0.0, height_sp=0.0,
                         vz=omega + rng.gauss(0.0, noise), vz_sp=ref, az=moment / K, torque=moment,
                         thrust=THRUST, vbat=12.3))
    return times, rows


# ---------------------------------------------------------------- diff


def test_diff_fit_recovers_known_plant():
    times, rows = diff_run()
    result = yaw_analysis.analyse_conditions(times, rows, yaw_conditions())
    assert result["inject"] == "diff"
    assert result["b"] == pytest.approx(60.0, rel=0.05)
    assert result["tau_s"] == pytest.approx(0.045, abs=0.011)
    assert result["d"] == pytest.approx(0.8, abs=0.15)
    assert result["s"] == pytest.approx(1.5, abs=0.4)
    assert result["r2"] > 0.97
    assert not result["warnings"]


def test_diff_fit_with_noise_and_gyro_bias():
    times, rows = diff_run(noise=0.02, gyro_bias=0.03, c=0.0)
    result = yaw_analysis.analyse_conditions(times, rows, yaw_conditions())
    assert result["b"] == pytest.approx(60.0, rel=0.10)
    assert result["tau_s"] == pytest.approx(0.045, abs=0.03)
    assert result["r2"] > 0.8


def test_k_ratio_is_b_times_model_inertia():
    """b·Izz_模型 = k 实测 / k 模型：实际力矩只有指令的 0.4 倍（Izz 与模型一致）时倍数应为 0.4。"""
    times, rows = diff_run(b=1.0 / 0.0125 * 0.4)          # 实际力矩只有指令的 0.4 倍（k 实测 = 0.4 k 模型）
    result = yaw_analysis.analyse_conditions(times, rows, yaw_conditions())
    assert result["k_ratio"] == pytest.approx(result["b"] * IZZ_MODEL)
    assert result["k_ratio"] == pytest.approx(0.4, rel=0.06)
    assert result["k_effective"] == pytest.approx(0.4 * K, rel=0.06)
    assert result["izz_effective_kg_m2"] == pytest.approx(1.0 / result["b"])


def test_recommended_gains_follow_contract_formula():
    times, rows = diff_run(b=60.0, tau=0.045)
    result = yaw_analysis.analyse_conditions(times, rows, yaw_conditions())
    b, tau = result["b"], result["tau_s"]
    wc = min(1.0 / (4.0 * tau), yaw_analysis.WC_MAX_RAD_S) if tau > 0 else yaw_analysis.WC_MAX_RAD_S
    assert result["wc_rad_s"] == pytest.approx(wc)
    assert result["rate_yaw_kp"] == pytest.approx(wc / b)
    assert result["rate_yaw_ki"] == pytest.approx(wc / b * wc / 5.0)


def test_zero_delay_caps_crossover():
    times, rows = diff_run(tau=0.0)
    result = yaw_analysis.analyse_conditions(times, rows, yaw_conditions())
    assert result["tau_s"] <= 0.011
    assert result["wc_rad_s"] <= yaw_analysis.WC_MAX_RAD_S + 1e-9


def test_hover_max_moment_and_acceleration():
    times, rows = diff_run()
    result = yaw_analysis.analyse_conditions(times, rows, yaw_conditions())
    # 悬停 11.2 N：min(F, 2·9 − F) = 6.8 N → 0.005 × 6.8 = 0.034 N·m
    assert result["moment_max_hover"] == pytest.approx(K * min(HOVER, 2 * T_MAX - HOVER))
    assert result["alpha_max_hover"] == pytest.approx(result["b"] * result["moment_max_hover"])


def test_negative_b_warns_and_gives_no_gains():
    times, rows = diff_run(b=-40.0, s=0.0)
    result = yaw_analysis.analyse_conditions(times, rows, yaw_conditions())
    assert result["b"] < 0
    assert "rate_yaw_kp" not in result
    assert any("b ≤ 0" in w for w in result["warnings"])


def test_no_excitation_is_refused():
    times, rows = diff_run(amp_dt=0.0)
    with pytest.raises(ValueError, match="没有激励|角加速度"):
        yaw_analysis.analyse_conditions(times, rows, yaw_conditions())


def test_saturation_fraction_counts_samples_at_the_line():
    times, rows = diff_run(amp_dt=1.5)
    # 把饱和线压到指令幅值以下：k·min(F, 2·Tmax − F) 的 Tmax 取小值
    conditions = yaw_conditions(yaw_single_max_mn="3000")        # 线 = 0.005×min(5.6, 0.4)=0.002 N·m
    result = yaw_analysis.analyse_conditions(times, rows, conditions)
    assert result["saturation_fraction"] > 0.9
    assert any("饱和" in w for w in result["warnings"])
    clean = yaw_analysis.analyse_conditions(times, rows, yaw_conditions())
    assert clean["saturation_fraction"] == 0.0


def test_missing_provenance_still_fits_but_has_no_ratio():
    times, rows = diff_run()
    conditions = yaw_conditions()
    conditions["yaw"] = {"yaw_inject": "diff"}
    conditions["parameter_echo"] = {}
    result = yaw_analysis.analyse_conditions(times, rows, conditions)
    assert result["b"] == pytest.approx(60.0, rel=0.05)
    assert "k_ratio" not in result and "moment_max_hover" not in result
    assert "没有 YAWSTART" in yaw_analysis.summary_text(result)


def test_ramp_down_after_excitation_is_cut():
    """激励结束后推力回落：回落段（推力 < 97%）不进拟合窗。"""
    times, rows = diff_run(seconds=13.0)
    cut = next(i for i, t in enumerate(times) if t > 10.0)
    for i in range(cut, len(rows)):
        rows[i]["thrust"] = THRUST * max(0.2, 1.0 - (times[i] - 10.0))
    result = yaw_analysis.analyse_conditions(times, rows, yaw_conditions())
    assert result["b"] == pytest.approx(60.0, rel=0.06)


def test_psi_comes_from_the_integrated_gyro_not_from_the_saturating_height_field():
    """记录里的 height（1e-4 rad、int16）会在 ±3.2767 rad 饱和：分析的 ψ 一律由陀螺 z 积分。"""
    times, rows = diff_run()
    clean = yaw_analysis.analyse_conditions(times, rows, yaw_conditions())
    for row in rows:
        row["height"] = 3.2767                       # 饱和成常数
    saturated = yaw_analysis.analyse_conditions(times, rows, yaw_conditions())
    assert saturated["s"] == pytest.approx(clean["s"]) and saturated["b"] == pytest.approx(clean["b"])
    truth = max(abs(v) for v in yaw_analysis.integrate_psi(times, [r["vz"] for r in rows]))
    assert saturated["psi_peak_rad"] == pytest.approx(truth) and truth < 3.0
    assert yaw_analysis.rename_samples(rows, times)[-1]["psi"] == pytest.approx(
        yaw_analysis.integrate_psi(times, [r["vz"] for r in rows])[-1])


def test_integrate_psi_is_a_trapezoid_from_zero():
    psi = yaw_analysis.integrate_psi([0.0, 1.0, 2.0, 3.0], [0.0, 2.0, 2.0, 0.0])
    assert psi == pytest.approx([0.0, 1.0, 3.0, 4.0])


def test_moment_is_rebuilt_from_delta_t_times_k_not_from_the_coarse_torque_field():
    """torque 字段分辨率 1e-4 N·m 太粗：M = k·az（az 分辨率 1e-3 N）；torque 清零也不影响拟合。"""
    times, rows = diff_run()
    for row in rows:
        row["torque"] = 0.0
    result = yaw_analysis.analyse_conditions(times, rows, yaw_conditions())
    assert result["b"] == pytest.approx(60.0, rel=0.05)
    assert result["m_source"] == "k·ΔT"
    assert result["moment_peak"] == pytest.approx(K * 1.5, rel=1e-6)


def test_without_k_the_torque_field_stands_in_and_the_result_says_so():
    times, rows = diff_run()
    conditions = yaw_conditions()
    conditions["yaw"] = {"yaw_inject": "diff"}
    result = yaw_analysis.analyse_conditions(times, rows, conditions)
    assert result["m_source"] == "torque 字段" and result["b"] == pytest.approx(60.0, rel=0.05)
    assert any("没有 YAWSTART 的 k" in w for w in result["warnings"])


def test_saturation_counts_a_command_beyond_the_line_even_if_the_applied_difference_is_clamped():
    """az 已是钳位后实际下发的差速；torque 是饱和前的指令：指令超线就算饱和。"""
    times, rows = rate_run()
    for row in rows:
        row["az"] = max(-0.2, min(0.2, row["az"]))
        row["torque"] = 0.0 if row["torque"] == 0.0 else 0.0045 * (1 if row["torque"] > 0 else -1) * 10
    conditions = yaw_conditions("rate", yaw_single_max_mn="3000")       # 线 0.002 N·m
    result = yaw_analysis.analyse_conditions(times, rows, conditions)
    assert result["saturation_fraction"] > 0.2


# ---------------------------------------------------------------- rate


def test_rate_steps_report_rise_overshoot_and_error():
    times, rows = rate_run(tau=0.08)
    result = yaw_analysis.analyse_conditions(times, rows, yaw_conditions("rate"))
    assert result["inject"] == "rate"
    assert result["followed"] >= 2
    # 一阶 τ=80 ms：90% 上升时间 ≈ 2.3τ = 184 ms（+ 自参考开始变化的斜坡）
    assert 0.15 < result["rise90_s"] < 0.45
    assert result["overshoot"] < 0.05
    assert result["final_error"] < 0.02
    assert result["saturation_fraction"] == 0.0


def test_rate_saturation_fraction_reported():
    times, rows = rate_run(limit=0.003)
    conditions = yaw_conditions("rate", yaw_single_max_mn="3000")          # 线 0.002 N·m
    result = yaw_analysis.analyse_conditions(times, rows, conditions)
    assert result["saturation_fraction"] > 0.2
    assert any("饱和" in w for w in result["warnings"])


def test_rate_without_reference_is_refused():
    times, rows = rate_run(amp=0.0)
    with pytest.raises(ValueError):
        yaw_analysis.analyse_conditions(times, rows, yaw_conditions("rate"))


# ---------------------------------------------------------------- 文字与入口


def test_summary_texts_are_chinese_and_have_the_key_numbers():
    times, rows = diff_run()
    text = yaw_analysis.summary_text(yaw_analysis.analyse_conditions(times, rows, yaw_conditions()))
    for needle in ("b = ", "k 实测 / k 模型", "rate_yaw_kp", "rate_yaw_ki", "最大可用偏航力矩", "最大角加速度"):
        assert needle in text
    times, rows = rate_run()
    text = yaw_analysis.summary_text(yaw_analysis.analyse_conditions(times, rows, yaw_conditions("rate")))
    assert "上升时间" in text and "超调" in text and "饱和时间占比" in text
    assert "rate_yaw_ff" in text and "没有参考模型前馈" in text


def test_unknown_inject_and_missing_fields_are_refused():
    times, rows = diff_run()
    with pytest.raises(ValueError, match="注入类型不明"):
        yaw_analysis.analyse_conditions(times, rows, {"mode": "6"})
    stripped = [{k: v for k, v in row.items() if k != "az"} for row in rows]
    with pytest.raises(ValueError, match="缺少 YAW 字段"):
        yaw_analysis.analyse_conditions(times, stripped, yaw_conditions())


def test_command_line_reads_an_archive_directory(tmp_path, capsys):
    times, rows = diff_run()
    folder = tmp_path / "yaw_120000_deadbeef"
    folder.mkdir()
    names = list(rows[0])
    lines = ["t_us,run_id,gap," + ",".join(names)]
    for t, row in zip(times, rows):
        lines.append(f"{int(t * 1e6)},7,0," + ",".join(repr(row[n]) for n in names))
    (folder / "samples.csv").write_text("\n".join(lines), encoding="utf-8")
    (folder / "conditions.json").write_text(json.dumps(yaw_conditions(), ensure_ascii=False),
                                            encoding="utf-8")
    assert yaw_analysis.main([str(folder)]) == 0
    out = capsys.readouterr().out
    assert "yaw_120000_deadbeef" in out and "b = " in out

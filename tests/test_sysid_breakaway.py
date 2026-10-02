"""槽式台架离地/滑落阈值分析（`tools/sysid/breakaway.py`，ALT inject=break）。

数据是合成的：一个带静/动摩擦的竖直槽 + 按跨侧契约（2026-09-30）照做的固件序列
（预升 → 慢升找离地 → 制停 → 慢降找滑落 → 滑回槽底 → 降到怠速），测距 8 样本平均做粗判，
记录里的高度叠 5 mm 噪声。它只验证算法能从这种记录里找回设定的阈值，不代表真机参数。
"""
from __future__ import annotations

import csv
import json
import math
import random

import pytest

from tools.sysid.breakaway import (analyse, analyse_conditions, locate_phases, main,
                                   summary_text)

G = 9.80665
DT = 0.004                    # 250 Hz 记录
MASS_KG = 1.184
RATE_N_S = 0.5
BOTTOM_M = 0.460


def simulate(*, seed=1, noise_m=0.005, scale=0.94, fs_n=0.8, kinetic_ratio=0.5):
    """返回 (时间, 高度, 推力指令, PHASE 回报 dict 列表, 真实阶段起点 {名: t}, 真值 {F_up, F_down})。"""
    rng = random.Random(seed)
    weight = MASS_KG * G
    fk = kinetic_ratio * fs_n
    f_start = 0.9 * weight
    z = v = 0.0
    stuck = True
    tof: list[float] = []
    times, heights, thrusts, entries, starts = [], [], [], [], {}
    phase, t_phase, command = "ramp_up", 0.0, 2.0
    h_base = h_stuck = f_hold = f_slide = None
    last_cut = last_check_h = last_check_t = None
    history: list[tuple[float, float]] = []

    def enter(name, t, thrust):
        nonlocal phase, t_phase
        phase, t_phase = name, t
        starts.setdefault(name, t)
        entries.append({"run": "3", "phase": name, "pulse_us": "1450",
                        "thrust_cn": str(int(round(thrust * 100)))})

    enter("ramp_up", 0.0, 0.0)
    step = 0
    while step < 20000:
        t = step * DT
        # 测距约 83 Hz（每 3 拍一个新样本），固件粗判用最近 8 个的平均。
        if step % 3 == 0:
            tof.append(BOTTOM_M + z + rng.gauss(0.0, noise_m))
        h_avg = sum(tof[-8:]) / len(tof[-8:])
        history.append((t, h_avg))
        since = t - t_phase
        if phase == "ramp_up":
            command = 2.0 + (f_start - 2.0) * min(since / 1.5, 1.0)
            if since >= 1.5:
                h_base = h_avg
                enter("climb", t, command)
        elif phase == "climb":
            command = f_start + RATE_N_S * since
            if h_avg >= h_base + 0.012:
                f_det = command
                enter("settle", t, command)
                last_cut, last_check_h, last_check_t = t, h_avg, t
                command = f_det - 0.4
        elif phase == "settle":
            if t - last_check_t >= 0.25:
                if h_avg - last_check_h > 0.006:
                    command -= 0.4
                    last_cut = t
                    quiet = False
                else:
                    quiet = abs(h_avg - last_check_h) <= 0.006
                last_check_h, last_check_t = h_avg, t
                if quiet and t - last_cut >= 0.8:
                    if h_avg <= h_base + 0.010:
                        enter("ramp_down", t, command)
                        ramp_from = command
                    else:
                        f_hold, h_stuck = command, h_avg
                        enter("excite", t, command)
        elif phase == "excite":
            command = f_hold - RATE_N_S * since
            if h_avg <= h_stuck - 0.012:
                f_slide = command
                enter("descend", t, command)
        elif phase == "descend":
            command = f_slide
            back = [h for tt, h in history if tt >= t - 0.2]
            if h_avg <= h_base + 0.010 and max(back) - min(back) <= 0.004 and since > 0.2:
                ramp_from = command
                enter("ramp_down", t, command)
        elif phase == "ramp_down":
            command = ramp_from + (2.0 - ramp_from) * min(since / 1.0, 1.0)
            if since >= 1.0:
                break
        # 物理：真实推力 = 表值 × scale；静摩擦 fs，动摩擦 fk；槽底托住。
        force = scale * command - weight
        if stuck:
            if force > fs_n or (z > 0.0 and force < -fs_n):
                stuck = False
        if not stuck:
            direction = 1.0 if (v > 0.0 or (v == 0.0 and force > 0.0)) else -1.0
            a = (force - direction * fk) / MASS_KG
            v_new = v + a * DT
            if v != 0.0 and (v_new > 0.0) != (v > 0.0) and abs(force) <= fs_n:
                v_new, stuck = 0.0, True
            v = v_new
            z += v * DT
            if z <= 0.0:
                z, v, stuck = 0.0, 0.0, True
        times.append(t)
        heights.append(round((BOTTOM_M + z + rng.gauss(0.0, noise_m)) * 1e4) / 1e4)
        thrusts.append(round(command * 100) / 100)
        step += 1
    truth = dict(f_up=(weight + fs_n) / scale, f_down=(weight - fs_n) / scale)
    return times, heights, thrusts, entries, starts, truth


@pytest.mark.parametrize("seed", [1, 2, 3])
def test_a_full_run_recovers_both_thresholds(seed):
    times, heights, thrusts, entries, starts, truth = simulate(seed=seed)
    assert [e["phase"] for e in entries] == ["ramp_up", "climb", "settle", "excite", "descend",
                                             "ramp_down"]
    result = analyse(times, heights, thrusts, entries, MASS_KG, vbat=[12.1] * len(times))
    assert result["ok"]
    assert result["f_up_n"] == pytest.approx(truth["f_up"], abs=0.15)
    assert result["f_down_n"] == pytest.approx(truth["f_down"], abs=0.15)
    assert result["mg_lut_n"] == pytest.approx((truth["f_up"] + truth["f_down"]) / 2, abs=0.1)
    assert result["scale"] == pytest.approx(0.94, abs=0.01)
    assert result["fs_n"] == pytest.approx(0.8 / 0.94, abs=0.15)
    # 粗判（8 样本平均升 12 mm）总在找回的起点之后。
    assert result["f_up_uncertainty_n"] >= 0.0 and result["f_down_uncertainty_n"] >= 0.0
    assert result["vbat_v"] == pytest.approx(12.1)
    text = summary_text(result)
    assert "离地推力 F_up ≈" in text and "滑落推力 F_down ≈" in text
    assert "悬停对应表值" in text and "静摩擦" in text and "推力表比例 0.9" in text
    assert "电池 12.1 V" in text


def test_phase_starts_are_found_from_the_thrust_series():
    times, _heights, thrusts, entries, starts, _truth = simulate(seed=4)
    located = {name: t for t, name, _f in locate_phases(times, thrusts, entries)}
    assert set(located) == set(starts)
    for name, t_true in starts.items():
        assert located[name] == pytest.approx(t_true, abs=0.08), name


def test_explicit_phase_times_give_the_same_answer():
    times, heights, thrusts, entries, starts, _truth = simulate(seed=5)
    derived = analyse(times, heights, thrusts, entries, MASS_KG)
    explicit = analyse(times, heights, thrusts,
                       [(starts[e["phase"]], e["phase"], int(e["thrust_cn"]) / 100) for e in entries],
                       MASS_KG)
    assert explicit["f_up_n"] == pytest.approx(derived["f_up_n"], abs=0.03)
    assert explicit["f_down_n"] == pytest.approx(derived["f_down_n"], abs=0.03)


def test_without_a_down_threshold_only_the_upper_bound_is_reported():
    """静摩擦小：制停那 0.4 N 已经低于滑落推力，机体直接滑回槽底 → 没有 excite。"""
    times, heights, thrusts, entries, _starts, truth = simulate(seed=6, fs_n=0.08)
    assert "excite" not in [e["phase"] for e in entries]
    result = analyse(times, heights, thrusts, entries, MASS_KG)
    assert result["ok"] and result["f_down_n"] is None and result["mg_lut_n"] is None
    # 摩擦几乎没有跳变时离地后升得慢，找回的起点偏晚；这里只要求量级对、上界照报。
    assert result["f_up_n"] == pytest.approx(truth["f_up"], abs=0.25)
    assert result["mg_lut_upper_n"] == result["f_up_n"]
    assert result["scale_lower"] == pytest.approx(MASS_KG * G / result["f_up_n"])
    text = summary_text(result)
    assert "没有滑落阈值" in text and "回到槽底" in text


def test_no_settle_means_no_liftoff_and_no_numbers():
    times, heights, thrusts, entries, _starts, _truth = simulate(seed=7)
    climb_only = [e for e in entries if e["phase"] in ("ramp_up", "climb")]
    result = analyse(times, heights, thrusts, climb_only, MASS_KG)
    assert not result["ok"] and result["f_up_n"] is None
    assert "未离地" in summary_text(result)


def test_bad_inputs_are_refused_in_chinese():
    with pytest.raises(ValueError, match="样本数不一致"):
        analyse([0.0] * 30, [0.0] * 29, [0.0] * 30, [], MASS_KG)
    with pytest.raises(ValueError, match="质量"):
        analyse([i * DT for i in range(30)], [0.0] * 30, [0.0] * 30, [], 0.0)
    with pytest.raises(ValueError, match="移动质量"):
        analyse_conditions([0.0], [{"height": 0.4, "thrust": 1.0}], {})


def _write_run(folder, seed, *, t0_us=4_294_000_000):
    """照 workflow.archive 的格式写一轮存档（时间戳故意跨 32 位回绕）。"""
    times, heights, thrusts, entries, _starts, _truth = simulate(seed=seed)
    folder.mkdir()
    with (folder / "samples.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["t_us", "run_id", "gap", "thrust", "height", "vbat"])
        for t, h, f in zip(times, heights, thrusts):
            writer.writerow([(t0_us + int(round(t * 1e6))) & 0xFFFFFFFF, 3, 0, f, h, 12.2])
    conditions = {"mode": "4", "alt": {"run": "3", "alt_mass_g": "1184", "alt_control": "breakaway"},
                  "alt_request": {"inject": "break", "mass_g": 1184, "control": "breakaway"},
                  "alt_phases": entries, "end": {"state": "done", "reason": "complete"},
                  "data_error": ""}
    (folder / "conditions.json").write_text(json.dumps(conditions, ensure_ascii=False),
                                            encoding="utf-8")


def test_cli_summarises_one_run_and_the_spread_of_several(tmp_path, capsys):
    _write_run(tmp_path / "alt_1", 1)
    _write_run(tmp_path / "alt_2", 2)
    assert main([str(tmp_path / "alt_1")]) == 0
    single = capsys.readouterr().out
    assert "alt_1：离地推力 F_up ≈" in single and "推力表比例" in single
    assert "多轮汇总" not in single
    assert main([str(tmp_path / "alt_1"), str(tmp_path / "alt_2"), str(tmp_path / "missing")]) == 0
    several = capsys.readouterr().out
    assert "missing：读不出来" in several
    assert "多轮汇总（2 轮算出了离地推力）" in several
    assert "推力表比例：均值 0.9" in several and "标准差" in several


def test_cli_returns_two_when_nothing_could_be_analysed(tmp_path, capsys):
    assert main([str(tmp_path / "nothing_here")]) == 2
    assert "读不出来" in capsys.readouterr().out


def test_smoothing_is_centred_and_ignores_missing_heights():
    from tools.sysid.breakaway import smooth
    times = [float(i) for i in range(11)]
    values = [0.0] * 5 + [1.0] + [0.0] * 5
    out = smooth(times, values, width_s=2.0)
    assert out[5] == pytest.approx(1 / 3) and out[4] == pytest.approx(1 / 3) and out[2] == 0.0
    assert smooth(times, [math.nan] + [0.0] * 10, width_s=2.0)[0] == 0.0

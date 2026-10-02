"""水平槽 XY 辨识端到端彩排：固件 -> 记录批 -> 地面站解码 -> 改名 -> 分析（R-XYID-1）。

固件 XY 模块（App/Src/app_sysid_xy.c）与地面站（tools/sysid/xy_analysis.py）是按契约
doc/sysid-xy-contract.md 分头写的；这里不再各测各的，而是把它们串起来跑一整轮：

* 固件侧：复用 test_sysid_xy.py 的主机编译装置（真固件代码 + 真分配器 + 真位置/速度环），
  `FillSample` 真填的记录样本经固件自己的批编码吐出二进制帧；
* 对象：水平槽里的一维质点，贴近实物 —— 质量 1.1453 kg + 页面默认随动质量 38.8 g，
  静摩擦等效加速度 0.3 m/s²、动摩擦 0.2 m/s²，光流延迟 40 ms + 白噪声，
  姿态环一阶滞后（时间常数 0.1 s）：实际倾角跟随固件给出的目标倾角而不是瞬时到位；
* 激励：地面站页面默认设置（`xy_config.INJECT_PRESETS`、`DEFAULT_WIN_MM`、`DEFAULT_EXTRA_MASS_G`），
  不自己编；采样率取内环页默认 250 Hz；
* 地面站侧：`sysid.decode.decode_batch`（按固件报的字段表解帧）-> 内环页 `_timestamps`（同一段代码）
  -> `_core.xy_rename_samples` -> `_core.xy_analysis`，conditions 用页面真正写的形状
  （mode / xy 溯源行 / xy_request）。

绕杆角口径（固件）：正绕杆角把推力偏向 -u，沿 +u 的指令加速度看 `az`；`angle_sp` 只含偏置。
"""
from __future__ import annotations

import ctypes
import math
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

from test_sysid_runtime_contract import Excitation, PROFILE_DOUBLET
from test_sysid_xy import (  # noqa: F401  夹具与装置
    FLAG_XY, MODE_XY, XYPlant, coax_params, fresh_lib, lib, prepare_xy, run_xy, u_of,
)
from test_sysid_alt import Capture, G, schema_of, teardown_run
from test_sysid_runtime_contract import texts

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from panel_lib.pages.sysid import _core as page_core  # noqa: E402
from panel_lib.pages.sysid import xy_config  # noqa: E402
from panel_lib.pages.sysid.inner_loop import SysIdInnerLoopPage  # noqa: E402
from sysid.decode import decode_batch, parse_schema_lines  # noqa: E402

# ---------------------------------------------------------------- 仿真真值（实物参数）

AIRFRAME_KG = 1.1453
EXTRA_G = float(xy_config.DEFAULT_EXTRA_MASS_G)             # 页面默认随动附加质量 38.8 g
MASS_G = round(AIRFRAME_KG * 1000.0 + EXTRA_G)             # 页面下发给固件的 mass_g
WIN_MM = int(xy_config.DEFAULT_WIN_MM)                      # 150
HOLD_N = 11.3                                               # 托住推力 ≈ 1.1845 kg * g
RATE_HZ = 250                                               # 内环页默认线上采样率
FS_M_S2, FK_M_S2 = 0.3, 0.2                                 # 静/动摩擦等效加速度
LAG_S = 0.1                                                 # 姿态环一阶滞后时间常数
FLOW_DELAY_TICKS = 20                                       # 40 ms
FLOW_SIGMA_V, FLOW_SIGMA_P = 0.008, 0.002                   # 光流速度 / 位置白噪声 [m/s] [m]
FLOW_PERIOD_MS = 20                                         # 光流 50 Hz
BREAK_ANGLE_TRUE = math.atan(FS_M_S2 / G)                   # 静摩擦门槛角真值 ≈ 0.0306 rad
MAX_TICKS = 40_000                                          # 80 s 上限，防死循环


def page_spec(inject: str) -> Excitation:
    """页面默认激励 -> 固件的 Excitation（字段映射同页面：amp/hold/repeat/dur/ramp）。"""
    preset = xy_config.INJECT_PRESETS[inject]
    return dict(profile=PROFILE_DOUBLET, amplitude_rad_s=float(preset["amp"]),
                duration_ms=int(preset["dur"]), hold_ms=int(preset["hold"]),
                repeat=int(preset["repeat"]), ramp_ms=int(preset["ramp"]),
                chirp_f0_hz=0.3, chirp_f1_hz=6.0, prbs_bit_ms=40, prbs_seed=1)


def excite_seconds(inject: str) -> float:
    """页面默认激励的激励段时长：doublet = min(2·平台·对数, 总时长)。"""
    spec = page_spec(inject)
    return min(2 * spec["hold_ms"] * spec["repeat"], spec["duration_ms"]) / 1000.0


def preset_amp(inject: str) -> float:
    return page_spec(inject)["amplitude_rad_s"]


def kv_line(line: str) -> dict:
    return dict(token.split("=", 1) for token in line.split()[2:] if "=" in token)


def full_round(lib, inject: str, psi: float, *, fs=FS_M_S2, fk=FK_M_S2, lag_s=LAG_S,
               win_mm=WIN_MM, noise=True, seed=5, spec=None):
    """整轮：配置 -> 开跑 -> 跑到固件收尾 -> 地面站解码 -> 改名 -> 分析。返回一个记录本轮的 SimpleNamespace。"""
    plant = XYPlant(lib, psi, fs=fs, fk=fk, lag_s=lag_s, delay_ticks=FLOW_DELAY_TICKS,
                    pos_sigma=FLOW_SIGMA_P if noise else 0.0,
                    vel_sigma=FLOW_SIGMA_V if noise else 0.0, seed=seed)
    prepare_xy(lib, inject=inject, psi=psi, win_mm=win_mm, mass_g=MASS_G, target_n=HOLD_N,
               rate_hz=RATE_HZ, spec=spec or page_spec(inject), plant=plant)
    schema = schema_of(lib)
    lib.harness_reset()
    assert lib.APP_SysId_Start() == 1, texts(lib)
    start_texts = texts(lib)
    capture = Capture(lib)
    capture.texts.extend(start_texts)
    lib.harness_reset()
    phases = []
    stopped = run_xy(lib, plant, MAX_TICKS, record=phases, capture=capture)
    state, reason = lib.APP_SysId_GetState(), lib.APP_SysId_GetLastReason().decode()

    batches = [decode_batch(payload, schema) for _function, payload in capture.frames]
    samples = [dict(sample) for batch in batches for sample in batch.samples]
    # 地面站同一段时间轴代码：内环页 _timestamps（按批头 base + offset_us，首样本为 0）。
    times = SysIdInnerLoopPage._timestamps(SimpleNamespace(batches=batches))
    xystart = next(line for line in capture.texts if line.startswith("SYSID XYSTART "))
    conditions = {"mode": str(MODE_XY), "xy": kv_line(xystart),
                  "xy_request": {"inject": inject, "win_mm": win_mm, "mass_g": MASS_G,
                                 "control": xy_config.CONTROL_BY_INJECT[inject]}}
    rows = page_core.xy_rename_samples(samples)
    try:
        result = page_core.xy_analysis(times, samples, conditions)
        error = None
    except ValueError as exc:     # 让断言去报具体原因
        result, error = None, str(exc)
    run = SimpleNamespace(
        plant=plant, schema=schema, batches=batches, samples=samples, rows=rows, times=times,
        conditions=conditions, result=result, error=error, texts=capture.texts, phases=phases,
        stopped=stopped, state=state, reason=reason, psi=psi, inject=inject,
        spec=spec or page_spec(inject), frames=[payload for _f, payload in capture.frames],
        end_line=next((line for line in capture.texts if line.startswith("SYSID end ")), ""),
        flags=[batch.flags for batch in batches])
    teardown_run(lib)
    return run


def excite_rows(run):
    """EXCITE 段的样本：用 angle_sp / pos_sp / vel_sp 非零来认（tilt 看 az）。"""
    t0 = None
    out = []
    for t, row in zip(run.times, run.rows):
        active = abs(row["acc_u"]) > 1e-6 or abs(row["pos_sp_u"]) > 1e-6 or abs(row["vel_sp_u"]) > 1e-6
        if active and t0 is None:
            t0 = t
        if t0 is not None:
            out.append((t, row))
    return out


# ---------------------------------------------------------------- 整轮的共同判据


def assert_round_is_sane(run, expected_excite_s):
    assert run.state == 2 or run.reason == "complete", (run.state, run.reason, run.end_line)
    assert run.reason == "complete", f"整轮应走完收尾，实际 {run.reason}：{run.end_line}"
    assert "xy_window" not in run.end_line and "xy_window" not in " ".join(run.texts), \
        "默认激励不应触发出窗软停"
    assert run.flags[0] & 0x0001 and run.flags[-1] & 0x0002, "首批/末批标志"
    assert all(flags & FLAG_XY for flags in run.flags)
    assert not any(flags & 0x0010 for flags in run.flags), "全程不应有断点"
    duration = run.times[-1]
    # 整轮 = RAMP_UP 1.5 + SETTLE 2 + PREROLL 0.5 + EXCITE + RAMP_DOWN 1
    assert duration == pytest.approx(5.0 + expected_excite_s, abs=0.3)
    assert len(run.samples) == pytest.approx(duration * RATE_HZ, abs=RATE_HZ * 0.1)
    # 地面站按字段表解出的 XY 尾字段都在，改名后不再有 height。
    for name in page_core._xy.REQUIRED_RENAMED:
        assert name in run.rows[0]
    assert run.conditions["xy"]["xy_inject"] == run.inject
    assert int(run.conditions["xy"]["xy_mass_g"]) == MASS_G
    assert int(run.conditions["xy"]["xy_psi_mrad"]) == pytest.approx(run.psi * 1000.0, abs=1)
    max_abs_pos = max(abs(row["pos_u"]) for row in run.rows)
    assert max_abs_pos < WIN_MM * 0.001 * 0.8, f"默认激励走到 {max_abs_pos * 1000:.0f} mm，离 {WIN_MM} mm 出窗太近"
    return max_abs_pos


PSIS = [0.0, math.pi / 2, math.pi / 4, -math.pi / 4]
PSI_IDS = ["0deg", "90deg", "p45deg", "m45deg"]


# ---------------------------------------------------------------- tilt 轮

def rich_tilt_spec():
    """页面默认 tilt 激励已是 0.07 rad / 400 ms / 3 对（原「信息量更足」那一组）。"""
    return page_spec("tilt")


def assert_tilt_signs(run):
    """固件口径的符号：az 沿 +u 为正；angle_sp 与 az 反号（正绕杆角把推力偏向 -u）；首个半周期沿 +u 走。"""
    ex = excite_rows(run)
    assert ex, "没有激励段记录"
    amp = run.spec["amplitude_rad_s"]
    # 2026-10-01 起 tilt 在激励前用速度环按住机体，激励时以那一刻的环输出为开环基准，az = 基准 + g·tanθ；
    # 符号与峰值按扣掉基准后的注入量判。
    trim = ex[0][1]["acc_u"]
    az_peak = max(abs(row["acc_u"] - trim) for _t, row in ex)
    assert az_peak == pytest.approx(G * math.tan(amp), rel=0.03)
    for _t, row in ex:
        if abs(row["acc_u"] - trim) > 0.5 * az_peak:
            assert row["angle_sp"] * (row["acc_u"] - trim) < 0, "angle_sp 必须与 az 反号（正绕杆角偏向 -u）"
    t_ex = ex[0][0]
    first = [row for t, row in ex if t < t_ex + 0.25]
    assert sum(row["pos_u"] for row in first) > 0 and sum(row["acc_u"] - trim for row in first) > 0
    assert all(row["angle"] - run.rows[0]["angle"] <= 1e-3 for row in first), "正指令时绕杆角应为负"


@pytest.mark.parametrize("psi", PSIS, ids=PSI_IDS)
def test_tilt_default_excitation_round_identifies_the_plant_with_the_right_signs(lib, psi):
    """页面默认 tilt 激励（0.07 rad / 400 ms / 3 对）+ 光流速度噪声 8 mm/s，实物摩擦 0.3/0.2。"""
    run = full_round(lib, "tilt", psi)
    assert run.error is None, run.error
    assert_round_is_sane(run, expected_excite_s=excite_seconds("tilt"))
    assert_tilt_signs(run)
    r = run.result
    assert r["flow_lag_s"] is not None, r
    # 按 IMU 实测倾角算的增益（真值 1）、光流滞后（40 ms 延迟 + 50 Hz 保持约 10 ms）、动摩擦（真值 0.2）。
    assert r["gain_vs_measured_angle"] == pytest.approx(1.0, abs=0.25), r
    assert 0.02 <= r["flow_lag_s"] <= 0.10, r
    assert r["kinetic_friction_m_s2"] == pytest.approx(FK_M_S2, abs=0.15), r
    assert r["gain"] is not None and 0.4 < r["gain"] < 1.2, r        # 指令口径含姿态环滞后损失，<1
    assert BREAK_ANGLE_TRUE * 0.9 < r["break_angle_rad"] < BREAK_ANGLE_TRUE * 2.0, r   # 斜坡陡，只会偏大
    assert r["break_direction"] == 1, "第一个半周期是 +θ，应沿 +u 开始滑动"
    assert not r["warnings"], r["warnings"]


@pytest.mark.parametrize("psi", PSIS, ids=PSI_IDS)
def test_tilt_rich_excitation_round_identifies_the_plant_tightly(lib, psi):
    """幅值 0.07 rad、平台 400 ms 的激励：速度够大，信噪比够，辨识值贴近真值。"""
    run = full_round(lib, "tilt", psi, spec=rich_tilt_spec())
    assert run.error is None, run.error
    assert_round_is_sane(run, expected_excite_s=excite_seconds("tilt"))
    assert_tilt_signs(run)
    r = run.result
    assert r["gain_vs_measured_angle"] == pytest.approx(1.0, abs=0.08), r
    assert r["kinetic_friction_m_s2"] == pytest.approx(FK_M_S2, abs=0.05), r
    assert r["flow_lag_s"] == pytest.approx(0.05, abs=0.02), r
    assert 0.6 < r["gain"] < 1.0, r                # 指令口径：姿态环 0.1 s 滞后 + 光流延迟带来的损失
    # 门槛角：检测要等速度越过噪声门限，斜坡越陡越晚，只会偏大（保守），不会偏小。
    assert BREAK_ANGLE_TRUE * 0.9 < r["break_angle_rad"] < BREAK_ANGLE_TRUE * 2.0, r
    assert r["break_direction"] == 1


def test_tilt_rich_round_truth_does_not_depend_on_the_noise_seed(lib):
    for seed in (1, 9):
        r = full_round(lib, "tilt", math.pi / 4, seed=seed, spec=rich_tilt_spec()).result
        assert r["kinetic_friction_m_s2"] == pytest.approx(FK_M_S2, abs=0.05), (seed, r)
        assert r["gain_vs_measured_angle"] == pytest.approx(1.0, abs=0.08), (seed, r)


def test_tilt_with_bigger_friction_raises_the_break_angle_and_friction(lib):
    run = full_round(lib, "tilt", math.pi / 4, fs=0.45, fk=0.3, spec=rich_tilt_spec())
    assert run.reason == "complete" and run.error is None, (run.reason, run.error)
    r = run.result
    assert r["break_angle_rad"] > math.atan(0.45 / G), r            # 保守偏大，不会偏小
    assert r["kinetic_friction_m_s2"] == pytest.approx(0.3, abs=0.06), r
    assert r["gain_vs_measured_angle"] == pytest.approx(1.0, abs=0.08), r


def test_tilt_that_barely_overcomes_the_friction_says_so_instead_of_inventing_numbers(lib):
    """静 0.6 / 动 0.4 m/s² 时 0.07 rad（a=0.69）净加速太小，速度上不去：分析给警告、不给增益与摩擦。"""
    run = full_round(lib, "tilt", math.pi / 4, fs=0.6, fk=0.4, spec=rich_tilt_spec())
    assert run.reason == "complete" and run.error is None, (run.reason, run.error)
    r = run.result
    assert r["gain"] is None and r["kinetic_friction_m_s2"] is None and r["warnings"], r


# ---------------------------------------------------------------- vel / pos 轮


@pytest.fixture
def strong_gains(coax_params):
    """试用增益（pos 1.2 / vel 3.0 / ki 3.0，x 通道，仍是生产位置/速度环按参数表跑）。"""
    for name, value in TRIAL_GAINS.items():
        assert coax_params(name, value) == 1, name
    return coax_params


def assert_steps_are_meaningful(run, amp):
    r = run.result
    steps = r["steps"]
    # doublet 两对 -> 平台 + - + -，受控的跳变 4 次（0->+A、+A->-A、-A->+A、+A->-A）；
    # 激励结束后回到 0 是 RAMP_DOWN（环已停），分析必须切掉，不能算成第 5 次阶跃。
    assert len(steps) == 4, [round(s["t_start_s"], 2) for s in steps]
    excite_end = run.times[-1] - 1.0           # RAMP_DOWN 最后 1 s
    assert all(step["t_start_s"] < excite_end - 0.5 for step in steps), "有阶跃起点落在回落段"
    assert r["ref_peak"] == pytest.approx(amp, rel=0.03), "参考峰值应是注入幅值，不是速度环输入"
    signs = [1 if s["delta_ref"] > 0 else -1 for s in steps]
    assert signs == [1, -1, 1, -1], signs
    assert r["followed"] == 4, r
    for step in steps:
        assert step["delta_meas"] * step["delta_ref"] > 0, "测量方向与参考同号"
        assert 0.2 < step["rise90_s"] < 2.0, step
        assert 0.0 <= step["overshoot"] < 0.8, step
    assert not r["warnings"], r["warnings"]


@pytest.mark.parametrize("psi", PSIS, ids=PSI_IDS)
@pytest.mark.parametrize("inject", ["vel", "pos"])
def test_closed_loop_rounds_cut_meaningful_steps_with_friction(lib, strong_gains, psi, inject):
    excite_s, amp = excite_seconds(inject), preset_amp(inject)
    run = full_round(lib, inject, psi)
    assert run.error is None, run.error
    assert_round_is_sane(run, expected_excite_s=excite_s)
    assert_steps_are_meaningful(run, amp)


@pytest.mark.parametrize("inject", ["vel", "pos"])
def test_closed_loop_driver_default_gains_without_friction_follow_only_partly(lib, inject):
    """驱动出厂默认增益（pos 0.375 / vel 0.8 / ki 0）+ 无摩擦 + 页面默认激励：整轮不出事；
    vel 能跟上 4 次（超调大，约 60%），pos 只跟上 1 次（环太软）。"""
    run = full_round(lib, inject, math.pi / 4, fs=0.0, fk=0.0)
    assert run.reason == "complete" and run.error is None, (run.reason, run.error)
    r = run.result
    if inject == "vel":
        assert r["followed"] >= 3 and 0.3 < r["rise90_s"] < 2.0, r
    else:
        assert r["followed"] < len(r["steps"]) and r["warnings"], r


@pytest.mark.parametrize("inject", ["vel", "pos"])
def test_closed_loop_driver_default_gains_cannot_break_the_stiction(lib, inject):
    """驱动出厂默认增益 ki=0，静摩擦 0.3 m/s² 下页面默认幅值完全推不动机体（最大位移约 9 mm）。
    整轮安全收尾、分析如实报「没跟上」，而不是编出一组上升时间。"""
    run = full_round(lib, inject, math.pi / 4)
    assert run.reason == "complete" and run.error is None, (run.reason, run.error)
    r = run.result
    assert r["followed"] == 0 and r["rise90_s"] is None and r["warnings"], r
    assert max(abs(row["pos_u"]) for row in run.rows) < 0.02


# ---------------------------------------------------------------- 默认激励 vs 窗（含新的预测出窗门）

SOFT_STOPS = ("xy_window", "xy_overspeed", "xy_yaw_limit")


def coast_bound_m(run, win_mm=WIN_MM):
    """软停后允许的最大位移：窗 + 停时速度 x 1 s 线性降推力期间的滑行 + 一点噪声。"""
    return win_mm * 0.001 + max(abs(row["vel_u"]) for row in run.rows) * 1.0 + 0.02


@pytest.mark.parametrize("inject", ["tilt", "vel", "pos"])
def test_default_excitation_with_trial_gains_is_not_stopped_at_realistic_friction(
        lib, coax_params, inject):
    """页面默认激励 + 试用增益 + 摩擦 0.3/0.2，四个 psi：不被预测出窗门/超速门误停，位移离窗有余量。"""
    with_params(coax_params, TRIAL_GAINS)
    worst = 0.0
    for psi in PSIS:
        run = full_round(lib, inject, psi)
        assert run.reason == "complete", (inject, psi, run.end_line)
        worst = max(worst, max(abs(row["pos_u"]) for row in run.rows))
    print(f"[e2e] {inject} 默认激励最大 |p_u| = {worst * 1000:.0f} mm（窗 {WIN_MM} mm）")
    assert worst < WIN_MM * 0.001 * 0.7


def test_default_tilt_excitation_at_lighter_friction_is_stopped_by_the_window_gate(lib):
    """现状（交付报告 b）：摩擦降到 0.15/0.1，默认 tilt（0.07 rad）最后一个半周期被预测出窗门软停
    （位移 138 mm，离窗只有 12 mm），整轮记为 xy_window 而不是 complete。调到 win_mm=200 则完整。"""
    stopped = full_round(lib, "tilt", PSI_BENCH, fs=0.15, fk=0.1)
    assert stopped.reason == "xy_window", stopped.end_line
    assert max(abs(row["pos_u"]) for row in stopped.rows) < coast_bound_m(stopped)
    wide = full_round(lib, "tilt", PSI_BENCH, fs=0.15, fk=0.1, win_mm=200)
    assert wide.reason == "complete", wide.end_line


@pytest.mark.parametrize("inject", ["tilt", "vel", "pos"])
def test_frictionless_slot_soft_stops_or_stays_inside_the_window(lib, coax_params, inject):
    """无摩擦：要么整轮在窗内走完，要么软停（xy_window / xy_overspeed），位移不超过 窗 + 滑行。"""
    with_params(coax_params, TRIAL_GAINS)
    run = full_round(lib, inject, PSI_BENCH, fs=0.0, fk=0.0)
    assert run.reason == "complete" or run.reason in SOFT_STOPS, run.end_line
    assert max(abs(row["pos_u"]) for row in run.rows) < coast_bound_m(run)
    if inject == "tilt":
        assert run.reason in SOFT_STOPS, "无摩擦的 tilt 默认激励应被软停"


# ---------------------------------------------------------------- 记录批编码两侧一致


def test_firmware_batch_layout_matches_the_encoder_the_page_tests_use(lib):
    """页面测试的 xy_frame 编码示例（ALT_FIELDS 字段表）与固件真正吐出的批逐字节一致，
    否则页面测试绿了、真固件的帧却解不对。"""
    import struct

    from test_sysid_altitude_mode import ALT_FIELDS
    from test_sysid_xy_page import xy_frame
    run = full_round(lib, "tilt", math.pi / 4, noise=False)
    fw = run.schema.fields
    assert [(f.name, f.unit, f.scale, "i16" if f.type == 0 else "u16") for f in fw] == \
        [(n, u, sc, k) for n, u, sc, k in ALT_FIELDS]
    # 取第 3 批整批：解码回定点整数，用页面测试的编码函数重新编码，要与固件原字节一致（包头 hash 除外）。
    payload = run.frames[2]
    batch = run.batches[2]
    raw_rows = [{f.name: round(sample[f.name] / f.scale) for f in fw} for sample in batch.samples]
    rebuilt = xy_frame(batch.run_id, raw_rows, flags=batch.flags & ~FLAG_XY, base_us=batch.base_t_us,
                       dt_us=batch.dt_us)
    assert len(rebuilt) == len(payload)
    assert rebuilt[16:] == payload[16:], "记录体字节与页面测试的编码不一致"
    assert struct.unpack_from("<BBHIIHH", rebuilt)[:3] == struct.unpack_from("<BBHIIHH", payload)[:3]


# ---------------------------------------------------------------- 板上实际增益 + 作者明天的 psi=-45 度

PSI_BENCH = -math.pi / 4
FRICTION_TIERS = [(0.3, 0.2), (0.15, 0.1)]                 # (静, 动) m/s^2
#: 板上 Flash 里的实际 x 通道参数（2026-09-30 读回）。
BOARD_GAINS = {"coax.pos_x_kp": 0.9, "coax.vel_x_kp": 2.0, "coax.vel_x_ki": 0.6, "coax.vel_x_kd": 0.0,
               "coax.vel_x_i_limit_m_s2": 1.5, "coax.pos_xy_vel_max_m_s": 1.5,
               "coax.accel_xy_max_m_s2": 3.7}
#: 只动 x 通道、两档摩擦下 vel/pos 都能跟上的一组试用增益（SYSID PARAM 写 RAM）。
TRIAL_GAINS = dict(BOARD_GAINS, **{"coax.pos_x_kp": 1.2, "coax.vel_x_kp": 3.0, "coax.vel_x_ki": 3.0})


def with_params(coax_params, values):
    for name, value in values.items():
        assert coax_params(name, value) == 1, name


def spec_with(inject, amp, hold_ms, repeat=2, ramp_ms=150):
    spec = page_spec(inject)
    spec.update(amplitude_rad_s=amp, hold_ms=hold_ms, repeat=repeat,
                duration_ms=2 * hold_ms * repeat, ramp_ms=ramp_ms)
    return spec


def max_pos(run):
    return max(abs(row["pos_u"]) for row in run.rows)


#: 板上实际增益 + 页面默认 vel/pos 激励的现状：(注入, 静摩擦) -> 跟上的阶跃数（共 4 次）。
BOARD_FOLLOWED = {("vel", 0.3): 0, ("vel", 0.15): 2, ("pos", 0.3): 0, ("pos", 0.15): 4}


@pytest.mark.parametrize("fs,fk", FRICTION_TIERS)
@pytest.mark.parametrize("inject", ["vel", "pos"])
def test_board_gains_with_page_default_excitation_characterisation(lib, coax_params, inject, fs, fk):
    """现状（交付报告 c）：板上实际增益 + 新页面默认激励。0.3/0.2 下 vel、pos 都推不动；
    0.15/0.1 下 pos 全部跟上（慢，约 2.3 s 上升），vel 只跟上一半。整轮都不被软停。"""
    with_params(coax_params, BOARD_GAINS)
    run = full_round(lib, inject, PSI_BENCH, fs=fs, fk=fk)
    assert run.reason == "complete" and run.error is None, (run.reason, run.error)
    r = run.result
    assert r["followed"] == BOARD_FOLLOWED[(inject, fs)], r
    if r["followed"] < len(r["steps"]):
        assert r["warnings"]
    assert max(abs(row["pos_u"]) for row in run.rows) < WIN_MM * 0.001 * 0.6


@pytest.mark.parametrize("inject", ["vel", "pos"])
def test_board_gains_on_a_frictionless_slot_follow_the_default_excitation(lib, coax_params, inject):
    with_params(coax_params, BOARD_GAINS)
    run = full_round(lib, inject, PSI_BENCH, fs=0.0, fk=0.0)
    assert run.reason == "complete", run.end_line
    assert_steps_are_meaningful(run, preset_amp(inject))
    assert run.result["overshoot"] < 0.4


@pytest.mark.parametrize("fs,fk", FRICTION_TIERS)
@pytest.mark.parametrize("inject", ["vel", "pos"])
def test_trial_gains_with_default_excitation_follow_under_both_frictions(
        lib, coax_params, inject, fs, fk):
    with_params(coax_params, TRIAL_GAINS)
    run = full_round(lib, inject, PSI_BENCH, fs=fs, fk=fk)
    assert run.reason == "complete" and run.error is None, (run.reason, run.error)
    assert_steps_are_meaningful(run, preset_amp(inject))
    assert run.result["overshoot"] < 0.9
    assert max_pos(run) < WIN_MM * 0.001 * 0.85


@pytest.mark.parametrize("inject", ["vel", "pos"])
def test_trial_gains_on_a_frictionless_slot_do_not_ring_or_trip_the_window(lib, coax_params, inject):
    """同一组试用增益放在无摩擦对象上：不出窗、超调有限、每次阶跃都收敛。"""
    with_params(coax_params, TRIAL_GAINS)
    run = full_round(lib, inject, PSI_BENCH, fs=0.0, fk=0.0)
    assert run.reason == "complete", run.end_line
    assert_steps_are_meaningful(run, preset_amp(inject))
    assert run.result["overshoot"] < 0.5, run.result
    if inject == "pos":
        assert run.result["final_error"] < 0.5 * preset_amp(inject)
    assert max_pos(run) < WIN_MM * 0.001 * 0.85

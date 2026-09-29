"""陀螺振动主频 ÷ (eRPM/60) = 1/极对数：用合成数据钉住算法，用实录钉住"没有转速就不报"。

合成：采样跟着控制拍走（2.0–3.1 ms 的拍、每 4 ms 到期后取第一拍），陀螺里有 1 Hz 的大摆动、
上桨机械转频（42000 eRPM / 60 / 7 对极 = 100 Hz）的一阶振动和噪声。
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import pytest

np = pytest.importorskip("numpy")
pytest.importorskip("scipy")

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from sysid import vibration  # noqa: E402
sys.path.pop(0)

UPPER_ERPM, LOWER_ERPM, POLE_PAIRS = 42000.0, 38000.0, 7


def jittered_record(seconds=6.0, *, vibration_hz=UPPER_ERPM / 60.0 / POLE_PAIRS, seed=2,
                    upper=UPPER_ERPM, lower=LOWER_ERPM):
    rng = np.random.default_rng(seed)
    ticks = np.cumsum(rng.uniform(0.0020, 0.0031, int(seconds / 0.002) + 10))
    stamps, due = [], 0.0
    for tick in ticks:
        if tick >= seconds:
            break
        if tick >= due:
            stamps.append(tick)
            due += 0.004
    t = np.array(stamps)
    swing = 0.3 * np.sin(2 * math.pi * 1.0 * t)
    buzz = 0.05 * np.sin(2 * math.pi * vibration_hz * t + 0.3)
    samples = [{"gx": float(s + b + 0.01 * rng.standard_normal()),
                "gy": float(0.5 * b + 0.01 * rng.standard_normal()),
                "gz": float(0.01 * rng.standard_normal()),
                "erpm": float(upper + 40.0 * rng.standard_normal()),
                "erpm_lower": float(lower)} for s, b in zip(swing, buzz)]
    return list(t - t[0]), samples


def test_the_vibration_line_matches_one_over_the_pole_pairs():
    times, samples = jittered_record()
    report = vibration.erpm_vibration_report(times, samples)
    assert report is not None
    assert report.dominant_hz == pytest.approx(UPPER_ERPM / 60.0 / POLE_PAIRS, abs=0.3)
    upper, lower = report.rotors
    assert upper.label == "上桨" and upper.erpm == pytest.approx(UPPER_ERPM, rel=0.002)
    assert upper.ratio == pytest.approx(1.0 / POLE_PAIRS, rel=0.005) and upper.pole_pairs == 7
    assert lower.ratio == pytest.approx(100.0 / (LOWER_ERPM / 60.0), rel=0.005)
    assert lower.pole_pairs is None, "下桨转速对不上任何整数极对数"
    text = "\n".join(report.lines())
    assert "eRPM→振动比 0.1429" in text and "对得上 7 对极的一阶" in text
    assert "名义奈奎斯特" not in text


def test_the_low_frequency_swing_does_not_win():
    """摆动比振动大 6 倍：先扣掉 15 Hz 以下的运动，主频仍是振动线。"""
    times, samples = jittered_record(vibration_hz=60.0, upper=60.0 * 60.0 * POLE_PAIRS)
    report = vibration.erpm_vibration_report(times, samples)
    assert report.dominant_hz == pytest.approx(60.0, abs=0.3)
    assert report.rotors[0].pole_pairs == POLE_PAIRS


def test_no_real_erpm_means_no_report():
    times, samples = jittered_record()
    for sample in samples:
        sample["erpm"] = 0.0
        sample["erpm_lower"] = 0.0
    assert vibration.erpm_vibration_report(times, samples) is None
    only_upper = [{**s, "erpm": UPPER_ERPM} for s in samples]
    report = vibration.erpm_vibration_report(times, only_upper)
    assert [rotor.label for rotor in report.rotors] == ["上桨"]


def test_the_v2_real_runs_carry_no_erpm():
    """2026-09-27 的实录是 v2：erpm 恒为 0，不报振动比（也不瞎报）。"""
    folder = ROOT / "data" / "identification" / "attitude" / "2026-09-27" / "rod_041353_b932d1c9"
    if not (folder / "samples.csv").is_file():
        pytest.skip("实录不在本机")
    raw = np.genfromtxt(folder / "samples.csv", delimiter=",", names=True)
    samples = [{name: float(row[name]) for name in raw.dtype.names} for row in raw]
    times = list((raw["t_us"] - raw["t_us"][0]) * 1e-6)
    assert all(s["erpm"] == 0.0 for s in samples)
    assert vibration.erpm_vibration_report(times, samples) is None
    json.loads((folder / "conditions.json").read_text(encoding="utf-8"))   # 只读


def test_the_page_summary_text():
    sys.path.insert(0, str(ROOT / "tools"))
    try:
        from panel_lib.pages.sysid.results import vibration_summary
    finally:
        sys.path.pop(0)
    times, samples = jittered_record()
    text = vibration_summary(times, samples)
    assert text.startswith("【振动与转速】") and "上桨 420" in text and "eRPM→振动比 0.1429" in text
    assert vibration_summary(times[:10], samples[:10]) == ""

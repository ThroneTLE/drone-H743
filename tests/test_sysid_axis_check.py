"""杆轴方向核对（tools/panel_lib/pages/sysid/axis_check.py）。

2026-09-28 实机：杆实际装在 −45°，页面选了 +45°，三轮舵机单独拟合只剩 3–37%，页面没说原因。
这里用合成数据钉住判据：机体只能绕真实杆轴转，页面按所选 ψ 投影；选对时垂直分量与激励不同步，
选反（±45° 对调）时同步部分大半落到"垂直"上。
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

np = pytest.importorskip("numpy")
pytest.importorskip("scipy")

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from panel_lib.pages.sysid import axis_check  # noqa: E402
import panel_lib.pages.sysid.amplitude_hint  # noqa: E402,F401  （hold_warning 用例）
sys.path.pop(0)


def synthetic_run(true_psi_deg: float, *, mode="3", seconds=8.0, seed=3):
    """真实杆轴 true_psi 上的响应：输入是 250 ms 双脉冲，响应是它的一阶滞后 + 摆 + 噪声。"""
    rng = np.random.default_rng(seed)
    t = np.arange(0.0, seconds, 0.004)
    u = np.where(((t - 0.5) % 0.5) < 0.25, 0.05, -0.05) * (t > 0.5)
    rate = np.zeros_like(t)
    for k in range(1, t.size):                       # 一阶低通（约 3 Hz）当作"机体对激励的响应"
        rate[k] = rate[k - 1] + (u[k] * 4.0 - rate[k - 1]) * 0.004 * 2 * math.pi * 3
    rate += 0.02 * np.sin(2 * math.pi * 0.66 * t)   # 台架自己的摆，沿杆
    psi = math.radians(true_psi_deg)
    gx = math.cos(psi) * rate + 0.003 * rng.standard_normal(t.size)
    gy = math.sin(psi) * rate + 0.003 * rng.standard_normal(t.size)
    key = "servo_tilt" if mode == "3" else "torque"
    samples = [{"gx": float(a), "gy": float(b), key: float(c)} for a, b, c in zip(gx, gy, u)]
    return t, samples


@pytest.mark.parametrize("mode", ["3", "0"])
def test_the_right_axis_passes_and_the_mirrored_one_is_flagged(mode):
    t, samples = synthetic_run(-45.0, mode=mode)
    right = axis_check.perpendicular_ratio(t, samples, math.radians(-45.0), mode)
    wrong = axis_check.perpendicular_ratio(t, samples, math.radians(45.0), mode)
    assert right is not None and right < 0.1
    assert wrong is not None and wrong > axis_check.AXIS_WRONG_RATIO
    assert axis_check.axis_warning(right, math.radians(-45.0)) == ""
    text = axis_check.axis_warning(wrong, math.radians(45.0))
    assert "杆轴方向很可能选错" in text and "+45°" in text and "不能用" in text


def test_the_pendulum_alone_does_not_trip_the_check():
    """台架自己的摆（与激励不同步）再大也不算：只看与激励同步的部分。"""
    t, samples = synthetic_run(90.0)
    for s, tk in zip(samples, t):
        s["gx"] += 0.05 * math.sin(2 * math.pi * 2.3 * tk)      # 垂直方向的不同步晃动
    ratio = axis_check.perpendicular_ratio(t, samples, math.radians(90.0), "3")
    assert ratio is not None and ratio < axis_check.AXIS_WRONG_RATIO


def test_missing_or_flat_input_gives_no_verdict():
    t, samples = synthetic_run(90.0)
    assert axis_check.perpendicular_ratio(t[:100], samples[:100], 0.0, "3") is None
    flat = [dict(s, servo_tilt=0.0) for s in samples]
    assert axis_check.perpendicular_ratio(t, flat, 0.0, "3") is None
    assert axis_check.perpendicular_ratio(t, [{"gx": 0.0} for _ in samples], 0.0, "3") is None
    assert axis_check.axis_warning(None, 0.0) == ""


def test_the_analysis_puts_the_warning_first():
    source = (ROOT / "tools/panel_lib/pages/sysid/analysis.py").read_text(encoding="utf-8")
    assert "axis_warning(perpendicular_ratio(times, samples, run[\"psi\"], mode), run[\"psi\"])" in source
    assert source.index("(axis_note,") < source.index("notch_provenance_text(self.snapshot)")


def test_a_long_hold_for_identification_is_warned_before_start():
    """FF / 舵机单独 + 双脉冲 + 单段 ≥ 500 ms：开始前提醒改回 250 ms（不挡开始）。"""
    from panel_lib.pages.sysid.amplitude_hint import AmplitudeHint

    class Var:
        def __init__(self, value): self.value = value
        def get(self): return self.value

    class Page(AmplitudeHint):
        def __init__(self, mode, profile, hold):
            self.mode_var, self.profile_var, self.hold_var = Var(mode), Var(profile), Var(hold)
        def amp_swing_estimate(self):
            return None

    assert "250 ms" in Page("FF", "doublet", "1000").amp_swing_warning()
    assert "250 ms" in Page("SERVO", "doublet", "500").hold_warning()
    assert Page("FF", "doublet", "250").amp_swing_warning() == ""
    assert Page("RATE", "doublet", "1000").hold_warning() == "", "验证轮本来就用 1000 ms"
    assert Page("FF", "chirp", "1000").hold_warning() == ""
    assert Page("FF", "doublet", "abc").hold_warning() == ""

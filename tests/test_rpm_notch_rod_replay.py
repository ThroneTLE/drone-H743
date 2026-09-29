"""实录重放：2026-09-27 光杆台架的真实陀螺，过一遍编译好的转速陷波（Driver）。

数据（只读）：`data/identification/attitude/2026-09-27/` 下
rod_035506_982cc822（RATE 验证轮，mode 1）与 rod_034624_52a38c9b（FF 轮，mode 0），
都是俯仰杆、7.4 N 悬停。gy 上有 107.55 Hz（约 0.068 rad/s）与 109.5 Hz 两条桨叶线。

方法：gx/gy 按真实 t_us 线性插值到 1 kHz 网格（float32），控制拍每 2～3 个样本一次
（种子固定），丢掉前 100 个样本，Hann 窗 FFT 求带功率比。

**局限**：原始记录是 250 Hz，线性插值把 107 Hz 线本身压低约 5.6 dB，陷波前后两边一样，
所以这里只看比值，不看绝对量；真正的绝对量要等 1 kHz IMUCAP 实录（硬件 H1）。

内置反向核对：旁路时衰减 < 1 dB（判据不是白给的）；中心 ×1.05（极对数错）掉到 17 dB 以下。
"""
from __future__ import annotations

import math
from pathlib import Path

import pytest

np = pytest.importorskip("numpy")

from test_rpm_notch_filter import build_notch_lib, fresh_bank, run_bank, tick_schedule  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
DAY = ROOT / "data" / "identification" / "attitude" / "2026-09-27"
RUNS = {"rate": DAY / "rod_035506_982cc822", "ff": DAY / "rod_034624_52a38c9b"}
FS = 1000.0
DROP = 100
Q = 3.0

if not all((folder / "samples.csv").is_file() for folder in RUNS.values()):
    pytest.skip("实录不在本机", allow_module_level=True)


@pytest.fixture(scope="module")
def lib(tmp_path_factory):
    return build_notch_lib(tmp_path_factory, name="rpm-notch-replay")


@pytest.fixture(scope="module")
def grids():
    out = {}
    for name, folder in RUNS.items():
        raw = np.genfromtxt(folder / "samples.csv", delimiter=",", names=True)
        t = (raw["t_us"] - raw["t_us"][0]) * 1e-6
        grid = np.arange(0.0, t[-1], 1.0 / FS)
        signal = np.zeros((grid.size, 3), np.float32)
        for axis, key in enumerate(("gx", "gy", "gz")):
            signal[:, axis] = np.interp(grid, t, raw[key])
        out[name] = signal
    return out


def replay(lib, signal, centers, fresh, seed=7):
    n = signal.shape[0]
    ticks, gaps = tick_schedule(n, seed=seed)
    hz = np.tile(np.asarray(centers, np.float32), (n, 1))
    fresh_arr = np.tile(np.asarray(fresh, np.uint8), (n, 1))
    fresh_bank(lib, FS, q=Q)
    out, _ = run_bank(lib, signal, ticks=ticks, hz=hz, fresh=fresh_arr, dt=gaps / FS)
    return signal[DROP:], out[DROP:]


def band_power(x, low, high):
    window = np.hanning(len(x))
    spectrum = np.fft.rfft((x - np.mean(x)) * window)
    freq = np.fft.rfftfreq(len(x), 1.0 / FS)
    mask = (freq >= low) & (freq <= high)
    return float(np.sum(np.abs(spectrum[mask]) ** 2))


def band_db(a, b, low=100.0, high=115.0):
    return 10.0 * math.log10(band_power(a, low, high) / band_power(b, low, high))


def line_amplitude(x, freq):
    k = np.arange(len(x)) / FS
    basis = np.stack([np.sin(2 * np.pi * freq * k), np.cos(2 * np.pi * freq * k), np.ones_like(k)], 1)
    coef, *_ = np.linalg.lstsq(basis, np.asarray(x, np.float64), rcond=None)
    return math.hypot(coef[0], coef[1])


def low_band(x, shift_s=0.0, high=10.0):
    """0–10 Hz 分量（FFT 置零），可按 shift_s 超前（去掉解析群延迟）。"""
    spectrum = np.fft.rfft(np.asarray(x, np.float64))
    freq = np.fft.rfftfreq(len(x), 1.0 / FS)
    spectrum[freq > high] = 0.0
    return np.fft.irfft(spectrum * np.exp(2j * np.pi * freq * shift_s), len(x))


def group_delay_s(centers, q=Q, at_hz=1.0):
    """级联 RBJ 陷波在低频的解析群延迟 −dφ/dω（双精度，同固件闭式系数）。"""
    def phase(f):
        z = np.exp(-2j * np.pi * f / FS)
        total = 1.0
        for center in centers:
            w0 = 2 * math.pi * center / FS
            alpha = math.sin(w0) / (2 * q)
            k = 1 / (1 + alpha)
            b0, b1, a2 = k, -2 * math.cos(w0) * k, (1 - alpha) * k
            total = total * (b0 + b1 * z + b0 * z * z) / (1 + b1 * z + a2 * z * z)
        return float(np.angle(total))
    step = 0.01
    return -(phase(at_hz + step) - phase(at_hz - step)) / (2 * 2 * math.pi * step)


def pointwise_low_diff(a, b, shift_s=0.0):
    core = slice(200, -200)                   # 避开 FFT 边缘
    la, lb = low_band(a), low_band(b, shift_s)
    return float(np.sqrt(np.mean((la[core] - lb[core]) ** 2)) / np.sqrt(np.mean(la[core] ** 2)))


RUN_AXES = [(run, axis) for run in RUNS for axis in (0, 1)]


# ---------------------------------------------------------------- 单中心（电机 2 过期）


@pytest.mark.parametrize("run,axis", RUN_AXES)
def test_single_centre_removes_the_band(lib, grids, run, axis):
    raw, out = replay(lib, grids[run], (107.5, 107.5), (1, 0))
    assert band_db(raw[:, axis], out[:, axis]) >= 17.0
    assert 20 * math.log10(line_amplitude(raw[:, axis], 107.55) /
                           line_amplitude(out[:, axis], 107.55)) >= 35.0


# ---------------------------------------------------------------- 两个电机各一个中心


@pytest.mark.parametrize("run,axis", RUN_AXES)
def test_two_motor_notches_take_the_band_down_by_25_db(lib, grids, run, axis):
    raw, out = replay(lib, grids[run], (107.55, 109.5), (1, 1))
    assert band_db(raw[:, axis], out[:, axis]) >= 25.0
    low_change = math.sqrt(band_power(out[:, axis], 0.3, 10.0) / band_power(raw[:, axis], 0.3, 10.0))
    assert abs(low_change - 1.0) <= 0.002


def test_the_rate_run_low_band_differs_only_by_the_group_delay(lib, grids):
    """gy 逐点 0–10 Hz 差 ≤ 1.5%；扣掉解析群延迟（约 0.94 ms）后 ≤ 0.15%。"""
    raw, out = replay(lib, grids["rate"], (107.55, 109.5), (1, 1))
    tau = group_delay_s((107.55, 109.5))
    assert 0.8e-3 < tau < 1.1e-3
    assert pointwise_low_diff(raw[:, 1], out[:, 1]) <= 0.015
    assert pointwise_low_diff(raw[:, 1], out[:, 1], tau) <= 0.0015


def test_the_ff_run_is_also_just_delayed(lib, grids):
    """FF 轮激励更陡，逐点差更大（约 2%），但扣掉群延迟后同样 ≤ 0.15%。"""
    raw, out = replay(lib, grids["ff"], (107.55, 109.5), (1, 1))
    assert pointwise_low_diff(raw[:, 1], out[:, 1], group_delay_s((107.55, 109.5))) <= 0.0015


# ---------------------------------------------------------------- 反向核对


@pytest.mark.parametrize("run", list(RUNS))
def test_bypass_shows_no_attenuation(lib, grids, run):
    raw, out = replay(lib, grids[run], (107.5, 109.5), (0, 0))
    assert abs(band_db(raw[:, 1], out[:, 1])) < 1.0
    assert out.tobytes() == raw.tobytes()


@pytest.mark.parametrize("run", list(RUNS))
def test_a_wrong_pole_pair_count_loses_the_line(lib, grids, run):
    raw, out = replay(lib, grids[run], (107.5 * 1.05, 107.5 * 1.05), (1, 0))
    assert band_db(raw[:, 1], out[:, 1]) < 17.0

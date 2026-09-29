"""转速陷波的算法层（Driver/Src/drv_rpm_notch.c）—— 在宿主上跑真 C 代码。

判据按"上机会出什么事"排：

* **挖得准**：107.5 Hz / Q3 的陷波在中心压 40 dB 以上，±1 Hz 仍有 20 dB；10 Hz 只丢
  约 1.72° 相位、增益几乎不动；直流增益恰为 1。这些在固件热路径（DRV_RpmNotch_Apply 内联的
  DF1）上量，公开的单步参照 DRV_Notch_Step 只是对照——不依赖实录，干净检出也守得住。
* **起步与复位不砸台阶**：启用、复位（缺口/倒退/帧复位/非有限之后）都以当前输入为直流稳态起步。
* **动得稳**：中心跟着转速走（DF1 换系数不注入台阶）；开、关、失效都按 20 ms 线性淡入淡出，
  输出每一步不比输入自己的一步大；转速一过期 20 ms 内权重归零。
* **关掉就是原样**：没有活动槽时输出逐位等于输入（memcmp），不活动的槽连状态都不碰。
* **坏数据不留后患**：NaN/inf 当拍放行、下一拍回稳态；零输入不留次正规数。

装置在 tests/fixtures/rpm_notch/notch_harness.c：批量运行器 + C 里才看得见的检查。
"""
from __future__ import annotations

import ctypes
import math
import os
import shutil
import subprocess
from pathlib import Path

import pytest

np = pytest.importorskip("numpy")

ROOT = Path(__file__).resolve().parents[1]
SOURCES = ["Driver/Src/drv_rpm_notch.c", "tests/fixtures/rpm_notch/notch_harness.c"]

F = ctypes.c_float
U8 = ctypes.c_uint8
U32 = ctypes.c_uint32
FP = ctypes.POINTER(ctypes.c_float)
U8P = ctypes.POINTER(ctypes.c_uint8)
SLOTS = 6


def build_notch_lib(tmp_path_factory, sources=SOURCES, name="rpm-notch"):
    """gcc 编 DLL；-Werror，没有 gcc 就跳过（同 test_sysid_runtime_contract）。"""
    compiler = shutil.which("gcc") or shutil.which("clang")
    if compiler is None:
        pytest.skip("host gcc/clang required for the rpm notch contract")
    build = tmp_path_factory.mktemp(name)
    out = build / ("notch.dll" if os.name == "nt" else "notch.so")
    result = subprocess.run(
        [compiler, "-shared", "-fPIC", "-std=c11", "-O1", "-Wall", "-Wextra", "-Werror",
         "-I", str(ROOT / "Driver" / "Inc"), *[str(ROOT / path) for path in sources],
         "-lm", "-o", str(out)],
        capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr
    lib = ctypes.CDLL(str(out))
    lib.harness_bank_init.argtypes = [U8, F, F, F]
    lib.harness_bank_init.restype = U8
    lib.harness_bank_set_fs.argtypes = [F]
    lib.harness_bank_set_motor.argtypes = [U8, F, U8, F]
    lib.harness_bank_apply.argtypes = [FP, FP]
    lib.harness_slot_weight.argtypes = [U8, U8]
    lib.harness_slot_weight.restype = F
    lib.harness_active_slots.restype = U8
    for name_ in ("harness_slot_center", "harness_slot_target"):
        getattr(lib, name_).argtypes = [U32]
        getattr(lib, name_).restype = F
    for name_ in ("harness_slot_active", "harness_slot_coef_valid"):
        getattr(lib, name_).argtypes = [U32]
        getattr(lib, name_).restype = U8
    lib.harness_slot_coef.argtypes = [U32, FP]
    lib.harness_slot_state.argtypes = [U32, U32, FP]
    lib.harness_motor_hz.argtypes = [U8]
    lib.harness_motor_hz.restype = F
    lib.harness_counter.argtypes = [U32]
    lib.harness_counter.restype = U32
    lib.harness_run.argtypes = [U32, FP, FP, U8P, FP, U8P, FP, FP]
    lib.harness_single.argtypes = [F, F, F, U32, FP, FP]
    lib.harness_single.restype = U8
    lib.harness_single_varying.argtypes = [FP, F, F, U32, FP, FP]
    lib.harness_single_varying.restype = U8
    lib.harness_bank_response.argtypes = [F, F, F, U32, U32, FP, FP]
    lib.harness_bank_response.restype = U8
    lib.harness_design_untouched.argtypes = [F, F, F, U8P]
    lib.harness_design_untouched.restype = U8
    lib.harness_design.argtypes = [F, F, F, FP]
    lib.harness_design.restype = U8
    lib.harness_bank_has_subnormal.restype = U8
    lib.harness_zero_tail_has_subnormal.argtypes = [F, F, F]
    lib.harness_zero_tail_has_subnormal.restype = U8
    return lib


@pytest.fixture(scope="module")
def lib(tmp_path_factory):
    return build_notch_lib(tmp_path_factory)


def ptr(array, kind=FP):
    return array.ctypes.data_as(kind)


def fresh_bank(lib, fs=1000.0, mask=1, q=3.0, min_hz=50.0, fade_hz=20.0):
    assert lib.harness_bank_init(mask, q, min_hz, fade_hz) == 1
    lib.harness_bank_set_fs(fs)


def run_bank(lib, signal, *, ticks=None, hz=None, fresh=None, dt=None, weights=False):
    """signal: (n,3) float32。ticks/hz/fresh/dt 按样本给；返回 (out, weights 或 None)。"""
    signal = np.ascontiguousarray(signal, dtype=np.float32)
    n = signal.shape[0]
    out = np.zeros_like(signal)
    tick_arr = np.zeros(n, np.uint8) if ticks is None else np.ascontiguousarray(ticks, np.uint8)
    hz_arr = np.zeros((n, 2), np.float32) if hz is None else np.ascontiguousarray(hz, np.float32)
    fresh_arr = np.zeros((n, 2), np.uint8) if fresh is None else np.ascontiguousarray(fresh, np.uint8)
    dt_arr = np.zeros(n, np.float32) if dt is None else np.ascontiguousarray(dt, np.float32)
    w = np.zeros((n, SLOTS), np.float32) if weights else None
    lib.harness_run(n, ptr(signal), ptr(out), ptr(tick_arr, U8P), ptr(hz_arr), ptr(fresh_arr, U8P),
                    ptr(dt_arr), ptr(w) if weights else None)
    return out, w


def tick_schedule(n, seed=1, first=0):
    """控制拍每 2～3 个 IMU 样本一次（种子固定）。返回 (tick 标志, 距上一拍的 dt 样本数)。"""
    rng = np.random.default_rng(seed)
    ticks = np.zeros(n, np.uint8)
    gaps = np.zeros(n, np.float32)
    i, last = first, first
    while i < n:
        ticks[i] = 1
        gaps[i] = max(i - last, 2)
        last = i
        i += int(rng.integers(2, 4))
    return ticks, gaps


def dtft(h, f, fs):
    k = np.arange(len(h))
    return np.sum(np.asarray(h, np.float64) * np.exp(-2j * np.pi * f * k / fs))


def respond(lib, x, f0=107.5, q=3.0, fs=1000.0, path="bank"):
    """同一个陷波从零状态跑 x。path="step" 走 DRV_Notch_Step（公开的单步参照）；
    path="bank" 走固件每个样本真正跑的 DRV_RpmNotch_Apply（内联的那份 DF1，权重已爬满）。"""
    x = np.ascontiguousarray(x, np.float32)
    y = np.zeros_like(x)
    if path == "step":
        assert lib.harness_single(f0, q, fs, len(x), ptr(x), ptr(y)) == 1
    else:
        assert lib.harness_bank_response(f0, q, fs, 40, len(x), ptr(x), ptr(y)) == 1
    return y


def impulse_response(lib, f0, q, fs, n=16384, path="bank"):
    x = np.zeros(n, np.float32)
    x[0] = 1.0
    return respond(lib, x, f0, q, fs, path)


# ---------------------------------------------------------------- 设计与频响


def test_coefficients_follow_the_closed_form(lib):
    out = (F * 3)()
    assert lib.harness_design(107.5, 3.0, 1000.0, out) == 1
    assert list(out) == pytest.approx([0.9056, -1.4136, 0.8113], abs=1e-4)
    w0 = 2 * math.pi * 107.5 / 1000.0
    alpha = math.sin(w0) / 6.0
    k = 1.0 / (1.0 + alpha)
    assert list(out) == pytest.approx([k, -2 * math.cos(w0) * k, (1 - alpha) * k], rel=1e-5)


@pytest.mark.parametrize("path", ["bank", "step"])
@pytest.mark.parametrize("fs", [1000.0, 800.0])
def test_compiled_response_meets_the_band_budget(lib, fs, path):
    """频响、相位、直流在两条路径上都要成立：bank 是固件热路径，step 是公开的参照实现。"""
    h = impulse_response(lib, 107.5, 3.0, fs, path=path)
    db = lambda f: 20 * math.log10(abs(dtft(h, f, fs)))  # noqa: E731
    assert db(107.5) < -40.0
    assert db(106.5) < -20.0 and db(108.5) < -20.0
    assert math.degrees(np.angle(dtft(h, 10.0, fs))) == pytest.approx(-1.72, abs=0.05)
    assert abs(db(10.0)) < 0.01
    assert abs(abs(dtft(h, 0.0, fs)) - 1.0) < 1e-6


def test_the_hot_path_is_the_reference_step_at_full_weight(lib):
    """权重为 1 时，滤波组每个样本的输出就是 DRV_Notch_Step 的输出（差在混合 x+w·(y−x) 的舍入里）。

    热路径是按槽内联的一份 DF1；这条把它钉在参照式子上：系数、状态移位、符号任何一处写错，
    带直流和带内噪声的输入上立刻差出几个数量级。"""
    rng = np.random.default_rng(11)
    n = 4000
    x = (3.0 + rng.normal(0.0, 0.5, n) + 0.2 * np.sin(2 * np.pi * 9.0 * np.arange(n) / 1000.0))
    bank = respond(lib, x, path="bank")
    step = respond(lib, x, path="step")
    scale = np.maximum(np.abs(x.astype(np.float32)), np.abs(step))
    assert np.all(np.abs(bank - step) <= 2.0 * np.spacing(scale))


@pytest.mark.parametrize("f0,q,fs", [
    (0.0, 3.0, 1000.0), (-5.0, 3.0, 1000.0), (500.0, 3.0, 1000.0), (600.0, 3.0, 1000.0),
    (107.5, 1.49, 1000.0), (107.5, 10.01, 1000.0), (float("nan"), 3.0, 1000.0),
    (107.5, float("nan"), 1000.0), (107.5, 3.0, float("nan")), (107.5, 3.0, 0.0),
    (107.5, 3.0, -1000.0), (float("inf"), 3.0, 1000.0)])
def test_invalid_design_is_refused_and_leaves_coefficients_untouched(lib, f0, q, fs):
    ok = U8()
    assert lib.harness_design_untouched(f0, q, fs, ctypes.byref(ok)) == 1
    assert ok.value == 0


# ---------------------------------------------------------------- DF1 瞬态


def test_centre_step_with_dc_does_not_kick_the_output(lib):
    """中心 100 → 110 Hz 跳变，DC 1.5 + 0.1 线（线与中心一起跳）：输出偏离 DC 的峰值 ≤ 0.035。"""
    fs, n, step = 1000.0, 1200, 600
    freq = np.where(np.arange(n) < step, 100.0, 110.0)
    phase = np.cumsum(2 * np.pi * freq / fs)
    x = (1.5 + 0.1 * np.sin(phase)).astype(np.float32)
    y = np.zeros(n, np.float32)
    assert lib.harness_single_varying(ptr(freq.astype(np.float32)), 3.0, fs, n, ptr(x), ptr(y)) == 1
    settled = slice(300, n)            # 开头从 x[0] 的稳态起步，前 300 ms 让振铃衰完
    assert np.max(np.abs(y[settled] - 1.5)) <= 0.035


def _tdf2(x, centers, q, fs):
    """Python 转置 II 型参照：同样的逐样本换系数，状态依赖系数，换一次注入一次。"""
    y = np.zeros(len(x))
    s1 = s2 = None
    for i, (xi, c) in enumerate(zip(x, centers)):
        w0 = 2 * math.pi * c / fs
        alpha = math.sin(w0) / (2 * q)
        k = 1 / (1 + alpha)
        b0, b1, a2 = k, -2 * math.cos(w0) * k, (1 - alpha) * k
        if s1 is None:                  # 起步即直流稳态
            s1 = xi - b0 * xi
            s2 = b0 * xi - a2 * xi
        yi = b0 * xi + s1
        s1 = b1 * xi - b1 * yi + s2
        s2 = b0 * xi - a2 * yi
        y[i] = yi
    return y


def test_coefficient_jitter_on_dc_injects_almost_nothing_in_df1(lib):
    """±0.3% 的中心抖动（eRPM 量化），每 2～3 样本换一次，DC 3：DF1 注入 ≤ 1e-3 rms。

    同一序列的 TDF2 参照注入明显更大（约 1.7e-3）——阈值分得开两种结构。"""
    fs, n = 1000.0, 4000
    rng = np.random.default_rng(3)
    ticks, _gaps = tick_schedule(n, seed=5)
    centers = np.empty(n, np.float32)
    current = 107.5
    for i in range(n):
        if ticks[i]:
            current = 107.5 * (1.0 + rng.uniform(-0.003, 0.003))
        centers[i] = current
    x = np.full(n, 3.0, np.float32)
    y = np.zeros(n, np.float32)
    assert lib.harness_single_varying(ptr(centers), 3.0, fs, n, ptr(x), ptr(y)) == 1
    df1 = float(np.sqrt(np.mean((y - 3.0) ** 2)))
    tdf2 = float(np.sqrt(np.mean((_tdf2(x, centers, 3.0, fs) - 3.0) ** 2)))
    assert df1 <= 1e-3
    assert tdf2 > 1e-3 and tdf2 > 3.0 * df1


def test_the_hot_path_also_injects_almost_nothing_on_jittering_dc(lib):
    """同样的 ±0.3% 抖动，经 SetMotor/Apply 走固件路径（0.1 Hz 以上才重算系数，控制拍 2～3 样本）。

    TDF2 参照按滤波组实际用的中心逐拍换系数：热路径若不是 DF1，这里就分得开。"""
    fs, n = 1000.0, 4000
    rng = np.random.default_rng(3)
    ticks, gaps = tick_schedule(n, seed=5)
    hz = np.zeros((n, 2), np.float32)
    fresh = np.zeros((n, 2), np.uint8)
    fresh[:, 0] = 1
    used = np.empty(n)
    current = center = None
    for i in range(n):
        if ticks[i]:
            current = 107.5 * (1.0 + rng.uniform(-0.003, 0.003))
            if center is None or abs(np.float32(current) - center) > 0.1:   # 滤波组的重算门
                center = float(np.float32(current))
        hz[i, 0] = current
        used[i] = center
    signal = np.full((n, 3), 3.0, np.float32)
    fresh_bank(lib, fs)
    out, _ = run_bank(lib, signal, ticks=ticks, hz=hz, fresh=fresh, dt=gaps / fs)
    bank = float(np.sqrt(np.mean((out[:, 0] - 3.0) ** 2)))
    tdf2 = float(np.sqrt(np.mean((_tdf2(signal[:, 0], used, 3.0, fs) - 3.0) ** 2)))
    assert bank <= 1e-3
    assert tdf2 > 3.0 * bank


# ---------------------------------------------------------------- 跟踪


def _sweep(lib, rate_hz_s, *, lead_s=0.3, f_start=100.0, f_end=130.0, tail_s=0.1, latency_s=0.003,
           amplitude=0.1, fs=1000.0, seed=2):
    sweep_s = (f_end - f_start) / rate_hz_s
    n = int((lead_s + sweep_s + tail_s) * fs)
    t = np.arange(n) / fs

    def freq_at(time):
        return np.clip(f_start + rate_hz_s * np.maximum(time - lead_s, 0.0), f_start, f_end)

    phase = np.cumsum(2 * np.pi * freq_at(t) / fs)
    line = amplitude * np.sin(phase)
    signal = np.zeros((n, 3), np.float32)
    signal[:, 0] = line
    ticks, gaps = tick_schedule(n, seed=seed)
    hz = np.zeros((n, 2), np.float32)
    hz[:, 0] = freq_at(t - latency_s)             # 回包晚 3 ms
    fresh = np.zeros((n, 2), np.uint8)
    fresh[:, 0] = 1
    fresh_bank(lib, fs)
    out, _ = run_bank(lib, signal, ticks=ticks, hz=hz, fresh=fresh, dt=gaps / fs)
    sweep = slice(int(lead_s * fs), int((lead_s + sweep_s) * fs))
    return out[:, 0], line, sweep


def test_a_150_hz_per_s_sweep_is_tracked(lib):
    out, line, sweep = _sweep(lib, 150.0)
    residual = float(np.sqrt(np.mean(out[sweep] ** 2)))
    assert residual <= 0.05 * float(np.sqrt(np.mean(line[sweep] ** 2)))


def test_a_1000_hz_per_s_punch_stays_small(lib):
    out, _line, sweep = _sweep(lib, 1000.0)
    after = slice(sweep.start, len(out))
    assert float(np.max(np.abs(out[after]))) <= 0.03


# ---------------------------------------------------------------- 旁路


def _noise(n, seed=4):
    rng = np.random.default_rng(seed)
    return (rng.normal(0.0, 0.3, (n, 3)) + np.array([0.1, -2.0, 7.0])).astype(np.float32)


@pytest.mark.parametrize("case", ["no_setmotor", "stale", "fs_zero"])
def test_bypass_is_bitwise_identity(lib, case):
    n = 3000
    signal = _noise(n)
    ticks, gaps = tick_schedule(n)
    hz = np.full((n, 2), 107.5, np.float32)
    fresh = np.ones((n, 2), np.uint8)
    fresh_bank(lib, 0.0 if case == "fs_zero" else 1000.0)
    if case == "no_setmotor":
        out, _ = run_bank(lib, signal)
    else:
        if case == "stale":
            fresh[:] = 0
        out, _ = run_bank(lib, signal, ticks=ticks, hz=hz, fresh=fresh, dt=gaps / 1000.0)
    assert out.tobytes() == signal.tobytes()
    assert lib.harness_active_slots() == 0


def test_an_inactive_slot_is_never_stepped(lib):
    n = 800
    signal = _noise(n)
    ticks, gaps = tick_schedule(n)
    hz = np.full((n, 2), 107.5, np.float32)
    fresh = np.zeros((n, 2), np.uint8)
    fresh[:, 0] = 1                         # 只有电机 1（通道 1）新鲜
    fresh_bank(lib)
    out, _ = run_bank(lib, signal, ticks=ticks, hz=hz, fresh=fresh, dt=gaps / 1000.0)
    assert lib.harness_slot_active(0) == 1 and lib.harness_slot_active(3) == 0
    state = (F * 4)()
    for axis in range(3):
        lib.harness_slot_state(3, axis, state)
        assert list(state) == [0.0, 0.0, 0.0, 0.0]
    assert out.tobytes() != signal.tobytes()


# ---------------------------------------------------------------- 淡入淡出与过期


@pytest.mark.parametrize("fs,samples", [(1000.0, 20), (800.0, 16)])
def test_activation_ramps_the_weight_over_twenty_ms(lib, fs, samples):
    n = 200
    signal = np.zeros((n, 3), np.float32)
    ticks = np.zeros(n, np.uint8)
    ticks[0] = 1
    hz = np.full((n, 2), 107.5, np.float32)
    fresh = np.zeros((n, 2), np.uint8)
    fresh[:, 0] = 1
    fresh_bank(lib, fs)
    _out, w = run_bank(lib, signal, ticks=ticks, hz=hz, fresh=fresh, dt=np.full(n, 0.002, np.float32),
                       weights=True)
    trace = w[:, 0]
    assert trace[0] == pytest.approx(1.0 / samples, rel=1e-4)
    assert np.all(np.diff(trace[:samples]) > 0)
    assert trace[samples - 1] == 1.0 and trace[samples - 2] < 1.0


def test_activation_starts_from_the_current_input_without_a_kick(lib):
    """启用（首次、以及淡出后重新捕获）以当前输入为直流稳态起步：纯直流上输出一点不动。

    起步若从 0 开始，7 rad/s 的直流会在 20 ms 爬升里漏出约 0.4 rad/s 的冲击，爬升盖不住。"""
    fs, n = 1000.0, 600
    dc = np.where(np.arange(n) < 300, 7.0, -4.0)
    signal = np.repeat(dc[:, None], 3, axis=1).astype(np.float32)
    ticks, gaps = tick_schedule(n)
    hz = np.full((n, 2), 107.5, np.float32)
    fresh = np.zeros((n, 2), np.uint8)
    fresh[:, 0] = 1
    fresh[200:350, 0] = 0                    # 淡出；不活动时直流从 7 变成 −4，然后重新捕获
    fresh_bank(lib, fs)
    out, w = run_bank(lib, signal, ticks=ticks, hz=hz, fresh=fresh, dt=gaps / fs, weights=True)
    assert w[100, 0] == 1.0 and w[300, 0] == 0.0 and w[-1, 0] == 1.0
    assert float(np.max(np.abs(out[:, 0] - signal[:, 0]))) <= 1e-5


def _dc_jump_across(lib, trigger):
    """全权重跟踪 0.5 + 0.1 线，然后触发复位；复位后真实角速度是 3、线的相位跳了 2 rad
    （一次长缺口里机体转起来了）。返回复位后的轴 0 输出与新直流。"""
    fs, n1, n2, dc2 = 1000.0, 400, 120, 3.0
    t = np.arange(n1 + n2) / fs
    line = 0.1 * np.sin(2 * np.pi * 107.5 * t + np.where(np.arange(n1 + n2) < n1, 0.0, 2.0))
    level = np.where(np.arange(n1 + n2) < n1, 0.5, dc2)
    signal = np.repeat((level + line)[:, None], 3, axis=1).astype(np.float32)
    ticks, gaps = tick_schedule(n1 + n2)
    hz = np.full((n1 + n2, 2), 107.5, np.float32)
    fresh = np.zeros((n1 + n2, 2), np.uint8)
    fresh[:, 0] = 1
    fresh_bank(lib, fs)
    first, second = slice(0, n1), slice(n1, n1 + n2)
    run_bank(lib, signal[first], ticks=ticks[first], hz=hz[first], fresh=fresh[first], dt=gaps[first] / fs)
    assert lib.harness_slot_weight(0, 1) == 1.0          # 复位落在一个全权重的活动槽上
    if trigger == "reset":
        lib.harness_bank_reset()
    else:
        bad = np.full((1, 3), np.nan, np.float32)
        run_bank(lib, bad)
    out, _ = run_bank(lib, signal[second], ticks=ticks[second], hz=hz[second], fresh=fresh[second],
                      dt=gaps[second] / fs)
    assert lib.harness_counter(3) == 1 and lib.harness_slot_weight(0, 1) == 1.0
    return out[:, 0], dc2


@pytest.mark.parametrize("trigger", ["reset", "nonfinite"])
def test_a_reset_restarts_active_slots_from_the_current_input(lib, trigger):
    """复位（大缺口、时间戳倒退、坐标系/校准复位、ODR 变化、非有限样本之后）把活动槽拉回以
    **当前输入**为直流的稳态：复位后输出偏离新直流不超过线幅（那是线本身），30 ms 后只剩残差。

    复位成 0 或干脆不复位，直流跳变的瞬态（约 0.7 rad/s，振铃约 10 ms）会全权重灌进控制用陀螺。"""
    out, dc = _dc_jump_across(lib, trigger)
    assert float(np.max(np.abs(out[:30] - dc))) <= 0.12
    assert float(np.max(np.abs(out[30:] - dc))) <= 0.01


def test_no_output_step_is_larger_than_the_inputs_own(lib):
    """开、关、再开：输出每一步都不比输入自己的最大一步大（+1e-6）。"""
    fs, n = 1000.0, 1500
    t = np.arange(n) / fs
    line = 0.8 + 0.1 * np.sin(2 * np.pi * 107.5 * t)
    signal = np.repeat(line[:, None], 3, axis=1).astype(np.float32)
    ticks, gaps = tick_schedule(n)
    hz = np.full((n, 2), 107.5, np.float32)
    hz[:, 1] = 109.5
    fresh = np.ones((n, 2), np.uint8)
    fresh[500:800] = 0                       # 转速过期一段：淡出再重新捕获
    fresh_bank(lib, fs)
    out, _ = run_bank(lib, signal, ticks=ticks, hz=hz, fresh=fresh, dt=gaps / fs)
    max_in = float(np.max(np.abs(np.diff(signal[:, 0]))))
    assert float(np.max(np.abs(np.diff(out[:, 0])))) <= max_in + 1e-6


def test_stale_speed_fades_out_within_twenty_ms(lib):
    fs, n, stop = 1000.0, 400, 200
    signal = _noise(n)
    ticks, gaps = tick_schedule(n)
    hz = np.full((n, 2), 107.5, np.float32)
    fresh = np.zeros((n, 2), np.uint8)
    fresh[:stop, 0] = 1
    last_fresh = max(i for i in range(stop) if ticks[i])
    fresh_bank(lib, fs)
    _out, w = run_bank(lib, signal, ticks=ticks, hz=hz, fresh=fresh, dt=gaps / fs, weights=True)
    first_stale_tick = next(i for i in range(stop, n) if ticks[i])
    assert w[first_stale_tick - 1, 0] == 1.0
    # 过期那一拍起 20 个样本内归零、槽不再活动；距最后一次新鲜更新不超过 20 ms + 一拍。
    assert np.all(w[first_stale_tick + 20:, 0] == 0.0)
    assert (first_stale_tick + 20) - last_fresh <= 40


# ---------------------------------------------------------------- 频段与掩码


@pytest.mark.parametrize("hz,expected", [(40.0, 0.0), (50.0, 0.0), (55.0, 0.25), (60.0, 0.5),
                                         (70.0, 1.0), (107.5, 1.0), (380.0, 1.0)])
def test_low_fade_is_linear_between_min_and_min_plus_fade(lib, hz, expected):
    fresh_bank(lib)
    lib.harness_bank_set_motor(0, hz, 1, 0.002)
    assert lib.harness_slot_target(0) == pytest.approx(expected, abs=1e-5)


@pytest.mark.parametrize("fs", [1000.0, 800.0])
def test_top_fade_runs_from_40_to_45_percent_of_fs(lib, fs):
    fresh_bank(lib, fs)
    for fraction, expected in ((0.39, 1.0), (0.40, 1.0), (0.425, 0.5), (0.45, 0.0), (0.47, 0.0)):
        lib.harness_bank_set_motor(0, fraction * fs, 1, 0.002)
        assert lib.harness_slot_target(0) == pytest.approx(expected, abs=1e-4), fraction


@pytest.mark.parametrize("mask", [1, 2, 4, 3, 5, 7])
def test_the_mask_selects_harmonic_slots(lib, mask):
    fresh_bank(lib, mask=mask)
    lib.harness_bank_set_motor(0, 100.0, 1, 0.002)
    lib.harness_bank_set_motor(1, 102.0, 1, 0.002)
    for motor in (0, 1):
        for h in range(3):
            target = lib.harness_slot_target(motor * 3 + h)
            assert (target > 0.0) == bool(mask & (1 << h)), (motor, h)
            if mask & (1 << h):
                assert lib.harness_slot_center(motor * 3 + h) == pytest.approx(
                    (h + 1) * (100.0 if motor == 0 else 102.0))


# ---------------------------------------------------------------- 限速与看门狗


def _activate(lib, hz=100.0, samples=30):
    zero = (F * 3)()
    out = (F * 3)()
    lib.harness_bank_set_motor(0, hz, 1, 0.002)
    for _ in range(samples):
        lib.harness_bank_apply(zero, out)


def test_slew_is_clamped_while_active_and_counted(lib):
    fresh_bank(lib)
    _activate(lib)
    before = lib.harness_counter(0)
    lib.harness_bank_set_motor(0, 150.0, 1, 0.002)
    assert lib.harness_motor_hz(0) == pytest.approx(106.0, abs=1e-3)
    assert lib.harness_counter(0) == before + 1
    lib.harness_bank_set_motor(0, 150.0, 1, 0.002)
    assert lib.harness_motor_hz(0) == pytest.approx(112.0, abs=1e-3)


def test_reacquisition_after_fading_out_is_not_clamped(lib):
    fresh_bank(lib)
    _activate(lib)
    zero = (F * 3)()
    out = (F * 3)()
    lib.harness_bank_set_motor(0, 100.0, 0, 0.002)       # 过期：淡出
    for _ in range(25):
        lib.harness_bank_apply(zero, out)
    assert lib.harness_active_slots() == 0
    before = lib.harness_counter(0)
    lib.harness_bank_set_motor(0, 150.0, 1, 0.002)
    assert lib.harness_motor_hz(0) == pytest.approx(150.0)
    assert lib.harness_counter(0) == before


def test_watchdog_zeroes_every_target_once_per_trip(lib):
    fresh_bank(lib)
    _activate(lib, samples=5)
    zero = (F * 3)()
    out = (F * 3)()
    for _ in range(35):                                 # 连同激活的 5 个共 0.04·fs 个样本：不触发
        lib.harness_bank_apply(zero, out)
    assert lib.harness_counter(2) == 0 and lib.harness_slot_target(0) > 0.0
    for _ in range(30):
        lib.harness_bank_apply(zero, out)
    assert lib.harness_counter(2) == 1
    assert lib.harness_slot_target(0) == 0.0 and lib.harness_active_slots() == 0
    lib.harness_bank_set_motor(0, 100.0, 1, 0.002)       # 控制拍恢复：重新捕获，下一次再算一次
    for _ in range(80):
        lib.harness_bank_apply(zero, out)
    assert lib.harness_counter(2) == 2


# ---------------------------------------------------------------- 数值


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), float("-inf")])
def test_nonfinite_input_passes_through_and_recovers(lib, bad):
    fs, n = 1000.0, 600
    t = np.arange(n) / fs
    signal = np.repeat((0.5 + 0.1 * np.sin(2 * np.pi * 107.5 * t))[:, None], 3, axis=1).astype(np.float32)
    signal[300, 1] = bad
    ticks, gaps = tick_schedule(n)
    hz = np.full((n, 2), 107.5, np.float32)
    fresh = np.ones((n, 2), np.uint8)
    fresh_bank(lib, fs)
    out, _ = run_bank(lib, signal, ticks=ticks, hz=hz, fresh=fresh, dt=gaps / fs)
    assert out[300].tobytes() == signal[300].tobytes()      # 当拍原样放行
    assert np.all(np.isfinite(out[301:]))
    assert lib.harness_counter(1) == 1 and lib.harness_counter(3) == 1
    assert np.all(np.isfinite(out[400:]))


def test_sixty_seconds_of_zeros_leave_no_subnormal_state(lib):
    fresh_bank(lib, mask=7)
    assert lib.harness_zero_tail_has_subnormal(1000.0, 107.5, 60.0) == 0


def test_a_sub_tenth_hz_move_keeps_the_coefficients_bitwise(lib):
    fresh_bank(lib)
    lib.harness_bank_set_motor(0, 107.5, 1, 0.002)
    first = (F * 3)()
    lib.harness_slot_coef(0, first)
    lib.harness_bank_set_motor(0, 107.58, 1, 0.002)
    second = (F * 3)()
    lib.harness_slot_coef(0, second)
    assert bytes(first) == bytes(second)
    assert lib.harness_slot_center(0) == pytest.approx(107.5)
    lib.harness_bank_set_motor(0, 107.65, 1, 0.002)
    lib.harness_slot_coef(0, second)
    assert bytes(first) != bytes(second)


def test_a_fs_change_over_a_tenth_percent_redesigns_both_motors(lib):
    fresh_bank(lib)
    lib.harness_bank_set_motor(0, 107.5, 1, 0.002)
    lib.harness_bank_set_motor(1, 109.5, 1, 0.002)
    before = [(F * 3)(), (F * 3)()]
    lib.harness_slot_coef(0, before[0])
    lib.harness_slot_coef(3, before[1])
    lib.harness_bank_set_fs(1000.5)                     # 0.05%：不重算
    lib.harness_bank_set_motor(0, 107.5, 1, 0.002)
    now = (F * 3)()
    lib.harness_slot_coef(0, now)
    assert bytes(now) == bytes(before[0])
    lib.harness_bank_set_fs(1003.0)                     # 0.3%：两个电机都要重算
    lib.harness_bank_set_motor(0, 107.5, 1, 0.002)
    lib.harness_bank_set_motor(1, 109.5, 1, 0.002)
    lib.harness_slot_coef(0, now)
    assert bytes(now) != bytes(before[0])
    lib.harness_slot_coef(3, now)
    assert bytes(now) != bytes(before[1])

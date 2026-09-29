"""舵机回差逆补偿的算法层（Driver/Src/drv_servo_backlash.c）—— 在宿主上跑真 C 代码。

判据按"上机会出什么事"排：

* **方向对、量对**：一路往正走，走出阈值之后输出 = cmd + b；往负走 = cmd − b。
  复位后第一个样本、起始迟滞带里一律原样（不知道负载贴在空程哪一边）。
* **只在真换向时换**：从本方向极值退回超过阈值才换向，换向那一拍偏置从 +b 翻到 −b（跳 2b）。
  峰峰值低于阈值的噪声叠在慢斜坡上一次都不换；往回探 0.8 倍阈值的"假换向"也不换。
* **关掉就是原样**：enable = 0 时逐位直通（含 −0.0、次正规数）；非有限输入原样放行、清状态、计数。
* **参数守门**：半宽 [0, 0.087] rad，阈值 (0, 0.05] rad，NaN 一律不合法；不合法的配置装默认（关）。

仿真证据（同一驱动，Python 里接一个舵机 + 空程 + 负载模型）：3° 方波下负载基波幅值比、
相位滞后、稳态误差、10% 起动延迟在补偿后都变好；0.3°（6σ）陀螺样噪声不引起抖振，
去掉迟滞同样的噪声会翻几千次。模型参数取 2026-09-27 舵机单独三轮拟合（fit_servo.json）。
"""
from __future__ import annotations

import ctypes
import math
import os
import shutil
import struct
import subprocess
from pathlib import Path

import pytest

np = pytest.importorskip("numpy")

ROOT = Path(__file__).resolve().parents[1]

F = ctypes.c_float
U8 = ctypes.c_uint8

B_MEASURED = 0.0199           # rad，俯仰舵机回差半宽（三轮幅值依赖拟合）
B_CFG = 0.020                 # rad，固件默认 20 mrad
THR = 0.005                   # rad，固件默认迟滞
DT = 0.002                    # 500 Hz 控制拍


class Config(ctypes.Structure):
    _fields_ = [("half_gap_rad", F), ("threshold_rad", F), ("enable", U8)]


class Backlash(ctypes.Structure):
    _fields_ = [("cfg", Config), ("direction", ctypes.c_int8), ("primed", U8),
                ("reference_rad", F), ("reversal_count", ctypes.c_uint32),
                ("nonfinite_count", ctypes.c_uint32)]


def build_backlash_lib(tmp_path_factory, name="servo-backlash"):
    """gcc 编 DLL；-Werror，没有 gcc 就跳过（同 test_rpm_notch_filter）。"""
    compiler = shutil.which("gcc") or shutil.which("clang")
    if compiler is None:
        pytest.skip("host gcc/clang required for the servo backlash contract")
    build = tmp_path_factory.mktemp(name)
    out = build / ("backlash.dll" if os.name == "nt" else "backlash.so")
    result = subprocess.run(
        [compiler, "-shared", "-fPIC", "-std=c11", "-O1", "-Wall", "-Wextra", "-Werror",
         "-I", str(ROOT / "Driver" / "Inc"), str(ROOT / "Driver" / "Src" / "drv_servo_backlash.c"),
         "-lm", "-o", str(out)],
        capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr
    lib = ctypes.CDLL(str(out))
    lib.DRV_ServoBacklash_DefaultConfig.argtypes = [ctypes.POINTER(Config)]
    lib.DRV_ServoBacklash_ConfigValid.argtypes = [ctypes.POINTER(Config)]
    lib.DRV_ServoBacklash_ConfigValid.restype = U8
    lib.DRV_ServoBacklash_Init.argtypes = [ctypes.POINTER(Backlash), ctypes.POINTER(Config)]
    lib.DRV_ServoBacklash_Init.restype = U8
    lib.DRV_ServoBacklash_Reset.argtypes = [ctypes.POINTER(Backlash)]
    lib.DRV_ServoBacklash_Step.argtypes = [ctypes.POINTER(Backlash), F]
    lib.DRV_ServoBacklash_Step.restype = F
    return lib


@pytest.fixture(scope="module")
def lib(tmp_path_factory):
    return build_backlash_lib(tmp_path_factory)


def make(lib, *, b=B_CFG, thr=THR, enable=1) -> Backlash:
    bl = Backlash()
    assert lib.DRV_ServoBacklash_Init(ctypes.byref(bl), ctypes.byref(Config(b, thr, enable))) == 1
    return bl


def step(lib, bl, cmd) -> float:
    return lib.DRV_ServoBacklash_Step(ctypes.byref(bl), cmd)


def run(lib, bl, commands) -> np.ndarray:
    return np.array([step(lib, bl, float(c)) for c in commands], dtype=np.float32)


def f32(x) -> np.float32:
    return np.float32(x)


# ---------------------------------------------------------------- 方向与偏置


def test_the_first_sample_after_init_and_after_reset_carries_no_offset(lib):
    bl = make(lib)
    assert step(lib, bl, 0.1) == f32(0.1)
    assert bl.direction == 0 and bl.primed == 1
    for cmd in (0.1004, 0.0961, 0.1049):           # 起始迟滞带（±5 mrad）里：原样
        assert step(lib, bl, cmd) == f32(cmd)
    assert bl.direction == 0
    assert step(lib, bl, 0.106) == f32(f32(0.106) + f32(B_CFG))    # 走出阈值：定向 +1
    assert bl.direction == 1 and bl.reversal_count == 0, "首次定向不算换向"
    lib.DRV_ServoBacklash_Reset(ctypes.byref(bl))
    assert (bl.direction, bl.primed) == (0, 0)
    assert step(lib, bl, 0.3) == f32(0.3), "复位后第一个样本同样不加偏置"


@pytest.mark.parametrize("sign", [1.0, -1.0])
def test_a_steady_move_carries_the_offset_in_its_own_direction(lib, sign):
    bl = make(lib)
    ramp = (sign * np.arange(0.0, 0.08, 0.0013)).astype(np.float32)   # 没有样本正好落在阈值上
    out = run(lib, bl, ramp)
    moved = np.abs(ramp - ramp[0]) > THR
    np.testing.assert_array_equal(out[~moved], ramp[~moved])
    expected = (ramp[moved] + np.float32(sign * B_CFG)).astype(np.float32)
    np.testing.assert_array_equal(out[moved], expected)
    assert bl.direction == int(sign) and bl.reversal_count == 0


def test_a_real_reversal_flips_the_offset_by_exactly_two_b(lib):
    bl = make(lib)
    up = np.arange(0.0, 0.05, 0.002, dtype=np.float32)
    run(lib, bl, up)
    peak = float(up[-1])
    before = step(lib, bl, peak)
    assert before == f32(f32(peak) + f32(B_CFG))
    # 往回退 5 mrad 以内：还是 +b（空程还没开始走）。
    inside = f32(peak - 0.0049)
    assert step(lib, bl, float(inside)) == f32(inside + f32(B_CFG))
    assert bl.direction == 1
    # 退过阈值：这一拍换向，偏置从 +b 变 −b。
    cross = f32(peak - 0.0052)
    after = step(lib, bl, float(cross))
    assert after == f32(cross - f32(B_CFG))
    assert bl.direction == -1 and bl.reversal_count == 1
    assert (float(after) - float(cross)) - (float(before) - peak) == pytest.approx(-2 * B_CFG, abs=1e-6)
    assert bl.reference_rad == cross


# ---------------------------------------------------------------- 迟滞：噪声不换向


@pytest.mark.parametrize("seed", range(6))
@pytest.mark.parametrize("rate_rad_s", [0.0, 0.002, 0.02, -0.02])
def test_noise_below_the_threshold_on_a_slow_ramp_never_reverses(lib, seed, rate_rad_s):
    """峰峰值 0.8 倍阈值（±0.4·thr）的噪声：不管底下是静止还是慢斜坡，一次都不换向。"""
    rng = np.random.default_rng(seed)
    n = 3000
    base = 0.01 + rate_rad_s * DT * np.arange(n)
    noise = rng.uniform(-0.4 * THR, 0.4 * THR, n)
    bl = make(lib)
    out = run(lib, bl, (base + noise).astype(np.float32))
    assert bl.reversal_count == 0
    if rate_rad_s != 0.0:
        assert bl.direction == int(math.copysign(1, rate_rad_s))
        assert np.all(np.sign(out[-100:] - (base + noise)[-100:].astype(np.float32)) ==
                      math.copysign(1, rate_rad_s))


def test_probing_back_by_eighty_percent_of_the_threshold_is_not_a_reversal(lib):
    """慢斜坡上每隔几拍往回探 0.8·thr（相对本方向极值）：不换向；探 1.2·thr：换且只换一次。"""
    bl = make(lib)
    commands = []
    level = 0.0
    for k in range(200):
        level += 0.0003
        commands.append(level)
        if k % 7 == 3:
            commands.append(level - 0.8 * THR)
    run(lib, bl, np.asarray(commands, np.float32))
    assert bl.reversal_count == 0 and bl.direction == 1
    step(lib, bl, level - 1.2 * THR)
    assert bl.reversal_count == 1 and bl.direction == -1


def test_symmetric_noise_wider_than_the_threshold_does_reverse(lib):
    """反面：±0.8·thr（峰峰 1.6·thr）超过迟滞，静止时就会被判换向——阈值必须盖过指令噪声的峰峰值。"""
    rng = np.random.default_rng(3)
    bl = make(lib)
    run(lib, bl, rng.uniform(-0.8 * THR, 0.8 * THR, 3000).astype(np.float32))
    assert bl.reversal_count > 10


# ---------------------------------------------------------------- 关、坏数据、参数


def bits(values) -> list[int]:
    return [struct.unpack("<I", struct.pack("<f", float(v)))[0] for v in values]


def test_disabled_is_a_bitwise_pass_through_and_holds_no_direction(lib):
    bl = make(lib, enable=0)
    specials = [0.0, -0.0, 1e-42, -1e-42, 0.087, -0.3, 3.0e38, 0.01, 0.02, 0.03]
    values = np.asarray(specials, np.float32)
    out = [lib.DRV_ServoBacklash_Step(ctypes.byref(bl), F(v)) for v in values]
    assert bits(out) == bits(values)
    assert (bl.direction, bl.primed, bl.reversal_count) == (0, 0, 0)


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), float("-inf")])
def test_a_nonfinite_command_passes_through_resets_and_is_counted(lib, bad):
    bl = make(lib)
    run(lib, bl, np.arange(0.0, 0.03, 0.002, dtype=np.float32))
    assert bl.direction == 1
    out = step(lib, bl, bad)
    assert (math.isnan(out) and math.isnan(bad)) or out == bad
    assert bl.nonfinite_count == 1 and (bl.direction, bl.primed) == (0, 0)
    assert step(lib, bl, 0.05) == f32(0.05), "下一个有限样本按复位后第一个处理"


@pytest.mark.parametrize("b,thr,enable,valid", [
    (0.0, THR, 1, 1), (0.087, THR, 1, 1), (0.020, 0.05, 0, 1), (0.020, 1e-4, 1, 1),
    (-0.001, THR, 1, 0), (0.0871, THR, 1, 0), (float("nan"), THR, 1, 0), (float("inf"), THR, 1, 0),
    (0.020, 0.0, 1, 0), (0.020, -0.001, 1, 0), (0.020, 0.0501, 1, 0), (0.020, float("nan"), 1, 0),
    (0.020, THR, 2, 0),
])
def test_config_ranges(lib, b, thr, enable, valid):
    cfg = Config(b, thr, enable)
    assert lib.DRV_ServoBacklash_ConfigValid(ctypes.byref(cfg)) == valid
    bl = Backlash()
    bl.reversal_count = 7
    assert lib.DRV_ServoBacklash_Init(ctypes.byref(bl), ctypes.byref(cfg)) == valid
    assert bl.reversal_count == 0
    if not valid:
        default = Config()
        lib.DRV_ServoBacklash_DefaultConfig(ctypes.byref(default))
        assert (bl.cfg.half_gap_rad, bl.cfg.threshold_rad, bl.cfg.enable) == \
            (default.half_gap_rad, default.threshold_rad, 0)
        assert step(lib, bl, 0.123) == f32(0.123) and step(lib, bl, 0.2) == f32(0.2)


def test_the_default_config_is_off_with_the_documented_threshold(lib):
    cfg = Config()
    lib.DRV_ServoBacklash_DefaultConfig(ctypes.byref(cfg))
    assert (cfg.half_gap_rad, cfg.enable) == (0.0, 0)
    assert cfg.threshold_rad == f32(THR)


# ---------------------------------------------------------------- 仿真证据

#: 舵机模型：纯延迟 + 临界阻尼二阶。ωn、延迟取 10° 那一轮（回差对大幅值影响最小）的拟合：
#: ωn = 43.1 rad/s、纯延迟 43 ms。拟合出的 ζ = 0.66 是整条链（含空程）的表观值；
#: 无质量的空程负载会把舵盘的任何过冲原样"留住"，两种情况都被它污染，所以这里取 ζ = 1，
#: 让差别只来自空程本身。
SERVO_WN = 43.1
SERVO_ZETA = 1.0
SERVO_DELAY_S = 0.043


def servo_and_gap(horn_cmd, b=B_MEASURED, substeps=10):
    """舵盘 = 延迟 + 二阶；负载经 ±b 空程被舵盘推着走（play 算子）。返回负载角。"""
    delay = int(round(SERVO_DELAY_S / DT))
    dt = DT / substeps
    x = v = y = 0.0
    load = np.zeros(len(horn_cmd))
    for k in range(len(horn_cmd)):
        r = float(horn_cmd[k - delay]) if k >= delay else float(horn_cmd[0])
        for _ in range(substeps):
            v += (SERVO_WN * SERVO_WN * (r - x) - 2.0 * SERVO_ZETA * SERVO_WN * v) * dt
            x += v * dt
            if x - y > b:
                y = x - b
            elif x - y < -b:
                y = x + b
        load[k] = y
    return load


def square(amplitude, freq=2.0, seconds=8.0, lead_s=0.25):
    t = np.arange(int(seconds / DT)) * DT
    u = amplitude * np.sign(np.sin(2 * np.pi * freq * (t - lead_s)))
    u[t < lead_s] = 0.0
    return t, u.astype(np.float32)


def fundamental(t, u, y, freq, t0=2.0):
    """基波幅值比与相位滞后（度）；只取 t0 之后的整周期。"""
    m = t >= t0
    phasor = np.exp(-2j * np.pi * freq * t[m])
    uu, yy = np.sum(u[m] * phasor), np.sum(y[m] * phasor)
    return abs(yy) / abs(uu), -math.degrees(np.angle(yy / uu))


def edge_metrics(t, u, y, amplitude, t0=2.0, hold_s=0.25):
    """每个沿：负载起动到指令台阶 10% 的延迟；半周期末（沿前一拍）的稳态误差。"""
    delays, errors = [], []
    edges = [i for i in range(1, len(u)) if u[i] != u[i - 1] and t[i] >= t0]
    hold = int(round(hold_s / DT))
    for e in edges:
        errors.append(abs(float(y[e - 1] - u[e - 1])))
        direction = math.copysign(1.0, float(u[e] - u[e - 1]))
        start = y[e - 1]
        hit = next((j for j in range(e, min(e + hold, len(u)))
                    if direction * (y[j] - start) >= 0.2 * amplitude), None)
        if hit is not None:
            delays.append((hit - e) * DT)
    return float(np.mean(delays)) if delays else float("inf"), float(np.mean(errors))


def simulate(lib, amplitude, *, enable, noise=None, b_cfg=B_CFG, thr=THR):
    t, u = square(amplitude)
    command = u if noise is None else (u + noise[: len(u)]).astype(np.float32)
    bl = make(lib, b=b_cfg, thr=thr, enable=enable)
    horn = run(lib, bl, command)
    load = servo_and_gap(horn)
    ratio, lag = fundamental(t, u, load, 2.0)
    delay, error = edge_metrics(t, u, load, 2 * amplitude)
    edges = int(np.count_nonzero(np.diff(u) != 0))
    return dict(ratio=ratio, lag_deg=lag, delay_s=delay, error_rad=error,
                reversals=bl.reversal_count, edges=edges)


def test_simulated_three_degree_square_wave_improves_with_compensation(lib, capsys):
    """3° 方波（2 Hz，同舵机单独轮的双脉冲节奏）经舵机 + 1.14° 空程：补偿前后对照。"""
    a = math.radians(3.0)
    off = simulate(lib, a, enable=0)
    on = simulate(lib, a, enable=1)
    t, u = square(a)
    ideal_ratio, ideal_lag = fundamental(t, u, servo_and_gap(u, b=0.0), 2.0)
    with capsys.disabled():
        print(f"\n[backlash sim 3deg] ideal ratio={ideal_ratio:.3f} lag={ideal_lag:.1f}deg | "
              f"off ratio={off['ratio']:.3f} lag={off['lag_deg']:.1f}deg delay20={off['delay_s']*1e3:.0f}ms "
              f"err={math.degrees(off['error_rad']):.3f}deg | on ratio={on['ratio']:.3f} "
              f"lag={on['lag_deg']:.1f}deg delay20={on['delay_s']*1e3:.0f}ms "
              f"err={math.degrees(on['error_rad']):.3f}deg reversals={on['reversals']}/{on['edges']}")
    # 不补：负载只走 A − b（幅值比 ≈ 0.62 × 舵机自身增益），停在离指令 b 的地方。
    assert off["ratio"] < 0.65 * ideal_ratio
    assert off["error_rad"] == pytest.approx(B_MEASURED, rel=0.05)
    # 补：幅值回到舵机自身的水平，稳态误差只剩 b 配置与实测之差（0.1 mrad）。
    assert on["ratio"] == pytest.approx(ideal_ratio, rel=0.03)
    assert on["error_rad"] < 0.0005
    # 相位与起动延迟：补偿后更靠近没有空程的理想舵机，但追不平——穿过 2b 空程的那段路
    # 舵盘还是要自己走（补偿只是让它一拍就开始走），所以只要求明显变好、不要求追平。
    assert on["lag_deg"] < off["lag_deg"] - 2.0
    assert on["delay_s"] < off["delay_s"] - 0.004
    assert ideal_lag < on["lag_deg"]
    # 每个沿换一次向（首沿定向不算）。
    assert on["reversals"] == on["edges"] - 1


def test_simulated_small_corrections_inside_the_gap_only_move_the_load_when_compensated(lib, capsys):
    """悬停修正的量级（±1°，比 1.14° 的半宽还小）：不补时负载根本不动，补了才跟上。"""
    a = math.radians(1.0)
    off = simulate(lib, a, enable=0)
    on = simulate(lib, a, enable=1)
    with capsys.disabled():
        print(f"\n[backlash sim 1deg] off ratio={off['ratio']:.3f} | on ratio={on['ratio']:.3f} "
              f"lag={on['lag_deg']:.1f}deg")
    assert off["ratio"] < 0.05
    assert on["ratio"] > 0.85


def gyro_like_noise(n, sigma_rad, seed, corner_hz=30.0):
    """白噪声过一阶低通（30 Hz），按 σ 缩放；6σ 当峰峰值。"""
    rng = np.random.default_rng(seed)
    white = rng.standard_normal(n)
    alpha = math.exp(-2 * math.pi * corner_hz * DT)
    out = np.zeros(n)
    state = 0.0
    for k in range(n):
        state = alpha * state + (1.0 - alpha) * white[k]
        out[k] = state
    return out / np.std(out) * sigma_rad


@pytest.mark.parametrize("seed", range(3))
def test_simulated_gyro_like_noise_does_not_chatter(lib, seed, capsys):
    """0.3°（6σ，σ = 0.05°）陀螺样噪声叠在 3° 方波上：8 s 里换向数只比沿数多 0～2 次（孤立的尾部
    事件：6σ 已经略大于 0.29° 的阈值），去掉迟滞（阈值 10 µrad）同样的噪声翻两千多次。
    峰峰值低于阈值（σ = 0.03°，6σ = 0.18°）时一次不多——这是迟滞能保证的那一半。"""
    a = math.radians(3.0)
    n = int(8.0 / DT)
    noise = gyro_like_noise(n, math.radians(0.05), seed)
    with_hyst = simulate(lib, a, enable=1, noise=noise)
    no_hyst = simulate(lib, a, enable=1, noise=noise, thr=1e-5)
    quiet = simulate(lib, a, enable=1, noise=gyro_like_noise(n, math.radians(0.03), seed))
    extra = with_hyst["reversals"] - (with_hyst["edges"] - 1)
    with capsys.disabled():
        print(f"\n[backlash noise seed={seed}] 6sigma=0.30deg extra_reversals={extra} "
              f"(no hysteresis: {no_hyst['reversals']}) | 6sigma=0.18deg extra="
              f"{quiet['reversals'] - (quiet['edges'] - 1)} | ratio={with_hyst['ratio']:.3f}")
    assert 0 <= extra <= 4, "每秒不到半次的孤立翻转不算抖振"
    assert no_hyst["reversals"] > 1000
    assert quiet["reversals"] == quiet["edges"] - 1
    assert with_hyst["ratio"] == pytest.approx(simulate(lib, a, enable=1)["ratio"], rel=0.02)

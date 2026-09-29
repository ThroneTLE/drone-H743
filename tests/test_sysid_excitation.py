"""激励剖面契约。

最要紧的一条：**alpha_ff 必须真的是 omega_sp 的导数**。前馈力矩 τ=I·α_ff 全靠它，
它要是和实际角速度剖面对不上，辨出来的惯量就会系统性地偏，而波形看着完全正常。
"""
from __future__ import annotations

import ctypes
import os
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]

OK, INVALID = 0, 1
STEP, DOUBLET, CHIRP, PRBS = 0, 1, 2, 3


class Spec(ctypes.Structure):
    _fields_ = [
        ("profile", ctypes.c_uint8),
        ("amplitude_rad_s", ctypes.c_float),
        ("duration_ms", ctypes.c_uint32),
        ("hold_ms", ctypes.c_uint32),
        ("repeat", ctypes.c_uint32),
        ("ramp_ms", ctypes.c_uint32),
        ("chirp_f0_hz", ctypes.c_float),
        ("chirp_f1_hz", ctypes.c_float),
        ("prbs_bit_ms", ctypes.c_uint32),
        ("prbs_seed", ctypes.c_uint32),
    ]


class Sample(ctypes.Structure):
    _fields_ = [
        ("omega_sp_rad_s", ctypes.c_float),
        ("alpha_ff_rad_s2", ctypes.c_float),
        ("finished", ctypes.c_uint8),
    ]


@pytest.fixture(scope="module")
def lib(tmp_path_factory):
    compiler = shutil.which("gcc") or shutil.which("clang")
    if compiler is None:
        pytest.skip("host gcc/clang required for the sysid excitation contract")
    build = tmp_path_factory.mktemp("sysid-exc")
    out = build / ("exc.dll" if os.name == "nt" else "exc.so")
    result = subprocess.run(
        [compiler, "-shared", "-fPIC", "-std=c11", "-O2",
         "-Wall", "-Wextra", "-Werror", "-pedantic",
         "-I", str(ROOT / "Driver/Inc"),
         str(ROOT / "Driver/Src/drv_sysid_excitation.c"), "-lm", "-o", str(out)],
        capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr

    handle = ctypes.CDLL(str(out))
    handle.DRV_SysIdExcitation_Validate.argtypes = [ctypes.POINTER(Spec)]
    handle.DRV_SysIdExcitation_TotalMs.argtypes = [
        ctypes.POINTER(Spec), ctypes.POINTER(ctypes.c_uint32)]
    handle.DRV_SysIdExcitation_Eval.argtypes = [
        ctypes.POINTER(Spec), ctypes.c_uint32, ctypes.POINTER(Sample)]
    return handle


def spec(**kwargs):
    base = dict(profile=STEP, amplitude_rad_s=1.0, duration_ms=3000, hold_ms=500,
                repeat=2, ramp_ms=20, chirp_f0_hz=0.5, chirp_f1_hz=5.0,
                prbs_bit_ms=100, prbs_seed=1)
    base.update(kwargs)
    return Spec(**base)


def total_ms(lib, s):
    out = ctypes.c_uint32()
    assert lib.DRV_SysIdExcitation_TotalMs(ctypes.byref(s), ctypes.byref(out)) == OK
    return out.value


def evaluate(lib, s, t_ms):
    sample = Sample()
    status = lib.DRV_SysIdExcitation_Eval(ctypes.byref(s), t_ms, ctypes.byref(sample))
    assert status == OK
    return sample


def trace(lib, s):
    return [evaluate(lib, s, t) for t in range(total_ms(lib, s))]


ALL_PROFILES = [
    pytest.param(spec(profile=STEP, hold_ms=400, duration_ms=3000), id="step"),
    pytest.param(spec(profile=DOUBLET, hold_ms=300, repeat=3, duration_ms=3000), id="doublet"),
    pytest.param(spec(profile=CHIRP, duration_ms=2000, chirp_f0_hz=0.5, chirp_f1_hz=4.0), id="chirp"),
    pytest.param(spec(profile=PRBS, prbs_bit_ms=120, duration_ms=2400), id="prbs"),
]


# ---------------------------------------------------------------- 前馈即导数


PIECEWISE_LINEAR = [p for p in ALL_PROFILES if p.values[0].profile != CHIRP]


@pytest.mark.parametrize("s", PIECEWISE_LINEAR)
def test_alpha_ff_is_the_derivative_of_omega_sp(lib, s):
    """在每个线性段的**内部**，alpha_ff 必须等于 omega_sp 的差分。

    只查内部点：段边界上前后差分本来就不相等（斜坡起止是折点），那不是错误。
    """
    samples = trace(lib, s)
    checked = 0
    for index in range(1, len(samples) - 1):
        forward = (samples[index + 1].omega_sp_rad_s -
                   samples[index].omega_sp_rad_s) / 0.001
        backward = (samples[index].omega_sp_rad_s -
                    samples[index - 1].omega_sp_rad_s) / 0.001
        if abs(forward - backward) > 1e-3 * max(1.0, abs(forward)):
            continue  # 折点，跳过
        checked += 1
        assert samples[index].alpha_ff_rad_s2 == pytest.approx(
            forward, abs=2e-2, rel=2e-2), f"t={index}ms"
    assert checked > len(samples) // 2, f"可判定的内部点只有 {checked}/{len(samples)}"


def test_chirp_alpha_ff_is_the_analytic_derivative(lib):
    """chirp 是光滑函数，用中心差分核对解析导数。

    前向差分在这里不够：1 ms 步长下它的误差是 O(Δt·A·ω²)，4 Hz 时就有 0.3 rad/s²，
    足以淹没要查的东西。中心差分是 O(Δt²)，小两个量级。
    """
    s = spec(profile=CHIRP, duration_ms=2000, chirp_f0_hz=0.5, chirp_f1_hz=4.0,
             amplitude_rad_s=1.0)
    samples = trace(lib, s)
    for index in range(1, len(samples) - 1):
        central = (samples[index + 1].omega_sp_rad_s -
                   samples[index - 1].omega_sp_rad_s) / 0.002
        assert samples[index].alpha_ff_rad_s2 == pytest.approx(
            central, abs=0.05, rel=0.02), f"t={index}ms"


@pytest.mark.parametrize("s", ALL_PROFILES)
def test_no_profile_ever_demands_an_impulse(lib, s):
    """方波的导数是冲激，而冲激力矩做不出来——所有翻转都必须走有限斜坡。

    doublet/PRBS 的一次翻转是 -A → +A，电平变化 **2A**（不是 A），
    所以斜率上界也是两倍。这是真实的物理需求，不是放宽判据。
    """
    if s.profile == CHIRP:
        bound = s.amplitude_rad_s * 2.0 * 3.14159265 * s.chirp_f1_hz * 1.05
    elif s.profile == STEP:
        bound = s.amplitude_rad_s / (s.ramp_ms * 0.001) * 1.05
    else:
        bound = 2.0 * s.amplitude_rad_s / (s.ramp_ms * 0.001) * 1.05
    for sample in trace(lib, s):
        assert abs(sample.alpha_ff_rad_s2) <= bound


@pytest.mark.parametrize("s", ALL_PROFILES)
def test_amplitude_is_never_exceeded(lib, s):
    for sample in trace(lib, s):
        assert abs(sample.omega_sp_rad_s) <= s.amplitude_rad_s * 1.001


@pytest.mark.parametrize("s", ALL_PROFILES)
def test_eval_is_a_pure_function_of_time(lib, s):
    """同样的 (spec, t) 必须永远给同样的值——丢一拍不会让序列跑偏。"""
    for t in (0, 37, 211, 999):
        if t >= total_ms(lib, s):
            continue
        first = evaluate(lib, s, t)
        for _ in range(3):
            again = evaluate(lib, s, t)
            assert again.omega_sp_rad_s == first.omega_sp_rad_s
            assert again.alpha_ff_rad_s2 == first.alpha_ff_rad_s2


@pytest.mark.parametrize("s", ALL_PROFILES)
def test_finished_latches_past_the_end_and_output_is_zero(lib, s):
    end = total_ms(lib, s)
    for t in (end, end + 1, end + 5000):
        sample = evaluate(lib, s, t)
        assert sample.finished == 1
        assert sample.omega_sp_rad_s == 0.0
        assert sample.alpha_ff_rad_s2 == 0.0
    assert evaluate(lib, s, end - 1).finished == 0


# ---------------------------------------------------------------- 各剖面形状


def test_step_ramps_holds_and_returns(lib):
    s = spec(profile=STEP, amplitude_rad_s=1.0, hold_ms=400, ramp_ms=20)
    assert total_ms(lib, s) == 20 + 400 + 20
    assert evaluate(lib, s, 0).omega_sp_rad_s == pytest.approx(0.0, abs=1e-6)
    assert evaluate(lib, s, 10).omega_sp_rad_s == pytest.approx(0.5, rel=1e-3)
    assert evaluate(lib, s, 20).omega_sp_rad_s == pytest.approx(1.0, rel=1e-3)
    assert evaluate(lib, s, 300).omega_sp_rad_s == pytest.approx(1.0, rel=1e-3)
    assert evaluate(lib, s, 420).omega_sp_rad_s == pytest.approx(1.0, rel=1e-3)
    assert evaluate(lib, s, 430).omega_sp_rad_s == pytest.approx(0.5, rel=1e-2)
    assert evaluate(lib, s, 439).omega_sp_rad_s == pytest.approx(0.05, abs=0.06)


def test_doublet_alternates_sign_each_half(lib):
    s = spec(profile=DOUBLET, amplitude_rad_s=1.0, hold_ms=200, repeat=2, ramp_ms=20)
    assert total_ms(lib, s) == 200 * 2 * 2
    assert evaluate(lib, s, 100).omega_sp_rad_s == pytest.approx(1.0, rel=1e-3)
    assert evaluate(lib, s, 300).omega_sp_rad_s == pytest.approx(-1.0, rel=1e-3)
    assert evaluate(lib, s, 500).omega_sp_rad_s == pytest.approx(1.0, rel=1e-3)
    assert evaluate(lib, s, 700).omega_sp_rad_s == pytest.approx(-1.0, rel=1e-3)


def test_doublet_is_truncated_by_duration(lib):
    s = spec(profile=DOUBLET, hold_ms=300, repeat=10, duration_ms=1000)
    assert total_ms(lib, s) == 1000


def test_chirp_frequency_increases(lib):
    """扫频必须真的在扫：后段的过零间隔要比前段密。"""
    s = spec(profile=CHIRP, duration_ms=4000, chirp_f0_hz=1.0, chirp_f1_hz=8.0,
             amplitude_rad_s=1.0)
    samples = trace(lib, s)

    def crossings(lo, hi):
        count = 0
        for i in range(lo + 1, hi):
            if samples[i - 1].omega_sp_rad_s <= 0.0 < samples[i].omega_sp_rad_s:
                count += 1
        return count

    early = crossings(0, 1000)
    late = crossings(3000, 4000)
    assert late > early * 2, f"早段 {early} 次、晚段 {late} 次，没有扫频"


def test_prbs_is_reproducible_and_seed_dependent(lib):
    a = [x.omega_sp_rad_s for x in trace(lib, spec(profile=PRBS, prbs_seed=1,
                                                   prbs_bit_ms=100, duration_ms=2000))]
    b = [x.omega_sp_rad_s for x in trace(lib, spec(profile=PRBS, prbs_seed=1,
                                                   prbs_bit_ms=100, duration_ms=2000))]
    c = [x.omega_sp_rad_s for x in trace(lib, spec(profile=PRBS, prbs_seed=7,
                                                   prbs_bit_ms=100, duration_ms=2000))]
    assert a == b, "同种子必须完全可复现"
    assert a != c, "不同种子必须给出不同序列"


def test_prbs_visits_both_signs(lib):
    values = [x.omega_sp_rad_s for x in trace(lib, spec(
        profile=PRBS, prbs_seed=3, prbs_bit_ms=80, duration_ms=4000))]
    assert max(values) > 0.9
    assert min(values) < -0.9


# ---------------------------------------------------------------- 拒绝


@pytest.mark.parametrize("bad,reason", [
    (dict(profile=9), "未知剖面"),
    (dict(amplitude_rad_s=0.0), "幅值为零"),
    (dict(amplitude_rad_s=-1.0), "幅值为负"),
    (dict(amplitude_rad_s=99.0), "幅值超安全上限"),
    (dict(amplitude_rad_s=float("nan")), "幅值非有限"),
    (dict(ramp_ms=0), "斜坡为零会要求冲激力矩"),
    (dict(ramp_ms=1), "斜坡短于下限"),
    (dict(duration_ms=0), "时长为零"),
    (dict(duration_ms=999999), "时长超上限"),
    (dict(profile=STEP, hold_ms=5, ramp_ms=20), "平台短于斜坡，到不了幅值"),
    (dict(profile=DOUBLET, repeat=0), "重复次数为零"),
    (dict(profile=DOUBLET, repeat=999), "重复次数超上限"),
    (dict(profile=CHIRP, chirp_f0_hz=5.0, chirp_f1_hz=1.0), "终止频率低于起始"),
    (dict(profile=CHIRP, chirp_f0_hz=0.0), "起始频率为零"),
    (dict(profile=CHIRP, chirp_f1_hz=999.0), "终止频率超上限"),
    (dict(profile=PRBS, prbs_bit_ms=5, ramp_ms=20), "位宽短于斜坡"),
])
def test_validate_rejects(lib, bad, reason):
    assert lib.DRV_SysIdExcitation_Validate(ctypes.byref(spec(**bad))) == INVALID, reason


def test_validate_accepts_the_nominal_specs(lib):
    for param in ALL_PROFILES:
        assert lib.DRV_SysIdExcitation_Validate(ctypes.byref(param.values[0])) == OK


def test_eval_refuses_an_invalid_spec_instead_of_emitting_something(lib):
    sample = Sample()
    assert lib.DRV_SysIdExcitation_Eval(
        ctypes.byref(spec(amplitude_rad_s=0.0)), 0, ctypes.byref(sample)) == INVALID


# ---------------------------------------------------------------- 分层


def test_module_is_hardware_independent():
    import re

    source = (ROOT / "Driver/Src/drv_sysid_excitation.c").read_text(encoding="utf-8")
    header = (ROOT / "Driver/Inc/drv_sysid_excitation.h").read_text(encoding="utf-8")
    includes = re.findall(r'^#include\s+[<"]([^>"\n]+)', source + "\n" + header, re.M)
    assert set(includes) == {"drv_sysid_excitation.h", "stdint.h", "stddef.h", "math.h"}
    assert "HAL_" not in source
    assert "static float" not in source, "不允许文件级可变状态，Eval 必须是纯函数"
    assert "Driver/Src/drv_sysid_excitation.c" in (
        ROOT / "CMakeLists.txt").read_text(encoding="utf-8")

"""Host mirror and firmware system-ID excitation agree sample by sample.

主机镜像与固件激励**逐样本**一致。

为什么非要逐样本：上位机画出来的波形是操作员判断"这条命令对不对"的唯一依据。
两边算出不同的东西时，实机上的表现不是报错，是图上画的和飞机做的不是一回事——
而那种偏差会被原样记进辨识结果里当成模型误差。

判据覆盖四种剖面、若干组参数、以及 PRBS 的种子行为（小种子曾经互相撞车）。
"""
from __future__ import annotations

import ctypes
import math
import os
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from sysid.excitation import (  # noqa: E402
    PROFILE_CHIRP, PROFILE_DOUBLET, PROFILE_PRBS, PROFILE_STEP,
    Excitation, ExcitationInvalid, prbs_bit,
)
sys.path.pop(0)


class CSpec(ctypes.Structure):
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


class CSample(ctypes.Structure):
    _fields_ = [
        ("omega_sp_rad_s", ctypes.c_float),
        ("alpha_ff_rad_s2", ctypes.c_float),
        ("finished", ctypes.c_uint8),
    ]


@pytest.fixture(scope="module")
def lib(tmp_path_factory):
    compiler = shutil.which("gcc") or shutil.which("clang")
    if compiler is None:
        pytest.skip("host gcc/clang required for the excitation parity contract")
    build = tmp_path_factory.mktemp("sysid-exc-parity")
    out = build / ("exc.dll" if os.name == "nt" else "exc.so")
    result = subprocess.run(
        [compiler, "-shared", "-fPIC", "-std=c11", "-O2", "-Wall", "-Wextra",
         "-I", str(ROOT / "Driver/Inc"),
         str(ROOT / "Driver/Src/drv_sysid_excitation.c"), "-lm", "-o", str(out)],
        capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr

    handle = ctypes.CDLL(str(out))
    handle.DRV_SysIdExcitation_Validate.argtypes = [ctypes.POINTER(CSpec)]
    handle.DRV_SysIdExcitation_Validate.restype = ctypes.c_int
    handle.DRV_SysIdExcitation_TotalMs.argtypes = [
        ctypes.POINTER(CSpec), ctypes.POINTER(ctypes.c_uint32)]
    handle.DRV_SysIdExcitation_TotalMs.restype = ctypes.c_int
    handle.DRV_SysIdExcitation_Eval.argtypes = [
        ctypes.POINTER(CSpec), ctypes.c_uint32, ctypes.POINTER(CSample)]
    handle.DRV_SysIdExcitation_Eval.restype = ctypes.c_int
    return handle


def to_c(spec: Excitation) -> CSpec:
    return CSpec(profile=spec.profile, amplitude_rad_s=spec.amplitude_rad_s,
                 duration_ms=spec.duration_ms, hold_ms=spec.hold_ms,
                 repeat=spec.repeat, ramp_ms=spec.ramp_ms,
                 chirp_f0_hz=spec.chirp_f0_hz, chirp_f1_hz=spec.chirp_f1_hz,
                 prbs_bit_ms=spec.prbs_bit_ms, prbs_seed=spec.prbs_seed)


def c_total_ms(lib, spec: Excitation) -> int:
    value = ctypes.c_uint32()
    assert lib.DRV_SysIdExcitation_TotalMs(
        ctypes.byref(to_c(spec)), ctypes.byref(value)) == 0
    return value.value


def c_eval(lib, spec: Excitation, t_ms: int) -> tuple[float, float, bool]:
    sample = CSample()
    assert lib.DRV_SysIdExcitation_Eval(
        ctypes.byref(to_c(spec)), t_ms, ctypes.byref(sample)) == 0
    return sample.omega_sp_rad_s, sample.alpha_ff_rad_s2, bool(sample.finished)


SPECS = [
    Excitation(profile=PROFILE_STEP, amplitude_rad_s=1.2, duration_ms=1000,
               hold_ms=400, ramp_ms=40),
    Excitation(profile=PROFILE_STEP, amplitude_rad_s=0.4, duration_ms=300,
               hold_ms=100, ramp_ms=25),
    Excitation(profile=PROFILE_DOUBLET, amplitude_rad_s=2.0, duration_ms=4000,
               hold_ms=200, repeat=5, ramp_ms=20),
    Excitation(profile=PROFILE_DOUBLET, amplitude_rad_s=0.8, duration_ms=900,
               hold_ms=150, repeat=8, ramp_ms=30),
    Excitation(profile=PROFILE_CHIRP, amplitude_rad_s=1.0, duration_ms=5000,
               ramp_ms=10, chirp_f0_hz=0.5, chirp_f1_hz=8.0),
    Excitation(profile=PROFILE_CHIRP, amplitude_rad_s=0.6, duration_ms=2000,
               ramp_ms=10, chirp_f0_hz=1.0, chirp_f1_hz=15.0),
    Excitation(profile=PROFILE_PRBS, amplitude_rad_s=1.5, duration_ms=3000,
               ramp_ms=15, prbs_bit_ms=60, prbs_seed=1),
    Excitation(profile=PROFILE_PRBS, amplitude_rad_s=1.5, duration_ms=3000,
               ramp_ms=15, prbs_bit_ms=60, prbs_seed=7),
    Excitation(profile=PROFILE_PRBS, amplitude_rad_s=0.9, duration_ms=1500,
               ramp_ms=20, prbs_bit_ms=40, prbs_seed=123457),
]


@pytest.mark.parametrize("spec", SPECS, ids=lambda s: f"{s.profile_name}-{s.prbs_seed}")
def test_total_duration_matches_the_firmware(lib, spec):
    assert spec.total_ms() == c_total_ms(lib, spec)


def tolerances(spec: Excitation) -> tuple[float, float]:
    """两边允许的差，来自 f32 与 f64 的舍入，不是"调到能过"。

    分段线性的剖面只做几次乘加，差在 1e-6 量级。**扫频不一样**：相位是
    φ(T) = 2π·T·(f0+f1)/2 累积出来的，f0=1→f1=15 跑 2 秒就是约 100 rad，
    f32 的 1.2e-7 相对精度在这里就是 1e-5 rad 量级的绝对相位误差；相位是由
    `f0·t + 0.5·sweep·t²` 几次乘加算出来的，每一步各自舍入，所以按约 8 ULP
    （1e-6 相对）而不是 1 ULP 来估。

    顺带说明一件事：这个量级（1e-5 rad 相位）远低于辨识关心的尺度——
    我们要测的延迟是毫秒级，对应的相位在 0.1 rad 量级。f32 够用。
    """
    if spec.profile != PROFILE_CHIRP:
        return 1e-5, 1e-3
    span_s = spec.total_ms() * 0.001
    peak_phase = 2 * math.pi * span_s * (spec.chirp_f0_hz + spec.chirp_f1_hz) / 2
    phase_error = peak_phase * 1.0e-6
    omega_tol = spec.amplitude_rad_s * phase_error + 1e-6
    alpha_tol = (spec.amplitude_rad_s * 2 * math.pi * spec.chirp_f1_hz
                 * phase_error + 1e-4)
    return omega_tol, alpha_tol


@pytest.mark.parametrize("spec", SPECS, ids=lambda s: f"{s.profile_name}-{s.prbs_seed}")
def test_every_millisecond_matches_the_firmware(lib, spec):
    omega_tol, alpha_tol = tolerances(spec)
    total = spec.total_ms()
    for t_ms in range(0, total + 20):
        host = spec.eval(t_ms)
        target = c_eval(lib, spec, t_ms)
        assert host[2] == target[2], f"t={t_ms} finished 不一致"
        assert host[0] == pytest.approx(target[0], abs=omega_tol, rel=1e-5), f"ω t={t_ms}"
        assert host[1] == pytest.approx(target[1], abs=alpha_tol, rel=1e-4), f"α t={t_ms}"


@pytest.mark.parametrize("spec", SPECS, ids=lambda s: f"{s.profile_name}-{s.prbs_seed}")
def test_validation_agrees_with_the_firmware(lib, spec):
    assert lib.DRV_SysIdExcitation_Validate(ctypes.byref(to_c(spec))) == 0
    spec.validate()


@pytest.mark.parametrize("broken", [
    Excitation(profile=PROFILE_STEP, ramp_ms=0, hold_ms=100, duration_ms=500),
    Excitation(profile=PROFILE_STEP, ramp_ms=50, hold_ms=10, duration_ms=500),
    Excitation(profile=PROFILE_DOUBLET, repeat=0, hold_ms=100, ramp_ms=20),
    Excitation(profile=PROFILE_DOUBLET, repeat=99, hold_ms=100, ramp_ms=20),
    Excitation(profile=PROFILE_CHIRP, chirp_f0_hz=5.0, chirp_f1_hz=1.0, ramp_ms=10),
    Excitation(profile=PROFILE_PRBS, prbs_bit_ms=5, ramp_ms=20),
    Excitation(amplitude_rad_s=0.0, ramp_ms=20, hold_ms=100),
    Excitation(amplitude_rad_s=99.0, ramp_ms=20, hold_ms=100),
])
def test_both_sides_reject_the_same_broken_specs(lib, broken):
    """两边的体检必须同进同退。主机放行而固件拒绝 = 「按了没反应」。"""
    assert lib.DRV_SysIdExcitation_Validate(ctypes.byref(to_c(broken))) != 0
    with pytest.raises(ExcitationInvalid):
        broken.validate()


def test_prbs_seeds_that_used_to_collide_now_differ(lib):
    """seed=1 与 seed=7 曾经给出**完全相同**的序列。

    小种子的高 30 位全是 0，左移式 LFSR 要 30 拍才把状态填满，而一趟激励也就
    几十位。两次实验以为换了激励其实没换——数据里看不出来。
    固件先把种子乘 Knuth 常数散开并取最高位，主机镜像必须照抄。
    """
    a = [prbs_bit(1, i) for i in range(64)]
    b = [prbs_bit(7, i) for i in range(64)]
    assert a != b
    assert 0 in a and 1 in a, "前 64 位应当两种电平都有"


def test_no_profile_demands_an_impulse(lib):
    """方波的导数是冲激，而冲激力矩做不出来：前馈会要求无穷大的倾角。

    所有翻转都摊在 ramp_ms 上，所以 α_ff 处处有界。
    """
    for spec in SPECS:
        if spec.profile == PROFILE_CHIRP:
            bound = spec.amplitude_rad_s * 2 * math.pi * spec.chirp_f1_hz * 1.1
        else:
            # doublet / prbs 的一次翻转是 −A→+A，落差 2A。
            swing = spec.amplitude_rad_s * (1 if spec.profile == PROFILE_STEP else 2)
            bound = swing / (spec.ramp_ms * 0.001) * 1.05
        for t_ms in range(0, spec.total_ms()):
            _omega, alpha, _done = c_eval(lib, spec, t_ms)
            assert abs(alpha) <= bound, f"{spec.profile_name} t={t_ms} α={alpha}"

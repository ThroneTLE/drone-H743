"""光杆台架几何契约。

整套内环辨识建在一句话上：**绕 45° 杆轴辨出来的惯量就是 Ixx**。
这里用真实 C 把它钉死，连同"结果符不符合"的机检判据（角速度必须平行于杆轴）。
"""
from __future__ import annotations

import ctypes
import math
import os
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]

OK, INVALID = 0, 1
PSI45 = math.pi / 4.0


class Rig(ctypes.Structure):
    _fields_ = [
        ("azimuth_rad", ctypes.c_float),
        ("axis_offset_above_cg_m", ctypes.c_float),
        ("imu_above_cg_m", ctypes.c_float),
    ]


@pytest.fixture(scope="module")
def lib(tmp_path_factory):
    compiler = shutil.which("gcc") or shutil.which("clang")
    if compiler is None:
        pytest.skip("host gcc/clang required for the sysid rig contract")
    build = tmp_path_factory.mktemp("sysid-rig")
    out = build / ("rig.dll" if os.name == "nt" else "rig.so")
    result = subprocess.run(
        [compiler, "-shared", "-fPIC", "-std=c11", "-O2",
         "-Wall", "-Wextra", "-Werror", "-pedantic",
         "-I", str(ROOT / "Driver/Inc"),
         str(ROOT / "Driver/Src/drv_sysid_rig.c"), "-lm", "-o", str(out)],
        capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr

    handle = ctypes.CDLL(str(out))
    f3 = ctypes.c_float * 3
    handle.DRV_SysIdRig_Axis.argtypes = [ctypes.POINTER(Rig), f3]
    handle.DRV_SysIdRig_EffectiveInertia.argtypes = [
        ctypes.POINTER(Rig), f3, ctypes.POINTER(ctypes.c_float)]
    handle.DRV_SysIdRig_ProjectRate.argtypes = [
        ctypes.POINTER(Rig), f3, ctypes.POINTER(ctypes.c_float)]
    handle.DRV_SysIdRig_RateResidual.argtypes = [
        ctypes.POINTER(Rig), f3, ctypes.POINTER(ctypes.c_float)]
    handle.DRV_SysIdRig_MomentAboutAxis.argtypes = [
        ctypes.POINTER(Rig), ctypes.c_float, f3]
    handle.DRV_SysIdRig_GravityMoment.argtypes = [
        ctypes.POINTER(Rig), ctypes.c_float, ctypes.c_float, ctypes.c_float,
        ctypes.POINTER(ctypes.c_float)]
    return handle


def rig(azimuth=PSI45, offset=0.0, imu=0.0):
    return Rig(azimuth_rad=azimuth, axis_offset_above_cg_m=offset, imu_above_cg_m=imu)


def call_scalar(fn, r, vec):
    out = ctypes.c_float()
    status = fn(ctypes.byref(r), (ctypes.c_float * 3)(*vec), ctypes.byref(out))
    return status, out.value


# ---------------------------------------------------------------- 惯量


@pytest.mark.parametrize("azimuth_deg", [0, 30, 45, 60, 90, 135, 180])
def test_effective_inertia_equals_ixx_for_any_azimuth_when_xy_symmetric(lib, azimuth_deg):
    """这条是全套辨识的地基：对称机体下，绕**任意**水平轴的有效惯量都等于 Ixx。

    所以 45° 光杆辨出来的数可以直接当 Ixx/Iyy 用，不必分轴做。
    """
    r = rig(azimuth=math.radians(azimuth_deg))
    status, value = call_scalar(lib.DRV_SysIdRig_EffectiveInertia, r,
                                [0.019, 0.019, 0.00035])
    assert status == OK
    assert value == pytest.approx(0.019, rel=1e-5)


def test_effective_inertia_is_azimuth_dependent_when_xy_differ(lib):
    """Ixx != Iyy 时它就**不再**与方位角无关——对称假设是有代价的，别当普适。"""
    at0 = call_scalar(lib.DRV_SysIdRig_EffectiveInertia, rig(azimuth=0.0),
                      [0.010, 0.030, 0.0004])[1]
    at45 = call_scalar(lib.DRV_SysIdRig_EffectiveInertia, rig(azimuth=PSI45),
                       [0.010, 0.030, 0.0004])[1]
    at90 = call_scalar(lib.DRV_SysIdRig_EffectiveInertia, rig(azimuth=math.pi / 2),
                       [0.010, 0.030, 0.0004])[1]
    assert at0 == pytest.approx(0.010, rel=1e-5)
    assert at90 == pytest.approx(0.030, rel=1e-5)
    assert at45 == pytest.approx(0.020, rel=1e-5)  # (Ixx+Iyy)/2


def test_izz_never_participates_because_the_rod_is_horizontal(lib):
    """杆在 XY 平面内，n_z = 0，所以 Izz 怎么变都不影响绕杆惯量。"""
    baseline = call_scalar(lib.DRV_SysIdRig_EffectiveInertia, rig(),
                           [0.019, 0.019, 0.00035])[1]
    inflated = call_scalar(lib.DRV_SysIdRig_EffectiveInertia, rig(),
                           [0.019, 0.019, 99.0])[1]
    assert baseline == pytest.approx(inflated, rel=1e-6)


def test_nonpositive_inertia_is_rejected(lib):
    for inertia in ([0.0, 0.019, 0.0004], [0.019, -1.0, 0.0004]):
        status, _ = call_scalar(lib.DRV_SysIdRig_EffectiveInertia, rig(), inertia)
        assert status == INVALID


# ---------------------------------------------------------------- 轴与投影


def test_axis_is_a_horizontal_unit_vector(lib):
    axis = (ctypes.c_float * 3)()
    r = rig()
    assert lib.DRV_SysIdRig_Axis(ctypes.byref(r), axis) == OK
    assert math.hypot(axis[0], axis[1]) == pytest.approx(1.0, rel=1e-6)
    assert axis[2] == 0.0
    assert axis[0] == pytest.approx(math.sqrt(0.5), rel=1e-6)
    assert axis[1] == pytest.approx(math.sqrt(0.5), rel=1e-6)


def test_rate_along_the_rod_projects_fully_and_leaves_no_residual(lib):
    """理想台架：ω 平行于杆轴，垂直残差为 0。"""
    magnitude = 2.5
    omega = [magnitude * math.sqrt(0.5), magnitude * math.sqrt(0.5), 0.0]
    assert call_scalar(lib.DRV_SysIdRig_ProjectRate, rig(), omega)[1] == pytest.approx(
        magnitude, rel=1e-5)
    assert call_scalar(lib.DRV_SysIdRig_RateResidual, rig(), omega)[1] == pytest.approx(
        0.0, abs=1e-5)


def test_rate_across_the_rod_is_all_residual(lib):
    """垂直于杆的转动在真台架上不可能发生；出现了就说明杆没夹紧或轴向填错。"""
    magnitude = 2.0
    omega = [magnitude * math.sqrt(0.5), -magnitude * math.sqrt(0.5), 0.0]
    assert call_scalar(lib.DRV_SysIdRig_ProjectRate, rig(), omega)[1] == pytest.approx(
        0.0, abs=1e-5)
    assert call_scalar(lib.DRV_SysIdRig_RateResidual, rig(), omega)[1] == pytest.approx(
        magnitude, rel=1e-5)


def test_yaw_rate_is_entirely_residual(lib):
    """绕 Z 的转动同样是杆约束不允许的，必须整条计入残差。"""
    assert call_scalar(lib.DRV_SysIdRig_RateResidual, rig(), [0.0, 0.0, 1.5])[1] == \
        pytest.approx(1.5, rel=1e-5)


def test_residual_and_projection_obey_pythagoras(lib):
    omega = [0.7, -0.2, 0.4]
    along = call_scalar(lib.DRV_SysIdRig_ProjectRate, rig(), omega)[1]
    residual = call_scalar(lib.DRV_SysIdRig_RateResidual, rig(), omega)[1]
    assert math.hypot(along, residual) == pytest.approx(
        math.sqrt(sum(v * v for v in omega)), rel=1e-5)


# ---------------------------------------------------------------- 力矩


def test_moment_about_axis_points_along_the_rod_with_the_right_magnitude(lib):
    moment = (ctypes.c_float * 3)()
    r = rig()
    assert lib.DRV_SysIdRig_MomentAboutAxis(ctypes.byref(r), 0.4, moment) == OK
    assert math.sqrt(sum(moment[i] ** 2 for i in range(3))) == pytest.approx(0.4, rel=1e-5)
    # 45° 台架上两个水平分量必须相等——这就是"两个舵机联动"的由来
    assert moment[0] == pytest.approx(moment[1], rel=1e-5)
    assert moment[2] == 0.0


def test_negative_moment_flips_the_vector(lib):
    positive = (ctypes.c_float * 3)()
    negative = (ctypes.c_float * 3)()
    r = rig()
    lib.DRV_SysIdRig_MomentAboutAxis(ctypes.byref(r), 0.4, positive)
    lib.DRV_SysIdRig_MomentAboutAxis(ctypes.byref(r), -0.4, negative)
    for i in range(3):
        assert negative[i] == pytest.approx(-positive[i], rel=1e-5)


# ---------------------------------------------------------------- 重力


def test_no_gravity_moment_when_the_rod_passes_through_the_cg(lib):
    """名义台架：杆过质心，系统退化成纯双积分，没有回中趋势。"""
    out = ctypes.c_float()
    r = rig(offset=0.0)
    for angle in (-0.3, 0.0, 0.26):
        assert lib.DRV_SysIdRig_GravityMoment(
            ctypes.byref(r), 1.367, 9.81, angle, ctypes.byref(out)) == OK
        assert out.value == pytest.approx(0.0, abs=1e-9)


def test_rod_above_cg_gives_a_restoring_moment(lib):
    """偏心不为零时是稳定单摆：恢复力矩与转角反号。"""
    out = ctypes.c_float()
    r = rig(offset=0.05)
    lib.DRV_SysIdRig_GravityMoment(ctypes.byref(r), 1.367, 9.81, 0.2, ctypes.byref(out))
    restoring = out.value
    assert restoring < 0.0
    assert restoring == pytest.approx(-1.367 * 9.81 * 0.05 * math.sin(0.2), rel=1e-5)

    lib.DRV_SysIdRig_GravityMoment(ctypes.byref(r), 1.367, 9.81, -0.2, ctypes.byref(out))
    assert out.value == pytest.approx(-restoring, rel=1e-5)


def test_gravity_moment_rejects_unphysical_inputs(lib):
    out = ctypes.c_float()
    r = rig(offset=0.05)
    assert lib.DRV_SysIdRig_GravityMoment(
        ctypes.byref(r), 0.0, 9.81, 0.1, ctypes.byref(out)) == INVALID
    assert lib.DRV_SysIdRig_GravityMoment(
        ctypes.byref(r), 1.367, 0.0, 0.1, ctypes.byref(out)) == INVALID


def test_nan_inputs_are_rejected_rather_than_propagated(lib):
    status, _ = call_scalar(lib.DRV_SysIdRig_ProjectRate, rig(),
                            [float("nan"), 0.0, 0.0])
    assert status == INVALID
    status, _ = call_scalar(lib.DRV_SysIdRig_EffectiveInertia,
                            rig(azimuth=float("inf")), [0.019, 0.019, 0.0004])
    assert status == INVALID


# ---------------------------------------------------------------- 分层


def test_module_is_hardware_independent():
    import re

    source = (ROOT / "Driver/Src/drv_sysid_rig.c").read_text(encoding="utf-8")
    header = (ROOT / "Driver/Inc/drv_sysid_rig.h").read_text(encoding="utf-8")
    includes = re.findall(r'^#include\s+[<"]([^>"\n]+)', source + "\n" + header, re.M)
    assert set(includes) == {"drv_sysid_rig.h", "stdint.h", "stddef.h", "math.h"}
    assert "HAL_" not in source
    assert "htim" not in source
    assert "Driver/Src/drv_sysid_rig.c" in (
        ROOT / "CMakeLists.txt").read_text(encoding="utf-8")

"""力矩 -> 倾角反解的公开出口契约。

辨识不能自己再写一份力学：它算出的倾角必须**和在飞的分配器逐位一致**。
所以这里编译真实的 `drv_coax_ctrl.c`，验证公开包装与内部反解器同源，
并钉住零推力下的安全行为。
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

SOURCES = [
    "Driver/Src/drv_coax_ctrl.c",
    "Driver/Src/drv_airframe_params.c",
    "Driver/Src/drv_prop_map.c",
    "Driver/Src/drv_position_control.c",
    "Driver/Src/drv_attitude_control.c",
    "Driver/Src/drv_rate_control.c",
]

# 与 drv_airframe_params.h 的 DRV_Airframe_Params 逐字段对应（顺序即 ABI）。
AIRFRAME_FIELDS = [
    "board_mass_g", "battery_mass_g", "base_mass_g", "servo_motor_mass_g",
    "board_cg_z_m", "battery_cg_z_m", "base_cg_z_m", "servo_motor_cg_z_m",
    "imu_z_m", "prop_plane_d_m", "roll_axis_to_prop_plane_m",
    "pitch_axis_to_prop_plane_m", "retired_pitch_thrust_lever_arm_m",
    "retired_roll_thrust_lever_arm_m", "servo1_axis_z_m", "servo2_axis_z_m",
    "thrust_point_z_m", "tether_attach_z_m", "tether_rope_m",
    "ixx_kgm2", "iyy_kgm2", "izz_kgm2", "lower_rotor_spin_sense",
    "gravity_m_s2", "max_total_thrust_g", "servo_deg_per_us",
    "mass_kg", "cg_z_m", "weight_n", "thrust_point_to_cg_z_m",
    "tether_attach_to_cg_m", "tether_rod_to_cg_m", "max_total_force_n",
    "hover_thrust_percent", "servo_us_per_deg", "derived_auto",
    # CFG v26（2026-09-29，R-FLOWMOUNT-1）：尾部追加的光流安装两项。少了它们 ctypes 结构
    # 比 C 结构短 8 字节，C 侧整体读写会越过 Python 分配的缓冲。
    "flow_mount_yaw_deg", "flow_mount_mirror",
]


class Airframe(ctypes.Structure):
    _fields_ = [(name, ctypes.c_float) for name in AIRFRAME_FIELDS]


@pytest.fixture(scope="module")
def lib(tmp_path_factory):
    compiler = shutil.which("gcc") or shutil.which("clang")
    if compiler is None:
        pytest.skip("host gcc/clang required for the moment-inverse contract")
    build = tmp_path_factory.mktemp("sysid-inverse")
    out = build / ("coax.dll" if os.name == "nt" else "coax.so")
    result = subprocess.run(
        [compiler, "-shared", "-fPIC", "-std=c11", "-O2", "-Wall", "-Wextra",
         "-I", str(ROOT / "Driver/Inc"), "-I", str(ROOT / "BSP/Inc"),
         *[str(ROOT / path) for path in SOURCES], "-lm", "-o", str(out)],
        capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr

    handle = ctypes.CDLL(str(out))
    f3 = ctypes.c_float * 3
    handle.DRV_COAX_CTRL_SolveBodyTiltFromMoment.argtypes = [
        f3, ctypes.c_float, ctypes.POINTER(ctypes.c_float),
        ctypes.POINTER(ctypes.c_float)]
    handle.DRV_COAX_CTRL_SolveBodyTiltFromMoment.restype = ctypes.c_uint8
    handle.DRV_COAX_CTRL_SetParams.argtypes = [ctypes.c_void_p]
    handle.DRV_Airframe_SetParams.argtypes = [ctypes.POINTER(Airframe)]
    handle.DRV_Airframe_IsValid.restype = ctypes.c_uint8
    handle.DRV_COAX_CTRL_GetParam.argtypes = [
        ctypes.c_char_p, ctypes.POINTER(ctypes.c_float)]
    handle.DRV_COAX_CTRL_GetParam.restype = ctypes.c_uint8
    handle.DRV_COAX_CTRL_BodyTiltRadToServoPulses.argtypes = [
        ctypes.c_float, ctypes.c_float,
        ctypes.POINTER(ctypes.c_uint16), ctypes.POINTER(ctypes.c_uint16)]

    # 机体模型只从 Flash 来；宿主测试必须先写一份，否则力臂为 0、力矩恒为 0。
    frame = Airframe()
    frame.board_mass_g = 75.0
    frame.battery_mass_g = 232.0
    frame.base_mass_g = 99.0
    frame.servo_motor_mass_g = 348.6
    frame.battery_cg_z_m = 0.109
    frame.base_cg_z_m = -0.117
    frame.servo_motor_cg_z_m = -0.244
    frame.imu_z_m = 0.0
    frame.prop_plane_d_m = 0.250
    frame.roll_axis_to_prop_plane_m = 0.145
    frame.pitch_axis_to_prop_plane_m = 0.105
    # 2026-09-27 当晚板上几何：两个倾转舵机转轴都在 z = -0.13 m，部件表算出的
    # 重心 -0.094558 m，力臂 r_z = -0.035442 m（作者随后实测重心约 -0.01 m，
    # 那时力臂约 0.12 m；这里只当一组合法几何用）。退役的"推力力臂"不再填。
    frame.servo1_axis_z_m = -0.13
    frame.servo2_axis_z_m = -0.13
    frame.thrust_point_z_m = -0.2955
    frame.ixx_kgm2 = 0.019
    frame.iyy_kgm2 = 0.019
    frame.izz_kgm2 = 0.00035
    frame.lower_rotor_spin_sense = -1.0
    frame.gravity_m_s2 = 9.81
    frame.max_total_thrust_g = 1595.342
    frame.servo_deg_per_us = 0.09
    frame.derived_auto = 1.0
    handle.DRV_Airframe_SetParams(ctypes.byref(frame))
    assert handle.DRV_Airframe_IsValid() == 1, "机体模型未通过体检，后续判据无意义"
    return handle


def solve(lib, moment, force_n):
    body_x = ctypes.c_float()
    body_y = ctypes.c_float()
    ok = lib.DRV_COAX_CTRL_SolveBodyTiltFromMoment(
        (ctypes.c_float * 3)(*moment), force_n,
        ctypes.byref(body_x), ctypes.byref(body_y))
    return ok, body_x.value, body_y.value


def tilt_limit(lib):
    value = ctypes.c_float()
    assert lib.DRV_COAX_CTRL_GetParam(b"coax.tilt_limit_rad", ctypes.byref(value)) == 1
    return value.value


# ---------------------------------------------------------------- 零推力守卫


@pytest.mark.parametrize("force", [0.0, -1.0, 0.4, float("nan")])
def test_no_thrust_returns_zero_tilt_instead_of_full_deflection(lib, force):
    """这是本次新加的守卫，单独钉死。

    推力为零时力矩的可达区间退化成一个点，二分会一路收敛到 -tilt_limit——
    也就是"没有推力反而把舵机打到底"。必须返回 0 并报失败。
    """
    ok, body_x, body_y = solve(lib, [0.3, 0.3, 0.0], force)
    assert ok == 0
    assert body_x == 0.0
    assert body_y == 0.0


@pytest.mark.parametrize("variant", ["axes_unset", "lever_zero"])
def test_invalid_airframe_centres_the_servos_instead_of_solving(lib, variant):
    """机体模型无效时反解回中并报失败，正向力矩记 0。

    axes_unset 是 2026-09-27 板上的实际状态：两个舵机转轴存的是 0，力臂成了
    重心 − 0 = −0.0946 m，符号是反的——照样反解，手扳核对舵机方向就会看起来
    "反了"。lever_zero 让转轴正好落在重心上，二分以前会一路顶到负限位还报成功。
    """
    lib.DRV_Airframe_Get.restype = ctypes.POINTER(Airframe)
    lib.DRV_COAX_CTRL_MomentFromServoPulses.argtypes = [
        ctypes.c_float, ctypes.c_uint16, ctypes.c_uint16,
        ctypes.POINTER(ctypes.c_float)]
    lib.DRV_COAX_CTRL_MomentFromServoPulses.restype = ctypes.c_uint8
    original = Airframe.from_buffer_copy(lib.DRV_Airframe_Get().contents)
    try:
        broken = Airframe.from_buffer_copy(original)
        if variant == "axes_unset":
            broken.servo1_axis_z_m = 0.0
            broken.servo2_axis_z_m = 0.0
        else:
            broken.derived_auto = 0.0
            broken.servo1_axis_z_m = original.cg_z_m
            broken.servo2_axis_z_m = original.cg_z_m
        lib.DRV_Airframe_SetParams(ctypes.byref(broken))
        assert lib.DRV_Airframe_IsValid() == 0

        assert solve(lib, [0.05, -0.04, 0.0], 13.4) == (0, 0.0, 0.0)
        moment = (ctypes.c_float * 3)(1.0, 1.0, 1.0)
        assert lib.DRV_COAX_CTRL_MomentFromServoPulses(13.4, 1600, 1400, moment) == 0
        assert list(moment) == [0.0, 0.0, 0.0]
    finally:
        lib.DRV_Airframe_SetParams(ctypes.byref(original))
    assert lib.DRV_Airframe_IsValid() == 1
    assert solve(lib, [0.05, -0.04, 0.0], 13.4)[0] == 1


def test_sufficient_thrust_reports_success(lib):
    ok, _, _ = solve(lib, [0.2, 0.2, 0.0], 13.4)
    assert ok == 1


def test_inverse_refreshes_airframe_after_model_update(lib):
    lib.DRV_Airframe_Get.restype = ctypes.POINTER(Airframe)
    original = Airframe.from_buffer_copy(lib.DRV_Airframe_Get().contents)
    try:
        # 0.051 -> 0.02 N*m: with this fixture's 0.035 m lever (was 0.083 m
        # effective), 0.051 N*m at 4 N already hits the tilt limit once the lever
        # is halved.
        _, before, _ = solve(lib, [0, 0.02, 0], 4.0)
        changed = Airframe.from_buffer_copy(original)
        # 力臂减半 = 2 号舵机转轴向重心挪近一半（力臂是转轴 z - 重心 z，不是输入字段）。
        changed.servo2_axis_z_m = original.cg_z_m + 0.5 * (
            original.servo2_axis_z_m - original.cg_z_m)
        lib.DRV_Airframe_SetParams(ctypes.byref(changed))
        assert lib.DRV_Airframe_IsValid() == 1
        _, after, _ = solve(lib, [0, 0.02, 0], 4.0)
        assert abs(after) > abs(before) * 1.8
    finally:
        lib.DRV_Airframe_SetParams(ctypes.byref(original))


# ---------------------------------------------------------------- 反解正确性


def test_zero_moment_gives_zero_tilt(lib):
    ok, body_x, body_y = solve(lib, [0.0, 0.0, 0.0], 13.4)
    assert ok == 1
    assert body_x == pytest.approx(0.0, abs=2e-3)
    assert body_y == pytest.approx(0.0, abs=2e-3)


def test_tilt_is_monotonic_in_the_requested_moment(lib):
    """力矩越大倾角越大；反号则倾角反号。方向错了整套辨识的符号都会翻。"""
    previous_x = None
    for moment in (0.0, 0.05, 0.10, 0.20, 0.40):
        _, _, body_x = solve(lib, [moment, 0.0, 0.0], 13.4)
        if previous_x is not None:
            assert abs(body_x) >= abs(previous_x)
        previous_x = body_x

    _, _, positive_x = solve(lib, [0.2, 0.0, 0.0], 13.4)
    _, _, negative_x = solve(lib, [-0.2, 0.0, 0.0], 13.4)
    assert positive_x == pytest.approx(-negative_x, rel=1e-3)
    assert positive_x != pytest.approx(0.0, abs=1e-4)


def test_roll_moment_uses_lateral_tilt_and_pitch_uses_longitudinal_tilt(lib):
    """两个轴不许串：roll 力矩只应动 body_x，pitch 力矩只应动 body_y。"""
    _, roll_x, roll_y = solve(lib, [0.2, 0.0, 0.0], 13.4)
    _, pitch_x, pitch_y = solve(lib, [0.0, 0.2, 0.0], 13.4)
    assert abs(roll_y) > 10 * abs(roll_x)
    assert abs(pitch_x) > 10 * abs(pitch_y)


def test_yaw_component_is_ignored_because_tilt_cannot_make_it(lib):
    """偏航由上下桨差速产生，不由倾转产生；Z 分量必须被忽略而不是悄悄影响倾角。"""
    _, without_yaw_x, without_yaw_y = solve(lib, [0.1, 0.1, 0.0], 13.4)
    _, with_yaw_x, with_yaw_y = solve(lib, [0.1, 0.1, 5.0], 13.4)
    assert with_yaw_x == pytest.approx(without_yaw_x, rel=1e-5)
    assert with_yaw_y == pytest.approx(without_yaw_y, rel=1e-5)


def test_unreachable_moment_saturates_at_the_configured_tilt_limit(lib):
    """要一个做不到的力矩时应当停在限位，而不是给出超限倾角。"""
    limit = tilt_limit(lib)
    _, body_x, body_y = solve(lib, [999.0, 999.0, 0.0], 13.4)
    assert abs(body_x) <= limit + 1e-4
    assert abs(body_y) <= limit + 1e-4
    assert abs(body_x) == pytest.approx(limit, rel=2e-2)


def test_45_degree_rig_moment_produces_two_equal_servo_tilts(lib):
    """光杆在 XY 之间 45°，所以绕杆力矩必然让两路倾角等量——两个舵机是联动的。"""
    magnitude = 0.15
    component = magnitude * math.sqrt(0.5)
    ok, body_x, body_y = solve(lib, [component, component, 0.0], 13.4)
    assert ok == 1
    assert abs(body_x) == pytest.approx(abs(body_y), rel=0.05)


def test_output_feeds_the_existing_servo_pulse_mapping(lib):
    """反解出来的机体倾角必须能直接喂给既有的脉宽换算，不需要中间再转一次。"""
    _, body_x, body_y = solve(lib, [0.1, 0.1, 0.0], 13.4)
    alpha = ctypes.c_uint16()
    beta = ctypes.c_uint16()
    lib.DRV_COAX_CTRL_BodyTiltRadToServoPulses(
        body_x, body_y, ctypes.byref(alpha), ctypes.byref(beta))
    assert 500 <= alpha.value <= 2500
    assert 500 <= beta.value <= 2500


# ---------------------------------------------------------------- 同源


def test_wrapper_calls_the_allocator_solvers_rather_than_reimplementing():
    source = (ROOT / "Driver/Src/drv_coax_ctrl.c").read_text(encoding="utf-8")
    body = source.split("uint8_t DRV_COAX_CTRL_SolveBodyTiltFromMoment(")[1]
    assert "coax_ctrl_solve_roll_tilt_from_moment(" in body
    assert "coax_ctrl_solve_pitch_tilt_from_moment(" in body
    # 限位取自运行时参数，调用方不能绕过
    assert "coax_ctrl_params.tilt_limit_rad" in body
    # 机体/舵机换向沿用同一处定义
    assert "*body_x_tilt_rad = alpha_rad;" in body
    assert "*body_y_tilt_rad = beta_rad;" in body

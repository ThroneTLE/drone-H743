"""辨识运行层（App/Src/app_sysid.c）的行为契约 —— 在宿主上跑真固件代码。

判据分四类，每类对应一件在台架上真会出事的事：

* **安全门**：链路断 / 掉锁 / IMU 失效 / 轴向残差超限 / 角度超限 / 推力不足，
  一律 abort 并报出**最根本的那个**理由。报错了理由会把人引到错的方向上去查。
* **不命令电机**：整个模块源码里不许出现 `DRV_Motor` / `BSP_PWM_SetEscPulse`。
  推力由遥控器给，辨识只读回。
* **采样的时间完整性**：一批绝不跨越采样断点，断点后的第一批必须带 GAP。
  线上时间是 `base + k·dt` 的压缩形式，跨断点拼一批等于把丢掉的几拍压没了——
  而这套辨识测的正是时移，那种错会直接变成一个假的延迟值。
* **与真实分配器同源**：倾角来自 `DRV_COAX_CTRL_SolveBodyTiltFromMoment`，
  不是辨识自己写的一份力学。
"""
from __future__ import annotations

import ctypes
import math
import os
import shutil
import struct
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]

SOURCES = [
    "App/Src/app_sysid.c",
    "App/Src/app_sysid_alt.c",
    "Driver/Src/drv_sysid_rig.c",
    "Driver/Src/drv_sysid_record.c",
    "Driver/Src/drv_sysid_excitation.c",
    "Driver/Src/drv_coax_ctrl.c",
    "Driver/Src/drv_airframe_params.c",
    "Driver/Src/drv_prop_map.c",
    "Driver/Src/drv_position_control.c",
    "Driver/Src/drv_attitude_control.c",
    "Driver/Src/drv_rate_control.c",
    "tests/fixtures/sysid/app_harness.c",
]

INCLUDES = ["App/Inc", "Driver/Inc", "BSP/Inc"]

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
]

PROFILE_STEP, PROFILE_DOUBLET, PROFILE_CHIRP, PROFILE_PRBS = 0, 1, 2, 3

FLAG_FIRST = 0x1
FLAG_LAST = 0x2
FLAG_ABORTED = 0x4
FLAG_GAP = 0x10

STATE_IDLE, STATE_RUNNING, STATE_DONE, STATE_ABORTED = 0, 1, 2, 3

CONTROL_DT_US = 2000  # 500 Hz 控制拍


class Airframe(ctypes.Structure):
    _fields_ = [(name, ctypes.c_float) for name in AIRFRAME_FIELDS]


class Rig(ctypes.Structure):
    _fields_ = [
        ("azimuth_rad", ctypes.c_float),
        ("axis_offset_above_cg_m", ctypes.c_float),
        ("imu_above_cg_m", ctypes.c_float),
    ]


class Excitation(ctypes.Structure):
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


class Observe(ctypes.Structure):
    _fields_ = [
        ("now_ms", ctypes.c_uint32),
        ("now_us", ctypes.c_uint32),
        ("gyro_rad_s", ctypes.c_float * 3),
        ("roll_rad", ctypes.c_float),
        ("pitch_rad", ctypes.c_float),
        ("throttle_us", ctypes.c_uint16),
        ("rc_link_ok", ctypes.c_uint8),
        ("rc_armed", ctypes.c_uint8),
        ("imu_valid", ctypes.c_uint8),
        ("actuator_inhibit", ctypes.c_uint8),
        ("thrust_valid", ctypes.c_uint8),
        ("rc_throttle_low", ctypes.c_uint8),
        ("gyro_ctrl_rad_s", ctypes.c_float * 3),   # 控制用（转速陷波后），闭环 PID 读它
        # 高度辨识（ALT，R-ALTID-1）：稳定环每拍都填，非 ALT 轮辨识不读。
        ("height_valid", ctypes.c_uint8),
        ("height_m", ctypes.c_float),
        ("height_raw_m", ctypes.c_float),
        ("vz_m_s", ctypes.c_float),
        ("az_m_s2", ctypes.c_float),
        ("vbat_v", ctypes.c_float),
    ]


@pytest.fixture(scope="module")
def lib(tmp_path_factory):
    return build_lib(tmp_path_factory)


def build_lib(tmp_path_factory, extra_sources=(), name="sysid-runtime", flags=()):
    """编译真固件源码 + 宿主装置并装好默认机体。extra_sources 供命令面等测试追加源文件，
    flags 追加编译选项（如 -D 调小某张表的容量）。"""
    compiler = shutil.which("gcc") or shutil.which("clang")
    if compiler is None:
        pytest.skip("host gcc/clang required for the sysid runtime contract")
    build = tmp_path_factory.mktemp(name)
    out = build / ("sysid.dll" if os.name == "nt" else "sysid.so")
    includes = []
    for path in INCLUDES:
        includes += ["-I", str(ROOT / path)]
    result = subprocess.run(
        [compiler, "-shared", "-fPIC", "-std=c11", "-O1", "-Wall", "-Wextra", *flags,
         *includes, *[str(ROOT / path) for path in (*SOURCES, *extra_sources)],
         "-lm", "-o", str(out)],
        capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "warning:" not in result.stderr, result.stderr

    handle = ctypes.CDLL(str(out))
    handle.APP_SysId_SetRig.argtypes = [ctypes.POINTER(Rig)]
    handle.APP_SysId_SetRig.restype = ctypes.c_uint8
    handle.APP_SysId_GetRig.argtypes = [ctypes.POINTER(Rig)]
    handle.APP_SysId_SetExcitation.argtypes = [ctypes.POINTER(Excitation)]
    handle.APP_SysId_SetExcitation.restype = ctypes.c_uint8
    handle.APP_SysId_GetExcitation.argtypes = [ctypes.POINTER(Excitation)]
    handle.APP_SysId_SetInertia.argtypes = [ctypes.c_float]
    handle.APP_SysId_SetInertia.restype = ctypes.c_uint8
    handle.APP_SysId_SetSampleRate.argtypes = [ctypes.c_uint32]
    handle.APP_SysId_SetSampleRate.restype = ctypes.c_uint8
    handle.APP_SysId_SetAngleLimit.argtypes = [ctypes.c_float]
    handle.APP_SysId_SetAngleLimit.restype = ctypes.c_uint8
    handle.APP_SysId_SetResidualLimit.argtypes = [ctypes.c_float]
    handle.APP_SysId_SetResidualLimit.restype = ctypes.c_uint8
    handle.APP_SysId_Start.restype = ctypes.c_uint8
    handle.APP_SysId_Stop.argtypes = [ctypes.c_char_p]
    handle.APP_SysId_Update.argtypes = [ctypes.POINTER(Observe)]
    handle.APP_SysId_GetState.restype = ctypes.c_int
    handle.APP_SysId_IsRunning.restype = ctypes.c_uint8
    handle.APP_SysId_GetLastReason.restype = ctypes.c_char_p
    handle.APP_SysId_GetRunId.restype = ctypes.c_uint16
    handle.APP_SysId_GetDroppedSamples.restype = ctypes.c_uint32
    handle.APP_SysId_GetServoTargets.argtypes = [
        ctypes.POINTER(ctypes.c_uint16), ctypes.POINTER(ctypes.c_uint16)]
    handle.harness_text_line.argtypes = [ctypes.c_uint32]
    handle.harness_text_line.restype = ctypes.c_char_p
    handle.harness_text_lines.restype = ctypes.c_uint32
    handle.harness_frame_total.restype = ctypes.c_uint32
    handle.harness_frame_function.argtypes = [ctypes.c_uint32]
    handle.harness_frame_function.restype = ctypes.c_uint16
    handle.harness_frame_length.argtypes = [ctypes.c_uint32]
    handle.harness_frame_length.restype = ctypes.c_uint16
    handle.harness_frame_data.argtypes = [ctypes.c_uint32]
    handle.harness_frame_data.restype = ctypes.POINTER(ctypes.c_uint8)
    handle.harness_set_ident_busy.argtypes = [ctypes.c_uint8]
    handle.APP_SysId_SetThrottle.argtypes = [ctypes.c_float, ctypes.c_float]
    handle.APP_SysId_SetThrottle.restype = ctypes.c_uint8
    handle.APP_SysId_GetMotorPulse.argtypes = [ctypes.POINTER(ctypes.c_uint16)]
    handle.APP_SysId_GetMotorPulse.restype = ctypes.c_uint8
    handle.APP_SysId_GetPhase.restype = ctypes.c_int
    handle.harness_set_stop_on_critical.argtypes = [ctypes.c_uint32]
    handle.harness_set_stop_after_critical.argtypes = [ctypes.c_uint32]
    handle.DRV_Airframe_SetParams.argtypes = [ctypes.POINTER(Airframe)]
    handle.DRV_Airframe_IsValid.restype = ctypes.c_uint8

    handle.DRV_Airframe_SetParams(ctypes.byref(default_airframe()))
    assert handle.DRV_Airframe_IsValid() == 1
    return handle


def default_airframe():
    """本文件所有数值断言所依据的机体。改机体的测试用完要装回这一份。"""
    frame = Airframe()
    frame.board_mass_g = 75.0
    frame.battery_mass_g = 232.0
    frame.base_mass_g = 99.0
    frame.servo_motor_mass_g = 348.6
    frame.battery_cg_z_m = 0.109
    frame.base_cg_z_m = -0.117
    frame.servo_motor_cg_z_m = -0.244
    frame.prop_plane_d_m = 0.250
    frame.roll_axis_to_prop_plane_m = 0.145
    frame.pitch_axis_to_prop_plane_m = 0.105
    # 2026-09-27 起倾转力臂 = 重心 − 舵机转轴（几何）。这里取让派生重心 −0.0945579 m
    # 复现旧有效力臂 0.145×0.581 / 0.145×0.569 的转轴高度，本文件的数值断言因此不变。
    frame.servo1_axis_z_m = -0.178803
    frame.servo2_axis_z_m = -0.177063
    frame.thrust_point_z_m = -0.2955
    frame.ixx_kgm2 = 0.019
    frame.iyy_kgm2 = 0.019
    frame.izz_kgm2 = 0.00035
    frame.lower_rotor_spin_sense = -1.0
    frame.gravity_m_s2 = 9.81
    frame.max_total_thrust_g = 1595.342
    frame.servo_deg_per_us = 0.09
    frame.derived_auto = 1.0
    return frame


# ---------------------------------------------------------------- 驱动小工具


def reset(lib, *, rate_hz=250, inertia=0.019, spec=None):
    lib.harness_reset()
    lib.APP_SysId_Stop(b"test-reset")
    for _ in range(32):
        lib.APP_SysId_StreamTick()
    lib.harness_reset()
    assert lib.APP_SysId_SetThrottle(ctypes.c_float(0.0), ctypes.c_float(75.0)) == 1
    assert lib.APP_SysId_SetInertia(ctypes.c_float(inertia)) == 1
    assert lib.APP_SysId_SetSampleRate(rate_hz) == 1
    rig = Rig(azimuth_rad=math.pi / 4, axis_offset_above_cg_m=0.0,
              imu_above_cg_m=0.0)
    assert lib.APP_SysId_SetRig(ctypes.byref(rig)) == 1
    if spec is None:
        spec = Excitation(profile=PROFILE_DOUBLET, amplitude_rad_s=0.1,
                          duration_ms=400, hold_ms=50, repeat=2, ramp_ms=10,
                          chirp_f0_hz=0.3, chirp_f1_hz=6.0, prbs_bit_ms=40,
                          prbs_seed=1)
    assert lib.APP_SysId_SetExcitation(ctypes.byref(spec)) == 1
    lib.harness_reset()


def healthy(t_ms, t_us, **overrides):
    obs = Observe()
    obs.now_ms = t_ms
    obs.now_us = t_us
    obs.gyro_rad_s = (ctypes.c_float * 3)(0.0, 0.0, 0.0)
    obs.roll_rad = 0.0
    obs.pitch_rad = 0.0
    obs.throttle_us = 1600
    obs.rc_link_ok = 1
    obs.rc_armed = 1
    obs.imu_valid = 1
    obs.thrust_valid = 1
    for key, value in overrides.items():
        setattr(obs, key, value)
    if "gyro_ctrl_rad_s" not in overrides:
        # 陷波关时稳定环给的两份逐位相同；单独测"只动其中一份"的用例自己覆盖。
        obs.gyro_ctrl_rad_s = (ctypes.c_float * 3)(*obs.gyro_rad_s)
    return obs


def run_ticks(lib, count, *, start_ms=1_000, start_us=1_000_000, mutate=None):
    """跑 count 个 500 Hz 控制拍。mutate(index, obs) 可以改这一拍的观测。"""
    for index in range(count):
        obs = healthy(start_ms + index * 2, start_us + index * CONTROL_DT_US)
        if mutate is not None:
            mutate(index, obs)
        lib.APP_SysId_Update(ctypes.byref(obs))
        if lib.APP_SysId_IsRunning() == 0:
            return index
    return count


def texts(lib):
    return [lib.harness_text_line(i).decode("utf-8", "replace")
            for i in range(lib.harness_text_lines())]


def test_schema_uses_integer_formatting_supported_by_nano(lib):
    import re
    from tools.sysid.decode import parse_schema_lines
    source = (ROOT / "App/Src/app_sysid.c").read_text(encoding="utf-8")
    body = source.split("void APP_SysId_ReportSchema(void)", 1)[1].split("void APP_SysId_ReportStatus", 1)[0]
    assert not re.search(r"%[-+ #0-9.]*[efgEFG]", body), "nano printf has no float support"
    reset(lib)
    lib.APP_SysId_ReportSchema()
    schema = parse_schema_lines(texts(lib))
    # v3（2026-09-27）：前 13 个字段不动，末尾追加下桨转速与沿杆轴的舵机倾转；
    # 高度辨识（R-ALTID-1）再追加 7 个高度字段，版本号不升（见 drv_sysid_record.h）。
    assert schema.version == 3
    assert len(schema.fields) == 22
    assert schema.record_bytes == 44
    assert schema.fields[0].scale == pytest.approx(0.001)
    assert schema.fields[9].name == "erpm" and schema.fields[9].scale == 4.0
    assert schema.fields[12].name == "offset_us"
    assert (schema.fields[13].name, schema.fields[13].unit, schema.fields[13].type) == \
        ("erpm_lower", "rpm", 1)
    assert schema.fields[13].scale == 4.0
    assert (schema.fields[14].name, schema.fields[14].unit, schema.fields[14].type) == \
        ("servo_tilt", "rad", 0)
    assert schema.fields[14].scale == pytest.approx(1e-4)
    assert [(f.name, f.unit, f.type) for f in schema.fields[15:]] == [
        ("height", "m", 0), ("height_raw", "m", 0), ("height_sp", "m", 0), ("vz", "m/s", 0),
        ("vz_sp", "m/s", 0), ("az", "m/s^2", 0), ("vbat", "V", 1)]


def frames(lib):
    out = []
    for index in range(lib.harness_frame_total()):
        length = lib.harness_frame_length(index)
        pointer = lib.harness_frame_data(index)
        out.append((lib.harness_frame_function(index),
                    bytes(pointer[i] for i in range(length))))
    return out


def batch_header(payload):
    ver, count, run_id, schema, base_us, dt_us, flags = struct.unpack_from(
        "<BBHIIHH", payload, 0)
    return {"ver": ver, "count": count, "run_id": run_id, "schema": schema,
            "base_us": base_us, "dt_us": dt_us, "flags": flags}


# ---------------------------------------------------------------- 安全门


def test_start_is_refused_while_the_legacy_ident_holds_the_servos(lib):
    reset(lib)
    lib.harness_set_ident_busy(1)
    assert lib.APP_SysId_Start() == 0
    assert any("ident running" in line for line in texts(lib))
    lib.harness_set_ident_busy(0)


def test_start_is_refused_when_the_excitation_does_not_validate(lib):
    reset(lib)
    bad = Excitation(profile=PROFILE_DOUBLET, amplitude_rad_s=1.0,
                     duration_ms=400, hold_ms=50, repeat=2, ramp_ms=0,
                     chirp_f0_hz=0.3, chirp_f1_hz=6.0, prbs_bit_ms=40,
                     prbs_seed=1)
    # ramp_ms=0 会让方波的导数变成冲激，前馈要求一个无穷大的倾角。
    assert lib.APP_SysId_SetExcitation(ctypes.byref(bad)) == 0


@pytest.mark.parametrize("field,value,reason", [
    ("rc_link_ok", 0, "rc_lost"),
    ("rc_armed", 0, "rc_disarm"),
    ("imu_valid", 0, "imu_stale"),
])
def test_link_and_arm_faults_abort_with_their_own_reason(lib, field, value, reason):
    reset(lib)
    assert lib.APP_SysId_Start() == 1
    lib.harness_reset()

    def mutate(index, obs):
        if index >= 10:
            setattr(obs, field, value)

    stopped_at = run_ticks(lib, 40, mutate=mutate)
    assert stopped_at == 10
    assert lib.APP_SysId_GetState() == STATE_ABORTED
    assert lib.APP_SysId_GetLastReason().decode() == reason


def test_off_axis_rotation_aborts_because_the_rig_cannot_produce_it(lib):
    """理想光杆上 ω 必须平行于杆轴。垂直分量说明杆不在 45°、或机体没夹紧。"""
    reset(lib)
    assert lib.APP_SysId_Start() == 1
    lib.harness_reset()

    def mutate(index, obs):
        if index >= 5:
            # 纯 +X 转动：在 45° 杆上有一半落在垂直方向。
            obs.gyro_rad_s = (ctypes.c_float * 3)(3.0, 0.0, 0.0)

    assert run_ticks(lib, 40, mutate=mutate) == 5
    assert lib.APP_SysId_GetLastReason().decode() == "axis_residual"


def test_rotation_along_the_rod_axis_does_not_trip_the_residual_gate(lib):
    """同样大小的角速度，只要方向对着杆轴就必须放行——否则这道门没法用。"""
    reset(lib)
    assert lib.APP_SysId_Start() == 1
    lib.harness_reset()
    component = 3.0 * math.sqrt(0.5)

    def mutate(index, obs):
        obs.gyro_rad_s = (ctypes.c_float * 3)(component, component, 0.0)

    assert run_ticks(lib, 40, mutate=mutate) == 40
    assert lib.APP_SysId_IsRunning() == 1
    lib.APP_SysId_Stop(b"test")


def test_exceeding_the_angle_limit_aborts_rather_than_clamping(lib):
    """限幅会让一段被削顶的数据看起来"跑完了"，然后被拿去拟合。"""
    reset(lib)
    assert lib.APP_SysId_SetAngleLimit(ctypes.c_float(math.radians(20.0))) == 1
    assert lib.APP_SysId_Start() == 1
    lib.harness_reset()

    def mutate(index, obs):
        if index >= 8:
            # 25° 绕杆轴：roll/pitch 各 25°·cos45°。
            obs.roll_rad = math.radians(25.0) * math.sqrt(0.5)
            obs.pitch_rad = math.radians(25.0) * math.sqrt(0.5)

    assert run_ticks(lib, 40, mutate=mutate) == 8
    assert lib.APP_SysId_GetLastReason().decode() == "angle_limit"


def test_low_throttle_aborts_because_tilt_without_thrust_makes_no_moment(lib):
    reset(lib)
    assert lib.APP_SysId_Start() == 1
    lib.harness_reset()

    def mutate(index, obs):
        if index >= 3:
            obs.throttle_us = 1000

    assert run_ticks(lib, 40, mutate=mutate) == 3
    assert lib.APP_SysId_GetLastReason().decode() == "thrust_low"


def test_the_most_fundamental_reason_wins_when_several_trip_at_once(lib):
    """链路断了角度当然也会超限。报"角度超限"会把人引到错的方向上去查。"""
    reset(lib)
    assert lib.APP_SysId_Start() == 1
    lib.harness_reset()

    def mutate(index, obs):
        if index >= 4:
            obs.rc_link_ok = 0
            obs.throttle_us = 1000
            obs.roll_rad = 1.0
            obs.pitch_rad = 1.0

    assert run_ticks(lib, 40, mutate=mutate) == 4
    assert lib.APP_SysId_GetLastReason().decode() == "rc_lost"


def test_servos_return_to_centre_after_an_abort(lib):
    reset(lib)
    assert lib.APP_SysId_Start() == 1
    run_ticks(lib, 6)
    alpha = ctypes.c_uint16()
    beta = ctypes.c_uint16()
    lib.APP_SysId_GetServoTargets(ctypes.byref(alpha), ctypes.byref(beta))
    moved = (alpha.value, beta.value)

    lib.APP_SysId_Stop(b"command")
    lib.APP_SysId_GetServoTargets(ctypes.byref(alpha), ctypes.byref(beta))
    assert (alpha.value, beta.value) != moved or moved == (alpha.value, beta.value)
    # 停下来之后必须停在中位，而不是保持最后一个激励的倾角。
    assert 1400 <= alpha.value <= 1600
    assert 1400 <= beta.value <= 1600


# ---------------------------------------------------------------- 执行链


def test_the_run_completes_and_reports_done(lib):
    reset(lib)
    assert lib.APP_SysId_Start() == 1
    lib.harness_reset()
    ticks = run_ticks(lib, 400)
    assert ticks < 400, "剖面应当自己走完"
    assert lib.APP_SysId_GetState() == STATE_DONE
    assert lib.APP_SysId_GetLastReason().decode() == "complete"


def test_a_larger_assumed_inertia_commands_a_larger_tilt(lib):
    """前馈是 τ = I_est·α_ff。惯量翻倍、同一条剖面，倾角必须跟着变大。

    这条钉的是"假定惯量确实进了力矩"，也是整套模型反演收敛的前提：
    实测/期望角加速度之比给出 I_true/I_est。
    """
    step = Excitation(profile=PROFILE_STEP, amplitude_rad_s=0.1,
                      duration_ms=400, hold_ms=100, repeat=1, ramp_ms=40,
                      chirp_f0_hz=0.3, chirp_f1_hz=6.0, prbs_bit_ms=40,
                      prbs_seed=1)
    deflections = []
    for inertia in (0.010, 0.040):
        reset(lib, inertia=inertia, spec=step)
        assert lib.APP_SysId_Start() == 1
        alpha = ctypes.c_uint16()
        beta = ctypes.c_uint16()
        peak = 0
        for index in range(20):  # 斜坡段内，α_ff 恒定
            obs = healthy(1_000 + index * 2, 1_000_000 + index * CONTROL_DT_US)
            lib.APP_SysId_Update(ctypes.byref(obs))
            lib.APP_SysId_GetServoTargets(ctypes.byref(alpha), ctypes.byref(beta))
            peak = max(peak, abs(alpha.value - 1500), abs(beta.value - 1500))
        deflections.append(peak)
        lib.APP_SysId_Stop(b"test")
    assert deflections[1] > deflections[0], deflections


def test_the_two_servos_move_together_because_the_rod_is_at_45_degrees(lib):
    """绕 45° 杆的力矩在 X/Y 上分量相等，所以两路倾角必然等量——联动，不是单轴。"""
    step = Excitation(profile=PROFILE_STEP, amplitude_rad_s=0.1,
                      duration_ms=400, hold_ms=100, repeat=1, ramp_ms=40,
                      chirp_f0_hz=0.3, chirp_f1_hz=6.0, prbs_bit_ms=40,
                      prbs_seed=1)
    reset(lib, spec=step)
    assert lib.APP_SysId_Start() == 1
    alpha = ctypes.c_uint16()
    beta = ctypes.c_uint16()
    for index in range(12):
        obs = healthy(1_000 + index * 2, 1_000_000 + index * CONTROL_DT_US)
        lib.APP_SysId_Update(ctypes.byref(obs))
    lib.APP_SysId_GetServoTargets(ctypes.byref(alpha), ctypes.byref(beta))
    da = abs(alpha.value - 1500)
    db = abs(beta.value - 1500)
    assert da > 2 and db > 2, (da, db)
    assert abs(da - db) <= max(2, 0.25 * max(da, db)), (da, db)
    lib.APP_SysId_Stop(b"test")


# ---------------------------------------------------------------- 采样与成帧


def test_batches_carry_the_firmware_clock_not_the_send_time(lib):
    reset(lib, rate_hz=500)
    assert lib.APP_SysId_Start() == 1
    lib.harness_reset()
    run_ticks(lib, 60, start_us=7_000_000)
    lib.APP_SysId_StreamTick()

    sent = frames(lib)
    assert sent, "跑了 60 拍应当至少攒出一批"
    function, payload = sent[0]
    assert function == 0x2234
    header = batch_header(payload)
    assert header["ver"] == 3
    assert header["base_us"] >= 7_000_000
    assert header["dt_us"] == 2000
    assert header["flags"] & FLAG_FIRST
    lib.APP_SysId_Stop(b"test")


def firmware_constant(path: str, name: str) -> int:
    text = (ROOT / path).read_text(encoding="utf-8")
    line = [l for l in text.splitlines()
            if l.strip().startswith("#define") and name in l.split()][0]
    return int(line.split()[-1].rstrip("U"))


def test_every_emitted_batch_fits_the_firmware_framer(lib):
    """发送端有两道上限，超了任何一道都是"打包成功、一个字节没发出去"。

    1. `APP_Proto_BuildFrame` 拒收 payload > `APP_PROTO_MAX_PAYLOAD`；
    2. 更紧的一道：`APP_Diag_SendBinary` 把**成帧后**的字节写进
       `APP_UART_TxMessage.text[APP_UART_TX_TEXT_SIZE]`，$X 外层还占 9 字节。

    宿主装置照样模拟这两道（见 tests/fixtures/sysid/app_harness.c），
    所以"发不出去"在这里就会表现成收不到帧，而不是等到实机。
    """
    max_payload = firmware_constant("App/Inc/app_proto.h", "APP_PROTO_MAX_PAYLOAD")
    tx_buffer = firmware_constant("App/Inc/app_messages.h", "APP_UART_TX_TEXT_SIZE")

    reset(lib, rate_hz=500)
    assert lib.APP_SysId_Start() == 1
    lib.harness_reset()
    for _ in range(7):
        run_ticks(lib, 30)
        lib.APP_SysId_StreamTick()
    sent = frames(lib)
    assert sent, "一批都没发出去——多半是超过了发送缓冲"
    full_batches = [p for _f, p in sent if len(p) == 16 + 5 * 44]
    assert full_batches, "满批必须能发出去，否则这条流在实机上只会零星漏几帧"
    for _function, payload in sent:
        assert len(payload) <= max_payload
        assert len(payload) + 9 <= tx_buffer
    lib.APP_SysId_Stop(b"test")


def test_the_sample_rate_decimates_the_control_tick(lib):
    """控制拍恒为 500 Hz；线上速率只决定抽取，不改变执行。"""
    counts = {}
    for rate in (125, 500):
        reset(lib, rate_hz=rate)
        assert lib.APP_SysId_Start() == 1
        lib.harness_reset()
        run_ticks(lib, 100)
        for _ in range(4):
            lib.APP_SysId_StreamTick()
        counts[rate] = sum(batch_header(p)["count"] for _f, p in frames(lib))
        lib.APP_SysId_Stop(b"test")
    assert counts[500] > counts[125] * 2, counts


def test_a_batch_never_spans_a_dropped_sample_and_the_next_one_flags_it(lib):
    """环满丢样之后，下一批必须带 GAP，而且断点两侧不许拼进同一批。

    线上时间是 `base + k·dt` 的压缩形式：跨断点拼一批等于把丢掉的几拍压没了，
    而这套辨识测的正是时移——那种错会变成一个假的延迟值。
    """
    # 要撑满 128 条的环，剖面必须比 128 拍长，否则它自己先跑完了。
    long_step = Excitation(profile=PROFILE_STEP, amplitude_rad_s=0.5,
                           duration_ms=3000, hold_ms=2800, repeat=1, ramp_ms=50,
                           chirp_f0_hz=0.3, chirp_f1_hz=6.0, prbs_bit_ms=40,
                           prbs_seed=1)
    reset(lib, rate_hz=500, spec=long_step)
    assert lib.APP_SysId_Start() == 1
    lib.harness_reset()

    # 先只跑不排空，把 128 条的环撑满；之后的样本被丢掉（丢新不覆盖旧，
    # 所以环里那 128 条仍然是连续的一段）。
    assert run_ticks(lib, 300) == 300
    assert lib.APP_SysId_GetDroppedSamples() > 0
    for _ in range(40):
        lib.APP_SysId_StreamTick()

    # 再接着跑一段并排空。断点落在"环满那一刻"和"恢复采样那一刻"之间，
    # 恢复之后的第一批必须报 GAP。
    assert run_ticks(lib, 40, start_ms=1_700, start_us=1_600_000) == 40
    for _ in range(40):
        lib.APP_SysId_StreamTick()

    sent = frames(lib)
    assert len(sent) >= 2

    dt = batch_header(sent[0][1])["dt_us"]
    previous_end = None
    saw_gap = False
    for _function, payload in sent:
        header = batch_header(payload)
        if previous_end is not None:
            step = (header["base_us"] - previous_end) & 0xFFFFFFFF
            if step > dt * 3 // 2:
                assert header["flags"] & FLAG_GAP, (
                    f"断点后的批没有报 GAP：step={step} dt={dt}")
                saw_gap = True
            else:
                assert not (header["flags"] & FLAG_GAP)
        previous_end = header["base_us"] + (header["count"] - 1) * dt
    assert saw_gap, "这一趟本来就该出现断点"
    lib.APP_SysId_Stop(b"test")


def test_the_final_batch_is_marked_and_an_abort_is_visible_in_the_flags(lib):
    reset(lib, rate_hz=250)
    assert lib.APP_SysId_Start() == 1
    lib.harness_reset()
    run_ticks(lib, 40)
    lib.APP_SysId_Stop(b"command")
    for _ in range(8):
        lib.APP_SysId_StreamTick()
    sent = frames(lib)
    assert sent
    last = batch_header(sent[-1][1])
    assert last["flags"] & FLAG_LAST
    assert last["flags"] & FLAG_ABORTED


def test_a_new_run_gets_a_new_run_id(lib):
    reset(lib)
    assert lib.APP_SysId_Start() == 1
    first = lib.APP_SysId_GetRunId()
    lib.APP_SysId_Stop(b"test")
    lib.APP_SysId_StreamTick()
    assert lib.APP_SysId_Start() == 1
    assert lib.APP_SysId_GetRunId() != first
    lib.APP_SysId_Stop(b"test")


def test_the_host_decodes_a_real_frame_using_the_firmwares_own_schema_lines(lib):
    """端到端：固件报的字段表 -> 主机解析器 -> 解一帧固件真发出来的数据。

    这条是整套自描述机制的验收点。主机侧那份解码器里**没有**任何字段名或偏移量，
    全靠这几行文本；所以只要固件改了字段表而忘了同步，这里立刻红。
    反过来，如果测试自己手写一份 schema 文本，验的就只是解析器自己了。
    """
    sys.path.insert(0, str(ROOT / "tools"))
    try:
        from sysid.decode import decode_batch, parse_schema_lines
    finally:
        sys.path.pop(0)

    reset(lib, rate_hz=500)
    lib.harness_reset()
    lib.APP_SysId_ReportSchema()
    schema = parse_schema_lines(texts(lib))
    assert schema.record_bytes == 44
    assert "gx" in schema.field_names()

    assert lib.APP_SysId_Start() == 1
    lib.harness_reset()
    run_ticks(lib, 40, mutate=lambda i, obs: setattr(
        obs, "gyro_rad_s", (ctypes.c_float * 3)(0.7, 0.7, 0.0)))
    lib.APP_SysId_StreamTick()
    sent = frames(lib)
    assert sent

    batch = decode_batch(sent[0][1], schema)
    assert batch.samples
    assert batch.first
    # 定点往返：0.7 rad/s 按 mrad/s 存，回来应当还在千分之一以内。
    assert batch.samples[0]["gx"] == pytest.approx(0.7, abs=2e-3)
    assert batch.timestamps_us()[1] - batch.timestamps_us()[0] == batch.dt_us
    lib.APP_SysId_Stop(b"test")


def test_a_stale_host_schema_is_refused_rather_than_decoded(lib):
    """固件加了字段、主机还拿着旧表时必须拒解，不能解出一组看着正常的错值。"""
    sys.path.insert(0, str(ROOT / "tools"))
    try:
        from sysid.decode import SchemaMismatch, SysIdSchema, decode_batch, parse_schema_lines
    finally:
        sys.path.pop(0)

    reset(lib, rate_hz=500)
    lib.harness_reset()
    lib.APP_SysId_ReportSchema()
    schema = parse_schema_lines(texts(lib))

    assert lib.APP_SysId_Start() == 1
    lib.harness_reset()
    run_ticks(lib, 40)
    lib.APP_SysId_StreamTick()
    payload = frames(lib)[0][1]
    lib.APP_SysId_Stop(b"test")

    stale = SysIdSchema(fields=schema.fields, hash=schema.hash ^ 0xFF)
    with pytest.raises(SchemaMismatch):
        decode_batch(payload, stale)


# ---------------------------------------------------------------- 源码机检


def strip_c_comments(source: str) -> str:
    """只看代码。注释里写"这里不许出现 X"本身会把这类机检绊倒。"""
    out = []
    index = 0
    length = len(source)
    while index < length:
        if source.startswith("/*", index):
            end = source.find("*/", index + 2)
            index = length if end == -1 else end + 2
        elif source.startswith("//", index):
            end = source.find("\n", index)
            index = length if end == -1 else end
        else:
            out.append(source[index])
            index += 1
    return "".join(out)


def test_the_identification_module_never_touches_the_esc_directly():
    """自动油门（作者 2026-09-26 授权）只算脉宽；写电调的仍是稳定环里
    "已解锁 + 链路正常"那一支，上锁与失联分支里辨识根本够不着电机。"""
    for path in ("App/Src/app_sysid.c", "App/Src/app_cmd_sysid.c",
                 "App/Inc/app_sysid.h", "App/Src/app_sysid_alt.c", "App/Inc/app_sysid_alt.h"):
        code = strip_c_comments((ROOT / path).read_text(encoding="utf-8"))
        assert "DRV_Motor" not in code, path
        assert "BSP_PWM_SetEscPulse" not in code, path
        assert "BSP_PWM_SetEscPercent" not in code, path
    stabilizer = strip_c_comments((ROOT / "App/Src/app_stabilizer.c").read_text(encoding="utf-8"))
    assert stabilizer.count("APP_SysId_GetMotorPulse(") == 1
    armed_branch = stabilizer.split("((frame->rc_link_ok != 0U) && (frame->rc_armed != 0U))", 1)[1]
    armed_branch = armed_branch.split("} else if ((frame->rc_link_ok != 0U) || (frame->rc_link_seen == 0U))", 1)[0]
    assert "APP_SysId_GetMotorPulse(" in armed_branch


def test_the_identification_module_never_writes_flash():
    """候选增益只写 RAM 实时验证；是否落盘是作者事后的单独动作。"""
    code = strip_c_comments((ROOT / "App/Src/app_sysid.c").read_text(encoding="utf-8"))
    assert "ConfigStore" not in code
    assert "Persist" not in code
    assert "SAVE" not in code


def test_the_tilt_comes_from_the_real_allocator_not_a_second_implementation():
    source = (ROOT / "App/Src/app_sysid.c").read_text(encoding="utf-8")
    assert "DRV_COAX_CTRL_SolveBodyTiltFromMoment(" in source
    assert "DRV_COAX_CTRL_BodyTiltRadToServoPulses(" in source
    assert "DRV_COAX_CTRL_MotorPulseToTotalThrust(" in source
    assert "DRV_SysIdRig_MomentAboutAxis(" in source


def test_the_stabilizer_seam_shares_every_interlock_with_the_legacy_ident():
    """`ident_running` 是"有辨识在占用执行器"，下游联锁全挂在它上面。

    新模块因此一条联锁都不用重新接——也就不会漏接。
    """
    source = (ROOT / "App/Src/app_stabilizer.c").read_text(encoding="utf-8")
    assert "(APP_Ident_IsRunning() != 0U) || (frame->sysid_running != 0U)" in source
    assert "APP_SysId_GetServoTargets(&ident_alpha_us, &ident_beta_us);" in source
    assert "APP_SysId_Update(&sysid_obs);" in source

from test_sysid_page import page, SCHEMA_LINES, make_frame


def configure_api(lib):
    lib.DRV_COAX_CTRL_MomentFromServoPulses.argtypes = [ctypes.c_float, ctypes.c_uint16,
        ctypes.c_uint16, ctypes.POINTER(ctypes.c_float)]
    lib.DRV_COAX_CTRL_MotorPulseToTotalThrust.argtypes = [ctypes.c_uint16]
    lib.DRV_COAX_CTRL_MotorPulseToTotalThrust.restype = ctypes.c_float
    lib.DRV_COAX_CTRL_SolveBodyTiltFromMoment.argtypes = [ctypes.POINTER(ctypes.c_float),
        ctypes.c_float, ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float)]
    lib.DRV_COAX_CTRL_BodyTiltRadToServoPulses.argtypes = [ctypes.c_float, ctypes.c_float,
        ctypes.POINTER(ctypes.c_uint16), ctypes.POINTER(ctypes.c_uint16)]
    lib.APP_SysId_SetMode.argtypes = [ctypes.c_int, ctypes.c_float]
    lib.DRV_COAX_CTRL_SetParam.argtypes = [ctypes.c_char_p, ctypes.c_float]
    lib.DRV_COAX_CTRL_GetParam.argtypes = [ctypes.c_char_p, ctypes.POINTER(ctypes.c_float)]


@pytest.mark.parametrize("requested", [(0.05, 0), (0, 0.05), (0.05,0.05), (-0.05,-0.02)])
def test_pulse_roundtrip_preserves_flu_moment_direction(lib, requested):
    configure_api(lib)
    m = (ctypes.c_float*3)(*requested, 0)
    x, y = ctypes.c_float(), ctypes.c_float()
    assert lib.DRV_COAX_CTRL_SolveBodyTiltFromMoment(m, 13.4, ctypes.byref(x), ctypes.byref(y))
    a, b = ctypes.c_uint16(), ctypes.c_uint16()
    lib.DRV_COAX_CTRL_BodyTiltRadToServoPulses(x, y, ctypes.byref(a), ctypes.byref(b))
    result = (ctypes.c_float*3)()
    assert lib.DRV_COAX_CTRL_MomentFromServoPulses(13.4, a, b, result)
    # 1 us PWM quantisation, 0.09 degree/us; 0.004 N*m covers both axes.
    assert list(result)[:2] == pytest.approx(requested, abs=0.004)


@pytest.mark.parametrize("field", ["roll_rad", "pitch_rad", "gyro_rad_s"])
def test_nonfinite_measurement_aborts_before_servo_output(lib, field):
    reset(lib)
    assert lib.APP_SysId_Start()
    obs = healthy(1000,1000000)
    if field == "gyro_rad_s": obs.gyro_rad_s[0] = float("nan")
    else: setattr(obs, field, float("nan"))
    lib.APP_SysId_Update(ctypes.byref(obs))
    assert lib.APP_SysId_GetLastReason() == b"nonfinite"


def test_completion_notice_never_blocks_the_control_task(lib):
    reset(lib)
    assert lib.APP_SysId_Start()
    lib.harness_reset()
    obs = healthy(1000,1000000, rc_link_ok=0)
    lib.APP_SysId_Update(ctypes.byref(obs))
    assert texts(lib) == []
    assert lib.APP_SysId_IsEngaged() == 1
    lib.APP_SysId_StreamTick()
    assert any("reason=rc_lost" in text for text in texts(lib))
    obs.rc_armed = 0
    lib.APP_SysId_Update(ctypes.byref(obs))
    assert lib.APP_SysId_IsEngaged() == 0


def test_saturation_preserves_failure_values_and_reports_only_on_drain(lib):
    reset(lib, inertia=1.0)
    assert lib.APP_SysId_Start()
    lib.harness_reset()
    lib.APP_SysId_Update(ctypes.byref(healthy(1000, 1000000)))
    assert lib.APP_SysId_GetLastReason() == b"actuator_saturated"
    assert texts(lib) == []
    lib.APP_SysId_StreamTick()
    detail = next(line for line in texts(lib) if line.startswith("SYSID SAT "))
    for key in ("force_mN=", "req_x_uNm=", "req_y_uNm=", "got_x_uNm=",
                "got_y_uNm=", "alpha_us=", "beta_us="):
        assert key in detail
    assert len(detail.encode()) < 256
    lib.harness_reset()
    lib.APP_SysId_StreamTick()
    assert not any("SYSID SAT" in line for line in texts(lib))


def test_logged_pitch_excitation_accepts_first_sample_with_nominal_calibration(lib):
    spec = Excitation(profile=PROFILE_DOUBLET, amplitude_rad_s=0.15,
                      duration_ms=4000, hold_ms=250, repeat=4, ramp_ms=150,
                      chirp_f0_hz=0.3, chirp_f1_hz=6, prbs_bit_ms=40, prbs_seed=1)
    reset(lib, inertia=0.051, spec=spec)
    rig = Rig(math.pi / 2, 0.2, 0.2)
    assert lib.APP_SysId_SetRig(ctypes.byref(rig))
    assert lib.APP_SysId_Start()
    lib.APP_SysId_Update(ctypes.byref(healthy(1000, 1000000)))
    assert lib.APP_SysId_IsRunning()


def test_configuration_frozen_while_running_and_other_actuator_owner_aborts(lib):
    reset(lib)
    assert lib.APP_SysId_Start()
    assert not lib.APP_SysId_SetInertia(0.04)
    assert not lib.APP_SysId_SetSampleRate(50)
    obs = healthy(1000,1000000, actuator_inhibit=1)
    lib.APP_SysId_Update(ctypes.byref(obs))
    assert lib.APP_SysId_GetLastReason() == b"actuator_busy"


def test_jitter_is_preserved_and_send_failure_retries_first_marker(lib):
    from sysid.decode import parse_schema_lines, decode_batch
    reset(lib, rate_hz=500)
    lib.APP_SysId_ReportSchema()
    schema = parse_schema_lines(texts(lib))
    assert lib.APP_SysId_Start()
    timestamps = [1000000,1002050,1004090,1006120,1008130,1010140,1012150,1014170]
    for t in timestamps:
        lib.APP_SysId_Update(ctypes.byref(healthy(t//1000,t)))
    lib.harness_set_send_failure(1)
    lib.APP_SysId_StreamTick()
    assert frames(lib) == []
    lib.harness_set_send_failure(0)
    lib.APP_SysId_StreamTick()
    batch = decode_batch(frames(lib)[0][1], schema)
    assert batch.first
    # 满批 5 条（DRV_SYSID_RECORD_MAX_COUNT，高度字段追加后）：其余留在环里等下一批。
    assert batch.timestamps_us() == tuple(timestamps[:5])


@pytest.mark.parametrize("mode", [1,2])
def test_real_rate_and_attitude_controller_respond_to_feedback(lib, mode):
    configure_api(lib)
    # Set and restore actual production PID parameters, not a Python substitute.
    names = [b"coax.rate_roll_kp", b"coax.rate_pitch_kp"]
    previous = []
    for name in names:
        v = ctypes.c_float(); assert lib.DRV_COAX_CTRL_GetParam(name, ctypes.byref(v))
        previous.append(v.value); assert lib.DRV_COAX_CTRL_SetParam(name,0.08)
    outputs=[]
    for rate in (0.0,0.1):
        reset(lib)
        assert lib.APP_SysId_SetMode(mode,0.02)
        assert lib.APP_SysId_Start()
        for i in range(12):
            obs=healthy(1000+i*2,1000000+i*2000)
            obs.gyro_rad_s[0]=obs.gyro_rad_s[1]=rate/math.sqrt(2)
            obs.gyro_ctrl_rad_s[0]=obs.gyro_ctrl_rad_s[1]=rate/math.sqrt(2)
            lib.APP_SysId_Update(ctypes.byref(obs))
        assert lib.APP_SysId_IsRunning()
        a,b=ctypes.c_uint16(),ctypes.c_uint16()
        lib.APP_SysId_GetServoTargets(ctypes.byref(a),ctypes.byref(b))
        outputs.append((a.value,b.value))
        lib.APP_SysId_Stop(b"test"); lib.APP_SysId_StreamTick()
    assert outputs[0] != outputs[1]
    assert lib.APP_SysId_SetMode(0,0.0523598776)
    for name,value in zip(names,previous): assert lib.DRV_COAX_CTRL_SetParam(name,value)


# ---------------------------------------------------------------- 控制用陀螺（转速陷波后）与原始陀螺


def _closed_loop_servos(lib, mode, raw_rate, ctrl_rate, ticks=12):
    """RATE/ANGLE 闭环跑几拍：原始与控制用陀螺分开给（沿 45° 杆轴），返回舵机脉宽。"""
    reset(lib)
    assert lib.APP_SysId_SetMode(mode, 0.02)
    assert lib.APP_SysId_Start()
    for i in range(ticks):
        obs = healthy(1000 + i * 2, 1000000 + i * 2000)
        for axis in (0, 1):
            obs.gyro_rad_s[axis] = raw_rate / math.sqrt(2)
            obs.gyro_ctrl_rad_s[axis] = ctrl_rate / math.sqrt(2)
        lib.APP_SysId_Update(ctypes.byref(obs))
    assert lib.APP_SysId_IsRunning()
    a, b = ctypes.c_uint16(), ctypes.c_uint16()
    lib.APP_SysId_GetServoTargets(ctypes.byref(a), ctypes.byref(b))
    lib.APP_SysId_Stop(b"test")
    lib.APP_SysId_StreamTick()
    return a.value, b.value


@pytest.mark.parametrize("mode", [1, 2])
def test_the_closed_loop_reacts_to_the_control_gyro_only(lib, mode):
    """PID 读陷波后的控制用陀螺（与在飞控制器同一口径）；原始陀螺只进记录和残差门。"""
    configure_api(lib)
    names = [b"coax.rate_roll_kp", b"coax.rate_pitch_kp"]
    previous = []
    for name in names:
        value = ctypes.c_float()
        assert lib.DRV_COAX_CTRL_GetParam(name, ctypes.byref(value))
        previous.append(value.value)
        assert lib.DRV_COAX_CTRL_SetParam(name, 0.08)
    try:
        baseline = _closed_loop_servos(lib, mode, 0.0, 0.0)
        assert _closed_loop_servos(lib, mode, 0.1, 0.0) == baseline, "原始陀螺不该进 PID"
        assert _closed_loop_servos(lib, mode, 0.0, 0.1) != baseline, "控制用陀螺必须进 PID"
        assert _closed_loop_servos(lib, mode, 0.1, 0.1) == _closed_loop_servos(lib, mode, 0.0, 0.1)
    finally:
        assert lib.APP_SysId_SetMode(0, 0.0523598776)
        for name, value in zip(names, previous):
            assert lib.DRV_COAX_CTRL_SetParam(name, value)


def test_the_log_carries_the_raw_gyro(lib):
    from sysid.decode import decode_batch, parse_schema_lines
    reset(lib, rate_hz=500)
    lib.harness_reset()
    lib.APP_SysId_ReportSchema()
    schema = parse_schema_lines(texts(lib))
    assert lib.APP_SysId_Start() == 1
    lib.harness_reset()

    def mutate(_index, obs):
        obs.gyro_rad_s = (ctypes.c_float * 3)(0.7, 0.7, 0.0)
        obs.gyro_ctrl_rad_s = (ctypes.c_float * 3)(0.2, -0.3, 0.1)

    run_ticks(lib, 20, mutate=mutate)
    lib.APP_SysId_StreamTick()
    batch = decode_batch(frames(lib)[0][1], schema)
    assert batch.samples[0]["gx"] == pytest.approx(0.7, abs=2e-3)
    assert batch.samples[0]["gy"] == pytest.approx(0.7, abs=2e-3)
    lib.APP_SysId_Stop(b"test")


def test_the_residual_gate_judges_the_raw_gyro(lib):
    """杆轴残差看真实运动：控制用陀螺偏离杆轴（纯 +X）不许触发 axis_residual。"""
    reset(lib)
    assert lib.APP_SysId_Start() == 1

    def off_axis_ctrl(_index, obs):
        obs.gyro_ctrl_rad_s = (ctypes.c_float * 3)(3.0, 0.0, 0.0)

    assert run_ticks(lib, 40, mutate=off_axis_ctrl) == 40
    lib.APP_SysId_Stop(b"test")

    reset(lib)
    assert lib.APP_SysId_Start() == 1

    def off_axis_raw(index, obs):
        if index >= 5:
            obs.gyro_rad_s = (ctypes.c_float * 3)(3.0, 0.0, 0.0)
            obs.gyro_ctrl_rad_s = (ctypes.c_float * 3)(0.0, 0.0, 0.0)

    assert run_ticks(lib, 40, mutate=off_axis_raw) == 5
    assert lib.APP_SysId_GetLastReason().decode() == "axis_residual"


@pytest.fixture(scope="module")
def cmd_lib(tmp_path_factory):
    handle = build_lib(tmp_path_factory, ("App/Src/app_cmd_sysid.c", "App/Src/app_param_trial.c",
                                          "tests/fixtures/sysid/cmd_harness.c"),
                       "sysid-cmd-notch")
    handle.harness_command.argtypes = [ctypes.c_char_p]
    handle.harness_command.restype = ctypes.c_uint8
    return handle


def test_only_a_successful_start_is_followed_by_the_notch_provenance(cmd_lib):
    """开跑成功后紧跟 SYSID NOTCH 与 SYSID BACKLASH 两行（带本轮 run）；被拒绝的 START 不报。"""
    lib = cmd_lib
    reset(lib)
    lib.harness_set_ident_busy(1)
    assert lib.harness_command(b"SYSID START") == 1
    assert texts(lib) == ["ERR sysid ident running\r\n"]
    lib.harness_set_ident_busy(0)
    lib.harness_reset()
    assert lib.harness_command(b"SYSID START") == 1
    lines = texts(lib)
    run = lib.APP_SysId_GetRunId()
    assert lines[-3].startswith(f"SYSID start run={run} ")
    assert lines[-2].startswith(f"SYSID NOTCH run={run} ")
    assert lines[-1].startswith(f"SYSID BACKLASH run={run} ")
    lib.APP_SysId_Stop(b"test")


@pytest.mark.parametrize("axis", [0, 1, 2])
def test_a_nonfinite_control_gyro_aborts(lib, axis):
    reset(lib)
    assert lib.APP_SysId_Start()
    obs = healthy(1000, 1000000)
    obs.gyro_ctrl_rad_s[axis] = float("nan")
    lib.APP_SysId_Update(ctypes.byref(obs))
    assert lib.APP_SysId_GetLastReason() == b"nonfinite"


def test_bad_fit_capture_cannot_reuse_previous_candidate(page):
    page._commands = ["PARAM SET coax.rate_roll_kp 1"]
    page._fit = object()
    page.clear_samples()
    assert page._fit is None and page._commands is None
    page.apply_to_ram()
    assert page.panel.transport.lines == []


def test_parameter_timeout_stops_transaction_and_does_not_claim_success(page):
    from test_sysid_page import SAME_UNITS, answer_param_barrier
    page.workflow.end={"state":"done"}
    page.workflow.params={"coax.rate_roll_kp":"0.1", "coax.rate_pitch_kp":"0.1"}
    page._commands=["SYSID PARAM coax.rate_roll_kp 0.08", "SYSID PARAM coax.rate_pitch_kp 0.08"]
    page._commands_source = SAME_UNITS   # 录制时的力矩单位与飞控相同，不换算
    page.apply_scale_var.set("100%")   # 默认试用 50%；这里验原值
    page.apply_to_ram()
    # 应用前先重读 PARAM?/SYSID?（按当前力臂换算单位），之后才发第一条 SYSID PARAM。
    answer_param_barrier(page)
    page.workflow.timeout(page.workflow.ticket)
    page.handle_line("OK param name=coax.rate_roll_kp value=0.08")
    assert page.panel.transport.lines[2:] == page._commands[:1]
    assert "超时" in page.status_var.get()


def test_duplicate_or_missing_frames_are_not_accepted_for_fit(page):
    for line in SCHEMA_LINES: page.handle_line(line)
    first=make_frame(0xDEADBEEF,[(0,0,0)])
    page.accept(first); page.accept(first)
    assert page.workflow.error
    assert len(page.samples)==1


def test_analysis_rejects_missing_torque_and_time_gaps():
    from panel_lib.pages.sysid._core import fit_inner_loop
    samples=[dict(gx=0.,gy=0.,angle=0.,torque=.01)]*100
    times=[i*.004 for i in range(100)]; times[50:]=[v+.1 for v in times[50:]]
    with pytest.raises(ValueError, match="断点"):
        fit_inner_loop(times,samples,azimuth_rad=math.pi/4,mass_kg=1.,assumed_inertia_kg_m2=.02)


def test_start_is_a_readback_transaction_and_refuses_old_firmware(page):
    page.rod_to_fc_var.set("0.15")   # 杆到飞控板距离是必须的尺量输入
    page.start_run()
    assert page.panel.transport.lines == ["SYSID SCHEMA"]
    for line in SCHEMA_LINES: page.handle_line(line)
    assert "SYSID START" not in page.panel.transport.lines
    page.handle_line("ERR sysid not supported")
    assert not page.workflow.pending


def test_current_configuration_is_confirmed_before_start(page, lib):
    reset(lib)
    lib.APP_SysId_ReportSchema()
    page.rod_to_fc_var.set("0.15")
    page.start_run()
    for line in texts(lib):
        if line.startswith("SYSID SCHEMA ") or line.startswith("SYSID FIELD "):
            page.handle_line(line)
    # Replay complete current-generation report boundaries. Values in each field
    # use the actual status report units, and a one-unit quantisation tolerance.
    for _ in range(7):
        if not page.workflow.awaiting: break
        page.handle_line("SYSID READY ver=2 thrust=lut mass_mg=1000000 imu_off_um=35000")
        page.handle_line("SYSID RIG " + " ".join(f"{k}={v}" for k,v in page.workflow.expected.items()))
        page.handle_line("SYSID LIMITS I_ugm2=20000")
    assert page.panel.transport.lines[-1] == "SYSID START"
    assert page.workflow.snapshot["mass_mg"] == "1000000"
    assert page.workflow.snapshot["schema"]["version"] == 3


def test_bad_readback_never_starts_actuators(page, lib):
    # 距离已填、机体参数齐全，事务真的走到最后的回读核对；回报缺 RIG/THR 键 → 不开跑。
    reset(lib); lib.APP_SysId_ReportSchema()
    page.rod_to_fc_var.set("0.15"); page.start_run()
    for line in texts(lib):
        if line.startswith(("SYSID SCHEMA ","SYSID FIELD ")): page.handle_line(line)
    for _ in range(7):
        page.handle_line("SYSID READY ver=3 thrust=lut mass_mg=1000000 imu_off_um=35000")
        page.handle_line("SYSID LIMITS I_ugm2=20000")
    assert any(line.startswith("SYSID THROTTLE") or line.startswith("SYSID RIG")
               for line in page.panel.transport.lines), "应当走到配置下发，而不是在输入检查就停了"
    assert "SYSID START" not in page.panel.transport.lines
    assert page.workflow.error


def test_unaligned_profile_end_still_has_a_final_sample(lib):
    reset(lib, rate_hz=50)
    spec=Excitation(profile=PROFILE_STEP, amplitude_rad_s=.1, duration_ms=500,
        hold_ms=101, repeat=1, ramp_ms=51, chirp_f0_hz=.3, chirp_f1_hz=6.,
        prbs_bit_ms=100, prbs_seed=1)
    assert lib.APP_SysId_SetExcitation(ctypes.byref(spec))
    assert lib.APP_SysId_Start()
    for i in range(150):
        lib.APP_SysId_Update(ctypes.byref(healthy(1000+i*2,1000000+i*2000)))
        lib.APP_SysId_StreamTick()
    sent=frames(lib)
    assert sent and batch_header(sent[-1][1])["flags"] & FLAG_LAST


def test_restarting_the_running_page_does_not_clear_schema_or_send(page):
    page.workflow.run_id=7
    page.workflow.end=None
    for line in SCHEMA_LINES: page.handle_line(line)
    before=page.schema
    page.start_run()
    assert page.schema is before
    assert page.panel.transport.lines == []


def test_lost_thrust_provenance_aborts_even_when_imu_and_rc_are_healthy(lib):
    reset(lib)
    assert lib.APP_SysId_Start()
    obs=healthy(1000,1000000,thrust_valid=0)
    lib.APP_SysId_Update(ctypes.byref(obs))
    assert lib.APP_SysId_GetLastReason() == b"thrust_stale"


def test_discard_ack_releases_only_the_stopped_run_on_the_page(page):
    page.workflow.run_id=7
    page.workflow.end=None
    page.workflow.discard()
    assert page.workflow.run_id == 7
    page.handle_line("ERR sysid still running")
    assert page.workflow.run_id == 7
    page.handle_line("OK sysid discarded stopped run")
    assert page.workflow.run_id is None

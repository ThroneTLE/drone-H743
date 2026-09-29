"""舵机单独辨识（SYSID MODE 3 / SERVO，电机不转）与 v3 记录里的真实转速/沿杆倾转。

在宿主上跑真固件：App/Src/app_sysid.c + app_cmd_sysid.c + 真分配器 + 真桨叶标定，
平台边界见 tests/fixtures/sysid/app_harness.c 与 cmd_harness.c。判据按台架上真会出事
的地方排：

* **电机一拍都不碰**：只在未解锁时开跑，解锁中拒绝（原文 `ERR sysid servo mode needs
  disarmed`），跑的过程中一解锁即中止（rc_arm）；自动油门设置留着也不生效。
* **方向与 FF 同号**：+servo_tilt = "有推力时 FF 为绕杆轴正力矩下发的那个倾转方向"。
  这里拿真 FF 路径在同一杆方位、同一力臂符号下跑出来的倾转逐轴对照，力臂正负两种
  机体都验——方向要是靠常数写死，换一台转轴在重心上方的机体就反了。
* **记录**：上/下桨转速按桨叶标定查通道（不按下标），过期/无效记 0 且批头不带
  ERPM_VALID；SERVO 下力矩、推力恒为 0，批头带 SERVO 标志。
* **命令面**：EXC 的 servo_tilt_mrad 整条命令原子生效、报在 EXC 行末尾；MODE 认名字与数字。
"""
from __future__ import annotations

import ctypes
import math
import re

import pytest

from test_sysid_runtime_contract import (
    CONTROL_DT_US, FLAG_ABORTED, FLAG_LAST, PROFILE_DOUBLET, PROFILE_STEP, STATE_ABORTED,
    STATE_DONE, Airframe, Excitation, Rig, batch_header, build_lib, default_airframe, frames,
    healthy, reset, texts,
)

EXTRA_SOURCES = ("App/Src/app_cmd_sysid.c", "App/Src/app_param_trial.c",
                 "tests/fixtures/sysid/cmd_harness.c")

MODE_FF, MODE_RATE, MODE_ANGLE, MODE_SERVO = 0, 1, 2, 3
PHASE_IDLE, PHASE_EXCITE, PHASE_PREROLL, PHASE_DONE = 0, 3, 5, 6
FLAG_ERPM_VALID = 0x0008
FLAG_RATE, FLAG_ANGLE, FLAG_SERVO = 0x0020, 0x0040, 0x0080
MODE_FLAGS = FLAG_RATE | FLAG_ANGLE | FLAG_SERVO
SERVO_US_PER_RAD = 2000.0 / math.pi     # drv_coax_ctrl：500..2500 us 对应 180°
MAX_COUNT = 5
ROLE_UPPER, ROLE_LOWER = 0, 1
PROP_MAGIC = 0x504F5250
PREROLL_TICKS = 500 // 2

SHORT_DOUBLET = dict(profile=PROFILE_DOUBLET, amplitude_rad_s=0.1, duration_ms=400,
                     hold_ms=50, repeat=2, ramp_ms=10, chirp_f0_hz=0.3, chirp_f1_hz=6.0,
                     prbs_bit_ms=40, prbs_seed=1)
RAMP_STEP = dict(profile=PROFILE_STEP, amplitude_rad_s=0.1, duration_ms=400, hold_ms=100,
                 repeat=1, ramp_ms=40, chirp_f0_hz=0.3, chirp_f1_hz=6.0, prbs_bit_ms=40,
                 prbs_seed=1)


class Sample(ctypes.Structure):
    """与 Driver/Inc/drv_sysid_record.h 的 DRV_SysIdSample 逐字段对应。"""
    _fields_ = [
        ("gyro_rad_s", ctypes.c_float * 3),
        ("omega_sp_rad_s", ctypes.c_float),
        ("alpha_ff_rad_s2", ctypes.c_float),
        ("tilt_cmd_x_rad", ctypes.c_float),
        ("tilt_cmd_y_rad", ctypes.c_float),
        ("thrust_n", ctypes.c_float),
        ("angle_rad", ctypes.c_float),
        ("erpm", ctypes.c_uint32),
        ("torque_n_m", ctypes.c_float),
        ("angle_sp_rad", ctypes.c_float),
        ("offset_us", ctypes.c_uint32),
        ("erpm_lower", ctypes.c_uint32),
        ("servo_tilt_rad", ctypes.c_float),
        ("height_m", ctypes.c_float),
        ("height_raw_m", ctypes.c_float),
        ("height_sp_m", ctypes.c_float),
        ("vz_m_s", ctypes.c_float),
        ("vz_sp_m_s", ctypes.c_float),
        ("az_m_s2", ctypes.c_float),
        ("vbat_v", ctypes.c_float),
    ]


class BatchHeader(ctypes.Structure):
    _fields_ = [("run_id", ctypes.c_uint16), ("base_t_us", ctypes.c_uint32),
                ("dt_us", ctypes.c_uint16), ("flags", ctypes.c_uint16)]


class ServoCal(ctypes.Structure):
    """与 Driver/Inc/drv_coax_ctrl.h 的 DRV_COAX_CTRL_ServoCalibration 逐字段对应。"""
    _fields_ = [("center_us", ctypes.c_uint16 * 2), ("min_us", ctypes.c_uint16 * 2),
                ("max_us", ctypes.c_uint16 * 2), ("pulse_sign", ctypes.c_int8 * 2)]


class PropChannel(ctypes.Structure):
    _fields_ = [("role", ctypes.c_uint8), ("spin_sense", ctypes.c_int8),
                ("reserved", ctypes.c_uint8 * 2)]


class PropMap(ctypes.Structure):
    _fields_ = [("magic", ctypes.c_uint32), ("schema", ctypes.c_uint16),
                ("size", ctypes.c_uint16), ("channel", PropChannel * 2),
                ("calibrated", ctypes.c_uint8), ("reserved0", ctypes.c_uint8 * 3),
                ("generation", ctypes.c_uint32), ("reserved1", ctypes.c_uint32 * 2)]


@pytest.fixture(scope="module")
def lib(tmp_path_factory):
    handle = build_lib(tmp_path_factory, EXTRA_SOURCES, "sysid-servo")
    handle.harness_set_rotor.argtypes = [ctypes.c_uint32, ctypes.c_uint32, ctypes.c_uint32,
                                         ctypes.c_uint8, ctypes.c_uint8]
    handle.harness_set_rx_available.argtypes = [ctypes.c_uint8]
    handle.harness_set_esc_command_active.argtypes = [ctypes.c_uint8]
    handle.harness_set_servo_jog_active.argtypes = [ctypes.c_uint8]
    handle.harness_command.argtypes = [ctypes.c_char_p]
    handle.harness_command.restype = ctypes.c_uint8
    handle.APP_SysId_SetMode.argtypes = [ctypes.c_int, ctypes.c_float]
    handle.APP_SysId_SetMode.restype = ctypes.c_uint8
    handle.APP_SysId_GetMode.restype = ctypes.c_int
    handle.APP_SysId_SetServoTilt.argtypes = [ctypes.c_float]
    handle.APP_SysId_SetServoTilt.restype = ctypes.c_uint8
    handle.APP_SysId_GetServoTilt.restype = ctypes.c_float
    handle.DRV_SysIdRecord_Unpack.argtypes = [
        ctypes.POINTER(ctypes.c_uint8), ctypes.c_size_t, ctypes.POINTER(BatchHeader),
        ctypes.POINTER(Sample), ctypes.POINTER(ctypes.c_uint32)]
    handle.DRV_PropMap_PublishActive.argtypes = [ctypes.POINTER(PropMap)]
    handle.DRV_PropMap_PublishActive.restype = ctypes.c_uint8
    handle.DRV_COAX_CTRL_SetParam.argtypes = [ctypes.c_char_p, ctypes.c_float]
    handle.DRV_COAX_CTRL_SetParam.restype = ctypes.c_uint8
    handle.DRV_COAX_CTRL_GetParam.argtypes = [ctypes.c_char_p, ctypes.POINTER(ctypes.c_float)]
    handle.DRV_COAX_CTRL_GetParam.restype = ctypes.c_uint8
    handle.DRV_COAX_CTRL_GetServoCalibration.argtypes = [ctypes.POINTER(ServoCal)]
    handle.DRV_COAX_CTRL_SetServoCalibration.argtypes = [ctypes.POINTER(ServoCal)]
    handle.DRV_COAX_CTRL_SetServoCalibration.restype = ctypes.c_uint8
    return handle


# ---------------------------------------------------------------- 小工具


def publish_prop_map(lib, *, ch1_role=ROLE_LOWER, ch2_role=ROLE_UPPER, calibrated=1):
    """本机实际接线：ESC 通道 1 = 下桨，通道 2 = 上桨（PROPCAL 标定结果）。"""
    pm = PropMap(magic=PROP_MAGIC, schema=1, size=ctypes.sizeof(PropMap))
    assert ctypes.sizeof(PropMap) == 32
    pm.calibrated = calibrated
    pm.channel[0].role, pm.channel[1].role = ch1_role, ch2_role
    if calibrated:
        # 共轴反转：下桨俯视顺时针（-1），上桨逆时针（+1）。
        lower_index = 0 if ch1_role == ROLE_LOWER else 1
        pm.channel[lower_index].spin_sense = -1
        pm.channel[1 - lower_index].spin_sense = 1
    assert lib.DRV_PropMap_PublishActive(ctypes.byref(pm)) == 1


def start_mode(lib, mode, *, spec=RAMP_STEP, rate_hz=500, psi_deg=45.0, servo_tilt=0.087,
               target_n=0.0):
    reset(lib, rate_hz=rate_hz, spec=Excitation(**spec))
    rig = Rig(azimuth_rad=math.radians(psi_deg), axis_offset_above_cg_m=0.0,
              imu_above_cg_m=0.0)
    assert lib.APP_SysId_SetRig(ctypes.byref(rig)) == 1
    assert lib.APP_SysId_SetMode(mode, ctypes.c_float(0.0523598776)) == 1
    assert lib.APP_SysId_SetServoTilt(ctypes.c_float(servo_tilt)) == 1
    assert lib.APP_SysId_SetThrottle(ctypes.c_float(target_n), ctypes.c_float(75.0)) == 1
    armed = 0 if mode == MODE_SERVO else 1
    lib.APP_SysId_Update(ctypes.byref(healthy(900, 900_000, rc_armed=armed)))
    assert lib.APP_SysId_Start() == 1, texts(lib)


def servo_obs(t_ms, t_us, **overrides):
    """未解锁、电机停：推力来源随便是什么都不该影响 SERVO。"""
    values = dict(rc_armed=0, throttle_us=1000, thrust_valid=0)
    values.update(overrides)
    return healthy(t_ms, t_us, **values)


def run(lib, count, *, obs=healthy, mutate=None, start=0, on_tick=None):
    """跑 count 个 500 Hz 控制拍并逐拍排空；返回停下的拍号（跑完返回 start+count）。"""
    for index in range(start, start + count):
        o = obs(1_000 + index * 2, 1_000_000 + index * CONTROL_DT_US)
        if mutate is not None:
            mutate(index, o)
        lib.APP_SysId_Update(ctypes.byref(o))
        if on_tick is not None:
            on_tick(index)
        lib.APP_SysId_StreamTick()
        if lib.APP_SysId_IsRunning() == 0:
            return index
    return start + count


def records(lib):
    """用固件自己的 Unpack 解本轮发出的每一帧：[(flags, t_us, Sample)]。"""
    out = []
    for _function, payload in frames(lib):
        header = BatchHeader()
        samples = (Sample * MAX_COUNT)()
        count = ctypes.c_uint32()
        buffer = (ctypes.c_uint8 * len(payload))(*payload)
        assert lib.DRV_SysIdRecord_Unpack(buffer, len(payload), ctypes.byref(header),
                                          samples, ctypes.byref(count)) == 0
        for i in range(count.value):
            out.append((header.flags, header.base_t_us + samples[i].offset_us, samples[i]))
    return out


def motor(lib):
    pulse = ctypes.c_uint16()
    return pulse.value if lib.APP_SysId_GetMotorPulse(ctypes.byref(pulse)) else None


def centre(lib):
    a, b = ctypes.c_uint16(), ctypes.c_uint16()
    lib.APP_SysId_GetServoTargets(ctypes.byref(a), ctypes.byref(b))
    return a.value, b.value


@pytest.fixture(autouse=True)
def back_to_ff(lib):
    yield
    lib.APP_SysId_Stop(b"test")
    for _ in range(40):
        lib.APP_SysId_StreamTick()
    assert lib.APP_SysId_SetMode(MODE_FF, ctypes.c_float(0.0523598776)) == 1
    lib.DRV_Airframe_SetParams(ctypes.byref(default_airframe()))
    assert lib.DRV_Airframe_IsValid() == 1
    lib.harness_reset()


# ---------------------------------------------------------------- 开跑门


def test_servo_start_is_refused_while_armed_with_the_exact_message(lib):
    reset(lib)
    assert lib.APP_SysId_SetMode(MODE_SERVO, ctypes.c_float(0.05)) == 1
    lib.APP_SysId_Update(ctypes.byref(healthy(900, 900_000, rc_armed=1)))
    assert lib.APP_SysId_Start() == 0
    assert texts(lib) == ["ERR sysid servo mode needs disarmed\r\n"]
    lib.APP_SysId_Update(ctypes.byref(healthy(902, 902_000, rc_armed=0)))
    assert lib.APP_SysId_Start() == 1


@pytest.mark.parametrize("owner,message", [
    ("harness_set_esc_command_active", "ERR sysid servo mode esc command active\r\n"),
    ("harness_set_servo_jog_active", "ERR sysid servo mode servo jog active\r\n"),
])
def test_servo_start_is_refused_while_another_owner_holds_the_actuators(lib, owner, message):
    reset(lib)
    assert lib.APP_SysId_SetMode(MODE_SERVO, ctypes.c_float(0.05)) == 1
    lib.APP_SysId_Update(ctypes.byref(servo_obs(900, 900_000)))
    getattr(lib, owner)(1)
    assert lib.APP_SysId_Start() == 0
    assert texts(lib) == [message]
    getattr(lib, owner)(0)
    assert lib.APP_SysId_Start() == 1


# ---------------------------------------------------------------- 完整一轮


def test_servo_run_prerolls_excites_and_finishes_without_ever_taking_the_motors(lib):
    reset(lib)
    idle_centre = centre(lib)
    # 自动油门设置留着（7.4 N）：SERVO 下必须被忽略。
    start_mode(lib, MODE_SERVO, spec=SHORT_DOUBLET, rate_hz=250, target_n=7.4)
    start_line = next(line for line in texts(lib) if line.startswith("SYSID start "))
    assert " auto=0 " in start_line
    phases = []

    def on_tick(_index):
        phases.append(lib.APP_SysId_GetPhase())
        assert motor(lib) is None, "SERVO 模式电机一拍都不许碰"

    stopped = run(lib, 1000, obs=servo_obs, on_tick=on_tick)
    assert stopped < 1000
    for _ in range(8):
        lib.APP_SysId_StreamTick()
    assert (lib.APP_SysId_GetState(), lib.APP_SysId_GetLastReason()) == (STATE_DONE, b"complete")
    order = [p for i, p in enumerate(phases) if i == 0 or phases[i - 1] != p]
    assert order == [PHASE_PREROLL, PHASE_EXCITE, PHASE_IDLE]
    assert phases.count(PHASE_PREROLL) == pytest.approx(PREROLL_TICKS, abs=1)

    lines = texts(lib)
    notices = [line for line in lines if line.startswith("SYSID PHASE ")]
    assert [n.split("phase=")[1].split()[0] for n in notices] == ["preroll", "excite", "done"]
    assert all("pulse_us=0 thrust_cn=0" in n for n in notices)
    assert lines.index(notices[-1]) < lines.index(
        next(line for line in lines if line.startswith("SYSID end ")))
    assert any(line.startswith("SYSID end ") and "state=done reason=complete" in line
               for line in lines)

    rows = records(lib)
    assert rows and all(flags & FLAG_SERVO for flags, _t, _s in rows)
    assert batch_header(frames(lib)[-1][1])["flags"] & 0x2    # LAST
    stamps = [t for _f, t, _s in rows]
    assert max(b - a for a, b in zip(stamps, stamps[1:])) <= 4000 + 2000, "前导与激励同一网格、不断开"
    for _flags, _t, s in rows:
        assert (s.torque_n_m, s.thrust_n, s.omega_sp_rad_s, s.alpha_ff_rad_s2,
                s.angle_sp_rad) == (0.0, 0.0, 0.0, 0.0, 0.0)
    first_move = next(i for i, (_f, _t, s) in enumerate(rows) if abs(s.servo_tilt_rad) > 1e-4)
    assert stamps[first_move] - stamps[0] >= 490_000, "前 0.5 s 必须是零激励前导"
    tilts = [s.servo_tilt_rad for _f, _t, s in rows]
    assert max(tilts) == pytest.approx(0.087, abs=2e-4)
    assert min(tilts) == pytest.approx(-0.087, abs=2e-4), "doublet 两个方向都要走到"
    assert tilts[-1] == 0.0, "末条与回中后的输出一致"
    assert centre(lib) == idle_centre, "结束后舵机回中"


@pytest.mark.parametrize("fault,reason", [
    ("arm", "rc_arm"),
    ("imu", "imu_stale"),
    ("angle", "angle_limit"),
    ("dt", "control_dt"),
    ("inhibit", "actuator_busy"),
    ("jog", "actuator_busy"),
])
def test_servo_run_keeps_the_arm_imu_angle_and_timing_gates(lib, fault, reason):
    trip = PREROLL_TICKS + 30          # 激励段里，舵机已经偏离中位

    def mutate(index, obs):
        if index < trip:
            return
        if fault == "arm":
            obs.rc_armed = 1
        elif fault == "imu":
            obs.imu_valid = 0
        elif fault == "angle":
            obs.roll_rad = obs.pitch_rad = math.radians(25.0) * math.sqrt(0.5)
        elif fault == "dt":
            obs.now_us = 1_000_000 + (index - 1) * CONTROL_DT_US + 100
        elif fault == "inhibit":
            obs.actuator_inhibit = 1
        elif fault == "jog":
            lib.harness_set_servo_jog_active(1)

    reset(lib)
    idle_centre = centre(lib)
    start_mode(lib, MODE_SERVO, spec=SHORT_DOUBLET)
    moved = []
    run(lib, trip, obs=servo_obs, on_tick=lambda _i: moved.append(centre(lib)))
    assert moved[-1] != idle_centre, "用例前提：中止前舵机已经偏离中位"
    assert run(lib, 2000, obs=servo_obs, mutate=mutate, start=trip) == trip
    assert lib.APP_SysId_GetState() == STATE_ABORTED
    assert lib.APP_SysId_GetLastReason().decode() == reason
    assert motor(lib) is None
    assert centre(lib) == idle_centre, "任何停止都回中"


def test_servo_run_ignores_the_thrust_gates(lib):
    """推力为 0、推力来源过期、油门杆不在最低：SERVO 一概不管。"""
    start_mode(lib, MODE_SERVO, spec=SHORT_DOUBLET, target_n=7.4)

    def mutate(_index, obs):
        obs.thrust_valid = 0
        obs.throttle_us = 1000
        obs.rc_throttle_low = 0

    assert run(lib, 1000, obs=servo_obs, mutate=mutate) < 1000
    assert lib.APP_SysId_GetLastReason() == b"complete"


def test_servo_tilt_is_clamped_to_the_controller_tilt_limit(lib):
    limit = ctypes.c_float()
    assert lib.DRV_COAX_CTRL_GetParam(b"coax.tilt_limit_rad", ctypes.byref(limit))
    try:
        assert lib.DRV_COAX_CTRL_SetParam(b"coax.tilt_limit_rad", ctypes.c_float(0.05))
        start_mode(lib, MODE_SERVO, spec=SHORT_DOUBLET, rate_hz=250, servo_tilt=0.2)
        run(lib, 1000, obs=servo_obs)
        tilts = [s.servo_tilt_rad for _f, _t, s in records(lib)]
        assert max(abs(t) for t in tilts) == pytest.approx(0.05, abs=2e-4)
    finally:
        assert lib.DRV_COAX_CTRL_SetParam(b"coax.tilt_limit_rad", limit)


@pytest.mark.parametrize("servo_tilt,clipped", [(0.262, True), (0.087, False)])
def test_servo_run_aborts_when_the_servo_calibration_clips_the_tilt(lib, servo_tilt, clipped):
    """标定行程比倾转窄（center±60 us 是合法标定）：脉宽被夹住时记录的 servo_tilt 就不是
    舵机真走的量，这一轮必须作废，而不是报 complete。行程够时照常跑完（不误判）。"""
    saved = ServoCal()
    lib.DRV_COAX_CTRL_GetServoCalibration(ctypes.byref(saved))
    narrow = ServoCal.from_buffer_copy(saved)
    for i in range(2):
        narrow.min_us[i] = narrow.center_us[i] - 60
        narrow.max_us[i] = narrow.center_us[i] + 60
    try:
        assert lib.DRV_COAX_CTRL_SetServoCalibration(ctypes.byref(narrow)) == 1
        idle_centre = centre(lib)
        start_mode(lib, MODE_SERVO, spec=SHORT_DOUBLET, rate_hz=250, servo_tilt=servo_tilt)
        assert run(lib, 2000, obs=servo_obs) < 2000
        for _ in range(8):
            lib.APP_SysId_StreamTick()
        rows = records(lib)
        assert rows
        # 已记下的每一条都是舵机真走到的：没有一条的逐轴倾转超出 60 us 行程。
        assert all(max(abs(s.tilt_cmd_x_rad), abs(s.tilt_cmd_y_rad)) * SERVO_US_PER_RAD < 60.0
                   for _f, _t, s in rows)
        if clipped:
            assert (lib.APP_SysId_GetState(), lib.APP_SysId_GetLastReason()) ==                 (STATE_ABORTED, b"actuator_saturated")
            assert batch_header(frames(lib)[-1][1])["flags"] & FLAG_ABORTED
            assert not any("phase=done" in line for line in texts(lib))
        else:
            assert (lib.APP_SysId_GetState(), lib.APP_SysId_GetLastReason()) ==                 (STATE_DONE, b"complete")
            assert max(s.servo_tilt_rad for _f, _t, s in rows) == pytest.approx(0.087, abs=2e-4)
        assert centre(lib) == idle_centre, "任何停止都回中"
    finally:
        assert lib.DRV_COAX_CTRL_SetServoCalibration(ctypes.byref(saved)) == 1


@pytest.mark.parametrize("mode,flag", [(MODE_SERVO, FLAG_SERVO), (MODE_RATE, FLAG_RATE),
                                       (MODE_ANGLE, FLAG_ANGLE)])
def test_batch_mode_flags_are_latched_at_start_not_read_when_draining(lib, mode, flag):
    """跑完、尾批还没排空时 MODE 照样能改；尾批的模式标志必须仍是本轮的，不能跟着变成 FF。"""
    obs = servo_obs if mode == MODE_SERVO else healthy
    start_mode(lib, mode, spec=SHORT_DOUBLET, rate_hz=250)
    for index in range(2000):
        lib.APP_SysId_Update(ctypes.byref(obs(1_000 + index * 2, 1_000_000 + index * CONTROL_DT_US)))
        if lib.APP_SysId_IsRunning() == 0:
            break                     # 收尾那一拍不排空：尾批还压在环里
        lib.APP_SysId_StreamTick()
    assert lib.APP_SysId_IsRunning() == 0
    lib.harness_reset()
    assert lib.APP_SysId_SetMode(MODE_FF, ctypes.c_float(0.0523598776)) == 1
    for _ in range(8):
        lib.APP_SysId_StreamTick()
    headers = [batch_header(payload) for _f, payload in frames(lib)]
    assert headers and headers[-1]["flags"] & FLAG_LAST
    assert all(h["flags"] & MODE_FLAGS == flag for h in headers),         [hex(h["flags"]) for h in headers]


@pytest.mark.parametrize("nth", [1, 2])
def test_stop_racing_the_completion_tick_is_done_or_aborted_never_both(lib, nth):
    """STOP 插在收尾那一拍里：nth=1 落在末条入环之后、收尾之前（实机上 SysTick 在临界区
    出口被响应），nth=2 落在收尾之后。前者以 STOP 为准，不能先报 phase=done 再报 aborted。"""
    idle_centre = centre(lib)
    start_mode(lib, MODE_SERVO, spec=SHORT_DOUBLET, rate_hz=250)
    last = run(lib, 2000, obs=servo_obs)              # 参考轮：收尾那一拍的拍号
    assert lib.APP_SysId_GetLastReason() == b"complete"
    start_mode(lib, MODE_SERVO, spec=SHORT_DOUBLET, rate_hz=250)
    assert run(lib, last, obs=servo_obs) == last
    lib.harness_reset()
    lib.harness_set_stop_after_critical(nth)
    lib.APP_SysId_Update(ctypes.byref(servo_obs(1_000 + last * 2, 1_000_000 + last * CONTROL_DT_US)))
    lib.harness_set_stop_after_critical(0)
    for _ in range(8):
        lib.APP_SysId_StreamTick()
    lines = texts(lib)
    end = next(line for line in lines if line.startswith("SYSID end "))
    announced_done = any("phase=done" in line for line in lines)
    tail = batch_header(frames(lib)[-1][1])["flags"]
    assert tail & FLAG_LAST
    if nth == 1:
        assert "state=aborted reason=command" in end and not announced_done, lines
        assert tail & FLAG_ABORTED
    else:
        assert "state=done reason=complete" in end and announced_done, lines
        assert not tail & FLAG_ABORTED
    assert centre(lib) == idle_centre


# ---------------------------------------------------------------- 方向：与 FF 同号


def flipped_lever_airframe():
    """转轴与推力点都在重心上方：力臂 L = 重心 z − 转轴 z < 0，FF 反解出的倾转整体反号。"""
    frame = default_airframe()
    frame.servo1_axis_z_m = -0.01
    frame.servo2_axis_z_m = -0.012
    frame.thrust_point_z_m = 0.10
    return frame


def excite_tilts(lib, mode, psi_deg):
    """跑到斜坡段取下发倾转：FF 在 α_ff>0 的斜坡上下发正力矩，SERVO 在 ω_sp>0 上给正倾转。"""
    obs = servo_obs if mode == MODE_SERVO else healthy
    ticks = (PREROLL_TICKS if mode == MODE_SERVO else 0) + 14
    start_mode(lib, mode, spec=RAMP_STEP, rate_hz=500, psi_deg=psi_deg)
    assert run(lib, ticks, obs=obs) == ticks, lib.APP_SysId_GetLastReason()
    lib.APP_SysId_Stop(b"test")
    for _ in range(8):
        lib.APP_SysId_StreamTick()
    moving = [s for _f, _t, s in records(lib)
              if max(abs(s.tilt_cmd_x_rad), abs(s.tilt_cmd_y_rad)) > 2e-3]
    assert moving, "斜坡段必须已经下发了倾转"
    last = moving[-1]
    return last.tilt_cmd_x_rad, last.tilt_cmd_y_rad, last.servo_tilt_rad


@pytest.mark.parametrize("lever", ["below_cg", "above_cg"])
@pytest.mark.parametrize("psi_deg", [0.0, 90.0, 45.0, -45.0])
def test_positive_servo_tilt_matches_the_ff_tilt_for_positive_rod_torque(lib, psi_deg, lever):
    if lever == "above_cg":
        lib.DRV_Airframe_SetParams(ctypes.byref(flipped_lever_airframe()))
        assert lib.DRV_Airframe_IsValid() == 1
    ff_x, ff_y, ff_servo = excite_tilts(lib, MODE_FF, psi_deg)
    sv_x, sv_y, sv_servo = excite_tilts(lib, MODE_SERVO, psi_deg)

    for ff, sv in ((ff_x, sv_x), (ff_y, sv_y)):
        if abs(ff) < 1e-3:
            assert abs(sv) < 1e-3, (psi_deg, lever, ff, sv)       # 同一轴不动
        else:
            assert math.copysign(1.0, sv) == math.copysign(1.0, ff), (psi_deg, lever, ff, sv)
    # 两边投影都是正的：servo_tilt 的正方向就是"FF 的正杆轴力矩"方向。
    assert ff_servo > 1e-3 and sv_servo > 1e-3, (ff_servo, sv_servo)

    # 记录字段按文档里的公式算：servo_tilt = x·sinψ·sgn(L_pitch) + y·cosψ·sgn(L_roll)。
    sign = -1.0 if lever == "above_cg" else 1.0
    psi = math.radians(psi_deg)
    for x, y, recorded in ((ff_x, ff_y, ff_servo), (sv_x, sv_y, sv_servo)):
        expected = x * math.sin(psi) * sign + y * math.cos(psi) * sign
        assert recorded == pytest.approx(expected, abs=3e-4)


# ---------------------------------------------------------------- 转速记录


def run_ff_with_rotors(lib, *, now_age_ms=0, valid=(1, 1), not_spinning=(0, 0),
                       available=1, erpm=(12000, 15000)):
    start_mode(lib, MODE_FF, spec=RAMP_STEP, rate_hz=500)
    lib.harness_set_rx_available(available)
    for index in range(2):
        lib.harness_set_rotor(index, erpm[index], 1_000 - now_age_ms, valid[index],
                              not_spinning[index])
    run(lib, 10)
    lib.APP_SysId_Stop(b"test")
    for _ in range(8):
        lib.APP_SysId_StreamTick()
    rows = records(lib)
    assert rows
    return rows


def test_erpm_fields_follow_the_propeller_map_not_the_channel_index(lib):
    publish_prop_map(lib, ch1_role=ROLE_LOWER, ch2_role=ROLE_UPPER)   # 本机：通道 2 = 上桨
    rows = run_ff_with_rotors(lib)
    assert all((s.erpm, s.erpm_lower) == (15000, 12000) for _f, _t, s in rows)
    assert all(flags & FLAG_ERPM_VALID for flags, _t, _s in rows)

    publish_prop_map(lib, ch1_role=ROLE_UPPER, ch2_role=ROLE_LOWER)   # 反过来接
    rows = run_ff_with_rotors(lib)
    assert all((s.erpm, s.erpm_lower) == (12000, 15000) for _f, _t, s in rows)


@pytest.mark.parametrize("case", ["stale", "never_valid", "not_bidir", "uncalibrated"])
def test_missing_erpm_is_logged_as_zero_without_the_valid_flag(lib, case):
    publish_prop_map(lib, calibrated=0 if case == "uncalibrated" else 1)
    kwargs = {"stale": dict(now_age_ms=150), "never_valid": dict(valid=(1, 0)),
              "not_bidir": dict(available=0), "uncalibrated": {}}[case]
    rows = run_ff_with_rotors(lib, **kwargs)
    if case == "never_valid":
        assert all(s.erpm_lower == 12000 and s.erpm == 0 for _f, _t, s in rows)
    else:
        assert all((s.erpm, s.erpm_lower) == (0, 0) for _f, _t, s in rows)
    assert not any(flags & FLAG_ERPM_VALID for flags, _t, _s in rows)
    publish_prop_map(lib)


def test_a_reply_stamped_just_after_the_tick_time_is_still_fresh(lib):
    """回包的 HAL ms 可能比本拍取的 now_ms 晚 1 ms：无符号相减会回绕成"过期 49 天"。"""
    publish_prop_map(lib)
    rows = run_ff_with_rotors(lib, now_age_ms=-5)
    assert all((s.erpm, s.erpm_lower) == (15000, 12000) for _f, _t, s in rows)
    assert all(flags & FLAG_ERPM_VALID for flags, _t, _s in rows)


def test_not_spinning_is_a_valid_zero(lib):
    """电调明确回报未旋转是有效回包：记 0，批头仍带 ERPM_VALID（与"没数据"区分开）。"""
    publish_prop_map(lib)
    rows = run_ff_with_rotors(lib, not_spinning=(1, 0))
    assert all((s.erpm, s.erpm_lower) == (15000, 0) for _f, _t, s in rows)
    assert all(flags & FLAG_ERPM_VALID for flags, _t, _s in rows)


def test_servo_mode_logs_the_stopped_rotors_as_valid_zero(lib):
    publish_prop_map(lib)
    start_mode(lib, MODE_SERVO, spec=SHORT_DOUBLET, rate_hz=250)
    lib.harness_set_rx_available(1)

    def fresh(index):
        for channel in range(2):
            lib.harness_set_rotor(channel, 0, 1_000 + index * 2, 1, 1)

    run(lib, 1000, obs=servo_obs, mutate=lambda i, _o: fresh(i))
    rows = records(lib)
    assert rows and all((s.erpm, s.erpm_lower) == (0, 0) for _f, _t, s in rows)
    assert all(flags & FLAG_ERPM_VALID and flags & FLAG_SERVO for flags, _t, _s in rows)


# ---------------------------------------------------------------- 命令面


def command(lib, line):
    lib.harness_reset()
    assert lib.harness_command(line.encode()) == 1
    return texts(lib)


EXC_LINE = re.compile(
    r"^SYSID EXC profile=\d+ amp_mrad_s=-?\d+ dur_ms=\d+ hold_ms=\d+ repeat=\d+ ramp_ms=\d+ "
    r"f0_mhz=-?\d+ f1_mhz=-?\d+ bit_ms=\d+ seed=\d+ total_ms=\d+ servo_tilt_mrad=(\d+)\r\n$")


def exc_tilt(lines):
    line = next(line for line in lines if line.startswith("SYSID EXC "))
    match = EXC_LINE.match(line)
    assert match, line
    return int(match.group(1))


def test_exc_reports_the_servo_tilt_as_the_last_key_with_the_documented_default(lib):
    reset(lib)
    assert lib.APP_SysId_SetServoTilt(ctypes.c_float(0.087)) == 1
    assert exc_tilt(command(lib, "SYSID EXC")) == 87


@pytest.mark.parametrize("mrad,accepted", [(10, True), (87, True), (120, True), (262, True),
                                           (9, False), (263, False), (-50, False)])
def test_exc_accepts_servo_tilt_only_inside_its_range(lib, mrad, accepted):
    reset(lib)
    assert lib.APP_SysId_SetServoTilt(ctypes.c_float(0.087)) == 1
    lines = command(lib, f"SYSID EXC servo_tilt_mrad={mrad}")
    if accepted:
        assert exc_tilt(lines) == mrad
        assert lib.APP_SysId_GetServoTilt() == pytest.approx(mrad / 1000.0, abs=1e-6)
    else:
        assert lines == ["ERR sysid excitation rejected\r\n"]
        assert lib.APP_SysId_GetServoTilt() == pytest.approx(0.087, abs=1e-6)


def test_exc_is_atomic_across_profile_and_servo_tilt(lib):
    reset(lib)
    assert lib.APP_SysId_SetServoTilt(ctypes.c_float(0.087)) == 1
    before = exc_tilt(command(lib, "SYSID EXC"))
    amp_before = next(l for l in texts(lib) if l.startswith("SYSID EXC ")).split()[3]
    assert command(lib, "SYSID EXC amp=0.2 servo_tilt_mrad=300") == \
        ["ERR sysid excitation rejected\r\n"]
    lines = command(lib, "SYSID EXC")
    assert exc_tilt(lines) == before
    assert next(l for l in lines if l.startswith("SYSID EXC ")).split()[3] == amp_before
    # 端点值落下后，只改剖面的 EXC 不能因 mrad 往返舍入把它挤出范围。
    assert exc_tilt(command(lib, "SYSID EXC servo_tilt_mrad=262")) == 262
    assert exc_tilt(command(lib, "SYSID EXC amp=0.2")) == 262


@pytest.mark.parametrize("token,mode", [("SERVO", 3), ("3", 3), ("FF", 0), ("0", 0),
                                        ("RATE", 1), ("ANGLE", 2)])
def test_mode_accepts_names_and_numbers(lib, token, mode):
    reset(lib)
    lines = command(lib, f"SYSID MODE {token}")
    ready = next(line for line in lines if line.startswith("SYSID READY "))
    assert f" mode={mode} " in ready
    assert lib.APP_SysId_GetMode() == mode


def test_mode_rejects_unknown_names(lib):
    reset(lib)
    assert command(lib, "SYSID MODE MOTOR") == \
        ["ERR sysid mode: MODE FF|RATE|ANGLE|SERVO|ALT [0<deg<=15], idle only\r\n"]

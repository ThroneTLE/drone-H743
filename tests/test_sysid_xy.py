"""水平槽 XY 速度/位置辨识（SYSID MODE XY = 5，R-XYID-1）—— 在宿主上跑真固件。

App/Src/app_sysid.c + app_sysid_xy.c + app_cmd_sysid.c + 真分配器 + 真 drv_position_control
（生产位置/速度环，x 通道），平台边界见 tests/fixtures/sysid/。两侧契约：doc/sysid-xy-contract.md。
宿主装置（build_lib / Capture / 文本与帧解码）直接复用 test_sysid_alt.py。

对象是"水平槽里的一维质点"（XYPlant）：机体绕杆倾斜，推力的水平分量沿槽 u = (−sin ψ, cos ψ)
产生加速度 g·tan θ（θ 由横滚/俯仰按推力方向 z' = (sin p cos r, −sin r, cos p cos r) 独立算出，
不复用固件公式），带库仑摩擦（静摩擦 > 动摩擦）、姿态一阶滞后、光流位置/速度的测量延迟与噪声。
这样"+u 的倾角使 +u 加速"是固件公式与物理模型对得上才成立，不是自证。

判据按台架上真会出事的地方排：

* **命令面**：`SYSID XY` 回显一行、拒绝整条不生效、上电默认 tilt/150/0；
* **开跑前检**：自动油门、幅值上限、光流/测距有效且新鲜；**溯源**：SYSID XYSTART 紧跟 start 行；
* **阶段与出力**：RAMP_UP → SETTLE → PREROLL → EXCITE → RAMP_DOWN，推力恒为 target_n；
* **方向**：u 随 ψ 变、符号正确；tilt 注入开环不跑环；vel/pos 用生产 x 通道增益；
* **安全**：出窗/光流失效软停（推力一拍掉不多、最后报理由），遥控/IMU 当拍交还；
* **记录**：7 个尾字段的 XY 含义、批头 XY 位；`TiltFromForce` 生产逐位不变；文本行长预算。
"""
from __future__ import annotations

import ctypes
import math
import random
import re
import shutil
import struct
import subprocess
import sys
from pathlib import Path

import pytest

from test_sysid_runtime_contract import (
    CONTROL_DT_US, PROFILE_DOUBLET, PROFILE_STEP, STATE_ABORTED, STATE_DONE, Excitation, Rig,
    build_lib, healthy, reset, texts,
)
from test_sysid_alt import (
    AIRFRAME_KG, EXTRA_SOURCES, G, IDLE_US, PHASE_EXCITE, PHASE_IDLE, PHASE_PREROLL,
    PHASE_RAMP_DOWN, PHASE_RAMP_UP, PHASE_SETTLE, WEIGHT_N, Capture, command, max_tick_drop,
    motor, phase_runs, prepare_handle, schema_of, step_spec, teardown_run, thrust_of, _worst_line,
)

ROOT = Path(__file__).resolve().parents[1]

MODE_XY = 5
FLAG_XY, FLAG_ALT = 0x0400, 0x0100
XY_SEQUENCE = [PHASE_RAMP_UP, PHASE_SETTLE, PHASE_PREROLL, PHASE_EXCITE, PHASE_RAMP_DOWN]
RAMP_TICKS, SETTLE_TICKS, PREROLL_TICKS, DOWN_TICKS = 750, 1000, 250, 500
TARGET_N = WEIGHT_N                 # 托住机体的推力
DT = CONTROL_DT_US * 1e-6
PSI = math.pi / 4                   # reset() 装的杆方位角
ORIGIN = (0.31, -0.22)              # 光流位置原点不在 0：位置必须相对开跑那一刻
XY_FIELDS = ("height", "height_raw", "height_sp", "vz", "vz_sp", "az", "vbat")

CLOSED_DOUBLET = dict(profile=PROFILE_DOUBLET, amplitude_rad_s=0.05, duration_ms=500,
                      hold_ms=250, repeat=1, ramp_ms=20, chirp_f0_hz=0.3, chirp_f1_hz=6.0,
                      prbs_bit_ms=40, prbs_seed=1)


def u_of(psi):
    return (-math.sin(psi), math.cos(psi))


@pytest.fixture(scope="module")
def lib(tmp_path_factory):
    return prepare_xy_handle(prepare_handle(build_lib(tmp_path_factory, EXTRA_SOURCES, "sysid-xy")))


@pytest.fixture(scope="module")
def fresh_lib(tmp_path_factory):
    """一份从没收过 SYSID XY 的库：看上电默认值用。"""
    return prepare_xy_handle(prepare_handle(
        build_lib(tmp_path_factory, EXTRA_SOURCES, "sysid-xy-defaults")))


def prepare_xy_handle(handle):
    f = ctypes.POINTER(ctypes.c_float)
    handle.APP_SysIdXy_GetTiltBias.argtypes = [f, f, f]
    handle.APP_SysIdXy_GetLive.argtypes = [f, f, ctypes.POINTER(ctypes.c_uint8)]
    handle.DRV_COAX_CTRL_TiltFromForce.argtypes = [ctypes.POINTER(ctypes.c_float), f, f]
    handle.DRV_COAX_CTRL_EffectiveMassKg.argtypes = [ctypes.c_float]
    handle.DRV_COAX_CTRL_EffectiveMassKg.restype = ctypes.c_float
    return handle


@pytest.fixture
def coax_params(lib):
    """改生产参数的用例用完要装回去（同一个库在本文件里共用）。"""
    names = ("coax.pos_x_kp", "coax.vel_x_kp", "coax.vel_x_ki", "coax.vel_x_kd",
             "coax.vel_x_i_limit_m_s2", "coax.accel_xy_max_m_s2", "coax.pos_xy_vel_max_m_s",
             "coax.tilt_limit_rad", "coax.hover_thrust_n")
    saved = {}
    for name in names:
        value = ctypes.c_float()
        assert lib.DRV_COAX_CTRL_GetParam(name.encode(), ctypes.byref(value)) == 1, name
        saved[name] = value.value
    yield lambda name, value: lib.DRV_COAX_CTRL_SetParam(name.encode(), ctypes.c_float(value))
    for name, value in saved.items():
        assert lib.DRV_COAX_CTRL_SetParam(name.encode(), ctypes.c_float(value)) == 1


@pytest.fixture
def tuned(coax_params):
    """闭环仿真用的一组增益（仍是生产代码按参数表跑）：默认 0.375 / 0.8 放在无阻尼质点上偏软偏慢。"""
    assert coax_params("coax.pos_x_kp", 1.5) == 1
    assert coax_params("coax.vel_x_kp", 3.0) == 1
    assert coax_params("coax.vel_x_ki", 1.5) == 1
    assert coax_params("coax.vel_x_kd", 0.0) == 1
    return coax_params


# ---------------------------------------------------------------- 水平槽对象


class XYPlant:
    """沿 u 的一维质点。姿态（横滚/俯仰）一阶滞后地跟固件给的目标（att0 = 0 + 倾角偏置），
    水平加速度按实际推力方向算；库仑摩擦：静止时 |a| ≤ fs 不动，动起来 fk 逆着速度。
    光流位置/速度每 10 拍（20 ms，50 Hz）出一个新样本，取 delay_ticks 拍之前的状态并加噪声。"""

    def __init__(self, lib, psi=PSI, *, fs=0.0, fk=0.0, lag_s=0.03, delay_ticks=15,
                 pos_sigma=0.0, vel_sigma=0.0, seed=5, stuck=False):
        self.lib, self.psi = lib, psi
        self.ux, self.uy = u_of(psi)
        self.nx, self.ny = math.cos(psi), math.sin(psi)
        self.fs, self.fk, self.lag_s, self.delay = fs, fk, lag_s, delay_ticks
        self.pos_sigma, self.vel_sigma, self.rng = pos_sigma, vel_sigma, random.Random(seed)
        self.stuck = stuck            # 槽被卡死：不动（看控制器输出用）
        self.s = self.v = self.a_u = 0.0
        self.perp = 0.0
        self.roll = self.pitch = 0.0
        self.calls, self.token = 0, 0
        self.hist = [(0.0, 0.0)] * (delay_ticks + 1)
        self.meas_s = self.meas_v = 0.0
        self.max_speed = 0.0

    def tilt_target(self):
        roll, pitch, accel = ctypes.c_float(), ctypes.c_float(), ctypes.c_float()
        self.lib.APP_SysIdXy_GetTiltBias(ctypes.byref(roll), ctypes.byref(pitch),
                                         ctypes.byref(accel))
        return roll.value, pitch.value

    def sample(self, t_ms):
        """每次构造观测时调：10 次一个新光流样本。"""
        if self.calls % 10 == 0 or self.token == 0:
            self.token = t_ms
            s, v = self.hist[0]
            self.meas_s = s + self.rng.gauss(0.0, self.pos_sigma) if self.pos_sigma else s
            self.meas_v = v + self.rng.gauss(0.0, self.vel_sigma) if self.vel_sigma else v
        self.calls += 1

    def flow_xy(self):
        pos = (ORIGIN[0] + self.meas_s * self.ux + self.perp * self.nx,
               ORIGIN[1] + self.meas_s * self.uy + self.perp * self.ny)
        vel = (self.meas_v * self.ux, self.meas_v * self.uy)
        return pos, vel

    def advance(self, pulse, dt=DT):
        target_roll, target_pitch = self.tilt_target()
        k = min(dt / self.lag_s, 1.0)
        self.roll += k * (target_roll - self.roll)
        self.pitch += k * (target_pitch - self.pitch)
        zx = math.sin(self.pitch) * math.cos(self.roll)
        zy = -math.sin(self.roll)
        zz = math.cos(self.pitch) * math.cos(self.roll)
        thrust_on = pulse is not None
        a_u = G * (zx * self.ux + zy * self.uy) / zz if thrust_on else 0.0
        if self.stuck:
            accel, self.v = 0.0, 0.0
        elif self.v == 0.0:
            accel = 0.0 if abs(a_u) <= self.fs else a_u - math.copysign(self.fk, a_u)
        else:
            accel = a_u - math.copysign(self.fk, self.v)
            if (self.v + accel * dt) * self.v < 0.0 and abs(a_u) <= self.fs:
                self.v, accel = 0.0, 0.0                # 这一拍减速到零：停住
        self.a_u = accel
        self.v += accel * dt
        self.s += self.v * dt
        self.max_speed = max(self.max_speed, abs(self.v))
        self.hist = self.hist[1:] + [(self.s, self.v)]


def xy_obs(t_ms, t_us, plant, **overrides):
    plant.sample(t_ms)
    pos, vel = plant.flow_xy()
    values = dict(throttle_us=IDLE_US, rc_throttle_low=1, height_valid=1,
                  height_sample_ms=t_ms, height_m=0.2, height_raw_m=0.2, vz_m_s=0.0,
                  az_m_s2=0.0, vbat_v=11.876, flow_valid=1, flow_sample_ms=plant.token,
                  flow_vel_m_s=(ctypes.c_float * 2)(*vel), flow_pos_m=(ctypes.c_float * 2)(*pos),
                  roll_rad=plant.roll, pitch_rad=plant.pitch)
    values.update(overrides)
    return healthy(t_ms, t_us, **values)


def xy_line(inject, win_mm, mass_g):
    control = "openloop" if inject == "tilt" else "closed_loop"
    return f"SYSID XY inject={inject} win_mm={win_mm} mass_g={mass_g} control={control}\r\n"


def configure_xy(lib, *, inject="tilt", win_mm=150, mass_g=0):
    line = f"SYSID XY inject={inject} win_mm={win_mm} mass_g={mass_g}"
    assert command(lib, line) == [xy_line(inject, win_mm, mass_g)]


def prepare_xy(lib, *, spec=None, inject="tilt", win_mm=150, mass_g=0, psi=PSI, max_pct=75.0,
               target_n=TARGET_N, rate_hz=50, plant=None, **obs):
    reset(lib, rate_hz=rate_hz, spec=Excitation(**(spec or CLOSED_DOUBLET)))
    rig = Rig(azimuth_rad=psi, axis_offset_above_cg_m=0.0, imu_above_cg_m=0.0)
    assert lib.APP_SysId_SetRig(ctypes.byref(rig)) == 1
    if target_n:
        assert lib.APP_SysId_SetThrottle(ctypes.c_float(target_n), ctypes.c_float(max_pct)) == 1
    assert lib.APP_SysId_SetMode(MODE_XY, ctypes.c_float(0.0523598776)) == 1
    configure_xy(lib, inject=inject, win_mm=win_mm, mass_g=mass_g)
    plant = plant or XYPlant(lib, psi)
    # 没跑时控制拍也调 Update：START 靠它知道解锁/油门杆/光流是否有效。
    for k in range(10):
        lib.APP_SysId_Update(ctypes.byref(xy_obs(880 + 2 * k, 880_000 + 2000 * k, plant, **obs)))
    lib.harness_reset()
    return plant


def start_xy(lib, **kwargs):
    plant = prepare_xy(lib, **kwargs)
    assert lib.APP_SysId_Start() == 1, texts(lib)
    return plant


def run_xy(lib, plant, ticks, *, mutate=None, record=None, capture=None, t0_ms=1_000, after=None):
    """跑 ticks 个 500 Hz 控制拍，质点跟着电机脉宽走。返回停在第几拍。"""
    stopped = ticks
    for index in range(ticks):
        obs = xy_obs(t0_ms + index * 2, t0_ms * 1000 + index * CONTROL_DT_US, plant)
        if mutate is not None:
            mutate(index, obs)
        lib.APP_SysId_Update(ctypes.byref(obs))
        pulse = motor(lib)
        if record is not None:
            record.append((lib.APP_SysId_GetPhase(), pulse, plant.s))
        plant.advance(pulse)
        if after is not None:
            after(index)
        lib.APP_SysId_StreamTick()
        if capture is not None and (lib.harness_frame_total() >= 100 or
                                    lib.harness_text_lines() >= 48):
            capture.flush()
        if lib.APP_SysId_IsRunning() == 0:
            stopped = index
            break
    if capture is not None:
        for _ in range(64):
            lib.APP_SysId_StreamTick()
            if lib.harness_frame_total() >= 100:
                capture.flush()
        capture.flush()
    return stopped


def tilt_bias(lib):
    roll, pitch, accel = ctypes.c_float(), ctypes.c_float(), ctypes.c_float()
    lib.APP_SysIdXy_GetTiltBias(ctypes.byref(roll), ctypes.byref(pitch), ctypes.byref(accel))
    return roll.value, pitch.value, accel.value


def first_tick_in(record, phase, after=0):
    return next(i for i, (p, _pulse, _s) in enumerate(record) if i >= after and p == phase)


EXCITE_START = RAMP_TICKS + SETTLE_TICKS + PREROLL_TICKS       # 第一个 EXCITE 拍的序号


# ---------------------------------------------------------------- 命令面


def test_power_on_defaults_are_tilt_150_0(fresh_lib):
    header = (ROOT / "App/Inc/app_sysid_xy.h").read_text(encoding="utf-8")
    assert re.search(r"#define APP_SYSID_XY_WIN_MM_DEFAULT\s+150U", header)
    assert command(fresh_lib, "SYSID XY") == [xy_line("tilt", 150, 0)]


def test_xy_command_echoes_one_line_with_control(lib):
    lib.APP_SysId_Stop(b"test")
    configure_xy(lib)
    assert command(lib, "SYSID XY") == [xy_line("tilt", 150, 0)]
    assert command(lib, "SYSID XY inject=vel win_mm=200 mass_g=1184") == [
        xy_line("vel", 200, 1184)]
    assert command(lib, "SYSID XY inject=pos") == [xy_line("pos", 200, 1184)]   # 其余保持原值
    assert command(lib, "SYSID XY mass_g=0 win_mm=30") == [xy_line("pos", 30, 0)]
    assert command(lib, "SYSID XY inject=tilt") == [xy_line("tilt", 30, 0)]
    configure_xy(lib)


@pytest.mark.parametrize("args,reason", [
    ("mass_g=499", "range"), ("mass_g=3001", "range"), ("mass_g=70000", "range"),
    ("win_mm=29", "range"), ("win_mm=401", "range"),
    ("inject=up", "usage"), ("inject=break", "usage"), ("mass_g=abc", "usage"),
    ("mass_g=1.5", "usage"), ("foo=1", "usage"), ("win_mm=100 bar", "usage"),
    ("lift_mm=60", "usage"),
])
def test_xy_command_rejects_whole_line_without_touching_the_config(lib, args, reason):
    lib.APP_SysId_Stop(b"test")
    configure_xy(lib, inject="vel", win_mm=200, mass_g=1184)
    assert command(lib, f"SYSID XY inject=pos {args}") == [
        f"SYSID XY event=rejected reason={reason}\r\n"]
    assert command(lib, "SYSID XY") == [xy_line("vel", 200, 1184)]
    configure_xy(lib)


@pytest.mark.parametrize("args", ["mass_g=500", "mass_g=3000", "mass_g=0", "win_mm=30",
                                  "win_mm=400"])
def test_xy_command_accepts_its_range_endpoints(lib, args):
    lib.APP_SysId_Stop(b"test")
    configure_xy(lib)
    line = command(lib, f"SYSID XY {args}")[0]
    assert f" {args} " in line.rstrip() + " "
    configure_xy(lib)


def test_xy_config_is_idle_only_but_readable_while_running(lib):
    plant = start_xy(lib)
    run_xy(lib, plant, 10)
    assert command(lib, "SYSID XY win_mm=50") == ["SYSID XY event=rejected reason=running\r\n"]
    assert command(lib, "SYSID XY") == [xy_line("tilt", 150, 0)]
    teardown_run(lib)


@pytest.mark.parametrize("token", ["XY", "5"])
def test_mode_xy_by_name_and_number(lib, token):
    reset(lib)
    lines = command(lib, f"SYSID MODE {token}")
    assert f" mode={MODE_XY} " in next(line for line in lines if line.startswith("SYSID READY "))
    assert lib.APP_SysId_GetMode() == MODE_XY
    assert lib.APP_SysId_SetMode(MODE_XY + 2, ctypes.c_float(0.05)) == 0   # 6 = YAW 已有，7 起非法


# ---------------------------------------------------------------- 开跑前检查


def refusal(lib):
    assert lib.APP_SysId_Start() == 0
    return texts(lib)


def test_start_needs_auto_throttle(lib):
    prepare_xy(lib, target_n=0)
    assert refusal(lib) == ["ERR sysid xy needs auto throttle (SYSID THROTTLE target_n>0)\r\n"]


@pytest.mark.parametrize("inject,amp,accepted", [
    ("tilt", 0.10, True), ("tilt", 0.101, False),
    ("vel", 0.3, True), ("vel", 0.301, False),
    ("pos", 0.15, True), ("pos", 0.151, False),
])
def test_start_checks_the_amplitude_per_injection_type(lib, inject, amp, accepted):
    spec = step_spec(amp)
    prepare_xy(lib, inject=inject, spec=spec, win_mm=400)
    assert lib.APP_SysId_Start() == (1 if accepted else 0)
    if accepted:
        teardown_run(lib)
        return
    unit = {"tilt": "amp<=0.10 rad", "vel": "amp<=0.3 m/s", "pos": "amp<=0.15 m"}[inject]
    assert texts(lib) == [f"ERR sysid xy amp over limit: inject={inject} {unit}\r\n"]


def test_position_amplitude_must_fit_inside_the_window(lib):
    """pos 幅值 <= 0.7*win：win=100/140 时上限 70/98 mm，0.10 被拒；win=150 时 105 mm 放行。"""
    for win in (100, 140):
        prepare_xy(lib, inject="pos", spec=step_spec(0.10), win_mm=win)
        assert refusal(lib) == ["ERR sysid xy amp over window: inject=pos amp<=0.7*win\r\n"]
    prepare_xy(lib, inject="pos", spec=step_spec(0.10), win_mm=150)
    assert lib.APP_SysId_Start() == 1
    teardown_run(lib)


@pytest.mark.parametrize("obs,reason", [
    (dict(flow_valid=0), "flow invalid (vel_valid=0 height_valid=1 age_ms=18)"),
    (dict(flow_sample_ms=0), "flow invalid (vel_valid=1 height_valid=1 age_ms=-1)"),
    (dict(height_valid=0), "height invalid (TOF)"),
    (dict(height_sample_ms=0), "height invalid (TOF)"),
])
def test_start_needs_valid_flow_and_range(lib, obs, reason):
    prepare_xy(lib, **obs)
    (line,) = refusal(lib)
    assert re.fullmatch(f"ERR sysid xy {re.escape(reason)}\r\n", line), line


def test_start_refuses_a_stale_flow_sample(lib):
    """样本时刻比当前拍早 201 ms 就不新鲜；200 ms 整还算新鲜（含边界）。"""
    prepare_xy(lib)
    for age, accepted in ((201, False), (200, True)):
        plant = XYPlant(lib)
        for k in range(3):
            obs = xy_obs(2000 + 2 * k, 2_000_000 + 2000 * k, plant, flow_sample_ms=2000 + 2 * k - age)
            lib.APP_SysId_Update(ctypes.byref(obs))
        lib.harness_reset()
        assert lib.APP_SysId_Start() == (1 if accepted else 0)
        if accepted:
            teardown_run(lib)
        else:
            assert texts(lib) == [
                "ERR sysid xy flow invalid (vel_valid=1 height_valid=1 age_ms=201)\r\n"]
        prepare_xy(lib)


def test_start_refuses_a_target_above_the_airframe_maximum(lib):
    prepare_xy(lib)
    assert lib.APP_SysId_SetThrottle(ctypes.c_float(1000.0), ctypes.c_float(75.0)) == 0   # 命令面本就拒


def test_start_line_is_followed_by_the_provenance_line(lib):
    plant = start_xy(lib, inject="vel", win_mm=200, mass_g=1184, psi=-0.6, spec=step_spec(0.1))
    lines = texts(lib)
    starts = [i for i, line in enumerate(lines) if line.startswith("SYSID start ")]
    assert len(starts) == 1 and " auto=1 " in lines[starts[0]]
    assert " psi_mrad=-600 " in lines[starts[0]]
    run = re.search(r"run=(\d+)", lines[starts[0]]).group(1)
    x0, y0 = round(ORIGIN[0] * 1000), round(ORIGIN[1] * 1000)
    assert lines[starts[0] + 1] == (
        f"SYSID XYSTART run={run} xy_inject=vel xy_win_mm=200 xy_mass_g=1184 xy_psi_mrad=-600 "
        f"xy_target_cn={round(TARGET_N * 100)} xy_x0_mm={x0} xy_y0_mm={y0}\r\n")
    del plant
    teardown_run(lib)


def test_default_mass_in_the_provenance_is_the_airframe_mass(lib):
    start_xy(lib)
    assert f" xy_mass_g={round(AIRFRAME_KG * 1000)} " in next(
        line for line in texts(lib) if line.startswith("SYSID XYSTART "))
    teardown_run(lib)


# ---------------------------------------------------------------- 阶段序列与推力


@pytest.mark.parametrize("inject", ["tilt", "vel", "pos"])
def test_full_run_follows_the_contracted_sequence_at_constant_thrust(lib, tuned, inject):
    schema = schema_of(lib)
    spec = step_spec(0.02 if inject == "tilt" else 0.02, hold_ms=400, ramp_ms=100)
    plant = start_xy(lib, inject=inject, spec=spec, plant=XYPlant(lib, fs=0.0))
    lib.harness_reset()
    record, capture = [], Capture(lib)
    stopped = run_xy(lib, plant, 6000, record=record, capture=capture)
    assert stopped < 6000
    assert lib.APP_SysId_GetState() == STATE_DONE
    assert lib.APP_SysId_GetLastReason() == b"complete"
    assert motor(lib) is None, "结束后必须交还遥控器油门"

    runs = phase_runs(record)
    assert [phase for phase, _count in runs[:-1]] == XY_SEQUENCE
    assert runs[-1][0] == PHASE_IDLE and runs[-1][1] == 1
    ticks = dict(runs[:-1])
    assert ticks[PHASE_RAMP_UP] == RAMP_TICKS
    assert ticks[PHASE_SETTLE] == SETTLE_TICKS
    assert ticks[PHASE_PREROLL] == PREROLL_TICKS
    assert ticks[PHASE_EXCITE] == pytest.approx(600 / 2, abs=1)
    assert ticks[PHASE_RAMP_DOWN] == DOWN_TICKS
    phases = [re.search(r"phase=(\w+)", line).group(1) for line in capture.texts
              if line.startswith("SYSID PHASE ")]
    assert phases == ["ramp_up", "settle", "preroll", "excite", "ramp_down"]
    assert any(line.startswith("SYSID end run=") and "state=done reason=complete " in line
               for line in capture.texts)

    # 推力恒为 target_n（RAMP 段除外）；RAMP_UP 单调上升、RAMP_DOWN 单调下降到怠速。
    thrusts = [(phase, thrust_of(lib, pulse)) for phase, pulse, _s in record if pulse is not None]
    for phase, thrust in thrusts:
        if phase in (PHASE_SETTLE, PHASE_PREROLL, PHASE_EXCITE):
            assert thrust == pytest.approx(TARGET_N, abs=0.05)
    up = [t for p, t in thrusts if p == PHASE_RAMP_UP]
    down = [t for p, t in thrusts if p == PHASE_RAMP_DOWN]
    assert all(b >= a - 1e-6 for a, b in zip(up, up[1:])) and up[-1] > 0.95 * TARGET_N
    assert all(b <= a + 1e-6 for a, b in zip(down, down[1:]))
    assert down[-1] < 0.05 * TARGET_N + thrust_of(lib, IDLE_US)

    rows, flags = capture.samples(schema)
    assert flags & FLAG_XY and not flags & FLAG_ALT
    assert len(rows) == pytest.approx(stopped / 10 + 1, abs=1)          # 50 Hz 网格，全程无断点


# ---------------------------------------------------------------- 方向


def deflection_run(lib, psi, amp=0.03, hold_ms=1000, plant=None):
    """tilt 阶跃、无摩擦：返回 (plant, 记录行, EXCITE 平台段的 (bias roll, pitch, a_u))。"""
    schema = schema_of(lib)
    plant = start_xy(lib, inject="tilt", psi=psi, spec=step_spec(amp, hold_ms=hold_ms, ramp_ms=20),
                     plant=plant or XYPlant(lib, psi, fs=0.0))
    lib.harness_reset()
    capture, mid = Capture(lib), {}

    def after(index):
        if index == EXCITE_START + (hold_ms // 2 + 20) // 2:
            mid["bias"] = tilt_bias(lib)

    run_xy(lib, plant, EXCITE_START + (hold_ms + 200) // 2 + 20, capture=capture, after=after)
    lib.APP_SysId_Stop(b"test")
    for _ in range(64):
        lib.APP_SysId_StreamTick()
    capture.flush()
    rows, _flags = capture.samples(schema)
    teardown_run(lib)
    return plant, rows, mid["bias"]


@pytest.mark.parametrize("psi", [math.pi / 4, -math.pi / 4, 0.0, math.pi / 2, 2.0])
def test_positive_tilt_accelerates_along_plus_u_for_every_rod_azimuth(lib, psi):
    """u = (−sin ψ, cos ψ)。tilt 注入 θ > 0 → 质点沿 +u 走；光流的世界系位移方向就是 u；
    倾角偏置的横滚/俯仰是生产公式对 F = m·(a·u, g) 的结果，对不上物理模型质点就走反。"""
    plant, rows, (roll, pitch, accel) = deflection_run(lib, psi)
    ux, uy = u_of(psi)
    assert plant.s > 0.05, plant.s
    theta = 0.03
    assert accel == pytest.approx(G * math.tan(theta), rel=1e-3)
    exp_pitch = math.atan2(accel * ux, G)
    exp_roll = -math.atan2(accel * uy * math.cos(exp_pitch), G)
    assert (pitch, roll) == pytest.approx((exp_pitch, exp_roll), abs=2e-4)
    # 记录：位置 height 沿 +u 为正，height_raw（垂直于 u、沿杆轴）不动，az = g·tanθ，
    # angle_sp = 偏置在杆轴 n 上的分量 = roll·cosψ + pitch·sinψ（绕 n 的正角使推力偏向 −u，故为负）。
    plateau = [r for r in rows if r["az"] > 0.9 * accel]
    assert plateau
    n = (math.cos(psi), math.sin(psi))
    assert all(r["angle_sp"] == pytest.approx(exp_roll * n[0] + exp_pitch * n[1], abs=3e-4)
               for r in plateau[3:-3])
    assert all(r["angle_sp"] < 0 for r in plateau[3:-3])
    assert rows[-1]["height"] == pytest.approx(plant.meas_s, abs=0.03) and rows[-1]["height"] > 0.05
    assert max(abs(r["height_raw"]) for r in rows) < 1e-4


def test_direction_flips_with_the_sign_of_the_tilt_injection_shape(lib):
    """DOUBLET 先正后负：质点先沿 +u 走、再被拉回（位置最大值出现在负半周之前）。"""
    psi = math.pi / 4
    spec = dict(CLOSED_DOUBLET, amplitude_rad_s=0.03, duration_ms=1200, hold_ms=600, repeat=1)
    plant = start_xy(lib, inject="tilt", psi=psi, spec=spec, plant=XYPlant(lib, psi))
    accel = []
    run_xy(lib, plant, EXCITE_START + 650, after=lambda i: accel.append(tilt_bias(lib)[2]))
    accel = accel[EXCITE_START:]
    peak = G * math.tan(0.03)
    first_pos = next(i for i, a in enumerate(accel) if a > 0.9 * peak)
    first_neg = next(i for i, a in enumerate(accel) if a < -0.9 * peak)
    assert accel[first_pos + 10] == pytest.approx(peak, rel=1e-3)           # 正半周：+u
    assert accel[first_neg + 10] == pytest.approx(-peak, rel=1e-3)          # 负半周：−u
    assert first_pos < first_neg
    assert plant.s > 0.02
    teardown_run(lib)


# ---------------------------------------------------------------- tilt 开环


def test_tilt_injection_is_open_loop_and_does_not_run_the_production_position_loop(lib, coax_params):
    """tilt 只给倾角 θ：改位置/速度环增益、把槽卡死（位置误差一直不动）输出都不变。"""
    outs = []
    for pos_kp, vel_kp in ((0.375, 0.8), (5.0, 9.0)):
        assert coax_params("coax.pos_x_kp", pos_kp) == 1
        assert coax_params("coax.vel_x_kp", vel_kp) == 1
        plant = start_xy(lib, inject="tilt", spec=step_spec(0.02, hold_ms=400, ramp_ms=20),
                         plant=XYPlant(lib, stuck=True))
        record = []

        def after(index):
            record.append(tilt_bias(lib))

        run_xy(lib, plant, EXCITE_START + 150, after=after)
        outs.append(record)
        teardown_run(lib)
    assert outs[0] == outs[1]
    # 卡死的质点位置误差恒为 0，若跑了位置环，vel/pos 里 SETTLE 段也是 0——这里看 EXCITE 段有输出。
    assert max(abs(a) for _r, _p, a in outs[0][EXCITE_START:]) == pytest.approx(
        G * math.tan(0.02), rel=1e-3)
    # SETTLE / PREROLL 段 θ = 0（偏置为 0）。
    assert all(a == 0.0 and r == 0.0 and p == 0.0
               for r, p, a in outs[0][RAMP_TICKS:EXCITE_START])


def test_tilt_records_no_position_reference(lib):
    schema = schema_of(lib)
    plant = start_xy(lib, inject="tilt", spec=step_spec(0.02, hold_ms=400, ramp_ms=20),
                     plant=XYPlant(lib, stuck=True))
    lib.harness_reset()
    capture = Capture(lib)
    run_xy(lib, plant, EXCITE_START + 150, capture=capture)
    lib.APP_SysId_Stop(b"test")
    for _ in range(64):
        lib.APP_SysId_StreamTick()
    capture.flush()
    rows, _flags = capture.samples(schema)
    teardown_run(lib)
    assert all(r["height_sp"] == 0.0 and r["vz_sp"] == 0.0 for r in rows)
    assert max(r["az"] for r in rows) == pytest.approx(G * math.tan(0.02), rel=2e-3)


def test_tilt_moves_only_above_the_friction_threshold(lib):
    """带库仑摩擦的槽：g·tanθ ≤ 静摩擦门槛 fs 时纹丝不动，超过之后才滑；动起来后按动摩擦减速度加速。"""
    fs, fk = 0.10, 0.06                                       # m/s²：门槛角 ≈ atan(fs/g) = 0.58°
    moved = {}
    for amp in (0.006, 0.009, 0.014, 0.03):
        plant = start_xy(lib, inject="tilt", spec=step_spec(amp, hold_ms=1500, ramp_ms=100),
                         plant=XYPlant(lib, fs=fs, fk=fk))
        run_xy(lib, plant, EXCITE_START + 600)
        moved[amp] = plant.s
        teardown_run(lib)
    assert G * math.tan(0.009) < fs < G * math.tan(0.014)
    assert moved[0.006] == 0.0 and moved[0.009] == 0.0
    # 动起来后按 (a − fk) 加速：位移之比 ≈ (g·tan0.03 − fk)/(g·tan0.014 − fk) ≈ 3。
    assert moved[0.014] > 0.02 and moved[0.03] > 2.5 * moved[0.014]


# ---------------------------------------------------------------- vel / pos：生产 x 通道


def stuck_run(lib, inject, amp, ticks_after_excite=60, **kwargs):
    """槽卡死（测得位置/速度恒为 0）：环的输出只由参考决定，逐拍读 (a_u, 记录参考)。"""
    plant = start_xy(lib, inject=inject, spec=step_spec(amp, hold_ms=4000, ramp_ms=20),
                     plant=XYPlant(lib, stuck=True), **kwargs)
    out = []
    run_xy(lib, plant, EXCITE_START + ticks_after_excite,
           after=lambda i: out.append(tilt_bias(lib)[2]))
    teardown_run(lib)
    return out[EXCITE_START:]


def test_closed_loop_uses_the_production_x_channel_gains(lib, coax_params):
    """pos 阶跃 A、速度为 0、无积分：a_u = vel_x_kp·pos_x_kp·A（先 P 再 P）。换增益输出跟着变。"""
    assert coax_params("coax.vel_x_ki", 0.0) == 1
    assert coax_params("coax.vel_x_kd", 0.0) == 1
    assert coax_params("coax.accel_xy_max_m_s2", 10.0) == 1
    assert coax_params("coax.pos_xy_vel_max_m_s", 1.0) == 1
    amp = 0.05
    for pos_kp, vel_kp in ((0.375, 0.8), (0.75, 0.8), (0.375, 1.6), (1.0, 2.0)):
        assert coax_params("coax.pos_x_kp", pos_kp) == 1
        assert coax_params("coax.vel_x_kp", vel_kp) == 1
        a = stuck_run(lib, "pos", amp, win_mm=400)
        assert max(a) == pytest.approx(vel_kp * pos_kp * amp, rel=0.02), (pos_kp, vel_kp)


def test_the_velocity_integrator_and_its_limit_are_the_production_ones(lib, coax_params):
    assert coax_params("coax.pos_x_kp", 0.375) == 1
    assert coax_params("coax.vel_x_kp", 0.8) == 1
    assert coax_params("coax.accel_xy_max_m_s2", 10.0) == 1
    assert coax_params("coax.vel_x_kd", 0.0) == 1
    assert coax_params("coax.vel_x_i_limit_m_s2", 1.5) == 1
    assert coax_params("coax.vel_x_ki", 0.0) == 1
    flat = stuck_run(lib, "pos", 0.1, ticks_after_excite=1500, win_mm=400)
    assert flat[-1] == pytest.approx(flat[60], rel=0.02)          # ki = 0：不积
    assert coax_params("coax.vel_x_ki", 2.0) == 1
    grow = stuck_run(lib, "pos", 0.1, ticks_after_excite=1500, win_mm=400)
    assert grow[-1] > grow[60] + 0.1                              # 有积分：持续误差把输出往上推
    assert coax_params("coax.vel_x_i_limit_m_s2", 0.01) == 1
    limited = stuck_run(lib, "pos", 0.1, ticks_after_excite=1500, win_mm=400)
    assert limited[-1] < grow[-1] - 0.1                           # 积分限幅生效
    assert limited[-1] == pytest.approx(0.8 * 0.375 * 0.1 + 0.01, abs=0.002)   # P 项 + 被限幅的 I 项


def test_the_acceleration_limit_is_the_production_xy_limit(lib, coax_params):
    assert coax_params("coax.vel_x_ki", 0.0) == 1
    assert coax_params("coax.pos_x_kp", 2.0) == 1
    assert coax_params("coax.vel_x_kp", 5.0) == 1
    assert coax_params("coax.pos_xy_vel_max_m_s", 5.0) == 1
    assert coax_params("coax.accel_xy_max_m_s2", 0.3) == 1
    a = stuck_run(lib, "pos", 0.1, win_mm=400)
    assert max(a) == pytest.approx(0.3, rel=0.01)


def test_velocity_injection_is_a_feedforward_and_position_injection_is_not(lib, coax_params):
    schema = schema_of(lib)
    assert coax_params("coax.vel_x_ki", 0.0) == 1
    assert coax_params("coax.pos_x_kp", 0.5) == 1
    seen = {}
    for inject in ("vel", "pos"):
        plant = start_xy(lib, inject=inject, spec=step_spec(0.1, hold_ms=400, ramp_ms=20),
                         plant=XYPlant(lib, stuck=True), win_mm=400)
        lib.harness_reset()
        capture = Capture(lib)
        run_xy(lib, plant, EXCITE_START + 100, capture=capture)
        lib.APP_SysId_Stop(b"test")
        for _ in range(64):
            lib.APP_SysId_StreamTick()
        capture.flush()
        seen[inject], _flags = capture.samples(schema)
        teardown_run(lib)
    vel, pos = seen["vel"], seen["pos"]
    # vel：vz_sp 含前馈 v_inj = 0.1·shape（≈ 0.1），p_sp = ∫v_inj dt 缓慢增长；
    assert max(r["vz_sp"] for r in vel) == pytest.approx(0.1 + 0.5 * max(r["height_sp"] for r in vel),
                                                        abs=0.01)
    assert max(r["vz_sp"] for r in vel) > 0.09
    assert 0.0 < max(r["height_sp"] for r in vel) < 0.02
    # pos：p_sp 直接是 0.1·shape，速度参考只有位置 P（无前馈）：vz_sp = pos_x_kp·(p_sp − p)。
    assert max(r["height_sp"] for r in pos) == pytest.approx(0.1, abs=1e-3)
    assert all(r["vz_sp"] == pytest.approx(0.5 * r["height_sp"], abs=0.006) for r in pos)


def test_loop_periods_are_the_production_ones(lib):
    header = (ROOT / "App/Src/app_sysid_xy.c").read_text(encoding="utf-8")
    assert "APP_CONTROL_SCHED_POSITION_PERIOD_US" in header
    assert "APP_CONTROL_SCHED_VELOCITY_PERIOD_US" in header
    assert "DRV_POSITION_CONTROL_PositionStep" in header and "DRV_POSITION_CONTROL_VelocityStep" in header
    code = re.sub(r"/\*.*?\*/", "", header, flags=re.S)
    assert "position_kp" not in code and "kp *" not in code, "本模块不另写 PI"


# ---------------------------------------------------------------- 水平槽仿真（带摩擦、延迟、噪声）


def closed_loop_trace(lib, inject, spec, plant, win_mm=150, extra=1200):
    plant = start_xy(lib, inject=inject, spec=spec, plant=plant, win_mm=win_mm)
    track = []
    run_xy(lib, plant, EXCITE_START + spec["duration_ms"] // 2 + extra,
           after=lambda i: track.append((plant.s, plant.v)))
    reason = lib.APP_SysId_GetLastReason()
    teardown_run(lib)
    return [s for s, _v in track[EXCITE_START:]], [v for _s, v in track[EXCITE_START:]], reason


def sign_changes(values, dead=0.004):
    signs = [1 if v > dead else -1 for v in values if abs(v) > dead]
    return sum(1 for a, b in zip(signs, signs[1:]) if a != b)


def test_position_step_reaches_the_target_without_oscillation(lib, tuned):
    """1 维水平槽：库仑摩擦 + 40 ms 光流延迟 + 噪声。pos 阶跃 5 cm 保持 6 s：进 ±5 mm 带、超调 < 15 %、
    速度不来回反号（不振荡）。"""
    amp = 0.05
    plant = XYPlant(lib, fs=0.04, fk=0.03, delay_ticks=20, pos_sigma=0.0005, vel_sigma=0.004)
    spec = step_spec(amp, hold_ms=6000, ramp_ms=100)
    pos, vel, _reason = closed_loop_trace(lib, "pos", spec, plant, extra=800)
    hold = pos[: (100 + 6000) // 2]
    plateau = hold[-500:]                                   # 平台最后 1 s
    assert max(abs(p - amp) for p in plateau) < 0.005, (min(plateau), max(plateau))
    assert max(hold) < amp * 1.15
    assert sign_changes(vel[: (100 + 6000) // 2], dead=0.008) <= 1
    assert plant.s < 0.15


def test_velocity_step_tracks_the_feedforward_reference_without_oscillation(lib, tuned):
    amp = 0.05
    plant = XYPlant(lib, fs=0.04, fk=0.03, delay_ticks=20, pos_sigma=0.0005, vel_sigma=0.004)
    spec = step_spec(amp, hold_ms=1500, ramp_ms=100)
    pos, vel, _reason = closed_loop_trace(lib, "vel", spec, plant, win_mm=400, extra=1500)
    cruise = vel[(100 + 400) // 2:(100 + 1500) // 2]
    assert sum(cruise) / len(cruise) == pytest.approx(amp, abs=0.012)
    assert max(vel) < amp * 1.4
    assert sign_changes(vel[: (200 + 1500) // 2], dead=0.008) <= 1


# ---------------------------------------------------------------- 安全：软停与当拍交还


def soft_stop_check(lib, plant, mutate, reason, *, hold_ticks=EXCITE_START + 120):
    record, bias = [], []

    def after(index):
        bias.append(tilt_bias(lib))

    stopped = run_xy(lib, plant, hold_ticks + 1500, mutate=mutate, record=record, after=after)
    assert lib.APP_SysId_GetState() == STATE_ABORTED
    assert lib.APP_SysId_GetLastReason() == reason
    assert motor(lib) is None
    entry = first_tick_in(record, PHASE_RAMP_DOWN, after=hold_ticks - 20)
    assert stopped >= entry + DOWN_TICKS - 5, "慢慢降完才报，不是当拍交还油门"
    assert stopped <= entry + DOWN_TICKS + 5
    assert max_tick_drop(lib, record, entry - 1) < 0.03      # 推力一拍掉不多（1 s 线性 ≈ 0.014 N/拍）
    assert thrust_of(lib, record[entry][1]) == pytest.approx(TARGET_N, abs=0.1)
    # 倾角偏置清零、姿态回 att0。
    assert all(b == (0.0, 0.0, 0.0) for b in bias[entry + 1: stopped])
    return entry, stopped


@pytest.mark.parametrize("inject", ["tilt", "vel", "pos"])
def test_leaving_the_window_lands_softly(lib, inject):
    plant = start_xy(lib, inject=inject, win_mm=100, spec=step_spec(0.05, hold_ms=3000, ramp_ms=20),
                     plant=XYPlant(lib, stuck=True))
    limit = EXCITE_START + 60

    def mutate(index, obs):
        if index >= limit:
            obs.flow_pos_m = (ctypes.c_float * 2)(
                ORIGIN[0] + 0.1001 * plant.ux, ORIGIN[1] + 0.1001 * plant.uy)

    soft_stop_check(lib, plant, mutate, b"xy_window", hold_ticks=limit)


def test_the_window_edge_itself_does_not_trip_and_direction_does_not_matter(lib):
    plant = start_xy(lib, inject="tilt", win_mm=100, plant=XYPlant(lib, stuck=True))

    def mutate(index, obs):
        if index >= RAMP_TICKS + 100:
            obs.flow_pos_m = (ctypes.c_float * 2)(ORIGIN[0] - 0.0999 * plant.ux,
                                                  ORIGIN[1] - 0.0999 * plant.uy)

    run_xy(lib, plant, RAMP_TICKS + 600, mutate=mutate)
    assert lib.APP_SysId_IsRunning() == 1
    plant.stuck = True

    def beyond(index, obs):
        obs.flow_pos_m = (ctypes.c_float * 2)(ORIGIN[0] - 0.1005 * plant.ux,
                                              ORIGIN[1] - 0.1005 * plant.uy)

    run_xy(lib, plant, 5, mutate=beyond, t0_ms=1_000 + (RAMP_TICKS + 600) * 2)
    assert lib.APP_SysId_GetPhase() == PHASE_RAMP_DOWN
    teardown_run(lib)


def test_a_small_perpendicular_excursion_is_recorded_and_does_not_trip_the_window(lib):
    schema = schema_of(lib)
    plant = start_xy(lib, inject="tilt", win_mm=100, plant=XYPlant(lib, stuck=True))
    plant.perp = 0.06                                         # 沿杆轴方向偏了 6 cm（< win）：不受控
    capture = Capture(lib)
    run_xy(lib, plant, RAMP_TICKS + 300, capture=capture)
    lib.APP_SysId_Stop(b"test")
    for _ in range(64):
        lib.APP_SysId_StreamTick()
    capture.flush()
    rows, _flags = capture.samples(schema)
    assert lib.APP_SysId_GetLastReason() == b"test"
    assert all(r["height_raw"] == pytest.approx(0.06, abs=2e-3) for r in rows)
    assert all(abs(r["height"]) < 1e-3 for r in rows)
    teardown_run(lib)


@pytest.mark.parametrize("kind", ["invalid", "stale", "no_token"])
def test_flow_loss_lands_softly(lib, kind):
    plant = start_xy(lib, inject="pos", spec=step_spec(0.05, hold_ms=3000, ramp_ms=20),
                     plant=XYPlant(lib, stuck=True), win_mm=200)
    limit = EXCITE_START + 60
    frozen = {}

    def mutate(index, obs):
        if index < limit:
            return
        if kind == "invalid":
            obs.flow_valid = 0
        elif kind == "no_token":
            obs.flow_sample_ms = 0
        else:
            frozen.setdefault("token", obs.flow_sample_ms)
            obs.flow_sample_ms = frozen["token"]              # 光流不再出新样本

    entry, _stopped = None, None
    record = []
    stopped = run_xy(lib, plant, limit + 1500, mutate=mutate, record=record)
    assert lib.APP_SysId_GetLastReason() == b"xy_flow_invalid"
    entry = first_tick_in(record, PHASE_RAMP_DOWN, after=limit - 5)
    delay = entry - limit
    # 缺口容忍 200 ms（100 拍）后软停；样本冻结先要 200 ms 才算不新鲜，再容忍 200 ms。
    assert (98 <= delay <= 106) if kind != "stale" else (196 <= delay <= 208), delay
    assert stopped >= entry + DOWN_TICKS - 5 and motor(lib) is None
    assert max_tick_drop(lib, record, entry - 1) < 0.03


def test_a_flow_gap_shorter_than_200_ms_does_not_stop_the_run(lib):
    plant = start_xy(lib, inject="tilt", plant=XYPlant(lib, stuck=True))
    frozen = {}

    def mutate(index, obs):
        if RAMP_TICKS + 50 <= index < RAMP_TICKS + 50 + 90:      # 冻结 180 ms
            frozen.setdefault("token", obs.flow_sample_ms)
            obs.flow_sample_ms = frozen["token"]

    run_xy(lib, plant, RAMP_TICKS + 400, mutate=mutate)
    assert lib.APP_SysId_IsRunning() == 1
    teardown_run(lib)


def test_a_module_abort_before_the_thrust_is_up_hands_back_at_once(lib):
    """推力 < 0.5·m_run·g（RAMP_UP 早段）任何中止都当拍交还，不软停。"""
    plant = start_xy(lib, inject="tilt", plant=XYPlant(lib, stuck=True))

    def mutate(index, obs):
        if index >= 100:
            obs.flow_valid = 0

    stopped = run_xy(lib, plant, 400, mutate=mutate)
    assert 198 <= stopped <= 206          # 200 ms 缺口容忍之后才中止
    assert lib.APP_SysId_GetLastReason() == b"xy_flow_invalid"
    assert motor(lib) is None
    teardown_run(lib)


@pytest.mark.parametrize("field,value,reason", [
    ("rc_link_ok", 0, b"rc_lost"),
    ("rc_armed", 0, b"rc_disarm"),
    ("rc_throttle_low", 0, b"rc_throttle_override"),
    ("imu_valid", 0, b"imu_stale"),
    ("thrust_valid", 0, b"thrust_stale"),
])
def test_rc_and_imu_faults_hand_back_the_throttle_on_the_same_tick(lib, field, value, reason):
    plant = start_xy(lib, inject="tilt", plant=XYPlant(lib, stuck=True))
    limit = EXCITE_START + 30

    def mutate(index, obs):
        if index >= limit:
            setattr(obs, field, value)

    stopped = run_xy(lib, plant, limit + 400, mutate=mutate)
    assert stopped == limit
    assert lib.APP_SysId_GetState() == STATE_ABORTED
    assert lib.APP_SysId_GetLastReason() == reason
    assert motor(lib) is None
    teardown_run(lib)


def test_rc_faults_still_win_over_a_soft_stop_in_progress(lib):
    plant = start_xy(lib, inject="tilt", plant=XYPlant(lib, stuck=True))
    limit = EXCITE_START + 30

    def mutate(index, obs):
        if index >= limit:
            obs.flow_valid = 0
        if index >= limit + 100:
            obs.rc_link_ok = 0

    stopped = run_xy(lib, plant, limit + 800, mutate=mutate)
    assert stopped == limit + 100
    assert lib.APP_SysId_GetLastReason() == b"rc_lost"
    teardown_run(lib)


def test_stop_command_hands_back_at_once_and_records_aborted(lib):
    plant = start_xy(lib, inject="tilt", plant=XYPlant(lib, stuck=True))
    run_xy(lib, plant, EXCITE_START + 30)
    lib.APP_SysId_Stop(b"command")
    assert lib.APP_SysId_GetState() == STATE_ABORTED
    assert motor(lib) is None
    teardown_run(lib)


def test_stop_in_ramp_down_is_aborted_not_complete(lib):
    plant = start_xy(lib, inject="tilt", spec=step_spec(0.02, hold_ms=200, ramp_ms=20),
                     plant=XYPlant(lib, stuck=True))
    record = []
    run_xy(lib, plant, EXCITE_START + 300, record=record)
    assert lib.APP_SysId_GetPhase() == PHASE_RAMP_DOWN
    lib.APP_SysId_Stop(b"command")
    assert lib.APP_SysId_GetState() == STATE_ABORTED
    teardown_run(lib)


def test_rod_residual_over_the_limit_lands_softly_like_alt(lib):
    plant = start_xy(lib, inject="tilt", spec=step_spec(0.02, hold_ms=3000, ramp_ms=20),
                     plant=XYPlant(lib, stuck=True))
    limit = EXCITE_START + 40

    def mutate(index, obs):
        if index >= limit:
            obs.gyro_rad_s = (ctypes.c_float * 3)(0.0, 0.0, 2.0)      # 与杆轴垂直的转动，> 1.5 rad/s

    soft_stop_check(lib, plant, mutate, b"axis_residual", hold_ticks=limit)


def test_rod_snaps_below_1_5_rad_s_are_tolerated(lib):
    plant = start_xy(lib, inject="tilt", plant=XYPlant(lib, stuck=True))

    def mutate(index, obs):
        if RAMP_TICKS + 20 <= index < RAMP_TICKS + 60:
            obs.gyro_rad_s = (ctypes.c_float * 3)(0.0, 0.0, 1.2)

    run_xy(lib, plant, RAMP_TICKS + 300, mutate=mutate)
    assert lib.APP_SysId_IsRunning() == 1
    teardown_run(lib)


# ---------------------------------------------------------------- 记录字段与 THR? 行


def test_record_fields_carry_the_xy_meaning(lib):
    """height = p_u（相对起点）、height_raw = 垂直分量、vz = v_u、vbat；start 那一刻的光流位置是原点。"""
    schema = schema_of(lib)
    psi = 0.5
    plant = start_xy(lib, inject="tilt", psi=psi, spec=step_spec(0.02, hold_ms=400, ramp_ms=20),
                     plant=XYPlant(lib, psi, stuck=True), rate_hz=500)
    plant.perp = 0.011
    ux, uy = u_of(psi)
    nx, ny = math.cos(psi), math.sin(psi)
    lib.harness_reset()

    def mutate(index, obs):
        s, p = 0.0234, 0.0117
        obs.flow_pos_m = (ctypes.c_float * 2)(ORIGIN[0] + s * ux + p * nx, ORIGIN[1] + s * uy + p * ny)
        obs.flow_vel_m_s = (ctypes.c_float * 2)(0.045 * ux + 0.02 * nx, 0.045 * uy + 0.02 * ny)
        obs.vbat_v = 12.3

    capture = Capture(lib)
    run_xy(lib, plant, 30, mutate=mutate)
    lib.APP_SysId_Stop(b"test")
    for _ in range(8):
        lib.APP_SysId_StreamTick()
    capture.flush()
    rows, flags = capture.samples(schema)
    assert flags & FLAG_XY
    row = rows[5]
    assert row["height"] == pytest.approx(0.0234, abs=1e-4)
    assert row["height_raw"] == pytest.approx(0.0117, abs=1e-4)
    assert row["vz"] == pytest.approx(0.045, abs=1e-3)
    assert row["vbat"] == pytest.approx(12.3, abs=1e-3)
    assert row["height_sp"] == 0.0 and row["vz_sp"] == 0.0 and row["az"] == 0.0
    teardown_run(lib)


def test_non_xy_runs_record_zero_xy_fields(lib):
    schema = schema_of(lib)
    reset(lib, rate_hz=500)
    assert lib.APP_SysId_SetMode(0, ctypes.c_float(0.0523598776)) == 1
    assert lib.APP_SysId_Start() == 1
    lib.harness_reset()
    plant = XYPlant(lib)
    for index in range(40):
        lib.APP_SysId_Update(ctypes.byref(
            xy_obs(1000 + index * 2, 1_000_000 + index * CONTROL_DT_US, plant, throttle_us=1600)))
    lib.APP_SysId_StreamTick()
    capture = Capture(lib)
    capture.flush()
    rows, flags = capture.samples(schema)
    teardown_run(lib)
    assert rows and not flags & FLAG_XY
    assert all(row[name] == 0.0 for row in rows for name in XY_FIELDS)


def test_thr_line_reports_live_xy_position_velocity_and_ok(lib):
    psi = PSI
    plant = prepare_xy(lib, psi=psi)
    ux, uy = u_of(psi)
    assert lib.APP_SysId_Start() == 1
    lib.harness_reset()

    def mutate(index, obs):
        s = 0.020 if index >= 10 else 0.0
        obs.flow_pos_m = (ctypes.c_float * 2)(ORIGIN[0] + s * ux, ORIGIN[1] + s * uy)
        obs.flow_vel_m_s = (ctypes.c_float * 2)(0.05 * ux, 0.05 * uy)

    run_xy(lib, plant, 30, mutate=mutate)
    lib.harness_reset()
    lib.APP_SysId_ReportThrottle()
    values = dict(item.split("=") for item in texts(lib)[0].split()[2:])
    assert (int(values["xy_pos_mm"]), int(values["xy_vel_mms"]), int(values["xy_ok"])) == (20, 50, 1)
    teardown_run(lib)
    # 起点跨轮保留：不在跑也报，相对最近一次记下的起点；光流失效则 xy_ok = 0。
    lib.APP_SysId_Update(ctypes.byref(xy_obs(9000, 9_000_000, plant, flow_valid=0)))
    lib.harness_reset()
    lib.APP_SysId_ReportThrottle()
    assert " xy_ok=0 xy_yaw_mrad=" in texts(lib)[0]


def test_the_new_text_lines_fit_the_text_buffer_at_their_widest():
    """XYSTART 另起一行（start 行最宽已约 246 字符）；回显；THR 行尾追加了三项。"""
    budget = int(re.search(r"#define\s+APP_UART_TX_TEXT_SIZE\s+(\d+)",
                           (ROOT / "App/Inc/app_messages.h").read_text(encoding="utf-8")).group(1)) - 1
    xy_source = (ROOT / "App/Src/app_sysid_xy.c").read_text(encoding="utf-8")
    xystart = _worst_line(xy_source, '"SYSID XYSTART run=', dict(
        run=65535, xy_inject="tilt", xy_win_mm=400, xy_mass_g=3000, xy_psi_mrad=-2147483648,
        xy_target_cn=1000000, xy_x0_mm=-2147483648, xy_y0_mm=-2147483648))
    echo = _worst_line(xy_source, '"SYSID XY inject=', dict(
        inject="tilt", win_mm=400, mass_g=3000, control="closed_loop"))
    reject = _worst_line((ROOT / "App/Src/app_cmd_sysid.c").read_text(encoding="utf-8"),
                         '"SYSID XY event=rejected', dict(reason="usage"))
    sysid_source = (ROOT / "App/Src/app_sysid.c").read_text(encoding="utf-8")
    thr = _worst_line(sysid_source, '"SYSID THR auto=', {
        "auto": 1, "target_cn": 1_000_000, "max_pct_x10": 950, "phase": "ramp_down",
        "pulse_us": 65535, "armed": 1, "thr_low": 1, "capped": 1,
        "alt_h_mm": -2147483648, "alt_h_ok": 1, "alt_sp_mm": -2147483648,
        "xy_pos_mm": -2147483648, "xy_vel_mms": -2147483648, "xy_ok": 1,
        "xy_yaw_mrad": -2147483648, "vbat_mv": 65535})
    refusal = "ERR sysid xy amp over limit: inject=tilt amp<=0.10 rad\r\n"
    for line in (xystart, echo, reject, thr, refusal):
        assert len(line) <= budget, (len(line), line)
    assert thr.endswith(" xy_pos_mm=-2147483648 xy_vel_mms=-2147483648 xy_ok=1"
                        " xy_yaw_mrad=-2147483648 vbat_mv=65535\r\n")


# ---------------------------------------------------------------- 观测填写（只读）


def test_the_stabilizer_fills_the_xy_observation_read_only():
    source = (ROOT / "App/Src/app_stabilizer.c").read_text(encoding="utf-8")
    block = source[source.index("SVC_FLOW_NAV_State sysid_nav;"):
                   source.index("APP_SysId_Update(&sysid_obs);")]
    for field in ("SVC_FlowNav_GetVelocity(&sysid_flow_vel_m_s[0], &sysid_flow_vel_m_s[1]);",
                  "SVC_FlowNav_GetPosition(&sysid_flow_pos_m[0], &sysid_flow_pos_m[1]);",
                  ".flow_valid = ((sysid_nav.velocity_valid != 0U) && (sysid_height_ok != 0U))",
                  ".flow_sample_ms = sysid_nav.velocity_sample_ms,",
                  ".flow_vel_m_s = {", ".flow_pos_m = {"):
        assert field in block, field
    assert not re.search(r"ctx->\w+(\[[^\]]*\])?\s*=[^=]", block)
    assert "SVC_FlowNav_Reset" not in block


# ---------------------------------------------------------------- TiltFromForce：生产逐位不变

LEGACY_TILT_C = r"""
#include <math.h>
/* 抽出公共函数之前 drv_coax_ctrl.c 里的两行公式，原样搬来（git HEAD 版本）。 */
void legacy_tilt(const float *f, float *roll, float *pitch)
{
    float target_pitch_rad;
    float target_roll_rad;
    target_pitch_rad = atan2f(f[0], f[2]);
    target_roll_rad = -atan2f(f[1] * cosf(target_pitch_rad), f[2]);
    *roll = target_roll_rad;
    *pitch = target_pitch_rad;
}
"""


def test_tilt_from_force_is_bit_identical_to_the_legacy_production_formula(lib, tmp_path):
    compiler = shutil.which("gcc") or shutil.which("clang")
    if compiler is None:
        pytest.skip("host gcc/clang required")
    source = tmp_path / "legacy.c"
    source.write_text(LEGACY_TILT_C, encoding="utf-8")
    out = tmp_path / ("legacy.dll" if sys.platform.startswith("win") else "legacy.so")
    result = subprocess.run([compiler, "-shared", "-fPIC", "-std=c11", "-O1", str(source),
                             "-lm", "-o", str(out)], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    legacy = ctypes.CDLL(str(out))
    f3 = ctypes.POINTER(ctypes.c_float)
    legacy.legacy_tilt.argtypes = [f3, f3, f3]

    rng = random.Random(20260930)
    cases = [(0.0, 0.0, 9.81), (1.0, 0.0, 9.81), (0.0, -1.0, 9.81), (3.0, 4.0, 7.4),
             (-2.5, 6.0, 0.05), (1e-3, -1e-3, 1e-3), (100.0, -100.0, 1.0), (0.0, 0.0, 1e-9)]
    cases += [(rng.uniform(-8, 8), rng.uniform(-8, 8), rng.uniform(0.01, 30)) for _ in range(5000)]
    for fx, fy, fz in cases:
        force = (ctypes.c_float * 3)(fx, fy, fz)
        new_r, new_p, old_r, old_p = (ctypes.c_float() for _ in range(4))
        lib.DRV_COAX_CTRL_TiltFromForce(force, ctypes.byref(new_r), ctypes.byref(new_p))
        legacy.legacy_tilt(force, ctypes.byref(old_r), ctypes.byref(old_p))
        assert struct.pack("<ff", new_r.value, new_p.value) == \
            struct.pack("<ff", old_r.value, old_p.value), (fx, fy, fz)
    # 输出指针允许为空（只要其一）。
    force = (ctypes.c_float * 3)(1.0, 2.0, 9.81)
    only = ctypes.c_float()
    lib.DRV_COAX_CTRL_TiltFromForce(force, ctypes.byref(only), None)


def test_the_production_path_calls_the_shared_function_and_keeps_no_copy_of_the_formula():
    source = (ROOT / "Driver/Src/drv_coax_ctrl.c").read_text(encoding="utf-8")
    body = re.sub(r"/\*.*?\*/", "", source, flags=re.S)
    assert body.count("DRV_COAX_CTRL_TiltFromForce(") == 2                 # 定义 + 生产路径一处调用
    assert "DRV_COAX_CTRL_TiltFromForce(solution->desired_force_local_n," in body
    assert body.count("atan2f(force_n[0], force_n[2])") == 1
    assert "atan2f(solution->desired_force_local_n" not in body
    xy = re.sub(r"/\*.*?\*/", "", (ROOT / "App/Src/app_sysid_xy.c").read_text(encoding="utf-8"),
                flags=re.S)
    assert "DRV_COAX_CTRL_TiltFromForce(force_n" in xy and "atan2f" not in xy


def test_xy_module_state_lives_in_axi_sram_not_dtcm():
    source = (ROOT / "App/Src/app_sysid_xy.c").read_text(encoding="utf-8")
    assert '__attribute__((section(".ram_d1_noinit"), aligned(32)))\n#endif\nstatic SysIdXyContext xy;' in source


# ---------------------------------------------------------------- 实物台架加固（R-XYID-1）


class DriftPlant(XYPlant):
    """机体沿真实槽方向以恒速漂移（start_tick 拍之后），软停开始后以 coast_decel 减速滑停。
    只用来做"固件的 psi 与真实槽对不上"——固件倾角推不动它，位移全是物理上的滑动。"""

    def __init__(self, lib, v_drift, start_tick, coast_decel=0.2, **kwargs):
        super().__init__(lib, **kwargs)
        self.v_drift, self.start_tick, self.coast = v_drift, start_tick, coast_decel
        self.ticks = 0
        self.max_abs_s = 0.0

    def advance(self, pulse, dt=DT):
        self.ticks += 1
        braking = lib_phase(self.lib) == PHASE_RAMP_DOWN or pulse is None
        if braking:
            self.v = math.copysign(max(abs(self.v) - self.coast * dt, 0.0), self.v)
        elif self.ticks >= self.start_tick:
            self.v = self.v_drift
        self.s += self.v * dt
        self.max_speed = max(self.max_speed, abs(self.v))
        self.max_abs_s = max(self.max_abs_s, abs(self.s))
        self.hist = self.hist[1:] + [(self.s, self.v)]


def lib_phase(lib):
    return lib.APP_SysId_GetPhase()


def test_a_wrong_rod_azimuth_by_90_degrees_still_trips_the_window_by_displacement_modulus(lib):
    """机体沿真实槽滑动，但固件的 psi 错了 90 度：滑动全落在 p_perp 上（旧判据看不见）。
    模长 + 预测停车距离照样越窗软停，滑停后的最大位移不超过窗口加上检测滞后。"""
    win_m, v = 0.10, 0.05
    plant = DriftPlant(lib, v, start_tick=RAMP_TICKS + 100, psi=PSI)
    start_xy(lib, inject="tilt", win_mm=100, psi=PSI + math.pi / 2, plant=plant)
    record = []
    run_xy(lib, plant, 6000, record=record)
    assert lib.APP_SysId_GetState() == STATE_ABORTED
    assert lib.APP_SysId_GetLastReason() == b"xy_window"
    assert any(phase == PHASE_RAMP_DOWN for phase, _p, _s in record)
    assert plant.max_abs_s <= win_m + v * 0.10          # 预测滑行已含在判据里，只留检测滞后
    assert plant.max_abs_s > 0.5 * win_m                # 确实是滑出来的，不是提前误停


def test_a_reversed_flow_direction_is_caught_by_overspeed_or_window_before_the_slot_end(lib, tuned):
    """vel 轮光流方向取反 = 闭环正反馈。超速门/出窗门兜底，最大位移 <= 窗口 + 检测滞后内的滑行。"""
    win_m = 0.30
    plant = XYPlant(lib, fk=0.2)
    start_xy(lib, inject="vel", win_mm=300, spec=step_spec(0.2, hold_ms=6000, ramp_ms=20), plant=plant)

    def mutate(index, obs):
        pos = obs.flow_pos_m
        vel = obs.flow_vel_m_s
        obs.flow_pos_m = (ctypes.c_float * 2)(2 * ORIGIN[0] - pos[0], 2 * ORIGIN[1] - pos[1])
        obs.flow_vel_m_s = (ctypes.c_float * 2)(-vel[0], -vel[1])

    max_s = [0.0]

    def after(index):
        max_s[0] = max(max_s[0], abs(plant.s))

    run_xy(lib, plant, 9000, mutate=mutate, after=after)
    assert lib.APP_SysId_GetState() == STATE_ABORTED
    assert lib.APP_SysId_GetLastReason() in (b"xy_overspeed", b"xy_window")
    # 物理界限：从峰值速度按动摩擦 0.2 m/s² 滑停的距离。正反馈发散时倾角偏置清零要经过
    # 光流延迟（约 50 ms）+ 姿态滞后，所以实际越窗量会比"判据触发点"大一截（实测约 win+0.13 m）。
    assert max_s[0] <= win_m + plant.max_speed ** 2 / (2 * 0.2), (max_s[0], plant.max_speed)
    assert max_s[0] < 1.5 * win_m
    assert plant.max_speed < 0.6


def test_overspeed_soft_stops_with_its_own_reason_and_ramp_down_does_not_judge(lib):
    plant = start_xy(lib, inject="tilt", win_mm=400, plant=XYPlant(lib, stuck=True))
    limit = RAMP_TICKS + 300

    def mutate(index, obs):
        if index >= limit:
            obs.flow_vel_m_s = (ctypes.c_float * 2)(0.41 * plant.ux, 0.41 * plant.uy)

    record = []
    run_xy(lib, plant, limit + 1200, mutate=mutate, record=record)
    assert lib.APP_SysId_GetState() == STATE_ABORTED
    assert lib.APP_SysId_GetLastReason() == b"xy_overspeed"
    entry = first_tick_in(record, PHASE_RAMP_DOWN, after=limit - 5)
    assert 0 <= entry - limit <= 3
    assert max_tick_drop(lib, record, entry - 1) < 0.03


def test_the_speed_below_the_limit_with_room_to_coast_keeps_running(lib):
    plant = start_xy(lib, inject="tilt", win_mm=400, plant=XYPlant(lib, stuck=True))

    def mutate(index, obs):
        obs.flow_vel_m_s = (ctypes.c_float * 2)(0.30 * plant.ux, 0.30 * plant.uy)   # 0.225 m 预测滑行 < 0.4

    run_xy(lib, plant, RAMP_TICKS + 400, mutate=mutate)
    assert lib.APP_SysId_IsRunning() == 1
    teardown_run(lib)


def test_the_predicted_coast_distance_counts_toward_the_window(lib):
    """位移 0.06 m + 速度 0.2 m/s 的预测滑行 0.1 m = 0.16 m > 0.15 m 窗口：提前软停。"""
    plant = start_xy(lib, inject="tilt", win_mm=150, plant=XYPlant(lib, stuck=True))

    def mutate(index, obs):
        if index >= RAMP_TICKS + 100:
            obs.flow_pos_m = (ctypes.c_float * 2)(ORIGIN[0] + 0.06 * plant.ux,
                                                  ORIGIN[1] + 0.06 * plant.uy)
            obs.flow_vel_m_s = (ctypes.c_float * 2)(0.2 * plant.ux, 0.2 * plant.uy)

    run_xy(lib, plant, RAMP_TICKS + 300, mutate=mutate)
    assert lib.APP_SysId_GetLastReason() == b"xy_window" or lib.APP_SysId_GetPhase() == PHASE_RAMP_DOWN
    assert lib.APP_SysId_GetPhase() == PHASE_RAMP_DOWN
    teardown_run(lib)


def yaw_drift_run(lib, rate_dps, ticks, **kwargs):
    plant = start_xy(lib, inject="tilt", plant=XYPlant(lib, stuck=True),
                     spec=step_spec(0.05, hold_ms=25000, ramp_ms=20), **kwargs)
    rate = math.radians(rate_dps)

    def mutate(index, obs):
        obs.gyro_rad_s = (ctypes.c_float * 3)(0.0, 0.0, rate)
        obs.gyro_ctrl_rad_s = (ctypes.c_float * 3)(0.0, 0.0, rate)

    record = []
    thr = {}

    def after(index):
        if index == 2000 and "mid" not in thr:
            lib.harness_reset()
            lib.APP_SysId_ReportThrottle()
            thr["mid"] = texts(lib)[0]

    stopped = run_xy(lib, plant, ticks, mutate=mutate, record=record, after=after)
    return plant, record, stopped, thr


def test_a_one_degree_per_second_yaw_drift_trips_the_yaw_limit_near_15_seconds(lib):
    plant, record, stopped, thr = yaw_drift_run(lib, 1.0, 9500)
    assert lib.APP_SysId_GetState() == STATE_ABORTED
    assert lib.APP_SysId_GetLastReason() == b"xy_yaw_limit"
    entry = first_tick_in(record, PHASE_RAMP_DOWN)
    assert 7400 <= entry <= 7560, entry                  # 15 度 / (1 度/秒) = 15 s = 7500 拍
    assert stopped >= entry + DOWN_TICKS - 5 and motor(lib) is None
    assert max_tick_drop(lib, record, entry - 1) < 0.03
    mid = dict(item.split("=") for item in thr["mid"].split()[2:])
    assert int(mid["xy_yaw_mrad"]) == pytest.approx(math.radians(4.0) * 1000, abs=3)
    lib.harness_reset()
    lib.APP_SysId_ReportThrottle()
    final = int(dict(item.split("=") for item in texts(lib)[0].split()[2:])["xy_yaw_mrad"])
    assert final >= 261                                 # 15 度 = 261.8 mrad，最近一轮的值保留到下次开跑


def test_yaw_is_zeroed_at_each_run_start_and_below_the_limit_does_not_stop(lib):
    yaw_drift_run(lib, 1.0, 3000)                       # 6 s，约 6 度
    assert lib.APP_SysId_IsRunning() == 1
    teardown_run(lib)
    plant = start_xy(lib, inject="tilt", plant=XYPlant(lib, stuck=True))
    run_xy(lib, plant, 10)
    lib.harness_reset()
    lib.APP_SysId_ReportThrottle()
    assert int(dict(item.split("=") for item in texts(lib)[0].split()[2:])["xy_yaw_mrad"]) == 0
    teardown_run(lib)


def test_yaw_is_not_judged_in_ramp_down(lib):
    """软停（别的理由）走回落段时，偏航超限不改写理由。"""
    plant = start_xy(lib, inject="tilt", win_mm=100, plant=XYPlant(lib, stuck=True))
    limit = RAMP_TICKS + 200
    rate = math.radians(40.0)

    def mutate(index, obs):
        if index >= limit:
            obs.flow_pos_m = (ctypes.c_float * 2)(ORIGIN[0] + 0.2 * plant.ux, ORIGIN[1] + 0.2 * plant.uy)
            obs.gyro_rad_s = (ctypes.c_float * 3)(0.0, 0.0, rate)
            obs.gyro_ctrl_rad_s = (ctypes.c_float * 3)(0.0, 0.0, rate)

    run_xy(lib, plant, limit + 1200, mutate=mutate)
    assert lib.APP_SysId_GetLastReason() == b"xy_window"


def flow_gap_run(lib, gap_ticks, kind):
    plant = start_xy(lib, inject="pos", spec=step_spec(0.05, hold_ms=3000, ramp_ms=20),
                     plant=XYPlant(lib, stuck=True), win_mm=200)
    first = EXCITE_START + 100
    frozen = {}
    biases = []

    def mutate(index, obs):
        if first <= index < first + gap_ticks:
            if kind == "invalid":
                obs.flow_valid = 0
            else:
                frozen.setdefault("token", obs.flow_sample_ms)
                obs.flow_sample_ms = frozen["token"]

    def after(index):
        biases.append(tilt_bias(lib))

    record = []
    run_xy(lib, plant, first + gap_ticks + 700, mutate=mutate, record=record, after=after)
    return first, biases, record


def test_a_150_ms_flow_gap_keeps_running_and_holds_the_tilt_bias(lib):
    first, biases, _record = flow_gap_run(lib, 75, "invalid")
    assert lib.APP_SysId_IsRunning() == 1
    held = biases[first - 1: first + 75]
    assert held[0] != (0.0, 0.0, 0.0)
    assert all(b == held[0] for b in held)              # 缺口内环不更新：倾角偏置与加速度保持
    teardown_run(lib)


def test_a_300_ms_flow_gap_soft_stops_with_xy_flow_invalid(lib):
    first, _biases, record = flow_gap_run(lib, 150, "invalid")
    assert lib.APP_SysId_GetState() == STATE_ABORTED
    assert lib.APP_SysId_GetLastReason() == b"xy_flow_invalid"
    entry = first_tick_in(record, PHASE_RAMP_DOWN, after=first - 5)
    assert 98 <= entry - first <= 106


def test_a_stale_flow_sample_counts_as_a_gap_after_its_200_ms_age(lib):
    """样本时间戳不前进：200 ms 年龄之后才算无效，再容忍 200 ms。"""
    first, _biases, record = flow_gap_run(lib, 290, "stale")
    assert lib.APP_SysId_GetLastReason() == b"xy_flow_invalid"
    entry = first_tick_in(record, PHASE_RAMP_DOWN, after=first - 5)
    assert 196 <= entry - first <= 208
    teardown_run(lib)


def test_the_loop_integrator_does_not_accumulate_during_a_flow_gap_and_resumes_cleanly(lib):
    first, biases, _record = flow_gap_run(lib, 60, "invalid")
    after_gap = biases[first + 60: first + 70]
    # 恢复后第一拍不把 120 ms 缺口当 dt 积进去：偏置不出现跳变（相邻拍变化小于 0.01 rad）。
    jumps = [abs(a[0] - b[0]) + abs(a[1] - b[1]) for a, b in zip(after_gap, after_gap[1:])]
    assert max(jumps) < 0.01
    assert lib.APP_SysId_IsRunning() == 1
    teardown_run(lib)


def test_a_flow_sample_one_ms_in_the_future_is_fresh_not_a_huge_age(lib):
    """时间戳比较用有符号差：样本比当前拍晚 1 ms 仍是新鲜。"""
    prepare_xy(lib)
    plant = XYPlant(lib)
    for k in range(3):
        obs = xy_obs(2000 + 2 * k, 2_000_000 + 2000 * k, plant, flow_sample_ms=2000 + 2 * k + 1,
                     height_sample_ms=2000 + 2 * k + 1)
        lib.APP_SysId_Update(ctypes.byref(obs))
    lib.harness_reset()
    assert lib.APP_SysId_Start() == 1, texts(lib)

    def mutate(index, obs):
        obs.flow_sample_ms = obs.now_ms + 1
        obs.height_sample_ms = obs.now_ms + 1

    run_xy(lib, plant, RAMP_TICKS + 100, mutate=mutate)
    assert lib.APP_SysId_IsRunning() == 1
    teardown_run(lib)


def test_the_source_uses_signed_age_and_the_documented_constants():
    source = (ROOT / "App/Src/app_sysid_xy.c").read_text(encoding="utf-8")
    assert "(int32_t)(obs->now_ms - obs->flow_sample_ms)" in source
    assert "(uint32_t)(obs->now_ms - obs->flow_sample_ms)" not in source
    for name, value in (("XY_COAST_DECEL_M_S2", "0.2f"), ("XY_MAX_SPEED_M_S", "0.4f"),
                        ("XY_FLOW_GAP_TOL_MS", "200")):
        assert re.search(rf"#define\s+{name}\s+{re.escape(value)}", source), name
    assert "xy_overspeed" in source and "xy_yaw_limit" in source


def _att0_after_start(lib, psi, **obs):
    plant = start_xy(lib, psi=psi, **obs)
    lib.APP_SysId_Update(ctypes.byref(xy_obs(1_000, 1_000_000, plant, **obs)))
    roll, pitch = ctypes.c_float(), ctypes.c_float()
    lib.APP_SysId_GetBenchAtt0(ctypes.byref(roll), ctypes.byref(pitch))
    return roll.value, pitch.value


def test_the_rod_axis_hold_target_is_gravity_level_not_the_hanging_attitude(lib):
    """2026-09-30 台架：开机零点取在悬挂姿态上，XY 保持悬挂角 = 推力偏 +2.2°，浮起推力下激励前溜 11 cm。
    绕杆轴 n 的保持目标改成重力水平（−开机零点），绕 u（被杆挡住）的分量仍是开跑姿态。"""
    psi = -math.pi / 4
    nx, ny = math.cos(psi), math.sin(psi)
    ux, uy = -ny, nx
    hang = dict(roll_rad=0.03, pitch_rad=0.01)
    level = dict(level_valid=1, level_roll_rad=-0.004, level_pitch_rad=0.006)
    roll, pitch = _att0_after_start(lib, psi, **hang, **level)
    assert roll * nx + pitch * ny == pytest.approx(level["level_roll_rad"] * nx + level["level_pitch_rad"] * ny, abs=1e-6)
    assert roll * ux + pitch * uy == pytest.approx(hang["roll_rad"] * ux + hang["pitch_rad"] * uy, abs=1e-6)
    # 零点没就绪：退回开跑姿态。
    roll, pitch = _att0_after_start(lib, psi, **hang, level_valid=0, level_roll_rad=0.5, level_pitch_rad=0.5)
    assert (roll, pitch) == (pytest.approx(0.03, abs=1e-6), pytest.approx(0.01, abs=1e-6))

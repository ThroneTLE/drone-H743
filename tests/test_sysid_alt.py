"""光杆台架高度辨识（SYSID MODE ALT = 4，R-ALTID-1）—— 在宿主上跑真固件。

App/Src/app_sysid.c + app_sysid_alt.c + app_cmd_sysid.c + 真分配器（推力表、反解器）+
真 drv_position_control（生产高度环），平台边界见 tests/fixtures/sysid/。机体的上下运动
由这里的一维质点给：F = 推力表读回的合推力，压在槽底时不往下掉；阈值实验（break）用带
静/动摩擦的槽里质点（SlotPlant），测距按约 80 Hz 出样本并带噪声（槽底实录 σ 3–8 mm）。

判据按台架上真会出事的地方排：

* **命令面**：`SYSID ALT` 回显一行（含槽底/槽顶与 control）、越界/格式/运行中拒绝（整条不生效）；
* **开跑前**：端点没配/行程不对/不在槽底、break 的搜索上限与速率、测高不够 8 个样本都拒绝；
* **break 序列**：预升 → 慢升找离地 → 制停 → 慢降找滑落 → 滑回槽底 → 降到怠速；PHASE 行的
  推力就是离地/停住/滑落时的指令；测距噪声不能冒充离地；不经过生产高度 PID；
* **vel/pos**：生产高度环 + 生产参数（改参数下一拍生效），前馈/参考注入口径不变；
* **安全门**：测高无效 > 100 ms、平均高度出硬窗（按物理槽底/槽顶）、break 的上行提前制动、
  落地未确认；遥控器那道门照旧、先报、交还油门；ALT 在回落段被停记 aborted；
* **记录**：7 个高度字段在 ALT 下是观测/参考、非 ALT 为 0；**溯源**：SYSID ALTSTART 紧跟 start 行。

生产默认 vel_z_kp = 1、pos_z_kp = 3.8 放在无摩擦质点上阻尼只有约 0.26（实机有风阻和槽的
摩擦），要看"跟得上"的用例临时把 vel_z_kp 调到 4（阻尼约 0.5）——同样是生产代码按参数表跑。
"""
from __future__ import annotations

import ctypes
import math
import random
import re
import sys
from pathlib import Path

import pytest

from test_sysid_runtime_contract import (
    CONTROL_DT_US, PROFILE_DOUBLET, PROFILE_STEP, STATE_ABORTED, STATE_DONE, Excitation,
    build_lib, frames, healthy, reset, strip_c_comments, texts,
)

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from sysid.decode import decode_batch, parse_schema_lines  # noqa: E402

EXTRA_SOURCES = ("App/Src/app_cmd_sysid.c", "App/Src/app_param_trial.c",
                 "tests/fixtures/sysid/cmd_harness.c")

MODE_FF, MODE_ALT = 0, 4
(PHASE_IDLE, PHASE_RAMP_UP, PHASE_SETTLE, PHASE_EXCITE, PHASE_RAMP_DOWN, PHASE_PREROLL,
 PHASE_DONE, PHASE_CLIMB, PHASE_DESCEND) = range(9)
ALT_SEQUENCE = [PHASE_RAMP_UP, PHASE_CLIMB, PHASE_SETTLE, PHASE_PREROLL, PHASE_EXCITE,
                PHASE_DESCEND, PHASE_RAMP_DOWN]
BREAK_SEQUENCE = [PHASE_RAMP_UP, PHASE_CLIMB, PHASE_SETTLE, PHASE_EXCITE, PHASE_DESCEND,
                  PHASE_RAMP_DOWN]
FLAG_ALT, FLAG_THRUST_CAPPED = 0x0100, 0x0200
IDLE_US = 1100
G = 9.81
AIRFRAME_KG = 0.7546            # default_airframe()：75 + 232 + 99 + 348.6 g
WEIGHT_N = AIRFRAME_KG * G
BOTTOM_MM, TOP_MM = 80, 265     # 槽底 TOF 读数 80 mm、行程 185 mm（上限）→ 槽顶 − 40 mm = 225 mm
H0 = BOTTOM_MM / 1000.0
SCENE_LIFT_MM, SCENE_WIN_MM = 60, 150
HOLD = H0 + SCENE_LIFT_MM / 1000.0
UPPER = min(H0 + (SCENE_LIFT_MM + SCENE_WIN_MM) / 1000.0, TOP_MM / 1000.0 - 0.040)
UPPER_BREAK = TOP_MM / 1000.0 - 0.010   # break 没有 lift/win：上沿只离槽顶 10 mm
RAW_OFFSET = 0.003              # 原始测距比滤波高度多 3 mm，区分两个字段
TARGET_N = 8.0                  # vel/pos 只作开关；break 是离地搜索上限（在 [W, W+3 N] 内）
ALT_FIELDS = ("height", "height_raw", "height_sp", "vz", "vz_sp", "az", "vbat")
RAMP_TICKS, CLIMB_TICKS, SETTLE_TICKS, PREROLL_TICKS = 750, 300, 1000, 250
DESCEND_TICKS = round((HOLD - H0 - 0.010) / 0.1 / 0.002)
AMP_TEXT = {"break": "1 N/s", "vel": "0.3 m/s", "pos": "0.15 m"}

CLOSED_DOUBLET = dict(profile=PROFILE_DOUBLET, amplitude_rad_s=0.05, duration_ms=500,
                      hold_ms=250, repeat=1, ramp_ms=20, chirp_f0_hz=0.3, chirp_f1_hz=6.0,
                      prbs_bit_ms=40, prbs_seed=1)
BREAK_SPEC = dict(profile=PROFILE_STEP, amplitude_rad_s=0.5, duration_ms=300, hold_ms=100,
                  repeat=1, ramp_ms=100, chirp_f0_hz=0.3, chirp_f1_hz=6.0, prbs_bit_ms=40,
                  prbs_seed=1)


def step_spec(amp, hold_ms=600, ramp_ms=100):
    return dict(profile=PROFILE_STEP, amplitude_rad_s=amp, duration_ms=2 * ramp_ms + hold_ms,
                hold_ms=hold_ms, repeat=1, ramp_ms=ramp_ms, chirp_f0_hz=0.3, chirp_f1_hz=6.0,
                prbs_bit_ms=40, prbs_seed=1)


def prepare_handle(handle):
    handle.harness_command.argtypes = [ctypes.c_char_p]
    handle.harness_command.restype = ctypes.c_uint8
    handle.APP_SysId_SetMode.argtypes = [ctypes.c_int, ctypes.c_float]
    handle.APP_SysId_SetMode.restype = ctypes.c_uint8
    handle.APP_SysId_GetMode.restype = ctypes.c_int
    handle.DRV_COAX_CTRL_SetParam.argtypes = [ctypes.c_char_p, ctypes.c_float]
    handle.DRV_COAX_CTRL_SetParam.restype = ctypes.c_uint8
    handle.DRV_COAX_CTRL_GetParam.argtypes = [ctypes.c_char_p, ctypes.POINTER(ctypes.c_float)]
    handle.DRV_COAX_CTRL_GetParam.restype = ctypes.c_uint8
    handle.DRV_COAX_CTRL_MotorPulseToTotalThrust.argtypes = [ctypes.c_uint16]
    handle.DRV_COAX_CTRL_MotorPulseToTotalThrust.restype = ctypes.c_float
    handle.DRV_COAX_CTRL_ThrustToMotorPulse.argtypes = [ctypes.c_float]
    handle.DRV_COAX_CTRL_ThrustToMotorPulse.restype = ctypes.c_uint16
    handle.APP_SysIdAlt_VerticalAccel.argtypes = [ctypes.c_float] * 6
    handle.APP_SysIdAlt_VerticalAccel.restype = ctypes.c_float
    return handle


@pytest.fixture(scope="module")
def lib(tmp_path_factory):
    return prepare_handle(build_lib(tmp_path_factory, EXTRA_SOURCES, "sysid-alt"))


@pytest.fixture(scope="module")
def fresh_lib(tmp_path_factory):
    """一份从没收过 SYSID ALT 的库：看上电默认值用。"""
    return prepare_handle(build_lib(tmp_path_factory, EXTRA_SOURCES, "sysid-alt-defaults"))


@pytest.fixture
def coax_params(lib):
    """改生产参数的用例用完要装回去（同一个库在本文件里共用）。"""
    names = ("coax.pos_z_kp", "coax.vel_z_kp", "coax.vel_z_ki", "coax.vel_z_kd",
             "coax.vel_z_i_limit_m_s2", "coax.hover_thrust_n")
    saved = {}
    for name in names:
        value = ctypes.c_float()
        assert lib.DRV_COAX_CTRL_GetParam(name.encode(), ctypes.byref(value)) == 1
        saved[name] = value.value
    yield lambda name, value: lib.DRV_COAX_CTRL_SetParam(name.encode(), ctypes.c_float(value))
    for name, value in saved.items():
        assert lib.DRV_COAX_CTRL_SetParam(name.encode(), ctypes.c_float(value)) == 1


@pytest.fixture
def damped(coax_params):
    """看"跟得上"的用例：vel_z_kp = 4（仍是生产代码按参数表跑）。"""
    assert coax_params("coax.vel_z_kp", 4.0) == 1
    return coax_params


# ---------------------------------------------------------------- 小工具


def command(lib, line):
    lib.harness_reset()
    assert lib.harness_command(line.encode()) == 1
    return texts(lib)


def motor(lib):
    pulse = ctypes.c_uint16()
    return pulse.value if lib.APP_SysId_GetMotorPulse(ctypes.byref(pulse)) else None


def thrust_of(lib, pulse):
    return lib.DRV_COAX_CTRL_MotorPulseToTotalThrust(pulse)


def capped_pulse(lib, total_n, max_pct=75.0):
    cap = int(1100 + max_pct * 840 / 100.0 + 0.5)
    return min(lib.DRV_COAX_CTRL_ThrustToMotorPulse(ctypes.c_float(total_n / 2.0)), cap)


class Plant:
    """槽里的一维质点：F（推力表读回）− m·g，压在槽底时不往下掉。az 是 IMU 该量到的竖直加速度。"""

    def __init__(self, lib, mass_kg=AIRFRAME_KG, z0=H0):
        self.lib, self.mass, self.floor = lib, mass_kg, z0
        self.z, self.vz, self.az = z0, 0.0, 0.0

    def advance(self, pulse, dt=CONTROL_DT_US * 1e-6):
        force = thrust_of(self.lib, pulse if pulse is not None else IDLE_US)
        accel = force / self.mass - G
        if self.z <= self.floor and accel < 0.0:
            accel, self.vz, self.z = 0.0, 0.0, self.floor
        self.az = accel
        self.vz += accel * dt
        self.z = max(self.floor, self.z + self.vz * dt)


class SlotPlant(Plant):
    """带库仑摩擦的槽：静止时 |F − m·g| ≤ Fs 就不动；动起来摩擦 Fk 逆着速度。
    scale = 真实推力 / 推力表读数（推力表偏差）。上/下行阈值 = m·g ± Fs（按真实推力）。"""

    def __init__(self, lib, *, mass_kg=AIRFRAME_KG, fs=0.4, fk=0.25, scale=1.0, z0=H0):
        super().__init__(lib, mass_kg, z0)
        self.fs, self.fk, self.scale = fs, fk, scale
        self.force_n = 0.0          # 本拍真实推力（Tof 的杆弹性形变用）
        self.impact = 0.0           # 落回槽底时最大的撞击速度 [m/s]

    def up_threshold(self):
        return (self.mass * G + self.fs) / self.scale

    def down_threshold(self):
        return (self.mass * G - self.fs) / self.scale

    def advance(self, pulse, dt=CONTROL_DT_US * 1e-6):
        self.force_n = self.scale * thrust_of(self.lib, pulse if pulse is not None else IDLE_US)
        net = self.force_n - self.mass * G
        if self.vz == 0.0:
            if abs(net) <= self.fs or (self.z <= self.floor and net < 0.0):
                self.az = 0.0
                return
            accel = (net - math.copysign(self.fk, net)) / self.mass
        else:
            accel = (net - math.copysign(self.fk, self.vz)) / self.mass
            if (self.vz + accel * dt) * self.vz < 0.0:      # 这一拍减速到零：停住
                self.vz, self.az = 0.0, 0.0
                return
        self.az = accel
        self.vz += accel * dt
        self.z += self.vz * dt
        if self.z <= self.floor:
            self.impact = max(self.impact, -self.vz)
            self.z, self.vz = self.floor, 0.0


class Tof:
    """光流 TOF：每 6 拍（12 ms，约 80 Hz）出一个新样本（新 token），带高斯噪声与偶发尖刺；
    速度估计也带噪声。flex：杆的弹性形变——推力把压弯的杆顶直，测距点随推力每牛抬高这么多
    （2026-09-30 实测约 2 mm/N），杆轴在槽里并没有动。用作 run_alt 的 mutate。"""

    def __init__(self, plant, *, sigma=0.004, spike=0.0, spike_every=0, v_sigma=0.05, seed=7,
                 flex=0.0, trace=None):
        self.plant, self.sigma, self.spike, self.spike_every = plant, sigma, spike, spike_every
        self.v_sigma, self.rng, self.flex = v_sigma, random.Random(seed), flex
        # trace：实测的测距漂移序列（每个样本一个值，去过均值）；给了就回放它代替高斯噪声。
        self.trace, self.offset = trace, (self.rng.randrange(len(trace)) if trace else 0)
        self.token, self.h, self.v, self.samples = 0, H0, 0.0, 0

    def __call__(self, index, obs):
        if index % 6 == 0 or self.token == 0:
            self.samples += 1
            self.token = obs.now_ms
            noise = (self.trace[(self.offset + self.samples) % len(self.trace)] if self.trace
                     else self.rng.gauss(0.0, self.sigma))
            self.h = self.plant.z + self.flex * getattr(self.plant, "force_n", 0.0) + noise
            if self.spike_every and self.samples % self.spike_every == 0:
                self.h += self.spike
            self.v = self.plant.vz + self.rng.gauss(0.0, self.v_sigma)
        obs.height_sample_ms = self.token
        obs.height_m, obs.height_raw_m, obs.vz_m_s = self.h, self.h + RAW_OFFSET, self.v


class Capture:
    """宿主装置的帧/文本缓冲有上限；长跑时边跑边搬走。"""

    def __init__(self, lib):
        self.lib, self.frames, self.texts = lib, [], []

    def flush(self):
        self.frames.extend(frames(self.lib))
        self.texts.extend(texts(self.lib))
        self.lib.harness_reset()

    def samples(self, schema):
        rows, flags = [], 0
        for _function, payload in self.frames:
            batch = decode_batch(payload, schema)
            flags |= batch.flags
            rows.extend(batch.samples)
        return rows, flags

    def phases(self):
        """PHASE 行：[(阶段名, 推力 N)]。"""
        out = []
        for line in self.texts:
            if line.startswith("SYSID PHASE "):
                values = dict(item.split("=", 1) for item in line.split()[2:])
                out.append((values["phase"], int(values["thrust_cn"]) / 100.0))
        return out


def alt_obs(t_ms, t_us, plant, **overrides):
    values = dict(throttle_us=IDLE_US, rc_throttle_low=1, height_valid=1,
                  height_sample_ms=t_ms, height_m=plant.z,
                  height_raw_m=plant.z + RAW_OFFSET, vz_m_s=plant.vz, az_m_s2=plant.az,
                  vbat_v=11.876)
    values.update(overrides)
    return healthy(t_ms, t_us, **values)


def alt_line(inject, mass_g, win_mm, lift_mm, bottom_mm=BOTTOM_MM, top_mm=TOP_MM):
    control = "breakaway" if inject == "break" else "closed_loop"
    return (f"SYSID ALT inject={inject} mass_g={mass_g} win_mm={win_mm} lift_mm={lift_mm} "
            f"bottom_mm={bottom_mm} top_mm={top_mm} control={control}\r\n")


def configure_alt(lib, *, inject="vel", mass_g=0, win_mm=SCENE_WIN_MM, lift_mm=SCENE_LIFT_MM,
                  bottom_mm=BOTTOM_MM, top_mm=TOP_MM):
    line = (f"SYSID ALT inject={inject} mass_g={mass_g} win_mm={win_mm} lift_mm={lift_mm} "
            f"bottom_mm={bottom_mm} top_mm={top_mm}")
    assert command(lib, line) == [alt_line(inject, mass_g, win_mm, lift_mm, bottom_mm, top_mm)]


def prepare_alt(lib, *, spec=None, inject="vel", mass_g=0, win_mm=SCENE_WIN_MM,
                lift_mm=SCENE_LIFT_MM, bottom_mm=BOTTOM_MM, top_mm=TOP_MM,
                max_pct=75.0, target_n=TARGET_N, rate_hz=50, plant=None, **obs):
    reset(lib, rate_hz=rate_hz,
          spec=Excitation(**(spec or (BREAK_SPEC if inject == "break" else CLOSED_DOUBLET))))
    if target_n:
        assert lib.APP_SysId_SetThrottle(ctypes.c_float(target_n), ctypes.c_float(max_pct)) == 1
    assert lib.APP_SysId_SetMode(MODE_ALT, ctypes.c_float(0.0523598776)) == 1
    configure_alt(lib, inject=inject, mass_g=mass_g, win_mm=win_mm, lift_mm=lift_mm,
                  bottom_mm=bottom_mm, top_mm=top_mm)
    plant = plant or Plant(lib)
    # 没跑时控制拍也调 Update：START 靠它知道解锁/油门杆/测高（要攒够 8 个新测距样本）。
    for k in range(10):
        lib.APP_SysId_Update(ctypes.byref(alt_obs(880 + 2 * k, 880_000 + 2000 * k, plant, **obs)))
    lib.harness_reset()
    return plant


def start_alt(lib, **kwargs):
    plant = prepare_alt(lib, **kwargs)
    assert lib.APP_SysId_Start() == 1, texts(lib)
    return plant


def run_alt(lib, plant, ticks, *, mutate=None, record=None, capture=None, t0_ms=1_000):
    """跑 ticks 个 500 Hz 控制拍，质点跟着电机脉宽走。返回停在第几拍。"""
    stopped = ticks
    for index in range(ticks):
        obs = alt_obs(t0_ms + index * 2, t0_ms * 1000 + index * CONTROL_DT_US, plant)
        if mutate is not None:
            mutate(index, obs)
        lib.APP_SysId_Update(ctypes.byref(obs))
        pulse = motor(lib)
        if record is not None:
            record.append((lib.APP_SysId_GetPhase(), pulse, plant.z))
        plant.advance(pulse)
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


def schema_of(lib):
    lib.harness_reset()
    lib.APP_SysId_ReportSchema()
    return parse_schema_lines(texts(lib))


def phase_runs(record):
    order = []
    for phase, _pulse, _z in record:
        if not order or order[-1][0] != phase:
            order.append([phase, 0])
        order[-1][1] += 1
    return order


def ticks_in(record, phase):
    return [i for i, (p, _pulse, _z) in enumerate(record) if p == phase]


def first_tick_in(record, phase, after=0):
    return next(i for i, (p, _pulse, _z) in enumerate(record) if i >= after and p == phase)


def max_tick_drop(lib, record, start):
    """从 start 起相邻两拍下发推力（按脉宽读回）最多掉多少 [N]：软着陆不许一拍掉很多。"""
    pulses = [pulse for _p, pulse, _z in record[start:] if pulse is not None]
    return max((thrust_of(lib, a) - thrust_of(lib, b) for a, b in zip(pulses, pulses[1:])),
               default=0.0)


def teardown_run(lib):
    lib.APP_SysId_Stop(b"test")
    for _ in range(64):
        lib.APP_SysId_StreamTick()
    lib.harness_reset()


# ---------------------------------------------------------------- 命令面


def test_power_on_defaults_match_the_slot_rig(fresh_lib):
    """上电默认 break / 0 / 60 / 60、端点未配置：不配端点就开不了跑。"""
    header = (ROOT / "App/Inc/app_sysid_alt.h").read_text(encoding="utf-8")
    assert re.search(r"#define APP_SYSID_ALT_WIN_MM_DEFAULT\s+60U", header)
    assert re.search(r"#define APP_SYSID_ALT_LIFT_MM_DEFAULT\s+60U", header)
    lib = fresh_lib
    assert command(lib, "SYSID ALT") == [alt_line("break", 0, 60, 60, 0, 0)]

    reset(lib, rate_hz=50, spec=Excitation(**BREAK_SPEC))
    assert lib.APP_SysId_SetThrottle(ctypes.c_float(TARGET_N), ctypes.c_float(75.0)) == 1
    assert lib.APP_SysId_SetMode(MODE_ALT, ctypes.c_float(0.0523598776)) == 1
    plant = Plant(lib)
    for k in range(10):
        lib.APP_SysId_Update(ctypes.byref(alt_obs(880 + 2 * k, 880_000 + 2000 * k, plant)))
    lib.harness_reset()
    assert lib.APP_SysId_Start() == 0
    assert texts(lib) == ["ERR sysid alt slot_endpoints_missing\r\n"]


def test_default_window_is_slot_bottom_plus_120_mm(fresh_lib):
    """槽行程 160 mm、lift 60 + win 60：上端硬窗 = min(槽底 + 120, 槽顶 − 40) = 槽底 + 120 mm。"""
    lib = fresh_lib
    reset(lib, rate_hz=50, spec=Excitation(**CLOSED_DOUBLET))
    assert lib.APP_SysId_SetThrottle(ctypes.c_float(TARGET_N), ctypes.c_float(75.0)) == 1
    assert lib.APP_SysId_SetMode(MODE_ALT, ctypes.c_float(0.0523598776)) == 1
    configure_alt(lib, inject="vel", win_mm=60, lift_mm=60, bottom_mm=80, top_mm=240)
    plant = Plant(lib)
    for k in range(10):
        lib.APP_SysId_Update(ctypes.byref(alt_obs(880 + 2 * k, 880_000 + 2000 * k, plant)))
    lib.harness_reset()
    assert lib.APP_SysId_Start() == 1
    assert any(" alt_win_mm=60 alt_lift_mm=60 alt_h0_mm=80 alt_control=closed_loop " in line
               for line in texts(lib))
    climb_end = RAMP_TICKS + 300 + 50                  # 60 mm @ 0.1 m/s = 300 拍，再进稳定段

    def peek(index, obs):
        if climb_end <= index < climb_end + 20:
            obs.height_m = 0.0800 + 0.1195             # 窗口上沿以内：不停
        elif index >= climb_end + 20:
            obs.height_m = 0.0800 + 0.1210             # 过了 120 mm：平均值一越线就转软着陆

    record = []
    stopped = run_alt(lib, plant, climb_end + 6000, mutate=peek, record=record)
    entry = first_tick_in(record, PHASE_RAMP_DOWN)
    assert climb_end + 20 <= entry <= climb_end + 28
    assert stopped > entry + 400                       # 慢慢降完才报，不是当拍交还油门
    assert lib.APP_SysId_GetLastReason() == b"height_window"


def test_alt_command_echoes_one_line(lib):
    lib.APP_SysId_Stop(b"test")
    configure_alt(lib)
    assert command(lib, "SYSID ALT") == [alt_line("vel", 0, SCENE_WIN_MM, SCENE_LIFT_MM)]
    assert command(lib, "SYSID ALT inject=break mass_g=1184 win_mm=200 lift_mm=120") == [
        alt_line("break", 1184, 200, 120)]
    # 只给一部分键：其余保持原值。
    assert command(lib, "SYSID ALT inject=pos") == [alt_line("pos", 1184, 200, 120)]
    assert command(lib, "SYSID ALT mass_g=0 bottom_mm=460 top_mm=620") == [
        alt_line("pos", 0, 200, 120, 460, 620)]
    configure_alt(lib)


@pytest.mark.parametrize("args,reason", [
    ("mass_g=499", "range"), ("mass_g=3001", "range"), ("mass_g=70000", "range"),
    ("win_mm=29", "range"), ("win_mm=401", "range"),
    ("lift_mm=29", "range"), ("lift_mm=301", "range"),
    ("bottom_mm=4001", "range"), ("top_mm=4001", "range"),
    ("inject=up", "usage"), ("inject=force", "usage"), ("mass_g=abc", "usage"),
    ("mass_g=1.5", "usage"), ("foo=1", "usage"), ("lift_mm=100 bar", "usage"),
])
def test_alt_command_rejects_whole_line_without_touching_the_config(lib, args, reason):
    lib.APP_SysId_Stop(b"test")
    configure_alt(lib, inject="vel", mass_g=1184, win_mm=200, lift_mm=120)
    assert command(lib, f"SYSID ALT inject=pos {args}") == [
        f"SYSID ALT event=rejected reason={reason}\r\n"]
    assert command(lib, "SYSID ALT") == [alt_line("vel", 1184, 200, 120)]
    configure_alt(lib)


@pytest.mark.parametrize("args", ["mass_g=500", "mass_g=3000", "win_mm=30", "win_mm=400",
                                  "lift_mm=30", "lift_mm=300", "bottom_mm=4000", "top_mm=0"])
def test_alt_command_accepts_its_range_endpoints(lib, args):
    lib.APP_SysId_Stop(b"test")
    configure_alt(lib)
    line = command(lib, f"SYSID ALT {args}")[0]
    assert f" {args} " in line.rstrip() + " "
    configure_alt(lib)


def test_alt_config_is_idle_only_but_readable_while_running(lib):
    plant = start_alt(lib)
    run_alt(lib, plant, 10)
    assert command(lib, "SYSID ALT lift_mm=50") == ["SYSID ALT event=rejected reason=running\r\n"]
    assert command(lib, "SYSID ALT") == [alt_line("vel", 0, SCENE_WIN_MM, SCENE_LIFT_MM)]
    teardown_run(lib)


@pytest.mark.parametrize("token", ["ALT", "4"])
def test_mode_alt_by_name_and_number(lib, token):
    reset(lib)
    lines = command(lib, f"SYSID MODE {token}")
    assert f" mode={MODE_ALT} " in next(line for line in lines if line.startswith("SYSID READY "))
    assert lib.APP_SysId_GetMode() == MODE_ALT
    assert lib.APP_SysId_SetMode(MODE_ALT + 3, ctypes.c_float(0.05)) == 0   # 5 = XY、6 = YAW 已启用，7 起才越界


# ---------------------------------------------------------------- 开跑前检查


def test_start_needs_auto_throttle(lib):
    prepare_alt(lib, target_n=0)                      # reset 已把 target_n 置 0（手动）
    assert lib.APP_SysId_Start() == 0
    assert texts(lib) == ["ERR sysid alt needs auto throttle (SYSID THROTTLE target_n>0)\r\n"]


@pytest.mark.parametrize("inject,amp,accepted", [
    ("break", 1.0, True), ("break", 1.01, False),
    ("vel", 0.3, True), ("vel", 0.31, False),
    ("pos", 0.15, True), ("pos", 0.151, False),
])
def test_start_checks_the_amplitude_per_injection_type(lib, inject, amp, accepted):
    base = BREAK_SPEC if inject == "break" else CLOSED_DOUBLET
    prepare_alt(lib, spec=dict(base, amplitude_rad_s=amp), inject=inject)
    assert lib.APP_SysId_Start() == int(accepted)
    if accepted:
        teardown_run(lib)
    else:
        assert texts(lib) == [
            f"ERR sysid alt amp over limit: inject={inject} amp<={AMP_TEXT[inject]}\r\n"]
    configure_alt(lib)


@pytest.mark.parametrize("target,reason", [
    (WEIGHT_N - 0.05, "break target below run weight"),
    (WEIGHT_N + 5.05, "break target over run weight + 5 N"),
])
def test_break_search_ceiling_must_sit_between_weight_and_weight_plus_3_n(lib, target, reason):
    prepare_alt(lib, inject="break", target_n=target)
    assert lib.APP_SysId_Start() == 0
    assert texts(lib) == [f"ERR sysid alt {reason}\r\n"]
    prepare_alt(lib, inject="break", target_n=WEIGHT_N + 2.9)
    assert lib.APP_SysId_Start() == 1
    teardown_run(lib)


def test_break_ceiling_uses_the_configured_rig_mass(lib):
    """重力按本轮随动质量算：1.184 kg 时 15.5 N 可用（整机最大推力 15.65 N 以内），11.5 N 低于重力
    被拒；同样 11.5 N 按机体质量（0.75 kg）算就是合法的。"""
    prepare_alt(lib, inject="break", mass_g=1184, target_n=15.5)
    assert lib.APP_SysId_Start() == 1
    teardown_run(lib)
    prepare_alt(lib, inject="break", mass_g=1184, target_n=11.5)
    assert lib.APP_SysId_Start() == 0
    assert texts(lib) == ["ERR sysid alt break target below run weight\r\n"]
    prepare_alt(lib, inject="break", mass_g=0, target_n=11.5)
    assert lib.APP_SysId_Start() == 1
    teardown_run(lib)


@pytest.mark.parametrize("bottom,top,reason", [
    (0, TOP_MM, "slot_endpoints_missing"), (BOTTOM_MM, 0, "slot_endpoints_missing"),
    (BOTTOM_MM, BOTTOM_MM + 134, "slot_travel"), (BOTTOM_MM, BOTTOM_MM + 186, "slot_travel"),
    (BOTTOM_MM, BOTTOM_MM - 10, "slot_travel"),
])
def test_start_needs_plausible_slot_endpoints(lib, bottom, top, reason):
    prepare_alt(lib, bottom_mm=bottom, top_mm=top)
    assert lib.APP_SysId_Start() == 0
    assert texts(lib) == [f"ERR sysid alt {reason}\r\n"]


@pytest.mark.parametrize("offset,accepted", [(0.019, True), (-0.019, True), (0.021, False)])
def test_start_needs_the_body_at_the_slot_bottom(lib, offset, accepted):
    """上一轮停在槽中部时不能开跑：上端保护窗按物理槽顶，不按开跑高度。"""
    prepare_alt(lib, plant=Plant(lib, z0=H0 + offset))
    assert lib.APP_SysId_Start() == int(accepted)
    if accepted:
        teardown_run(lib)
    else:
        assert texts(lib) == ["ERR sysid alt slot_start\r\n"]


def test_start_needs_a_valid_height(lib):
    prepare_alt(lib, height_valid=0)
    assert lib.APP_SysId_Start() == 0
    assert texts(lib) == ["ERR sysid alt height invalid (TOF)\r\n"]


def test_start_needs_eight_fresh_height_samples(lib):
    """同一个测距 token 重复送来不算新样本：攒不够 8 个就不开跑。"""
    prepare_alt(lib, height_sample_ms=877)
    assert lib.APP_SysId_Start() == 0
    assert texts(lib) == ["ERR sysid alt height invalid (TOF)\r\n"]


def test_zero_height_token_is_invalid(lib):
    prepare_alt(lib, inject="break", height_sample_ms=0)
    assert lib.APP_SysId_Start() == 0
    assert texts(lib) == ["ERR sysid alt height invalid (TOF)\r\n"]


# ---------------------------------------------------------------- break：离地/滑落阈值


def run_break(lib, plant, *, ticks=12_000, tof=None, record=None, capture=None, **kwargs):
    start_alt(lib, inject="break", plant=plant, max_pct=95.0, **kwargs)
    tof = tof or Tof(plant)
    return run_alt(lib, plant, ticks, mutate=tof, record=record, capture=capture)


def test_break_finds_liftoff_and_slide_thresholds_and_completes(lib):
    schema = schema_of(lib)
    plant = SlotPlant(lib, fs=0.8, fk=0.6)       # 离地与滑落之间 1.6 N 的静摩擦带
    lib.harness_reset()
    record, capture = [], Capture(lib)
    stopped = run_break(lib, plant, rate_hz=50, record=record, capture=capture,
                        target_n=WEIGHT_N + 2.0)
    assert stopped < 12_000
    assert lib.APP_SysId_GetState() == STATE_DONE, lib.APP_SysId_GetLastReason()
    assert lib.APP_SysId_GetLastReason() == b"complete"
    assert motor(lib) is None, "结束后必须交还遥控器油门"
    assert [phase for phase, _count in phase_runs(record)] == BREAK_SEQUENCE + [PHASE_IDLE]

    phases = dict(capture.phases())
    up, down = plant.up_threshold(), plant.down_threshold()
    assert phases["climb"] == pytest.approx(0.9 * WEIGHT_N, abs=0.05)
    # 粗判：离地判定时的指令只比真阈值高一点（慢升 0.5 N/s × 检测迟滞）。
    assert up <= phases["settle"] <= up + 0.35
    assert down < phases["excite"] < up                      # 停住：落在静摩擦带里
    assert down - 0.35 <= phases["descend"] <= down + 0.02   # 滑落判定时的指令
    # 全程离槽底不超过 90 mm（离地门槛 25 mm，按实测测距漂移定）；回到槽底。
    assert max(z for _p, _pulse, z in record) < H0 + 0.090
    assert record[-1][2] == pytest.approx(H0, abs=0.001)

    rows, flags = capture.samples(schema)
    assert flags & FLAG_ALT and not flags & FLAG_THRUST_CAPPED
    assert rows and all(row["height_sp"] == row["vz_sp"] == 0.0 for row in rows)
    assert all(row["angle_sp"] == 0.0 for row in rows)       # 姿态目标恒为杆轴角 0


def test_break_reports_its_control_and_endpoints_in_the_start_provenance(lib):
    plant = start_alt(lib, inject="break", plant=SlotPlant(lib))
    lines = texts(lib)
    starts = [i for i, line in enumerate(lines) if line.startswith("SYSID start ")]
    run = re.search(r"run=(\d+)", lines[starts[0]]).group(1)
    assert lines[starts[0] + 1] == (
        f"SYSID ALTSTART run={run} alt_inject=break alt_mass_g=755 alt_win_mm={SCENE_WIN_MM} "
        f"alt_lift_mm={SCENE_LIFT_MM} alt_h0_mm=80 alt_control=breakaway alt_target_cn=800 "
        f"alt_bottom_mm={BOTTOM_MM} alt_top_mm={TOP_MM}\r\n")
    teardown_run(lib)
    del plant


@pytest.mark.parametrize("scale", [0.95, 1.0, 1.05])
def test_break_thresholds_follow_the_real_thrust_scale(lib, scale):
    """推力表偏差：真实推力 = scale × 表值。粗判阈值跟着真阈值走（这正是要测的量）。"""
    plant = SlotPlant(lib, fs=0.3, fk=0.2, scale=scale)
    target = min(WEIGHT_N + 2.9, plant.up_threshold() + 1.0)
    capture = Capture(lib)
    lib.harness_reset()
    run_break(lib, plant, capture=capture, target_n=target)
    assert lib.APP_SysId_GetState() == STATE_DONE, lib.APP_SysId_GetLastReason()
    phases = dict(capture.phases())
    assert plant.up_threshold() <= phases["settle"] <= plant.up_threshold() + 0.35
    if "descend" in phases:
        assert plant.down_threshold() - 0.35 <= phases["descend"] <= plant.down_threshold() + 0.02


def test_break_low_friction_slides_back_during_the_stop_and_still_completes(lib):
    """静摩擦很小：制停那一档就滑回槽底——本轮只有上行阈值，直接降到怠速完成。"""
    plant = SlotPlant(lib, fs=0.05, fk=0.03)
    record, capture = [], Capture(lib)
    lib.harness_reset()
    run_break(lib, plant, record=record, capture=capture)
    assert lib.APP_SysId_GetState() == STATE_DONE, lib.APP_SysId_GetLastReason()
    runs = [phase for phase, _count in phase_runs(record)]
    assert runs == [PHASE_RAMP_UP, PHASE_CLIMB, PHASE_SETTLE, PHASE_RAMP_DOWN, PHASE_IDLE]


@pytest.mark.parametrize("flex", [0.002, 0.003])
def test_rod_flex_under_rising_thrust_never_counts_as_liftoff(lib, flex):
    """2026-09-30 实机：推力把压弯的杆顶直，测距涨 10～25 mm，杆轴在槽里没动——
    固件曾在 1.6 N / 6.1 N 误判离地。现在预升段不判、慢升段要"高 10 mm 且在升"：不许误判。"""
    plant = SlotPlant(lib, fs=50.0, fk=50.0)            # 杆轴卡在槽底不动
    record = []
    tof = Tof(plant, sigma=0.004, seed=5, flex=flex)
    run_break(lib, plant, tof=tof, record=record)
    assert PHASE_SETTLE not in [phase for phase, _pulse, _z in record]
    assert lib.APP_SysId_GetLastReason() == b"no_liftoff"


def test_slow_tof_wander_near_hover_never_counts_as_liftoff(lib):
    """2026-09-30 实测：推力接近重力时测距慢漂，8 样本平均最大偏离 10～14 mm、斜率到 ±0.1 m/s，
    曾在 11.0 N / 14.1 N 误判离地。这里用 ±12 mm、0.9 Hz 与 2.3 Hz 叠加的慢漂 + 杆形变：不许误判。"""
    wander = [0.009 * math.sin(2 * math.pi * 0.9 * k / 80.0) +
              0.004 * math.sin(2 * math.pi * 2.3 * k / 80.0 + 1.0) for k in range(4000)]
    plant = SlotPlant(lib, fs=50.0, fk=50.0)            # 杆轴卡在槽底不动
    record = []
    run_break(lib, plant, tof=Tof(plant, trace=wander, seed=3, flex=0.002), record=record,
              target_n=WEIGHT_N + 2.0)
    assert PHASE_SETTLE not in [phase for phase, _pulse, _z in record]
    assert lib.APP_SysId_GetLastReason() == b"no_liftoff"


@pytest.mark.parametrize("fs,fk", [(0.4, 0.25), (1.0, 0.6)])
def test_real_liftoff_is_still_found_on_a_flexing_rod(lib, fs, fk):
    plant = SlotPlant(lib, fs=fs, fk=fk)
    capture = Capture(lib)
    lib.harness_reset()
    run_break(lib, plant, tof=Tof(plant, sigma=0.004, seed=5, flex=0.002), capture=capture,
              target_n=WEIGHT_N + 2.0)
    assert lib.APP_SysId_GetState() == STATE_DONE, lib.APP_SysId_GetLastReason()
    phases = dict(capture.phases())
    assert plant.up_threshold() <= phases["settle"] <= plant.up_threshold() + 0.4


def test_the_stop_floor_never_raises_the_thrust(lib):
    """制停的推力下限只防降过头：离地判定时的指令再低，也不许被夹紧"抬"上去。"""
    source = strip_c_comments((ROOT / "App/Src/app_sysid_alt.c").read_text(encoding="utf-8"))
    assert ("alt.cmd_n = fmaxf(alt.cmd_n - alt.stop_first_n, fminf(alt.f_floor_n, alt.cmd_n));"
            in source)
    assert "alt.cmd_n = alt.f_floor_n;" not in source


def test_break_tof_noise_alone_never_counts_as_liftoff(lib):
    """机体压在槽底不动（静摩擦极大），测距 σ 5 mm 且每 20 个样本一次 +15 mm 尖刺：
    不许判离地；升到搜索上限再等 1 s 仍未离地 → no_liftoff：人在槽底，1 s 降到怠速再报、交还油门。"""
    plant = SlotPlant(lib, fs=50.0, fk=50.0)
    record = []
    tof = Tof(plant, sigma=0.005, spike=0.015, spike_every=20, seed=3)
    stopped = run_break(lib, plant, tof=tof, record=record)
    assert lib.APP_SysId_GetState() == STATE_ABORTED
    assert lib.APP_SysId_GetLastReason() == b"no_liftoff"
    assert PHASE_SETTLE not in [phase for phase, _pulse, _z in record]
    climb = (TARGET_N - 0.9 * WEIGHT_N) / 0.5
    assert first_tick_in(record, PHASE_RAMP_DOWN) * 0.002 == pytest.approx(1.5 + climb + 1.0,
                                                                            abs=0.05)
    assert stopped * 0.002 == pytest.approx(1.5 + climb + 1.0 + 1.0, abs=0.05)
    assert max_tick_drop(lib, record, first_tick_in(record, PHASE_RAMP_DOWN)) < 0.1
    assert motor(lib) is None


def test_break_bypasses_the_production_height_pid(lib, coax_params):
    """慢升段的推力只由 F_start + r·t 决定：换生产 Z 增益，脉宽一拍不差。"""
    def climb_pulses(kp):
        assert coax_params("coax.vel_z_kp", kp) == 1
        plant = SlotPlant(lib, fs=50.0, fk=50.0)
        record = []
        run_break(lib, plant, ticks=RAMP_TICKS + 400, record=record)
        teardown_run(lib)
        return [pulse for phase, pulse, _z in record if phase == PHASE_CLIMB]

    base = climb_pulses(1.0)
    assert base and base == climb_pulses(10.0)
    assert thrust_of(lib, base[-1]) == pytest.approx(0.9 * WEIGHT_N + 0.5 * len(base) * 0.002,
                                                     abs=0.05)


def test_break_upper_brake_gate_trips_before_the_hard_window(lib):
    """起升刚开始（推力不到 0.5·m·g，机体必压在槽底）就预测会冲过上沿：不用软着陆，当拍中止。"""
    plant = SlotPlant(lib, fs=50.0, fk=50.0)
    start_alt(lib, inject="break", plant=plant, max_pct=95.0)

    def rising(index, obs):
        obs.height_sample_ms = obs.now_ms
        obs.height_m = H0 + (0.16 if index >= 5 else 0.0)
        obs.vz_m_s = 0.5 if index >= 5 else 0.0

    stopped = run_alt(lib, plant, 60, mutate=rising)
    assert 5 <= stopped <= 13
    assert lib.APP_SysId_GetLastReason() == b"height_brake"
    assert lib.APP_SysId_GetState() == STATE_ABORTED


def test_break_rejects_a_replayed_height_token_even_if_the_valid_bit_stays_set(lib):
    plant = SlotPlant(lib, fs=50.0, fk=50.0)
    start_alt(lib, inject="break", plant=plant)

    def stale(_index, obs):
        obs.height_sample_ms = 1000  # 控制拍继续，新测距没有来
        obs.height_m = H0

    assert run_alt(lib, plant, 100, mutate=stale) == 51
    assert lib.APP_SysId_GetLastReason() == b"height_invalid"
    assert lib.APP_SysId_GetState() == STATE_ABORTED


def test_break_keeps_axis_attitude_once_thrust_is_sufficient_and_checks_lut_freshness(lib):
    plant = SlotPlant(lib, fs=50.0, fk=50.0)
    start_alt(lib, inject="break", plant=plant)
    alpha, beta = ctypes.c_uint16(), ctypes.c_uint16()
    for index in range(700):
        obs = alt_obs(1000 + 2 * index, 1_000_000 + CONTROL_DT_US * index, plant,
                      roll_rad=0.01 if index >= 500 else 0.0)
        lib.APP_SysId_Update(ctypes.byref(obs))
        lib.APP_SysId_StreamTick()
    lib.APP_SysId_GetServoTargets(ctypes.byref(alpha), ctypes.byref(beta))
    assert (alpha.value, beta.value) != (1500, 1500)
    obs = alt_obs(2400, 2_400_000, plant, roll_rad=0.01, thrust_valid=0)
    lib.APP_SysId_Update(ctypes.byref(obs))
    assert lib.APP_SysId_GetLastReason() == b"thrust_stale"
    assert motor(lib) is None


def test_closed_loop_ramp_up_already_holds_attitude(lib):
    """vel/pos 预升段推力过 2 N 就保持姿态（同 break）：2026-09-30 pos 轮预升 1.5 s 舵机回中，
    机体绕杆扭到 −4～−5° 才在慢升段扶正——作者看到"起飞时往一边偏然后才扶正"。"""
    plant = start_alt(lib, inject="pos", plant=SlotPlant(lib, fs=50.0, fk=50.0))
    alpha, beta = ctypes.c_uint16(), ctypes.c_uint16()
    for index in range(600):
        obs = alt_obs(1000 + 2 * index, 1_000_000 + CONTROL_DT_US * index, plant,
                      roll_rad=0.03 if index >= 300 else 0.0)
        lib.APP_SysId_Update(ctypes.byref(obs))
        plant.advance(motor(lib))
        lib.APP_SysId_StreamTick()
    assert lib.APP_SysId_GetPhase() == PHASE_RAMP_UP
    assert thrust_of(lib, motor(lib)) >= 2.0
    lib.APP_SysId_GetServoTargets(ctypes.byref(alpha), ctypes.byref(beta))
    assert (alpha.value, beta.value) != (1500, 1500)
    teardown_run(lib)


def _alt_moment_after_tilt(lib, roll, pitch):
    """break 卡在槽底（推力足够、已在保持姿态），最后 150 拍把姿态改成 (roll, pitch)：
    返回舵机下发的机体力矩（按当时推力由舵机脉宽反算）。"""
    lib.DRV_COAX_CTRL_MomentFromServoPulses.argtypes = [ctypes.c_float, ctypes.c_uint16,
                                                         ctypes.c_uint16, ctypes.POINTER(ctypes.c_float)]
    plant = SlotPlant(lib, fs=50.0, fk=50.0)
    start_alt(lib, inject="break", plant=plant, max_pct=95.0)
    alpha, beta = ctypes.c_uint16(), ctypes.c_uint16()
    for index in range(900):
        tilt = index >= 750
        obs = alt_obs(1000 + 2 * index, 1_000_000 + CONTROL_DT_US * index, plant,
                      roll_rad=roll if tilt else 0.0, pitch_rad=pitch if tilt else 0.0)
        lib.APP_SysId_Update(ctypes.byref(obs))
        plant.advance(motor(lib))
        lib.APP_SysId_StreamTick()
    lib.APP_SysId_GetServoTargets(ctypes.byref(alpha), ctypes.byref(beta))
    thrust = thrust_of(lib, motor(lib))
    moment = (ctypes.c_float * 3)()
    assert lib.DRV_COAX_CTRL_MomentFromServoPulses(ctypes.c_float(thrust), alpha.value,
                                                   beta.value, moment)
    teardown_run(lib)
    return moment[0], moment[1]


def test_alt_holds_both_level_axes_not_only_the_rod_axis(lib):
    """槽式台架上杆两端能一高一低（绕水平且垂直于杆的轴 m 倾斜）：2026-09-30 只管杆轴 n 时
    "一边起来一边没起来"。现在两个水平轴都用生产姿态环保持在开跑时的姿态上：
    绕 m 倾斜给出沿 −m 的纠正力矩，绕 n 倾斜给出沿 −n 的纠正力矩（负反馈）。"""
    psi = math.pi / 4                                 # 装置的杆轴方位（test_sysid_runtime_contract.reset）
    n = (math.cos(psi), math.sin(psi))
    m = (-math.sin(psi), math.cos(psi))
    beta = 0.05
    for axis, other in ((m, n), (n, m)):
        mx, my = _alt_moment_after_tilt(lib, beta * axis[0], beta * axis[1])
        along = mx * axis[0] + my * axis[1]
        across = mx * other[0] + my * other[1]
        assert along < -0.005, (axis, along, across)          # 纠正方向与倾斜相反
        assert abs(across) < 0.3 * abs(along), (axis, along, across)


def test_break_brakes_when_it_slides_down_too_fast(lib):
    """滑落后下降过快（100 ms 平均高度降 >15 mm，即 >0.15 m/s）：推力短时 +0.4 N 刹车；
    落回槽底停稳后照常完成。静摩擦 1 N、动摩擦 0.6 N：停在约 40 mm 处，一滑就越滑越快。"""
    plant = SlotPlant(lib, fs=1.0, fk=0.6)
    record, capture = [], Capture(lib)
    lib.harness_reset()
    run_break(lib, plant, tof=Tof(plant, sigma=0.0), record=record, capture=capture,
              target_n=WEIGHT_N + 2.0)
    descend = [thrust_of(lib, pulse) for phase, pulse, _z in record if phase == PHASE_DESCEND]
    assert descend
    base = dict(capture.phases())["descend"]
    assert max(descend) == pytest.approx(base + 0.4, abs=0.05)
    assert lib.APP_SysId_GetState() == STATE_DONE, lib.APP_SysId_GetLastReason()


def test_break_stop_timeout_when_it_keeps_wandering(lib):
    """制停阶段平均高度一直来回晃（既不停也不回底）：4 s 后 stop_timeout。"""
    plant = SlotPlant(lib, fs=0.4, fk=0.25)
    start_alt(lib, inject="break", plant=plant, max_pct=95.0)
    tof = Tof(plant, sigma=0.0)

    def wander(index, obs):
        tof(index, obs)
        if lib.APP_SysId_GetPhase() == PHASE_SETTLE:
            # 测距照 12 ms 一个样本；每 250 ms 在 ±12 mm 之间来回（既不停住也不回槽底）。
            obs.height_m = H0 + 0.030 + (0.012 if (index // 125) % 2 else -0.012)
            obs.vz_m_s = 0.0

    run_alt(lib, plant, 12_000, mutate=wander)
    assert lib.APP_SysId_GetState() == STATE_ABORTED
    assert lib.APP_SysId_GetLastReason() in (b"stop_timeout", b"stop_failed")


def test_break_brake_gate_steps_the_thrust_down_during_the_stop_instead_of_aborting(lib):
    """2026-09-30 实机：离地后降了一档仍以约 0.1 m/s 匀速上爬，提前制动门当拍中止、把电机交还怠速。
    现在制停中预测会冲过上沿（槽顶 − 10 mm）：每 100 ms 再降一档 0.4 N，不中止。"""
    plant = SlotPlant(lib, fs=0.4, fk=0.25)
    tof = Tof(plant, sigma=0.0)
    state = {}

    def near_top(index, obs):
        tof(index, obs)
        if "t0" in state or lib.APP_SysId_GetPhase() == PHASE_SETTLE:
            t0 = state.setdefault("t0", index)
            # 离上沿 24 mm、以 0.05 m/s 往上走（机体真实位置不管）：制动门判"会冲过去"，硬窗还没到。
            obs.height_m = UPPER_BREAK - 0.024 + 0.05 * (index - t0) * 0.002
            if index - t0 >= 250:
                lib.APP_SysId_Stop(b"test")

    record = []
    start_alt(lib, inject="break", plant=plant, max_pct=95.0, target_n=WEIGHT_N + 2.0)
    run_alt(lib, plant, 12_000, mutate=near_top, record=record)
    t0 = state["t0"]
    assert all(phase == PHASE_SETTLE for phase, _pulse, _z in record[t0:t0 + 250])
    steps = [(i, thrust_of(lib, a) - thrust_of(lib, b))
             for i, ((_p, a, _z), (_q, b, _w)) in enumerate(zip(record[t0 + 2:t0 + 250],
                                                               record[t0 + 3:t0 + 250]))
             if thrust_of(lib, a) - thrust_of(lib, b) > 0.2]
    assert len(steps) >= 3, steps
    assert all(drop == pytest.approx(0.4, abs=0.06) for _i, drop in steps)
    assert all(b - a >= 50 for (a, _x), (b, _y) in zip(steps, steps[1:]))
    assert lib.APP_SysId_GetLastReason() == b"test"


def test_an_internal_abort_lands_softly_instead_of_cutting_the_motors(lib):
    """2026-09-30：height_brake 当拍把推力从 14.3 N 交还到怠速，机体从槽底以上约 8 cm 摔下来弹跳。
    现在本模块自己的中止先软着陆：阶段报 ramp_down，推力一拍掉不多，落回槽底撞击速度小，
    落稳再 1 s 降到怠速，最后才报原中止理由、交还油门。"""
    plant = SlotPlant(lib, fs=0.8, fk=0.6)
    tof = Tof(plant, sigma=0.0)
    state = {}

    def glitch(index, obs):
        tof(index, obs)
        if "t0" not in state and lib.APP_SysId_GetPhase() == PHASE_EXCITE:
            state["t0"] = index
        if "t0" in state and index - state["t0"] < 60:
            obs.height_m = UPPER_BREAK + 0.020      # 测距一阵越过上沿：height_window

    record = []
    run_break(lib, plant, tof=glitch, record=record, target_n=WEIGHT_N + 2.0)
    entry = first_tick_in(record, PHASE_RAMP_DOWN, after=state["t0"])
    assert record[entry][2] > H0 + 0.020                  # 在半空（停住的地方）开始软着陆
    assert lib.APP_SysId_GetLastReason() == b"height_window"
    assert lib.APP_SysId_GetState() == STATE_ABORTED
    assert max_tick_drop(lib, record, entry) < 0.1
    assert plant.impact < 0.3
    assert record[-1][2] == pytest.approx(H0, abs=0.001)
    assert motor(lib) is None


def _yaw_snap(rate, start, ticks):
    """start 起 ticks 拍的测量角速度绕竖直轴 rate（与杆轴垂直，全算残差；控制用的那份不动）。"""
    def mutate(index, obs):
        if start <= index < start + ticks:
            obs.gyro_rad_s = (ctypes.c_float * 3)(0.0, 0.0, rate)
    return mutate


def test_alt_tolerates_rod_end_snaps_up_to_1_5_rad_s(lib):
    """2026-09-30 两轮：杆一端卡住、歪到约 3.5° 后突然松开，绕垂直于杆的水平轴超过单轴台架的残差门
    0.52 rad/s，当拍断电摔下。槽式台架上这个方向本来就能转，ALT 的残差门至少 1.5 rad/s。"""
    plant = SlotPlant(lib, fs=50.0, fk=50.0)
    start_alt(lib, inject="break", plant=plant, max_pct=95.0)
    assert run_alt(lib, plant, 1200, mutate=_yaw_snap(1.2, 1000, 20)) == 1200
    teardown_run(lib)


def test_alt_residual_over_the_limit_lands_softly(lib):
    """超过 1.5 rad/s、推力已过 0.5·m·g：先软着陆（在槽底，1 s 降到怠速）再报 axis_residual。"""
    plant = SlotPlant(lib, fs=50.0, fk=50.0)
    start_alt(lib, inject="break", plant=plant, max_pct=95.0)
    record = []
    stopped = run_alt(lib, plant, 4000, mutate=_yaw_snap(1.6, 1000, 20), record=record)
    assert first_tick_in(record, PHASE_RAMP_DOWN) == 1000
    assert 1000 + 500 <= stopped <= 1000 + 510
    assert max_tick_drop(lib, record, 1000) < 0.1
    assert lib.APP_SysId_GetLastReason() == b"axis_residual"
    assert motor(lib) is None


def test_soft_landing_starts_from_the_recent_average_thrust_not_the_peak(lib):
    """2026-09-30 alt_053452：高度环振荡时推力正顶在 16 N，软着陆从它起降，机体先被推到槽顶附近。
    现在起点取 min(当前, 近 0.5 s 一阶平均)：推力还在往上走时，起点就是那个落后的平均值。"""
    plant = SlotPlant(lib, fs=50.0, fk=50.0)
    start_alt(lib, inject="break", plant=plant, max_pct=95.0)
    record = []
    run_alt(lib, plant, 1100, mutate=_yaw_snap(1.6, 1000, 5), record=record)
    avg = 0.0
    for _phase, pulse, _z in record[:1000]:
        thrust = thrust_of(lib, pulse)
        avg = avg + (0.002 / 0.5) * (thrust - avg) if avg > 0.0 else thrust
    before = thrust_of(lib, record[999][1])
    first = thrust_of(lib, record[1000][1])
    assert record[1000][0] == PHASE_RAMP_DOWN
    assert avg < before - 0.15                            # 前提：平均确实落后于当前
    assert first == pytest.approx(avg, abs=0.05)
    teardown_run(lib)


def test_alt_attitude_hold_saturation_keeps_running(lib):
    """ALT 的姿态保持只护着机体：舵机按机构极限夹紧后照样下发，不当拍中止（半空断电会摔）。"""
    plant = SlotPlant(lib, fs=50.0, fk=50.0)
    start_alt(lib, inject="break", plant=plant, max_pct=95.0)
    for index in range(1100):
        obs = alt_obs(1000 + 2 * index, 1_000_000 + CONTROL_DT_US * index, plant,
                      roll_rad=0.6 if index >= 900 else 0.0, pitch_rad=0.6 if index >= 900 else 0.0)
        lib.APP_SysId_Update(ctypes.byref(obs))
        plant.advance(motor(lib))
        lib.APP_SysId_StreamTick()
    assert lib.APP_SysId_IsRunning() == 1, lib.APP_SysId_GetLastReason()
    teardown_run(lib)


def test_alt_residual_before_the_thrust_is_up_still_stops_at_once(lib):
    plant = SlotPlant(lib, fs=50.0, fk=50.0)
    start_alt(lib, inject="break", plant=plant, max_pct=95.0)
    assert run_alt(lib, plant, 100, mutate=_yaw_snap(1.6, 20, 5)) == 20
    assert lib.APP_SysId_GetLastReason() == b"axis_residual"
    assert motor(lib) is None


def test_alt_stop_in_ramp_down_is_aborted_not_complete(lib):
    plant = SlotPlant(lib, fs=0.05, fk=0.03)
    tof = Tof(plant)

    def land_and_stop(index, obs):
        tof(index, obs)
        if lib.APP_SysId_GetPhase() == PHASE_RAMP_DOWN:
            lib.APP_SysId_Stop(b"command")

    start_alt(lib, inject="break", plant=plant, max_pct=95.0)
    record = []
    run_alt(lib, plant, 12_000, mutate=land_and_stop, record=record)
    assert any(phase == PHASE_RAMP_DOWN for phase, _pulse, _z in record)
    assert lib.APP_SysId_GetState() == STATE_ABORTED
    assert lib.APP_SysId_GetLastReason() == b"command"


def test_landing_must_be_confirmed_at_the_end_of_the_ramp_down(lib, damped):
    """回落走完时平均高度还在槽底 20 mm 以上（比如卡在半路）：landing_unconfirmed。"""
    plant = start_alt(lib)

    def stuck(index, obs):
        if lib.APP_SysId_GetPhase() == PHASE_RAMP_DOWN:
            obs.height_m = H0 + 0.030

    run_alt(lib, plant, 8000, mutate=stuck)
    assert lib.APP_SysId_GetState() == STATE_ABORTED
    assert lib.APP_SysId_GetLastReason() == b"landing_unconfirmed"


# ---------------------------------------------------------------- vel/pos：序列与溯源


def test_full_alt_run_follows_the_contracted_sequence_and_samples_throughout(lib, damped):
    schema = schema_of(lib)
    plant = start_alt(lib, rate_hz=50)
    start_lines = texts(lib)
    lib.harness_reset()
    record, capture = [], Capture(lib)
    stopped = run_alt(lib, plant, 6000, record=record, capture=capture)
    assert stopped < 6000
    assert lib.APP_SysId_GetState() == STATE_DONE
    assert lib.APP_SysId_GetLastReason() == b"complete"
    assert motor(lib) is None, "结束后必须交还遥控器油门"

    runs = phase_runs(record)
    assert [phase for phase, _count in runs[:-1]] == ALT_SEQUENCE
    assert runs[-1][0] == PHASE_IDLE and runs[-1][1] == 1
    ticks = dict(runs[:-1])
    assert ticks[PHASE_RAMP_UP] == RAMP_TICKS
    assert ticks[PHASE_CLIMB] == pytest.approx(CLIMB_TICKS, abs=2)          # 60 mm @ 0.1 m/s
    assert ticks[PHASE_SETTLE] == SETTLE_TICKS
    assert ticks[PHASE_PREROLL] == PREROLL_TICKS
    assert ticks[PHASE_EXCITE] == pytest.approx(500 / 2, abs=1)
    assert ticks[PHASE_DESCEND] == pytest.approx(DESCEND_TICKS, abs=4)  # vel 前馈留下微小参考位移
    assert ticks[PHASE_RAMP_DOWN] == 1000 // 2

    # 生产高度环 + 质点：稳定段末尾已在 z_hold 附近，回落后落回槽底。
    preroll_z = [record[i][2] for i in ticks_in(record, PHASE_PREROLL)]
    assert max(abs(z - HOLD) for z in preroll_z) < 0.003
    assert record[-1][2] == pytest.approx(H0, abs=0.002)

    # 溯源：start 行之后紧跟 ALTSTART。
    starts = [i for i, line in enumerate(start_lines) if line.startswith("SYSID start ")]
    assert len(starts) == 1 and " auto=1 " in start_lines[starts[0]]
    run = re.search(r"run=(\d+)", start_lines[starts[0]]).group(1)
    assert start_lines[starts[0] + 1] == (
        f"SYSID ALTSTART run={run} alt_inject=vel alt_mass_g=755 alt_win_mm={SCENE_WIN_MM} "
        f"alt_lift_mm={SCENE_LIFT_MM} alt_h0_mm=80 alt_control=closed_loop alt_target_cn=800 "
        f"alt_bottom_mm={BOTTOM_MM} alt_top_mm={TOP_MM}\r\n")
    phases = [re.search(r"phase=(\w+)", line).group(1) for line in capture.texts
              if line.startswith("SYSID PHASE ")]
    assert phases == ["ramp_up", "climb", "settle", "preroll", "excite", "descend", "ramp_down"]
    assert any(line.startswith(f"SYSID end run={run} state=done reason=complete ")
               for line in capture.texts)

    # 全程采样：从起升第一拍到回落结束，50 Hz 网格一条不少、没有断点。
    rows, flags = capture.samples(schema)
    assert flags & FLAG_ALT and not flags & FLAG_THRUST_CAPPED
    assert len(rows) == pytest.approx(stopped / 10 + 1, abs=1)
    assert rows[0]["height"] == pytest.approx(H0, abs=1e-4)
    assert rows[0]["height_raw"] == pytest.approx(H0 + RAW_OFFSET, abs=1e-4)
    assert rows[0]["height_sp"] == pytest.approx(H0, abs=1e-4)
    assert rows[0]["vbat"] == pytest.approx(11.876, abs=1e-3)
    assert max(row["height_sp"] for row in rows) == pytest.approx(HOLD + 0.05 * 0.25, abs=5e-4)
    assert rows[-1]["height_sp"] == pytest.approx(H0 + 0.010, abs=1e-4)
    assert all(row["angle_sp"] == 0.0 for row in rows)   # 姿态目标恒为杆轴角 0
    # thrust 字段是实际下发脉宽读回的推力：稳定段 ≈ m·g。
    settle_rows = rows[(RAMP_TICKS + CLIMB_TICKS + SETTLE_TICKS) // 10 - 5:
                       (RAMP_TICKS + CLIMB_TICKS + SETTLE_TICKS) // 10]
    assert all(row["thrust"] == pytest.approx(WEIGHT_N, abs=0.05) for row in settle_rows)


def test_record_carries_the_observed_height_fields(lib):
    """ALT 记录的 height/height_raw/vz/az/vbat 就是稳定环填进来的观测，height_sp/vz_sp 是参考。"""
    schema = schema_of(lib)
    plant = start_alt(lib, rate_hz=500)
    lib.harness_reset()

    def mutate(index, obs):
        obs.height_m, obs.height_raw_m = 0.0812, 0.0853
        obs.vz_m_s, obs.az_m_s2, obs.vbat_v = -0.012, 0.345, 12.3

    capture = Capture(lib)
    run_alt(lib, plant, 20, mutate=mutate)
    lib.APP_SysId_Stop(b"test")
    for _ in range(8):
        lib.APP_SysId_StreamTick()
    capture.flush()
    rows, _flags = capture.samples(schema)
    row = rows[5]
    assert row["height"] == pytest.approx(0.0812, abs=1e-4)
    assert row["height_raw"] == pytest.approx(0.0853, abs=1e-4)
    assert row["vz"] == pytest.approx(-0.012, abs=1e-3)
    assert row["az"] == pytest.approx(0.345, abs=1e-3)
    assert row["vbat"] == pytest.approx(12.3, abs=1e-3)
    assert row["height_sp"] == pytest.approx(H0, abs=1e-4)       # 起升段参考 = 开跑高度
    assert row["vz_sp"] == 0.0


def test_non_alt_runs_record_zero_height_fields(lib):
    """所有模式都带这 7 个字段，非 ALT 轮一律 0——即便稳定环照样填了测高。"""
    schema = schema_of(lib)
    reset(lib, rate_hz=500)
    assert lib.APP_SysId_SetMode(MODE_FF, ctypes.c_float(0.0523598776)) == 1
    assert lib.APP_SysId_Start() == 1
    lib.harness_reset()
    for index in range(40):
        obs = healthy(1000 + index * 2, 1_000_000 + index * CONTROL_DT_US, height_valid=1,
                      height_m=0.2, height_raw_m=0.21, vz_m_s=0.1, az_m_s2=1.0, vbat_v=12.0)
        lib.APP_SysId_Update(ctypes.byref(obs))
    lib.APP_SysId_StreamTick()
    capture = Capture(lib)
    capture.flush()
    rows, flags = capture.samples(schema)
    teardown_run(lib)
    assert rows and not flags & FLAG_ALT
    assert all(row[name] == 0.0 for row in rows for name in ALT_FIELDS)


def test_thr_line_reports_live_height_and_alt_reference(lib):
    plant = start_alt(lib)
    run_alt(lib, plant, RAMP_TICKS + 50)                 # 进入爬升段
    lib.harness_reset()
    lib.APP_SysId_ReportThrottle()
    line = texts(lib)[0]
    assert " phase=climb " in line
    values = dict(item.split("=") for item in line.split()[2:])
    assert int(values["alt_h_ok"]) == 1
    assert int(values["alt_h_mm"]) == pytest.approx(plant.z * 1000, abs=2)   # 8 样本平均
    assert int(values["alt_sp_mm"]) == pytest.approx((H0 + 0.1 * 0.1) * 1000, abs=1)
    teardown_run(lib)
    lib.APP_SysId_ReportThrottle()
    assert " alt_sp_mm=0 xy_pos_mm=" in texts(lib)[0]   # 不在跑就不报参考


# ---------------------------------------------------------------- m_run


@pytest.mark.parametrize("mass_g,expect_kg", [(0, AIRFRAME_KG), (1184, 1.184)])
def test_run_mass_comes_from_mass_g_or_the_airframe(lib, mass_g, expect_kg):
    plant = start_alt(lib, mass_g=mass_g, plant=Plant(lib, mass_kg=expect_kg))
    assert any(line.startswith("SYSID ALTSTART ") and
               f" alt_mass_g={round(expect_kg * 1000)} " in line for line in texts(lib))
    record = []
    run_alt(lib, plant, RAMP_TICKS + 1, record=record)
    teardown_run(lib)
    # 起升是脉宽线性插值：最后一拍在 1498 ms，终点 = 封顶后的 pulse(0.9·m_run·g)。
    ramp_end = [pulse for phase, pulse, _z in record if phase == PHASE_RAMP_UP][-1]
    target = capped_pulse(lib, 0.9 * expect_kg * G)
    assert ramp_end == IDLE_US + ((target - IDLE_US) * 1498) // 1500
    # 高度环接管第一拍：F = m_run·(g + a_z)，a_z = vel_kp·(0.1 + pos_kp·Δz_ref)（积分首拍复位）。
    first_climb = [pulse for phase, pulse, _z in record if phase == PHASE_CLIMB][0]
    a_z = 1.0 * (0.1 + 3.8 * 0.1 * 0.002)
    assert abs(first_climb - capped_pulse(lib, expect_kg * (G + a_z))) <= 1


# ---------------------------------------------------------------- 生产高度环与生产参数


def first_climb_pulse(lib):
    plant = start_alt(lib)
    record = []
    run_alt(lib, plant, RAMP_TICKS + 1, record=record)
    teardown_run(lib)
    return record[-1][1]


def test_the_height_loop_uses_the_production_velocity_gain(lib, coax_params):
    base = first_climb_pulse(lib)
    assert coax_params("coax.vel_z_kp", 3.0) == 1
    stiff = first_climb_pulse(lib)
    a_base = 1.0 * (0.1 + 3.8 * 0.0002)
    a_stiff = 3.0 * (0.1 + 3.8 * 0.0002)
    assert abs(base - capped_pulse(lib, AIRFRAME_KG * (G + a_base))) <= 1
    assert abs(stiff - capped_pulse(lib, AIRFRAME_KG * (G + a_stiff))) <= 1
    assert stiff > base + 5


def test_the_height_loop_uses_the_production_hover_thrust(lib, coax_params):
    """coax.hover_thrust_n > 0：合推力 = hover/g·(g + a)，与生产同一换算（台架实测 14.25 N）。"""
    assert coax_params("coax.hover_thrust_n", 9.0) == 1
    pulse = first_climb_pulse(lib)
    a = 1.0 * (0.1 + 3.8 * 0.0002)
    assert abs(pulse - capped_pulse(lib, 9.0 / G * (G + a))) <= 1
    assert abs(pulse - capped_pulse(lib, AIRFRAME_KG * (G + a))) > 5


def test_the_height_loop_uses_the_production_position_gain(lib, coax_params):
    """位置 P 的输出就是记录里的 vz_sp：让稳定段测高低 20 mm，vz_sp = pos_z_kp × 0.02。"""
    schema = schema_of(lib)
    trip = RAMP_TICKS + CLIMB_TICKS + 100

    def vz_sp_after_sag():
        plant = start_alt(lib, rate_hz=500)
        lib.harness_reset()

        def sag(index, obs):
            if index >= trip:
                obs.height_m, obs.vz_m_s = HOLD - 0.020, 0.0

        capture = Capture(lib)
        run_alt(lib, plant, trip + 30, mutate=sag, capture=capture)
        teardown_run(lib)
        rows, _flags = capture.samples(schema)
        return rows[trip + 25]["vz_sp"]

    assert vz_sp_after_sag() == pytest.approx(3.8 * 0.020, abs=2e-3)
    assert coax_params("coax.pos_z_kp", 2.0) == 1
    assert vz_sp_after_sag() == pytest.approx(2.0 * 0.020, abs=2e-3)


def test_the_height_loop_integrator_is_the_production_one(lib, coax_params):
    """稳定段把测高钉在 z_hold − 50 mm：纯 P 推力不变；运行中经参数表开 coax.vel_z_ki，
    下一次速度环更新起积分按 ki·(pos_kp·0.05) 的斜率往上爬——生产积分器、参数每拍现取。"""
    trip = RAMP_TICKS + CLIMB_TICKS + 100

    def thrust_after_trip(ki):
        plant = start_alt(lib)

        def mutate(index, obs):
            if index == trip and ki:
                assert coax_params("coax.vel_z_ki", ki) == 1
            if index >= trip:
                obs.height_m, obs.vz_m_s, obs.az_m_s2 = HOLD - 0.050, 0.0, 0.0

        record = []
        run_alt(lib, plant, trip + 230, mutate=mutate, record=record)
        teardown_run(lib)
        return [thrust_of(lib, pulse) for _phase, pulse, _z in record[trip + 20:]]

    flat = thrust_after_trip(0.0)
    ramp = thrust_after_trip(5.0)
    assert max(flat) - min(flat) < 0.03
    expected = AIRFRAME_KG * 5.0 * (3.8 * 0.050) * (len(ramp) - 1) * 0.002
    assert ramp[-1] - ramp[0] == pytest.approx(expected, rel=0.2)


def test_the_source_reuses_the_production_loop_and_does_not_write_its_own():
    code = strip_c_comments((ROOT / "App/Src/app_sysid_alt.c").read_text(encoding="utf-8"))
    assert "DRV_COAX_CTRL_GetParams(&params);" in code
    assert "DRV_POSITION_CONTROL_PositionStep(&params.position," in code
    assert "DRV_POSITION_CONTROL_VelocityStep(&params.position, &alt.loop," in code
    for private_gain in ("vel_ki", "vel_kp", "pos_kp", "vel_integrator", "kp *", "ki *"):
        assert private_gain not in code, f"ALT 里不许另写一套高度环：{private_gain}"
    for forbidden in ("DRV_Motor", "BSP_PWM_SetEscPulse", "BSP_PWM_SetEscPercent", "HAL_"):
        assert forbidden not in code
    cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8").replace("\r\n", "\n")
    assert "    App/Src/app_sysid_alt.c\n" in cmake


# ---------------------------------------------------------------- vel/pos 注入


def run_injection(lib, *, spec, inject, win_mm=SCENE_WIN_MM):
    schema = schema_of(lib)
    plant = start_alt(lib, spec=spec, inject=inject, rate_hz=500, win_mm=win_mm)
    lib.harness_reset()
    record, capture = [], Capture(lib)
    run_alt(lib, plant, 8000, record=record, capture=capture)
    rows, _flags = capture.samples(schema)
    assert lib.APP_SysId_GetState() == STATE_DONE, lib.APP_SysId_GetLastReason()
    assert len(rows) == len(record)          # 500 Hz：每拍一条，记录下标 = 控制拍下标
    return record, rows, ticks_in(record, PHASE_EXCITE)


def test_velocity_injection_is_a_feedforward_and_integrates_into_the_reference(lib, damped):
    amp, hold_ms, ramp_ms = 0.05, 600, 100
    _record, rows, excite = run_injection(lib, spec=step_spec(amp, hold_ms, ramp_ms),
                                          inject="vel")
    # 平台中段：参考以 amp 的速度匀速移动，速度参考里带着 amp 的前馈。
    mid = excite[0] + (ramp_ms + hold_ms // 2) // 2
    slope = (rows[mid + 25]["height_sp"] - rows[mid - 25]["height_sp"]) / (50 * 0.002)
    assert slope == pytest.approx(amp, rel=0.02)
    # vz_sp = v_ff + pos_kp·(z_sp − z)：扣掉位置 P 那一份剩下的就是 amp 的前馈
    # （生产增益在无摩擦质点上收敛要 ~2 s，平台中段机体还落后几毫米）。
    row = rows[mid]
    assert row["vz_sp"] - 3.8 * (row["height_sp"] - row["height"]) == pytest.approx(amp, abs=0.006)
    # 激励结束时参考停在 z_hold + ∫v_inj dt = z_hold + amp·(hold + ramp)。
    assert rows[excite[-1]]["height_sp"] == pytest.approx(
        HOLD + amp * (hold_ms + ramp_ms) / 1000, abs=5e-4)
    # 回落从那里开始，照样回到槽底 + 10 mm。
    assert rows[-1]["height_sp"] == pytest.approx(H0 + 0.010, abs=1e-4)


def test_position_injection_steps_the_reference_without_feedforward(lib, damped):
    amp, hold_ms, ramp_ms = 0.05, 600, 20
    record, rows, excite = run_injection(lib, spec=step_spec(amp, hold_ms, ramp_ms),
                                         inject="pos")
    assert rows[excite[0] + ramp_ms // 2 + 5]["height_sp"] == pytest.approx(HOLD + amp, abs=1e-4)
    # 没有速度前馈：阶跃刚到时速度参考就是位置 P 的 pos_kp × 误差（机体还没动）。
    early = max(rows[i]["vz_sp"] for i in excite[:40])
    assert early == pytest.approx(3.8 * amp, abs=0.02)
    # 最后一拍激励在回零斜坡的最后 2 ms 上：shape = 0.1。
    assert rows[excite[-1]]["height_sp"] == pytest.approx(HOLD + 0.1 * amp, abs=1e-4)
    # 机体真的跟上了阶跃（生产环 + 质点）。
    peak = max(record[i][2] for i in excite)
    assert HOLD + 0.8 * amp < peak < HOLD + 1.3 * amp


# ---------------------------------------------------------------- 安全门


def test_height_invalid_for_more_than_100_ms_aborts(lib):
    start = RAMP_TICKS + CLIMB_TICKS + 100      # 稳定段里

    def dropout(ticks):
        def mutate(index, obs):
            if start <= index < start + ticks:
                obs.height_valid = 0
                obs.height_m = float("nan")
        return mutate

    plant = start_alt(lib)
    assert run_alt(lib, plant, start + 200, mutate=dropout(50)) == start + 200   # 整 100 ms：扛住
    teardown_run(lib)
    plant = start_alt(lib)
    record = []
    # 最后有效在 start−1；无效超过 100 ms 的第一拍（start+50）转软着陆。看不到高度：按时间 2 s
    # 线性降到怠速，走完才报中止、交还油门（测高中途恢复也不再改主意）。
    assert run_alt(lib, plant, start + 1200, mutate=dropout(200), record=record) == start + 1050
    assert first_tick_in(record, PHASE_RAMP_DOWN) == start + 50
    assert max_tick_drop(lib, record, start + 50) < 0.1
    assert lib.APP_SysId_GetState() == STATE_ABORTED
    assert lib.APP_SysId_GetLastReason() == b"height_invalid"
    assert motor(lib) is None


@pytest.mark.parametrize("height", [UPPER + 0.0011, H0 - 0.0311], ids=["above", "below"])
def test_leaving_the_height_window_aborts(lib, height):
    """硬窗按 8 样本平均判：持续越线，平均值一越过就转软着陆（最多晚 8 个样本）。高过上沿：按高度
    慢降（这里测距被钉住看不到下降，就一路慢降到怠速）；低过槽底：已在槽底，1 s 降到怠速。"""
    plant = start_alt(lib)
    trip = RAMP_TICKS + CLIMB_TICKS + 50

    def mutate(index, obs):
        if index >= trip:
            obs.height_m = height

    record = []
    stopped = run_alt(lib, plant, trip + 6000, mutate=mutate, record=record)
    entry = first_tick_in(record, PHASE_RAMP_DOWN)
    assert trip <= entry <= trip + 8
    if height < H0:
        assert entry + 500 <= stopped <= entry + 508   # 高度跳变的斜率回稳（8 个样本）后 1 s 降到怠速
    else:
        assert entry + 1500 < stopped < entry + 4500
    assert max_tick_drop(lib, record, entry) < 0.1
    assert lib.APP_SysId_GetLastReason() == b"height_window"
    assert lib.APP_SysId_GetState() == STATE_ABORTED
    assert motor(lib) is None


def test_a_single_noisy_height_sample_does_not_trip_the_window(lib):
    """一个越线 30 mm 的尖刺被 8 样本平均摊掉：不中止。"""
    plant = start_alt(lib)
    base = RAMP_TICKS + CLIMB_TICKS

    def spike(index, obs):
        if index == base + 20:
            obs.height_m = H0 - 0.060

    assert run_alt(lib, plant, base + 60, mutate=spike) == base + 60
    teardown_run(lib)


def test_the_window_edges_themselves_do_not_trip(lib):
    plant = start_alt(lib)
    base = RAMP_TICKS + CLIMB_TICKS

    def mutate(index, obs):
        if base <= index < base + 100:
            obs.height_m = UPPER - 0.0005
        elif base + 100 <= index < base + 200:
            obs.height_m = H0 - 0.0295

    assert run_alt(lib, plant, base + 210, mutate=mutate) == base + 210
    teardown_run(lib)


def test_rc_gates_still_apply_first_and_hand_back_the_throttle(lib):
    plant = start_alt(lib)

    def mutate(index, obs):
        if index >= 900:
            obs.rc_throttle_low = 0
            obs.height_m = 5.0            # 同一拍也出窗：先报遥控这个根本原因

    assert run_alt(lib, plant, 1000, mutate=mutate) == 900
    assert lib.APP_SysId_GetLastReason() == b"rc_throttle_override"
    assert motor(lib) is None


def test_capping_is_recorded_as_the_thrust_actually_sent(lib):
    schema = schema_of(lib)
    plant = start_alt(lib, max_pct=40.0, rate_hz=50)
    lib.harness_reset()
    record, capture = [], Capture(lib)
    run_alt(lib, plant, RAMP_TICKS + 300, record=record)
    lib.APP_SysId_ReportThrottle()
    thr = texts(lib)[-1]
    lib.APP_SysId_Stop(b"test")
    for _ in range(64):
        lib.APP_SysId_StreamTick()
    capture.flush()
    cap = int(1100 + 40 * 840 / 100 + 0.5)
    climb = [pulse for phase, pulse, _z in record if phase == PHASE_CLIMB]
    assert climb and all(pulse == cap for pulse in climb)
    assert thr.startswith("SYSID THR ") and " capped=1 " in thr
    rows, flags = capture.samples(schema)
    assert flags & FLAG_THRUST_CAPPED
    assert rows[-2]["thrust"] == pytest.approx(thrust_of(lib, cap), abs=0.01)
    assert rows[-2]["thrust"] < WEIGHT_N       # 封顶后实际推力，不是高度环要的


# ---------------------------------------------------------------- 竖直加速度


@pytest.mark.parametrize("accel,roll,pitch,expected", [
    ((0.0, 0.0, 1.0), 0.0, 0.0, 0.0),                               # 静止水平
    ((0.0, 0.0, 0.0), 0.0, 0.0, -G),                                # 自由落体
    ((0.0, 0.0, 1.0 + 1.0 / G), 0.0, 0.0, 1.0),                     # 向上 1 m/s²
    ((-math.sin(0.2), 0.0, math.cos(0.2)), 0.0, 0.2, 0.0),          # 机头下俯静止
    ((0.0, math.sin(0.3), math.cos(0.3)), 0.3, 0.0, 0.0),           # 右翼下沉静止
    ((-math.sin(0.2), math.cos(0.2) * math.sin(-0.3), math.cos(0.2) * math.cos(-0.3)),
     -0.3, 0.2, 0.0),                                               # 横滚+俯仰静止
])
def test_vertical_accel_rotates_the_specific_force_to_up_and_removes_g(lib, accel, roll, pitch,
                                                                       expected):
    got = lib.APP_SysIdAlt_VerticalAccel(*[ctypes.c_float(v) for v in (*accel, roll, pitch, G)])
    assert got == pytest.approx(expected, abs=1e-5)


def test_the_stabilizer_fills_the_alt_observation_read_only():
    source = strip_c_comments((ROOT / "App/Src/app_stabilizer.c").read_text(encoding="utf-8"))
    block = source[source.index("SVC_FLOW_NAV_State sysid_nav;"):
                   source.index("APP_SysId_Update(&sysid_obs);")]
    for field in (".height_valid = sysid_height_ok,", ".height_m = sysid_height_m,",
                  ".height_raw_m = sysid_nav.height_raw_m,", ".vz_m_s = sysid_vz_m_s,",
                  ".az_m_s2 = APP_SysIdAlt_VerticalAccel(", ".vbat_v = "):
        assert field in block, field
    assert ("APP_OpticalFlow_GetHeightSample(&sysid_height_m, &sysid_vz_m_s, &sysid_height_ms)"
            in block)
    # 只读：这一段不给稳定环自己的状态赋值（生产高度原点、垂直加速度估计都不碰）。
    assert not re.search(r"ctx->\w+(\[[^\]]*\])?\s*=[^=]", block)


def _worst_line(source: str, anchor: str, worst: dict) -> str:
    call = source[source.index(anchor):]
    call = call[:call.index(");")]
    fmt = "".join(re.findall(r'"((?:[^"\\]|\\.)*)"', call)).replace("\\r\\n", "\r\n")
    keys = re.findall(r"(\w+)=%l?[uds]", fmt)
    assert set(keys) <= set(worst), set(keys) - set(worst)
    return fmt % tuple(worst[k] for k in keys)


def test_the_new_text_lines_fit_the_text_buffer_at_their_widest():
    """ALTSTART 另起一行（start 行最宽已约 246 字符）；THR 行尾追加了三项——都要 ≤ APP_UART_TX_TEXT_SIZE − 1。"""
    budget = int(re.search(r"#define\s+APP_UART_TX_TEXT_SIZE\s+(\d+)",
                           (ROOT / "App/Inc/app_messages.h").read_text(encoding="utf-8")).group(1)) - 1
    alt_source = (ROOT / "App/Src/app_sysid_alt.c").read_text(encoding="utf-8")
    altstart = _worst_line(alt_source, '"SYSID ALTSTART run=', dict(
        run=65535, alt_inject="break", alt_mass_g=3000, alt_win_mm=400, alt_lift_mm=300,
        alt_h0_mm=-2147483648, alt_control="closed_loop", alt_target_cn=1000000,
        alt_bottom_mm=4000, alt_top_mm=4000))
    echo = _worst_line(alt_source, '"SYSID ALT inject=', dict(
        inject="break", mass_g=3000, win_mm=400, lift_mm=300, bottom_mm=4000, top_mm=4000,
        control="closed_loop"))
    sysid_source = (ROOT / "App/Src/app_sysid.c").read_text(encoding="utf-8")
    thr = _worst_line(sysid_source, '"SYSID THR auto=', {
        "auto": 1, "target_cn": 1_000_000, "max_pct_x10": 950, "phase": "ramp_down",
        "pulse_us": 65535, "armed": 1, "thr_low": 1, "capped": 1,
        "alt_h_mm": -2147483648, "alt_h_ok": 1, "alt_sp_mm": -2147483648,
        "xy_pos_mm": -2147483648, "xy_vel_mms": -2147483648, "xy_ok": 1,
        "xy_yaw_mrad": -2147483648, "vbat_mv": 65535})
    for line in (altstart, echo, thr):
        assert len(line) <= budget, (len(line), line)
    # 行尾另追加了水平槽 XY 的三项（tests/test_sysid_xy.py 钉它们），ALT 三项保持原样在前。
    assert " alt_h_mm=-2147483648 alt_h_ok=1 alt_sp_mm=-2147483648 xy_pos_mm=" in thr

"""光杆台架高度辨识（SYSID MODE ALT = 4，R-ALTID-1）—— 在宿主上跑真固件。

App/Src/app_sysid.c + app_sysid_alt.c + app_cmd_sysid.c + 真分配器（推力表、反解器）+
真 drv_position_control（生产高度环），平台边界见 tests/fixtures/sysid/。机体的上下运动
由这里一个一维质点给：F = 推力表读回的合推力，压在槽底时不往下掉，没有摩擦。

判据按台架上真会出事的地方排：

* **命令面**：`SYSID ALT` 回显一行、越界/格式/运行中拒绝（整条不生效）、MODE ALT|4；
* **序列**：起升 → 爬升 → 稳定 → 前导 → 激励 → 下降 → 回落，时长与契约一致，全程采样；
* **注入位置**：force 加在高度环之后（高度参考不动）、vel 作速度前馈并积分成高度参考、
  pos 直接改高度参考（没有前馈）；
* **高度环是生产代码 + 生产参数**：coax.vel_z_kp / coax.pos_z_kp / coax.vel_z_ki 改了行为跟着变，
  而且下一拍就生效（SYSID PARAM 试用走的就是它）；
* **m_run** 取 mass_g 或机体质量；
* **安全门**：测高无效 > 100 ms、出窗中止；封顶照记（记录推力 = 封顶后实际推力 + 批头标志）；
  遥控器那道门照旧、先报、交还油门；
* **记录**：7 个高度字段在 ALT 下是观测/参考、非 ALT 为 0；**溯源**：SYSID ALTSTART 紧跟 start 行。

生产默认 vel_z_kp = 1、pos_z_kp = 3.8 放在无摩擦质点上阻尼只有约 0.26（实机有风阻和槽的
摩擦），要看"跟得上"的用例临时把 vel_z_kp 调到 4（阻尼约 0.5）——同样是生产代码按参数表跑。
"""
from __future__ import annotations

import ctypes
import math
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
FLAG_ALT, FLAG_THRUST_CAPPED = 0x0100, 0x0200
IDLE_US = 1100
G = 9.81
AIRFRAME_KG = 0.7546            # default_airframe()：75 + 232 + 99 + 348.6 g
H0 = 0.080                      # 槽底时 TOF 读数
# 场景参数：lift 100 mm / win 150 mm，给无摩擦质点的超调留足余量。固件上电默认是 60 / 60
# （作者槽式台架行程 160 mm），由 test_power_on_defaults_match_the_slot_rig 单独钉住。
SCENE_LIFT_MM, SCENE_WIN_MM = 100, 150
HOLD = H0 + SCENE_LIFT_MM / 1000.0
RAW_OFFSET = 0.003              # 原始测距比滤波高度多 3 mm，区分两个字段
TARGET_N = 7.4                  # ALT 下只作"开启自动油门"的标志
ALT_FIELDS = ("height", "height_raw", "height_sp", "vz", "vz_sp", "az", "vbat")
RAMP_TICKS, CLIMB_TICKS, SETTLE_TICKS, PREROLL_TICKS = 750, 500, 1000, 250
AMP_TEXT = {"force": "3 N", "vel": "0.3 m/s", "pos": "0.15 m"}

FORCE_DOUBLET = dict(profile=PROFILE_DOUBLET, amplitude_rad_s=0.6, duration_ms=1200,
                     hold_ms=300, repeat=2, ramp_ms=20, chirp_f0_hz=0.3, chirp_f1_hz=6.0,
                     prbs_bit_ms=40, prbs_seed=1)


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
             "coax.vel_z_i_limit_m_s2")
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


def alt_obs(t_ms, t_us, plant, **overrides):
    values = dict(throttle_us=IDLE_US, rc_throttle_low=1, height_valid=1, height_m=plant.z,
                  height_raw_m=plant.z + RAW_OFFSET, vz_m_s=plant.vz, az_m_s2=plant.az,
                  vbat_v=11.876)
    values.update(overrides)
    return healthy(t_ms, t_us, **values)


def configure_alt(lib, *, inject="force", mass_g=0, win_mm=SCENE_WIN_MM, lift_mm=SCENE_LIFT_MM):
    line = f"SYSID ALT inject={inject} mass_g={mass_g} win_mm={win_mm} lift_mm={lift_mm}"
    assert command(lib, line) == [f"{line}\r\n"]


def prepare_alt(lib, *, spec=None, inject="force", mass_g=0, win_mm=SCENE_WIN_MM,
                lift_mm=SCENE_LIFT_MM,
                max_pct=75.0, target_n=TARGET_N, rate_hz=50, plant=None, **obs):
    reset(lib, rate_hz=rate_hz, spec=Excitation(**(spec or FORCE_DOUBLET)))
    if target_n:
        assert lib.APP_SysId_SetThrottle(ctypes.c_float(target_n), ctypes.c_float(max_pct)) == 1
    assert lib.APP_SysId_SetMode(MODE_ALT, ctypes.c_float(0.0523598776)) == 1
    configure_alt(lib, inject=inject, mass_g=mass_g, win_mm=win_mm, lift_mm=lift_mm)
    plant = plant or Plant(lib)
    # 没跑时控制拍也调 Update：START 靠它知道解锁/油门杆/测高。
    lib.APP_SysId_Update(ctypes.byref(alt_obs(900, 900_000, plant, **obs)))
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


def teardown_run(lib):
    lib.APP_SysId_Stop(b"test")
    for _ in range(64):
        lib.APP_SysId_StreamTick()
    lib.harness_reset()


# ---------------------------------------------------------------- 命令面


def test_power_on_defaults_match_the_slot_rig(fresh_lib):
    """上电默认 force / 0 / 60 / 60：作者槽式台架行程 160 mm，槽底起抬升 60 mm 悬停，
    超过 120 mm（z_hold + 60）中止，离槽顶留 40 mm。不发 SYSID ALT 直接开跑就按它走。"""
    header = (ROOT / "App/Inc/app_sysid_alt.h").read_text(encoding="utf-8")
    assert re.search(r"#define APP_SYSID_ALT_WIN_MM_DEFAULT\s+60U", header)
    assert re.search(r"#define APP_SYSID_ALT_LIFT_MM_DEFAULT\s+60U", header)
    lib = fresh_lib
    assert command(lib, "SYSID ALT") == ["SYSID ALT inject=force mass_g=0 win_mm=60 lift_mm=60\r\n"]

    reset(lib, rate_hz=50, spec=Excitation(**FORCE_DOUBLET))
    assert lib.APP_SysId_SetThrottle(ctypes.c_float(TARGET_N), ctypes.c_float(75.0)) == 1
    assert lib.APP_SysId_SetMode(MODE_ALT, ctypes.c_float(0.0523598776)) == 1
    plant = Plant(lib)
    lib.APP_SysId_Update(ctypes.byref(alt_obs(900, 900_000, plant)))
    lib.harness_reset()
    assert lib.APP_SysId_Start() == 1
    assert any(line.endswith(" alt_win_mm=60 alt_lift_mm=60 alt_h0_mm=80\r\n")
               for line in texts(lib))
    record = []
    climb_end = RAMP_TICKS + 300 + 50                  # 60 mm @ 0.1 m/s = 300 拍，再进稳定段

    def peek(index, obs):
        if index == climb_end:
            obs.height_m = H0 + 0.060 + 0.0595         # 窗口上沿以内：不停
        elif index == climb_end + 1:
            obs.height_m = H0 + 0.060 + 0.0601         # 过了 120 mm：中止

    assert run_alt(lib, plant, climb_end + 10, mutate=peek, record=record) == climb_end + 1
    assert lib.APP_SysId_GetLastReason() == b"height_window"
    assert len(ticks_in(record, PHASE_CLIMB)) == pytest.approx(300, abs=2)


def test_alt_command_echoes_one_line(lib):
    lib.APP_SysId_Stop(b"test")
    configure_alt(lib)
    assert command(lib, "SYSID ALT") == [
        "SYSID ALT inject=force mass_g=0 win_mm=150 lift_mm=100\r\n"]
    assert command(lib, "SYSID ALT inject=vel mass_g=1184 win_mm=200 lift_mm=120") == [
        "SYSID ALT inject=vel mass_g=1184 win_mm=200 lift_mm=120\r\n"]
    # 只给一部分键：其余保持原值。
    assert command(lib, "SYSID ALT inject=pos") == [
        "SYSID ALT inject=pos mass_g=1184 win_mm=200 lift_mm=120\r\n"]
    assert command(lib, "SYSID ALT mass_g=0") == [
        "SYSID ALT inject=pos mass_g=0 win_mm=200 lift_mm=120\r\n"]
    configure_alt(lib)


@pytest.mark.parametrize("args,reason", [
    ("mass_g=499", "range"), ("mass_g=3001", "range"), ("mass_g=70000", "range"),
    ("win_mm=29", "range"), ("win_mm=401", "range"),
    ("lift_mm=29", "range"), ("lift_mm=301", "range"),
    ("inject=up", "usage"), ("mass_g=abc", "usage"), ("mass_g=1.5", "usage"),
    ("foo=1", "usage"), ("lift_mm=100 bar", "usage"),
])
def test_alt_command_rejects_whole_line_without_touching_the_config(lib, args, reason):
    lib.APP_SysId_Stop(b"test")
    configure_alt(lib, inject="vel", mass_g=1184, win_mm=200, lift_mm=120)
    assert command(lib, f"SYSID ALT inject=pos {args}") == [
        f"SYSID ALT event=rejected reason={reason}\r\n"]
    assert command(lib, "SYSID ALT") == [
        "SYSID ALT inject=vel mass_g=1184 win_mm=200 lift_mm=120\r\n"]
    configure_alt(lib)


@pytest.mark.parametrize("args", ["mass_g=500", "mass_g=3000", "win_mm=30", "win_mm=400",
                                  "lift_mm=30", "lift_mm=300"])
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
    assert command(lib, "SYSID ALT") == [
        "SYSID ALT inject=force mass_g=0 win_mm=150 lift_mm=100\r\n"]
    teardown_run(lib)


@pytest.mark.parametrize("token", ["ALT", "4"])
def test_mode_alt_by_name_and_number(lib, token):
    reset(lib)
    lines = command(lib, f"SYSID MODE {token}")
    assert f" mode={MODE_ALT} " in next(line for line in lines if line.startswith("SYSID READY "))
    assert lib.APP_SysId_GetMode() == MODE_ALT
    assert lib.APP_SysId_SetMode(MODE_ALT + 1, ctypes.c_float(0.05)) == 0


# ---------------------------------------------------------------- 开跑前检查


def test_start_needs_auto_throttle(lib):
    prepare_alt(lib, target_n=0)                      # reset 已把 target_n 置 0（手动）
    assert lib.APP_SysId_Start() == 0
    assert texts(lib) == ["ERR sysid alt needs auto throttle (SYSID THROTTLE target_n>0)\r\n"]


@pytest.mark.parametrize("inject,amp,accepted", [
    ("force", 3.0, True), ("force", 3.01, False),
    ("vel", 0.3, True), ("vel", 0.31, False),
    ("pos", 0.15, True), ("pos", 0.151, False),
])
def test_start_checks_the_amplitude_per_injection_type(lib, inject, amp, accepted):
    prepare_alt(lib, spec=dict(FORCE_DOUBLET, amplitude_rad_s=amp), inject=inject)
    assert lib.APP_SysId_Start() == int(accepted)
    if accepted:
        teardown_run(lib)
    else:
        assert texts(lib) == [
            f"ERR sysid alt amp over limit: inject={inject} amp<={AMP_TEXT[inject]}\r\n"]
    configure_alt(lib)


def test_start_needs_a_valid_height(lib):
    prepare_alt(lib, height_valid=0)
    assert lib.APP_SysId_Start() == 0
    assert texts(lib) == ["ERR sysid alt height invalid (TOF)\r\n"]


# ---------------------------------------------------------------- 序列与溯源


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
    assert ticks[PHASE_CLIMB] == pytest.approx(CLIMB_TICKS, abs=2)          # 100 mm @ 0.1 m/s
    assert ticks[PHASE_SETTLE] == SETTLE_TICKS
    assert ticks[PHASE_PREROLL] == PREROLL_TICKS
    assert ticks[PHASE_EXCITE] == pytest.approx(1200 / 2, abs=1)            # 双脉冲 2×2×300 ms
    assert ticks[PHASE_DESCEND] == pytest.approx(0.090 / 0.1 / 0.002, abs=2)  # 回到 h0 + 10 mm
    assert ticks[PHASE_RAMP_DOWN] == 1000 // 2

    # 生产高度环 + 质点：稳定段末尾已在 z_hold 附近，回落后落回槽底。
    preroll_z = [record[i][2] for i in ticks_in(record, PHASE_PREROLL)]
    assert max(abs(z - HOLD) for z in preroll_z) < 0.003
    assert record[-1][2] == pytest.approx(H0, abs=0.002)

    # 溯源：start 行之后紧跟 ALTSTART，alt_* 五项。
    starts = [i for i, line in enumerate(start_lines) if line.startswith("SYSID start ")]
    assert len(starts) == 1 and " auto=1 " in start_lines[starts[0]]
    run = re.search(r"run=(\d+)", start_lines[starts[0]]).group(1)
    assert start_lines[starts[0] + 1] == (
        f"SYSID ALTSTART run={run} alt_inject=force alt_mass_g=755 alt_win_mm=150 "
        f"alt_lift_mm=100 alt_h0_mm=80\r\n")
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
    assert max(row["height_sp"] for row in rows) == pytest.approx(HOLD, abs=1e-4)
    assert rows[-1]["height_sp"] == pytest.approx(H0 + 0.010, abs=1e-4)
    assert all(row["angle_sp"] == 0.0 for row in rows)   # 姿态目标恒为杆轴角 0
    # thrust 字段是实际下发脉宽读回的推力：稳定段 ≈ m·g。
    settle_rows = rows[(RAMP_TICKS + CLIMB_TICKS + SETTLE_TICKS) // 10 - 5:
                       (RAMP_TICKS + CLIMB_TICKS + SETTLE_TICKS) // 10]
    assert all(row["thrust"] == pytest.approx(AIRFRAME_KG * G, abs=0.05) for row in settle_rows)


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
    assert int(values["alt_h_mm"]) == pytest.approx(plant.z * 1000, abs=2)
    assert int(values["alt_sp_mm"]) == pytest.approx((H0 + 0.1 * 0.1) * 1000, abs=1)
    teardown_run(lib)
    lib.APP_SysId_ReportThrottle()
    assert texts(lib)[0].rstrip().endswith(" alt_sp_mm=0")   # 不在跑就不报参考


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


# ---------------------------------------------------------------- 三种注入


def run_injection(lib, *, spec, inject, win_mm=SCENE_WIN_MM):
    schema = schema_of(lib)
    plant = start_alt(lib, spec=spec, inject=inject, rate_hz=500, win_mm=win_mm)
    lib.harness_reset()
    record, capture = [], Capture(lib)
    run_alt(lib, plant, 8000, record=record, capture=capture)
    rows, _flags = capture.samples(schema)
    assert lib.APP_SysId_GetState() == STATE_DONE
    assert len(rows) == len(record)          # 500 Hz：每拍一条，记录下标 = 控制拍下标
    return record, rows, ticks_in(record, PHASE_EXCITE)


def test_force_injection_adds_after_the_loop_and_leaves_the_reference_alone(lib, damped):
    record, rows, excite = run_injection(lib, spec=step_spec(0.6, hold_ms=300, ramp_ms=20),
                                         inject="force")
    before = thrust_of(lib, record[excite[0] - 1][1])
    # 斜坡 20 ms 走完的那一拍：高度环还来不及反应（质点只动了零点几毫米），推力台阶 ≈ amp。
    assert thrust_of(lib, record[excite[0] + 10][1]) - before == pytest.approx(0.6, abs=0.08)
    assert rows[excite[0] + 10]["thrust"] - rows[excite[0] - 1]["thrust"] == \
        pytest.approx(0.6, abs=0.08)
    assert all(rows[i]["height_sp"] == pytest.approx(HOLD, abs=1e-4) for i in excite)
    assert all(abs(rows[i]["vz_sp"]) < 0.25 for i in excite)   # 没有速度前馈，只有位置 P


def test_velocity_injection_is_a_feedforward_and_integrates_into_the_reference(lib, damped):
    amp, hold_ms, ramp_ms = 0.05, 600, 100
    _record, rows, excite = run_injection(lib, spec=step_spec(amp, hold_ms, ramp_ms),
                                          inject="vel", win_mm=200)
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
    # 回落从那里开始，照样回到 h0 + 10 mm。
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
    # 最后有效在 start−1；无效超过 100 ms 的第一拍（start+50）中止。
    assert run_alt(lib, plant, start + 200, mutate=dropout(200)) == start + 50
    assert lib.APP_SysId_GetState() == STATE_ABORTED
    assert lib.APP_SysId_GetLastReason() == b"height_invalid"
    assert motor(lib) is None


@pytest.mark.parametrize("height", [HOLD + 0.1501, H0 - 0.0201], ids=["above", "below"])
def test_leaving_the_height_window_aborts(lib, height):
    plant = start_alt(lib)
    trip = RAMP_TICKS + CLIMB_TICKS + 50

    def mutate(index, obs):
        if index >= trip:
            obs.height_m = height

    assert run_alt(lib, plant, trip + 50, mutate=mutate) == trip
    assert lib.APP_SysId_GetLastReason() == b"height_window"
    assert lib.APP_SysId_GetState() == STATE_ABORTED
    assert motor(lib) is None


def test_the_window_edges_themselves_do_not_trip(lib):
    plant = start_alt(lib)
    base = RAMP_TICKS + CLIMB_TICKS

    def mutate(index, obs):
        if base <= index < base + 100:
            obs.height_m = HOLD + 0.1495
        elif base + 100 <= index < base + 200:
            obs.height_m = H0 - 0.0195

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
    assert rows[-2]["thrust"] < AIRFRAME_KG * G       # 封顶后实际推力，不是高度环要的


def test_stop_during_the_ramp_down_keeps_the_run(lib):
    plant = start_alt(lib)

    def mutate(index, obs):
        if lib.APP_SysId_GetPhase() == PHASE_RAMP_DOWN:
            lib.APP_SysId_Stop(b"command")

    run_alt(lib, plant, 6000, mutate=mutate)
    assert lib.APP_SysId_GetState() == STATE_DONE
    assert lib.APP_SysId_GetLastReason() == b"complete"


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
        run=65535, alt_inject="force", alt_mass_g=3000, alt_win_mm=400, alt_lift_mm=300,
        alt_h0_mm=-2147483648))
    echo = _worst_line(alt_source, '"SYSID ALT inject=', dict(
        inject="force", mass_g=3000, win_mm=400, lift_mm=300))
    sysid_source = (ROOT / "App/Src/app_sysid.c").read_text(encoding="utf-8")
    thr = _worst_line(sysid_source, '"SYSID THR auto=', {
        "auto": 1, "target_cn": 1_000_000, "max_pct_x10": 950, "phase": "ramp_down",
        "pulse_us": 65535, "armed": 1, "thr_low": 1, "capped": 1,
        "alt_h_mm": -2147483648, "alt_h_ok": 1, "alt_sp_mm": -2147483648})
    for line in (altstart, echo, thr):
        assert len(line) <= budget, (len(line), line)
    assert thr.endswith(" alt_h_mm=-2147483648 alt_h_ok=1 alt_sp_mm=-2147483648\r\n")

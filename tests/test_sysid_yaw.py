"""吊绳偏航辨识（SYSID MODE YAW = 6）—— 在宿主上跑真固件。

App/Src/app_sysid.c + app_sysid_yaw.c + app_cmd_sysid.c + 真分配器（drv_coax_ctrl 的偏航差动分配）+
真 drv_rate_control（生产偏航角速度环），平台边界见 tests/fixtures/sysid/。两侧契约：
doc/sysid-yaw-contract.md。宿主装置（build_lib / Capture / 文本与帧解码）复用 test_sysid_alt.py。

对象是"吊绳上的偏航转子"（YawPlant）：J·ω̇ = M_实 − d·ω，M_实 = P·(k·(T_lower − T_upper))，推力取
推力表对两路实际脉宽的读回，k 乘一个"实际效能"因子（默认 1/3，模拟作者实测的偏航效能偏低）。
这样"+ΔT 使 gyro_z 往正走"是固件的分配极性与物理模型对得上才成立，不是自证。

判据按台架上真会出事的地方排：

* **命令面**：`SYSID YAW` 回显一行、拒绝（running/range/lift/usage）整条不生效、上电默认；
* **开跑前检**：幅值上限（diff 按分配边界、rate 2.0）、推力 < 0.8×机重、桨位/偏航极性已标定；**溯源**：YAWSTART；
* **分配与脉宽**：`DRV_COAX_CTRL_AllocateYawPair` 极性/求和/钳位；上下桨脉宽确实不同，且方向随极性翻；
* **阶段与出力**：RAMP_UP → SETTLE → PREROLL → EXCITE → RAMP_DOWN；舵机全程中位；ΔT = 0 的段两桨同脉宽；
* **安全**：超速/绞角软停（1 s 回落、最后报理由、ΔT 清零），遥控/IMU 当拍交还，STOP 记 aborted；
* **记录**：7 个尾字段的 YAW 含义、批头 YAW 位；非 YAW 模式的电机脉宽两路逐位相同。
"""
from __future__ import annotations

import ctypes
import math
import re
from pathlib import Path

import pytest

from test_sysid_runtime_contract import (
    CONTROL_DT_US, PROFILE_DOUBLET, PROFILE_STEP, STATE_ABORTED, STATE_DONE, Excitation, Rig,
    build_lib, healthy, reset, strip_c_comments, texts,
)
from test_sysid_alt import (
    AIRFRAME_KG, EXTRA_SOURCES, G, IDLE_US, PHASE_EXCITE, PHASE_IDLE, PHASE_PREROLL,
    PHASE_RAMP_DOWN, PHASE_RAMP_UP, PHASE_SETTLE, WEIGHT_N, Capture, command, motor,
    phase_runs, prepare_handle, schema_of, step_spec, teardown_run,
)

ROOT = Path(__file__).resolve().parents[1]

MODE_YAW = 6
FLAG_YAW, FLAG_XY, FLAG_ALT = 0x0800, 0x0400, 0x0100
YAW_SEQUENCE = [PHASE_RAMP_UP, PHASE_SETTLE, PHASE_PREROLL, PHASE_EXCITE, PHASE_RAMP_DOWN]
RAMP_TICKS, SETTLE_TICKS, PREROLL_TICKS, DOWN_TICKS = 750, 1000, 250, 500
EXCITE_START = RAMP_TICKS + SETTLE_TICKS + PREROLL_TICKS
DT = CONTROL_DT_US * 1e-6
THRUST_MN = 3700                     # 0.5×悬停推力（机重 7.40 N）
K_MODEL = 0.005                      # DRV_COAX_CTRL_PROP9047_YAW_M_PER_N
IZZ = 0.00035                        # 默认机体的 izz_kgm2
LIFT_LIMIT_MN = 0.8 * WEIGHT_N * 1000.0
YAW_FIELDS = ("height", "height_raw", "height_sp", "vz", "vz_sp", "az", "vbat")

DOUBLET = dict(profile=PROFILE_DOUBLET, amplitude_rad_s=0.3, duration_ms=500, hold_ms=250,
               repeat=1, ramp_ms=20, chirp_f0_hz=0.3, chirp_f1_hz=6.0, prbs_bit_ms=40, prbs_seed=1)


class PropChannel(ctypes.Structure):
    _fields_ = [("role", ctypes.c_uint8), ("spin_sense", ctypes.c_int8),
                ("reserved", ctypes.c_uint8 * 2)]


class PropMap(ctypes.Structure):
    _fields_ = [("magic", ctypes.c_uint32), ("schema", ctypes.c_uint16), ("size", ctypes.c_uint16),
                ("channel", PropChannel * 2), ("calibrated", ctypes.c_uint8),
                ("reserved0", ctypes.c_uint8 * 3), ("generation", ctypes.c_uint32),
                ("reserved1", ctypes.c_uint32 * 2)]


def publish_prop_map(lib, *, lower_spin):
    """装桨叶标定：通道 1 = 上桨（逆时针）、通道 2 = 下桨。lower_spin=-1 → 偏航极性 P=+1。"""
    prop = PropMap()
    lib.DRV_PropMap_Defaults(ctypes.byref(prop))
    lower_spin = int(lower_spin)
    prop.channel[0].role, prop.channel[0].spin_sense = 0, -lower_spin
    prop.channel[1].role, prop.channel[1].spin_sense = 1, lower_spin
    prop.calibrated = 1
    prop.generation += 1
    assert lib.DRV_PropMap_Validate(ctypes.byref(prop)) == 1
    assert lib.DRV_PropMap_PublishActive(ctypes.byref(prop)) == 1


def prepare_yaw_handle(handle):
    f, u16 = ctypes.c_float, ctypes.c_uint16
    handle.DRV_PropMap_Defaults.argtypes = [ctypes.POINTER(PropMap)]
    handle.DRV_PropMap_Validate.argtypes = [ctypes.POINTER(PropMap)]
    handle.DRV_PropMap_Validate.restype = ctypes.c_uint8
    handle.DRV_PropMap_PublishActive.argtypes = [ctypes.POINTER(PropMap)]
    handle.DRV_PropMap_PublishActive.restype = ctypes.c_uint8
    handle.DRV_PropMap_ResetActive.argtypes = []
    handle.DRV_PropMap_YawTorquePolarity.restype = f
    handle.DRV_COAX_CTRL_AllocateYawPair.argtypes = [
        f, f, ctypes.POINTER(f), ctypes.POINTER(f), ctypes.POINTER(ctypes.c_uint8)]
    handle.DRV_COAX_CTRL_SingleMaxThrustN.restype = f
    handle.DRV_COAX_CTRL_GetYawTorqueCoefficients.argtypes = [ctypes.POINTER(f), ctypes.POINTER(f)]
    handle.DRV_COAX_CTRL_YawLimitMomentNm.argtypes = [f]
    handle.DRV_COAX_CTRL_YawLimitMomentNm.restype = f
    handle.APP_SysId_GetMotorPulsePair.argtypes = [ctypes.POINTER(u16), ctypes.POINTER(u16)]
    handle.APP_SysId_GetMotorPulsePair.restype = ctypes.c_uint8
    handle.APP_SysIdYaw_GetPsiRad.restype = f
    handle.APP_SysId_GetLastReason.restype = ctypes.c_char_p
    publish_prop_map(handle, lower_spin=-1)
    return handle


@pytest.fixture(scope="module")
def lib(tmp_path_factory):
    return prepare_yaw_handle(prepare_handle(build_lib(tmp_path_factory, EXTRA_SOURCES, "sysid-yaw")))


@pytest.fixture(scope="module")
def fresh_lib(tmp_path_factory):
    """一份从没收过 SYSID YAW 的库：看上电默认值用。"""
    return prepare_yaw_handle(prepare_handle(
        build_lib(tmp_path_factory, EXTRA_SOURCES, "sysid-yaw-defaults")))


# ---------------------------------------------------------------- 吊绳偏航对象


class YawPlant:
    """J·ω̇ = M_实 − d·ω；M_实 = P·k·efficacy·(T_lower − T_upper)，T = 推力表读回的单桨推力。"""

    def __init__(self, lib, *, efficacy=1.0 / 3.0, damping=1.0, inertia=IZZ, polarity=1.0):
        self.lib, self.eff, self.d, self.j, self.p = lib, efficacy, damping, inertia, polarity
        self.w = 0.0
        self.m_actual = 0.0
        self.max_w = 0.0

    def thrust(self, pulse):
        return 0.5 * self.lib.DRV_COAX_CTRL_MotorPulseToTotalThrust(pulse)

    def advance(self, pair):
        if pair is None:
            self.m_actual = 0.0
        else:
            upper, lower = pair
            self.m_actual = self.p * K_MODEL * self.eff * (self.thrust(lower) - self.thrust(upper))
        self.w += (self.m_actual / self.j - self.d * self.w) * DT
        self.max_w = max(self.max_w, abs(self.w))


def pair_of(lib):
    upper, lower = ctypes.c_uint16(), ctypes.c_uint16()
    if lib.APP_SysId_GetMotorPulsePair(ctypes.byref(upper), ctypes.byref(lower)) == 0:
        return None
    return upper.value, lower.value


def yaw_obs(t_ms, t_us, plant, **overrides):
    values = dict(throttle_us=IDLE_US, rc_throttle_low=1, vbat_v=11.876,
                  gyro_rad_s=(ctypes.c_float * 3)(0.0, 0.0, plant.w))
    values.update(overrides)
    return healthy(t_ms, t_us, **values)


def yaw_line(inject, thrust_mn, twist_deg):
    control = "openloop" if inject == "diff" else "closed_loop"
    return (f"SYSID YAW inject={inject} thrust_mn={thrust_mn} twist_deg={twist_deg} "
            f"control={control}\r\n")


def configure_yaw(lib, *, inject="diff", thrust_mn=THRUST_MN, twist_deg=720):
    line = f"SYSID YAW inject={inject} thrust_mn={thrust_mn} twist_deg={twist_deg}"
    assert command(lib, line) == [yaw_line(inject, thrust_mn, twist_deg)]


def prepare_yaw(lib, *, spec=None, inject="diff", thrust_mn=THRUST_MN, twist_deg=720,
                max_pct=75.0, rate_hz=50, plant=None, polarity=1.0, **obs):
    publish_prop_map(lib, lower_spin=-polarity)
    reset(lib, rate_hz=rate_hz, spec=Excitation(**(spec or DOUBLET)))
    # SYSID THROTTLE 只借它的最高油门 %；target_n 保持 0，YAW 不看它。
    assert lib.APP_SysId_SetThrottle(ctypes.c_float(0.0), ctypes.c_float(max_pct)) == 1
    assert lib.APP_SysId_SetMode(MODE_YAW, ctypes.c_float(0.0523598776)) == 1
    configure_yaw(lib, inject=inject, thrust_mn=thrust_mn, twist_deg=twist_deg)
    plant = plant or YawPlant(lib, polarity=polarity)
    for k in range(10):   # 没跑时控制拍也调 Update：START 靠它知道解锁/油门杆
        lib.APP_SysId_Update(ctypes.byref(yaw_obs(880 + 2 * k, 880_000 + 2000 * k, plant, **obs)))
    lib.harness_reset()
    return plant


def start_yaw(lib, **kwargs):
    plant = prepare_yaw(lib, **kwargs)
    assert lib.APP_SysId_Start() == 1, texts(lib)
    return plant


def run_yaw(lib, plant, ticks, *, mutate=None, record=None, capture=None, t0_ms=1_000):
    """跑 ticks 个 500 Hz 控制拍，转子跟着上下桨脉宽走。返回停在第几拍。
    record 每拍 (阶段, (上, 下) 脉宽或 None, 舵机 (alpha, beta), ω)。"""
    stopped = ticks
    for index in range(ticks):
        obs = yaw_obs(t0_ms + index * 2, t0_ms * 1000 + index * CONTROL_DT_US, plant)
        if mutate is not None:
            mutate(index, obs)
        lib.APP_SysId_Update(ctypes.byref(obs))
        pair = pair_of(lib)
        if record is not None:
            alpha, beta = ctypes.c_uint16(), ctypes.c_uint16()
            lib.APP_SysId_GetServoTargets(ctypes.byref(alpha), ctypes.byref(beta))
            record.append((lib.APP_SysId_GetPhase(), pair, (alpha.value, beta.value), plant.w))
        plant.advance(pair)
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


def full_run(lib, *, ticks=EXCITE_START + 800 + DOWN_TICKS + 60, **kwargs):
    """跑完整一轮并收齐帧与文本。"""
    schema = schema_of(lib)
    plant = start_yaw(lib, **kwargs)
    capture, record = Capture(lib), []
    capture.texts.extend(texts(lib))
    lib.harness_reset()
    stopped = run_yaw(lib, plant, ticks, record=record, capture=capture)
    if lib.APP_SysId_IsRunning() == 0 and record:
        record.pop()        # 收尾那一拍运行层已交还油门（GetPhase = idle、没有脉宽），不是出力
    return schema, plant, capture, record, stopped


# ---------------------------------------------------------------- 源码契约


def test_new_module_is_built_and_wired():
    cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
    assert "App/Src/app_sysid_yaw.c" in cmake
    header = (ROOT / "App/Inc/app_sysid_yaw.h").read_text(encoding="utf-8")
    for token in ("APP_SYSID_YAW_THRUST_MN_MIN      2000U", "APP_SYSID_YAW_TWIST_DEG_DEFAULT   720U",
                  "APP_SYSID_YAW_RATE_AMP_MAX_RAD_S  2.0f", "APP_SYSID_YAW_OVERSPEED_RAD_S     8.0f"):
        assert token in header
    record = (ROOT / "Driver/Inc/drv_sysid_record.h").read_text(encoding="utf-8")
    assert re.search(r"#define DRV_SYSID_FLAG_YAW\s+0x0800U", record)
    sysid_h = (ROOT / "App/Inc/app_sysid.h").read_text(encoding="utf-8")
    assert re.search(r"APP_SYSID_XY,.*\r?\n\s*APP_SYSID_YAW", sysid_h)


def test_module_stays_free_of_hal_esc_writes_and_float_printf():
    code = strip_c_comments((ROOT / "App/Src/app_sysid_yaw.c").read_text(encoding="utf-8"))
    assert "stm32h7xx" not in code and "HAL_" not in code
    for forbidden in ("DRV_Motor", "BSP_PWM_SetEscPulse", "BSP_PWM_SetEscPercent", "ConfigStore"):
        assert forbidden not in code, forbidden
    assert not re.search(r"%[-+ #0-9.]*[efgEFG]", code), "newlib-nano 没有浮点 printf"


def test_command_handler_routes_yaw_and_forbidden_files_are_untouched():
    cmd = (ROOT / "App/Src/app_cmd_sysid.c").read_text(encoding="utf-8")
    assert 'strcmp(sub, "YAW") == 0' in cmd and "APP_SysIdYaw_ReportConfig" in cmd
    assert 'strcmp(tokens[2], "YAW") == 0 || strcmp(tokens[2], "6") == 0' in cmd
    assert "app_sysid_yaw" not in (ROOT / "App/Src/app_control.c").read_text(encoding="utf-8")
    assert "app_sysid_yaw" not in (ROOT / "tools/drone_tcp_panel.py").read_text(encoding="utf-8")


def test_stabilizer_sends_the_pulse_pair_only_when_they_differ():
    code = strip_c_comments((ROOT / "App/Src/app_stabilizer.c").read_text(encoding="utf-8"))
    armed = code.split("((frame->rc_link_ok != 0U) && (frame->rc_armed != 0U))", 1)[1]
    armed = armed.split("} else if ((frame->rc_link_ok != 0U) || (frame->rc_link_seen == 0U))", 1)[0]
    assert "APP_SysId_GetMotorPulsePair(" in armed
    assert "if (direct_us == direct_lower_us)" in armed
    assert "stabilizer_commit_rotor_pulses(direct_us, direct_lower_us)" in armed
    # 相同脉宽时仍是原来的两条通道直写（其余模式逐位不变）。
    assert "BSP_PWM_SetEscPulse(1, direct_us);" in armed
    assert "BSP_PWM_SetEscPulse(2, direct_us);" in armed


# ---------------------------------------------------------------- 命令面


def test_power_on_defaults_are_diff_half_hover_720(fresh_lib):
    header = (ROOT / "App/Inc/app_sysid_yaw.h").read_text(encoding="utf-8")
    assert re.search(r"#define APP_SYSID_YAW_TWIST_DEG_DEFAULT\s+720U", header)
    line = command(fresh_lib, "SYSID YAW")[0]
    match = re.fullmatch(r"SYSID YAW inject=diff thrust_mn=(\d+) twist_deg=720 control=openloop\r\n", line)
    assert match, line
    assert int(match.group(1)) == pytest.approx(0.5 * WEIGHT_N * 1000.0, abs=1.5)


def test_yaw_command_echoes_one_line_with_control(lib):
    lib.APP_SysId_Stop(b"test")
    configure_yaw(lib)
    assert command(lib, "SYSID YAW") == [yaw_line("diff", THRUST_MN, 720)]
    assert command(lib, "SYSID YAW inject=rate thrust_mn=4200 twist_deg=360") == [
        yaw_line("rate", 4200, 360)]
    assert command(lib, "SYSID YAW inject=diff") == [yaw_line("diff", 4200, 360)]   # 其余保持原值
    assert command(lib, "SYSID YAW twist_deg=1440 thrust_mn=2000") == [yaw_line("diff", 2000, 1440)]
    configure_yaw(lib)


@pytest.mark.parametrize("args,reason", [
    ("thrust_mn=1999", "range"), ("thrust_mn=70000", "range"), ("twist_deg=89", "range"),
    ("twist_deg=1441", "range"), ("twist_deg=70000", "range"),
    ("thrust_mn=5930", "lift"), ("thrust_mn=12000", "lift"),
    ("inject=up", "usage"), ("inject=tilt", "usage"), ("thrust_mn=abc", "usage"),
    ("thrust_mn=3.5", "usage"), ("foo=1", "usage"), ("twist_deg=100 bar", "usage"),
    ("win_mm=100", "usage"),
])
def test_yaw_command_rejects_whole_line_without_touching_the_config(lib, args, reason):
    lib.APP_SysId_Stop(b"test")
    configure_yaw(lib, inject="rate", thrust_mn=4100, twist_deg=500)
    assert command(lib, f"SYSID YAW inject=diff {args}") == [
        f"SYSID YAW event=rejected reason={reason}\r\n"]
    assert command(lib, "SYSID YAW") == [yaw_line("rate", 4100, 500)]
    configure_yaw(lib)


def test_lift_limit_is_zero_point_eight_of_the_weight(lib):
    lib.APP_SysId_Stop(b"test")
    inside, outside = int(LIFT_LIMIT_MN) - 2, int(LIFT_LIMIT_MN) + 3
    assert command(lib, f"SYSID YAW thrust_mn={inside}") == [yaw_line("diff", inside, 720)]
    assert command(lib, f"SYSID YAW thrust_mn={outside}") == [
        "SYSID YAW event=rejected reason=lift\r\n"]
    for endpoint in ("thrust_mn=2000", "twist_deg=90", "twist_deg=1440"):
        assert command(lib, f"SYSID YAW {endpoint}")[0].startswith("SYSID YAW inject=")
    configure_yaw(lib)


def test_yaw_config_is_idle_only_but_readable_while_running(lib):
    plant = start_yaw(lib)
    run_yaw(lib, plant, 10)
    assert command(lib, "SYSID YAW twist_deg=100") == ["SYSID YAW event=rejected reason=running\r\n"]
    assert command(lib, "SYSID YAW") == [yaw_line("diff", THRUST_MN, 720)]
    teardown_run(lib)


@pytest.mark.parametrize("token", ["YAW", "6"])
def test_mode_yaw_by_name_and_number(lib, token):
    reset(lib)
    lines = command(lib, f"SYSID MODE {token}")
    assert f" mode={MODE_YAW} " in next(line for line in lines if line.startswith("SYSID READY "))
    assert lib.APP_SysId_GetMode() == MODE_YAW
    assert lib.APP_SysId_SetMode(MODE_YAW + 1, ctypes.c_float(0.05)) == 0
    assert "YAW" in command(lib, "SYSID MODE NOPE")[0]


# ---------------------------------------------------------------- 分配（生产同一函数的公开包装）


def allocate(lib, force, moment):
    upper, lower, sat = ctypes.c_float(), ctypes.c_float(), ctypes.c_uint8()
    lib.DRV_COAX_CTRL_AllocateYawPair(ctypes.c_float(force), ctypes.c_float(moment),
                                      ctypes.byref(upper), ctypes.byref(lower), ctypes.byref(sat))
    return upper.value, lower.value, sat.value


@pytest.mark.parametrize("polarity", [1.0, -1.0])
def test_allocation_sums_to_force_and_follows_the_polarity(lib, polarity):
    publish_prop_map(lib, lower_spin=-polarity)
    assert lib.DRV_PropMap_YawTorquePolarity() == polarity
    ku, kl = ctypes.c_float(), ctypes.c_float()
    lib.DRV_COAX_CTRL_GetYawTorqueCoefficients(ctypes.byref(ku), ctypes.byref(kl))
    assert ku.value == pytest.approx(K_MODEL) and kl.value == pytest.approx(K_MODEL)
    force, moment = 3.7, 0.001
    upper, lower, sat = allocate(lib, force, moment)
    assert sat == 0 and upper + lower == pytest.approx(force, abs=1e-5)
    # Mz = P·(kl·T_lower − ku·T_upper)：正力矩在 P=+1 时下桨出得多，P=−1 时上桨出得多。
    assert polarity * (kl.value * lower - ku.value * upper) == pytest.approx(moment, rel=1e-4)
    assert (lower - upper) == pytest.approx(polarity * moment / K_MODEL, rel=1e-4)
    zero = allocate(lib, force, 0.0)
    assert zero[0] == pytest.approx(force / 2) and zero[1] == pytest.approx(force / 2)
    publish_prop_map(lib, lower_spin=-1)


def test_allocation_clamps_to_the_single_motor_limit_and_flags_it(lib):
    single_max = lib.DRV_COAX_CTRL_SingleMaxThrustN()
    assert 5.0 < single_max <= 10.2
    upper, lower, sat = allocate(lib, 3.7, 0.05)     # 差速远超能给的：弱的那路钳到 0
    assert sat == 1 and min(upper, lower) == 0.0 and max(upper, lower) <= single_max
    upper, lower, sat = allocate(lib, 2.0 * single_max + 2.0, 0.0)
    assert sat == 1 and upper == pytest.approx(single_max) and lower == pytest.approx(single_max)
    # 抗饱和上限与分配边界对偶：给到上限的力矩恰好不钳。
    limit = lib.DRV_COAX_CTRL_YawLimitMomentNm(ctypes.c_float(3.7))
    assert limit == pytest.approx(K_MODEL * 3.7, rel=1e-4)    # F 小时受 k·F 约束
    assert allocate(lib, 3.7, 0.99 * limit)[2] == 0
    assert allocate(lib, 3.7, 1.05 * limit)[2] == 1


# ---------------------------------------------------------------- 开跑前检查


def test_start_refuses_diff_amp_over_the_allocation_bound(lib):
    spec = dict(DOUBLET, amplitude_rad_s=3.0)       # 上限 min(0.8·3.7, 2·(T单max − 1.85)) = 2.96
    prepare_yaw(lib, spec=spec)
    assert lib.APP_SysId_Start() == 0
    assert texts(lib) == ["ERR sysid yaw amp over limit (inject=diff max=2.960)\r\n"]
    assert lib.APP_SysId_IsRunning() == 0
    ok = dict(DOUBLET, amplitude_rad_s=2.9)
    start_yaw(lib, spec=ok)
    teardown_run(lib)


def test_diff_amp_bound_follows_the_thrust_and_the_single_motor_limit(lib):
    single_max = lib.DRV_COAX_CTRL_SingleMaxThrustN()
    for thrust_mn in (2000, 3000, 5000):
        force = thrust_mn / 1000.0
        expected = min(0.8 * force, 2.0 * (single_max - force / 2.0))
        prepare_yaw(lib, thrust_mn=thrust_mn, spec=dict(DOUBLET, amplitude_rad_s=expected + 0.05))
        assert lib.APP_SysId_Start() == 0
        line = texts(lib)[0]
        shown = float(re.search(r"max=([0-9.]+)\)", line).group(1))
        assert shown == pytest.approx(expected, abs=0.001), line


def test_start_refuses_rate_amp_over_two_rad_s(lib):
    prepare_yaw(lib, inject="rate", spec=dict(DOUBLET, amplitude_rad_s=2.1))
    assert lib.APP_SysId_Start() == 0
    assert texts(lib) == ["ERR sysid yaw amp over limit (inject=rate max=2.000)\r\n"]
    start_yaw(lib, inject="rate", spec=dict(DOUBLET, amplitude_rad_s=2.0))
    teardown_run(lib)


def test_start_refuses_an_uncalibrated_prop_map(lib):
    prepare_yaw(lib)
    lib.DRV_PropMap_ResetActive()
    assert lib.APP_SysId_Start() == 0
    assert texts(lib) == ["ERR sysid yaw prop map uncalibrated (yaw polarity)\r\n"]
    publish_prop_map(lib, lower_spin=-1)


def test_start_does_not_need_the_sysid_throttle_target_but_needs_arming(lib):
    """YAW 的总推力在 thrust_mn 里，不借 SYSID THROTTLE 的 target_n；解锁/油门杆低照旧要求。"""
    prepare_yaw(lib, rc_armed=0)
    assert lib.APP_SysId_Start() == 0
    assert "not armed" in texts(lib)[0]
    prepare_yaw(lib, rc_throttle_low=0)
    assert lib.APP_SysId_Start() == 0
    assert "throttle stick not low" in texts(lib)[0]
    start_yaw(lib)
    teardown_run(lib)


# ---------------------------------------------------------------- 溯源与阶段


def test_start_line_and_yawstart_provenance_are_verbatim(lib):
    prepare_yaw(lib, inject="rate", thrust_mn=4100, twist_deg=540, spec=dict(DOUBLET, amplitude_rad_s=0.5))
    assert lib.APP_SysId_Start() == 1
    lines = texts(lib)
    assert lines[0].startswith("SYSID start run=") and " auto=1 target_cn=410 " in lines[0]
    single_max_mn = round(lib.DRV_COAX_CTRL_SingleMaxThrustN() * 1000)
    assert lines[1] == (f"SYSID YAWSTART run={lib.APP_SysId_GetRunId()} yaw_inject=rate "
                        f"yaw_thrust_mn=4100 yaw_k_um_per_n=5000 yaw_izz_ugm2=350 "
                        f"yaw_single_max_mn={single_max_mn} yaw_twist_deg=540\r\n")
    assert max(len(line) for line in lines) < 255
    teardown_run(lib)


def test_phases_servos_and_pulses_over_a_diff_run(lib):
    schema, plant, capture, record, stopped = full_run(lib)
    assert lib.APP_SysId_GetState() == STATE_DONE
    assert lib.APP_SysId_GetLastReason() == b"complete"
    seq = [(phase, ticks) for phase, ticks in phase_runs([(p, 0, 0) for p, *_ in record])]
    assert [p for p, _ in seq] == YAW_SEQUENCE
    assert [n for _p, n in seq][:3] == [RAMP_TICKS, SETTLE_TICKS, PREROLL_TICKS]
    # 舵机全程中位。
    assert len({servo for _p, _pair, servo, _w in record}) == 1
    for phase, pair, _servo, _w in record:
        assert pair is not None and phase != PHASE_IDLE
    # RAMP_UP 由怠速线性升、两桨同脉宽；SETTLE/PREROLL ΔT=0 两桨同脉宽（legacy 曲线下相等）。
    ramp = [pair for phase, pair, *_ in record if phase == PHASE_RAMP_UP]
    assert ramp[0][0] == ramp[0][1] and ramp[0][0] >= IDLE_US
    assert all(b[0] >= a[0] for a, b in zip(ramp, ramp[1:]))
    for phase, pair, *_ in record:
        if phase in (PHASE_RAMP_UP, PHASE_SETTLE, PHASE_PREROLL):
            assert pair[0] == pair[1], phase
    # 稳定段合推力 = 配置的 F。
    settle = next(pair for phase, pair, *_ in record if phase == PHASE_SETTLE)
    assert lib.DRV_COAX_CTRL_MotorPulseToTotalThrust(settle[0]) == pytest.approx(THRUST_MN / 1000.0, abs=0.06)
    # EXCITE 里上下桨确实不同，且 ΔT 的符号与脉宽差一致（P=+1：ΔT>0 ⇒ 下桨更高）。
    diffs = [pair[1] - pair[0] for phase, pair, *_ in record if phase == PHASE_EXCITE]
    assert max(diffs) > 5 and min(diffs) < -5
    # RAMP_DOWN 线性回落到怠速，两桨同脉宽。
    down = [pair for phase, pair, *_ in record if phase == PHASE_RAMP_DOWN]
    assert all(a[0] == a[1] for a in down[1:])
    assert down[-1][0] <= IDLE_US + 3 and all(b[0] <= a[0] for a, b in zip(down, down[1:]))
    assert len(down) == pytest.approx(DOWN_TICKS, abs=2)


@pytest.mark.parametrize("polarity", [1.0, -1.0])
def test_positive_delta_t_lifts_the_right_motor_and_turns_the_rotor_positive(lib, polarity):
    """分配极性是辨识的命门：同一个 +ΔT，P=±1 时高的是下桨/上桨，而物理模型里转子都往 +gz 走。"""
    spec = step_spec(0.4, hold_ms=300, ramp_ms=20)
    spec["amplitude_rad_s"] = 0.4
    plant = start_yaw(lib, spec=spec, polarity=polarity)
    record = []
    run_yaw(lib, plant, EXCITE_START + 200, record=record)
    pair = record[EXCITE_START + 100][1]
    assert (pair[1] - pair[0]) * polarity > 12, pair
    assert plant.w > 0.2
    teardown_run(lib)
    publish_prop_map(lib, lower_spin=-1)


def test_pulses_follow_the_thrust_map_pair_when_injected(lib):
    """注入了 pulses_for_pair 的映射优先用它：宿主这里没有注入，走逐桨查表，两条路径的接口同形。"""
    code = strip_c_comments((ROOT / "App/Src/app_sysid_yaw.c").read_text(encoding="utf-8"))
    assert "pulses_for_pair" in code and "DRV_COAX_CTRL_ThrustToMotorPulse" in code
    assert "DRV_COAX_CTRL_AllocateYawPair" in code and "DRV_RateControl_Step" in code


def test_max_pct_caps_both_motors_and_flags_it(lib):
    schema, plant, capture, record, stopped = full_run(lib, max_pct=10.0)
    cap = int(1100 + 10.0 * 840 / 100.0 + 0.5)
    assert all(max(pair) <= cap for _p, pair, *_ in record)
    _rows, flags = capture.samples(schema)
    assert flags & 0x0200        # THRUST_CAPPED


# ---------------------------------------------------------------- 记录


def test_record_fields_have_the_yaw_meaning(lib):
    schema, plant, capture, record, stopped = full_run(lib, inject="diff", rate_hz=250)
    rows, flags = capture.samples(schema)
    assert flags & FLAG_YAW and not flags & (FLAG_XY | FLAG_ALT)
    assert len(rows) == pytest.approx(stopped / 2 + 1, abs=4)
    ku = K_MODEL
    for row in rows:
        assert row["height_raw"] == 0.0 and row["height_sp"] == 0.0
        assert row["vz_sp"] == 0.0                          # diff：没有角速度参考
        assert row["angle"] == 0.0 and row["angle_sp"] == 0.0 and row["servo_tilt"] == 0.0
        assert row["tilt_x"] == 0.0 and row["tilt_y"] == 0.0
        assert row["vz"] == pytest.approx(row["gz"], abs=1.5e-3)       # 原始陀螺 z
        assert row["torque"] == pytest.approx(ku * row["az"], abs=1.2e-4)   # M = k·ΔT
        assert row["vbat"] == pytest.approx(11.876, abs=1e-3)
    thrusts = [row["thrust"] for row in rows]
    assert max(thrusts) == pytest.approx(THRUST_MN / 1000.0, abs=0.02)
    excite_rows = [row for row in rows if abs(row["az"]) > 0.05]
    assert excite_rows and max(abs(row["az"]) for row in rows) == pytest.approx(0.3, abs=0.01)
    # ψ = 陀螺 z 的积分：与转子自己的积分一致，开跑清零。
    psi = [row["height"] for row in rows]
    assert abs(psi[0]) < 0.01 and psi[-1] == pytest.approx(lib.APP_SysIdYaw_GetPsiRad(), abs=2e-3)
    assert abs(psi[-1]) > 0.005 or max(abs(p) for p in psi) > 0.005


def test_diff_az_is_the_allocated_delta_t_not_the_requested_one(lib):
    """az = P·(T_lower − T_upper)（分配后）：差速没钳时等于 amp·shape，正值对应正 M。"""
    spec = step_spec(0.5, hold_ms=300, ramp_ms=20)
    spec["amplitude_rad_s"] = 0.5
    schema, plant, capture, record, stopped = full_run(lib, spec=spec, rate_hz=250)
    rows, _flags = capture.samples(schema)
    az = [row["az"] for row in rows]
    assert max(az) == pytest.approx(0.5, abs=0.01) and abs(min(az)) < 0.01
    torque = [row["torque"] for row in rows]
    assert max(torque) == pytest.approx(0.5 * K_MODEL, abs=2e-4)


def test_erpm_columns_carry_the_rotor_speeds(lib):
    """erpm = 上桨、erpm_lower = 下桨，按桨叶标定查通道（通道 1 = 上桨）。"""
    schema = schema_of(lib)
    try:
        plant = start_yaw(lib, rate_hz=250)
        lib.harness_set_rx_available(1)             # harness_reset 会清回传快照：开跑之后再装
        for index, value in enumerate((4000, 4400)):       # 通道 1、通道 2；样本时刻在跑的窗口内
            lib.harness_set_rotor(index, value, 1_010, 1, 0)
        capture = Capture(lib)
        assert run_yaw(lib, plant, 40) == 40
        lib.APP_SysId_Stop(b"test")
        for _ in range(64):
            lib.APP_SysId_StreamTick()
        capture.flush()
        rows, _flags = capture.samples(schema)
        assert rows and all(row["erpm"] == 4000 and row["erpm_lower"] == 4400 for row in rows)
    finally:
        lib.harness_set_rx_available(0)
        teardown_run(lib)


# ---------------------------------------------------------------- rate（闭环）


def test_rate_loop_uses_the_production_yaw_gains(lib):
    """r = 0 且转子静止：M=0；给了 r 之后 M = kp·(r − ω) + I…，改 coax.rate_yaw_kp 下一拍生效。"""
    spec = step_spec(0.8, hold_ms=600, ramp_ms=20)
    spec["amplitude_rad_s"] = 0.8
    saved = ctypes.c_float()
    assert lib.DRV_COAX_CTRL_GetParam(b"coax.rate_yaw_kp", ctypes.byref(saved)) == 1
    ki = ctypes.c_float()
    assert lib.DRV_COAX_CTRL_GetParam(b"coax.rate_yaw_ki", ctypes.byref(ki)) == 1
    try:
        # 先看 kp=0：只剩积分项，首拍 M 必然比大 kp 时小得多。
        results = {}
        for kp in (0.0, 0.01):
            assert lib.DRV_COAX_CTRL_SetParam(b"coax.rate_yaw_kp", ctypes.c_float(kp)) == 1
            schema = schema_of(lib)
            plant = start_yaw(lib, inject="rate", spec=spec, rate_hz=250)
            capture = Capture(lib)
            run_yaw(lib, plant, EXCITE_START + 60, capture=capture)
            teardown_run(lib)
            rows, _flags = capture.samples(schema)
            results[kp] = max(abs(row["torque"]) for row in rows)
        assert results[0.01] > results[0.0] + 5e-3, results
    finally:
        lib.DRV_COAX_CTRL_SetParam(b"coax.rate_yaw_kp", ctypes.c_float(saved.value))


def test_rate_run_tracks_the_reference_and_records_it(lib):
    spec = step_spec(0.8, hold_ms=1500, ramp_ms=20)
    spec["amplitude_rad_s"] = 0.8
    saved = {}
    for name, value in (("coax.rate_yaw_kp", 0.012), ("coax.rate_yaw_ki", 0.02)):
        old = ctypes.c_float()
        assert lib.DRV_COAX_CTRL_GetParam(name.encode(), ctypes.byref(old)) == 1
        saved[name] = old.value
        assert lib.DRV_COAX_CTRL_SetParam(name.encode(), ctypes.c_float(value)) == 1
    try:
        schema, plant, capture, record, stopped = full_run(lib, inject="rate", spec=spec,
                                                           rate_hz=250, ticks=EXCITE_START + 700)
        rows, flags = capture.samples(schema)
        assert flags & FLAG_YAW
        # SETTLE/PREROLL 段 r = 0 且转子不动：参考记 0；激励段参考 = 0.8。
        assert abs(rows[10]["vz_sp"]) < 1e-3 and rows[10]["torque"] == pytest.approx(0.0, abs=2e-4)
        assert max(row["vz_sp"] for row in rows) == pytest.approx(0.8, abs=0.01)
        # 阶跃后 ω 追上参考（效能 1/3 的转子，PI 积分补上）。
        tail = [row["vz"] for row in rows[-40:]]
        assert sum(tail) / len(tail) == pytest.approx(0.8, abs=0.15)
        # M（饱和前）与 ΔT 通过 k 对应；az 是分配后的差速。
        for row in rows[-30:]:
            assert row["torque"] == pytest.approx(K_MODEL * row["az"], abs=2.5e-4)
    finally:
        for name, value in saved.items():
            assert lib.DRV_COAX_CTRL_SetParam(name.encode(), ctypes.c_float(value)) == 1


# ---------------------------------------------------------------- 安全


def test_overspeed_triggers_a_soft_stop_with_delta_t_cleared(lib):
    schema = schema_of(lib)
    plant = start_yaw(lib, inject="diff")
    capture, record = Capture(lib), []

    def spin(index, obs):
        if index >= EXCITE_START + 40:
            obs.gyro_rad_s = (ctypes.c_float * 3)(0.0, 0.0, 8.5)
            obs.gyro_ctrl_rad_s = (ctypes.c_float * 3)(0.0, 0.0, 8.5)

    stopped = run_yaw(lib, plant, EXCITE_START + 1000, mutate=spin, record=record, capture=capture)
    assert lib.APP_SysId_GetState() == STATE_ABORTED
    assert lib.APP_SysId_GetLastReason() == b"yaw_overspeed"
    # 触发后 1 s 软停：阶段报 ramp_down，推力线性回落，两桨同脉宽（ΔT 已清零）。
    assert stopped == pytest.approx(EXCITE_START + 40 + DOWN_TICKS, abs=3)
    tail = record[EXCITE_START + 41:]
    assert tail and all(phase == PHASE_RAMP_DOWN for phase, *_ in tail[:-1])
    assert all(pair[0] == pair[1] for _p, pair, *_ in tail[:-1])      # ΔT 当拍清零
    pulses = [pair[0] for _p, pair, *_ in tail[:-1]]
    assert all(b <= a for a, b in zip(pulses, pulses[1:])) and pulses[-1] < pulses[0] - 20
    rows, flags = capture.samples(schema)
    assert flags & 0x0004                       # ABORTED
    assert any(line.startswith("SYSID end ") and "reason=yaw_overspeed" in line for line in capture.texts)


def test_overspeed_boundary_is_eight_rad_s(lib):
    for gz, expect_abort in ((7.9, False), (8.1, True)):
        plant = start_yaw(lib)

        def spin(index, obs, gz=gz):
            if index >= 5:
                obs.gyro_rad_s = (ctypes.c_float * 3)(0.0, 0.0, gz)
                obs.gyro_ctrl_rad_s = (ctypes.c_float * 3)(0.0, 0.0, gz)

        run_yaw(lib, plant, 300, mutate=spin)       # 软停要 1 s，300 拍时还在回落中
        assert lib.APP_SysId_IsRunning() == 1
        assert (lib.APP_SysId_GetPhase() == PHASE_RAMP_DOWN) == expect_abort
        teardown_run(lib)


def test_twist_limit_triggers_a_soft_stop_and_psi_is_the_gyro_integral(lib):
    plant = start_yaw(lib, twist_deg=90)
    record = []

    def turn(index, obs):
        obs.gyro_rad_s = (ctypes.c_float * 3)(0.0, 0.0, 1.0)
        obs.gyro_ctrl_rad_s = (ctypes.c_float * 3)(0.0, 0.0, 1.0)

    stopped = run_yaw(lib, plant, 2000, mutate=turn, record=record)
    # 1 rad/s 积到 90° = π/2 rad 需要约 1.571 s ≈ 785 拍，然后 1 s 软停。
    assert lib.APP_SysId_GetState() == STATE_ABORTED
    assert lib.APP_SysId_GetLastReason() == b"yaw_twist"
    assert stopped == pytest.approx(785 + DOWN_TICKS, abs=6)
    assert lib.APP_SysIdYaw_GetPsiRad() == pytest.approx((stopped + 1) * DT, abs=0.03)   # 软停期间继续积
    teardown_run(lib)


def test_twist_limit_boundary_follows_twist_deg(lib):
    plant = start_yaw(lib, twist_deg=720)

    def turn(index, obs):
        obs.gyro_rad_s = (ctypes.c_float * 3)(0.0, 0.0, 3.0)    # 3 rad/s：积 12.57 rad = 720° 需 4.19 s
        obs.gyro_ctrl_rad_s = (ctypes.c_float * 3)(0.0, 0.0, 3.0)

    run_yaw(lib, plant, 1500, mutate=turn)        # 3 s：8.6 rad < 720°
    assert lib.APP_SysId_IsRunning() == 1 and lib.APP_SysId_GetPhase() != PHASE_RAMP_DOWN
    teardown_run(lib)


@pytest.mark.parametrize("override,reason", [
    (dict(rc_link_ok=0), b"rc_lost"), (dict(rc_armed=0), b"rc_disarm"),
    (dict(imu_valid=0), b"imu_stale"), (dict(rc_throttle_low=0), b"rc_throttle_override"),
])
def test_rc_and_imu_failures_hand_the_throttle_back_on_the_same_tick(lib, override, reason):
    plant = start_yaw(lib)
    stopped = run_yaw(lib, plant, 600, mutate=lambda i, obs: (
        [setattr(obs, k, v) for k, v in override.items()] if i == 300 else None))
    assert stopped == 300
    assert lib.APP_SysId_GetState() == STATE_ABORTED and lib.APP_SysId_GetLastReason() == reason
    assert pair_of(lib) is None                    # 油门当拍交还
    teardown_run(lib)


def test_stop_command_aborts_even_during_ramp_down(lib):
    plant = start_yaw(lib)
    for index in range(4000):
        lib.APP_SysId_Update(ctypes.byref(yaw_obs(1_000 + index * 2, 1_000_000 + index * 2000, plant)))
        plant.advance(pair_of(lib))
        lib.APP_SysId_StreamTick()
        if lib.APP_SysId_GetPhase() == PHASE_RAMP_DOWN:
            break
    assert lib.APP_SysId_GetPhase() == PHASE_RAMP_DOWN and lib.APP_SysId_IsRunning() == 1
    lib.APP_SysId_Stop(b"command")
    assert lib.APP_SysId_GetState() == STATE_ABORTED
    teardown_run(lib)


def test_stale_thrust_source_hands_back(lib):
    plant = start_yaw(lib)
    stopped = run_yaw(lib, plant, 1400, mutate=lambda i, obs: setattr(obs, "thrust_valid", 0) if i == 1200 else None)
    assert lib.APP_SysId_GetLastReason() == b"thrust_stale" and stopped == 1200
    teardown_run(lib)


# ---------------------------------------------------------------- 其余模式的电机输出不变


def test_non_yaw_modes_send_the_same_pulse_to_both_motors(lib):
    reset(lib)
    assert lib.APP_SysId_SetThrottle(ctypes.c_float(5.0), ctypes.c_float(75.0)) == 1
    assert lib.APP_SysId_SetMode(0, ctypes.c_float(0.05)) == 1       # FF
    plant = YawPlant(lib)
    for k in range(10):
        lib.APP_SysId_Update(ctypes.byref(yaw_obs(880 + 2 * k, 880_000 + 2000 * k, plant)))
    assert lib.APP_SysId_Start() == 1, texts(lib)
    run_yaw(lib, plant, 100)
    pair, single = pair_of(lib), motor(lib)
    assert pair is not None and pair[0] == pair[1] == single
    teardown_run(lib)
    reset(lib)      # target_n=0（手动）：两个接口都说"不接管"
    assert lib.APP_SysId_SetMode(0, ctypes.c_float(0.05)) == 1
    assert pair_of(lib) is None and motor(lib) is None

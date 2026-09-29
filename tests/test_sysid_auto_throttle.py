"""光杆辨识的自动油门与控制节拍 —— 在宿主上跑 App/Src/app_sysid.c 真代码。

作者 2026-09-26 授权："解锁交给我，我解锁之后你可以操控油门进行辨识"。
判据按台架上真会出事的地方排：

* **只在解锁且油门杆最低时接管**，否则 START 当场拒绝并说明原因；
* 升油门单调、永不超过"最高油门 %"封顶；
* 飞手推杆、上锁、点停止 → 本拍起不再给电机脉宽（交还遥控器）；
* 回落阶段被打断不作废已经完整的数据；
* 杆上三轮 1/102/4 条样本即 `control_dt` 中止的根因：辨识占用时稳定环每拍
  清调度器，控制拍跟着 IMU 唤醒走，两次唤醒挨近时间隔 <0.5 ms。
"""
from __future__ import annotations

import ctypes
import os
import shutil
import subprocess
from pathlib import Path

import pytest

from test_sysid_runtime_contract import (  # noqa: F401  lib 是 fixture
    CONTROL_DT_US, FLAG_ABORTED, FLAG_LAST, PROFILE_DOUBLET, STATE_ABORTED,
    STATE_DONE, Excitation, batch_header, frames, healthy, lib, reset, strip_c_comments, texts,
)

ROOT = Path(__file__).resolve().parents[1]

PHASE_IDLE, PHASE_RAMP_UP, PHASE_SETTLE, PHASE_EXCITE, PHASE_RAMP_DOWN = range(5)
IDLE_US = 1100
TARGET_N = 7.4
CAP_US_75 = round(1100 + 75 * 8.4)


def target_us(lib, total_n):
    """真实分配器里的推力表（宿主上没装查补表 = 旧曲线），单桨 = 合推力/2。"""
    lib.DRV_COAX_CTRL_ThrustToMotorPulse.argtypes = [ctypes.c_float]
    lib.DRV_COAX_CTRL_ThrustToMotorPulse.restype = ctypes.c_uint16
    return lib.DRV_COAX_CTRL_ThrustToMotorPulse(ctypes.c_float(total_n / 2.0))


def idle_obs(t_ms, t_us, **overrides):
    values = dict(throttle_us=IDLE_US, rc_throttle_low=1)
    values.update(overrides)
    return healthy(t_ms, t_us, **values)


def prime(lib, *, armed=1, low=1):
    """没跑辨识时控制拍也调 Update；START 靠它知道当前解锁与油门杆状态。"""
    lib.APP_SysId_Update(ctypes.byref(idle_obs(900, 900_000, rc_armed=armed,
                                               rc_throttle_low=low)))


def motor(lib):
    pulse = ctypes.c_uint16()
    return pulse.value if lib.APP_SysId_GetMotorPulse(ctypes.byref(pulse)) else None


def start_auto(lib, target_n=TARGET_N, max_pct=75.0):
    # 台架实际用的小幅双脉冲（150 ms 斜坡）；10 ms 斜坡的默认夹具要的力矩装置推力给不起。
    reset(lib, spec=Excitation(profile=PROFILE_DOUBLET, amplitude_rad_s=0.15,
                               duration_ms=4000, hold_ms=250, repeat=2, ramp_ms=150,
                               chirp_f0_hz=0.3, chirp_f1_hz=6.0, prbs_bit_ms=40,
                               prbs_seed=1))
    assert lib.APP_SysId_SetThrottle(ctypes.c_float(target_n), ctypes.c_float(max_pct)) == 1
    prime(lib)
    assert lib.APP_SysId_Start() == 1


def run_auto(lib, count, *, mutate=None, record=None, drain=True):
    for index in range(count):
        obs = idle_obs(1_000 + index * 2, 1_000_000 + index * CONTROL_DT_US)
        if mutate is not None:
            mutate(index, obs)
        lib.APP_SysId_Update(ctypes.byref(obs))
        if record is not None:
            record.append((lib.APP_SysId_GetPhase(), motor(lib)))
        if drain:
            lib.APP_SysId_StreamTick()
        if lib.APP_SysId_IsRunning() == 0:
            return index
    return count


RAMP_TICKS = 1500 // 2
SETTLE_TICKS = 1500 // 2
PREROLL_TICKS = 500 // 2


# ---------------------------------------------------------------- 接管前提


def test_auto_start_is_refused_until_armed_with_the_stick_at_the_bottom(lib):
    reset(lib)
    assert lib.APP_SysId_SetThrottle(ctypes.c_float(TARGET_N), ctypes.c_float(75.0)) == 1
    prime(lib, armed=0)
    assert lib.APP_SysId_Start() == 0
    assert any("not armed" in line for line in texts(lib))
    lib.harness_reset()
    prime(lib, armed=1, low=0)
    assert lib.APP_SysId_Start() == 0
    assert any("throttle stick not low" in line for line in texts(lib))
    prime(lib)
    assert lib.APP_SysId_Start() == 1
    lib.APP_SysId_Stop(b"test")


def test_throttle_settings_are_range_checked_and_frozen_while_running(lib):
    reset(lib)
    set_ = lib.APP_SysId_SetThrottle
    assert set_(ctypes.c_float(1.0), ctypes.c_float(75.0)) == 0      # 低于最小合推力
    assert set_(ctypes.c_float(99.0), ctypes.c_float(75.0)) == 0     # 超过机体最大合推力
    assert set_(ctypes.c_float(TARGET_N), ctypes.c_float(5.0)) == 0
    assert set_(ctypes.c_float(TARGET_N), ctypes.c_float(99.0)) == 0
    assert set_(ctypes.c_float(0.0), ctypes.c_float(75.0)) == 1      # 手动模式
    start_auto(lib)
    assert set_(ctypes.c_float(0.0), ctypes.c_float(75.0)) == 0
    lib.APP_SysId_Stop(b"test")


# ---------------------------------------------------------------- 完整一轮


def test_auto_run_ramps_up_settles_excites_and_ramps_back_down(lib):
    start_auto(lib)
    lib.harness_reset()
    record = []
    stopped = run_auto(lib, 4000, record=record)
    assert stopped < 4000
    assert (lib.APP_SysId_GetState(), lib.APP_SysId_GetLastReason(), record[-1]) == \
        (STATE_DONE, b"complete", (PHASE_IDLE, None))
    assert motor(lib) is None, "结束后必须交还遥控器油门"

    order = []
    for phase, _pulse in record:
        if not order or order[-1] != phase:
            order.append(phase)
    assert order[:4] == [PHASE_RAMP_UP, PHASE_SETTLE, PHASE_EXCITE, PHASE_RAMP_DOWN]

    ramp = [pulse for phase, pulse in record if phase == PHASE_RAMP_UP]
    assert len(ramp) == pytest.approx(RAMP_TICKS, abs=2)
    assert ramp[0] <= IDLE_US + 2
    assert all(b >= a for a, b in zip(ramp, ramp[1:])), "升油门必须单调"
    held = [pulse for phase, pulse in record if phase in (PHASE_SETTLE, PHASE_EXCITE)]
    assert set(held) == {target_us(lib, TARGET_N)}
    down = [pulse for phase, pulse in record if phase == PHASE_RAMP_DOWN]
    assert all(b <= a for a, b in zip(down, down[1:])), "降油门必须单调"
    assert len(down) == pytest.approx(1000 // 2, abs=2)
    assert max(a - b for a, b in zip(down, down[1:])) <= (down[0] - IDLE_US) // 500 + 1,         "回落必须是 1 s 线性斜坡，不是一拍掉到怠速"
    assert all(p is None or p <= CAP_US_75 for _ph, p in record)

    lines = texts(lib)
    phases = [line.split("phase=")[1].split()[0] for line in lines if line.startswith("SYSID PHASE")]
    assert phases == ["ramp_up", "settle", "excite", "ramp_down"]
    assert any(line.startswith("SYSID end") and "state=done reason=complete" in line
               for line in lines)
    sent = frames(lib)
    assert sent, "激励段的样本必须发出去"
    last = batch_header(sent[-1][1])
    assert last["flags"] & FLAG_LAST and not last["flags"] & FLAG_ABORTED


def test_the_throttle_cap_wins_over_the_thrust_target_and_is_reported(lib):
    start_auto(lib, target_n=15.0, max_pct=40.0)
    cap = round(1100 + 40 * 8.4)
    assert target_us(lib, 15.0) > cap, "用例前提：目标推力要超过封顶"
    record = []
    run_auto(lib, RAMP_TICKS + 20, record=record)
    assert max(p for _ph, p in record if p is not None) == cap
    lib.harness_reset()
    lib.APP_SysId_ReportStatus()
    thr = next(line for line in texts(lib) if line.startswith("SYSID THR"))
    assert "auto=1" in thr and "capped=1" in thr and "target_cn=1500" in thr
    lib.APP_SysId_Stop(b"test")


# ---------------------------------------------------------------- 交还


@pytest.mark.parametrize("tick", [100, RAMP_TICKS + SETTLE_TICKS + 20])
def test_pushing_the_stick_up_hands_the_motors_back_on_the_same_tick(lib, tick):
    start_auto(lib)

    def mutate(index, obs):
        if index >= tick:
            obs.rc_throttle_low = 0

    assert run_auto(lib, 4000, mutate=mutate) == tick
    assert lib.APP_SysId_GetState() == STATE_ABORTED
    assert lib.APP_SysId_GetLastReason() == b"rc_throttle_override"
    assert motor(lib) is None


def test_disarm_and_stop_release_the_motors(lib):
    start_auto(lib)

    def mutate(index, obs):
        if index >= 200:
            obs.rc_armed = 0

    assert run_auto(lib, 4000, mutate=mutate) == 200
    assert lib.APP_SysId_GetLastReason() == b"rc_disarm"
    assert motor(lib) is None

    start_auto(lib)
    run_auto(lib, 50)
    assert motor(lib) is not None
    lib.APP_SysId_Stop(b"command")
    assert motor(lib) is None
    assert lib.APP_SysId_GetLastReason() == b"command"


def test_stopping_during_ramp_down_keeps_the_completed_data(lib):
    start_auto(lib)
    record = []

    def until_ramp_down():
        for index in range(4000):
            obs = idle_obs(1_000 + index * 2, 1_000_000 + index * CONTROL_DT_US)
            lib.APP_SysId_Update(ctypes.byref(obs))
            lib.APP_SysId_StreamTick()
            record.append(lib.APP_SysId_GetPhase())
            if record[-1] == PHASE_RAMP_DOWN:
                return
        raise AssertionError("未进入回落阶段")

    until_ramp_down()
    lib.APP_SysId_Stop(b"command")
    assert lib.APP_SysId_GetState() == STATE_DONE
    assert lib.APP_SysId_GetLastReason() == b"complete"
    assert motor(lib) is None


# ---------------------------------------------------------------- 节拍


def test_a_control_dt_abort_reports_the_offending_interval(lib):
    reset(lib)
    assert lib.APP_SysId_Start() == 1
    lib.harness_reset()
    lib.APP_SysId_Update(ctypes.byref(healthy(1000, 1_000_000)))
    lib.APP_SysId_Update(ctypes.byref(healthy(1000, 1_000_100)))
    assert lib.APP_SysId_GetLastReason() == b"control_dt"
    for _ in range(8):
        lib.APP_SysId_StreamTick()
    lines = texts(lib)
    assert any(line.startswith("SYSID DT") and "dt_us=100" in line for line in lines)
    assert lines.index(next(l for l in lines if l.startswith("SYSID DT"))) < \
        lines.index(next(l for l in lines if l.startswith("SYSID end")))


def test_angle_mode_holds_the_natural_hanging_angle_instead_of_pulling_to_zero(lib):
    """质心在杆下方时机体自然下垂到一个非零平衡角；ANGLE 以激励开始时的角为零点。"""
    lib.APP_SysId_SetMode.argtypes = [ctypes.c_int, ctypes.c_float]
    reset(lib)
    centre_a, centre_b = ctypes.c_uint16(), ctypes.c_uint16()
    lib.APP_SysId_GetServoTargets(ctypes.byref(centre_a), ctypes.byref(centre_b))
    assert lib.APP_SysId_SetMode(2, ctypes.c_float(0.05)) == 1
    assert lib.APP_SysId_Start() == 1
    for index in range(2):
        lib.APP_SysId_Update(ctypes.byref(healthy(1000 + 2 * index, 1_000_000 + 2000 * index,
                                                  roll_rad=0.1, pitch_rad=0.1)))
    alpha, beta = ctypes.c_uint16(), ctypes.c_uint16()
    lib.APP_SysId_GetServoTargets(ctypes.byref(alpha), ctypes.byref(beta))
    assert abs(alpha.value - centre_a.value) <= 3
    assert abs(beta.value - centre_b.value) <= 3
    lib.APP_SysId_Stop(b"test")
    assert lib.APP_SysId_SetMode(0, ctypes.c_float(0.0523598776)) == 1


# ---------------------------------------------------------------- 根因：调度器


SCHED_HARNESS = r"""
#include "app_control_scheduler.h"
/* 按给定的唤醒时刻跑稳定环的节拍判定；reset_each=1 复刻辨识占用时每拍清调度器。 */
int simulate(const unsigned long long *wake_us, int n, int reset_each,
             unsigned long long *executed_us)
{
    APP_ControlSchedulerState state;
    APP_ControlSchedule schedule;
    int count = 0;
    APP_ControlScheduler_Reset(&state);
    for (int i = 0; i < n; ++i) {
        APP_ControlScheduler_Step(&state, wake_us[i], 0ULL, 0U, &schedule);
        if (schedule.rate_due) {
            APP_ControlScheduler_Commit(&state, wake_us[i], 0ULL, &schedule);
            executed_us[count++] = wake_us[i];
            if (reset_each) { APP_ControlScheduler_Reset(&state); }
        }
    }
    return count;
}
"""


@pytest.fixture(scope="module")
def sched(tmp_path_factory):
    compiler = shutil.which("gcc") or shutil.which("clang")
    if compiler is None:
        pytest.skip("host gcc/clang required")
    build = tmp_path_factory.mktemp("sched")
    harness = build / "sched_harness.c"
    harness.write_text(SCHED_HARNESS, encoding="utf-8")
    out = build / ("sched.dll" if os.name == "nt" else "sched.so")
    result = subprocess.run(
        [compiler, "-shared", "-fPIC", "-std=c11", "-O1", "-Wall", "-Wextra",
         "-I", str(ROOT / "App/Inc"), str(ROOT / "App/Src/app_control_scheduler.c"),
         str(harness), "-o", str(out)], capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr
    return ctypes.CDLL(str(out))


def _intervals(sched, wakes, reset_each):
    n = len(wakes)
    wake_arr = (ctypes.c_ulonglong * n)(*wakes)
    out = (ctypes.c_ulonglong * n)()
    count = sched.simulate(wake_arr, n, reset_each, out)
    times = list(out)[:count]
    return [b - a for a, b in zip(times, times[1:])]


def test_resetting_the_scheduler_every_tick_is_what_produced_sub_half_ms_ticks(sched):
    # 1 ms 信号量超时唤醒 + 偶尔紧跟着一次 IMU 唤醒（间隔 80 us）。
    wakes = []
    t = 1_000_000
    for k in range(400):
        wakes.append(t)
        if k % 37 == 5:
            wakes.append(t + 80)
        t += 1000
    broken = _intervals(sched, wakes, 1)
    fixed = _intervals(sched, wakes, 0)
    assert min(broken) < 500, "复刻不出根因说明模型不对"
    assert min(fixed) >= 2000


def test_the_stabilizer_keeps_the_schedule_while_the_rod_rig_owns_the_servos():
    source = strip_c_comments((ROOT / "App/Src/app_stabilizer.c").read_text(encoding="utf-8"))
    branch = source.split("if (frame->ident_running != 0U) {", 1)[1].split("} else if", 1)[0]
    assert "if (frame->sysid_running == 0U) {" in branch
    guarded = branch.split("if (frame->sysid_running == 0U) {", 1)[1].split("}", 1)[0]
    assert "APP_ControlScheduler_Reset(&ctx->control_scheduler);" in guarded
    assert branch.count("APP_ControlScheduler_Reset(") == 1


# ---------------------------------------------------------------- 审查补充（2026-09-27）


def _until_ramp_down(lib):
    for index in range(4000):
        lib.APP_SysId_Update(ctypes.byref(idle_obs(1_000 + index * 2,
                                                   1_000_000 + index * CONTROL_DT_US)))
        lib.APP_SysId_StreamTick()
        if lib.APP_SysId_GetPhase() == PHASE_RAMP_DOWN:
            return index
    raise AssertionError("未进入回落阶段")


def test_stop_racing_the_last_excite_tick_does_not_announce_ramp_down(lib):
    """STOP 落在激励末条入环之后、转入回落之前（实机上 SysTick 在临界区出口被响应）：
    以 STOP 为准。不能先报 phase=ramp_down（"数据已完整"）再报 end state=aborted。"""
    start_auto(lib)
    last = _until_ramp_down(lib)                       # 参考轮：转入回落的那一拍
    lib.APP_SysId_Stop(b"test")
    start_auto(lib)
    assert run_auto(lib, last) == last
    lib.harness_reset()
    lib.harness_set_stop_after_critical(1)
    lib.APP_SysId_Update(ctypes.byref(idle_obs(1_000 + last * 2, 1_000_000 + last * CONTROL_DT_US)))
    lib.harness_set_stop_after_critical(0)
    for _ in range(8):
        lib.APP_SysId_StreamTick()
    lines = texts(lib)
    assert (lib.APP_SysId_GetState(), lib.APP_SysId_GetLastReason()) == (STATE_ABORTED, b"command")
    assert not any("phase=ramp_down" in line for line in lines), lines
    assert any(line.startswith("SYSID end") and "state=aborted reason=command" in line
               for line in lines)
    assert motor(lib) is None


@pytest.mark.parametrize("fault", ["rc_throttle_low", "rc_armed", "rc_link_ok", "dt"])
def test_any_gate_during_ramp_down_keeps_the_completed_data(lib, fault):
    start_auto(lib)
    index = _until_ramp_down(lib)
    for _ in range(50):
        index += 1
        lib.APP_SysId_Update(ctypes.byref(idle_obs(1_000 + index * 2, 1_000_000 + index * CONTROL_DT_US)))
        lib.APP_SysId_StreamTick()
    lib.harness_reset()
    t_us = 1_000_000 + (index + 1) * CONTROL_DT_US
    obs = idle_obs(1_000 + (index + 1) * 2, t_us)
    if fault == "dt":
        obs.now_us = 1_000_000 + index * CONTROL_DT_US + 100
    else:
        setattr(obs, fault, 0)
    lib.APP_SysId_Update(ctypes.byref(obs))
    for _ in range(8):
        lib.APP_SysId_StreamTick()
    assert lib.APP_SysId_GetState() == STATE_DONE
    assert lib.APP_SysId_GetLastReason() == b"complete"
    assert motor(lib) is None
    assert any(line.startswith("SYSID end") and "state=done reason=complete" in line
               for line in texts(lib))


def test_manual_mode_never_takes_the_motors(lib):
    reset(lib)                                   # target_n=0：遥控器给油门
    prime(lib)
    assert lib.APP_SysId_Start() == 1
    for index in range(300):
        obs = healthy(1_000 + index * 2, 1_000_000 + index * CONTROL_DT_US)
        obs.throttle_us = 1600 if index < 100 else 1500      # 飞手在辨识中动杆
        obs.rc_throttle_low = 0                              # 手动模式没有推杆交还门
        lib.APP_SysId_Update(ctypes.byref(obs))
        if lib.APP_SysId_IsRunning() == 0:
            break
        assert motor(lib) is None
    assert lib.APP_SysId_GetLastReason() != b"rc_throttle_override"
    lib.harness_reset()
    lib.APP_SysId_ReportThrottle()
    assert texts(lib) and texts(lib)[0].startswith("SYSID THR auto=0 ")


def test_stop_preempting_an_update_never_leaves_the_servos_deflected(lib):
    """STOP 插进 Update 中间（写完倾角、入环之前）：本拍取到的舵机目标必须是中位，
    也不能在尾批之后再多压一条样本。"""
    centre_a, centre_b = ctypes.c_uint16(), ctypes.c_uint16()
    reset(lib)
    lib.APP_SysId_GetServoTargets(ctypes.byref(centre_a), ctypes.byref(centre_b))
    start_auto(lib)
    index = 0
    while lib.APP_SysId_GetPhase() != PHASE_EXCITE:
        lib.APP_SysId_Update(ctypes.byref(idle_obs(1_000 + index * 2, 1_000_000 + index * CONTROL_DT_US)))
        lib.APP_SysId_StreamTick()
        index += 1
    for _ in range(40):      # 进到斜坡中段，舵机已经偏离中位
        lib.APP_SysId_Update(ctypes.byref(idle_obs(1_000 + index * 2, 1_000_000 + index * CONTROL_DT_US)))
        index += 1
    lib.harness_set_stop_on_critical(1)          # 下一次入环前的临界区：STOP 抢占
    before = None
    for _ in range(4):                           # 250 Hz 采样隔拍入环
        head_before = lib.APP_SysId_GetDroppedSamples()
        lib.APP_SysId_Update(ctypes.byref(idle_obs(1_000 + index * 2, 1_000_000 + index * CONTROL_DT_US)))
        index += 1
        if lib.APP_SysId_GetState() != 1:
            before = head_before
            break
    assert lib.APP_SysId_GetState() == STATE_ABORTED
    alpha, beta = ctypes.c_uint16(), ctypes.c_uint16()
    lib.APP_SysId_GetServoTargets(ctypes.byref(alpha), ctypes.byref(beta))
    assert (alpha.value, beta.value) == (centre_a.value, centre_b.value)
    assert motor(lib) is None
    assert lib.APP_SysId_GetDroppedSamples() == before
    lib.harness_reset()
    for _ in range(8):
        lib.APP_SysId_StreamTick()
    # 抢占那一拍的样本没进环：尾批仍以 STOP 前最后一条入环样本收尾
    sent = frames(lib)
    assert sent and batch_header(sent[-1][1])["flags"] & FLAG_LAST


def test_jittery_control_ticks_without_drops_never_flag_a_gap(lib):
    """修好节拍后控制拍在 2~3 ms 之间抖；250 Hz 采样最坏间隔约 6 ms。没丢样就不许报断点。"""
    reset(lib, rate_hz=250, spec=Excitation(profile=PROFILE_DOUBLET, amplitude_rad_s=0.15,
                                           duration_ms=4000, hold_ms=250, repeat=2, ramp_ms=150,
                                           chirp_f0_hz=0.3, chirp_f1_hz=6.0, prbs_bit_ms=40,
                                           prbs_seed=1))
    assert lib.APP_SysId_Start() == 1
    lib.harness_reset()
    t_us, t_ms = 1_000_000, 1_000
    for index in range(3000):
        step = 2000 if index % 3 else 3050
        t_us += step
        t_ms = t_us // 1000
        lib.APP_SysId_Update(ctypes.byref(healthy(t_ms, t_us)))
        lib.APP_SysId_StreamTick()
        if lib.APP_SysId_IsRunning() == 0:
            break
    for _ in range(8):
        lib.APP_SysId_StreamTick()
    assert lib.APP_SysId_GetState() == STATE_DONE
    headers = [batch_header(payload) for _fn, payload in frames(lib)]
    assert headers and not any(h["flags"] & 0x10 for h in headers)
    assert lib.APP_SysId_GetDroppedSamples() == 0


def test_the_thrust_lut_never_publishes_a_transient_stale_flag():
    """查表的"新鲜"标志只在真过期时从 1 变 0：开头先清 0 再末尾置 1 会让抢占的控制拍
    读到瞬时 0，光杆辨识以 thrust_stale 中止。"""
    source = strip_c_comments((ROOT / "App/Src/app_thrust_lut.c").read_text(encoding="utf-8"))
    body = source.split("void APP_ThrustLut_Step(void)", 1)[1].split("\n}\n", 1)[0]
    before_checks = body.split("if ((battery.state.valid == 0U)", 1)[0]
    assert "thrust_lut_fresh = 0U" not in before_checks
    assert body.count("thrust_lut_fresh = 0U") == 2          # 两个真过期分支
    assert "volatile uint8_t thrust_lut_fresh" in source


def test_sysid_param_is_ram_only_and_limited_to_controller_gains():
    """「临时应用到飞控（RAM）」走 SYSID PARAM：不能排 Flash 自动保存（普通 PARAM SET
    成功后 1.5 s 会存盘），只认控制增益，回显带 ram=1 让上位机能 fail-closed。

    2026-09-28：写入改走试用记录（APP_ParamTrial_Apply），别处触发的保存也存试用前的
    值；不许再走 PARAM SET 的写入口 app_control_param_set_any——那是显式持久写，会结束
    试用。行为由 tests/test_param_trial.py 在真 C 上验。"""
    source = strip_c_comments((ROOT / "App/Src/app_cmd_sysid.c").read_text(encoding="utf-8"))
    branch = source.split('strcmp(sub, "PARAM") == 0', 1)[1].split('strcmp(sub, "THR?") == 0', 1)[0]
    assert "APP_ParamTrial_Apply(" in branch
    assert "app_control_param_set_any(" not in branch
    assert "schedule_flash_autosave" not in branch and "Save" not in branch
    assert '"coax.rate_"' in branch and '"coax.att_"' in branch
    assert "ram=1" in branch
    assert "%f" not in branch and "%g" not in branch, "newlib-nano 没有浮点 printf"


def test_a_zero_excitation_preroll_precedes_the_excitation_without_a_gap(lib):
    """稳定段最后 0.5 s 照常采样（力矩 0）：拟合要丢掉开头约 0.5 s 的起始瞬态，
    让它落在前导上而不是吃掉激励。前导与激励同一采样网格、时间连续、不报断点。"""
    from tools.sysid.decode import parse_schema_lines, decode_batch
    lib.harness_reset()
    lib.APP_SysId_ReportSchema()
    schema = parse_schema_lines(texts(lib))
    start_auto(lib)
    lib.harness_reset()
    run_auto(lib, 4000)
    for _ in range(8):
        lib.APP_SysId_StreamTick()
    batches = [decode_batch(payload, schema) for _fn, payload in frames(lib)]
    assert batches and batches[0].first and batches[-1].last
    assert not any(b.gap for b in batches)
    samples = [s for b in batches for s in b.samples]
    stamps = [t for b in batches for t in b.timestamps_us()]
    steps = [b - a for a, b in zip(stamps, stamps[1:])]
    assert max(steps) <= 2 * 4000 + 2000
    # 前导：开头约 0.5 s 力矩与期望角速度都是 0，随后才出现激励
    first_excite = next(i for i, s in enumerate(samples) if abs(s["torque"]) > 1e-3)
    assert (stamps[first_excite] - stamps[0]) >= 450_000
    assert all(abs(s["omega_sp"]) < 1e-6 for s in samples[:first_excite])

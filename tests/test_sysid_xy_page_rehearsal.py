"""XY 速度 / 位置环页上实物台架前的流程彩排（2026-09-30）。

和 `test_sysid_xy_page.py` 的区别：这里的**假飞控回复文本逐字取自固件源码的格式串**，不照抄页面测试里
页面作者自己写的那些。出处（行号以 2026-09-30 工作树为准，改固件要回来核对）：

* `SYSID READY / state / RIG / EXC / THR / LIMITS` 整份报告：`App/Src/app_sysid.c` `APP_SysId_ReportStatus`
  与 `APP_SysId_ReportThrottle`（THR 行末尾追加 `xy_pos_mm xy_vel_mms xy_ok`）。
* `SYSID XY ...` 回显与 `event=rejected reason=running|range|usage`：`app_cmd_sysid.c::sysid_cmd_xy`、
  `app_sysid_xy.c::APP_SysIdXy_ReportConfig`。
* `ERR sysid xy <理由>`：`app_sysid_xy.c::APP_SysIdXy_Precheck` 的返回串，原文照抄。
* `SYSID start ...` / `SYSID XYSTART ...`：`app_sysid.c::APP_SysId_Start`、`app_sysid_xy.c::APP_SysIdXy_ReportStart`
  （`%ld` 全部是 lroundf；START 行里的 psi_mrad 是 C 截断、XYSTART 的是四舍五入）。
* `SYSID PHASE ...` / `SYSID end ...`：`app_sysid.c::APP_SysId_StreamTick`。
* `HOVER ...`：`app_cmd_hover.c`（`HOVER state=not_ready` 与三位小数的 est_n）。
* `PARAM name=.. value=..`：`app_control.c::app_control_format_float`（六位小数）。

固件里的 float32 运算用 `f32()` 还原，这样「页面 Python 双精度取整」与「固件 float 取整」的差别才测得出来。
"""
from __future__ import annotations

import json
import math
import re
import struct
import sys
from pathlib import Path

import pytest

from test_sysid_altitude_mode import ALT_PARAMS, SCHEMA_ALT_LINES, no_fit  # noqa: F401
from test_sysid_page import (  # noqa: F401  页面夹具
    FLAG_FIRST_BATCH, FLAG_LAST_BATCH, FakePanel, page, wait_for,
)
from test_sysid_xy_page import step_series, tilt_series, to_raw, xy_frame  # noqa: F401

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from panel_lib.pages.sysid import xy_config  # noqa: E402

RUN_ID = 7
PROFILES = ("step", "doublet", "chirp", "prbs")


def f32(value: float) -> float:
    return struct.unpack("<f", struct.pack("<f", value))[0]


def lroundf(value: float) -> int:
    """C 的 lroundf：半数远离零。"""
    return int(math.copysign(math.floor(abs(value) + 0.5), value))


def ctrunc(value: float) -> int:
    """C 的 (long) 强转：向零截断。"""
    return int(value)


DEG_TO_RAD = f32(0.01745329252)
PARAMS = ALT_PARAMS + (("coax.hover_thrust_n", "13.500000"),
                       ("airframe.max_total_force_n", "30.000000"))


def param_value(text: str) -> str:
    return f"{float(text):.6f}"


# ---------------------------------------------------------------- 假飞控（回复格式取自固件）


class FakeFirmware:
    """只实现页面会发的那些命令；每条命令返回固件会回的行（按发送顺序）。"""

    def __init__(self, *, armed=1, thr_low=1, flow_ok=True, tof_ok=True, airframe_mass_kg=1.1453,
                 max_force_n=30.0, ixx=0.02, hover=None):
        self.armed, self.thr_low, self.flow_ok, self.tof_ok = armed, thr_low, flow_ok, tof_ok
        self.airframe_mass_kg, self.max_force_n, self.ixx = airframe_mass_kg, max_force_n, ixx
        self.hover = hover                      # None = 估计器未就绪；否则 (est_n, std_n, converged)
        self.mode = 0
        self.angle_amp_rad = f32(3.0 * 0.01745329252)
        self.rate_hz, self.inertia = 250, 0.0
        self.angle_limit_rad = f32(20 * 0.01745329252)
        self.resid_limit_rad_s = f32(30 * 0.01745329252)
        self.exc = dict(profile=1, amp=f32(0.3), dur=4000, hold=250, repeat=8, ramp=150, f0=f32(0.3),
                        f1=f32(6.0), bit=40, seed=1)
        self.servo_tilt = 0.0
        self.psi, self.axis_off, self.imu_off = f32(0.785398), 0.0, 0.0
        self.target_n, self.max_pct = 0.0, 75.0
        self.xy = dict(inject="tilt", win_mm=150, mass_g=0)
        self.state, self.run_id, self.phase, self.reason = "idle", 0, "idle", "init"
        self.xy_pos_m, self.xy_vel_m_s = 0.0, 0.0
        self.x0_m = self.y0_m = 0.0
        self.live_xy = (0.012, -0.007)
        self.log: list[str] = []
        self.param_dump_size = 0

    # ---- 报告
    def thr_line(self) -> str:
        return (f"SYSID THR auto={int(self.target_n > 0.0)} "
                f"target_cn={ctrunc(f32(self.target_n) * 100.0 + 0.5)} "
                f"max_pct_x10={ctrunc(f32(self.max_pct) * 10.0 + 0.5)} phase={self.phase} pulse_us=0 "
                f"armed={self.armed} thr_low={self.thr_low} capped=0 alt_h_mm=0 alt_h_ok=0 alt_sp_mm=0 "
                f"xy_pos_mm={lroundf(self.xy_pos_m * 1000.0)} xy_vel_mms={lroundf(self.xy_vel_m_s * 1000.0)} "
                f"xy_ok={int(self.flow_ok and self.tof_ok)}")

    def report(self) -> list[str]:
        e = self.exc
        total = min(e["hold"] * 2 * e["repeat"], e["dur"]) if e["profile"] == 1 else e["dur"]
        return [
            f"SYSID READY ver=3 mode={self.mode} angle_amp_mrad={ctrunc(self.angle_amp_rad * 1000.0)} "
            f"mass_mg={ctrunc(f32(self.airframe_mass_kg) * 1000000.0)} imu_off_um=35000 thrust=lut engaged=0",
            f"SYSID state={self.state} run={self.run_id} reason={self.reason} queued=0 dropped=0",
            f"SYSID RIG psi_mrad={ctrunc(self.psi * 1000.0)} axis_off_um={ctrunc(f32(self.axis_off) * 1000000.0)} "
            f"imu_off_um={ctrunc(f32(self.imu_off) * 1000000.0)}",
            f"SYSID EXC profile={e['profile']} amp_mrad_s={ctrunc(e['amp'] * 1000.0)} dur_ms={e['dur']} "
            f"hold_ms={e['hold']} repeat={e['repeat']} ramp_ms={e['ramp']} f0_mhz={ctrunc(e['f0'] * 1000.0)} "
            f"f1_mhz={ctrunc(e['f1'] * 1000.0)} bit_ms={e['bit']} seed={e['seed']} total_ms={total} "
            f"servo_tilt_mrad={ctrunc(self.servo_tilt * 1000.0 + 0.5)}",
            self.thr_line(),
            f"SYSID LIMITS rate_hz={self.rate_hz} I_ugm2={ctrunc(self.inertia * 1000000.0)} "
            f"angle_mrad={ctrunc(self.angle_limit_rad * 1000.0)} "
            f"resid_mrad_s={ctrunc(self.resid_limit_rad_s * 1000.0)} min_thrust_cn=200",
        ]

    def xy_echo(self) -> str:
        control = "openloop" if self.xy["inject"] == "tilt" else "closed_loop"
        return (f"SYSID XY inject={self.xy['inject']} win_mm={self.xy['win_mm']} "
                f"mass_g={self.xy['mass_g']} control={control}")

    # ---- 命令
    def handle(self, line: str) -> list[str]:
        self.log.append(line)
        tokens = line.split()
        if line == "SYSID SCHEMA":
            return list(SCHEMA_ALT_LINES)
        if line == "PARAM?":
            return [f"PARAM name={name} value={param_value(value)}" for name, value in self.params()]
        if line == "HOVER?":
            if self.hover is None:
                return ["HOVER state=not_ready"]
            est, std, conv = self.hover
            return [f"HOVER est_n={est:.3f} std_n={std:.3f} converged={conv} learning=1 samples=812 "
                    f"rejected=3 innov=0.012 init_n=13.500 meas_std=0.350 gate=0x3f airborne=1 above_m=0.080"]
        if tokens[:2] == ["SYSID", "THR?"]:
            return [self.thr_line()]
        if tokens[:2] == ["SYSID", "XY"]:
            return [self.cmd_xy(tokens)]
        if tokens[:2] == ["SYSID", "MODE"]:
            return self.cmd_mode(tokens)
        if tokens[:2] == ["SYSID", "EXC"]:
            return self.cmd_exc(tokens)
        if tokens[:2] == ["SYSID", "RATE"]:
            self.rate_hz = int(tokens[2])
            return self.report()
        if tokens[:2] == ["SYSID", "INERTIA"]:
            self.inertia = float(tokens[2])
            return self.report()
        if tokens[:2] == ["SYSID", "LIMIT"]:
            for token in tokens[2:]:
                key, value = token.split("=")
                if key == "angle_deg":
                    self.angle_limit_rad = f32(float(value) * DEG_TO_RAD)
                elif key == "resid_dps":
                    self.resid_limit_rad_s = f32(float(value) * DEG_TO_RAD)
            return self.report()
        if tokens[:2] == ["SYSID", "RIG"]:
            return self.cmd_rig(tokens)
        if tokens[:2] == ["SYSID", "THROTTLE"]:
            return self.cmd_throttle(tokens)
        if tokens[:2] == ["SYSID", "START"]:
            return self.cmd_start()
        if tokens[:2] == ["SYSID", "STOP"]:
            if self.state == "running":
                self.state, self.reason = "aborted", "command"
            return self.report()
        if line in ("SYSID?", "SYSID STATUS"):
            return self.report()
        return [f"ERR unknown sysid subcmd {tokens[1] if len(tokens) > 1 else ''}"]

    def params(self):
        table = dict(PARAMS)
        table["airframe.mass_kg"] = f"{self.airframe_mass_kg:.6f}"
        table["airframe.max_total_force_n"] = f"{self.max_force_n:.6f}"
        return list(table.items())

    def cmd_xy(self, tokens):
        config = dict(self.xy)
        reason = None
        for token in tokens[2:]:
            if token.startswith("inject="):
                if token[7:] not in ("tilt", "vel", "pos"):
                    reason = "usage"
                    break
                config["inject"] = token[7:]
            elif token.startswith(("win_mm=", "mass_g=")):
                key, value = token.split("=")
                if not value.isdigit():
                    reason = "usage"
                    break
                if int(value) > 65535:
                    reason = "range"
                    break
                config[key] = int(value)
            else:
                reason = "usage"
                break
        if reason is None and len(tokens) > 2:
            if self.state == "running":
                reason = "running"
            elif not (30 <= config["win_mm"] <= 400 and (config["mass_g"] == 0 or 500 <= config["mass_g"] <= 3000)):
                reason = "range"
            else:
                self.xy = config
        if reason:
            return f"SYSID XY event=rejected reason={reason}"
        return self.xy_echo()

    def cmd_mode(self, tokens):
        names = {"FF": 0, "RATE": 1, "ANGLE": 2, "SERVO": 3, "ALT": 4, "XY": 5}
        mode = names.get(tokens[2], int(tokens[2]) if tokens[2].isdigit() else 99)
        angle = float(tokens[3]) if len(tokens) > 3 else 3.0
        if mode > 5 or self.state == "running" or not 0 < angle <= 15:
            return ["ERR sysid mode: MODE FF|RATE|ANGLE|SERVO|ALT|XY [0<deg<=15], idle only"]
        self.mode, self.angle_amp_rad = mode, f32(angle * 0.01745329252)
        return self.report()

    def cmd_exc(self, tokens):
        spec = dict(self.exc)
        for token in tokens[2:]:
            key, value = token.split("=")
            if key == "profile":
                spec["profile"] = PROFILES.index(value)
            elif key == "amp":
                spec["amp"] = f32(float(value))
            elif key == "f0":
                spec["f0"] = f32(float(value))
            elif key == "f1":
                spec["f1"] = f32(float(value))
            elif key == "dur_ms":
                spec["dur"] = int(value)
            else:
                spec[{"hold_ms": "hold", "repeat": "repeat", "ramp_ms": "ramp", "bit_ms": "bit",
                      "seed": "seed"}[key]] = int(value)
        if not (0.0 < spec["amp"] <= 5.0 and spec["ramp"] >= 5 and 0 < spec["dur"] <= 30000
                and (spec["profile"] != 1 or (spec["hold"] >= spec["ramp"] and 0 < spec["repeat"] <= 20))):
            return ["ERR sysid excitation rejected"]
        self.exc = spec
        return self.report()

    def cmd_rig(self, tokens):
        for token in tokens[2:]:
            key, value = token.split("=")
            if key == "psi_deg":
                self.psi = f32(f32(float(value)) * DEG_TO_RAD)
            elif key == "axis_off_m":
                self.axis_off = f32(float(value))
            elif key == "imu_off_m":
                self.imu_off = f32(float(value))
        return self.report()

    def cmd_throttle(self, tokens):
        target, pct = self.target_n, self.max_pct
        for token in tokens[2:]:
            key, value = token.split("=")
            if key == "target_n":
                target = f32(float(value))
            elif key == "max_pct":
                pct = f32(float(value))
        if self.state == "running" or not ((target == 0.0 or 2.0 <= target <= self.max_force_n)
                                           and 10.0 <= pct <= 95.0):
            return ["ERR sysid throttle: target_n=0 (manual) or 2..max_total_force_n, "
                    "max_pct 10..95, idle only"]
        self.target_n, self.max_pct = target, pct
        return self.report()

    # ---- 开跑前检（逐条照 APP_SysIdXy_Precheck）
    def xy_precheck(self):
        if not self.target_n > 0.0:
            return "needs auto throttle (SYSID THROTTLE target_n>0)"
        if self.target_n > self.max_force_n:
            return "target over max thrust"
        amp = self.exc["amp"]
        limits = {"tilt": (0.10, "amp over limit: inject=tilt amp<=0.10 rad"),
                  "vel": (0.30, "amp over limit: inject=vel amp<=0.3 m/s"),
                  "pos": (0.15, "amp over limit: inject=pos amp<=0.15 m")}
        limit, text = limits[self.xy["inject"]]
        if not (amp > 0.0) or amp > f32(limit):
            return text
        if self.xy["inject"] == "pos" and amp > f32(0.7 * self.xy["win_mm"] * 0.001):
            return "amp over window: inject=pos amp<=0.7*win"
        mass = self.xy["mass_g"] / 1000.0 if self.xy["mass_g"] else self.airframe_mass_kg
        if not mass > 0.0:
            return "mass unknown (set SYSID XY mass_g= or airframe mass)"
        if not self.flow_ok:
            return "flow invalid"
        if not self.tof_ok:
            return "height invalid (TOF)"
        return None

    def cmd_start(self):
        if self.state == "running":
            return ["ERR sysid already running"]
        if self.target_n > 0.0:
            if self.armed == 0:
                return ["ERR sysid not armed: arm with throttle stick low first"]
            if self.thr_low == 0:
                return ["ERR sysid throttle stick not low"]
        if self.mode == 5:
            refusal = self.xy_precheck()
            if refusal:
                return [f"ERR sysid xy {refusal}"]
        if self.inertia <= 0.0:
            self.inertia = self.ixx
        self.run_id += 1
        self.state, self.reason, self.phase = "running", "running", "idle"
        self.x0_m, self.y0_m = self.live_xy
        e = self.exc
        total = min(e["hold"] * 2 * e["repeat"], e["dur"]) if e["profile"] == 1 else e["dur"]
        lines = [
            f"SYSID start run={self.run_id} profile={e['profile']} amp_mrad_s={ctrunc(e['amp'] * 1000.0)} "
            f"dur_ms={total} rate_hz={self.rate_hz} I={ctrunc(self.inertia * 1000000.0)} ugm2 "
            f"psi_mrad={ctrunc(self.psi * 1000.0)} auto={int(self.target_n > 0.0)} "
            f"target_cn={lroundf(f32(self.target_n * 100.0))} ref_wr_mrad_s=0 ref_td_us=0 onotch_mhz=0 "
            f"onotch_q_milli=0 onotch2_mhz=0 onotch2_q_milli=0"]
        if self.mode == 5:
            mass_kg = f32(self.xy["mass_g"] / 1000.0) if self.xy["mass_g"] else f32(self.airframe_mass_kg)
            lines.append(
                f"SYSID XYSTART run={self.run_id} xy_inject={self.xy['inject']} xy_win_mm={self.xy['win_mm']} "
                f"xy_mass_g={lroundf(f32(mass_kg * 1000.0))} xy_psi_mrad={lroundf(f32(self.psi * 1000.0))} "
                f"xy_target_cn={lroundf(f32(f32(self.target_n) * 100.0))} "
                f"xy_x0_mm={lroundf(self.x0_m * 1000.0)} xy_y0_mm={lroundf(self.y0_m * 1000.0)}")
        lines.append(f"SYSID NOTCH run={self.run_id} en=0 state=off src=none pp=7 harm=1 q_x100=200 "
                     f"min_hz=30 fade_hz=20 fs_x10=40000")
        lines.append(f"SYSID BACKLASH run={self.run_id} en=0 alpha_mrad=0 beta_mrad=0 thr_mrad=0 servo_hz=333")
        return lines

    def phase_line(self, phase: str, thrust_cn=1350) -> str:
        self.phase = phase
        return f"SYSID PHASE run={self.run_id} phase={phase} pulse_us=1400 thrust_cn={thrust_cn}"

    def end_line(self, state="done", reason="complete", dropped=0) -> str:
        self.state, self.reason, self.phase = state, reason, "idle"
        return f"SYSID end run={self.run_id} state={state} reason={reason} dropped={dropped}"


# ---------------------------------------------------------------- 驱动


@pytest.fixture(autouse=True)
def _mount_xy_view(request):
    if "page" not in request.fixturenames:
        return
    engine = request.getfixturevalue("page")
    from panel_lib.pages.sysid.horizontal import SysIdHorizontalPage
    from tkinter import ttk
    engine.test_xy_view = SysIdHorizontalPage(engine.panel,
                                              ttk.Frame(engine.parent.winfo_toplevel()), engine)


class Bench:
    """把页面发出的每一行喂给假飞控，回复再喂回页面，直到安静。"""

    def __init__(self, page, fw):
        self.page, self.fw = page, fw
        self.cursor = len(page.panel.transport.lines)

    def settle(self, limit=400):
        transport = self.page.panel.transport
        for _ in range(limit):
            if self.cursor >= len(transport.lines):
                return
            line = transport.lines[self.cursor]
            self.cursor += 1
            for reply in self.fw.handle(line):
                self.page.handle_line(reply)
        raise AssertionError("命令来回没有收敛")

    def feed(self, *lines):
        for line in lines:
            self.page.handle_line(line)
        self.settle()

    def sent(self):
        return list(self.page.panel.transport.lines)


def bench_for(page, **kwargs):
    fw = FakeFirmware(**kwargs)
    bench = Bench(page, fw)
    # 面板开着的时候飞控早就把状态报告发过了（连上即读 SYSID?）：READY 里的质量等在这里进页面。
    bench.feed(*fw.report())
    return bench


def fill_bench_settings(page, *, target="", inject=None, win=None, extra=None, rod="0.15"):
    page.set_front_view("XY")
    if inject:
        page.xy_inject_var.set(inject)
        page.select_xy_inject()
    page.xy_target_var.set(target)
    if win is not None:
        page.xy_win_var.set(str(win))
    if extra is not None:
        page.xy_extra_mass_var.set(str(extra))
    page.rod_to_fc_var.set(rod)
    page.roll_pivot_var.set("-0.2")
    page.pitch_pivot_var.set("-0.2")


def start_to_running(bench, **settings):
    fill_bench_settings(bench.page, **settings)
    bench.page.start_xy_run()
    bench.settle()
    return bench


def push_samples(page, rows, run_id=RUN_ID, per=5, dt_us=4000, extra_flags=0):
    raw = to_raw(rows)
    for start in range(0, len(raw), per):
        flags = ((FLAG_FIRST_BATCH if start == 0 else 0) | (FLAG_LAST_BATCH if start + per >= len(raw) else 0)
                 | extra_flags)
        page.accept(xy_frame(run_id, raw[start:start + per], flags=flags, base_us=1000 + start * dt_us,
                             dt_us=dt_us))


def redirect_archive(monkeypatch, tmp_path):
    from panel_lib.pages.sysid import workflow
    root = tmp_path / "xy" / "2026-09-30"
    monkeypatch.setattr(workflow, "_xy_directory", lambda: root)
    no_fit(monkeypatch)
    return root


def conditions_of(page):
    assert wait_for(page, lambda: page.workflow.saved is not None and not page.workflow.saving)
    folder = page.workflow.saved
    return folder, json.loads((folder / "conditions.json").read_text(encoding="utf-8"))


# ---------------------------------------------------------------- 1. 悬停推力读取


def test_hover_not_ready_then_ready_then_used_in_the_run(page):
    bench = bench_for(page, hover=None)
    page.set_front_view("XY")
    page.read_hover_thrust()
    bench.settle()
    assert page.xy_target_var.get() == ""
    assert "还没就绪" in page.status_var.get()
    assert page._hover_pending is False
    bench.fw.hover = (11.2345, 0.080, 1)
    page.read_hover_thrust()
    bench.settle()
    assert page.xy_target_var.get() == "11.23"            # 固件 11.235 被页面 .2f 进一位后的值
    assert "已收敛" in page.status_var.get()
    # 「读取」填的值就是开跑时下发的托住推力；不碰飞控参数。
    start_to_running(bench, target=page.xy_target_var.get())
    assert "SYSID THROTTLE target_n=11.23 max_pct=90" in bench.sent()
    assert not any(line.startswith("PARAM SET") for line in bench.sent())


# ---------------------------------------------------------------- 2. 整轮正常流程


def drive_normal_run(page, bench, monkeypatch, tmp_path, *, inject="tilt", **settings):
    root = redirect_archive(monkeypatch, tmp_path)
    start_to_running(bench, inject=inject, **settings)
    fw = bench.fw
    assert fw.state == "running" and page.workflow.run_id == fw.run_id
    rows = (tilt_series()[1] if inject == "tilt" else step_series(kind=inject)[1])
    bench.feed(fw.phase_line("ramp_up"))
    bench.feed(fw.phase_line("settle"))
    bench.feed(fw.phase_line("preroll"))
    bench.feed(fw.phase_line("excite"))
    push_samples(page, rows, run_id=fw.run_id)
    bench.feed(fw.phase_line("ramp_down"))
    bench.feed(fw.end_line())
    return root


@pytest.mark.parametrize("inject", ["tilt", "vel", "pos"])
def test_normal_run_sends_the_expected_commands_and_archives_everything(page, monkeypatch, tmp_path, inject):
    bench = bench_for(page)
    root = drive_normal_run(page, bench, monkeypatch, tmp_path, inject=inject, target="13.5")
    sent = bench.sent()
    assert sent[0] == "SYSID SCHEMA" and sent[1] == "PARAM?"
    assert "SYSID MODE XY 3" in sent
    xy_at = sent.index(f"SYSID XY inject={inject} win_mm=150 mass_g=1184")
    assert sent.index("SYSID MODE XY 3") < xy_at < sent.index("SYSID THROTTLE target_n=13.5 max_pct=90")
    assert sent[-1] == "SYSID START"
    assert page.banner_var.get() == "完成", (page.banner_var.get(), page.status_var.get())
    assert not page.workflow.data_error
    folder, cond = conditions_of(page)
    assert folder.parent == root and folder.name.startswith("xy_")
    assert cond["mode"] == "5" and cond["end"]["state"] == "done" and cond["data_error"] == ""
    # ψ、托住推力与质量在三处都对得上：开跑快照、XYSTART、页面请求。
    assert cond["xy"]["xy_inject"] == inject and cond["xy"]["xy_mass_g"] == "1184"
    assert cond["xy"]["xy_psi_mrad"] in ("785", "786") and cond["xy"]["xy_target_cn"] == "1350"
    request = cond["xy_request"]
    assert request["inject"] == inject and request["win_mm"] == 150 and request["mass_g"] == 1184
    assert request["hold_thrust_n"] == 13.5 and request["target_cn"] == 1350
    assert request["psi_mrad"] == 785 and request["control"] == (
        "openloop" if inject == "tilt" else "closed_loop")
    assert request["airframe_mass_g"] == 1145.3
    assert cond["start"]["psi_mrad"] == "785"
    assert cond["parameter_echo"]["airframe.mass_kg"] == "1.145300"
    assert cond["rpm_notch"]["run"] == str(bench.fw.run_id) or "rpm_notch" not in cond
    text = page.xy_result_var.get()
    assert "本轮水平槽设置（飞控开跑时报告）" in text and "托住推力 13.5 N" in text
    assert "height" not in text


def test_no_stray_banner_or_status_while_running_then_done(page, monkeypatch, tmp_path):
    bench = bench_for(page)
    redirect_archive(monkeypatch, tmp_path)
    start_to_running(bench, target="13.5")
    fw = bench.fw
    titles = []
    for phase in ("ramp_up", "settle", "preroll", "excite", "ramp_down"):
        bench.feed(fw.phase_line(phase))
        titles.append(page.banner_var.get())
    assert titles[0] == "升推力…" and titles[3].startswith("激励中") and titles[4] == "降推力…"
    # 运行中的 THR 轮询回复（真实格式）不应打乱状态：实时读数刷新、阶段不被它冲掉。
    fw.xy_pos_m, fw.xy_vel_m_s = 0.0421, -0.0183
    bench.feed(fw.thr_line())
    page.refresh_xy_live()
    assert "沿 u 位置 42 mm" in page.xy_live_var.get() and "-18 mm/s" in page.xy_live_var.get()


# ---------------------------------------------------------------- 3. 开跑前检被拒：固件每一条原文


PRECHECKS = [
    # (让固件拒绝的设置, 固件原话, 页面必须说到的中文)
    (dict(flow_ok=False), "ERR sysid xy flow invalid", "光流"),
    (dict(tof_ok=False), "ERR sysid xy height invalid (TOF)", "测距"),
    (dict(armed=0), "ERR sysid not armed: arm with throttle stick low first", "解锁"),
    (dict(thr_low=0), "ERR sysid throttle stick not low", "油门杆"),
]


@pytest.mark.parametrize("settings,line,hint", PRECHECKS)
def test_firmware_refusals_before_start_are_shown_in_chinese_and_the_page_is_not_stuck(page, settings, line, hint):
    bench = bench_for(page, **settings)
    start_to_running(bench, target="13.5")
    assert bench.fw.state == "idle"
    assert line in page.status_var.get()
    assert hint in page.status_var.get()
    assert page.workflow.awaiting is None and page.workflow.run_id is None
    assert page.banner_var.get() == "没能开始"
    assert hint in page.banner_detail_var.get()
    # 没卡住：修好条件后再点一次就能开跑。
    for name, value in settings.items():
        setattr(bench.fw, name, {"flow_ok": True, "tof_ok": True, "armed": 1, "thr_low": 1}[name])
    bench.page.start_xy_run()
    bench.settle()
    assert bench.fw.state == "running"


def test_xy_thrust_end_reasons_point_at_the_xy_page():
    from panel_lib.pages.sysid import reasons
    for reason in ("thrust_low", "actuator_saturated"):
        _what, step = reasons.explain_end(reason, "5")
        assert "托住推力" in step and "「准备」页" not in step
        assert "「准备」页" in reasons.explain_end(reason, "0")[1]     # 内环页的说法不变


def test_the_xy_precheck_reasons_that_the_page_cannot_reach_are_still_translated():
    """页面开始前已把这些挡掉（幅值/推力/质量），但固件原文真来了（例如旧面板、AI 接口改了设置）也要说人话。"""
    from panel_lib.pages.sysid import reasons
    fw = FakeFirmware()
    cases = {
        "needs auto throttle (SYSID THROTTLE target_n>0)": "托住推力",
        "config invalid": "水平槽设置无效",
        "airframe gravity invalid": "重力",
        "target over max thrust": "最大总推力",
        "amp over limit: inject=tilt amp<=0.10 rad": "幅值",
        "amp over limit: inject=vel amp<=0.3 m/s": "幅值",
        "amp over limit: inject=pos amp<=0.15 m": "幅值",
        "amp over window: inject=pos amp<=0.7*win": "出窗余量",
        "mass unknown (set SYSID XY mass_g= or airframe mass)": "质量",
        "flow invalid": "光流",
        "height invalid (TOF)": "测距",
    }
    for reason, hint in cases.items():
        text = reasons.explain_error(f"ERR sysid xy {reason}")
        assert hint in text, reason
        assert "飞控原话" in text and "水平槽辨识开始前自检没过" not in text, reason
    del fw


@pytest.mark.parametrize("reason,hint", [("running", "正在进行"), ("range", "超出范围"), ("usage", "格式不对")])
def test_sysid_xy_rejections_use_the_firmware_text(page, reason, hint):
    bench = bench_for(page)
    fill_bench_settings(page, target="13.5")
    original = bench.fw.cmd_xy
    bench.fw.cmd_xy = lambda tokens: f"SYSID XY event=rejected reason={reason}" if len(tokens) > 2 else original(tokens)
    page.start_xy_run()
    bench.settle()
    assert hint in page.status_var.get() and f"reason={reason}" in page.status_var.get()
    assert page.workflow.awaiting is None and page.banner_var.get() == "没能开始"


@pytest.mark.parametrize("win", [30, 400])
def test_window_limits_the_page_accepts_are_accepted_by_the_firmware(page, win):
    bench = bench_for(page)
    start_to_running(bench, target="13.5", win=win)
    assert bench.fw.state == "running", page.status_var.get()


def test_the_throttle_refusal_text_points_at_the_xy_page_not_the_prep_page(page):
    """固件拒 THROTTLE（上限在页面读过参数之后被改小）：提示要指到 XY 页的「托住推力」，不是「准备」页。"""
    bench = bench_for(page, max_force_n=30.0)
    original = bench.fw.cmd_throttle

    def strict(tokens):
        bench.fw.max_force_n = 12.0          # 页面手里的整机最大推力还是 30 N
        return original(tokens)
    bench.fw.cmd_throttle = strict
    start_to_running(bench, target="13.5")
    assert bench.fw.state == "idle"
    assert "托住推力" in page.status_var.get() and "XY 速度 / 位置环" in page.status_var.get(), page.status_var.get()


def test_empty_hold_thrust_uses_the_firmware_hover_parameter(page):
    bench = bench_for(page)
    start_to_running(bench, target="")
    assert "SYSID THROTTLE target_n=13.5 max_pct=90" in bench.sent() and bench.fw.state == "running"


def test_the_stop_button_during_a_run_hands_back_and_reports_the_stop(page, monkeypatch, tmp_path):
    bench = bench_for(page)
    redirect_archive(monkeypatch, tmp_path)
    start_to_running(bench, target="13.5")
    fw = bench.fw
    bench.feed(fw.phase_line("ramp_up"), fw.phase_line("settle"))
    page.stop_run()
    bench.settle()
    assert "SYSID STOP" in bench.sent() and fw.state == "aborted"
    push_samples(page, tilt_series(seconds=1.0)[1], run_id=fw.run_id)
    bench.feed(fw.end_line("aborted", "command"))
    assert "你点了停止" in page.banner_var.get()
    assert "你点了停止" in page.xy_result_var.get()


def test_a_start_the_firmware_never_answers_times_out_with_advice(page):
    bench = bench_for(page)
    bench.fw.cmd_start = lambda: []
    fill_bench_settings(page, target="13.5")
    page.start_xy_run()
    bench.settle()
    token = page.workflow.ticket
    assert page.workflow.awaiting == "SYSID START"
    page.workflow.timeout(token)
    assert "3 秒内没有确认开始" in page.status_var.get() and "停止" in page.status_var.get()
    assert page.workflow.awaiting is None


def test_a_missing_xy_echo_times_out_naming_the_command(page):
    bench = bench_for(page)
    original = bench.fw.cmd_xy
    bench.fw.cmd_xy = lambda tokens: (_ for _ in ()).throw(StopIteration) if False else original(tokens)
    fill_bench_settings(page, target="13.5")
    real_handle = bench.fw.handle
    bench.fw.handle = lambda line: [] if line.startswith("SYSID XY inject=") else real_handle(line)
    page.start_xy_run()
    bench.settle()
    assert page.workflow.awaiting.startswith("SYSID XY inject=")
    page.workflow.timeout(page.workflow.ticket)
    assert "等待回显超时" in page.status_var.get() and "SYSID XY" in page.status_var.get()


# ---------------------------------------------------------------- 4. 运行中被软停


@pytest.mark.parametrize("reason,what", [
    ("xy_window", "沿槽位移超出出窗余量"),
    ("xy_flow_invalid", "光流速度/位置失效"),
    ("axis_residual", "机体转得太快"),
    ("angle_limit", "摆角超过了保护角"),
    ("rc_throttle_override", "推了油门杆"),
    ("rc_disarm", "上锁"),
    ("rc_lost", "遥控信号丢失"),
    ("thrust_stale", "推力查表"),
    ("imu_stale", "陀螺仪"),
    ("command", "停止"),
])
def test_soft_stops_and_handovers_are_explained_and_archived_as_diagnostic(page, monkeypatch, tmp_path, reason, what):
    bench = bench_for(page)
    root = redirect_archive(monkeypatch, tmp_path)
    start_to_running(bench, target="13.5")
    fw = bench.fw
    for phase in ("ramp_up", "settle", "preroll", "excite"):
        bench.feed(fw.phase_line(phase))
    push_samples(page, tilt_series(seconds=2.0)[1], run_id=fw.run_id)
    bench.feed(fw.phase_line("ramp_down"))
    bench.feed(fw.end_line("aborted", reason))
    assert what in page.xy_result_var.get(), page.xy_result_var.get()
    assert "数据不完整，不能分析" in page.xy_result_var.get()
    assert page.banner_var.get().startswith("中止：") and what in page.banner_var.get()
    assert reason in page.status_var.get()
    folder, cond = conditions_of(page)
    assert cond["end"]["state"] == "aborted" and cond["end"]["reason"] == reason
    assert cond["mode"] == "5" and cond["xy"]["xy_inject"] == "tilt" and cond["xy_request"]["inject"] == "tilt"
    assert cond["data_error"] and reason in cond["data_error"]


def test_the_page_is_free_again_after_an_abort(page, monkeypatch, tmp_path):
    bench = bench_for(page)
    redirect_archive(monkeypatch, tmp_path)
    start_to_running(bench, target="13.5")
    fw = bench.fw
    push_samples(page, tilt_series(seconds=1.0)[1], run_id=fw.run_id)
    bench.feed(fw.end_line("aborted", "xy_window"))
    wait_for(page, lambda: not page.workflow.saving)
    bench.page.start_xy_run()
    bench.settle()
    assert fw.state == "running" and fw.run_id == 2 and page.workflow.run_id == 2


# ---------------------------------------------------------------- 5. 固件数值舍入不得误判


@pytest.mark.parametrize("target", ["12.345", "9.825", "13.505", "11.115", "10.125"])
def test_half_centinewton_targets_do_not_trip_the_provenance_check(page, monkeypatch, tmp_path, target):
    """页面 Python round()（银行家舍入、双精度）与固件 lroundf（半数远离零、float32）在 x.xx5 N 上差 1 cN。"""
    bench = bench_for(page)
    redirect_archive(monkeypatch, tmp_path)
    start_to_running(bench, target=target)
    fw = bench.fw
    assert fw.state == "running"
    assert page.workflow.data_error == "", page.workflow.data_error
    assert "SYSID STOP" not in bench.sent(), "页面不该因为 1 cN 的取整差就把刚转起来的电机停掉"
    push_samples(page, tilt_series(seconds=2.0)[1], run_id=fw.run_id)
    bench.feed(fw.end_line())
    assert not page.workflow.data_error


@pytest.mark.parametrize("psi", ["45", "-45", "0", "90", "30", "135", "-135", "22.5", "1"])
def test_psi_rounding_between_page_and_firmware_is_tolerated(page, psi):
    bench = bench_for(page)
    page.psi_var.set(psi)
    start_to_running(bench, target="13.5")
    assert bench.fw.state == "running"
    assert page.workflow.data_error == "", page.workflow.data_error


@pytest.mark.parametrize("extra", ["38.8", "0", "38.5", "38.45", "100"])
def test_mass_rounding_never_trips_the_echo_check(page, extra):
    bench = bench_for(page)
    start_to_running(bench, target="13.5", extra=extra)
    assert bench.fw.state == "running", page.status_var.get()
    assert page.workflow.data_error == ""


@pytest.mark.parametrize("amp", ["0.07", "0.035", "0.029", "0.0999", "0.01"])
def test_fractional_amplitude_rounding_is_tolerated(page, amp):
    """固件 EXC 回显的 amp_mrad_s 是 float32 截断：0.07、0.035、0.029 这类幅值页面按 int(x*1000) 也差不了 1。"""
    bench = bench_for(page)
    fill_bench_settings(page, target="13.5")
    page.amp_var.set(amp)
    page.start_xy_run()
    bench.settle()
    assert bench.fw.state == "running", (amp, page.status_var.get())


# ---------------------------------------------------------------- 6. AI 接口在事务中插进来的行


class AiInjector:
    """模拟 AI 接口：它走 transport.send_line，**不经过页面的 send/note_sent**，回复却落进同一条行流。"""

    def __init__(self, bench):
        self.bench = bench

    def send(self, line):
        self.bench.page.panel.transport.send_line(line)        # 与 ai_bridge._tk_send 相同


def ai_send(page, raw_send, line):
    """与 `AiBridge._tk_send` 同一条路径：传输层直发，成功后向系统辨识页记账（ai_bridge.note_sysid_sent）。"""
    from panel_lib.ai_bridge import note_sysid_sent
    page.panel.sysid_page = page
    if raw_send(line):
        note_sysid_sent(page.panel, line)


def ai_before_each_step(bench, line):
    """每当页面发出一条命令，AI 就紧接着发一条查询（最坏情形）。"""
    page = bench.page
    transport = page.panel.transport
    original = transport.send_line

    def send(text):
        ok = original(text)
        if text != line and not getattr(send, "busy", False):
            send.busy = True
            ai_send(page, original, line)
            send.busy = False
        return ok
    transport.send_line = send


@pytest.mark.parametrize("query", ["SYSID?", "SYSID THR?", "SYSID XY", "PARAM?", "HOVER?"])
def test_ai_queries_interleaved_with_the_start_transaction_do_not_break_it(page, query):
    bench = bench_for(page)
    ai_before_each_step(bench, query)
    start_to_running(bench, target="13.5")
    assert bench.fw.state == "running", (query, page.status_var.get())
    assert page.workflow.data_error == "", page.workflow.data_error


@pytest.mark.parametrize("query", ["SYSID?", "SYSID XY", "SYSID THR?"])
def test_ai_query_sent_between_mode_and_xy_does_not_fake_the_xy_echo(page, query):
    """最坏点：页面刚收到 MODE 的回复、AI 的查询恰好排在 `SYSID XY inject=...` 之前。"""
    bench = bench_for(page)
    fill_bench_settings(page, target="13.5", inject="vel")
    transport = page.panel.transport
    original = transport.send_line
    fired = []

    def send(text):
        if text.startswith("SYSID XY inject=") and not fired:
            fired.append(True)
            ai_send(page, original, query)   # AI 的查询先到固件：它回的是「vel 还没生效」的旧配置
        return original(text)
    transport.send_line = send
    page.start_xy_run()
    bench.settle()
    assert bench.fw.state == "running", (query, page.status_var.get())
    assert "配置回读不符" not in page.status_var.get()


@pytest.mark.parametrize("stray,when", [
    ("SYSID XY inject=bad", "before"),      # 固件回 event=rejected reason=usage：不能当成页面这条的拒绝
    ("SYSID XY", "after"),                  # 排在页面命令后面：回复晚到，不能打乱后面的 RIG/THROTTLE 核对
    ("SYSID XY", "before"),
])
def test_stray_sysid_xy_replies_are_neither_taken_as_the_echo_nor_break_later_steps(page, stray, when):
    bench = bench_for(page)
    fill_bench_settings(page, target="13.5", inject="vel")
    transport = page.panel.transport
    original = transport.send_line

    def send(text):
        is_page_xy = text.startswith("SYSID XY inject=vel")
        if is_page_xy and when == "before":
            ai_send(page, original, stray)
        ok = original(text)
        if is_page_xy and when == "after":
            ai_send(page, original, stray)
        return ok
    transport.send_line = send
    page.start_xy_run()
    bench.settle()
    assert bench.fw.state == "running", page.status_var.get()
    assert "配置回读不符" not in page.status_var.get() and "飞控拒绝水平槽设置" not in page.status_var.get()


def test_an_unanswered_stray_does_not_hang_forever_it_expires(page):
    import time
    bench = bench_for(page)
    w = page.workflow
    w.xy_note_sent("SYSID XY")               # AI 发了、回复丢了
    w.xy_strays = [time.monotonic() - 1.0]   # 已过期
    start_to_running(bench, target="13.5")
    assert bench.fw.state == "running", page.status_var.get()


def test_hover_reply_to_someone_elses_query_does_not_overwrite_the_typed_thrust(page):
    bench = bench_for(page, hover=(11.0, 0.1, 1))
    page.set_front_view("XY")
    page.xy_target_var.set("14.0")
    page.panel.transport.send_line("HOVER?")            # AI 接口发的，页面没点读取按钮
    bench.settle()
    assert page.xy_target_var.get() == "14.0"


# ---------------------------------------------------------------- 7. ALT / XY / 内环切换时激励设置不互相串


def test_excitation_settings_do_not_bleed_between_inner_alt_and_xy(page):
    page.set_front_view(None)
    page.profile_var.set("step")
    page.amp_var.set("0.55")
    page.dur_var.set("1234")
    inner = (page.profile_var.get(), page.amp_var.get(), page.dur_var.get())
    page.set_front_view("XY")
    page.xy_inject_var.set("vel")
    page.select_xy_inject()
    from panel_lib.pages.sysid import xy_config
    assert page.amp_var.get() == xy_config.INJECT_PRESETS["vel"]["amp"] and page.profile_var.get() == "doublet"
    page.amp_var.set("0.07")
    page.set_front_view("ALT")
    assert page.amp_var.get() not in ("0.55", "0.07") or page.mode_var.get() == "ALT"
    alt_amp = page.amp_var.get()
    page.amp_var.set("0.04" if alt_amp != "0.04" else "0.05")
    alt_after = page.amp_var.get()
    page.set_front_view("XY")
    assert page.amp_var.get() == "0.07", "XY 的幅值被 ALT 改掉了"
    page.set_front_view("ALT")
    assert page.amp_var.get() == alt_after
    page.set_front_view(None)
    assert (page.profile_var.get(), page.amp_var.get(), page.dur_var.get()) == inner
    assert page.mode_var.get() not in ("ALT", "XY")


def test_xy_and_inner_keep_their_own_throttle_inputs(page):
    page.xy_target_var.set("12.0")
    page.target_thrust_var.set("9.5")
    page.max_pct_var.set("60")
    page.set_front_view("XY")
    assert page.throttle_settings() == (False, 12.0, float(page.xy_max_pct_var.get()))
    page.set_front_view(None)
    assert page.throttle_settings()[1:] == (9.5, 60.0)


def test_start_from_the_inner_page_after_xy_never_runs_xy(page):
    bench = bench_for(page)
    fill_bench_settings(page, target="13.5")
    page.start_inner_run()
    bench.settle()
    assert "SYSID MODE FF" in " ".join(bench.sent()) and "SYSID XY" not in " ".join(bench.sent())


def test_the_real_ai_bridge_send_path_keeps_the_page_ledger(page):
    """`AiBridge._tk_send` 真实路径：AI 发的 `SYSID?` 必须记进页面的「非事务报告」账。"""
    import threading
    import time
    from panel_lib import ai_bridge as ab

    panel = page.panel
    panel.sysid_page = page
    panel.last_cmd_var = type("V", (), {"set": lambda self, value: None})()
    panel._transport_connected = lambda: True
    bridge = ab.AiBridge(panel, info_file=str(ROOT / "data" / "_unused_ai_info.json"))
    results = {}

    def call():
        results["r"] = bridge._tk_send("SYSID?")
    thread = threading.Thread(target=call, daemon=True)
    thread.start()
    deadline = time.monotonic() + 3
    while thread.is_alive() and time.monotonic() < deadline:
        bridge.pump()
        time.sleep(0.005)
    assert results["r"] == (True, "")
    assert panel.transport.lines[-1] == "SYSID?"
    assert page.workflow.reports_in_flight() == 1
    assert not (ROOT / "data" / "_unused_ai_info.json").exists()


FLAG_THRUST_CAPPED = 0x0200       # drv_sysid_record.h：推力被最高油门封顶


def test_a_thrust_capped_run_is_still_accepted_and_archived(page, monkeypatch, tmp_path):
    """最高油门 90% 不够托住机体时固件在批头置 0x0200，数据仍然有效（只是实际推力低于目标）。"""
    bench = bench_for(page)
    redirect_archive(monkeypatch, tmp_path)
    start_to_running(bench, target="13.5")
    push_samples(page, tilt_series()[1], run_id=bench.fw.run_id, extra_flags=FLAG_THRUST_CAPPED)
    bench.feed(bench.fw.end_line())
    assert page.banner_var.get() == "完成" and page.workflow.data_error == ""


def test_dropped_samples_mark_the_run_diagnostic_only_but_still_archive(page, monkeypatch, tmp_path):
    bench = bench_for(page)
    redirect_archive(monkeypatch, tmp_path)
    start_to_running(bench, target="13.5")
    push_samples(page, tilt_series()[1], run_id=bench.fw.run_id)
    bench.feed(bench.fw.end_line(dropped=3))
    assert page.workflow.data_error and "丢弃计数 3" in page.workflow.data_error
    assert page.banner_var.get().startswith("完成（仅诊断）")
    _folder, cond = conditions_of(page)
    assert "丢弃计数 3" in cond["data_error"] and cond["end"]["dropped"] == "3"

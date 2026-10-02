"""「XY 速度 / 位置环」页的水平槽台架辨识模式 XY（R-XYID-1）：操作、解码、分析与存档。

控件在「XY 速度 / 位置环」页（horizontal.py），状态与事务仍在内环页对象里（固件同一时间只跑一轮
SYSID）；下面的夹具在每个用 `page` 的测试里把 XY 页挂到同一个对象上，和真面板一样。
报文照跨侧契约 `doc/sysid-xy-contract.md` 写：

* `SYSID MODE XY` → 模式编号 5；
* `SYSID XY inject=tilt|vel|pos win_mm=.. mass_g=..` 回一行带 control（openloop|closed_loop）的回显，
  拒绝回 `SYSID XY event=rejected reason=running|range|usage`；
* 开跑后紧跟 `SYSID XYSTART run=.. xy_inject=.. xy_win_mm=.. xy_mass_g=.. xy_psi_mrad=..
  xy_target_cn=.. xy_x0_mm=.. xy_y0_mm=..`；
* `SYSID THR` 回报末尾带 xy_pos_mm / xy_vel_mms / xy_ok；
* 记录字段表沿用 ALT 追加的 7 个尾字段（height/height_raw/height_sp/vz/vz_sp/az/vbat），XY 里含义变了。

批头里 XY 自己的标志位契约没写死数值（只说不与 ALT 0x0100、封顶 0x0200 冲突），这里借用 0x0400；
页面不要求它，只拒绝属于别的模式的位（0x20/0x40/0x80）。
"""
from __future__ import annotations

import csv
import json
import math
import re
import struct
import sys
from pathlib import Path

import pytest

from test_sysid_altitude_mode import (  # noqa: F401  字段表与报文工具
    ALT_FIELDS, ALT_HASH, ALT_PARAMS, SCHEMA_ALT_LINES, SCHEMA_V3_LINES, START_LINE, archive_into,
    no_fit,
)
from test_sysid_page import (  # noqa: F401  页面夹具与报文工具
    FLAG_FIRST_BATCH, FLAG_LAST_BATCH, FakePanel, feed, fresh_page, page, settings_file,
    status_report, thr_line, wait_for,
)

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from panel_lib.pages.sysid import reasons, xy_config  # noqa: E402
from sysid import xy_analysis  # noqa: E402

FLAG_XY = 0x0400
GRAVITY = 9.80665
#: 机体参数：整机 1145.3 g + 悬停推力 13.5 N + 整机最大推力 30 N。
XY_PARAMS = ALT_PARAMS + (("coax.hover_thrust_n", "13.500000"),
                          ("airframe.max_total_force_n", "30.000000"))


@pytest.fixture(autouse=True)
def _mount_xy_view(request):
    """用到 `page` 的测试都挂上 XY 页：水平槽的控件、图和结果区在那一页。"""
    if "page" not in request.fixturenames:
        return
    engine = request.getfixturevalue("page")
    from panel_lib.pages.sysid.horizontal import SysIdHorizontalPage
    from tkinter import ttk
    engine.test_xy_view = SysIdHorizontalPage(engine.panel,
                                              ttk.Frame(engine.parent.winfo_toplevel()), engine)


def xy_echo(expected: dict) -> str:
    return (f"SYSID XY inject={expected['inject']} win_mm={expected['win_mm']} "
            f"mass_g={expected['mass_g']} control={expected['control']}")


def drive_xy(page, *, params=XY_PARAMS, schema=SCHEMA_ALT_LINES, echo=xy_echo, inject=None,
             target="", customize=None, **report):
    """前台切到 XY 页 → 点开始 → 回字段表、参数；每条配置命令回整份报告，SYSID XY 回一行。

    托住推力留空 = 取飞控参数 coax.hover_thrust_n（13.5 N）。"""
    page.set_front_view("XY")
    if inject is not None:
        page.xy_inject_var.set(inject)
        page.select_xy_inject()
    page.xy_target_var.set(target)
    page.rod_to_fc_var.set("0.15")
    page.roll_pivot_var.set("-0.2")
    page.pitch_pivot_var.set("-0.2")
    if customize:
        customize(page)
    page.start_run()
    if page.panel.transport.lines[-1:] != ["SYSID SCHEMA"]:
        return
    feed(page, schema)
    feed(page, [f"PARAM name={name} value={value}" for name, value in params])
    for _ in range(14):
        awaiting = page.workflow.awaiting
        if not awaiting or awaiting == "SYSID START":
            break
        if awaiting.startswith("SYSID XY "):
            feed(page, [echo(page.workflow.xy_expected)])
        else:
            feed(page, status_report(page.workflow.expected, **report))


def xystart_line(page, **changes):
    expected = page.workflow.xy_expected
    fields = dict(run=3, xy_inject=expected["inject"], xy_win_mm=expected["win_mm"],
                  xy_mass_g=expected["mass_g"], xy_psi_mrad=page.workflow.expected["psi_mrad"],
                  xy_target_cn=page.workflow.expected["target_cn"], xy_x0_mm=12, xy_y0_mm=-7)
    fields.update(changes)
    return "SYSID XYSTART " + " ".join(f"{key}={value}" for key, value in fields.items())


def xy_frame(run, rows, *, flags, base_us, dt_us=4000):
    """`rows`：每条一个 {字段名: 定点原始值}，没给的字段填 0（offset_us 照实填）。"""
    header = struct.pack("<BBHIIHH", 3, len(rows), run, ALT_HASH, base_us, dt_us, flags | FLAG_XY)
    fmt = "<" + "".join("h" if kind == "i16" else "H" for *_rest, kind in ALT_FIELDS)

    def value(row, index, name):
        return row.get(name, index * dt_us if name == "offset_us" else 0)
    return header + b"".join(
        struct.pack(fmt, *(value(row, index, name) for name, *_rest in ALT_FIELDS))
        for index, row in enumerate(rows))


# ---------------------------------------------------------------- 合成数据（分析与整轮共用）


def tilt_series(*, gain=0.9, static_acc=0.4, kinetic_acc=0.15, lag_s=0.06, dt=0.004, seconds=12.0,
                angle_offset=0.02):
    """开环 tilt 轮：倾角指令是渐增的正弦，沿 u 的加速度 = 增益 × g·tanθ − 摩擦，光流速度晚 lag_s。

    返回 (时间 s, 样本)，样本用 schema 里的原始字段名（height 等，未改名）。静摩擦门槛角的真值
    = atan(static_acc / (gain·g))。"""
    count = int(seconds / dt)
    times = [i * dt for i in range(count)]
    velocity, speeds, rows = 0.0, [], []
    for t in times:
        command = 0.0 if t < 1.5 else 0.08 * min(1.0, (t - 1.5) / 6.0) * math.sin(2 * math.pi * 0.4 * (t - 1.5))
        a_cmd = GRAVITY * math.tan(command)
        force = gain * a_cmd
        if abs(velocity) < 1e-6:
            acc = 0.0 if abs(force) <= static_acc else force - math.copysign(kinetic_acc, force)
        else:
            acc = force - math.copysign(kinetic_acc, velocity)
        new = velocity + acc * dt
        if velocity != 0.0 and new * velocity < 0 and abs(force) <= static_acc:
            new = 0.0
        velocity = new
        speeds.append(velocity)
        # 固件口径：正绕杆角把推力偏向 −u，所以推向 +u 的 θ 记成负的 angle / angle_sp。
        rows.append(dict(angle=-command + angle_offset, angle_sp=-command, az=a_cmd, thrust=13.5))
    delay = int(lag_s / dt)
    for i, row in enumerate(rows):
        row.update(vz=speeds[max(0, i - delay)], height=0.0, height_sp=0.0, vz_sp=0.0)
    return times, rows


def step_series(*, tau=0.1, zeta=None, wn=10.0, dt=0.004, seconds=8.0, kind="pos", noise=0.0):
    """闭环阶跃轮：参考是 ±0.04（m 或 m/s）的双脉冲（保持 1.85 s、斜坡 0.15 s），测量 = 一阶/二阶响应。"""
    import random

    rng = random.Random(3)
    times = [i * dt for i in range(int(seconds / dt))]
    y = yd = 0.0
    rows = []

    def shape(t):
        if t < 1:
            return 0.0
        if t < 1.15:
            return (t - 1) / 0.15
        if t < 3:
            return 1.0
        if t < 3.3:
            return 1 - 2 * (t - 3) / 0.3
        if t < 5:
            return -1.0
        if t < 5.15:
            return -1 + (t - 5) / 0.15
        return 0.0
    for t in times:
        ref = 0.04 * shape(t)
        if zeta is None:
            y += (ref - y) / tau * dt
        else:
            yd += (wn * wn * (ref - y) - 2 * zeta * wn * yd) * dt
            y += yd * dt
        measured = y + rng.gauss(0.0, noise)
        row = dict(angle=0.0, angle_sp=0.0, az=0.0, thrust=13.5, height_raw=0.0)
        if kind == "pos":
            row.update(height=measured, height_sp=ref, vz=0.0, vz_sp=0.0)
        else:
            row.update(height=0.0, height_sp=0.0, vz=measured, vz_sp=ref)
        rows.append(row)
    return times, rows


# 固定点缩放（schema 里的 scale）：把物理量换成 i16 原始值。
_SCALES = {"height": 1e-4, "height_raw": 1e-4, "height_sp": 1e-4, "vz": 1e-3, "vz_sp": 1e-3,
           "az": 1e-3, "angle": 1e-4, "angle_sp": 1e-4, "thrust": 1e-2, "vbat": 1e-3}


def to_raw(rows):
    return [{name: int(round(value / _SCALES[name])) for name, value in row.items() if name in _SCALES}
            for row in rows]


def feed_run(page, rows, *, per=5, dt_us=4000, flags=0):
    """按固件批量（每批最多 5 条）把样本喂给页面；第一批带首包、最后一批带末包标志。"""
    raw = to_raw(rows)
    for start in range(0, len(raw), per):
        batch_flags = (FLAG_FIRST_BATCH if start == 0 else 0) | (
            FLAG_LAST_BATCH if start + per >= len(raw) else 0) | flags
        page.accept(xy_frame(3, raw[start:start + per], flags=batch_flags,
                             base_us=1000 + start * dt_us, dt_us=dt_us))


def run_xy(page, rows=None, *, inject="tilt", end_line=None, extra_lines=(), **kwargs):
    drive_xy(page, inject=inject, **kwargs)
    assert page.panel.transport.lines[-1] == "SYSID START"
    page.handle_line(START_LINE)
    page.handle_line(xystart_line(page))
    feed(page, extra_lines)
    if rows is None:
        rows = tilt_series()[1] if inject == "tilt" else step_series(kind=inject)[1]
    feed_run(page, rows)
    if end_line:
        page.handle_line(end_line)


# ---------------------------------------------------------------- 模式、设置与命令事务


def test_xy_is_appended_as_mode_five_and_lives_on_its_own_page():
    from panel_lib.pages.sysid import inner_loop, settings_store
    assert settings_store.MODES.index("XY") == xy_config.XY_MODE_CODE == 5
    assert settings_store.MODES[:5] == ("FF", "RATE", "ANGLE", "SERVO", "ALT")
    assert "XY" not in inner_loop.INNER_MODES and "ALT" not in inner_loop.INNER_MODES


def test_the_mode_hint_warns_that_motors_spin_and_the_body_translates(page):
    page.mode_var.set("XY")
    hint = page.mode_hint_var.get()
    assert "电机会转" in hint and "水平槽" in hint and "挡块" in hint


def test_the_xy_page_is_mounted_and_has_a_start_button_and_three_steps(page):
    view = page.test_xy_view
    assert view.start_button.cget("text") == "开始水平槽辨识"
    assert [view.steps_notebook.tab(tab, "text") for tab in view.steps_notebook.tabs()] == [
        "1 · 准备", "2 · 看波形", "3 · 结果"]


def test_start_configures_mode_xy_then_the_xy_line_then_rig_then_throttle(page):
    drive_xy(page)
    lines = page.panel.transport.lines
    assert lines[-1] == "SYSID START"
    mode = lines.index("SYSID MODE XY 3")
    xy = lines.index("SYSID XY inject=tilt win_mm=150 mass_g=1184")
    rig = next(i for i, line in enumerate(lines) if line.startswith("SYSID RIG "))
    throttle = next(i for i, line in enumerate(lines) if line.startswith("SYSID THROTTLE "))
    assert mode < xy < rig < throttle
    assert any(line.startswith("SYSID EXC ") for line in lines[:mode])
    assert lines[throttle] == "SYSID THROTTLE target_n=13.5 max_pct=90"      # 托住推力 = 飞控悬停推力参数
    assert page.workflow.expected["mode"] == 5
    assert page.workflow.xy_request["hold_thrust_n"] == 13.5


def test_a_typed_hold_thrust_wins_over_the_firmware_hover_parameter(page):
    drive_xy(page, target="12.4")
    assert "SYSID THROTTLE target_n=12.4 max_pct=90" in page.panel.transport.lines


def test_win_mass_and_inject_are_sent_as_typed(page):
    def custom(p):
        p.xy_win_var.set("200")
        p.xy_extra_mass_var.set("100")
    drive_xy(page, inject="vel", customize=custom)
    assert "SYSID XY inject=vel win_mm=200 mass_g=1245" in page.panel.transport.lines
    assert page.workflow.xy_expected["control"] == "closed_loop"


@pytest.mark.parametrize("inject,control", [("tilt", "openloop"), ("vel", "closed_loop"),
                                            ("pos", "closed_loop")])
def test_control_mode_follows_the_inject_type(page, inject, control):
    drive_xy(page, inject=inject)
    assert page.workflow.xy_expected["control"] == control
    assert page.panel.transport.lines[-1] == "SYSID START"


def test_every_echo_item_is_checked_before_start(page):
    def wrong_mass(expected):
        return xy_echo(dict(expected, mass_g=expected["mass_g"] + 50))
    drive_xy(page, echo=wrong_mass)
    assert "SYSID START" not in page.panel.transport.lines
    assert "配置回读不符：水平槽 mass_g" in page.status_var.get()

    page2 = fresh_page(page)
    def wrong_control(expected):
        return xy_echo(dict(expected, control="closed_loop"))
    page2.panel.transport.is_connected = True
    drive_xy_on(page2, wrong_control)
    assert "SYSID START" not in page2.panel.transport.lines
    assert "control" in page2.status_var.get()


def drive_xy_on(engine, echo):
    """在另一页对象上跑一遍（同样挂上 XY 页）。"""
    from panel_lib.pages.sysid.horizontal import SysIdHorizontalPage
    from tkinter import ttk
    engine.test_xy_view = SysIdHorizontalPage(engine.panel,
                                              ttk.Frame(engine.parent.winfo_toplevel()), engine)
    drive_xy(engine, echo=echo)


@pytest.mark.parametrize("reason,text", [("running", "辨识正在进行"), ("range", "超出范围"),
                                         ("usage", "命令格式不对")])
def test_firmware_rejections_are_translated(page, reason, text):
    def reject(_expected):
        return f"SYSID XY event=rejected reason={reason}"
    drive_xy(page, echo=reject)
    assert "SYSID START" not in page.panel.transport.lines
    assert text in page.status_var.get() and f"reason={reason}" in page.status_var.get()


def test_an_old_schema_without_the_tail_fields_refuses_to_start(page):
    drive_xy(page, schema=SCHEMA_V3_LINES)
    assert "SYSID START" not in page.panel.transport.lines
    assert "水平槽辨识（XY）需要记录里带" in page.workflow.notice


def test_manual_throttle_checkbox_does_not_apply_to_xy(page):
    page.manual_throttle_var.set(True)
    drive_xy(page)
    assert page.panel.transport.lines[-1] == "SYSID START"
    assert "SYSID THROTTLE target_n=13.5 max_pct=90" in page.panel.transport.lines


# ---------------------------------------------------------------- 范围检查


def sent_nothing(page) -> bool:
    return not page.panel.transport.lines


@pytest.mark.parametrize("inject,amp,unit", [("tilt", "0.11", "rad"), ("vel", "0.31", "m/s"),
                                              ("pos", "0.16", "m")])
def test_amplitude_over_the_inject_limit_is_refused_before_anything_is_sent(page, inject, amp, unit):
    page.set_front_view("XY")
    page.xy_inject_var.set(inject)
    page.select_xy_inject()
    page.amp_var.set(amp)
    page.xy_win_var.set("400")
    page.rod_to_fc_var.set("0.15")
    page.start_run()
    assert sent_nothing(page)
    assert f"单位是 {unit}" in page.status_var.get() and "上限" in page.status_var.get()


def test_pos_amplitude_may_not_exceed_the_window(page):
    page.set_front_view("XY")
    page.xy_inject_var.set("pos")
    page.select_xy_inject()
    page.xy_win_var.set("30")
    page.amp_var.set("0.04")
    page.rod_to_fc_var.set("0.15")
    page.start_run()
    assert sent_nothing(page)
    assert "上限 0.021 m" in page.status_var.get() and "0.7×出窗余量" in page.status_var.get()


def test_amplitude_limits_are_exactly_the_contract_limits():
    assert xy_config.amplitude_limit("tilt") == ("rad", 0.10)
    assert xy_config.amplitude_limit("vel") == ("m/s", 0.30)
    assert xy_config.amplitude_limit("pos") == ("m", 0.15)
    assert xy_config.amplitude_limit("pos", 100) == ("m", pytest.approx(0.07))
    xy_config.check_amplitude("tilt", 0.10)
    xy_config.check_amplitude("pos", 0.15, 400)
    with pytest.raises(ValueError, match="上限"):
        xy_config.check_amplitude("tilt", 0.1001)
    with pytest.raises(ValueError):
        xy_config.check_amplitude("vel", 0.0)


@pytest.mark.parametrize("win,message", [("29", "30～400"), ("401", "30～400"), ("abc", "填数字"),
                                         ("150.5", "整数")])
def test_window_margin_range_is_checked(page, win, message):
    page.set_front_view("XY")
    page.xy_win_var.set(win)
    page.rod_to_fc_var.set("0.15")
    page.start_run()
    assert sent_nothing(page)
    assert message in page.status_var.get()


@pytest.mark.parametrize("extra,message", [("-1", "0～500"), ("501", "0～500"), ("x", "填数字")])
def test_extra_mass_range_is_checked(page, extra, message):
    page.set_front_view("XY")
    page.xy_extra_mass_var.set(extra)
    page.rod_to_fc_var.set("0.15")
    page.start_run()
    assert sent_nothing(page)
    assert message in page.status_var.get()


def test_total_mass_outside_the_firmware_range_is_refused(page):
    drive_xy(page, customize=lambda p: p.xy_extra_mass_var.set("500"), params=XY_PARAMS[:-2]
             + (("airframe.mass_kg", "2.800000"), ("coax.hover_thrust_n", "13.5")))
    assert "SYSID START" not in page.panel.transport.lines
    assert "3300 g 超出飞控接受的 500～3000 g" in page.status_var.get()


@pytest.mark.parametrize("target,message", [("1.5", "太小"), ("31", "超过飞控整机最大推力 30.00 N")])
def test_hold_thrust_is_bounded_by_two_newtons_and_the_airframe_maximum(page, target, message):
    drive_xy(page, target=target)
    assert "SYSID START" not in page.panel.transport.lines
    assert message in page.status_var.get()


def test_unknown_hover_thrust_with_empty_field_asks_for_a_value(page):
    drive_xy(page, params=XY_PARAMS[:-2] + (("coax.hover_thrust_n", "0.000000"),
                                            ("airframe.max_total_force_n", "30.0")))
    assert "SYSID START" not in page.panel.transport.lines
    assert "托住推力没填" in page.status_var.get()


def test_hold_thrust_default_and_hint_follow_the_firmware_parameter(page):
    assert "连上飞控后读取" in page.xy_thrust_hint_var.get()
    feed(page, ["PARAM name=coax.hover_thrust_n value=13.500000",
                "PARAM name=airframe.max_total_force_n value=30.000000"])
    page.handle_line("PARAM name=coax.hover_thrust_n value=13.500000")
    assert "13.50 N" in page.xy_thrust_hint_var.get()
    page.xy_target_var.set("12")
    assert page.xy_thrust_hint_var.get().startswith("将使用 12 N")


def test_amplitude_unit_and_limit_are_shown_beside_the_inject_type(page):
    page.set_front_view("XY")
    # pos 的实际上限还受 0.7×出窗余量（默认 150 mm → 0.105 m）约束，「激励」旁提示的是实际上限。
    for inject, text, hint in (("tilt", "上限 0.1 rad", "上限 0.1 rad"), ("vel", "上限 0.3 m/s", "上限 0.3 m/s"),
                               ("pos", "上限 0.15 m", "上限 0.105 m")):
        page.xy_inject_var.set(inject)
        assert text in page.xy_amp_note_var.get()
        assert hint in page.amp_hint_var.get()
        assert page.amp_label_var.get() == f"幅值 [{xy_config.AMP_UNITS[inject][0]}]"
    page.xy_win_var.set("60")
    assert "不超过 0.7×出窗余量 0.042 m" in page.xy_amp_note_var.get()


def test_the_direction_of_u_follows_the_rod_azimuth(page):
    page.psi_var.set("45")
    assert "x -0.707" in page.xy_direction_var.get() and "y +0.707" in page.xy_direction_var.get()
    page.psi_var.set("0")
    assert "x -0.000" in page.xy_direction_var.get() or "x +0.000" in page.xy_direction_var.get()
    assert "y +1.000" in page.xy_direction_var.get()
    assert xy_config.direction_text(90).startswith("杆轴方位角 ψ = 90°")


# ---------------------------------------------------------------- 切页还原


def test_the_front_page_decides_what_this_run_does(page):
    page.mode_var.set("ANGLE")
    page.set_front_view("XY")
    assert page.mode_var.get() == "XY"
    page.set_front_view("ALT")
    assert page.mode_var.get() == "ALT"
    page.set_front_view(None)
    assert page.mode_var.get() == "ANGLE"          # 回内环页：还原上次选的模式


def test_each_page_keeps_its_own_excitation_settings(page):
    inner = {"amp": page.amp_var.get(), "hold": page.hold_var.get(), "repeat": page.repeat_var.get()}
    page.set_front_view("XY")
    tilt = xy_config.INJECT_PRESETS["tilt"]
    assert (page.amp_var.get(), page.hold_var.get(), page.dur_var.get()) == (tilt["amp"], tilt["hold"], tilt["dur"])
    page.xy_inject_var.set("vel")
    page.select_xy_inject()
    vel = xy_config.INJECT_PRESETS["vel"]
    assert (page.amp_var.get(), page.hold_var.get()) == (vel["amp"], vel["hold"])
    page.amp_var.set("0.07")                      # 作者在 XY 页改了幅值
    page.set_front_view(None)
    assert {"amp": page.amp_var.get(), "hold": page.hold_var.get(),
            "repeat": page.repeat_var.get()} == inner
    page.set_front_view("ALT")                     # Z 页用自己的默认值，不带着 XY 的 0.07
    assert page.amp_var.get() != "0.07"
    page.set_front_view("XY")
    assert page.amp_var.get() == "0.07" and page.hold_var.get() == "1500"
    page.set_front_view(None)
    assert page.amp_var.get() == inner["amp"]


def test_the_page_does_not_switch_while_a_transaction_or_run_is_going(page):
    drive_xy(page)
    assert page.panel.transport.lines[-1] == "SYSID START"
    page.set_front_view(None)                      # 等开始确认：不切
    assert page.mode_var.get() == "XY"
    page.handle_line(START_LINE)                   # 飞控确认开跑
    page.set_front_view(None)
    assert page.mode_var.get() == "XY"


def test_the_inner_page_start_button_never_runs_xy(page):
    page.mode_var.set("XY")
    page.start_inner_run()
    assert page.mode_var.get() != "XY"


def test_xy_page_start_button_runs_xy_from_anywhere(page):
    page.mode_var.set("RATE")
    page.start_xy_run()
    assert page.mode_var.get() == "XY"


# ---------------------------------------------------------------- 实时读数与悬停推力


def test_live_readout_comes_from_the_thr_line(page):
    assert "没有新数据" in page.xy_live_var.get()
    page.handle_line(thr_line() + " xy_pos_mm=52 xy_vel_mms=-14 xy_ok=1")
    page.refresh_xy_live()
    assert "沿 u 位置 52 mm" in page.xy_live_var.get() and "-14 mm/s" in page.xy_live_var.get()
    assert "光流有效" in page.xy_live_var.get()
    page.handle_line(thr_line() + " xy_pos_mm=52 xy_vel_mms=-14 xy_ok=0")
    page.refresh_xy_live()
    assert "无效" in page.xy_live_var.get()
    page.handle_line(thr_line())
    page.refresh_xy_live()
    assert "固件没报" in page.xy_live_var.get()


def test_reading_the_learned_hover_thrust_only_fills_the_field(page):
    page.workflow.params["airframe.max_total_force_n"] = "30.0"
    page.read_hover_thrust()
    assert page.panel.transport.lines[-1] == "HOVER?"
    before = list(page.panel.transport.lines)
    page.handle_line("HOVER est_n=14.586 std_n=0.022 converged=1 learning=1 samples=2905 rejected=0 "
                     "innov=0.010 init_n=14.250 meas_std=0.300 gate=0x3f airborne=1 above_m=0.200")
    assert page.xy_target_var.get() == "14.59"
    assert page.panel.transport.lines == before            # 只填不写飞控
    assert "没有写飞控" in page.status_var.get() and "已收敛" in page.status_var.get()
    # 已经取走：后来冒出来的 HOVER 行（别处发的查询）不再改输入框。
    page.xy_target_var.set("12")
    page.handle_line("HOVER est_n=15.000 std_n=0.022 converged=1")
    assert page.xy_target_var.get() == "12"


def test_hover_estimator_not_ready_or_out_of_range_is_not_filled(page):
    page.workflow.params["airframe.max_total_force_n"] = "30.0"
    page.xy_target_var.set("12")
    page.read_hover_thrust()
    page.handle_line("HOVER state=not_ready")
    assert page.xy_target_var.get() == "12" and "还没就绪" in page.status_var.get()
    page.read_hover_thrust()
    page.handle_line("HOVER est_n=45.000 std_n=0.1 converged=1")
    assert page.xy_target_var.get() == "12" and "无效" in page.status_var.get()


def test_an_unconverged_hover_estimate_falls_back_to_the_stored_parameter(page):
    """2026-09-30 晚台架原话：估计器被台架托着时学成 8.66 N，页面照填，首轮 tilt 推不动。"""
    page.workflow.params["airframe.max_total_force_n"] = "30.0"
    page.read_hover_thrust()
    page.handle_line("HOVER est_n=8.657 std_n=1.500 converged=0 learning=0 samples=620 rejected=549 "
                     "innov=5.200 init_n=14.250 meas_std=0.153 gate=0x1c airborne=0 above_m=-0.002")
    assert page.xy_target_var.get() == "14.25"
    assert "尚未收敛" in page.status_var.get() and "14.25" in page.status_var.get()
    page.xy_target_var.set("12")
    page.read_hover_thrust()
    page.handle_line("HOVER est_n=8.657 std_n=1.500 converged=0 init_n=0.000")
    assert page.xy_target_var.get() == "12" and "手填" in page.status_var.get()


def test_hover_button_sends_nothing_while_a_transaction_waits_for_an_echo(page):
    drive_xy(page)
    page.handle_line(START_LINE)
    count = len(page.panel.transport.lines)
    page.workflow.awaiting = "SYSID XY inject=tilt win_mm=150 mass_g=1184"
    page.read_hover_thrust()
    assert len(page.panel.transport.lines) == count


# ---------------------------------------------------------------- 溯源与数据质量


def test_xystart_is_stored_under_xy_and_the_request_under_xy_request(page):
    run_xy(page, end_line="SYSID end run=3 state=done reason=complete dropped=0")
    snapshot = page.workflow.snapshot
    assert snapshot["mode"] == "5"
    assert snapshot["xy"]["xy_x0_mm"] == "12" and snapshot["xy"]["xy_inject"] == "tilt"
    assert snapshot["xy_request"]["inject"] == "tilt" and snapshot["xy_request"]["mass_g"] == 1184
    assert snapshot["xy_request"]["control"] == "openloop"
    assert snapshot["xy_request"]["target_cn"] == 1350
    assert page.workflow.data_error == ""


def test_a_provenance_that_disagrees_with_the_request_marks_the_run_diagnostic_only(page):
    drive_xy(page)
    page.handle_line(START_LINE)
    page.handle_line(xystart_line(page, xy_win_mm=300))
    assert "xy_win_mm" in page.workflow.data_error
    assert page.panel.transport.lines[-1] == "SYSID STOP"


def test_psi_provenance_tolerates_one_milliradian_of_rounding(page):
    drive_xy(page)
    page.handle_line(START_LINE)
    page.handle_line(xystart_line(page, xy_psi_mrad=page.workflow.expected["psi_mrad"] + 1))
    assert page.workflow.data_error == ""


def test_a_run_without_provenance_is_flagged(page):
    drive_xy(page)
    page.handle_line(START_LINE)
    rows = tilt_series(seconds=2.0)[1]
    feed_run(page, rows)
    page.handle_line("SYSID end run=3 state=done reason=complete dropped=0")
    assert "缺少飞控开跑溯源（SYSID XYSTART）" in page.workflow.data_error


def test_foreign_mode_flag_bits_are_refused_but_the_xy_bit_is_not_required(page):
    drive_xy(page)
    page.handle_line(START_LINE)
    page.handle_line(xystart_line(page))
    rows = to_raw(tilt_series(seconds=1.0)[1])[:5]
    page.accept(xy_frame(3, rows, flags=FLAG_FIRST_BATCH | 0x20, base_us=1000))
    assert "采样模式与本轮快照不符" in page.workflow.data_error


# ---------------------------------------------------------------- 记录解码与改名


def test_the_tail_fields_are_decoded_by_the_schema_then_renamed_for_xy(page):
    drive_xy(page)
    page.handle_line(START_LINE)
    page.handle_line(xystart_line(page))
    rows = [dict(height=-253, height_raw=41, height_sp=-300, vz=-127, vz_sp=-150, az=1234,
                 vbat=12300, angle=-175, angle_sp=-180, thrust=1350)] * 5
    page.accept(xy_frame(3, rows, flags=FLAG_FIRST_BATCH | FLAG_LAST_BATCH, base_us=1000))
    raw = page.samples[0]
    assert raw["height"] == pytest.approx(-0.0253) and raw["vz"] == pytest.approx(-0.127)
    from panel_lib.pages.sysid._core import xy_rename_samples
    renamed = xy_rename_samples(page.samples)[0]
    assert renamed["pos_u"] == pytest.approx(-0.0253)          # 沿 u 位置 [m]
    assert renamed["pos_perp"] == pytest.approx(0.0041)        # 垂直于 u 的位置 [m]
    assert renamed["pos_sp_u"] == pytest.approx(-0.03)
    assert renamed["vel_u"] == pytest.approx(-0.127)
    assert renamed["vel_sp_u"] == pytest.approx(-0.15)
    assert renamed["acc_u"] == pytest.approx(1.234)
    assert renamed["vbat"] == pytest.approx(12.3)
    assert renamed["angle"] == pytest.approx(-0.0175) and renamed["thrust"] == pytest.approx(13.5)
    assert not {"height", "height_raw", "height_sp", "vz", "vz_sp", "az"} & set(renamed)
    assert "height" in raw                                     # 原始样本与存档不动


def test_rename_does_not_touch_the_input_rows():
    rows = [dict(height=1.0, vz=2.0, angle=3.0)]
    assert xy_analysis.rename_samples(rows) == [dict(pos_u=1.0, vel_u=2.0, angle=3.0)]
    assert rows == [dict(height=1.0, vz=2.0, angle=3.0)]


def test_the_word_height_never_shows_on_the_xy_page(page):
    run_xy(page, end_line="SYSID end run=3 state=done reason=complete dropped=0")
    plot = page.xy_plot
    assert plot.figure is not None
    labels = []
    for axis in (plot.pos_axis, plot.vel_axis, plot.angle_axis, plot.thrust_axis):
        labels += [text.get_text() for text in axis.get_legend().get_texts()] if axis.get_legend() else []
        labels += [axis.get_ylabel(), axis.get_xlabel()]
    assert labels and not any("height" in label.lower() for label in labels)
    assert any("pos_u" in label for label in labels) and any("vel_u" in label for label in labels)
    assert any("angle_sp" in label for label in labels)
    source = (ROOT / "tools/panel_lib/pages/sysid/horizontal.py").read_text(encoding="utf-8")
    source += (ROOT / "tools/panel_lib/pages/sysid/xy_section.py").read_text(encoding="utf-8")
    assert "height" not in source.replace("height_", "")


def test_tilt_plot_does_not_draw_the_not_applicable_references(page):
    run_xy(page, inject="tilt", end_line="SYSID end run=3 state=done reason=complete dropped=0")
    labels = [text.get_text() for text in page.xy_plot.pos_axis.get_legend().get_texts()]
    assert all("pos_sp_u" not in label for label in labels)


def test_closed_loop_plot_draws_position_and_velocity_references(page):
    run_xy(page, inject="pos", end_line="SYSID end run=3 state=done reason=complete dropped=0")
    pos_labels = [text.get_text() for text in page.xy_plot.pos_axis.get_legend().get_texts()]
    vel_labels = [text.get_text() for text in page.xy_plot.vel_axis.get_legend().get_texts()]
    assert any("pos_sp_u" in label for label in pos_labels)
    assert any("vel_sp_u" in label for label in vel_labels)


# ---------------------------------------------------------------- 整轮：结果、存档与不拟合


def test_a_finished_tilt_run_is_analysed_on_the_page_and_not_fitted(page, monkeypatch, tmp_path):
    no_fit(monkeypatch)
    from panel_lib.pages.sysid import workflow
    monkeypatch.setattr(workflow, "_xy_directory", lambda: tmp_path / "xy" / "2026-09-30")
    run_xy(page, inject="tilt", end_line="SYSID end run=3 state=done reason=complete dropped=0")
    text = page.xy_result_var.get()
    assert "本轮水平槽设置（飞控开跑时报告）" in text and "注入 tilt" in text and "托住推力 13.5 N" in text
    assert "倾角 → 加速度增益" in text and "静摩擦门槛角" in text and "光流相对 IMU 的滞后" in text
    assert "height" not in text
    assert "本轮采集完整" in text
    assert "分析见「3 · 结果」" in page.analysis_note
    assert page.banner_var.get() == "完成"


def test_a_finished_closed_loop_run_reports_step_metrics(page, monkeypatch, tmp_path):
    no_fit(monkeypatch)
    from panel_lib.pages.sysid import workflow
    monkeypatch.setattr(workflow, "_xy_directory", lambda: tmp_path)
    run_xy(page, inject="vel", end_line="SYSID end run=3 state=done reason=complete dropped=0")
    text = page.xy_result_var.get()
    assert "速度闭环阶跃：识别到 3 次" in text and "90% 上升时间" in text and "超调" in text
    assert "末值误差" in text and "稳态抖动" in text


def test_the_archive_goes_to_the_xy_directory_with_the_alt_layout(page, monkeypatch, tmp_path):
    from panel_lib.pages.sysid import workflow
    root = tmp_path / "xy" / "2026-09-30"
    monkeypatch.setattr(workflow, "_xy_directory", lambda: root)
    run_xy(page, end_line="SYSID end run=3 state=done reason=complete dropped=0")
    wait_for(page, lambda: page.workflow.saved is not None and not page.workflow.saving)
    folder = page.workflow.saved
    assert folder.parent == root and re.fullmatch(r"xy_\d{6}_[0-9a-f]{8}", folder.name)
    with (folder / "samples.csv").open(newline="", encoding="utf-8") as handle:
        table = list(csv.DictReader(handle))
    assert len(table) == len(tilt_series()[1])
    assert {"t_us", "run_id", "gap", "height", "vz", "az", "angle", "angle_sp"} <= set(table[0])
    conditions = json.loads((folder / "conditions.json").read_text(encoding="utf-8"))
    assert conditions["mode"] == "5"
    assert conditions["xy"]["xy_inject"] == "tilt" and conditions["xy"]["xy_y0_mm"] == "-7"
    assert conditions["xy_request"]["inject"] == "tilt" and conditions["xy_request"]["win_mm"] == 150
    assert conditions["end"]["state"] == "done"


def test_the_default_archive_location_is_data_identification_xy_by_date():
    from panel_lib.pages.sysid import workflow
    folder = workflow._xy_directory()
    assert folder.parent.name == "xy" and folder.parent.parent.name == "identification"
    assert re.fullmatch(r"\d{4}-\d{2}-\d{2}", folder.name)


def test_xy_archive_does_not_leak_into_the_attitude_directory(page, monkeypatch, tmp_path):
    from panel_lib.pages.sysid import workflow
    monkeypatch.setattr(workflow, "_attitude_directory", lambda: tmp_path / "attitude")
    monkeypatch.setattr(workflow, "_xy_directory", lambda: tmp_path / "xy")
    drive_xy(page)
    page.handle_line(START_LINE)
    assert page.workflow.archive_directory().parent == tmp_path / "xy"
    assert page.workflow.archive_directory().name.startswith("xy_")


# ---------------------------------------------------------------- 中止原因文案


@pytest.mark.parametrize("reason,what", [("xy_window", "沿槽位移超出出窗余量"),
                                          ("xy_flow_invalid", "光流速度/位置失效")])
def test_xy_abort_reasons_are_explained_in_chinese(reason, what):
    text, step = reasons.explain_end(reason, "5")
    assert what in text and step
    assert reason not in text
    assert reasons.explain_end(reason, "XY") == (text, step)


def test_xy_window_advice_points_at_the_page_settings():
    _what, step = reasons.explain_end("xy_window", "5")
    assert "出窗余量" in step and "挡块" in step


def test_attitude_gates_are_soft_stops_on_the_xy_page():
    residual = reasons.explain_end("axis_residual", "5")
    assert "软停" in residual[0] and residual != reasons.explain_end("axis_residual", "0")
    assert "软停" in reasons.explain_end("angle_limit", "5")[0]
    assert reasons.explain_end("axis_residual", "4") != residual      # ALT 仍是自己的文案


def test_an_xy_abort_is_shown_on_the_xy_result_and_the_banner(page, monkeypatch, tmp_path):
    from panel_lib.pages.sysid import workflow
    monkeypatch.setattr(workflow, "_xy_directory", lambda: tmp_path)
    run_xy(page, rows=tilt_series(seconds=2.0)[1],
           end_line="SYSID end run=3 state=aborted reason=xy_window dropped=0")
    assert "沿槽位移超出出窗余量" in page.xy_result_var.get()
    assert "数据不完整，不能分析" in page.xy_result_var.get()
    assert "中止：沿槽位移超出出窗余量" in page.banner_var.get()
    assert "xy_window" in page.status_var.get()


@pytest.mark.parametrize("line,hint", [
    ("ERR sysid xy needs auto throttle", "托住推力"),
    ("ERR sysid xy amp over limit: inject=pos", "tilt ≤ 0.10 rad"),
    ("ERR sysid xy flow invalid", "光流"),
    ("ERR sysid xy range invalid", "测距"),
    ("ERR sysid xy something new", "水平槽辨识开始前自检没过"),
    ("ERR usage SYSID XY", "命令格式不对"),
    ("ERR unknown sysid subcmd XY", "太旧"),
])
def test_xy_precheck_errors_are_translated(line, hint):
    text = reasons.explain_error(line)
    assert hint in text and line in text


def test_a_precheck_error_during_the_transaction_reaches_the_status_line(page):
    drive_xy(page)
    page.handle_line(START_LINE)
    page.workflow.awaiting = "SYSID START"
    page.handle_line("ERR sysid xy flow invalid")
    assert "光流" in page.status_var.get()


# ---------------------------------------------------------------- 状态条


def test_banner_walks_through_the_xy_phases(page):
    drive_xy(page)
    page.handle_line(START_LINE)
    page.phase = "ramp_up"
    page.refresh_banner()
    assert "托住机体" in page.banner_detail_var.get() and page.banner_var.get() == "升推力…"
    page.phase = "preroll"
    page.refresh_banner()
    assert page.banner_var.get() == "前导中…"
    page.phase = "excite"
    page.refresh_banner()
    assert "沿水平槽平移" in page.banner_detail_var.get()


def test_banner_before_start_is_xy_specific(page):
    page.set_front_view("XY")
    page.handle_line(thr_line())
    page.refresh_banner()
    assert page.banner_var.get() == "就绪（水平槽辨识），可以开始"
    assert "挡块" in page.banner_detail_var.get()


# ---------------------------------------------------------------- 设置存档


def test_xy_settings_are_remembered_and_restored(page):
    def custom(p):
        p.xy_win_var.set("220")
        p.xy_extra_mass_var.set("40.5")
        p.xy_max_pct_var.set("80")
    drive_xy(page, inject="pos", target="12.8", customize=custom)
    data = json.loads(settings_file().read_text(encoding="utf-8"))
    assert data["xy_inject"] == "pos" and data["xy_win_mm"] == 220
    assert data["xy_extra_mass_g"] == 40.5 and data["xy_target_thrust_n"] == 12.8
    assert data["xy_max_throttle_pct"] == 80 and data["mode"] == "XY"
    reopened = fresh_page(page)
    assert reopened.xy_inject_var.get() == "pos" and reopened.xy_win_var.get() == "220"
    assert reopened.xy_target_var.get() == "12.8" and reopened.xy_max_pct_var.get() == "80"
    assert reopened.xy_extra_mass_var.get() == "40.5"
    pos = xy_config.INJECT_PRESETS["pos"]
    assert (reopened.amp_var.get(), reopened.hold_var.get()) == (pos["amp"], pos["hold"])   # pos 的默认激励


def test_old_settings_without_xy_keys_still_load(page):
    settings_file().write_text(json.dumps({"version": 1, "mode": "ANGLE", "alt_inject": "vel"}),
                               encoding="utf-8")
    reopened = fresh_page(page)
    assert reopened.xy_inject_var.get() == "tilt" and reopened.xy_win_var.get() == "150"
    assert reopened.xy_max_pct_var.get() == "90"
    assert "无效" not in reopened.status_var.get()


def test_bad_xy_settings_fall_back_to_defaults_with_a_note(page):
    settings_file().write_text(json.dumps({"version": 1, "xy_inject": "force", "xy_win_mm": 9999,
                                           "xy_max_throttle_pct": 100}), encoding="utf-8")
    reopened = fresh_page(page)
    assert reopened.xy_inject_var.get() == "tilt" and reopened.xy_win_var.get() == "150"
    assert "3 项无效" in reopened.status_var.get()


# ---------------------------------------------------------------- xy_analysis：tilt


def test_tilt_analysis_recovers_gain_friction_threshold_and_flow_lag_without_friction():
    times, rows = tilt_series(static_acc=0.0, kinetic_acc=0.0)
    result = xy_analysis.analyse_conditions(times, rows, {"xy_request": {"inject": "tilt"}})
    assert result["gain"] == pytest.approx(0.9, abs=0.03)
    assert result["gain_vs_measured_angle"] == pytest.approx(0.9, abs=0.03)
    assert result["flow_lag_s"] == pytest.approx(0.06, abs=0.011)
    assert abs(result["kinetic_friction_m_s2"]) < 0.02
    assert result["break_angle_rad"] < math.radians(0.6)        # 没有摩擦：一给倾角就动


def test_tilt_analysis_with_stiction_finds_the_breakaway_angle():
    times, rows = tilt_series(gain=0.9, static_acc=0.4, kinetic_acc=0.15)
    result = xy_analysis.analyse_conditions(times, rows, {"xy_request": {"inject": "tilt"}})
    truth = math.degrees(math.atan(0.4 / (0.9 * GRAVITY)))
    assert math.degrees(result["break_angle_rad"]) == pytest.approx(truth, abs=0.6)
    assert result["break_angle_cmd_rad"] == pytest.approx(result["break_angle_rad"], abs=1e-3)
    assert result["gain"] == pytest.approx(0.9, abs=0.1)
    assert result["kinetic_friction_m_s2"] == pytest.approx(0.15, abs=0.06)
    assert result["flow_lag_s"] == pytest.approx(0.06, abs=0.031)
    assert result["break_direction"] in (1, -1) and result["break_time_s"] > 1.5


@pytest.mark.parametrize("gain", [0.6, 1.0])
def test_tilt_gain_tracks_the_true_gain(gain):
    times, rows = tilt_series(gain=gain, static_acc=0.0, kinetic_acc=0.0)
    result = xy_analysis.analyse_conditions(times, rows, {"xy_request": {"inject": "tilt"}})
    assert result["gain"] == pytest.approx(gain, abs=0.04)


def test_tilt_lag_follows_the_flow_delay():
    for lag in (0.0, 0.12):
        times, rows = tilt_series(static_acc=0.0, kinetic_acc=0.0, lag_s=lag)
        result = xy_analysis.analyse_conditions(times, rows, {"xy_request": {"inject": "tilt"}})
        assert result["flow_lag_s"] == pytest.approx(lag, abs=0.021)


def test_tilt_measured_angle_offset_is_removed_as_the_quiet_baseline():
    times, rows = tilt_series(static_acc=0.0, kinetic_acc=0.0, angle_offset=0.05)
    result = xy_analysis.analyse_conditions(times, rows, {"xy_request": {"inject": "tilt"}})
    assert result["base_angle_rad"] == pytest.approx(0.05, abs=1e-3)
    assert result["gain_vs_measured_angle"] == pytest.approx(0.9, abs=0.03)


def test_tilt_run_that_never_slides_says_so():
    times, rows = tilt_series(static_acc=50.0)            # 静摩擦大得推不动
    result = xy_analysis.analyse_conditions(times, rows, {"xy_request": {"inject": "tilt"}})
    assert result["break_angle_rad"] is None
    assert any("没滑动" in warning for warning in result["warnings"])
    assert "整轮没滑动" in xy_analysis.summary_text(result)


def test_tilt_summary_is_chinese_and_carries_every_metric():
    times, rows = tilt_series()
    text = xy_analysis.summary_text(
        xy_analysis.analyse_conditions(times, rows, {"xy_request": {"inject": "tilt"}}))
    for part in ("倾角 → 加速度增益", "动摩擦等效加速度", "静摩擦门槛角", "光流相对 IMU 的滞后", "ms"):
        assert part in text


@pytest.mark.parametrize("make,message", [
    (lambda: (list(range(10)), [dict(vz=0.0)] * 10), "样本太少"),
    (lambda: ([i * 0.004 for i in range(100)], [dict(vz=0.0, angle=0.0, angle_sp=0.0, az=0.0,
                                                     height=0.0, height_sp=0.0, vz_sp=0.0)] * 100),
     "不足 1 秒"),
])
def test_tilt_analysis_refuses_short_records(make, message):
    times, rows = make()
    with pytest.raises(ValueError, match=message):
        xy_analysis.analyse_tilt(times, xy_analysis.rename_samples(rows))


def test_tilt_analysis_without_any_excitation_refuses():
    times, rows = tilt_series()
    for row in rows:
        row["angle_sp"] = 0.0
    with pytest.raises(ValueError, match="没有激励"):
        xy_analysis.analyse_conditions(times, rows, {"xy_request": {"inject": "tilt"}})


# ---------------------------------------------------------------- xy_analysis：vel / pos 阶跃


def test_first_order_position_step_metrics():
    times, rows = step_series(kind="pos", tau=0.1)
    result = xy_analysis.analyse_conditions(times, rows, {"xy_request": {"inject": "pos"}})
    assert len(result["steps"]) == 3 and result["followed"] == 3
    # 一阶 τ = 0.1 s：90% 上升 ≈ 2.3τ = 0.23 s，再加参考 0.15 s 斜坡带来的滞后（约 0.1 s）
    assert 0.25 < result["rise90_s"] < 0.40
    assert result["overshoot"] < 0.01
    assert result["final_error"] < 1e-4 and result["jitter"] < 1e-4


def test_second_order_velocity_step_shows_overshoot():
    times, rows = step_series(kind="vel", zeta=0.5, wn=10.0)
    result = xy_analysis.analyse_conditions(times, rows, {"xy_request": {"inject": "vel"}})
    assert result["inject"] == "vel"
    assert 0.08 < result["overshoot"] < 0.22          # 理论 16.3%，参考有斜坡所以略小
    assert result["followed"] == 3 and 0.15 < result["rise90_s"] < 0.45


def test_final_error_and_jitter_are_reported_with_noise_and_offset():
    times, rows = step_series(kind="pos", tau=0.08, noise=0.001)
    for row in rows:
        row["height"] -= 0.002                          # 稳态偏低 2 mm
    result = xy_analysis.analyse_conditions(times, rows, {"xy_request": {"inject": "pos"}})
    assert result["final_error"] == pytest.approx(0.002, abs=0.0008)
    assert result["jitter"] == pytest.approx(0.001, abs=0.0004)


def test_a_step_the_measurement_does_not_follow_is_flagged():
    times, rows = step_series(kind="pos", tau=0.1)
    for row in rows:
        row["height"] *= 0.1                            # 只走了参考的 10%
    result = xy_analysis.analyse_conditions(times, rows, {"xy_request": {"inject": "pos"}})
    assert result["followed"] == 0 and result["rise90_s"] is None
    assert any("没跟上" in warning for warning in result["warnings"])


def test_steps_metrics_summary_is_chinese_with_units():
    times, rows = step_series(kind="pos")
    text = xy_analysis.summary_text(
        xy_analysis.analyse_conditions(times, rows, {"xy_request": {"inject": "pos"}}))
    assert "位置闭环阶跃" in text and "90% 上升时间" in text and "mm" in text
    times, rows = step_series(kind="vel")
    text = xy_analysis.summary_text(
        xy_analysis.analyse_conditions(times, rows, {"xy_request": {"inject": "vel"}}))
    assert "速度闭环阶跃" in text and "mm/s" in text


def test_a_reference_that_stays_zero_is_refused():
    times, rows = step_series(kind="pos")
    for row in rows:
        row["height_sp"] = 0.0
    with pytest.raises(ValueError, match="参考全程为 0"):
        xy_analysis.analyse_conditions(times, rows, {"xy_request": {"inject": "pos"}})


def test_analysis_uses_the_request_then_the_firmware_provenance_for_the_inject_type():
    assert xy_analysis.run_inject({"xy_request": {"inject": "vel"}, "xy": {"xy_inject": "pos"}}) == "vel"
    assert xy_analysis.run_inject({"xy": {"xy_inject": "pos"}}) == "pos"
    assert xy_analysis.run_inject({}) is None
    times, rows = step_series(kind="pos")
    with pytest.raises(ValueError, match="注入类型不明"):
        xy_analysis.analyse_conditions(times, rows, {})


def test_analysis_refuses_records_without_the_xy_fields():
    times, rows = step_series(kind="pos")
    for row in rows:
        del row["az"]
    with pytest.raises(ValueError, match="缺少 XY 字段：acc_u"):
        xy_analysis.analyse_conditions(times, rows, {"xy_request": {"inject": "pos"}})


def test_an_archived_run_can_be_analysed_offline(page, monkeypatch, tmp_path):
    from panel_lib.pages.sysid import workflow
    monkeypatch.setattr(workflow, "_xy_directory", lambda: tmp_path)
    run_xy(page, inject="pos", end_line="SYSID end run=3 state=done reason=complete dropped=0")
    wait_for(page, lambda: page.workflow.saved is not None and not page.workflow.saving)
    times, samples, conditions = xy_analysis.read_run_dir(page.workflow.saved)
    result = xy_analysis.analyse_conditions(times, samples, conditions)
    assert result["inject"] == "pos" and result["followed"] == 3

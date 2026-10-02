"""「Z 高度」页的光杆台架高度辨识模式 ALT（R-ALTID-1）：操作与存档。

控件在「Z 高度」页（altitude.py），状态与事务仍在内环页对象里（固件同一时间只跑一轮 SYSID）；
下面的夹具在每个用 `page` 的测试里把高度页挂到同一个对象上，和真面板一样。

报文照跨侧契约（2026-09-30，break 离地/滑落阈值取代开环 force）写：

* `SYSID MODE ALT` → 模式编号 4；
* `SYSID ALT inject=break|vel|pos mass_g=.. win_mm=.. lift_mm=.. bottom_mm=.. top_mm=..` 回一行带
  control（breakaway|closed_loop）的回显，拒绝回 `SYSID ALT event=rejected reason=running|range|usage`；
* 记录字段表末尾追加 height/height_raw/height_sp/vz/vz_sp/az/vbat；
* 溯源在 `SYSID start` 行末尾（alt_*），或紧跟的 `SYSID ALTSTART` 行（含 alt_bottom_mm/alt_top_mm）；
* `SYSID THR` 回报末尾带 alt_h_mm/alt_h_ok（8 样本平均测距），「记为槽底/槽顶」用它。

字段表照本批固件 `Driver/Src/drv_sysid_record.c` 的 sysid_fields（22 项、记录 v3、每条 44 字节）；
页面按 SCHEMA 自描述解码，不依赖这些缩放。批头 ALT 位 0x0100 照 `drv_sysid_record.h`（不在契约里，
页面不要求它）。
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

from test_sysid_page import (  # noqa: F401  页面夹具与报文工具
    FLAG_FIRST_BATCH, FLAG_LAST_BATCH, START_PARAMS, feed, fresh_page, page, settings_file,
    status_report, thr_line, wait_for,
)

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(autouse=True)
def _mount_altitude_view(request):
    """用到 `page` 的测试都挂上「Z 高度」页：高度的控件、图和结果区在那一页。"""
    if "page" not in request.fixturenames:
        return
    engine = request.getfixturevalue("page")
    from panel_lib.pages.sysid.altitude import SysIdAltitudePage
    from tkinter import ttk
    engine.test_alt_view = SysIdAltitudePage(engine.panel,
                                             ttk.Frame(engine.parent.winfo_toplevel()), engine)

#: 固件字段表（名字、单位、缩放、类型），末尾 7 项是本批追加的高度字段。
ALT_FIELDS = (("gx", "rad/s", 1e-3, "i16"), ("gy", "rad/s", 1e-3, "i16"),
              ("gz", "rad/s", 1e-3, "i16"), ("omega_sp", "rad/s", 1e-3, "i16"),
              ("alpha_ff", "rad/s^2", 1e-2, "i16"), ("tilt_x", "rad", 1e-4, "i16"),
              ("tilt_y", "rad", 1e-4, "i16"), ("thrust", "N", 1e-2, "i16"),
              ("angle", "rad", 1e-4, "i16"), ("erpm", "rpm", 4.0, "u16"),
              ("torque", "N*m", 1e-4, "i16"), ("angle_sp", "rad", 1e-4, "i16"),
              ("offset_us", "us", 1.0, "u16"), ("erpm_lower", "rpm", 4.0, "u16"),
              ("servo_tilt", "rad", 1e-4, "i16"),
              ("height", "m", 1e-4, "i16"), ("height_raw", "m", 1e-4, "i16"),
              ("height_sp", "m", 1e-4, "i16"), ("vz", "m/s", 1e-3, "i16"),
              ("vz_sp", "m/s", 1e-3, "i16"), ("az", "m/s^2", 1e-3, "i16"),
              ("vbat", "V", 1e-3, "u16"))
NEW_FIELDS = ("height", "height_raw", "height_sp", "vz", "vz_sp", "az", "vbat")
FLAG_ALT = 0x0100


def schema_lines(fields, schema_hash):
    return [f"SYSID SCHEMA ver=3 n={len(fields)} hash={schema_hash:08X} rec={2 * len(fields)}"] + [
        f"SYSID FIELD idx={i} name={name} unit={unit} scale={scale:.9f} type={kind}"
        for i, (name, unit, scale, kind) in enumerate(fields)]


ALT_HASH = 0xA17A0004
SCHEMA_ALT_LINES = schema_lines(ALT_FIELDS, ALT_HASH)
#: 没有高度字段的 v3 表（旧固件，15 项）。
SCHEMA_V3_LINES = schema_lines(ALT_FIELDS[:15], 0x5E7B0003)
#: 机体参数：实测整机 1145.3 g（2026-09-27）。
ALT_PARAMS = START_PARAMS + (("airframe.mass_kg", "1.145300"),)


def alt_echo(expected: dict) -> str:
    return (f"SYSID ALT inject={expected['inject']} mass_g={expected['mass_g']} "
            f"win_mm={expected['win_mm']} lift_mm={expected['lift_mm']} "
            f"bottom_mm={expected['bottom_mm']} top_mm={expected['top_mm']} "
            f"control={expected['control']}")


def drive_alt(page, *, params=ALT_PARAMS, schema=SCHEMA_ALT_LINES, echo=alt_echo, inject=None,
              target="13.2", slot=("460", "620"), customize=None, **report):
    """「本轮做」选 ALT → 点开始 → 回字段表、参数；每条配置命令回整份报告，SYSID ALT 回一行。

    1184 g 的移动质量重力 11.61 N：离地搜索上限 13.2 N 在 [重力, 重力 + 5 N] 里。
    槽底/槽顶读数 460/620 mm（行程 160 mm）。"""
    page.mode_var.set("ALT")
    if inject is not None:
        page.alt_inject_var.set(inject)
        page.select_alt_inject()
    page.alt_target_var.set(target)
    if slot is not None:
        page.alt_bottom_var.set(slot[0])
        page.alt_top_var.set(slot[1])
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
        if awaiting.startswith("SYSID ALT "):
            feed(page, [echo(page.workflow.alt_expected)])
        else:
            feed(page, status_report(page.workflow.expected, **report))


START_LINE = ("SYSID start run=3 profile=1 amp_mrad_s=300 dur_ms=300 rate_hz=250 I=20000 ugm2 "
              "psi_mrad=785 auto=1 target_cn=1320")


def altstart_line(page, *, h0_mm=48, **changes):
    expected = page.workflow.alt_expected
    fields = dict(run=3, alt_inject=expected["inject"], alt_mass_g=expected["mass_g"],
                  alt_win_mm=expected["win_mm"], alt_lift_mm=expected["lift_mm"],
                  alt_h0_mm=h0_mm, alt_control=expected["control"],
                  alt_target_cn=page.workflow.expected["target_cn"],
                  alt_bottom_mm=expected["bottom_mm"], alt_top_mm=expected["top_mm"])
    fields.update(changes)
    return "SYSID ALTSTART " + " ".join(f"{key}={value}" for key, value in fields.items())


def alt_frame(run, rows, *, flags, base_us, dt_us=4000):
    """`rows`：每条一个 {字段名: 定点原始值}，没给的字段填 0。"""
    header = struct.pack("<BBHIIHH", 3, len(rows), run, ALT_HASH, base_us, dt_us,
                         flags | FLAG_ALT)
    fmt = "<" + "".join("h" if kind == "i16" else "H" for *_rest, kind in ALT_FIELDS)
    def value(row, index, name):             # offset_us：相对批头的采样时刻（固件照实填）
        return row.get(name, index * dt_us if name == "offset_us" else 0)
    return header + b"".join(
        struct.pack(fmt, *(value(row, index, name) for name, *_rest in ALT_FIELDS))
        for index, row in enumerate(rows))


def alt_rows(n):
    """高度约 0.1 m 上下 ±0.02 m，推力约 11.6 N，电池 12.3 V（定点原始值）。"""
    rows = []
    for i in range(n):
        height = 1000 + int(200 * math.sin(i / 8))
        rows.append(dict(gx=10, gy=-10, thrust=1160 + int(30 * math.sin(i / 8)), erpm=3000,
                         erpm_lower=2800, height=height,
                         height_raw=height + 5, height_sp=1000 + (200 if (i // 25) % 2 else -200),
                         vz=int(40 * math.cos(i / 8)), az=50, vbat=12300))
    return rows


SETTLE_LINE = "SYSID PHASE run=3 phase=settle pulse_us=1450 thrust_cn=1290"


def run_alt(page, n=100, *, start_line=START_LINE, extra_lines=(), inject="break"):
    drive_alt(page, inject=inject)
    assert page.panel.transport.lines[-1] == "SYSID START"
    page.handle_line(start_line)
    if not any(line.startswith("SYSID ALTSTART ") for line in extra_lines):
        h0 = next((int(token.partition("=")[2]) for token in start_line.split()
                   if token.startswith("alt_h0_mm=")), 48)
        page.handle_line(altstart_line(page, h0_mm=h0))
    feed(page, extra_lines)
    if inject == "break":
        page.handle_line(SETTLE_LINE)           # 离地粗判：break 轮至少要到 settle
    rows = alt_rows(n)
    if inject == "break":
        for row in rows:
            row["height_sp"] = row["vz_sp"] = 0
    per = 5                                   # 固件每批最多 5 条（44 字节一条）
    for start in range(0, n, per):
        flags = (FLAG_FIRST_BATCH if start == 0 else 0) | (FLAG_LAST_BATCH if start + per >= n else 0)
        page.accept(alt_frame(3, rows[start:start + per], flags=flags, base_us=1000 + start * 4000))


def no_fit(monkeypatch):
    """ALT 轮绝不能跑姿态拟合：一旦调用就让测试失败。"""
    from panel_lib.pages.sysid import analysis

    def forbidden(*_a, **_k):
        raise AssertionError("ALT 轮不应调用姿态/舵机拟合")
    for name in ("fit_inner_loop", "fit_inner_loop_multi", "fit_servo_only"):
        monkeypatch.setattr(analysis, name, forbidden)


def archive_into(monkeypatch, root):
    from panel_lib.pages.sysid import workflow
    monkeypatch.setattr(workflow, "_attitude_directory", lambda: root)


# ---------------------------------------------------------------- 模式与设置


def test_alt_is_appended_as_mode_four_and_old_modes_keep_their_numbers():
    sys.path.insert(0, str(ROOT / "tools"))
    try:
        from panel_lib.pages.sysid import settings_store
        from panel_lib.pages.sysid.alt_config import ALT_MODE_CODE
    finally:
        sys.path.pop(0)
    assert settings_store.MODES[:5] == ("FF", "RATE", "ANGLE", "SERVO", "ALT")
    assert settings_store.MODES[5:] == ("XY", "YAW")      # 水平槽 XY 追加为 5，吊绳偏航 YAW 追加为 6
    assert settings_store.MODES.index("ALT") == ALT_MODE_CODE == 4


def test_the_mode_hint_warns_that_motors_spin_and_the_body_moves(page):
    page.mode_var.set("ALT")
    hint = page.mode_hint_var.get()
    assert "电机会转" in hint and "上下移动" in hint and "限位" in hint and "测距" in hint


def test_old_settings_without_alt_keys_still_load(page):
    settings_file().write_text(json.dumps({"version": 1, "mode": "SERVO", "servo_tilt_deg": 6}),
                               encoding="utf-8")
    again = fresh_page(page)
    assert again.mode_var.get() == "SERVO" and again.servo_tilt_var.get() == "6"
    assert again.alt_inject_var.get() == "break" and again.alt_extra_mass_var.get() == "38.8"
    assert (again.alt_win_var.get(), again.alt_lift_var.get()) == ("70", "50")
    assert (again.alt_bottom_var.get(), again.alt_top_var.get()) == ("", "")
    assert again.status_var.get() == ""


def test_old_settings_with_the_retired_force_injection_fall_back_to_break(page):
    """2026-09-30 前存的 alt_inject=force（开环已删除）：照常读，回落默认 break，不报坏项。"""
    settings_file().write_text(json.dumps({"version": 1, "mode": "ALT", "alt_inject": "force",
                                           "alt_win_mm": 70, "alt_target_thrust_n": 13.0}),
                               encoding="utf-8")
    again = fresh_page(page)
    assert again.alt_inject_var.get() == "break" and again.alt_target_var.get() == "13"
    assert again.status_var.get() == ""


def test_alt_settings_are_remembered_with_the_injection_preset(page):
    page.alt_extra_mass_var.set("40")
    page.alt_win_var.set("200")
    page.alt_lift_var.set("80")
    drive_alt(page, inject="vel", slot=("455", "618"))
    saved = json.loads(settings_file().read_text(encoding="utf-8"))
    assert saved["mode"] == "ALT" and saved["alt_inject"] == "vel"
    assert (saved["alt_extra_mass_g"], saved["alt_win_mm"], saved["alt_lift_mm"]) == (40, 200, 80)
    assert (saved["alt_bottom_mm"], saved["alt_top_mm"]) == (455, 618)
    again = fresh_page(page)
    assert again.mode_var.get() == "ALT" and again.alt_inject_var.get() == "vel"
    assert (again.alt_extra_mass_var.get(), again.alt_win_var.get(), again.alt_lift_var.get()) == (
        "40", "200", "80")
    assert (again.alt_bottom_var.get(), again.alt_top_var.get()) == ("455", "618")
    # 「实验类型」的角速度预设不能盖掉高度激励。
    assert (again.amp_var.get(), again.hold_var.get(), again.repeat_var.get()) == ("0.04", "1000", "3")


def test_empty_slot_readings_are_saved_as_null_and_load_back_empty(page):
    from panel_lib.pages.sysid import settings_store
    page.alt_bottom_var.set("")
    page.alt_top_var.set("612")
    saved = settings_store.collect_from_page(page)
    assert saved["alt_bottom_mm"] is None and saved["alt_top_mm"] == 612
    settings_store.save(saved, settings_file())
    assert json.loads(settings_file().read_text(encoding="utf-8"))["alt_bottom_mm"] is None
    again = fresh_page(page)
    assert (again.alt_bottom_var.get(), again.alt_top_var.get()) == ("", "612")
    assert again.status_var.get() == ""


# ---------------------------------------------------------------- 注入类型与默认激励


def test_break_is_the_default_and_writes_a_rate_not_a_pulse(page):
    """break 的「幅值」是慢升/慢降速率 [N/s]；剖面给一个能过体检的短阶跃，固件不用它。"""
    assert page.alt_inject_var.get() == "break"
    page.mode_var.set("ALT")
    page.alt_inject_combo.current(0)
    page.alt_inject_combo.event_generate("<<ComboboxSelected>>")
    assert (page.profile_var.get(), page.amp_var.get(), page.hold_var.get(), page.ramp_var.get(),
            page.repeat_var.get(), page.dur_var.get()) == ("step", "0.5", "100", "100", "1", "300")
    assert page.excitation().total_ms() == 300, "默认剖面要过激励体检"
    assert page.amp_label_var.get() == "速率 [N/s]"
    assert page.amp_hint_var.get() == "慢升/慢降速率，默认 0.5 N/s，上限 1 N/s；剖面不用"
    assert "N/s" in page.alt_amp_note_var.get() and "剖面" in page.alt_amp_note_var.get()
    assert "离地/滑落阈值" in page.alt_inject_hint_var.get()


@pytest.mark.parametrize("index,inject,amp,hold,repeat,dur,unit", [
    (1, "vel", "0.04", "1000", "3", "6000", "m/s"),
    (2, "pos", "0.04", "2000", "2", "8000", "m"),
])
def test_choosing_an_injection_writes_its_default_excitation(page, index, inject, amp, hold, repeat,
                                                              dur, unit):
    page.mode_var.set("ALT")
    page.alt_inject_combo.current(index)
    page.alt_inject_combo.event_generate("<<ComboboxSelected>>")
    assert page.alt_inject_var.get() == inject
    assert (page.profile_var.get(), page.amp_var.get(), page.hold_var.get(),
            page.repeat_var.get(), page.dur_var.get()) == ("doublet", amp, hold, repeat, dur)
    spec = page.excitation()
    assert spec.total_ms() == 2 * int(hold) * int(repeat), "默认激励不能被总时长截掉"
    assert page.amp_label_var.get() == f"幅值 [{unit}]"
    assert f"幅值单位 {unit}" in page.amp_hint_var.get()
    assert unit in page.alt_amp_note_var.get()


def test_the_injection_preset_is_only_written_in_alt_mode_and_is_undone_on_leaving(page):
    page.alt_inject_combo.current(2)                         # pos，但还在 FF
    page.alt_inject_combo.event_generate("<<ComboboxSelected>>")
    assert page.amp_var.get() == "0.065" and page.amp_label_var.get() == "幅值 [rad/s]"
    page.mode_var.set("ALT")                                 # 切进 ALT：写入 pos 的默认激励
    assert (page.amp_var.get(), page.hold_var.get()) == ("0.04", "2000")
    assert "高度辨识" in str(page.excitation_box.cget("text"))
    page.amp_var.set("0.05")                                 # 作者仍可改
    assert page.excitation().amplitude_rad_s == pytest.approx(0.05)
    page.mode_var.set("FF")                                  # 切回：换回实验类型的角速度预设
    assert (page.amp_var.get(), page.hold_var.get(), page.repeat_var.get()) == ("0.065", "250", "16")
    assert page.amp_label_var.get() == "幅值 [rad/s]"
    assert "期望角速度" in str(page.excitation_box.cget("text"))


def test_alt_never_autoscales_the_amplitude_from_the_rod_lever(page):
    """幅值建议按舵机摆幅（角速度）算，ALT 下不适用：不估算、不改值、不提醒。"""
    page.mode_var.set("ALT")
    feed(page, [f"PARAM name={name} value={value}" for name, value in ALT_PARAMS]
         + ["PARAM name=airframe.servo1_axis_z_m value=-0.050000",
            "PARAM name=airframe.servo2_axis_z_m value=-0.050000", thr_line()])
    page.refresh_amp_hint()
    assert page.amp_var.get() == "0.5"
    assert page.amp_swing_estimate() is None and page.amp_swing_warning() == ""
    assert "剖面不用" in page.amp_hint_var.get()
    page.alt_inject_var.set("vel")
    page.select_alt_inject()
    page.refresh_amp_hint()
    assert page.amp_var.get() == "0.04" and "不自动改幅值" in page.amp_hint_var.get()


def test_the_mass_hint_adds_the_rod_to_the_airframe_mass(page):
    assert "还没读到" in page.alt_mass_hint_var.get()
    feed(page, ["PARAM name=airframe.mass_kg value=1.145300"])
    assert "合计 1184.1 g = 机体 1145.3 g" in page.alt_mass_hint_var.get()
    assert "mass_g=1184" in page.alt_mass_hint_var.get()
    page.alt_extra_mass_var.set("abc")
    assert "要填" in page.alt_mass_hint_var.get()


# ---------------------------------------------------------------- 开跑前下发与核对


def test_alt_start_sends_and_verifies_the_alt_line(page):
    drive_alt(page)
    sent = page.panel.transport.lines
    assert "SYSID MODE ALT 3" in sent
    alt = sent.index("SYSID ALT inject=break mass_g=1184 win_mm=70 lift_mm=50 bottom_mm=460 top_mm=620")
    assert sent[alt - 1] == "SYSID MODE ALT 3", "SYSID ALT 排在 MODE 之后"
    assert sent[alt + 1].startswith("SYSID RIG ")
    assert sent[alt + 2].startswith("SYSID THROTTLE target_n=13.2 ")
    assert sent[-1] == "SYSID START"
    exc = next(line for line in sent if line.startswith("SYSID EXC "))
    assert "profile=step " in exc and "amp=0.5 " in exc, "break 的 amp 是慢升/慢降速率 0.5 N/s"
    assert page.workflow.expected["mode"] == 4
    assert page.workflow.alt_expected == dict(inject="break", mass_g=1184, win_mm=70, lift_mm=50,
                                              bottom_mm=460, top_mm=620, control="breakaway")


def test_the_mass_falls_back_to_the_ready_report(page):
    params = tuple(p for p in ALT_PARAMS if p[0] != "airframe.mass_kg")
    page.alt_extra_mass_var.set("0")
    drive_alt(page, params=params, mass_mg=1145300)
    assert ("SYSID ALT inject=break mass_g=1145 win_mm=70 lift_mm=50 bottom_mm=460 top_mm=620"
            in page.panel.transport.lines)
    assert page.panel.transport.lines[-1] == "SYSID START"


def test_no_airframe_mass_means_no_start(page):
    params = tuple(p for p in ALT_PARAMS if p[0] != "airframe.mass_kg")
    drive_alt(page, params=params, mass_mg=0)
    sent = page.panel.transport.lines
    assert not any(line.startswith("SYSID ALT") for line in sent)
    assert "SYSID START" not in sent
    assert "读不到机体质量" in page.status_var.get()


def test_non_alt_modes_never_send_the_alt_line(page):
    from test_sysid_page import drive_config
    drive_config(page)
    sent = page.panel.transport.lines
    assert sent[-1] == "SYSID START"
    assert not any(line.startswith("SYSID ALT") for line in sent)
    assert page.workflow.alt_expected == {} and "alt_request" not in (page.workflow.snapshot or {})


@pytest.mark.parametrize("echo,message", [
    (lambda e: alt_echo({**e, "mass_g": 1100}), "mass_g"),
    (lambda e: alt_echo({**e, "inject": "pos"}), "inject"),
    (lambda e: alt_echo({**e, "control": "closed_loop"}), "control"),
    (lambda e: alt_echo(e).replace(" control=breakaway", ""), "control"),
    (lambda e: alt_echo({**e, "bottom_mm": 0}), "bottom_mm"),
    (lambda e: alt_echo(e).replace(" top_mm=620", ""), "top_mm"),
    (lambda e: "SYSID ALT event=rejected reason=range", "超出范围"),
    (lambda e: "SYSID ALT event=rejected reason=running", "空闲"),
])
def test_a_wrong_or_refused_alt_echo_cancels_the_start(page, echo, message):
    drive_alt(page, echo=echo)
    assert "SYSID START" not in page.panel.transport.lines
    assert message in page.status_var.get()
    assert not any(line.startswith("SYSID RIG ") for line in page.panel.transport.lines)


def test_a_status_report_cannot_stand_in_for_the_alt_echo(page):
    """SYSID ALT 只回一行；别处的整份报告（以 LIMITS 结尾）不能被当成它的回显放行。"""
    limits = "SYSID LIMITS rate_hz=250 I_ugm2=20000 angle_mrad=349 resid_mrad_s=523 min_thrust_cn=200"
    drive_alt(page, echo=lambda _e: limits)
    assert page.workflow.awaiting.startswith("SYSID ALT ")
    assert not any(line.startswith("SYSID RIG ") for line in page.panel.transport.lines)
    feed(page, [alt_echo(page.workflow.alt_expected)])
    assert page.panel.transport.lines[-1].startswith("SYSID RIG ")


def test_old_firmware_refusing_the_alt_line_is_translated(page):
    drive_alt(page, echo=lambda _e: "ERR unknown sysid subcmd ALT")
    assert "SYSID START" not in page.panel.transport.lines
    assert "不认高度辨识设置" in page.status_var.get()


def test_the_inner_manual_throttle_box_never_turns_alt_into_manual_throttle(page):
    """高度辨识总是程序油门：内环页的「遥控器手动给油门」勾着也照样下发程序油门。"""
    page.manual_throttle_var.set(True)
    page.target_thrust_var.set("11.5")          # 内环页自己的目标推力，不许串到高度轮
    drive_alt(page)
    assert "SYSID THROTTLE target_n=13.2 max_pct=90" in page.panel.transport.lines
    assert page.workflow.expected["auto"] == 1
    assert page.panel.transport.lines[-1] == "SYSID START"


def test_break_requires_an_explicit_search_ceiling_before_any_command(page):
    drive_alt(page, target="")
    assert page.panel.transport.lines == []
    assert "明确填写" in page.banner_detail_var.get()
    assert "不会自动用机重" in page.alt_thrust_hint_var.get()
    assert page.alt_thrust_label_var.get() == "离地搜索上限 [N]"
    assert page.thrust_label_var.get() == "目标合推力 [N]", "内环页的标签不跟着高度变"


def test_the_ceiling_hint_shows_the_allowed_span_once_the_mass_is_known(page):
    page.mode_var.set("ALT")
    feed(page, ["PARAM name=airframe.mass_kg value=1.145300"])
    page.alt_target_var.set("")
    assert "11.61～16.61 N" in page.alt_thrust_hint_var.get()
    page.alt_target_var.set("13.2")
    assert "最多升到 13.2 N" in page.alt_thrust_hint_var.get()


def test_break_search_ceiling_below_two_newtons_is_refused_before_any_command(page):
    drive_alt(page, target="1.9")
    assert page.panel.transport.lines == []
    assert "至少 2 N" in page.banner_detail_var.get()


@pytest.mark.parametrize("target", ["abc", "-1"])
def test_break_bad_search_ceiling_never_suggests_using_empty_weight_default(page, target):
    drive_alt(page, target=target)
    assert page.panel.transport.lines == []
    assert "离地搜索上限" in page.banner_detail_var.get()
    assert "留空用机重" not in page.banner_detail_var.get()


def test_break_search_ceiling_below_the_run_weight_is_refused_before_the_alt_line(page):
    drive_alt(page, target="11.5")               # 1184 g 的重力 11.61 N
    sent = page.panel.transport.lines
    assert "SYSID START" not in sent and not any(line.startswith("SYSID ALT ") for line in sent)
    assert "低于本轮移动质量（1184 g）的重力 11.61 N" in page.status_var.get()
    assert "11.7～16.6 N" in page.status_var.get()


def test_closed_loop_alt_can_still_use_weight_default_and_longer_excitation(page):
    drive_alt(page, inject="vel", target="")
    assert "SYSID THROTTLE target_n=9.8 max_pct=90" in page.panel.transport.lines
    assert page.workflow.alt_expected["control"] == "closed_loop"
    assert page.alt_thrust_label_var.get() == "目标合推力 [N]"
    assert page.panel.transport.lines[-1] == "SYSID START"


@pytest.mark.parametrize("inject,amp", [("break", "1.1"), ("vel", "0.4"), ("pos", "0.2")])
def test_alt_amplitude_limits_follow_the_injection(page, inject, amp):
    page.mode_var.set("ALT")
    page.alt_inject_var.set(inject)
    page.select_alt_inject()
    page.amp_var.set(amp)
    drive_alt(page)
    assert page.panel.transport.lines == []
    assert "上限" in page.banner_detail_var.get()


def test_break_ceiling_is_also_capped_by_the_airframe_max_total_force(page):
    """飞控 SYSID THROTTLE 不收超过整机最大推力（airframe.max_total_force_n）的目标：
    提示范围按它截断，超了在发任何 ALT 命令前拦下并说明原因（2026-09-30 板上 15.65 N）。"""
    page.mode_var.set("ALT")
    feed(page, ["PARAM name=airframe.mass_kg value=1.145300",
                "PARAM name=airframe.max_total_force_n value=15.650306"])
    page.alt_target_var.set("")
    assert "11.61～15.65 N" in page.alt_thrust_hint_var.get()
    drive_alt(page, target="16", params=ALT_PARAMS + (("airframe.max_total_force_n", "15.650306"),))
    sent = page.panel.transport.lines
    assert "SYSID START" not in sent and not any(line.startswith("SYSID ALT ") for line in sent)
    assert "整机最大推力 15.65 N" in page.status_var.get()


def test_break_search_ceiling_above_the_run_weight_plus_five_newtons_is_refused(page):
    drive_alt(page, target="16.7")
    assert "SYSID START" not in page.panel.transport.lines
    assert not any(line.startswith("SYSID ALT ") for line in page.panel.transport.lines)
    assert "重力 + 5 N" in page.status_var.get()


@pytest.mark.parametrize("field,value", [("alt_win_var", "20"), ("alt_lift_var", "500"),
                                         ("alt_extra_mass_var", "-5"), ("alt_lift_var", "99.5"),
                                         ("alt_bottom_var", "460.5"), ("alt_top_var", "5000")])
def test_bad_alt_inputs_are_refused_before_sending(page, field, value):
    drive_alt(page, customize=lambda p: getattr(p, field).set(value))
    assert page.panel.transport.lines == []


@pytest.mark.parametrize("slot,message", [
    (("", "620"), "槽底测距读数还没填"),
    (("460", ""), "槽顶测距读数还没填"),
    (("460", "560"), "槽行程（槽顶 − 槽底）= 100 mm"),
    (("620", "460"), "槽行程（槽顶 − 槽底）= -160 mm"),
    (("460", "660"), "135～185 mm"),
])
def test_slot_readings_are_checked_before_any_command(page, slot, message):
    drive_alt(page, slot=slot)
    assert page.panel.transport.lines == []
    assert message in page.banner_detail_var.get()


def test_vel_and_pos_also_need_the_slot_readings(page):
    drive_alt(page, inject="pos", slot=("", ""))
    assert page.panel.transport.lines == []
    assert "记为槽底" in page.banner_detail_var.get()


def test_alt_needs_the_height_fields_in_the_record(page):
    drive_alt(page, schema=SCHEMA_V3_LINES)
    assert "SYSID START" not in page.panel.transport.lines
    assert "高度字段" in page.banner_detail_var.get()


# ---------------------------------------------------------------- 运行、存档与结果


def test_an_alt_run_is_archived_with_the_new_fields_and_not_fitted(page, monkeypatch, tmp_path):
    no_fit(monkeypatch)
    archive_into(monkeypatch, tmp_path)
    run_alt(page, start_line=START_LINE + " alt_inject=break alt_mass_g=1184 alt_win_mm=70 "
                                          "alt_lift_mm=50 alt_h0_mm=52 alt_control=breakaway "
                                          "alt_target_cn=1320 alt_bottom_mm=460 alt_top_mm=620")
    assert page.samples[0]["height"] == pytest.approx(0.1)
    page.handle_line("SYSID end run=3 state=done reason=complete dropped=0")
    assert wait_for(page, lambda: not page.workflow.saving and page.workflow.saved is not None)
    folder = page.workflow.saved
    assert folder.name.startswith("alt_"), "高度轮目录与杆上姿态轮（rod_）分开"
    with (folder / "samples.csv").open(encoding="utf-8") as handle:
        header = next(csv.reader(handle))
    assert header[-len(NEW_FIELDS):] == list(NEW_FIELDS), "新字段按 SCHEMA 自描述进 samples.csv"
    conditions = json.loads((folder / "conditions.json").read_text(encoding="utf-8"))
    assert conditions["mode"] == "4"
    assert conditions["start"]["alt_mass_g"] == "1184" and conditions["start"]["alt_h0_mm"] == "52"
    assert conditions["alt_request"]["mass_g"] == 1184
    assert conditions["alt_request"]["control"] == "breakaway"
    assert conditions["alt_request"]["target_cn"] == 1320
    assert (conditions["alt_request"]["bottom_mm"], conditions["alt_request"]["top_mm"]) == (460, 620)
    assert conditions["alt"]["alt_control"] == "breakaway"
    assert conditions["alt"]["alt_target_cn"] == "1320"
    assert (conditions["alt"]["alt_bottom_mm"], conditions["alt"]["alt_top_mm"]) == ("460", "620")
    assert conditions["alt_phases"] == [dict(run="3", phase="settle", pulse_us="1450",
                                             thrust_cn="1290")]
    assert conditions["alt_thrust_capped_batches"] == 0
    assert conditions["alt_thrust_capped_flag"] is False
    assert conditions["data_error"] == ""
    assert conditions["alt_request"]["airframe_mass_g"] == pytest.approx(1145.3)
    assert conditions["alt_request"]["extra_mass_g"] == pytest.approx(38.8)
    assert conditions["parameter_echo"]["airframe.mass_kg"] == "1.145300"
    card = page.alt_result_var.get()
    assert "起始高度 52 mm" in card and "质量 1184 g" in card
    assert "离地搜索上限 13.2 N" in card and "槽底 460 mm / 槽顶 620 mm" in card
    # 这组合成行不是一次真的慢升（推力不到 settle 回报值）：阈值写明算不出来，不瞎报数。
    assert "离地/滑落阈值" in card and "对不上" in card and "tools.sysid.breakaway" in card
    assert "离地" not in page.fit_var.get(), "高度轮的结果写在「Z 高度」页，不占内环结果卡"
    assert page.test_alt_view.steps_notebook.select() == str(page.test_alt_view.tabs["results"])
    assert page.gain_var.get() == "" and page.workflow.job is None
    assert not any(line.startswith(("SYSID PARAM", "PARAM SET")) for line in page.panel.transport.lines)
    assert page.banner_var.get() == "完成" and "3 · 结果" in page.banner_detail_var.get()
    # 「重新分析」也不跑拟合，只重算结果区。
    page.run_fit()
    assert page.workflow.job is None and "离地/滑落阈值" in page.alt_result_var.get()
    page.apply_to_ram()
    assert "不给候选参数" in page.status_var.get()


def test_the_altstart_line_is_kept_as_conditions_alt(page, monkeypatch, tmp_path):
    no_fit(monkeypatch)
    archive_into(monkeypatch, tmp_path)
    run_alt(page, inject="vel", extra_lines=[
        "SYSID ALTSTART run=3 alt_inject=vel alt_mass_g=1184 alt_win_mm=70 "
        "alt_lift_mm=50 alt_h0_mm=48 alt_control=closed_loop alt_target_cn=1320 "
        "alt_bottom_mm=460 alt_top_mm=620"])
    page.handle_line("SYSID end run=3 state=done reason=complete dropped=0")
    assert wait_for(page, lambda: not page.workflow.saving and page.workflow.saved is not None)
    conditions = json.loads((page.workflow.saved / "conditions.json").read_text(encoding="utf-8"))
    assert conditions["alt"]["alt_h0_mm"] == "48" and conditions["alt"]["run"] == "3"
    assert conditions["alt"]["alt_control"] == "closed_loop"
    assert conditions["data_error"] == ""
    card = page.alt_result_var.get()
    assert "起始高度 48 mm" in card and "离线分析" in card, "vel/pos 轮不算阈值，仍离线分析"
    assert "离地推力" not in card


def _feed_break_capture(page, samples_every=2):
    """合成的一轮 break（tests/test_sysid_breakaway.simulate：带静/动摩擦的槽 + 契约序列），
    按固件格式回 PHASE 行与批量帧；为了快，每 `samples_every` 个样本取一个（250 → 125 Hz）。"""
    from test_sysid_breakaway import simulate
    times, heights, thrusts, entries, _starts, truth = simulate(seed=2)
    for entry in entries:
        page.handle_line("SYSID PHASE " + " ".join(f"{k}={v}" for k, v in entry.items()))
    rows = [dict(height=int(round(h * 1e4)), height_raw=int(round(h * 1e4)),
                 thrust=int(round(f * 100)), erpm=3000, erpm_lower=2800, vbat=12150)
            for h, f in list(zip(heights, thrusts))[::samples_every]]
    dt_us = int(round((times[1] - times[0]) * 1e6)) * samples_every
    per = 5
    for start in range(0, len(rows), per):
        flags = ((FLAG_FIRST_BATCH if start == 0 else 0)
                 | (FLAG_LAST_BATCH if start + per >= len(rows) else 0))
        page.accept(alt_frame(3, rows[start:start + per], flags=flags,
                              base_us=1000 + start * dt_us, dt_us=dt_us))
    return truth


def test_a_break_run_shows_the_thresholds_on_the_altitude_page(page, monkeypatch, tmp_path):
    no_fit(monkeypatch)
    archive_into(monkeypatch, tmp_path)
    drive_alt(page, target="14")                  # 合成槽的真实离地推力约 13.2 N，上限留余量
    page.handle_line(START_LINE.replace("target_cn=1320", "target_cn=1400"))
    page.handle_line(altstart_line(page))
    truth = _feed_break_capture(page)
    page.handle_line("SYSID end run=3 state=done reason=complete dropped=0")
    card = page.alt_result_var.get()
    assert "离地推力 F_up ≈" in card and "滑落推力 F_down ≈" in card
    assert "悬停对应表值" in card and "静摩擦" in card and "推力表比例 0.9" in card
    assert "电池 12.2 V" in card or "电池 12.1 V" in card
    found = {key: float(re.search(rf"{key} ≈ ([0-9.]+) N", card).group(1))
             for key in ("F_up", "F_down")}
    assert found["F_up"] == pytest.approx(truth["f_up"], abs=0.15)
    assert found["F_down"] == pytest.approx(truth["f_down"], abs=0.15)
    assert "本轮采集完整" in card and page.workflow.data_error == ""
    assert [e["phase"] for e in page.workflow.snapshot["alt_phases"]] == [
        "ramp_up", "climb", "settle", "excite", "descend", "ramp_down"]
    assert page.banner_var.get() == "完成" and "离地/滑落阈值" in page.banner_detail_var.get()
    assert wait_for(page, lambda: not page.workflow.saving and page.workflow.saved is not None)
    # 存档读回来用命令行工具算，和页面上的是同一组数。
    from tools.sysid.breakaway import analyse_run_dir, summary_text
    archived = analyse_run_dir(page.workflow.saved)
    assert summary_text(archived).splitlines()[0] in card


def test_an_altstart_line_from_another_run_is_ignored(page):
    drive_alt(page)
    page.handle_line(START_LINE)
    page.handle_line("SYSID ALTSTART run=2 alt_inject=pos alt_mass_g=1184 alt_win_mm=150 "
                     "alt_lift_mm=100 alt_h0_mm=48")
    assert "alt" not in page.workflow.snapshot


@pytest.mark.parametrize("change,field", [
    ({"alt_control": "closed_loop"}, "alt_control"),
    ({"alt_target_cn": 980}, "alt_target_cn"),
    ({"alt_bottom_mm": 450}, "alt_bottom_mm"),
    ({"alt_top_mm": 0}, "alt_top_mm"),
])
def test_altstart_mismatch_is_saved_as_diagnostic_and_requests_stop(page, change, field):
    drive_alt(page)
    page.handle_line(START_LINE)
    page.handle_line(altstart_line(page, **change))
    assert "SYSID STOP" in page.panel.transport.lines
    assert field in page.workflow.data_error
    assert "仅留作诊断" in page.status_var.get()


def test_done_alt_without_start_provenance_is_not_fit_eligible(page, monkeypatch, tmp_path):
    archive_into(monkeypatch, tmp_path)
    drive_alt(page)
    page.handle_line(START_LINE)
    page.accept(alt_frame(3, alt_rows(5), flags=FLAG_FIRST_BATCH | FLAG_LAST_BATCH, base_us=1000))
    page.handle_line("SYSID end run=3 state=done reason=complete dropped=0")
    assert wait_for(page, lambda: not page.workflow.saving and page.workflow.saved is not None)
    conditions = json.loads((page.workflow.saved / "conditions.json").read_text(encoding="utf-8"))
    assert "ALTSTART" in conditions["data_error"]
    assert "不可用于拟合" in page.alt_result_var.get()


def test_break_without_a_settle_report_from_the_same_run_is_diagnostic(page):
    drive_alt(page)
    page.handle_line(START_LINE)
    page.handle_line(altstart_line(page))
    page.handle_line("SYSID PHASE run=3 phase=ramp_up pulse_us=1100 thrust_cn=0")
    page.handle_line("SYSID PHASE run=2 phase=settle pulse_us=1450 thrust_cn=1290")
    page.accept(alt_frame(3, alt_rows(5), flags=FLAG_FIRST_BATCH | FLAG_LAST_BATCH, base_us=1000))
    page.handle_line("SYSID end run=3 state=done reason=complete dropped=0")
    assert [e["phase"] for e in page.workflow.snapshot["alt_phases"]] == ["ramp_up"], "别的轮次不记"
    assert "未离地" in page.workflow.data_error
    assert "仅供诊断" in page.alt_result_var.get()


def test_phase_reports_are_only_kept_for_alt_runs(page):
    from test_sysid_page import start_run as start_inner_run
    start_inner_run(page)
    page.handle_line("SYSID PHASE run=3 phase=settle pulse_us=1450 thrust_cn=980")
    assert "alt_phases" not in page.workflow.snapshot


def test_done_alt_with_dropped_samples_stays_diagnostic(page):
    run_alt(page, n=5)
    page.handle_line("SYSID end run=3 state=done reason=complete dropped=1")
    assert "丢弃计数 1" in page.workflow.data_error
    assert "不可用于拟合" in page.alt_result_var.get()
    assert page.banner_var.get() == "完成（仅诊断）"


def test_thrust_capped_batch_is_counted_archived_and_never_marked_fit_ready(page, monkeypatch, tmp_path):
    archive_into(monkeypatch, tmp_path)
    drive_alt(page)
    page.handle_line(START_LINE)
    page.handle_line(altstart_line(page))
    page.handle_line(SETTLE_LINE)
    page.accept(alt_frame(3, alt_rows(5),
                          flags=FLAG_FIRST_BATCH | FLAG_LAST_BATCH | 0x0200, base_us=1000))
    page.handle_line("SYSID end run=3 state=done reason=complete dropped=0")
    assert wait_for(page, lambda: not page.workflow.saving and page.workflow.saved is not None)
    conditions = json.loads((page.workflow.saved / "conditions.json").read_text(encoding="utf-8"))
    assert conditions["alt_thrust_capped_batches"] == 1
    assert conditions["alt_thrust_capped_flag"] is True
    assert "THRUST_CAPPED" in conditions["data_error"]
    assert "推力封顶批次 1" in page.alt_result_var.get()
    assert "不可用于拟合" in page.alt_result_var.get()
    assert page.banner_var.get() == "完成（仅诊断）"


def test_alt_frames_must_not_carry_another_modes_flag(page):
    drive_alt(page)
    page.handle_line(START_LINE)
    page.accept(alt_frame(3, alt_rows(5), flags=FLAG_FIRST_BATCH | 0x0080, base_us=1000))
    assert "采样模式与本轮快照不符" in page.status_var.get()


def test_an_aborted_alt_run_explains_the_height_reason(page, monkeypatch, tmp_path):
    archive_into(monkeypatch, tmp_path)
    run_alt(page)
    page.handle_line("SYSID end run=3 state=aborted reason=height_window dropped=0")
    assert page.banner_var.get() == "中止：高度超出允许窗口"
    assert "出窗余量" in page.banner_detail_var.get()


def test_the_height_abort_reasons_are_in_chinese():
    sys.path.insert(0, str(ROOT / "tools"))
    try:
        from panel_lib.pages.sysid.reasons import END_REASONS, explain_end
    finally:
        sys.path.pop(0)
    assert {"height_invalid", "height_window", "no_liftoff", "height_brake", "stop_timeout",
            "stop_failed", "no_slide", "descent_timeout", "landing_unconfirmed"} <= set(END_REASONS)
    what, step = explain_end("height_invalid", "4")
    assert "测距" in what and "遮挡" in step


@pytest.mark.parametrize("reason,what,step", [
    ("no_liftoff", "仍未离地", "提高离地搜索上限"),
    ("stop_timeout", "没能让机体停稳", "速率调慢"),
    ("stop_failed", "仍在上升", "附加质量"),
    ("no_slide", "仍未滑落", "卡滞"),
    ("descent_timeout", "滑回槽底超时", "槽是否卡滞"),
    ("landing_unconfirmed", "不能确认落回槽底", "记为槽底"),
    ("height_window", "超出允许窗口", "槽底/槽顶读数"),
    ("height_brake", "槽上端", "速率调慢"),
])
def test_each_break_abort_reason_gives_a_distinct_action(reason, what, step):
    sys.path.insert(0, str(ROOT / "tools"))
    try:
        from panel_lib.pages.sysid.reasons import explain_end
    finally:
        sys.path.pop(0)
    title, next_step = explain_end(reason, "4")
    assert what in title and step in next_step


@pytest.mark.parametrize("line,meaning", [
    ("ERR sysid alt needs auto throttle (SYSID THROTTLE target_n>0)", "程序油门"),
    ("ERR sysid alt config invalid", "高度设置无效"),
    ("ERR sysid alt amp over limit: inject=break amp<=1 N/s", "速率超过 1 N/s"),
    ("ERR sysid alt amp over limit: inject=vel amp<=0.3 m/s", "vel ≤ 0.3 m/s"),
    ("ERR sysid alt mass unknown (set SYSID ALT mass_g= or airframe mass)", "移动质量"),
    ("ERR sysid alt airframe gravity invalid", "重力加速度"),
    ("ERR sysid alt break target below run weight", "离不了地"),
    ("ERR sysid alt break target over run weight + 5 N", "重力 + 5 N"),
    ("ERR sysid alt slot_endpoints_missing", "记为槽底"),
    ("ERR sysid alt slot_travel", "135～185 mm"),
    ("ERR sysid alt height invalid (TOF)", "8 个新读数"),
    ("ERR sysid alt slot_start", "不在槽底"),
])
def test_break_precheck_error_has_a_specific_chinese_fix(line, meaning):
    sys.path.insert(0, str(ROOT / "tools"))
    try:
        from panel_lib.pages.sysid.reasons import explain_error
    finally:
        sys.path.pop(0)
    message = explain_error(line)
    assert meaning in message and line in message


# ---------------------------------------------------------------- 顶部状态与波形


@pytest.mark.parametrize("phase,title,detail", [
    ("ramp_up", "预升推力…", "九成机重"),
    ("climb", "慢升找离地…", "不经高度 PID"),
    ("settle", "离地了，制停…", "离地粗判"),
    ("excite", "慢降找滑落…", "慢慢减"),
    ("descend", "滑回槽底…", "刹车"),
    ("ramp_down", "降到怠速…", "槽底"),
])
def test_the_banner_follows_the_break_phases(page, phase, title, detail):
    drive_alt(page)
    page.handle_line(START_LINE)
    assert page.banner_var.get() == "预升推力…"
    page.handle_line(f"SYSID PHASE run=3 phase={phase} pulse_us=1450 thrust_cn=1290")
    assert page.banner_var.get() == title and detail in page.banner_detail_var.get()
    assert "离地搜索上限" in page.rc_var.get() or "状态未知" in page.rc_var.get()


def test_closed_loop_alt_still_describes_its_height_pid_climb(page):
    drive_alt(page, inject="vel")
    page.handle_line(START_LINE)
    page.handle_line("SYSID PHASE run=3 phase=climb pulse_us=1450 thrust_cn=1160")
    assert page.banner_var.get() == "升高中…"
    assert "高度环" in page.banner_detail_var.get()


def test_the_ready_banner_names_the_alt_mode(page):
    page.mode_var.set("ALT")
    page.handle_line(thr_line(armed=1, thr_low=1))
    page.refresh_banner()
    assert page.banner_var.get() == "就绪（高度辨识），可以开始"
    assert "限位" in page.banner_detail_var.get() and "槽底/槽顶读数" in page.banner_detail_var.get()
    assert "慢升推力找离地" in page.banner_detail_var.get()
    page.manual_throttle_var.set(True)                      # 内环页的勾选对高度辨识不起作用
    assert page.banner_var.get() == "就绪（高度辨识），可以开始"
    page.alt_inject_var.set("pos")
    page.refresh_banner()
    assert "高度环" in page.banner_detail_var.get()


def _alt_plot_labels(page):
    figure = page.alt_plot.figure
    return [line.get_label() for axis in figure.axes for line in axis.get_lines()]


def test_break_waveform_draws_height_and_thrust_without_an_inapplicable_setpoint(page):
    if getattr(page.alt_plot, "figure", None) is None:
        pytest.skip("matplotlib 不可用")
    run_alt(page, n=40)
    labels = _alt_plot_labels(page)
    assert "高度 height" in labels and "高度设定 height_sp" not in labels
    assert "推力 thrust [N]" in labels
    assert page.alt_plot.axis.get_ylabel() == "高度 [m]"
    # 高度轮只画在「Z 高度」页：内环页的图不出现高度曲线，也不长副轴。
    inner = [line.get_label() for axis in page.figure.axes for line in axis.get_lines()]
    assert "高度 height" not in inner and len(page.figure.axes) == 1
    page.clear_samples()
    page.mode_var.set("FF")
    page._draw(page.excitation())
    assert len(page.figure.axes) == 1 and page.plot_axis.get_ylabel() == "角速度 [rad/s]"


def test_closed_loop_waveform_keeps_the_height_setpoint(page):
    if getattr(page.alt_plot, "figure", None) is None:
        pytest.skip("matplotlib 不可用")
    run_alt(page, n=40, inject="vel")
    labels = _alt_plot_labels(page)
    assert "高度 height" in labels and "高度设定 height_sp" in labels


# ---------------------------------------------------------------- 「Z 高度」页与内环页的分工


def test_alt_lives_on_the_altitude_page_not_in_the_inner_mode_list(page):
    from panel_lib.pages.sysid.inner_loop import INNER_MODES
    view = page.test_alt_view
    assert "ALT" not in INNER_MODES and INNER_MODES == ("FF", "RATE", "ANGLE", "SERVO")
    assert str(page.alt_inject_combo.winfo_toplevel()) == str(view.parent.winfo_toplevel())
    # 下拉框 → 注入类型那一行 → 「高度设置」分区 → 「Z 高度」页的「1 · 准备」
    assert page.alt_inject_combo.master.master.master is view.tabs["prepare"]
    assert page.alt_view is view and view.start_button.cget("text") == "开始高度辨识"


def _thr_with_height(h_mm=463, ok=1):
    return thr_line() + f" alt_h_mm={h_mm} alt_h_ok={ok} alt_sp_mm=0"


def _grid_labels(widget):
    """「高度设置」分区里所有标签的文字（含子框）。"""
    texts = []
    for child in widget.winfo_children():
        try:
            texts.append(str(child.cget("text")))
        except Exception:
            pass
        texts.extend(_grid_labels(child))
    return texts


def test_the_slot_entries_and_record_buttons_are_on_the_altitude_page(page):
    view = page.test_alt_view
    box = page.alt_bottom_button.master.master
    assert box.master is view.tabs["prepare"] and str(box.cget("text")) == "高度设置"
    labels = _grid_labels(box)
    assert "槽底测距读数 [mm]" in labels and "槽顶测距读数 [mm]" in labels
    assert page.alt_bottom_button.cget("text") == "记为槽底"
    assert page.alt_top_button.cget("text") == "记为槽顶"


def test_the_live_height_follows_fresh_thr_reports(page):
    assert "没有新数据" in page.alt_live_height_var.get()
    page.handle_line(_thr_with_height(463))
    assert page.alt_live_height_var.get() == "测距当前读数：463 mm（8 样本平均）"
    page.handle_line(_thr_with_height(470, ok=0))
    assert "无效" in page.alt_live_height_var.get()
    page.handle_line(thr_line())                              # 旧固件：THR 里没有测距
    assert "固件没报" in page.alt_live_height_var.get()
    page.handle_line(_thr_with_height(463))
    page.thr_time -= 10.0                                     # 过期的读数不能冒充当前
    page.refresh_banner()
    assert "没有新数据" in page.alt_live_height_var.get()


def test_record_buttons_copy_only_a_fresh_valid_height(page):
    page.alt_bottom_button.invoke()
    assert page.alt_bottom_var.get() == "" and "没有记为槽底" in page.status_var.get()
    page.handle_line(_thr_with_height(461))
    page.alt_bottom_button.invoke()
    assert page.alt_bottom_var.get() == "461" and "记为槽底" in page.status_var.get()
    page.handle_line(_thr_with_height(619, ok=0))
    page.alt_top_button.invoke()
    assert page.alt_top_var.get() == "" and "没有记为槽顶" in page.status_var.get()
    page.handle_line(_thr_with_height(619))
    page.alt_top_button.invoke()
    assert page.alt_top_var.get() == "619"
    page.handle_line(_thr_with_height(470))
    page.thr_time -= 10.0
    page.alt_bottom_button.invoke()
    assert page.alt_bottom_var.get() == "461", "过期读数不覆盖已记的槽底"


def test_the_window_hint_shows_the_travel_and_the_abort_bounds(page):
    assert "填好槽底、槽顶读数" in page.alt_window_hint_var.get()
    page.alt_bottom_var.set("460")
    page.alt_top_var.set("620")
    hint = page.alt_window_hint_var.get()                     # 默认 break：上沿 = 槽顶 − 10 mm
    assert "槽行程 160 mm" in hint and "高于 610 mm（槽底以上 150 mm）" in hint and "低于 430 mm" in hint
    page.alt_inject_var.set("vel")                            # vel/pos：min(槽底+抬升+余量, 槽顶−40)
    assert "高于 580 mm（槽底以上 120 mm）" in page.alt_window_hint_var.get()
    page.alt_lift_var.set("30")                               # 460+30+70=560 比槽顶−40 低
    assert "高于 560 mm" in page.alt_window_hint_var.get()
    page.alt_top_var.set("560")
    assert "不在 135～185 mm" in page.alt_window_hint_var.get()


def test_the_page_in_front_decides_what_the_next_run_is(page):
    """切到「Z 高度」= 本轮做 ALT；切回内环页还原内环上次选的模式；两边的激励各自保留。"""
    page.mode_var.set("RATE")
    page.amp_var.set("0.07")                                  # 内环手改的幅值
    page.set_alt_view_active(True)
    assert page.mode_var.get() == "ALT" and page.amp_var.get() == "0.5"   # break 默认速率
    page.amp_var.set("0.25")                                  # 高度手改的幅值
    page.set_alt_view_active(False)
    assert page.mode_var.get() == "RATE" and page.amp_var.get() == "0.07"
    page.set_alt_view_active(True)
    assert page.amp_var.get() == "0.25"


def test_the_page_switch_waits_while_a_run_is_open(page):
    drive_alt(page)
    page.handle_line(START_LINE)
    page.set_alt_view_active(False)                           # 跑着切回内环页：不改模式
    assert page.mode_var.get() == "ALT"
    page.handle_line("SYSID end run=3 state=aborted reason=rc_disarm dropped=0")
    assert page.mode_var.get() == "FF", "跑完按前台那一页对齐"


def test_each_start_button_only_starts_its_own_kind(page):
    page.set_alt_view_active(True)
    page.start_inner_run()                                    # 内环页的按钮绝不会开 ALT
    assert page.mode_var.get() == "FF"
    page.panel.transport.lines.clear()
    page.workflow.fail("reset")
    page.start_alt_run()
    assert page.mode_var.get() == "ALT"


def test_inner_and_altitude_throttle_settings_do_not_leak(page):
    page.target_thrust_var.set("11.5")
    page.max_pct_var.set("70")
    page.alt_target_var.set("14")
    page.alt_max_pct_var.set("90")
    page.mode_var.set("ALT")
    assert page.throttle_settings() == (False, 14.0, 90.0)
    page.mode_var.set("FF")
    assert page.throttle_settings() == (False, 11.5, 70.0)


def test_old_settings_saved_during_alt_hand_the_throttle_to_the_altitude_page(page):
    """2026-09-30 前两页共用一份程序油门：上次存的是 ALT，就归给高度页，内环回到默认。"""
    settings_file().write_text(json.dumps({"version": 1, "mode": "ALT", "target_thrust_n": 13.0,
                                           "max_throttle_pct": 95.0}), encoding="utf-8")
    again = fresh_page(page)
    assert (again.alt_target_var.get(), again.alt_max_pct_var.get()) == ("13", "95")
    assert (again.target_thrust_var.get(), again.max_pct_var.get()) == ("", "75")


def test_altitude_throttle_is_remembered_separately(page):
    page.target_thrust_var.set("11.5")
    drive_alt(page, target="14")
    saved = json.loads(settings_file().read_text(encoding="utf-8"))
    assert (saved["alt_target_thrust_n"], saved["alt_max_throttle_pct"]) == (14.0, 90.0)
    assert saved["target_thrust_n"] == 11.5
    again = fresh_page(page)
    assert (again.alt_target_var.get(), again.target_thrust_var.get()) == ("14", "11.5")


@pytest.mark.slow_ui  # 真面板尺寸/缩放矩阵，慢；默认只在界面文件有改动时跑（tests/conftest.py）
@pytest.mark.parametrize("scale", [1.0, 1.25, 1.5])
@pytest.mark.parametrize("size", [(1080, 700), (1366, 768), (1500, 900)])
def test_the_alt_section_fits_the_real_panel(size, scale):
    """「Z 高度」页在三尺寸三缩放下横向不越界；切页即 ALT、换注入类型不抛回调异常。"""
    from tools.panel_qa import OfflinePanel
    with OfflinePanel.launch(size=size, scale=scale, connected=False) as session:
        panel = session.panel
        panel.notebook.select(panel.sysid_tab)
        panel.sysid_notebook.select(2)
        view = panel.sysid_altitude_page
        engine = panel.sysid_page
        view.steps_notebook.select(0)
        panel.update()                     # 切页事件是排队投递的（ttk 的 <<NotebookTabChanged>>）
        assert engine.mode_var.get() == "ALT"
        for widget in (engine.alt_inject_combo, view.profile_combo):
            assert widget.winfo_width() > 40
            assert widget.winfo_rootx() >= panel.winfo_rootx()
            assert widget.winfo_rootx() + widget.winfo_width() <= panel.winfo_rootx() + panel.winfo_width()
        engine.alt_inject_combo.current(1)
        engine.alt_inject_combo.event_generate("<<ComboboxSelected>>")
        assert (engine.amp_var.get(), engine.amp_label_var.get()) == ("0.04", "幅值 [m/s]")
        for button in (view.start_button, view.stop_button):
            assert button.winfo_ismapped()
        panel.sysid_notebook.select(0)
        panel.update()
        assert engine.mode_var.get() != "ALT"
        assert not session.callback_errors


def test_alt_rate_and_angle_gates_explain_the_slot_rig_and_the_soft_landing():
    """槽式台架上杆端卡住又松开会转得快：ALT 的残差/角度中止说明写明门限、已慢降，单轴台架的说法不变。"""
    sys.path.insert(0, str(ROOT / "tools"))
    try:
        from panel_lib.pages.sysid.reasons import explain_end
    finally:
        sys.path.pop(0)
    title, step = explain_end("axis_residual", "4")
    assert "86°/s" in title and "慢降" in title and "卡住" in step
    assert "慢降" in explain_end("angle_limit", "ALT")[0]
    assert explain_end("axis_residual", "1")[0] == "机体没有只绕杆转"

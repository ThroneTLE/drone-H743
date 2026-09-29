"""「内环」页的光杆台架高度辨识模式 ALT（R-ALTID-1）：操作与存档。

报文照本批钉死的固件契约写（固件子任务同批实现）：

* `SYSID MODE ALT` → 模式编号 4；
* `SYSID ALT inject=.. mass_g=.. win_mm=.. lift_mm=..` 回一行同名回显，拒绝回
  `SYSID ALT event=rejected reason=running|range|usage`；
* 记录字段表末尾追加 height/height_raw/height_sp/vz/vz_sp/az/vbat；
* 溯源在 `SYSID start` 行末尾（alt_*），或紧跟的 `SYSID ALTSTART` 行。

字段表照本批固件 `Driver/Src/drv_sysid_record.c` 的 sysid_fields（22 项、记录 v3、每条 44 字节）；
页面按 SCHEMA 自描述解码，不依赖这些缩放。批头 ALT 位 0x0100 照 `drv_sysid_record.h`（不在契约里，
页面不要求它）。
"""
from __future__ import annotations

import csv
import json
import math
import struct
import sys
from pathlib import Path

import pytest

from test_sysid_page import (  # noqa: F401  页面夹具与报文工具
    FLAG_FIRST_BATCH, FLAG_LAST_BATCH, START_PARAMS, feed, fresh_page, page, settings_file,
    status_report, thr_line, wait_for,
)

ROOT = Path(__file__).resolve().parents[1]

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
            f"win_mm={expected['win_mm']} lift_mm={expected['lift_mm']}")


def drive_alt(page, *, params=ALT_PARAMS, schema=SCHEMA_ALT_LINES, echo=alt_echo, inject=None,
              **report):
    """「本轮做」选 ALT → 点开始 → 回字段表、参数；每条配置命令回整份报告，SYSID ALT 回一行。"""
    page.mode_var.set("ALT")
    if inject is not None:
        page.alt_inject_var.set(inject)
        page.select_alt_inject()
    page.rod_to_fc_var.set("0.15")
    page.roll_pivot_var.set("-0.2")
    page.pitch_pivot_var.set("-0.2")
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


START_LINE = ("SYSID start run=3 profile=1 amp_mrad_s=600 dur_ms=4800 rate_hz=250 I=20000 ugm2 "
              "psi_mrad=785 auto=1 target_cn=980")


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


def run_alt(page, n=100, *, start_line=START_LINE, extra_lines=()):
    drive_alt(page)
    assert page.panel.transport.lines[-1] == "SYSID START"
    page.handle_line(start_line)
    feed(page, extra_lines)
    rows = alt_rows(n)
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
    assert settings_store.MODES == ("FF", "RATE", "ANGLE", "SERVO", "ALT")
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
    assert again.alt_inject_var.get() == "force" and again.alt_extra_mass_var.get() == "38.8"
    assert (again.alt_win_var.get(), again.alt_lift_var.get()) == ("70", "50")
    assert again.status_var.get() == ""


def test_alt_settings_are_remembered_with_the_injection_preset(page):
    page.alt_extra_mass_var.set("40")
    page.alt_win_var.set("200")
    page.alt_lift_var.set("80")
    drive_alt(page, inject="vel")
    saved = json.loads(settings_file().read_text(encoding="utf-8"))
    assert saved["mode"] == "ALT" and saved["alt_inject"] == "vel"
    assert (saved["alt_extra_mass_g"], saved["alt_win_mm"], saved["alt_lift_mm"]) == (40, 200, 80)
    again = fresh_page(page)
    assert again.mode_var.get() == "ALT" and again.alt_inject_var.get() == "vel"
    assert (again.alt_extra_mass_var.get(), again.alt_win_var.get(), again.alt_lift_var.get()) == (
        "40", "200", "80")
    # 「实验类型」的角速度预设不能盖掉高度激励。
    assert (again.amp_var.get(), again.hold_var.get(), again.repeat_var.get()) == ("0.04", "1000", "3")


# ---------------------------------------------------------------- 注入类型与默认激励


@pytest.mark.parametrize("index,inject,amp,hold,repeat,dur,unit", [
    (0, "force", "0.4", "300", "8", "4800", "N"),
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
    assert page.amp_var.get() == "0.4"
    assert page.amp_swing_estimate() is None and page.amp_swing_warning() == ""
    assert "不自动改幅值" in page.amp_hint_var.get()


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
    alt = sent.index("SYSID ALT inject=force mass_g=1184 win_mm=70 lift_mm=50")
    assert sent[alt - 1] == "SYSID MODE ALT 3", "SYSID ALT 排在 MODE 之后"
    assert sent[alt + 1].startswith("SYSID RIG ")
    assert sent[alt + 2].startswith("SYSID THROTTLE target_n=9.8 ")
    assert sent[-1] == "SYSID START"
    exc = next(line for line in sent if line.startswith("SYSID EXC "))
    assert "amp=0.4 " in exc and "hold_ms=300 " in exc and "repeat=8 " in exc
    assert page.workflow.expected["mode"] == 4
    assert page.workflow.alt_expected == dict(inject="force", mass_g=1184, win_mm=70, lift_mm=50)


def test_the_mass_falls_back_to_the_ready_report(page):
    params = tuple(p for p in ALT_PARAMS if p[0] != "airframe.mass_kg")
    page.alt_extra_mass_var.set("0")
    drive_alt(page, params=params, mass_mg=1145300)
    assert "SYSID ALT inject=force mass_g=1145 win_mm=70 lift_mm=50" in page.panel.transport.lines
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


def test_alt_needs_program_throttle(page):
    page.manual_throttle_var.set(True)
    drive_alt(page)
    assert page.panel.transport.lines == []
    assert "程序油门" in page.banner_detail_var.get()


@pytest.mark.parametrize("inject,amp", [("force", "3.5"), ("vel", "0.4"), ("pos", "0.2")])
def test_alt_amplitude_limits_follow_the_injection(page, inject, amp):
    page.mode_var.set("ALT")
    page.alt_inject_var.set(inject)
    page.select_alt_inject()
    page.amp_var.set(amp)
    drive_alt(page)
    assert page.panel.transport.lines == []
    assert "上限" in page.banner_detail_var.get()


@pytest.mark.parametrize("field,value", [("alt_win_var", "20"), ("alt_lift_var", "500"),
                                         ("alt_extra_mass_var", "-5"), ("alt_lift_var", "99.5")])
def test_bad_alt_inputs_are_refused_before_sending(page, field, value):
    getattr(page, field).set(value)
    drive_alt(page)
    assert page.panel.transport.lines == []


def test_alt_needs_the_height_fields_in_the_record(page):
    drive_alt(page, schema=SCHEMA_V3_LINES)
    assert "SYSID START" not in page.panel.transport.lines
    assert "高度字段" in page.banner_detail_var.get()


# ---------------------------------------------------------------- 运行、存档与结果


def test_an_alt_run_is_archived_with_the_new_fields_and_not_fitted(page, monkeypatch, tmp_path):
    no_fit(monkeypatch)
    archive_into(monkeypatch, tmp_path)
    run_alt(page, start_line=START_LINE + " alt_inject=force alt_mass_g=1184 alt_win_mm=150 "
                                          "alt_lift_mm=100 alt_h0_mm=52")
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
    assert conditions["alt_request"]["airframe_mass_g"] == pytest.approx(1145.3)
    assert conditions["alt_request"]["extra_mass_g"] == pytest.approx(38.8)
    assert conditions["parameter_echo"]["airframe.mass_kg"] == "1.145300"
    card = page.fit_var.get()
    assert "离线分析" in card and "起始高度 52 mm" in card and "质量 1184 g" in card
    assert page.gain_var.get() == "" and page.workflow.job is None
    assert not any(line.startswith(("SYSID PARAM", "PARAM SET")) for line in page.panel.transport.lines)
    assert page.banner_var.get() == "完成" and "离线" in page.banner_detail_var.get()
    # 「重新分析」也不跑拟合，只说明离线分析。
    page.run_fit()
    assert page.workflow.job is None and "离线分析" in page.fit_var.get()
    page.apply_to_ram()
    assert "不给候选参数" in page.status_var.get()


def test_the_altstart_line_is_kept_as_conditions_alt(page, monkeypatch, tmp_path):
    no_fit(monkeypatch)
    archive_into(monkeypatch, tmp_path)
    run_alt(page, extra_lines=["SYSID ALTSTART run=3 alt_inject=vel alt_mass_g=1184 alt_win_mm=150 "
                               "alt_lift_mm=100 alt_h0_mm=48"])
    page.handle_line("SYSID end run=3 state=done reason=complete dropped=0")
    assert wait_for(page, lambda: not page.workflow.saving and page.workflow.saved is not None)
    conditions = json.loads((page.workflow.saved / "conditions.json").read_text(encoding="utf-8"))
    assert conditions["alt"]["alt_h0_mm"] == "48" and conditions["alt"]["run"] == "3"
    assert "起始高度 48 mm" in page.fit_var.get()


def test_an_altstart_line_from_another_run_is_ignored(page):
    drive_alt(page)
    page.handle_line(START_LINE)
    page.handle_line("SYSID ALTSTART run=2 alt_inject=pos alt_mass_g=1184 alt_win_mm=150 "
                     "alt_lift_mm=100 alt_h0_mm=48")
    assert "alt" not in page.workflow.snapshot


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
    assert {"height_invalid", "height_window"} <= set(END_REASONS)
    what, step = explain_end("height_invalid", "4")
    assert "测距" in what and "遮挡" in step


# ---------------------------------------------------------------- 顶部状态与波形


def test_the_banner_walks_through_the_alt_phases(page):
    drive_alt(page)
    page.handle_line(START_LINE)
    assert page.banner_var.get() == "升推力…"
    page.handle_line("SYSID PHASE run=3 phase=climb pulse_us=1450 thrust_cn=1160")
    assert page.banner_var.get() == "升高中…"
    page.handle_line("SYSID PHASE run=3 phase=preroll pulse_us=1450 thrust_cn=1160")
    assert page.banner_var.get() == "前导中…" and "上锁" not in page.banner_detail_var.get()
    page.handle_line("SYSID PHASE run=3 phase=excite pulse_us=1450 thrust_cn=1160")
    assert page.banner_var.get().startswith("激励中") and "上下移动" in page.banner_detail_var.get()
    page.handle_line("SYSID PHASE run=3 phase=descend pulse_us=1400 thrust_cn=1100")
    assert page.banner_var.get() == "下降中…"
    page.handle_line("SYSID PHASE run=3 phase=hover_check pulse_us=1400 thrust_cn=1100")
    assert page.banner_var.get().startswith("阶段 hover_check")


def test_the_ready_banner_names_the_alt_mode(page):
    page.mode_var.set("ALT")
    page.handle_line(thr_line(armed=1, thr_low=1))
    page.refresh_banner()
    assert page.banner_var.get() == "就绪（高度辨识），可以开始"
    assert "限位" in page.banner_detail_var.get()
    page.manual_throttle_var.set(True)
    assert page.banner_var.get() == "高度辨识要程序油门"


def test_the_waveform_draws_height_and_thrust_then_returns_to_one_axis(page):
    if getattr(page, "figure", None) is None:
        pytest.skip("matplotlib 不可用")
    run_alt(page, n=40)
    labels = [line.get_label() for axis in page.figure.axes for line in axis.get_lines()]
    assert "高度 height" in labels and "高度设定 height_sp" in labels and "推力 thrust [N]" in labels
    assert len(page.figure.axes) == 2
    assert page.plot_axis.get_ylabel() == "高度 [m]"
    page.clear_samples()
    page.mode_var.set("FF")
    page._draw(page.excitation())
    assert len(page.figure.axes) == 1 and page.plot_axis.get_ylabel() == "角速度 [rad/s]"


def test_the_altitude_placeholder_points_to_the_alt_mode():
    source = (ROOT / "tools/panel_lib/pages/sysid/altitude.py").read_text(encoding="utf-8")
    assert "ALT" in source and "槽式台架" in source and "离线" in source


@pytest.mark.parametrize("scale", [1.0, 1.25, 1.5])
@pytest.mark.parametrize("size", [(1080, 700), (1366, 768), (1500, 900)])
def test_the_alt_section_fits_the_real_panel(size, scale):
    """新分区在三尺寸三缩放下横向不越界；在真实面板里选 ALT、换注入类型不抛回调异常。"""
    from tools.panel_qa import OfflinePanel
    with OfflinePanel.launch(size=size, scale=scale, connected=False) as session:
        panel = session.panel
        panel.notebook.select(panel.sysid_tab)
        panel.sysid_notebook.select(0)
        page = panel.sysid_page
        page.steps_notebook.select(0)
        page.mode_var.set("ALT")
        panel.update_idletasks()
        for widget in (page.alt_inject_combo,):
            assert widget.winfo_width() > 40
            assert widget.winfo_rootx() >= panel.winfo_rootx()
            assert widget.winfo_rootx() + widget.winfo_width() <= panel.winfo_rootx() + panel.winfo_width()
        page.alt_inject_combo.current(1)
        page.alt_inject_combo.event_generate("<<ComboboxSelected>>")
        assert (page.amp_var.get(), page.amp_label_var.get()) == ("0.04", "幅值 [m/s]")
        for button in (page.start_button, page.stop_button):
            assert button.winfo_ismapped()
        assert not session.callback_errors

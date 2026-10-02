"""「偏航（吊绳）」页的吊绳偏航辨识模式 YAW：操作、命令、解析、存档与分析。

控件在「偏航（吊绳）」页（yaw_view.py），状态与事务仍在内环页对象里（固件同一时间只跑一轮 SYSID）；
下面的夹具在每个用 `page` 的测试里把偏航页挂到同一个对象上，和真面板一样。报文照跨侧契约
`doc/sysid-yaw-contract.md` 写：

* `SYSID MODE YAW` → 模式编号 6；固件不读 `SYSID THROTTLE` 的 target_n，YAW 轮不发 `SYSID THROTTLE`；
* `SYSID YAW inject=diff|rate thrust_mn=<mN> twist_deg=<90..1440>` 回一行带 control
  （openloop|closed_loop）的回显，拒绝回 `SYSID YAW event=rejected reason=running|range|lift|usage`；
* 开跑后紧跟 `SYSID YAWSTART run=.. yaw_inject=.. yaw_thrust_mn=.. yaw_k_um_per_n=.. yaw_izz_ugm2=..
  yaw_single_max_mn=.. yaw_twist_deg=..`；
* 记录字段表沿用 ALT 追加的 7 个尾字段，YAW 里含义变了（height=ψ、vz=ω、vz_sp=r、az=ΔT）；
* 批头标志 `DRV_SYSID_FLAG_YAW 0x0800`。
"""
from __future__ import annotations

import csv
import json
import re
import struct
import sys
from pathlib import Path

import pytest

from test_sysid_altitude_mode import (  # noqa: F401  字段表与报文工具
    ALT_FIELDS, ALT_HASH, ALT_PARAMS, SCHEMA_ALT_LINES, SCHEMA_V3_LINES, START_LINE, no_fit,
)
from test_sysid_page import (  # noqa: F401  页面夹具与报文工具
    FLAG_FIRST_BATCH, FLAG_LAST_BATCH, FakePanel, feed, fresh_page, page, settings_file,
    status_report, thr_line, wait_for,
)
from test_yaw_analysis import diff_run, rate_run

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from panel_lib.pages.sysid import reasons, yaw_config  # noqa: E402

FLAG_YAW = 0x0800
#: 机体参数：整机 1145.3 g（机重 11.23 N）+ 悬停推力 11.2 N + 整机最大推力 30 N。
YAW_PARAMS = ALT_PARAMS + (("airframe.weight_n", "11.231600"), ("coax.hover_thrust_n", "11.200000"),
                           ("airframe.max_total_force_n", "30.000000"))


@pytest.fixture(autouse=True)
def _mount_yaw_view(request):
    """用到 `page` 的测试都挂上偏航页：偏航的控件、图和结果区在那一页。"""
    if "page" not in request.fixturenames:
        return
    engine = request.getfixturevalue("page")
    from panel_lib.pages.sysid.yaw_view import SysIdYawPage
    from tkinter import ttk
    engine.test_yaw_view = SysIdYawPage(engine.panel, ttk.Frame(engine.parent.winfo_toplevel()), engine)


def yaw_echo(expected: dict) -> str:
    return (f"SYSID YAW inject={expected['inject']} thrust_mn={expected['thrust_mn']} "
            f"twist_deg={expected['twist_deg']} control={expected['control']}")


def drive_yaw(page, *, params=YAW_PARAMS, schema=SCHEMA_ALT_LINES, echo=yaw_echo, inject=None,
              thrust="", twist=None, customize=None, **report):
    """前台切到偏航页 → 点开始 → 回字段表、参数；每条配置命令回整份报告，SYSID YAW 回一行。

    总推力留空 = 0.5×悬停推力参数 coax.hover_thrust_n（11.2 N → 5.6 N）。"""
    page.set_front_view("YAW")
    if inject is not None:
        page.yaw_inject_var.set(inject)
        page.select_yaw_inject()
    page.yaw_thrust_var.set(thrust)
    if twist is not None:
        page.yaw_twist_var.set(twist)
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
        if awaiting.startswith("SYSID YAW "):
            feed(page, [echo(page.workflow.yaw_expected)])
        else:
            feed(page, status_report(page.workflow.expected, **report))


def yawstart_line(page, **changes):
    expected = page.workflow.yaw_expected
    fields = dict(run=3, yaw_inject=expected["inject"], yaw_thrust_mn=expected["thrust_mn"],
                  yaw_k_um_per_n=5000, yaw_izz_ugm2=12500, yaw_single_max_mn=9000,
                  yaw_twist_deg=expected["twist_deg"])
    fields.update(changes)
    return "SYSID YAWSTART " + " ".join(f"{key}={value}" for key, value in fields.items())


#: 固定点缩放（schema 里的 scale）：把物理量换成 i16/u16 原始值。
_SCALES = {"height": 1e-4, "height_raw": 1e-4, "height_sp": 1e-4, "vz": 1e-3, "vz_sp": 1e-3,
           "az": 1e-3, "torque": 1e-4, "thrust": 1e-2, "vbat": 1e-3, "erpm": 4.0, "erpm_lower": 4.0}


def to_raw(rows):
    return [{name: int(round(value / _SCALES[name])) for name, value in row.items() if name in _SCALES}
            for row in rows]


def yaw_frame(run, rows, *, flags, base_us, dt_us=4000):
    """`rows`：每条一个 {字段名: 定点原始值}；offset_us 照实填（u16，所以一帧最多 16 条）。"""
    header = struct.pack("<BBHIIHH", 3, len(rows), run, ALT_HASH, base_us, dt_us, flags | FLAG_YAW)
    fmt = "<" + "".join("h" if kind == "i16" else "H" for *_rest, kind in ALT_FIELDS)

    def value(row, index, name):
        return row.get(name, index * dt_us if name == "offset_us" else 0)
    return header + b"".join(
        struct.pack(fmt, *(value(row, index, name) for name, *_rest in ALT_FIELDS))
        for index, row in enumerate(rows))


def feed_run(page, rows, *, per=16, dt_us=4000, flags=0):
    raw = to_raw(rows)
    for start in range(0, len(raw), per):
        batch_flags = (FLAG_FIRST_BATCH if start == 0 else 0) | (
            FLAG_LAST_BATCH if start + per >= len(raw) else 0) | flags
        page.accept(yaw_frame(3, raw[start:start + per], flags=batch_flags,
                              base_us=1000 + start * dt_us, dt_us=dt_us))


def run_yaw(page, rows=None, *, inject="diff", end_line=None, extra_lines=(), **kwargs):
    drive_yaw(page, inject=inject, **kwargs)
    assert page.panel.transport.lines[-1] == "SYSID START"
    page.handle_line(START_LINE)
    page.handle_line(yawstart_line(page))
    feed(page, extra_lines)
    if rows is None:
        rows = diff_run()[1] if inject == "diff" else rate_run()[1]
    feed_run(page, rows)
    if end_line:
        page.handle_line(end_line)


END_DONE = "SYSID end run=3 state=done reason=complete dropped=0"


# ---------------------------------------------------------------- 模式、页面与命令事务


def test_yaw_is_appended_as_mode_six_and_lives_on_its_own_page():
    from panel_lib.pages.sysid import inner_loop, settings_store
    assert settings_store.MODES.index("YAW") == yaw_config.YAW_MODE_CODE == 6
    assert settings_store.MODES[:6] == ("FF", "RATE", "ANGLE", "SERVO", "ALT", "XY")
    assert not {"ALT", "XY", "YAW"} & set(inner_loop.INNER_MODES)


def test_the_yaw_tab_is_registered_in_the_sysid_group():
    from panel_lib.pages import sysid
    assert sysid.SYSID_YAW_TAB_TEXT == "偏航（吊绳）"
    package = (ROOT / "tools/panel_lib/pages/sysid/__init__.py").read_text(encoding="utf-8")
    assert "mount_sysid_yaw(panel, yaw_scroll.content)" in package
    assert 'else "YAW" if selected == str(yaw_scroll)' in package


def test_the_mode_hint_warns_that_motors_spin_and_the_body_yaws(page):
    page.mode_var.set("YAW")
    hint = page.mode_hint_var.get()
    assert "电机会转" in hint and "绕绳偏航" in hint and "绳子绷紧" in hint


def test_the_yaw_page_is_mounted_with_start_stop_imuzero_and_three_steps(page):
    view = page.test_yaw_view
    assert view.start_button.cget("text") == "开始吊绳偏航辨识"
    assert [view.steps_notebook.tab(tab, "text") for tab in view.steps_notebook.tabs()] == [
        "1 · 准备", "2 · 看波形", "3 · 结果"]
    assert "e.build_imuzero_button(box)" in (
        ROOT / "tools/panel_lib/pages/sysid/yaw_view.py").read_text(encoding="utf-8")
    assert page.yaw_view is view


def test_the_page_shows_weight_lift_limit_default_thrust_and_amplitude_limit(page):
    feed(page, [f"PARAM name={name} value={value}" for name, value in YAW_PARAMS])
    page.set_front_view("YAW")
    page.refresh_yaw_hint()
    thrust = page.yaw_thrust_hint_var.get()
    assert "0.5×悬停推力 = 5.60 N" in thrust and "机重 11.23 N" in thrust and "上限 8.99 N" in thrust
    # 还没读到单桨最大推力：保守值 6 N → min(0.8×5.6, 2×(6−2.8)) = 4.48 N
    limit = page.yaw_limit_var.get()
    assert "4.48 N" in limit and "保守值" in limit
    assert page.amp_label_var.get() == "幅值 [N]" and page.amp_var.get() == "0.5"
    assert "幅值单位 N，上限 4.48 N" in page.yaw_amp_hint()
    assert "720" in page.yaw_twist_hint_var.get() and "90" in page.yaw_twist_hint_var.get()


def test_the_lift_limit_uses_mass_times_g_not_the_weight_parameter_or_hover(page):
    """固件 lift 判据 = 机体质量 × g：airframe.weight_n 参数、悬停推力都不参与。"""
    feed(page, [f"PARAM name={name} value={value}" for name, value in YAW_PARAMS])
    feed(page, ["PARAM name=airframe.weight_n value=9.000000"])
    page.set_front_view("YAW")
    assert "机重 11.23 N" in page.yaw_thrust_hint_var.get() and "上限 8.99 N" in page.yaw_thrust_hint_var.get()
    page.yaw_thrust_var.set("8.9")
    page.start_run()
    assert page.panel.transport.lines == ["SYSID SCHEMA"]


def test_thrmode_reply_sets_the_single_motor_limit_and_the_diff_limit(page):
    feed(page, [f"PARAM name={name} value={value}" for name, value in YAW_PARAMS])
    page.set_front_view("YAW")
    page.handle_line("THRMODE mode=flight state=disarmed zone=low thr_x1000=0 h_mm=0 range=none "
                     "tmax_mn=9000 hover_mn=11200")
    assert page.yaw_t_single_max_n == 9.0
    assert "9.00 N" in page.yaw_limit_var.get() and "4.48 N" in page.yaw_limit_var.get()
    page.handle_line("THRMODE state=rejected reason=armed")          # 没有 tmax_mn：不动
    assert page.yaw_t_single_max_n == 9.0
    page.handle_line("THRMODE mode=flight state=landed tmax_mn=3000 hover_mn=11200")
    assert "0.4 N" in page.yaw_limit_var.get()                       # 2×(3 − 2.8)


def test_the_idle_poll_asks_thrmode_only_on_the_yaw_page(page):
    page.panel.transport.lines.clear()
    page.yaw_poll_idle()
    assert page.panel.transport.lines == []              # 不在偏航页
    page.set_front_view("YAW")
    page.test_yaw_view.parent.winfo_toplevel().deiconify()
    page.test_yaw_view.parent.pack(fill="both", expand=True)
    page.parent.winfo_toplevel().update()
    page.yaw_poll_idle()
    assert page.panel.transport.lines == ["THRMODE?"]
    page.yaw_poll_idle()
    assert page.panel.transport.lines == ["THRMODE?"], "5 s 内不重发"


def test_start_configures_mode_yaw_then_the_yaw_line_then_rig_and_sends_no_throttle(page):
    drive_yaw(page)
    lines = page.panel.transport.lines
    assert lines[-1] == "SYSID START"
    mode = lines.index("SYSID MODE YAW 3")
    yaw = lines.index("SYSID YAW inject=diff thrust_mn=5600 twist_deg=720")
    rig = next(i for i, line in enumerate(lines) if line.startswith("SYSID RIG "))
    assert mode < yaw < rig == len(lines) - 2
    exc = next(line for line in lines[:mode] if line.startswith("SYSID EXC "))
    assert "profile=doublet" in exc and "amp=0.5" in exc and "hold_ms=800" in exc
    assert not any("THROTTLE" in line for line in lines), "固件不读 target_n：YAW 轮不发 SYSID THROTTLE"
    # 杆距没填也不挡：杆到飞控按 0，只剩飞控到质心 0.035
    assert lines[rig] == "SYSID RIG psi_deg=45 axis_off_m=0.035 imu_off_m=0.035"
    assert page.workflow.expected["mode"] == 6
    assert page.workflow.yaw_request["thrust_mn"] == 5600


def test_a_typed_thrust_and_twist_are_sent_as_typed(page):
    drive_yaw(page, thrust="6.75", twist="360")
    assert "SYSID YAW inject=diff thrust_mn=6750 twist_deg=360" in page.panel.transport.lines
    assert page.workflow.yaw_expected["control"] == "openloop"


@pytest.mark.parametrize("inject,control,amp,label", [("diff", "openloop", "0.5", "幅值 [N]"),
                                                       ("rate", "closed_loop", "0.8", "幅值 [rad/s]")])
def test_control_mode_and_default_excitation_follow_the_inject_type(page, inject, control, amp, label):
    drive_yaw(page, inject=inject)
    assert page.workflow.yaw_expected["control"] == control
    assert page.amp_var.get() == amp and page.amp_label_var.get() == label
    assert f"SYSID YAW inject={inject} thrust_mn=5600 twist_deg=720" in page.panel.transport.lines
    assert page.panel.transport.lines[-1] == "SYSID START"


def test_every_echo_item_is_checked_before_start(page):
    def wrong_thrust(expected):
        return yaw_echo(dict(expected, thrust_mn=expected["thrust_mn"] + 100))
    drive_yaw(page, echo=wrong_thrust)
    assert "SYSID START" not in page.panel.transport.lines
    assert "配置回读不符：偏航 thrust_mn" in page.status_var.get()

    page2 = fresh_page(page)
    page2.panel.transport.is_connected = True
    from panel_lib.pages.sysid.yaw_view import SysIdYawPage
    from tkinter import ttk
    page2.test_yaw_view = SysIdYawPage(page2.panel, ttk.Frame(page2.parent.winfo_toplevel()), page2)
    drive_yaw(page2, echo=lambda e: yaw_echo(dict(e, control="closed_loop")))
    assert "SYSID START" not in page2.panel.transport.lines
    assert "control" in page2.status_var.get()


@pytest.mark.parametrize("reason,text", [("running", "辨识正在进行"), ("range", "超出范围"),
                                         ("lift", "提起来"), ("usage", "命令格式不对")])
def test_firmware_rejections_are_translated(page, reason, text):
    drive_yaw(page, echo=lambda _e: f"SYSID YAW event=rejected reason={reason}")
    assert "SYSID START" not in page.panel.transport.lines
    assert text in page.status_var.get() and f"reason={reason}" in page.status_var.get()


def test_thrust_above_eighty_percent_of_weight_is_refused_before_anything_is_sent(page):
    feed(page, [f"PARAM name={name} value={value}" for name, value in YAW_PARAMS])
    page.set_front_view("YAW")
    page.yaw_thrust_var.set("9.5")              # 0.8×11.23 = 8.99
    page.start_run()
    assert page.panel.transport.lines == []
    assert "0.8×机重" in page.status_var.get() and "8.99" in page.status_var.get()
    page.yaw_thrust_var.set("1.5")
    page.start_run()
    assert page.panel.transport.lines == [] and "至少 2" in page.status_var.get()
    page.yaw_thrust_var.set("abc")
    page.start_run()
    assert page.panel.transport.lines == [] and "总推力要填数字" in page.status_var.get()


def test_unknown_weight_or_hover_blocks_start_with_a_clear_reason(page):
    """机体参数没回来：开跑前拦不了，轮到 SYSID YAW 那一步（参数已收齐）再拦，START 绝不会发。"""
    drive_yaw(page, params=(), mass_mg=0)       # 没有悬停推力参数、也没有机重
    assert "SYSID START" not in page.panel.transport.lines
    assert not any(line.startswith("SYSID YAW ") for line in page.panel.transport.lines)
    assert "悬停推力" in page.status_var.get()
    again = fresh_page(page)
    again.panel.transport.is_connected = True
    from panel_lib.pages.sysid.yaw_view import SysIdYawPage
    from tkinter import ttk
    again.test_yaw_view = SysIdYawPage(again.panel, ttk.Frame(again.parent.winfo_toplevel()), again)
    drive_yaw(again, params=(), thrust="5", mass_mg=0)
    assert "SYSID START" not in again.panel.transport.lines and "机重" in again.status_var.get()


def test_twist_limit_range_is_checked(page):
    feed(page, [f"PARAM name={name} value={value}" for name, value in YAW_PARAMS])
    page.set_front_view("YAW")
    for bad in ("50", "2000", "100.5", "x"):
        page.yaw_twist_var.set(bad)
        page.start_run()
        assert page.panel.transport.lines == [], bad
        assert "绞绳上限" in page.status_var.get()


def test_amplitude_is_checked_against_the_differential_limit_before_sending(page):
    feed(page, [f"PARAM name={name} value={value}" for name, value in YAW_PARAMS])
    page.set_front_view("YAW")
    page.amp_var.set("5")                      # 保守上限 4.48 N
    page.start_run()
    assert page.panel.transport.lines == []
    assert "上限 4.48 N" in page.status_var.get() and "保守值" in page.status_var.get()
    # 读到单桨最大推力 3 N：上限只剩 0.4 N，1.5 被拦；9 N 时 1.5 通过
    page.handle_line("THRMODE mode=flight state=disarmed tmax_mn=3000 hover_mn=11200")
    page.amp_var.set("1.5")
    page.start_run()
    assert page.panel.transport.lines == [] and "上限 0.4 N" in page.status_var.get()
    page.handle_line("THRMODE mode=flight state=disarmed tmax_mn=9000 hover_mn=11200")
    page.start_run()
    assert page.panel.transport.lines == ["SYSID SCHEMA"]


def test_rate_amplitude_limit_is_two_rad_per_s(page):
    feed(page, [f"PARAM name={name} value={value}" for name, value in YAW_PARAMS])
    page.set_front_view("YAW")
    page.yaw_inject_var.set("rate")
    page.select_yaw_inject()
    page.amp_var.set("2.5")
    page.start_run()
    assert page.panel.transport.lines == [] and "上限 2 rad/s" in page.status_var.get()
    page.amp_var.set("2")
    page.start_run()
    assert page.panel.transport.lines == ["SYSID SCHEMA"]


def test_an_old_schema_without_the_tail_fields_refuses_to_start(page):
    drive_yaw(page, schema=SCHEMA_V3_LINES)
    assert "SYSID START" not in page.panel.transport.lines
    assert "吊绳偏航辨识（YAW）需要记录里带" in page.workflow.notice


def test_manual_throttle_checkbox_does_not_apply_to_yaw(page):
    page.manual_throttle_var.set(True)
    drive_yaw(page)
    assert page.panel.transport.lines[-1] == "SYSID START"


def test_a_stray_yaw_query_reply_is_not_taken_for_the_echo(page):
    """别处（AI 接口等）发的 `SYSID YAW` 查询，回复排在页面命令前面：先到的那行是旧配置，不能当本次回显。"""
    page.send("SYSID YAW")                      # 页面外的查询：note_sent 记一笔
    stale = "SYSID YAW inject=rate thrust_mn=3000 twist_deg=90 control=closed_loop"
    page.set_front_view("YAW")
    page.start_run()
    feed(page, SCHEMA_ALT_LINES)
    feed(page, [f"PARAM name={name} value={value}" for name, value in YAW_PARAMS])
    for _ in range(14):
        awaiting = page.workflow.awaiting
        if not awaiting or awaiting == "SYSID START":
            break
        if awaiting.startswith("SYSID YAW "):
            feed(page, [stale, yaw_echo(page.workflow.yaw_expected)])
        else:
            feed(page, status_report(page.workflow.expected))
    assert page.panel.transport.lines[-1] == "SYSID START"
    assert "配置回读不符" not in page.status_var.get()


def test_a_reply_while_not_awaiting_is_ignored(page):
    page.handle_line("SYSID YAW inject=diff thrust_mn=1 twist_deg=1 control=openloop")
    assert page.workflow.awaiting is None and page.workflow.error == ""


def test_the_yaw_echo_does_not_get_swallowed_as_a_status_report(page):
    """整份状态报告以 SYSID LIMITS 结尾，不能把它当 SYSID YAW 的回显（回显是它自己的一行）。"""
    drive_yaw(page)
    awaiting_during = [line for line in page.panel.transport.lines if line.startswith("SYSID YAW ")]
    assert len(awaiting_during) == 1


# ---------------------------------------------------------------- 开跑溯源与存档


def test_the_yawstart_line_is_kept_as_provenance_and_checked(page):
    drive_yaw(page)
    page.handle_line(START_LINE)
    page.handle_line(yawstart_line(page))
    snap = page.workflow.snapshot
    assert snap["yaw"]["yaw_inject"] == "diff" and snap["yaw"]["yaw_single_max_mn"] == "9000"
    assert snap["yaw"]["yaw_k_um_per_n"] == "5000" and snap["yaw"]["yaw_izz_ugm2"] == "12500"
    assert not page.workflow.data_error
    assert snap["yaw_request"]["control"] == "openloop"
    assert snap["yaw_request"]["thrust_mn"] == 5600 and snap["yaw_request"]["twist_deg"] == 720


@pytest.mark.parametrize("field,wrong", [("yaw_inject", "rate"), ("yaw_thrust_mn", 5000),
                                         ("yaw_twist_deg", 360)])
def test_a_provenance_that_disagrees_with_the_request_stops_the_run(page, field, wrong):
    drive_yaw(page)
    page.handle_line(START_LINE)
    page.panel.transport.lines.clear()
    page.handle_line(yawstart_line(page, **{field: wrong}))
    assert "飞控开跑溯源不符" in page.workflow.data_error and field in page.workflow.data_error
    assert page.panel.transport.lines == ["SYSID STOP"]


def test_provenance_off_by_one_millinewton_is_accepted(page):
    drive_yaw(page)
    page.handle_line(START_LINE)
    page.handle_line(yawstart_line(page, yaw_thrust_mn=5601))
    assert not page.workflow.data_error


def test_a_finished_diff_run_is_archived_and_analysed(page, monkeypatch, tmp_path):
    no_fit(monkeypatch)
    from panel_lib.pages.sysid import workflow
    root = tmp_path / "yaw" / "2026-10-01"
    monkeypatch.setattr(workflow, "_yaw_directory", lambda: root)
    run_yaw(page, end_line=END_DONE)
    wait_for(page, lambda: page.workflow.saved is not None and not page.workflow.saving)
    folder = page.workflow.saved
    assert folder.parent == root and re.fullmatch(r"yaw_\d{6}_[0-9a-f]{8}", folder.name)
    with (folder / "samples.csv").open(newline="", encoding="utf-8") as handle:
        table = list(csv.DictReader(handle))
    assert len(table) == len(diff_run()[1])
    assert {"t_us", "run_id", "gap", "height", "vz", "vz_sp", "az", "torque", "thrust",
            "erpm", "erpm_lower"} <= set(table[0])
    conditions = json.loads((folder / "conditions.json").read_text(encoding="utf-8"))
    assert conditions["mode"] == "6"
    assert conditions["yaw"]["yaw_inject"] == "diff" and conditions["yaw"]["yaw_single_max_mn"] == "9000"
    request = conditions["yaw_request"]
    assert (request["inject"], request["thrust_mn"], request["twist_deg"], request["control"]) == (
        "diff", 5600, 720, "openloop")
    assert request["thrust_n"] == 5.6 and request["t_single_max_source"] == "保守值"
    assert conditions["end"]["state"] == "done" and not conditions["data_error"]
    text = page.yaw_result_var.get()
    assert "本轮偏航设置（飞控开跑时报告）" in text and "注入 diff" in text
    assert "b = " in text and "rate_yaw_kp" in text and "k 实测 / k 模型" in text
    assert "最大可用偏航力矩" in text and "最大角加速度" in text
    assert page.banner_var.get() == "完成"


def test_the_diff_analysis_on_the_page_recovers_b_from_quantised_records(page, monkeypatch, tmp_path):
    """定点量化（力矩 0.1 mN·m、角速度 1 mrad/s）后经解码、存档再分析，b 仍在 10% 内。"""
    no_fit(monkeypatch)
    from panel_lib.pages.sysid import workflow
    from sysid import yaw_analysis
    monkeypatch.setattr(workflow, "_yaw_directory", lambda: tmp_path)
    run_yaw(page, end_line=END_DONE)
    wait_for(page, lambda: page.workflow.saved is not None and not page.workflow.saving)
    result = yaw_analysis.analyse_run_dir(page.workflow.saved)
    assert result["b"] == pytest.approx(60.0, rel=0.10)
    assert result["k_ratio"] == pytest.approx(result["b"] * 0.0125)


def test_rate_result_notes_that_there_is_no_reference_feedforward(page, monkeypatch, tmp_path):
    no_fit(monkeypatch)
    from panel_lib.pages.sysid import workflow
    monkeypatch.setattr(workflow, "_yaw_directory", lambda: tmp_path)
    run_yaw(page, inject="rate", end_line=END_DONE)
    assert "rate_yaw_ff" in page.yaw_result_var.get()


def test_a_finished_rate_run_reports_step_metrics(page, monkeypatch, tmp_path):
    no_fit(monkeypatch)
    from panel_lib.pages.sysid import workflow
    monkeypatch.setattr(workflow, "_yaw_directory", lambda: tmp_path)
    run_yaw(page, inject="rate", end_line=END_DONE)
    text = page.yaw_result_var.get()
    assert "偏航角速度闭环阶跃" in text and "90% 上升时间" in text and "超调" in text
    assert "饱和时间占比" in text


def test_an_aborted_run_is_not_analysed_and_explains_the_reason(page, monkeypatch, tmp_path):
    no_fit(monkeypatch)
    from panel_lib.pages.sysid import workflow
    monkeypatch.setattr(workflow, "_yaw_directory", lambda: tmp_path)
    run_yaw(page, end_line="SYSID end run=3 state=aborted reason=yaw_twist dropped=0")
    text = page.yaw_result_var.get()
    assert "这一轮中止了" in text and "偏航角超过绞绳上限" in text and "绞绳上限" in text
    assert "b = " not in text
    assert "yaw_twist" in page.status_var.get()
    assert page.workflow.data_error and "固件中止本轮" in page.workflow.data_error


def test_a_batch_carrying_another_modes_flag_is_rejected(page):
    drive_yaw(page)
    page.handle_line(START_LINE)
    page.handle_line(yawstart_line(page))
    raw = to_raw(diff_run()[1][:5])
    page.accept(yaw_frame(3, raw, flags=FLAG_FIRST_BATCH | 0x40, base_us=1000))
    assert "采样模式与本轮快照不符" in page.status_var.get()


def test_the_live_readout_follows_the_last_sample(page):
    drive_yaw(page)
    assert "没有数据" in page.yaw_live_var.get()
    page.handle_line(START_LINE)
    page.handle_line(yawstart_line(page))
    rows = diff_run()[1][:16]
    for row in rows:
        row.update(vz=0.5, height=3.2767)          # height 已饱和：偏航角只能来自陀螺积分
    raw = to_raw(rows)
    page.accept(yaw_frame(3, raw, flags=FLAG_FIRST_BATCH, base_us=1000))
    page.refresh_yaw_live()
    # 16 条、间隔 4 ms：ψ = 0.5 rad/s × 15 × 0.004 s = 0.03 rad = 1.7°
    assert "+0.500 rad/s" in page.yaw_live_var.get() and "+1.7°" in page.yaw_live_var.get()
    assert "上限 720°" in page.yaw_live_var.get()


def test_the_plot_labels_never_mention_the_alt_column_names(page):
    """图用改名后的 YAW 字段，画图不抛，图例里没有 height。"""
    drive_yaw(page)
    page.handle_line(START_LINE)
    page.handle_line(yawstart_line(page))
    feed_run(page, diff_run()[1][:160])
    plot = page.yaw_plot
    if plot.figure is None:
        pytest.skip("matplotlib 不可用")
    page._draw_measured()
    labels = []
    for axis in (plot.psi_axis, plot.omega_axis, plot.delta_axis, plot.moment_axis, plot.erpm_axis):
        labels += [line.get_label() for line in axis.get_lines()]
    assert labels and not any("height" in label for label in labels)
    assert any("ψ" in label for label in labels) and any("下桨" in label for label in labels)


# ---------------------------------------------------------------- 拒绝/中止理由的中文解释


@pytest.mark.parametrize("token,needle", [("yaw_overspeed", "8 rad/s"), ("yaw_twist", "绞绳上限")])
def test_abort_reasons_are_explained(token, needle):
    what, step = reasons.explain_end(token, "YAW")
    assert needle in what + step and "已软停" in what
    assert reasons.explain_end(token, "6") == (what, step)
    assert "未知" not in what


def test_thrust_gates_in_yaw_point_at_the_yaw_page():
    what, step = reasons.explain_end("actuator_saturated", "YAW")
    assert "差速" in what and "偏航（吊绳）" in step
    what, step = reasons.explain_end("thrust_low", "6")
    assert "偏航（吊绳）" in step
    # 别的模式的说法不变
    assert "偏航（吊绳）" not in reasons.explain_end("thrust_low", "FF")[1]


@pytest.mark.parametrize("line,needle", [
    ("ERR sysid yaw amp over limit (inject=diff max=2.00)", "差速推力"),
    ("ERR sysid yaw amp over limit (inject=rate max=2.00)", "2 rad/s"),
    ("ERR sysid yaw amp over limit (inject=diff max=2.960)", "差速推力"),
    ("ERR sysid yaw prop map uncalibrated (yaw polarity)", "桨位/旋向还没标定"),
    ("ERR sysid yaw thrust over lift limit (thrust_mn>0.8*weight)", "0.8×机重"),
    ("ERR sysid yaw thrust too low (thrust_mn<2000)", "小于 2 N"),
    ("ERR sysid yaw thrust over max", "最大总推力"),
    ("ERR sysid yaw airframe invalid", "机体参数无效"),
    ("ERR sysid yaw something new", "开始前自检没过"),
    ("ERR unknown sysid subcmd YAW", "更新固件"),
    ("ERR usage SYSID YAW", "格式不对"),
])
def test_start_rejections_are_explained_with_the_original_line(line, needle):
    text = reasons.explain_error(line)
    assert needle in text and line in text and "飞控拒绝了这条命令" not in text


def test_a_firmware_err_line_during_the_transaction_fails_it_with_the_chinese_text(page):
    drive_yaw(page, echo=lambda _e: "ERR sysid yaw amp over limit (inject=diff max=2.00)")
    assert "SYSID START" not in page.panel.transport.lines
    assert "差速推力" in page.status_var.get()


def test_the_banner_speaks_yaw_while_running(page):
    drive_yaw(page)
    page.handle_line(START_LINE)
    page.handle_line(yawstart_line(page))
    page.phase = "excite"
    page.refresh_banner()
    assert "绕绳偏航" in page.banner_detail_var.get()
    page.phase = "ramp_up"
    page.refresh_banner()
    assert "小于机重" in page.banner_detail_var.get()


def test_excitation_is_kept_apart_per_page_when_switching(page):
    """偏航页的幅值（N）与内环页的（rad/s）各存一份，切回去还原。"""
    inner_amp = page.amp_var.get()
    page.set_front_view("YAW")
    assert page.amp_var.get() == "0.5"
    page.set_front_view(None)
    assert page.amp_var.get() == inner_amp and page.mode_var.get() != "YAW"
    page.set_front_view("YAW")
    assert page.amp_var.get() == "0.5" and page.mode_var.get() == "YAW"


def test_new_code_lives_in_new_modules_and_not_in_the_banned_files():
    panel = (ROOT / "tools/drone_tcp_panel.py").read_text(encoding="utf-8")
    assert "yaw_view" not in panel and "SYSID YAW" not in panel
    for name in ("yaw_config", "yaw_section", "yaw_view", "yaw_plot", "yaw_result"):
        assert (ROOT / f"tools/panel_lib/pages/sysid/{name}.py").is_file()
    assert (ROOT / "tools/sysid/yaw_analysis.py").is_file()


@pytest.mark.slow_ui  # 真面板：切到偏航页即 YAW，控件在窗口内，换注入类型不抛回调异常
def test_the_yaw_page_fits_the_real_panel_and_switching_tabs_switches_the_mode():
    from tools.panel_qa import OfflinePanel
    with OfflinePanel.launch(size=(1080, 700), scale=1.25, connected=False) as session:
        panel = session.panel
        panel.notebook.select(panel.sysid_tab)
        yaw_index = 3
        panel.sysid_notebook.select(yaw_index)
        view = panel.sysid_yaw_page
        engine = panel.sysid_page
        view.steps_notebook.select(0)
        panel.update()
        assert panel.sysid_notebook.tab(yaw_index, "text") == "偏航（吊绳）"
        assert engine.mode_var.get() == "YAW"
        for widget in (engine.yaw_inject_combo, view.profile_combo):
            assert widget.winfo_width() > 40
            assert widget.winfo_rootx() + widget.winfo_width() <= panel.winfo_rootx() + panel.winfo_width()
        engine.yaw_inject_combo.current(1)
        engine.yaw_inject_combo.event_generate("<<ComboboxSelected>>")
        assert (engine.amp_var.get(), engine.amp_label_var.get()) == ("0.8", "幅值 [rad/s]")
        for button in (view.start_button, view.stop_button):
            assert button.winfo_ismapped()
        panel.sysid_notebook.select(0)
        panel.update()
        assert engine.mode_var.get() != "YAW"
        assert not session.callback_errors

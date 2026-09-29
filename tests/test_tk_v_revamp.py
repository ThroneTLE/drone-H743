"""V 线 TK-01/TK-02 contracts against the real offline DronePanel."""

from __future__ import annotations

from pathlib import Path
import json
import subprocess
import sys

import pytest
import tkinter as tk
from tkinter import ttk

from tools import panel_qa
from tools.panel_qa.geometry import SCALES, WINDOW_SIZES


ROOT = Path(__file__).resolve().parents[1]


def _descendants(widget):
    for child in widget.winfo_children():
        yield child
        yield from _descendants(child)


def _qa_session(tmp_path, *, scale: float = 1.0, size: tuple[int, int] = (1366, 768)):
    environment = panel_qa.isolated_environment(tmp_path)
    environment.__enter__()
    try:
        session = panel_qa.OfflinePanel.launch(scale=scale, size=size)
    except BaseException as exc:
        environment.__exit__(type(exc), exc, exc.__traceback__)
        if panel_qa.is_display_unavailable(exc):
            pytest.skip(f"Tk display unavailable: {exc}")
        raise
    return session, environment


@pytest.fixture(scope="module")
def qa(tmp_path_factory):
    session, environment = _qa_session(tmp_path_factory.mktemp("tk-v"))
    try:
        yield session
    finally:
        session.destroy()
        environment.__exit__(None, None, None)


def test_theme_is_global_semantic_and_contrasted(qa):
    """All input states use the shared palette; pages do not own a theme patch."""
    assert not (ROOT / "tools" / "panel_lib" / "servo_debug_theme.py").exists()

    style = ttk.Style(qa.panel)
    for style_name, option in (
        ("TEntry", "fieldbackground"),
        ("TSpinbox", "fieldbackground"),
        ("TCombobox", "fieldbackground"),
    ):
        for states in ((), ("focus",), ("disabled",), ("readonly",), ("invalid",)):
            background = style.lookup(style_name, option, state=states)
            foreground = style.lookup(style_name, "foreground", state=states)
            assert background, (style_name, option, states)
            assert foreground, (style_name, states)

    def rgb(value: str) -> tuple[int, int, int]:
        channels = qa.panel.winfo_rgb(value)
        return tuple(channel // 257 for channel in channels)

    def luminance(value: str) -> float:
        def linear(channel: int) -> float:
            channel /= 255.0
            return channel / 12.92 if channel <= 0.03928 else ((channel + 0.055) / 1.055) ** 2.4

        red, green, blue = rgb(value)
        return 0.2126 * linear(red) + 0.7152 * linear(green) + 0.0722 * linear(blue)

    background = luminance(qa.panel.ui_palette["panel"])
    foreground = luminance(qa.panel.ui_palette["ink"])
    assert (max(background, foreground) + 0.05) / (min(background, foreground) + 0.05) >= 4.5

    spinboxes = [widget for widget in _descendants(qa.panel) if isinstance(widget, ttk.Spinbox)]
    assert spinboxes
    assert all(widget.cget("style") == "Numeric.TSpinbox" for widget in spinboxes)


def test_matplotlib_pages_share_the_dark_chart_theme(qa):
    checked = 0
    # 2026-09-13：辨识页的图从 `panel.ident_figure` 搬到了
    # `panel.sysid_page.figure`（整页迁出 drone_tcp_panel.py 重写）。
    figures = {
        "baro_figure": getattr(qa.panel, "baro_figure", None),
        "gps_figure": getattr(qa.panel, "gps_figure", None),
        "sysid_figure": getattr(getattr(qa.panel, "sysid_page", None), "figure", None),
    }
    for figure_name, figure in figures.items():
        if figure is None:
            continue
        expected = qa.panel.winfo_rgb(qa.panel.ui_palette["panel"])
        assert tuple(round(channel, 6) for channel in figure.get_facecolor()[:3]) == tuple(
            round(channel / 65535.0, 6) for channel in expected
        )
        assert figure.axes
        assert figure.axes[0].get_facecolor() == figure.get_facecolor()
        checked += 1
    # 三张图全是 None 时这个循环一条断言都不跑，会静默变绿——本批一直在打的
    # 就是这个形态，别在自己的新测试里再造一遍。
    assert checked == 3, f"只核到 {checked} 张图，应当三张都在"


def test_viewports_route_wheel_locally_and_support_keyboard(qa):
    dashboard = qa.panel.dashboard_host
    dashboard.set_content_height(2400)
    qa.panel.update_idletasks()
    assert dashboard.canvas.bind("<MouseWheel>")
    assert qa.panel.bind_all("<MouseWheel>") in ("", None)

    before = dashboard.canvas.yview()
    dashboard.canvas.event_generate("<MouseWheel>", delta=-120)
    qa.pump()
    assert dashboard.canvas.yview() != before

    dashboard.canvas.focus_set()
    start = dashboard.canvas.yview()
    dashboard.canvas.event_generate("<KeyPress-Next>")
    qa.pump()
    assert dashboard.canvas.yview() != start
    dashboard.canvas.event_generate("<KeyPress-Home>")
    qa.pump()
    assert dashboard.canvas.yview()[0] == pytest.approx(0.0)


@pytest.mark.parametrize("scale", SCALES)
def test_every_leaf_page_has_reachable_interactive_controls(tmp_path, scale):
    # Windows Tcl can retain a just-destroyed interpreter's library handle for
    # a short interval.  Run each simulated DPI matrix in its own process so
    # the three required scales are independent and deterministic.
    child = """
import json
import sys
import tempfile
from pathlib import Path
from tools import panel_qa

scale = float(sys.argv[1])
with tempfile.TemporaryDirectory() as root:
    with panel_qa.hardware_guards() as guard:
        with panel_qa.isolated_environment(Path(root)):
            try:
                session = panel_qa.OfflinePanel.launch(scale=scale, size=(1366, 768))
            except BaseException as exc:
                if panel_qa.is_display_unavailable(exc):
                    print("DISPLAY_UNAVAILABLE")
                    raise SystemExit(0)
                raise
            try:
                reports = session.probe_geometry(sizes=panel_qa.WINDOW_SIZES)
                bad = [report.to_json() for report in reports if not report.clean]
                print(json.dumps({"bad": bad, "reports": len(reports),
                                  "serial": guard.serial_opens,
                                  "flash": guard.flash_invocations}))
            finally:
                session.destroy()
        if not guard.clean:
            raise AssertionError((guard.serial_opens, guard.flash_invocations))
"""
    result = subprocess.run(
        [sys.executable, "-c", child, str(scale)],
        cwd=ROOT, capture_output=True, text=True, timeout=120,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    if "DISPLAY_UNAVAILABLE" in result.stdout:
        pytest.skip("Tk display unavailable")
    payload = result.stdout.strip().splitlines()[-1]
    report = json.loads(payload)
    # 电源页、状态灯及其余叶页，每页覆盖三个尺寸。
    # 2026-09-20：电流计与电池电压并成「电源」页（29 → 28 叶页，R-PWR-1）。
    # 2026-09-20：新增「校准 / 磁力计校准」叶页（28 → 29 叶页，R-MAG-1）：
    # 29 页 × 3 个窗口尺寸 = 87。
    assert report["reports"] == 87
    assert report["bad"] == [], report["bad"]


def test_stop_and_key_actions_are_present_without_deleting_controls(qa):
    servo_tab = next(
        tab for tab in qa.panel.maintenance_notebook.tabs()
        if qa.panel.maintenance_notebook.tab(tab, "text") == "舵机调试"
    )
    servo_page = qa.panel.nametowidget(servo_tab)
    fixed = getattr(servo_page, "fixed")
    fixed_stops = [
        widget for widget in _descendants(fixed)
        if isinstance(widget, ttk.Button) and widget.cget("text") == "停止"
    ]
    assert len(fixed_stops) == 2
    assert all("canvas" not in str(widget.master).lower() for widget in fixed_stops)

    qa.transport.lines.clear()
    qa.transport.frames.clear()
    for button in fixed_stops:
        button.invoke()
    sent = list(qa.transport.lines) + [
        payload.decode("utf-8") for _function, payload in qa.transport.frames
    ]
    assert sent == ["SERVO CMD 0 DST", "SERVO CMD 1 DST"]

    labels = [
        str(widget.cget("text"))
        for widget in _descendants(qa.panel)
        if isinstance(widget, ttk.Button)
    ]
    assert labels.count("停止") >= 3  # connection stop + one fixed servo stop per slot
    # 2026-09-13：这一组按钮全在旧辨识页上，整页已删除重写。
    # 新页的按钮名换成中文并落在 panel_lib/pages/sysid/ 里。
    # 2026-09-27：一键开始后分析与建议参数自动完成，保留手动「重新分析」与写 RAM/撤销。
    assert {"开始辨识", "停止（电机回到遥控器油门）", "重新分析",
            "临时应用到飞控（RAM，不存 Flash）", "恢复原参数",
            "下发台架几何", "读取采集格式（诊断）"} <= set(labels)
    assert "重置累计位移" in labels

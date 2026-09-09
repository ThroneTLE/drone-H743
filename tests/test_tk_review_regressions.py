"""Regressions for the V/D-line review findings (R-S1-4 … R-S1-7).

Every test here exists because the reviewer found a criterion that the delivered
suite either violated or never asserted.  Grouped by finding so a failure names
the criterion it belongs to.
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest
import tkinter as tk
from tkinter import ttk

from tools import panel_qa
from tools.panel_lib import parameter_model
from tools.panel_lib.viewport import KEYBOARD_OWNER_CLASSES, VerticalScrolledFrame


ROOT = Path(__file__).resolve().parents[1]
DRIVER_PARAMS = ROOT / "Driver" / "Src" / "drv_coax_ctrl.c"


@pytest.fixture(scope="module")
def offline(tmp_path_factory):
    root = tmp_path_factory.mktemp("tk-review")
    with panel_qa.isolated_environment(root):
        try:
            session = panel_qa.OfflinePanel.launch(scale=1.0, size=(1366, 768))
        except BaseException as exc:
            if panel_qa.is_display_unavailable(exc):
                pytest.skip(f"Tk display unavailable: {exc}")
            raise
        try:
            yield session
        finally:
            session.destroy()


@pytest.fixture
def app(offline):
    offline.transport.lines.clear()
    offline.transport.frames.clear()
    offline.transport.is_connected = True
    return offline.panel


# --------------------------------------------------------------------------
# R-S1-5 · 视口不得抢走键盘焦点
#
# 审核复现（舵机调试页）：焦点给输入控件 → 鼠标移到视口空白处 →
# `canvas.focus_set()` 抢走焦点 → 之后敲的键全落进 canvas，输入框内容不变。
# 滚轮走的是 bindtag，本来就不需要焦点；只有 PageUp/Home 需要。
# --------------------------------------------------------------------------


def _viewport_of(panel) -> VerticalScrolledFrame:
    host = getattr(panel, "dashboard_host", None)
    assert isinstance(host, VerticalScrolledFrame), "面板应当持有真实视口，不能是替身"
    return host


def test_hovering_a_viewport_does_not_steal_focus_from_an_entry(app, offline):
    viewport = _viewport_of(app)
    entry = ttk.Entry(viewport.content)
    entry.pack()
    entry.focus_force()
    offline.pump()
    assert str(app.tk.call("focus")) == str(entry)

    viewport.content.event_generate("<Enter>", x=5, y=5)
    viewport.canvas.event_generate("<Enter>", x=5, y=5)
    offline.pump()

    # 焦点没被抢走，所以后续按键仍然落在输入框里。
    assert str(app.tk.call("focus")) == str(entry)
    entry.insert(0, "bus")
    entry.event_generate("<KeyPress>", keysym="1")
    offline.pump()
    assert entry.get().startswith("bus")
    entry.destroy()


@pytest.mark.parametrize(
    "widget_factory",
    [ttk.Entry, ttk.Spinbox, ttk.Combobox, tk.Entry, tk.Text, tk.Listbox],
)
def test_every_keyboard_owner_class_is_protected(app, offline, widget_factory):
    """夹具直接核对 winfo_class()，避免类名表和真实控件对不上。"""
    viewport = _viewport_of(app)
    widget = widget_factory(viewport.content)
    widget.pack()
    widget.focus_force()
    offline.pump()
    assert widget.winfo_class() in KEYBOARD_OWNER_CLASSES

    viewport.canvas.event_generate("<Enter>", x=5, y=5)
    offline.pump()
    assert str(app.tk.call("focus")) == str(widget)
    widget.destroy()


def test_hovering_still_grants_the_keyboard_when_nobody_is_typing(app, offline):
    """PageUp/Home 的悬停可用性不许因为上面的修复而丢掉。"""
    viewport = _viewport_of(app)
    button = ttk.Button(viewport.content, text="qa")
    button.pack()
    button.focus_force()
    offline.pump()
    assert button.winfo_class() not in KEYBOARD_OWNER_CLASSES

    viewport.canvas.event_generate("<Enter>", x=5, y=5)
    offline.pump()
    assert str(app.tk.call("focus")) == str(viewport.canvas)
    button.destroy()


def test_clicking_the_viewport_always_claims_the_keyboard(app, offline):
    """点击是显式动作：用户点空白处就是要把焦点交给视口。"""
    viewport = _viewport_of(app)
    entry = ttk.Entry(viewport.content)
    entry.pack()
    entry.focus_force()
    offline.pump()

    viewport.canvas.event_generate("<Button-1>", x=5, y=5)
    offline.pump()
    assert str(app.tk.call("focus")) == str(viewport.canvas)
    entry.destroy()


# --------------------------------------------------------------------------
# R-S1-4 · 五态不能只靠颜色区分，且查值要等于调色板取值
# --------------------------------------------------------------------------


def test_input_state_lookups_equal_the_palette(app):
    """原用例只断言 lookup 非空——空串以外的任何脏值都能蒙混过关。"""
    style = ttk.Style(app)
    palette = app.ui_palette
    expected = {
        (): palette["panel"],
        ("focus",): palette["panel"],
        ("readonly",): palette["panel"],
        ("disabled",): palette["disabled"],
    }
    for style_name in ("TEntry", "TSpinbox", "TCombobox", "Numeric.TSpinbox"):
        for states, colour in expected.items():
            actual = style.lookup(style_name, "fieldbackground", list(states))
            assert actual, f"{style_name}{states} 的 fieldbackground 为空"
            assert app.winfo_rgb(actual) == app.winfo_rgb(colour), (
                f"{style_name}{states} 取到 {actual!r}，与调色板 {colour!r} 不符"
            )


def test_the_five_semantic_states_are_distinguishable_without_colour(app):
    """禁用 / 未支持 / 过期(草稿) / 待确认 / 失败：各自有文字，且互不相同。"""
    from tools.panel_lib.parameter_model import ParameterState, validate_parameter_text

    def status(**kwargs) -> str:
        state = ParameterState(name=kwargs.pop("name", "coax.att_roll_kp"))
        for key, value in kwargs.items():
            setattr(state, key, value)
        return app._parameter_status_text(state)

    texts = {
        # 只读 = 固件不可写；用真实的只读能力项，不是伪造的
        "未支持": validate_parameter_text("coax.removed_parameter", "1")[1],
        "失败": status(error="超出范围"),
        "待确认": status(pending="1.5"),
        "过期": status(draft="1.5", draft_source="local"),
        "已回读": status(target="1.0", target_source="PARAM"),
        "未读取": status(),
    }
    for label, text in texts.items():
        assert text.strip(), f"{label} 态没有任何文字，只能靠颜色区分"
    assert len(set(texts.values())) == len(texts), f"存在文字相同的状态：{texts}"
    # "未支持" 必须自己说清楚不可写，不能只是灰一点。
    assert "不在当前固件能力表" in texts["未支持"]


# --------------------------------------------------------------------------
# R-S1-7 · 图表断言不得在对象为 None 时静默跳过
# --------------------------------------------------------------------------


def test_the_chart_theme_assertion_covers_at_least_one_real_figure(app):
    checked = 0
    for figure_name in ("baro_figure", "gps_figure", "ident_figure"):
        figure = getattr(app, figure_name, None)
        if figure is None:
            continue
        expected = app.winfo_rgb(app.ui_palette["panel"])
        assert tuple(round(c, 6) for c in figure.get_facecolor()[:3]) == tuple(
            round(c / 65535.0, 6) for c in expected
        )
        checked += 1
    assert checked, "三张图全是 None —— 这不是通过，是没测到"


# --------------------------------------------------------------------------
# R-S1-6 · 字段级错误 + 焦点定位 + 零发送；范围边界对齐固件
# --------------------------------------------------------------------------


def _select_page_owning(offline, widget) -> None:
    """把持有 `widget` 的叶页切到前台。

    Tk 的 `focus` 对未 map 的控件是 no-op（实测 `focus -lastfor` 仍停在 `.`），
    所以"焦点定位"这条判据只有在页面真的显示出来时才成立——真实用户点"暂存"
    时那一页当然是显示着的。
    """
    target = str(widget)
    for page in offline.leaf_pages():
        # 带上分隔点：`.!verticalscrolledframe` 是 `.!verticalscrolledframe4`
        # 的字符串前缀，纯 startswith 会选错页。
        if target.startswith(f"{page.top}."):
            offline.select(page)
            assert widget.winfo_viewable(), f"{page.label} 选中后 {target} 仍不可见"
            return
    raise AssertionError(f"找不到持有 {target} 的叶页")


def test_invalid_input_marks_the_field_positions_focus_and_sends_nothing(app, offline):
    _select_page_owning(offline, app.param_value_entry)
    app.param_name_var.set("coax.att_roll_kp")
    app.param_value_var.set("abc")
    assert app._stage_param_edit() is False

    assert app.param_value_error_var.get(), "错误没有落到字段级提示上"
    entry = getattr(app, "param_value_entry", None)
    assert entry is not None
    offline.pump()
    # 焦点定位是判据里和错误文案并列的一条，原用例只断了文案。
    assert str(app.tk.call("focus")) == str(entry)
    assert offline.transport.lines == []
    assert offline.transport.frames == []


def _firmware_bounds() -> dict[str, object]:
    """从固件源码里抽出校验常量，不手抄。"""
    source = DRIVER_PARAMS.read_text(encoding="utf-8", errors="replace")
    validator = re.search(
        r"coax_ctrl_param_value_valid\(const[^)]*\)\s*\{(.*?)\n\}", source, re.S
    )
    assert validator, "固件里找不到 coax_ctrl_param_value_valid()"
    body = validator.group(1)

    abs_limit = re.search(r"fabsf\(value\)\s*>\s*([0-9.]+)f", body)
    assert abs_limit, "取不到 |value| 的上界"

    tilt_macro = re.search(
        r"#define\s+DRV_COAX_CTRL_TILT_LIMIT_RAD\s+([0-9.]+)f", source
    )
    assert tilt_macro, "取不到 DRV_COAX_CTRL_TILT_LIMIT_RAD"

    positive_only = set(re.findall(r"rate_limit_rad_s\[(\d)\]", body))
    return {
        "abs_limit": float(abs_limit.group(1)),
        "tilt_limit": float(tilt_macro.group(1)),
        "rate_limit_axes": positive_only,
        "tilt_is_positive_only": "value > 0.0f" in body,
    }


def test_host_bounds_track_the_firmware_validator():
    bounds = _firmware_bounds()
    assert parameter_model.COAX_PARAM_ABS_LIMIT == bounds["abs_limit"]
    assert parameter_model.COAX_TILT_LIMIT_RAD == pytest.approx(bounds["tilt_limit"])
    assert bounds["rate_limit_axes"] == {"0", "1", "2"}
    assert bounds["tilt_is_positive_only"]


@pytest.mark.parametrize(
    "name, value, ok",
    [
        # |value| > 2000 → 固件 ERR，主机必须先拦。
        ("coax.att_roll_kp", "2000", True),
        ("coax.att_roll_kp", "2000.1", False),
        ("coax.att_roll_kp", "1e9", False),
        ("coax.rate_roll_kp", "1e9", False),
        # 派生的 PID SET 项走 try_set_params，同样受 |value| ≤ 2000 约束。
        ("coax.roll_angle_kp", "1e9", False),
        # value > 0，0 不是合法值。
        ("coax.roll_rate_limit_rad_s", "0", False),
        ("coax.pitch_rate_limit_rad_s", "0", False),
        ("coax.yaw_rate_limit_rad_s", "0", False),
        ("coax.roll_rate_limit_rad_s", "0.001", True),
        # tilt_limit_rad ∈ (0, DRV_COAX_CTRL_TILT_LIMIT_RAD]
        ("coax.tilt_limit_rad", "0", False),
        ("coax.tilt_limit_rad", "0.4886922", True),
        ("coax.tilt_limit_rad", "0.6", False),
        # 其余项仍是 value >= 0，0 合法——不许顺手收紧固件允许的值。
        ("coax.att_roll_kp", "0", True),
        ("coax.vel_loop_enable", "1", True),
        ("coax.vel_loop_enable", "2", False),
    ],
)
def test_host_rejects_exactly_what_the_firmware_rejects(name, value, ok):
    valid, reason = parameter_model.validate_parameter_text(name, value)
    assert valid is ok, f"{name}={value} 期望 {ok}，实得 {valid}（{reason}）"
    if not ok:
        assert reason, "拒绝必须带原因，不能空着"

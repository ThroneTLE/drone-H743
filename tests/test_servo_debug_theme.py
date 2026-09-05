"""Servo dark-theme contrast and PWM gates."""

import tkinter as tk
from tkinter import ttk

import pytest

from tools import drone_tcp_panel as panel


def _page(app):
    tab = next(tab for tab in app.notebook.tabs() if app.notebook.tab(tab, "text") == "维护 · 舵机调试")
    return app.nametowidget(tab)


def descendants(parent):
    for widget in parent.winfo_children():
        yield widget
        yield from descendants(widget)


@pytest.fixture(scope="module")
def app():
    try:
        instance = panel.DronePanel()
    except tk.TclError as exc:
        pytest.skip(str(exc))
    instance.withdraw()
    yield instance
    instance.destroy()


def test_dark_spinboxes(app):
    style = ttk.Style(app)
    boxes = [widget for widget in descendants(_page(app)) if isinstance(widget, ttk.Spinbox)]
    assert len(boxes) == 12  # Both slots: ID, pulse, duration, mode, new ID, baud.
    for box in boxes:
        name = str(box.cget("style")) or "TSpinbox"
        assert name == "Numeric.TSpinbox"
        for state in ((), ("focus",), ("readonly",), ("disabled",)):
            background = style.lookup(name, "fieldbackground", state=state)
            foreground = style.lookup(name, "foreground", state=state)
            assert background in (app.ui_palette["panel"], app.ui_palette["disabled"])
            assert foreground != background
            assert foreground in (
                app.ui_palette["ink"], app.ui_palette["ink_dim"], app.ui_palette["muted"],
                app.ui_palette["disabled_ink"],
            )


def test_subtle_borders(app):
    style = ttk.Style(app)
    widgets = [widget for widget in descendants(_page(app))
               if isinstance(widget, (ttk.Notebook, ttk.Scale))]
    assert len(widgets) == 3
    colors = {app.ui_palette[key] for key in ("border", "border_strong", "raised", "panel", "accent")}
    for widget in widgets:
        name = str(widget.cget("style"))
        if not name:
            name = "TNotebook" if isinstance(widget, ttk.Notebook) else "Numeric.Horizontal.TScale"
        for option in ("bordercolor", "lightcolor", "darkcolor"):
            assert style.lookup(name, option) in colors


def test_pwm_gates(app):
    original = [app._servo_values(index) for index in range(2)]
    app.servo_type_active_var.set("pwm")
    app._refresh_servo_output_controls()
    assert all(widget.instate(["disabled"]) for widget in app._servo_bus_widgets)
    style = ttk.Style(app)
    checks = [widget for widget in descendants(_page(app)) if isinstance(widget, ttk.Checkbutton)]
    assert checks
    for check in checks:
        name = str(check.cget("style")) or "TCheckbutton"
        assert style.lookup(name, "background", state=("disabled",)) == app.ui_palette["surface"]
    app.servo_type_active_var.set("bus")
    app._refresh_servo_output_controls()
    assert all(not widget.instate(["disabled"]) for widget in app._servo_bus_widgets)
    assert [app._servo_values(index) for index in range(2)] == original

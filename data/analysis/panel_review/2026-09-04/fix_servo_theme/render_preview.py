"""Render a temporary, disconnected Tk panel for visual QA; never open serial."""

import sys
from pathlib import Path
import serial
from PIL import ImageGrab

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT))
from tools import drone_tcp_panel as panel

opens = []


def blocked_open(port, *args, **kwargs):
    opens.append(str(getattr(port, "port", None)))
    raise AssertionError("Preview cannot open physical serial")


serial.Serial.open = blocked_open
for method in ("_restore_last_connection", "_save_panel_state",
               "_validation_load_latest_artifact", "_v1_load_latest_session"):
    setattr(panel.DronePanel, method, lambda self: None)
panel.DronePanel._load_panel_state = lambda self: {}

app = panel.DronePanel()
try:
    for ident in app.tk.call("after", "info"):
        app.tk.call("after", "cancel", ident)
    app.geometry("1320x940+30+20")
    app.attributes("-topmost", True)
    tab = next(tab for tab in app.notebook.tabs()
               if app.notebook.tab(tab, "text") == "维护 · 舵机调试")
    app.notebook.select(tab)
    page = app.nametowidget(tab)
    app.update()
    for mode in ("bus", "pwm"):
        app.servo_type_var.set(mode)
        app.servo_type_active_var.set(mode)
        app._refresh_servo_output_controls()
        app.update()
        x, y = page.winfo_rootx(), page.winfo_rooty()
        ImageGrab.grab(bbox=(x, y, x + page.winfo_width(), y + page.winfo_height())).save(
            Path(__file__).parent / f"{mode}.png")
finally:
    app.destroy()
print(f"Physical serial open attempts: {len(opens)}")

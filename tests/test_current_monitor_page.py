"""Current readback UI contracts."""
from pathlib import Path
import shutil
import subprocess
import time
import os
from types import SimpleNamespace

import pytest
from tools import drone_tcp_panel as panel
from tools.panel_lib.connection_state import ReceivedMessage, receive_context
from tools.panel_lib import rx_dispatch
from tools.panel_lib.pages import current_monitor as current
from test_dashboard_page import FakeTransport

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="module")
def firmware_line(tmp_path_factory):
    d = tmp_path_factory.mktemp("current-report")
    (d / "app_control.h").write_text("void APP_Control_QueueText(const char*,...);\n")
    (d / "report.c").write_text(r'''
#include "app_current.h"
#include "bsp_current.h"
#include <stdio.h>
#include <stdarg.h>
uint32_t BSP_Critical_Enter(void){return 0;}
void BSP_Critical_Exit(uint32_t x){(void)x;}
uint32_t SVC_Timestamp_Ms(void){return 100;}
BSP_CurrentStatus BSP_Current_Init(void){return BSP_CURRENT_OK;}
BSP_CurrentStatus BSP_Current_Read(uint32_t *raw){*raw=12000;return BSP_CURRENT_OK;}
void APP_Control_QueueText(const char *fmt,...){va_list a;va_start(a,fmt);vprintf(fmt,a);va_end(a);}
int main(void){APP_Current_Init();APP_Current_Step();APP_Current_Report();return 0;}
''')
    cmd = [shutil.which("gcc"), "-std=c11", "-Wall", "-Wextra", "-Werror", "-O2"]
    for inc in (d, ROOT / "App/Inc", ROOT / "BSP/Inc", ROOT / "Driver/Inc", ROOT / "Services/Inc"):
        cmd += ["-I", str(inc)]
    cmd += [str(d / "report.c"), str(ROOT / "App/Src/app_current.c"),
            str(ROOT / "Driver/Src/drv_current.c"), "-lm", "-o", str(d / "report.exe")]
    subprocess.run(cmd, check=True, capture_output=True)
    return subprocess.check_output([str(d / "report.exe")], text=True).strip()


@pytest.fixture(scope="module")
def app():
    p = panel.DronePanel()
    p.current_page.auto_var.set(False)
    yield p
    p.destroy()


def deliver(app, line, *, age=0, generation=None, framed=False):
    context = receive_context(app.transport, received_at=time.monotonic() - age, generation=generation)
    payload = ("proto", panel.PROTO_MSG_TEXT_LINE, line) if framed else line
    app.rx_queue.put(ReceivedMessage(payload, context))
    rx_dispatch.drain_rx(app, 1000, 5, 50)


@pytest.fixture
def connected(app):
    app.transport = FakeTransport()
    app.transport.connection_generation = 1
    app.current_page.refresh()
    return app


def test_real_firmware_readback_and_both_queue_formats(connected, firmware_line):
    p = connected.current_page
    for framed in (False, True):
        deliver(connected, firmware_line, framed=framed)
        assert p.current_var.get().endswith(" A") and p.current_var.get() != "— A"
        assert p.fields["raw"].get() == "12000"
        assert p.fields["calibrated"].get() == "未校准 · 标称换算"
        assert p.fields["source"].get() == "AM32_55A_CURR"
        assert p.fields["nominal_mv_per_a"].get() == "12.75 mV/A"
    if output := os.environ.get("CURRENT_REVIEW_IMAGES"):
        from PIL import ImageGrab
        from tools.panel_lib import shell
        output = Path(output)
        output.mkdir(parents=True, exist_ok=True)

        def capture(window, name):
            window.geometry("1200x900+40+40")
            window.title("离线界面核对 · 非实机电流")
            window.notebook.select(window.sensor_group_tab)
            window.deiconify()
            window.lift()
            window.update()
            # Capture our own window, independent of another app taking focus.
            ImageGrab.grab(window=int(window.frame(), 0)).save(output / name)

        old = subprocess.check_output(["git", "show", "ebf973f0:tools/panel_lib/shell.py"], cwd=ROOT).decode("utf-8")
        namespace = {"__package__": "tools.panel_lib"}
        exec(compile(old, "baseline-shell.py", "exec"), namespace)
        with pytest.MonkeyPatch.context() as patch:
            patch.setattr(shell, "build_ui", namespace["build_ui"])
            # Legacy pages create master-less variables: bind them to this
            # baseline interpreter, then restore the current panel's default.
            patch.setattr(panel.tk, "_default_root", None)
            baseline = panel.DronePanel()
            try:
                capture(baseline, "current-before.png")
            finally:
                baseline.destroy()
        connected.sensor_notebook.select(connected.current_tab)
        deliver(connected, firmware_line)
        capture(connected, "current-after.png")


def test_old_session_delayed_queue_other_reports_disconnect(connected, firmware_line):
    p = connected.current_page
    deliver(connected, firmware_line, age=4)
    assert p.current_var.get() == "— A" and "过期" in p.state_var.get()
    deliver(connected, "PONG ok=1")
    p.refresh()
    assert "过期" in p.state_var.get()
    deliver(connected, firmware_line)
    assert p.current_var.get() != "— A"
    connected.transport.connection_generation += 1
    p.refresh()
    assert p.current_var.get() == "— A"
    deliver(connected, firmware_line, generation=1)
    assert p.current_var.get() == "— A"
    deliver(connected, firmware_line)
    connected.transport.is_connected = False
    p.refresh()
    assert p.current_var.get() == "— A" and p.fields["raw"].get() == "—"


@pytest.mark.parametrize("old,new", [(" raw=12000", ""), ("valid=1", "valid=2"),
                                    ("adc_status=0", "adc_status=2"), ("saturated=0", "saturated=1")])
def test_malformed_reply_clears_previous_reading(connected, firmware_line, old, new):
    deliver(connected, firmware_line)
    deliver(connected, firmware_line.replace(old, new))
    assert connected.current_page.current_var.get() == "— A"
    assert "无效" in connected.current_page.state_var.get()


def test_invalid_measurement_is_not_zero(connected, firmware_line):
    parsed = current.parse_current(firmware_line)
    line = firmware_line.replace("valid=1", "valid=0").replace(f"current_a={parsed['current_a']:.3f}", "current_a=nan")
    deliver(connected, line)
    assert connected.current_page.current_var.get() == "— A"
    assert connected.current_page.fields["raw"].get() == "12000"


def test_poll_visibility_rate_timeout_and_refusal(connected, monkeypatch):
    p = connected.current_page
    clock = [100.0]
    monkeypatch.setattr(current, "time", SimpleNamespace(monotonic=lambda: clock[0]))
    p.auto_var.set(True)
    connected.notebook.select(connected.sensor_group_tab)
    connected.sensor_notebook.select(connected.current_tab)
    p._tick()
    assert connected.transport.frames[-1] == (panel.PROTO_REQ_STATUS, b"STATUS?")
    n = len(connected.transport.frames)
    p.request_read()
    assert len(connected.transport.frames) == n
    clock[0] += 4
    p.refresh()
    assert "未收到" in p.state_var.get()
    connected.sensor_notebook.select(connected.baro_tab)
    p._tick()
    assert len(connected.transport.frames) == n
    monkeypatch.setattr(connected.transport, "send_frame", lambda *a: False)
    assert not p.request_read()
    assert "未发送" in p.state_var.get()
    p.auto_var.set(False)


@pytest.mark.parametrize("size", ["1024x768", "1280x800", "1600x1000"])
@pytest.mark.parametrize("scaling", [1.0, 1.5, 2.0])
def test_page_controls_fit_scrollable_sensor_tab(connected, size, scaling):
    old = connected.tk.call("tk", "scaling")
    try:
        connected.tk.call("tk", "scaling", scaling)
        connected.geometry(size)
        connected.notebook.select(connected.sensor_group_tab)
        connected.sensor_notebook.select(connected.current_tab)
        connected.update_idletasks()
        p = connected.current_page
        assert p.read_button.winfo_ismapped()
        assert all(label.winfo_x() + label.winfo_reqwidth() <= p.winfo_width() for label in p.value_labels)
        assert connected.current_tab.canvas.cget("scrollregion")
    finally:
        connected.tk.call("tk", "scaling", old)

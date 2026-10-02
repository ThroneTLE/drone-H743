"""USB 意外断开自动切蓝牙：假面板 + 假传输 + 手动时钟，不开串口、不建 Tk 窗口。"""
from types import SimpleNamespace

from tools.panel_lib import link_failover as lf
from tools.panel_lib.serial_session import DisconnectInfo


class Var:
    def __init__(self, value=""):
        self.value = value

    def get(self):
        return self.value

    def set(self, value):
        self.value = value


class FakeSerial:
    def __init__(self):
        self.connected = False
        self.active_port = None
        self.last_disconnect = None

    @property
    def is_connected(self):
        return self.connected


class FakePanel:
    """扫描/打开行为由 bt_script 控制：每次 request 弹出一个结果。"""

    def __init__(self, bt_script=()):
        self.serial_transport = FakeSerial()
        self.transport = self.serial_transport
        self.transport_var = Var("serial")
        self.autoconnect_var = Var("")
        self.usb_failover_var = Var(True)
        self._panel_state = {}
        self._serial_port_identity = {}
        self._serial_port_map = {}
        self._bt_scanning = False
        self._bt_opening = False
        self.bt_script = list(bt_script)
        self.requests = []
        self.log = []
        self.cancelled = 0

    def after(self, ms, fn=None):
        return "id"

    def after_cancel(self, ident):
        pass

    def _append(self, text):
        self.log.append(text)

    def _transport_connected(self):
        return self.serial_transport.connected

    def _refresh_serial_ports(self):
        return list(self._serial_port_map)

    def _cancel_bluetooth(self):
        self.cancelled += 1

    def _request_bluetooth_scan(self, connect=False, preferred=None):
        self.requests.append((connect, preferred))
        outcome = self.bt_script.pop(0) if self.bt_script else "notfound"
        if outcome == "ok":
            self.serial_transport.connected = True
            self.serial_transport.active_port = "COM9"
        elif outcome == "openfail":
            pass
        # notfound：扫描结束，未开始打开
        self._bt_scanning = False
        self._bt_opening = False


def drop(panel, phase="read", port="COM5", now=100.0, gen=1):
    panel.serial_transport.connected = False
    panel.serial_transport.last_disconnect = DisconnectInfo(
        port, gen, phase, "x", None, None, now)


def make(panel):
    clock = SimpleNamespace(t=100.0)
    fo = lf.LinkFailover(panel, clock=lambda: clock.t)
    return fo, clock


def run(fo, clock, seconds, step=0.5):
    end = clock.t + seconds
    while clock.t < end:
        clock.t += step
        fo._step()


def test_unexpected_usb_drop_switches_to_remembered_bluetooth():
    panel = FakePanel(["ok"])
    panel._panel_state = {"bluetooth_address": "AABBCCDDEEFF"}
    fo, clock = make(panel)
    drop(panel)
    run(fo, clock, 5)
    assert panel.requests == [(True, "AABBCCDDEEFF")]
    assert panel.transport_var.get() == "蓝牙"
    assert "已自动切到蓝牙" in panel.autoconnect_var.get()
    assert fo._phase == "idle"


def test_retries_then_falls_back_to_name_search_and_succeeds():
    panel = FakePanel(["notfound", "ok"])
    panel._panel_state = {"bluetooth_address": "AABBCCDDEEFF"}
    fo, clock = make(panel)
    drop(panel)
    run(fo, clock, 15)
    assert panel.requests == [(True, "AABBCCDDEEFF"), (True, "")]
    assert "已自动切到蓝牙" in panel.autoconnect_var.get()


def test_gives_up_after_max_attempts_without_dialog_and_restores_serial_mode():
    panel = FakePanel()
    fo, clock = make(panel)
    drop(panel)
    run(fo, clock, 40)
    assert len(panel.requests) == lf.MAX_ATTEMPTS
    assert "自动切蓝牙失败" in panel.autoconnect_var.get()
    assert panel.transport_var.get() == "serial"
    assert fo._phase == "idle"


def test_user_stop_and_cancel_phases_never_trigger():
    for phase in ("stop", "open cancelled", "cancel"):
        panel = FakePanel(["ok"])
        fo, clock = make(panel)
        drop(panel, phase=phase)
        run(fo, clock, 6)
        assert panel.requests == [], phase


def test_switch_off_does_not_trigger():
    panel = FakePanel(["ok"])
    panel.usb_failover_var.set(False)
    fo, clock = make(panel)
    drop(panel)
    run(fo, clock, 6)
    assert panel.requests == []


def test_dfu_hold_window_suppresses_and_cancels_pending():
    panel = FakePanel(["ok"])
    fo, clock = make(panel)
    fo.hold(90)
    drop(panel, now=clock.t)
    run(fo, clock, 6)
    assert panel.requests == []
    # 已进入宽限期时再 hold 也会取消
    panel2 = FakePanel(["ok"])
    fo2, clock2 = make(panel2)
    drop(panel2)
    run(fo2, clock2, 1)
    assert fo2._phase == "grace"
    fo2.hold(90)
    run(fo2, clock2, 6)
    assert panel2.requests == [] and fo2._phase == "idle"


def test_module_level_hold_reaches_panel_instance():
    panel = FakePanel()
    panel.link_failover, clock = make(panel)
    lf.hold(panel)
    drop(panel)
    run(panel.link_failover, clock, 6)
    assert panel.requests == []


def test_firmware_flow_suppresses():
    for flag in ("firmware_update_pending", "firmware_update_running", "firmware_programming"):
        panel = FakePanel(["ok"])
        setattr(panel, flag, True)
        fo, clock = make(panel)
        drop(panel)
        run(fo, clock, 6)
        assert panel.requests == [], flag


def test_bluetooth_port_drop_is_not_usb():
    panel = FakePanel(["ok"])
    panel._serial_port_identity["COM5"] = {"bluetooth_address": "AABBCCDDEEFF"}
    fo, clock = make(panel)
    drop(panel)
    run(fo, clock, 6)
    assert panel.requests == []


def test_usb_returning_within_grace_does_not_switch():
    panel = FakePanel(["ok"])
    fo, clock = make(panel)
    drop(panel)
    run(fo, clock, 1)
    panel._serial_port_map["USB (COM5)"] = "COM5"
    run(fo, clock, 5)
    assert panel.requests == []
    assert "已恢复" in panel.autoconnect_var.get()


def test_user_reconnect_during_grace_cancels():
    panel = FakePanel(["ok"])
    fo, clock = make(panel)
    drop(panel)
    run(fo, clock, 1)
    panel.serial_transport.connected = True
    run(fo, clock, 5)
    assert panel.requests == []


def test_usb_replug_after_switch_does_not_take_back():
    panel = FakePanel(["ok"])
    fo, clock = make(panel)
    drop(panel)
    run(fo, clock, 5)
    assert panel.serial_transport.connected
    panel._serial_port_map["USB (COM5)"] = "COM5"
    run(fo, clock, 10)
    assert len(panel.requests) == 1 and panel.serial_transport.active_port == "COM9"


def test_existing_disconnect_at_startup_is_ignored():
    panel = FakePanel(["ok"])
    drop(panel)
    fo, clock = make(panel)          # 构造时已有的旧断开记录不触发
    run(fo, clock, 6)
    assert panel.requests == []


def test_panel_wiring_and_persistence_key():
    import inspect
    from tools.panel_lib import connection_controls, state, ai_bridge
    assert "_link_failover.install(self)" in inspect.getsource(connection_controls)
    assert "usb_failover_bluetooth" in inspect.getsource(state)
    assert "link_failover.hold" in inspect.getsource(ai_bridge)
    assert lf.STATE_KEY == "usb_failover_bluetooth"

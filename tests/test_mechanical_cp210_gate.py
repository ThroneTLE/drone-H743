"""Regression for the author's CP210x COM10 telemetry-link screenshot."""

from types import SimpleNamespace

import pytest

from tools.drone_tcp_panel import DronePanel
from tools.panel_lib.pages import mechanical


def variable(value):
    return SimpleNamespace(get=lambda: value, set=lambda _: None)


def make_panel(description="CP210x", advisory=("ok", "safe")):
    serial = SimpleNamespace(active_port="COM10")
    sent = []
    obj = SimpleNamespace(
        transport=serial, serial_transport=serial,
        serial_port_var=variable(f"{description} (COM10)"),
        _serial_port_map={f"{description} (COM10)": "COM10"},
        _serial_port_identity={"COM10": {"description": description}},
        _transport_connected=lambda: True,
        _firmware_safety_advisory=lambda: advisory,
        validation_props_removed_var=variable(True),
        validation_power_safe_var=variable(True),
        _mechanical_row_values=lambda _: (1500, 1000, 2000),
        mechanical_rows={0: {"axis": variable("alpha"), "minimum": variable(1000),
                             "maximum": variable(2000), "center": variable(1500)}},
        mechanical_status_var=variable(""),
        mechanical_target_var=variable(""),
        _mechanical_local_target=lambda **_: {
            f"{axis}_{field}": value for axis in ("alpha", "beta")
            for field, value in (("center", 1500), ("min", 1000), ("max", 2000), ("sign", 1))
        },
        _send_proto=lambda *args: sent.append(args),
    )
    obj._firmware_link_gate = lambda: DronePanel._firmware_link_gate(obj)
    obj._validation_live_safety_gate = lambda: DronePanel._validation_live_safety_gate(obj)
    return obj, sent


def invoke(obj, action):
    if action == "move":
        mechanical.MechanicalPageMixin._mechanical_move(obj, 0, "positive")
    elif action == "nudge":
        mechanical.MechanicalPageMixin._mechanical_nudge_center(obj, 0, 10)
    else:
        mechanical.MechanicalPageMixin._mechanical_apply_target(obj)


@pytest.mark.parametrize("action,command", [
    ("move", "SERVO JOG 0 1550"), ("nudge", "SERVO JOG 0 1510"),
    ("apply", "SERVOCAL APPLY ac=1500 an=1000 ax=2000 as=1 bc=1500 bn=1000 bx=2000 bs=1"),
])
def test_cp210_move_allowed_but_firmware_upgrade_still_rejected(action, command, monkeypatch):
    monkeypatch.setattr(mechanical.messagebox, "showwarning", lambda *args: None)
    obj, sent = make_panel()
    assert not obj._firmware_link_gate()[0]
    invoke(obj, action)
    assert len(sent) == 1
    assert sent[0][1] == command


@pytest.mark.parametrize("condition", [
    "disconnected", "wrong_port", "tcp", "props", "power",
    "stale", "armed", "unknown", "CH340", "FTDI", "STLink",
])
@pytest.mark.parametrize("action", ["move", "nudge", "apply"])
def test_cp210_exception_preserves_other_guards(condition, action, monkeypatch):
    monkeypatch.setattr(mechanical.messagebox, "showwarning", lambda *args: None)
    obj, sent = make_panel()
    if condition == "disconnected":
        obj._transport_connected = lambda: False
    elif condition == "wrong_port":
        obj.serial_transport.active_port = "COM11"
    elif condition == "tcp":
        obj.transport = object()
    elif condition == "props":
        obj.validation_props_removed_var = variable(False)
    elif condition == "power":
        obj.validation_power_safe_var = variable(False)
    elif condition in ("stale", "armed", "unknown"):
        obj._firmware_safety_advisory = lambda: ("blocked", condition)
    else:
        obj._serial_port_identity["COM10"]["description"] = condition
    invoke(obj, action)
    assert sent == []

"""Mechanical-only CP210 telemetry allowance; firmware USB policy stays strict.

Consumes panel connection identity and its existing snapshot advisory; no I/O.
Tests exercise the real mechanical command handler with a fake transport.
"""

from .transport import serial_device_identity_policy


def mechanical_live_safety_gate(panel) -> tuple[bool, str]:
    if panel.transport is not panel.serial_transport:
        return False, "请选择顶部 serial 通道"
    if not panel._transport_connected():
        return False, "串口尚未连接"
    display = panel.serial_port_var.get().strip()
    selected = panel._serial_port_map.get(display, display)
    active = panel.serial_transport.active_port
    if not active or selected.casefold() != active.casefold():
        return False, f"连接端口与下拉选择不一致：active={active or '-'} selected={selected or '-'}"
    policy, reason = serial_device_identity_policy(
        panel._serial_port_identity.get(active), allow_cp210=True,
    )
    if policy == "rejected":
        return False, reason
    level, advisory = panel._firmware_safety_advisory()
    if level != "ok":
        return False, advisory
    return True, f"{active} · {reason} · {advisory}"

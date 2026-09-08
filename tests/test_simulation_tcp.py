import socket
import queue
import threading
import time
import tkinter as tk

import pytest
from tools import drone_tcp_panel as panel
from tools.panel_lib.proto import PROTO_REQ_CAPS, PROTO_REQ_PARAMS
from tools.panel_lib.proto import PROTO_MSG_TELEM_FRAME
from tools.panel_lib.transport import build_proto_frame
from tools.panel_lib.transport import TcpTransport
from tools.sim_xz.device import SimulatorDevice
from tools.sim_xz.experiments import SimulationEngine
from tools.sim_xz.protocol import FrameDecoder
from tools.sim_xz.controller_bridge import ControllerBridge
from tools.sim_xz.physics import SimulationState
from tools.sim_xz.protocol import SimulatorProtocol


class _DronePanelLoopback:
    is_connected = True

    def __init__(self, app, protocol: SimulatorProtocol) -> None:
        self.app = app
        self.protocol = protocol
        self.frames: list[tuple[int, bytes]] = []

    def set_binary_sink(self, _sink) -> None:
        return

    def send_line(self, line: str) -> bool:
        for response in self.protocol.handle_line(line):
            self._deliver(response)
        return True

    def send_frame(self, function: int, payload: bytes = b"") -> bool:
        self.frames.append((function, payload))
        for response in self.protocol.handle_frame(function, payload):
            self._deliver(response)
        return True

    def _deliver(self, raw: bytes) -> None:
        for frame in FrameDecoder().feed(raw):
            if frame.function == PROTO_MSG_TELEM_FRAME:
                self.app._dashboard_on_binary_frame(frame.function, frame.payload)
            else:
                self.app._handle_proto_frame(frame.function,
                                             frame.payload.decode("utf-8", "replace"))


def test_simulator_is_tcp_client_and_answers_caps_on_loopback() -> None:
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind(("127.0.0.1", 0))
    server.listen(1)
    port = server.getsockname()[1]
    received: list[bytes] = []

    def accept_once() -> None:
        client, _ = server.accept()
        client.sendall(build_proto_frame(ord("<"), PROTO_REQ_CAPS, b""))
        client.settimeout(2.0)
        received.append(client.recv(4096))
        client.close()

    thread = threading.Thread(target=accept_once, daemon=True)
    thread.start()
    device = SimulatorDevice(port=port)
    device.start()
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline and not received:
        time.sleep(0.02)
    device.stop()
    server.close()
    thread.join(timeout=1.0)
    assert received
    frames = FrameDecoder().feed(received[0])
    assert any(b"CAPS sim=1" in frame.payload for frame in frames)


def test_real_panel_tcp_transport_routes_caps_params_and_telem() -> None:
    rx: queue.Queue[object] = queue.Queue()
    transport = TcpTransport(rx)
    probe = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    probe.bind(("127.0.0.1", 0))
    port = probe.getsockname()[1]
    probe.close()
    transport.start("127.0.0.1", port)
    device = SimulatorDevice(port=port)
    device.start()
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline and not transport.is_connected:
        time.sleep(0.02)
    assert transport.is_connected
    transport.send_frame(PROTO_REQ_CAPS)
    transport.send_frame(PROTO_REQ_PARAMS)
    transport.send_line("TELEM?")
    lines: list[object] = []
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        try:
            lines.append(rx.get(timeout=0.1))
        except queue.Empty:
            if all(token in "\n".join(str(item) for item in lines)
                   for token in ("CAPS sim=1", "coax.pos_x_kp", "TELEM ver=3")):
                break
    device.stop()
    transport.stop()
    text = "\n".join(str(item) for item in lines)
    assert "CAPS sim=1" in text
    assert "coax.pos_x_kp" in text
    assert "TELEM ver=3" in text


def test_real_drone_panel_param_control_reaches_c_and_telem_ring() -> None:
    try:
        app = panel.DronePanel()
    except tk.TclError as exc:
        pytest.skip(f"Tk display unavailable: {exc}")
    bridge = ControllerBridge(instance_tag="drone_panel_e2e")
    protocol = SimulatorProtocol(bridge, lambda: SimulationState(time_s=1.0))
    transport = _DronePanelLoopback(app, protocol)
    app.transport = transport
    try:
        for response in protocol.handle_line("TELEM?") + protocol.handle_line("TELEM CH from=6"):
            transport._deliver(response)
        app.update_idletasks()
        assert app.dashboard_schema.complete
        assert app._dashboard_send_param("sim_pos_x_kp", 0.91)
        assert bridge.get_param("coax.pos_x_kp") == pytest.approx(0.91, rel=1e-5)
        protocol.handle_line("TELEM STREAM on")
        transport._deliver(protocol.telemetry_frame())
        assert app.dashboard_decoder.stats.frames_ok >= 1
        assert app.dashboard_frames_seen >= 1
    finally:
        app.destroy()


def test_real_drone_panel_real_tcp_simulator_e2e() -> None:
    try:
        app = panel.DronePanel()
    except tk.TclError as exc:
        pytest.skip(f"Tk display unavailable: {exc}")
    bridge = ControllerBridge(instance_tag="real_drone_panel_tcp")
    rx: queue.Queue[object] = queue.Queue()
    transport = TcpTransport(rx)
    probe = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    probe.bind(("127.0.0.1", 0))
    port = probe.getsockname()[1]
    probe.close()
    engine = SimulationEngine(bridge=bridge)
    device = SimulatorDevice(port=port, engine=engine)
    transport.start("127.0.0.1", port)
    app.transport = transport
    transport.set_binary_sink(lambda function, payload: app._dashboard_on_binary_frame(
        function, payload, transport=transport))
    app.validation_session_active = False
    device.start()

    def drain_until(predicate, timeout=4.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                item = rx.get(timeout=0.05)
            except queue.Empty:
                continue
            payload = getattr(item, "payload", item)
            if isinstance(payload, tuple) and len(payload) == 3 and payload[0] == "proto":
                app._handle_proto_frame(int(payload[1]), str(payload[2]))
            if predicate():
                return True
        return predicate()

    try:
        assert drain_until(lambda: transport.is_connected)
        transport.send_line("TELEM?")
        assert drain_until(lambda: app.dashboard_schema.complete)
        if app._dashboard_channel("sim_pos_x_kp") is None:
            app._dashboard_adopt_schema()
        assert app._dashboard_channel("sim_pos_x_kp") is not None
        for channel, parameter, value in (
            ("sim_pos_x_kp", "coax.pos_x_kp", 0.93),
            ("sim_vel_x_kp", "coax.vel_x_kp", 0.73),
            ("sim_att_pitch_kp", "coax.att_pitch_kp", 1.13),
            ("sim_rate_pitch_kp", "coax.rate_pitch_kp", 0.17),
        ):
            assert app._dashboard_send_param(channel, value)
            assert drain_until(lambda parameter=parameter, value=value:
                               abs((bridge.get_param(parameter) or 0.0) - value) < 1e-5)
        transport.send_line("TELEM STREAM on")
        assert drain_until(lambda: app.dashboard_frames_seen >= 1)
        for channel, _parameter, value in (
            ("sim_pos_x_kp", "coax.pos_x_kp", 0.93),
            ("sim_vel_x_kp", "coax.vel_x_kp", 0.73),
            ("sim_att_pitch_kp", "coax.att_pitch_kp", 1.13),
            ("sim_rate_pitch_kp", "coax.rate_pitch_kp", 0.17),
        ):
            assert app._dashboard_latest(channel) == pytest.approx(value, rel=1e-4)
    finally:
        device.stop()
        transport.stop()
        app.destroy()

"""Existing $X text protocol adapter for the simulated device."""

from __future__ import annotations

import re
import struct
from dataclasses import dataclass
from typing import Callable

from tools.panel_lib.proto import (PROTO_DIR_FROM_FC, PROTO_DIR_TO_FC, PROTO_HEADER,
    PROTO_MAX_FRAME_PAYLOAD, PROTO_REQ_CAPS, PROTO_REQ_PARAMS, PROTO_REQ_PARAM_SET,
    PROTO_MSG_CAPS_RECORD, PROTO_MSG_CMD_ERR,
    PROTO_MSG_CMD_LINE, PROTO_MSG_CMD_OK, PROTO_MSG_PARAM_RECORD, PROTO_MSG_PONG,
    PROTO_MSG_TELEM_FRAME, PROTO_MSG_TEXT_LINE)
from tools.panel_lib.transport import build_proto_frame, proto_crc8_dvb_s2
from tools.panel_lib.telem_stream import TelemSchema

from .controller_bridge import ControllerBridge
from .physics import SimulationState
from .control_catalog import GAIN_CHANNELS

CHANNELS = (("sim_x", "m", "nav", -2.0, 2.0, "-"),
            ("sim_z", "m", "nav", 0.0, 3.0, "-"),
            ("sim_vx", "m/s", "nav", -2.0, 2.0, "-"),
            ("sim_vz", "m/s", "nav", -2.0, 2.0, "-"),
            ("sim_pitch", "rad", "attitude", -1.0, 1.0, "-"),
) + GAIN_CHANNELS + (
    ("sim_pitch_rate", "rad/s", "attitude", -5., 5., "-"),
    ("sim_thrust", "N", "actuator", 0., 20., "-"),
    ("sim_tilt", "rad", "actuator", -1., 1., "-"),
)



def _fnv1a(text: str, seed: int = 0x811C9DC5) -> int:
    result = seed
    for byte in text.encode("ascii"):
        result = ((result ^ byte) * 0x01000193) & 0xFFFFFFFF
    return result


def _schema_hash() -> int:
    schema = TelemSchema()
    schema.feed_line(f"TELEM ver=3 n={len(CHANNELS)} rate=40 page=6 hash=00000000 frame=body_flu contract=1")
    for index, (name, unit, group, minimum, maximum, param) in enumerate(CHANNELS):
        schema.feed_line(f"TELEM CH idx={index} name={name} unit={unit} min={minimum:.6f} "
                         f"max={maximum:.6f} grp={group} param={param}")
    schema.feed_line(f"TELEM PAGE from=0 count={len(CHANNELS)} next=-1")
    return schema.computed_hash()


@dataclass(frozen=True)
class InboundFrame:
    function: int
    payload: bytes


class FrameDecoder:
    def __init__(self, expected_direction: int | None = None) -> None:
        self.buffer = bytearray()
        self.expected_direction = expected_direction

    def feed(self, data: bytes) -> list[InboundFrame]:
        self.buffer.extend(data)
        frames: list[InboundFrame] = []
        while True:
            start = self.buffer.find(PROTO_HEADER)
            if start < 0:
                self.buffer = bytearray(self.buffer[-1:] if self.buffer.endswith(b"$") else b"")
                break
            if start:
                del self.buffer[:start]
            if len(self.buffer) < 9:
                break
            if self.expected_direction is not None and self.buffer[2] != self.expected_direction:
                del self.buffer[0]
                continue
            length = self.buffer[6] | (self.buffer[7] << 8)
            if length > PROTO_MAX_FRAME_PAYLOAD:
                del self.buffer[0]
                continue
            frame_length = 9 + length
            if len(self.buffer) < frame_length:
                break
            frame = bytes(self.buffer[:frame_length])
            if proto_crc8_dvb_s2(frame[3:-1]) != frame[-1]:
                del self.buffer[0]
                continue
            del self.buffer[:frame_length]
            frames.append(InboundFrame(frame[4] | (frame[5] << 8), frame[8:-1]))
        return frames


class SimulatorProtocol:
    def __init__(self, bridge: ControllerBridge,
                 snapshot_provider: Callable[[], SimulationState]) -> None:
        self.bridge = bridge
        self.snapshot_provider = snapshot_provider
        self.stream_enabled = False
        self.stream_rate_hz = 40
        self.stream_mask = (1 << len(CHANNELS)) - 1
        self.sequence = 0
        self.schema_hash = _schema_hash()

    @staticmethod
    def _frame(function: int, text: str) -> bytes:
        payload = text.rstrip("\r\n").encode("utf-8") + b"\r\n"
        return build_proto_frame(PROTO_DIR_FROM_FC, function, payload)

    def _param_frames(self) -> list[bytes]:
        frames = []
        for name in self.bridge.parameter_names():
            value = self.bridge.get_param(name)
            if value is not None:
                frames.append(self._frame(PROTO_MSG_PARAM_RECORD,
                                          f"PARAM name={name} value={value:.6f}"))
        return frames

    def _schema_frames(self, start: int = 0) -> list[bytes]:
        frames: list[bytes] = []
        if start == 0:
            frames.append(self._frame(PROTO_MSG_TEXT_LINE,
                f"TELEM ver=3 n={len(CHANNELS)} rate={self.stream_rate_hz} page=6 hash={self.schema_hash:08X} frame=body_flu contract=1"))
        end = min(start + 6, len(CHANNELS))
        for index in range(start, end):
            name, unit, group, minimum, maximum, param = CHANNELS[index]
            frames.append(self._frame(PROTO_MSG_TEXT_LINE,
                f"TELEM CH idx={index} name={name} unit={unit} min={minimum:.6f} max={maximum:.6f} grp={group} param={param}"))
        frames.append(self._frame(PROTO_MSG_TEXT_LINE,
            f"TELEM PAGE from={start} count={end - start} next={end if end < len(CHANNELS) else -1}"))
        return frames

    def telemetry_frame(self) -> bytes:
        state = self.snapshot_provider()
        parameters = self.bridge.parameter_snapshot()
        all_values = ((state.x_m, state.z_m, state.vx_m_s, state.vz_m_s, state.pitch_rad)
                      + tuple(parameters[channel[5]] for channel in GAIN_CHANNELS)
                      + (state.pitch_rate_rad_s, state.thrust_n, state.pitch_tilt_rad))
        indices = [index for index in range(len(CHANNELS)) if self.stream_mask & (1 << index)]
        values = tuple(all_values[index] for index in indices)
        payload = struct.pack("<BBHIIHHQ", 2, 1, self.sequence & 0xFFFF,
                              self.schema_hash, int(state.time_s * 1000000) & 0xFFFFFFFF,
                              0, 1, self.stream_mask)
        self.sequence = (self.sequence + 1) & 0xFFFF
        return build_proto_frame(PROTO_DIR_FROM_FC, PROTO_MSG_TELEM_FRAME,
                                 payload + struct.pack(f"<{len(values)}f", *values))

    def handle_frame(self, function: int, payload: bytes) -> list[bytes]:
        text = payload.decode("utf-8", "replace")
        if function == PROTO_REQ_CAPS:
            return [self._frame(PROTO_MSG_CAPS_RECORD, "CAPS sim=1 name=xz-simulator mode=teaching")]
        if function == PROTO_REQ_PARAMS:
            return self._param_frames()
        if function == PROTO_REQ_PARAM_SET:
            return self.handle_line(text)
        if function == PROTO_MSG_CMD_LINE:
            return self.handle_line(text)
        return [self._frame(PROTO_MSG_CMD_ERR, f"ERR function=0x{function:04X}")]

    def handle_line(self, line: str) -> list[bytes]:
        command = line.strip()
        if command == "PING":
            return [self._frame(PROTO_MSG_PONG, "PONG sim-xz")]
        if command == "CAPS?":
            return [self._frame(PROTO_MSG_CAPS_RECORD, "CAPS sim=1 name=xz-simulator mode=teaching")]
        if command == "PARAM?":
            return self._param_frames()
        match = re.fullmatch(r"PARAM SET (\S+) ([+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?)", command)
        if match:
            name, raw = match.groups()
            value = float(raw)
            if self.bridge.set_param(name, value):
                actual = self.bridge.get_param(name)
                return [self._frame(PROTO_MSG_PARAM_RECORD, f"PARAM name={name} value={(actual if actual is not None else value):.6f}"),
                        self._frame(PROTO_MSG_CMD_OK, f"OK param name={name}")]
            return [self._frame(PROTO_MSG_CMD_ERR, f"ERR param name={name}")]
        if command == "TELEM?":
            return self._schema_frames(0) + [self._frame(PROTO_MSG_TEXT_LINE,
                f"TELEM STREAM stream={int(self.stream_enabled)} rate={self.stream_rate_hz} mask={self.stream_mask:X} refresh=1 fmt=bin sink=usb")]
        match = re.fullmatch(r"TELEM CH from=(\d+)", command)
        if match:
            return self._schema_frames(int(match.group(1)))
        match = re.fullmatch(r"TELEM STREAM (on|off)", command)
        if match:
            self.stream_enabled = match.group(1) == "on"
            return [self._frame(PROTO_MSG_CMD_OK, f"OK stream={int(self.stream_enabled)}")]
        match = re.fullmatch(r"TELEM RATE (\d+)", command)
        if match:
            rate = int(match.group(1))
            if not 1 <= rate <= 1000:
                return [self._frame(PROTO_MSG_CMD_ERR, "ERR rate range=1..1000")]
            self.stream_rate_hz = rate
            return [self._frame(PROTO_MSG_CMD_OK, f"OK rate={rate}")]
        match = re.fullmatch(r"TELEM MASK ([0-9A-Fa-f]+)", command)
        if match:
            mask = int(match.group(1), 16)
            if mask == 0 or mask & ~((1 << len(CHANNELS)) - 1):
                return [self._frame(PROTO_MSG_CMD_ERR, "ERR mask")]
            self.stream_mask = mask
            return [self._frame(PROTO_MSG_CMD_OK, f"OK mask={mask:X}")]
        return [self._frame(PROTO_MSG_CMD_ERR, f"ERR unknown={command}")]

import struct

from tools.panel_lib.proto import (PROTO_MSG_CMD_LINE, PROTO_MSG_TELEM_FRAME,
                                   PROTO_REQ_CAPS, PROTO_REQ_PARAM_SET)
from tools.panel_lib.transport import build_proto_frame
from tools.panel_lib.telem_stream import TelemSchema
from tools.sim_xz.controller_bridge import ControllerBridge
from tools.sim_xz.physics import SimulationState
from tools.sim_xz.protocol import FrameDecoder, SimulatorProtocol, CHANNELS


def test_frame_decoder_accepts_fragmented_real_protocol_frame() -> None:
    frame = build_proto_frame(ord("<"), PROTO_MSG_CMD_LINE, b"CAPS?")
    decoder = FrameDecoder()
    assert decoder.feed(frame[:4]) == []
    decoded = decoder.feed(frame[4:])
    assert decoded[0].function == PROTO_MSG_CMD_LINE
    assert decoded[0].payload == b"CAPS?"


def test_frame_decoder_preserves_a_split_dollar_prefix() -> None:
    frame = build_proto_frame(ord("<"), PROTO_MSG_CMD_LINE, b"CAPS?")
    decoder = FrameDecoder()
    assert decoder.feed(frame[:1]) == []
    assert decoder.feed(frame[1:])[0].payload == b"CAPS?"


def test_caps_param_and_telem_requests_use_existing_function_ids() -> None:
    bridge = ControllerBridge()
    protocol = SimulatorProtocol(bridge, lambda: SimulationState(time_s=1.0))
    assert b"CAPS sim=1" in protocol.handle_line("CAPS?")[0]
    assert any(b"coax.pos_x_kp" in frame for frame in protocol.handle_line("PARAM?"))
    assert len(protocol.handle_line("TELEM?")[:7]) >= 7
    protocol.handle_line("TELEM STREAM on")
    telemetry = protocol.telemetry_frame()
    assert telemetry[0:3] == b"$X>"
    assert struct.unpack_from("<H", telemetry, 4)[0] == PROTO_MSG_TELEM_FRAME


def test_real_request_functions_and_schema_mask_are_applied() -> None:
    bridge = ControllerBridge()
    protocol = SimulatorProtocol(bridge, lambda: SimulationState(time_s=1.0))
    assert b"CAPS sim=1" in protocol.handle_frame(PROTO_REQ_CAPS, b"")[0]
    set_result = protocol.handle_frame(PROTO_REQ_PARAM_SET, b"PARAM SET coax.pos_x_kp 0.91")
    assert any(b"value=0.910000" in frame for frame in set_result)
    protocol.handle_line("TELEM MASK 3")
    frame = FrameDecoder().feed(protocol.telemetry_frame())[0]
    assert struct.unpack_from("<Q", frame.payload, 16)[0] == 3
    assert len(frame.payload) == 24 + 2 * 4


def test_schema_lines_recompute_the_reported_hash() -> None:
    bridge = ControllerBridge()
    protocol = SimulatorProtocol(bridge, lambda: SimulationState())
    schema = TelemSchema()
    decoder = FrameDecoder()
    for raw in sum((protocol.handle_line("TELEM?" if offset == 0 else f"TELEM CH from={offset}") for offset in range(0, len(CHANNELS), 6)), []):
        for frame in decoder.feed(raw):
            schema.feed_line(frame.payload.decode("utf-8"))
    assert schema.complete
    assert schema.reported_hash == protocol.schema_hash

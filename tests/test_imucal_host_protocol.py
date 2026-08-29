from __future__ import annotations

import struct
from pathlib import Path

from tools.imucal_protocol import (
    V1_CANDIDATE_FORMAT,
    V1_CANDIDATE_SIZE,
    parse_imucal_line,
    upload_commands,
    upload_and_apply,
)


class DummyEncoded:
    payload = bytes(range(128))
    crc32 = 0x12345678
    firmware_crc32 = 0x1234ABCD
    base_generation = 7
    orientation_code = 3


class FakeLink:
    def __init__(self) -> None:
        self.lines: list[str] = []

    def send(self, command: str) -> None:
        if command == "IMUCAL?":
            self.lines.extend([
                "IMUCAL event=status reason=none transfer=empty received=0 expected=0 candidate=0 applied=0 commit_pending=0 dirty=0 arm_lock=0 request=0",
                "IMUCAL generations active=7 persisted=7 record=7 base=0 active_mask=0x07 persisted_mask=0x07 candidate_mask=0x00 active_orientation=3 persisted_orientation=3 candidate_orientation=255",
                "IMUCAL identity firmware_crc32=0x1234ABCD image_bytes=123",
            ])
        elif " BEGIN " in command:
            self.lines.append("IMUCAL event=begin reason=none transfer=receiving")
        elif " DATA " in command:
            self.lines.append("IMUCAL event=data reason=none transfer=receiving")
        elif command == "IMUCAL END":
            self.lines.append("IMUCAL event=ready reason=none transfer=ready")
        elif command == "IMUCAL APPLY":
            self.lines.append("IMUCAL event=applied reason=none transfer=ready applied=1 arm_lock=1")

    def read_text_line(self, _deadline: float) -> str | None:
        return self.lines.pop(0) if self.lines else None


def test_wire_abi_and_chunk_commands_match_firmware() -> None:
    assert V1_CANDIDATE_SIZE == 128
    assert struct.calcsize(V1_CANDIDATE_FORMAT) == 128
    commands = upload_commands(DummyEncoded())  # type: ignore[arg-type]
    assert commands[0] == "IMUCAL BEGIN size=128 crc=12345678"
    assert commands[-1] == "IMUCAL END"
    assert len(commands) == 6
    for index, command in enumerate(commands[1:-1]):
        assert command.startswith(f"IMUCAL DATA offset={index * 32} hex=")
        assert len(command.rsplit("=", 1)[1]) == 64


def test_status_parser_ignores_text_and_extracts_machine_fields() -> None:
    assert parse_imucal_line("hello") == {}
    values = parse_imucal_line(
        "IMUCAL event=applied reason=none transfer=ready candidate=1 applied=1 arm_lock=1"
    )
    assert values == {
        "event": "applied", "reason": "none", "transfer": "ready",
        "candidate": "1", "applied": "1", "arm_lock": "1",
    }


def test_successful_receiving_ready_states_and_target_binding_are_accepted() -> None:
    result = upload_and_apply(FakeLink(), DummyEncoded())  # type: ignore[arg-type]
    assert result.action == "apply"
    assert result.final_status["applied"] == "1"


def test_panel_exposes_real_apply_revert_commit_controls() -> None:
    source = (Path(__file__).resolve().parents[1] / "tools" / "drone_tcp_panel.py").read_text(encoding="utf-8")
    assert "_v1_apply_candidate" in source
    assert "_v1_revert_candidate" in source
    assert "_v1_commit_candidate" in source
    assert "validate_candidate_for_application" in (
        Path(__file__).resolve().parents[1] / "tools" / "imucal_protocol.py"
    ).read_text(encoding="utf-8")

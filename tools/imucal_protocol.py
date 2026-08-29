#!/usr/bin/env python3
"""Host-side encoder and guarded transport for the target IMUCAL protocol."""

from __future__ import annotations

import json
import math
import re
import struct
import time
import zlib
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Sequence

try:
    from .imu_metrology import (
        MetrologyCandidate,
        MetrologySample,
        candidate_from_json,
        validate_candidate_for_application,
        validate_room_temperature_candidate_for_application,
    )
except ImportError:
    from imu_metrology import MetrologyCandidate, MetrologySample, candidate_from_json, validate_candidate_for_application, validate_room_temperature_candidate_for_application


V1_CANDIDATE_MAGIC = 0x31564349
V1_CANDIDATE_SCHEMA = 1
V1_CANDIDATE_FORMAT = "<IHHIHBB3f9f3f9f3ff"
V1_CANDIDATE_SIZE = struct.calcsize(V1_CANDIDATE_FORMAT)
V1_CHUNK_BYTES = 32
VALID_ORIENTATION = 0x01
VALID_ACCEL = 0x02
VALID_GYRO = 0x04
VALID_GYRO_TEMP = 0x08
FULL_V1_VALID_MASK = VALID_ORIENTATION | VALID_ACCEL | VALID_GYRO | VALID_GYRO_TEMP


class ImuCalProtocolError(RuntimeError):
    pass


@dataclass(frozen=True)
class EncodedV1Candidate:
    payload: bytes
    crc32: int
    orientation_code: int
    base_generation: int
    candidate_generation: int
    firmware_crc32: int
    session_id: str
    profile: str


@dataclass(frozen=True)
class ImuCalTransactionResult:
    action: str
    transcript: tuple[str, ...]
    final_status: dict[str, str]


def _flatten(matrix: Sequence[Sequence[float]]) -> tuple[float, ...]:
    if len(matrix) != 3 or any(len(row) != 3 for row in matrix):
        raise ValueError("calibration matrix must be 3x3")
    return tuple(float(value) for row in matrix for value in row)


def _bounded(values: Iterable[float], maximum: float, name: str) -> None:
    if any(not math.isfinite(float(value)) or abs(float(value)) > maximum for value in values):
        raise ValueError(f"{name} exceeds target physical bounds")


def encode_v1_candidate(
    candidate: MetrologyCandidate,
    samples: Sequence[MetrologySample],
) -> EncodedV1Candidate:
    """Replay the exact raw evidence, then encode the target's 128-byte ABI."""

    try:
        accepted = validate_candidate_for_application(candidate, samples)
        profile = "full_v1"
    except ValueError as full_error:
        try:
            accepted = validate_room_temperature_candidate_for_application(
                candidate, samples)
            profile = "room_temperature_v1"
        except ValueError as room_error:
            raise room_error from full_error
    accel = accepted.accelerometer
    gyro_static = accepted.gyro_static
    rotation = accepted.gyro_rotation
    temperature = accepted.temperature
    if (
        accel.bias_g is None
        or accel.correction_matrix is None
        or gyro_static.bias_dps is None
    ):
        raise ValueError("application-eligible candidate is missing base coefficients")
    firmware_match = re.fullmatch(r"crc32:([0-9a-fA-F]{8})", accepted.firmware_hash)
    if firmware_match is None:
        raise ValueError("candidate firmware_hash must be crc32:<8hex>")

    accel_bias = tuple(float(value) for value in accel.bias_g)
    accel_matrix = _flatten(accel.correction_matrix)
    gyro_bias = tuple(float(value) for value in gyro_static.bias_dps)
    if profile == "full_v1":
        if (rotation.correction_matrix is None or
                temperature.gyro_slope_dps_per_c is None or
                temperature.reference_temperature_c is None):
            raise ValueError("full V1 candidate is missing rotation/temperature coefficients")
        gyro_matrix = _flatten(rotation.correction_matrix)
        gyro_slope = tuple(float(value) for value in temperature.gyro_slope_dps_per_c)
        reference_temp = float(temperature.reference_temperature_c)
        valid_mask = FULL_V1_VALID_MASK
    else:
        gyro_matrix = (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)
        gyro_slope = (0.0, 0.0, 0.0)
        reference_temp = 25.0
        valid_mask = VALID_ORIENTATION | VALID_ACCEL | VALID_GYRO
    _bounded(accel_bias, 0.50, "accelerometer bias")
    _bounded(gyro_bias, 20.0, "gyro residual bias")
    _bounded(gyro_slope, 2.0, "gyro temperature slope")
    if not -40.0 <= reference_temp <= 85.0:
        raise ValueError("reference temperature exceeds target physical bounds")

    payload = struct.pack(
        V1_CANDIDATE_FORMAT,
        V1_CANDIDATE_MAGIC,
        V1_CANDIDATE_SCHEMA,
        V1_CANDIDATE_SIZE,
        accepted.base_calibration_generation,
        accepted.frame_contract_version,
        accepted.orientation_code,
        valid_mask,
        *accel_bias,
        *accel_matrix,
        *gyro_bias,
        *gyro_matrix,
        *gyro_slope,
        reference_temp,
    )
    return EncodedV1Candidate(
        payload=payload,
        crc32=zlib.crc32(payload) & 0xFFFFFFFF,
        orientation_code=accepted.orientation_code,
        base_generation=accepted.base_calibration_generation,
        candidate_generation=accepted.candidate_calibration_generation,
        firmware_crc32=int(firmware_match.group(1), 16),
        session_id=accepted.session_id,
        profile=profile,
    )


def load_and_encode_v1_candidate(
    candidate_path: Path,
    samples: Sequence[MetrologySample],
) -> EncodedV1Candidate:
    candidate = candidate_from_json(candidate_path.read_text(encoding="utf-8"))
    return encode_v1_candidate(candidate, samples)


def upload_commands(encoded: EncodedV1Candidate) -> tuple[str, ...]:
    commands = [
        f"IMUCAL BEGIN size={len(encoded.payload)} crc={encoded.crc32:08x}"
    ]
    for offset in range(0, len(encoded.payload), V1_CHUNK_BYTES):
        chunk = encoded.payload[offset : offset + V1_CHUNK_BYTES]
        commands.append(f"IMUCAL DATA offset={offset} hex={chunk.hex()}")
    commands.append("IMUCAL END")
    return tuple(commands)


def parse_imucal_line(line: str) -> dict[str, str]:
    if not line.startswith("IMUCAL "):
        return {}
    result: dict[str, str] = {}
    for token in line.split()[1:]:
        if "=" in token:
            key, value = token.split("=", 1)
            result[key] = value
    return result


def _read_until_event(link: Any, expected: set[str], timeout_s: float,
                      transcript: list[str]) -> dict[str, str]:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        line = link.read_text_line(deadline)
        if not line:
            continue
        transcript.append(line)
        if line.startswith("ERR "):
            raise ImuCalProtocolError(line)
        values = parse_imucal_line(line)
        event = values.get("event")
        if event in expected:
            reason = values.get("reason", "none")
            if reason != "none":
                raise ImuCalProtocolError(
                    f"target rejected IMUCAL event={event} reason={reason} "
                    f"transfer={values.get('transfer', 'unknown')}"
                )
            return values
    raise ImuCalProtocolError(f"timeout waiting for IMUCAL event {sorted(expected)}")


def _send_expect(link: Any, command: str, event: str,
                 transcript: list[str], timeout_s: float = 3.0) -> dict[str, str]:
    link.send(command)
    return _read_until_event(link, {event}, timeout_s, transcript)


def _verify_target_context(link: Any, encoded: EncodedV1Candidate,
                           transcript: list[str]) -> None:
    deadline = time.monotonic() + 3.0
    event_values: dict[str, str] = {}
    generation_values: dict[str, str] = {}
    identity_values: dict[str, str] = {}
    link.send("IMUCAL?")
    while time.monotonic() < deadline:
        line = link.read_text_line(deadline)
        if not line:
            continue
        transcript.append(line)
        values = parse_imucal_line(line)
        if "candidate" in values:
            event_values = values
        if "persisted" in values and "persisted_orientation" in values:
            generation_values = values
        if "firmware_crc32" in values:
            identity_values = values
        if event_values and generation_values and identity_values:
            break
    if not event_values or not generation_values or not identity_values:
        raise ImuCalProtocolError("target IMUCAL context reply is incomplete")
    if any(event_values.get(name) != "0" for name in
           ("candidate", "applied", "commit_pending", "dirty")):
        raise ImuCalProtocolError("target already has a candidate or dirty parameter state")
    target_crc = int(identity_values.get("firmware_crc32", "0"), 0)
    if target_crc != encoded.firmware_crc32:
        raise ImuCalProtocolError(
            f"firmware CRC mismatch target={target_crc:08x} evidence={encoded.firmware_crc32:08x}"
        )
    if int(generation_values.get("persisted", "-1"), 0) != encoded.base_generation:
        raise ImuCalProtocolError("target persisted generation does not match capture evidence")
    if int(generation_values.get("persisted_orientation", "-1"), 0) != encoded.orientation_code:
        raise ImuCalProtocolError("target orientation does not match capture evidence")


def upload_and_apply(link: Any, encoded: EncodedV1Candidate) -> ImuCalTransactionResult:
    transcript: list[str] = []
    _verify_target_context(link, encoded, transcript)
    commands = upload_commands(encoded)
    _send_expect(link, commands[0], "begin", transcript)
    for command in commands[1:-1]:
        _send_expect(link, command, "data", transcript)
    _send_expect(link, commands[-1], "ready", transcript)
    final = _send_expect(link, "IMUCAL APPLY", "applied", transcript)
    if final.get("applied") != "1" or final.get("arm_lock") != "1":
        raise ImuCalProtocolError("target did not retain the RAM candidate arm lock")
    return ImuCalTransactionResult("apply", tuple(transcript), final)


def revert_candidate(link: Any) -> ImuCalTransactionResult:
    transcript: list[str] = []
    final = _send_expect(link, "IMUCAL REVERT", "reverted", transcript)
    return ImuCalTransactionResult("revert", tuple(transcript), final)


def commit_candidate(link: Any, *, timeout_s: float = 12.0) -> ImuCalTransactionResult:
    transcript: list[str] = []
    queued = _send_expect(link, "IMUCAL COMMIT", "commit_queued", transcript)
    if queued.get("commit_pending") != "1":
        raise ImuCalProtocolError("target did not queue the parameter save")
    deadline = time.monotonic() + timeout_s
    final: dict[str, str] = queued
    while time.monotonic() < deadline:
        time.sleep(0.20)
        link.send("IMUCAL?")
        query_deadline = min(deadline, time.monotonic() + 1.0)
        while time.monotonic() < query_deadline:
            line = link.read_text_line(query_deadline)
            if not line:
                continue
            transcript.append(line)
            values = parse_imucal_line(line)
            if "commit_pending" not in values:
                continue
            final = values
            if (
                values.get("commit_pending") == "0"
                and values.get("dirty") == "0"
                and values.get("applied") == "0"
                and values.get("candidate") == "0"
            ):
                return ImuCalTransactionResult("commit", tuple(transcript), final)
            break
    raise ImuCalProtocolError(
        "timeout waiting for confirmed dirty=0 after IMUCAL COMMIT"
    )


def write_transaction_record(path: Path, *, encoded: EncodedV1Candidate,
                             result: ImuCalTransactionResult) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "format": "drone-h743-imucal-transaction",
        "schema": 1,
        "created_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "action": result.action,
        "candidate_crc32": f"{encoded.crc32:08x}",
        "candidate_size": len(encoded.payload),
        "orientation_code": encoded.orientation_code,
        "base_generation": encoded.base_generation,
        "candidate_generation": encoded.candidate_generation,
        "firmware_crc32": f"{encoded.firmware_crc32:08x}",
        "session_id": encoded.session_id,
        "profile": encoded.profile,
        "final_status": result.final_status,
        "transcript": list(result.transcript),
    }
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    temporary.replace(path)
    return path


__all__ = [
    "EncodedV1Candidate", "ImuCalProtocolError", "ImuCalTransactionResult",
    "V1_CANDIDATE_FORMAT", "V1_CANDIDATE_SIZE", "commit_candidate",
    "encode_v1_candidate", "load_and_encode_v1_candidate", "parse_imucal_line",
    "revert_candidate", "upload_and_apply", "upload_commands", "write_transaction_record",
]

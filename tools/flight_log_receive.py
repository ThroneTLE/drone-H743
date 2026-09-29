#!/usr/bin/env python3
"""Receive FlightLog dumps from USART1 and convert them to bin/csv/json."""

from __future__ import annotations

import argparse
import csv
import json
import queue
import re
import struct
import sys
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import BinaryIO, Iterable

try:
    from .project_paths import FLIGHT_LOG_DIR, dated_directory
except ImportError:  # Allows running as: python tools/flight_log_receive.py
    try:
        from tools.project_paths import FLIGHT_LOG_DIR, dated_directory
    except ImportError:
        from project_paths import FLIGHT_LOG_DIR, dated_directory

try:
    import tkinter as tk
    from tkinter import filedialog, messagebox, ttk
except Exception:  # pragma: no cover - CLI/parser tests do not need Tk.
    tk = None
    ttk = None
    filedialog = None
    messagebox = None

try:
    import serial
    from serial.tools import list_ports
except Exception:  # pragma: no cover - handled at runtime in the GUI.
    serial = None
    list_ports = None


DEFAULT_BAUD = 57600
DEFAULT_OUT_DIR = dated_directory(FLIGHT_LOG_DIR)
DEFAULT_TIMEOUT_S = 2.0
SECTOR_SIZE = 4096
SECTOR_HEADER_SIZE = 256
SECTOR_MAGIC = 0x31534C46
RECORD_MAGIC = 0x31524C46
EXPORT_BLOCK_MAGIC = 0x31424C46
EXPORT_BLOCK_LAST = 0x0001
EXPORT_PAYLOAD_MAX = 1024
EXPORT_TEXT_LINE_MAX = 160 + (EXPORT_PAYLOAD_MAX * 2)
TEXT_LOG_PREFIXES = ("FLOG ", "OK ", "ERR ", "BOOT ")

MOTOR_REASON_NAMES = {
    0: "unknown",
    1: "stabilized_mix",
    2: "direct_throttle",
    3: "disarmed_min",
    4: "no_rc_seen_min",
    5: "rc_loss_disable",
    6: "ident_direct",
    7: "imu_invalid_direct",
    8: "attitude_debug",
    9: "prop_spin_test",
}

LEGACY_PARAM_NAMES = [
    "pos_x_kp",
    "pos_y_kp",
    "pos_z_kp",
    "vel_x_kd",
    "vel_y_kd",
    "vel_z_kd",
    "vel_loop_enable",
    "vel_loop_x_kp",
    "vel_loop_x_ki",
    "vel_loop_x_kd",
    "vel_loop_y_kp",
    "vel_loop_y_ki",
    "vel_loop_y_kd",
    "mass_kg",
    "gravity_m_s2",
    "tilt_lever_arm_m",
    "roll_angle_kp",
    "pitch_angle_kp",
    "roll_rate_kd",
    "pitch_rate_kd",
    "tilt_limit_rad",
    "yaw_angle_kp",
    "yaw_rate_kd",
    "yaw_inertia",
    "motor_single_max_thrust_n",
    "yaw_torque_upper_m_per_n",
    "yaw_torque_lower_m_per_n",
]

# v6~v9 参数快照里的 pitch/roll_tilt_lever_arm_m 存的是当时的**输入力臂**（0.145 m），
# 固件再乘 0.569/0.581 的经验系数才得到倾转力矩。2026-09-27 起固件同名字段改存带符号
# 几何力臂 = 重心 z − 舵机转轴 z（实测 +0.035442 m，无系数）；v10 起的快照不含这两项。
# 解读旧日志时按旧口径，不要拿它和新固件的数直接比。布局不变，只是说明口径。
V8_PARAM_NAMES = [
    *LEGACY_PARAM_NAMES[:3],
    "pos_z_ki",
    *LEGACY_PARAM_NAMES[3:15],
    "pitch_tilt_lever_arm_m",
    "roll_tilt_lever_arm_m",
    *LEGACY_PARAM_NAMES[16:],
]

# v9 删除 vel_loop_x/y_kp/ki/kd：那六个增益从未接入控制律。
VEL_LOOP_GAIN_NAMES = (
    "vel_loop_x_kp",
    "vel_loop_x_ki",
    "vel_loop_x_kd",
    "vel_loop_y_kp",
    "vel_loop_y_ki",
    "vel_loop_y_kd",
)
V9_PARAM_NAMES = [name for name in V8_PARAM_NAMES if name not in VEL_LOOP_GAIN_NAMES]

# v10 is the physically named four-loop cascade parameter snapshot.
PARAM_NAMES = [
    "pos_x_kp", "pos_y_kp", "pos_z_kp",
    "pos_xy_vel_max_m_s", "pos_z_vel_up_max_m_s", "pos_z_vel_down_max_m_s",
    "vel_x_kp", "vel_y_kp", "vel_z_kp",
    "vel_x_ki", "vel_y_ki", "vel_z_ki",
    "vel_x_kd", "vel_y_kd", "vel_z_kd",
    "vel_x_i_limit_m_s2", "vel_y_i_limit_m_s2", "vel_z_i_limit_m_s2",
    "accel_lpf_cutoff_hz", "accel_xy_max_m_s2",
    "accel_z_up_max_m_s2", "accel_z_down_max_m_s2",
    "att_roll_kp", "att_pitch_kp", "att_yaw_kp",
    "roll_rate_limit_rad_s", "pitch_rate_limit_rad_s", "yaw_rate_limit_rad_s",
    "rate_roll_kp", "rate_pitch_kp", "rate_yaw_kp",
    "rate_roll_ki", "rate_pitch_ki", "rate_yaw_ki",
    "rate_roll_kd", "rate_pitch_kd", "rate_yaw_kd",
    "rate_roll_i_limit_n_m", "rate_pitch_i_limit_n_m", "rate_yaw_i_limit_n_m",
    "angular_accel_lpf_cutoff_hz",
    "rate_roll_ff", "rate_pitch_ff", "rate_yaw_ff",
    "tilt_limit_rad", "vel_loop_enable",
]

V6_PARAM_NAMES = [
    *LEGACY_PARAM_NAMES[:15],
    "pitch_tilt_lever_arm_m",
    "roll_tilt_lever_arm_m",
    *LEGACY_PARAM_NAMES[16:],
]

SECTOR_HEADER_PREFIX = struct.Struct("<IHHIIIIIIIIQII")
PARAMS_STRUCT = struct.Struct("<" + "f" * len(PARAM_NAMES))
V9_PARAMS_STRUCT = struct.Struct("<" + "f" * len(V9_PARAM_NAMES))
V8_PARAMS_STRUCT = struct.Struct("<" + "f" * len(V8_PARAM_NAMES))
V6_PARAMS_STRUCT = struct.Struct("<" + "f" * len(V6_PARAM_NAMES))
LEGACY_PARAMS_STRUCT = struct.Struct("<" + "f" * len(LEGACY_PARAM_NAMES))
# 坐标溯源块（扇区头 v8 起），紧跟 params，占原 reserved 区的前 12 字节。
# v7 及更早在该位置是清零保留区，故解析出来天然是「无溯源」。
FRAME_PROVENANCE_STRUCT = struct.Struct("<BBBBII")
FRAME_PROVENANCE_NAMES = (
    "frame_provenance_valid",
    "frame_orientation_code",
    "frame_contract_version",
    "frame_reserved0",
    "firmware_crc32",
    "calibration_generation",
)
# V0 码 0..23 表示姿态已是规范 FLU；255 表示 legacy 中间轴。
FRAME_ORIENTATION_LEGACY = 255
FRAME_ORIENTATION_COUNT = 24

EXPORT_HEADER = struct.Struct("<IHHIIHHI")
EXPORT_BLOCK_MAGIC_BYTES = struct.pack("<I", EXPORT_BLOCK_MAGIC)
# V3 (320B) was the record layout until 2026-07-24; it has no servo
# feedback / ident_att / v6/v7 diagnostic sections.  Layout derived from
# commit 85a5cac (APP_FlightLogRecord packed) and verified by CRC on real logs.
V3_RECORD_STRUCT = struct.Struct(
    "<IHHIIQII"
    + "h" * 7
    + "f" * 7
    + "f" * 3
    + "H" * 8
    + "H" * 5
    + "B" * 12
    + "f" * 18
    + "f"
    + "f" * 28
    + "f"
    + "I"
)
V3_RECORD_SIZE = V3_RECORD_STRUCT.size
LEGACY_RECORD_STRUCT = struct.Struct(
    "<IHHIIQII"
    + "h" * 7
    + "f" * 7
    + "f" * 3
    + "H" * 8
    + "H" * 5
    + "B" * 12
    + "f" * 64
    + "I"
    + "f"
    + "I"
)
LEGACY_RECORD_SIZE = LEGACY_RECORD_STRUCT.size
V4_RECORD_STRUCT = struct.Struct(
    "<IHHIIQII"
    + "h" * 7
    + "f" * 7
    + "f" * 3
    + "H" * 8
    + "H" * 5
    + "H" * 6
    + "B" * 4
    + "B" * 12
    + "f" * 64
    + "I"
    + "f"
    + "I"
)
V4_RECORD_SIZE = V4_RECORD_STRUCT.size
V5_RECORD_STRUCT = struct.Struct(
    "<IHHIIQII"
    + "h" * 7
    + "f" * 7
    + "f" * 3
    + "H" * 8
    + "H" * 5
    + "H" * 6
    + "B" * 4
    + "B" * 12
    + "f" * 64
    + "I"
    + "f"
    + "B" * 4
    + "I"
    + "f" * 2
    + "I"
    + "I"
)
V5_RECORD_SIZE = V5_RECORD_STRUCT.size
V6_RECORD_STRUCT = struct.Struct(
    "<IHHIIQII"
    + "h" * 7
    + "f" * 7
    + "f" * 3
    + "H" * 8
    + "H" * 5
    + "H" * 6
    + "B" * 4
    + "B" * 12
    + "f" * 64
    + "I"
    + "f"
    + "B" * 4
    + "I"
    + "f" * 2
    + "I"
    + "H" * 2
    + "h" * 2
    + "H" * 2
    + "B" * 4
    + "f" * 10
    + "I" * 10
    + "I"
)
V6_RECORD_SIZE = V6_RECORD_STRUCT.size
V8_RECORD_STRUCT = struct.Struct(
    "<IHHIIQII"
    + "h" * 7
    + "f" * 7
    + "f" * 3
    + "H" * 8
    + "H" * 5
    + "H" * 6
    + "B" * 4
    + "B" * 12
    + "f" * 65
    + "I"
    + "f"
    + "B" * 4
    + "I"
    + "f" * 2
    + "I"
    + "H" * 2
    + "h" * 2
    + "H" * 2
    + "B" * 4
    + "f" * 10
    + "I" * 10
    + "I"
)
V8_RECORD_SIZE = V8_RECORD_STRUCT.size
V9_RECORD_STRUCT = struct.Struct(
    "<IHHIIQII"
    + "h" * 7
    + "f" * 7
    + "f" * 3
    + "H" * 8
    + "H" * 5
    + "H" * 6
    + "B" * 4
    + "B" * 12
    + "f" * 61
    + "I"
    + "f"
    + "B" * 4
    + "I"
    + "f" * 2
    + "I"
    + "H" * 2
    + "h" * 2
    + "H" * 2
    + "B" * 4
    + "f" * 10
    + "I" * 10
    + "I"
)
V9_RECORD_SIZE = V9_RECORD_STRUCT.size
V10_RECORD_STRUCT = struct.Struct(
    "<IHHIIQII"
    + "h" * 7
    + "f" * 7
    + "f" * 3
    + "H" * 8
    + "H" * 5
    + "H" * 6
    + "B" * 4
    + "B" * 12
    + "f" * 121
    + "B" * 9
    + "3x"
    + "f" * 3
    + "I"
    + "f"
    + "B" * 4
    + "I"
    + "f" * 2
    + "I"
    + "H" * 2
    + "h" * 2
    + "H" * 2
    + "B" * 4
    + "f" * 10
    + "I" * 10
    + "I"
)
V10_RECORD_SIZE = V10_RECORD_STRUCT.size
DSHOT_NAMES = ("dshot_present", "dshot_enabled_mask", "dshot_busy", "dshot_fault",
               "dshot_code_ch1", "dshot_code_ch2", "dshot_submitted", "dshot_completed",
               "dshot_busy_rejected", "dshot_errors", "dshot_cancelled", "dshot_timer_clock_hz")
RECORD_STRUCT = struct.Struct(V10_RECORD_STRUCT.format[:-1] + "4B2H6II")
RECORD_SIZE = RECORD_STRUCT.size


class FlightLogError(RuntimeError):
    pass


@dataclass
class ExportBegin:
    fields: dict[str, str]

    @property
    def total(self) -> int:
        return int(self.fields.get("total", "0"), 0)


@dataclass
class ExportBlock:
    seq: int
    offset: int
    payload: bytes
    flags: int
    crc32: int


@dataclass
class ExportEnd:
    fields: dict[str, str]

    @property
    def reason(self) -> str:
        return self.fields.get("reason", "")

    @property
    def sent(self) -> int:
        return int(self.fields.get("sent", "0"), 0)

    @property
    def total(self) -> int:
        return int(self.fields.get("total", "0"), 0)


@dataclass
class ReceiveResult:
    bin_path: Path
    csv_path: Path
    meta_path: Path
    total_bytes: int
    blocks: int
    records: int
    sectors: int
    errors: list[str] = field(default_factory=list)
    good_bytes: int = 0
    missing_bytes: int = 0
    complete: bool = False


def crc32(data: bytes) -> int:
    crc = 0xFFFFFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            mask = -(crc & 1) & 0xFFFFFFFF
            crc = ((crc >> 1) ^ (0xEDB88320 & mask)) & 0xFFFFFFFF
    return (~crc) & 0xFFFFFFFF


def whiten_byte(offset: int) -> int:
    x = (offset + 0xA5A5A5A5) & 0xFFFFFFFF
    x ^= x >> 7
    x = (x * 0x045D9F3B) & 0xFFFFFFFF
    x ^= x >> 11
    return x & 0xFF | 0x01


def xor_whiten(data: bytes, offset: int) -> bytes:
    return bytes(byte ^ whiten_byte(offset + index) for index, byte in enumerate(data))


def parse_key_values(line: str) -> dict[str, str]:
    return dict(re.findall(r"([A-Za-z_]+)=([^ \r\n]+)", line))


def reset_input_buffer(port: BinaryIO) -> None:
    reset = getattr(port, "reset_input_buffer", None)
    if callable(reset):
        reset()


def should_log_text_line(text: str) -> bool:
    return text.startswith(TEXT_LOG_PREFIXES)


def log_text_line(log: callable | None, text: str) -> None:
    if log and text and should_log_text_line(text):
        log(text)


def find_status_line(text: str, marker: str) -> str | None:
    index = text.find(marker)
    if index < 0:
        return None
    return text[index:].strip()


def parse_begin_line(line: str) -> ExportBegin:
    if not line.startswith("FLOG BEGIN"):
        raise FlightLogError(f"expected FLOG BEGIN, got {line!r}")
    fields = parse_key_values(line)
    if int(fields.get("block_magic", "0"), 0) != EXPORT_BLOCK_MAGIC:
        raise FlightLogError("unexpected export block magic")
    return ExportBegin(fields=fields)


def parse_end_line(line: str) -> ExportEnd:
    if not line.startswith("FLOG END"):
        raise FlightLogError(f"expected FLOG END, got {line!r}")
    return ExportEnd(fields=parse_key_values(line))


def read_until_begin(port: BinaryIO, log: callable | None = None) -> ExportBegin:
    while True:
        line = port.readline()
        if not line:
            raise FlightLogError("timeout waiting for FLOG BEGIN")
        text = line.decode("ascii", errors="replace").strip()
        error_line = find_status_line(text, "FLOG ERROR")
        if error_line is not None:
            log_text_line(log, error_line)
            raise FlightLogError(error_line)
        begin_line = find_status_line(text, "FLOG BEGIN")
        if begin_line is not None:
            log_text_line(log, begin_line)
            return parse_begin_line(begin_line)
        log_text_line(log, text)


def read_until_end(
    port: BinaryIO,
    received: int,
    expected_total: int,
    log: callable | None = None,
    require_counts: bool = True,
) -> ExportEnd:
    while True:
        line = port.readline()
        if not line:
            raise FlightLogError("timeout waiting for FLOG END")
        text = line.decode("ascii", errors="replace").strip()
        error_line = find_status_line(text, "FLOG ERROR")
        if error_line is not None:
            log_text_line(log, error_line)
            raise FlightLogError(error_line)
        end_line = find_status_line(text, "FLOG END")
        if end_line is not None:
            log_text_line(log, end_line)
            end = parse_end_line(end_line)
            if end.reason != "done":
                raise FlightLogError(f"export ended with reason={end.reason or 'unknown'}")
            if require_counts and (end.sent != received or end.total != expected_total):
                raise FlightLogError(
                    f"export end mismatch sent={end.sent} total={end.total} "
                    f"received={received} expected={expected_total}"
                )
            return end
        log_text_line(log, text)


def read_exact(port: BinaryIO, length: int) -> bytes:
    chunks: list[bytes] = []
    remaining = length
    while remaining > 0:
        chunk = port.read(remaining)
        if not chunk:
            raise FlightLogError(f"timeout reading {remaining} bytes")
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def read_export_block(port: BinaryIO) -> ExportBlock:
    header_bytes = read_exact(port, EXPORT_HEADER.size)
    if header_bytes.startswith(b"FLOG END"):
        raise FlightLogError(header_bytes.decode("ascii", errors="replace").strip())
    magic, version, header_size, seq, offset, length, flags, payload_crc = EXPORT_HEADER.unpack(header_bytes)
    if magic != EXPORT_BLOCK_MAGIC:
        raise FlightLogError(f"bad block magic 0x{magic:08X}")
    if version != 1 or header_size != EXPORT_HEADER.size:
        raise FlightLogError("unsupported export block version/header")
    if length > EXPORT_PAYLOAD_MAX:
        raise FlightLogError(f"bad block length {length}")
    payload = read_exact(port, length)
    actual_crc = crc32(payload)
    if actual_crc != payload_crc:
        raise FlightLogError(f"crc mismatch at block {seq}: 0x{actual_crc:08X} != 0x{payload_crc:08X}")
    return ExportBlock(seq=seq, offset=offset, payload=payload, flags=flags, crc32=payload_crc)


def parse_export_block_line(line: str) -> ExportBlock:
    if not line.startswith("FLOG BLK"):
        raise FlightLogError(f"expected FLOG BLK, got {line!r}")
    fields = parse_key_values(line)
    try:
        seq = int(fields["seq"], 0)
        offset = int(fields["offset"], 0)
        length = int(fields["len"], 0)
        flags = int(fields["flags"], 0)
        payload_crc = int(fields["crc"], 16)
        payload = xor_whiten(bytes.fromhex(fields["data"]), offset)
    except (KeyError, ValueError) as exc:
        raise FlightLogError(f"bad FLOG BLK line: {line!r}") from exc
    if length > EXPORT_PAYLOAD_MAX:
        raise FlightLogError(f"bad FLOG BLK length {length}")
    if len(payload) != length:
        raise FlightLogError(f"FLOG BLK length mismatch seq={seq}")
    actual_crc = crc32(payload)
    if actual_crc != payload_crc:
        raise FlightLogError(
            f"crc mismatch at block {seq}: 0x{actual_crc:08X} != 0x{payload_crc:08X}"
        )
    return ExportBlock(seq=seq, offset=offset, payload=payload, flags=flags, crc32=payload_crc)


def read_export_item_resync(
    port: BinaryIO,
    errors: list[str],
    log: callable | None = None,
) -> ExportBlock | ExportEnd:
    line = bytearray()
    magic_window = bytearray()

    while True:
        byte = port.read(1)
        if not byte:
            raise FlightLogError("timeout waiting for export block")

        line.extend(byte)
        magic_window.extend(byte)
        if len(magic_window) > len(EXPORT_BLOCK_MAGIC_BYTES):
            del magic_window[0 : len(magic_window) - len(EXPORT_BLOCK_MAGIC_BYTES)]

        if bytes(magic_window) == EXPORT_BLOCK_MAGIC_BYTES:
            header_tail = read_exact(port, EXPORT_HEADER.size - len(EXPORT_BLOCK_MAGIC_BYTES))
            header_bytes = EXPORT_BLOCK_MAGIC_BYTES + header_tail
            magic, version, header_size, seq, offset, length, flags, payload_crc = EXPORT_HEADER.unpack(header_bytes)
            line.clear()
            magic_window.clear()

            if magic != EXPORT_BLOCK_MAGIC:
                errors.append(f"resync skipped bad block magic 0x{magic:08X}")
                continue
            if version != 1 or header_size != EXPORT_HEADER.size:
                errors.append(
                    f"resync skipped bad block header seq={seq} version={version} header_size={header_size}"
                )
                continue
            if length > EXPORT_PAYLOAD_MAX:
                errors.append(f"resync skipped bad block length seq={seq} length={length}")
                continue

            payload = read_exact(port, length)
            actual_crc = crc32(payload)
            if actual_crc != payload_crc:
                errors.append(
                    f"crc mismatch at block {seq} offset={offset}: "
                    f"0x{actual_crc:08X} != 0x{payload_crc:08X}"
                )
                continue

            return ExportBlock(seq=seq, offset=offset, payload=payload, flags=flags, crc32=payload_crc)

        if byte == b"\n":
            text = line.decode("ascii", errors="replace").strip()
            line.clear()
            magic_window.clear()
            if not text:
                continue
            error_line = find_status_line(text, "FLOG ERROR")
            if error_line is not None:
                log_text_line(log, error_line)
                raise FlightLogError(error_line)
            end_line = find_status_line(text, "FLOG END")
            if end_line is not None:
                log_text_line(log, end_line)
                return parse_end_line(end_line)
            block_line = find_status_line(text, "FLOG BLK")
            if block_line is not None:
                try:
                    return parse_export_block_line(block_line)
                except FlightLogError as exc:
                    errors.append(str(exc))
                    continue
            begin_line = find_status_line(text, "FLOG BEGIN")
            if begin_line is not None:
                errors.append("skipped duplicate FLOG BEGIN while waiting for binary block")
                log_text_line(log, begin_line)
                continue
            log_text_line(log, text)
            continue

        if len(line) > EXPORT_TEXT_LINE_MAX:
            errors.append(f"discarded {len(line)} bytes while resyncing export stream")
            line.clear()


def attitude_frame_from_provenance(provenance: dict[str, object]) -> str:
    """把扇区溯源翻成姿态口径标识。

    没有溯源的扇区一律判为 legacy_frd：v7 及更早既可能录于 V0 候选生效前
    （FRD 姿态），也可能录于其后，帧内无从分辨。按 spec §10「历史数据永不
    重释义」，宁可保持旧口径，也不能猜成 FLU。
    """
    if not provenance.get("frame_provenance_valid"):
        return "unknown_legacy_frd"
    code = int(provenance.get("frame_orientation_code", FRAME_ORIENTATION_LEGACY))
    if code < FRAME_ORIENTATION_COUNT:
        return "canonical_flu"
    return "legacy_frd"


def parse_sector_header(data: bytes, offset: int) -> dict[str, object] | None:
    header = data[offset : offset + SECTOR_HEADER_SIZE]
    if len(header) < SECTOR_HEADER_SIZE:
        return None
    prefix = list(SECTOR_HEADER_PREFIX.unpack_from(header, 0))
    magic = prefix[0]
    if magic in (0xFFFFFFFF, 0x00000000):
        return None
    if magic != SECTOR_MAGIC:
        return None
    saved_crc = prefix[-1]
    check = bytearray(header)
    check[52:56] = b"\x00\x00\x00\x00"
    if crc32(bytes(check)) != saved_crc:
        return None
    params_size = int(prefix[12])
    if params_size == PARAMS_STRUCT.size:
        param_names = PARAM_NAMES
        params_struct = PARAMS_STRUCT
    elif params_size == V9_PARAMS_STRUCT.size:
        param_names = V9_PARAM_NAMES
        params_struct = V9_PARAMS_STRUCT
    elif params_size == V8_PARAMS_STRUCT.size:
        param_names = V8_PARAM_NAMES
        params_struct = V8_PARAMS_STRUCT
    elif params_size == V6_PARAMS_STRUCT.size:
        param_names = V6_PARAM_NAMES
        params_struct = V6_PARAMS_STRUCT
    elif params_size == LEGACY_PARAMS_STRUCT.size:
        param_names = LEGACY_PARAM_NAMES
        params_struct = LEGACY_PARAMS_STRUCT
    else:
        return None
    params_values = params_struct.unpack_from(header, SECTOR_HEADER_PREFIX.size)
    # 缺失域装默认：只有 v8 起才有溯源块，更早的版本一律报「无溯源」，
    # 不去读那段保留区，免得把任意字节当成合法方向码。
    provenance = dict.fromkeys(FRAME_PROVENANCE_NAMES, 0)
    if int(prefix[1]) >= 8:
        offset_provenance = SECTOR_HEADER_PREFIX.size + params_struct.size
        provenance = dict(
            zip(
                FRAME_PROVENANCE_NAMES,
                FRAME_PROVENANCE_STRUCT.unpack_from(header, offset_provenance),
            )
        )
    # Only the tagged V10/V11 extension defines ESC protocol; never infer it from the host.
    esc_protocol = "legacy_unspecified"
    if int(prefix[1]) in (10, 11) and header[252:254] == b"\xd5\x01" and header[255] == 0:
        # 3 = DSHOT300_BIDIR（2026-09-21 起）。两档 DShot 分开编码：线上是不同的
        # 协议（极性取反、校验取反），事后复盘油门与转速时分不清档位，就分不清
        # "电调没执行"和"帧格式对不上"。旧值 1/2 含义一字未动，老日志照旧可读。
        esc_protocol = {1: "PWM", 2: "DSHOT300",
                        3: "DSHOT300_BIDIR"}.get(header[254], "unknown")
    return {
        **provenance,
        "esc_protocol": esc_protocol,
        # 两档 DShot 的电机命令都是**等效微秒**，不是真微秒脉宽。双向档漏进 else
        # 分支会让日志把单位标成 "us"——一个说谎的单位比没有单位更难查。
        "motor_command_unit": ("pwm_equivalent_us"
                               if esc_protocol in ("DSHOT300", "DSHOT300_BIDIR")
                               else "us"),
        "attitude_frame": attitude_frame_from_provenance(provenance),
        "magic": magic,
        "version": prefix[1],
        "header_size": prefix[2],
        "sector_size": prefix[3],
        "record_size": prefix[4],
        "session_id": prefix[5],
        "sector_seq": prefix[6],
        "sector_index": prefix[7],
        "log_rate_hz": prefix[8],
        "region_start": prefix[9],
        "region_end_excl": prefix[10],
        "created_us": prefix[11],
        "params_size": prefix[12],
        "header_crc32": saved_crc,
        "params": dict(zip(param_names, params_values)),
    }


def _parse_record_v3(record_bytes: bytes) -> dict[str, object] | None:
    """Parse a 320-byte (V3) record into a row dict in the historical column
    order (no servo-feedback / ident_att / v6 / v7 sections)."""
    values = V3_RECORD_STRUCT.unpack(record_bytes)
    if values[0] != RECORD_MAGIC or values[2] != len(record_bytes):
        return None
    saved_crc = values[-1]
    check = bytearray(record_bytes)
    check[-4:] = b"\x00\x00\x00\x00"
    if crc32(bytes(check)) != saved_crc:
        return None

    i = 0
    row: dict[str, object] = {}
    for name in ("magic", "version", "size", "sequence", "dropped_records", "timestamp_us", "tick_ms", "imu_sequence"):
        row[name] = values[i]
        i += 1
    for name in ("raw_temperature", "raw_accel_x", "raw_accel_y", "raw_accel_z", "raw_gyro_x", "raw_gyro_y", "raw_gyro_z"):
        row[name] = values[i]
        i += 1
    for name in ("temperature_c", "accel_x_g", "accel_y_g", "accel_z_g", "gyro_x_dps", "gyro_y_dps", "gyro_z_dps"):
        row[name] = values[i]
        i += 1
    for name in ("roll_deg", "pitch_deg", "yaw_deg"):
        row[name] = values[i]
        i += 1
    for ch in range(8):
        row[f"rc_ch{ch + 1}_us"] = values[i]
        i += 1
    for name in ("throttle_us", "servo_alpha_us", "servo_beta_us", "motor_upper_us", "motor_lower_us"):
        row[name] = values[i]
        i += 1
    for name in (
        "rc_armed",
        "rc_link_ok",
        "throttle_over_20",
        "imu_valid",
        "motor_output_reason",
        "rc_link_seen",
        "arm_switch_high",
        "arm_throttle_low",
        "arm_switch_seen_low",
        "arm_switch_prev_high",
        "imu_fault_latched",
        "imu_fault_reason",
    ):
        row[name] = values[i]
        i += 1
    row["motor_output_reason_name"] = MOTOR_REASON_NAMES.get(
        int(row["motor_output_reason"]), "unknown"
    )
    for prefix, count in (
        ("acc_nav_m_s2", 3),
        ("vel_est_m_s", 3),
        ("vel_ref_m_s", 2),
        ("vel_err_m_s", 2),
        ("vel_pid_out_m_s2", 2),
        ("vel_pid_p_m_s2", 2),
        ("vel_pid_i_m_s2", 2),
        ("vel_pid_d_m_s2", 2),
    ):
        for axis in range(count):
            row[f"{prefix}_{axis}"] = values[i]
            i += 1
    row["vel_loop_active"] = values[i]
    i += 1
    for prefix, count in (
        ("ctrl_pos_p_m_s2", 3),
        ("ctrl_vel_d_m_s2", 3),
        ("ctrl_accel_out_m_s2", 3),
        ("ctrl_force_cmd_n", 3),
        ("ctrl_tilt_ff_rad", 2),
        ("ctrl_tilt_angle_p_rad", 2),
        ("ctrl_tilt_rate_d_rad", 2),
        ("ctrl_tilt_out_rad", 2),
    ):
        for axis in range(count):
            row[f"{prefix}_{axis}"] = values[i]
            i += 1
    for name in (
        "ctrl_yaw_angle_p_rad_s",
        "ctrl_yaw_rate_d_rad_s",
        "ctrl_yaw_torque_cmd",
        "ctrl_total_force_n",
    ):
        row[name] = values[i]
        i += 1
    for prefix, count in (
        ("ctrl_motor_thrust_cmd_n", 2),
        ("ctrl_motor_cmd_us", 2),
    ):
        for axis in range(count):
            row[f"{prefix}_{axis}"] = values[i]
            i += 1
    row["z_ref_m"] = values[i]
    i += 1
    row["record_crc32"] = saved_crc
    return row


def parse_record(record_bytes: bytes) -> dict[str, object] | None:
    if len(record_bytes) == V3_RECORD_SIZE:
        row = _parse_record_v3(record_bytes)
        if row is not None:
            row.update(dict.fromkeys(DSHOT_NAMES))
        return row
    has_v6_diagnostics = False
    has_v7_z_integral = False
    has_v9_layout = False
    has_v10_layout = False
    has_v11_layout = False
    if len(record_bytes) == RECORD_SIZE:
        record_struct = RECORD_STRUCT
        has_v11_layout = True
        has_servo_feedback = True
        has_ident_att = True
        has_v6_diagnostics = True
        has_v7_z_integral = True
        has_v9_layout = True
        has_v10_layout = True
    elif len(record_bytes) == V10_RECORD_SIZE:
        record_struct = V10_RECORD_STRUCT
        has_servo_feedback = True
        has_ident_att = True
        has_v6_diagnostics = True
        has_v7_z_integral = True
        has_v9_layout = True
        has_v10_layout = True
    elif len(record_bytes) == V9_RECORD_SIZE:
        record_struct = V9_RECORD_STRUCT
        has_servo_feedback = True
        has_ident_att = True
        has_v6_diagnostics = True
        has_v7_z_integral = True
        has_v9_layout = True
    elif len(record_bytes) == V8_RECORD_SIZE:
        record_struct = V8_RECORD_STRUCT
        has_servo_feedback = True
        has_ident_att = True
        has_v6_diagnostics = True
        has_v7_z_integral = True
    elif len(record_bytes) == V6_RECORD_SIZE:
        record_struct = V6_RECORD_STRUCT
        has_servo_feedback = True
        has_ident_att = True
        has_v6_diagnostics = True
    elif len(record_bytes) == V5_RECORD_SIZE:
        record_struct = V5_RECORD_STRUCT
        has_servo_feedback = True
        has_ident_att = True
    elif len(record_bytes) == V4_RECORD_SIZE:
        record_struct = V4_RECORD_STRUCT
        has_servo_feedback = True
        has_ident_att = False
    elif len(record_bytes) == LEGACY_RECORD_SIZE:
        record_struct = LEGACY_RECORD_STRUCT
        has_servo_feedback = False
        has_ident_att = False
    else:
        return None
    values = record_struct.unpack(record_bytes)
    if values[0] != RECORD_MAGIC or values[2] != len(record_bytes):
        return None
    if (has_v11_layout and values[1] != 11) or (has_v10_layout and not has_v11_layout and values[1] != 10):
        return None
    saved_crc = values[-1]
    check = bytearray(record_bytes)
    check[-4:] = b"\x00\x00\x00\x00"
    if crc32(bytes(check)) != saved_crc:
        return None

    i = 0
    row: dict[str, object] = {}
    for name in ("magic", "version", "size", "sequence", "dropped_records", "timestamp_us", "tick_ms", "imu_sequence"):
        row[name] = values[i]
        i += 1
    for name in ("raw_temperature", "raw_accel_x", "raw_accel_y", "raw_accel_z", "raw_gyro_x", "raw_gyro_y", "raw_gyro_z"):
        row[name] = values[i]
        i += 1
    for name in ("temperature_c", "accel_x_g", "accel_y_g", "accel_z_g", "gyro_x_dps", "gyro_y_dps", "gyro_z_dps"):
        row[name] = values[i]
        i += 1
    for name in ("roll_deg", "pitch_deg", "yaw_deg"):
        row[name] = values[i]
        i += 1
    for ch in range(8):
        row[f"rc_ch{ch + 1}_us"] = values[i]
        i += 1
    for name in ("throttle_us", "servo_alpha_us", "servo_beta_us", "motor_upper_us", "motor_lower_us"):
        row[name] = values[i]
        i += 1
    feedback_names = (
        "servo_alpha_feedback_us",
        "servo_beta_feedback_us",
        "servo_alpha_feedback_age_ms",
        "servo_beta_feedback_age_ms",
        "servo_alpha_feedback_sequence",
        "servo_beta_feedback_sequence",
    )
    if has_servo_feedback:
        for name in feedback_names:
            row[name] = values[i]
            i += 1
        row["servo_feedback_valid_mask"] = values[i]
        i += 4  # valid mask plus three reserved bytes
    else:
        for name in feedback_names:
            row[name] = 0
        row["servo_feedback_valid_mask"] = 0
    for axis, valid_bit in (("alpha", 0x01), ("beta", 0x02)):
        position_us = int(row[f"servo_{axis}_feedback_us"])
        if ((int(row["servo_feedback_valid_mask"]) & valid_bit) != 0 and
                500 <= position_us <= 2500):
            row[f"servo_{axis}_feedback_deg"] = (position_us - 500) * 0.09
            row[f"servo_{axis}_feedback_tilt_deg"] = (position_us - 1500) * 0.09
        else:
            row[f"servo_{axis}_feedback_deg"] = None
            row[f"servo_{axis}_feedback_tilt_deg"] = None
    for name in (
        "rc_armed",
        "rc_link_ok",
        "throttle_over_20",
        "imu_valid",
        "motor_output_reason",
        "rc_link_seen",
        "arm_switch_high",
        "arm_throttle_low",
        "arm_switch_seen_low",
        "arm_switch_prev_high",
        "imu_fault_latched",
        "imu_fault_reason",
    ):
        row[name] = values[i]
        i += 1
    row["motor_output_reason_name"] = MOTOR_REASON_NAMES.get(
        int(row["motor_output_reason"]), "unknown"
    )
    nav_axes = 2 if has_v9_layout else 3
    for prefix, count in (
        ("acc_nav_m_s2", nav_axes),
        ("vel_est_m_s", nav_axes),
        ("vel_ref_m_s", 2),
        ("vel_err_m_s", 2),
        ("vel_pid_out_m_s2", 2),
        ("vel_pid_p_m_s2", 2),
        ("vel_pid_i_m_s2", 2),
        ("vel_pid_d_m_s2", 2),
    ):
        for axis in range(count):
            row[f"{prefix}_{axis}"] = values[i]
            i += 1
    row["vel_loop_active"] = values[i]
    i += 1
    for prefix, count in (
        ("ctrl_pos_p_m_s2", 3),
    ):
        for axis in range(count):
            row[f"{prefix}_{axis}"] = values[i]
            i += 1
    if has_v7_z_integral:
        row["ctrl_pos_z_i_m_s2"] = values[i]
        i += 1
    else:
        row["ctrl_pos_z_i_m_s2"] = 0.0
    for prefix, count in (
        ("ctrl_vel_d_m_s2", 3),
        ("ctrl_accel_out_m_s2", 3),
        ("ctrl_force_cmd_n", 3),
        ("ctrl_target_attitude_rp_rad", 2),
        ("ctrl_tilt_angle_p_rad", 2),
        ("ctrl_tilt_rate_d_rad", 2),
        ("ctrl_tilt_out_rad", 2),
    ):
        for axis in range(count):
            row[f"{prefix}_{axis}"] = values[i]
            i += 1
    for name in (
        "ctrl_yaw_angle_p_rad_s",
        "ctrl_yaw_rate_d_rad_s",
        "ctrl_yaw_torque_cmd",
        "ctrl_total_force_n",
    ):
        row[name] = values[i]
        i += 1
    for prefix, count in (
        ("ctrl_motor_thrust_cmd_n", 2),
        ("ctrl_motor_cmd_us", 2),
        ("ctrl_velocity_integral_m", 0 if has_v9_layout else 2),
        ("ctrl_desired_attitude_rpy_rad", 3),
        ("ctrl_attitude_error", 3),
        ("ctrl_rate_error_rad_s", 3),
        ("ctrl_moment_cmd_n_m", 3),
    ):
        for axis in range(count):
            row[f"{prefix}_{axis}"] = values[i]
            i += 1
    v10_vectors = (
        "ctrl_moment_achieved_n_m",
        "ctrl_position_sp_m",
        "ctrl_position_m",
        "ctrl_position_error_m",
        "ctrl_velocity_ff_m_s",
        "ctrl_velocity_sp_m_s",
        "ctrl_velocity_m_s",
        "ctrl_velocity_error_m_s",
        "ctrl_velocity_p_m_s2",
        "ctrl_velocity_i_m_s2",
        "ctrl_velocity_d_m_s2",
        "ctrl_velocity_ff_m_s2",
        "ctrl_accel_unsat_m_s2",
        "ctrl_omega_ff_rad_s",
        "ctrl_omega_sp_rad_s",
        "ctrl_omega_rad_s",
        "ctrl_rate_limit_rad_s",
        "ctrl_rate_p_n_m",
        "ctrl_rate_i_n_m",
        "ctrl_rate_d_n_m",
        "ctrl_rate_ff_n_m",
    )
    if has_v10_layout:
        for prefix in v10_vectors:
            for axis in range(3):
                row[f"{prefix}_{axis}"] = values[i]
                i += 1
        for prefix in ("ctrl_saturation_positive", "ctrl_saturation_negative"):
            for axis in range(3):
                row[f"{prefix}_{axis}"] = values[i]
                i += 1
        for name in (
            "ctrl_thrust_saturated",
            "ctrl_tilt_saturated",
            "ctrl_yaw_differential_saturated",
        ):
            row[name] = values[i]
            i += 1
    else:
        for prefix in v10_vectors:
            for axis in range(3):
                row[f"{prefix}_{axis}"] = 0.0
        for prefix in ("ctrl_saturation_positive", "ctrl_saturation_negative"):
            for axis in range(3):
                row[f"{prefix}_{axis}"] = 0
        row["ctrl_thrust_saturated"] = 0
        row["ctrl_tilt_saturated"] = 0
        row["ctrl_yaw_differential_saturated"] = 0
    for name in (
        "ctrl_horizontal_command_scale",
        "ctrl_moment_utilization",
        "ctrl_thrust_utilization",
    ):
        row[name] = values[i]
        i += 1
    row["ctrl_protection_flags"] = values[i]
    i += 1
    row["z_ref_m"] = values[i]
    i += 1
    if has_ident_att:
        for name in (
            "ident_att_active",
            "ident_att_axis",
            "ident_att_mode",
            "ident_att_reserved",
            "ident_att_seq",
            "ident_att_signal_rad",
            "ident_att_signal_m_s2",
            "ident_att_elapsed_ms",
        ):
            row[name] = values[i]
            i += 1
    else:
        row["ident_att_active"] = 0
        row["ident_att_axis"] = 0
        row["ident_att_mode"] = 0
        row["ident_att_reserved"] = 0
        row["ident_att_seq"] = 0
        row["ident_att_signal_rad"] = 0.0
        row["ident_att_signal_m_s2"] = 0.0
        row["ident_att_elapsed_ms"] = 0
    if has_v6_diagnostics:
        for name in ("servo_alpha_sent_us", "servo_beta_sent_us"):
            row[name] = values[i]
            i += 1
        for name in ("flow_raw_x", "flow_raw_y"):
            row[name] = values[i]
            i += 1
        for name in ("flow_sample_age_ms", "flow_height_age_ms"):
            row[name] = values[i]
            i += 1
        for name in (
            "flow_quality",
            "flow_valid",
            "flow_velocity_valid",
            "flow_height_valid",
        ):
            row[name] = values[i]
            i += 1
        for name in ("flow_height_raw_m", "flow_height_m"):
            row[name] = values[i]
            i += 1
        for prefix in (
            "flow_sensor_velocity_m_s",
            "flow_optical_rot_comp_m_s",
            "flow_offset_rot_comp_m_s",
            "flow_corrected_velocity_m_s",
        ):
            for axis in range(2):
                row[f"{prefix}_{axis}"] = values[i]
                i += 1
        for name in (
            "servo_move_attempt_count",
            "servo_move_sent_count",
            "servo_move_busy_count",
            "servo_move_error_count",
            "servo_feedback_request_count",
            "servo_feedback_response_count",
            "servo_feedback_timeout_count",
            "servo_feedback_parse_error_count",
            "servo_feedback_uart_error_count",
            "servo_feedback_busy_count",
        ):
            row[name] = values[i]
            i += 1
    else:
        row["servo_alpha_sent_us"] = 0
        row["servo_beta_sent_us"] = 0
        row["flow_raw_x"] = 0
        row["flow_raw_y"] = 0
        row["flow_sample_age_ms"] = 0
        row["flow_height_age_ms"] = 0
        row["flow_quality"] = 0
        row["flow_valid"] = 0
        row["flow_velocity_valid"] = 0
        row["flow_height_valid"] = 0
        row["flow_height_raw_m"] = None
        row["flow_height_m"] = None
        for prefix in (
            "flow_sensor_velocity_m_s",
            "flow_optical_rot_comp_m_s",
            "flow_offset_rot_comp_m_s",
            "flow_corrected_velocity_m_s",
        ):
            for axis in range(2):
                row[f"{prefix}_{axis}"] = None
        for name in (
            "servo_move_attempt_count",
            "servo_move_sent_count",
            "servo_move_busy_count",
            "servo_move_error_count",
            "servo_feedback_request_count",
            "servo_feedback_response_count",
            "servo_feedback_timeout_count",
            "servo_feedback_parse_error_count",
            "servo_feedback_uart_error_count",
            "servo_feedback_busy_count",
        ):
            row[name] = 0
    row.update(dict.fromkeys(DSHOT_NAMES))
    if has_v11_layout:
        dshot = dict(zip(DSHOT_NAMES, values[i:i + len(DSHOT_NAMES)]))
        if (dshot["dshot_present"] not in (0, 1) or dshot["dshot_enabled_mask"] > 3 or
                dshot["dshot_busy"] not in (0, 1) or dshot["dshot_fault"] not in (0, 1)):
            return None
        row["dshot_present"] = dshot["dshot_present"]
        if dshot["dshot_present"]:
            if any(code != 0 and not 48 <= code <= 2047 for code in
                   (dshot["dshot_code_ch1"], dshot["dshot_code_ch2"])):
                return None
            row.update(dshot)
    row["record_crc32"] = saved_crc
    return row


def parse_flash_image(data: bytes) -> tuple[list[dict[str, object]], list[dict[str, object]], list[str]]:
    sectors: list[dict[str, object]] = []
    records: list[dict[str, object]] = []
    errors: list[str] = []
    for offset in range(0, len(data), SECTOR_SIZE):
        sector = parse_sector_header(data, offset)
        if sector is None:
            continue
        sector["image_offset"] = offset
        sectors.append(sector)
        record_size = int(sector.get("record_size", RECORD_SIZE))
        pos = offset + SECTOR_HEADER_SIZE
        end = offset + SECTOR_SIZE
        while pos + record_size <= end:
            chunk = data[pos : pos + record_size]
            if chunk == b"\xFF" * len(chunk) or chunk == b"\x00" * len(chunk):
                pos += record_size
                continue
            if record_size not in (
                RECORD_SIZE,
                V10_RECORD_SIZE,
                V9_RECORD_SIZE,
                V8_RECORD_SIZE,
                V6_RECORD_SIZE,
                V5_RECORD_SIZE,
                V4_RECORD_SIZE,
                V3_RECORD_SIZE,
                LEGACY_RECORD_SIZE,
            ):
                errors.append(f"unsupported record size {record_size} at sector offset {offset}")
                break
            record = parse_record(chunk)
            if record is not None:
                record["sector_seq"] = sector["sector_seq"]
                record["sector_index"] = sector["sector_index"]
                # 让溯源随记录进 CSV，回放才能逐文件选口径而不必回头读扇区。
                record["attitude_frame"] = sector["attitude_frame"]
                record["frame_orientation_code"] = sector["frame_orientation_code"]
                record["frame_contract_version"] = sector["frame_contract_version"]
                record["firmware_crc32"] = sector["firmware_crc32"]
                record["calibration_generation"] = sector["calibration_generation"]
                record["esc_protocol"] = sector["esc_protocol"]
                record["motor_command_unit"] = sector["motor_command_unit"]
                records.append(record)
            elif sector["version"] == 11:
                errors.append(f"invalid V11 record at image offset {pos}")
            pos += record_size
    return sectors, records, errors


def mark_range(coverage: bytearray, start: int, length: int) -> int:
    end = min(start + length, len(coverage))
    newly_seen = 0
    for index in range(start, end):
        if coverage[index] == 0:
            coverage[index] = 1
            newly_seen += 1
    return newly_seen


def missing_ranges(coverage: bytearray) -> list[dict[str, int]]:
    ranges: list[dict[str, int]] = []
    index = 0
    while index < len(coverage):
        if coverage[index] != 0:
            index += 1
            continue
        start = index
        while index < len(coverage) and coverage[index] == 0:
            index += 1
        ranges.append({"offset": start, "length": index - start})
    return ranges


def write_csv(path: Path, records: list[dict[str, object]]) -> None:
    if records:
        fieldnames = list(records[0].keys())
    else:
        fieldnames = ["sequence", "timestamp_us"]
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(records)


def receive_dump(
    port: BinaryIO,
    output_dir: Path,
    log: callable | None = None,
    progress: callable | None = None,
    should_cancel: callable | None = None,
) -> ReceiveResult:
    output_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    bin_path = output_dir / f"flightlog_{stamp}.bin"
    csv_path = output_dir / f"flightlog_{stamp}.csv"
    meta_path = output_dir / f"flightlog_{stamp}_meta.json"

    port.write(b"Sensor_Data:0\r\n")
    time.sleep(0.05)
    port.write(b"FLOG CANCEL\r\n")
    time.sleep(0.15)
    reset_input_buffer(port)
    port.write(b"Sensor_Data:0\r\n")
    time.sleep(0.05)
    port.write(b"FLOG DUMP\r\n")

    begin = read_until_begin(port, log)
    total = begin.total
    image = bytearray(b"\xFF" * total)
    coverage = bytearray(total)
    good_bytes = 0
    blocks = 0
    last_seq: int | None = None
    end: ExportEnd | None = None
    errors: list[str] = []

    try:
        while end is None:
            if should_cancel is not None and should_cancel():
                port.write(b"FLOG CANCEL\r\n")
                raise FlightLogError("cancelled")

            item = read_export_item_resync(port, errors, log)
            if isinstance(item, ExportEnd):
                end = item
                break

            block = item
            if last_seq is not None and block.seq != (last_seq + 1):
                errors.append(f"block seq gap: got {block.seq}, expected {last_seq + 1}")
            last_seq = block.seq

            block_end = block.offset + len(block.payload)
            if block.offset >= total or block_end > total:
                errors.append(
                    f"skipped out-of-range block seq={block.seq} "
                    f"offset={block.offset} length={len(block.payload)} total={total}"
                )
                continue

            image[block.offset:block_end] = block.payload
            good_bytes += mark_range(coverage, block.offset, len(block.payload))
            blocks += 1
            if progress:
                progress(good_bytes, total)
            if block.flags & EXPORT_BLOCK_LAST:
                end = read_until_end(port, good_bytes, total, log, require_counts=False)
                break
    except FlightLogError as exc:
        errors.append(str(exc))
        if blocks == 0:
            raise

    bin_path.write_bytes(bytes(image))
    missing = missing_ranges(coverage)
    sectors, records, parse_errors = parse_flash_image(image)
    errors.extend(parse_errors)
    write_csv(csv_path, records)
    complete = (end is not None) and (good_bytes == total) and (len(missing) == 0)
    meta = {
        "begin": begin.fields,
        "end": end.fields if end is not None else None,
        "complete": complete,
        "total_bytes": total,
        "good_bytes": good_bytes,
        "missing_bytes": total - good_bytes,
        "missing_ranges": missing,
        "blocks": blocks,
        "sectors": sectors,
        "record_count": len(records),
        "record_size": RECORD_SIZE,
        "sector_size": SECTOR_SIZE,
        "baud": DEFAULT_BAUD,
        "errors": errors,
    }
    meta_path.write_text(json.dumps(meta, ensure_ascii=False, indent=2), encoding="utf-8")

    if log:
        if complete:
            log(f"saved {bin_path.name}, {csv_path.name}, {meta_path.name}")
        else:
            log(
                f"saved partial {bin_path.name}, {csv_path.name}, {meta_path.name} "
                f"good={good_bytes}/{total}"
            )
    return ReceiveResult(
        bin_path,
        csv_path,
        meta_path,
        total,
        blocks,
        len(records),
        len(sectors),
        errors,
        good_bytes=good_bytes,
        missing_bytes=total - good_bytes,
        complete=complete,
    )


class FlightLogGui:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        if isinstance(root, (tk.Tk, tk.Toplevel)):
            self.root.title("FlightLog USART1 Receiver")
        self.events: queue.Queue[tuple[str, object]] = queue.Queue()
        self.worker: threading.Thread | None = None
        self.cancel_requested = False
        self.serial_port = None

        self.port_var = tk.StringVar()
        self.baud_var = tk.StringVar(value=str(DEFAULT_BAUD))
        self.dir_var = tk.StringVar(value=str(DEFAULT_OUT_DIR))
        self.progress_var = tk.DoubleVar(value=0.0)

        self._build()
        self._refresh_ports()
        self.poll_after_id = self.root.after(100, self._poll_events)

    def _build(self) -> None:
        frame = ttk.Frame(self.root, padding=10)
        frame.grid(row=0, column=0, sticky="nsew")
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)

        ttk.Label(frame, text="Serial").grid(row=0, column=0, sticky="w")
        self.port_combo = ttk.Combobox(frame, textvariable=self.port_var, width=18)
        self.port_combo.grid(row=0, column=1, sticky="ew", padx=4)
        ttk.Button(frame, text="Refresh", command=self._refresh_ports).grid(row=0, column=2, padx=4)

        ttk.Label(frame, text="Baud").grid(row=0, column=3, sticky="w")
        ttk.Entry(frame, textvariable=self.baud_var, width=10).grid(row=0, column=4, sticky="ew", padx=4)

        ttk.Label(frame, text="Save Dir").grid(row=1, column=0, sticky="w", pady=(8, 0))
        ttk.Entry(frame, textvariable=self.dir_var).grid(row=1, column=1, columnspan=3, sticky="ew", padx=4, pady=(8, 0))
        ttk.Button(frame, text="Browse", command=self._browse).grid(row=1, column=4, padx=4, pady=(8, 0))

        self.receive_btn = ttk.Button(frame, text="接收数据", command=self._start_receive)
        self.receive_btn.grid(row=2, column=0, columnspan=2, sticky="ew", pady=(10, 0))
        self.cancel_btn = ttk.Button(frame, text="取消", command=self._cancel, state="disabled")
        self.cancel_btn.grid(row=2, column=2, sticky="ew", padx=4, pady=(10, 0))

        self.progress = ttk.Progressbar(frame, variable=self.progress_var, maximum=100.0)
        self.progress.grid(row=3, column=0, columnspan=5, sticky="ew", pady=(10, 0))

        self.log_text = tk.Text(frame, height=14, width=80)
        self.log_text.grid(row=4, column=0, columnspan=5, sticky="nsew", pady=(10, 0))
        frame.columnconfigure(1, weight=1)
        frame.columnconfigure(3, weight=1)
        frame.rowconfigure(4, weight=1)

    def _refresh_ports(self) -> None:
        ports = []
        if list_ports is not None:
            ports = [p.device for p in list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def _browse(self) -> None:
        chosen = filedialog.askdirectory(initialdir=self.dir_var.get()) if filedialog else None
        if chosen:
            self.dir_var.set(chosen)

    def _append_log(self, text: str) -> None:
        self.log_text.insert("end", text + "\n")
        self.log_text.see("end")

    def _start_receive(self) -> None:
        if serial is None:
            messagebox.showerror("pyserial missing", "Install pyserial: python -m pip install pyserial")
            return
        port_name = self.port_var.get().strip()
        if not port_name:
            messagebox.showerror("Serial", "Select a serial port")
            return
        try:
            baud = int(self.baud_var.get())
        except ValueError:
            messagebox.showerror("Serial", f"Invalid baud rate: {self.baud_var.get()!r}")
            return
        self.cancel_requested = False
        self.receive_btn.configure(state="disabled")
        self.cancel_btn.configure(state="normal")
        self.progress_var.set(0.0)
        output_dir = Path(self.dir_var.get())
        self.worker = threading.Thread(target=self._worker, args=(port_name, baud, output_dir), daemon=True)
        self.worker.start()

    def _worker(self, port_name: str, baud: int, output_dir: Path) -> None:
        try:
            with serial.Serial(port_name, baudrate=baud, timeout=DEFAULT_TIMEOUT_S, write_timeout=2.0) as port:
                self.serial_port = port
                result = receive_dump(
                    port,
                    output_dir,
                    log=lambda msg: self.events.put(("log", msg)),
                    progress=lambda done, total: self.events.put(("progress", (done, total))),
                    should_cancel=lambda: self.cancel_requested,
                )
            self.events.put(("done", result))
        except Exception as exc:
            self.events.put(("error", str(exc)))
        finally:
            self.serial_port = None

    def _cancel(self) -> None:
        self.cancel_requested = True
        try:
            if self.serial_port is not None:
                self.serial_port.write(b"FLOG CANCEL\r\n")
        except Exception as exc:
            self._append_log(f"cancel send failed: {exc}")

    def _poll_events(self) -> None:
        try:
            while True:
                kind, payload = self.events.get_nowait()
                if kind == "log":
                    self._append_log(str(payload))
                elif kind == "progress":
                    done, total = payload
                    self.progress_var.set(100.0 * float(done) / float(total or 1))
                elif kind == "done":
                    result = payload
                    self._append_log(f"complete={result.complete}: {result.records} records, missing={result.missing_bytes} bytes")
                    if hasattr(self, "on_result"):
                        self.on_result(result)
                    self.receive_btn.configure(state="normal")
                    self.cancel_btn.configure(state="disabled")
                elif kind == "error":
                    self._append_log(f"error: {payload}")
                    self.receive_btn.configure(state="normal")
                    self.cancel_btn.configure(state="disabled")
        except queue.Empty:
            pass
        self.poll_after_id = self.root.after(100, self._poll_events)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Receive FlightLog dumps from USART1 and convert them to bin/csv/json."
    )
    parser.add_argument("--port", default=None, help="serial port; if given, dump to files and exit (headless CLI)")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help=f"serial baud rate, default {DEFAULT_BAUD}")
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT_DIR, help="output directory for bin/csv/json")
    args = parser.parse_args()

    if args.port is None:
        if tk is None or ttk is None:
            raise SystemExit("tkinter is not available; pass --port for headless dump")
        root = tk.Tk()
        FlightLogGui(root)
        root.mainloop()
        return 0

    if serial is None:
        print("error: pyserial is required for serial dump: python -m pip install pyserial", file=sys.stderr)
        return 1
    output_dir = args.out_dir
    output_dir.mkdir(parents=True, exist_ok=True)
    try:
        with serial.Serial(args.port, baudrate=args.baud, timeout=DEFAULT_TIMEOUT_S, write_timeout=2.0) as port:
            result = receive_dump(port, output_dir)
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    print(f"done: {result.records} records, {result.total_bytes} bytes, {result.good_bytes} good, {result.missing_bytes} missing")
    for label, path in (("bin", result.bin_path), ("csv", result.csv_path), ("meta", result.meta_path)):
        print(f"{label}: {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

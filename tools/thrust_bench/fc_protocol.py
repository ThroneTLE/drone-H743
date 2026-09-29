"""Frozen v1 flight-controller wire contract for the coaxial thrust bench.

The ``$X`` envelope supplies framing and CRC.  This module decodes only the
fixed little-endian payload carried by function ``0x2235``.  Missing values are
represented by validity flags rather than plausible zeroes.
"""
from __future__ import annotations

import math
import struct

from .records import FcSnapshot

FUNCTION_ID = 0x2235
WIRE_VERSION = 1

# version, payload bytes, flags, nonce, FC ms, protocol/channels/max,
# equivalent-us commands, eRPM + ages, bus voltage/current + ages,
# per-ESC EDT current (whole ampere) + independent ages, then the MCU-wide
# last accepted ARM/SET request id.  The sequence survives window closure.
_FRAME = struct.Struct("<BBHII4B2H2I2IIi2I2H2II")
PAYLOAD_BYTES = _FRAME.size

FLAG_ERPM_1 = 1 << 0
FLAG_ERPM_2 = 1 << 1
FLAG_VOLTAGE = 1 << 2
FLAG_TOTAL_CURRENT = 1 << 3
FLAG_ESC_CURRENT_1 = 1 << 4
FLAG_ESC_CURRENT_2 = 1 << 5
FLAG_CURRENT_CALIBRATED = 1 << 6
FLAG_BENCH_ACTIVE = 1 << 7
FLAG_ARMED = 1 << 8
FLAG_NOT_SPINNING_1 = 1 << 9
FLAG_NOT_SPINNING_2 = 1 << 10
_KNOWN_FLAGS = (1 << 11) - 1


def _optional_int(value: int, flags: int, bit: int) -> int | None:
    return value if flags & bit else None


def _optional_scaled(value: int, flags: int, bit: int, scale: float) -> float | None:
    return value / scale if flags & bit else None


def decode_snapshot(payload: bytes) -> FcSnapshot:
    """Decode one exact v1 payload; reject version and length drift."""
    if len(payload) != PAYLOAD_BYTES:
        raise ValueError(f"thrust-bench payload length {len(payload)} != {PAYLOAD_BYTES}")
    values = _FRAME.unpack(payload)
    version, encoded_bytes, flags = values[:3]
    if version != WIRE_VERSION:
        raise ValueError(f"unsupported thrust-bench wire version {version}")
    if encoded_bytes != PAYLOAD_BYTES:
        raise ValueError(f"thrust-bench embedded length {encoded_bytes} != {PAYLOAD_BYTES}")
    if flags & ~_KNOWN_FLAGS:
        raise ValueError(f"unknown thrust-bench flags {flags & ~_KNOWN_FLAGS:#x}")
    (nonce, fc_time_ms, esc_protocol, upper_channel, lower_channel,
     bench_max_percent, command_1, command_2, erpm_1, erpm_2, age_1,
     age_2, voltage_mv, total_current_ma, voltage_age, current_age,
     esc_current_1, esc_current_2, esc_current_age_1,
     esc_current_age_2, last_request_id) = values[3:]
    if esc_protocol not in (0, 1, 2):
        raise ValueError(f"unknown ESC protocol {esc_protocol}")
    if (upper_channel, lower_channel) not in ((0, 0), (1, 2), (2, 1)):
        raise ValueError("upper/lower channel mapping must be absent or a permutation of 1,2")
    if not 0 <= bench_max_percent <= 100:
        raise ValueError("bench max percent is out of range")
    active = bool(flags & FLAG_BENCH_ACTIVE)
    armed = bool(flags & FLAG_ARMED)
    if active and armed:
        raise ValueError("bench and flight arm states are mutually exclusive")
    if active != (bench_max_percent != 0):
        raise ValueError("bench max percent does not match active state")
    for command in (command_1, command_2):
        if command != 0 and not 1100 <= command <= 1940:
            raise ValueError("ESC command must be disabled or in the 1100..1940 equivalent-us range")
    if active and (command_1 == 0 or command_2 == 0):
        raise ValueError("active bench must command a defined zero-or-throttle value to both ESCs")
    for valid_bit, stopped_bit, erpm, age in (
        (FLAG_ERPM_1, FLAG_NOT_SPINNING_1, erpm_1, age_1),
        (FLAG_ERPM_2, FLAG_NOT_SPINNING_2, erpm_2, age_2),
    ):
        valid = bool(flags & valid_bit)
        stopped = bool(flags & stopped_bit)
        if stopped and (not valid or erpm != 0):
            raise ValueError("not-spinning requires a valid zero-eRPM observation")
        if valid and not stopped and erpm == 0:
            raise ValueError("zero eRPM must carry the not-spinning flag")
        if valid and age > 100:
            raise ValueError("firmware marked stale eRPM as valid")
    for bit, age in ((FLAG_ESC_CURRENT_1, esc_current_age_1),
                     (FLAG_ESC_CURRENT_2, esc_current_age_2)):
        if flags & bit and age > 1000:
            raise ValueError("firmware marked stale EDT current as valid")
    if flags & FLAG_VOLTAGE and voltage_age > 250:
        raise ValueError("firmware marked stale voltage as valid")
    if flags & FLAG_TOTAL_CURRENT and current_age > 250:
        raise ValueError("firmware marked stale total current as valid")
    return FcSnapshot(
        nonce=nonce,
        fc_time_ms=fc_time_ms,
        esc_protocol=esc_protocol,
        upper_channel=upper_channel,
        lower_channel=lower_channel,
        command_us=(command_1, command_2),
        erpm=(_optional_int(erpm_1, flags, FLAG_ERPM_1),
              _optional_int(erpm_2, flags, FLAG_ERPM_2)),
        erpm_age_ms=(_optional_int(age_1, flags, FLAG_ERPM_1),
                     _optional_int(age_2, flags, FLAG_ERPM_2)),
        voltage_v=_optional_scaled(voltage_mv, flags, FLAG_VOLTAGE, 1000.0),
        total_current_a=_optional_scaled(total_current_ma, flags, FLAG_TOTAL_CURRENT, 1000.0),
        voltage_age_ms=_optional_int(voltage_age, flags, FLAG_VOLTAGE),
        current_age_ms=_optional_int(current_age, flags, FLAG_TOTAL_CURRENT),
        current_calibrated=bool(flags & FLAG_CURRENT_CALIBRATED),
        bench_active=active,
        bench_max_percent=bench_max_percent,
        armed=armed,
        esc_current_a=(_optional_int(esc_current_1, flags, FLAG_ESC_CURRENT_1),
                       _optional_int(esc_current_2, flags, FLAG_ESC_CURRENT_2)),
        esc_current_age_ms=(
            _optional_int(esc_current_age_1, flags, FLAG_ESC_CURRENT_1),
            _optional_int(esc_current_age_2, flags, FLAG_ESC_CURRENT_2)),
        not_spinning=(bool(flags & FLAG_NOT_SPINNING_1),
                      bool(flags & FLAG_NOT_SPINNING_2)),
        flags=flags,
        last_request_id=last_request_id,
    )


def snapshot_command(nonce: int) -> str:
    if not isinstance(nonce, int) or isinstance(nonce, bool) or not 0 <= nonce <= 0xFFFFFFFF:
        raise ValueError("nonce must be a uint32")
    return f"TBENCH? {nonce}"


def _request_id(value: int) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or not 1 <= value <= 0xFFFFFFFF:
        raise ValueError("request_id/token must be a nonzero uint32")
    return value


def arm_command(max_percent: int, *, request_id: int) -> str:
    if not isinstance(max_percent, int) or isinstance(max_percent, bool) or not 1 <= max_percent <= 100:
        raise ValueError("max_percent must be an integer in 1..100")
    return (f"TBENCH ARM confirm=bench max_pct={max_percent} "
            f"request_id={_request_id(request_id)}")


def _percent(value: float) -> str:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("percent must be numeric")
    value = float(value)
    if not math.isfinite(value) or not 0.0 <= value <= 100.0:
        raise ValueError("percent must be finite and in 0..100")
    return format(value, ".6g")


def set_command(upper_percent: float, lower_percent: float, *,
                request_id: int, window_token: int) -> str:
    return (f"TBENCH SET upper_pct={_percent(upper_percent)} "
            f"lower_pct={_percent(lower_percent)} request_id={_request_id(request_id)} "
            f"token={_request_id(window_token)}")


def stop_command() -> str:
    return "TBENCH STOP"


__all__ = [
    "FUNCTION_ID", "PAYLOAD_BYTES", "WIRE_VERSION", "arm_command",
    "decode_snapshot", "set_command", "snapshot_command", "stop_command",
]

"""Protocol identifiers and line-parsing helpers for the panel."""

from __future__ import annotations

import math
import re


PROTO_COMPAT_FALLBACK_DELAY_MS = 400
PROTO_PROBE_SETTLE_MS = 900
PROTO_HEADER = b"$X"
PROTO_DIR_TO_FC = ord("<")
PROTO_DIR_FROM_FC = ord(">")
# $X 的 len 字段是 u16，但线上永远不会有这么长的帧：固件 APP_PROTO_MAX_PAYLOAD
# 是 256，遥测流的 USB 高速档（R-T2）上限 1024。超过这个数的 len 一定是被打坏
# 的字节，不是"还没收全"，接收端必须据此重新找帧头而不是继续等。
PROTO_MAX_FRAME_PAYLOAD = 1024
PROTO_REQ_PING = 0x1000
PROTO_REQ_STATUS = 0x1001
PROTO_REQ_CONFIG = 0x1002
PROTO_REQ_PARAMS = 0x1003
PROTO_REQ_PID = 0x1004
PROTO_REQ_BARO = 0x1005
PROTO_REQ_BARO_STREAM = 0x1006
PROTO_REQ_FLASH = 0x1007
PROTO_REQ_IMU = 0x1008
PROTO_REQ_MODULES = 0x1009
PROTO_REQ_CAPS = 0x100A
PROTO_REQ_SAVE = 0x100B
PROTO_REQ_LOAD = 0x100C
PROTO_REQ_DEFAULTS = 0x100D
PROTO_REQ_PARAM_SET = 0x100E
PROTO_REQ_PID_SET = 0x100F
PROTO_REQ_SERVO_MOVE = 0x1010
PROTO_REQ_SERVO_MOVE_ALL = 0x1011
PROTO_REQ_SERVO_ID = 0x1012
PROTO_REQ_SERVO_SETID = 0x1013
PROTO_REQ_SERVO_MODE = 0x1014
PROTO_REQ_SERVO_ENABLE = 0x1015
PROTO_REQ_SERVO_ACTION = 0x1016
PROTO_REQ_SERVO_RAW = 0x1017
PROTO_REQ_WIFI = 0x1018
PROTO_REQ_GPS = 0x1019
PROTO_REQ_MAG = 0x101A
PROTO_REQ_RTOS = 0x101B
PROTO_REQ_AIRFRAME = 0x101C
PROTO_REQ_IDENT = 0x101D
PROTO_REQ_IMU_FRAME = 0x101E
PROTO_REQ_IMU_CAL = 0x1020
PROTO_REQ_ACCEPTANCE = 0x1021
PROTO_REQ_RC = 0x1022
PROTO_REQ_RCMAP = 0x1023
# SERVOTYPE has its own S7 request slot; SERVO_CAL remains at 0x1024.
PROTO_REQ_SERVOTYPE = 0x1025
PROTO_REQ_SERVO_CAL = 0x1024
PROTO_MSG_CMD_LINE = 0x2000
PROTO_MSG_TEXT_LINE = 0x2001
PROTO_MSG_CMD_RX = 0x2100
PROTO_MSG_CMD_ACK = 0x2101
PROTO_MSG_CMD_ERR = 0x2102
PROTO_MSG_CMD_OK = 0x2103
PROTO_MSG_PONG = 0x2200
PROTO_MSG_HW_FLASH = 0x2201
PROTO_MSG_HW_BARO = 0x2202
PROTO_MSG_HW_IMU = 0x2203
PROTO_MSG_STATUS_FLASH = 0x2204
PROTO_MSG_STATUS_BARO = 0x2205
PROTO_MSG_STATUS_IMU = 0x2206
PROTO_MSG_UART_STATS = 0x2207
PROTO_MSG_CONFIG_SUMMARY = 0x2208
PROTO_MSG_CONFIG_SERVO = 0x2209
PROTO_MSG_PARAM_RECORD = 0x220A
PROTO_MSG_PID_RECORD = 0x220B
PROTO_MSG_FLASH_RECORD = 0x220C
PROTO_MSG_BARO_STATE = 0x220D
PROTO_MSG_BARO_DIAG = 0x220E
PROTO_MSG_BARO_RAW = 0x220F
PROTO_MSG_BARO_STREAM = 0x2210
PROTO_MSG_IMU_STATE = 0x2211
PROTO_MSG_IMU_SCALED = 0x2212
PROTO_MSG_MODULES_SUMMARY = 0x2213
PROTO_MSG_CAPS_RECORD = 0x2214
PROTO_MSG_READY = 0x2215
PROTO_MSG_SAVE_RESULT = 0x2216
PROTO_MSG_LOAD_RESULT = 0x2217
PROTO_MSG_DEFAULTS_RESULT = 0x2218
PROTO_MSG_SERVO_RESULT = 0x2219
PROTO_MSG_WIFI_RECORD = 0x221A
PROTO_MSG_GPS_RECORD = 0x221B
PROTO_MSG_MAG_RECORD = 0x221C
PROTO_MSG_RTOS_RECORD = 0x221D
PROTO_MSG_FLASH_BENCH = 0x221E
PROTO_MSG_AIRFRAME_RECORD = 0x221F
PROTO_MSG_RC_LIVE = 0x2223
PROTO_MSG_RC_MAP = 0x2224
PROTO_MSG_SERVO_CAL = 0x2225
PROTO_MSG_SERVO_TYPE = 0x2226
# 遥测流 v2 的自描述掩码帧（S8 / R-T1-1）。payload 是二进制，不是 UTF-8 文本；
# 固件侧同名登记在 App/Inc/app_proto.h::APP_PROTO_MSG_TELEM_FRAME。
# 历史教训：0x1022/0x1023 曾被两端各自定义过一次，所以新号一律两端同一提交登记。
PROTO_MSG_TELEM_FRAME = 0x2230


def parse_kv(line: str) -> dict[str, str]:
    result: dict[str, str] = {}
    for match in re.finditer(r"([A-Za-z0-9_.-]+)=([^ \r\n]+)", line):
        result[match.group(1)] = match.group(2).rstrip(",")
    return result


def safe_int(value: str | None, default: int = 0) -> int:
    if value is None:
        return default
    try:
        return int(value, 0)
    except ValueError:
        return default


def safe_float(value: str | None, default: float = 0.0) -> float:
    if value is None:
        return default
    try:
        parsed = float(value)
    except ValueError:
        return default
    return parsed if math.isfinite(parsed) else default


def first_value(values: dict[str, str], *names: str) -> str:
    for name in names:
        if name in values:
            return values[name]
    return "-"


def first_float(values: dict[str, str], *names: str) -> float | None:
    for name in names:
        if name not in values:
            continue
        try:
            return float(values[name])
        except ValueError:
            return None
    return None


class ProtocolLineMixin:
    def _normalize_proto_line(self, function: int, text: str) -> str:
        stripped = text.strip()
        if function == PROTO_MSG_TEXT_LINE:
            return stripped
        if function == PROTO_MSG_CMD_RX:
            return self._ensure_line_prefix(stripped, "RX")
        if function == PROTO_MSG_CMD_ACK:
            return self._ensure_line_prefix(stripped, "ACK")
        if function == PROTO_MSG_CMD_ERR:
            return self._ensure_line_prefix(stripped, "ERR")
        if function == PROTO_MSG_CMD_OK:
            return self._ensure_line_prefix(stripped, "OK")
        if function == PROTO_MSG_PONG:
            return self._ensure_line_prefix(stripped, "PONG")
        if function == PROTO_MSG_READY:
            return self._ensure_line_prefix(stripped, "READY")
        if function == PROTO_MSG_WIFI_RECORD:
            if stripped.startswith(("WIFI ", "RSP ")):
                return stripped
            return self._ensure_line_prefix(stripped, "WIFI")
        if function == PROTO_MSG_GPS_RECORD:
            if stripped.startswith(("GPS ", "GPS_USART2 ", "M9N ", "HW ", "STATUS ", "RSP ")):
                return stripped
            return self._ensure_line_prefix(stripped, "GPS")
        if function == PROTO_MSG_MAG_RECORD:
            if stripped.startswith(("MAG ", "MAG_I2C1 ", "HW ", "STATUS ", "RSP ")):
                return stripped
            return self._ensure_line_prefix(stripped, "MAG")

        prefix_map = {
            PROTO_MSG_HW_FLASH: "HW FLASH",
            PROTO_MSG_HW_BARO: "HW SPL06",
            PROTO_MSG_HW_IMU: "HW ICM42688",
            PROTO_MSG_STATUS_FLASH: "STATUS flash",
            PROTO_MSG_STATUS_BARO: "STATUS baro",
            PROTO_MSG_STATUS_IMU: "STATUS imu",
            PROTO_MSG_UART_STATS: "UART1",
            PROTO_MSG_CONFIG_SUMMARY: "CFG",
            PROTO_MSG_CONFIG_SERVO: "CFG",
            PROTO_MSG_PARAM_RECORD: "PARAM",
            PROTO_MSG_PID_RECORD: "PID",
            PROTO_MSG_FLASH_RECORD: "FLASH",
            PROTO_MSG_BARO_STATE: "BARO",
            PROTO_MSG_BARO_DIAG: "BARO",
            PROTO_MSG_BARO_RAW: "BARO",
            PROTO_MSG_BARO_STREAM: "BARO",
            PROTO_MSG_RTOS_RECORD: "RTOS",
            PROTO_MSG_FLASH_BENCH: "FLASH",
            PROTO_MSG_AIRFRAME_RECORD: "AIRFRAME",
            PROTO_MSG_SERVO_TYPE: "SERVOTYPE",
        }
        prefix = prefix_map.get(function)
        if prefix is None:
            return stripped if stripped else f"fn=0x{function:04X}"
        return self._ensure_line_prefix(stripped, prefix)

    def _ensure_line_prefix(self, text: str, prefix: str) -> str:
        stripped = text.strip()
        if not stripped:
            return prefix
        if stripped.startswith(prefix):
            return stripped
        return f"{prefix} {stripped}"


__all__ = [
    name
    for name in globals()
    if name.startswith("PROTO_")
] + [
    "ProtocolLineMixin",
    "first_float",
    "first_value",
    "parse_kv",
    "safe_float",
    "safe_int",
]

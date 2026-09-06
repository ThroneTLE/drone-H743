"""协议夹具：键名从固件源码的格式串里抽出来，不许手打。

SKILL.md「Validation」写死了这条：主机工具解析固件输出时，测试夹具必须钉在**真实
发出的格式**上，因为固件从没发过的字段名不会报错，只会安静地永远匹配不上。改版
报告里的 GPS 夹具就踩了这个——它写的是
`GPS ok=1 init=0 nav=7 fix=0 flags=0x00 sv=0 valid=0 age_ms=5000`，而固件那行根本
没有 `flags=`，字段顺序也不是这个。键值解析让它侥幸没出事，但这种夹具下一次就会
掩盖真问题。

所以这里的每个 builder 都配一个 `firmware_format_keys()` 抽出来的键集合，测试可以
直接断言“我造的这行的键 == 固件那行的键”。固件改了字段，夹具当场红灯。
"""

from __future__ import annotations

import re
import struct
from pathlib import Path

from tools.panel_lib import telem_stream


PROJECT_ROOT = Path(__file__).resolve().parents[2]

# 每条格式串的出处，写在这里而不是散在各处的注释里。
FIRMWARE_FORMAT_SOURCES = {
    "GPS status": (PROJECT_ROOT / "App" / "Src" / "app_gps.c", "GPS ok="),
    "GPS pos": (PROJECT_ROOT / "App" / "Src" / "app_gps.c", "GPS pos "),
    "TELEM header": (PROJECT_ROOT / "App" / "Src" / "app_telemetry.c", "TELEM ver="),
    "TELEM CH": (PROJECT_ROOT / "App" / "Src" / "app_telemetry.c", "TELEM CH "),
    "TELEM PAGE": (PROJECT_ROOT / "App" / "Src" / "app_telemetry.c", "TELEM PAGE "),
}

_KEY_RE = re.compile(r"([A-Za-z_][A-Za-z0-9_]*)=")
_STRING_RE = re.compile(r'"((?:[^"\\]|\\.)*)"')


def firmware_format_keys(name: str) -> tuple[str, ...]:
    """从固件源码里取出该格式串的 `key=` 名字，保持源码顺序。

    C 里相邻字符串常量会拼接（`"TELEM ver=..." "frame=%s ..."`），所以这里把紧邻的
    字面量按源码顺序接起来再抽键，否则会漏掉换行续写的后半截。
    """
    if name not in FIRMWARE_FORMAT_SOURCES:
        raise KeyError(f"unknown firmware format {name!r}")
    path, prefix = FIRMWARE_FORMAT_SOURCES[name]
    text = path.read_text(encoding="utf-8", errors="replace")
    literals = [(m.start(), m.group(1)) for m in _STRING_RE.finditer(text)]
    for position, (start, literal) in enumerate(literals):
        if not literal.startswith(prefix):
            continue
        joined = literal
        cursor = start + len(literal) + 2
        for next_start, next_literal in literals[position + 1:]:
            between = text[cursor:next_start]
            if between.strip():                  # 中间夹了别的代码，续写到此为止
                break
            joined += next_literal
            cursor = next_start + len(next_literal) + 2
        return tuple(_KEY_RE.findall(joined))
    raise LookupError(f"format string starting with {prefix!r} not found in {path}")


# ---------------------------------------------------------------- GPS

GPS_STATUS_KEYS = (
    "ok", "init", "fix", "valid", "sv", "pkts", "nav", "nmea", "gga", "age_ms",
)
GPS_POSITION_KEYS = (
    "lon", "lat", "hmsl_mm", "hacc_mm", "vacc_mm", "vn", "ve", "vd", "head_e5", "utc",
)


def gps_status_line(*, ok: int = 1, init: int = 0, fix: int = 0, valid: int = 0,
                    sv: int = 0, pkts: int = 0, nav: int = 0, nmea: int = 0,
                    gga: int = 0, age_ms: int = 0) -> str:
    """`App/Src/app_gps.c` 的 `GPS ok=...` 行。默认就是“没定位”。"""
    return (
        f"GPS ok={ok} init={init} fix={fix} valid={valid} sv={sv} pkts={pkts} "
        f"nav={nav} nmea={nmea} gga={gga} age_ms={age_ms}"
    )


def gps_position_line(*, lon_e7: int = 1210000000, lat_e7: int = 310000000,
                      hmsl_mm: int = 0, hacc_mm: int = 99999, vacc_mm: int = 99999,
                      vn: int = 0, ve: int = 0, vd: int = 0, head_e5: int = 0,
                      utc: str = "2026-09-04T00:00:00") -> str:
    """`App/Src/app_gps.c` 的 `GPS pos ...` 行。

    默认坐标是**有值但精度栏爆表**——这正是 N08 的现场：固件在 `valid=0` 时仍然把
    上一次的坐标发出来，主机不能把它当成一个新的有效轨迹点。
    """
    return (
        f"GPS pos lon={lon_e7} lat={lat_e7} hmsl_mm={hmsl_mm} hacc_mm={hacc_mm} "
        f"vacc_mm={vacc_mm} vn={vn} ve={ve} vd={vd} head_e5={head_e5} utc={utc}"
    )


# ---------------------------------------------------------------- 遥测

TELEM_FRAME_HEADER_FORMAT = "<BBHIIHHQ"

# 从被测代码里取，不在装置里另抄一份数字：抄的那份迟早和协议漂移，而装置漂移
# 的表现是"测试全绿、实机全红"。
TELEM_FRAME_VERSION = telem_stream.TELEM_FRAME_VERSION
TELEM_FRAME_FLAG_WIDE_MASK = telem_stream.TELEM_FRAME_FLAG_WIDE_MASK

DEFAULT_CHANNELS = (
    # (name, unit, min, max, group, param)
    ("roll", "deg", -180.0, 180.0, "attitude", "-"),
    ("pitch", "deg", -90.0, 90.0, "attitude", "-"),
    ("yaw", "deg", -180.0, 180.0, "attitude", "-"),
    ("vel_est_x", "m/s", -5.0, 5.0, "nav", "-"),
    ("vel_est_y", "m/s", -5.0, 5.0, "nav", "-"),
    ("flow_height", "m", 0.0, 5.0, "nav", "-"),
    ("roll_rate_kd", "-", 0.0, 10.0, "gain", "coax.roll_rate_kd"),
    ("pitch_rate_kd", "-", 0.0, 10.0, "gain", "coax.pitch_rate_kd"),
)


def telemetry_schema_lines(channels=DEFAULT_CHANNELS, *, rate: int = 40,
                           version: int = 3, frame: str = "body_flu",
                           contract: int = 1) -> list[str]:
    """一整份通道表：header + 每路 CH + PAGE 收尾，键名与 `app_telemetry.c` 一致。"""
    count = len(channels)
    lines = [
        f"TELEM ver={version} n={count} rate={rate} page={count} hash=00000000 "
        f"frame={frame} contract={contract}"
    ]
    for index, (name, unit, low, high, group, param) in enumerate(channels):
        lines.append(
            f"TELEM CH idx={index} name={name} unit={unit} "
            f"min={low:.3f} max={high:.3f} grp={group} param={param}"
        )
    lines.append(f"TELEM PAGE from=0 count={count} next=-1")
    return lines


def telemetry_frame(values: dict[int, float], *, schema_hash: int, seq: int = 0,
                    t_us: int = 0) -> bytes:
    """一帧掩码 payload（transport 已经剥掉 `$X` 壳）。

    与 `tests/test_dashboard_page.py::telem_frame` 同一个布局；抽到这里是为了让录制、
    几何、后续包共用一份，而不是每个测试文件各拼一次结构体。
    """
    mask = 0
    for index in values:
        mask |= 1 << index
    # v2 的掩码是变长的：高 64 位有东西才发 16 字节并置 WIDE_MASK。宽窄由内容
    # 决定，装置这里也照这条来，否则装置造出来的帧和固件造的不是同一种帧。
    wide = mask >> 64 != 0
    flags = TELEM_FRAME_FLAG_WIDE_MASK if wide else 0
    head = struct.pack(
        TELEM_FRAME_HEADER_FORMAT, TELEM_FRAME_VERSION, 1, seq & 0xFFFF,
        schema_hash, t_us, 0, flags, mask & ((1 << 64) - 1),
    )
    if wide:
        head += struct.pack("<Q", mask >> 64)
    body = b"".join(struct.pack("<f", values[index]) for index in sorted(values))
    return head + body


__all__ = [
    "DEFAULT_CHANNELS",
    "FIRMWARE_FORMAT_SOURCES",
    "GPS_POSITION_KEYS",
    "GPS_STATUS_KEYS",
    "PROJECT_ROOT",
    "firmware_format_keys",
    "gps_position_line",
    "gps_status_line",
    "telemetry_frame",
    "telemetry_schema_lines",
]

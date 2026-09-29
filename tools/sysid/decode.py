"""SYSID 批量帧解码。

**布局不在这里写死。** 固件通过 `SYSID SCHEMA?` 报出字段表（名字/单位/缩放/类型），
主机照表逐字段取值；帧里带的 `schema_hash` 与固件报的对不上就拒绝解码。

这条纪律是有来历的：主机侧抄一份偏移量，迟早和固件对不上，而对不上的表现不是
崩溃或报错，是**安静地解出一组看着正常的数**——一整阶辨识就此作废且查不出来。
"""
from __future__ import annotations

import struct
import math
from dataclasses import dataclass, field

HEADER_BYTES = 16
RECORD_VERSION = 2

FIELD_TYPE_I16 = 0
FIELD_TYPE_U16 = 1

_UNPACK = {FIELD_TYPE_I16: "<h", FIELD_TYPE_U16: "<H"}

# flags 位，与 drv_sysid_record.h 同源
FLAG_FIRST_BATCH = 0x0001
FLAG_LAST_BATCH = 0x0002
FLAG_ABORTED = 0x0004
FLAG_ERPM_VALID = 0x0008
# 本批之前丢过样本。必须由固件显式报，不能让主机从时间戳里猜——
# 线上时间是 base + k*dt 的压缩形式，没报出来的断点会变成一个假的延迟值。
FLAG_GAP = 0x0010


class SchemaMismatch(ValueError):
    """帧里的 schema_hash 与固件报的字段表对不上。"""


@dataclass(frozen=True)
class SysIdField:
    name: str
    unit: str
    scale: float
    type: int

    @property
    def width(self) -> int:
        return 2


@dataclass(frozen=True)
class SysIdSchema:
    """固件报上来的字段表。`hash` 是固件算的，主机只比对、不重算。"""

    fields: tuple[SysIdField, ...]
    hash: int
    version: int = RECORD_VERSION

    @property
    def record_bytes(self) -> int:
        return sum(f.width for f in self.fields)

    def field_names(self) -> tuple[str, ...]:
        return tuple(f.name for f in self.fields)


@dataclass(frozen=True)
class SysIdBatch:
    run_id: int
    base_t_us: int
    dt_us: int
    flags: int
    samples: tuple[dict[str, float], ...] = field(default_factory=tuple)

    @property
    def first(self) -> bool:
        return bool(self.flags & FLAG_FIRST_BATCH)

    @property
    def last(self) -> bool:
        return bool(self.flags & FLAG_LAST_BATCH)

    @property
    def aborted(self) -> bool:
        return bool(self.flags & FLAG_ABORTED)

    @property
    def erpm_valid(self) -> bool:
        return bool(self.flags & FLAG_ERPM_VALID)

    @property
    def gap(self) -> bool:
        """本批之前丢过样本：与上一批之间的时间轴不连续。"""
        return bool(self.flags & FLAG_GAP)

    def timestamps_us(self) -> tuple[int, ...]:
        """逐样本的固件时间戳。

        用 base + k*dt 而不是收包时刻：辨识要测的就是延迟，
        把主机的收包抖动掺进时间轴等于把答案搅浑。
        """
        return tuple((self.base_t_us + int(sample.get("offset_us", index*self.dt_us))) & 0xffffffff
                     for index, sample in enumerate(self.samples))


def decode_batch(data: bytes, schema: SysIdSchema) -> SysIdBatch:
    """解一帧 SYSID 批量数据。data 是 $X 外层剥掉之后的净荷。"""
    if len(data) < HEADER_BYTES:
        raise ValueError(f"帧长 {len(data)} 不足一个包头 {HEADER_BYTES}")

    version, count, run_id, schema_hash, base_t_us, dt_us, flags = struct.unpack_from(
        "<BBHIIHH", data, 0)

    if version not in (1, 2, 3) or version != schema.version:
        raise SchemaMismatch(f"记录版本 {version} != 期望 {schema.version}")
    if schema_hash != schema.hash:
        raise SchemaMismatch(
            f"schema_hash 0x{schema_hash:08X} != 固件报的 0x{schema.hash:08X}；"
            "先重新请求 SYSID SCHEMA?，不要按旧表硬解")
    if count == 0:
        raise ValueError("批量帧的样本数为 0")

    record_bytes = schema.record_bytes
    needed = HEADER_BYTES + count * record_bytes
    if not dt_us or not record_bytes or len(data) != needed:
        raise ValueError(f"帧长 {len(data)} 不足 {needed}（{count} 条 × {record_bytes}）")

    samples: list[dict[str, float]] = []
    for index in range(count):
        offset = HEADER_BYTES + index * record_bytes
        row: dict[str, float] = {}
        for item in schema.fields:
            raw = struct.unpack_from(_UNPACK[item.type], data, offset)[0]
            row[item.name] = raw * item.scale
            offset += item.width
        samples.append(row)

    return SysIdBatch(run_id=run_id, base_t_us=base_t_us, dt_us=dt_us,
                      flags=flags, samples=tuple(samples))


def parse_schema_lines(lines: list[str]) -> SysIdSchema:
    """解析固件对 `SYSID SCHEMA?` 的回复。

    期望格式（与 TELEM CH 同风格）::

        SYSID SCHEMA ver=1 n=10 hash=1A2B3C4D rec=20
        SYSID FIELD idx=0 name=gx unit=rad/s scale=0.001 type=i16
    """
    header: dict[str, str] = {}
    fields: dict[int, SysIdField] = {}
    type_by_name = {"i16": FIELD_TYPE_I16, "u16": FIELD_TYPE_U16}

    for line in lines:
        text = line.strip()
        if text.startswith("SYSID SCHEMA "):
            header = _parse_kv(text[len("SYSID SCHEMA "):])
        elif text.startswith("SYSID FIELD "):
            parts = _parse_kv(text[len("SYSID FIELD "):])
            index = int(parts["idx"])
            fields[index] = SysIdField(
                name=parts["name"], unit=parts["unit"],
                scale=float(parts["scale"]),
                type=type_by_name[parts["type"]])

    if not header or "hash" not in header:
        raise ValueError("没有收到 SYSID SCHEMA 头行")
    count = int(header.get("n", len(fields)))
    if len(fields) != count:
        raise ValueError(f"字段行收到 {len(fields)} 条，头行声明 {count} 条")
    if sorted(fields) != list(range(count)):
        raise ValueError(f"字段索引不连续：{sorted(fields)}")

    ordered = tuple(fields[index] for index in sorted(fields))
    if not (1 <= count <= 32) or len({f.name for f in ordered}) != count:
        raise ValueError("字段数量或名称重复")
    if any(not math.isfinite(f.scale) or f.scale <= 0 for f in ordered):
        raise ValueError("字段缩放无效")
    schema = SysIdSchema(fields=ordered, hash=int(header["hash"], 16),
                         version=int(header.get("ver", RECORD_VERSION)))

    # 头行报的记录长度必须和字段表加起来对得上。对不上说明这两行不是同一版固件
    # 发出来的（串口上两段回复交错时真会这样），此时按任一边解都是错的。
    if "rec" in header and int(header["rec"]) != schema.record_bytes:
        raise SchemaMismatch(
            f"头行 rec={header['rec']} 与字段表合计 {schema.record_bytes} 不符")
    return schema


def _parse_kv(text: str) -> dict[str, str]:
    result: dict[str, str] = {}
    for token in text.split():
        if "=" in token:
            key, _, value = token.partition("=")
            result[key] = value
    return result

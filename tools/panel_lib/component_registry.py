"""Bounded, request-correlated component snapshots. No Tk or hardware knowledge."""
from __future__ import annotations
from dataclasses import dataclass
import math
import struct
import zlib

HEADER = struct.Struct("<BBHIH")
RECORD = struct.Struct("<HBBiII")
MAX_COUNT = 16
MAX_PAYLOAD = 247


@dataclass(frozen=True)
class Field:
    label: str
    unit: str
    type: int
    value: int | float | None


@dataclass(frozen=True)
class Component:
    id: int
    state: int
    code: int
    age_ms: int | None
    samples: int
    name: str
    model: str
    bus: str
    stage: str
    note: str
    fields: tuple[Field, ...]


def decode_record(data: bytes) -> Component:
    if len(data) < RECORD.size:
        raise ValueError("元件记录不完整")
    identifier, state, n, code, age, samples = RECORD.unpack_from(data)
    if not identifier or state > 4 or n > 4:
        raise ValueError("元件状态或字段数量非法")
    pos = RECORD.size

    def text(limit):
        nonlocal pos
        if pos >= len(data):
            raise ValueError("缺少字符串长度")
        size = data[pos]; pos += 1
        if size > limit or pos + size > len(data):
            raise ValueError("字符串越界")
        value = data[pos:pos+size].decode("utf-8", errors="strict"); pos += size
        if any(ord(c) < 32 or 127 <= ord(c) < 160 for c in value):
            raise ValueError("字符串含控制字符")
        return value

    strings = [text(limit) for limit in (32, 24, 24, 20, 64)]
    if not strings[0]:
        raise ValueError("元件没有名称")
    fields = []
    for _ in range(n):
        label, unit = text(16), text(8)
        if not label or any(f.label == label for f in fields):
            raise ValueError("字段名称为空或重复")
        if pos + 6 > len(data):
            raise ValueError("数值字段不完整")
        kind, valid = struct.unpack_from("<BB", data, pos)
        if kind not in (1, 2, 3, 4) or valid not in (0, 1):
            raise ValueError("字段类型或有效位非法")
        value = struct.unpack_from({1:"<I", 2:"<i", 3:"<f", 4:"<I"}[kind], data, pos+2)[0]
        pos += 6
        if valid and kind == 3 and not math.isfinite(value):
            raise ValueError("有效浮点字段为非有限值")
        fields.append(Field(label, unit, kind, value if valid else None))
    if pos != len(data):
        raise ValueError("元件记录有额外字节")
    return Component(identifier, state, code, None if age==0xffffffff else age,
                     samples, *strings, tuple(fields))


class RegistryTransaction:
    def __init__(self, nonce: int):
        self.nonce = nonce
        self.count = None
        self.records: list[Component] = []
        self.crc = 0

    def feed(self, payload: bytes) -> tuple[Component, ...] | None:
        if not HEADER.size <= len(payload) <= MAX_PAYLOAD:
            raise ValueError("注册帧长度非法")
        version, kind, count, nonce, index = HEADER.unpack_from(payload)
        if nonce != self.nonce:
            return None
        if version != 1 or count > MAX_COUNT:
            raise ValueError("不支持的注册协议版本或数量")
        body = payload[HEADER.size:]
        if kind == 4:
            if len(body) != 4:
                raise ValueError("注册错误帧不完整")
            raise ValueError(f"飞控注册响应错误 code={struct.unpack('<I', body)[0]}")
        if kind == 1:
            if self.count is not None or index or body:
                raise ValueError("注册起始帧重复或非法")
            self.count = count
        elif self.count is None or self.count != count:
            raise ValueError("缺少注册起始帧或数量改变")
        elif kind == 2:
            if index != len(self.records) or index >= count:
                raise ValueError("元件记录丢失、乱序或重复")
            record = decode_record(body)
            if any(item.id == record.id for item in self.records):
                raise ValueError("元件ID重复")
            self.records.append(record)
            self.crc = zlib.crc32(payload, self.crc)
        elif kind == 3:
            if index != count or len(self.records) != count or len(body) != 4:
                raise ValueError("注册清单未接收完整")
            if struct.unpack("<I", body)[0] != self.crc:
                raise ValueError("注册清单校验失败")
            return tuple(self.records)
        else:
            raise ValueError("未知注册帧类型")
        return None


def field_text(field: Field) -> str:
    if field.value is None:
        value = "—"
    elif field.type == 4:
        value = f"0x{field.value:X}"
    elif field.type == 3:
        value = f"{field.value:.6g}"
    else:
        value = str(field.value)
    return f"{field.label}={value}" + (f" {field.unit}" if field.unit else "")

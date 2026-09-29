"""称重传感器：Modbus-RTU 读数 + 标定 + 去皮。

ScaleCalibration 提供显式选择的最小二乘直线格式。
原 RS485 界面的单点比例和分段线性标定由 legacy_calibration 直接继承，
实测辨识不得在用户不知情时重新拟合已有标定点。
"""

from __future__ import annotations

import json
import struct
import threading
from dataclasses import dataclass, field
from pathlib import Path

FUNC_READ_HOLDING = 0x03

#: 标定残差超过量程的这个比例就判为可疑。
RESIDUAL_WARN_FRACTION = 0.01


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc & 0xFFFF


def add_crc(frame: bytes) -> bytes:
    return frame + struct.pack("<H", crc16_modbus(frame))


def check_crc(frame: bytes) -> bool:
    if len(frame) < 4:
        return False
    return crc16_modbus(frame[:-2]) == (frame[-2] | (frame[-1] << 8))


def dip_to_addr(dip: str) -> int:
    """拨码 -> 站号。权重 1/2/4/8，第一位是 1x。"""
    cleaned = "".join(ch for ch in dip if ch in "01")
    if len(cleaned) != 4:
        raise ValueError("拨码必须是四位 0/1，例如 1000")
    return sum(1 << index for index, bit in enumerate(cleaned) if bit == "1")


def read_request(addr: int, register: int, count: int) -> bytes:
    return add_crc(bytes([addr & 0xFF, FUNC_READ_HOLDING,
                          (register >> 8) & 0xFF, register & 0xFF,
                          (count >> 8) & 0xFF, count & 0xFF]))


def parse_signed32(response: bytes, addr: int) -> int:
    """解一帧两寄存器的读回复，低字在前的有符号 32 位。

    校验不过就抛异常而不是返回 0：一个静默的 0 会被当成"秤上没东西"，
    而那恰好是最像正常的读数。
    """
    if len(response) < 9:
        raise ValueError(f"回复太短：{len(response)} 字节")
    if response[0] != (addr & 0xFF):
        raise ValueError(f"站号不符：收到 {response[0]}，期望 {addr}")
    if response[1] != FUNC_READ_HOLDING:
        raise ValueError(f"功能码不符：{response[1]:#04x}")
    if not check_crc(response[:9]):
        raise ValueError("CRC 校验失败")
    low, high = struct.unpack_from(">HH", response, 3)
    value = (high << 16) | low
    return value - (1 << 32) if value >= (1 << 31) else value


@dataclass
class ScaleCalibration:
    """raw -> 克 的直线标定 `grams = gain * (raw - offset_raw)`。"""

    points: list[tuple[float, float]] = field(default_factory=list)
    gain: float = 0.0
    offset_raw: float = 0.0
    residual_g: float = 0.0

    def fit(self) -> "ScaleCalibration":
        if len(self.points) < 2:
            raise ValueError("至少要两个标定点（含空载点）才能定一条直线")
        n = len(self.points)
        sum_x = sum(p[0] for p in self.points)
        sum_y = sum(p[1] for p in self.points)
        sum_xx = sum(p[0] * p[0] for p in self.points)
        sum_xy = sum(p[0] * p[1] for p in self.points)
        denominator = n * sum_xx - sum_x * sum_x
        if abs(denominator) < 1e-12:
            raise ValueError("所有标定点的 raw 都一样，定不出斜率")
        self.gain = (n * sum_xy - sum_x * sum_y) / denominator
        intercept = (sum_y - self.gain * sum_x) / n
        self.offset_raw = -intercept / self.gain if self.gain != 0.0 else 0.0
        worst = 0.0
        for raw, grams in self.points:
            worst = max(worst, abs(self.grams(raw) - grams))
        self.residual_g = worst
        return self

    def grams(self, raw: float) -> float:
        return self.gain * (raw - self.offset_raw)

    def suspicious(self) -> str | None:
        """残差大到该重做标定时给一句话，否则 None。"""
        if not self.points:
            return "还没有标定点"
        span = max(p[1] for p in self.points) - min(p[1] for p in self.points)
        if span <= 0.0:
            return "所有参考砝码一样重，定不出斜率"
        if self.residual_g > span * RESIDUAL_WARN_FRACTION:
            return (f"最大残差 {self.residual_g:.2f} g，超过量程的 "
                    f"{RESIDUAL_WARN_FRACTION * 100:.0f}%：多半有一个点没称稳，"
                    "重做那一点比接受这条曲线划算")
        return None

    def to_dict(self) -> dict:
        return {"points": [{"raw": raw, "grams": grams}
                           for raw, grams in self.points],
                "gain": self.gain, "offset_raw": self.offset_raw,
                "residual_g": self.residual_g}

    @classmethod
    def from_dict(cls, data: dict) -> "ScaleCalibration":
        points = [(float(item["raw"]), float(item["grams"]))
                  for item in data.get("points", [])]
        instance = cls(points=points)
        if len(points) >= 2:
            instance.fit()
        else:
            instance.gain = float(data.get("gain", 0.0))
            instance.offset_raw = float(data.get("offset_raw", 0.0))
        return instance

    def save(self, path: Path) -> Path:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(self.to_dict(), indent=2), encoding="utf-8")
        return path

    @classmethod
    def load(cls, path: Path) -> "ScaleCalibration":
        return cls.from_dict(json.loads(path.read_text(encoding="utf-8")))


class LoadCell:
    """一路称重通道。`transport` 只要有 `request(bytes, expected) -> bytes`。

    去皮（tare）记在**这里**而不是写进标定：装上桨、接上线之后的自重每次都不一样，
    把它算进标定等于每换一次装配就要重标一次传感器。
    """

    def __init__(self, transport, addr: int, calibration: ScaleCalibration,
                 register: int = 0x0000) -> None:
        self.transport = transport
        self.addr = addr
        self.calibration = calibration
        self.register = register
        # 没去皮之前，零点就是标定直线的零点。
        self.tare_raw = calibration.offset_raw
        self._lock = threading.RLock()

    def read_raw(self) -> int:
        with self._lock:
            response = self.transport.request(
                read_request(self.addr, self.register, 2), 9)
            return parse_signed32(response, self.addr)

    def read_grams(self) -> float:
        """当前零点之上的重量。去皮点是 `tare_raw`，不动标定本身。"""
        return self.grams_from_raw(self.read_raw())

    def grams_from_raw(self, raw: float) -> float:
        """Convert an already captured sample without reading the sensor twice."""
        with self._lock:
            return self.calibration.gain * (raw - self.tare_raw)

    def tare(self, samples: int = 8) -> float:
        """空载取平均作为零点。单点去皮会把一次抖动永久写进整条曲线。"""
        if samples < 1:
            raise ValueError("去皮至少要一个样本")
        with self._lock:
            total = 0
            for _ in range(samples):
                total += self.read_raw()
            self.tare_raw = total / samples
        return self.tare_raw

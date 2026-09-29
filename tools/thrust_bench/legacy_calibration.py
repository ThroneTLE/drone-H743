"""Reuse the original pressure GUI's calibration without refitting its points."""
from __future__ import annotations

from dataclasses import dataclass
import math

from .scale import LoadCell


def legacy_grams(raw_value: float, points) -> float | None:
    """Original single-point ratio and piecewise interpolation/extrapolation."""
    points = sorted(points, key=lambda item: item[0])
    if not points:
        return None
    if len(points) == 1:
        raw, grams = points[0]
        return grams if raw_value == raw else raw_value * (grams / raw) if raw != 0 else None
    if raw_value <= points[0][0]:
        p0, p1 = points[0], points[1]
    elif raw_value >= points[-1][0]:
        p0, p1 = points[-2], points[-1]
    else:
        p0, p1 = points[0], points[1]
        for left, right in zip(points, points[1:]):
            if left[0] <= raw_value <= right[0]:
                p0, p1 = left, right
                break
    raw0, grams0 = p0
    raw1, grams1 = p1
    if raw1 == raw0:
        return grams0
    return grams0 + (raw_value - raw0) / (raw1 - raw0) * (grams1 - grams0)


@dataclass(frozen=True)
class LegacyCalibration:
    points: tuple[tuple[float, float], ...]

    @classmethod
    def from_points(cls, points) -> "LegacyCalibration":
        snapshot = tuple(sorted((float(raw), float(grams)) for raw, grams in points))
        if not snapshot:
            raise ValueError("请先在称重页面采集砝码标定点，再开始实测辨识")
        if not all(math.isfinite(v) for point in snapshot for v in point):
            raise ValueError("称重标定点必须是有限数字")
        if len({raw for raw, _ in snapshot}) != len(snapshot):
            raise ValueError("称重标定点有重复原始读数，请重新标定")
        if len(snapshot) == 1 and (snapshot[0][0] == 0 or snapshot[0][1] == 0):
            raise ValueError("只有空载点不能换算重量，请再采集一个已知砝码点")
        if len(snapshot) > 1 and len({grams for _, grams in snapshot}) == 1:
            raise ValueError("请使用不同的已知重量进行称重标定")
        return cls(snapshot)

    def grams(self, raw: float) -> float:
        value = legacy_grams(raw, self.points)
        if value is None or not math.isfinite(value):
            raise ValueError("称重标定不能换算当前读数")
        return value

    def to_dict(self) -> dict:
        return {"kind": "legacy_single_point_ratio" if len(self.points) == 1 else "legacy_piecewise_linear",
                "points": [{"raw": r, "grams": g} for r, g in self.points],
                "single_point_assumes_zero_origin": len(self.points) == 1}


class LegacyLoadCell(LoadCell):
    """Take raw readings through the canonical Modbus reader; keep legacy units."""
    def __init__(self, transport, addr: int, calibration: LegacyCalibration,
                 register: int = 0x0000) -> None:
        # LoadCell's I/O lock and tare acquisition are shared. Its linear default
        # zero does not apply to piecewise calibration, so initialize explicitly.
        import threading
        self.transport = transport
        self.addr = addr
        self.calibration = calibration
        self.register = register
        self.tare_raw = None
        self._lock = threading.RLock()

    def grams_from_raw(self, raw: float) -> float:
        with self._lock:
            gross = self.calibration.grams(raw)
            return gross if self.tare_raw is None else gross - self.calibration.grams(self.tare_raw)

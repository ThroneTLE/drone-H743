"""推力源抽象。

辨识要把"当前油门"换算成"当前总推力"，因为力矩 τ = P·eff·l·F·sin(tilt) 里的 F
就是它。换算有三种来源，可信度差得很远，所以每一种都**自报家门**：

* `PwmTable` —— 固件里那张 21 点脉宽→推力表。飞控实际用的就是它，所以辨识默认
  用这一份：辨出来的模型才和在飞的那套对得上。
* `RpmThrust` —— 推力台上按实测转速标定的 `F = f(eRPM, V)`。最准，但需要双向
  DShot 回传的 eRPM。
* `VoltageScaled` —— 在基准模型上按电池电压缩放。掉压之后推力会跟着掉，
  一趟长激励的前后半段可以差出 10%，这一项就是补它的。

`source_quality` 不是装饰：报告里必须写清楚推力是量出来的还是算出来的，
否则一个 ±10% 的推力误差会原样变成 ±10% 的惯量误差，而看报告的人无从分辨。
"""
from __future__ import annotations

import bisect
from dataclasses import dataclass, field
from typing import Protocol

#: 固件 `drv_coax_ctrl.c` 的 21 点表覆盖的脉宽范围。
PULSE_MIN_US = 1000
PULSE_MAX_US = 2000


class ThrustSource(Protocol):
    """统一接口：给脉宽/电压/转速，回总推力 [N]。"""

    quality: str

    def thrust_n(self, pulse_us: float, voltage_v: float | None = None,
                 erpm: float | None = None) -> float:
        ...


@dataclass
class PwmTable:
    """脉宽 -> 总推力的分段线性表。

    默认与固件同源：`table` 是 21 个等距点上的**总**推力 [N]（两个桨加起来）。
    不给表时按 `max_total_force_n` 走平方律近似——注明是近似，不是标定。
    """

    table_n: tuple[float, ...] = ()
    max_total_force_n: float = 0.0
    quality: str = "firmware-table"

    def __post_init__(self) -> None:
        if not self.table_n:
            if self.max_total_force_n <= 0.0:
                raise ValueError("既没有推力表也没有最大推力，无法换算")
            # 桨的推力大致正比于转速平方，而脉宽到转速大致线性。
            points = 21
            self.table_n = tuple(
                self.max_total_force_n * (index / (points - 1)) ** 2
                for index in range(points))
            self.quality = "square-law-approximation"

    def thrust_n(self, pulse_us: float, voltage_v: float | None = None,
                 erpm: float | None = None) -> float:
        del voltage_v, erpm
        count = len(self.table_n)
        span = PULSE_MAX_US - PULSE_MIN_US
        position = (pulse_us - PULSE_MIN_US) / span * (count - 1)
        if position <= 0.0:
            return self.table_n[0]
        if position >= count - 1:
            return self.table_n[-1]
        low = int(position)
        fraction = position - low
        return self.table_n[low] + (self.table_n[low + 1] - self.table_n[low]) * fraction


@dataclass
class RpmThrust:
    """实测 `F = f(eRPM)`，可选按电压分层。

    `points` 是升序的 (erpm, thrust_n)。`voltage_v` 给出时在相邻两层之间线性插值；
    超出标定过的电压范围就**夹住**并把 quality 降级——外推推力比内插危险得多，
    而外推出来的数看起来和内插的一模一样。
    """

    points: tuple[tuple[float, float], ...] = ()
    layers: dict[float, tuple[tuple[float, float], ...]] = field(default_factory=dict)
    quality: str = "measured-rpm"

    def thrust_n(self, pulse_us: float, voltage_v: float | None = None,
                 erpm: float | None = None) -> float:
        del pulse_us
        if erpm is None:
            raise ValueError("RpmThrust 需要 eRPM；没有回传时请改用 PwmTable")
        if not self.layers:
            return _interp(self.points, erpm)
        voltages = sorted(self.layers)
        if voltage_v is None:
            voltage_v = voltages[len(voltages) // 2]
        if voltage_v <= voltages[0]:
            return _interp(self.layers[voltages[0]], erpm)
        if voltage_v >= voltages[-1]:
            return _interp(self.layers[voltages[-1]], erpm)
        index = bisect.bisect_left(voltages, voltage_v)
        low_v, high_v = voltages[index - 1], voltages[index]
        fraction = (voltage_v - low_v) / (high_v - low_v)
        low = _interp(self.layers[low_v], erpm)
        high = _interp(self.layers[high_v], erpm)
        return low + (high - low) * fraction


@dataclass
class VoltageScaled:
    """在一个基准推力源上按电池电压缩放。

    桨推力 ∝ 转速²，而转速 ∝ 电压（定占空比下），所以缩放因子取 (V/V_ref)²。
    这是近似，不是标定——一趟 4 秒的激励里电池掉 0.3 V 就有约 8% 的推力差，
    补上它比装作没有强，但不能当成量出来的值。
    """

    base: ThrustSource
    reference_voltage_v: float
    quality: str = "voltage-scaled-approximation"

    def thrust_n(self, pulse_us: float, voltage_v: float | None = None,
                 erpm: float | None = None) -> float:
        value = self.base.thrust_n(pulse_us, voltage_v, erpm)
        if voltage_v is None or self.reference_voltage_v <= 0.0:
            return value
        ratio = voltage_v / self.reference_voltage_v
        return value * ratio * ratio


def _interp(points: tuple[tuple[float, float], ...], x: float) -> float:
    if not points:
        raise ValueError("标定表为空")
    xs = [p[0] for p in points]
    if x <= xs[0]:
        return points[0][1]
    if x >= xs[-1]:
        return points[-1][1]
    index = bisect.bisect_left(xs, x)
    x0, y0 = points[index - 1]
    x1, y1 = points[index]
    return y0 + (y1 - y0) * (x - x0) / (x1 - x0)


def from_airframe(values: dict[str, float]) -> PwmTable:
    """按机体参数造一个推力源。`max_total_force_n` 由 `compute_derived` 派生。"""
    return PwmTable(max_total_force_n=float(values.get("max_total_force_n", 0.0)))

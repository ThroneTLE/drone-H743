"""扫描编排：油门阶梯、电压分层、上下行回程。

三件旧那套没做、而且每一件都会直接让标定结果失效的事：

1. **电压分层。** 同一档油门在 12.6 V 和 11.1 V 下推力差 15~20%。不记电压的
   标定表只在"当时那块电池那个电量"下成立，换一块电池就不对了——而表面上
   完全看不出来。这里每个点都带**带载电压**（不是静态电压），并支持按电压
   分层重复整条阶梯。

2. **上下行回程。** 从低到高扫一遍、再从高到低扫一遍。两条曲线的差就是迟滞
   （电调的加减速不对称 + 桨的惯量）。只扫单向的话迟滞会整体表现成一个
   "曲线偏了一点"，看不出来是迟滞。

3. **每档先等稳再取样。** 桨的转速是一阶惯性，时间常数几十到上百毫秒。
   下发之后立刻读数读到的是过渡过程，而过渡过程读出来的推力**总是偏小**——
   于是整条曲线系统性偏低，而形状正常。
"""

from __future__ import annotations

from dataclasses import dataclass


#: 每档下发之后等多久再开始取样 [s]。覆盖电调 + 桨的一阶过渡。
DEFAULT_SETTLE_S = 1.2
#: 每档取几个样本求平均。
DEFAULT_SAMPLES = 8
#: 相邻样本间隔 [s]。
DEFAULT_SAMPLE_INTERVAL_S = 0.08


@dataclass(frozen=True)
class SweepPoint:
    percent: float
    #: "up" 或 "down"。回程点用来量迟滞。
    direction: str
    #: 目标电压层 [V]；None 表示不分层。
    voltage_layer_v: float | None = None
    settle_s: float = DEFAULT_SETTLE_S
    samples: int = DEFAULT_SAMPLES
    sample_interval_s: float = DEFAULT_SAMPLE_INTERVAL_S


@dataclass
class SweepPlan:
    points: tuple[SweepPoint, ...]
    motor_label: str = "dual"
    notes: str = ""

    @property
    def duration_s(self) -> float:
        return sum(p.settle_s + p.samples * p.sample_interval_s
                   for p in self.points)

    def voltage_layers(self) -> tuple[float | None, ...]:
        seen: list[float | None] = []
        for point in self.points:
            if point.voltage_layer_v not in seen:
                seen.append(point.voltage_layer_v)
        return tuple(seen)


def plan_sweep(*, minimum_percent: float = 0.0, maximum_percent: float = 100.0,
               step_percent: float = 5.0,
               voltage_layers: tuple[float, ...] = (),
               include_return: bool = True,
               settle_s: float = DEFAULT_SETTLE_S,
               samples: int = DEFAULT_SAMPLES,
               sample_interval_s: float = DEFAULT_SAMPLE_INTERVAL_S,
               motor_label: str = "dual") -> SweepPlan:
    """生成一条扫描计划。

    `include_return=True` 时每层扫完上行再扫下行（**不重复最高点**：
    它已经在上行的末尾量过一次了，重复只会在迟滞图上多出一个零差的点）。
    """
    if step_percent <= 0.0:
        raise ValueError("步长必须为正")
    if maximum_percent < minimum_percent:
        raise ValueError("最大油门不能低于最小油门")

    ladder: list[float] = []
    value = minimum_percent
    while value <= maximum_percent + 1e-9:
        ladder.append(round(value, 6))
        value += step_percent
    if ladder and ladder[-1] < maximum_percent - 1e-9:
        ladder.append(maximum_percent)

    layers: tuple[float | None, ...] = voltage_layers or (None,)
    points: list[SweepPoint] = []
    for layer in layers:
        for percent in ladder:
            points.append(SweepPoint(percent, "up", layer, settle_s, samples,
                                     sample_interval_s))
        if include_return:
            for percent in reversed(ladder[:-1]):
                points.append(SweepPoint(percent, "down", layer, settle_s,
                                         samples, sample_interval_s))
    return SweepPlan(points=tuple(points), motor_label=motor_label)


@dataclass
class Measurement:
    """一档的实测结果。"""

    percent: float
    direction: str
    pulse_us: int
    thrust_g: float
    #: **带载**电压，不是静态电压。带载才是桨真正工作的那个电压。
    voltage_v: float | None = None
    current_a: float | None = None
    rpm: float | None = None
    voltage_layer_v: float | None = None
    samples: int = 0
    spread_g: float = 0.0

    @property
    def suspicious(self) -> str | None:
        """这一档的读数是否可疑。"""
        if self.samples < 2:
            return "只取了一个样本，看不出稳没稳"
        if self.thrust_g != 0.0 and self.spread_g > abs(self.thrust_g) * 0.05:
            return (f"{self.samples} 个样本的极差 {self.spread_g:.1f} g "
                    f"超过读数的 5%：多半还没稳，或者台架在共振")
        return None


def hysteresis(measurements: list[Measurement]) -> dict[float | tuple[float | None, float], float]:
    """同一档上行与下行的推力差 [g]。**上行 − 下行**。

    正值只表示相同油门下下行稳态推力更小。单凭符号不能把原因唯一归到减速速度；
    电调策略、热状态、供电和装配气动都可能参与。
    """
    explicit_layers = {m.voltage_layer_v for m in measurements if m.voltage_layer_v is not None}
    # Historical callers sometimes recorded the layer only on the up leg.  A
    # sole explicit layer is unambiguous and retains the old float-key API.
    sole_layer = next(iter(explicit_layers)) if len(explicit_layers) == 1 else None
    def layer_of(item: Measurement) -> float | None:
        return item.voltage_layer_v if item.voltage_layer_v is not None else sole_layer
    layers = {layer_of(m) for m in measurements}
    stratified = len(layers) > 1
    up = {(layer_of(m), m.percent): m for m in measurements if m.direction == "up"}
    down = {(layer_of(m), m.percent): m for m in measurements if m.direction == "down"}
    result: dict[float | tuple[float | None, float], float] = {}
    for key in sorted(set(up) & set(down), key=lambda value: (value[0] is None, value[0] or 0, value[1])):
        output_key = key if stratified else key[1]
        result[output_key] = up[key].thrust_g - down[key].thrust_g
    return result


def voltage_sag(measurements: list[Measurement]) -> dict[float, float]:
    """每档的掉压 [V]：该层的标称电压减去实测带载电压。

    掉得多说明电池内阻大或线细。它会直接改变推力，所以必须和推力一起记。
    """
    result: dict[float, float] = {}
    for item in measurements:
        if item.voltage_layer_v is None or item.voltage_v is None:
            continue
        result[item.percent] = item.voltage_layer_v - item.voltage_v
    return result

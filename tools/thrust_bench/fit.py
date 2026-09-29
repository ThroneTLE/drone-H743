"""推力台的拟合：F(油门)、F(转速, 电压)、以及 C_T。

一条纪律：**没有实测转速就不给 C_T**。

`C_T = F / (ρ n² D⁴)` 里转速是平方项。用 kv 推算的转速去算 C_T，那个 0.8 的
"带载系数"实际在 0.6~0.9 之间飘，于是 C_T 会差出两倍多——而两倍的 C_T 足以让
一架按图纸算着能飞的飞机实际飞不起来。所以这里宁可返回 None。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .rpm import RpmSource
from .sweep import Measurement

#: 海平面 15°C 的空气密度 [kg/m³]。
AIR_DENSITY = 1.225
GRAMS_TO_NEWTON = 9.80665 / 1000.0


def trimmed_mean(values: list[float], trim: float = 0.2) -> float:
    """去掉两端各 `trim` 比例后求平均。

    推力台上的离群值来自碰一下台子、线缆晃动这类一次性扰动，
    它们在均值里的权重和正常读数一样，而在修剪均值里没有。
    """
    if not values:
        raise ValueError("没有样本")
    ordered = sorted(values)
    cut = int(len(ordered) * trim)
    core = ordered[cut:len(ordered) - cut] or ordered
    return sum(core) / len(core)


@dataclass(frozen=True)
class LinearFit:
    slope: float
    intercept: float
    r_squared: float

    def __call__(self, x: float) -> float:
        return self.slope * x + self.intercept


def linear_regression(xs: list[float], ys: list[float]) -> LinearFit:
    if len(xs) != len(ys) or len(xs) < 2:
        raise ValueError("至少两个点，且 x/y 等长")
    n = len(xs)
    mean_x = sum(xs) / n
    mean_y = sum(ys) / n
    sxx = sum((x - mean_x) ** 2 for x in xs)
    if sxx < 1e-15:
        raise ValueError("所有 x 都相同，定不出斜率")
    sxy = sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys))
    slope = sxy / sxx
    intercept = mean_y - slope * mean_x
    syy = sum((y - mean_y) ** 2 for y in ys)
    r_squared = 1.0 if syy < 1e-15 else max(0.0, 1.0 - sum(
        (y - (slope * x + intercept)) ** 2 for x, y in zip(xs, ys)) / syy)
    return LinearFit(slope=slope, intercept=intercept, r_squared=r_squared)


@dataclass(frozen=True)
class ThrustCurve:
    """推力 vs 油门。二次拟合 `F = a·p² + b·p + c`。

    用二次而不是直线：桨推力大致正比于转速平方，而油门到转速大致线性，
    所以推力对油门本来就是二次的。用直线拟合会在两端各偏一截，
    而"两端偏"正好落在悬停点和满油这两个最要紧的地方。
    """

    a: float
    b: float
    c: float
    r_squared: float
    points: int

    def __call__(self, percent: float) -> float:
        return self.a * percent * percent + self.b * percent + self.c

    def newtons(self, percent: float) -> float:
        return self(percent) * GRAMS_TO_NEWTON


def fit_thrust_curve(measurements: list[Measurement]) -> ThrustCurve:
    usable = [m for m in measurements if m.direction == "up"]
    layers = {m.voltage_layer_v for m in usable if m.voltage_layer_v is not None}
    if len(layers) > 1:
        raise ValueError("不能把多个电压层混成一条油门曲线；请逐层拟合")
    if len(usable) < 3:
        raise ValueError("二次拟合至少要三个上行点")
    xs = [m.percent for m in usable]
    ys = [m.thrust_g for m in usable]

    # 正规方程解二次最小二乘。点数只有二三十个，不引 numpy。
    n = len(xs)
    s1 = sum(xs)
    s2 = sum(x ** 2 for x in xs)
    s3 = sum(x ** 3 for x in xs)
    s4 = sum(x ** 4 for x in xs)
    t0 = sum(ys)
    t1 = sum(x * y for x, y in zip(xs, ys))
    t2 = sum(x * x * y for x, y in zip(xs, ys))
    matrix = [[s4, s3, s2, t2], [s3, s2, s1, t1], [s2, s1, float(n), t0]]
    solution = _solve3(matrix)
    if solution is None:
        raise ValueError("二次拟合的正规方程奇异：油门档位太少或全都一样")
    a, b, c = solution
    mean_y = t0 / n
    syy = sum((y - mean_y) ** 2 for y in ys)
    residual = sum((y - (a * x * x + b * x + c)) ** 2 for x, y in zip(xs, ys))
    r_squared = 1.0 if syy < 1e-15 else max(0.0, 1.0 - residual / syy)
    return ThrustCurve(a=a, b=b, c=c, r_squared=r_squared, points=n)


def _solve3(matrix: list[list[float]]) -> tuple[float, float, float] | None:
    """3×4 增广矩阵的高斯消元（带部分主元）。"""
    rows = [row[:] for row in matrix]
    for column in range(3):
        pivot = max(range(column, 3), key=lambda r: abs(rows[r][column]))
        if abs(rows[pivot][column]) < 1e-15:
            return None
        rows[column], rows[pivot] = rows[pivot], rows[column]
        for target in range(3):
            if target == column:
                continue
            factor = rows[target][column] / rows[column][column]
            for index in range(column, 4):
                rows[target][index] -= factor * rows[column][index]
    return tuple(rows[index][3] / rows[index][index] for index in range(3))


def thrust_coefficient(thrust_g: float, rpm: float, diameter_m: float,
                       source: RpmSource,
                       air_density: float = AIR_DENSITY) -> float | None:
    """C_T = F / (ρ n² D⁴)，n 是**每秒**转数。

    转速不是实测的就返回 None——见模块头。
    """
    if not source.measured:
        return None
    if rpm is None or rpm <= 0.0 or diameter_m <= 0.0:
        return None
    n = rpm / 60.0
    return (thrust_g * GRAMS_TO_NEWTON) / (air_density * n * n * diameter_m ** 4)


@dataclass(frozen=True)
class VoltageModel:
    """`F(p, V) = F_ref(p) · (V/V_ref)^exponent`。

    指数由实测的电压层拟合出来，不写死 2：理论上推力 ∝ 转速² 而转速 ∝ 电压，
    但电调有限流、桨有失速、电池有内阻，实测指数常落在 1.6~2.2。
    写死 2 会在低电压端系统性高估推力——正好是电量快没的时候。
    """

    reference_voltage_v: float
    exponent: float
    r_squared: float

    def scale(self, voltage_v: float) -> float:
        if self.reference_voltage_v <= 0.0 or voltage_v <= 0.0:
            return 1.0
        return (voltage_v / self.reference_voltage_v) ** self.exponent


def fit_voltage_model(layers: dict[float, list[Measurement]],
                      percent: float) -> VoltageModel | None:
    """在某一档油门上，用各电压层的推力拟合指数。

    至少要两层；只有一层时返回 None 而不是假装指数是 2。
    """
    samples: list[tuple[float, float]] = []
    for layer_voltage, measurements in layers.items():
        matched = [m for m in measurements
                   if abs(m.percent - percent) < 1e-6 and m.direction == "up"]
        if not matched:
            continue
        thrust = trimmed_mean([m.thrust_g for m in matched])
        voltage = matched[0].voltage_v or layer_voltage
        if thrust > 0.0 and voltage > 0.0:
            samples.append((voltage, thrust))
    if len(samples) < 2:
        return None

    reference_voltage, reference_thrust = max(samples, key=lambda item: item[0])
    xs = [math.log(v / reference_voltage) for v, _ in samples]
    ys = [math.log(f / reference_thrust) for _, f in samples]
    fit = linear_regression(xs, ys)
    return VoltageModel(reference_voltage_v=reference_voltage,
                        exponent=fit.slope, r_squared=fit.r_squared)

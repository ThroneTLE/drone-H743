"""Pure experiment plans for the two installed coaxial propellers."""
from __future__ import annotations

from dataclasses import dataclass
from itertools import product


@dataclass(frozen=True)
class PlanPoint:
    segment_id: str
    upper_percent: float
    lower_percent: float
    phase: str = "steady"
    direction: str = "steady"
    settle_s: float = 1.2
    duration_s: float = 0.8
    voltage_layer_v: float | None = None
    voltage_layer_label: str = ""
    adaptive: bool = False
    max_wait_s: float = 0.0
    stable_window_s: float = 0.0
    min_scale_updates: int = 3


@dataclass(frozen=True)
class ExperimentPlan:
    name: str
    mode: str
    points: tuple[PlanPoint, ...]
    reference_voltage_v: float | None = None
    voltage_tolerance_v: float = 0.0

    @property
    def max_percent(self) -> float:
        return max((max(p.upper_percent, p.lower_percent) for p in self.points), default=0.0)


def with_voltage_layer(plan: ExperimentPlan, *, label: str,
                       target_v: float) -> ExperimentPlan:
    """Tag one operator-confirmed battery layer; never treats target as measured V."""
    clean_label = label.strip()
    if not clean_label:
        raise ValueError("电压层标签不能为空")
    if not 0.0 < target_v < 100.0:
        raise ValueError("电压层目标中心必须是合理的正电压")
    return ExperimentPlan(
        name=f"{plan.name}-{clean_label}", mode=plan.mode,
        points=tuple(PlanPoint(
            segment_id=point.segment_id,
            upper_percent=point.upper_percent,
            lower_percent=point.lower_percent,
            phase=point.phase,
            direction=point.direction,
            settle_s=point.settle_s,
            duration_s=point.duration_s,
            voltage_layer_v=float(target_v),
            voltage_layer_label=clean_label,
            adaptive=point.adaptive,
            max_wait_s=point.max_wait_s,
            stable_window_s=point.stable_window_s,
            min_scale_updates=point.min_scale_updates,
        ) for point in plan.points),
        reference_voltage_v=plan.reference_voltage_v,
        voltage_tolerance_v=plan.voltage_tolerance_v,
    )


def _ladder(minimum: float, maximum: float, step: float) -> list[float]:
    if not 0 <= minimum <= maximum <= 100:
        raise ValueError("指令范围必须满足 0 <= min <= max <= 100")
    if step <= 0:
        raise ValueError("步长必须为正")
    values, value = [], minimum
    while value <= maximum + 1e-9:
        values.append(round(value, 6))
        value += step
    if values[-1] < maximum:
        values.append(maximum)
    return values


def single_prop(role: str, *, minimum: float, maximum: float, step: float,
                include_return: bool = True, settle_s: float = 1.2,
                duration_s: float = 0.8) -> ExperimentPlan:
    if role not in {"upper", "lower"}:
        raise ValueError("role 必须是 upper 或 lower")
    up = _ladder(minimum, maximum, step)
    sequence = [(v, "up") for v in up]
    if include_return:
        sequence += [(v, "down") for v in reversed(up[:-1])]
    points = []
    for index, (value, direction) in enumerate(sequence):
        upper, lower = (value, 0.0) if role == "upper" else (0.0, value)
        points.append(PlanPoint(f"{role}-{index:03d}", upper, lower, "steady",
                                direction, settle_s, duration_s))
    return ExperimentPlan(f"single-{role}", role, tuple(points))


def common_sweep(*, minimum: float, maximum: float, step: float,
                 include_return: bool = True, settle_s: float = 1.2,
                 duration_s: float = 0.8) -> ExperimentPlan:
    base = single_prop("upper", minimum=minimum, maximum=maximum, step=step,
                       include_return=include_return, settle_s=settle_s,
                       duration_s=duration_s)
    return ExperimentPlan("dual-common", "dual", tuple(
        PlanPoint(f"dual-{i:03d}", p.upper_percent, p.upper_percent,
                  p.phase, p.direction, p.settle_s, p.duration_s)
        for i, p in enumerate(base.points)))


def grid(*, upper_values: tuple[float, ...], lower_values: tuple[float, ...],
         settle_s: float = 1.2, duration_s: float = 0.8) -> ExperimentPlan:
    if not upper_values or not lower_values:
        raise ValueError("二维指令网格两轴都不能为空")
    values = tuple(product(upper_values, lower_values))
    if any(not 0 <= v <= 100 for pair in values for v in pair):
        raise ValueError("二维指令网格必须在 0..100%")
    return ExperimentPlan("dual-grid", "dual", tuple(
        PlanPoint(f"grid-{i:03d}", u, l, "steady", "steady", settle_s, duration_s)
        for i, (u, l) in enumerate(values)))


def dynamic_steps(*, baseline: tuple[float, float], axis: str,
                  delta: float, repeats: int = 3, hold_s: float = 1.0) -> ExperimentPlan:
    if axis not in {"upper", "lower"} or repeats < 1 or delta <= 0:
        raise ValueError("动态阶跃需要 upper/lower 轴、正增量和至少一次重复")
    high = list(baseline)
    high[0 if axis == "upper" else 1] += delta
    if any(not 0 <= v <= 100 for v in (*baseline, *high)):
        raise ValueError("动态阶跃超出 0..100%")
    points = [PlanPoint(f"dyn-{axis}-warmup", *baseline,
                        "dynamic", "steady", 0.0, hold_s)]
    for repeat in range(repeats):
        segment = f"dyn-{axis}-{repeat:02d}-rise"
        points.append(PlanPoint(segment, *baseline,
                                "dynamic", "up", 0.0, hold_s))
        points.append(PlanPoint(segment, *high,
                                "dynamic", "up", 0.0, hold_s))
        segment = f"dyn-{axis}-{repeat:02d}-fall"
        points.append(PlanPoint(segment, *high,
                                "dynamic", "down", 0.0, hold_s))
        points.append(PlanPoint(segment, *baseline,
                                "dynamic", "down", 0.0, hold_s))
    return ExperimentPlan(f"dynamic-{axis}", "dual", tuple(points))

"""Throttle-grid order for auto collection.

The model regresses thrust on *measured* eRPM and loaded voltage, so a grid
cell only needs one steady measurement; it never has to hit an exact eRPM.
Coverage is compared on a load-compensated voltage: at one state of charge a
high-throttle cell reads ~0.3 V lower than a low-throttle one, and comparing
raw loaded voltage would keep re-opening already measured cells.
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

import numpy as np

MIN_COMMAND_PCT = 2.0
MIN_GRID_STEP_PCT = 5.0
MAX_GRID_TIERS = 3
CHARGE_BAND_V = 0.30  # Author-approved voltage granularity (2026-09-23).
SAG_K_DEFAULT = 0.0035  # V per (1e4 eRPM)^3; 2026-09-22/23 bench runs gave 0.0030..0.0042.
SAG_K_LIMITS = (0.001, 0.012)
_SAG_FIT_MIN_POINTS = 8
_SAG_FIT_MIN_LOAD_SPAN = 20.0
_STALE_BIN_V = 0.05


@dataclass(frozen=True)
class GridCell:
    upper_pct: float
    lower_pct: float
    tier: int


def build_command_grid(max_percent: float) -> tuple[list[GridCell], float]:
    """Nested grid: tier 0 is 5x5 incl. corners, finer tiers halve the step."""
    high = float(max_percent)
    low = min(MIN_COMMAND_PCT, high)
    span = high - low
    if span <= 0:
        return [GridCell(high, high, 0)], 0.0
    tiers = 1
    while tiers < MAX_GRID_TIERS and span / (4 * 2 ** tiers) >= MIN_GRID_STEP_PCT:
        tiers += 1
    intervals = 4 * 2 ** (tiers - 1)
    step = span / intervals
    cells = []
    for i in range(intervals + 1):
        for j in range(intervals + 1):
            tier = next(t for t in range(tiers)
                        if i % 2 ** (tiers - 1 - t) == 0 and j % 2 ** (tiers - 1 - t) == 0)
            # 0.01 % half-up, the board's command resolution (see acquisition SET).
            cells.append(GridCell(math.floor((low + i * step) * 100 + 0.5) / 100,
                                  math.floor((low + j * step) * 100 + 0.5) / 100, tier))
    return cells, step


def load_index(upper_erpm: float, lower_erpm: float) -> float:
    """Propeller power grows ~w^3, so battery sag tracks this index."""
    return (float(upper_erpm) / 1e4) ** 3 + (float(lower_erpm) / 1e4) ** 3


class ChargeEstimator:
    """Loaded voltage plus estimated sag, i.e. a state-of-charge voltage."""

    def __init__(self, k: float = SAG_K_DEFAULT) -> None:
        self.k = float(k)
        self._history: deque[tuple[float, float, float]] = deque(maxlen=60)

    def compensated(self, voltage_v: float, upper_erpm: float, lower_erpm: float) -> float:
        return float(voltage_v) + self.k * load_index(upper_erpm, lower_erpm)

    def observe(self, time_s: float, voltage_v: float, upper_erpm: float, lower_erpm: float) -> None:
        """Refit k from this battery's own steady points: V = a + b*t - k*L."""
        self._history.append((float(time_s), float(voltage_v), load_index(upper_erpm, lower_erpm)))
        if len(self._history) < _SAG_FIT_MIN_POINTS:
            return
        t, v, load = (np.asarray(column) for column in zip(*self._history))
        if float(np.ptp(load)) < _SAG_FIT_MIN_LOAD_SPAN:
            return
        design = np.column_stack((np.ones_like(t), t - t[0], -load))
        coefficients, *_ = np.linalg.lstsq(design, v, rcond=None)
        if math.isfinite(float(coefficients[2])):
            self.k = min(SAG_K_LIMITS[1], max(SAG_K_LIMITS[0], float(coefficients[2])))


class CoverageSchedule:
    """Picks the next throttle cell: gaps at this charge first, never idle."""

    def __init__(self, max_percent: float, covered_points=(), estimator: ChargeEstimator | None = None) -> None:
        self.cells, self.step = build_command_grid(max_percent)
        self.estimator = estimator or ChargeEstimator()
        self._low = self.cells[0].upper_pct
        self._intervals = round((self.cells[-1].upper_pct - self._low) / self.step) if self.step else 0
        self._points: list[list[tuple[float, float]]] = [[] for _ in self.cells]  # (loaded V, load index)
        self._failed: dict[int, float] = {}
        for point in covered_points:
            self.add_point(point)

    @property
    def tolerance_v(self) -> float:
        return CHARGE_BAND_V / 2

    def _index(self, upper_pct: float, lower_pct: float) -> int | None:
        if not self.step:
            return 0
        indices = []
        for value in (upper_pct, lower_pct):
            index = round((float(value) - self._low) / self.step)
            if not 0 <= index <= self._intervals:
                return None
            indices.append(index)
        return indices[0] * (self._intervals + 1) + indices[1]

    def add_point(self, point) -> bool:
        try:
            values = [float(point[name]) for name in (
                "upper_command_pct", "lower_command_pct", "voltage_v", "upper_erpm", "lower_erpm")]
        except (KeyError, TypeError, ValueError):
            return False
        if not all(math.isfinite(value) for value in values) or min(values[3], values[4]) <= 0:
            return False
        index = self._index(values[0], values[1])
        if index is None:
            return False
        self._points[index].append((values[2], load_index(values[3], values[4])))
        return True

    def _gap(self, index: int, charge_v: float) -> float:
        k = self.estimator.k
        return min((abs(voltage + k * load - charge_v) for voltage, load in self._points[index]),
                   default=math.inf)

    def covered(self, cell: GridCell, charge_v: float) -> bool:
        index = self._index(cell.upper_pct, cell.lower_pct)
        return index is not None and self._gap(index, charge_v) <= self.tolerance_v

    def missing_count(self, charge_v: float) -> int:
        return sum(self._gap(index, charge_v) > self.tolerance_v for index in range(len(self.cells)))

    def mark_failed(self, cell: GridCell, charge_v: float) -> None:
        """Skip a cell that would not settle until the charge moves one band."""
        index = self._index(cell.upper_pct, cell.lower_pct)
        if index is not None:
            self._failed[index] = float(charge_v)

    def next_cell(self, charge_v: float, current: tuple[float, float]) -> GridCell | None:
        best = best_key = None
        for index, cell in enumerate(self.cells):
            failed = self._failed.get(index)
            if failed is not None and abs(failed - charge_v) <= self.tolerance_v:
                continue
            gap = self._gap(index, charge_v)
            distance = max(abs(cell.upper_pct - current[0]), abs(cell.lower_pct - current[1]))
            if gap > self.tolerance_v:
                key = (0, cell.tier, 0, distance)
            else:
                # Band already complete: refresh the cells measured longest ago.
                key = (1, -int(gap / _STALE_BIN_V), cell.tier, distance)
            if best_key is None or key < best_key:
                best, best_key = cell, key
        return best

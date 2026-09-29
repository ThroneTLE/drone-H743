"""Thrust lookup tables: grid lookup with bilinear interpolation.

Two tables fitted to the post-KV steady points:
  speed table:    thrust = T(upper eRPM, lower eRPM); battery charge barely matters at a fixed speed
  throttle table: thrust = T(upper e, lower e), e = throttle % x charge V / 12 (effective motor voltage)
Node values are least squares with a small second-difference penalty, so every node follows
the measurements around it and new sessions refine the cells they cover. Fixed grid plus
bilinear reads port to firmware unchanged. Outside the measured convex hull a query returns
None. Voltage is folded into the throttle axis, so the throttle table may reach 0.75 V below
the lowest measured charge (checked on every build by predicting the lowest band from the rest).
"""
from __future__ import annotations

import json
from datetime import datetime
from pathlib import Path
from typing import Any
from uuid import uuid4

import numpy as np

from .sweep_schedule import SAG_K_DEFAULT
from .throttle_model import CV_BLOCKS, MIN_POINTS, V_REF, _metrics, held_out_residuals

SCHEMA = "thrust_lut_v1"
SPEED_STEP_ERPM = 5000.0
SPEED_MAX_ERPM = 80000.0
EFFECTIVE_STEP_PCT = 5.0
EFFECTIVE_MAX_PCT = 110.0
SMOOTHING = 0.1  # 043627 time-block hold-out: 5.7 g (speed) / 6.5 g (throttle); 0.03-0.3 within 1 g
MIN_NODE_SUPPORT = 0.1  # summed bilinear weight: below it a node is set by smoothing, not by data
EXTRAPOLATE_BELOW_V = 0.75  # 043627: fit >= 11.5 V, predict 10.75-11.5 V -> RMSE 7.4 g, no bias
# A fresh pack read 12.60-12.65 V on 2026-09-24 and was refused. 043627: fit <= 12.28 V,
# predict the top 0.25 V -> RMSE 11.5 g, max 25 g.
EXTRAPOLATE_ABOVE_V = 0.25
FULL_CHARGE_V = 12.6  # 3S x 4.2 V
DIAG_STEP_PCT = 0.5
# Pair allocation shifts both rotors together; 043627 needed at most 3.5 % (1050 g, 30 % split),
# so a wrong table value can never move the throttles further than this.
PAIR_SHIFT_LIMIT_PCT = 10.0
PAIR_BISECTION_STEPS = 24
_EDGE = 1e-9
_SPEED_AXIS = np.arange(0.0, SPEED_MAX_ERPM + 1.0, SPEED_STEP_ERPM)
_EFFECTIVE_AXIS = np.arange(0.0, EFFECTIVE_MAX_PCT + 1e-6, EFFECTIVE_STEP_PCT)


def _weights(axes, xy):
    """Sparse bilinear weights: row i spreads point i over its cell's four nodes (row-major y, x)."""
    from scipy.sparse import coo_matrix
    ax, ay = (np.asarray(axis, float) for axis in axes)
    x, y = np.asarray(xy, float).T
    ix = np.clip(np.searchsorted(ax, x, side="right") - 1, 0, len(ax) - 2)
    iy = np.clip(np.searchsorted(ay, y, side="right") - 1, 0, len(ay) - 2)
    fx = (x - ax[ix]) / (ax[ix + 1] - ax[ix])
    fy = (y - ay[iy]) / (ay[iy + 1] - ay[iy])
    rows, cols, values = [], [], []
    for dy, wy in ((0, 1 - fy), (1, fy)):
        for dx, wx in ((0, 1 - fx), (1, fx)):
            rows.append(np.arange(len(x))); cols.append((iy + dy) * len(ax) + ix + dx); values.append(wx * wy)
    return coo_matrix((np.concatenate(values), (np.concatenate(rows), np.concatenate(cols))),
                      shape=(len(x), len(ax) * len(ay))).tocsr()


def _smoothness(nx: int, ny: int):
    from scipy.sparse import coo_matrix, vstack
    index = np.arange(nx * ny).reshape(ny, nx)
    blocks = []
    for a, b, c in ((index[:, :-2], index[:, 1:-1], index[:, 2:]), (index[:-2, :], index[1:-1, :], index[2:, :])):
        count = a.size
        blocks.append(coo_matrix((np.tile([1.0, -2.0, 1.0], count),
                                  (np.repeat(np.arange(count), 3), np.column_stack((a.ravel(), b.ravel(), c.ravel())).ravel())),
                                 shape=(count, nx * ny)))
    return vstack(blocks).tocsr()


def _fit_nodes(axes, xy, thrust) -> np.ndarray:
    from scipy.sparse import vstack
    from scipy.sparse.linalg import lsqr
    penalty = _smoothness(len(axes[0]), len(axes[1]))
    system = vstack([_weights(axes, xy), SMOOTHING * penalty])
    target = np.concatenate([np.asarray(thrust, float), np.zeros(penalty.shape[0])])
    return lsqr(system, target, atol=1e-12, btol=1e-12, iter_lim=50000)[0].reshape(len(axes[1]), len(axes[0]))


def _hull(xy) -> list[list[float]]:
    from scipy.spatial import ConvexHull
    points = np.asarray(xy, float)
    return points[ConvexHull(points).vertices].tolist()  # counter-clockwise


def _inside(hull, x: float, y: float) -> bool:
    vertices = np.asarray(hull, float)
    edges = np.roll(vertices, -1, axis=0) - vertices
    cross = edges[:, 0] * (y - vertices[:, 1]) - edges[:, 1] * (x - vertices[:, 0])
    return bool(np.all(cross >= -_EDGE * max(1.0, float(np.abs(vertices).max())) ** 2))


def _read(table: dict[str, Any], x: float, y: float) -> float | None:
    if not (np.isfinite(x) and np.isfinite(y)) or not _inside(table["hull"], x, y):
        return None
    ax, ay = (np.asarray(axis, float) for axis in table["axes"])
    values = np.asarray(table["values"], float)
    ix = int(np.clip(np.searchsorted(ax, x, side="right") - 1, 0, len(ax) - 2))
    iy = int(np.clip(np.searchsorted(ay, y, side="right") - 1, 0, len(ay) - 2))
    fx = (x - ax[ix]) / (ax[ix + 1] - ax[ix]); fy = (y - ay[iy]) / (ay[iy + 1] - ay[iy])
    return float((1 - fy) * ((1 - fx) * values[iy, ix] + fx * values[iy, ix + 1])
                 + fy * ((1 - fx) * values[iy + 1, ix] + fx * values[iy + 1, ix + 1]))


def _held_out(axes, xy, thrust) -> np.ndarray:
    """Prediction minus measurement; contiguous time blocks as in the throttle model."""
    xy = np.asarray(xy, float); thrust = np.asarray(thrust, float)
    blocks = np.minimum(np.arange(len(thrust)) * CV_BLOCKS // len(thrust), CV_BLOCKS - 1)
    error = np.empty(len(thrust))
    for block in range(CV_BLOCKS):
        train = blocks != block
        nodes = _fit_nodes(axes, xy[train], thrust[train])
        error[~train] = _weights(axes, xy[~train]) @ nodes.ravel() - thrust[~train]
    return error


def _table(name: str, unit: str, axes, xy, thrust) -> dict[str, Any]:
    xy = np.asarray(xy, float)
    nodes = _fit_nodes(axes, xy, thrust)
    hull = _hull(xy)
    ax, ay = (np.asarray(axis, float) for axis in axes)
    support = np.asarray(_weights(axes, xy).sum(axis=0)).reshape(len(ay), len(ax))
    unsupported = [[float(x), float(y)] for j, y in enumerate(ay) for i, x in enumerate(ax)
                   if support[j, i] < MIN_NODE_SUPPORT and _inside(hull, x, y)]
    return {"name": name, "unit": unit, "axes": [ax.tolist(), ay.tolist()],
            "values": np.round(nodes, 2).tolist(), "hull": hull, "node_support": np.round(support, 3).tolist(),
            "unsupported_nodes": unsupported,
            "held_out_time_blocks": {"blocks": CV_BLOCKS, **_metrics(_held_out(axes, xy, thrust))}}


def _effective(pct, charge_v):
    return np.asarray(pct, float) * np.asarray(charge_v, float) / V_REF


def _coordinates(points: list[dict[str, Any]]):
    column = lambda name: np.array([p[name] for p in points], float)
    charge = column("charge_v")
    return (np.column_stack((column("upper_erpm"), column("lower_erpm"))),
            np.column_stack((_effective(column("upper_command_pct"), charge), _effective(column("lower_command_pct"), charge))),
            column("thrust_gf"))


def held_out_errors(points: list[dict[str, Any]]) -> dict[str, np.ndarray]:
    """Per-point held-out prediction minus measurement of both tables, in point order."""
    speed_xy, throttle_xy, thrust = _coordinates(points)
    return {"speed": _held_out((_SPEED_AXIS, _SPEED_AXIS), speed_xy, thrust),
            "throttle": _held_out((_EFFECTIVE_AXIS, _EFFECTIVE_AXIS), throttle_xy, thrust)}


def build(points: list[dict[str, Any]], sources: list[dict[str, Any]] | None = None,
          rejected: list[dict[str, Any]] | None = None) -> dict[str, Any]:
    if len(points) < MIN_POINTS:
        raise ValueError(f"KV 修改后的稳态点只有 {len(points)} 个，至少需要 {MIN_POINTS} 个")
    column = lambda name: np.array([p[name] for p in points], float)
    upper, lower, charge = column("upper_command_pct"), column("lower_command_pct"), column("charge_v")
    speed_xy, throttle_xy, thrust = _coordinates(points)
    speed = _table("speed", "eRPM", (_SPEED_AXIS, _SPEED_AXIS), speed_xy, thrust)
    throttle = _table("throttle", "throttle% x charge_v / 12", (_EFFECTIVE_AXIS, _EFFECTIVE_AXIS), throttle_xy, thrust)
    return {"schema": SCHEMA, "model_id": uuid4().hex[:12], "created_at": datetime.now().astimezone().isoformat(),
            "interpolation": "bilinear on a fixed grid; values in grams-force; outside hull -> None",
            "smoothing": SMOOTHING, "v_ref": V_REF, "speed": speed, "throttle": throttle,
            "charge_voltage": {"definition": "loaded V + k*sum((eRPM/1e4)^3)", "k": SAG_K_DEFAULT},
            "domain": {"charge_v": [float(charge.min()), float(charge.max())],
                       "charge_v_usable": usable_range(float(charge.min()), float(charge.max())),
                       "command_pct": [float(min(upper.min(), lower.min())), float(max(upper.max(), lower.max()))]},
            "training": {"points": len(points), "sources": sources or [], "rejected_sessions": rejected or [],
                         "command_levels_pct": sorted({round(float(v), 1) for v in np.concatenate((upper, lower))}),
                         "reference_polynomial_held_out": _metrics(held_out_residuals(points)),
                         "voltage_extrapolation_check": _extrapolation_check(points),
                         "voltage_extrapolation_check_above": _extrapolation_check(points, above=True)}}


def usable_range(low_v: float, high_v: float) -> list[float]:
    return [low_v - EXTRAPOLATE_BELOW_V, max(high_v + EXTRAPOLATE_ABOVE_V, FULL_CHARGE_V)]


def _extrapolation_check(points: list[dict[str, Any]], *, above: bool = False) -> dict[str, Any] | None:
    """Fit without the outermost band (lowest EXTRAPOLATE_BELOW_V or highest EXTRAPOLATE_ABOVE_V), predict it."""
    charge = np.array([p["charge_v"] for p in points], float)
    if above:
        cut = float(charge.max()) - EXTRAPOLATE_ABOVE_V
        train = [p for p, v in zip(points, charge) if v <= cut]
        test = [p for p, v in zip(points, charge) if v > cut]
    else:
        cut = float(charge.min()) + EXTRAPOLATE_BELOW_V
        train = [p for p, v in zip(points, charge) if v >= cut]
        test = [p for p, v in zip(points, charge) if v < cut]
    if len(train) < MIN_POINTS or len(test) < 10:
        return None
    _speed_xy, throttle_xy, thrust = _coordinates(train)
    table = {"hull": _hull(throttle_xy), "axes": [_EFFECTIVE_AXIS.tolist()] * 2,
             "values": _fit_nodes((_EFFECTIVE_AXIS, _EFFECTIVE_AXIS), throttle_xy, thrust).tolist()}
    _unused, test_xy, test_thrust = _coordinates(test)
    error = [value - measured for (x, y), measured in zip(test_xy, test_thrust)
             if (value := _read(table, float(x), float(y))) is not None]
    if not error:
        return None
    return {"trained_from_v": cut, "tested_points": len(error), **_metrics(np.asarray(error))}


def usable_charge(lut: dict[str, Any], charge_v: float) -> bool:
    low_v, high_v = lut["domain"].get("charge_v_usable", lut["domain"]["charge_v"])
    return low_v <= charge_v <= high_v


def is_extrapolated(lut: dict[str, Any], charge_v: float) -> bool:
    low_v, high_v = lut["domain"]["charge_v"]
    return usable_charge(lut, charge_v) and not low_v <= charge_v <= high_v


def thrust_from_speed(lut: dict[str, Any], upper_erpm: float, lower_erpm: float) -> float | None:
    return _read(lut["speed"], float(upper_erpm), float(lower_erpm))


def predict_gf(lut: dict[str, Any], upper_pct: float, lower_pct: float, charge_v: float) -> float | None:
    low_c, high_c = lut["domain"]["command_pct"]
    if not (usable_charge(lut, charge_v) and low_c <= upper_pct <= high_c and low_c <= lower_pct <= high_c):
        return None
    return _read(lut["throttle"], float(_effective(upper_pct, charge_v)), float(_effective(lower_pct, charge_v)))


def _invert(read, grid: np.ndarray, thrust_gf: float) -> float | None:
    """Monotone envelope over the reachable part of a line, then linear interpolation."""
    values = np.array([read(value) for value in grid], dtype=float)
    reachable = np.isfinite(values)
    if not reachable.any():
        return None
    first = int(np.argmax(reachable)); last = len(values) - int(np.argmax(reachable[::-1]))
    if not reachable[first:last].all():
        return None  # A hole in the hull on this line; the convex hull never produces one.
    grid, envelope = grid[first:last], np.maximum.accumulate(values[first:last])
    if not envelope[0] <= thrust_gf <= envelope[-1]:
        return None
    index = int(np.searchsorted(envelope, thrust_gf))
    if index == 0:
        return float(grid[0])
    left, right = envelope[index - 1], envelope[index]
    fraction = 0.0 if right == left else (thrust_gf - left) / (right - left)
    return float(grid[index - 1] + fraction * (grid[index] - grid[index - 1]))


def balanced_throttle(lut: dict[str, Any], thrust_gf: float, charge_v: float) -> float | None:
    """Both rotors at one throttle; None when the target is unreachable at this charge."""
    low_c, high_c = lut["domain"]["command_pct"]
    return _invert(lambda pct: predict_gf(lut, pct, pct, charge_v), np.linspace(low_c, high_c, 801), thrust_gf)


def balanced_speed(lut: dict[str, Any], thrust_gf: float) -> float | None:
    """Both rotors at one eRPM (for a future speed loop); None when not in the measured range."""
    axis = lut["speed"]["axes"][0]
    return _invert(lambda erpm: thrust_from_speed(lut, erpm, erpm), np.linspace(axis[0], axis[-1], 801), thrust_gf)


def balanced_diagonal(lut: dict[str, Any]) -> tuple[list[float], list[float]]:
    """Measured stretch of the same-throttle line, thrust made non-decreasing for the inverse.

    The firmware stores exactly these samples (thrust_lut_export), so both sides invert alike.
    """
    table = lut["throttle"]
    grid = np.arange(0.0, table["axes"][0][-1] + 1e-9, DIAG_STEP_PCT)
    values = [_read(table, float(e), float(e)) for e in grid]
    inside = [i for i, value in enumerate(values) if value is not None]
    if not inside or inside != list(range(inside[0], inside[-1] + 1)):
        raise ValueError("同油门线不在实测范围内或不连续")
    effective = grid[inside[0]:inside[-1] + 1]
    thrust = np.maximum.accumulate(np.array(values[inside[0]:inside[-1] + 1], float))
    return [round(float(e), 3) for e in effective], [round(float(t), 3) for t in np.maximum(thrust, 0.0)]


def _diag_effective(diag: tuple[list[float], list[float]], total_g: float) -> float:
    """Same-throttle inverse as DRV_ThrustLut_BalancedEffectiveForThrust."""
    effective, thrust = diag
    if not total_g > 0.0:
        return 0.0
    if total_g <= thrust[0]:
        return effective[0] * total_g / thrust[0] if thrust[0] > 0.0 else effective[0]
    for i in range(1, len(thrust)):
        if total_g <= thrust[i]:
            rise = thrust[i] - thrust[i - 1]
            ratio = (total_g - thrust[i - 1]) / rise if rise > 0.0 else 0.0
            return effective[i - 1] + ratio * (effective[i] - effective[i - 1])
    return effective[-1]


def _grid_read(table: dict[str, Any], x: float, y: float) -> float:
    """Clamped bilinear read as DRV_ThrustLut_ThrustFromEffective (no hull: the firmware must answer)."""
    ax, ay = table["axes"]
    values = table["values"]

    def cell(value: float, axis: list[float]) -> tuple[int, float]:
        position = min(max((value - axis[0]) / (axis[1] - axis[0]), 0.0), len(axis) - 1.0)
        index = min(int(position), len(axis) - 2)
        return index, position - index

    ix, fx = cell(x, ax)
    iy, fy = cell(y, ay)
    value = ((1 - fy) * ((1 - fx) * values[iy][ix] + fx * values[iy][ix + 1])
             + fy * ((1 - fx) * values[iy + 1][ix] + fx * values[iy + 1][ix + 1]))
    return max(0.0, value)


def pair_shift(table: dict[str, Any], e_upper: float, e_lower: float, total_g: float, e_max: float,
               floor_e: float) -> float:
    """Common shift making the 2D table total equal total_g; mirrors DRV_ThrustLut_PairShift.

    A saturated rotor (beyond e_max) or one below the lowest measured throttle gets no correction:
    lowering both to fit a saturated rotor would take thrust away from the other.
    """
    low_e, high_e = min(e_upper, e_lower), max(e_upper, e_lower)
    if not total_g > 0.0 or low_e < floor_e or high_e > e_max:
        return 0.0
    lo = max(floor_e - low_e, -PAIR_SHIFT_LIMIT_PCT)
    hi = min(e_max - high_e, PAIR_SHIFT_LIMIT_PCT)
    if _grid_read(table, e_upper + hi, e_lower + hi) < total_g:
        return hi
    if _grid_read(table, e_upper + lo, e_lower + lo) >= total_g:
        return lo
    for _ in range(PAIR_BISECTION_STEPS):
        mid = 0.5 * (lo + hi)
        if _grid_read(table, e_upper + mid, e_lower + mid) < total_g:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def pair_throttle(lut: dict[str, Any], upper_gf: float, lower_gf: float, charge_v: float, *,
                  correct: bool = True, diag: tuple[list[float], list[float]] | None = None) -> dict[str, float] | None:
    """Controller split (per-rotor thrust, same-throttle-half convention) -> both throttles, as the firmware.

    Each rotor first goes through the same-throttle line (the convention the flight controller always
    used); with correct=True both are then shifted together so the 2D table total equals the command.
    The throttle difference the yaw loop asked for is kept.
    """
    if not usable_charge(lut, charge_v):
        return None
    low_v, high_v = lut["domain"].get("charge_v_usable", lut["domain"]["charge_v"])
    voltage = min(max(charge_v, low_v), high_v)
    diag = diag or balanced_diagonal(lut)
    e_upper = _diag_effective(diag, 2.0 * upper_gf)
    e_lower = _diag_effective(diag, 2.0 * lower_gf)
    shift = (pair_shift(lut["throttle"], e_upper, e_lower, upper_gf + lower_gf, 100.0 * voltage / V_REF, diag[0][0])
             if correct else 0.0)
    percent = lambda thrust, e: min(max((e + shift) * V_REF / voltage, 0.0), 100.0) if thrust > 0.0 else 0.0
    return {"upper_pct": percent(upper_gf, e_upper), "lower_pct": percent(lower_gf, e_lower), "shift_pct": shift,
            "e_upper": e_upper, "e_lower": e_lower}


def save(lut: dict[str, Any], root: Path) -> Path:
    stamp = datetime.now().astimezone()
    folder = Path(root) / "models" / "lut" / stamp.date().isoformat() / f"{stamp:%H%M%S}-{lut['model_id']}"
    folder.mkdir(parents=True, exist_ok=False)
    path = folder / "thrust_lut.json"
    path.write_text(json.dumps(lut, ensure_ascii=False, indent=1) + "\n", encoding="utf-8")
    return path


CURRENT_FILE = "current.json"


def current(root: Path) -> tuple[dict[str, Any], Path] | None:
    """The table the flight controller runs (written by thrust_lut_export), not merely the newest build."""
    pointer = Path(root) / "models" / "lut" / CURRENT_FILE
    if not pointer.is_file():
        return None
    record = json.loads(pointer.read_text(encoding="utf-8"))
    path = Path(record["path"])
    if not path.is_absolute():
        path = Path(__file__).resolve().parents[2] / path  # recorded relative to the repository root
    if not path.is_file():
        return None
    return json.loads(path.read_text(encoding="utf-8")), path


def set_current(root: Path, lut: dict[str, Any], source: str, firmware_table: str, exported_at: str) -> Path:
    pointer = Path(root) / "models" / "lut" / CURRENT_FILE
    pointer.parent.mkdir(parents=True, exist_ok=True)
    pointer.write_text(json.dumps({
        "model_id": lut["model_id"], "path": source, "firmware_table": firmware_table, "exported_at": exported_at,
        "meaning": "飞控当前使用的推力查补表；models/lut 下其他目录是历史或候选。换表：验证通过后运行 "
                   "python -m tools.thrust_bench.thrust_lut_export <thrust_lut.json>，再编译烧录。"},
        ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return pointer


def latest(root: Path) -> tuple[dict[str, Any], Path] | None:
    """Newest build: a candidate for bench validation, not necessarily what the flight controller runs."""
    paths = sorted((Path(root) / "models" / "lut").glob("*/*/thrust_lut.json"))
    if not paths:
        return None
    return json.loads(paths[-1].read_text(encoding="utf-8")), paths[-1]


def summary(lut: dict[str, Any]) -> str:
    speed, throttle = lut["speed"]["held_out_time_blocks"], lut["throttle"]["held_out_time_blocks"]
    reference = lut["training"]["reference_polynomial_held_out"]
    low_v, high_v = lut["domain"]["charge_v"]
    use_low, use_high = lut["domain"].get("charge_v_usable", (low_v, high_v))
    check = lut["training"].get("voltage_extrapolation_check")
    above = lut["training"].get("voltage_extrapolation_check_above")
    reach = (f"；按电压外推可用 {use_low:.2f}～{use_high:.2f} V（用 {check['trained_from_v']:.2f} V 以上建表、"
             f"预测更低电量：均方根 {check['rmse_gf']:.1f} 克，最大 {check['max_abs_gf']:.0f} 克"
             + (f"；去掉最高 {EXTRAPOLATE_ABOVE_V:g} V 预测它：均方根 {above['rmse_gf']:.1f} 克" if above else "") + "）"
             if check else "")
    rejected = lut["training"].get("rejected_sessions") or []
    skipped = ("；未用 " + "、".join(f"{item['session'].split('/')[-1]}（{item['reason']}）" for item in rejected)
               if rejected else "")
    levels = lut["training"]["command_levels_pct"]
    grid_note = (f"实测只在 {len(levels)} 档油门（{levels[0]:g}～{levels[-1]:g}%）的网格上，档与档之间是表格插值，需实测检验"
                 if len(levels) <= 20 else f"实测覆盖 {len(levels)} 档油门")
    return (f"查补表已建立：{len(lut['training']['sources'])} 个会话、{lut['training']['points']} 个稳态点{skipped}。"
            f"按时间段留出验证——转速表（每 {SPEED_STEP_ERPM / 1000:g}k eRPM 一格）均方根 {speed['rmse_gf']:.1f} 克、"
            f"最大 {speed['max_abs_gf']:.0f} 克；油门表（油门×电量电压，每 {EFFECTIVE_STEP_PCT:g}% 一格）"
            f"均方根 {throttle['rmse_gf']:.1f} 克、最大 {throttle['max_abs_gf']:.0f} 克"
            f"（原多项式 {reference['rmse_gf']:.1f} 克）。{grid_note}；无实测支撑的表格节点：转速表 "
            f"{len(lut['speed']['unsupported_nodes'])} 个、油门表 {len(lut['throttle']['unsupported_nodes'])} 个，是后续补测对象。"
            f"实测电量电压 {low_v:.2f}～{high_v:.2f} V{reach}，再往外不查。")


def write_figure(lut: dict[str, Any], points: list[dict[str, Any]], folder: Path) -> Path | None:
    """Both tables as filled contours inside their hull, measured points, and empty cells."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.colors import LinearSegmentedColormap
        from matplotlib.lines import Line2D
        from matplotlib.path import Path as MplPath
        from scipy.interpolate import RegularGridInterpolator
    except ImportError:
        return None
    plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "Noto Sans CJK SC", "DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False
    surface, ink, muted, grid_ink = "#fcfcfb", "#52514e", "#898781", "#e1e0d9"
    ramp = LinearSegmentedColormap.from_list("blue", ("#dcebfb", "#86b6ef", "#2a78d6", "#0d366b"))
    column = lambda name: np.array([p[name] for p in points], float)
    charge = column("charge_v")
    coordinates = {"speed": (column("upper_erpm") / 1e3, column("lower_erpm") / 1e3, 1e3,
                             "上桨电转速 [k eRPM]", "下桨电转速 [k eRPM]", "转速查补表"),
                   "throttle": (_effective(column("upper_command_pct"), charge), _effective(column("lower_command_pct"), charge),
                                1.0, "上桨等效油门 [油门%×电量V/12]", "下桨等效油门 [油门%×电量V/12]", "油门查补表（电压已折入坐标）")}
    fig, axes = plt.subplots(1, 2, figsize=(13, 5.8))
    fig.patch.set_facecolor(surface)
    for axis, key in zip(axes, ("speed", "throttle")):
        table = lut[key]; px, py, scale, xlabel, ylabel, title = coordinates[key]
        ax, ay = (np.asarray(v, float) for v in table["axes"])
        fine_x = np.linspace(ax[0], ax[-1], 240); fine_y = np.linspace(ay[0], ay[-1], 240)
        mesh_x, mesh_y = np.meshgrid(fine_x, fine_y)
        mesh = np.column_stack((mesh_x.ravel(), mesh_y.ravel()))
        grid = RegularGridInterpolator((ay, ax), np.asarray(table["values"], float))(mesh[:, ::-1])  # bilinear, as _read
        grid[~MplPath(np.asarray(table["hull"], float)).contains_points(mesh, radius=1e-6)] = np.nan
        grid = grid.reshape(mesh_x.shape)
        filled = axis.contourf(fine_x / scale, fine_y / scale, grid, levels=np.arange(0, 2001, 125), cmap=ramp)
        lines = axis.contour(fine_x / scale, fine_y / scale, grid, levels=np.arange(250, 2001, 250),
                             colors="#ffffff", linewidths=.7)
        axis.clabel(lines, fmt="%d 克", fontsize=7, colors="#ffffff")
        hull = np.asarray(table["hull"] + table["hull"][:1], float) / scale
        axis.plot(hull[:, 0], hull[:, 1], color=muted, linewidth=.8)
        axis.scatter(px, py, s=7, color="#0b0b0b", alpha=.55, linewidths=0, zorder=3)
        if table["unsupported_nodes"]:
            empty = np.asarray(table["unsupported_nodes"], float) / scale
            axis.scatter(empty[:, 0], empty[:, 1], marker="x", s=22, color="#d6453d", linewidths=1.1, zorder=4)
        held = table["held_out_time_blocks"]
        axis.set_title(f"{title}：留出均方根 {held['rmse_gf']:.1f} 克，最大 {held['max_abs_gf']:.0f} 克",
                       color="#0b0b0b", loc="left", fontsize=10)
        axis.set(xlabel=xlabel, ylabel=ylabel, facecolor=surface, aspect="equal")
        axis.grid(True, color=grid_ink, linewidth=.4)
        axis.tick_params(colors=muted, labelcolor=ink)
        axis.xaxis.label.set_color(ink); axis.yaxis.label.set_color(ink)
        for side in ("top", "right"):
            axis.spines[side].set_visible(False)
    fig.subplots_adjust(left=.06, right=.9, bottom=.16, top=.92, wspace=.18)
    bar = fig.colorbar(filled, cax=fig.add_axes((.925, .22, .012, .64)))
    bar.set_label("推力 [克]", color=ink); bar.ax.tick_params(colors=muted, labelcolor=ink)
    fig.legend(handles=[Line2D([], [], marker="o", linestyle="", color="#0b0b0b", alpha=.55, markersize=4, label="实测稳态点"),
                        Line2D([], [], marker="x", linestyle="", color="#d6453d", markersize=6, label="无实测支撑的表格节点（待验证/补测）"),
                        Line2D([], [], color=muted, linewidth=.8, label="实测范围（范围外不查）")],
               loc="lower left", ncol=3, frameon=False, labelcolor=ink, fontsize=8)
    path = Path(folder) / "thrust_lut.png"
    fig.savefig(path, dpi=150, facecolor=surface)
    plt.close(fig)
    return path

"""Throttle thrust model.

thrust = f(upper %, lower %, battery charge voltage), built to be inverted
(desired thrust -> throttle). Throttle data is only valid for one ESC setting:
before the AM32 KV change the same throttle produced far less thrust, so only
sessions recorded after the recorded KV write are used.

The polynomial model itself (models/throttle/*) is superseded by the lookup table in
thrust_lut.py: not used by the flight controller or by new bench validations. This module
still owns the shared data selection (load_points, charge_voltage, ESC KV record).
"""
from __future__ import annotations

import json
from datetime import datetime
from pathlib import Path
from typing import Any
from uuid import uuid4

import numpy as np

from .model import _steady_points
from .session import read_samples
from .sweep_schedule import SAG_K_DEFAULT, load_index

SCHEMA = "throttle_model_v1"
ESC_CONFIG_FILE = "esc_config.json"
V_REF = 12.0
MIN_POINTS = 40
CV_BLOCKS = 6
EXCLUDED_PLANS = ("model_validation", "split_validation")  # hardware checks stay independent test data
IDLE_COMMAND_PCT = 3.0
ZERO_TOLERANCE_GF = 10.0  # 2026-09-23 043415 read +28 g at 2 %/2 %: scale zero not set
GRAMS_PER_NEWTON = 1000.0 / 9.80665
_TERMS = ("1", "xu", "xl", "xu^2", "xl^2", "xu*xl", "xu^3", "xl^3", "xu^2*xl", "xu*xl^2")


def charge_voltage(voltage_v: float, upper_erpm: float, lower_erpm: float) -> float:
    return float(voltage_v) + SAG_K_DEFAULT * load_index(upper_erpm, lower_erpm)


def _design(upper_pct, lower_pct, charge_v) -> np.ndarray:
    xu = np.asarray(upper_pct, float) / 100.0 * np.asarray(charge_v, float) / V_REF
    xl = np.asarray(lower_pct, float) / 100.0 * np.asarray(charge_v, float) / V_REF
    one = np.ones_like(xu)
    return np.column_stack((one, xu, xl, xu ** 2, xl ** 2, xu * xl,
                            xu ** 3, xl ** 3, xu ** 2 * xl, xu * xl ** 2))


def record_esc_kv_write(root: Path, session_id: str, kv: int) -> Path:
    """Sessions strictly after `session_id` run with the new KV (ESC re-reads it on power-up)."""
    path = Path(root) / ESC_CONFIG_FILE
    path.write_text(json.dumps({
        "am32_motor_kv": int(kv), "written_in_session": session_id,
        "valid_sessions": "strictly after written_in_session (ESC power cycle required)",
        "written_at": datetime.now().astimezone().isoformat()}, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8")
    return path


def eligible_sessions(root: Path) -> tuple[dict[str, Any] | None, list[Path]]:
    root = Path(root)
    config_path = root / ESC_CONFIG_FILE
    if not config_path.is_file():
        return None, []
    config = json.loads(config_path.read_text(encoding="utf-8"))
    cutoff = str(config["written_in_session"])
    sessions = sorted(path for path in root.glob("????-??-??/*") if (path / "samples.csv").is_file())
    return config, [path for path in sessions if f"{path.parent.name}/{path.name}" > cutoff]


def load_points(root: Path) -> tuple[list[dict[str, Any]], list[dict[str, Any]], list[dict[str, Any]]]:
    """Steady dual points from eligible sessions whose scale zero checks out, oldest first."""
    _config, sessions = eligible_sessions(root)
    points, sources, rejected = [], [], []
    for session in sessions:
        metadata = json.loads((session / "metadata.json").read_text(encoding="utf-8"))
        plans = {str(run.get("run_id")): str(run.get("plan_name", "")) for run in metadata.get("runs", [])}
        samples = [sample for sample in read_samples(session / "samples.csv")
                   if plans.get(sample.run_id, "") not in EXCLUDED_PLANS]
        first_time: dict[str, float] = {}
        for sample in samples:
            first_time.setdefault(sample.segment_id, sample.host_time_s)
        steady, _ = _steady_points(samples, metadata)
        kept_runs: set[str] = set()
        session_points = []
        for point in steady:
            if point.get("mode") != "dual" or any(
                    point.get(name) is None for name in ("upper_command_pct", "lower_command_pct", "voltage_v")):
                continue
            if min(point["upper_erpm"], point["lower_erpm"]) <= 500:
                continue  # A stopped rotor is not a dual operating point.
            session_points.append({**point, "session": f"{session.parent.name}/{session.name}",
                                   "time_s": first_time.get(point["segment_id"], 0.0),
                                   "charge_v": charge_voltage(point["voltage_v"], point["upper_erpm"], point["lower_erpm"]),
                                   "thrust_gf": float(point["thrust_n"]) * GRAMS_PER_NEWTON})
            kept_runs.add(point["run_id"])
        if not session_points:
            continue
        name = f"{session.parent.name}/{session.name}"
        idle = [p["thrust_gf"] for p in session_points
                if max(p["upper_command_pct"], p["lower_command_pct"]) <= IDLE_COMMAND_PCT]
        if not idle:
            rejected.append({"session": name, "steady_points": len(session_points),
                             "reason": "无低油门点，无法核对称重零点"})
            continue
        zero = float(np.mean(idle))
        if abs(zero) > ZERO_TOLERANCE_GF:
            rejected.append({"session": name, "steady_points": len(session_points),
                             "reason": f"低油门推力 {zero:+.0f} 克，称重零点未去皮"})
            continue
        points.extend(session_points)
        sources.append({"session": name, "steady_points": len(session_points),
                        "idle_thrust_gf": zero, "runs": sorted(kept_runs)})
    points.sort(key=lambda point: (point["session"], point["time_s"]))
    return points, sources, rejected


def _metrics(error: np.ndarray) -> dict[str, float]:
    magnitude = np.abs(error)
    return {"rmse_gf": float(np.sqrt(np.mean(error ** 2))), "mae_gf": float(np.mean(magnitude)),
            "p95_abs_gf": float(np.percentile(magnitude, 95)), "max_abs_gf": float(magnitude.max()),
            "over_50gf": int((magnitude > 50.0).sum())}


def held_out_residuals(points: list[dict[str, Any]]) -> np.ndarray:
    """Prediction minus measurement, each point predicted by a fit that never saw its time block."""
    design = _design([p["upper_command_pct"] for p in points], [p["lower_command_pct"] for p in points],
                     [p["charge_v"] for p in points])
    thrust = np.array([p["thrust_gf"] for p in points])
    # Contiguous time blocks, so neighbouring samples never straddle train/test.
    blocks = np.minimum(np.arange(len(points)) * CV_BLOCKS // len(points), CV_BLOCKS - 1)
    held_out = np.empty(len(points))
    for block in range(CV_BLOCKS):
        train = blocks != block
        coefficients, *_ = np.linalg.lstsq(design[train], thrust[train], rcond=None)
        held_out[~train] = design[~train] @ coefficients - thrust[~train]
    return held_out


def fit(points: list[dict[str, Any]], sources: list[dict[str, Any]] | None = None,
        rejected: list[dict[str, Any]] | None = None) -> dict[str, Any]:
    if len(points) < MIN_POINTS:
        raise ValueError(f"KV 修改后的稳态点只有 {len(points)} 个，至少需要 {MIN_POINTS} 个")
    upper = np.array([p["upper_command_pct"] for p in points])
    lower = np.array([p["lower_command_pct"] for p in points])
    charge = np.array([p["charge_v"] for p in points])
    thrust = np.array([p["thrust_gf"] for p in points])
    design = _design(upper, lower, charge)
    held_out = held_out_residuals(points)
    coefficients, *_ = np.linalg.lstsq(design, thrust, rcond=None)
    return {"schema": SCHEMA, "model_id": uuid4().hex[:12],
            "created_at": datetime.now().astimezone().isoformat(),
            "form": "thrust_gf = poly3(xu, xl), x = throttle/100 * charge_v / 12",
            "terms": list(_TERMS), "coefficients": coefficients.tolist(), "v_ref": V_REF,
            "charge_voltage": {"definition": "loaded V + k*sum((eRPM/1e4)^3)", "k": SAG_K_DEFAULT},
            "domain": {"charge_v": [float(charge.min()), float(charge.max())],
                       "command_pct": [float(min(upper.min(), lower.min())), float(max(upper.max(), lower.max()))]},
            "training": {"points": len(points), "sources": sources or [], "rejected_sessions": rejected or [],
                         "fit": _metrics(design @ coefficients - thrust),
                         "held_out_time_blocks": {"blocks": CV_BLOCKS, **_metrics(held_out)}}}


def predict_gf(model: dict[str, Any], upper_pct: float, lower_pct: float, charge_v: float) -> float | None:
    low_v, high_v = model["domain"]["charge_v"]
    low_c, high_c = model["domain"]["command_pct"]
    if not (low_v <= charge_v <= high_v and low_c <= upper_pct <= high_c and low_c <= lower_pct <= high_c):
        return None  # Outside the training domain: never extrapolate.
    return float(_design([upper_pct], [lower_pct], [charge_v])[0] @ np.asarray(model["coefficients"]))


def balanced_throttle(model: dict[str, Any], thrust_gf: float, charge_v: float) -> float | None:
    """Both rotors at the same throttle; None when the target is unreachable at this charge."""
    low_c, high_c = model["domain"]["command_pct"]
    low_v, high_v = model["domain"]["charge_v"]
    if not low_v <= charge_v <= high_v:
        return None
    grid = np.linspace(low_c, high_c, 801)
    curve = _design(grid, grid, np.full_like(grid, charge_v)) @ np.asarray(model["coefficients"])
    envelope = np.maximum.accumulate(curve)  # guard the inverse against small cubic wiggles
    if not envelope[0] <= thrust_gf <= envelope[-1]:
        return None
    index = int(np.searchsorted(envelope, thrust_gf))
    if index == 0:
        return float(grid[0])
    left, right = envelope[index - 1], envelope[index]
    fraction = 0.0 if right == left else (thrust_gf - left) / (right - left)
    return float(grid[index - 1] + fraction * (grid[index] - grid[index - 1]))


def save(model: dict[str, Any], root: Path) -> Path:
    stamp = datetime.now().astimezone()
    folder = Path(root) / "models" / "throttle" / stamp.date().isoformat() / f"{stamp:%H%M%S}-{model['model_id']}"
    folder.mkdir(parents=True, exist_ok=False)
    path = folder / "throttle_model.json"
    path.write_text(json.dumps(model, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return path


def latest(root: Path) -> tuple[dict[str, Any], Path] | None:
    paths = sorted((Path(root) / "models" / "throttle").glob("*/*/throttle_model.json"))
    if not paths:
        return None
    return json.loads(paths[-1].read_text(encoding="utf-8")), paths[-1]


def summary(model: dict[str, Any]) -> str:
    held = model["training"]["held_out_time_blocks"]
    low_v, high_v = model["domain"]["charge_v"]
    low_c, high_c = model["domain"]["command_pct"]
    sessions = len(model["training"]["sources"])
    rejected = model["training"].get("rejected_sessions") or []
    skipped = ("；未用 " + "、".join(f"{item['session'].split('/')[-1]}（{item['reason']}）" for item in rejected)
               if rejected else "")
    return (f"油门模型已建立：用 KV 修改后的 {sessions} 个会话、{model['training']['points']} 个稳态点{skipped}；"
            f"按时间段留出验证：均方根误差 {held['rmse_gf']:.1f} 克，95% 的点 ≤ {held['p95_abs_gf']:.0f} 克，"
            f"最大 {held['max_abs_gf']:.0f} 克（超 50 克 {held['over_50gf']} 个）。"
            f"适用范围：油门 {low_c:.0f}～{high_c:.0f}%，电量电压 {low_v:.2f}～{high_v:.2f} V，范围外不预测。")


def write_figure(model: dict[str, Any], points: list[dict[str, Any]], folder: Path) -> Path | None:
    """Same-throttle points and model curves per 0.3 V charge band, plus held-out errors."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return None
    from .sweep_schedule import CHARGE_BAND_V
    plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "Noto Sans CJK SC", "DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False
    ramp = ("#86b6ef", "#5598e7", "#2a78d6", "#1c5cab", "#0d366b")  # validated ordinal blue
    surface, ink, muted, grid_ink, axis_ink = "#fcfcfb", "#52514e", "#898781", "#e1e0d9", "#c3c2b7"
    balanced = [p for p in points if abs(p["upper_command_pct"] - p["lower_command_pct"]) <= 0.6]
    band = lambda v: round(np.floor(v / CHARGE_BAND_V + 1e-9) * CHARGE_BAND_V, 2)
    bands = sorted({band(p["charge_v"]) for p in balanced})[-len(ramp):]
    colors = [ramp[round(i * (len(ramp) - 1) / max(1, len(bands) - 1))] for i in range(len(bands))]
    fig, (top, bottom) = plt.subplots(2, 1, figsize=(10, 7), gridspec_kw={"height_ratios": [3, 1.3]})
    fig.patch.set_facecolor(surface)
    low_c, high_c = model["domain"]["command_pct"]
    grid = np.linspace(low_c, high_c, 200)
    for low, color in zip(bands, colors):
        members = [p for p in balanced if band(p["charge_v"]) == low]
        top.scatter([p["upper_command_pct"] for p in members], [p["thrust_gf"] for p in members],
                    s=30, color=color, edgecolor=surface, linewidth=1.2, zorder=3)
        mid = min(max(low + CHARGE_BAND_V / 2, model["domain"]["charge_v"][0]), model["domain"]["charge_v"][1])
        curve = _design(grid, grid, np.full_like(grid, mid)) @ np.asarray(model["coefficients"])
        top.plot(grid, curve, color=color, linewidth=1.2, label=f"电量 {low:.1f}–{low + CHARGE_BAND_V:.1f} V", zorder=2)
    top.legend(loc="upper left", frameon=False, fontsize=8, labelcolor=ink)
    top.set(ylabel="推力 [克]", xlabel="上下桨同油门 [%]")
    top.set_title("油门模型：点为实测（同油门），线为模型", color="#0b0b0b", loc="left", fontsize=11)
    held = held_out_residuals(points)
    predicted = np.array([p["thrust_gf"] for p in points]) + held
    bottom.scatter(predicted, held, s=12, color=ramp[2], edgecolor=surface, linewidth=.6, zorder=3)
    for value in (-50.0, 50.0):
        bottom.axhline(value, color=muted, linewidth=.8)
    bottom.axhline(0.0, color=axis_ink, linewidth=.8)
    bottom.set(xlabel="模型预测推力 [克]", ylabel="留出误差 [克]")
    for axis in (top, bottom):
        axis.set_facecolor(surface); axis.grid(True, color=grid_ink, linewidth=.5)
        for side in ("top", "right"):
            axis.spines[side].set_visible(False)
        axis.tick_params(colors=muted, labelcolor=ink)
        axis.xaxis.label.set_color(ink); axis.yaxis.label.set_color(ink)
    fig.tight_layout()
    path = Path(folder) / "throttle_model.png"
    fig.savefig(path, dpi=150, facecolor=surface)
    plt.close(fig)
    return path

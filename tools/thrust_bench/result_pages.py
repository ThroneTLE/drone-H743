"""Result pages behind the "查看拟合效果" button.

Lookup-table build: measured same-throttle points against the table, and each
table's held-out error. Bench validation: the error of every target. Static HTML
with the figures and a data table (the numbers behind every mark).
"""
from __future__ import annotations

import html
import math
from pathlib import Path
from typing import Any

import numpy as np

SURFACE, INK, HEAD_INK, MUTED, GRID_INK, AXIS_INK = "#fcfcfb", "#52514e", "#0b0b0b", "#898781", "#e1e0d9", "#c3c2b7"
RAMP = ("#86b6ef", "#5598e7", "#2a78d6", "#1c5cab", "#0d366b")  # validated ordinal blue
MARK = "#2a78d6"
LIMIT_GF = 50.0
BALANCED_TOLERANCE_PCT = 0.6


def _pyplot():
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return None
    plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "Noto Sans CJK SC", "DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False
    return plt


def _style(axis) -> None:
    axis.set_facecolor(SURFACE)
    axis.grid(True, color=GRID_INK, linewidth=.5)
    for side in ("top", "right"):
        axis.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        axis.spines[side].set_color(AXIS_INK)
    axis.tick_params(colors=MUTED, labelcolor=INK)
    axis.xaxis.label.set_color(INK); axis.yaxis.label.set_color(INK)


def _error_panel(axis, x, error, xlabel: str, title: str) -> None:
    for value in (-LIMIT_GF, LIMIT_GF):
        axis.axhline(value, color=MUTED, linewidth=.8, linestyle="--")
    axis.axhline(0.0, color=AXIS_INK, linewidth=.8)
    axis.scatter(x, error, s=26, color=MARK, edgecolor=SURFACE, linewidth=1.0, zorder=3)
    top = max(LIMIT_GF * 1.3, float(np.nanmax(np.abs(error))) * 1.15) if len(error) else LIMIT_GF * 1.3
    axis.set_ylim(-top, top)
    axis.set(xlabel=xlabel, ylabel="误差 [克]")
    axis.set_title(title, color=HEAD_INK, loc="left", fontsize=10)
    _style(axis)


def _cell(value: Any, digits: int = 0) -> str:
    if value is None or (isinstance(value, float) and not math.isfinite(value)):
        return "—"
    if isinstance(value, (int, float)):
        return f"{value:.{digits}f}"
    return html.escape(str(value))


def _page(path: Path, title: str, summary: str, images: list[str], headers: list[str], rows: list[list[str]]) -> Path:
    body = "".join(f'<img src="{html.escape(name)}" alt="{html.escape(title)}">' for name in images)
    head = "".join(f"<th>{html.escape(text)}</th>" for text in headers)
    table = "".join("<tr>" + "".join(f"<td>{cell}</td>" for cell in row) + "</tr>" for row in rows)
    path.write_text(f"""<!doctype html><html lang="zh-CN"><head><meta charset="utf-8"><title>{html.escape(title)}</title>
<style>body{{font-family:"Microsoft YaHei","PingFang SC",sans-serif;background:{SURFACE};color:{INK};margin:24px;max-width:1280px}}
h1{{font-size:20px;color:{HEAD_INK};font-weight:600}}p{{line-height:1.7}}img{{max-width:100%;display:block;margin:16px 0}}
table{{border-collapse:collapse;font-size:13px;margin-top:8px}}th,td{{border-bottom:1px solid {GRID_INK};padding:5px 10px;text-align:right}}
th{{color:{HEAD_INK};font-weight:600}}td:first-child,th:first-child{{text-align:left}}</style></head><body>
<h1>{html.escape(title)}</h1><p>{html.escape(summary)}</p>{body}<table><tr>{head}</tr>{table}</table></body></html>
""", encoding="utf-8")
    return path


def _bands(charge: np.ndarray) -> tuple[float, np.ndarray]:
    from .sweep_schedule import CHARGE_BAND_V
    width = CHARGE_BAND_V
    while len(np.unique(np.floor(charge / width + 1e-9))) > len(RAMP):
        width *= 2
    return width, np.round(np.floor(charge / width + 1e-9) * width, 3)


def lut_page(lut: dict[str, Any], points: list[dict[str, Any]], folder: Path) -> Path:
    """Fit view of a freshly built lookup table: thrust_lut.html next to thrust_lut.json."""
    from . import thrust_lut
    folder = Path(folder)
    column = lambda name: np.array([p[name] for p in points], float)
    upper, lower, charge, thrust = (column(name) for name in
                                    ("upper_command_pct", "lower_command_pct", "charge_v", "thrust_gf"))
    errors = thrust_lut.held_out_errors(points)
    images = []
    plt = _pyplot()
    if plt is not None:
        fig = plt.figure(figsize=(11, 8.2))
        fig.patch.set_facecolor(SURFACE)
        grid = fig.add_gridspec(2, 2, height_ratios=(1.25, 1), hspace=.38, wspace=.22)
        curves = fig.add_subplot(grid[0, :])
        balanced = np.abs(upper - lower) <= BALANCED_TOLERANCE_PCT
        width, band = _bands(charge)
        lows = sorted(set(band[balanced].tolist()))[-len(RAMP):]
        colors = [RAMP[round(i * (len(RAMP) - 1) / max(1, len(lows) - 1))] for i in range(len(lows))]
        low_v, high_v = lut["domain"]["charge_v"]
        line_x = np.linspace(*lut["domain"]["command_pct"], 200)
        for low, color in zip(lows, colors):
            member = balanced & (band == low)
            curves.scatter((upper[member] + lower[member]) / 2, thrust[member], s=26, color=color,
                           edgecolor=SURFACE, linewidth=1.0, zorder=3)
            mid = min(max(float(np.median(charge[member])), low_v), high_v)
            line = [thrust_lut.predict_gf(lut, x, x, mid) for x in line_x]
            curves.plot(line_x, [np.nan if y is None else y for y in line], color=color, linewidth=1.4,
                        label=f"电量 {low:.1f}–{low + width:.1f} V", zorder=2)
        curves.legend(loc="upper left", frameon=False, fontsize=8, labelcolor=INK)
        curves.set(xlabel="上下桨同油门 [%]", ylabel="推力 [克]")
        curves.set_title("拟合效果：点为实测（同油门），线为油门查补表在该电量下的查值", color=HEAD_INK, loc="left", fontsize=11)
        _style(curves)
        for position, key, name in ((grid[1, 0], "speed", "转速表"), (grid[1, 1], "throttle", "油门表")):
            held = lut[key]["held_out_time_blocks"]
            _error_panel(fig.add_subplot(position), thrust, errors[key], "实测推力 [克]",
                         f"{name}留出误差：均方根 {held['rmse_gf']:.1f} 克，最大 {held['max_abs_gf']:.0f} 克")
        fig.savefig(folder / "thrust_lut_fit.png", dpi=140, facecolor=SURFACE, bbox_inches="tight")
        plt.close(fig)
        images.append("thrust_lut_fit.png")
        if (folder / "thrust_lut.png").is_file():
            images.append("thrust_lut.png")
    reference = lut["training"]["reference_polynomial_held_out"]
    rows = []
    for key, name, spacing in (("speed", "转速表", f"{thrust_lut.SPEED_STEP_ERPM / 1000:g}k eRPM"),
                               ("throttle", "油门表", f"{thrust_lut.EFFECTIVE_STEP_PCT:g}%（油门×电量V/12）")):
        held = lut[key]["held_out_time_blocks"]
        rows.append([name, _cell(spacing), _cell(held["rmse_gf"], 1), _cell(held["p95_abs_gf"]),
                     _cell(held["max_abs_gf"]), _cell(held["over_50gf"]), _cell(len(lut[key]["unsupported_nodes"]))])
    rows.append(["原油门×电压多项式（对照）", "—", _cell(reference["rmse_gf"], 1), _cell(reference["p95_abs_gf"]),
                 _cell(reference["max_abs_gf"]), _cell(reference["over_50gf"]), "—"])
    return _page(folder / "thrust_lut.html", "查补表拟合效果", thrust_lut.summary(lut), images,
                 ["表", "格距", "留出均方根 [克]", "95% ≤ [克]", "最大 [克]", "超 50 克", "无实测支撑节点"], rows)


def validation_page(record: dict[str, Any], folder: Path) -> Path:
    """Bench validation view: validation-<run_id>.html next to the table it checked."""
    folder = Path(folder)
    stem = f"validation-{record.get('run_id') or 'run'}"
    measured = [entry for entry in record.get("targets", []) if "error_gf" in entry]
    summary = record.get("summary", {})
    images = []
    plt = _pyplot()
    if plt is not None and measured:
        speed = [entry for entry in measured if "speed_error_gf" in entry]
        fig, axes = plt.subplots(1, 2 if speed else 1, figsize=(11 if speed else 6.5, 4.2), squeeze=False)
        fig.patch.set_facecolor(SURFACE)
        _error_panel(axes[0, 0], [e["target_gf"] for e in measured], np.array([e["error_gf"] for e in measured]),
                     "目标推力 [克]", "按表反推油门：实测 − 目标")
        if speed:
            _error_panel(axes[0, 1], [e["measured_gf"] for e in speed], np.array([e["speed_error_gf"] for e in speed]),
                         "实测推力 [克]", "按实测转速查转速表：实测 − 查表")
        fig.tight_layout()
        fig.savefig(folder / f"{stem}.png", dpi=140, facecolor=SURFACE)
        plt.close(fig)
        images.append(f"{stem}.png")
    if summary.get("measured"):
        text = (f"{summary['measured']} 个目标实测完成，跳过 {summary.get('skipped', 0)} 个；按表反推：平均误差 "
                f"{summary['mae_gf']:.0f} 克，最大 {summary['max_abs_gf']:.0f} 克，"
                f"{summary['within_limit']}/{summary['measured']} 个在 ±{LIMIT_GF:g} 克内。")
        if summary.get("speed_checked"):
            text += f"按实测转速查转速表：平均误差 {summary['speed_mae_gf']:.0f} 克，最大 {summary['speed_max_abs_gf']:.0f} 克。"
    else:
        text = "本轮没有得到实测对比点。"
    text += f"停止原因：{record.get('stop_reason')}。表：{record.get('model_id')}。"
    rows = [[_cell(index), _cell(entry.get("target_gf")), _cell(entry.get("command_pct"), 1),
             _cell(entry.get("charge_v_before"), 2), _cell(entry.get("measured_gf")), _cell(entry.get("error_gf")),
             _cell(entry.get("speed_table_gf")), _cell(entry.get("speed_error_gf")), _cell(entry.get("skipped", ""))]
            for index, entry in enumerate(record.get("targets", []), 1)]
    return _page(folder / f"{stem}.html", "实测验证结果", text, images,
                 ["序号", "目标 [克]", "油门 [%]", "电量 [V]", "实测 [克]", "误差 [克]", "转速表查值 [克]",
                  "转速表误差 [克]", "备注"], rows)


def split_validation_page(record: dict[str, Any], folder: Path) -> Path:
    """Split validation: total-thrust error against the split, old vs new allocation side by side."""
    folder = Path(folder)
    stem = f"validation-{record.get('run_id') or 'run'}"
    summary = record.get("summary", {})
    measured = [entry for entry in record.get("targets", []) if "error_gf" in entry]
    images = []
    plt = _pyplot()
    if plt is not None and measured:
        fig, axes = plt.subplots(1, 2, figsize=(11, 4.2), sharey=True)
        fig.patch.set_facecolor(SURFACE)
        for axis, allocation, title in ((axes[0], "old", "旧分配（逐桨查同油门线）"), (axes[1], "new", "新分配（按二维表保持合推力）")):
            group = [e for e in measured if e.get("allocation") == allocation]
            stats = summary.get(allocation) or {}
            label = (f"：平均误差 {stats['mae_gf']:.0f} 克，最大 {stats['max_abs_gf']:.0f} 克"
                     if stats.get("measured") else "")
            _error_panel(axis, [100.0 * e["split"] for e in group], np.array([e["error_gf"] for e in group]),
                         "上下分配 [%]（正 = 上桨多）", title + label)
        top = max(LIMIT_GF * 1.3, max(abs(e["error_gf"]) for e in measured) * 1.15)  # shared axis: both panels' data
        for axis in axes:
            axis.set_ylim(-top, top)
            axis.set_xlim(-40, 40)
            axis.set_xticks([-30, -15, 0, 15, 30])
        fig.tight_layout()
        fig.savefig(folder / f"{stem}.png", dpi=140, facecolor=SURFACE)
        plt.close(fig)
        images.append(f"{stem}.png")
    parts = []
    for allocation, name in (("new", "新分配"), ("old", "旧分配")):
        stats = summary.get(allocation) or {}
        if stats.get("measured"):
            parts.append(f"{name} {stats['measured']} 点：平均误差 {stats['mae_gf']:.0f} 克，最大 {stats['max_abs_gf']:.0f} 克，"
                         f"平均偏 {stats['bias_gf']:+.0f} 克")
    text = ("；".join(parts) or "本轮没有得到实测对比点") + (
        f"。误差 = 实测合推力 − 目标合推力。停止原因：{record.get('stop_reason')}。表：{record.get('model_id')}。")
    rows = [[_cell(index), _cell({"new": "新", "old": "旧"}.get(entry.get("allocation"), "")), _cell(entry.get("target_gf")),
             _cell(100.0 * entry["split"] if "split" in entry else None), _cell(entry.get("upper_pct"), 1),
             _cell(entry.get("lower_pct"), 1), _cell(entry.get("charge_v_before"), 2), _cell(entry.get("measured_gf")),
             _cell(entry.get("error_gf")), _cell(entry.get("predicted_at_measured_charge_gf")), _cell(entry.get("skipped", ""))]
            for index, entry in enumerate(record.get("targets", []), 1)]
    return _page(folder / f"{stem}.html", "差速实测验证结果", text, images,
                 ["序号", "分配", "目标合推力 [克]", "上下分配 [%]", "上油门 [%]", "下油门 [%]", "电量 [V]",
                  "实测 [克]", "误差 [克]", "二维表预测 [克]", "备注"], rows)

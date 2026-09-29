"""Reviewable artifacts for whole-run thrust dataset training."""
from __future__ import annotations

from collections.abc import Mapping, Sequence
import html
import json
from pathlib import Path
from typing import Any

from .dataset_model import reason_zh, train_dataset
from .records import BenchSample
from .throttle_curve import BALANCED_TOLERANCE_PCT, throttle_curves


def _fmt(value: object, digits: int = 2) -> str:
    return "--" if value is None else f"{float(value):.{digits}f}"


def _headline(analysis: Mapping[str, Any]) -> dict[str, str]:
    runs = analysis["runs"]
    validation = analysis.get("selected_validation_diagnostics", {})
    metrics = validation.get("metrics") or {}
    grams = metrics.get("gram_force") or {}
    acceptance = analysis.get("prediction_acceptance", {})
    limit = acceptance.get("max_allowed_abs_error_gf", 50.0)
    selected = analysis["selection"].get("selected_candidate")
    selected_name = {"erpm_only": "仅 eRPM 候选",
                     "continuous_loaded_voltage": "连续实测电压候选"}.get(
                         selected, "尚未选出")
    candidate = analysis["candidates"].get(selected, {}) if selected else {}
    domain = candidate.get("domain", {})
    domain_name = {"training_delaunay_3d": "训练 eRPM/V 三维联合支持域",
                   "training_convex_hull_2d": "训练 eRPM 二维凸包并叠加实际 V 范围"}.get(
                       domain.get("kind"), "支持域不可用")
    applicability = candidate.get("applicability", {})
    validation_text = (f"平均相差 {_fmt(grams.get('mae'))} 克力；"
                       f"95%的点误差不超过 {_fmt(grams.get('p95_abs_error'))} 克力；"
                       f"最大相差 {_fmt(grams.get('max_abs_error'))} 克力"
                       f"（目标：每点不超过 {float(limit):g} 克力）；"
                       f"RMSE {_fmt(grams.get('rmse'))} 克力（详细指标）；覆盖 "
                       f"{float(validation.get('coverage_fraction', 0.0)):.0%}"
                       if validation.get("status") == "available" else
                       "没有独立选择验证结果；当前只能看训练诊断。")
    return {
        "runs": f"训练 {len(runs['train'])} 轮，选择验证 {len(runs['validation'])} 轮；run_id 无重叠。",
        "error": validation_text,
        "range": (f"{selected_name}；上路 eRPM "
                  f"{_fmt((applicability.get('upper_erpm') or [None])[0], 0)}～"
                  f"{_fmt((applicability.get('upper_erpm') or [None, None])[-1], 0)}，"
                  f"下路 eRPM {_fmt((applicability.get('lower_erpm') or [None])[0], 0)}～"
                  f"{_fmt((applicability.get('lower_erpm') or [None, None])[-1], 0)}，"
                  f"实际带载电压 {_fmt((applicability.get('loaded_voltage_v') or [None])[0])}～"
                  f"{_fmt((applicability.get('loaded_voltage_v') or [None, None])[-1])} V。"
                  f"支持域：{domain_name}；域外拒绝，不外推。"),
        "next": "；".join(analysis.get("next_steps", [])) or "当前没有自动生成的补测建议。",
    }


def _candidate_rows(analysis: Mapping[str, Any]) -> list[dict[str, str]]:
    names = {"erpm_only": "仅 eRPM", "continuous_loaded_voltage": "连续实测电压"}
    rows = []
    for key in ("erpm_only", "continuous_loaded_voltage"):
        candidate = analysis["candidates"][key]
        validation = analysis["selection_validation"][key]
        grams = (validation.get("metrics") or {}).get("gram_force") or {}
        rows.append({"key": key, "name": names[key],
                     "status": "可用" if candidate.get("status") == "available" else "不可用",
                     "reason": ("--" if candidate.get("status") == "available" else
                                reason_zh(candidate.get("reason"))),
                     "mae_gf": _fmt(grams.get("mae")),
                     "coverage": f"{float(validation.get('coverage_fraction', 0.0)):.0%}"})
    return rows


def _markdown(analysis: Mapping[str, Any], curves: Mapping[str, Any] | None = None,
              throttle_image: str | None = None) -> str:
    head = _headline(analysis)
    candidates = _candidate_rows(analysis)
    lines = ["# 推力实验库整轮建模报告", "",
             "## 首页四个问题", "",
             f"1. **用了几轮？** {head['runs']}",
             f"2. **误差多少克？** {head['error']}",
             f"3. **适用范围？** {head['range']}",
             f"4. **具体缺口与下一步？** {head['next']}", "",
             "## 候选比较", "",
             "| 候选 | 可用性 | 不可用原因 | 选择验证平均误差 [克力] | 覆盖 |",
             "|---|---|---|---:|---:|",
             *[f"| {row['name']} | {row['status']} | {row['reason']} | "
               f"{row['mae_gf']} | {row['coverage']} |" for row in candidates],
             "", f"- 选择：`{analysis['selection']}`", "",
             *_throttle_markdown(curves, throttle_image),
             "## 选择验证", "",
             "验证集用于候选选择，因此不是未触碰的最终测试集；系数、缩放和支持域均未使用验证数据，"
             "选择后也未重训。", "",
             f"`{analysis['selection_validation']}`", "",
             "## 数据与排除", "",
             f"`{analysis['data']}`", "",
             "## 使用边界", "",
             "- 只处理 schema v2 原始 DShot eRPM，不接受机械 RPM 重标。",
             "- 无效电流或缺失电流不影响推力候选；本报告不扩展电流模型。",
             "- 域外验证点计为拒绝，不进入误差统计；覆盖率必须与误差一起看。",
             "- 仅训练或局部可用时仍展示图，但不会宣称已验证。",
             "- 不自动下发固件、不写 Flash、不替换控制推力表。", ""]
    return "\n".join(lines)


def _throttle_markdown(curves: Mapping[str, Any] | None, image: str | None) -> list[str]:
    if curves is None or not curves["bands"]:
        return []
    return ["## 推力—油门曲线（按电池电量分段）", "",
            *([f"![推力—油门曲线]({image})", ""] if image else []),
            _throttle_caption(curves), "",
            "| 电量段 | 油门 [%] | 点数 | 实测平均 [克] | 模型 [克] | 误差 [克] |",
            "|---|---:|---:|---:|---:|---:|",
            *[f"| {' | '.join(row)} |" for row in _throttle_table(curves)], ""]


def _plot_rows(analysis: Mapping[str, Any]) -> tuple[list[Mapping[str, Any]], str]:
    validation = analysis.get("selected_validation_diagnostics", {})
    if validation.get("predictions"):
        return list(validation["predictions"]), "选择验证集（不是最终测试集）"
    selected = analysis["selection"].get("selected_candidate")
    candidate = analysis["candidates"].get(selected, {}) if selected else {}
    return list(candidate.get("training_predictions", [])), "仅训练诊断（未验证）"


# Validated ordinal ramp (dataviz blue 250/350/450/550/700, light surface): low -> high charge.
_CHARGE_RAMP = ("#86b6ef", "#5598e7", "#2a78d6", "#1c5cab", "#0d366b")
_SURFACE, _INK, _INK_2, _MUTED, _GRID, _AXIS = (
    "#fcfcfb", "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7")


def _band_colors(count: int) -> list[str]:
    if count <= 1:
        return [_CHARGE_RAMP[2]] * count
    last = len(_CHARGE_RAMP) - 1
    return [_CHARGE_RAMP[round(index * last / (count - 1))] for index in range(count)]


def _style_axis(axis) -> None:
    axis.set_facecolor(_SURFACE)
    axis.grid(True, color=_GRID, linewidth=0.5, linestyle="-")
    axis.set_axisbelow(True)
    for side in ("top", "right"):
        axis.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        axis.spines[side].set_color(_AXIS)
    axis.tick_params(colors=_MUTED, labelcolor=_INK_2)
    axis.xaxis.label.set_color(_INK_2)
    axis.yaxis.label.set_color(_INK_2)


def _throttle_plot(curves: Mapping[str, Any], output_dir: Path, plt) -> Path | None:
    from matplotlib.lines import Line2D
    bands = curves["bands"]
    if not bands:
        return None
    colors = _band_colors(len(bands))
    fig, (top, bottom) = plt.subplots(2, 1, figsize=(10, 7), sharex=True,
                                      gridspec_kw={"height_ratios": [3, 1.3]})
    fig.patch.set_facecolor(_SURFACE)
    handles = []
    label_ends = []
    for band, color in zip(bands, colors):
        for role, face in (("train", color), ("validation", _SURFACE)):
            members = [p for p in band["points"] if p["role"] == role]
            ring = color if role == "validation" else _SURFACE
            if members:
                top.scatter([p["throttle_pct"] for p in members], [p["measured_gf"] for p in members],
                            s=30, facecolor=face, edgecolor=ring, linewidth=1.2, zorder=3)
            residual = [p for p in members if p["predicted_gf"] is not None]
            if residual:
                bottom.scatter([p["throttle_pct"] for p in residual],
                               [p["predicted_gf"] - p["measured_gf"] for p in residual],
                               s=22, facecolor=face, edgecolor=ring, linewidth=1.0, zorder=3)
        run_x, run_y = [], []
        for level in band["levels"] + [{"predicted_gf": None}]:
            if level["predicted_gf"] is None:
                if len(run_x) > 1:
                    top.plot(run_x, run_y, color=color, linewidth=1.2, solid_capstyle="round",
                             solid_joinstyle="round", zorder=2)
                if run_x:
                    label_ends.append((run_x[-1], run_y[-1], band["label"]))
                run_x, run_y = [], []
                continue
            run_x.append(level["throttle_pct"])
            run_y.append(level["predicted_gf"])
        handles.append(Line2D([], [], color=color, marker="o", markersize=6,
                              markeredgecolor=_SURFACE, linewidth=1.2, label=f"电量 {band['label']}"))
    if len(bands) <= 4 and label_ends:
        measured = [p["measured_gf"] for band in bands for p in band["points"]]
        span = max(1.0, max(measured) - min(measured))
        placed = []
        for x, y, text in sorted(label_ends, key=lambda item: item[1]):
            if all(abs(y - other) > 0.05 * span for other in placed):
                top.annotate(text, (x, y), xytext=(6, 0), textcoords="offset points",
                             va="center", fontsize=8, color=_INK_2)
                placed.append(y)
    handles += [Line2D([], [], color=_MUTED, marker="o", linestyle="", markersize=6, label="训练实测"),
                Line2D([], [], color=_MUTED, marker="o", linestyle="", markersize=6,
                       markerfacecolor=_SURFACE, label="验证实测"),
                Line2D([], [], color=_MUTED, linewidth=1.2, label="模型预测（训练范围内）")]
    top.legend(handles=handles, loc="upper left", frameon=False, fontsize=8, labelcolor=_INK_2)
    top.set_ylabel("推力 [克]")
    top.set_title(f"推力—油门曲线：上下桨同油门，按电池电量每 {curves['band_width_v']:g} V 分段",
                  color=_INK, loc="left", fontsize=11)
    limit = 50.0
    for value in (-limit, limit):
        bottom.axhline(value, color=_MUTED, linewidth=0.8, zorder=1)
    bottom.axhline(0.0, color=_AXIS, linewidth=0.8, zorder=1)
    bottom.annotate(f"±{limit:g} 克目标", (1.0, limit), xycoords=("axes fraction", "data"),
                    xytext=(-4, 3), textcoords="offset points", ha="right", fontsize=8, color=_MUTED)
    bottom.set_ylabel("模型误差 [克]\n预测−实测")
    bottom.set_xlabel("油门 [%]")
    for axis in (top, bottom):
        _style_axis(axis)
    fig.tight_layout()
    path = output_dir / "dataset_throttle_curve.png"
    fig.savefig(path, dpi=150, facecolor=_SURFACE)
    plt.close(fig)
    return path


def _throttle_table(curves: Mapping[str, Any]) -> list[list[str]]:
    return [[band["label"], f"{level['throttle_pct']:g}", str(level["count"]),
             _fmt(level["measured_gf"], 1), _fmt(level["predicted_gf"], 1), _fmt(level["error_gf"], 1)]
            for band in curves["bands"] for level in band["levels"]]


def _throttle_caption(curves: Mapping[str, Any]) -> str:
    return (f"只取上下桨油门相差不超过 {BALANCED_TOLERANCE_PCT:g}% 的稳态点："
            f"{curves['balanced_points']} / {curves['total_points']} 个，"
            f"其中 {curves['predicted_points']} 个在模型训练范围内。"
            f"电量段按“带载电压 + 负载压降估计”（k={curves['sag_k']:g}，与自动补数同一口径）"
            f"每 {curves['band_width_v']:g} V 一段，所以同一块电池、同一时刻的高低油门点落在同一段；"
            "模型本身不分段，直接用每个点的实测电压。"
            "实心点为训练轮、空心点为验证轮；线为模型在这些实测工况上的预测，训练范围外不画、不外推。")


def _plots(analysis: Mapping[str, Any], output_dir: Path,
           curves: Mapping[str, Any] | None = None) -> dict[str, Path]:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return {}
    plt.rcParams["font.sans-serif"] = [
        "Microsoft YaHei", "SimHei", "Noto Sans CJK SC", "DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False
    result = {}
    if curves is not None:
        path = _throttle_plot(curves, output_dir, plt)
        if path is not None:
            result["throttle_plot"] = path
    rows, label = _plot_rows(analysis)
    if rows:
        actual = [float(row["actual_thrust_n"]) * 1000.0 / 9.80665 for row in rows]
        predicted = [float(row["predicted_thrust_n"]) * 1000.0 / 9.80665 for row in rows]
        residual = [float(row["residual_n"]) * 1000.0 / 9.80665 for row in rows]
        fig, axes = plt.subplots(1, 2, figsize=(10, 4))
        axes[0].scatter(actual, predicted, alpha=0.75)
        bounds = [min(actual + predicted), max(actual + predicted)]
        axes[0].plot(bounds, bounds, "k--", linewidth=1)
        axes[0].set(xlabel="实测推力 [克力]", ylabel="预测推力 [克力]",
                    title=f"实测与预测\n{label}")
        axes[1].axhline(0.0, color="black", linestyle="--", linewidth=1)
        axes[1].scatter(predicted, residual, alpha=0.75)
        axes[1].set(xlabel="预测推力 [克力]", ylabel="残差（预测-实测）[克力]", title="残差")
        fig.tight_layout()
        path = output_dir / "dataset_fit.png"
        fig.savefig(path, dpi=150)
        plt.close(fig)
        result["fit_plot"] = path
    train = analysis["data"].get("train_operating_points", [])
    validation = analysis["data"].get("validation_operating_points", [])
    if train or validation:
        fig = plt.figure(figsize=(7, 5))
        axis = fig.add_subplot(111, projection="3d")
        for points, name, marker in ((train, "训练", "o"), (validation, "选择验证", "^")):
            usable = [point for point in points if point.get("voltage_v") is not None]
            if usable:
                axis.scatter([p["upper_erpm"] for p in usable], [p["lower_erpm"] for p in usable],
                             [p["voltage_v"] for p in usable], label=name, marker=marker, alpha=0.75)
        axis.set(xlabel="上路 eRPM", ylabel="下路 eRPM", zlabel="实际带载电压 [V]",
                 title="训练与选择验证的联合覆盖")
        axis.legend()
        fig.tight_layout()
        path = output_dir / "dataset_coverage.png"
        fig.savefig(path, dpi=150)
        plt.close(fig)
        result["coverage_plot"] = path
    return result


def _dashboard(analysis: Mapping[str, Any], metadata: Mapping[str, Any],
               paths: Mapping[str, Path], curves: Mapping[str, Any] | None = None) -> str:
    esc = lambda value: html.escape(str(value), quote=True)
    head = _headline(analysis)
    validation = analysis.get("selected_validation_diagnostics", {})
    metrics = validation.get("metrics") or {}
    grams = metrics.get("gram_force") or {}
    acceptance = analysis.get("prediction_acceptance", {})
    status_zh = {"meets_selection_target":"选择验证达到误差目标；仍需全新独立测试轮",
                 "above_tolerance":"选择验证最大误差超过目标",
                 "insufficient_coverage":"验证覆盖不足，不能判定整个范围达标",
                 "unvalidated":"尚无选择验证轮"}.get(acceptance.get("status"),"未判定")
    rejected = esc(validation.get("rejected", {}))
    candidate_rows = "".join(
        f"<tr><td>{esc(row['name'])}</td><td>{esc(row['status'])}</td>"
        f"<td>{esc(row['reason'])}</td><td>{esc(row['mae_gf'])}</td>"
        f"<td>{esc(row['coverage'])}</td></tr>" for row in _candidate_rows(analysis))
    images = "".join(f"<section><img src='{esc(path.name)}' alt='{esc(key)}'></section>"
                     for key, path in paths.items() if key.endswith("_plot") and key != "throttle_plot")
    throttle = ""
    if curves is not None and curves["bands"]:
        table = "".join("<tr>" + "".join(f"<td>{esc(cell)}</td>" for cell in row) + "</tr>"
                        for row in _throttle_table(curves))
        figure = (f"<img src='{esc(paths['throttle_plot'].name)}' alt='推力—油门曲线'>"
                  if "throttle_plot" in paths else "")
        throttle = (f"<section><h2>推力—油门曲线（按电池电量分段）</h2>{figure}"
                    f"<p>{esc(_throttle_caption(curves))}</p><details><summary>曲线数据表</summary>"
                    "<table><thead><tr><th>电量段</th><th>油门 [%]</th><th>点数</th><th>实测平均 [克]</th>"
                    f"<th>模型 [克]</th><th>误差 [克]</th></tr></thead><tbody>{table}</tbody></table>"
                    "</details></section>")
    metadata_text = esc(json.dumps(dict(metadata), ensure_ascii=False, indent=2, default=str))
    return f"""<!doctype html><html lang="zh-CN"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<meta http-equiv="Content-Security-Policy" content="default-src 'none'; img-src 'self'; style-src 'unsafe-inline'">
<title>推力实验库整轮建模</title><style>
body{{font-family:Segoe UI,Microsoft YaHei,sans-serif;background:#f4f7f9;color:#15232d;margin:0}}
main{{max-width:1180px;margin:auto;padding:24px}}.grid{{display:grid;grid-template-columns:repeat(auto-fit,minmax(220px,1fr));gap:12px}}
.card,section{{background:white;padding:16px;border-radius:8px;margin-bottom:14px;box-shadow:0 1px 4px #ccd}}
h1,h2{{color:#174c6b}}.value{{font-size:1.1rem;font-weight:650}}img{{max-width:100%}}code,pre{{word-break:break-word;white-space:pre-wrap}}
table{{border-collapse:collapse;width:100%}}th,td{{border:1px solid #ccd;padding:7px;text-align:left}}th{{background:#e8f1f6}}
.warning{{border-left:5px solid #d89b00;background:#fff6d8}}
</style></head><body><main><h1>推力实验库整轮建模结果</h1>
<section class="warning"><strong>结论：</strong>{esc(analysis.get('summary', ''))}</section>
<section class="warning"><strong>50 克力目标：</strong>{esc(status_zh)}；
本次限值为每个可预测验证点绝对误差不超过 {esc(_fmt(acceptance.get('max_allowed_abs_error_gf'),1))} 克力。
验证集参与了候选选择，最终结论还需全新独立复测。</section>
<div class="grid"><div class="card"><h2>用了几轮？</h2><div class="value">{esc(head['runs'])}</div></div>
<div class="card"><h2>误差多少克？</h2><div class="value">{esc(head['error'])}</div></div>
<div class="card"><h2>适用范围？</h2><div class="value">{esc(head['range'])}</div></div>
<div class="card"><h2>缺口与下一步？</h2><div class="value">{esc(head['next'])}</div></div></div>
<section class="warning"><strong>验证角色：</strong>当前留出数据用于候选选择，不是未触碰的最终测试集；
选择后没有用验证数据重训。没有留出验证数据时只展示训练诊断。</section>
<section><h2>候选比较与选择</h2><table><thead><tr><th>候选</th><th>可用性</th><th>原因</th>
<th>选择验证平均误差 [克力]</th><th>覆盖</th></tr></thead><tbody>{candidate_rows}</tbody></table>
<p>选择：<code>{esc(analysis['selection'])}</code></p>
<details><summary>候选原始详情</summary><pre>{esc(json.dumps(analysis['candidates'], ensure_ascii=False, indent=2))}</pre></details></section>
<section><h2>选择验证指标</h2><p>MAE {_fmt(grams.get('mae'))} 克力(gf)；
RMSE {_fmt(grams.get('rmse'))} 克力(gf)；P95 {_fmt(grams.get('p95_abs_error'))} 克力(gf)；
预测覆盖 {float(validation.get('coverage_fraction', 0.0)):.1%}。</p>
<p>域外或缺失拒绝：<code>{rejected}</code>。误差只统计支持域内可预测点，必须连同覆盖率阅读。</p></section>
{throttle}{images}
<section><h2>具体边界</h2><ul><li>训练与验证 run_id 必须完全不重叠。</li><li>所有缩放、系数、支持域只来自训练轮。</li>
<li>连续 V 候选不需要手工电压层，但 V 与 eRPM 共线时会明确不可辨识。</li><li>局部可用或仅训练时仍画图，不会把训练误差称为验证。</li></ul></section>
<details><summary>元数据</summary><pre>{metadata_text}</pre></details></main></body></html>"""


def write_dataset_analysis(train_samples: Sequence[BenchSample],
                           validation_samples: Sequence[BenchSample],
                           metadata: Mapping[str, Any] | None,
                           output_dir: str | Path) -> dict[str, Path]:
    destination = Path(output_dir)
    destination.mkdir(parents=True, exist_ok=False)
    meta = dict(metadata or {})
    analysis = train_dataset(train_samples, validation_samples, meta)
    model_path = destination / "dataset_analysis.json"
    model_path.write_text(json.dumps(analysis, ensure_ascii=False, indent=2,
                                     allow_nan=False) + "\n", encoding="utf-8")
    report_path = destination / "dataset_report.md"
    paths = {"model": model_path, "report": report_path}
    curves = throttle_curves(analysis)
    paths.update(_plots(analysis, destination, curves))
    throttle_image = paths["throttle_plot"].name if "throttle_plot" in paths else None
    report_path.write_text(_markdown(analysis, curves, throttle_image), encoding="utf-8")
    dashboard_path = destination / "dataset_dashboard.html"
    dashboard_path.write_text(_dashboard(analysis, meta, paths, curves), encoding="utf-8")
    paths["dashboard"] = dashboard_path
    return paths

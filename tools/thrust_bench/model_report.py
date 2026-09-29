"""Auditable JSON, Markdown and plots for installed coaxial bench models."""
from __future__ import annotations

import json
import html
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any

from .model import analyze_samples
from .records import BenchSample


def _fmt(value: object, digits: int = 3) -> str:
    return "--" if value is None else f"{float(value):.{digits}f}"


def _reason_zh(reason: object) -> str:
    key = str(reason)
    meanings = {
        "fewer_than_2_measured_voltage_layers": "真实带载电压层少于两个",
        "voltage_layer_fit_unavailable": "至少一个电压层内部的 eRPM 模型不可辨识",
        "measured_voltage_layers_not_distinct": "各层真实带载电压中心没有充分分开",
        "no_overlapping_erpm_coverage_across_voltage_layers": "不同电压层没有共同 eRPM 覆盖，电压效应与转速变化混淆",
        "voltage_layer_span_too_wide": "同一电压层内的真实带载电压跨度过大",
        "measured_voltage_layer_ranges_overlap": "相邻电压层的真实带载电压范围重叠或间隔不足",
        "fewer_than_6_steady_operating_points": "双驱动稳态工作点少于六个",
        "inputs_not_independently_varied": "两路 eRPM 没有独立变化",
        "fewer_than_4_single_drive_points": "单驱动稳态工作点少于四个",
        "insufficient_active_erpm_span": "主动路 eRPM 范围不足",
        "rank_deficient_or_ill_conditioned": "拟合矩阵秩不足或条件数过大",
        "None": "未提供原因",
    }
    return meanings.get(key, "数据不满足模型可用条件")


def _report(analysis: Mapping[str, Any], metadata: Mapping[str, Any]) -> str:
    static = analysis["static"]
    power = analysis["power"]
    dynamics = analysis["dynamics"]
    validation = analysis["validation"]
    validation_diagnostics = validation.get("diagnostics", {})
    coverage = static.get("coverage", {})
    lines = ["# 共轴双桨实际安装状态离线模型报告", "",
             "> 两片桨均保持安装。本报告拟合称重得到的总推力；未驱动桨可能被动风车，"
             "因此不能把任何系数解释为孤立单桨推力或扭矩。", "",
             "## 数据身份与适用范围", "",
             f"- 样本 schema：`{metadata.get('schema_version', 2)}`",
             f"- 速度域：`{metadata.get('speed_domain', 'electrical_erpm')}`（不做极对数换算）",
             f"- 固件/配置身份：`{metadata.get('firmware_id', metadata.get('firmware', '未提供'))}`",
             f"- 两桨安装：`{metadata.get('both_propellers_installed', '未提供')}`",
             f"- 未驱动桨状态：`{metadata.get('undriven_propeller_state', '未知；可能静止或风车')}`",
             f"- 稳态工作点：{coverage.get('points', 0)}",
             f"- 上桨电转速范围：{coverage.get('upper_erpm', '--')} eRPM",
             f"- 下桨电转速范围：{coverage.get('lower_erpm', '--')} eRPM",
             f"- 电压层：{coverage.get('voltage_layers_v', [])} V", ""]
    lines += ["## 总推力静态模型", ""]
    dual_voltage = static.get("voltage_models", {}).get("dual", {})
    if dual_voltage.get("status") == "available":
        layer_summary = [{"label": layer["label"],
                          "center_v": layer["voltage_center_v"],
                          "range_v": layer["voltage_range_v"]}
                         for layer in dual_voltage["layers"]]
        lines += ["- **双驱动主要标定结果**：`T_total = f(eRPM_upper, eRPM_lower, V_loaded)`。",
                  "- 各真实带载电压层独立拟合 eRPM 二维面，只在相邻实测层中心之间线性插值。",
                  f"- 实测电压层：`{layer_summary}`",
                  f"- 可插值电压范围：{dual_voltage['voltage_range_v']} V",
                  f"- 相邻电压层 eRPM 凸包交集：`{dual_voltage['common_coverage']}`",
                  f"- 电压层判据：`{static['voltage_layer_limits']}`",
                  "- 超出共同 eRPM 覆盖或实测电压层中心范围会拒绝预测，不做外推。", ""]
    else:
        lines += [f"- **双驱动模型不可用**；原因：{_reason_zh(dual_voltage.get('reason'))}"
                  f"（`{dual_voltage.get('reason')}`）。",
                  "- 这不影响下方各自满足条件的 upper/lower 单驱动安装状态模型。",
                  "- 双驱动需要至少两个稳定且分离的真实带载电压层，并在相邻层具备有面积的 eRPM 凸包交集。", ""]
    lines += [f"- 本次电压层默认/配置判据：`{static.get('voltage_layer_limits')}`。"
              "标签只用于分组，稳定性和分离性按真实带载电压范围判断。",
              f"- 分组策略：`{static.get('voltage_grouping_policy')}`。未标层按 run 分组，不自动把连续放电曲线分箱。", ""]
    baseline = static.get("erpm_only_baseline", {})
    lines += ["### eRPM-only 基线", ""]
    if baseline.get("status") == "available":
        model = baseline["model"]
        metrics = baseline["training_metrics"]
        lines += [f"- 模型：`{model['kind']}`；它仅用于比较，不是本批主要标定结果。",
                  f"- 公式：`{model['equation']}`",
                  f"- 训练 RMSE：{_fmt(metrics['rmse_n'])} N；MAE：{_fmt(metrics['mae_n'])} N。", ""]
    else:
        lines += [f"- 不可用：{_reason_zh(baseline.get('reason'))}"
                  f"（`{baseline.get('reason')}`）。", ""]
    single = static.get("single_drive_installed_baselines", {})
    lines += ["### 单驱动安装状态模型", ""]
    for mode in ("upper", "lower"):
        item = static.get("voltage_models", {}).get(mode, {})
        lines.append(f"- {mode}：{item.get('status', 'unavailable')}"
                     + (f"；范围 {item.get('voltage_range_v')} V，使用主动路实测 eRPM。"
                        if item.get("status") == "available" else
                        f"；原因 {_reason_zh(item.get('reason'))}（`{item.get('reason')}`）。"))
    lines += [f"- 共保留 {len(single.get('points', []))} 个单驱动工作点。结果是两片桨均安装时的总测力，"
              "未驱动桨可能风车，不能称为孤立单桨推力。", ""]
    quality = static.get("data_quality", {})
    lines += ["### 数据质量", "",
              f"- 输入样本：{quality.get('input_samples', 0)}",
              f"- 接受的稳态点：{quality.get('accepted_steady_points', 0)}",
              f"- 排除原因：`{quality.get('excluded_samples_by_reason', {})}`", ""]
    lines += ["### 使用方法", "",
              "- 调用 `predict_thrust(analysis, upper_erpm=..., lower_erpm=..., voltage_v=..., mode=...)`。",
              "- eRPM 必须直接来自有效的双向 DShot 回传；不除极对数，不使用 KV 估算兜底。",
              "- `dual` 需要两路实测 eRPM；`upper/lower` 只要求主动路实测 eRPM。",
              "- 输入超出共同 eRPM 覆盖或相邻实测电压层中心范围时函数会拒绝，不做外推。", ""]
    lines += ["## 输入功耗", ""]
    lines += [f"- 总状态：`{power.get('status')}`；来源：`{power.get('source')}`。",
              f"- 上路电流模型：`{power.get('upper_current_model', {}).get('status')}`；"
              "`I_upper=f(eRPM_upper,eRPM_lower,V_loaded)`。",
              f"- 下路电流模型：`{power.get('lower_current_model', {}).get('status')}`；"
              "`I_lower=f(eRPM_upper,eRPM_lower,V_loaded)`。",
              f"- 同步组合功耗模型：`{power.get('combined_input_power_model', {}).get('status')}`；"
              "`P=V_loaded×(I_upper+I_lower)`，只有两路电流与电压均新鲜且对齐才形成样本。",
              f"- 当前源分辨率：{power.get('current_resolution_a')} A"
              f"（{power.get('current_resolution_source')}）；DShot ESC 电流外部校验："
              f"`{power.get('external_calibration_verified')}`。未外校验不阻断观察模型，但不宣称正式安培标定。",
              "- 分路预测调用 `predict_current(analysis, upper_erpm=..., lower_erpm=..., "
              "voltage_v=..., role='upper'|'lower')`；覆盖范围外拒绝外推。",
              "- 板总 ADC 电流只作诊断，不参与拟合，也不在 DShot 电流缺失时回退。", ""]
    for mode in ("dual", "upper", "lower"):
        models = power.get("current_models", {}).get(mode, {})
        lines.append(f"- {mode} 扫描：upper I `{models.get('upper', {}).get('status')}`，"
                     f"lower I `{models.get('lower', {}).get('status')}`，"
                     f"组合功耗 `{models.get('combined_input_power', {}).get('status')}`。")
    lines.append("")
    lines += ["## 升降速动态", ""]
    if dynamics.get("status") == "available":
        lines += ["- 模型：一阶加纯延迟；上/下通道与 rise/fall 分开。",
                  f"- 汇总：`{dynamics.get('summaries', {})}`",
                  f"- 拒绝项：`{dynamics.get('rejected', [])}`",
                  "- 每项给出实际 eRPM 更新间隔、tau 覆盖的更新点数及记录覆盖的 tau 数；"
                  "tau 少于 2 个独立更新间隔或记录不足 4 tau 时拒绝。纯延迟分辨率不优于实际更新间隔。",
                  "- 动态拟合只使用 eRPM，不使用称重响应；因此不会把称重内部滤波拟成电机延迟。"
                  "结果仍包含 eRPM 遥测与采集延迟，不能解释为纯电机固有延迟。", ""]
    else:
        lines += [f"- 状态：unavailable；原因：`{dynamics.get('reason')}`",
                  f"- 拒绝项：`{dynamics.get('rejected', [])}`", ""]
    lines += ["## 独立验证", "",
              f"- 状态：{validation.get('status')}；策略：`{validation.get('strategy')}`",
              f"- 可用留出预测点：{validation_diagnostics.get('points', 0)}；"
              f"指标：`{validation_diagnostics.get('metrics')}`",
              f"- 按实际带载电压分组误差：`{validation_diagnostics.get('error_by_loaded_voltage', [])}`",
              f"- 不可用留出组：`{validation_diagnostics.get('unavailable_groups', [])}`",
              "- 同一 segment 的相邻样本不会随机拆到训练集和验证集。独立 run/电压层不足时，"
              "只报告训练拟合，不能宣称泛化。", "",
              "## 数据充分性", "",
              f"- 结果：`{static.get('sufficiency')}`",
              f"- 已采模式及聚合稳态点：`{static.get('sufficiency', {}).get('mode_point_counts')}`",
              f"- 指令网格：`{static.get('sufficiency', {}).get('command_grid')}`。该数量来自实际记录的"
              " command 组合，不代表唯一 eRPM 工况；实际覆盖看 eRPM/V 图。",
              "- 25 个非零有效组合 × 3 个实际电量覆盖 + 独立验证轮只是起步采集建议，不是硬数量门。",
              "- raw 行数、聚合稳态点、唯一指令组合和独立验证轮是不同数量；零点/不转点另计。",
              "- 默认 5% 满量程 RMSE、10% 满量程 P95 只是起步参考，不代表飞行放行。", "",
              "### 电压层留出限制", "",
              "- 整层留出只允许在剩余电压层之间插值，不外推；最低/最高边界层通常需用同层独立重复轮验证。",
              "- 两个电压层不会执行整层留出；至少三个电压层才可能验证中间层。", "",
              "## 待实测与使用边界", "",
              "- 在计划使用的转速、电压和双桨组合范围内补齐二维覆盖；窄的同油门线不能辨识二维模型。",
              "- 用独立 run 和独立电压层复测误差；确认称重滤波、采样新鲜度、丢帧与间断。",
              "- 电池放电时建议交错/正反扫描、缩短一组，并跨电量重复相同有效组合；当前分层约束"
              "不表示任意连续放电曲线都能直接建模。",
              "- DShot 电流未外部校验时只称遥测尺度观察模型，不称正式安培标定。",
              "- 本报告不会自动下发模型、写 Flash 或替换 C 控制推力表。", "",
              "## 警告", ""]
    lines.extend(f"- {warning}" for warning in analysis["warnings"])
    lines.append("")
    return "\n".join(lines)


def _write_plots(analysis: Mapping[str, Any], output_dir: Path) -> dict[str, Path]:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        plt.rcParams["font.sans-serif"] = [
            "Microsoft YaHei", "SimHei", "Noto Sans CJK SC", "DejaVu Sans"]
        plt.rcParams["axes.unicode_minus"] = False
    except ImportError:
        return {}
    result: dict[str, Path] = {}
    points = analysis["static"].get("operating_points", [])
    if points:
        fig = plt.figure(figsize=(7, 5))
        axis = fig.add_subplot(111, projection="3d")
        scatter = axis.scatter([p["upper_erpm"] for p in points], [p["lower_erpm"] for p in points],
                               [p["thrust_n"] for p in points], c=[p["thrust_n"] for p in points],
                               cmap="viridis")
        axis.set(xlabel="上路 eRPM", ylabel="下路 eRPM", zlabel="总推力 [N]")
        fig.colorbar(scatter, ax=axis, label="总推力 [N]")
        fig.tight_layout()
        path = output_dir / "static_coverage.png"
        fig.savefig(path, dpi=150)
        plt.close(fig)
        result["static_plot"] = path
        voltage_points = [point for point in points if point.get("voltage_v") is not None]
        fig = plt.figure(figsize=(7, 5))
        axis = fig.add_subplot(111, projection="3d")
        scatter = axis.scatter([p["upper_erpm"] for p in voltage_points],
                               [p["lower_erpm"] for p in voltage_points],
                               [p["voltage_v"] for p in voltage_points],
                               c=[p["voltage_v"] for p in voltage_points], cmap="plasma")
        axis.set(xlabel="上路 eRPM", ylabel="下路 eRPM", zlabel="实际带载电压 [V]")
        fig.colorbar(scatter, ax=axis, label="实际带载电压 [V]")
        fig.tight_layout()
        path = output_dir / "erpm_voltage_coverage.png"
        fig.savefig(path, dpi=150)
        plt.close(fig)
        result["coverage_plot"] = path
    validation_rows = analysis.get("validation", {}).get("diagnostics", {}).get("predictions", [])
    diagnostic_kind = "独立留出验证"
    diagnostic_rows = validation_rows
    if not diagnostic_rows:
        diagnostic_rows = analysis.get("static", {}).get("training_diagnostics", {}).get(
            "predictions", [])
        diagnostic_kind = "训练拟合（不是验证）"
    if diagnostic_rows:
        actual = [float(row["actual_thrust_n"]) for row in diagnostic_rows]
        predicted = [float(row["predicted_thrust_n"]) for row in diagnostic_rows]
        residual = [float(row["residual_n"]) for row in diagnostic_rows]
        voltage = [float(row["voltage_v"]) for row in diagnostic_rows]
        fig, axes = plt.subplots(1, 3, figsize=(14, 4))
        axes[0].scatter(actual, predicted, alpha=0.75)
        bounds = [min(actual + predicted), max(actual + predicted)]
        axes[0].plot(bounds, bounds, "k--", linewidth=1)
        axes[0].set(xlabel="实测推力 [N]", ylabel="预测推力 [N]",
                    title=f"实测与预测\n{diagnostic_kind}")
        axes[1].axhline(0.0, color="black", linestyle="--", linewidth=1)
        axes[1].scatter(predicted, residual, alpha=0.75)
        axes[1].set(xlabel="预测推力 [N]", ylabel="残差（预测-实测）[N]",
                    title="残差")
        axes[2].axhline(0.0, color="black", linestyle="--", linewidth=1)
        axes[2].scatter(voltage, residual, alpha=0.75)
        axes[2].set(xlabel="实际带载电压 [V]", ylabel="残差 [N]",
                    title="按实际带载电压查看误差")
        fig.tight_layout()
        path = output_dir / "fit_diagnostics.png"
        fig.savefig(path, dpi=150)
        plt.close(fig)
        result["fit_plot"] = path
    observations = [point for point in analysis["power"].get("observations", [])
                    if point.get("input_power_w") is not None]
    if observations:
        fig, axis = plt.subplots(figsize=(7, 4))
        axis.scatter(range(len(observations)), [p["input_power_w"] for p in observations])
        axis.set(xlabel="已接受工作点", ylabel="总输入 V×I [W]",
                 title="DShot 遥测尺度（未外部校验）")
        fig.tight_layout()
        path = output_dir / "power_observations.png"
        fig.savefig(path, dpi=150)
        plt.close(fig)
        result["power_plot"] = path
    fits = analysis["dynamics"].get("fits", [])
    if fits:
        fig, axis = plt.subplots(figsize=(7, 4))
        channel_name = {"upper": "上路", "lower": "下路"}
        direction_name = {"rise": "上升", "fall": "下降"}
        labels = [f"{channel_name.get(p['channel'], p['channel'])} "
                  f"{direction_name.get(p['direction'], p['direction'])}" for p in fits]
        axis.scatter([p["time_constant_s"] for p in fits], [p["delay_s"] for p in fits])
        for label, point in zip(labels, fits):
            axis.annotate(label, (point["time_constant_s"], point["delay_s"]), fontsize=7)
        axis.set(xlabel="时间常数 [s]", ylabel="纯延迟 [s]")
        fig.tight_layout()
        path = output_dir / "dynamic_fits.png"
        fig.savefig(path, dpi=150)
        plt.close(fig)
        result["dynamics_plot"] = path
    return result


def _dashboard_html(analysis: Mapping[str, Any], metadata: Mapping[str, Any],
                    paths: Mapping[str, Path]) -> str:
    esc = lambda value: html.escape(str(value), quote=True)
    static = analysis["static"]
    validation = analysis["validation"].get("diagnostics", {})
    training = static.get("training_diagnostics", {})
    sufficiency = static.get("sufficiency", {})
    validation_metrics = validation.get("metrics") or {}
    training_metrics = training.get("metrics") or {}
    rows = validation.get("predictions", [])
    validation_groups = validation.get("groups", {})
    run_groups = validation_groups.get("run", {})
    voltage_groups = validation_groups.get("voltage_layer", {})
    if validation.get("status") == "available":
        metric_source = (
            f"整轮留出可用 {run_groups.get('available', 0)}/{run_groups.get('attempted', 0)} 组；"
            f"中间电压层留出可用 {voltage_groups.get('available', 0)}/"
            f"{voltage_groups.get('attempted', 0)} 组；"
            f"总体预测覆盖 {float(validation.get('prediction_coverage_fraction', 0.0)):.1%}")
    else:
        metric_source = "没有可用的独立留出预测，当前未验证"
    fit_image = paths.get("fit_plot")
    coverage_image = paths.get("coverage_plot")
    prediction_rows = "".join(
        "<tr>" + "".join(f"<td>{esc(value)}</td>" for value in (
            row.get("holdout_group", ""), row.get("holdout_value", ""),
            row.get("run_id", ""), row.get("segment_id", ""),
            _fmt(row.get("voltage_v")), _fmt(row.get("actual_thrust_n")),
            _fmt(row.get("predicted_thrust_n")), _fmt(row.get("residual_n")))) + "</tr>"
        for row in rows)
    if not prediction_rows:
        prediction_rows = "<tr><td colspan='8'>没有可用的独立留出预测；训练拟合不能代替验证。</td></tr>"
    voltage_rows = "".join(
        f"<tr><td>{esc(item.get('loaded_voltage_v'))}</td><td>{esc(item.get('points'))}</td>"
        f"<td>{esc(_fmt((item.get('metrics') or {}).get('rmse_n')))}</td>"
        f"<td>{esc(_fmt((item.get('metrics') or {}).get('mae_n')))}</td>"
        f"<td>{esc(_fmt((item.get('metrics') or {}).get('p95_abs_error_n')))}</td></tr>"
        for item in validation.get("error_by_loaded_voltage", []))
    if not voltage_rows:
        voltage_rows = "<tr><td colspan='5'>无独立留出电压分组误差。</td></tr>"
    images = ""
    for title, path in (("实测、预测与残差", fit_image),
                        ("eRPM 与实际带载电压覆盖", coverage_image)):
        if path is not None:
            images += f"<section><h2>{esc(title)}</h2><img src='{esc(path.name)}' alt='{esc(title)}'></section>"
    metadata_text = esc(json.dumps(dict(metadata), ensure_ascii=False, indent=2, default=str))
    unavailable_text = esc(validation.get("unavailable_groups", []))
    gaps = "".join(f"<li>{esc(gap)}</li>" for gap in sufficiency.get("gaps", [])) or "<li>无</li>"
    return f"""<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<meta http-equiv="Content-Security-Policy" content="default-src 'none'; img-src 'self'; style-src 'unsafe-inline'">
<title>推力模型拟合与验证</title><style>
body{{font-family:Segoe UI,Microsoft YaHei,sans-serif;margin:0;background:#f4f6f8;color:#17202a}}
main{{max-width:1200px;margin:auto;padding:24px}}h1,h2{{color:#12344d}}.notice{{background:#fff3cd;padding:14px;border-left:5px solid #d39e00}}
.grid{{display:grid;grid-template-columns:repeat(auto-fit,minmax(180px,1fr));gap:12px}}.card{{background:white;border-radius:8px;padding:14px;box-shadow:0 1px 4px #ccd}}
.value{{font-size:1.5rem;font-weight:700}}table{{border-collapse:collapse;width:100%;background:white}}th,td{{border:1px solid #ccd;padding:7px;text-align:right}}th{{background:#e9f2f8}}td:nth-child(-n+4),th:nth-child(-n+4){{text-align:left}}
img{{max-width:100%;background:white;border:1px solid #ccd}}pre{{white-space:pre-wrap;background:#17202a;color:#eef;padding:12px}}code{{word-break:break-word}}
</style></head><body><main>
<h1>推力模型拟合与独立验证</h1>
<p class="notice"><strong>验证状态：{esc(metric_source)}</strong>。默认 5% 满量程 RMSE / 10% 满量程 P95 只是起步参考，不是飞行放行。</p>
<section><h2>独立验证指标</h2><div class="grid">
<div class="card"><div>状态</div><div class="value">{esc(validation.get('status', 'unvalidated'))}</div></div>
<div class="card"><div>预测点</div><div class="value">{esc(validation.get('points', 0))}</div></div>
<div class="card"><div>RMSE [N]</div><div class="value">{esc(_fmt(validation_metrics.get('rmse_n')))}</div></div>
<div class="card"><div>MAE [N]</div><div class="value">{esc(_fmt(validation_metrics.get('mae_n')))}</div></div>
<div class="card"><div>P95 |误差| [N]</div><div class="value">{esc(_fmt(validation_metrics.get('p95_abs_error_n')))}</div></div>
</div><p>不能预测或不可用的留出组：<code>{unavailable_text}</code></p></section>
<section><h2>训练拟合（仅诊断）</h2><p>覆盖 {esc(training.get('points', 0))}/{esc(training.get('input_points', 0))} 个点，覆盖比例 {esc(_fmt(training.get('coverage_fraction')))}。不能预测：<code>{esc(training.get('rejected', {}))}</code></p>
<p>训练 RMSE {_fmt(training_metrics.get('rmse_n'))} N；训练误差不计入独立验证结论。</p></section>
<section><h2>数据充分性</h2><div class="grid"><div class="card"><div>状态</div><div class="value">{esc(sufficiency.get('status'))}</div></div>
<div class="card"><div>双驱动聚合稳态点</div><div class="value">{esc(sufficiency.get('aggregated_steady_points'))}</div></div>
<div class="card"><div>每电压组最少唯一非零指令组合</div><div class="value">{esc(sufficiency.get('minimum_unique_nonzero_command_combinations_per_dual_voltage_group'))}</div></div>
<div class="card"><div>独立轮次</div><div class="value">{esc(sufficiency.get('independent_runs'))}</div></div></div>
<p><strong>{esc(sufficiency.get('summary', ''))}</strong></p>
<p>已采模式及聚合稳态点：<code>{esc(sufficiency.get('mode_point_counts', {}))}</code></p>
<p>指令网格统计：<code>{esc(sufficiency.get('command_grid', {}))}</code>。这是实际记录的指令组合数，不代表唯一 eRPM 工况；实际 eRPM/V 覆盖请看覆盖图。</p>
<p>起步建议：每个实际电量组约 25 个唯一非零指令组合，并另做独立验证轮。这是采集规划参考，不是硬数量门；零点/不转点另计，raw 行数、聚合稳态点和唯一指令组合互不等同。</p>
<ul>{gaps}</ul></section>
<section><h2>电压层留出限制</h2><p>整层留出只允许在剩余电压层之间插值，不允许外推。因此最低和最高电压边界层通常不能通过 leave-one-layer-out 预测；边界层必须用同一电压层的独立重复轮次验证。只有两个电压层时不会执行整层留出；至少三个电压层才可能验证中间层。</p></section>
{images}
<section><h2>按实际带载电压分组的独立误差</h2><table><thead><tr><th>V</th><th>点数</th><th>RMSE N</th><th>MAE N</th><th>P95 N</th></tr></thead><tbody>{voltage_rows}</tbody></table></section>
<section><h2>独立留出实测与预测明细</h2><table><thead><tr><th>分组</th><th>组值</th><th>run</th><th>segment</th><th>V</th><th>实测 N</th><th>预测 N</th><th>残差 N</th></tr></thead><tbody>{prediction_rows}</tbody></table></section>
<section><h2>电池放电与采集建议</h2><ul><li>每个点使用实际带载电压；标签不能替代实测 V。</li><li>当前实现要求每个 run/标签内电压跨度受限，且相邻层实际范围分离；它不等于可以直接拟合任意连续放电曲线。</li><li>建议交错或正反扫描、缩短单组时间，并跨电量重复相同有效 eRPM 组合，以降低电压与转速共线。</li></ul></section>
<details><summary>元数据</summary><pre>{metadata_text}</pre></details>
</main></body></html>"""


def write_analysis(samples: Sequence[BenchSample], metadata: Mapping[str, Any] | None,
                   output_dir: str | Path) -> dict[str, Path]:
    """Write a reviewable analysis to a caller-owned, non-existing directory."""
    destination = Path(output_dir)
    destination.mkdir(parents=True, exist_ok=False)
    meta = metadata or {}
    analysis = analyze_samples(samples, meta)
    model_path = destination / "analysis.json"
    model_path.write_text(json.dumps(analysis, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
                          encoding="utf-8")
    report_path = destination / "report.md"
    report_path.write_text(_report(analysis, meta), encoding="utf-8")
    paths = {"model": model_path, "model_json": model_path, "report": report_path}
    paths.update(_write_plots(analysis, destination))
    dashboard_path = destination / "dashboard.html"
    dashboard_path.write_text(_dashboard_html(analysis, meta, paths), encoding="utf-8")
    paths["dashboard"] = dashboard_path
    return paths

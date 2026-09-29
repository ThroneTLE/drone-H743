"""产出：CSV、Markdown 报告、以及给固件用的 C 头。

旧流程（2026-09-24 起不再用于飞控）：它生成 drv_coax_ctrl.c 里那张 21 点旧曲线。
飞控推力换算改由 thrust_lut_export 生成查补表，当前版本见 models/lut/current.json。

报告里有三样东西是**必须**出现的，缺一样这份标定就没法被别人复核：

* 转速是量的还是算的（`RpmSource.quality` / `measured`）；
* 每一档的**带载电压**与掉压；
* 上下行的迟滞。

C 头是给 `drv_coax_ctrl.c` 那张 21 点脉宽→推力表用的。生成它的时候会顺手把
"这张表是在哪个电压层、用哪个转速来源标的"写进注释——表本身是一串数字，
半年后没有那行注释就没人知道它适不适用。
"""

from __future__ import annotations

import csv
from datetime import date
from pathlib import Path

from .fit import ThrustCurve, VoltageModel, thrust_coefficient
from .rpm import RpmSource, annotate
from .sweep import Measurement, hysteresis, voltage_sag

ROOT = Path(__file__).resolve().parents[2]
THRUST_DIR = ROOT / "data" / "identification" / "thrust"
MOTOR_DIR = ROOT / "data" / "identification" / "motor"


def run_directory(base: Path | None = None, when: date | None = None) -> Path:
    directory = (base or THRUST_DIR) / (when or date.today()).isoformat()
    directory.mkdir(parents=True, exist_ok=True)
    return directory


def write_csv(measurements: list[Measurement], path: Path,
              source: RpmSource) -> Path:
    """一行一档。`rpm_measured` 列把"量的还是算的"落到数据里，不只落在报告里。

    数据文件会被别的脚本读走，而那些脚本看不到报告。
    """
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow([
            "percent", "direction", "pulse_us", "thrust_g", "voltage_v",
            "voltage_layer_v", "current_a", "rpm", "rpm_measured",
            "rpm_source", "samples", "spread_g", "warning",
        ])
        for item in measurements:
            writer.writerow([
                item.percent, item.direction, item.pulse_us,
                f"{item.thrust_g:.3f}",
                "" if item.voltage_v is None else f"{item.voltage_v:.3f}",
                "" if item.voltage_layer_v is None else f"{item.voltage_layer_v:.3f}",
                "" if item.current_a is None else f"{item.current_a:.3f}",
                "" if item.rpm is None else f"{item.rpm:.1f}",
                int(source.measured), source.quality,
                item.samples, f"{item.spread_g:.3f}",
                item.suspicious or "",
            ])
    return path


def write_report(measurements: list[Measurement], path: Path, *,
                 source: RpmSource, curve: ThrustCurve | None = None,
                 voltage_model: VoltageModel | None = None,
                 propeller_diameter_m: float = 0.0,
                 notes: str = "") -> Path:
    lines: list[str] = ["# 推力台标定报告", ""]
    lines.append(f"- 采集点：{len(measurements)}")
    lines.append(f"- 转速来源：`{source.quality}`"
                 + ("（**实测**）" if source.measured else "（**推算，非实测**）"))
    if not source.measured:
        lines.append("  - 推算的转速**不得**用来算 C_T：转速在 `F/(ρn²D⁴)` 里是"
                     "平方项，带载系数的不确定度会被放大成两倍以上的 C_T 误差。")
    layers = sorted({m.voltage_layer_v for m in measurements
                     if m.voltage_layer_v is not None})
    if layers:
        lines.append("- 电压分层：" + "、".join(f"{v:.2f} V" for v in layers))
    else:
        lines.append("- **未做电压分层**：这张表只在采集当时那块电池那个电量下成立。")
    lines.append("")

    lines.append("## 推力曲线")
    lines.append("")
    if curve is not None:
        lines.append(f"`F[g] = {curve.a:.5g}·p² + {curve.b:.5g}·p + {curve.c:.5g}`"
                     f"（R² = {curve.r_squared:.4f}，{curve.points} 点）")
        lines.append("")
        lines.append("二次而不是直线：桨推力大致正比于转速平方、油门到转速大致线性。"
                     "用直线拟合会在两端各偏一截，而两端正是悬停点和满油。")
    else:
        lines.append("点数不足，未拟合。")
    lines.append("")

    lines.append("## 逐档数据")
    lines.append("")
    lines.append("| 油门% | 方向 | 推力 g | 带载电压 V | 电流 A | 转速 | C_T | 备注 |")
    lines.append("|---|---|---|---|---|---|---|---|")
    for item in measurements:
        coefficient = thrust_coefficient(item.thrust_g, item.rpm or 0.0,
                                         propeller_diameter_m, source)
        lines.append(
            f"| {item.percent:g} | {item.direction} | {item.thrust_g:.1f} | "
            f"{'--' if item.voltage_v is None else f'{item.voltage_v:.2f}'} | "
            f"{'--' if item.current_a is None else f'{item.current_a:.2f}'} | "
            f"{annotate(item.rpm, source)} | "
            f"{'--' if coefficient is None else f'{coefficient:.4f}'} | "
            f"{item.suspicious or ''} |")
    lines.append("")

    gaps = hysteresis(measurements)
    lines.append("## 迟滞（上行 − 下行）")
    lines.append("")
    if gaps:
        worst = max(gaps.items(), key=lambda item: abs(item[1]))
        if isinstance(worst[0], tuple):
            layer, percent = worst[0]
            location = f"{percent:g}% 档、{'未分层' if layer is None else f'{layer:.2f} V 层'}"
        else:
            location = f"{worst[0]:g}% 档"
        lines.append(f"最大 {worst[1]:+.1f} g，出现在 {location}。")
        lines.append("")
        lines.append("正值只表示相同油门下下行稳态推力更小。其根因可能包含电调策略、"
                     "热状态、供电和装配气动，不能仅凭符号把它解释成减速更慢。")
    else:
        lines.append("**没有回程数据**，测不出迟滞。单向扫的话迟滞会整体表现成"
                     "「曲线偏了一点」，看不出来是迟滞。")
    lines.append("")

    sag = voltage_sag(measurements)
    if sag:
        worst_sag = max(sag.items(), key=lambda item: item[1])
        lines.append("## 掉压")
        lines.append("")
        lines.append(f"最大 {worst_sag[1]:.2f} V，出现在 {worst_sag[0]:g}% 档。"
                     "掉压直接改变推力，所以必须和推力一起记。")
        lines.append("")

    if voltage_model is not None:
        lines.append("## 电压模型")
        lines.append("")
        lines.append(f"`F(p, V) = F_ref(p) · (V/{voltage_model.reference_voltage_v:.2f})"
                     f"^{voltage_model.exponent:.3f}`（R² = {voltage_model.r_squared:.4f}）")
        lines.append("")
        lines.append("指数是拟出来的，不是写死的 2。理论上推力 ∝ 转速² 而转速 ∝ 电压，"
                     "但电调限流、桨失速、电池内阻都会把它拉低；写死 2 会在低电压端"
                     "系统性高估推力——正好是电量快没的时候。")
        lines.append("")

    if notes:
        lines.append("## 备注")
        lines.append("")
        lines.append(notes)
        lines.append("")

    path.write_text("\n".join(lines), encoding="utf-8")
    return path


def write_c_header(curve: ThrustCurve, path: Path, *, source: RpmSource,
                   points: int = 21, max_total_thrust_g: float | None = None,
                   voltage_layer_v: float | None = None) -> Path:
    """生成 `drv_coax_ctrl.c` 那张脉宽→推力表。

    注释里写清楚标定条件。表本身只是一串数字，半年后没有那行注释就没人知道
    它适不适用——而"拿一张不适用的表当成适用的"不会报错，只会让推力估计
    整体偏一截。
    """
    values = [curve(index * 100.0 / (points - 1)) for index in range(points)]
    peak = max(values) if values else 0.0
    lines = [
        "/* 由 tools/thrust_bench 生成，请勿手改。 */",
        "/*",
        f" * 转速来源：{source.quality}"
        + ("（实测）" if source.measured else "（推算，非实测）"),
    ]
    if voltage_layer_v is not None:
        lines.append(f" * 标定电压层：{voltage_layer_v:.2f} V"
                     "（换电压层要重标；推力对电压是近平方关系）")
    else:
        lines.append(" * **未做电压分层**：这张表只在标定当时那块电池那个电量下成立。")
    lines.append(f" * 曲线：F[g] = {curve.a:.6g}*p^2 + {curve.b:.6g}*p + "
                 f"{curve.c:.6g}，R^2 = {curve.r_squared:.4f}")
    lines.append(" */")
    lines.append("")
    lines.append(f"#define THRUST_TABLE_POINTS {points}U")
    if max_total_thrust_g is not None:
        lines.append(f"#define THRUST_TABLE_MAX_G {max_total_thrust_g:.3f}f")
    else:
        lines.append(f"#define THRUST_TABLE_MAX_G {peak:.3f}f")
    lines.append("")
    lines.append("static const float thrust_table_g[THRUST_TABLE_POINTS] = {")
    for index in range(0, points, 7):
        chunk = ", ".join(f"{value:.3f}f" for value in values[index:index + 7])
        lines.append(f"    {chunk},")
    lines.append("};")
    lines.append("")
    path.write_text("\n".join(lines), encoding="utf-8")
    return path

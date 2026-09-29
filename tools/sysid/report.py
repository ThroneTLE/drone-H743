"""辨识原始数据与报告导出。

写文件这件事有一条硬纪律：**原始批量样本必须先落盘，再谈结论**。
仓库的规矩是关于算法行为的结论要落在 `data/` 的实录上、并与基线同数据对比
（decoupling-spec D5-2）。一份只有结论没有原始数据的报告，半年后既无法复核也
无法重跑——而辨识结论恰恰是最需要被后来的人重新质疑的那类东西。

报告里**必须**出现的东西（缺一项就等于把不确定性藏起来）：
  * 推力是量出来的还是算出来的（`ThrustSource.quality`）；
  * 阻尼与偏心是否可分离；
  * 丢样与 GAP；
  * 前馈用的假定惯量，以及它和辨出来的比值——那个比值就是收敛进度。
"""
from __future__ import annotations

import csv
from datetime import date
from pathlib import Path

from .decode import SysIdBatch
from .profile import IdentProfile

ROOT = Path(__file__).resolve().parents[2]
ATTITUDE_DIR = ROOT / "data" / "identification" / "attitude"


def run_directory(when: date | None = None, base: Path | None = None) -> Path:
    """当日目录。仓库既有布局是 `data/identification/attitude/YYYY-MM-DD/`。"""
    base = base or ATTITUDE_DIR
    directory = base / (when or date.today()).isoformat()
    directory.mkdir(parents=True, exist_ok=True)
    return directory


def write_samples_csv(batches: list[SysIdBatch], path: Path) -> Path:
    """原始样本落盘。一行一个样本，时间戳是**固件的**微秒值。

    额外写一列 `gap`：这一行之前是否丢过样。丢样点在时间轴上是不连续的，
    后续任何差分/拟合都必须知道断在哪儿，否则会把断点当成一次巨大的角加速度。
    """
    if not batches:
        raise ValueError("没有样本可写")
    names = list(batches[0].samples[0].keys())
    with path.open("x", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["t_us", "run_id", "gap", *names])
        for batch in batches:
            stamps = batch.timestamps_us()
            for index, sample in enumerate(batch.samples):
                gap = 1 if (batch.gap and index == 0) else 0
                writer.writerow([stamps[index], batch.run_id, gap,
                                 *(sample[name] for name in names)])
    return path


def write_report(profile: IdentProfile, path: Path, *,
                 thrust_quality: str = "unknown",
                 dropped_samples: int = 0,
                 gap_count: int = 0,
                 samples: int = 0,
                 extra_notes: list[str] | None = None) -> Path:
    fit = profile.fit
    lines: list[str] = []
    lines.append(f"# 内环辨识报告 —— {profile.name}")
    lines.append("")
    lines.append(f"- 台架：{profile.rig.name}，杆轴方位角 {profile.rig.azimuth_deg:.1f}°")
    lines.append(f"- 杆轴相对质心偏移：{profile.rig.axis_offset_above_cg_m * 1000:.1f} mm")
    lines.append(f"- 飞控/IMU 相对质心偏移：{profile.rig.imu_above_cg_m * 1000:.1f} mm")
    lines.append(f"- 激励：{profile.excitation.profile_name}，"
                 f"幅值 {profile.excitation.amplitude_rad_s:.3f} rad/s，"
                 f"总时长 {profile.excitation.total_ms()} ms")
    lines.append(f"- 固件：{profile.firmware_id or '未记录'}")
    lines.append(f"- 样本：{samples} 条；丢样 {dropped_samples} 条；断点 {gap_count} 处")
    lines.append("")

    lines.append("## 辨识结果")
    lines.append("")
    lines.append("| 量 | 值 | 说明 |")
    lines.append("|---|---|---|")
    lines.append(_row("绕杆轴有效惯量 I_n", fit.inertia_kg_m2, "kg·m²",
                      "对称机体上等于 Ixx = Iyy"))
    lines.append(_row("阻尼 c", fit.damping_n_m_s, "N·m·s", "桨叶气动 + 轴承"))
    lines.append(_row("残余偏心 d_eff", fit.eccentricity_m, "m",
                      "仅在参数可分离且模型有效时解释为偏心估计"))
    lines.append(_row("总延迟 T_d", fit.delay_s, "s", "舵机 + 传输 + 控制拍"))
    lines.append(_row("拟合优度", fit.fit_percent, "%", "越高说明模型描述得越好"))
    lines.append("")

    lines.append("## 不确定性")
    lines.append("")
    lines.append(f"- **推力来源**：`{thrust_quality}`。"
                 "推力若是算出来的，它的相对误差会**原样**变成惯量的相对误差——"
                 "τ = P·eff·l·F·sin(tilt) 里 F 和 I 是乘除关系。")
    correlation = fit.damping_eccentricity_correlation
    if correlation is None:
        lines.append("- 阻尼与偏心的相关性：未评估。")
    elif abs(correlation) >= 0.95:
        lines.append(f"- **阻尼与偏心不可分离**（相关系数 {correlation:.3f}）："
                     "这段激励里两者都只表现为「回中」。"
                     "换不同频率的 chirp 重测可以分开；在此之前这两个数不要单独引用。")
    else:
        lines.append(f"- 阻尼与偏心可分离（相关系数 {correlation:.3f}）。")
    if dropped_samples:
        lines.append(f"- 丢样 {dropped_samples} 条：时间轴在断点处不连续，"
                     "跨断点的差分无效，CSV 里的 `gap` 列标出了位置。")
    lines.append("- 舵机延迟**不得**引用 `drv_servo_actuator_model.h` 的历史值："
                 "那组是停电机、±200 µs 大信号、总线舵机条件下测的。")
    lines.append("")

    lines.append("## 收敛进度")
    lines.append("")
    assumed = profile.assumed_inertia_kg_m2
    if assumed and fit.inertia_kg_m2:
        ratio = fit.inertia_kg_m2 / assumed
        lines.append(f"- 前馈假定惯量 {assumed:.6g} kg·m²，辨出 {fit.inertia_kg_m2:.6g}，"
                     f"比值 **{ratio:.3f}**。")
        if abs(ratio - 1.0) < 0.05:
            lines.append("- 比值已经收到 1 附近，仍需独立重复采集核验，接近初值不能作为精度证据。")
        else:
            lines.append("- 比值离 1 还远：把辨出的惯量填回假定值再跑一轮。"
                         "并用不同激励复测；结果受推力与力臂标定误差影响。")
    else:
        lines.append("- 缺少假定惯量或辨识结果，无法给出收敛进度。")
    lines.append("")

    lines.append("## 候选增益（**只写 RAM，未落 Flash**）")
    lines.append("")
    commands = profile.gains.param_commands()
    if commands:
        lines.append("```")
        lines.extend(commands)
        lines.append("```")
        lines.append("")
        lines.append("是否写 Flash 由作者在验证之后单独决定；本程序不会自动 `SAVE`。")
    else:
        lines.append("本次没有合成候选增益。")
    lines.append("")

    if extra_notes:
        lines.append("## 备注")
        lines.append("")
        lines.extend(f"- {note}" for note in extra_notes)
        lines.append("")

    path.write_text("\n".join(lines), encoding="utf-8")
    return path


def _row(label: str, value: float | None, unit: str, note: str) -> str:
    shown = "未辨出" if value is None else f"{value:.6g} {unit}"
    return f"| {label} | {shown} | {note} |"

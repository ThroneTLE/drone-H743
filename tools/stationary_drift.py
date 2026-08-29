#!/usr/bin/env python3
"""静止漂移自检：飞机不动放 30~60 秒，看它自己以为发生了什么。

为什么需要这一项：
    六面标定解出来的是"摆在六个姿态下读数对不对"，但用户真正在意的是"放着不动
    时它会不会自己飘"。这两件事不等价 —— 系数写对了、静止时照样可能因为零偏残留
    或融合没收敛而漂。

    这个检查不需要转台：飞机不动，真实角速度就是 0，真实比力就是 1 g。任何非零
    读数都是误差本身，不需要外部基准。

    也因此它是 APPLY 前后 A/B 对比的天然载体：同一张桌子、同一段时长，标定前测
    一次、标定后测一次，差值就是这次标定的实际收益。

数据来源是 10 Hz 的遥测流（不是 IMUCAP 的 1 kHz 缓冲），因为 IMUCAP 只有 6144
个样本 ≈ 6.1 秒，装不下 30 秒以上的窗口。漂移是低频量，10 Hz 足够。
"""

from __future__ import annotations

import json
import math
import statistics
from dataclasses import asdict, dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Sequence

try:
    from .project_paths import IMU_METROLOGY_CALIBRATION_DIR, dated_directory, ensure_directory
except ImportError:  # pragma: no cover - 脚本式运行
    from project_paths import IMU_METROLOGY_CALIBRATION_DIR, dated_directory, ensure_directory


Vec3 = tuple[float, float, float]

REPORT_FORMAT = "drone-h743-stationary-drift"
REPORT_SCHEMA = 1
DRIFT_DIRNAME = "stationary_drift"

# 30 秒以下测不出漂移趋势：10 Hz 下噪声还没被平均掉，斜率全是噪声。
MIN_DURATION_S = 30.0
RECOMMENDED_DURATION_S = 60.0
MIN_SAMPLES = 200

# 时间戳缺口：超过中位间隔这个倍数就算掉了一拍。
GAP_FACTOR = 3.0
GAP_FRACTION_WARN = 0.02
# 单个大洞按比例算不出来（60 秒里断一次只占 0.2%），但那几秒飞机干了什么完全没
# 记录，必须单独拦一道。
MAX_GAP_S = 2.0

# 姿态漂移门限（°/min）。
# roll/pitch 有重力持续修正，本来就不该漂；yaw 没有绝对参考，只能靠陀螺积分，
# 漂一点是正常的 —— MEMS 陀螺零偏残留 0.01 dps 就是 0.6 °/min。
LEVEL_DRIFT_PASS_DEG_PER_MIN = 0.3
LEVEL_DRIFT_WARN_DEG_PER_MIN = 1.0
YAW_DRIFT_PASS_DEG_PER_MIN = 1.0
YAW_DRIFT_WARN_DEG_PER_MIN = 5.0

# 静止时 |a| 应该正好是 1 g；偏差直接反映加速度计标定质量。
ACCEL_NORM_PASS_G = 0.010
ACCEL_NORM_WARN_G = 0.030

# 静止时陀螺读数应该是 0；残留零偏。
GYRO_BIAS_PASS_DPS = 0.05
GYRO_BIAS_WARN_DPS = 0.20

# IMU 自热：刚上电的几分钟温度会往上走，零偏跟着动。跨度太大说明还没热平衡。
TEMPERATURE_SETTLED_C = 2.0


@dataclass(frozen=True)
class DriftSample:
    """一帧遥测。角度用飞控融合出来的姿态，它是板上 1 kHz 积分的结果。"""

    timestamp_s: float
    gyro_dps: Vec3
    accel_g: Vec3
    attitude_deg: Vec3
    temperature_c: float
    sequence: int


@dataclass(frozen=True)
class DriftReport:
    duration_s: float
    sample_count: int
    sample_rate_hz: float

    gyro_mean_dps: Vec3
    gyro_std_dps: Vec3
    accel_mean_g: Vec3
    accel_std_g: Vec3
    accel_norm_g: float

    # 主机侧对 10 Hz 读数积分；和 attitude_drift_deg 对不上说明采样太稀。
    gyro_integral_deg: Vec3
    attitude_drift_deg: Vec3
    attitude_drift_deg_per_min: Vec3

    temperature_start_c: float
    temperature_end_c: float
    temperature_span_c: float

    median_interval_s: float
    gap_count: int
    gap_fraction: float
    max_gap_s: float
    stale_snapshots: int

    status: str
    findings: tuple[str, ...]
    context: dict[str, Any] = field(default_factory=dict)


def _mean3(rows: Sequence[Vec3]) -> Vec3:
    return tuple(statistics.fmean(row[i] for row in rows) for i in range(3))  # type: ignore[return-value]


def _std3(rows: Sequence[Vec3]) -> Vec3:
    if len(rows) < 2:
        return (0.0, 0.0, 0.0)
    return tuple(statistics.pstdev([row[i] for row in rows]) for i in range(3))  # type: ignore[return-value]


def _wrapped(previous: float, current: float) -> float:
    """偏航过 ±180° 会翻绕，按最短弧累加才不会凭空多出 360°。"""
    return (current - previous + 180.0) % 360.0 - 180.0


def _worst(*statuses: str) -> str:
    order = ("PASS", "WARN", "FAIL", "INCOMPLETE")
    return max(statuses, key=order.index)


def _grade(value: float, pass_limit: float, warn_limit: float) -> str:
    magnitude = abs(value)
    if magnitude <= pass_limit:
        return "PASS"
    if magnitude <= warn_limit:
        return "WARN"
    return "FAIL"


def analyze_drift(samples: Sequence[DriftSample], *,
                  context: dict[str, Any] | None = None) -> DriftReport:
    """把一段静止录制折算成"它自己以为发生了什么"。"""

    rows = list(samples)
    context = dict(context or {})
    if len(rows) < 2:
        return DriftReport(
            0.0, len(rows), 0.0,
            (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.0,
            (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0),
            0.0, 0.0, 0.0, 0.0, 0, 0.0, 0.0, 0,
            "INCOMPLETE", ("样本不足，无法判断漂移。",), context,
        )

    times = [row.timestamp_s for row in rows]
    duration = times[-1] - times[0]
    intervals = [b - a for a, b in zip(times, times[1:])]
    positive = [value for value in intervals if value > 0.0]
    median_interval = statistics.median(positive) if positive else 0.0
    gaps = [value for value in intervals if median_interval > 0.0 and value > median_interval * GAP_FACTOR]
    gap_fraction = len(gaps) / len(intervals) if intervals else 0.0
    max_gap = max(intervals) if intervals else 0.0
    rate_hz = (len(rows) - 1) / duration if duration > 0.0 else 0.0
    # seq 是飞控 1 kHz 的快照序号。主机按 10 Hz 轮询，本来就会跳一百多号 ——
    # 跳号是正常的，不算异常。真正有问题的是同号重复（读到同一份陈旧快照）或
    # 号码倒退（飞控中途重启），这两种会让积分算错。
    stale_snapshots = sum(
        1 for a, b in zip(rows, rows[1:])
        if b.sequence > 0 and a.sequence > 0 and b.sequence <= a.sequence
    )

    gyro_rows = [row.gyro_dps for row in rows]
    accel_rows = [row.accel_g for row in rows]
    gyro_mean = _mean3(gyro_rows)
    gyro_std = _std3(gyro_rows)
    accel_mean = _mean3(accel_rows)
    accel_std = _std3(accel_rows)
    accel_norm = math.sqrt(sum(value * value for value in accel_mean))

    integral = [0.0, 0.0, 0.0]
    for previous, current in zip(rows, rows[1:]):
        step = current.timestamp_s - previous.timestamp_s
        if step <= 0.0:
            continue
        for axis in range(3):
            integral[axis] += (previous.gyro_dps[axis] + current.gyro_dps[axis]) * 0.5 * step

    attitude_drift = [0.0, 0.0, 0.0]
    for previous, current in zip(rows, rows[1:]):
        for axis in range(3):
            attitude_drift[axis] += _wrapped(previous.attitude_deg[axis], current.attitude_deg[axis])
    minutes = duration / 60.0 if duration > 0.0 else 0.0
    per_minute = tuple(value / minutes if minutes > 0.0 else 0.0 for value in attitude_drift)

    temperatures = [row.temperature_c for row in rows]
    span = max(temperatures) - min(temperatures)

    findings: list[str] = []
    status = "PASS"
    if duration < MIN_DURATION_S:
        status = "INCOMPLETE"
        findings.append(
            f"只录了 {duration:.1f} 秒，至少要 {MIN_DURATION_S:.0f} 秒才看得出漂移趋势。")
    if len(rows) < MIN_SAMPLES:
        status = "INCOMPLETE"
        findings.append(f"只有 {len(rows)} 帧，至少要 {MIN_SAMPLES} 帧。")

    if gap_fraction > GAP_FRACTION_WARN:
        status = _worst(status, "WARN")
        findings.append(
            f"{gap_fraction:.1%} 的采样间隔超过中位值 {GAP_FACTOR:.0f} 倍"
            f"（最大 {max_gap:.2f}s）：遥测掉帧，积分结果会偏。")
    elif max_gap > MAX_GAP_S:
        status = _worst(status, "WARN")
        findings.append(
            f"中间断了 {max_gap:.1f} 秒（只占 {gap_fraction:.1%}，按比例看不出来）："
            "那几秒遥测掉帧、飞机状态没有记录，积分结果会偏。")
    if stale_snapshots > 0:
        status = _worst(status, "WARN")
        findings.append(
            f"有 {stale_snapshots} 帧读到的是重复或倒退的快照序号："
            "同一份数据被算了两次，或者飞控中途重启过，积分结果不可信。")

    level_status = _worst(
        _grade(per_minute[0], LEVEL_DRIFT_PASS_DEG_PER_MIN, LEVEL_DRIFT_WARN_DEG_PER_MIN),
        _grade(per_minute[1], LEVEL_DRIFT_PASS_DEG_PER_MIN, LEVEL_DRIFT_WARN_DEG_PER_MIN))
    if level_status != "PASS":
        findings.append(
            f"横滚/俯仰在漂：{per_minute[0]:+.2f} / {per_minute[1]:+.2f} °/min。"
            "这两个角有重力持续修正，本来不该漂 —— 说明加速度计修正没起作用，"
            "或者桌子/机体在动。")
    yaw_status = _grade(per_minute[2], YAW_DRIFT_PASS_DEG_PER_MIN, YAW_DRIFT_WARN_DEG_PER_MIN)
    if yaw_status != "PASS":
        findings.append(
            f"偏航漂移 {per_minute[2]:+.2f} °/min：偏航没有绝对参考、只能靠陀螺积分，"
            "漂一点正常，但这个量级说明陀螺零偏残留偏大。")

    norm_status = _grade(accel_norm - 1.0, ACCEL_NORM_PASS_G, ACCEL_NORM_WARN_G)
    if norm_status != "PASS":
        findings.append(
            f"静止时合加速度 {accel_norm:.4f} g，偏离 1 g 达 {abs(accel_norm - 1.0) * 1000:.1f} mg："
            "加速度计标定还有残差。")

    gyro_worst = max(abs(value) for value in gyro_mean)
    gyro_status = _grade(gyro_worst, GYRO_BIAS_PASS_DPS, GYRO_BIAS_WARN_DPS)
    if gyro_status != "PASS":
        findings.append(
            f"静止时陀螺读数不是 0，最大 {gyro_worst:.3f} dps：零偏没扣干净。")

    if span > TEMPERATURE_SETTLED_C:
        findings.append(
            f"录制期间温度变了 {span:.1f} °C（{temperatures[0]:.1f} → {temperatures[-1]:.1f}）："
            "IMU 还在自热，零偏会跟着动。等它稳下来再测，A/B 对比才有意义。")

    if status != "INCOMPLETE":
        status = _worst(status, level_status, yaw_status, norm_status, gyro_status)
    if not findings:
        findings.append("静止表现正常：角度不漂、合加速度贴着 1 g、陀螺读数接近 0。")

    return DriftReport(
        duration_s=duration, sample_count=len(rows), sample_rate_hz=rate_hz,
        gyro_mean_dps=gyro_mean, gyro_std_dps=gyro_std,
        accel_mean_g=accel_mean, accel_std_g=accel_std, accel_norm_g=accel_norm,
        gyro_integral_deg=(integral[0], integral[1], integral[2]),
        attitude_drift_deg=(attitude_drift[0], attitude_drift[1], attitude_drift[2]),
        attitude_drift_deg_per_min=per_minute,  # type: ignore[arg-type]
        temperature_start_c=temperatures[0], temperature_end_c=temperatures[-1],
        temperature_span_c=span,
        median_interval_s=median_interval, gap_count=len(gaps),
        gap_fraction=gap_fraction, max_gap_s=max_gap, stale_snapshots=stale_snapshots,
        status=status, findings=tuple(findings), context=context,
    )


def summarise(report: DriftReport) -> str:
    """一行人话结论。"""
    return (
        f"{report.status} · {report.duration_s:.0f}s/{report.sample_count}帧 · "
        f"横滚{report.attitude_drift_deg_per_min[0]:+.2f} "
        f"俯仰{report.attitude_drift_deg_per_min[1]:+.2f} "
        f"偏航{report.attitude_drift_deg_per_min[2]:+.2f} °/min · "
        f"|a|={report.accel_norm_g:.4f}g · "
        f"陀螺零偏max={max(abs(v) for v in report.gyro_mean_dps):.3f}dps · "
        f"温度{report.temperature_span_c:+.1f}°C"
    )


def compare(before: DriftReport, after: DriftReport) -> tuple[str, ...]:
    """APPLY 前后的 A/B：只报会变的那几项，并说清楚变好还是变差。"""

    lines: list[str] = []

    def row(label: str, old: float, new: float, unit: str, *, lower_is_better: bool = True) -> None:
        delta = new - old
        if lower_is_better:
            better = abs(new) < abs(old)
        else:
            better = new > old
        mark = "改善" if better else ("变差" if abs(delta) > 0.0 else "持平")
        lines.append(f"{label}: {old:+.4f} → {new:+.4f} {unit}（{delta:+.4f}，{mark}）")

    for index, name in enumerate(("横滚", "俯仰", "偏航")):
        row(f"{name}漂移", before.attitude_drift_deg_per_min[index],
            after.attitude_drift_deg_per_min[index], "°/min")
    row("合加速度偏离 1g", before.accel_norm_g - 1.0, after.accel_norm_g - 1.0, "g")
    row("陀螺零偏最大轴",
        max(before.gyro_mean_dps, key=abs), max(after.gyro_mean_dps, key=abs), "dps")

    if before.context.get("cal_generation") == after.context.get("cal_generation"):
        lines.append(
            "注意：两次录制的标定代次相同，这不是标定前后的对比，"
            "差值只反映重复性。")
    if max(before.temperature_span_c, after.temperature_span_c) > TEMPERATURE_SETTLED_C:
        lines.append("注意：至少有一次录制期间温度没稳住，差值里混着自热的影响。")
    return tuple(lines)


def report_to_dict(report: DriftReport) -> dict[str, Any]:
    payload = asdict(report)
    payload["format"] = REPORT_FORMAT
    payload["schema"] = REPORT_SCHEMA
    return payload


def report_from_dict(payload: Any) -> DriftReport:
    if not isinstance(payload, dict):
        raise TypeError("drift report must be an object")
    if payload.get("format") != REPORT_FORMAT or payload.get("schema") != REPORT_SCHEMA:
        raise ValueError("unsupported stationary drift report format/schema")
    fields = {key: value for key, value in payload.items() if key not in ("format", "schema")}
    for key in ("gyro_mean_dps", "gyro_std_dps", "accel_mean_g", "accel_std_g",
                "gyro_integral_deg", "attitude_drift_deg", "attitude_drift_deg_per_min"):
        fields[key] = tuple(float(value) for value in fields[key])
    fields["findings"] = tuple(str(value) for value in fields["findings"])
    return DriftReport(**fields)


def write_report(report: DriftReport, *, now: datetime | None = None,
                 root: Path = IMU_METROLOGY_CALIBRATION_DIR) -> Path:
    value = now or datetime.now().astimezone()
    directory = ensure_directory(dated_directory(root, value) / DRIFT_DIRNAME)
    path = directory / ("drift_" + value.strftime("%Y%m%d_%H%M%S") + ".json")
    path.write_text(
        json.dumps(report_to_dict(report), ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8")
    return path


def load_report(path: Path) -> DriftReport:
    return report_from_dict(json.loads(path.read_text(encoding="utf-8")))


def recent_reports(*, limit: int = 8,
                   root: Path = IMU_METROLOGY_CALIBRATION_DIR) -> list[tuple[Path, DriftReport]]:
    """按时间倒序列出最近的录制，用来挑 A/B 的对照组。"""
    if not root.exists():
        return []
    paths = sorted(root.glob(f"20??-??-??/{DRIFT_DIRNAME}/drift_*.json"),
                   key=lambda item: (item.stat().st_mtime_ns, str(item)), reverse=True)
    result: list[tuple[Path, DriftReport]] = []
    for path in paths[:limit]:
        try:
            result.append((path, load_report(path)))
        except (OSError, ValueError, TypeError, json.JSONDecodeError):
            continue
    return result


__all__ = [
    "DRIFT_DIRNAME", "DriftReport", "DriftSample", "MIN_DURATION_S",
    "RECOMMENDED_DURATION_S", "analyze_drift", "compare", "load_report",
    "recent_reports", "report_from_dict", "report_to_dict", "summarise", "write_report",
]

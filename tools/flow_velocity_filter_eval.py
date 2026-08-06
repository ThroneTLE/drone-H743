#!/usr/bin/env python3
"""Evaluate optical-flow velocity robustness from H743 flight-log CSV files.

这个脚本用于对比旧的光流直通速度和当前固件里的抗离群思路：

1. 原始 Micolink 光流: raw_flow * 0.01 * height
2. 5 点中值滤波后的光流速度
3. 质量自适应的一阶离线 Kalman 速度估计

它只依赖 CSV 字段，不控制飞机、不写串口，适合每次导出日志后快速复盘。
"""

from __future__ import annotations

import argparse
import csv
import math
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


MIN_QUALITY = 80
QUALITY_HIGH = 180
FLOW_NOISE_MIN_M_S = 0.04
FLOW_NOISE_MAX_M_S = 0.45
MEDIAN_WINDOW = 5
MEDIAN_MIN_SAMPLES = 3
FLOW_SCALE = 0.01
NIS_GATE = 0.0
PROCESS_NOISE_M_S2 = 1.50
PREDICT_LEAK_HZ = 1.50
MAX_SPEED_M_S = 2.50
FLOW_SOFT_HOLD_S = 0.150
FLOW_STALE_RESET_S = 0.250
FLOW_LOST_DECAY_HZ = 1.0
FLOW_STALE_DECAY_HZ = 12.0
ZERO_FLOW_SPEED_M_S = 0.035
ZERO_ACCEL_M_S2 = 0.30
ZERO_FLOW_COUNT = 8


@dataclass
class SeriesStats:
    name: str
    samples: int
    rms_speed_m_s: float
    peak_speed_m_s: float
    spike_count: int
    drift_x_m: float
    drift_y_m: float
    drift_norm_m: float


@dataclass
class ReplayResult:
    rows: int
    duration_s: float
    usable_flow_samples: int
    median_wait_samples: int
    quality_reject_samples: int
    gate_reject_samples: int
    accepted_samples: int
    stats: list[SeriesStats]


def parse_float(row: dict[str, str], name: str, default: float = 0.0) -> float:
    text = row.get(name, "")
    if text in ("", "None", "nan", "NaN"):
        return default
    try:
        return float(text)
    except ValueError:
        return default


def parse_int(row: dict[str, str], name: str, default: int = 0) -> int:
    text = row.get(name, "")
    if text in ("", "None"):
        return default
    try:
        return int(float(text))
    except ValueError:
        return default


def time_s(row: dict[str, str]) -> float:
    if row.get("timestamp_us"):
        return parse_float(row, "timestamp_us") * 1.0e-6
    return parse_float(row, "tick_ms") * 1.0e-3


def quality_to_noise(quality: int) -> float:
    if quality <= MIN_QUALITY:
        return FLOW_NOISE_MAX_M_S
    if quality >= QUALITY_HIGH:
        return FLOW_NOISE_MIN_M_S
    norm = (quality - MIN_QUALITY) / float(QUALITY_HIGH - MIN_QUALITY)
    weak = 1.0 - max(0.0, min(1.0, norm))
    return FLOW_NOISE_MIN_M_S + (FLOW_NOISE_MAX_M_S - FLOW_NOISE_MIN_M_S) * weak * weak


def median_i(values: Iterable[int]) -> int:
    ordered = sorted(values)
    return int(ordered[len(ordered) // 2])


def vector_stats(
    name: str,
    values: list[tuple[float, float, float]],
    spike_threshold_m_s: float,
) -> SeriesStats:
    if not values:
        return SeriesStats(name, 0, 0.0, 0.0, 0, 0.0, 0.0, 0.0)

    sum_speed_sq = 0.0
    peak_speed = 0.0
    spike_count = 0
    drift_x = 0.0
    drift_y = 0.0
    last_t = values[0][0]
    for t, vx, vy in values:
        speed = math.hypot(vx, vy)
        sum_speed_sq += speed * speed
        peak_speed = max(peak_speed, speed)
        if speed >= spike_threshold_m_s:
            spike_count += 1
        dt = max(0.0, min(0.05, t - last_t))
        drift_x += vx * dt
        drift_y += vy * dt
        last_t = t

    return SeriesStats(
        name=name,
        samples=len(values),
        rms_speed_m_s=math.sqrt(sum_speed_sq / len(values)),
        peak_speed_m_s=peak_speed,
        spike_count=spike_count,
        drift_x_m=drift_x,
        drift_y_m=drift_y,
        drift_norm_m=math.hypot(drift_x, drift_y),
    )


def load_rows(path: Path, start_s: float | None, end_s: float | None) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8-sig", newline="") as f:
        rows = list(csv.DictReader(f))

    if not rows:
        return []
    t0 = time_s(rows[0])
    filtered: list[dict[str, str]] = []
    for row in rows:
        rel = time_s(row) - t0
        if start_s is not None and rel < start_s:
            continue
        if end_s is not None and rel > end_s:
            continue
        filtered.append(row)
    return filtered


def replay(rows: list[dict[str, str]], spike_threshold_m_s: float) -> ReplayResult:
    raw_direct: list[tuple[float, float, float]] = []
    corrected_direct: list[tuple[float, float, float]] = []
    filtered_ekf: list[tuple[float, float, float]] = []
    logged_est: list[tuple[float, float, float]] = []

    flow_x_window: deque[int] = deque(maxlen=MEDIAN_WINDOW)
    flow_y_window: deque[int] = deque(maxlen=MEDIAN_WINDOW)
    vel_x = 0.0
    vel_y = 0.0
    cov_x = 0.25
    cov_y = 0.25
    last_t: float | None = None
    last_update_t: float | None = None
    last_flow_age: int | None = None
    zero_flow_count = 0
    usable = 0
    median_wait = 0
    quality_reject = 0
    gate_reject = 0
    accepted = 0

    for row in rows:
        t = time_s(row)
        if last_t is None:
            dt = 0.004
        else:
            dt = max(0.0, min(0.05, t - last_t))
        last_t = t

        if last_update_t is None:
            flow_update_age_s = None
        else:
            flow_update_age_s = max(0.0, t - last_update_t)

        leak_hz = PREDICT_LEAK_HZ
        if flow_update_age_s is not None:
            leak_hz = FLOW_LOST_DECAY_HZ
            if flow_update_age_s > FLOW_SOFT_HOLD_S:
                leak_hz = FLOW_STALE_DECAY_HZ
        leak = max(0.0, min(1.0, 1.0 - leak_hz * dt))
        vel_x *= leak
        vel_y *= leak
        q = (PROCESS_NOISE_M_S2 * dt) ** 2
        cov_x = min(4.0, cov_x + q)
        cov_y = min(4.0, cov_y + q)

        raw_x = parse_int(row, "flow_raw_x")
        raw_y = parse_int(row, "flow_raw_y")
        height = parse_float(row, "flow_height_m", parse_float(row, "flow_height_raw_m", 0.0))
        quality = parse_int(row, "flow_quality")
        flow_valid = parse_int(row, "flow_valid", 1)
        velocity_valid = parse_int(row, "flow_velocity_valid", flow_valid)
        height_valid = parse_int(row, "flow_height_valid", 1 if height > 0.0 else 0)
        flow_age = parse_int(row, "flow_sample_age_ms", 0)
        new_sample = last_flow_age is None or flow_age <= last_flow_age
        last_flow_age = flow_age

        raw_vx = raw_x * FLOW_SCALE * height
        raw_vy = raw_y * FLOW_SCALE * height
        raw_direct.append((t, raw_vx, raw_vy))

        corr_vx = parse_float(row, "flow_corrected_velocity_m_s_0", raw_vx)
        corr_vy = parse_float(row, "flow_corrected_velocity_m_s_1", raw_vy)
        corrected_direct.append((t, corr_vx, corr_vy))
        acc_x = parse_float(row, "acc_nav_m_s2_0", 0.0)
        acc_y = parse_float(row, "acc_nav_m_s2_1", 0.0)

        if "vel_est_m_s_0" in row and "vel_est_m_s_1" in row:
            logged_est.append(
                (t, parse_float(row, "vel_est_m_s_0"), parse_float(row, "vel_est_m_s_1"))
            )

        if not new_sample:
            if flow_update_age_s is not None and flow_update_age_s > FLOW_STALE_RESET_S:
                vel_x = 0.0
                vel_y = 0.0
                zero_flow_count = 0
            filtered_ekf.append((t, vel_x, vel_y))
            continue
        if flow_valid == 0 or velocity_valid == 0 or height_valid == 0 or height <= 0.0:
            if flow_update_age_s is not None and flow_update_age_s > FLOW_STALE_RESET_S:
                vel_x = 0.0
                vel_y = 0.0
                zero_flow_count = 0
            filtered_ekf.append((t, vel_x, vel_y))
            continue
        if quality < MIN_QUALITY:
            quality_reject += 1
            flow_x_window.clear()
            flow_y_window.clear()
            zero_flow_count = 0
            filtered_ekf.append((t, vel_x, vel_y))
            continue

        usable += 1
        flow_x_window.append(raw_x)
        flow_y_window.append(raw_y)
        if len(flow_x_window) < MEDIAN_MIN_SAMPLES:
            median_wait += 1
            filtered_ekf.append((t, vel_x, vel_y))
            continue

        med_vx = median_i(flow_x_window) * FLOW_SCALE * height
        med_vy = median_i(flow_y_window) * FLOW_SCALE * height
        med_vx += parse_float(row, "flow_optical_rot_comp_m_s_0", 0.0)
        med_vx += parse_float(row, "flow_offset_rot_comp_m_s_0", 0.0)
        med_vy += parse_float(row, "flow_optical_rot_comp_m_s_1", 0.0)
        med_vy += parse_float(row, "flow_offset_rot_comp_m_s_1", 0.0)
        speed = math.hypot(med_vx, med_vy)
        if speed > MAX_SPEED_M_S:
            gate_reject += 1
            flow_x_window.clear()
            flow_y_window.clear()
            zero_flow_count = 0
            filtered_ekf.append((t, vel_x, vel_y))
            continue

        noise = quality_to_noise(quality)
        rx = noise * noise
        innov_x = med_vx - vel_x
        innov_y = med_vy - vel_y
        nis = (innov_x * innov_x) / (cov_x + rx) + (innov_y * innov_y) / (cov_y + rx)
        if NIS_GATE > 0.0 and nis > NIS_GATE:
            gate_reject += 1
            filtered_ekf.append((t, vel_x, vel_y))
            continue

        gain_x = cov_x / (cov_x + rx)
        gain_y = cov_y / (cov_y + rx)
        vel_x += gain_x * innov_x
        vel_y += gain_y * innov_y
        cov_x *= 1.0 - gain_x
        cov_y *= 1.0 - gain_y
        last_update_t = t
        if speed <= ZERO_FLOW_SPEED_M_S and math.hypot(acc_x, acc_y) <= ZERO_ACCEL_M_S2:
            zero_flow_count = min(ZERO_FLOW_COUNT, zero_flow_count + 1)
            if zero_flow_count >= ZERO_FLOW_COUNT:
                vel_x = 0.0
                vel_y = 0.0
        else:
            zero_flow_count = 0
        accepted += 1
        filtered_ekf.append((t, vel_x, vel_y))

    duration = 0.0
    if len(rows) >= 2:
        duration = max(0.0, time_s(rows[-1]) - time_s(rows[0]))

    stats = [
        vector_stats("old_raw_height_direct", raw_direct, spike_threshold_m_s),
        vector_stats("old_corrected_direct", corrected_direct, spike_threshold_m_s),
        vector_stats("new_median_quality_ekf", filtered_ekf, spike_threshold_m_s),
    ]
    if logged_est:
        stats.append(vector_stats("logged_firmware_vel_est", logged_est, spike_threshold_m_s))

    return ReplayResult(
        rows=len(rows),
        duration_s=duration,
        usable_flow_samples=usable,
        median_wait_samples=median_wait,
        quality_reject_samples=quality_reject,
        gate_reject_samples=gate_reject,
        accepted_samples=accepted,
        stats=stats,
    )


def print_result(path: Path, result: ReplayResult) -> None:
    print(f"文件: {path}")
    print(f"样本: {result.rows}, 时长: {result.duration_s:.3f} s")
    print(
        "光流重放: "
        f"可用={result.usable_flow_samples}, "
        f"中值预热={result.median_wait_samples}, "
        f"质量拒绝={result.quality_reject_samples}, "
        f"门限拒绝={result.gate_reject_samples}, "
        f"融合接受={result.accepted_samples}"
    )
    print("")
    print("series,rms_m_s,peak_m_s,spikes,drift_x_m,drift_y_m,drift_norm_m")
    for stat in result.stats:
        print(
            f"{stat.name},"
            f"{stat.rms_speed_m_s:.5f},"
            f"{stat.peak_speed_m_s:.5f},"
            f"{stat.spike_count},"
            f"{stat.drift_x_m:.5f},"
            f"{stat.drift_y_m:.5f},"
            f"{stat.drift_norm_m:.5f}"
        )

    old_peak = result.stats[0].peak_speed_m_s if result.stats else 0.0
    new_peak = result.stats[2].peak_speed_m_s if len(result.stats) >= 3 else 0.0
    if old_peak > 1.0e-6:
        reduction = 100.0 * (1.0 - new_peak / old_peak)
        print("")
        print(f"峰值速度抑制: {reduction:.1f}%")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="重放 H743 飞行日志，评估光流速度滤波/EKF 对尖峰和漂移的抑制。"
    )
    parser.add_argument("csv", type=Path, help="flight_log_receive.py 导出的 CSV")
    parser.add_argument("--start-s", type=float, default=None, help="只分析相对起点后的时间")
    parser.add_argument("--end-s", type=float, default=None, help="只分析相对起点前的时间")
    parser.add_argument(
        "--spike-threshold",
        type=float,
        default=0.05,
        help="速度尖峰统计阈值，单位 m/s，默认 0.05",
    )
    args = parser.parse_args()

    rows = load_rows(args.csv, args.start_s, args.end_s)
    if not rows:
        print("没有可分析的 CSV 行。")
        return 1

    required = {"flow_raw_x", "flow_raw_y"}
    if not required.issubset(rows[0].keys()):
        print("这个 CSV 没有 flow_raw_x/flow_raw_y，无法重放光流滤波。")
        print("请刷入包含 APP_FLIGHT_LOG_VERSION 7 的固件后重新导出日志。")
        return 2

    result = replay(rows, args.spike_threshold)
    print_result(args.csv, result)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

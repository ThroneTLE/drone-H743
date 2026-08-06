#!/usr/bin/env python3
"""把 H743 飞行日志导出为 Rerun 现场回放。"""

from __future__ import annotations

import argparse
import math
import os
import subprocess
import sys
import tkinter as tk
from dataclasses import dataclass
from pathlib import Path
from tkinter import filedialog, messagebox, ttk
from typing import Any, Iterable

try:
    import numpy as np
    import pandas as pd
except Exception:  # pragma: no cover - depends on host optional packages
    np = None
    pd = None


ROOT_DIR = Path(__file__).resolve().parents[1]
DEFAULT_LOG_DIR = ROOT_DIR / "log"
DEFAULT_OUT_DIR = ROOT_DIR / ".tmp" / "rerun_replay"
DEFAULT_GAP_MS = 200.0
DEFAULT_MAX_ROWS_PER_SEGMENT = 2600
RERUN_INSTALL_HINT = "python -m pip install rerun-sdk"


@dataclass(frozen=True)
class ReplaySegment:
    index: int
    start_row: int
    end_row: int
    count: int
    duration_s: float
    sample_rate_hz: float
    start_sequence: int | None
    end_sequence: int | None
    missing_sequence_count: int
    reason_start: str
    reason_end: str
    throttle_min_us: float | None
    throttle_max_us: float | None
    max_abs_roll_deg: float | None
    max_abs_pitch_deg: float | None


def require_pandas() -> None:
    if pd is None or np is None:
        raise RuntimeError("缺少 pandas/numpy，请先执行：python -m pip install pandas numpy")


def require_rerun() -> Any:
    try:
        import rerun as rr
    except Exception as exc:  # pragma: no cover - depends on optional package
        raise RuntimeError(f"缺少 Rerun Python SDK，请先执行：{RERUN_INSTALL_HINT}") from exc
    return rr


def latest_csv_in(folder: Path) -> Path | None:
    if not folder.exists() or not folder.is_dir():
        return None
    files = [path for path in folder.glob("flightlog_*.csv") if path.is_file()]
    if not files:
        files = [path for path in folder.glob("*.csv") if path.is_file()]
    if not files:
        return None
    return max(files, key=lambda path: (path.stat().st_mtime, path.name))


def resolve_csv_path(path: Path | None) -> Path:
    candidate = path or DEFAULT_LOG_DIR
    if candidate.is_file():
        return candidate
    latest = latest_csv_in(candidate)
    if latest is None:
        raise FileNotFoundError(f"没有在 {candidate} 找到 CSV 日志")
    return latest


def safe_float(value: object) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def safe_int(value: object) -> int | None:
    result = safe_float(value)
    return int(result) if result is not None else None


def series_float(frame: "pd.DataFrame", column: str) -> "pd.Series":
    if column not in frame.columns:
        return pd.Series(np.zeros(len(frame), dtype=float), index=frame.index)
    return pd.to_numeric(frame[column], errors="coerce").astype(float)


def finite_or(value: object, default: float = 0.0) -> float:
    result = safe_float(value)
    return result if result is not None else default


def sequence_missing(values: Iterable[object]) -> int:
    missing = 0
    previous: int | None = None
    for value in values:
        current = safe_int(value)
        if current is None:
            continue
        if previous is not None and current > previous + 1:
            missing += current - previous - 1
        previous = current
    return missing


def split_segments(frame: "pd.DataFrame", gap_ms: float = DEFAULT_GAP_MS) -> list[ReplaySegment]:
    require_pandas()
    if frame.empty:
        return []
    if "timestamp_us" not in frame.columns:
        return [
            summarize_segment(frame, 0, 0, len(frame) - 1)
        ]

    timestamps = series_float(frame, "timestamp_us").to_numpy(dtype=float)
    sequences = series_float(frame, "sequence").to_numpy(dtype=float) if "sequence" in frame.columns else None
    starts = [0]
    gap_s = gap_ms * 0.001
    for index in range(1, len(frame)):
        before = timestamps[index - 1]
        current = timestamps[index]
        break_time = (
            not math.isfinite(before)
            or not math.isfinite(current)
            or ((current - before) * 1.0e-6 < 0.0)
            or ((current - before) * 1.0e-6 > gap_s)
        )
        break_sequence = False
        if sequences is not None:
            previous_seq = sequences[index - 1]
            current_seq = sequences[index]
            break_sequence = (
                math.isfinite(previous_seq)
                and math.isfinite(current_seq)
                and current_seq < previous_seq
            )
        if break_time or break_sequence:
            starts.append(index)

    starts.append(len(frame))
    return [
        summarize_segment(frame, segment_index, starts[segment_index], starts[segment_index + 1] - 1)
        for segment_index in range(len(starts) - 1)
    ]


def summarize_segment(frame: "pd.DataFrame", index: int, start_row: int, end_row: int) -> ReplaySegment:
    segment = frame.iloc[start_row : end_row + 1]
    first = segment.iloc[0]
    last = segment.iloc[-1]
    t0 = finite_or(first.get("timestamp_us"), 0.0)
    t1 = finite_or(last.get("timestamp_us"), t0)
    duration_s = max(0.0, (t1 - t0) * 1.0e-6)
    sample_rate_hz = ((len(segment) - 1) / duration_s) if duration_s > 0.0 and len(segment) > 1 else 0.0

    throttle = series_float(segment, "throttle_us").dropna()
    roll = series_float(segment, "roll_deg").dropna().abs()
    pitch = series_float(segment, "pitch_deg").dropna().abs()
    seq_values = segment["sequence"].to_list() if "sequence" in segment.columns else []

    return ReplaySegment(
        index=index,
        start_row=start_row,
        end_row=end_row,
        count=len(segment),
        duration_s=duration_s,
        sample_rate_hz=sample_rate_hz,
        start_sequence=safe_int(first.get("sequence")),
        end_sequence=safe_int(last.get("sequence")),
        missing_sequence_count=sequence_missing(seq_values),
        reason_start=str(first.get("motor_output_reason_name", "")),
        reason_end=str(last.get("motor_output_reason_name", "")),
        throttle_min_us=float(throttle.min()) if not throttle.empty else None,
        throttle_max_us=float(throttle.max()) if not throttle.empty else None,
        max_abs_roll_deg=float(roll.max()) if not roll.empty else None,
        max_abs_pitch_deg=float(pitch.max()) if not pitch.empty else None,
    )


def parse_segment_selection(text: str, count: int) -> list[int]:
    value = text.strip().lower()
    if value in ("", "all", "*", "全部"):
        return list(range(count))
    selected: set[int] = set()
    for part in value.split(","):
        item = part.strip()
        if not item:
            continue
        if "-" in item:
            start_text, end_text = item.split("-", 1)
            start = int(start_text)
            end = int(end_text)
            if start > end:
                start, end = end, start
            selected.update(range(start, end + 1))
        else:
            selected.add(int(item))
    return [index for index in sorted(selected) if 0 <= index < count]


def export_stride(count: int, max_rows: int, requested_stride: int) -> int:
    if requested_stride > 0:
        return requested_stride
    if max_rows <= 0 or count <= max_rows:
        return 1
    return max(1, math.ceil(count / max_rows))


def rrd_name(csv_path: Path, segment: ReplaySegment) -> str:
    reason = (segment.reason_start or "segment").replace(" ", "_").replace("/", "_")
    return f"{csv_path.stem}_seg{segment.index:02d}_{reason}_{segment.duration_s:.2f}s.rrd"


def rpy_body_to_local_down(roll_rad: float, pitch_rad: float, yaw_rad: float) -> "np.ndarray":
    # 中文注释：使用常见航向-俯仰-横滚顺序，坐标为 X前/Y右/Z下。
    cr, sr = math.cos(roll_rad), math.sin(roll_rad)
    cp, sp = math.cos(pitch_rad), math.sin(pitch_rad)
    cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=float,
    )


def local_down_to_rerun(point: Iterable[float]) -> list[float]:
    # 中文注释：控制器本地坐标 X前/Y右/Z下 转成 Rerun 右手 Z-up: X右/Y前/Z上。
    x_forward, y_right, z_down = list(point)
    return [float(y_right), float(x_forward), float(-z_down)]


def body_to_rerun(
    body_point: Iterable[float],
    position_local_down: "np.ndarray",
    rotation_body_to_local_down: "np.ndarray",
) -> list[float]:
    local_point = position_local_down + rotation_body_to_local_down @ np.asarray(list(body_point), dtype=float)
    return local_down_to_rerun(local_point)


def segment_time_s(segment: "pd.DataFrame") -> "pd.Series":
    if "timestamp_us" not in segment.columns:
        return pd.Series(np.arange(len(segment), dtype=float), index=segment.index)
    timestamps = series_float(segment, "timestamp_us")
    first = timestamps.dropna().iloc[0] if not timestamps.dropna().empty else 0.0
    return (timestamps - first) * 1.0e-6


def integrate_velocity_position(segment: "pd.DataFrame", time_s: "pd.Series") -> tuple["pd.Series", "pd.Series"]:
    vx = series_float(segment, "flow_corrected_velocity_m_s_0")
    vy = series_float(segment, "flow_corrected_velocity_m_s_1")
    x_values = [0.0]
    y_values = [0.0]
    times = time_s.to_numpy(dtype=float)
    for index in range(1, len(segment)):
        dt = times[index] - times[index - 1]
        if not math.isfinite(dt) or dt < 0.0 or dt > 0.2:
            dt = 0.0
        x_values.append(x_values[-1] + finite_or(vx.iloc[index], 0.0) * dt)
        y_values.append(y_values[-1] + finite_or(vy.iloc[index], 0.0) * dt)
    return pd.Series(x_values, index=segment.index), pd.Series(y_values, index=segment.index)


def position_columns(segment: "pd.DataFrame", time_s: "pd.Series") -> tuple["pd.Series", "pd.Series", "pd.Series"]:
    if "ctrl_velocity_integral_m_0" in segment.columns and "ctrl_velocity_integral_m_1" in segment.columns:
        x = series_float(segment, "ctrl_velocity_integral_m_0")
        y = series_float(segment, "ctrl_velocity_integral_m_1")
        if x.abs().max(skipna=True) > 1.0e-4 or y.abs().max(skipna=True) > 1.0e-4:
            return x, y, height_column(segment)
    x, y = integrate_velocity_position(segment, time_s)
    return x, y, height_column(segment)


def height_column(segment: "pd.DataFrame") -> "pd.Series":
    for column in ("flow_height_m", "flow_height_raw_m"):
        if column in segment.columns:
            return series_float(segment, column).clip(lower=-1.0, upper=3.0)
    return pd.Series(np.zeros(len(segment), dtype=float), index=segment.index)


def build_ground_grid(size_m: float = 2.0, step_m: float = 0.25) -> list[list[list[float]]]:
    lines: list[list[list[float]]] = []
    count = int(round(size_m / step_m))
    values = [(-size_m * 0.5) + step_m * index for index in range(count + 1)]
    for value in values:
        lines.append([[value, -size_m * 0.5, 0.0], [value, size_m * 0.5, 0.0]])
        lines.append([[-size_m * 0.5, value, 0.0], [size_m * 0.5, value, 0.0]])
    return lines


def drone_geometry(
    x_m: float,
    y_m: float,
    height_m: float,
    roll_deg: float,
    pitch_deg: float,
    yaw_deg: float,
    force_body_n: tuple[float, float, float],
) -> tuple[list[list[list[float]]], list[list[int]], list[list[list[float]]], list[list[int]]]:
    rotation = rpy_body_to_local_down(
        math.radians(roll_deg),
        math.radians(pitch_deg),
        math.radians(yaw_deg),
    )
    position = np.array([x_m, y_m, -height_m], dtype=float)

    body_strips_body = [
        [[-0.18, 0.0, 0.0], [0.35, 0.0, 0.0]],
        [[0.0, -0.22, 0.0], [0.0, 0.22, 0.0]],
        [[0.35, 0.0, 0.0], [0.25, -0.06, 0.0], [0.25, 0.06, 0.0], [0.35, 0.0, 0.0]],
        [[0.0, 0.0, -0.12], [0.0, 0.0, 0.30]],
    ]
    prop_center = np.array([0.0, 0.0, 0.25], dtype=float)
    prop_radius = 0.16
    prop_circle = [
        [
            prop_radius * math.cos(theta),
            prop_radius * math.sin(theta),
            prop_center[2],
        ]
        for theta in np.linspace(0.0, math.tau, 48)
    ]
    body_strips_body.append(prop_circle)

    body_lines = [
        [body_to_rerun(point, position, rotation) for point in strip]
        for strip in body_strips_body
    ]
    body_colors = [
        [255, 80, 60],
        [40, 170, 255],
        [255, 180, 40],
        [180, 180, 180],
        [160, 255, 160],
    ]

    force = np.array(force_body_n, dtype=float)
    norm = float(np.linalg.norm(force))
    if norm < 1.0e-6:
        force = np.array([0.0, 0.0, -1.0], dtype=float)
    else:
        force = -force / norm
    thrust_tip = prop_center + force * 0.35
    thrust_lines = [[
        body_to_rerun(prop_center, position, rotation),
        body_to_rerun(thrust_tip, position, rotation),
    ]]
    thrust_colors = [[255, 40, 40]]
    return body_lines, body_colors, thrust_lines, thrust_colors


def scalar_channels() -> list[tuple[str, str, float]]:
    return [
        ("signals/attitude/roll_deg", "roll_deg", 1.0),
        ("signals/attitude/pitch_deg", "pitch_deg", 1.0),
        ("signals/attitude/yaw_deg", "yaw_deg", 1.0),
        ("signals/target/roll_deg", "ctrl_target_attitude_rp_rad_0", 180.0 / math.pi),
        ("signals/target/pitch_deg", "ctrl_target_attitude_rp_rad_1", 180.0 / math.pi),
        ("signals/rate/gyro_x_dps", "gyro_x_dps", 1.0),
        ("signals/rate/gyro_y_dps", "gyro_y_dps", 1.0),
        ("signals/servo/alpha_cmd_us", "servo_alpha_us", 1.0),
        ("signals/servo/beta_cmd_us", "servo_beta_us", 1.0),
        ("signals/servo/alpha_fb_us", "servo_alpha_feedback_us", 1.0),
        ("signals/servo/beta_fb_us", "servo_beta_feedback_us", 1.0),
        ("signals/motor/throttle_us", "throttle_us", 1.0),
        ("signals/motor/upper_us", "motor_upper_us", 1.0),
        ("signals/motor/lower_us", "motor_lower_us", 1.0),
        ("signals/flow/height_m", "flow_height_m", 1.0),
        ("signals/flow/vx_m_s", "flow_corrected_velocity_m_s_0", 1.0),
        ("signals/flow/vy_m_s", "flow_corrected_velocity_m_s_1", 1.0),
        ("signals/control/moment_roll_n_m", "ctrl_moment_cmd_n_m_0", 1.0),
        ("signals/control/moment_pitch_n_m", "ctrl_moment_cmd_n_m_1", 1.0),
        ("signals/control/moment_utilization", "ctrl_moment_utilization", 1.0),
        ("signals/control/thrust_utilization", "ctrl_thrust_utilization", 1.0),
        ("signals/rc/ch1_us", "rc_ch1_us", 1.0),
        ("signals/rc/ch2_us", "rc_ch2_us", 1.0),
        ("signals/rc/ch3_us", "rc_ch3_us", 1.0),
    ]


def set_rerun_time(rr: Any, time_s: float, row_sequence: int) -> None:
    # 中文注释：兼容 Rerun 新旧 Python API；新版用 set_time，旧版用 set_time_seconds/sequence。
    if hasattr(rr, "set_time"):
        rr.set_time("flight_time", duration=time_s)
        rr.set_time("row", sequence=row_sequence)
    else:  # pragma: no cover - only for older rerun-sdk
        rr.set_time_seconds("flight_time", time_s)
        rr.set_time_sequence("row", row_sequence)


def rerun_scalars(rr: Any, value: float) -> Any:
    # 中文注释：Rerun 旧版叫 Scalar，新版叫 Scalars，这里运行时选择可用类型。
    scalar_type = getattr(rr, "Scalars", None)
    if scalar_type is None:
        scalar_type = getattr(rr, "Scalar")
    return scalar_type(value)


def export_segment_to_rrd(
    frame: "pd.DataFrame",
    csv_path: Path,
    segment: ReplaySegment,
    out_dir: Path,
    max_rows: int = DEFAULT_MAX_ROWS_PER_SEGMENT,
    stride: int = 0,
) -> Path:
    require_pandas()
    rr = require_rerun()
    out_dir.mkdir(parents=True, exist_ok=True)
    rrd_path = out_dir / rrd_name(csv_path, segment)
    segment_frame = frame.iloc[segment.start_row : segment.end_row + 1].copy()
    step = export_stride(len(segment_frame), max_rows, stride)
    segment_frame = segment_frame.iloc[::step].copy()
    time_s = segment_time_s(segment_frame)
    pos_x, pos_y, height = position_columns(segment_frame, time_s)
    path_points = [
        local_down_to_rerun([finite_or(x), finite_or(y), -finite_or(z)])
        for x, y, z in zip(pos_x, pos_y, height)
    ]

    app_id = f"drone_h743_{csv_path.stem}_seg{segment.index:02d}"
    rr.init(app_id, spawn=False)
    rr.save(str(rrd_path))
    rr.log("world/ground_grid", rr.LineStrips3D(build_ground_grid(), colors=[70, 70, 70], radii=0.002), static=True)
    rr.log("world/path", rr.LineStrips3D([path_points], colors=[255, 255, 255], radii=0.006), static=True)

    for row_offset, (_row_index, row) in enumerate(segment_frame.iterrows()):
        t = finite_or(time_s.iloc[row_offset], float(row_offset))
        set_rerun_time(rr, t, int(segment.start_row + row_offset * step))

        force = (
            finite_or(row.get("ctrl_force_cmd_n_0"), 0.0),
            finite_or(row.get("ctrl_force_cmd_n_1"), 0.0),
            finite_or(row.get("ctrl_force_cmd_n_2"), 0.0),
        )
        body_lines, body_colors, thrust_lines, thrust_colors = drone_geometry(
            finite_or(pos_x.iloc[row_offset], 0.0),
            finite_or(pos_y.iloc[row_offset], 0.0),
            finite_or(height.iloc[row_offset], 0.0),
            finite_or(row.get("roll_deg"), 0.0),
            finite_or(row.get("pitch_deg"), 0.0),
            finite_or(row.get("yaw_deg"), 0.0),
            force,
        )
        rr.log("scene/drone_body", rr.LineStrips3D(body_lines, colors=body_colors, radii=[0.016, 0.016, 0.012, 0.01, 0.006]))
        rr.log("scene/thrust_axis", rr.LineStrips3D(thrust_lines, colors=thrust_colors, radii=0.018))
        rr.log("scene/current_position", rr.Points3D([path_points[row_offset]], colors=[255, 255, 255], radii=0.035))

        for entity_path, column, scale in scalar_channels():
            if column in row.index:
                value = safe_float(row.get(column))
                if value is not None:
                    rr.log(entity_path, rerun_scalars(rr, value * scale))

    if hasattr(rr, "disconnect"):
        rr.disconnect()
    return rrd_path


def launch_rerun(rrd_path: Path) -> subprocess.Popen[Any]:
    commands = [
        [sys.executable, "-m", "rerun", str(rrd_path)],
        ["rerun", str(rrd_path)],
    ]
    last_error: Exception | None = None
    for command in commands:
        try:
            return subprocess.Popen(command, cwd=str(ROOT_DIR))
        except Exception as exc:  # pragma: no cover - depends on host viewer
            last_error = exc
    raise RuntimeError(f"无法启动 Rerun Viewer：{last_error}")


def load_csv(csv_path: Path) -> "pd.DataFrame":
    require_pandas()
    return pd.read_csv(csv_path, low_memory=False)


def format_optional(value: float | int | None, digits: int = 2) -> str:
    if value is None:
        return ""
    if isinstance(value, int):
        return str(value)
    return f"{value:.{digits}f}"


def print_segments(segments: list[ReplaySegment]) -> None:
    print("idx rows duration_s rate_hz seq_start seq_end missing reason_start reason_end thr_min thr_max max_roll max_pitch")
    for segment in segments:
        print(
            segment.index,
            segment.count,
            f"{segment.duration_s:.3f}",
            f"{segment.sample_rate_hz:.1f}",
            segment.start_sequence,
            segment.end_sequence,
            segment.missing_sequence_count,
            segment.reason_start,
            segment.reason_end,
            format_optional(segment.throttle_min_us, 0),
            format_optional(segment.throttle_max_us, 0),
            format_optional(segment.max_abs_roll_deg, 1),
            format_optional(segment.max_abs_pitch_deg, 1),
        )


class RerunReplayUI(tk.Tk):
    # 中文注释：只负责离线选择和启动 Rerun，不连接飞控、不发串口命令。
    def __init__(self, initial_path: Path | None = None) -> None:
        super().__init__()
        self.title("H743 Rerun 现场回放")
        self.geometry("1180x720")
        self.minsize(980, 560)

        self.csv_path: Path | None = None
        self.frame: pd.DataFrame | None = None if pd is not None else None
        self.segments: list[ReplaySegment] = []

        self.path_var = tk.StringVar(value="")
        self.status_var = tk.StringVar(value="选择日志后，可以播放某个分片")
        self.out_dir_var = tk.StringVar(value=str(DEFAULT_OUT_DIR))
        self.gap_ms_var = tk.StringVar(value=str(int(DEFAULT_GAP_MS)))
        self.max_rows_var = tk.StringVar(value=str(DEFAULT_MAX_ROWS_PER_SEGMENT))

        self._build_ui()
        try:
            self.load(resolve_csv_path(initial_path))
        except Exception as exc:
            self.status_var.set(str(exc))

    def _build_ui(self) -> None:
        root = ttk.Frame(self, padding=10)
        root.pack(fill=tk.BOTH, expand=True)

        top = ttk.Frame(root)
        top.pack(fill=tk.X)
        ttk.Button(top, text="打开最新", command=self.open_latest).pack(side=tk.LEFT)
        ttk.Button(top, text="选择CSV", command=self.choose_csv).pack(side=tk.LEFT, padx=(6, 0))
        ttk.Button(top, text="重新分片", command=self.refresh_segments).pack(side=tk.LEFT, padx=(6, 0))
        ttk.Entry(top, textvariable=self.path_var).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=8)

        options = ttk.Frame(root)
        options.pack(fill=tk.X, pady=(8, 6))
        ttk.Label(options, text="分片断点ms").pack(side=tk.LEFT)
        ttk.Entry(options, textvariable=self.gap_ms_var, width=8).pack(side=tk.LEFT, padx=(4, 12))
        ttk.Label(options, text="每片最多帧").pack(side=tk.LEFT)
        ttk.Entry(options, textvariable=self.max_rows_var, width=8).pack(side=tk.LEFT, padx=(4, 12))
        ttk.Label(options, text="输出目录").pack(side=tk.LEFT)
        ttk.Entry(options, textvariable=self.out_dir_var).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(4, 8))
        ttk.Button(options, text="选择输出", command=self.choose_out_dir).pack(side=tk.LEFT)

        table_frame = ttk.Frame(root)
        table_frame.pack(fill=tk.BOTH, expand=True, pady=(4, 8))
        self.segment_tree = ttk.Treeview(
            table_frame,
            columns=(
                "idx",
                "rows",
                "duration",
                "rate",
                "seq",
                "missing",
                "reason",
                "throttle",
                "angle",
            ),
            show="headings",
            selectmode="browse",
        )
        headings = {
            "idx": ("片段", 60, tk.E),
            "rows": ("记录数", 90, tk.E),
            "duration": ("时长 s", 80, tk.E),
            "rate": ("采样 Hz", 80, tk.E),
            "seq": ("Seq范围", 140, tk.W),
            "missing": ("丢记录", 80, tk.E),
            "reason": ("输出模式", 220, tk.W),
            "throttle": ("油门 us", 120, tk.W),
            "angle": ("最大角度", 140, tk.W),
        }
        for column, (text, width, anchor) in headings.items():
            self.segment_tree.heading(column, text=text)
            self.segment_tree.column(column, width=width, anchor=anchor)
        yscroll = ttk.Scrollbar(table_frame, orient=tk.VERTICAL, command=self.segment_tree.yview)
        self.segment_tree.configure(yscrollcommand=yscroll.set)
        self.segment_tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        yscroll.pack(side=tk.LEFT, fill=tk.Y)

        actions = ttk.Frame(root)
        actions.pack(fill=tk.X)
        ttk.Button(actions, text="播放选中片段", command=self.play_selected).pack(side=tk.LEFT)
        ttk.Button(actions, text="导出选中片段", command=self.export_selected).pack(side=tk.LEFT, padx=(8, 0))
        ttk.Button(actions, text="导出全部片段", command=self.export_all).pack(side=tk.LEFT, padx=(8, 0))
        ttk.Button(actions, text="打开输出文件夹", command=self.open_out_dir).pack(side=tk.LEFT, padx=(8, 0))
        ttk.Label(actions, textvariable=self.status_var).pack(side=tk.RIGHT)

    def _read_float(self, var: tk.StringVar, default: float) -> float:
        try:
            return float(var.get())
        except ValueError:
            return default

    def _read_int(self, var: tk.StringVar, default: int) -> int:
        try:
            return int(float(var.get()))
        except ValueError:
            return default

    def open_latest(self) -> None:
        try:
            self.load(resolve_csv_path(DEFAULT_LOG_DIR))
        except Exception as exc:
            messagebox.showerror("打开失败", str(exc))

    def choose_csv(self) -> None:
        path = filedialog.askopenfilename(
            title="选择 flightlog CSV",
            initialdir=str(DEFAULT_LOG_DIR if DEFAULT_LOG_DIR.exists() else ROOT_DIR),
            filetypes=(("CSV files", "*.csv"), ("All files", "*.*")),
        )
        if path:
            self.load(Path(path))

    def choose_out_dir(self) -> None:
        path = filedialog.askdirectory(
            title="选择 Rerun 输出目录",
            initialdir=str(Path(self.out_dir_var.get()).parent),
        )
        if path:
            self.out_dir_var.set(path)

    def load(self, csv_path: Path) -> None:
        try:
            frame = load_csv(csv_path)
        except Exception as exc:
            messagebox.showerror("读取失败", str(exc))
            return
        self.csv_path = csv_path
        self.frame = frame
        self.path_var.set(str(csv_path))
        self.refresh_segments()

    def refresh_segments(self) -> None:
        if self.frame is None:
            return
        gap_ms = self._read_float(self.gap_ms_var, DEFAULT_GAP_MS)
        self.segments = split_segments(self.frame, gap_ms=gap_ms)
        self.segment_tree.delete(*self.segment_tree.get_children())
        for segment in self.segments:
            self.segment_tree.insert(
                "",
                tk.END,
                iid=str(segment.index),
                values=(
                    segment.index,
                    segment.count,
                    format_optional(segment.duration_s),
                    format_optional(segment.sample_rate_hz, 1),
                    f"{segment.start_sequence}..{segment.end_sequence}",
                    segment.missing_sequence_count,
                    f"{segment.reason_start} -> {segment.reason_end}",
                    f"{format_optional(segment.throttle_min_us, 0)}..{format_optional(segment.throttle_max_us, 0)}",
                    f"R {format_optional(segment.max_abs_roll_deg, 1)} / P {format_optional(segment.max_abs_pitch_deg, 1)}",
                ),
            )
        if self.segments:
            self.segment_tree.selection_set(str(self.segments[-1].index))
            self.segment_tree.see(str(self.segments[-1].index))
        self.status_var.set(f"已加载 {len(self.frame)} 行，分成 {len(self.segments)} 片")

    def selected_segment(self) -> ReplaySegment | None:
        selection = self.segment_tree.selection()
        if not selection:
            return None
        index = int(selection[0])
        return self.segments[index] if 0 <= index < len(self.segments) else None

    def export_segments(self, segments: list[ReplaySegment]) -> list[Path]:
        if self.frame is None or self.csv_path is None:
            return []
        out_dir = Path(self.out_dir_var.get())
        max_rows = self._read_int(self.max_rows_var, DEFAULT_MAX_ROWS_PER_SEGMENT)
        exported = []
        for segment in segments:
            exported.append(export_segment_to_rrd(self.frame, self.csv_path, segment, out_dir, max_rows=max_rows))
        self.status_var.set(f"已导出 {len(exported)} 个 RRD 到 {out_dir}")
        return exported

    def export_selected(self) -> None:
        segment = self.selected_segment()
        if segment is None:
            return
        try:
            self.export_segments([segment])
        except Exception as exc:
            messagebox.showerror("导出失败", str(exc))

    def export_all(self) -> None:
        try:
            self.export_segments(self.segments)
        except Exception as exc:
            messagebox.showerror("导出失败", str(exc))

    def play_selected(self) -> None:
        segment = self.selected_segment()
        if segment is None:
            return
        try:
            paths = self.export_segments([segment])
            if paths:
                launch_rerun(paths[0])
        except Exception as exc:
            messagebox.showerror("播放失败", str(exc))

    def open_out_dir(self) -> None:
        path = Path(self.out_dir_var.get())
        path.mkdir(parents=True, exist_ok=True)
        os.startfile(path)  # type: ignore[attr-defined]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="H743 飞行日志 Rerun 回放导出器")
    parser.add_argument("path", nargs="?", type=Path, help="CSV 文件或日志文件夹；默认使用 ./log 最新 flightlog")
    parser.add_argument("--list", action="store_true", help="只列出分片，不导出")
    parser.add_argument("--export", action="store_true", help="导出 RRD")
    parser.add_argument("--play", action="store_true", help="导出后启动 Rerun Viewer")
    parser.add_argument("--segment", default="all", help="片段选择：all、3、1,4,7 或 2-5")
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT_DIR, help="RRD 输出目录")
    parser.add_argument("--gap-ms", type=float, default=DEFAULT_GAP_MS, help="超过该时间间隔则切分片段")
    parser.add_argument("--max-rows", type=int, default=DEFAULT_MAX_ROWS_PER_SEGMENT, help="每个片段最多导出帧数，自动降采样")
    parser.add_argument("--stride", type=int, default=0, help="固定导出步长；0 表示按 max-rows 自动")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not any((args.list, args.export, args.play)):
        app = RerunReplayUI(initial_path=args.path)
        app.mainloop()
        return 0

    csv_path = resolve_csv_path(args.path)
    frame = load_csv(csv_path)
    segments = split_segments(frame, gap_ms=args.gap_ms)
    selected = parse_segment_selection(args.segment, len(segments))
    if args.list:
        print(f"csv={csv_path}")
        print_segments(segments)
    if args.export or args.play:
        exported: list[Path] = []
        for index in selected:
            exported.append(
                export_segment_to_rrd(
                    frame,
                    csv_path,
                    segments[index],
                    args.out_dir,
                    max_rows=args.max_rows,
                    stride=args.stride,
                )
            )
        for path in exported:
            print(f"rrd={path}")
        if args.play and exported:
            launch_rerun(exported[0])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Reusable system-identification summaries for H743 flight-log CSV files."""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Iterable


DEFAULT_GAP_S = 0.100
DEFAULT_TILT_LIMIT_RAD = 0.314159
DEFAULT_MOTOR_MIN_US = 1100.0
DEFAULT_MOTOR_MAX_US = 1940.0
DEFAULT_SERVO_MIN_US = 1300.0
DEFAULT_SERVO_MAX_US = 1700.0

DEFAULT_GROUP_PARAM_FIELDS = (
    "roll_angle_kp",
    "roll_rate_kd",
    "pitch_angle_kp",
    "pitch_rate_kd",
    "pos_z_kp",
    "yaw_rate_kd",
)

DEFAULT_CHANNELS = (
    "roll_deg",
    "pitch_deg",
    "yaw_deg",
    "gyro_x_dps",
    "gyro_y_dps",
    "gyro_z_dps",
    "acc_nav_m_s2_0",
    "acc_nav_m_s2_1",
    "acc_nav_m_s2_2",
    "vel_est_m_s_0",
    "vel_est_m_s_1",
    "vel_est_m_s_2",
    "throttle_us",
    "servo_alpha_us",
    "servo_beta_us",
    "motor_upper_us",
    "motor_lower_us",
    "ctrl_total_force_n",
    "ctrl_force_cmd_n_0",
    "ctrl_force_cmd_n_1",
    "ctrl_force_cmd_n_2",
    "ctrl_tilt_ff_rad_0",
    "ctrl_tilt_ff_rad_1",
    "ctrl_tilt_angle_p_rad_0",
    "ctrl_tilt_angle_p_rad_1",
    "ctrl_tilt_rate_d_rad_0",
    "ctrl_tilt_rate_d_rad_1",
    "ctrl_tilt_out_rad_0",
    "ctrl_tilt_out_rad_1",
    "z_ref_m",
)


Row = dict[str, str]


@dataclass(frozen=True)
class ChannelStats:
    count: int
    minimum: float
    p05: float
    mean: float
    rms: float
    p95: float
    maximum: float


@dataclass(frozen=True)
class SegmentSummary:
    index: int
    start_row: int
    end_row: int
    count: int
    duration_s: float
    sample_rate_hz: float
    start_time_s: float
    end_time_s: float
    start_sequence: int | None
    end_sequence: int | None
    missing_sequence_count: int
    motor_reason_start: str
    motor_reason_end: str


@dataclass(frozen=True)
class GainGroupSummary:
    key: str
    params: dict[str, float | None]
    count: int
    duration_s: float
    roll_rms_deg: float | None
    pitch_rms_deg: float | None
    gyro_xy_rms_dps: float | None
    yaw_rate_rms_dps: float | None
    tilt_saturation_pct: float
    motor_high_saturation_pct: float
    motor_low_saturation_pct: float
    direct_throttle_pct: float
    throttle_mean_us: float | None
    total_force_mean_n: float | None


@dataclass(frozen=True)
class LinearFit:
    label: str
    input_channel: str
    output_channel: str
    count: int
    slope: float
    intercept: float
    r2: float
    correlation: float
    input_min: float
    input_max: float
    output_min: float
    output_max: float


@dataclass(frozen=True)
class FlightLogAnalysis:
    csv_path: str
    meta_path: str | None
    row_count: int
    column_count: int
    duration_s: float
    nominal_log_rate_hz: float | None
    observed_contiguous_rate_hz: float | None
    sequence_missing_count: int
    dropped_delta: int | None
    export_complete: bool | None
    export_errors: list[str]
    segments: list[SegmentSummary]
    channel_stats: dict[str, ChannelStats]
    gain_groups: list[GainGroupSummary]
    actuator_fits: list[LinearFit]
    flags: list[str]

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def infer_meta_path(csv_path: Path) -> Path | None:
    candidate = csv_path.with_name(f"{csv_path.stem}_meta.json")
    return candidate if candidate.exists() else None


def load_rows(csv_path: Path) -> tuple[list[Row], list[str]]:
    with csv_path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        rows = list(reader)
        return rows, list(reader.fieldnames or [])


def load_meta(meta_path: Path | None) -> dict[str, Any]:
    if meta_path is None:
        return {}
    return json.loads(meta_path.read_text(encoding="utf-8"))


def to_float(row: Row, name: str) -> float | None:
    value = row.get(name)
    if value is None or value == "":
        return None
    try:
        number = float(value)
    except ValueError:
        return None
    if not math.isfinite(number):
        return None
    return number


def to_int(row: Row, name: str) -> int | None:
    value = to_float(row, name)
    return int(value) if value is not None else None


def numeric_values(rows: Iterable[Row], channel: str) -> list[float]:
    values: list[float] = []
    for row in rows:
        value = to_float(row, channel)
        if value is not None:
            values.append(value)
    return values


def percentile(sorted_values: list[float], fraction: float) -> float:
    if not sorted_values:
        return math.nan
    index = min(len(sorted_values) - 1, max(0, int(round(fraction * (len(sorted_values) - 1)))))
    return sorted_values[index]


def channel_stats(values: list[float]) -> ChannelStats | None:
    if not values:
        return None
    ordered = sorted(values)
    mean = statistics.fmean(ordered)
    rms = math.sqrt(statistics.fmean([value * value for value in ordered]))
    return ChannelStats(
        count=len(ordered),
        minimum=ordered[0],
        p05=percentile(ordered, 0.05),
        mean=mean,
        rms=rms,
        p95=percentile(ordered, 0.95),
        maximum=ordered[-1],
    )


def rms_or_none(values: list[float]) -> float | None:
    if not values:
        return None
    return math.sqrt(statistics.fmean([value * value for value in values]))


def sequence_missing(rows: list[Row]) -> int:
    missing = 0
    previous: int | None = None
    for row in rows:
        current = to_int(row, "sequence")
        if current is None:
            continue
        if previous is not None and current > previous + 1:
            missing += current - previous - 1
        previous = current
    return missing


def contiguous_duration_s(rows: list[Row], gap_s: float) -> float:
    total = 0.0
    for before, after in zip(rows, rows[1:]):
        t0 = to_float(before, "timestamp_us")
        t1 = to_float(after, "timestamp_us")
        if t0 is None or t1 is None:
            continue
        dt_s = (t1 - t0) / 1_000_000.0
        if 0.0 <= dt_s <= gap_s:
            total += dt_s
    return total


def split_segments(rows: list[Row], gap_s: float) -> list[SegmentSummary]:
    if not rows:
        return []

    starts = [0]
    for index in range(1, len(rows)):
        before = to_float(rows[index - 1], "timestamp_us")
        current = to_float(rows[index], "timestamp_us")
        if before is None or current is None:
            continue
        dt_s = (current - before) / 1_000_000.0
        if dt_s < 0.0 or dt_s > gap_s:
            starts.append(index)

    starts.append(len(rows))
    summaries: list[SegmentSummary] = []
    for segment_index, start in enumerate(starts[:-1]):
        end_excl = starts[segment_index + 1]
        segment_rows = rows[start:end_excl]
        first = segment_rows[0]
        last = segment_rows[-1]
        t0 = to_float(first, "timestamp_us")
        t1 = to_float(last, "timestamp_us")
        duration = ((t1 - t0) / 1_000_000.0) if t0 is not None and t1 is not None else 0.0
        rate = ((len(segment_rows) - 1) / duration) if duration > 0.0 and len(segment_rows) > 1 else 0.0
        summaries.append(
            SegmentSummary(
                index=segment_index,
                start_row=start,
                end_row=end_excl - 1,
                count=len(segment_rows),
                duration_s=duration,
                sample_rate_hz=rate,
                start_time_s=(t0 or 0.0) / 1_000_000.0,
                end_time_s=(t1 or 0.0) / 1_000_000.0,
                start_sequence=to_int(first, "sequence"),
                end_sequence=to_int(last, "sequence"),
                missing_sequence_count=sequence_missing(segment_rows),
                motor_reason_start=first.get("motor_output_reason_name", ""),
                motor_reason_end=last.get("motor_output_reason_name", ""),
            )
        )
    return summaries


def sector_param_lookup(meta: dict[str, Any]) -> dict[tuple[str, str], dict[str, Any]]:
    lookup: dict[tuple[str, str], dict[str, Any]] = {}
    for sector in meta.get("sectors", []):
        params = sector.get("params")
        if not isinstance(params, dict):
            continue
        key = (str(sector.get("sector_seq")), str(sector.get("sector_index")))
        lookup[key] = params
    return lookup


def row_params(row: Row, lookup: dict[tuple[str, str], dict[str, Any]]) -> dict[str, Any]:
    key = (row.get("sector_seq", ""), row.get("sector_index", ""))
    return lookup.get(key, {})


def param_key(params: dict[str, Any], fields: tuple[str, ...]) -> tuple[float | None, ...]:
    values: list[float | None] = []
    for field in fields:
        value = params.get(field)
        if isinstance(value, (int, float)) and math.isfinite(float(value)):
            values.append(float(value))
        else:
            values.append(None)
    return tuple(values)


def key_label(fields: tuple[str, ...], key: tuple[float | None, ...]) -> str:
    parts = []
    for field, value in zip(fields, key):
        parts.append(f"{field}={value:.6g}" if value is not None else f"{field}=NA")
    return ", ".join(parts)


def pct(count: int, total: int) -> float:
    return 100.0 * count / total if total > 0 else 0.0


def near_abs_limit(value: float | None, limit: float, eps: float = 1.0e-5) -> bool:
    return value is not None and abs(abs(value) - limit) <= eps


def build_gain_groups(
    rows: list[Row],
    meta: dict[str, Any],
    gap_s: float,
    fields: tuple[str, ...] = DEFAULT_GROUP_PARAM_FIELDS,
) -> list[GainGroupSummary]:
    lookup = sector_param_lookup(meta)
    grouped: dict[tuple[float | None, ...], list[Row]] = {}
    params_by_key: dict[tuple[float | None, ...], dict[str, float | None]] = {}

    for row in rows:
        params = row_params(row, lookup)
        key = param_key(params, fields)
        grouped.setdefault(key, []).append(row)
        params_by_key.setdefault(key, dict(zip(fields, key)))

    summaries: list[GainGroupSummary] = []
    for key, group_rows in grouped.items():
        count = len(group_rows)
        roll_rms = rms_or_none(numeric_values(group_rows, "roll_deg"))
        pitch_rms = rms_or_none(numeric_values(group_rows, "pitch_deg"))
        gyro_x_rms = rms_or_none(numeric_values(group_rows, "gyro_x_dps"))
        gyro_y_rms = rms_or_none(numeric_values(group_rows, "gyro_y_dps"))
        gyro_xy = None
        if gyro_x_rms is not None and gyro_y_rms is not None:
            gyro_xy = math.sqrt((gyro_x_rms * gyro_x_rms + gyro_y_rms * gyro_y_rms) / 2.0)

        yaw_rate_rms = rms_or_none(numeric_values(group_rows, "gyro_z_dps"))
        tilt0_sat = sum(
            near_abs_limit(to_float(row, "ctrl_tilt_out_rad_0"), DEFAULT_TILT_LIMIT_RAD)
            for row in group_rows
        )
        tilt1_sat = sum(
            near_abs_limit(to_float(row, "ctrl_tilt_out_rad_1"), DEFAULT_TILT_LIMIT_RAD)
            for row in group_rows
        )
        motor_high = sum(
            ((to_float(row, "motor_upper_us") or 0.0) >= DEFAULT_MOTOR_MAX_US - 0.5)
            or ((to_float(row, "motor_lower_us") or 0.0) >= DEFAULT_MOTOR_MAX_US - 0.5)
            for row in group_rows
        )
        motor_low = sum(
            ((to_float(row, "motor_upper_us") or 9999.0) <= DEFAULT_MOTOR_MIN_US + 0.5)
            or ((to_float(row, "motor_lower_us") or 9999.0) <= DEFAULT_MOTOR_MIN_US + 0.5)
            for row in group_rows
        )
        direct = sum(row.get("motor_output_reason_name") == "direct_throttle" for row in group_rows)
        throttle_values = numeric_values(group_rows, "throttle_us")
        force_values = numeric_values(group_rows, "ctrl_total_force_n")
        summaries.append(
            GainGroupSummary(
                key=key_label(fields, key),
                params=params_by_key[key],
                count=count,
                duration_s=contiguous_duration_s(group_rows, gap_s),
                roll_rms_deg=roll_rms,
                pitch_rms_deg=pitch_rms,
                gyro_xy_rms_dps=gyro_xy,
                yaw_rate_rms_dps=yaw_rate_rms,
                tilt_saturation_pct=pct(tilt0_sat + tilt1_sat, 2 * count),
                motor_high_saturation_pct=pct(motor_high, count),
                motor_low_saturation_pct=pct(motor_low, count),
                direct_throttle_pct=pct(direct, count),
                throttle_mean_us=statistics.fmean(throttle_values) if throttle_values else None,
                total_force_mean_n=statistics.fmean(force_values) if force_values else None,
            )
        )

    return sorted(
        summaries,
        key=lambda item: (
            item.params.get("roll_angle_kp") is None,
            item.params.get("roll_angle_kp") or 0.0,
            item.params.get("roll_rate_kd") or 0.0,
        ),
    )


def fit_line(label: str, rows: list[Row], input_channel: str, output_channel: str) -> LinearFit | None:
    pairs: list[tuple[float, float]] = []
    for row in rows:
        x = to_float(row, input_channel)
        y = to_float(row, output_channel)
        if x is not None and y is not None:
            pairs.append((x, y))
    if len(pairs) < 2:
        return None

    xs = [item[0] for item in pairs]
    ys = [item[1] for item in pairs]
    mean_x = statistics.fmean(xs)
    mean_y = statistics.fmean(ys)
    ss_x = sum((value - mean_x) ** 2 for value in xs)
    ss_y = sum((value - mean_y) ** 2 for value in ys)
    if ss_x <= 0.0 or ss_y <= 0.0:
        return None

    cov = sum((x - mean_x) * (y - mean_y) for x, y in pairs)
    slope = cov / ss_x
    intercept = mean_y - slope * mean_x
    residual = sum((y - (slope * x + intercept)) ** 2 for x, y in pairs)
    r2 = 1.0 - residual / ss_y if ss_y > 0.0 else math.nan
    corr = cov / math.sqrt(ss_x * ss_y)
    return LinearFit(
        label=label,
        input_channel=input_channel,
        output_channel=output_channel,
        count=len(pairs),
        slope=slope,
        intercept=intercept,
        r2=r2,
        correlation=corr,
        input_min=min(xs),
        input_max=max(xs),
        output_min=min(ys),
        output_max=max(ys),
    )


def build_actuator_fits(rows: list[Row]) -> list[LinearFit]:
    candidates = (
        ("body_x_tilt_to_servo_beta", "ctrl_tilt_out_rad_0", "servo_beta_us"),
        ("body_y_tilt_to_servo_alpha", "ctrl_tilt_out_rad_1", "servo_alpha_us"),
        ("total_force_to_motor_upper", "ctrl_total_force_n", "motor_upper_us"),
        ("total_force_to_motor_lower", "ctrl_total_force_n", "motor_lower_us"),
    )
    fits = [fit_line(label, rows, x_channel, y_channel) for label, x_channel, y_channel in candidates]
    return [fit for fit in fits if fit is not None]


def nominal_log_rate(meta: dict[str, Any]) -> float | None:
    begin = meta.get("begin")
    if isinstance(begin, dict):
        value = begin.get("log_rate")
        if value is not None:
            try:
                return float(value)
            except ValueError:
                pass
    sectors = meta.get("sectors", [])
    if sectors:
        value = sectors[0].get("log_rate_hz")
        if isinstance(value, (int, float)):
            return float(value)
    return None


def build_flags(
    rows: list[Row],
    meta: dict[str, Any],
    segments: list[SegmentSummary],
    gain_groups: list[GainGroupSummary],
    channel_names: list[str],
) -> list[str]:
    flags: list[str] = []
    if len(segments) > 1:
        flags.append(f"log is discontinuous: {len(segments)} time segments")
    missing = sequence_missing(rows)
    if missing > 0:
        flags.append(f"flight recorder dropped {missing} records before export")
    if meta.get("missing_ranges"):
        flags.append("export has missing byte ranges")
    if any(group.tilt_saturation_pct > 5.0 for group in gain_groups):
        flags.append("tilt saturation is significant; avoid linear PID fits in saturated samples")
    if any(group.motor_high_saturation_pct > 10.0 for group in gain_groups):
        flags.append("motor high saturation is significant; thrust authority is a limiting factor")
    if "vel_loop_active" in channel_names:
        active_values = numeric_values(rows, "vel_loop_active")
        if active_values and max(active_values) == 0.0:
            flags.append("velocity loop is disabled in this log")
    if "ctrl_tilt_angle_p_rad_0" in channel_names and "ctrl_tilt_angle_p_rad_1" in channel_names:
        p0 = numeric_values(rows, "ctrl_tilt_angle_p_rad_0")
        p1 = numeric_values(rows, "ctrl_tilt_angle_p_rad_1")
        if p0 and p1 and max(abs(value) for value in p0 + p1) == 0.0:
            flags.append("angle-P tilt contribution is zero; angle KP is acting through force terms or disabled")
    if not any(name in channel_names for name in ("range_m", "height_m", "z_est_m", "rangefinder_m")):
        flags.append("no direct range/height channel found; Z position-loop identification is limited")
    return flags


def analyze_flight_log(
    csv_path: str | Path,
    meta_path: str | Path | None = None,
    gap_s: float = DEFAULT_GAP_S,
    channels: tuple[str, ...] = DEFAULT_CHANNELS,
) -> FlightLogAnalysis:
    csv_file = Path(csv_path)
    meta_file = Path(meta_path) if meta_path is not None else infer_meta_path(csv_file)
    rows, column_names = load_rows(csv_file)
    meta = load_meta(meta_file)

    timestamps = numeric_values(rows, "timestamp_us")
    duration = ((timestamps[-1] - timestamps[0]) / 1_000_000.0) if len(timestamps) > 1 else 0.0
    contiguous_duration = contiguous_duration_s(rows, gap_s)
    observed_rate = ((len(rows) - 1) / contiguous_duration) if contiguous_duration > 0.0 and len(rows) > 1 else None
    dropped_values = numeric_values(rows, "dropped_records")
    dropped_delta = int(dropped_values[-1] - dropped_values[0]) if len(dropped_values) > 1 else None

    stats: dict[str, ChannelStats] = {}
    for channel in channels:
        if channel in column_names:
            result = channel_stats(numeric_values(rows, channel))
            if result is not None:
                stats[channel] = result

    segments = split_segments(rows, gap_s)
    gain_groups = build_gain_groups(rows, meta, gap_s)
    actuator_fits = build_actuator_fits(rows)
    flags = build_flags(rows, meta, segments, gain_groups, column_names)
    return FlightLogAnalysis(
        csv_path=str(csv_file),
        meta_path=str(meta_file) if meta_file is not None else None,
        row_count=len(rows),
        column_count=len(column_names),
        duration_s=duration,
        nominal_log_rate_hz=nominal_log_rate(meta),
        observed_contiguous_rate_hz=observed_rate,
        sequence_missing_count=sequence_missing(rows),
        dropped_delta=dropped_delta,
        export_complete=meta.get("complete") if "complete" in meta else None,
        export_errors=[str(error) for error in meta.get("errors", [])],
        segments=segments,
        channel_stats=stats,
        gain_groups=gain_groups,
        actuator_fits=actuator_fits,
        flags=flags,
    )


def write_json_report(analysis: FlightLogAnalysis, path: Path) -> None:
    path.write_text(json.dumps(analysis.to_dict(), indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def write_gain_csv(analysis: FlightLogAnalysis, path: Path) -> None:
    fieldnames = [
        "key",
        "count",
        "duration_s",
        "roll_rms_deg",
        "pitch_rms_deg",
        "gyro_xy_rms_dps",
        "yaw_rate_rms_dps",
        "tilt_saturation_pct",
        "motor_high_saturation_pct",
        "motor_low_saturation_pct",
        "direct_throttle_pct",
        "throttle_mean_us",
        "total_force_mean_n",
    ]
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for group in analysis.gain_groups:
            row = asdict(group)
            row.pop("params", None)
            writer.writerow({name: row.get(name) for name in fieldnames})


def write_markdown_report(analysis: FlightLogAnalysis, path: Path) -> None:
    lines = [
        "# Flight Log System Identification Summary",
        "",
        f"- CSV: `{analysis.csv_path}`",
        f"- rows: {analysis.row_count}",
        f"- columns: {analysis.column_count}",
        f"- wall-clock span: {analysis.duration_s:.3f} s",
    ]
    if analysis.nominal_log_rate_hz is not None:
        lines.append(f"- nominal log rate: {analysis.nominal_log_rate_hz:.1f} Hz")
    if analysis.observed_contiguous_rate_hz is not None:
        lines.append(f"- observed contiguous rate: {analysis.observed_contiguous_rate_hz:.1f} Hz")
    lines.append(f"- sequence missing: {analysis.sequence_missing_count}")
    if analysis.dropped_delta is not None:
        lines.append(f"- dropped_records delta: {analysis.dropped_delta}")
    if analysis.export_complete is not None:
        lines.append(f"- export complete: {analysis.export_complete}")
    lines.append("")

    if analysis.flags:
        lines.append("## Flags")
        lines.extend(f"- {flag}" for flag in analysis.flags)
        lines.append("")

    lines.append("## Segments")
    lines.append("|idx|rows|duration_s|rate_hz|seq_start|seq_end|missing|reason_start|reason_end|")
    lines.append("|---:|---:|---:|---:|---:|---:|---:|---|---|")
    for segment in analysis.segments:
        lines.append(
            f"|{segment.index}|{segment.count}|{segment.duration_s:.3f}|"
            f"{segment.sample_rate_hz:.1f}|{segment.start_sequence}|{segment.end_sequence}|"
            f"{segment.missing_sequence_count}|{segment.motor_reason_start}|{segment.motor_reason_end}|"
        )
    lines.append("")

    lines.append("## Gain Groups")
    lines.append(
        "|params|rows|duration_s|roll_rms|pitch_rms|gyro_xy_rms|yaw_rate_rms|tilt_sat_pct|motor_hi_sat_pct|direct_pct|"
    )
    lines.append("|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|")
    for group in analysis.gain_groups:
        lines.append(
            f"|{group.key}|{group.count}|{group.duration_s:.3f}|"
            f"{format_optional(group.roll_rms_deg)}|{format_optional(group.pitch_rms_deg)}|"
            f"{format_optional(group.gyro_xy_rms_dps)}|{format_optional(group.yaw_rate_rms_dps)}|"
            f"{group.tilt_saturation_pct:.1f}|{group.motor_high_saturation_pct:.1f}|"
            f"{group.direct_throttle_pct:.1f}|"
        )
    lines.append("")

    lines.append("## Actuator Fits")
    lines.append("|label|input|output|slope|intercept|r2|corr|count|")
    lines.append("|---|---|---|---:|---:|---:|---:|---:|")
    for fit in analysis.actuator_fits:
        lines.append(
            f"|{fit.label}|{fit.input_channel}|{fit.output_channel}|"
            f"{fit.slope:.6g}|{fit.intercept:.6g}|{fit.r2:.4f}|{fit.correlation:.4f}|{fit.count}|"
        )
    lines.append("")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def format_optional(value: float | None) -> str:
    return "" if value is None else f"{value:.3f}"


def write_reports(analysis: FlightLogAnalysis, out_dir: Path, stem: str) -> dict[str, Path]:
    out_dir.mkdir(parents=True, exist_ok=True)
    paths = {
        "json": out_dir / f"{stem}_sysid_summary.json",
        "gain_csv": out_dir / f"{stem}_sysid_gain_groups.csv",
        "markdown": out_dir / f"{stem}_sysid_report.md",
    }
    write_json_report(analysis, paths["json"])
    write_gain_csv(analysis, paths["gain_csv"])
    write_markdown_report(analysis, paths["markdown"])
    return paths


def print_console_summary(analysis: FlightLogAnalysis) -> None:
    print(f"rows={analysis.row_count} columns={analysis.column_count}")
    print(
        f"duration_s={analysis.duration_s:.3f} segments={len(analysis.segments)} "
        f"missing_seq={analysis.sequence_missing_count}"
    )
    if analysis.nominal_log_rate_hz is not None or analysis.observed_contiguous_rate_hz is not None:
        print(
            f"log_rate_nominal={format_optional(analysis.nominal_log_rate_hz)} "
            f"observed_contiguous={format_optional(analysis.observed_contiguous_rate_hz)}"
        )
    for flag in analysis.flags:
        print(f"FLAG: {flag}")
    print("gain groups:")
    for group in analysis.gain_groups:
        print(
            f"  {group.key}: rows={group.count} roll_rms={format_optional(group.roll_rms_deg)} "
            f"pitch_rms={format_optional(group.pitch_rms_deg)} tilt_sat={group.tilt_saturation_pct:.1f}% "
            f"motor_hi_sat={group.motor_high_saturation_pct:.1f}%"
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Analyze H743 FLOG CSV files for system identification.")
    parser.add_argument("csv", type=Path, help="flightlog_*.csv path")
    parser.add_argument("--meta", type=Path, default=None, help="flightlog_*_meta.json path")
    parser.add_argument("--out-dir", type=Path, default=None, help="directory for JSON/CSV/Markdown reports")
    parser.add_argument("--gap-ms", type=float, default=DEFAULT_GAP_S * 1000.0, help="segment break gap in ms")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    analysis = analyze_flight_log(args.csv, args.meta, gap_s=args.gap_ms / 1000.0)
    print_console_summary(analysis)
    if args.out_dir is not None:
        paths = write_reports(analysis, args.out_dir, args.csv.stem)
        for label, path in paths.items():
            print(f"{label}: {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

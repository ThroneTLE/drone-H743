#!/usr/bin/env python3
"""Fit a motor Hammerstein model from thrust-identification CSV data.

The model is:

    throttle/PWM -> static pressure/thrust lookup -> first-order lag -> measured output

By default the output is converted from the pressure module raw register:

    output_g = raw * 0.1

That matches modules where raw 100 means 10 g. Use --source grams only if the
CSV grams column has already been calibrated to the desired physical units.
"""

from __future__ import annotations

import argparse
import csv
import math
import statistics
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt


@dataclass(frozen=True)
class Sample:
    host_time: float
    seq: int
    motor: int
    pct: float
    pulse_us: int
    fc_ms: int
    dwell_ms: int
    sample_index: int
    raw: int
    grams: float


@dataclass(frozen=True)
class StepFit:
    seq: int
    pct: float
    pulse_us: int
    count: int
    full_mean_g: float
    full_std_g: float
    steady_mean_g: float
    steady_std_g: float
    steady_min_g: float
    steady_max_g: float
    first_mean_g: float
    drift_g: float


@dataclass(frozen=True)
class TauFit:
    from_pct: float
    to_pct: float
    from_pulse_us: int
    to_pulse_us: int
    tau_s: float
    delay_s: float
    r2: float
    used_points: int


@dataclass(frozen=True)
class PredictionMetrics:
    full_rmse_g: float
    full_mae_g: float
    full_max_abs_g: float
    tail_rmse_g: float
    tail_mae_g: float
    tail_max_abs_g: float
    sample_count: int
    tail_sample_count: int


def load_samples(path: Path, source: str, raw_scale: float) -> dict[int, list[Sample]]:
    blocks: dict[int, list[Sample]] = {}
    with path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            if row.get("kind") != "sample":
                continue
            if row.get("error"):
                continue
            if not row.get("grams"):
                continue
            raw_value = int(row["raw"])
            if source == "raw":
                output_g = raw_value * raw_scale
            elif source == "grams":
                output_g = float(row["grams"])
            else:
                raise ValueError(f"unsupported source {source}")

            sample = Sample(
                host_time=float(row["host_time"]),
                seq=int(row["seq"]),
                motor=int(row["motor"]),
                pct=float(row["pct"]),
                pulse_us=int(row["pulse_us"]),
                fc_ms=int(row["fc_ms"]),
                dwell_ms=int(row["dwell_ms"]),
                sample_index=int(row["sample_index"]),
                raw=raw_value,
                grams=output_g,
            )
            blocks.setdefault(sample.seq, []).append(sample)

    for seq, samples in blocks.items():
        samples.sort(key=lambda item: item.sample_index)
        if not samples:
            raise ValueError(f"seq {seq} has no valid samples")
    return dict(sorted(blocks.items()))


def trimmed_mean(values: list[float], trim_fraction: float) -> float:
    if not values:
        raise ValueError("empty values")
    if trim_fraction <= 0.0:
        return statistics.fmean(values)
    if trim_fraction >= 0.5:
        raise ValueError("trim_fraction must be < 0.5")

    sorted_values = sorted(values)
    trim_count = int(len(sorted_values) * trim_fraction)
    if trim_count == 0:
        return statistics.fmean(sorted_values)
    trimmed = sorted_values[trim_count:-trim_count]
    return statistics.fmean(trimmed)


def mean_std(values: list[float]) -> tuple[float, float]:
    mean = statistics.fmean(values)
    std = statistics.pstdev(values) if len(values) > 1 else 0.0
    return mean, std


def fit_steps(
    blocks: dict[int, list[Sample]],
    tail_samples: int,
    trim_fraction: float,
) -> list[StepFit]:
    fits: list[StepFit] = []
    for seq, samples in blocks.items():
        if len(samples) < tail_samples:
            raise ValueError(f"seq {seq} only has {len(samples)} samples, need {tail_samples}")

        values = [sample.grams for sample in samples]
        first_values = values[:tail_samples]
        steady_values = values[-tail_samples:]
        full_mean, full_std = mean_std(values)
        steady_mean = trimmed_mean(steady_values, trim_fraction)
        steady_std = statistics.pstdev(steady_values) if len(steady_values) > 1 else 0.0
        first_mean = trimmed_mean(first_values, trim_fraction)

        first = samples[0]
        fits.append(
            StepFit(
                seq=seq,
                pct=first.pct,
                pulse_us=first.pulse_us,
                count=len(samples),
                full_mean_g=full_mean,
                full_std_g=full_std,
                steady_mean_g=steady_mean,
                steady_std_g=steady_std,
                steady_min_g=min(steady_values),
                steady_max_g=max(steady_values),
                first_mean_g=first_mean,
                drift_g=steady_mean - first_mean,
            )
        )
    return fits


def linear_regression(xs: list[float], ys: list[float]) -> tuple[float, float, float]:
    if len(xs) != len(ys) or len(xs) < 2:
        raise ValueError("linear regression needs at least two paired values")
    x_mean = statistics.fmean(xs)
    y_mean = statistics.fmean(ys)
    sxx = sum((x - x_mean) ** 2 for x in xs)
    if sxx == 0.0:
        raise ValueError("all x values are identical")
    sxy = sum((x - x_mean) * (y - y_mean) for x, y in zip(xs, ys))
    slope = sxy / sxx
    intercept = y_mean - slope * x_mean

    predictions = [intercept + slope * x for x in xs]
    ss_res = sum((y - pred) ** 2 for y, pred in zip(ys, predictions))
    ss_tot = sum((y - y_mean) ** 2 for y in ys)
    r2 = 1.0 if ss_tot == 0.0 else 1.0 - ss_res / ss_tot
    return intercept, slope, r2


def fit_tau_for_transition(
    prev_fit: StepFit,
    curr_fit: StepFit,
    curr_samples: list[Sample],
    z_min: float,
    z_max: float,
) -> TauFit | None:
    start_g = prev_fit.steady_mean_g
    final_g = curr_fit.steady_mean_g
    delta_g = final_g - start_g
    if abs(delta_g) < 1e-6:
        return None

    t0 = curr_samples[0].host_time
    times: list[float] = []
    logs: list[float] = []
    delay_s: float | None = None

    for sample in curr_samples:
        z = (sample.grams - start_g) / delta_g
        if delay_s is None and z >= z_min:
            delay_s = sample.host_time - t0
        if z <= z_min or z >= z_max:
            continue
        remain = 1.0 - z
        if remain <= 0.0:
            continue
        times.append(sample.host_time - t0)
        logs.append(math.log(remain))

    if len(times) < 4:
        return None

    _intercept, slope, r2 = linear_regression(times, logs)
    if slope >= 0.0:
        return None
    tau_s = -1.0 / slope
    if not math.isfinite(tau_s) or tau_s <= 0.0:
        return None

    return TauFit(
        from_pct=prev_fit.pct,
        to_pct=curr_fit.pct,
        from_pulse_us=prev_fit.pulse_us,
        to_pulse_us=curr_fit.pulse_us,
        tau_s=tau_s,
        delay_s=0.0 if delay_s is None else delay_s,
        r2=r2,
        used_points=len(times),
    )


def fit_tau_table(
    blocks: dict[int, list[Sample]],
    step_fits: list[StepFit],
    z_min: float,
    z_max: float,
) -> list[TauFit]:
    by_seq = {fit.seq: fit for fit in step_fits}
    tau_fits: list[TauFit] = []
    seqs = sorted(blocks)
    for prev_seq, curr_seq in zip(seqs, seqs[1:]):
        fit = fit_tau_for_transition(
            by_seq[prev_seq],
            by_seq[curr_seq],
            blocks[curr_seq],
            z_min,
            z_max,
        )
        if fit is not None:
            tau_fits.append(fit)
    return tau_fits


def format_float_array(values: list[float], per_line: int = 6) -> str:
    lines: list[str] = []
    for offset in range(0, len(values), per_line):
        chunk = values[offset : offset + per_line]
        lines.append("    " + ", ".join(f"{value:.3f}f" for value in chunk) + ",")
    return "\n".join(lines)


def format_uint16_array(values: list[int], per_line: int = 8) -> str:
    lines: list[str] = []
    for offset in range(0, len(values), per_line):
        chunk = values[offset : offset + per_line]
        lines.append("    " + ", ".join(f"{value}U" for value in chunk) + ",")
    return "\n".join(lines)


def interp_table(xs: list[float], ys: list[float], x: float) -> float:
    if not xs:
        raise ValueError("empty interpolation table")
    if x <= xs[0]:
        return ys[0]
    for index in range(1, len(xs)):
        if x <= xs[index]:
            dx = xs[index] - xs[index - 1]
            ratio = 0.0 if dx == 0.0 else (x - xs[index - 1]) / dx
            return ys[index - 1] + ratio * (ys[index] - ys[index - 1])
    return ys[-1]


def predict_blocks(
    blocks: dict[int, list[Sample]],
    step_fits: list[StepFit],
    tau_s: float | None,
) -> dict[int, list[tuple[float, float, float, float]]]:
    pct_table = [fit.pct for fit in step_fits]
    thrust_table = [fit.steady_mean_g for fit in step_fits]
    predictions: dict[int, list[tuple[float, float, float, float]]] = {}
    thrust_est: float | None = None
    prev_time: float | None = None

    for seq in sorted(blocks):
        seq_predictions: list[tuple[float, float, float, float]] = []
        for sample in blocks[seq]:
            thrust_ss = interp_table(pct_table, thrust_table, sample.pct)
            if thrust_est is None or tau_s is None or tau_s <= 0.0 or prev_time is None:
                thrust_est = thrust_ss
            else:
                dt_s = max(0.0, sample.host_time - prev_time)
                beta = 1.0 - math.exp(-dt_s / tau_s)
                thrust_est = thrust_est + beta * (thrust_ss - thrust_est)
            prev_time = sample.host_time
            error_g = sample.grams - thrust_est
            seq_predictions.append((sample.host_time, sample.grams, thrust_est, error_g))
        predictions[seq] = seq_predictions
    return predictions


def compute_prediction_metrics(
    predictions: dict[int, list[tuple[float, float, float, float]]],
    tail_samples: int,
) -> PredictionMetrics:
    errors: list[float] = []
    tail_errors: list[float] = []
    for seq_predictions in predictions.values():
        seq_errors = [item[3] for item in seq_predictions]
        errors.extend(seq_errors)
        tail_errors.extend(seq_errors[-tail_samples:])

    def rmse(values: list[float]) -> float:
        return math.sqrt(statistics.fmean(value * value for value in values)) if values else 0.0

    def mae(values: list[float]) -> float:
        return statistics.fmean(abs(value) for value in values) if values else 0.0

    def max_abs(values: list[float]) -> float:
        return max((abs(value) for value in values), default=0.0)

    return PredictionMetrics(
        full_rmse_g=rmse(errors),
        full_mae_g=mae(errors),
        full_max_abs_g=max_abs(errors),
        tail_rmse_g=rmse(tail_errors),
        tail_mae_g=mae(tail_errors),
        tail_max_abs_g=max_abs(tail_errors),
        sample_count=len(errors),
        tail_sample_count=len(tail_errors),
    )


def write_prediction_csv(
    path: Path,
    blocks: dict[int, list[Sample]],
    predictions: dict[int, list[tuple[float, float, float, float]]],
) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=[
                "seq",
                "sample_index",
                "pct",
                "pulse_us",
                "time_s",
                "measured_g",
                "predicted_g",
                "error_g",
            ],
        )
        writer.writeheader()
        for seq in sorted(blocks):
            samples = blocks[seq]
            seq_predictions = predictions[seq]
            t0 = samples[0].host_time
            for sample, prediction in zip(samples, seq_predictions):
                _host_time, measured_g, predicted_g, error_g = prediction
                writer.writerow(
                    {
                        "seq": seq,
                        "sample_index": sample.sample_index,
                        "pct": f"{sample.pct:.3f}",
                        "pulse_us": sample.pulse_us,
                        "time_s": f"{sample.host_time - t0:.6f}",
                        "measured_g": f"{measured_g:.3f}",
                        "predicted_g": f"{predicted_g:.3f}",
                        "error_g": f"{error_g:.3f}",
                    }
                )


def plot_static_curve(path: Path, step_fits: list[StepFit]) -> None:
    pct = [fit.pct for fit in step_fits]
    thrust = [fit.steady_mean_g for fit in step_fits]
    std = [fit.steady_std_g for fit in step_fits]

    fig, ax = plt.subplots(figsize=(9.0, 5.2))
    ax.errorbar(pct, thrust, yerr=std, marker="o", capsize=3, linewidth=1.8)
    ax.set_title("Static Motor Pressure Curve")
    ax.set_xlabel("Throttle (%)")
    ax.set_ylabel("Pressure / thrust equivalent (g, raw * 0.1)")
    ax.grid(True, alpha=0.35)
    ax.set_xlim(min(pct), max(pct))
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def plot_dynamic_prediction(
    path: Path,
    blocks: dict[int, list[Sample]],
    predictions: dict[int, list[tuple[float, float, float, float]]],
) -> None:
    times: list[float] = []
    measured: list[float] = []
    predicted: list[float] = []
    errors: list[float] = []
    pcts: list[float] = []
    t0 = min(samples[0].host_time for samples in blocks.values() if samples)

    for seq in sorted(blocks):
        for sample, prediction in zip(blocks[seq], predictions[seq]):
            times.append(sample.host_time - t0)
            measured.append(prediction[1])
            predicted.append(prediction[2])
            errors.append(prediction[3])
            pcts.append(sample.pct)

    fig, (ax_top, ax_bottom) = plt.subplots(
        2,
        1,
        figsize=(10.5, 7.0),
        sharex=True,
        gridspec_kw={"height_ratios": [3, 1]},
    )
    ax_top.plot(times, measured, label="measured", linewidth=1.2)
    ax_top.plot(times, predicted, label="Hammerstein prediction", linewidth=1.6)
    ax_top.set_title("Hammerstein Dynamic Prediction")
    ax_top.set_ylabel("Pressure / thrust equivalent (g, raw * 0.1)")
    ax_top.grid(True, alpha=0.35)
    ax_top.legend(loc="upper left")

    ax_pct = ax_top.twinx()
    ax_pct.step(times, pcts, where="post", color="0.45", alpha=0.35, label="throttle")
    ax_pct.set_ylabel("Throttle (%)")
    ax_pct.set_ylim(-3, 103)

    ax_bottom.plot(times, errors, color="tab:red", linewidth=1.0)
    ax_bottom.axhline(0.0, color="0.2", linewidth=0.8)
    ax_bottom.set_xlabel("Time (s)")
    ax_bottom.set_ylabel("Error (g)")
    ax_bottom.grid(True, alpha=0.35)
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def plot_tau_curve(path: Path, tau_fits: list[TauFit]) -> None:
    if not tau_fits:
        return
    to_pct = [fit.to_pct for fit in tau_fits]
    tau_ms = [fit.tau_s * 1000.0 for fit in tau_fits]
    r2 = [fit.r2 for fit in tau_fits]

    fig, ax = plt.subplots(figsize=(9.0, 5.0))
    scatter = ax.scatter(to_pct, tau_ms, c=r2, cmap="viridis", s=52)
    ax.plot(to_pct, tau_ms, color="0.45", alpha=0.45)
    ax.set_title("First-order Time Constant by Step")
    ax.set_xlabel("Target throttle (%)")
    ax.set_ylabel("Tau (ms)")
    ax.grid(True, alpha=0.35)
    colorbar = fig.colorbar(scatter, ax=ax)
    colorbar.set_label("Fit R^2")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def write_static_csv(path: Path, step_fits: list[StepFit]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=[
                "seq",
                "pct",
                "pulse_us",
                "count",
                "steady_mean_g",
                "steady_std_g",
                "steady_min_g",
                "steady_max_g",
                "full_mean_g",
                "full_std_g",
                "first_mean_g",
                "drift_g",
            ],
        )
        writer.writeheader()
        for fit in step_fits:
            writer.writerow(
                {
                    "seq": fit.seq,
                    "pct": f"{fit.pct:.3f}",
                    "pulse_us": fit.pulse_us,
                    "count": fit.count,
                    "steady_mean_g": f"{fit.steady_mean_g:.3f}",
                    "steady_std_g": f"{fit.steady_std_g:.3f}",
                    "steady_min_g": f"{fit.steady_min_g:.3f}",
                    "steady_max_g": f"{fit.steady_max_g:.3f}",
                    "full_mean_g": f"{fit.full_mean_g:.3f}",
                    "full_std_g": f"{fit.full_std_g:.3f}",
                    "first_mean_g": f"{fit.first_mean_g:.3f}",
                    "drift_g": f"{fit.drift_g:.3f}",
                }
            )


def write_tau_csv(path: Path, tau_fits: list[TauFit]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=[
                "from_pct",
                "to_pct",
                "from_pulse_us",
                "to_pulse_us",
                "tau_s",
                "delay_s",
                "r2",
                "used_points",
            ],
        )
        writer.writeheader()
        for fit in tau_fits:
            writer.writerow(
                {
                    "from_pct": f"{fit.from_pct:.3f}",
                    "to_pct": f"{fit.to_pct:.3f}",
                    "from_pulse_us": fit.from_pulse_us,
                    "to_pulse_us": fit.to_pulse_us,
                    "tau_s": f"{fit.tau_s:.6f}",
                    "delay_s": f"{fit.delay_s:.6f}",
                    "r2": f"{fit.r2:.6f}",
                    "used_points": fit.used_points,
                }
            )


def summarize_tau(tau_fits: list[TauFit], min_pct: float, max_pct: float) -> tuple[list[TauFit], float | None]:
    selected = [
        fit
        for fit in tau_fits
        if fit.to_pct >= min_pct and fit.to_pct <= max_pct and fit.r2 >= 0.90
    ]
    if not selected:
        return selected, None
    return selected, statistics.median(fit.tau_s for fit in selected)


def write_report(
    path: Path,
    source: Path,
    step_fits: list[StepFit],
    tau_fits: list[TauFit],
    selected_tau: list[TauFit],
    tau_median: float | None,
    prediction_metrics: PredictionMetrics | None,
    tail_samples: int,
    trim_fraction: float,
    z_min: float,
    z_max: float,
) -> None:
    pct_values = [fit.pct for fit in step_fits]
    thrust_values = [fit.steady_mean_g for fit in step_fits]
    pulse_values = [fit.pulse_us for fit in step_fits]
    max_noise = max(step_fits, key=lambda fit: fit.steady_std_g)
    max_drift = max(step_fits, key=lambda fit: abs(fit.drift_g))

    lines = [
        "# Motor Hammerstein Fit Report",
        "",
        f"Source: `{source}`",
        f"Tail samples for steady state: {tail_samples}",
        f"Tail trim fraction: {trim_fraction:.3f}",
        f"Tau fit z window: {z_min:.2f}..{z_max:.2f}",
        "",
        "## Static Lookup",
        "",
        f"Points: {len(step_fits)}",
        f"Throttle range: {pct_values[0]:.1f}%..{pct_values[-1]:.1f}%",
        f"Pulse range: {pulse_values[0]}us..{pulse_values[-1]}us",
        f"Thrust range: {thrust_values[0]:.1f}g..{thrust_values[-1]:.1f}g",
        f"Max tail std: {max_noise.steady_std_g:.1f}g at {max_noise.pct:.1f}%",
        f"Max first-to-tail drift: {max_drift.drift_g:.1f}g at {max_drift.pct:.1f}%",
        "",
        "Use the `steady_mean_g` column as the static nonlinearity `f(u)`.",
        "",
        "## Dynamic Lag",
        "",
        f"Fitted transitions: {len(tau_fits)}",
        f"Selected transitions for global tau: {len(selected_tau)}",
    ]
    if tau_median is not None:
        lines.append(f"Recommended global tau: {tau_median:.4f}s")
    else:
        lines.append("Recommended global tau: unavailable")

    if prediction_metrics is not None:
        lines.extend(
            [
                "",
                "## Prediction Error",
                "",
                f"Full-response samples: {prediction_metrics.sample_count}",
                f"Full-response RMSE: {prediction_metrics.full_rmse_g:.3f}g",
                f"Full-response MAE: {prediction_metrics.full_mae_g:.3f}g",
                f"Full-response max abs error: {prediction_metrics.full_max_abs_g:.3f}g",
                f"Tail samples: {prediction_metrics.tail_sample_count}",
                f"Tail RMSE: {prediction_metrics.tail_rmse_g:.3f}g",
                f"Tail MAE: {prediction_metrics.tail_mae_g:.3f}g",
                f"Tail max abs error: {prediction_metrics.tail_max_abs_g:.3f}g",
            ]
        )

    lines.extend(
        [
            "",
            "## C Lookup Snippet",
            "",
            "```c",
            f"#define MOTOR_HAMMERSTEIN_POINTS {len(step_fits)}U",
            "",
            "static const uint16_t motor_pwm_us_table[MOTOR_HAMMERSTEIN_POINTS] = {",
            format_uint16_array(pulse_values),
            "};",
            "",
            "static const float motor_pct_table[MOTOR_HAMMERSTEIN_POINTS] = {",
            format_float_array(pct_values),
            "};",
            "",
            "static const float motor_thrust_g_table[MOTOR_HAMMERSTEIN_POINTS] = {",
            format_float_array(thrust_values),
            "};",
        ]
    )
    if tau_median is not None:
        lines.append(f"\nstatic const float motor_tau_s = {tau_median:.6f}f;")
    lines.extend(["```", ""])

    path.write_text("\n".join(lines), encoding="utf-8")


def write_c_header(
    path: Path,
    step_fits: list[StepFit],
    tau_median: float | None,
) -> None:
    pct_values = [fit.pct for fit in step_fits]
    pulse_values = [fit.pulse_us for fit in step_fits]
    thrust_values = [fit.steady_mean_g for fit in step_fits]
    tau_value = 0.0 if tau_median is None else tau_median

    lines = [
        "#ifndef MOTOR_HAMMERSTEIN_MODEL_H",
        "#define MOTOR_HAMMERSTEIN_MODEL_H",
        "",
        "#include <math.h>",
        "#include <stdint.h>",
        "",
        f"#define MOTOR_HAMMERSTEIN_POINTS {len(step_fits)}U",
        "",
        "static const uint16_t motor_hammerstein_pwm_us[MOTOR_HAMMERSTEIN_POINTS] = {",
        format_uint16_array(pulse_values),
        "};",
        "",
        "static const float motor_hammerstein_pct[MOTOR_HAMMERSTEIN_POINTS] = {",
        format_float_array(pct_values),
        "};",
        "",
        "static const float motor_hammerstein_thrust_g[MOTOR_HAMMERSTEIN_POINTS] = {",
        format_float_array(thrust_values),
        "};",
        "",
        f"static const float motor_hammerstein_tau_s = {tau_value:.6f}f;",
        "",
        "static inline float motor_hammerstein_interp_f32(",
        "    const float *xs, const float *ys, uint32_t count, float x)",
        "{",
        "    if (count == 0U) {",
        "        return 0.0f;",
        "    }",
        "    if (x <= xs[0]) {",
        "        return ys[0];",
        "    }",
        "    for (uint32_t i = 1U; i < count; ++i) {",
        "        if (x <= xs[i]) {",
        "            const float dx = xs[i] - xs[i - 1U];",
        "            const float ratio = (dx > 0.0f) ? ((x - xs[i - 1U]) / dx) : 0.0f;",
        "            return ys[i - 1U] + ratio * (ys[i] - ys[i - 1U]);",
        "        }",
        "    }",
        "    return ys[count - 1U];",
        "}",
        "",
        "static inline float motor_hammerstein_interp_pwm_to_thrust(uint16_t pwm_us)",
        "{",
        "    if (pwm_us <= motor_hammerstein_pwm_us[0]) {",
        "        return motor_hammerstein_thrust_g[0];",
        "    }",
        "    for (uint32_t i = 1U; i < MOTOR_HAMMERSTEIN_POINTS; ++i) {",
        "        if (pwm_us <= motor_hammerstein_pwm_us[i]) {",
        "            const float left = (float)motor_hammerstein_pwm_us[i - 1U];",
        "            const float right = (float)motor_hammerstein_pwm_us[i];",
        "            const float ratio = (((float)pwm_us) - left) / (right - left);",
        "            return motor_hammerstein_thrust_g[i - 1U] +",
        "                   ratio * (motor_hammerstein_thrust_g[i] - motor_hammerstein_thrust_g[i - 1U]);",
        "        }",
        "    }",
        "    return motor_hammerstein_thrust_g[MOTOR_HAMMERSTEIN_POINTS - 1U];",
        "}",
        "",
        "static inline float motor_hammerstein_interp_pct_to_thrust(float pct)",
        "{",
        "    return motor_hammerstein_interp_f32(",
        "        motor_hammerstein_pct, motor_hammerstein_thrust_g, MOTOR_HAMMERSTEIN_POINTS, pct);",
        "}",
        "",
        "static inline float motor_hammerstein_interp_thrust_to_pct(float thrust_g)",
        "{",
        "    return motor_hammerstein_interp_f32(",
        "        motor_hammerstein_thrust_g, motor_hammerstein_pct, MOTOR_HAMMERSTEIN_POINTS, thrust_g);",
        "}",
        "",
        "static inline uint16_t motor_hammerstein_interp_thrust_to_pwm(float thrust_g)",
        "{",
        "    if (thrust_g <= motor_hammerstein_thrust_g[0]) {",
        "        return motor_hammerstein_pwm_us[0];",
        "    }",
        "    for (uint32_t i = 1U; i < MOTOR_HAMMERSTEIN_POINTS; ++i) {",
        "        if (thrust_g <= motor_hammerstein_thrust_g[i]) {",
        "            const float left = motor_hammerstein_thrust_g[i - 1U];",
        "            const float right = motor_hammerstein_thrust_g[i];",
        "            const float ratio = (right > left) ? ((thrust_g - left) / (right - left)) : 0.0f;",
        "            const float pwm = (float)motor_hammerstein_pwm_us[i - 1U] +",
        "                              ratio * ((float)motor_hammerstein_pwm_us[i] -",
        "                                       (float)motor_hammerstein_pwm_us[i - 1U]);",
        "            return (uint16_t)(pwm + 0.5f);",
        "        }",
        "    }",
        "    return motor_hammerstein_pwm_us[MOTOR_HAMMERSTEIN_POINTS - 1U];",
        "}",
        "",
        "static inline float motor_hammerstein_update_lag(",
        "    float thrust_est_g, float thrust_ss_g, float dt_s)",
        "{",
        "    if ((dt_s <= 0.0f) || (motor_hammerstein_tau_s <= 0.0f)) {",
        "        return thrust_ss_g;",
        "    }",
        "    const float beta = 1.0f - expf(-dt_s / motor_hammerstein_tau_s);",
        "    return thrust_est_g + beta * (thrust_ss_g - thrust_est_g);",
        "}",
        "",
        "#endif",
        "",
    ]
    path.write_text("\n".join(lines), encoding="utf-8")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path, help="thrust identification CSV")
    parser.add_argument(
        "--source",
        choices=("raw", "grams"),
        default="raw",
        help="output source: raw uses raw*raw-scale, grams uses the CSV grams column",
    )
    parser.add_argument(
        "--raw-scale",
        type=float,
        default=0.1,
        help="scale applied when --source raw; default raw 100 -> 10 g",
    )
    parser.add_argument(
        "--tail-samples",
        type=int,
        default=25,
        help="number of samples at each step tail used for steady-state thrust",
    )
    parser.add_argument(
        "--trim",
        type=float,
        default=0.10,
        help="trim fraction for tail steady-state mean, e.g. 0.10 drops 10%% low/high",
    )
    parser.add_argument("--z-min", type=float, default=0.10, help="minimum normalized transition value")
    parser.add_argument("--z-max", type=float, default=0.90, help="maximum normalized transition value")
    parser.add_argument(
        "--tau-min-pct",
        type=float,
        default=20.0,
        help="minimum target throttle for global tau median selection",
    )
    parser.add_argument(
        "--tau-max-pct",
        type=float,
        default=90.0,
        help="maximum target throttle for global tau median selection",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=None,
        help="output directory, default is the CSV directory",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    csv_path = args.csv.resolve()
    out_dir = args.out_dir.resolve() if args.out_dir is not None else csv_path.parent
    out_dir.mkdir(parents=True, exist_ok=True)

    if args.tail_samples <= 0:
        raise ValueError("--tail-samples must be > 0")
    if args.z_min <= 0.0 or args.z_max >= 1.0 or args.z_min >= args.z_max:
        raise ValueError("--z-min/--z-max must satisfy 0 < z_min < z_max < 1")

    blocks = load_samples(csv_path, args.source, args.raw_scale)
    step_fits = fit_steps(blocks, args.tail_samples, args.trim)
    tau_fits = fit_tau_table(blocks, step_fits, args.z_min, args.z_max)
    selected_tau, tau_median = summarize_tau(tau_fits, args.tau_min_pct, args.tau_max_pct)
    predictions = predict_blocks(blocks, step_fits, tau_median)
    prediction_metrics = compute_prediction_metrics(predictions, args.tail_samples)

    stem = csv_path.stem
    static_csv = out_dir / f"{stem}_hammerstein_static.csv"
    tau_csv = out_dir / f"{stem}_hammerstein_tau.csv"
    prediction_csv = out_dir / f"{stem}_hammerstein_prediction.csv"
    report_md = out_dir / f"{stem}_hammerstein_report.md"
    header_h = out_dir / f"{stem}_hammerstein_model.h"
    static_png = out_dir / f"{stem}_hammerstein_static.png"
    prediction_png = out_dir / f"{stem}_hammerstein_prediction.png"
    tau_png = out_dir / f"{stem}_hammerstein_tau.png"

    write_static_csv(static_csv, step_fits)
    write_tau_csv(tau_csv, tau_fits)
    write_prediction_csv(prediction_csv, blocks, predictions)
    plot_static_curve(static_png, step_fits)
    plot_dynamic_prediction(prediction_png, blocks, predictions)
    plot_tau_curve(tau_png, tau_fits)
    write_report(
        report_md,
        csv_path,
        step_fits,
        tau_fits,
        selected_tau,
        tau_median,
        prediction_metrics,
        args.tail_samples,
        args.trim,
        args.z_min,
        args.z_max,
    )
    write_c_header(header_h, step_fits, tau_median)

    print(f"static: {static_csv}")
    print(f"tau:    {tau_csv}")
    print(f"pred:   {prediction_csv}")
    print(f"report: {report_md}")
    print(f"header: {header_h}")
    print(f"plot:   {static_png}")
    print(f"plot:   {prediction_png}")
    print(f"plot:   {tau_png}")
    if tau_median is not None:
        print(f"recommended_tau_s: {tau_median:.6f}")
    print(f"prediction_full_rmse_g: {prediction_metrics.full_rmse_g:.3f}")
    print(f"prediction_tail_rmse_g: {prediction_metrics.tail_rmse_g:.3f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

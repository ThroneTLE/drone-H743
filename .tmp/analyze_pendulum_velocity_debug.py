import csv
import math
import statistics
from pathlib import Path


def corr(a_values, b_values):
    if len(a_values) < 4:
        return float("nan")
    mean_a = statistics.fmean(a_values)
    mean_b = statistics.fmean(b_values)
    var_a = sum((value - mean_a) ** 2 for value in a_values)
    var_b = sum((value - mean_b) ** 2 for value in b_values)
    if var_a <= 1.0e-12 or var_b <= 1.0e-12:
        return float("nan")
    cov = sum(
        (a_value - mean_a) * (b_value - mean_b)
        for a_value, b_value in zip(a_values, b_values)
    )
    return cov / math.sqrt(var_a * var_b)


def fit_scale(x_values, y_values):
    denom = sum(value * value for value in x_values)
    if denom <= 1.0e-12:
        return float("nan")
    return sum(x_value * y_value for x_value, y_value in zip(x_values, y_values)) / denom


def rms(values):
    return math.sqrt(statistics.fmean([value * value for value in values])) if values else 0.0


def main():
    path = Path(".tmp/pendulum_openocd_velocity_debug.csv")
    rows = []
    with path.open(encoding="utf-8") as csv_file:
        for raw in csv.DictReader(csv_file):
            row = {}
            for key, value in raw.items():
                if value == "":
                    continue
                row[key] = float(value)
            rows.append(row)

    rows.sort(key=lambda row: row["t_s"])
    rows = [row for row in rows if 0.25 <= row["height_m"] <= 0.8]

    window = 7
    for key in ["roll_est_deg", "pitch_est_deg"]:
        values = [row[key] for row in rows]
        smoothed = []
        for index in range(len(values)):
            start = max(0, index - window // 2)
            end = min(len(values), index + window // 2 + 1)
            smoothed.append(statistics.fmean(values[start:end]))
        for row, value in zip(rows, smoothed):
            row[key + "_sm"] = value

    for index, row in enumerate(rows):
        if index == 0 or index == len(rows) - 1:
            row["roll_dot_rad_s"] = 0.0
            row["pitch_dot_rad_s"] = 0.0
            continue
        before = rows[index - 1]
        after = rows[index + 1]
        dt_s = after["t_s"] - before["t_s"]
        if dt_s <= 0.0:
            row["roll_dot_rad_s"] = 0.0
            row["pitch_dot_rad_s"] = 0.0
            continue
        row["roll_dot_rad_s"] = math.radians(
            after["roll_est_deg_sm"] - before["roll_est_deg_sm"]
        ) / dt_s
        row["pitch_dot_rad_s"] = math.radians(
            after["pitch_est_deg_sm"] - before["pitch_est_deg_sm"]
        ) / dt_s

    selected = [
        row
        for row in rows[3:-3]
        if abs(row["raw_flow_x"]) + abs(row["raw_flow_y"]) >= 3
    ]
    print(
        "filtered_rows",
        len(rows),
        "motion",
        len(selected),
        "height",
        min(row["height_m"] for row in rows),
        max(row["height_m"] for row in rows),
    )

    for flow_key in [
        "flow_x_no_height_m_s",
        "flow_x_with_height_m_s",
        "flow_y_no_height_m_s",
        "flow_y_with_height_m_s",
    ]:
        for dot_key in ["pitch_dot_rad_s", "roll_dot_rad_s"]:
            velocity = [row[flow_key] for row in selected]
            omega = [row[dot_key] for row in selected]
            print(
                f"{flow_key} vs {dot_key}: "
                f"R={fit_scale(omega, velocity):.3f}m "
                f"corr={corr(velocity, omega):.3f} "
                f"corr_inv={corr(velocity, [-value for value in omega]):.3f} "
                f"rms_v={rms(velocity):.3f} rms_omega={rms(omega):.3f}"
            )

    for radius_m in [0.15, 0.25, 0.35, 0.50, 0.70]:
        pitch_velocity = [
            radius_m
            * row["pitch_dot_rad_s"]
            * math.cos(math.radians(row["pitch_est_deg_sm"]))
            for row in selected
        ]
        roll_velocity = [
            radius_m
            * row["roll_dot_rad_s"]
            * math.cos(math.radians(row["roll_est_deg_sm"]))
            for row in selected
        ]
        print(
            f"R={radius_m:.2f} expected "
            f"pitch_rms={rms(pitch_velocity):.3f} "
            f"roll_rms={rms(roll_velocity):.3f}"
        )

    for key in [
        "flow_x_no_height_m_s",
        "flow_x_with_height_m_s",
        "flow_y_no_height_m_s",
        "flow_y_with_height_m_s",
        "pitch_dot_rad_s",
        "roll_dot_rad_s",
    ]:
        values = sorted(abs(row[key]) for row in selected)
        def quantile(probability):
            return values[min(len(values) - 1, int(probability * (len(values) - 1)))]
        print(
            key,
            "abs q50/q90/q99",
            quantile(0.5),
            quantile(0.9),
            quantile(0.99),
        )


if __name__ == "__main__":
    main()

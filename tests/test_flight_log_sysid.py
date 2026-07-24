from __future__ import annotations

import csv
import json
from pathlib import Path

from tools import flight_log_sysid as sysid


HEADER = [
    "timestamp_us",
    "sequence",
    "dropped_records",
    "sector_seq",
    "sector_index",
    "roll_deg",
    "pitch_deg",
    "gyro_x_dps",
    "gyro_y_dps",
    "gyro_z_dps",
    "vel_loop_active",
    "throttle_us",
    "servo_alpha_us",
    "servo_beta_us",
    "motor_upper_us",
    "motor_lower_us",
    "motor_output_reason_name",
    "ctrl_total_force_n",
    "ctrl_tilt_angle_p_rad_0",
    "ctrl_tilt_angle_p_rad_1",
    "ctrl_tilt_out_rad_0",
    "ctrl_tilt_out_rad_1",
]


def write_sample_log(tmp_path: Path) -> Path:
    csv_path = tmp_path / "flightlog_sample.csv"
    rows = [
        [0, 10, 0, 1, 2, 1.0, 2.0, 3.0, 4.0, 5.0, 0, 1500, 1520, 1480, 1600, 1605, "stabilized_mix", 10.0, 0, 0, 0.10, -0.10],
        [4000, 11, 0, 1, 2, 2.0, 4.0, 6.0, 8.0, 10.0, 0, 1510, 1540, 1460, 1700, 1705, "stabilized_mix", 11.0, 0, 0, 0.20, -0.20],
        [8000, 13, 1, 1, 2, 3.0, 6.0, 9.0, 12.0, 15.0, 0, 1520, 1560, 1440, 1940, 1705, "stabilized_mix", 12.0, 0, 0, 0.314159, -0.30],
        [250000, 14, 1, 3, 4, 1.0, 1.0, 2.0, 2.0, 2.0, 0, 1400, 1510, 1490, 1500, 1500, "direct_throttle", 8.0, 0, 0, 0.05, -0.05],
        [254000, 15, 1, 3, 4, 1.5, 1.2, 2.5, 2.2, 2.1, 0, 1410, 1520, 1480, 1500, 1500, "direct_throttle", 8.2, 0, 0, 0.10, -0.10],
    ]
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(HEADER)
        writer.writerows(rows)

    meta = {
        "complete": True,
        "errors": [],
        "begin": {"log_rate": "250"},
        "sectors": [
            {
                "sector_seq": 1,
                "sector_index": 2,
                "params": {
                    "roll_angle_kp": -8.0,
                    "roll_rate_kd": -1.0,
                    "pitch_angle_kp": -8.0,
                    "pitch_rate_kd": -1.0,
                    "pos_z_kp": 0.0,
                    "yaw_rate_kd": 0.15,
                },
            },
            {
                "sector_seq": 3,
                "sector_index": 4,
                "params": {
                    "roll_angle_kp": -1.0,
                    "roll_rate_kd": -1.5,
                    "pitch_angle_kp": -1.0,
                    "pitch_rate_kd": -1.5,
                    "pos_z_kp": 0.0,
                    "yaw_rate_kd": 0.5,
                },
            },
        ],
    }
    csv_path.with_name("flightlog_sample_meta.json").write_text(
        json.dumps(meta),
        encoding="utf-8",
    )
    return csv_path


def test_analyze_flight_log_segments_groups_and_flags(tmp_path: Path) -> None:
    csv_path = write_sample_log(tmp_path)

    analysis = sysid.analyze_flight_log(csv_path)

    assert analysis.row_count == 5
    assert analysis.nominal_log_rate_hz == 250.0
    assert len(analysis.segments) == 2
    assert analysis.sequence_missing_count == 1
    assert analysis.dropped_delta == 1
    assert len(analysis.gain_groups) == 2
    assert analysis.gain_groups[0].params["roll_angle_kp"] == -8.0
    assert analysis.gain_groups[0].tilt_saturation_pct > 0.0
    assert any("velocity loop is disabled" in flag for flag in analysis.flags)
    assert any("no direct range/height channel" in flag for flag in analysis.flags)


def test_actuator_fit_and_report_outputs(tmp_path: Path) -> None:
    csv_path = write_sample_log(tmp_path)
    analysis = sysid.analyze_flight_log(csv_path)

    beta_fit = next(fit for fit in analysis.actuator_fits if fit.label == "body_x_tilt_to_servo_beta")
    assert beta_fit.slope < 0.0
    assert beta_fit.r2 > 0.9

    outputs = sysid.write_reports(analysis, tmp_path / "report", csv_path.stem)
    assert outputs["json"].exists()
    assert outputs["gain_csv"].exists()
    assert outputs["markdown"].read_text(encoding="utf-8").startswith(
        "# Flight Log System Identification Summary"
    )

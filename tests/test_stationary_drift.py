"""Stationary drift self-check: a still aircraft reads 0 rad/s and 1 g, so error is direct.

静止漂移自检。

背景：六面标定解的是"摆在六个姿态下读数对不对"，但用户真正在意的是"放着不动会
不会自己飘"。这两件事不等价。飞机不动时真实角速度就是 0、真实比力就是 1 g，所以
这个检查不需要转台 —— 读数偏多少就是误差多少，也因此天然适合做 APPLY 前后的 A/B。

这组测试锁住四件事：
  1. 静止时该报的四个量：角度漂移、合加速度、陀螺零偏、温度变化；
  2. 录制太短/掉帧/快照重复必须说出来，不能给一个看起来正常的结论；
  3. 偏航和横滚俯仰的门限不同 —— 后者有重力修正，本就不该漂；
  4. A/B 对比要说清哪项改善、哪项变差，并在两次代次相同时提醒这不是前后对比。
"""

from __future__ import annotations

import math
from datetime import datetime, timezone
from pathlib import Path

import pytest

from tools import stationary_drift as drift


def make_samples(*, seconds: float = 60.0, hz: float = 10.0,
                 yaw_rate_deg_per_min: float = 0.0,
                 gyro_bias_dps: tuple[float, float, float] = (0.0, 0.0, 0.0),
                 accel_z_g: float = 1.0,
                 temperature_drift_c: float = 0.0,
                 gap_at: int | None = None,
                 repeat_sequence: bool = False):
    rows = []
    count = int(seconds * hz)
    for index in range(count):
        timestamp = index / hz
        if gap_at is not None and index >= gap_at:
            timestamp += 5.0        # 中途卡了 5 秒
        yaw = (yaw_rate_deg_per_min / 60.0) * timestamp
        rows.append(drift.DriftSample(
            timestamp_s=timestamp,
            gyro_dps=gyro_bias_dps,
            accel_g=(0.0, 0.0, accel_z_g),
            attitude_deg=(0.0, 0.0, ((yaw + 180.0) % 360.0) - 180.0),
            temperature_c=25.0 + temperature_drift_c * timestamp / max(seconds, 1e-9),
            sequence=1 if repeat_sequence else (index + 1) * 100,
        ))
    return rows


def test_a_still_aircraft_reports_no_drift_and_one_g() -> None:
    report = drift.analyze_drift(make_samples())

    assert report.status == "PASS"
    assert report.accel_norm_g == pytest.approx(1.0, abs=1e-9)
    assert max(abs(value) for value in report.attitude_drift_deg_per_min) < 0.01
    assert "正常" in report.findings[0]


def test_the_window_must_be_long_enough_to_show_a_trend() -> None:
    report = drift.analyze_drift(make_samples(seconds=10.0))

    assert report.status == "INCOMPLETE"
    assert any("至少要 30 秒" in item for item in report.findings)


def test_yaw_is_allowed_to_drift_more_than_roll_and_pitch() -> None:
    """偏航只能靠陀螺积分，漂一点正常；横滚俯仰有重力持续修正，不该漂。"""
    yaw_only = drift.analyze_drift(make_samples(yaw_rate_deg_per_min=0.8))
    assert yaw_only.status == "PASS"

    rows = make_samples()
    tilted = [
        drift.DriftSample(
            row.timestamp_s, row.gyro_dps, row.accel_g,
            ((0.8 / 60.0) * row.timestamp_s, 0.0, 0.0),   # 横滚 0.8 °/min
            row.temperature_c, row.sequence)
        for row in rows
    ]
    report = drift.analyze_drift(tilted)

    assert report.status == "WARN"
    assert any("横滚/俯仰在漂" in item for item in report.findings)
    assert any("重力持续修正" in item for item in report.findings)


def test_a_big_yaw_drift_points_at_leftover_gyro_bias() -> None:
    report = drift.analyze_drift(make_samples(yaw_rate_deg_per_min=8.0))

    assert report.status == "FAIL"
    assert any("偏航漂移" in item and "零偏残留" in item for item in report.findings)


def test_the_gravity_magnitude_is_the_accelerometer_scorecard() -> None:
    """不动时 |a| 必须是 1 g，任何偏差都是加速度计标定的残差。"""
    report = drift.analyze_drift(make_samples(accel_z_g=1.040))

    assert report.accel_norm_g == pytest.approx(1.040)
    assert report.status == "FAIL"
    assert any("偏离 1 g 达 40.0 mg" in item for item in report.findings)


def test_leftover_gyro_bias_is_reported_even_without_attitude_drift() -> None:
    report = drift.analyze_drift(make_samples(gyro_bias_dps=(0.30, 0.0, 0.0)))

    assert any("陀螺读数不是 0" in item for item in report.findings)
    # 主机侧积分是交叉验证：0.30 dps × 60 s = 18°
    assert report.gyro_integral_deg[0] == pytest.approx(0.30 * report.duration_s, rel=1e-6)


def test_a_telemetry_gap_is_called_out_because_it_skews_the_integral() -> None:
    report = drift.analyze_drift(make_samples(gap_at=300))

    # 60 秒里断一次只占 0.2%，按比例根本看不出来 —— 单个大洞必须另有一道门。
    assert report.gap_count == 1
    assert report.gap_fraction < drift.GAP_FRACTION_WARN
    assert report.max_gap_s > 4.0
    assert report.status == "WARN"
    assert any("中间断了" in item and "遥测掉帧" in item for item in report.findings)


def test_repeated_snapshots_are_flagged_but_normal_sequence_jumps_are_not() -> None:
    """10 Hz 轮询 1 kHz 的流，seq 本来就跳一百多号；只有重复/倒退才是问题。"""
    healthy = drift.analyze_drift(make_samples())
    assert healthy.stale_snapshots == 0
    assert not any("快照序号" in item for item in healthy.findings)

    stale = drift.analyze_drift(make_samples(repeat_sequence=True))
    assert stale.stale_snapshots > 0
    assert stale.status == "WARN"
    assert any("重复或倒退" in item for item in stale.findings)


def test_self_heating_is_surfaced_because_it_invalidates_an_ab_comparison() -> None:
    report = drift.analyze_drift(make_samples(temperature_drift_c=4.0))

    assert report.temperature_span_c == pytest.approx(4.0, abs=0.02)
    assert any("自热" in item and "A/B" in item for item in report.findings)


def test_the_ab_comparison_says_which_way_each_number_moved() -> None:
    before = drift.analyze_drift(
        make_samples(yaw_rate_deg_per_min=6.0, accel_z_g=1.035,
                     gyro_bias_dps=(0.11, 0.0, 0.0)),
        context={"cal_generation": 1})
    after = drift.analyze_drift(
        make_samples(yaw_rate_deg_per_min=0.4, accel_z_g=1.001,
                     gyro_bias_dps=(0.008, 0.0, 0.0)),
        context={"cal_generation": 2})

    lines = drift.compare(before, after)
    text = "\n".join(lines)

    assert "偏航漂移" in text and "改善" in text
    assert "合加速度偏离 1g" in text
    assert "陀螺零偏最大轴" in text
    assert "代次相同" not in text


def test_comparing_two_runs_of_the_same_generation_warns_it_is_not_an_ab() -> None:
    """同一代次的两次录制只能说明重复性，不能当成标定收益。"""
    same = drift.analyze_drift(make_samples(), context={"cal_generation": 2})

    assert any("代次相同" in item for item in drift.compare(same, same))


def test_a_report_survives_a_round_trip_through_json(tmp_path: Path) -> None:
    report = drift.analyze_drift(
        make_samples(yaw_rate_deg_per_min=1.5), context={"cal_generation": 2})

    path = drift.write_report(report, now=datetime(2026, 8, 29, tzinfo=timezone.utc),
                              root=tmp_path)
    restored = drift.load_report(path)

    assert restored == report
    assert path.parent.name == drift.DRIFT_DIRNAME
    assert drift.recent_reports(root=tmp_path)[0][1] == report


def test_a_foreign_json_payload_is_refused(tmp_path: Path) -> None:
    path = tmp_path / "bad.json"
    path.write_text('{"format": "something-else", "schema": 1}', encoding="utf-8")

    with pytest.raises(ValueError, match="unsupported"):
        drift.load_report(path)


def test_yaw_wraparound_does_not_invent_a_360_degree_drift() -> None:
    """偏航从 +179° 走到 -179° 是走了 2°，不是 -358°。"""
    rows = make_samples(seconds=60.0)
    walked = []
    for index, row in enumerate(rows):
        yaw = 179.0 + index * (4.0 / len(rows))
        walked.append(drift.DriftSample(
            row.timestamp_s, row.gyro_dps, row.accel_g,
            (0.0, 0.0, ((yaw + 180.0) % 360.0) - 180.0),
            row.temperature_c, row.sequence))

    report = drift.analyze_drift(walked)

    assert report.attitude_drift_deg[2] == pytest.approx(4.0, abs=0.05)
    assert not math.isclose(abs(report.attitude_drift_deg[2]), 356.0, abs_tol=5.0)

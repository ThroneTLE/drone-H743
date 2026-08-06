from __future__ import annotations

from pathlib import Path
import os

import pandas as pd

from tools import flight_log_rerun_replay as replay


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def test_latest_csv_and_segment_helpers(tmp_path: Path) -> None:
    older = tmp_path / "flightlog_older.csv"
    newer = tmp_path / "flightlog_newer.csv"
    older.write_text("timestamp_us,sequence\n0,1\n", encoding="utf-8")
    newer.write_text("timestamp_us,sequence\n0,1\n", encoding="utf-8")
    os.utime(older, (1000, 1000))
    os.utime(newer, (2000, 2000))

    assert replay.latest_csv_in(tmp_path) == newer
    assert replay.parse_segment_selection("all", 4) == [0, 1, 2, 3]
    assert replay.parse_segment_selection("0,2-3", 5) == [0, 2, 3]
    assert replay.export_stride(1000, 250, 0) == 4
    assert replay.export_stride(1000, 250, 7) == 7


def test_split_segments_and_summary_fields() -> None:
    frame = pd.DataFrame(
        {
            "timestamp_us": [0, 4000, 8000, 500000, 504000, 508000],
            "sequence": [10, 11, 12, 1, 2, 5],
            "motor_output_reason_name": [
                "attitude_debug",
                "attitude_debug",
                "attitude_debug",
                "direct_throttle",
                "direct_throttle",
                "direct_throttle",
            ],
            "throttle_us": [1500, 1600, 1700, 1100, 1100, 1200],
            "roll_deg": [0.0, -2.0, 3.0, 1.0, 2.0, 4.0],
            "pitch_deg": [0.0, 1.0, -2.0, 2.0, 3.0, 5.0],
        }
    )

    segments = replay.split_segments(frame, gap_ms=200.0)

    assert len(segments) == 2
    assert segments[0].count == 3
    assert segments[0].reason_start == "attitude_debug"
    assert segments[0].throttle_max_us == 1700
    assert segments[1].start_sequence == 1
    assert segments[1].missing_sequence_count == 2
    assert segments[1].max_abs_pitch_deg == 5.0


def test_coordinate_mapping_and_rpy_identity() -> None:
    assert replay.local_down_to_rerun([1.0, 2.0, -3.0]) == [2.0, 1.0, 3.0]

    rotation = replay.rpy_body_to_local_down(0.0, 0.0, 0.0)

    assert rotation.tolist() == [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [-0.0, 0.0, 1.0]]


def test_rerun_replay_tool_exposes_ui_and_export_path() -> None:
    source = read("tools/flight_log_rerun_replay.py")

    assert "class RerunReplayUI(tk.Tk)" in source
    assert '"播放选中片段"' in source
    assert '"导出全部片段"' in source
    assert "import rerun as rr" in source
    assert "rr.save" in source
    assert "rr.LineStrips3D" in source
    assert "rr.Points3D" in source
    assert "rerun_scalars" in source
    assert "set_rerun_time" in source
    assert "RERUN_INSTALL_HINT" in source
    assert "中文注释" in source


def test_rerun_replay_wrapper_keeps_sdk_in_isolated_venv() -> None:
    source = read("tools/run_flight_log_rerun_replay.ps1")
    readme = read("tools/README.md")

    assert ".tmp\\rerun_env" in source
    assert "rerun-sdk" in source
    assert "flight_log_rerun_replay.py" in source
    assert "run_flight_log_rerun_replay.ps1" in readme
    assert ".tmp/rerun_env" in readme

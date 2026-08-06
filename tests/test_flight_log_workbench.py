from __future__ import annotations

from pathlib import Path

from tools import flight_log_workbench as workbench


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def test_workbench_helpers_build_segment_labels_and_presets() -> None:
    segment = workbench.replay.ReplaySegment(
        index=3,
        start_row=10,
        end_row=20,
        count=11,
        duration_s=1.25,
        sample_rate_hz=100.0,
        start_sequence=42,
        end_sequence=52,
        missing_sequence_count=0,
        reason_start="attitude_debug",
        reason_end="direct_throttle",
        throttle_min_us=1100.0,
        throttle_max_us=1800.0,
        max_abs_roll_deg=4.0,
        max_abs_pitch_deg=5.0,
    )

    assert workbench.segment_label(segment) == (
        "seg03 | 1.25s | seq 42..52 | attitude_debug->direct_throttle"
    )
    assert workbench.available_preset_channels(
        ["roll_deg", "pitch_deg", "servo_alpha_us"],
        "姿态",
    ) == ["roll_deg", "pitch_deg"]


def test_workbench_rerun_command_uses_isolated_wrapper() -> None:
    command = workbench.make_rerun_wrapper_command(
        Path(r"D:\logs\flightlog.csv"),
        13,
        "--play",
        500,
    )

    assert command[:4] == ["powershell", "-ExecutionPolicy", "Bypass", "-File"]
    assert "run_flight_log_rerun_replay.ps1" in command[4]
    assert command[-5:] == [
        "--play",
        "--segment",
        "13",
        "--max-rows",
        "500",
    ]


def test_workbench_exposes_all_in_one_ui() -> None:
    source = read("tools/flight_log_workbench.py")

    assert "class FlightLogWorkbench(tk.Tk)" in source
    assert '"日志文件"' in source
    assert '"文件内片段"' in source
    assert '"画选中片段"' in source
    assert '"弹出Rerun回放"' in source
    assert '"导出RRD"' in source
    assert '"打开RRD目录"' in source
    assert "flight_log_rerun_replay" in source
    assert "flight_log_waveform_ui" in source
    assert "make_rerun_wrapper_command" in source
    assert "subprocess.Popen" in source
    assert "中文注释" in source

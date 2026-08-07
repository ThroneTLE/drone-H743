from __future__ import annotations

from pathlib import Path

from tools import flight_log_receive as flog


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def test_receive_canonical_output_dir_is_tools_data_flight_logs() -> None:
    assert flog.DEFAULT_OUT_DIR == ROOT / "tools" / "data" / "flight_logs"


def test_receive_has_headless_cli_and_canonical_gui_default() -> None:
    source = read("tools/flight_log_receive.py")

    assert "import argparse" in source
    assert "import sys" in source
    assert '"tools" / "data" / "flight_logs"' in source
    assert "--port" in source
    assert "--baud" in source
    assert "--out-dir" in source
    assert "value=str(DEFAULT_OUT_DIR)" in source


def test_waveform_and_rerun_read_canonical_log_dir() -> None:
    for name in ("flight_log_waveform_ui.py", "flight_log_rerun_replay.py"):
        source = read(f"tools/{name}")
        assert 'DEFAULT_LOG_DIR = ROOT_DIR / "tools" / "data" / "flight_logs"' in source


def test_sysid_ui_points_open_dialog_at_flight_logs() -> None:
    source = read("tools/flight_log_sysid_ui.py")

    assert 'DEFAULT_LOG_DIR = Path(__file__).resolve().parent / "data" / "flight_logs"' in source
    assert "initialdir=str(DEFAULT_LOG_DIR)" in source


def test_sysid_cli_writes_reports_by_default() -> None:
    source = read("tools/flight_log_sysid.py")

    assert "def default_report_dir" in source
    assert "out_dir = args.out_dir if args.out_dir is not None else default_report_dir(args.csv)" in source


def test_pressure_gui_paths_aligned_to_data_dirs() -> None:
    source = read("tools/pressure_rs485_gui.py")

    assert "pressure_calibration.json" in source
    assert '"data" / "pressure"' in source
    assert '"data" / "thrust_ident"' in source


def test_tcp_panel_uses_data_root_for_exports() -> None:
    source = read("tools/drone_tcp_panel.py")

    assert 'DATA_ROOT = Path(__file__).resolve().parent / "data"' in source
    assert 'DATA_ROOT / "ident_runs"' in source
    assert 'DATA_ROOT / "telemetry"' in source
    assert "initialdir=str(initial.parent)" in source


def test_saleae_and_vofa_default_out_dirs() -> None:
    saleae = read("tools/saleae_imu_spi_capture.py")
    vofa = read("tools/vofa_serial_capture.py")

    assert 'DEFAULT_OUT_DIR = Path(__file__).resolve().parent / "data" / "spi_captures"' in saleae
    assert "default=DEFAULT_OUT_DIR" in saleae
    assert 'DEFAULT_OUT_DIR = Path(__file__).resolve().parent / "data" / "imu_attitude_data"' in vofa
    assert "default=DEFAULT_OUT_DIR" in vofa

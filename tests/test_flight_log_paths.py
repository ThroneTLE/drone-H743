from __future__ import annotations

from pathlib import Path

from tools import flight_log_receive as flog
from tools import project_paths as paths


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def test_canonical_data_tree_is_root_scoped() -> None:
    assert paths.DATA_ROOT == ROOT / "data"
    assert paths.FLIGHT_LOG_DIR == ROOT / "data" / "flight_logs"
    assert paths.IMU_VIBRATION_CAPTURE_DIR == ROOT / "data" / "captures" / "imu_vibration"
    assert paths.ATTITUDE_IDENT_DIR == ROOT / "data" / "identification" / "attitude"
    assert paths.PRESSURE_CALIBRATION_DIR == ROOT / "data" / "calibration" / "pressure"
    assert paths.AIRFRAME_CALIBRATION_DIR == ROOT / "data" / "calibration" / "airframe"
    assert (paths.DATA_ROOT / "README.md").is_file()


def test_receive_uses_canonical_flight_log_dir() -> None:
    assert flog.DEFAULT_OUT_DIR.parent == paths.FLIGHT_LOG_DIR
    assert paths.DATE_DIRECTORY_RE.fullmatch(flog.DEFAULT_OUT_DIR.name)


def test_receive_has_headless_cli_and_canonical_gui_default() -> None:
    source = read("tools/flight_log_receive.py")

    assert "import argparse" in source
    assert "import sys" in source
    assert "DEFAULT_OUT_DIR = dated_directory(FLIGHT_LOG_DIR)" in source
    assert "--port" in source
    assert "--baud" in source
    assert "--out-dir" in source
    assert "value=str(DEFAULT_OUT_DIR)" in source


def test_waveform_and_rerun_read_canonical_log_dir() -> None:
    for name in ("flight_log_waveform_ui.py", "flight_log_rerun_replay.py"):
        source = read(f"tools/{name}")
        assert "DEFAULT_LOG_DIR = FLIGHT_LOG_DIR" in source


def test_sysid_ui_points_open_dialog_at_flight_logs() -> None:
    source = read("tools/flight_log_sysid_ui.py")

    assert "DEFAULT_LOG_DIR = FLIGHT_LOG_DIR" in source
    assert "latest_dated_directory(DEFAULT_LOG_DIR)" in source


def test_sysid_cli_writes_reports_by_default() -> None:
    source = read("tools/flight_log_sysid.py")

    assert "def default_report_dir" in source
    assert "dated_directory_for_name(FLIGHT_LOG_ANALYSIS_DIR, csv_path.name)" in source
    assert "out_dir = args.out_dir if args.out_dir is not None else default_report_dir(args.csv)" in source


def test_pressure_gui_paths_aligned_to_data_dirs() -> None:
    source = read("tools/pressure_rs485_gui.py")

    assert "pressure_calibration.json" in source
    assert "PRESSURE_CALIBRATION_DIR" in source
    assert "THRUST_IDENT_DIR" in source
    assert "dated_directory(THRUST_IDENT_DIR)" in source


def test_tcp_panel_uses_data_root_for_exports() -> None:
    source = read("tools/drone_tcp_panel.py")

    assert "ATTITUDE_IDENT_DIR" in source
    assert "TELEMETRY_DIR" in source
    assert "dated_directory(ATTITUDE_IDENT_DIR)" in source
    assert "dated_directory(TELEMETRY_DIR)" in source
    assert "initialdir=str(initial.parent)" in source


def test_saleae_and_vofa_default_out_dirs() -> None:
    saleae = read("tools/saleae_imu_spi_capture.py")
    vofa = read("tools/vofa_serial_capture.py")

    assert "DEFAULT_OUT_DIR = dated_directory(SALEAE_SPI_CAPTURE_DIR)" in saleae
    assert "default=DEFAULT_OUT_DIR" in saleae
    assert "DEFAULT_OUT_DIR = dated_directory(IMU_ATTITUDE_CAPTURE_DIR)" in vofa
    assert "default=DEFAULT_OUT_DIR" in vofa


def test_tool_sources_do_not_restore_legacy_data_roots() -> None:
    forbidden = ("tools/data", "tools\\data", 'parent / "data"')
    for path in (ROOT / "tools").glob("*.py"):
        source = path.read_text(encoding="utf-8")
        for token in forbidden:
            assert token not in source, f"legacy path {token!r} in {path.name}"

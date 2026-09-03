"""Canonical repository paths for captures, logs, calibration, and analysis data."""

from __future__ import annotations

import re
from datetime import date, datetime
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DATA_ROOT = PROJECT_ROOT / "data"

FLIGHT_LOG_DIR = DATA_ROOT / "flight_logs"
FLIGHT_LOG_DEBUG_DIR = FLIGHT_LOG_DIR / "debug_exports"

CAPTURE_ROOT = DATA_ROOT / "captures"
VOFA_CAPTURE_DIR = CAPTURE_ROOT / "vofa"
USB_FLIGHT_LOG_CAPTURE_DIR = CAPTURE_ROOT / "usb_flight_logs"
IMU_ATTITUDE_CAPTURE_DIR = CAPTURE_ROOT / "imu_attitude"
IMU_VIBRATION_CAPTURE_DIR = CAPTURE_ROOT / "imu_vibration"
SALEAE_SPI_CAPTURE_DIR = CAPTURE_ROOT / "saleae_spi"
SALEAE_PIN_ID_CAPTURE_DIR = CAPTURE_ROOT / "saleae_pin_id"
SALEAE_TEST_CAPTURE_DIR = CAPTURE_ROOT / "saleae_tests"

IDENTIFICATION_ROOT = DATA_ROOT / "identification"
ATTITUDE_IDENT_DIR = IDENTIFICATION_ROOT / "attitude"
MOTOR_IDENT_DIR = IDENTIFICATION_ROOT / "motor"
THRUST_IDENT_DIR = IDENTIFICATION_ROOT / "thrust"

CALIBRATION_ROOT = DATA_ROOT / "calibration"
PRESSURE_CALIBRATION_DIR = CALIBRATION_ROOT / "pressure"
AIRFRAME_CALIBRATION_DIR = CALIBRATION_ROOT / "airframe"
IMU_METROLOGY_CALIBRATION_DIR = CALIBRATION_ROOT / "imu_metrology"
FLIGHT_ACCEPTANCE_CALIBRATION_DIR = CALIBRATION_ROOT / "flight_acceptance_v2"
SERVO_MECHANICAL_CALIBRATION_DIR = CALIBRATION_ROOT / "servo_mechanical"
FLOW_RANGE_CALIBRATION_DIR = CALIBRATION_ROOT / "flow_range"

TELEMETRY_DIR = DATA_ROOT / "telemetry"
ANALYSIS_ROOT = DATA_ROOT / "analysis"
# PIPELINE M1 底层健康基线报告（tools/m1_baseline_check.py 产出）
M1_BASELINE_ANALYSIS_DIR = ANALYSIS_ROOT / "m1_baseline"
FLIGHT_LOG_ANALYSIS_DIR = ANALYSIS_ROOT / "flight_logs"
RERUN_REPLAY_DIR = FLIGHT_LOG_ANALYSIS_DIR / "rerun_replay"
FLIGHT_LOG_FLASH_TIMING_ANALYSIS_DIR = ANALYSIS_ROOT / "flight_log_flash_timing"
LOG_DIR = DATA_ROOT / "logs"
FIRMWARE_UPDATE_DIR = DATA_ROOT / "firmware_updates"

# 地面站的界面状态（上次连接的通道/串口/波特率）。不是测量数据，但和 data/ 下别的
# 东西一样属于"本机产物、不进版本库"。
PANEL_STATE_PATH = DATA_ROOT / "panel_state.json"

DATE_DIRECTORY_RE = re.compile(r"^20\d{2}-\d{2}-\d{2}$")
DATE_TOKEN_RE = re.compile(
    r"(?<!\d)(20\d{2})[-_]?([01]\d)[-_]?([0-3]\d)(?!\d)"
)


def ensure_directory(path: Path) -> Path:
    """Create a canonical output directory and return it."""
    path.mkdir(parents=True, exist_ok=True)
    return path


def date_from_name(name: str) -> date | None:
    """Extract and validate a YYYYMMDD/YYYY-MM-DD-style date token."""
    match = DATE_TOKEN_RE.search(name)
    if match is None:
        return None
    try:
        return date(int(match.group(1)), int(match.group(2)), int(match.group(3)))
    except ValueError:
        return None


def dated_directory(root: Path, when: date | datetime | None = None) -> Path:
    """Return a lexicographically sortable YYYY-MM-DD child directory."""
    value = when.date() if isinstance(when, datetime) else when or date.today()
    return root / value.isoformat()


def dated_directory_for_name(root: Path, name: str) -> Path:
    """Use a date embedded in a filename, falling back to today's date."""
    return dated_directory(root, date_from_name(name))


def latest_dated_directory(root: Path) -> Path:
    """Return the newest valid date child, or the category root if none exists."""
    if not root.is_dir():
        return root
    candidates = [
        path
        for path in root.iterdir()
        if path.is_dir() and DATE_DIRECTORY_RE.fullmatch(path.name)
    ]
    return max(candidates, key=lambda path: path.name, default=root)

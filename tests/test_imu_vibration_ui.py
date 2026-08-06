"""Tests for the vibration capture UI helpers.

The UI is a thin driver over tools/imu_vibration_capture.py. These tests cover
the parts that carry real risk: the test-step presets, the safety confirmation
set, and report rendering — all without starting Tk.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

from tools import imu_vibration_capture as ivc
from tools import imu_vibration_ui as ui


ROOT = Path(__file__).resolve().parents[1]


def _synthetic_rows(tone_hz: float = 250.0, count: int = 2048,
                    accel_lsb: int = 2048, clip: bool = False):
    t = np.arange(count) / 1000.0
    samples = []
    for i in range(count):
        gyro = int(3000 * np.sin(2 * np.pi * tone_hz * t[i]))
        ax = 32767 if clip else accel_lsb
        samples.append((i * 1000, ax, 0, accel_lsb, gyro, 0, 0,
                        5, 0, 1000, 20, 0, 0,
                        10, -20, 300, 1500, 1500, 1500, 1500, 0, 3, 25))
    meta = {"accel_range_g": 16, "gyro_range_dps": 1000,
            "accel_aaf_hz": 213, "gyro_aaf_hz": 213}
    return ivc.samples_to_rows(samples, meta), meta


def test_test_steps_cover_the_documented_sweep() -> None:
    tags = [tag for _, tag, _ in ui.TEST_STEPS]

    # Static gives the noise floor; the throttle sweep is what separates a real
    # blade tone (moves up with rpm) from an alias (moves down).
    assert tags == ["static", "thr30", "thr50", "thr70", "thr90", "hover"]
    assert all(name and hint for name, _, hint in ui.TEST_STEPS)


def test_spinning_steps_require_frame_confirmation() -> None:
    # Every step that spins the rotors must be gated behind a confirmation,
    # and the non-spinning ones must not be.
    assert ui.FRAME_WARNING_TAGS == {"thr30", "thr50", "thr70", "thr90"}
    assert "static" not in ui.FRAME_WARNING_TAGS


def test_report_formatting_includes_key_diagnostics() -> None:
    rows, meta = _synthetic_rows()
    report = ivc.analyse(rows, meta, label="synthetic.csv")

    text = ui._format_report(report)

    assert "synthetic.csv" in text
    assert "250.0Hz" in text
    # The AAF warning must survive into the rendered text, not just the dict.
    assert "AAF" in text


def test_report_formatting_surfaces_clipping() -> None:
    rows, meta = _synthetic_rows(clip=True)
    report = ivc.analyse(rows, meta, label="clipped.csv")

    text = ui._format_report(report)

    assert report["clipping"]["accel"]["clipped_pct"] > 30.0
    assert "clipping" in text


def test_ui_reuses_capture_module_rather_than_reimplementing() -> None:
    source = (ROOT / "tools" / "imu_vibration_ui.py").read_text(encoding="utf-8")

    # Protocol details belong in one place; the UI must not grow its own copy.
    for token in ("struct.unpack", "CAPTURE_MAGIC", "def crc32"):
        assert token not in source, f"{token} must stay in imu_vibration_capture"
    assert "ivc.run_capture" in source
    assert "ivc.analyse" in source


def test_capture_runs_off_the_ui_thread() -> None:
    source = (ROOT / "tools" / "imu_vibration_ui.py").read_text(encoding="utf-8")

    # Serial IO on the Tk thread would freeze the window for the whole dump.
    assert "threading.Thread" in source
    assert "_poll_events" in source

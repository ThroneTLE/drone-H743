from __future__ import annotations

import csv
import json
import socket
import struct
import threading
from pathlib import Path

from tools import imu_attitude_tuner as tuner


ROOT = Path(__file__).resolve().parents[1]
REAL_CAPTURE = (
    ROOT
    / "tools"
    / "data"
    / "imu_attitude_data"
    / "imu_vertical_shake_20260727_130109.csv"
)


def float_word(value: float) -> str:
    return f"{struct.unpack('<I', struct.pack('<f', value))[0]:08x}"


def packed_int16(low: int, high: int) -> str:
    value = struct.unpack("<I", struct.pack("<hh", low, high))[0]
    return f"{value:08x}"


def test_persistent_openocd_telnet_word_parser() -> None:
    listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listener.bind(("127.0.0.1", 0))
    listener.listen(1)
    port = listener.getsockname()[1]

    def serve() -> None:
        connection, _ = listener.accept()
        with connection:
            connection.sendall(b"Open On-Chip Debugger\r\n> ")
            stream = connection.makefile("rb")
            for raw_line in stream:
                line = raw_line.strip()
                if line == b"exit":
                    break
                connection.sendall(
                    line
                    + b"\r\n\x000x20000000: deadbeef 0000002a \r\n\r> "
                )
        listener.close()

    thread = threading.Thread(target=serve)
    thread.start()
    with tuner.OpenOcdTelnet("127.0.0.1", port) as openocd:
        assert openocd.read_words(0x20000000, 2) == ["deadbeef", "0000002a"]
    thread.join(timeout=2.0)
    assert not thread.is_alive()


def test_decode_openocd_message_layout() -> None:
    words = ["00000000"] * tuner.MESSAGE_WORD_COUNT
    words[0] = "89abcdef"
    words[1] = "01234567"
    words[3] = "0000002a"
    words[4] = packed_int16(410, 20)
    words[5] = packed_int16(-30, 8000)
    words[6] = packed_int16(-9, 12)
    words[7] = packed_int16(-7, 0)
    words[8] = float_word(28.25)
    words[9] = float_word(0.01)
    words[10] = float_word(-0.02)
    words[11] = float_word(0.99)
    words[12] = float_word(-0.1)
    words[13] = float_word(0.2)
    words[14] = float_word(-0.3)
    words[15] = float_word(1.25)
    words[16] = float_word(-2.5)
    words[17] = float_word(33.0)
    words[25] = float_word(1.0)
    words[26] = float_word(-2.0)
    words[27] = float_word(1.2)
    words[28] = float_word(-2.4)
    words[29] = float_word(-0.2)
    words[30] = float_word(0.4)
    words[31] = float_word(0.991)
    words[33] = float_word(0.45)
    words[35] = float_word(1.008)
    words[36] = "00000001"
    words[37] = "00000064"
    words[38] = "00000003"
    words[39] = float_word(12.5)
    words[40] = float_word(0.75)
    words[41] = "000004d2"
    words[42] = "01010001"

    row = tuner.decode_message(words, 1.5, "shake")

    assert row["timestamp_us"] == 0x0123456789ABCDEF
    assert row["sequence"] == 42
    assert row["raw_temperature"] == 410
    assert row["raw_accel_x"] == 20
    assert row["raw_accel_y"] == -30
    assert row["raw_accel_z"] == 8000
    assert row["raw_gyro_x"] == -9
    assert row["raw_gyro_y"] == 12
    assert row["roll_deg"] == 1.25
    assert row["pitch_deg"] == -2.5
    assert row["gyro_bias_ready"] == 1
    assert row["fusion_acceleration_error_deg"] == 12.5
    assert row["fusion_acceleration_recovery_trigger"] == 0.75
    assert row["fusion_accel_correction_count"] == 1234
    assert row["fusion_accelerometer_ignored"] == 1
    assert row["fusion_acceleration_recovery"] == 0
    assert row["fusion_angular_rate_recovery"] == 1
    assert row["fusion_accel_norm_rejected"] == 1


def test_decode_archived_160_byte_message_defaults_fusion_diagnostics() -> None:
    words = ["00000000"] * tuner.LEGACY_MESSAGE_WORD_COUNT
    row = tuner.decode_message(words, 0.0, "legacy")

    assert row["fusion_acceleration_error_deg"] == 0.0
    assert row["fusion_accel_correction_count"] == 0
    assert row["fusion_accelerometer_ignored"] == 0


def test_archived_capture_is_marked_as_pre_fusion() -> None:
    report = tuner.analyze_capture(REAL_CAPTURE)
    quality = report["quality"]

    assert quality["row_count"] == 3000
    assert quality["valid"] is True
    assert quality["timestamp_nonforward"] == 0
    assert quality["sequence_nonforward"] == 0
    assert report["fusion_diagnostics_present"] is False
    assert report["decision"] == "new_fusion_capture_required"


def test_fusion_capture_analysis_reports_rejection_and_final_tilt(tmp_path: Path) -> None:
    path = tmp_path / "fusion.csv"
    fields = [
        "host_time_s",
        "stage",
        "timestamp_us",
        "sequence",
        "roll_deg",
        "pitch_deg",
        "fusion_acceleration_error_deg",
        "fusion_acceleration_recovery_trigger",
        "fusion_accel_correction_count",
        "fusion_accelerometer_ignored",
        "fusion_acceleration_recovery",
        "fusion_angular_rate_recovery",
        "fusion_accel_norm_rejected",
    ]
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for index in range(100):
            writer.writerow(
                {
                    "host_time_s": index * 0.01,
                    "stage": "settle" if index >= 50 else "shake",
                    "timestamp_us": index * 10000,
                    "sequence": index,
                    "roll_deg": 0.2 if index >= 50 else 10.0,
                    "pitch_deg": -0.1 if index >= 50 else -5.0,
                    "fusion_acceleration_error_deg": 0.3 if index >= 50 else 20.0,
                    "fusion_acceleration_recovery_trigger": 0.0 if index >= 50 else 0.2,
                    "fusion_accel_correction_count": index,
                    "fusion_accelerometer_ignored": int(index < 50),
                    "fusion_acceleration_recovery": 0,
                    "fusion_angular_rate_recovery": 0,
                    "fusion_accel_norm_rejected": int(index < 20),
                }
            )

    report = tuner.analyze_capture(path, final_window_s=0.4)

    assert report["fusion_diagnostics_present"] is True
    assert report["decision"] == "keep_current_settings"
    assert report["fusion"]["accelerometer_ignored_fraction"] == 0.5
    assert report["fusion"]["accel_norm_rejected_fraction"] == 0.2
    assert report["final"]["tilt_median_deg"] < 0.25

    json_path = tuner.write_analysis_report(report, path, tmp_path)
    assert json.loads(json_path.read_text(encoding="utf-8"))["format"] == report["format"]


def test_firmware_uses_fusion_rejection_and_recovery_settings() -> None:
    source = (ROOT / "Driver" / "Src" / "drv_attitude_fusion.c").read_text(encoding="utf-8")
    assert "DRV_ATTITUDE_FUSION_GAIN 0.5f" in source
    # Widened deliberately; see tests/test_attitude_fusion_contract.py for the
    # measured reason the previous gates starved the accelerometer in flight.
    assert "DRV_ATTITUDE_FUSION_ACCEL_REJECTION_DEG 45.0f" in source
    assert "DRV_ATTITUDE_FUSION_REJECTION_TIMEOUT_S 0.5f" in source
    assert "DRV_ATTITUDE_FUSION_ACCEL_NORM_MIN_G 0.40f" in source
    assert "DRV_ATTITUDE_FUSION_ACCEL_NORM_MAX_G 1.80f" in source

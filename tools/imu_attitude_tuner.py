#!/usr/bin/env python3
"""Record IMU samples and analyse x-io Fusion rejection/recovery diagnostics."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import re
import shutil
import socket
import statistics
import struct
import subprocess
import sys
import threading
import time
from datetime import datetime
from pathlib import Path


LEGACY_MESSAGE_BYTES = 160
CURRENT_MESSAGE_BYTES = 176
LEGACY_MESSAGE_WORD_COUNT = LEGACY_MESSAGE_BYTES // 4
MESSAGE_WORD_COUNT = CURRENT_MESSAGE_BYTES // 4
QUEUE_ITEM_SIZE_WORD_INDEX = 16

CSV_FIELDS = (
    "host_time_s",
    "stage",
    "timestamp_us",
    "sequence",
    "raw_temperature",
    "raw_accel_x",
    "raw_accel_y",
    "raw_accel_z",
    "raw_gyro_x",
    "raw_gyro_y",
    "raw_gyro_z",
    "temperature_c",
    "accel_x_g",
    "accel_y_g",
    "accel_z_g",
    "gyro_x_dps",
    "gyro_y_dps",
    "gyro_z_dps",
    "roll_deg",
    "pitch_deg",
    "yaw_deg",
    "roll_acc_deg",
    "pitch_acc_deg",
    "roll_gyro_deg",
    "pitch_gyro_deg",
    "roll_residual_deg",
    "pitch_residual_deg",
    "accel_norm_g",
    "accel_residual_deg",
    "dt_ms",
    "gyro_bias_ready",
    "imu_data_ready_count",
    "imu_poll_ready_count",
    "fusion_acceleration_error_deg",
    "fusion_acceleration_recovery_trigger",
    "fusion_accel_correction_count",
    "fusion_accelerometer_ignored",
    "fusion_acceleration_recovery",
    "fusion_angular_rate_recovery",
    "fusion_accel_norm_rejected",
)

DIAGNOSTIC_ALIASES = {
    "acceleration_error_deg": (
        "fusion_acceleration_error_deg",
        "imu_accel_error_deg",
    ),
    "acceleration_recovery_trigger": (
        "fusion_acceleration_recovery_trigger",
        "imu_accel_recovery",
    ),
    "accel_correction_count": (
        "fusion_accel_correction_count",
        "imu_accel_correction_count",
    ),
    "accelerometer_ignored": (
        "fusion_accelerometer_ignored",
        "imu_accel_ignored",
    ),
    "acceleration_recovery": ("fusion_acceleration_recovery",),
    "angular_rate_recovery": ("fusion_angular_rate_recovery",),
    "accel_norm_rejected": (
        "fusion_accel_norm_rejected",
        "imu_accel_norm_rejected",
    ),
}


class OpenOcdTelnet:
    """Minimal persistent OpenOCD telnet client for non-mutating mdw reads."""

    def __init__(self, host: str, port: int, timeout_s: float = 2.0) -> None:
        self._socket = socket.create_connection((host, port), timeout=timeout_s)
        self._socket.settimeout(timeout_s)
        self._timeout_s = timeout_s
        self._read_until_prompt()

    def close(self) -> None:
        try:
            self._socket.sendall(b"exit\n")
        except OSError:
            pass
        self._socket.close()

    def __enter__(self) -> "OpenOcdTelnet":
        return self

    def __exit__(self, _type: object, _value: object, _traceback: object) -> None:
        self.close()

    def _read_until_prompt(self) -> str:
        deadline = time.monotonic() + self._timeout_s
        data = bytearray()
        while time.monotonic() < deadline:
            chunk = self._socket.recv(4096)
            if not chunk:
                break
            data.extend(chunk)
            if data.endswith(b"> "):
                return data.decode("latin1", errors="replace")
        raise TimeoutError("OpenOCD telnet prompt timeout")

    def command(self, command: str) -> str:
        self._socket.sendall(command.encode("ascii") + b"\n")
        return self._read_until_prompt()

    def read_words(self, address: int, count: int) -> list[str]:
        response = self.command(f"mdw 0x{address:08x} {count}")
        if "Failed to read memory" in response or "Fail reading CTRL/STAT" in response:
            detail = " ".join(
                line.strip("\x00\r ")
                for line in response.splitlines()
                if "Fail" in line
            )
            raise RuntimeError(f"OpenOCD target memory read failed: {detail}")
        words: list[str] = []
        pattern = re.compile(r"0x[0-9a-fA-F]+:\s+((?:[0-9a-fA-F]{8}\s*)+)")
        for match in pattern.finditer(response):
            words.extend(match.group(1).split())
        return words


def _read_words_with_retry(
    openocd: OpenOcdTelnet,
    address: int,
    count: int,
    description: str,
    attempts: int = 5,
) -> list[str]:
    last_error: Exception | None = None
    for _ in range(attempts):
        try:
            words = openocd.read_words(address, count)
            if len(words) >= count:
                return words
            last_error = RuntimeError(f"received {len(words)} of {count} words")
        except (OSError, TimeoutError, RuntimeError) as error:
            last_error = error
        time.sleep(0.05)
    raise RuntimeError(
        f"Unable to read {description} after {attempts} attempts: {last_error}"
    )


def _float_from_hex(word: str) -> float:
    return struct.unpack("<f", struct.pack("<I", int(word, 16)))[0]


def _low_int16(word: str) -> int:
    return struct.unpack("<h", struct.pack("<I", int(word, 16))[:2])[0]


def _high_int16(word: str) -> int:
    return struct.unpack("<h", struct.pack("<I", int(word, 16))[2:])[0]


def decode_message(
    words: list[str], host_time_s: float, stage: str
) -> dict[str, float | int | str]:
    if len(words) < LEGACY_MESSAGE_WORD_COUNT:
        raise ValueError(
            f"Expected at least {LEGACY_MESSAGE_WORD_COUNT} words, got {len(words)}"
        )
    timestamp_us = (int(words[1], 16) << 32) | int(words[0], 16)
    row: dict[str, float | int | str] = {
        "host_time_s": host_time_s,
        "stage": stage,
        "timestamp_us": timestamp_us,
        "sequence": int(words[3], 16),
        "raw_temperature": _low_int16(words[4]),
        "raw_accel_x": _high_int16(words[4]),
        "raw_accel_y": _low_int16(words[5]),
        "raw_accel_z": _high_int16(words[5]),
        "raw_gyro_x": _low_int16(words[6]),
        "raw_gyro_y": _high_int16(words[6]),
        "raw_gyro_z": _low_int16(words[7]),
        "temperature_c": _float_from_hex(words[8]),
        "accel_x_g": _float_from_hex(words[9]),
        "accel_y_g": _float_from_hex(words[10]),
        "accel_z_g": _float_from_hex(words[11]),
        "gyro_x_dps": _float_from_hex(words[12]),
        "gyro_y_dps": _float_from_hex(words[13]),
        "gyro_z_dps": _float_from_hex(words[14]),
        "roll_deg": _float_from_hex(words[15]),
        "pitch_deg": _float_from_hex(words[16]),
        "yaw_deg": _float_from_hex(words[17]),
        "roll_acc_deg": _float_from_hex(words[25]),
        "pitch_acc_deg": _float_from_hex(words[26]),
        "roll_gyro_deg": _float_from_hex(words[27]),
        "pitch_gyro_deg": _float_from_hex(words[28]),
        "roll_residual_deg": _float_from_hex(words[29]),
        "pitch_residual_deg": _float_from_hex(words[30]),
        "accel_norm_g": _float_from_hex(words[31]),
        "accel_residual_deg": _float_from_hex(words[33]),
        "dt_ms": _float_from_hex(words[35]),
        "gyro_bias_ready": int(words[36], 16) & 0xFF,
        "imu_data_ready_count": int(words[37], 16),
        "imu_poll_ready_count": int(words[38], 16),
    }
    if len(words) >= MESSAGE_WORD_COUNT:
        flags = int(words[42], 16)
        row.update(
            {
                "fusion_acceleration_error_deg": _float_from_hex(words[39]),
                "fusion_acceleration_recovery_trigger": _float_from_hex(words[40]),
                "fusion_accel_correction_count": int(words[41], 16),
                "fusion_accelerometer_ignored": flags & 0xFF,
                "fusion_acceleration_recovery": (flags >> 8) & 0xFF,
                "fusion_angular_rate_recovery": (flags >> 16) & 0xFF,
                "fusion_accel_norm_rejected": (flags >> 24) & 0xFF,
            }
        )
    else:
        row.update(
            {
                "fusion_acceleration_error_deg": 0.0,
                "fusion_acceleration_recovery_trigger": 0.0,
                "fusion_accel_correction_count": 0,
                "fusion_accelerometer_ignored": 0,
                "fusion_acceleration_recovery": 0,
                "fusion_angular_rate_recovery": 0,
                "fusion_accel_norm_rejected": 0,
            }
        )
    return row


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _find_nm(cmake_cache: Path, explicit_nm: Path | None) -> Path:
    if explicit_nm is not None:
        return explicit_nm.resolve()
    if cmake_cache.exists():
        match = re.search(
            r"^CMAKE_NM:FILEPATH=(.+)$",
            cmake_cache.read_text(encoding="utf-8"),
            re.MULTILINE,
        )
        if match:
            candidate = Path(match.group(1).strip())
            if candidate.exists():
                return candidate.resolve()
    found = shutil.which("arm-none-eabi-nm")
    if found:
        return Path(found).resolve()
    raise FileNotFoundError("arm-none-eabi-nm was not found; pass --nm")


def resolve_symbol_address(elf_path: Path, symbol: str, nm_path: Path) -> int:
    completed = subprocess.run(
        [str(nm_path), "-g", "--defined-only", str(elf_path)],
        check=True,
        capture_output=True,
        text=True,
    )
    pattern = re.compile(
        rf"^([0-9a-fA-F]+)\s+\w\s+{re.escape(symbol)}$", re.MULTILINE
    )
    match = pattern.search(completed.stdout)
    if not match:
        raise RuntimeError(f"Symbol {symbol} was not found in {elf_path}")
    return int(match.group(1), 16)


def _beep(frequency_hz: int, duration_ms: int) -> None:
    def worker() -> None:
        try:
            import winsound

            winsound.Beep(frequency_hz, duration_ms)
        except (ImportError, RuntimeError):
            sys.stdout.write("\a")
            sys.stdout.flush()

    threading.Thread(target=worker, daemon=True).start()


def _stage_at(elapsed_s: float, baseline_s: float, shake_s: float) -> str:
    if elapsed_s < baseline_s:
        return "baseline"
    if elapsed_s < baseline_s + shake_s:
        return "shake"
    return "settle"


def quality_summary(
    rows: list[dict[str, float | int | str]],
) -> dict[str, object]:
    if not rows:
        return {"row_count": 0, "valid": False}
    timestamps = [int(row["timestamp_us"]) for row in rows]
    sequences = [int(row["sequence"]) for row in rows]
    host_times = [float(row["host_time_s"]) for row in rows]
    stage_counts: dict[str, int] = {}
    for row in rows:
        stage = str(row.get("stage", "unknown"))
        stage_counts[stage] = stage_counts.get(stage, 0) + 1
    duration_s = host_times[-1] - host_times[0]
    return {
        "row_count": len(rows),
        "valid": len(rows) >= 2
        and all(
            timestamps[index] > timestamps[index - 1]
            for index in range(1, len(rows))
        )
        and all(
            sequences[index] > sequences[index - 1]
            for index in range(1, len(rows))
        ),
        "stage_counts": stage_counts,
        "host_duration_s": duration_s,
        "observed_rate_hz": (len(rows) - 1) / duration_s
        if duration_s > 0.0
        else 0.0,
        "timestamp_nonforward": sum(
            timestamps[index] <= timestamps[index - 1]
            for index in range(1, len(rows))
        ),
        "sequence_nonforward": sum(
            sequences[index] <= sequences[index - 1]
            for index in range(1, len(rows))
        ),
        "sequence_gap_max": max(
            (
                sequences[index] - sequences[index - 1]
                for index in range(1, len(rows))
            ),
            default=0,
        ),
    }


def record_openocd(
    output_path: Path,
    elf_path: Path,
    nm_path: Path,
    host: str,
    port: int,
    duration_s: float,
    sample_rate_hz: float,
    baseline_s: float,
    shake_s: float,
    queue_handle_address: int | None = None,
) -> tuple[Path, Path]:
    if duration_s <= baseline_s + shake_s:
        raise ValueError("duration must include a settle interval after baseline + shake")
    if sample_rate_hz <= 0.0:
        raise ValueError("sample rate must be positive")
    if queue_handle_address is None:
        queue_handle_address = resolve_symbol_address(
            elf_path, "vofaLogQueueHandle", nm_path
        )

    rows: list[dict[str, float | int | str]] = []
    read_failures = 0
    consecutive_read_failures = 0
    with OpenOcdTelnet(host, port) as openocd:
        handle_words = _read_words_with_retry(
            openocd, queue_handle_address, 1, "vofaLogQueueHandle"
        )
        queue_address = int(handle_words[0], 16)
        queue_words = _read_words_with_retry(
            openocd,
            queue_address,
            QUEUE_ITEM_SIZE_WORD_INDEX + 1,
            "the FreeRTOS queue structure",
        )
        message_address = int(queue_words[0], 16)
        item_size = int(queue_words[QUEUE_ITEM_SIZE_WORD_INDEX], 16)
        if item_size < LEGACY_MESSAGE_BYTES:
            raise RuntimeError(
                f"APP_Sensor_SampleMessage is {item_size} bytes; "
                f"at least {LEGACY_MESSAGE_BYTES} expected"
            )
        message_word_count = min(item_size // 4, MESSAGE_WORD_COUNT)

        print("BASELINE: keep the aircraft still")
        _beep(700, 200)
        previous_stage = "baseline"
        start = time.perf_counter()
        next_deadline = start
        period_s = 1.0 / sample_rate_hz
        while True:
            now = time.perf_counter()
            if now < next_deadline:
                time.sleep(next_deadline - now)
            sample_start = time.perf_counter()
            elapsed_s = sample_start - start
            if elapsed_s >= duration_s:
                break
            stage = _stage_at(elapsed_s, baseline_s, shake_s)
            if stage != previous_stage:
                previous_stage = stage
                if stage == "shake":
                    print("SHAKE: move the aircraft")
                    _beep(1200, 400)
                else:
                    print("SETTLE: put the aircraft down and keep it still")
                    _beep(800, 200)
                    threading.Timer(0.25, _beep, args=(800, 200)).start()
            try:
                words = openocd.read_words(message_address, message_word_count)
                rows.append(decode_message(words, elapsed_s, stage))
                consecutive_read_failures = 0
            except (OSError, TimeoutError, RuntimeError, ValueError) as error:
                read_failures += 1
                consecutive_read_failures += 1
                if consecutive_read_failures >= 10:
                    raise RuntimeError(
                        "OpenOCD capture stopped after 10 consecutive read "
                        f"failures: {error}"
                    ) from error
            next_deadline += period_s
            after_read = time.perf_counter()
            if next_deadline < after_read:
                missed = math.floor((after_read - next_deadline) / period_s) + 1
                next_deadline += missed * period_s

    if not rows:
        raise RuntimeError("OpenOCD capture produced no valid rows")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS)
        writer.writeheader()
        writer.writerows(rows)
    _beep(1400, 500)

    metadata = {
        "format": "drone-h743-openocd-imu-v2",
        "created_at": datetime.now().astimezone().isoformat(),
        "capture": {
            "duration_s": duration_s,
            "sample_rate_hz": sample_rate_hz,
            "baseline_s": baseline_s,
            "shake_s": shake_s,
            "settle_s": duration_s - baseline_s - shake_s,
            "read_failures": read_failures,
        },
        "openocd": {
            "host": host,
            "port": port,
            "read_only": True,
            "queue_handle_address": f"0x{queue_handle_address:08x}",
            "queue_address": f"0x{queue_address:08x}",
            "message_address": f"0x{message_address:08x}",
            "queue_item_size": item_size,
        },
        "elf": {"path": str(elf_path.resolve()), "sha256": _sha256(elf_path)},
        "csv": {"path": str(output_path.resolve()), "sha256": _sha256(output_path)},
        "quality": quality_summary(rows),
    }
    metadata_path = output_path.with_name(output_path.stem + "_meta.json")
    metadata_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    print(f"DONE: {len(rows)} rows -> {output_path}")
    return output_path, metadata_path


def _first_field(fieldnames: set[str], aliases: tuple[str, ...]) -> str | None:
    return next((name for name in aliases if name in fieldnames), None)


def _read_analysis_rows(
    path: Path,
) -> tuple[list[dict[str, str]], set[str], str, str, str | None]:
    with path.open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        fieldnames = set(reader.fieldnames or ())
        rows = list(reader)
    time_field = _first_field(fieldnames, ("host_time_s", "pc_elapsed_s"))
    sequence_field = _first_field(fieldnames, ("sequence", "frame_index"))
    timestamp_field = _first_field(fieldnames, ("timestamp_us",))
    if time_field is None or sequence_field is None:
        raise ValueError("CSV lacks host time or sequence/frame index")
    if "roll_deg" not in fieldnames or "pitch_deg" not in fieldnames:
        raise ValueError("CSV lacks roll_deg or pitch_deg")
    if len(rows) < 2:
        raise ValueError("CSV needs at least two rows")
    return rows, fieldnames, time_field, sequence_field, timestamp_field


def _float(row: dict[str, str], field: str) -> float:
    return float(row[field])


def analyze_capture(csv_path: Path, final_window_s: float = 2.0) -> dict[str, object]:
    rows, fieldnames, time_field, sequence_field, timestamp_field = (
        _read_analysis_rows(csv_path)
    )
    times = [_float(row, time_field) for row in rows]
    sequences = [int(float(row[sequence_field])) for row in rows]
    if timestamp_field is not None:
        timestamps = [int(float(row[timestamp_field])) for row in rows]
    else:
        timestamps = [int(time_value * 1_000_000.0) for time_value in times]
    stages = [row.get("stage", "stream") for row in rows]
    quality_rows: list[dict[str, float | int | str]] = [
        {
            "host_time_s": times[index],
            "timestamp_us": timestamps[index],
            "sequence": sequences[index],
            "stage": stages[index],
        }
        for index in range(len(rows))
    ]
    quality = quality_summary(quality_rows)

    resolved = {
        key: _first_field(fieldnames, aliases)
        for key, aliases in DIAGNOSTIC_ALIASES.items()
    }
    required = (
        "acceleration_error_deg",
        "acceleration_recovery_trigger",
        "accel_correction_count",
        "accelerometer_ignored",
        "accel_norm_rejected",
    )
    diagnostics_present = all(resolved[key] is not None for key in required)
    final_start = times[-1] - final_window_s
    final_rows = [row for row, row_time in zip(rows, times) if row_time >= final_start]
    roll_final = [_float(row, "roll_deg") for row in final_rows]
    pitch_final = [_float(row, "pitch_deg") for row in final_rows]
    roll_median = statistics.median(roll_final)
    pitch_median = statistics.median(pitch_final)
    final = {
        "window_s": final_window_s,
        "row_count": len(final_rows),
        "roll_median_deg": roll_median,
        "pitch_median_deg": pitch_median,
        "tilt_median_deg": math.hypot(roll_median, pitch_median),
        "roll_std_deg": statistics.pstdev(roll_final),
        "pitch_std_deg": statistics.pstdev(pitch_final),
    }

    fusion: dict[str, object] = {}
    decision = "new_fusion_capture_required"
    if diagnostics_present:
        acceleration_error = [
            _float(row, resolved["acceleration_error_deg"])  # type: ignore[arg-type]
            for row in rows
        ]
        recovery_trigger = [
            _float(row, resolved["acceleration_recovery_trigger"])  # type: ignore[arg-type]
            for row in rows
        ]
        correction_count = [
            int(_float(row, resolved["accel_correction_count"]))  # type: ignore[arg-type]
            for row in rows
        ]
        ignored = [
            int(_float(row, resolved["accelerometer_ignored"])) != 0  # type: ignore[arg-type]
            for row in rows
        ]
        norm_rejected = [
            int(_float(row, resolved["accel_norm_rejected"])) != 0  # type: ignore[arg-type]
            for row in rows
        ]
        fusion = {
            "acceleration_error_max_deg": max(acceleration_error),
            "acceleration_error_final_median_deg": statistics.median(
                acceleration_error[-len(final_rows) :]
            ),
            "acceleration_recovery_trigger_max": max(recovery_trigger),
            "acceleration_recovery_trigger_final": recovery_trigger[-1],
            "accelerometer_ignored_fraction": sum(ignored) / len(ignored),
            "accel_norm_rejected_fraction": sum(norm_rejected) / len(norm_rejected),
            "accel_correction_count_delta": correction_count[-1]
            - correction_count[0],
        }
        decision = (
            "keep_current_settings"
            if final["tilt_median_deg"] <= 1.0
            and fusion["acceleration_recovery_trigger_final"] < 0.1
            else "review_capture_before_parameter_change"
        )

    return {
        "format": "drone-h743-fusion-analysis-v1",
        "created_at": datetime.now().astimezone().isoformat(),
        "input_csv": str(csv_path.resolve()),
        "input_sha256": _sha256(csv_path),
        "quality": quality,
        "fusion_diagnostics_present": diagnostics_present,
        "firmware_settings": {
            "gain": 0.5,
            "acceleration_rejection_deg": 10.0,
            "rejection_timeout_s": 5.0,
            "accel_norm_gate_g": [0.85, 1.15],
        },
        "final": final,
        "fusion": fusion,
        "decision": decision,
        "parameter_changes_applied": False,
    }


def write_analysis_report(
    report: dict[str, object], csv_path: Path, output_dir: Path | None
) -> Path:
    directory = output_dir.resolve() if output_dir is not None else csv_path.resolve().parent
    directory.mkdir(parents=True, exist_ok=True)
    json_path = directory / f"{csv_path.stem}_fusion_analysis.json"
    json_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return json_path


def _default_output() -> Path:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return (
        Path(__file__).resolve().parent
        / "data"
        / "imu_attitude_data"
        / f"imu_fusion_bench_{timestamp}.csv"
    )


def _add_record_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--output", type=Path)
    parser.add_argument("--elf", type=Path, default=Path("build/Debug/drone-H743.elf"))
    parser.add_argument("--nm", type=Path)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=4444)
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--sample-rate", type=float, default=100.0)
    parser.add_argument("--baseline", type=float, default=5.0)
    parser.add_argument("--shake", type=float, default=8.0)
    parser.add_argument("--queue-handle-address", type=lambda value: int(value, 0))


def _print_analysis(report: dict[str, object], path: Path) -> None:
    print(f"decision={report['decision']}")
    print(f"fusion_diagnostics_present={report['fusion_diagnostics_present']}")
    print(f"final={json.dumps(report['final'], ensure_ascii=False)}")
    if report["fusion_diagnostics_present"]:
        print(f"fusion={json.dumps(report['fusion'], ensure_ascii=False)}")
    print(f"report={path}")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    record_parser = subparsers.add_parser(
        "record", help="optional read-only OpenOCD bench capture"
    )
    _add_record_arguments(record_parser)
    analyze_parser = subparsers.add_parser(
        "analyze", help="analyse OpenOCD or 28-channel VOFA CSV diagnostics"
    )
    analyze_parser.add_argument("csv", type=Path)
    analyze_parser.add_argument("--final-window", type=float, default=2.0)
    analyze_parser.add_argument("--report-dir", type=Path)
    run_parser = subparsers.add_parser(
        "run", help="record a bench sample and analyse it"
    )
    _add_record_arguments(run_parser)
    run_parser.add_argument("--final-window", type=float, default=2.0)
    run_parser.add_argument("--report-dir", type=Path)
    args = parser.parse_args(argv)

    if args.command in {"record", "run"}:
        csv_path = (args.output or _default_output()).resolve()
        elf_path = args.elf.resolve()
        nm_path = _find_nm(elf_path.parent / "CMakeCache.txt", args.nm)
        csv_path, _ = record_openocd(
            output_path=csv_path,
            elf_path=elf_path,
            nm_path=nm_path,
            host=args.host,
            port=args.port,
            duration_s=args.duration,
            sample_rate_hz=args.sample_rate,
            baseline_s=args.baseline,
            shake_s=args.shake,
            queue_handle_address=args.queue_handle_address,
        )
        if args.command == "record":
            return 0
    else:
        csv_path = args.csv.resolve()

    report = analyze_capture(csv_path, final_window_s=args.final_window)
    report_path = write_analysis_report(report, csv_path, args.report_dir)
    _print_analysis(report, report_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

"""Capture USART1/VOFA JustFloat telemetry from the flight controller.

The firmware currently sends 24 little-endian float32 values followed by the
VOFA JustFloat tail 00 00 80 7f. This tool records those frames to CSV and
keeps text replies in a sidecar log so flight tuning can be reviewed later.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import struct
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Iterable


VOFA_TAIL = b"\x00\x00\x80\x7f"
VOFA_FLOAT_COUNT = 24
VOFA_PAYLOAD_BYTES = VOFA_FLOAT_COUNT * 4
VOFA_FRAME_BYTES = VOFA_PAYLOAD_BYTES + len(VOFA_TAIL)

CHANNEL_NAMES = [
    "roll_deg",
    "pitch_deg",
    "yaw_deg",
    "range_height_m",
    "fc_time_s",
    "vel_est_x_m_s",
    "vel_est_y_m_s",
    "coax_roll_rate_kd",
    "coax_pitch_rate_kd",
    "coax_yaw_angle_kp",
    "coax_yaw_rate_kd",
    "coax_pitch_angle_kp",
    "coax_roll_angle_kp",
    "coax_accel_xy_limit_m_s2",
    "coax_accel_z_limit_m_s2",
    "vel_loop_x_kp",
    "vel_loop_y_kp",
    "vel_loop_output_limit_m_s2",
    "vel_loop_x_ki",
    "vel_loop_y_ki",
    "vel_loop_i_limit_m_s2",
    "vel_loop_x_kd",
    "vel_loop_y_kd",
    "vel_loop_enable",
]

TUNING_CHANNELS = [
    "roll_deg",
    "pitch_deg",
    "yaw_deg",
    "range_height_m",
    "vel_est_x_m_s",
    "vel_est_y_m_s",
]


@dataclass
class RunningStats:
    count: int = 0
    min_value: float = math.inf
    max_value: float = -math.inf
    sum_value: float = 0.0
    max_abs: float = 0.0

    def update(self, value: float) -> None:
        if not math.isfinite(value):
            return
        self.count += 1
        self.min_value = min(self.min_value, value)
        self.max_value = max(self.max_value, value)
        self.sum_value += value
        self.max_abs = max(self.max_abs, abs(value))

    def as_dict(self) -> dict[str, float | int | None]:
        if self.count == 0:
            return {
                "count": 0,
                "min": None,
                "max": None,
                "mean": None,
                "max_abs": None,
            }
        return {
            "count": self.count,
            "min": self.min_value,
            "max": self.max_value,
            "mean": self.sum_value / self.count,
            "max_abs": self.max_abs,
        }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Capture COM telemetry into CSV for flight tuning analysis."
    )
    parser.add_argument("--port", default="COM10", help="serial port, default COM10")
    parser.add_argument("--baud", type=int, default=57600, help="baudrate, default 57600")
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="capture duration in seconds; 0 means until Ctrl-C",
    )
    parser.add_argument(
        "--out-dir",
        default="captures",
        help="output directory, default captures",
    )
    parser.add_argument(
        "--prefix",
        default="vofa",
        help="output filename prefix, default vofa",
    )
    parser.add_argument(
        "--no-start",
        action="store_true",
        help="do not send Sensor_Data:1 before capture",
    )
    parser.add_argument(
        "--no-stop",
        action="store_true",
        help="do not send Sensor_Data:0 when exiting",
    )
    parser.add_argument(
        "--raw-bin",
        action="store_true",
        help="also save all received serial bytes to a .bin file",
    )
    parser.add_argument(
        "--status-period",
        type=float,
        default=1.0,
        help="console status print interval in seconds",
    )
    return parser


def safe_text(data: bytes) -> str:
    text = data.decode("utf-8", "replace")
    return "".join(
        ch if ch in "\r\n\t" or 32 <= ord(ch) < 127 or "\u4e00" <= ch <= "\u9fff" else "."
        for ch in text
    )


def split_lines(pending: bytearray, data: bytes) -> Iterable[str]:
    pending.extend(data)
    while True:
        newline_indexes = [idx for idx in (pending.find(b"\n"), pending.find(b"\r")) if idx >= 0]
        if not newline_indexes:
            break
        idx = min(newline_indexes)
        line = bytes(pending[:idx])
        del pending[: idx + 1]
        if line:
            yield safe_text(line).strip()
    if len(pending) > 1024:
        yield safe_text(bytes(pending)).strip()
        pending.clear()


def parse_frame(payload: bytes) -> tuple[float, ...] | None:
    if len(payload) != VOFA_PAYLOAD_BYTES:
        return None
    values = struct.unpack(f"<{VOFA_FLOAT_COUNT}f", payload)
    sanity_indexes = range(VOFA_FLOAT_COUNT)
    if not all(math.isfinite(values[idx]) and abs(values[idx]) < 1.0e9 for idx in sanity_indexes):
        return None
    return values


def write_metadata(
    path: Path,
    args: argparse.Namespace,
    csv_path: Path,
    text_path: Path,
    raw_path: Path | None,
    frames: int,
    text_lines: int,
    dropped_bytes: int,
    started_at: float,
    ended_at: float,
    stats: dict[str, RunningStats],
) -> None:
    elapsed = max(ended_at - started_at, 0.0)
    metadata = {
        "port": args.port,
        "baud": args.baud,
        "duration_s": elapsed,
        "frames": frames,
        "average_frame_hz": (frames / elapsed) if elapsed > 0.0 else None,
        "text_lines": text_lines,
        "dropped_or_noise_bytes": dropped_bytes,
        "csv": str(csv_path),
        "text_log": str(text_path),
        "raw_bin": str(raw_path) if raw_path is not None else None,
        "channel_names": CHANNEL_NAMES,
        "tuning_stats": {name: value.as_dict() for name, value in stats.items()},
        "notes": [
            "0/1/2 are roll, pitch, and yaw.",
            "3 is filtered rangefinder height; 4 is FC time.",
            "5/6 are fused X/Y velocity estimates.",
            "7-23 are dashboard slider parameter feedback channels.",
        ],
    }
    path.write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8")


def main() -> int:
    args = build_parser().parse_args()

    try:
        import serial
    except ImportError:
        print("pyserial is required: python -m pip install pyserial", file=sys.stderr)
        return 2

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    stem = f"{args.prefix}_{stamp}"
    csv_path = out_dir / f"{stem}.csv"
    text_path = out_dir / f"{stem}_text.txt"
    meta_path = out_dir / f"{stem}_meta.json"
    raw_path = out_dir / f"{stem}.bin" if args.raw_bin else None

    print(f"Opening {args.port} @ {args.baud} ...")
    stats = {name: RunningStats() for name in TUNING_CHANNELS}
    channel_index = {name: idx for idx, name in enumerate(CHANNEL_NAMES)}
    frames = 0
    text_lines = 0
    dropped_bytes = 0
    rx_buffer = bytearray()
    text_pending = bytearray()
    started_at = time.time()
    last_status_at = started_at

    with serial.Serial(args.port, args.baud, timeout=0.03, write_timeout=1.0) as port:
        port.reset_input_buffer()
        if not args.no_start:
            port.write(b"Sensor_Data:1\r\n")
            port.flush()

        with csv_path.open("w", newline="", encoding="utf-8") as csv_file, text_path.open(
            "w", encoding="utf-8"
        ) as text_file:
            raw_file = raw_path.open("wb") if raw_path is not None else None
            try:
                writer = csv.writer(csv_file)
                writer.writerow(["pc_elapsed_s", "pc_unix_s", "frame_index", *CHANNEL_NAMES])
                print(f"Capturing to {csv_path}")
                print("Press Ctrl-C to stop.")

                while True:
                    now = time.time()
                    if args.duration > 0.0 and (now - started_at) >= args.duration:
                        break

                    chunk = port.read(1024)
                    if not chunk:
                        continue
                    if raw_file is not None:
                        raw_file.write(chunk)
                    rx_buffer.extend(chunk)

                    while True:
                        tail_index = rx_buffer.find(VOFA_TAIL)
                        if tail_index < 0:
                            if len(rx_buffer) > VOFA_FRAME_BYTES * 3:
                                noise_len = len(rx_buffer) - VOFA_FRAME_BYTES
                                noise = bytes(rx_buffer[:noise_len])
                                del rx_buffer[:noise_len]
                                dropped_bytes += len(noise)
                                for line in split_lines(text_pending, noise):
                                    text_file.write(f"{time.time() - started_at:.6f},{line}\n")
                                    text_lines += 1
                            break

                        if tail_index < VOFA_PAYLOAD_BYTES:
                            noise = bytes(rx_buffer[: tail_index + len(VOFA_TAIL)])
                            del rx_buffer[: tail_index + len(VOFA_TAIL)]
                            dropped_bytes += len(noise)
                            for line in split_lines(text_pending, noise):
                                text_file.write(f"{time.time() - started_at:.6f},{line}\n")
                                text_lines += 1
                            continue

                        frame_start = tail_index - VOFA_PAYLOAD_BYTES
                        if frame_start > 0:
                            text_bytes = bytes(rx_buffer[:frame_start])
                            for line in split_lines(text_pending, text_bytes):
                                text_file.write(f"{time.time() - started_at:.6f},{line}\n")
                                text_lines += 1

                        payload = bytes(rx_buffer[frame_start:tail_index])
                        del rx_buffer[: tail_index + len(VOFA_TAIL)]
                        values = parse_frame(payload)
                        if values is None:
                            dropped_bytes += VOFA_FRAME_BYTES
                            continue

                        frames += 1
                        row_time = time.time()
                        writer.writerow(
                            [
                                f"{row_time - started_at:.6f}",
                                f"{row_time:.6f}",
                                frames,
                                *[f"{value:.9g}" for value in values],
                            ]
                        )

                        for name, running in stats.items():
                            running.update(values[channel_index[name]])

                    if now - last_status_at >= args.status_period:
                        elapsed = max(now - started_at, 1.0e-6)
                        hz = frames / elapsed
                        print(
                            f"frames={frames} hz={hz:.1f} "
                            f"text={text_lines} noise={dropped_bytes} "
                            f"csv={csv_path.name}"
                        )
                        csv_file.flush()
                        text_file.flush()
                        if raw_file is not None:
                            raw_file.flush()
                        last_status_at = now
            except KeyboardInterrupt:
                print("\nStopping capture ...")
            finally:
                if not args.no_stop:
                    try:
                        port.write(b"Sensor_Data:0\r\n")
                        port.flush()
                    except Exception:
                        pass
                if raw_file is not None:
                    raw_file.close()

    ended_at = time.time()
    write_metadata(
        meta_path,
        args,
        csv_path,
        text_path,
        raw_path,
        frames,
        text_lines,
        dropped_bytes,
        started_at,
        ended_at,
        stats,
    )
    elapsed = max(ended_at - started_at, 0.0)
    print(f"Saved {frames} frames in {elapsed:.1f}s")
    print(f"CSV : {csv_path}")
    print(f"Text: {text_path}")
    print(f"Meta: {meta_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

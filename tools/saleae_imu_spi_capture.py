#!/usr/bin/env python3
"""Capture the ICM42688 SPI bus with Saleae Logic 2 Automation.

Default channel mapping follows the current Saleae hookup:
  D0 = CS, D1 = SCK, D2 = MOSI, D3 = MISO
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
import time

from saleae import automation


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=10430, help="Logic 2 automation port")
    parser.add_argument("--duration", type=float, default=3.0, help="capture duration in seconds")
    parser.add_argument("--sample-rate", type=int, default=8_000_000, help="digital sample rate")
    parser.add_argument("--threshold", type=float, default=None, help="digital threshold volts")
    parser.add_argument("--channels", type=int, nargs="+", default=[0, 1, 2, 3, 4])
    parser.add_argument("--cs", type=int, default=0)
    parser.add_argument("--sck", type=int, default=1)
    parser.add_argument("--mosi", type=int, default=2)
    parser.add_argument("--miso", type=int, default=3)
    parser.add_argument("--mode", type=int, choices=[0, 3], default=0)
    parser.add_argument("--no-analyzer", action="store_true", help="skip adding/exporting Saleae SPI analyzer")
    parser.add_argument("--out", type=Path, default=Path("saleae_capture"))
    parser.add_argument("--device-id", default=None)
    return parser.parse_args()


def add_spi_analyzer(capture: automation.Capture, args: argparse.Namespace):
    settings = {
        "MOSI": args.mosi,
        "MISO": args.miso,
        "Clock": args.sck,
        "Enable": args.cs,
        "Bits per Transfer": "8 Bits per Transfer (Standard)",
    }

    if args.mode == 3:
        settings.update({
            "Clock State": "Clock is High when inactive (CPOL = 1)",
            "Clock Phase": "Data is Valid on Clock Trailing Edge (CPHA = 1)",
        })
    else:
        settings.update({
            "Clock State": "Clock is Low when inactive (CPOL = 0)",
            "Clock Phase": "Data is Valid on Clock Leading Edge (CPHA = 0)",
        })

    try:
        return capture.add_analyzer("SPI", label="ICM42688 SPI", settings=settings)
    except Exception as exc:
        print(f"[saleae] SPI analyzer with mode settings failed: {exc}")
        print("[saleae] retrying with channel-only SPI analyzer settings")
        return capture.add_analyzer("SPI", label="ICM42688 SPI", settings={
            "MOSI": args.mosi,
            "MISO": args.miso,
            "Clock": args.sck,
            "Enable": args.cs,
            "Bits per Transfer": "8 Bits per Transfer (Standard)",
        })


def export_capture(args: argparse.Namespace) -> Path:
    args.out = args.out.resolve()
    args.out.mkdir(parents=True, exist_ok=True)

    with automation.Manager.connect(port=args.port, connect_timeout_seconds=5.0) as manager:
        devices = manager.get_devices()
        if not devices:
            raise RuntimeError("No Saleae device found. Check Logic 2 and the USB connection.")

        device_id = args.device_id or devices[0].device_id
        print(f"[saleae] using device {device_id}")
        print("[saleae] start capture now; trigger IMU traffic with STATUS? / MODULES?")

        capture = manager.start_capture(
            device_id=device_id,
            device_configuration=automation.LogicDeviceConfiguration(
                enabled_digital_channels=args.channels,
                digital_sample_rate=args.sample_rate,
                digital_threshold_volts=args.threshold,
            ),
            capture_configuration=automation.CaptureConfiguration(
                capture_mode=automation.TimedCaptureMode(duration_seconds=args.duration),
            ),
        )
        capture.wait()

        timestamp = time.strftime("%Y%m%d_%H%M%S")
        export_dir = args.out / timestamp
        export_dir.mkdir(parents=True, exist_ok=True)

        spi_analyzer = None
        if not args.no_analyzer:
            spi_analyzer = add_spi_analyzer(capture, args)
            capture.export_data_table(str(export_dir / "spi.csv"), analyzers=[spi_analyzer])

        capture.export_raw_data_csv(str(export_dir), digital_channels=args.channels)
        capture.save_capture(str(export_dir / "imu_spi.sal"))
        capture.close()

    return export_dir


def load_digital_csv(path: Path) -> tuple[list[str], list[list[int | float]]]:
    with path.open("r", newline="") as f:
        reader = csv.reader(f)
        header = next(reader)
        rows: list[list[int | float]] = []
        for row in reader:
            if not row:
                continue
            rows.append([float(row[0])] + [int(v) for v in row[1:]])
    return header, rows


def summarize_edges(export_dir: Path, args: argparse.Namespace) -> None:
    digital_csv = export_dir / "digital.csv"
    if not digital_csv.exists():
        print(f"[saleae] raw export missing: {digital_csv}")
        return

    header, rows = load_digital_csv(digital_csv)
    if not rows:
        print("[saleae] no digital samples exported")
        return

    names = {name: idx for idx, name in enumerate(header)}
    cs_i = names.get(f"Channel {args.cs}", args.cs + 1)
    sck_i = names.get(f"Channel {args.sck}", args.sck + 1)

    cs_falls: list[float] = []
    sck_edges = 0
    last = rows[0]
    for row in rows[1:]:
        if last[cs_i] == 1 and row[cs_i] == 0:
            cs_falls.append(row[0])
        if last[sck_i] != row[sck_i]:
            sck_edges += 1
        last = row

    print(f"[saleae] exported {len(rows)} samples to {digital_csv}")
    print(f"[saleae] mapping: CS=D{args.cs}, SCK=D{args.sck}, MOSI=D{args.mosi}, MISO=D{args.miso}, mode={args.mode}")
    print(f"[saleae] CS falling edges: {len(cs_falls)}")
    print(f"[saleae] SCK edges: {sck_edges}")
    print(f"[saleae] first CS falls: {', '.join(f'{t:.6f}s' for t in cs_falls[:8])}")
    print(f"[saleae] columns: {header}")
    print("[saleae] files: digital.csv, spi.csv if analyzer export succeeded, imu_spi.sal")


def main() -> None:
    args = parse_args()
    export_dir = export_capture(args)
    summarize_edges(export_dir, args)


if __name__ == "__main__":
    main()

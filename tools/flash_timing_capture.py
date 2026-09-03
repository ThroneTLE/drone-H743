#!/usr/bin/env python3
"""Capture GD25Q32 page/32K/64K timings for the R-M1-3 decision.

This is deliberately a measurement tool, not the flight-log preerase
implementation.  It reuses the existing FLOG TESTFILL and FLASH SCRATCH TEST
commands, and reads the thin firmware probe through ST-Link/OpenOCD.
"""

from __future__ import annotations

import argparse
from datetime import datetime
import json
from pathlib import Path
import re
import struct
import subprocess
import sys
import tempfile
import time
from typing import Any

from flash_diag_test import LineReader, SerialTransport
from flash_timing_analysis import analyse_capture
from project_paths import (
    FLIGHT_LOG_FLASH_TIMING_ANALYSIS_DIR,
    dated_directory,
    ensure_directory,
)


PROBE_SYMBOL = "g_drv_gd25q32_timing_probe"
PROBE_MAGIC = 0x544C4646
PROBE_VERSION = 1
PROBE_WORDS = 148
SERIES_WORDS = 36
TARGET_ADDRESS = 0x00010000


def _default_openocd() -> Path:
    return Path("D:/Program Files/OpenOCD-20240916-0.12.0/bin/openocd.exe")


def _default_openocd_scripts() -> Path:
    return Path("D:/Program Files/OpenOCD-20240916-0.12.0/share/openocd/scripts")


def _default_nm() -> Path:
    return Path("E:/ST/STM32CubeCLT_1.18.0/GNU-tools-for-STM32/bin/arm-none-eabi-nm.exe")


def _send_command(
    reader: LineReader,
    command: str,
    prefixes: tuple[str, ...],
    timeout_s: float,
    transcript: list[dict[str, Any]],
) -> list[str]:
    reader.transport.reset_input()
    reader.transport.write_line(command)
    deadline = time.monotonic() + timeout_s
    lines: list[str] = []
    matched_at: float | None = None

    while time.monotonic() < deadline:
        for line in reader.poll():
            lines.append(line)
            if line.startswith(prefixes):
                matched_at = time.monotonic()
        if matched_at is not None and time.monotonic() - matched_at >= 0.15:
            break

    matched = [line for line in lines if line.startswith(prefixes)]
    transcript.append({"command": command, "lines": matched})
    if not matched:
        raise RuntimeError(f"no {prefixes!r} response for {command!r}")
    return matched


def _require_safety(lines_by_command: dict[str, list[str]]) -> None:
    flash = " ".join(lines_by_command["FLASH?"])
    flog = " ".join(lines_by_command["FLOG?"])
    rc = " ".join(lines_by_command["RC?"])
    imu = " ".join(lines_by_command["IMU?"])

    if not ("FLASH ok=1" in flash and "id=C84016" in flash):
        raise RuntimeError(f"unexpected flash identity/status: {flash}")
    for token in ("recording=0", "export=0", "pending=0", "buffered=0"):
        if token not in flog:
            raise RuntimeError(f"flight log is not idle ({token} missing): {flog}")
    if "armed=0" not in rc:
        raise RuntimeError(f"RC reports armed state: {rc}")
    for token in ("armed=0", "m1=1100", "m2=1100"):
        if token not in imu:
            raise RuntimeError(f"motor-safe IMU snapshot missing {token}: {imu}")


def _symbol_address(nm: Path, elf: Path) -> tuple[int, int]:
    output = subprocess.run(
        [str(nm), "-S", "--defined-only", str(elf)],
        check=True,
        capture_output=True,
        text=True,
    ).stdout
    pattern = re.compile(
        rf"^([0-9A-Fa-f]+)\s+([0-9A-Fa-f]+)\s+\w\s+{re.escape(PROBE_SYMBOL)}$",
        re.MULTILINE,
    )
    match = pattern.search(output)
    if match is None:
        raise RuntimeError(f"{PROBE_SYMBOL} not found in {elf}")
    return int(match.group(1), 16), int(match.group(2), 16)


def _run_openocd(openocd: Path, scripts: Path, commands: str) -> str:
    result = subprocess.run(
        [
            str(openocd),
            "-s",
            str(scripts),
            "-f",
            "interface/stlink.cfg",
            "-f",
            "target/stm32h7x.cfg",
            "-c",
            f"init; halt; {commands}; resume; shutdown",
        ],
        capture_output=True,
        text=True,
        timeout=30,
    )
    output = result.stdout + result.stderr
    if result.returncode != 0:
        raise RuntimeError(f"OpenOCD failed ({result.returncode}):\n{output}")
    return output


def _arm_probe(openocd: Path, scripts: Path, address: int) -> str:
    # Header: magic/version/tight/pending. Four series follow, 36 words each.
    commands = (
        f"mww 0x{address:08X} 0x{PROBE_MAGIC:08X}; "
        f"mww 0x{address + 4:08X} 0x{PROBE_VERSION:08X}; "
        f"mww 0x{address + 8:08X} 1; "
        f"mww 0x{address + 12:08X} 0; "
        f"mww 0x{address + 16:08X} 0 {4 * SERIES_WORDS}"
    )
    return _run_openocd(openocd, scripts, commands)


def _read_probe_words(openocd: Path, scripts: Path, address: int) -> tuple[list[int], str]:
    with tempfile.TemporaryDirectory(prefix="flog-flash-timing-") as temp_dir:
        dump_path = Path(temp_dir) / "probe.bin"
        output = _run_openocd(
            openocd,
            scripts,
            f"dump_image {dump_path.as_posix()} 0x{address:08X} {PROBE_WORDS * 4}",
        )
        data = dump_path.read_bytes()
    if len(data) != PROBE_WORDS * 4:
        raise RuntimeError(
            f"OpenOCD dumped {len(data)}/{PROBE_WORDS * 4} probe bytes:\n{output}"
        )
    return list(struct.unpack(f"<{PROBE_WORDS}I", data)), output


def _decode_series(words: list[int], offset: int) -> dict[str, Any]:
    count, min_us, max_us, sum_us = words[offset : offset + 4]
    stored = min(count, 32)
    samples = words[offset + 4 : offset + 4 + stored]
    return {
        "count": count,
        "min_us": min_us,
        "max_us": max_us,
        "sum_us": sum_us,
        "samples_us": samples,
    }


def _decode_probe(words: list[int]) -> dict[str, Any]:
    if words[0] != PROBE_MAGIC or words[1] != PROBE_VERSION:
        raise RuntimeError(
            f"probe ABI mismatch magic=0x{words[0]:08X} version={words[1]}"
        )
    names = ("erase_32k", "erase_64k", "page_after_32k", "page_after_64k")
    result: dict[str, Any] = {
        "magic": words[0],
        "version": words[1],
        "tight_poll_enabled": words[2],
        "pending_page_block_kb": words[3],
    }
    offset = 4
    for name in names:
        result[name] = _decode_series(words, offset)
        offset += SERIES_WORDS
    return result


def _run_samples(
    reader: LineReader,
    iterations: int,
    block_kb: int,
    transcript: list[dict[str, Any]],
) -> None:
    fill_sectors = 22 if block_kb == 32 else 30
    for iteration in range(iterations):
        fill = _send_command(
            reader,
            f"FLOG TESTFILL {fill_sectors}",
            ("FLOG TESTFILL", "ERR "),
            15.0,
            transcript,
        )
        if not any("FLOG TESTFILL ok" in line for line in fill):
            raise RuntimeError(f"prefill failed at {block_kb}K iteration {iteration + 1}: {fill}")
        scratch = _send_command(
            reader,
            f"FLASH SCRATCH TEST 0x{TARGET_ADDRESS:06X} {block_kb}",
            ("FLASH scratch_test", "ERR "),
            8.0,
            transcript,
        )
        line = next((item for item in scratch if item.startswith("FLASH scratch_test")), "")
        for token in ("erase_st=0", "read_st=0", "write_st=0", "match=1", "sr1=0x00"):
            if token not in line:
                raise RuntimeError(
                    f"scratch failed at {block_kb}K iteration {iteration + 1} ({token}): {line}"
                )
        print(f"{block_kb}K {iteration + 1}/{iterations}: {line}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--serial", default="COM31")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--iterations", type=int, default=20)
    parser.add_argument("--elf", type=Path, default=Path("build/Debug/drone-H743.elf"))
    parser.add_argument("--nm", type=Path, default=_default_nm())
    parser.add_argument("--openocd", type=Path, default=_default_openocd())
    parser.add_argument("--openocd-scripts", type=Path, default=_default_openocd_scripts())
    parser.add_argument("--output-dir", type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not 5 <= args.iterations <= 32:
        raise SystemExit("--iterations must be in [5, 32]")
    for path in (args.elf, args.nm, args.openocd, args.openocd_scripts):
        if not path.exists():
            raise SystemExit(f"required path not found: {path}")

    address, size = _symbol_address(args.nm, args.elf)
    if size != PROBE_WORDS * 4:
        raise SystemExit(f"probe size mismatch: ELF={size}, host={PROBE_WORDS * 4}")

    transcript: list[dict[str, Any]] = []
    transport = SerialTransport(args.serial, args.baud)
    try:
        reader = LineReader(transport)
        time.sleep(0.2)
        safety: dict[str, list[str]] = {}
        safety["FLASH?"] = _send_command(reader, "FLASH?", ("FLASH ", "ERR "), 3.0, transcript)
        safety["FLOG?"] = _send_command(reader, "FLOG?", ("FLOG ", "ERR "), 3.0, transcript)
        safety["RC?"] = _send_command(reader, "RC?", ("RC ", "ERR "), 3.0, transcript)
        safety["IMU?"] = _send_command(reader, "IMU?", ("IMU ", "ERR "), 3.0, transcript)
        _require_safety(safety)

        arm_log = _arm_probe(args.openocd, args.openocd_scripts, address)
        _run_samples(reader, args.iterations, 32, transcript)
        _run_samples(reader, args.iterations, 64, transcript)

        final: dict[str, list[str]] = {}
        final["FLASH?"] = _send_command(reader, "FLASH?", ("FLASH ", "ERR "), 3.0, transcript)
        final["FLOG?"] = _send_command(reader, "FLOG?", ("FLOG ", "ERR "), 3.0, transcript)
        final["RC?"] = _send_command(reader, "RC?", ("RC ", "ERR "), 3.0, transcript)
        final["IMU?"] = _send_command(reader, "IMU?", ("IMU ", "ERR "), 3.0, transcript)
        _require_safety(final)
    finally:
        transport.close()

    words, read_log = _read_probe_words(args.openocd, args.openocd_scripts, address)
    probe = _decode_probe(words)
    for name in ("erase_32k", "erase_64k", "page_after_32k", "page_after_64k"):
        if probe[name]["count"] != args.iterations:
            raise SystemExit(
                f"{name} captured {probe[name]['count']}, expected {args.iterations}"
            )

    now = datetime.now().astimezone()
    output_dir = ensure_directory(
        args.output_dir or dated_directory(FLIGHT_LOG_FLASH_TIMING_ANALYSIS_DIR, now)
    )
    stamp = now.strftime("%Y%m%d_%H%M%S")
    capture_path = output_dir / f"flash_timing_capture_{stamp}.json"
    report_path = output_dir / f"flash_timing_analysis_{stamp}.json"
    capture = {
        "format": "drone-h743-flight-log-flash-timing-capture",
        "schema": 1,
        "created_at": now.isoformat(),
        "firmware_elf": str(args.elf),
        "probe_symbol_address": f"0x{address:08X}",
        "target_address": f"0x{TARGET_ADDRESS:06X}",
        "iterations_per_block": args.iterations,
        "destructive_scope": (
            "physical flight-log sectors 0..29 at 0x002000..0x01FFFF were "
            "overwritten by prefill; sectors 14..29 were erased at the end"
        ),
        "safety_before": safety,
        "safety_after": final,
        "probe": probe,
        "samples_us": {
            name: probe[name]["samples_us"]
            for name in ("erase_32k", "erase_64k", "page_after_32k", "page_after_64k")
        },
        "flight_log_layout": {
            "sector_size": 4096,
            "header_size": 256,
            "record_size": 528,
            "batch_records": 4,
            "page_size": 256,
            "background_wait_ms": 5,
        },
        "acceptance_policy": {
            "required_rate_hz": 250.0,
            "required_margin_fraction": 0.20,
        },
        "transcript": transcript,
        "openocd_arm_log": arm_log,
        "openocd_read_log": read_log,
    }
    capture_path.write_text(
        json.dumps(capture, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
        newline="\n",
    )
    capture["capture_path"] = str(capture_path)
    report = analyse_capture(capture)
    report["created_at"] = now.isoformat()
    report_path.write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
        newline="\n",
    )
    print(f"capture={capture_path}")
    print(f"analysis={report_path}")
    print(f"verdict={report['verdict']} recommended={report['recommended_block']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

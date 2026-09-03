#!/usr/bin/env python3
"""Calculate conservative flight-log throughput from real Flash timings."""

from __future__ import annotations

import argparse
import json
import math
import statistics
from pathlib import Path
from typing import Any, Sequence


def page_program_operations(offset: int, length: int, page_size: int) -> int:
    if offset < 0 or length <= 0 or page_size <= 0:
        raise ValueError("offset, length and page_size must describe a non-empty range")
    return math.ceil(((offset % page_size) + length) / page_size)


def page_programs_per_sector(
    *,
    sector_size: int,
    header_size: int,
    record_size: int,
    batch_records: int,
    page_size: int,
) -> int:
    records = (sector_size - header_size) // record_size
    if records <= 0 or batch_records <= 0:
        raise ValueError("sector layout must fit at least one record")

    operations = page_program_operations(0, header_size, page_size)
    written = 0
    while written < records:
        batch = min(batch_records, records - written)
        offset = header_size + written * record_size
        operations += page_program_operations(offset, batch * record_size, page_size)
        written += batch
    return operations


def distribution(samples: Sequence[int]) -> dict[str, float | int]:
    if not samples:
        raise ValueError("at least one timing sample is required")
    values = sorted(int(value) for value in samples)
    if values[0] <= 0:
        raise ValueError("timing samples must be positive")
    p95_index = max(0, math.ceil(len(values) * 0.95) - 1)
    return {
        "count": len(values),
        "min_us": values[0],
        "median_us": float(statistics.median(values)),
        "mean_us": float(statistics.fmean(values)),
        "p95_us": values[p95_index],
        "max_us": values[-1],
    }


def block_throughput(
    *,
    block_size: int,
    sector_size: int,
    header_size: int,
    record_size: int,
    batch_records: int,
    page_size: int,
    block_erase_samples_us: Sequence[int],
    page_program_samples_us: Sequence[int],
    background_wait_ms: int,
    required_rate_hz: float,
    required_margin_fraction: float,
) -> dict[str, Any]:
    if block_size % sector_size:
        raise ValueError("block_size must contain whole logical sectors")
    logical_sectors = block_size // sector_size
    records_per_sector = (sector_size - header_size) // record_size
    page_ops = page_programs_per_sector(
        sector_size=sector_size,
        header_size=header_size,
        record_size=record_size,
        batch_records=batch_records,
        page_size=page_size,
    )
    background_waits = math.ceil(records_per_sector / batch_records)
    erase_stats = distribution(block_erase_samples_us)
    page_stats = distribution(page_program_samples_us)
    erase_worst_us = int(erase_stats["max_us"])
    page_worst_us = int(page_stats["max_us"])
    page_mean_us = float(page_stats["mean_us"])
    per_sector_us = (
        page_ops * page_worst_us
        + background_waits * background_wait_ms * 1000
    )
    cycle_us = erase_worst_us + logical_sectors * per_sector_us
    mean_page_per_sector_us = (
        page_ops * page_mean_us
        + background_waits * background_wait_ms * 1000
    )
    mean_page_cycle_us = erase_worst_us + logical_sectors * mean_page_per_sector_us
    no_wait_mean_page_cycle_us = (
        erase_worst_us + logical_sectors * page_ops * page_mean_us
    )
    no_wait_worst_page_cycle_us = (
        erase_worst_us + logical_sectors * page_ops * page_worst_us
    )
    records_per_block = logical_sectors * records_per_sector
    throughput_hz = records_per_block * 1_000_000.0 / cycle_us
    mean_page_throughput_hz = records_per_block * 1_000_000.0 / mean_page_cycle_us
    no_wait_mean_page_throughput_hz = (
        records_per_block * 1_000_000.0 / no_wait_mean_page_cycle_us
    )
    no_wait_worst_page_throughput_hz = (
        records_per_block * 1_000_000.0 / no_wait_worst_page_cycle_us
    )
    required_with_margin_hz = required_rate_hz * (1.0 + required_margin_fraction)

    return {
        "logical_sectors": logical_sectors,
        "records_per_sector": records_per_sector,
        "records_per_block": records_per_block,
        "page_programs_per_sector": page_ops,
        "background_waits_per_sector": background_waits,
        "erase_distribution": erase_stats,
        "page_program_distribution": page_stats,
        "erase_worst_us": erase_worst_us,
        "page_program_worst_us": page_worst_us,
        "per_sector_non_erase_us": per_sector_us,
        "cycle_us": cycle_us,
        "throughput_hz": throughput_hz,
        "mean_page_cycle_us": mean_page_cycle_us,
        "mean_page_throughput_hz": mean_page_throughput_hz,
        "no_wait_mean_page_cycle_us": no_wait_mean_page_cycle_us,
        "no_wait_mean_page_throughput_hz": no_wait_mean_page_throughput_hz,
        "no_wait_worst_page_cycle_us": no_wait_worst_page_cycle_us,
        "no_wait_worst_page_throughput_hz": no_wait_worst_page_throughput_hz,
        "required_rate_hz": required_rate_hz,
        "required_with_margin_hz": required_with_margin_hz,
        "margin_fraction": throughput_hz / required_rate_hz - 1.0,
        "go": throughput_hz >= required_with_margin_hz,
    }


def analyse_capture(capture: dict[str, Any]) -> dict[str, Any]:
    layout = capture["flight_log_layout"]
    policy = capture["acceptance_policy"]
    samples = capture["samples_us"]
    common = {
        "sector_size": int(layout["sector_size"]),
        "header_size": int(layout["header_size"]),
        "record_size": int(layout["record_size"]),
        "batch_records": int(layout["batch_records"]),
        "page_size": int(layout["page_size"]),
        "background_wait_ms": int(layout["background_wait_ms"]),
        "required_rate_hz": float(policy["required_rate_hz"]),
        "required_margin_fraction": float(policy["required_margin_fraction"]),
    }
    result_32k = block_throughput(
        block_size=32768,
        block_erase_samples_us=samples["erase_32k"],
        page_program_samples_us=samples["page_after_32k"],
        **common,
    )
    result_64k = block_throughput(
        block_size=65536,
        block_erase_samples_us=samples["erase_64k"],
        page_program_samples_us=samples["page_after_64k"],
        **common,
    )
    go = bool(result_32k["go"] or result_64k["go"])
    return {
        "format": "drone-h743-flight-log-flash-timing-analysis",
        "schema": 1,
        "source_capture": capture.get("capture_path"),
        "blocks": {"32k": result_32k, "64k": result_64k},
        "verdict": "GO" if go else "NO_GO",
        "recommended_block": (
            "64k" if result_64k["throughput_hz"] >= result_32k["throughput_hz"] else "32k"
        ) if go else None,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("capture", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    capture = json.loads(args.capture.read_text(encoding="utf-8"))
    capture["capture_path"] = str(args.capture)
    report = analyse_capture(capture)
    text = json.dumps(report, indent=2, ensure_ascii=False) + "\n"
    if args.output is None:
        print(text, end="")
    else:
        args.output.write_text(text, encoding="utf-8", newline="\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

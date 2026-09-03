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


def overhead_distribution(samples: Sequence[int]) -> dict[str, float | int]:
    if not samples:
        raise ValueError("at least one handshake sample is required")
    values = sorted(int(value) for value in samples)
    if values[0] < 0:
        raise ValueError("handshake samples must be non-negative")
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


def cooperative_block_throughput(
    *,
    block_size: int,
    sector_size: int,
    header_size: int,
    record_size: int,
    batch_records: int,
    page_size: int,
    block_erase_samples_us: Sequence[int],
    page_program_samples_us: Sequence[int],
    suspend_samples_us: Sequence[int],
    resume_samples_us: Sequence[int],
    required_rate_hz: float,
    required_margin_fraction: float,
) -> dict[str, Any]:
    """Conservative single-chip erase-suspend throughput envelope.

    An erase and a page program cannot make forward progress simultaneously.
    Suspending merely lets each flight-log write batch consume the same Flash
    time budget before the erase resumes.  The bound therefore adds the full
    measured erase time, every page-program operation, and one suspend/resume
    handshake per logical write batch.
    """
    if block_size % sector_size:
        raise ValueError("block_size must contain whole logical sectors")
    logical_sectors = block_size // sector_size
    records_per_sector = (sector_size - header_size) // record_size
    if records_per_sector <= 0 or batch_records <= 0:
        raise ValueError("sector layout must fit at least one record")
    page_ops_per_sector = page_programs_per_sector(
        sector_size=sector_size,
        header_size=header_size,
        record_size=record_size,
        batch_records=batch_records,
        page_size=page_size,
    )
    interruptions_per_sector = math.ceil(records_per_sector / batch_records)
    records_per_block = logical_sectors * records_per_sector
    page_ops_per_block = logical_sectors * page_ops_per_sector
    interruptions_per_block = logical_sectors * interruptions_per_sector

    erase_stats = distribution(block_erase_samples_us)
    page_stats = distribution(page_program_samples_us)
    suspend_stats = overhead_distribution(suspend_samples_us)
    resume_stats = overhead_distribution(resume_samples_us)
    erase_worst_us = int(erase_stats["max_us"])
    page_worst_us = int(page_stats["max_us"])
    suspend_worst_us = int(suspend_stats["max_us"])
    resume_worst_us = int(resume_stats["max_us"])

    zero_handshake_cycle_us = erase_worst_us + page_ops_per_block * page_worst_us
    handshake_us = interruptions_per_block * (suspend_worst_us + resume_worst_us)
    cycle_us = zero_handshake_cycle_us + handshake_us
    zero_handshake_upper_bound_hz = records_per_block * 1_000_000.0 / zero_handshake_cycle_us
    throughput_hz = records_per_block * 1_000_000.0 / cycle_us
    required_with_margin_hz = required_rate_hz * (1.0 + required_margin_fraction)

    return {
        "logical_sectors": logical_sectors,
        "records_per_sector": records_per_sector,
        "records_per_block": records_per_block,
        "page_programs_per_sector": page_ops_per_sector,
        "page_programs_per_block": page_ops_per_block,
        "interruptions_per_sector": interruptions_per_sector,
        "interruptions_per_block": interruptions_per_block,
        "erase_distribution": erase_stats,
        "page_program_distribution": page_stats,
        "suspend_distribution": suspend_stats,
        "resume_distribution": resume_stats,
        "zero_handshake_cycle_us": zero_handshake_cycle_us,
        "zero_handshake_upper_bound_hz": zero_handshake_upper_bound_hz,
        "handshake_worst_us_per_interruption": suspend_worst_us + resume_worst_us,
        "handshake_us_per_block": handshake_us,
        "cycle_us": cycle_us,
        "throughput_hz": throughput_hz,
        "required_rate_hz": required_rate_hz,
        "required_with_margin_hz": required_with_margin_hz,
        "margin_fraction": throughput_hz / required_rate_hz - 1.0,
        "go": throughput_hz >= required_with_margin_hz,
    }


def _simulate_queue_peak(
    *,
    start_sector: int,
    arrival_phase_us: int,
    sectors_to_open: int,
    rate_hz: int,
    batch_records: int,
    sector_size: int,
    header_size: int,
    record_size: int,
    page_size: int,
    page_program_us: int,
    sector_erase_us: int,
    block_erase_us: int,
    background_wait_us: int,
    region_start: int,
    region_end_excl: int,
    block_size: int,
) -> dict[str, int]:
    period_us = 1_000_000 // rate_hz
    sector_count = (region_end_excl - region_start) // sector_size
    records_per_sector = (sector_size - header_size) // record_size
    first_full_block = ((region_start + block_size - 1) // block_size) * block_size
    full_blocks_end = (region_end_excl // block_size) * block_size

    now_us = 0
    next_arrival_us = arrival_phase_us
    queue_records = 0
    peak_records = 0
    total_arrivals = 0
    total_written = 0
    next_sector = start_sector
    sector_open = False
    sector_records = 0
    prepared_block_base: int | None = None
    opened = 0

    def advance(duration_us: int) -> None:
        nonlocal now_us, next_arrival_us, queue_records, peak_records, total_arrivals
        end_us = now_us + duration_us
        if next_arrival_us <= end_us:
            arrivals = ((end_us - next_arrival_us) // period_us) + 1
            queue_records += arrivals
            total_arrivals += arrivals
            next_arrival_us += arrivals * period_us
            peak_records = max(peak_records, queue_records)
        now_us = end_us

    advance(0)
    guard = 0
    while opened < sectors_to_open:
        guard += 1
        if guard > sectors_to_open * 20:
            raise RuntimeError("queue simulation did not make progress")

        if queue_records < batch_records:
            advance(background_wait_us)
            continue

        if not sector_open:
            address = region_start + next_sector * sector_size
            in_prepared_block = (
                prepared_block_base is not None
                and prepared_block_base <= address < prepared_block_base + block_size
            )
            is_full_block_start = (
                address == first_full_block
                or (
                    first_full_block < address < full_blocks_end
                    and address % block_size == 0
                )
            )
            if in_prepared_block:
                pass
            elif is_full_block_start:
                advance(block_erase_us)
                prepared_block_base = address
            else:
                advance(sector_erase_us)
                prepared_block_base = None
            advance(page_program_us)
            sector_open = True
            sector_records = 0
            opened += 1

        available = records_per_sector - sector_records
        batch = min(batch_records, available, queue_records)
        queue_records -= batch
        total_written += batch
        offset = header_size + sector_records * record_size
        advance(page_program_operations(offset, batch * record_size, page_size) * page_program_us)
        sector_records += batch
        if sector_records == records_per_sector:
            sector_open = False
            next_sector = (next_sector + 1) % sector_count
            if prepared_block_base is not None:
                next_address = region_start + next_sector * sector_size
                if not (
                    prepared_block_base
                    <= next_address
                    < prepared_block_base + block_size
                ):
                    prepared_block_base = None
        advance(background_wait_us)

    return {
        "peak_records": peak_records,
        "elapsed_us": now_us,
        "total_arrivals": total_arrivals,
        "total_written": total_written,
    }


def worst_case_queue_peak(
    *,
    rate_hz: int,
    queue_capacity: int,
    batch_records: int,
    sector_size: int,
    header_size: int,
    record_size: int,
    page_size: int,
    page_program_us: int,
    sector_erase_us: int,
    block_erase_us: int,
    background_wait_us: int,
    region_start: int,
    region_end_excl: int,
    block_size: int,
) -> dict[str, Any]:
    if 1_000_000 % rate_hz:
        raise ValueError("rate_hz must have an integral microsecond period")
    if (region_end_excl - region_start) % sector_size:
        raise ValueError("log region must contain whole logical sectors")

    sector_count = (region_end_excl - region_start) // sector_size
    first_full_block = ((region_start + block_size - 1) // block_size) * block_size
    full_blocks_end = (region_end_excl // block_size) * block_size
    head_fringe = (first_full_block - region_start) // sector_size
    tail_fringe = (region_end_excl - full_blocks_end) // sector_size
    block_sectors = block_size // sector_size
    last_block_start = (full_blocks_end - block_size - region_start) // sector_size
    startup_worst_sector_erases = (block_sectors - 1) + tail_fringe + head_fringe
    candidate_starts = sorted(
        {
            last_block_start,
            last_block_start + 1,
            sector_count - tail_fringe,
            0,
            head_fringe,
        }
    )
    sectors_to_open = startup_worst_sector_erases + (2 * block_sectors) + 2
    period_us = 1_000_000 // rate_hz
    worst: dict[str, int] | None = None
    worst_start = 0
    worst_phase = 0
    for start_sector in candidate_starts:
        for phase_us in range(period_us):
            result = _simulate_queue_peak(
                start_sector=start_sector,
                arrival_phase_us=phase_us,
                sectors_to_open=sectors_to_open,
                rate_hz=rate_hz,
                batch_records=batch_records,
                sector_size=sector_size,
                header_size=header_size,
                record_size=record_size,
                page_size=page_size,
                page_program_us=page_program_us,
                sector_erase_us=sector_erase_us,
                block_erase_us=block_erase_us,
                background_wait_us=background_wait_us,
                region_start=region_start,
                region_end_excl=region_end_excl,
                block_size=block_size,
            )
            if worst is None or result["peak_records"] > worst["peak_records"]:
                worst = result
                worst_start = start_sector
                worst_phase = phase_us
    assert worst is not None
    comfort_limit = math.floor(queue_capacity * 0.75)
    return {
        **worst,
        "rate_hz": rate_hz,
        "queue_capacity": queue_capacity,
        "comfort_limit": comfort_limit,
        "comfortable": worst["peak_records"] <= comfort_limit,
        "overflow_records": max(0, worst["peak_records"] - queue_capacity),
        "worst_start_sector": worst_start,
        "worst_arrival_phase_us": worst_phase,
        "head_fringe_sectors": head_fringe,
        "tail_fringe_sectors": tail_fringe,
        "steady_wrap_sector_erases": head_fringe + tail_fringe,
        "startup_worst_sector_erases": startup_worst_sector_erases,
        "candidate_start_sectors": candidate_starts,
        "simulated_sector_opens": sectors_to_open,
    }


def analyse_queue_peak_capture(capture: dict[str, Any]) -> dict[str, Any]:
    layout = capture["flight_log_layout"]
    policy = capture["acceptance_policy"]
    baseline = capture["baseline_samples_us"]
    samples = capture["samples_us"]
    common = {
        "rate_hz": int(policy["candidate_rate_hz"]),
        "batch_records": int(layout["batch_records"]),
        "sector_size": int(layout["sector_size"]),
        "header_size": int(layout["header_size"]),
        "record_size": int(layout["record_size"]),
        "page_size": int(layout["page_size"]),
        "page_program_us": max(int(value) for value in baseline["page_after_32k"]),
        "sector_erase_us": max(int(value) for value in samples["erase_4k"]),
        "block_erase_us": max(int(value) for value in baseline["erase_32k"]),
        "background_wait_us": int(layout["background_wait_ms"]) * 1000,
        "region_start": int(layout["region_start"]),
        "region_end_excl": int(layout["region_end_excl"]),
        "block_size": 32768,
    }
    queue_32 = worst_case_queue_peak(queue_capacity=32, **common)
    queue_64 = worst_case_queue_peak(queue_capacity=64, **common)
    selected_capacity = 32 if queue_32["comfortable"] else 64
    selected = queue_32 if selected_capacity == 32 else queue_64
    return {
        "format": "drone-h743-flight-log-queue-peak-analysis",
        "schema": 1,
        "source_capture": capture.get("capture_path"),
        "timing_worst_us": {
            "erase_4k": common["sector_erase_us"],
            "erase_32k": common["block_erase_us"],
            "page_program": common["page_program_us"],
        },
        "queue_candidates": {"32": queue_32, "64": queue_64},
        "selected_queue_capacity": selected_capacity,
        "verdict": "GO" if selected["comfortable"] else "NO_GO",
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
    result: dict[str, Any] = {
        "format": "drone-h743-flight-log-flash-timing-analysis",
        "schema": 2 if "suspend_to_ready" in samples else 1,
        "source_capture": capture.get("capture_path"),
        "blocks": {"32k": result_32k, "64k": result_64k},
    }
    if "suspend_to_ready" in samples:
        baseline = capture["baseline_samples_us"]
        cooperative_32k = cooperative_block_throughput(
            block_size=32768,
            block_erase_samples_us=baseline["erase_32k"],
            page_program_samples_us=baseline["page_after_32k"],
            suspend_samples_us=samples["suspend_to_ready"],
            resume_samples_us=samples["resume_to_running"],
            **{key: value for key, value in common.items() if key != "background_wait_ms"},
        )
        cooperative_64k = cooperative_block_throughput(
            block_size=65536,
            block_erase_samples_us=baseline["erase_64k"],
            page_program_samples_us=baseline["page_after_64k"],
            suspend_samples_us=samples["suspend_to_ready"],
            resume_samples_us=samples["resume_to_running"],
            **{key: value for key, value in common.items() if key != "background_wait_ms"},
        )
        cooperative_go = bool(cooperative_32k["go"] or cooperative_64k["go"])
        result["cooperative_blocks"] = {
            "32k": cooperative_32k,
            "64k": cooperative_64k,
        }
        result["verdict"] = "GO" if cooperative_go else "NO_GO"
        result["recommended_block"] = (
            "64k"
            if cooperative_64k["throughput_hz"] >= cooperative_32k["throughput_hz"]
            else "32k"
        ) if cooperative_go else None
    else:
        go = bool(result_32k["go"] or result_64k["go"])
        result["verdict"] = "GO" if go else "NO_GO"
        result["recommended_block"] = (
            "64k" if result_64k["throughput_hz"] >= result_32k["throughput_hz"] else "32k"
        ) if go else None
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("capture", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    capture = json.loads(args.capture.read_text(encoding="utf-8"))
    capture["capture_path"] = str(args.capture)
    report = (
        analyse_queue_peak_capture(capture)
        if "erase_4k" in capture.get("samples_us", {})
        and not capture["samples_us"].get("erase_32k")
        else analyse_capture(capture)
    )
    text = json.dumps(report, indent=2, ensure_ascii=False) + "\n"
    if args.output is None:
        print(text, end="")
    else:
        args.output.write_text(text, encoding="utf-8", newline="\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""R-S5-1 evidence gate for same-recording controller cascade comparison.

The tool deliberately refuses to compare an old/unknown-frame log against the
current controller.  Synthetic vectors belong in unit tests; this report only
becomes a performance comparison when a real V10 log with matching firmware,
frame provenance, and parameter snapshot is supplied.
"""

from __future__ import annotations

import argparse
import json
from datetime import datetime, timezone
from pathlib import Path

try:
    from .flight_log_receive import parse_flash_image
    from .project_paths import ANALYSIS_ROOT, FLIGHT_LOG_DIR
except ImportError:
    from flight_log_receive import parse_flash_image
    from project_paths import ANALYSIS_ROOT, FLIGHT_LOG_DIR


DEFAULT_OUTPUT = ANALYSIS_ROOT / "controller_cascade" / "2026-09-04" / "report.json"


def inspect_candidate(path: Path) -> dict[str, object]:
    sectors, records, errors = parse_flash_image(path.read_bytes())
    versions = sorted({int(sector["version"]) for sector in sectors})
    firmware = sorted(
        {
            f"0x{int(sector.get('firmware_crc32', 0)):08X}"
            for sector in sectors
            if int(sector.get("firmware_crc32", 0)) != 0
        }
    )
    frames = sorted({str(sector.get("attitude_frame")) for sector in sectors})
    return {
        "path": str(path),
        "versions": versions,
        "sector_count": len(sectors),
        "record_count": len(records),
        "parse_error_count": len(errors),
        "firmware_crc32": firmware,
        "frame_provenance": frames,
        "eligible": bool(
            records
            and versions == [10]
            and not errors
            and frames == ["canonical_flu"]
            and firmware
        ),
    }


def build_report(candidates: list[dict[str, object]]) -> dict[str, object]:
    eligible = [candidate for candidate in candidates if candidate["eligible"]]
    if eligible:
        verdict = "INCOMPLETE"
        reason = "matching V10 capture found; numerical replay not yet selected"
        input_path: str | None = str(eligible[0]["path"])
    else:
        verdict = "INCOMPLETE"
        reason = "缺匹配实录，性能待验证"
        input_path = None
    return {
        "format": "drone-h743-controller-cascade-analysis",
        "schema": 1,
        "created_at": datetime.now(timezone.utc).astimezone().isoformat(),
        "verdict": verdict,
        "reason": reason,
        "input_data_path": input_path,
        "current_source_revision": "R-S5-1 working tree",
        "firmware_crc32": None,
        "frame_provenance": None,
        "parameter_snapshot": None,
        "candidates": candidates,
        "comparisons": {
            "old_controller": None,
            "new_cascade": None,
            "new_cascade_id_zero_equivalence": None,
            "a_sp_error": None,
            "omega_sp": None,
            "moment_cmd_error": None,
            "moment_achieved": None,
            "saturation_time_ratio": None,
            "integrator_peak_and_release": None,
        },
        "failure_modes": [
            "V7/V8 candidates predate CFG V19 and FlightLog V10",
            "unknown_legacy_frd cannot be reinterpreted as current controller input",
            "Z-axis migration is not sample-for-sample equivalent because V18 vel_z_kd=0",
        ],
        "cannot_prove": [
            "closed-loop convergence or stability margins",
            "actuator polarity and achieved moment on hardware",
            "tethered or free-flight performance",
            "non-zero Z velocity-P suitability",
        ],
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", action="append", type=Path, default=[])
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    paths = args.input or sorted(FLIGHT_LOG_DIR.rglob("*.bin"))
    candidates = [inspect_candidate(path) for path in paths if path.is_file()]
    report = build_report(candidates)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"{report['verdict']}: {report['reason']}")
    print(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

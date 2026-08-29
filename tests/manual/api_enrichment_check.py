"""Smoke-test the enriched API surface.

Hits each of the 5 commands we updated in this pass, dumps the result so
we can eyeball the new fields. Run against a live Serial Studio with a
project loaded:

    python3 tests/manual/api_enrichment_check.py

Asserts that every result contains the new friendly fields. Use as a
regression guard once we wire it into pytest.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from utils.api_client import SerialStudioClient  # noqa: E402

REQUIRED_FIELDS = {
    "project.source.list": ["_summary"],
    "project.group.list": ["_summary"],
    "project.dataset.list": ["_summary"],
    "io.getStatus": ["_summary", "busTypeLabel", "busTypeSlug"],
    "dashboard.getStatus": ["_summary", "operationModeLabel", "operationModeSlug"],
}


def main() -> int:
    failures = 0

    with SerialStudioClient() as client:
        for command, required in REQUIRED_FIELDS.items():
            print(f"\n=== {command} ===")
            try:
                resp = client.command(command, {})
            except Exception as exc:
                print(f"  ERROR: {exc}")
                failures += 1
                continue

            print(json.dumps(resp, indent=2))

            for field in required:
                if field not in resp:
                    print(f"  MISSING: {field}")
                    failures += 1

            # Per-source / per-group / per-dataset enrichment
            if command == "project.source.list":
                for src in resp.get("sources", []):
                    for k in (
                        "busTypeLabel",
                        "busTypeSlug",
                        "frameDetectionLabel",
                        "decoderMethodLabel",
                    ):
                        if k not in src:
                            print(f"  MISSING in source: {k}")
                            failures += 1

            if command == "project.group.list":
                for grp in resp.get("groups", []):
                    if "datasetCount" not in grp:
                        print("  MISSING in group: datasetCount")
                        failures += 1

            if command == "project.dataset.list":
                for ds in resp.get("datasets", []):
                    for k in ("enabledFeatures", "hasTransform", "isVirtual"):
                        if k not in ds:
                            print(f"  MISSING in dataset: {k}")
                            failures += 1

    if failures:
        print(f"\n{failures} field(s) missing")
        return 1

    print("\nAll enriched fields present.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

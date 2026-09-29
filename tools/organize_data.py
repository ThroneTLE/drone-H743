#!/usr/bin/env python3
"""Move canonical project data into sortable YYYY-MM-DD subdirectories."""

from __future__ import annotations

import argparse
import shutil
from collections import Counter
from dataclasses import dataclass
from pathlib import Path

try:
    from .project_paths import (
        ATTITUDE_IDENT_DIR,
        AIRFRAME_CALIBRATION_DIR,
        DATA_ROOT,
        FLIGHT_LOG_DEBUG_DIR,
        FLIGHT_LOG_DIR,
        IMU_ATTITUDE_CAPTURE_DIR,
        IMU_METROLOGY_CALIBRATION_DIR,
        FLIGHT_ACCEPTANCE_CALIBRATION_DIR,
        IMU_VIBRATION_CAPTURE_DIR,
        LOG_DIR,
        MOTOR_IDENT_DIR,
        SALEAE_PIN_ID_CAPTURE_DIR,
        SALEAE_SPI_CAPTURE_DIR,
        SALEAE_TEST_CAPTURE_DIR,
        THRUST_IDENT_DIR,
        USB_FLIGHT_LOG_CAPTURE_DIR,
        VOFA_CAPTURE_DIR,
        DATE_DIRECTORY_RE,
        date_from_name,
    )
except ImportError:  # Allows running as: python tools/organize_data.py
    try:
        from tools.project_paths import (
            ATTITUDE_IDENT_DIR,
            AIRFRAME_CALIBRATION_DIR,
            DATA_ROOT,
            FLIGHT_LOG_DEBUG_DIR,
            FLIGHT_LOG_DIR,
            IMU_ATTITUDE_CAPTURE_DIR,
            IMU_METROLOGY_CALIBRATION_DIR,
            FLIGHT_ACCEPTANCE_CALIBRATION_DIR,
            IMU_VIBRATION_CAPTURE_DIR,
            LOG_DIR,
            MOTOR_IDENT_DIR,
            SALEAE_PIN_ID_CAPTURE_DIR,
            SALEAE_SPI_CAPTURE_DIR,
            SALEAE_TEST_CAPTURE_DIR,
            THRUST_IDENT_DIR,
            USB_FLIGHT_LOG_CAPTURE_DIR,
            VOFA_CAPTURE_DIR,
            DATE_DIRECTORY_RE,
            date_from_name,
        )
    except ImportError:
        from project_paths import (
            ATTITUDE_IDENT_DIR,
            AIRFRAME_CALIBRATION_DIR,
            DATA_ROOT,
            FLIGHT_LOG_DEBUG_DIR,
            FLIGHT_LOG_DIR,
            IMU_ATTITUDE_CAPTURE_DIR,
            IMU_METROLOGY_CALIBRATION_DIR,
            FLIGHT_ACCEPTANCE_CALIBRATION_DIR,
            IMU_VIBRATION_CAPTURE_DIR,
            LOG_DIR,
            MOTOR_IDENT_DIR,
            SALEAE_PIN_ID_CAPTURE_DIR,
            SALEAE_SPI_CAPTURE_DIR,
            SALEAE_TEST_CAPTURE_DIR,
            THRUST_IDENT_DIR,
            USB_FLIGHT_LOG_CAPTURE_DIR,
            VOFA_CAPTURE_DIR,
            DATE_DIRECTORY_RE,
            date_from_name,
        )


@dataclass(frozen=True)
class CategoryRule:
    root: Path
    reserved_names: frozenset[str] = frozenset()
    static_names: frozenset[str] = frozenset()


@dataclass(frozen=True)
class MovePlan:
    source: Path
    target: Path


CATEGORY_RULES = (
    CategoryRule(FLIGHT_LOG_DIR, frozenset({"debug_exports", "legacy"})),
    CategoryRule(FLIGHT_LOG_DEBUG_DIR),
    CategoryRule(FLIGHT_LOG_DIR / "legacy"),
    CategoryRule(VOFA_CAPTURE_DIR, frozenset({"analysis"})),
    CategoryRule(VOFA_CAPTURE_DIR / "analysis"),
    CategoryRule(USB_FLIGHT_LOG_CAPTURE_DIR),
    CategoryRule(IMU_ATTITUDE_CAPTURE_DIR, static_names=frozenset({"README.md"})),
    CategoryRule(IMU_VIBRATION_CAPTURE_DIR),
    CategoryRule(SALEAE_SPI_CAPTURE_DIR),
    CategoryRule(SALEAE_PIN_ID_CAPTURE_DIR),
    CategoryRule(SALEAE_TEST_CAPTURE_DIR),
    # Rod-rig sysid page state: rig geometry / throttle settings persisted across sessions.
    CategoryRule(ATTITUDE_IDENT_DIR, static_names=frozenset({"rig_settings.json"})),
    CategoryRule(MOTOR_IDENT_DIR),
    # Thrust-bench tool files: experiment library, ESC KV record (post-KV data cut-off), fitted models.
    CategoryRule(THRUST_IDENT_DIR, frozenset({"models"}),
                 frozenset({"experiments.sqlite3", "esc_config.json"})),
    CategoryRule(AIRFRAME_CALIBRATION_DIR, static_names=frozenset({"README.md"})),
    CategoryRule(IMU_METROLOGY_CALIBRATION_DIR, static_names=frozenset({"README.md"})),
    CategoryRule(FLIGHT_ACCEPTANCE_CALIBRATION_DIR, static_names=frozenset({"README.md"})),
    CategoryRule(LOG_DIR / "legacy"),
)


def plan_category(rule: CategoryRule) -> list[MovePlan]:
    if not rule.root.is_dir():
        return []
    plans: list[MovePlan] = []
    for source in sorted(rule.root.iterdir(), key=lambda path: path.name.casefold()):
        if source.name in rule.reserved_names or source.name in rule.static_names:
            continue
        if source.name == "undated" or DATE_DIRECTORY_RE.fullmatch(source.name):
            continue
        captured_date = date_from_name(source.name)
        bucket = captured_date.isoformat() if captured_date is not None else "undated"
        plans.append(MovePlan(source, rule.root / bucket / source.name))
    return plans


def build_plan(rules: tuple[CategoryRule, ...] = CATEGORY_RULES) -> list[MovePlan]:
    return [plan for rule in rules for plan in plan_category(rule)]


def _is_within(path: Path, root: Path) -> bool:
    try:
        path.resolve().relative_to(root.resolve())
    except ValueError:
        return False
    return True


def apply_plan(plans: list[MovePlan], allowed_root: Path = DATA_ROOT) -> None:
    for plan in plans:
        if not _is_within(plan.source, allowed_root) or not _is_within(plan.target, allowed_root):
            raise ValueError(f"move escapes data root: {plan.source} -> {plan.target}")
        if not plan.source.exists():
            raise FileNotFoundError(plan.source)
        if plan.target.exists():
            raise FileExistsError(plan.target)
    for plan in plans:
        plan.target.parent.mkdir(parents=True, exist_ok=True)
        shutil.move(str(plan.source), str(plan.target))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--apply", action="store_true", help="perform moves; default is a dry run")
    parser.add_argument("--verbose", action="store_true", help="list every planned move")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    plans = build_plan()
    if not plans:
        print("Data directories are already organized.")
        return 0
    if args.verbose:
        for plan in plans:
            print(f"{plan.source.relative_to(DATA_ROOT)} -> {plan.target.relative_to(DATA_ROOT)}")
    counts = Counter(str(plan.target.parent.relative_to(DATA_ROOT)) for plan in plans)
    for bucket, count in sorted(counts.items()):
        print(f"{bucket}: {count} item(s)")
    if not args.apply:
        print(f"Dry run: {len(plans)} moves. Re-run with --apply.")
        return 0
    apply_plan(plans)
    print(f"Applied {len(plans)} moves.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

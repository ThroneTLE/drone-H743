#!/usr/bin/env python3
"""Icon-tree migration driver for spec 0028 (centralized icon registry).

Audit mode scans app/rcc/icons/** (minus the exempt `buttons/` set), groups the
files into logical icons, and writes the migration manifest
doc/claude/specs/0028-icon-registry/icon-map.csv mapping every file to its
`<category>/<tier>/<name>.svg` target (tier set {16, 24, 32, 48}), with
byte-duplicate drops and review flags. The CSV is the maintainer sign-off
artifact for spec Q2/Q3; nothing is moved until the (separate) apply mode runs
against a signed-off manifest. Output is deterministic: re-running audit on an
unchanged tree is byte-identical.

Apply mode executes a signed-off manifest exactly once: moves every kept file to
its target, deletes the byte-duplicate drops, writes the placeholder icon, and
rewrites the rcc.qrc icon block as real entries plus one compat alias per old
path (so every pre-migration `qrc:/icons/...` URL keeps resolving until the
consumers migrate). It refuses to run unless the on-disk tree still matches the
manifest byte-for-byte and the manifest is conflict-free. Audit mode only works
on the pre-migration layout; after apply, `registry-verify.py` owns the tree.

Usage:
    python3 scripts/icon-migrate.py audit [--csv PATH]
    python3 scripts/icon-migrate.py apply [--csv PATH]
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import re
import sys
from collections import defaultdict
from dataclasses import dataclass, field
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
ICONS = ROOT / "app" / "rcc" / "icons"
QRC = ROOT / "app" / "rcc" / "rcc.qrc"
DEFAULT_CSV = ROOT / "doc" / "claude" / "specs" / "0028-icon-registry" / "icon-map.csv"

EXEMPT_FOLDERS = {"buttons"}

# Source top-level folder -> registry category (plan.md taxonomy, spec Q3).
FOLDER_CATEGORY = {
    "toolbar": "commands",
    "start": "commands",
    "taskbar": "commands",
    "dashboard-buttons": "commands",
    "dashboard-small": "widgets",
    "dashboard-large": "widgets",
    "csd": "window",
    "miniwindow": "window",
    "devices": "devices",
    "project-editor": "editor",
    "panes": "panes",
    "console": "console",
    "database": "database",
    "code-editor": "code",
    "licensing": "licensing",
    "notifications": "notifications",
}

# Byte-duplicate canonical-home precedence: code-bound categories keep their glyphs
# (C++ resolves widgets/window/editor names from fixed category strings); data-bound
# commands manifests can reference any category, so commands cede dups to everyone.
CATEGORY_PRIORITY = [
    "widgets",
    "window",
    "editor",
    "devices",
    "panes",
    "console",
    "database",
    "code",
    "licensing",
    "notifications",
    "commands",
]

# Folders whose name pins the display tier regardless of viewBox spread.
FOLDER_TIER_HINT = {"dashboard-small": 16, "dashboard-large": 32}

# Maintainer-approved renames (2026-07-21): each of these carries different path
# geometry from the same-named file in project-editor/actions/, so both glyphs
# survive under distinct logical names (no visual merge without sign-off, R5).
NAME_OVERRIDES = {
    "project-editor/treeview/output-button.svg": "output-button-alt",
    "project-editor/treeview/output-knob.svg": "output-knob-alt",
    "project-editor/treeview/output-slider.svg": "output-slider-alt",
    "project-editor/treeview/output-textfield.svg": "output-textfield-alt",
    "project-editor/treeview/output-toggle.svg": "output-toggle-alt",
    "project-editor/treeview/shared-table.svg": "shared-table-alt",
}

TIERS = (16, 24, 32, 48)

# viewBox widths observed in the Office-pack exports, keyed by their tier.
KNOWN_CLUSTERS = {
    16: (12.0, 13.5, 14.0, 16.0),
    24: (18.0, 22.5, 24.0),
    32: (30.0, 36.0, 37.5, 40.0),
    48: (48.0, 60.0, 64.0, 72.0),
}

VIEWBOX_RE = re.compile(
    r"viewBox\s*=\s*[\"']\s*([\d.\-]+)[,\s]+([\d.\-]+)[,\s]+([\d.\-]+)[,\s]+([\d.\-]+)"
)

CSV_COLUMNS = [
    "old_path",
    "category",
    "tier",
    "name",
    "target_path",
    "action",
    "md5_8",
    "viewbox_w",
    "group",
    "flags",
]


@dataclass
class IconFile:
    rel: str
    category: str
    name: str
    norm: str
    size_hint: int
    md5: str
    width: float
    tier: int = 0
    action: str = "move"
    target: str = ""
    flags: list[str] = field(default_factory=list)


def tier_for_width(width: float) -> int:
    if width <= 17.0:
        return 16
    if width <= 25.5:
        return 24
    if width <= 42.0:
        return 32
    return 48


def in_known_cluster(width: float) -> bool:
    return any(abs(width - w) < 0.6 for ws in KNOWN_CLUSTERS.values() for w in ws)


def normalize_name(stem: str) -> tuple[str, int]:
    """Strip -small/-large size suffixes, kebab-normalize; return (name, size hint)."""
    stem = stem.replace("_", "-")
    if stem.endswith("-small"):
        return stem[: -len("-small")], -1
    if stem.endswith("-large"):
        return stem[: -len("-large")], 1
    return stem, 0


def scan_tree() -> tuple[list[IconFile], int]:
    files: list[IconFile] = []
    exempt = 0
    for path in sorted(ICONS.rglob("*.svg")):
        rel = path.relative_to(ICONS).as_posix()
        top = rel.split("/", 1)[0]
        if top in EXEMPT_FOLDERS:
            exempt += 1
            continue
        if top not in FOLDER_CATEGORY:
            sys.exit(
                f"icon-migrate: unmapped source folder '{top}' ({rel}) -- "
                f"extend FOLDER_CATEGORY before auditing"
            )
        data = path.read_bytes()
        match = VIEWBOX_RE.search(data[:2048].decode("utf-8", errors="replace"))
        width = float(match.group(3)) if match else 0.0
        norm, hint = normalize_name(path.stem)
        if rel in NAME_OVERRIDES:
            norm, hint = NAME_OVERRIDES[rel], 0
        icon = IconFile(
            rel=rel,
            category=FOLDER_CATEGORY[top],
            name=path.stem,
            norm=norm,
            size_hint=hint,
            md5=hashlib.md5(data).hexdigest(),
            width=width,
        )
        if match is None:
            icon.flags.append("no-viewbox")
        elif not in_known_cluster(width):
            icon.flags.append("between-tier")
        files.append(icon)
    return files, exempt


def dedupe(files: list[IconFile]) -> int:
    """Mark byte-identical copies as drop-dup; return duplicate group count."""
    by_md5: dict[str, list[IconFile]] = defaultdict(list)
    for icon in files:
        by_md5[icon.md5].append(icon)
    dup_groups = [m for m in by_md5.values() if len(m) > 1]
    for members in dup_groups:
        members.sort(
            key=lambda i: (CATEGORY_PRIORITY.index(i.category), len(i.name), i.rel)
        )
        canonical = members[0]
        for extra in members[1:]:
            extra.action = "drop-dup"
            if extra.norm != canonical.norm:
                extra.flags.append("renamed")
            if extra.category != canonical.category:
                extra.flags.append("cross-category")
            extra.category = canonical.category
            extra.norm = canonical.norm
            extra.size_hint = canonical.size_hint
    return len(dup_groups)


def assign_tiers(files: list[IconFile]) -> None:
    """Give every kept file a tier slot inside its (category, name) group."""
    groups: dict[tuple[str, str], list[IconFile]] = defaultdict(list)
    for icon in files:
        if icon.action == "move":
            groups[(icon.category, icon.norm)].append(icon)
    for members in groups.values():
        members.sort(key=lambda i: (i.width, i.size_hint, i.rel))
        taken: dict[int, IconFile] = {}
        for icon in members:
            top = icon.rel.split("/", 1)[0]
            tier = FOLDER_TIER_HINT.get(top, tier_for_width(icon.width))
            if icon.size_hint < 0:
                tier = min(tier, 16)
            if tier in taken:
                free = [t for t in TIERS if t not in taken and t > tier]
                if free and abs(free[0] - icon.width) <= free[0]:
                    tier = free[0]
                else:
                    icon.flags.append("tier-conflict")
            icon.tier = tier
            if "tier-conflict" not in icon.flags:
                taken[tier] = icon
            icon.target = f"{icon.category}/{icon.tier}/{icon.norm}.svg"
    for icon in files:
        if icon.action == "drop-dup":
            twin = next(f for f in files if f.action == "move" and f.md5 == icon.md5)
            icon.tier = twin.tier
            icon.target = twin.target


def qrc_registered_paths() -> set[str]:
    text = QRC.read_text(encoding="utf-8")
    return set(re.findall(r"<file[^>]*>icons/([^<]+)</file>", text))


PLACEHOLDER_REL = "system/16/missing.svg"

PLACEHOLDER_SVG = (
    '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 16 16">\n'
    '  <rect x="1.5" y="1.5" width="13" height="13" fill="none" stroke="#e91e63"\n'
    '        stroke-width="1.5" stroke-dasharray="2.5 1.5"/>\n'
    '  <path d="M4.5 4.5 L11.5 11.5 M11.5 4.5 L4.5 11.5" stroke="#e91e63"\n'
    '        stroke-width="1.5" fill="none"/>\n'
    "</svg>\n"
)


def load_manifest(csv_path: Path) -> list[dict[str, str]]:
    with csv_path.open("r", encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))
    if not rows or set(rows[0]) != set(CSV_COLUMNS):
        sys.exit(f"icon-migrate: {csv_path} missing or malformed -- run audit first")
    return rows


def refuse(reason: str) -> None:
    sys.exit(f"icon-migrate: REFUSING to apply -- {reason}")


def apply_manifest(csv_path: Path) -> None:
    rows = load_manifest(csv_path)
    targets: dict[str, list[dict[str, str]]] = defaultdict(list)
    for row in rows:
        if row["flags"] and "tier-conflict" in row["flags"]:
            refuse(f"unresolved tier-conflict on {row['old_path']}")
        if row["action"] == "move":
            targets[row["target_path"]].append(row)
    for target, movers in sorted(targets.items()):
        if len(movers) > 1:
            refuse(f"target collision on {target}")
    disk = {
        p.relative_to(ICONS).as_posix()
        for p in ICONS.rglob("*.svg")
        if p.relative_to(ICONS).as_posix().split("/", 1)[0] not in EXEMPT_FOLDERS
    }
    manifest_paths = {row["old_path"].removeprefix("icons/") for row in rows}
    if disk != manifest_paths:
        extra = sorted(disk - manifest_paths)[:3]
        missing = sorted(manifest_paths - disk)[:3]
        refuse(f"tree drifted since audit (extra={extra}, missing={missing})")
    for row in rows:
        data = (ICONS / row["old_path"].removeprefix("icons/")).read_bytes()
        if hashlib.md5(data).hexdigest()[:8] != row["md5_8"]:
            refuse(f"content drifted since audit: {row['old_path']}")

    moved = dropped = 0
    for row in rows:
        src = ICONS / row["old_path"].removeprefix("icons/")
        if row["action"] == "move":
            dst = ICONS / row["target_path"].removeprefix("icons/")
            dst.parent.mkdir(parents=True, exist_ok=True)
            src.rename(dst)
            moved += 1
        else:
            src.unlink()
            dropped += 1
    for folder in sorted((p for p in ICONS.rglob("*") if p.is_dir()), reverse=True):
        if folder.name not in EXEMPT_FOLDERS and not any(folder.iterdir()):
            folder.rmdir()
    placeholder = ICONS / PLACEHOLDER_REL
    placeholder.parent.mkdir(parents=True, exist_ok=True)
    placeholder.write_text(PLACEHOLDER_SVG, encoding="utf-8", newline="\n")

    rewrite_qrc(rows)
    print(f"applied            : {moved} moved, {dropped} dropped, placeholder written")
    print(f"qrc                : real entries + {len(rows)} compat aliases")


def rewrite_qrc(rows: list[dict[str, str]]) -> None:
    lines = QRC.read_text(encoding="utf-8").splitlines(keepends=True)
    kept: list[str] = []
    insert_at = -1
    for line in lines:
        if "<file>icons/" in line and "<file>icons/buttons/" not in line:
            if insert_at < 0:
                insert_at = len(kept)
            continue
        kept.append(line)
    if insert_at < 0:
        refuse("no icon entries found in rcc.qrc")
    real = sorted({row["target_path"] for row in rows} | {f"icons/{PLACEHOLDER_REL}"})
    block = [f"        <file>{path}</file>\n" for path in real]
    block += [
        f'        <file alias="{row["old_path"]}">{row["target_path"]}</file>\n'
        for row in sorted(rows, key=lambda r: r["old_path"])
    ]
    QRC.write_text(
        "".join(kept[:insert_at] + block + kept[insert_at:]),
        encoding="utf-8",
        newline="",
    )


def write_csv(files: list[IconFile], csv_path: Path) -> None:
    files = sorted(files, key=lambda i: (i.category, i.norm, i.tier, i.rel))
    with csv_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle, lineterminator="\n")
        writer.writerow(CSV_COLUMNS)
        for icon in files:
            writer.writerow(
                [
                    f"icons/{icon.rel}",
                    icon.category,
                    icon.tier,
                    icon.norm,
                    f"icons/{icon.target}",
                    icon.action,
                    icon.md5[:8],
                    f"{icon.width:g}",
                    f"{icon.category}/{icon.norm}",
                    ";".join(sorted(icon.flags)),
                ]
            )


def summarize(files: list[IconFile], exempt: int, dup_groups: int) -> None:
    moves = [i for i in files if i.action == "move"]
    drops = [i for i in files if i.action == "drop-dup"]
    logical = {(i.category, i.norm) for i in moves}
    registered = qrc_registered_paths()
    unregistered = [i.rel for i in files if i.rel not in registered]
    targets: dict[str, int] = defaultdict(int)
    for icon in moves:
        targets[icon.target] += 1
    collisions = {t: n for t, n in targets.items() if n > 1}

    print(f"scanned            : {len(files)} SVGs (+{exempt} exempt in buttons/)")
    print(f"byte-dup groups    : {dup_groups} ({len(drops)} files dropped)")
    print(f"kept files         : {len(moves)}")
    print(f"logical icons      : {len(logical)}")
    per_cat: dict[str, set[str]] = defaultdict(set)
    per_tier: dict[int, int] = defaultdict(int)
    for icon in moves:
        per_cat[icon.category].add(icon.norm)
        per_tier[icon.tier] += 1
    print(
        "per category       : "
        + ", ".join(f"{c}={len(per_cat[c])}" for c in CATEGORY_PRIORITY if per_cat[c])
    )
    print(
        "per tier (files)   : "
        + ", ".join(f"{t}px={per_tier[t]}" for t in TIERS if per_tier[t])
    )
    flag_counts: dict[str, int] = defaultdict(int)
    for icon in files:
        for flag in icon.flags:
            flag_counts[flag] += 1
    print(
        "flags              : "
        + (", ".join(f"{k}={v}" for k, v in sorted(flag_counts.items())) or "none")
    )
    if unregistered:
        print(f"NOT in rcc.qrc     : {len(unregistered)} (first: {unregistered[0]})")
    if collisions:
        print(f"TARGET COLLISIONS  : {len(collisions)} -- apply mode will refuse")
        for target, count in sorted(collisions.items()):
            print(f"  {target} x{count}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=["audit", "apply"])
    parser.add_argument("--csv", type=Path, default=DEFAULT_CSV)
    args = parser.parse_args()

    if not ICONS.is_dir():
        sys.exit(f"icon-migrate: missing icon tree {ICONS}")
    if args.mode == "apply":
        apply_manifest(args.csv)
        return 0
    files, exempt = scan_tree()
    dup_groups = dedupe(files)
    assign_tiers(files)
    write_csv(files, args.csv)
    summarize(files, exempt, dup_groups)
    print(f"manifest written   : {args.csv.relative_to(ROOT).as_posix()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

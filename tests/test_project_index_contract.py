from __future__ import annotations

import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SKILL = ROOT / ".agents" / "skills" / "drone-h743-project"
GENERATOR = SKILL / "scripts" / "update_repository_index.py"
INDEX_DIR = SKILL / "references" / "repository-index"


def test_repository_index_is_current() -> None:
    result = subprocess.run(
        [sys.executable, str(GENERATOR), "--check"],
        cwd=ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=30,
    )
    assert result.returncode == 0, result.stdout


def test_repository_index_has_hard_context_limits() -> None:
    root_size = (INDEX_DIR / "README.md").stat().st_size
    shard_sizes = [path.stat().st_size for path in INDEX_DIR.glob("*.md") if path.name != "README.md"]

    assert root_size <= 8 * 1024
    assert shard_sizes
    assert max(shard_sizes) <= 32 * 1024
    assert root_size + sum(shard_sizes) <= 96 * 1024


def test_large_vendor_and_data_trees_are_aggregated() -> None:
    combined = "\n".join(path.read_text(encoding="utf-8") for path in INDEX_DIR.glob("*.md"))

    assert "`driver_doc/`" in combined
    assert "`data/`" in combined
    assert "stm32h7xx_hal_gpio.c" not in combined

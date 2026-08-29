from __future__ import annotations

import re
from datetime import date
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PIPELINE = ROOT / "PIPELINE.md"
SKILL = ROOT / ".agents" / "skills" / "drone-h743-project" / "SKILL.md"
FRAME_HEADER = ROOT / "Driver" / "Inc" / "drv_frame_contract.h"

STATUS_EMOJIS = ("✅", "\U0001f7e1", "⏸")  # ✅ 完成 / 🟡 未完成 / ⏸ 冻结
DONE, INCOMPLETE, FROZEN = STATUS_EMOJIS
MAINLINE_NODES = tuple(f"M{i}" for i in range(8))


def pipeline_text() -> str:
    return PIPELINE.read_text(encoding="utf-8")


def section(text: str, heading: str) -> str:
    match = re.search(rf"^## {re.escape(heading)}\s*$(.*?)(?=^## |\Z)", text, re.MULTILINE | re.DOTALL)
    assert match is not None, f"PIPELINE.md missing section: {heading}"
    return match.group(1)


def mermaid_nodes(text: str) -> dict[str, str]:
    """Map node id -> status emoji for every node defined in the mermaid diagram."""
    diagram_match = re.search(r"```mermaid\n(.*?)```", text, re.DOTALL)
    assert diagram_match is not None, "PIPELINE.md missing mermaid diagram"
    nodes: dict[str, str] = {}
    for node_id, label in re.findall(r"^\s*([BMS]\d)\[\"(.+?)\"\]", diagram_match.group(1), re.MULTILINE):
        assert node_id not in nodes, f"duplicate node definition: {node_id}"
        status = next((emoji for emoji in STATUS_EMOJIS if label.startswith(emoji)), None)
        assert status is not None, f"node {node_id} label must start with ✅/🟡/⏸: {label!r}"
        nodes[node_id] = status
    return nodes


def macro_uint(name: str) -> int:
    source = FRAME_HEADER.read_text(encoding="utf-8")
    match = re.search(rf"^#define\s+{re.escape(name)}\s+(0x[0-9A-Fa-f]+|\d+)U\b", source, re.MULTILINE)
    assert match is not None, f"missing integer contract macro {name}"
    return int(match.group(1), 0)


def test_pipeline_exists_with_required_sections() -> None:
    text = pipeline_text()
    for heading in ("状态定义", "主线、副线与当前状态", "主线验收门", "最近验证证据", "推进与更新规则"):
        section(text, heading)
    assert re.search(r"最后更新：\d{4}-\d{2}-\d{2}", text), "missing 最后更新 date"


def test_mainline_nodes_match_between_diagram_and_gate_table() -> None:
    text = pipeline_text()
    diagram_mainline = {node for node in mermaid_nodes(text) if node.startswith("M")}
    gate_rows = set(re.findall(r"^\|\s*(M\d)\s", section(text, "主线验收门"), re.MULTILINE))
    assert diagram_mainline == set(MAINLINE_NODES), f"diagram mainline nodes: {sorted(diagram_mainline)}"
    assert gate_rows == set(MAINLINE_NODES), f"gate table rows: {sorted(gate_rows)}"


def test_current_position_points_at_an_incomplete_mainline_node() -> None:
    text = pipeline_text()
    pointer = re.search(r"当前主线位置：\*\*(M\d)", text)
    assert pointer is not None, "missing 当前主线位置 pointer"
    node = pointer.group(1)
    status = mermaid_nodes(text)[node]
    assert status == INCOMPLETE, (
        f"current mainline node {node} must be 🟡; a completed node requires moving the pointer forward "
        f"in the same update (rule 8), and the pointer can never rest on a frozen node"
    )


def test_flight_node_stays_frozen_until_runtime_flu_migration_completes() -> None:
    required = macro_uint("DRV_FRAME_RUNTIME_MIGRATION_REQUIRED_MASK")
    done = macro_uint("DRV_FRAME_RUNTIME_MIGRATION_DONE_MASK")
    if (done & required) != required:
        status = mermaid_nodes(pipeline_text())["M7"]
        assert status == FROZEN, (
            "runtime FLU migration is incomplete in drv_frame_contract.h, so M7 (props-on flight) "
            "must stay ⏸ in PIPELINE.md"
        )


def test_evidence_table_rows_are_dated_and_not_ahead_of_last_update() -> None:
    text = pipeline_text()
    evidence = section(text, "最近验证证据")
    dates = [date.fromisoformat(d) for d in re.findall(r"^\|\s*(\d{4}-\d{2}-\d{2})\s*\|", evidence, re.MULTILINE)]
    assert dates, "evidence table must contain at least one dated row"
    last_updated = date.fromisoformat(re.search(r"最后更新：(\d{4}-\d{2}-\d{2})", text).group(1))
    assert max(dates) <= last_updated, "最后更新 date must be at least as recent as the newest evidence row"
    assert max(dates) <= date.today(), "evidence rows must not carry future dates"


def test_skill_routes_project_tasks_through_pipeline_governance() -> None:
    skill = SKILL.read_text(encoding="utf-8")
    assert "Pipeline Governance" in skill
    assert "PIPELINE.md" in skill
    assert "test_pipeline_contract.py" in skill

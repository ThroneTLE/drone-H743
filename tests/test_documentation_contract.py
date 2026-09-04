"""Current documentation stays small, routed, and distinct from history."""

import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]

CURRENT_DOCS = (
    "README.md",
    "AGENTS.md",
    "PIPELINE.md",
    "doc/current-architecture.md",
    "doc/hardware-reference.md",
    "doc/technical-spec.md",
    "doc/telemetry-protocol.md",
    "tools/README.md",
)

REMOVED_DOCS = (
    "doc/architecture.md",
    "doc/controller_walkthrough.md",
    "doc/m1-baseline-runbook.md",
    "doc/telemetry-scope-plan.md",
    ".agents/skills/drone-h743-project/references/progress-notes.md",
    "tools/ground_station/ROADMAP.md",
)

HISTORY_DOCS = (
    "doc/history/identification-and-tether-geometry-2026-07-25.md",
    "doc/history/nonlinear-balance-controller-2026-07-25.md",
)


def read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def test_current_document_map_has_one_small_human_entry() -> None:
    for relative in CURRENT_DOCS:
        assert (ROOT / relative).is_file(), relative
    entry = read("README.md")
    for relative in CURRENT_DOCS[2:]:
        assert relative in entry or relative == "tools/README.md"
    assert len(entry.encode("utf-8")) < 4096


def test_retired_guides_do_not_return() -> None:
    for relative in REMOVED_DOCS:
        assert not (ROOT / relative).exists(), relative


def test_historical_measurements_force_a_user_choice() -> None:
    rules = read("doc/history/README.md")
    assert "必须先询问用户" in rules
    assert "日期、数据集、固件提交和硬件配置" in rules
    for relative in HISTORY_DOCS:
        text = read(relative)[:1000]
        assert "历史" in text, relative
        assert "必须先问用户" in text, relative
        assert "数据集" in text and "固件提交" in text, relative
    data_rules = read("data/README.md")
    assert "ask the user which date, dataset, firmware commit" in data_rules
    assert "Do not silently choose the newest file" in data_rules


def test_current_architecture_does_not_restore_the_old_blueprint() -> None:
    architecture = read("doc/current-architecture.md")
    assert "App：任务" in architecture
    assert "Services：同步" in architecture
    assert "svc_flow_nav" in architecture
    assert "app_telem_stream" in architecture
    for obsolete in ("Middleware/", "mw_mavlink", "app_controller.h", "app_ekf.h"):
        assert obsolete not in architecture


def test_telemetry_document_is_current_and_old_plan_is_unreferenced() -> None:
    protocol = read("doc/telemetry-protocol.md")
    assert "Schema v3" in protocol
    assert "frame=body_flu contract=1" in protocol
    assert "APP_PROTO_MSG_TELEM_FRAME = 0x2230" in protocol
    checked = (
        read("AGENTS.md")
        + read("doc/technical-spec.md")
        + read("tools/README.md")
        + read("tools/ground_station/README.md")
        + read(".agents/skills/drone-h743-project/SKILL.md")
        + read(".agents/skills/drone-h743-project/references/modes/protocol-telemetry.md")
    )
    assert "telemetry-scope-plan.md" not in checked
    assert "progress-notes.md" not in checked


def test_agent_entry_avoids_loading_history_and_volatile_counts() -> None:
    agents = read("AGENTS.md")
    skill = read(".agents/skills/drone-h743-project/SKILL.md")
    assert "不要为普通任务加载整份历史证据表" in agents
    assert "Do not load the complete historical evidence table" in skill
    for stale in ("5449 行", "4131 行", "基线 782+"):
        assert stale not in agents
    spec = read("doc/technical-spec.md")
    assert "当前基线 696+" not in spec
    assert "data/calibration/** 全部历史证据" in spec


def test_current_tool_and_req_guides_do_not_point_to_retired_plans() -> None:
    tools = read("tools/README.md")
    assert "tools/attitude_ident_pid.py" in tools
    assert (ROOT / "tools/attitude_ident_pid.py").is_file()
    assert "tools/ident_analysis.py" not in tools

    req_table = read("PIPELINE.md").split("## 最近验证证据", 1)[0]
    assert "telemetry-scope-plan.md" not in req_table
    assert "规划 §" not in req_table
    assert "§11.1" not in req_table
    assert "R-F6" in req_table
    assert "不是运行时迁移完成" in req_table


def test_worktree_spec_is_not_presented_as_hardware_acceptance() -> None:
    protocol = read("doc/telemetry-protocol.md")
    tools = read("tools/README.md")
    root = read("README.md")
    assert "当前工作树的线上规范" in protocol
    assert "本文存在不等于相应固件已经烧录或验收" in protocol
    assert "哪些版本已经烧录或审核，以 `PIPELINE.md` 为准" in tools
    assert "默认启用自动连接" in tools
    assert "未经授权不要启动" in root


def test_current_markdown_links_resolve() -> None:
    checked = [ROOT / relative for relative in CURRENT_DOCS]
    checked.extend(
        [
            ROOT / "doc/history/README.md",
            ROOT / "tools/ground_station/README.md",
            ROOT / ".agents/skills/drone-h743-project/SKILL.md",
            ROOT / ".agents/skills/drone-h743-project/references/flash-architecture.md",
            ROOT / ".agents/skills/drone-h743-project/references/modes/protocol-telemetry.md",
        ]
    )
    for document in checked:
        for target in re.findall(r"\[[^]]+\]\(([^)]+)\)", document.read_text("utf-8")):
            if "://" in target or target.startswith("#"):
                continue
            path_text = target.split("#", 1)[0]
            assert (document.parent / path_text).resolve().exists(), (
                f"{document.relative_to(ROOT)} -> {target}"
            )

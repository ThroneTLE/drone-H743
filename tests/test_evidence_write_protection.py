"""历史验收证据不可变性契约。

2026-08-29 实际发生过一次证据破坏：面板启动时自动加载了 2026-08-28 的历史
验收会话，随后自动保存把 workflow.json 的 target_state_at_save（14 个键的
归档快照）用当时为空的实时状态覆盖成 {}。本文件锁死修复后的契约：
只读浏览历史验收时，两个自动保存入口都必须直接拒绝写盘。
"""

from __future__ import annotations

import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PANEL = ROOT / "tools" / "drone_tcp_panel.py"


def method_source(source: str, name: str) -> str:
    match = re.search(
        rf"^    def {re.escape(name)}\(.*?(?=^    def |\Z)",
        source,
        re.MULTILINE | re.DOTALL,
    )
    assert match is not None, f"panel method missing: {name}"
    return match.group(0)


def test_session_autosave_refuses_to_write_while_browsing_history() -> None:
    source = PANEL.read_text(encoding="utf-8")
    body = method_source(source, "_validation_autosave_session")
    guard = body.find("if self.validation_loaded_history:")
    write = body.find("write_session(")
    assert guard != -1, "session autosave lost its loaded-history guard"
    assert write != -1
    assert guard < write, "loaded-history guard must run before any session write"


def test_workflow_autosave_refuses_to_write_while_browsing_history() -> None:
    source = PANEL.read_text(encoding="utf-8")
    body = method_source(source, "_validation_autosave_workflow")
    guard = body.find("if self.validation_loaded_history:")
    write = body.find("temporary.replace(path)")
    assert guard != -1, "workflow autosave lost its loaded-history guard"
    assert write != -1
    assert guard < write, "loaded-history guard must run before the workflow file replace"


def test_provenance_flag_lifecycle_backs_the_guard() -> None:
    """守卫依赖的出处标志：加载置位；新建/继续显式清零后才恢复保存资格。"""
    source = PANEL.read_text(encoding="utf-8")
    apply_loaded = method_source(source, "_validation_apply_loaded_session")
    assert "self.validation_loaded_history = True" in apply_loaded
    for reactivation in ("_validation_start_session", "_validation_resume_session"):
        body = method_source(source, reactivation)
        assert "self.validation_loaded_history = False" in body, (
            f"{reactivation} must explicitly clear the loaded-history flag before saving resumes"
        )


def test_ui_promise_matches_the_enforced_contract() -> None:
    """对话框对用户的承诺文本仍在；守卫就是这两句承诺的实现。"""
    source = PANEL.read_text(encoding="utf-8")
    assert "不会覆盖或删除历史报告" in source
    assert "不会删除任何历史文件" in source

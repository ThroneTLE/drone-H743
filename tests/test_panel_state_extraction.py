"""S6 panel-state persistence extraction ownership and compatibility contract."""

from __future__ import annotations

import ast
import json
from pathlib import Path
from types import SimpleNamespace

from tools import drone_tcp_panel as legacy_panel
from tools import project_paths
from tools.panel_lib import state


ROOT = Path(__file__).resolve().parents[1]
LEGACY_PANEL_PATH = ROOT / "tools" / "drone_tcp_panel.py"
STATE_PATH = ROOT / "tools" / "panel_lib" / "state.py"

MOVED_FUNCTIONS = {"append_log", "record_panel_crash"}
MOVED_METHODS = {"_load_panel_state", "_save_panel_state"}
OWNED_LOG_PATHS = {"PANEL_CRASH_LOG", "RC_WIZARD_TRACE_LOG"}


def parsed(path: Path) -> ast.Module:
    return ast.parse(path.read_text(encoding="utf-8"))


def top_level_definitions(path: Path) -> set[str]:
    return {
        node.name
        for node in parsed(path).body
        if isinstance(node, (ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef))
    }


def class_methods(path: Path, class_name: str) -> set[str]:
    owner = next(
        node
        for node in parsed(path).body
        if isinstance(node, ast.ClassDef) and node.name == class_name
    )
    return {
        node.name
        for node in owner.body
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
    }


def owned_path_assignments(path: Path) -> set[str]:
    result: set[str] = set()
    for node in parsed(path).body:
        if not isinstance(node, ast.Assign) or isinstance(node.value, ast.Attribute):
            continue
        result.update(
            target.id
            for target in node.targets
            if isinstance(target, ast.Name) and target.id in OWNED_LOG_PATHS
        )
    return result


def test_state_module_owns_persistence_and_logging_implementation() -> None:
    assert MOVED_FUNCTIONS <= top_level_definitions(STATE_PATH)
    assert MOVED_FUNCTIONS.isdisjoint(top_level_definitions(LEGACY_PANEL_PATH))
    assert MOVED_METHODS <= class_methods(STATE_PATH, "PanelStateMixin")
    assert MOVED_METHODS.isdisjoint(class_methods(LEGACY_PANEL_PATH, "DronePanel"))
    assert owned_path_assignments(STATE_PATH) == OWNED_LOG_PATHS
    assert not owned_path_assignments(LEGACY_PANEL_PATH)


def test_legacy_panel_forwards_every_state_symbol_without_wrappers() -> None:
    for name in state.__all__:
        assert getattr(legacy_panel, name) is getattr(state, name), name

    assert legacy_panel.DronePanel._load_panel_state is state.PanelStateMixin._load_panel_state
    assert legacy_panel.DronePanel._save_panel_state is state.PanelStateMixin._save_panel_state
    assert state.PANEL_STATE_PATH is project_paths.PANEL_STATE_PATH
    assert state.LOG_DIR is project_paths.LOG_DIR


class Value:
    def __init__(self, value: object) -> None:
        self.value = value

    def get(self) -> object:
        return self.value


def test_panel_state_round_trip_uses_only_the_configured_state_path(
    tmp_path: Path, monkeypatch,
) -> None:
    panel_state_path = tmp_path / "panel_state.json"
    monkeypatch.setattr(state, "PANEL_STATE_PATH", panel_state_path)
    serial_transport = SimpleNamespace(active_port="COM7")
    subject = SimpleNamespace(
        _panel_state={"preserved": "yes"},
        transport_var=Value("serial"),
        auto_connect_var=Value(True),
        transport=serial_transport,
        serial_transport=serial_transport,
        tcp_transport=object(),
        _serial_port_identity={
            "COM7": {"vid": 0x0483, "pid": 0x5740, "serial_number": "board-7"}
        },
        serial_baud_var=Value(115200),
    )

    state.PanelStateMixin._save_panel_state(subject)
    payload = json.loads(panel_state_path.read_text(encoding="utf-8"))

    assert payload["preserved"] == "yes"
    assert payload["serial_port"] == "COM7"
    assert payload["serial_fingerprint"] == "0483:5740/board-7"
    assert state.PanelStateMixin._load_panel_state(subject) == payload


def test_logging_helpers_write_only_to_injected_temporary_paths(
    tmp_path: Path, monkeypatch,
) -> None:
    regular_log = tmp_path / "nested" / "panel.log"
    crash_log = tmp_path / "panel_crash.log"
    monkeypatch.setattr(state, "PANEL_CRASH_LOG", crash_log)

    state.append_log(regular_log, "hello")
    try:
        raise ValueError("synthetic crash")
    except ValueError as exc:
        summary = state.record_panel_crash(type(exc), exc, exc.__traceback__)

    assert "hello" in regular_log.read_text(encoding="utf-8")
    assert "synthetic crash" in summary
    assert "PANEL EXCEPTION" in crash_log.read_text(encoding="utf-8")

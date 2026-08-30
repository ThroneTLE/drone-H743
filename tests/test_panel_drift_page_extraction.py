"""S6 stationary-drift page extraction ownership and compatibility contract."""

from __future__ import annotations

import ast
import hashlib
from pathlib import Path
from types import SimpleNamespace

from tools import drone_tcp_panel as legacy_panel
from tools.panel_lib.pages import drift as drift_page


ROOT = Path(__file__).resolve().parents[1]
LEGACY_PANEL_PATH = ROOT / "tools" / "drone_tcp_panel.py"
DRIFT_PAGE_PATH = ROOT / "tools" / "panel_lib" / "pages" / "drift.py"

BUILD_METHOD = "_build_drift_page"
MOVED_HANDLERS = {
    "_drift_accept_sample",
    "_drift_load_previous",
    "_drift_render_comparison",
    "_drift_set_baseline",
    "_drift_start",
    "_drift_stop",
    "_drift_tick",
}

# SHA-256 of ast.dump(..., include_attributes=False) at pre-extraction HEAD
# 88e33205.  These make the no-behaviour-change requirement mechanical rather
# than relying only on names moving between classes.
HANDLER_AST_SHA256 = {
    "_drift_accept_sample": "dacaba39672ed4e62332be0bdb572b762fdda9a5bb35d12678e8c713dbbd622b",
    "_drift_load_previous": "d4077accc0860991f63051bdb7d79c69092035f9aa24afd97e9aa427a48895a2",
    "_drift_render_comparison": "b28f49ef211fd6b17cfe257a759c79ad0572aacc2af3d99e183da75d762b072a",
    "_drift_set_baseline": "4cf6c587241ccf166abc345246a385add7b5ad523c18071c82e679d6feeaf5cd",
    "_drift_start": "550e9f49a3c855a823797e128c10d6f6f6fb9a0fc203a448d84f7a211fdcef43",
    "_drift_stop": "c174f728ff0ff06610a10cb3f03f959b832686dd0735c07f961ca1724312ff90",
    "_drift_tick": "ee8b65cb081b000f615170fdcdab66bbdb192cec242e105eb91a12d0579f35f7",
}
BUILD_BODY_AST_SHA256 = (
    "6dcd04113540ca206fd0a37076c56f52e167be564350af8364f72bbb91fb5f31"
)


def parsed(path: Path) -> ast.Module:
    return ast.parse(path.read_text(encoding="utf-8"))


def class_node(path: Path, class_name: str) -> ast.ClassDef:
    return next(
        node
        for node in parsed(path).body
        if isinstance(node, ast.ClassDef) and node.name == class_name
    )


def class_methods(path: Path, class_name: str) -> dict[str, ast.FunctionDef]:
    return {
        node.name: node
        for node in class_node(path, class_name).body
        if isinstance(node, ast.FunctionDef)
    }


def ast_sha256(node: ast.AST) -> str:
    payload = ast.dump(node, include_attributes=False).encode()
    return hashlib.sha256(payload).hexdigest()


def test_drift_page_mixin_owns_only_its_builder_and_handlers() -> None:
    owned = class_methods(DRIFT_PAGE_PATH, "DriftPageMixin")
    legacy = class_methods(LEGACY_PANEL_PATH, "DronePanel")

    assert set(owned) == {BUILD_METHOD, *MOVED_HANDLERS}
    assert MOVED_HANDLERS.isdisjoint(legacy)
    assert BUILD_METHOD not in legacy


def test_legacy_panel_forwards_the_drift_page_without_wrappers() -> None:
    assert legacy_panel.DriftPageMixin is drift_page.DriftPageMixin
    assert legacy_panel.drift is drift_page.drift
    assert legacy_panel.DronePanel._build_drift_page is drift_page.DriftPageMixin._build_drift_page
    for name in MOVED_HANDLERS:
        assert getattr(legacy_panel.DronePanel, name) is getattr(
            drift_page.DriftPageMixin, name
        )


def test_moved_drift_page_ast_matches_the_pre_extraction_implementation() -> None:
    owned = class_methods(DRIFT_PAGE_PATH, "DriftPageMixin")

    assert {
        name: ast_sha256(owned[name]) for name in MOVED_HANDLERS
    } == HANDLER_AST_SHA256
    build_body = ast.Module(body=owned[BUILD_METHOD].body, type_ignores=[])
    assert ast_sha256(build_body) == BUILD_BODY_AST_SHA256


def test_v1_page_keeps_the_drift_builder_at_the_original_slot() -> None:
    v1 = class_methods(LEGACY_PANEL_PATH, "DronePanel")["_build_v1_page"]
    calls = [
        node
        for node in ast.walk(v1)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and isinstance(node.func.value, ast.Name)
        and node.func.value.id == "self"
        and node.func.attr == BUILD_METHOD
    ]

    assert len(calls) == 1
    assert len(calls[0].args) == 1
    assert isinstance(calls[0].args[0], ast.Name)
    assert calls[0].args[0].id == "parent"
    assert calls[0].lineno < next(
        node.lineno
        for node in v1.body
        if isinstance(node, ast.Assign)
        and any(isinstance(target, ast.Name) and target.id == "pending" for target in node.targets)
    )


def test_drift_sample_pairing_behavior_is_preserved_in_memory() -> None:
    subject = SimpleNamespace(
        drift_recording=True,
        _drift_pending_motion=None,
        drift_last_sequence=-1,
        drift_samples=[],
    )

    drift_page.DriftPageMixin._drift_accept_sample(
        subject, {"seq": "17", "ts_ms": "1250", "temp_cdeg": "2634"}
    )
    drift_page.DriftPageMixin._drift_accept_sample(
        subject,
        {
            "seq": "17",
            "gx": "1000",
            "gy": "-2000",
            "gz": "3000",
            "ax": "0",
            "ay": "10",
            "az": "1000",
            "roll": "125",
            "pitch": "-250",
            "yaw": "375",
        },
    )

    assert len(subject.drift_samples) == 1
    sample = subject.drift_samples[0]
    assert sample.sequence == 17
    assert sample.timestamp_s == 1.25
    assert sample.temperature_c == 26.34
    assert sample.gyro_dps == (1.0, -2.0, 3.0)
    assert sample.attitude_deg == (1.25, -2.5, 3.75)

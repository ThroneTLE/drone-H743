"""S6 V1-metrology page extraction ownership and compatibility contract."""

from __future__ import annotations

import ast
import hashlib
import os
from pathlib import Path
import subprocess
import sys

from tools import drone_tcp_panel as legacy_panel
from tools.panel_lib.pages import mechanical as mechanical_page
from tools.panel_lib.pages import v1_metrology as v1_page


ROOT = Path(__file__).resolve().parents[1]
LEGACY_PANEL_PATH = ROOT / "tools" / "drone_tcp_panel.py"
MECHANICAL_PAGE_PATH = ROOT / "tools" / "panel_lib" / "pages" / "mechanical.py"
V1_PAGE_PATH = ROOT / "tools" / "panel_lib" / "pages" / "v1_metrology.py"
FLOW_PAGE_PATH = ROOT / "tools" / "panel_lib" / "pages" / "flow_ranging.py"

METHOD_AST_SHA256 = {
    "_build_v1_page": "406c0f27969e064cfe7132b65a655613b61fc3e853439cfddf8c94565bc0efd9",
    "_v1_selected_plan": "292dbe0f38926e62ba68965f2655337935ed3de9279c53f7b0bc6e6173849f6b",
    "_v1_refresh_controls": "b0e2fc2b70fd2a2ffbb81f431b62a93fd2379a64701623a4594328b81250d2ee",
    "_v1_new_session": "0d06e546da8ba5c9cd0fe3737297e81d679c8e74a5cc141bab30644de483c30c",
    "_v1_load_latest_session": "a63b56374b4ba54d37f07c23a49345b685a5117b7f173fcc565634ccabff9ca5",
    "_v1_probe": "46257ba52097e0631f2b95fc9ac0b6fe80f472b8218c7b6fcb4a58ece8076985",
    "_v1_describe_analysis": "f5000e377a6d98006d53e3d343c5b7704ca94c96d56a7fff4397ebc2915c2583",
    "_v1_stage_verdict": "c7e17237b58b7731fc8dbf55a903d25f3347cbaf473b08dc97fdb1a0c7987ca5",
    "_v1_session_rows": "5f291bf36919dfa841e166fbfdeb19d6678ffd40e874eb8fb99b7d53c930d793",
    "_v1_render_session": "8ca54b0c9b0f7128378dc9a3c9a87137629d61d7465a2b42d7c1c216d5dd70a4",
    "_v1_on_row_selected": "6e413fae446dd9e55f04e42f22cdc37cdfc3f9a468c8e3a57adba2ce70f6d998",
    "_v1_discard_selected": "b9f26a2a12d3f3746a3b9984a597420ffc799ecf07b0a32de08da436fe9356e5",
    "_v1_start_capture": "87c8af5de0a54455abbfc3a9383c15fe6eef27d2ed67eca63baccf665490006e",
    "_v1_finish_capture": "681baed76d9f9ae33e14c2395f7c7fa93cc01ca27f7cf854f23d4719f97aa082",
    "_v1_capture_worker": "fad0c4923f9bb6ced735e23c1d8946173a6078bac95d3ec6303bddee2ce7fcc0",
    "_v1_start_analysis": "4886b9605909d4e1e8c066d72db4c49cee1e4d4c6b391ade23d1da253025e321",
    "_v1_analysis_worker": "66f3e57ab96c21b1fc7e9902576efed10342388b119aa0e4625e25da843e7fb2",
    "_v1_begin_target_action": "f16b28d177676810a76a3db2fbfab811ef41c023841e8cb3e86d085b4ca9ceb6",
    "_v1_sync_target_state": "9b5c828b0b9e850ace3a9f7f9afe18450e117f712da39a433dcb5fd6f3d566c9",
    "_v1_apply_candidate": "04f9ee04f3985765b477fb9b1d93e7c2101003862ef7c92ae22df8a54999568f",
    "_v1_revert_candidate": "315c3236dfc4bd09f86f6b427ea22a7ac3213e5d84fc0d5ad437e4b15db1a70d",
    "_v1_commit_candidate": "c4b4db719058ad298e6bcf8f52a0f0658208867a834246d70905a11eff21ef10",
    "_v1_target_worker": "ab7013ab7d4a53b86232c75aa9a5bf4800e8e5411c46b6f5c4e37fb4ca4ed4a8",
    "_v1_drain_events": "26e7ed7959bf9e7d52a8c1b6b1010334656154ac393c131efed877ec292e0483",
}
# 增量11 起光流页也搬走了，这里从"仍留在旧文件"改成"已归 flow_ranging.py"。
EXTRACTED_FLOW_BUILDERS = {"_build_flow_range_calibration_page"}
EXTRACTED_MECHANICAL_BUILDERS = {"_build_mechanical_calibration_page"}


def parsed(path: Path) -> ast.Module:
    return ast.parse(path.read_text(encoding="utf-8"))


def class_methods(path: Path, class_name: str) -> dict[str, ast.FunctionDef]:
    owner = next(
        node
        for node in parsed(path).body
        if isinstance(node, ast.ClassDef) and node.name == class_name
    )
    return {
        node.name: node
        for node in owner.body
        if isinstance(node, ast.FunctionDef)
    }


def ast_sha256(node: ast.AST) -> str:
    payload = ast.dump(node, include_attributes=False).encode()
    return hashlib.sha256(payload).hexdigest()


def test_v1_page_mixin_owns_the_builder_and_every_v1_handler() -> None:
    owned = class_methods(V1_PAGE_PATH, "V1PageMixin")
    legacy = class_methods(LEGACY_PANEL_PATH, "DronePanel")
    mechanical = class_methods(MECHANICAL_PAGE_PATH, "MechanicalPageMixin")
    flow = class_methods(FLOW_PAGE_PATH, "FlowRangingPageMixin")

    assert set(owned) == set(METHOD_AST_SHA256)
    assert set(METHOD_AST_SHA256).isdisjoint(legacy)
    assert EXTRACTED_FLOW_BUILDERS <= set(flow)
    assert EXTRACTED_FLOW_BUILDERS.isdisjoint(legacy)
    assert EXTRACTED_MECHANICAL_BUILDERS <= set(mechanical)
    assert (EXTRACTED_FLOW_BUILDERS | EXTRACTED_MECHANICAL_BUILDERS).isdisjoint(owned)
    assert legacy_panel.MechanicalPageMixin is mechanical_page.MechanicalPageMixin


def test_all_24_methods_match_the_post_increment4_ast() -> None:
    owned = class_methods(V1_PAGE_PATH, "V1PageMixin")

    assert {
        name: ast_sha256(owned[name]) for name in METHOD_AST_SHA256
    } == METHOD_AST_SHA256
    assert len(owned["_build_v1_page"].body) == 54


def test_legacy_panel_forwards_v1_methods_without_wrappers() -> None:
    assert legacy_panel.V1PageMixin is v1_page.V1PageMixin
    for name in METHOD_AST_SHA256:
        assert getattr(legacy_panel.DronePanel, name) is getattr(
            v1_page.V1PageMixin, name
        )

    for name in v1_page.__all__:
        assert getattr(legacy_panel, name) is getattr(v1_page, name), name


def test_v1_page_keeps_the_extracted_drift_builder_composition() -> None:
    build = class_methods(V1_PAGE_PATH, "V1PageMixin")["_build_v1_page"]
    calls = [
        node
        for node in ast.walk(build)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "_build_drift_page"
    ]

    assert len(calls) == 1


def test_v1_module_keeps_the_same_dependencies_and_page_constants() -> None:
    for name in (
        "CAPTURE_PLANS",
        "CaptureLink",
        "EncodedV1Candidate",
        "MetrologyStage",
        "MetrologyStatus",
        "V1Session",
    ):
        assert getattr(v1_page, name) is getattr(legacy_panel, name), name

    for name in (
        "V1_CAPTURE_PREP_SECONDS",
        "V1_FACE_RESIDUAL_FAIL_G",
        "V1_FACE_RESIDUAL_WARN_G",
        "V1_STAGE_LABELS",
    ):
        assert getattr(v1_page, name) == getattr(legacy_panel, name), name

    assert all(
        v1_page.UI_PALETTE[name] == legacy_panel.UI_PALETTE[name]
        for name in v1_page.UI_PALETTE
    )


def test_legacy_panel_keeps_the_direct_script_import_context() -> None:
    env = os.environ.copy()
    env.pop("PYTHONPATH", None)
    result = subprocess.run(
        [
            sys.executable,
            "-c",
            (
                "import drone_tcp_panel as panel; "
                "import panel_lib.pages.v1_metrology as owner; "
                "assert panel.V1PageMixin is owner.V1PageMixin; "
                "assert panel.DronePanel._v1_drain_events is owner.V1PageMixin._v1_drain_events"
            ),
        ],
        cwd=ROOT / "tools",
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=20,
    )
    assert result.returncode == 0, result.stdout

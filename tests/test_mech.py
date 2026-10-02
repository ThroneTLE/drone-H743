# CP210 mechanical gate update: behavioral guards covered by test_mechanical_cp210_gate.py.
from __future__ import annotations

import ast
import hashlib
import os
from pathlib import Path
import subprocess
import sys

from tools import drone_tcp_panel as legacy_panel
from tools.panel_lib.pages import mechanical as mechanical_page


ROOT = Path(__file__).resolve().parents[1]
LEGACY_PANEL_PATH = ROOT / "tools" / "drone_tcp_panel.py"
MECHANICAL_PAGE_PATH = ROOT / "tools" / "panel_lib" / "pages" / "mechanical.py"
FLOW_PAGE_PATH = ROOT / "tools" / "panel_lib" / "pages" / "flow_ranging.py"
PARENT_COMMIT = "e88b3d99a061d199ab1fe9fb69d7e952c0a6e2bf"

METHOD_AST_SHA256 = {
    "_build_mechanical_calibration_page": "a361b5ff4e5dc1f5dc69cd3a7877e795abdca1ec4d6c8d029700902d7f2e5154",
    "_mechanical_row_values": "91112bfc4ed0c275a94a5bab26e39ff0caf68b8e3377991765cd5f9cdeb2ab82",
    "_mechanical_local_target": "9f151f75ecdee6a72f283491518fb793abbaee323a1616a1b84032e16d7bd156",
    "_mechanical_target_matches_local": "7a959f7bd8f9fcf114d521e56f1369342bad5a2e2f1557053f8c55f421e99427",
    "_mechanical_read_target": "e5f4c3d944d72bc3aac93eb6e70e278b95a9560412ee2dac47350c2f0ea59cbf",
    "_mechanical_apply_target": "6f7fbfc7ee70b1326d35d61864323ded331fae346bdfa118096f55c80a32902f",
    "_mechanical_revert_target": "0536ff47b7aacfd7de2938baf6b8e2d4d4c75b76dc9cd801da1a93a878967e26",
    "_mechanical_commit_target": "dc5c31afddf057cdd754628527d6ae4825bab0dd12348e0e9be1bbd0daa29e4c",
    "_mechanical_handle_target_line": "d97d2de7c02932f7163bad3a37caeb4696d0b2a2eacf82427ba71aa24aaa5230",
    "_mechanical_move": "0e021c2f275be8bc1531275a8cef1210536b4a6002012f980664dfab92005d2b",
    "_mechanical_nudge_center": "697e45805c5fb159cd4c409a1656cb08515169d1a2d9599be3afbc4734fbc265",
    "_mechanical_jog_stop": "bd1fc4be0f2a5c43eb1968f7929e4de85ee93fb1183307f4fe80231926295170",
    "_mechanical_save_evidence": "090a8861825e49b5b1c3440138e9260fb09d8b80e093eecaa07852939574e9f2",
}

FLOW_AST_SHA256 = {
    # R-FLOWMOUNT-1（2026-09-29）：光流安装方向挂进标定页，下面四个方法有意改动后重钉；
    # 改了什么见 tests/test_panel_flow_page_extraction.py 的同名说明。
    "_build_flow_range_calibration_page": "1167a5e700a9d6685ad7f01983e5720d4255ec2dc633be87a0f6cc5d8c98dcf6",
    "_flow_range_request_once": "2ff7e3a41f240d644b4e729c14f68ef8f563784a0157d7f5c9908cb0c61c3428",
    "_flow_cal_start": "32aa4592565d667b0612e72aa125a313ec357a7749096c787a43bbef64cec556",
    "_flow_cal_stop": "88dd7b33827268f34ff4b61a7bf2b15d5f97a2b81452b3c2bc80547882aad7e3",
    "_flow_cal_analyze_stage": "49fd8a48b303597d37b42694f58ddd861e1232b1af6ef6708676bd0357f1db69",
    "_flow_cal_result_summary": "7af89dde1f64b009dc2bba0988b26211734ed9e533ba1b1832e2bb639fe84f72",
    "_flow_cal_refresh_tree": "08aa97e663548e84313924767a17b976c148e61974fce1e58f5225a5f268e747",
    "_flow_cal_save_report": "5e89a65dbd66e141f34917108733de34d75e7ed7cd5499229adefd909076fb59",
}


def parsed(path: Path) -> ast.Module:
    return ast.parse(path.read_text(encoding="utf-8"))


def class_methods(path: Path, class_name: str) -> dict[str, ast.FunctionDef]:
    owner = next(
        node for node in parsed(path).body
        if isinstance(node, ast.ClassDef) and node.name == class_name
    )
    return {
        node.name: node for node in owner.body if isinstance(node, ast.FunctionDef)
    }


def ast_sha256(node: ast.AST) -> str:
    return hashlib.sha256(ast.dump(node, include_attributes=False).encode()).hexdigest()


def _check_method_ownership() -> None:
    owned = class_methods(MECHANICAL_PAGE_PATH, "MechanicalPageMixin")
    legacy = class_methods(LEGACY_PANEL_PATH, "DronePanel")
    assert set(owned) == set(METHOD_AST_SHA256)
    assert set(owned).isdisjoint(legacy)
    assert PARENT_COMMIT == "e88b3d99a061d199ab1fe9fb69d7e952c0a6e2bf"


def _check_method_ast() -> None:
    owned = class_methods(MECHANICAL_PAGE_PATH, "MechanicalPageMixin")
    assert {name: ast_sha256(owned[name]) for name in METHOD_AST_SHA256} == METHOD_AST_SHA256
    assert len(owned["_build_mechanical_calibration_page"].body) == 34


def _check_forwarding() -> None:
    assert legacy_panel.MechanicalPageMixin is mechanical_page.MechanicalPageMixin
    for name in METHOD_AST_SHA256:
        assert getattr(legacy_panel.DronePanel, name) is getattr(
            mechanical_page.MechanicalPageMixin, name
        )
    for name in mechanical_page.__all__:
        assert getattr(legacy_panel, name) is getattr(mechanical_page, name)


def _check_dependencies() -> None:
    for name in (
        "GroundCalibrationError",
        "SERVO_MECHANICAL_CALIBRATION_DIR",
        "dated_directory",
        "ensure_directory",
        "validate_servo_geometry",
    ):
        assert getattr(mechanical_page, name) is getattr(legacy_panel, name), name


def _check_flow_page_frozen() -> None:
    """增量11 把光流页搬进 flow_ranging.py；这里的哈希仍是搬家前的值。

    键不变、哈希不变、只有所有者变了——这正是"搬家不改行为"的判据。
    """
    owner = class_methods(FLOW_PAGE_PATH, "FlowRangingPageMixin")
    legacy = class_methods(LEGACY_PANEL_PATH, "DronePanel")
    assert {name: ast_sha256(owner[name]) for name in FLOW_AST_SHA256} == FLOW_AST_SHA256
    assert set(FLOW_AST_SHA256).isdisjoint(legacy), "旧文件里不许留副本"


def _check_direct_import() -> None:
    env = os.environ.copy()
    env.pop("PYTHONPATH", None)
    result = subprocess.run(
        [
            sys.executable,
            "-c",
            (
                "import drone_tcp_panel as panel; "
                "import panel_lib.pages.mechanical as owner; "
                "assert panel.MechanicalPageMixin is owner.MechanicalPageMixin; "
                "assert panel.DronePanel._mechanical_jog_stop is "
                "owner.MechanicalPageMixin._mechanical_jog_stop"
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


def test_s6() -> None:
    _check_method_ownership()
    _check_method_ast()
    _check_forwarding()
    _check_dependencies()
    _check_flow_page_frozen()
    _check_direct_import()

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
    "_mechanical_apply_target": "bc9b9b3f31e451919c580c89e44b089a7bbd2d2e41aec5fe5d079fe8aa9bfef6",
    "_mechanical_revert_target": "0536ff47b7aacfd7de2938baf6b8e2d4d4c75b76dc9cd801da1a93a878967e26",
    "_mechanical_commit_target": "dc5c31afddf057cdd754628527d6ae4825bab0dd12348e0e9be1bbd0daa29e4c",
    "_mechanical_handle_target_line": "d97d2de7c02932f7163bad3a37caeb4696d0b2a2eacf82427ba71aa24aaa5230",
    "_mechanical_move": "5d375849112026204ab10ebff4eaf4a7d99ff1f287bab24a48e00e0c78d48183",
    "_mechanical_nudge_center": "44b1f0da89a2b39e291c8520f994d197fccd99760d25a700469bd25d821633bf",
    "_mechanical_jog_stop": "bd1fc4be0f2a5c43eb1968f7929e4de85ee93fb1183307f4fe80231926295170",
    "_mechanical_save_evidence": "090a8861825e49b5b1c3440138e9260fb09d8b80e093eecaa07852939574e9f2",
}

FLOW_AST_SHA256 = {
    "_build_flow_range_calibration_page": "b82cfc6332ac43ec89dcea660fc61a76080e7bd4fe203c1ad7a9d18fcbd7b448",
    "_flow_range_request_once": "2ff7e3a41f240d644b4e729c14f68ef8f563784a0157d7f5c9908cb0c61c3428",
    "_flow_cal_start": "59f98967d8d888edbfb924652470a982f9428b45711c45c93ecc8329fbe30147",
    "_flow_cal_stop": "7f0514ad7a8b81f308c642468e4cfe6f8191b833ea4779a5aa0e292104355d9a",
    "_flow_cal_analyze_stage": "49fd8a48b303597d37b42694f58ddd861e1232b1af6ef6708676bd0357f1db69",
    "_flow_cal_result_summary": "7af89dde1f64b009dc2bba0988b26211734ed9e533ba1b1832e2bb639fe84f72",
    "_flow_cal_refresh_tree": "08aa97e663548e84313924767a17b976c148e61974fce1e58f5225a5f268e747",
    "_flow_cal_save_report": "5796e238f117707a1272c649853af1973dacaa8a9fdfd45d2da775c5432054f0",
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

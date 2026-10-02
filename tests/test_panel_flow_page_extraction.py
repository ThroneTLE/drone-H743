"""R-S6-1 增量11（收官）：光流与测距页搬进 panel_lib/pages/flow_ranging.py。

搬家的判据只有一条——行为零变更。这里用三种独立方式钉：
  1. 逐方法 AST 哈希与搬家前逐字节一致（哈希取自父提交 925356ba 的旧文件）；
  2. 旧文件里不许留副本，页面常量也只允许有一份定义；
  3. `python tools/drone_tcp_panel.py` 这种直跑姿势仍能拿到同一个函数对象。
"""

from __future__ import annotations

import ast
import hashlib
import os
from pathlib import Path
import subprocess
import sys

from tools import drone_tcp_panel as legacy_panel
from tools.panel_lib.pages import flow_ranging as flow_page
from tools.panel_lib.pages import v1_metrology as v1_page


ROOT = Path(__file__).resolve().parents[1]
LEGACY_PANEL_PATH = ROOT / "tools" / "drone_tcp_panel.py"
FLOW_PAGE_PATH = ROOT / "tools" / "panel_lib" / "pages" / "flow_ranging.py"
V1_PAGE_PATH = ROOT / "tools" / "panel_lib" / "pages" / "v1_metrology.py"
PARENT_COMMIT = "925356ba"

# 前八个是增量1 起就钉住的旧哈希，原样沿用；后两个是本次新搬的两个遥测行处理器，
# 哈希取自父提交的 DronePanel，未作任何改动。
# 2026-09-29（R-FLOWMOUNT-1）：光流安装方向的推荐/写入挂进本页，有意改了五个方法——
# 建页（挂安装方向面板）、开始/停止一步（记下采样期间的安装参数）、存证据（并入写入记录）、
# 实时行（显示 mount_yaw/mount_mirror）——这五个哈希按改后的源码重钉；逻辑本体在
# pages/flow_mount.py，本类只多了这几处一行挂钩。其余四个方法仍是搬家前的原字节。
# 同日主控又把建页里的小标题「只读地面采样」改成「地面采样与安装方向写回」（页面已能写回飞控），建页哈希再重钉一次。
METHOD_AST_SHA256 = {
    "_build_flow_range_calibration_page": "cd5fe5c90e8a2590080428296ea59959724ef104a675436086f978f0149d0ac4",
    "_flow_range_request_once": "2ff7e3a41f240d644b4e729c14f68ef8f563784a0157d7f5c9908cb0c61c3428",
    "_flow_cal_start": "32aa4592565d667b0612e72aa125a313ec357a7749096c787a43bbef64cec556",
    "_flow_cal_stop": "88dd7b33827268f34ff4b61a7bf2b15d5f97a2b81452b3c2bc80547882aad7e3",
    "_flow_cal_analyze_stage": "49fd8a48b303597d37b42694f58ddd861e1232b1af6ef6708676bd0357f1db69",
    "_flow_cal_result_summary": "7af89dde1f64b009dc2bba0988b26211734ed9e533ba1b1832e2bb639fe84f72",
    "_flow_cal_refresh_tree": "08aa97e663548e84313924767a17b976c148e61974fce1e58f5225a5f268e747",
    "_flow_cal_save_report": "5e89a65dbd66e141f34917108733de34d75e7ed7cd5499229adefd909076fb59",
    "_update_flow_line": "ae7214d78960b071ee09ddb4e812438290870c44d37ac938b394280a9be4b2b1",
}


def parsed(path: Path) -> ast.Module:
    return ast.parse(path.read_text(encoding="utf-8"))


def class_methods(path: Path, class_name: str) -> dict[str, ast.FunctionDef]:
    owner = next(
        node for node in parsed(path).body
        if isinstance(node, ast.ClassDef) and node.name == class_name
    )
    return {node.name: node for node in owner.body if isinstance(node, ast.FunctionDef)}


def own_definitions(path: Path) -> set[str]:
    """只算"自己定义"的顶层名；`X = _panel_v1.X` 这种转发不算定义。"""
    names: set[str] = set()
    for node in parsed(path).body:
        if not isinstance(node, ast.Assign):
            continue
        value = node.value
        forwarded = (
            isinstance(value, ast.Attribute)
            and isinstance(value.value, ast.Name)
            and value.value.id.startswith("_panel_")
        )
        if forwarded:
            continue
        names.update(t.id for t in node.targets if isinstance(t, ast.Name))
    return names


def ast_sha256(node: ast.AST) -> str:
    return hashlib.sha256(ast.dump(node, include_attributes=False).encode()).hexdigest()


def test_flow_mixin_owns_every_moved_method_with_the_pre_move_ast() -> None:
    owned = class_methods(FLOW_PAGE_PATH, "FlowRangingPageMixin")
    legacy = class_methods(LEGACY_PANEL_PATH, "DronePanel")

    assert set(owned) == set(METHOD_AST_SHA256)
    assert {name: ast_sha256(owned[name]) for name in METHOD_AST_SHA256} == METHOD_AST_SHA256
    assert set(METHOD_AST_SHA256).isdisjoint(legacy), "搬家是移动，不是复制"


def test_page_constant_has_exactly_one_definition() -> None:
    """§11.1 的铁律：提取即搬移，不许两边各留一份常量各改各的。"""
    assert "FLOW_CALIBRATION_STAGES" in own_definitions(FLOW_PAGE_PATH)
    assert "FLOW_CALIBRATION_STAGES" not in own_definitions(LEGACY_PANEL_PATH)
    assert legacy_panel.FLOW_CALIBRATION_STAGES is flow_page.FLOW_CALIBRATION_STAGES
    # 同一条铁律回头修掉了 V1 那四个此前两边各留一份的副本。
    for name in (
        "V1_CAPTURE_PREP_SECONDS",
        "V1_FACE_RESIDUAL_FAIL_G",
        "V1_FACE_RESIDUAL_WARN_G",
        "V1_STAGE_LABELS",
    ):
        assert name not in own_definitions(LEGACY_PANEL_PATH), name
        assert name in own_definitions(V1_PAGE_PATH), name
        assert getattr(legacy_panel, name) is getattr(v1_page, name), name


def test_legacy_panel_forwards_the_same_objects() -> None:
    assert legacy_panel.FlowRangingPageMixin is flow_page.FlowRangingPageMixin
    assert issubclass(legacy_panel.DronePanel, flow_page.FlowRangingPageMixin)
    for name in METHOD_AST_SHA256:
        assert getattr(legacy_panel.DronePanel, name) is getattr(
            flow_page.FlowRangingPageMixin, name
        ), name
    for name in flow_page.__all__:
        assert getattr(legacy_panel, name) is getattr(flow_page, name), name


def test_flow_page_dependencies_stay_the_same_objects() -> None:
    for name in (
        "FlowRangeSample",
        "GroundCalibrationError",
        "analyze_flow_axis",
        "analyze_flow_zero",
        "analyze_rotation_compensation",
        "fit_range_two_point",
        "FLOW_RANGE_CALIBRATION_DIR",
        "dated_directory",
        "ensure_directory",
    ):
        assert getattr(flow_page, name) is getattr(legacy_panel, name), name


def test_direct_script_import_context_still_resolves() -> None:
    env = os.environ.copy()
    env.pop("PYTHONPATH", None)
    result = subprocess.run(
        [
            sys.executable,
            "-c",
            (
                "import drone_tcp_panel as panel; "
                "import panel_lib.pages.flow_ranging as owner; "
                "assert panel.FlowRangingPageMixin is owner.FlowRangingPageMixin; "
                "assert panel.DronePanel._update_flow_line is "
                "owner.FlowRangingPageMixin._update_flow_line"
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

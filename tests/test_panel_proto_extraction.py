"""S6 panel protocol extraction ownership and compatibility contract."""

from __future__ import annotations

import ast
from pathlib import Path

from tools import drone_tcp_panel as legacy_panel
from tools.panel_lib import proto
from tools.panel_lib import transport


ROOT = Path(__file__).resolve().parents[1]
LEGACY_PANEL_PATH = ROOT / "tools" / "drone_tcp_panel.py"
PROTO_PATH = ROOT / "tools" / "panel_lib" / "proto.py"
TRANSPORT_PATH = ROOT / "tools" / "panel_lib" / "transport.py"

MOVED_FUNCTIONS = {
    "first_float",
    "first_value",
    "parse_kv",
    "safe_float",
    "safe_int",
}
MOVED_METHODS = {"_ensure_line_prefix", "_normalize_proto_line"}
TRANSPORT_PROTO_NAMES = {
    "PROTO_DIR_FROM_FC",
    "PROTO_DIR_TO_FC",
    "PROTO_HEADER",
    "PROTO_MSG_CMD_LINE",
}


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


def owned_proto_assignments(path: Path) -> set[str]:
    result: set[str] = set()
    for node in parsed(path).body:
        if not isinstance(node, ast.Assign) or isinstance(node.value, ast.Attribute):
            continue
        result.update(
            target.id
            for target in node.targets
            if isinstance(target, ast.Name) and target.id.startswith("PROTO_")
        )
    return result


def test_proto_module_owns_the_protocol_table_and_parsing_helpers() -> None:
    proto_names = {name for name in vars(proto) if name.startswith("PROTO_")}
    legacy_names = {name for name in vars(legacy_panel) if name.startswith("PROTO_")}

    # 87 = 84 + PROTO_MSG_TELEM_FRAME（S8 / R-T1-1 的遥测掩码帧）
    #         + PROTO_MAX_FRAME_PAYLOAD（$X 解析器重同步用的长度上限）
    #         + PROTO_BINARY_FUNCTIONS（payload 是二进制、不许按 UTF-8 解的 fn 集合）。
    # R-MODULES-1 adds COMPONENTS; retain every legacy forwarding contract.
    # 89 + R-SYSID-1 的三个（REQ_SYSID / MSG_SYSID_SCHEMA / MSG_SYSID_BATCH）
    # + R-THRUST-2 的 THRUST_BENCH = 93。
    assert len(proto_names) == 93
    assert proto.PROTO_MSG_THRUST_BENCH == 0x2235
    assert proto.PROTO_MSG_THRUST_BENCH in proto.PROTO_BINARY_FUNCTIONS

    protocol_ids = {
        name: value for name, value in vars(proto).items()
        if isinstance(value, int)
        and name.startswith(("PROTO_REQ_", "PROTO_MSG_"))
    }
    assert len(protocol_ids.values()) == len(set(protocol_ids.values())), protocol_ids

    # 旧面板**只转发、不自创**：它拥有的每个名字都必须来自 proto.py。
    assert legacy_names <= proto_names
    # 反过来不再要求相等。`drone_tcp_panel.py` 只减不增，而 R-SYSID-1 之后的新
    # 报文只被 panel_lib 用；为它们在那个文件里加转发行等于给一个已经在收缩的
    # 文件重新开口子。没被转发的必须逐个列在这里——让下一次新增仍然是显式决定，
    # 而不是悄悄地多一个。
    assert proto_names - legacy_names == {
        "PROTO_REQ_SYSID", "PROTO_MSG_SYSID_SCHEMA", "PROTO_MSG_SYSID_BATCH",
        "PROTO_MSG_THRUST_BENCH",
    }
    assert proto_names == owned_proto_assignments(PROTO_PATH)
    assert not owned_proto_assignments(LEGACY_PANEL_PATH)
    assert not owned_proto_assignments(TRANSPORT_PATH)
    assert MOVED_FUNCTIONS <= top_level_definitions(PROTO_PATH)
    assert MOVED_FUNCTIONS.isdisjoint(top_level_definitions(LEGACY_PANEL_PATH))


def test_legacy_panel_forwards_every_proto_symbol_without_wrappers() -> None:
    """转发的必须是**同一个对象**，不能是抄了一份的常量。

    抄一份的表现是两边慢慢对不上，而且没人会想到去比。
    """
    legacy_names = {name for name in vars(legacy_panel) if name.startswith("PROTO_")}
    for name in proto.__all__:
        if name.startswith("PROTO_") and name not in legacy_names:
            continue  # R-SYSID-1 之后的新报文只走 panel_lib，见上一条测试
        assert getattr(legacy_panel, name) is getattr(proto, name), name

    assert legacy_panel.DronePanel._normalize_proto_line is proto.ProtocolLineMixin._normalize_proto_line
    assert legacy_panel.DronePanel._ensure_line_prefix is proto.ProtocolLineMixin._ensure_line_prefix


def test_protocol_line_methods_move_intact_to_the_mixin() -> None:
    assert MOVED_METHODS <= class_methods(PROTO_PATH, "ProtocolLineMixin")
    assert MOVED_METHODS.isdisjoint(class_methods(LEGACY_PANEL_PATH, "DronePanel"))


def test_transport_uses_proto_constants_but_keeps_existing_frame_builders() -> None:
    for name in TRANSPORT_PROTO_NAMES:
        assert getattr(transport, name) is getattr(proto, name), name

    assert transport.build_proto_frame.__module__ == transport.__name__
    assert transport.proto_crc8_dvb_s2.__module__ == transport.__name__
    assert not hasattr(proto, "build_proto_frame")
    assert not hasattr(proto, "proto_crc8_dvb_s2")

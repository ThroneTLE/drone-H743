"""S6 panel transport extraction ownership and compatibility contract."""

from __future__ import annotations

import ast
import os
from pathlib import Path
import subprocess
import sys

from tools import drone_tcp_panel as legacy_panel
from tools.panel_lib import transport


ROOT = Path(__file__).resolve().parents[1]
LEGACY_PANEL_PATH = ROOT / "tools" / "drone_tcp_panel.py"
TRANSPORT_PATH = ROOT / "tools" / "panel_lib" / "transport.py"

MOVED_DEFINITIONS = {
    "udp_payload_is_probably_text",
    "serial_device_identity_policy",
    "serial_port_identity",
    "serial_port_fingerprint",
    "match_remembered_serial_port",
    "select_reenumerated_application_port",
    "wait_for_application_serial",
    "proto_crc8_dvb_s2",
    "build_proto_frame",
    "TransportBase",
    "TcpTransport",
    "UdpTransport",
    "SerialTransport",
}


def top_level_definitions(path: Path) -> set[str]:
    tree = ast.parse(path.read_text(encoding="utf-8"))
    return {
        node.name
        for node in tree.body
        if isinstance(node, (ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef))
    }


def test_transport_module_owns_moved_definitions() -> None:
    assert MOVED_DEFINITIONS <= top_level_definitions(TRANSPORT_PATH)
    assert MOVED_DEFINITIONS.isdisjoint(top_level_definitions(LEGACY_PANEL_PATH))


def test_legacy_panel_forwards_the_transport_api_without_wrappers() -> None:
    for name in transport.__all__:
        assert getattr(legacy_panel, name) is getattr(transport, name), name


def test_legacy_panel_keeps_the_direct_script_import_context() -> None:
    env = os.environ.copy()
    env.pop("PYTHONPATH", None)
    result = subprocess.run(
        [
            sys.executable,
            "-c",
            (
                "import drone_tcp_panel as panel; "
                "import panel_lib.transport as owner; "
                "assert panel.SerialTransport is owner.SerialTransport"
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

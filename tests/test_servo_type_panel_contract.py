"""R-S7-5 host contracts for servo type controls and transactions."""

import ast
from pathlib import Path

from tools import drone_tcp_panel as panel
from tools.panel_lib import proto
from tools.panel_lib.pages.servo_type_controls import ServoTypeControlsMixin


ROOT = Path(__file__).resolve().parents[1]
TYPE_PAGE = ROOT / "tools" / "panel_lib" / "pages" / "servo_type_controls.py"
SERVO_PAGE = ROOT / "tools" / "panel_lib" / "pages" / "servo_debug.py"


class Value:
    def __init__(self, value):
        self.value = value

    def get(self):
        return self.value

    def set(self, value):
        self.value = value


class DummyPanel(ServoTypeControlsMixin):
    def __init__(self, output_type="bus"):
        self.servo_type_var = Value(output_type)
        self.servo_type_active_var = Value(output_type)
        self.servo_type_status_var = Value("尚未查询")
        self.sent = []

    def _send_proto(self, function, label, payload="", expect_reply=True):
        self.sent.append((function, label, payload, expect_reply))

    def _refresh_servo_output_controls(self):
        self.refreshed = True


def test_servo_type_protocol_ids_are_unique_and_forwarded() -> None:
    assert proto.PROTO_REQ_SERVOTYPE == 0x1025
    assert proto.PROTO_REQ_SERVOTYPE != proto.PROTO_REQ_SERVO_CAL
    assert proto.PROTO_MSG_SERVO_TYPE == 0x2226
    assert panel.PROTO_REQ_SERVOTYPE == proto.PROTO_REQ_SERVOTYPE
    assert panel.PROTO_MSG_SERVO_TYPE == proto.PROTO_MSG_SERVO_TYPE
    assert panel.ProtocolLineMixin()._normalize_proto_line(
        proto.PROTO_MSG_SERVO_TYPE, "state=ok active=pwm"
    ) == "SERVOTYPE state=ok active=pwm"


def test_type_transaction_buttons_send_only_frozen_commands() -> None:
    host = DummyPanel("pwm")
    host._servo_type_query()
    host._servo_type_apply()
    host._servo_type_revert()
    host._servo_type_commit()
    assert [item[:3] for item in host.sent] == [
        (0x1025, "SERVOTYPE?", ""),
        (0x1025, "SERVOTYPE APPLY type=pwm", ""),
        (0x1025, "SERVOTYPE REVERT", ""),
        (0x1025, "SERVOTYPE COMMIT", ""),
    ]


def test_type_readback_updates_active_candidate_and_status_fields() -> None:
    host = DummyPanel("bus")
    host._update_servo_type_line(
        "SERVOTYPE state=applied active=pwm persisted=bus dirty=1 valid=1 "
        "explicit=1 generation=3 record_generation=7 request=9"
    )
    assert host.servo_type_var.get() == "pwm"
    assert host.servo_type_active_var.get() == "pwm"
    assert host.servo_type_status_var.get() == (
        "state=applied active=pwm persisted=bus dirty=1 valid=1 explicit=1 "
        "generation=3 record_generation=7 request=9"
    )
    assert host.refreshed is True


def test_type_controls_are_owned_by_new_module_and_debug_page_calls_builder() -> None:
    type_module = ast.parse(TYPE_PAGE.read_text(encoding="utf-8"))
    type_owner = next(node for node in type_module.body if isinstance(node, ast.ClassDef))
    type_methods = {node.name for node in type_owner.body if isinstance(node, ast.FunctionDef)}
    assert {
        "_build_servo_type_controls",
        "_servo_type_query",
        "_servo_type_apply",
        "_servo_type_revert",
        "_servo_type_commit",
        "_servo_output_is_pwm",
        "_update_servo_type_line",
    } <= type_methods

    debug_source = SERVO_PAGE.read_text(encoding="utf-8")
    assert "self._build_servo_type_controls(parent)" in debug_source
    assert "def _servo_type_" not in debug_source
    assert "def _update_servo_type_line" not in debug_source
    assert "def _servo_output_is_pwm" not in debug_source

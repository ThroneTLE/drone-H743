"""R-S7-4 host contracts for BUS-only controls and PWM JOG routing."""

from pathlib import Path

from tools.panel_lib.pages.servo_debug import ServoDebugPageMixin


ROOT = Path(__file__).resolve().parents[1]
SERVO_PAGE = ROOT / "tools" / "panel_lib" / "pages" / "servo_debug.py"


class Value:
    def __init__(self, value):
        self.value = value

    def get(self):
        return self.value

    def set(self, value):
        self.value = value


class DummyPanel(ServoDebugPageMixin):
    def __init__(self, output_type="bus"):
        self.servo_type_active_var = Value(output_type)
        self.servo_type_var = Value(output_type)
        self.servo_widgets = [
            {
                "id": Value(1),
                "pulse": Value(1600),
                "time": Value(500),
                "mode": Value(1),
                "enabled": Value(1),
                "new_id": Value(1),
                "baud": Value(4),
            }
        ]
        self.sent = []

    def _send_proto(self, function, label, payload="", expect_reply=True):
        self.sent.append((function, label, payload, expect_reply))


def test_pwm_jog_has_no_bus_command_and_bus_move_is_unchanged() -> None:
    pwm = DummyPanel("pwm")
    pwm._servo_move(0)
    assert pwm.sent == [(0x1010, "SERVO JOG 0 1600", "SERVO JOG 0 1600", True)]

    bus = DummyPanel("bus")
    bus._servo_move(0)
    assert bus.sent == [(0x1010, "SERVO MOVE 0 1600 500", "SERVO MOVE 0 1600 500", True)]


def test_active_pwm_stays_gated_when_candidate_is_changed_to_bus() -> None:
    host = DummyPanel("pwm")
    host.servo_type_var.set("bus")
    host._servo_move(0)
    host._servo_move_all()
    host._servo_mode(0)
    host._servo_enable(0)
    host._servo_set_id(0)
    host._servo_set_physical_id(0)
    host._servo_cmd(0, "ULK")
    host._servo_baud(0)
    assert host.sent == [(0x1010, "SERVO JOG 0 1600", "SERVO JOG 0 1600", True)]


def test_pwm_page_keeps_bus_surfaces_disabled_and_does_not_add_bus_subcommands() -> None:
    source = SERVO_PAGE.read_text(encoding="utf-8")
    for command in ("ULK", "ULR", "DPT", "DCT", "DST", "BD", "SERVO RAW"):
        assert command in source
    assert 'state = "disabled" if pwm else "normal"' in source
    assert 'payload = f"SERVO JOG {index} {values[\'pulse\']}"' in source

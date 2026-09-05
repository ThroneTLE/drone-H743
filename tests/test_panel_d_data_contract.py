"""D 线 TK-03/TK-04 contracts: values must say what state they represent."""

from __future__ import annotations

import re
from pathlib import Path

import pytest

from tools import panel_qa
from tools.panel_lib.parameter_model import PARAMETER_CAPABILITIES, PID_QUICK_ALIASES
from tools.panel_qa import fixtures as qa_fixtures


ROOT = Path(__file__).resolve().parents[1]
DRIVER_PARAMS = ROOT / "Driver" / "Src" / "drv_coax_ctrl.c"
PANEL_SOURCE = ROOT / "tools" / "drone_tcp_panel.py"


@pytest.fixture(scope="module")
def offline(tmp_path_factory):
    root = tmp_path_factory.mktemp("panel-d-data")
    with panel_qa.isolated_environment(root):
        try:
            session = panel_qa.OfflinePanel.launch(scale=1.0, size=(1366, 768))
        except Exception as exc:
            if not panel_qa.is_display_unavailable(exc):
                raise
            pytest.skip(f"Tk display unavailable: {exc}")
        try:
            yield session
        finally:
            session.destroy()


@pytest.fixture
def app(offline):
    panel = offline.panel
    offline.transport.lines.clear()
    offline.transport.frames.clear()
    offline.transport.is_connected = True

    panel.param_states.clear()
    panel.params.clear()
    panel.param_iids.clear()
    panel.param_names_by_iid.clear()
    for item in panel.param_tree.get_children():
        panel.param_tree.delete(item)
    panel.param_name_var.set("")
    panel.param_value_var.set("")
    panel.param_value_error_var.set("")
    panel._init_gps_state()
    for index, values in enumerate(panel.servo_widgets):
        defaults = {"id": index + 1, "pulse": 1500, "time": 500, "mode": 1,
                    "enabled": 1, "new_id": index + 1, "baud": 4}
        for name, value in defaults.items():
            values[name].set(value)
    for terms in panel.pid_vars.values():
        for variable in terms.values():
            variable.set("")
    for variable in panel.servo_validation_vars:
        variable.set("")
    yield panel


def _driver_parameter_names() -> set[str]:
    source = DRIVER_PARAMS.read_text(encoding="utf-8")
    source = source.split("coax_ctrl_param_table[]", 1)[1].split(
        "coax_ctrl_param_count", 1
    )[0]
    named = re.findall(r'DRV_COAX_CTRL_NAMED_PARAM_ENTRY\("([^"]+)"', source)
    plain = re.findall(r"DRV_COAX_CTRL_PARAM_ENTRY\(([_A-Za-z][_A-Za-z0-9]*)\)", source)
    return {f"coax.{name}" for name in (*named, *plain)}


def test_d_modules_are_imported_in_all_three_panel_contexts() -> None:
    """The independent-loader path must bind the same D modules as package import."""
    source = PANEL_SOURCE.read_text(encoding="utf-8")
    assert source.count("pages import gps_validity as _panel_gps_validity") == 3
    assert source.count("import parameter_editor as _panel_parameter_editor") == 3


def test_parameter_capabilities_match_the_driver_table() -> None:
    """Units are host annotations, but names must come from the real driver table."""
    assert set(PARAMETER_CAPABILITIES) == _driver_parameter_names()
    assert PARAMETER_CAPABILITIES["coax.vel_x_kp"].unit == "gain"
    assert PARAMETER_CAPABILITIES["coax.tilt_limit_rad"].unit == "rad"


def test_pid_quick_names_are_real_driver_get_set_names() -> None:
    source = DRIVER_PARAMS.read_text(encoding="utf-8")
    for name in set(PID_QUICK_ALIASES.values()):
        assert f'"{name}"' in source


def test_invalid_servo_input_is_field_error_and_sends_nothing(app) -> None:
    app.servo_widgets[0]["pulse"].set("abc")
    app._servo_move(0)

    assert app.transport.frames == []
    assert "目标位置 us" in app.servo_validation_vars[0].get()
    assert "整数" in app.servo_validation_vars[0].get()


def test_servo_action_validates_only_the_fields_that_action_needs(app) -> None:
    # A malformed pulse must not prevent the independent mode action from using
    # its own valid field.  The old all-fields conversion failed here first.
    app.servo_widgets[0]["pulse"].set("abc")
    app.servo_widgets[0]["mode"].set("2")
    app._servo_output_is_pwm = lambda: False
    app._servo_mode(0)

    assert [payload.decode() for _, payload in app.transport.frames] == ["SERVO MODE 0 2"]


def test_pid_quick_editor_never_emits_unsupported_ki(app) -> None:
    app.pid_vars["roll"]["ki"].set("0.1")
    app._send_pid_values()

    assert app.transport.frames == []
    assert str(app.pid_widgets["roll"]["ki"].cget("state")) == "disabled"
    assert app.params["pid.roll.ki"]["error"]
    assert "pid.roll.ki" in app.param_editor_status_var.get()


def test_pid_quick_editor_preserves_the_existing_kp_kd_payload(app) -> None:
    app.pid_vars["roll"]["kp"].set("1.2")
    app.pid_vars["roll"]["kd"].set("0.3")
    app._send_pid_values()

    assert [payload.decode() for _, payload in app.transport.frames] == [
        "PID SET roll kp=1.2 kd=0.3"
    ]


def test_parameter_target_echo_does_not_overwrite_a_local_draft(app) -> None:
    name = "coax.vel_x_kp"
    app._update_param_line(f"PARAM name={name} value=0.8")
    app._set_param(name, "1.2", "local", dirty=True)
    app._update_param_line(f"PARAM name={name} value=0.8")

    state = app.params[name]
    assert state["target"] == "0.8"
    assert state["draft"] == "1.2"
    assert state["value"] == "1.2"
    assert state["dirty"] is True


@pytest.mark.parametrize("value", ["", "-", "abc", "nan", "inf", "-1"])
def test_invalid_parameter_values_have_zero_send(value, app) -> None:
    app.param_name_var.set("coax.vel_x_kp")
    app.param_value_var.set(value)
    app._send_param_edit()

    assert app.transport.frames == []
    assert app.params.get("coax.vel_x_kp", {}).get("draft", "") == ""
    assert app.param_value_error_var.get()


def test_parameter_send_is_pending_until_matching_target_echo(app) -> None:
    name = "coax.vel_x_kp"
    app._update_param_line(f"PARAM name={name} value=0.8")
    app.param_name_var.set(name)
    app.param_value_var.set("1.2")
    app._send_param_edit()

    assert [payload.decode() for _, payload in app.transport.frames] == [
        "PARAM SET coax.vel_x_kp 1.2"
    ]
    assert app.params[name]["pending"] == "1.2"
    assert "尚未确认" in app.param_editor_status_var.get()

    app._update_param_line(f"PARAM name={name} value=1.2")
    assert app.params[name]["target"] == "1.2"
    assert app.params[name]["draft"] == ""
    assert app.params[name]["pending"] == ""
    assert app.params[name]["dirty"] is False


def test_parameter_rejection_and_disconnect_do_not_look_applied(app) -> None:
    name = "coax.vel_x_kp"
    app._update_param_line(f"PARAM name={name} value=0.8")
    app.param_name_var.set(name)
    app.param_value_var.set("1.2")
    app._send_param_edit()
    app._handle_board_line(f"ERR param target {name}")
    assert app.params[name]["pending"] == ""
    assert app.params[name]["error"]

    app.param_value_var.set("1.3")
    app._send_param_edit()
    app._parameter_on_disconnect()
    assert app.params[name]["draft"] == "1.3"
    assert app.params[name]["target"] == "0.8"
    assert app.params[name]["pending"] == ""
    assert app.params[name]["error"]


def test_invalid_gps_is_raw_only_and_hidden_plot_is_deferred(app) -> None:
    app.notebook.select(app.calibration_group_tab)
    app.calibration_notebook.select(app.mechanical_tab)
    draws: list[str] = []
    original_plot = app._update_gps_plot
    app._update_gps_plot = lambda: draws.append("draw")
    try:
        for _ in range(5):
            app._update_gps_line(qa_fixtures.gps_status_line(
                ok=1, init=0, fix=0, valid=0, sv=0, age_ms=5000
            ))
            app._update_gps_line(qa_fixtures.gps_position_line())
    finally:
        app._update_gps_plot = original_plot

    assert len(app.gps_raw_samples) == 10
    assert app.gps_track == []
    assert app.gps_last_known is None
    assert app.gps_origin_lat is None
    assert draws == []
    assert app.gps_vars["state"].get() == "无有效定位"


def test_gps_recovery_starts_a_new_track_segment(app) -> None:
    app._update_gps_line(qa_fixtures.gps_status_line(valid=1, fix=3, sv=8))
    app._update_gps_line(qa_fixtures.gps_position_line())
    app._update_gps_line(qa_fixtures.gps_status_line(valid=0, fix=0, sv=0))
    app._update_gps_line(qa_fixtures.gps_position_line(lon_e7=1210001000, lat_e7=310001000))
    app._update_gps_line(qa_fixtures.gps_status_line(valid=1, fix=3, sv=8))
    app._update_gps_line(qa_fixtures.gps_position_line(lon_e7=1210001000, lat_e7=310001000))

    assert len(app.gps_track) == 2
    assert [point["segment"] for point in app.gps_track] == [0, 1]
    assert app.gps_last_known["valid"] is True


def test_switching_back_to_gps_consumes_dirty_plot_once(app) -> None:
    app.notebook.select(app.calibration_group_tab)
    app.calibration_notebook.select(app.mechanical_tab)
    app._update_gps_line(qa_fixtures.gps_status_line(valid=0, fix=0))
    app._update_gps_line(qa_fixtures.gps_position_line())
    assert app.gps_plot_dirty is True

    draws: list[str] = []
    original_plot = app._update_gps_plot
    app._update_gps_plot = lambda: draws.append("draw")
    try:
        app.notebook.select(app.sensor_group_tab)
        app.sensor_notebook.select(app.gps_tab)
        app._gps_on_tab_changed()
        app._gps_on_tab_changed()
    finally:
        app._update_gps_plot = original_plot

    assert draws == ["draw"]
    assert app.gps_plot_dirty is False

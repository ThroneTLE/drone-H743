"""R-PARAM-1: execute real C name lookup; no legacy online conversion."""
from pathlib import Path

import pytest

from tools.sim_xz.controller_bridge import ControllerBridge
from tools.panel_lib.parameter_model import parameter_capability

OLD_NAMES = tuple(f"coax.{axis}_{suffix}" for axis in ("roll", "pitch", "yaw")
                  for suffix in ("angle_kp", "rate_kd")) + ("coax.pos_z_ki",)


@pytest.mark.parametrize("name", OLD_NAMES)
def test_real_c_rejects_old_names_without_mutating_current_params(name):
    bridge = ControllerBridge(instance_tag="current_names")
    bridge.reset_params()
    before = bridge.parameter_snapshot()
    assert bridge.get_param(name) is None
    assert not bridge.set_param(name, 0.2)
    assert bridge.parameter_snapshot() == before
    assert parameter_capability(name) is None


@pytest.mark.parametrize("axis", ("roll", "pitch", "yaw"))
def test_attitude_p_and_rate_pid_are_independent(axis):
    bridge = ControllerBridge(instance_tag="independent_names")
    bridge.reset_params()
    keys = [f"coax.att_{axis}_kp"] + [f"coax.rate_{axis}_{term}" for term in ("kp", "ki", "kd")]
    for key in keys:
        before = {name: bridge.get_param(name) for name in keys}
        assert bridge.set_param(key, 0.17)
        assert bridge.get_param(key) == pytest.approx(0.17)
        for name in set(keys) - {key}:
            assert bridge.get_param(name) == before[name]


def test_runtime_lookup_callers_only_use_table_names():
    import re
    bridge = ControllerBridge(instance_tag="lookup_callers")
    root = Path(__file__).resolve().parents[1]
    for path in (root / "App" / "Src").glob("*.c"):
        for name in re.findall(r'DRV_COAX_CTRL_(?:Get|Set)Param\("([^"]+)"', path.read_text(encoding="utf-8")):
            assert bridge.get_param(name) is not None, (path.name, name)


def test_retired_pid_commands_explicitly_reject_and_no_slider_dispatch():
    root = Path(__file__).resolve().parents[1]
    source = (root / "App/Src/app_control.c").read_text(encoding="utf-8")
    assert source.count("ERR retired PID use PARAM? or PARAM SET") == 2
    assert "app_control_handle_pid_slider_line" not in source
    assert "app_control_report_pid_legacy" not in source


def test_saved_layout_migration_never_reuses_old_derived_values():
    from tools.panel_lib.dashboard.layout import TileSpec
    old = {"type": "param", "bindings": ["pitch_angle_kp"],
           "options": {"title": "Old gain", "value": .066, "max": 10}}
    current = TileSpec.from_json(old)
    assert current.bindings == ["att_pitch_kp"]
    assert current.options == {}
    assert old["options"]["value"] == .066
    assert TileSpec.from_json(current.to_json()).to_json() == current.to_json()


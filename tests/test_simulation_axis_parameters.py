"""X, Z and pitch gains are distinct C fields, including after mode changes."""
import pytest
from tools.sim_xz.controller_bridge import ControllerBridge
from tools.sim_xz.control_catalog import GAIN_CHANNELS, PARAMETER_GROUPS
from tools.sim_xz.experiments import SimulationEngine, ExperimentKind
from tools.panel_lib.simulation_workspaces import simulation_workspaces


@pytest.mark.parametrize('axis', ['x', 'z'])
@pytest.mark.parametrize('field', ['pos_{}_kp', 'vel_{}_kp', 'vel_{}_ki', 'vel_{}_kd'])
def test_each_axis_gain_changes_only_its_own_c_field(axis, field):
    bridge=ControllerBridge(instance_tag='axis_isolation')
    bridge.reset_params()
    names={entry[5] for entry in GAIN_CHANNELS}
    before={name: bridge.get_param(name) for name in names}
    key='coax.'+field.format(axis)
    requested=before[key]+.13
    assert bridge.set_param(key,requested)
    assert bridge.get_param(key)==pytest.approx(requested)
    for name in names-{key}:
        assert bridge.get_param(name)==before[name], (key,name)


def test_mode_switch_keeps_independent_horizontal_and_height_parameters():
    bridge=ControllerBridge(instance_tag='axis_mode_switch')
    bridge.reset_params()
    engine=SimulationEngine(bridge)
    values={'coax.pos_x_kp':.42,'coax.vel_x_kp':.81,'coax.vel_x_ki':.07,'coax.vel_x_kd':.03,
            'coax.pos_z_kp':2.7,'coax.vel_z_kp':1.2,'coax.vel_z_ki':.15,'coax.vel_z_kd':.08}
    for name,value in values.items(): assert bridge.set_param(name,value)
    for kind in (ExperimentKind.HEIGHT_STEP,ExperimentKind.VELOCITY_STEP,
                 ExperimentKind.PITCH_STEP,ExperimentKind.POSITION_STEP):
        engine.set_kind(kind)
        for name,value in values.items():
            assert bridge.get_param(name)==pytest.approx(value)


def test_three_workspaces_have_disjoint_parameter_bindings():
    workspaces=simulation_workspaces()
    assert [w.name for w in workspaces]==[
        '水平平动 · P—PID','高度 · P—PID','俯仰姿态 · P—PID']
    groups=[]
    for workspace,key in zip(workspaces,('horizontal','vertical','attitude')):
        names={tile.bindings[0] for tile in workspace.tiles if tile.type=='param'}
        assert names=={name for name,_ in PARAMETER_GROUPS[key]}
        assert len(names)==4
        groups.append(names)
    assert groups[0].isdisjoint(groups[1])
    assert groups[0].isdisjoint(groups[2])
    assert groups[1].isdisjoint(groups[2])

"""The simulator exposes and executes the actual planar P-PID-P-PID cascade."""
import pytest
from tools.sim_xz.controller_bridge import ControllerBridge
from tools.sim_xz.control_catalog import GAIN_CHANNELS
from tools.sim_xz.experiments import ExperimentKind, ExperimentTargets, run_experiment, _reference
from tools.sim_xz.physics import SimulationState
from tools.panel_lib.simulation_workspaces import simulation_workspaces


EXPECTED = {
    'coax.pos_x_kp', 'coax.vel_x_kp', 'coax.vel_x_ki', 'coax.vel_x_kd',
    'coax.att_pitch_kp', 'coax.rate_pitch_kp', 'coax.rate_pitch_ki', 'coax.rate_pitch_kd',
    'coax.pos_z_kp', 'coax.vel_z_kp', 'coax.vel_z_ki', 'coax.vel_z_kd',
}


def test_full_gains_are_bound_to_existing_c_parameters_and_visible():
    bridge = ControllerBridge(instance_tag='catalog_contract')
    assert {c[5] for c in GAIN_CHANNELS} == EXPECTED
    assert EXPECTED <= set(bridge.parameter_names())
    bindings = {b for workspace in simulation_workspaces() for tile in workspace.tiles for b in tile.bindings}
    assert {c[0] for c in GAIN_CHANNELS} <= bindings
    assert {'sim_z', 'sim_vz', 'sim_pitch_rate'} <= bindings


@pytest.mark.parametrize('parameter,field,index', [
    ('coax.vel_x_ki','force_cmd_n',0), ('coax.vel_x_kd','force_cmd_n',0),
    ('coax.vel_z_ki','force_cmd_n',2), ('coax.vel_z_kd','force_cmd_n',2),
    ('coax.rate_pitch_ki','moment_achieved_n_m',1), ('coax.rate_pitch_kd','moment_achieved_n_m',1),
])
def test_each_i_and_d_changes_the_real_c_output(parameter,field,index):
    bridge = ControllerBridge(instance_tag='pid_terms')
    bridge.reset_params()
    # 2026-09-28 俯仰默认增益换成实测机体的辨识值后，这个小推力场景（俯仰力矩上限约 7.6e-4 N·m）
    # 里 P 项单独就顶到分配器限幅，I/D 项改多少输出都一样；把 P 压到不饱和的量级，只考"I、D 真的进了输出"。
    assert bridge.set_param('coax.rate_pitch_kp', 0.02)
    def sequence(gain):
        assert bridge.set_param(parameter,gain)
        bridge.reset()
        output=[]
        for tick in range(120):
            result=bridge.step(target_x_m=.03,target_z_m=1.04,
                pitch_rate_rad_s=tick*.0002,dt_s=.001,
                accel_x_m_s2=.02,accel_z_m_s2=.03,acceleration_valid=1,
                integrator_reset=int(tick==0))
            output.append(getattr(result,field)[index])
        return output
    baseline=sequence(0)
    changed=sequence(.02)
    assert max(abs(a-b) for a,b in zip(baseline,changed)) > 1e-7


def test_height_experiment_uses_z_position_and_velocity_cascade():
    targets=ExperimentTargets(height_step_m=.25)
    assert _reference(ExperimentKind.HEIGHT_STEP,SimulationState(time_s=.9),targets)['target_z_m']==1
    ref=_reference(ExperimentKind.HEIGHT_STEP,SimulationState(time_s=1.1),targets)
    assert ref=={'target_x_m':0.,'target_z_m':1.25}
    bridge=ControllerBridge(instance_tag='height_step_contract')
    bridge.reset_params()
    samples=run_experiment(bridge,ExperimentKind.HEIGHT_STEP,duration_s=2,targets=targets)
    assert samples[-1].z_m > 1.01
    assert abs(samples[-1].x_m) < 1e-4
    assert max(s.vz_m_s for s in samples) > .01

def test_height_gain_changes_actual_z_trajectory():
    bridge=ControllerBridge(instance_tag='height_gain_effect')
    bridge.reset_params()
    before=bridge.get_param('coax.pos_z_kp')
    # Stay below the real 0.3 m/s vertical speed cap for both P values.
    cap=bridge.get_param('coax.pos_z_vel_up_max_m_s')
    targets=ExperimentTargets(height_step_m=cap/(2*before*1.5))
    baseline=run_experiment(bridge,ExperimentKind.HEIGHT_STEP,duration_s=2,targets=targets)
    assert bridge.set_param('coax.pos_z_kp',before*1.5)
    tuned=run_experiment(bridge,ExperimentKind.HEIGHT_STEP,duration_s=2,targets=targets)
    assert max(abs(a.z_m-b.z_m) for a,b in zip(baseline,tuned)) > .001

def test_switch_to_height_resets_experiment_state_but_preserves_gains():
    from tools.sim_xz.experiments import SimulationEngine
    engine=SimulationEngine()
    engine.bridge.set_param('coax.vel_z_ki',.12)
    engine.running=True
    engine.step()
    engine.set_kind(ExperimentKind.HEIGHT_STEP)
    assert engine.snapshot().time_s==0
    assert not engine.running
    assert engine.bridge.get_param('coax.vel_z_ki')==pytest.approx(.12)

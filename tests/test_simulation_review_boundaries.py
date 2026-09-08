"""Regression cases from the 2026-09-08 final review."""
from dataclasses import replace
from types import SimpleNamespace
import queue

import pytest

from tools.sim_xz import app as ui
from tools.sim_xz.experiments import SimulationEngine, ExperimentKind


@pytest.mark.parametrize("value", [float('nan'), float('inf'), -float('inf')])
@pytest.mark.parametrize("index", range(3))
def test_nonfinite_target_is_rejected_without_changing_active_targets(value, index):
    engine = SimulationEngine()
    previous = engine.targets
    values = [0.5, 0.2, 3.0]
    values[index] = value
    with pytest.raises(ValueError):
        engine.set_targets(*values)
    assert engine.targets == previous


def test_nonfinite_state_pauses_before_entering_controller(monkeypatch):
    engine = SimulationEngine()
    engine.plant.state = replace(engine.plant.state, vx_m_s=float('nan'))
    engine.running = True
    def forbidden(**_):
        pytest.fail('invalid state reached controller')
    monkeypatch.setattr(engine.bridge, 'step', forbidden)
    engine.step()
    assert not engine.running
    assert engine.pause_reason


class Value:
    def __init__(self, value=''):
        self.value = value
    def get(self):
        return self.value
    def set(self, value):
        self.value = value


def test_ab_job_freezes_click_time_inputs_before_thread_runs(monkeypatch, tmp_path):
    engine = SimulationEngine()
    current = {'coax.pos_x_kp': 0.9}
    monkeypatch.setattr(engine.bridge, 'parameter_snapshot', lambda: dict(current))
    root = SimpleNamespace(device=SimpleNamespace(engine=engine),
        _baseline_params={'coax.pos_x_kp': 0.5},
        _baseline_config={'kind': engine.kind, 'targets': engine.targets,
            'model': {**engine.bridge.physical_model(), 'thrust_tau_s': engine.plant.thrust_tau_s}},
        _ab_thread=None, _ab_busy=False, _closing=False,
        kind=Value(ExperimentKind.POSITION_STEP.value), ab_status=Value(),
        _ui_queue=queue.Queue())
    tasks = []
    class DeferredThread:
        def __init__(self, target, **_):
            self.target = target
            tasks.append(target)
        def start(self): pass
        def is_alive(self): return True
    monkeypatch.setattr(ui.threading, 'Thread', DeferredThread)
    captured = {}
    def record(_bridge, _kind, **kwargs):
        captured.update(kwargs)
        return object()
    monkeypatch.setattr(ui, 'run_ab', record)
    monkeypatch.setattr(ui, 'write_ab_artifact', lambda *_: (tmp_path/'result.json', tmp_path/'result.csv'))
    ui.SimulationApp._run_ab(root)
    current['coax.pos_x_kp'] = 2.0
    root._baseline_params['coax.pos_x_kp'] = 4.0
    engine.plant.thrust_tau_s = 0.7
    tasks[0]()
    assert captured['baseline_params']['coax.pos_x_kp'] == 0.5
    assert captured['tuned_params']['coax.pos_x_kp'] == 0.9
    assert captured['thrust_tau_s'] == 0.08

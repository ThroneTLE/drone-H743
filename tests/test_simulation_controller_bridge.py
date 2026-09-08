import pytest

from tools.sim_xz.controller_bridge import ControllerBridge
from tools.sim_xz.experiments import ExperimentKind, run_experiment


@pytest.fixture(scope="module")
def bridge() -> ControllerBridge:
    try:
        return ControllerBridge()
    except (RuntimeError, OSError) as exc:
        pytest.fail(f"real host C bridge unavailable: {exc}")


def test_bridge_exposes_real_runtime_parameters(bridge: ControllerBridge) -> None:
    names = bridge.parameter_names()
    assert "coax.pos_x_kp" in names
    before = bridge.get_param("coax.pos_x_kp")
    assert before is not None
    assert bridge.set_param("coax.pos_x_kp", before * 1.25)
    assert bridge.get_param("coax.pos_x_kp") == pytest.approx(before * 1.25, rel=1e-5)
    bridge.reset()


def test_parameter_change_changes_position_experiment(bridge: ControllerBridge) -> None:
    isolated = ControllerBridge(instance_tag="parameter_effect")
    isolated.reset_params()
    baseline = run_experiment(isolated, ExperimentKind.POSITION_STEP, duration_s=1.4)
    value = isolated.get_param("coax.pos_x_kp")
    assert value is not None
    assert isolated.set_param("coax.pos_x_kp", value * 1.8)
    tuned = run_experiment(isolated, ExperimentKind.POSITION_STEP, duration_s=1.4,
                           reset_controller=False)
    assert max(abs(a.x_m - b.x_m) for a, b in zip(baseline, tuned)) > 1e-4
    bridge.reset()

import pytest

from tools.sim_xz.controller_bridge import ControllerBridge, _dependency_digest
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
    # 1.4 s -> 1.6 s (step at 1.0 s): since 2026-09-27 the plant uses the real 0.035 m
    # tilt lever and default rate gains are x0.43, so pitch (hence x) reacts ~2.3x slower
    # than the old sim that overstated tilt authority; at +0.4 s the gap is only 4e-5 m.
    baseline = run_experiment(isolated, ExperimentKind.POSITION_STEP, duration_s=1.6)
    value = isolated.get_param("coax.pos_x_kp")
    assert value is not None
    assert isolated.set_param("coax.pos_x_kp", value * 1.8)
    tuned = run_experiment(isolated, ExperimentKind.POSITION_STEP, duration_s=1.6,
                           reset_controller=False)
    assert max(abs(a.x_m - b.x_m) for a, b in zip(baseline, tuned)) > 1e-4
    bridge.reset()


def test_controller_build_digest_changes_when_a_control_dependency_changes(tmp_path) -> None:
    dependency = tmp_path / "app_control_scheduler.c"
    dependency.write_text("scheduler-v1", encoding="ascii")
    first = _dependency_digest((dependency,))
    dependency.write_text("scheduler-v2", encoding="ascii")
    second = _dependency_digest((dependency,))
    assert first != second

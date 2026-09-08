from tools.sim_xz.controller_bridge import ControllerBridge
from tools.sim_xz.experiments import ExperimentKind, run_ab, run_experiment


def test_three_approved_experiments_produce_five_state_channels() -> None:
    bridge = ControllerBridge()
    for kind in ExperimentKind:
        samples = run_experiment(bridge, kind, duration_s=1.1)
        assert len(samples) > 100
        assert all(sample.z_m == sample.z_m for sample in samples)
        assert samples[-1].time_s > 1.0


def test_ab_result_contains_five_comparable_channels() -> None:
    bridge = ControllerBridge()
    params = bridge.parameter_snapshot()
    params["coax.vel_x_kp"] *= 1.5
    result = run_ab(bridge, ExperimentKind.VELOCITY_STEP,
                    tuned_params=params, duration_s=2.0)
    assert set(result.peak_abs_delta) == {"pitch_rad", "pitch_rate_rad_s", "vx_m_s", "x_m", "z_m"}
    assert any(value > 1e-5 for value in result.peak_abs_delta.values())

from tools.sim_xz.controller_bridge import ControllerOutput
from tools.sim_xz.physics import XZPlant


def command(thrust: float, tilt: float = 0.0, pitch_moment: float = 0.0) -> ControllerOutput:
    return ControllerOutput(thrust / 2, thrust / 2, tilt, 0.0,
                            (0.0, pitch_moment, 0.0), (0.0, 0.0, thrust),
                            (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))


def test_hover_does_not_accumulate_vertical_acceleration() -> None:
    plant = XZPlant()
    plant.reset(z_m=1.0, thrust_n=plant.mass_kg * plant.gravity_m_s2)
    for _ in range(500):
        plant.step(command(plant.mass_kg * plant.gravity_m_s2), 0.002)
    assert abs(plant.state.z_m - 1.0) < 0.02
    assert abs(plant.state.vz_m_s) < 0.02


def test_positive_pitch_tilt_produces_signed_x_motion_and_actuator_lag() -> None:
    plant = XZPlant()
    plant.reset(z_m=1.0)
    plant.step(command(plant.mass_kg * plant.gravity_m_s2, tilt=0.25), 0.002)
    assert abs(plant.state.pitch_tilt_rad) < 0.25
    for _ in range(300):
        plant.step(command(plant.mass_kg * plant.gravity_m_s2, tilt=0.25), 0.002)
    assert plant.state.x_m < 0.0


def test_body_thrust_is_rotated_by_actual_pitch_before_world_acceleration() -> None:
    level = XZPlant()
    pitched = XZPlant()
    hover = level.hover_thrust_n
    level.reset(z_m=1.0, thrust_n=hover)
    pitched.reset(z_m=1.0, pitch_rad=0.20, thrust_n=hover)
    level.step(command(hover), 0.001)
    pitched.step(command(hover), 0.001)
    assert abs(level.state.accel_x_m_s2) < 1e-3
    assert pitched.state.accel_x_m_s2 > 0.1


def test_positive_and_negative_actual_tilt_produce_opposite_pitch_moments() -> None:
    positive = XZPlant()
    negative = XZPlant()
    positive.reset(z_m=1.0, thrust_n=positive.hover_thrust_n)
    negative.reset(z_m=1.0, thrust_n=negative.hover_thrust_n)
    positive.step(command(positive.hover_thrust_n, tilt=0.10), 0.001)
    negative.step(command(negative.hover_thrust_n, tilt=-0.10), 0.001)
    assert positive.state.pitch_rate_rad_s > 0.0
    assert negative.state.pitch_rate_rad_s < 0.0


def test_invalid_step_is_rejected() -> None:
    try:
        XZPlant().step(command(0.0), 0.0)
    except ValueError:
        pass
    else:
        raise AssertionError("invalid dt must be rejected")

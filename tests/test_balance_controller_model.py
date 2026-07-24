from __future__ import annotations

import math

import pytest


MASS_KG = 1.367
GRAVITY_M_S2 = 9.81
INERTIA_KG_M2 = 0.051
TILT_LIMIT_RAD = math.radians(18.0)
SERVO_DELAY_S = 0.080


@pytest.mark.parametrize(
    ("effectiveness", "lever_arm_m", "kr", "kw"),
    [
        (0.581, 0.105, 0.0671, 0.1104),
        (0.569, 0.145, 0.0660, 0.1138),
    ],
)
def test_identified_attitude_gains_stabilize_with_80_ms_delay(
    effectiveness: float,
    lever_arm_m: float,
    kr: float,
    kw: float,
) -> None:
    dt_s = 0.001
    delay_steps = round(SERVO_DELAY_S / dt_s)
    delayed_tilt = [0.0] * delay_steps
    thrust_n = MASS_KG * GRAVITY_M_S2
    moment_per_sin = effectiveness * lever_arm_m * thrust_n
    theta_rad = math.radians(10.0)
    omega_rad_s = 0.0
    peak_rad = abs(theta_rad)

    for _ in range(round(8.0 / dt_s)):
        attitude_error = math.sin(theta_rad)
        moment_cmd_n_m = -kr * attitude_error - kw * omega_rad_s
        inverse_argument = max(
            -math.sin(TILT_LIMIT_RAD),
            min(math.sin(TILT_LIMIT_RAD), moment_cmd_n_m / moment_per_sin),
        )
        tilt_cmd_rad = math.asin(inverse_argument)
        delayed_tilt.append(tilt_cmd_rad)
        actual_tilt_rad = delayed_tilt.pop(0)
        actual_moment_n_m = moment_per_sin * math.sin(actual_tilt_rad)
        angular_accel_rad_s2 = actual_moment_n_m / INERTIA_KG_M2
        omega_rad_s += angular_accel_rad_s2 * dt_s
        theta_rad += omega_rad_s * dt_s
        peak_rad = max(peak_rad, abs(theta_rad))

    natural_frequency_rad_s = math.sqrt(kr / INERTIA_KG_M2)
    damping_ratio = kw / (2.0 * math.sqrt(kr * INERTIA_KG_M2))

    assert 1.10 < natural_frequency_rad_s < 1.20
    assert 0.90 < damping_ratio < 1.00
    assert peak_rad < math.radians(10.5)
    assert abs(theta_rad) < math.radians(0.1)
    assert abs(omega_rad_s) < math.radians(0.1)


def test_velocity_pi_has_critical_poles_and_bounded_step_response() -> None:
    dt_s = 0.002
    velocity_m_s = 0.0
    integral_m = 0.0
    reference_m_s = 0.8
    kv = 0.50
    ki = 0.0625
    maximum_m_s = velocity_m_s

    for _ in range(round(40.0 / dt_s)):
        error_m_s = velocity_m_s - reference_m_s
        integral_m += error_m_s * dt_s
        accel_m_s2 = -kv * error_m_s - ki * integral_m
        velocity_m_s += accel_m_s2 * dt_s
        maximum_m_s = max(maximum_m_s, velocity_m_s)

    assert math.isclose(kv, 2.0 * math.sqrt(ki), rel_tol=0.0, abs_tol=1.0e-12)
    # The poles are critically damped, but PI reference tracking adds a zero.
    # For Kv = 2*wn and Ki = wn^2, its exact step peak is 1 + exp(-2).
    expected_peak_m_s = reference_m_s * (1.0 + math.exp(-2.0))
    assert math.isclose(maximum_m_s,
                        expected_peak_m_s,
                        rel_tol=0.0,
                        abs_tol=2.0e-4)
    assert abs(velocity_m_s - reference_m_s) < 1.0e-3

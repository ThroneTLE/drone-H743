"""R-F4 seam 4 RC/actuator polarity contract."""

# Author ruling 2026-08-30: seam 4 is the same shape as seam 3 -- pin the
# polarity contract and name the frames, do not perform a representation
# migration that would cancel out at both ends.
#
# The RC lateral convention is body-right-positive, matching the controller's
# own reference frame (X forward, Y right, Z down).  Migrating RC/velocity to
# FLU left-positive would require adapting straight back to right-positive at
# the controller input: adapters on both sides, zero observable change.  The
# real cleanup waits for the controller reference frame itself, which M6
# props-off direction verification gates.
#
# So this module pins the polarity chain exactly as it stands:
#   * stick direction is decided in exactly one place,
#   * measurement polarity is a named constant, not a folded-in gain,
#   * mechanical servo polarity is applied after allocation, from the runtime
#     ServoCalibration, never inside the control law.
#
# It deliberately asserts no physical roll/yaw stick direction: only the pitch
# constant carries a bench confirmation in-tree, and the full direction table
# is M6's deliverable, not a host test's.

from __future__ import annotations

from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
STABILIZER = ROOT / "App" / "Src" / "app_stabilizer.c"
RC_HEADER = ROOT / "App" / "Inc" / "app_rc_config.h"
CTRL_SOURCE = ROOT / "Driver" / "Src" / "drv_coax_ctrl.c"


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_stick_direction_has_exactly_one_decision_point() -> None:
    """Both RC attitude sign constants, and nothing else, set stick direction."""
    source = read(STABILIZER)
    assert "#define STABILIZER_RC_ATTITUDE_TARGET_PITCH_SIGN (-1.0f)" in source
    assert "#define STABILIZER_RC_ATTITUDE_TARGET_ROLL_SIGN  (-1.0f)" in source
    # Each is consumed exactly once: at the RC -> target attitude conversion.
    assert source.count("STABILIZER_RC_ATTITUDE_TARGET_PITCH_SIGN") == 2
    assert source.count("STABILIZER_RC_ATTITUDE_TARGET_ROLL_SIGN") == 2
    # The bench confirmation that fixed the pitch polarity must stay recorded.
    assert "实机确认：原来 PITCH_SIGN = +1 时前推变成后倾，故取 -1。" in source


def test_velocity_measurement_polarity_is_a_named_constant() -> None:
    """Flow/fusion lateral polarity must not be folded into a gain."""
    source = read(STABILIZER)
    assert "#define STABILIZER_VELOCITY_MEAS_Y_SIGN (1.0f)" in source
    assert "机体系右正" in source


def test_yaw_stick_maps_to_a_rate_reference_without_a_hidden_sign() -> None:
    """Yaw intent is a pure scale; any polarity change must be visible."""
    source = read(STABILIZER)
    assert "return yaw_norm * STABILIZER_YAW_RATE_REF_MAX_RAD_S;" in source


def test_servo_mechanical_polarity_is_applied_after_allocation() -> None:
    """pulse_sign is mechanical, comes from runtime calibration, and stays last.

    drv_coax_ctrl.h: 极性活在分配之后，绝不进负增益.  The M4 bench values
    (alpha/beta pulse_sign = -1) live in the persisted ServoCalibration, so a
    compile-time sign macro must never reappear here.
    """
    source = read(CTRL_SOURCE)
    # Allocation first (body tilt -> per-servo tilt), mechanical sign after.
    allocation = source.index("coax_ctrl_body_tilt_to_servo_tilts(body_x_tilt_rad,")
    alpha_sign = source.index("coax_ctrl_servo_calibration.pulse_sign[", allocation)
    assert allocation < alpha_sign
    assert "servo_alpha_tilt_rad *" in source
    assert "servo_beta_tilt_rad *" in source


@pytest.mark.xfail(
    strict=True,
    reason="R-F4 red test: app_rc_config.h documents norm[] only as "
    "'[-1,+1] 已反向/死区/标定' and never states what +1 means physically or "
    "that 'reversed' is transmitter-side only; cleared by the seam 4 "
    "implementation commit",
)
def test_rc_header_names_the_intent_frame() -> None:
    header = read(RC_HEADER)
    # norm[] must be identified as stick space, not a body-frame vector.
    assert "摇杆空间" in header
    # The single decision point must be named from here.
    assert "STABILIZER_RC_ATTITUDE_TARGET_PITCH_SIGN" in header
    # reversed must be fenced off from body-frame corrections.
    assert "禁止用它补偿机体坐标系" in header

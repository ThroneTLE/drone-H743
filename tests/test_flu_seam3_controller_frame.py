"""R-F3 seam 3 controller FLU boundary contract."""

# drv_coax_ctrl.h still declares "IMU axes are already rotated to body FRD
# before this layer" and describes its reference frame as "X is forward, Y is
# right".  Neither matches what the runtime actually feeds it:
#
#   * attitude angles and body rates arrive in canonical FLU (seam 0/1),
#   * position/velocity Z arrives already negated to down-positive
#     (app_stabilizer.c: z_m = -relative_height_m, vz_m_s =
#     -range_velocity_m_s),
#   * the FLU -> internal force-frame conversion is real but lives in
#     DRV_COAX_CTRL_FORCE_FRAME_* / DRV_COAX_CTRL_RATE_FRAME_* constants
#     inside the driver, not at a named boundary.
#
# So the controller is a deliberate hybrid, not an unadapted FRD block.  Seam 3
# makes that contract explicit and pins the constants so the polarity cannot
# drift silently, without changing a single output: servo pulse widths and
# motor commands stay bit-identical.
#
# Whether those constants are numerically right for the airframe is an M6
# bench question (props-off direction verification), not a host-test question.
# This module deliberately does not assert their correctness -- only that they
# exist, are isolated, keep gains positive, and are applied symmetrically to
# actual and target attitude.
#
# Symmetry is NOT cancellation: the attitude error is the SO(3) expression
# 0.5*vee(R_d^T R_a - R_a^T R_d), so flipping one Euler argument on both sides
# is not a similarity transform and does not drop out.  Measured, executable
# counter-evidence lives in tests/test_flu_seam3_force_frame_derivation.py.

from __future__ import annotations

from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[1]
CTRL_SOURCE = ROOT / "Driver" / "Src" / "drv_coax_ctrl.c"
CTRL_HEADER = ROOT / "Driver" / "Inc" / "drv_coax_ctrl.h"
STABILIZER = ROOT / "App" / "Src" / "app_stabilizer.c"


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_frame_signs_and_gains() -> None:
    """Polarity lives in named frame constants, never in a gain.

    The force-frame sign must also be applied to *both* the measured and the
    target attitude.  One-sided application would bias the attitude error
    outright; symmetric application keeps the loop self-consistent, but the
    sign still changes the output -- it is load-bearing, not inert.
    """
    source = read(CTRL_SOURCE)
    assert "#define DRV_COAX_CTRL_FORCE_FRAME_ROLL_SIGN  (-1.0f)" in source
    assert "#define DRV_COAX_CTRL_FORCE_FRAME_PITCH_SIGN (1.0f)" in source
    assert "#define DRV_COAX_CTRL_RATE_FRAME_ROLL_SIGN   (1.0f)" in source
    assert "#define DRV_COAX_CTRL_RATE_FRAME_PITCH_SIGN  (1.0f)" in source

    # Cascade gains are non-negative; negative feedback is structural.
    attitude = read(ROOT / "Driver/Src/drv_attitude_control.c")
    rate = read(ROOT / "Driver/Src/drv_rate_control.c")
    assert "params->att_kp[axis] < 0.0f" in attitude
    assert "params->kp[axis] < 0.0f" in rate
    assert "output->omega_ff[axis] -" in attitude
    assert "input->omega_sp[axis] - input->omega[axis]" in rate

    assert re.search(
        r"DRV_COAX_CTRL_FORCE_FRAME_ROLL_SIGN \* attitude->roll_rad", source
    ), "force-frame roll sign is no longer applied to measured attitude"
    assert re.search(
        r"DRV_COAX_CTRL_FORCE_FRAME_ROLL_SIGN \* target_roll_rad", source
    ), "force-frame roll sign is no longer applied to target attitude"


def test_mount_outside_control_law() -> None:
    """The 90 deg mount rotation belongs to allocation, after the control law."""
    source = read(CTRL_SOURCE)
    assert "*servo_alpha_tilt_rad = -body_y_tilt_rad;" in source
    assert "*servo_beta_tilt_rad = -body_x_tilt_rad;" in source


def test_stabilizer_feed_pinned() -> None:
    """Pin the mixed feed exactly, so seam 4 cannot change it unnoticed."""
    source = read(STABILIZER)
    # FLU attitude and rates, unconverted at the call site.
    assert "frame->attitude.roll_rad = ctx->roll_control * STABILIZER_DEG_TO_RAD;" in source
    assert "frame->attitude.gyro_x_rad_s = ctx->last_msg.imu.gyro_x_dps" in source
    # Altitude channel is explicitly converted to down-positive.
    assert "frame->attitude.z_m = -frame->relative_height_m;" in source
    assert "frame->attitude.vz_m_s = -frame->range_velocity_m_s;" in source


def test_header_input_contract() -> None:
    header = read(CTRL_HEADER)
    assert "IMU axes are already rotated to body FRD before this layer." not in header
    # The header must name what actually arrives, and where polarity lives.
    assert "canonical FLU" in header
    assert "DRV_COAX_CTRL_FORCE_FRAME_ROLL_SIGN" in header

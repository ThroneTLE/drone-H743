"""R-F4 seam 4 RC/actuator polarity contract."""

# Author ruling 2026-08-30: seam 4 is the same shape as seam 3 -- pin the
# polarity contract and name the frames, do not perform a representation
# migration that would cancel out at both ends.
#
# The RC lateral convention is still body-right-positive, while the seam 3
# controller interface is now canonical FLU left-positive.  Deciding and
# adapting RC intent is seam 4 work and remains gated by M6 props-off direction
# verification; R-F6-2 only adapts the navigation measurement boundary.
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
import re
import shutil
import subprocess

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
    assert "#define STABILIZER_RC_ATTITUDE_TARGET_ROLL_SIGN  (1.0f)" in source
    # Each is consumed exactly once: at the RC -> target attitude conversion.
    assert source.count("STABILIZER_RC_ATTITUDE_TARGET_PITCH_SIGN") == 2
    assert source.count("STABILIZER_RC_ATTITUDE_TARGET_ROLL_SIGN") == 2
    # The bench confirmation that fixed the pitch polarity must stay recorded.
    assert "实机确认：原来 PITCH_SIGN = +1 时前推变成后倾，故取 -1。" in source


def test_left_stick_reaches_the_flu_controller_with_left_positive_intent(
    tmp_path: Path,
) -> None:
    """Author ruling: CH1 left means vy>0 and roll_target<0 in canonical FLU.

    The harness compiles the real cascade controller.  Values for both App
    adapters are extracted from the production source so the test cannot pass
    with a hand-copied opposite sign.
    """
    gcc = shutil.which("gcc")
    if gcc is None:
        pytest.skip("host gcc is unavailable")

    source = read(STABILIZER)
    velocity_sign = re.search(
        r"#define STABILIZER_RC_VELOCITY_Y_TO_FLU_SIGN \((-?1\.0f)\)",
        source,
    )
    roll_sign = re.search(
        r"#define STABILIZER_RC_ATTITUDE_TARGET_ROLL_SIGN\s+\((-?1\.0f)\)",
        source,
    )
    assert velocity_sign and velocity_sign.group(1) == "-1.0f"
    assert roll_sign and roll_sign.group(1) == "1.0f"

    stub = tmp_path / "stub"
    stub.mkdir()
    (stub / "bsp_pwm.h").write_text(
        "#ifndef BSP_PWM_H\n#define BSP_PWM_H\n"
        "#define BSP_PWM_ESC_MIN_US 1000U\n"
        "#define BSP_PWM_ESC_MAX_US 2000U\n#endif\n",
        encoding="ascii",
    )
    harness = tmp_path / "harness.c"
    harness.write_text(
        r'''#include "drv_coax_ctrl.h"
#include <stdio.h>
#include <string.h>

int main(void) {
  DRV_COAX_CTRL_AttitudeInput att;
  DRV_COAX_CTRL_Reference ref;
  DRV_COAX_CTRL_Output out;
  DRV_COAX_CTRL_Debug debug;
  DRV_COAX_CTRL_Params params;
  const float left_stick = -1.0f; /* APP_RcInputs CH1: right-positive. */
  memset(&att, 0, sizeof(att));
  memset(&ref, 0, sizeof(ref));
  DRV_COAX_CTRL_Init();
  DRV_COAX_CTRL_GetDefaultParams(&params);
  params.vel_loop_enable = 1.0f;
  DRV_COAX_CTRL_SetParams(&params);

  ref.dt_sec = 0.002f;
  ref.horizontal_velocity_valid = 1U;
  ref.navigation_velocity_valid = 1U;
  ref.position_control_bypass = 1U;
  ref.vy_m_s = left_stick * 0.40f * VELOCITY_Y_SIGN;
  DRV_COAX_CTRL_Run(&att, &ref, &out);
  DRV_COAX_CTRL_GetLastDebug(&debug);
  printf("velocity %.6f %.6f %.9f\n", (double)ref.vy_m_s,
         (double)debug.target_attitude_rp_rad[0],
         (double)debug.moment_cmd_n_m[0]);

  DRV_COAX_CTRL_ResetState();
  memset(&ref, 0, sizeof(ref));
  ref.direct_attitude_target_valid = 1U;
  ref.manual_total_force_valid = 1U;
  ref.manual_total_force_n = 7.84f;
  ref.target_roll_rad = left_stick * 0.20f * ROLL_TARGET_SIGN;
  DRV_COAX_CTRL_Run(&att, &ref, &out);
  DRV_COAX_CTRL_GetLastDebug(&debug);
  printf("attitude %.6f %.9f\n", (double)ref.target_roll_rad,
         (double)debug.moment_cmd_n_m[0]);
  return 0;
}
''',
        encoding="ascii",
    )
    executable = tmp_path / "harness.exe"
    subprocess.run(
        [gcc, "-std=c11", "-O1", "-DVELOCITY_Y_SIGN=-1.0f",
         "-DROLL_TARGET_SIGN=1.0f", f"-I{stub}",
         f"-I{ROOT / 'Driver' / 'Inc'}",
         str(ROOT / "Driver" / "Src" / "drv_coax_ctrl.c"),
         str(ROOT / "Driver" / "Src" / "drv_position_control.c"),
         str(ROOT / "Driver" / "Src" / "drv_attitude_control.c"),
         str(ROOT / "Driver" / "Src" / "drv_rate_control.c"),
         str(harness), "-lm", "-o", str(executable)],
        check=True, capture_output=True, text=True,
    )
    lines = subprocess.run(
        [str(executable)], check=True, capture_output=True, text=True,
    ).stdout.strip().splitlines()
    _, vy_ref, velocity_roll, velocity_moment = lines[0].split()
    _, target_roll, attitude_moment = lines[1].split()
    assert float(vy_ref) > 0.0, "左打必须成为 FLU +Y（左）速度意图"
    assert float(velocity_roll) < 0.0, "向左速度意图必须要求左翼下沉"
    assert float(velocity_moment) < 0.0, "左翼下沉的初始滚转力矩必须为负"
    assert float(target_roll) < 0.0, "左打直接姿态必须要求左翼下沉"
    assert float(attitude_moment) < 0.0, "左翼下沉的初始滚转力矩必须为负"


def test_velocity_measurement_polarity_is_a_named_constant() -> None:
    """Flow/fusion lateral polarity must not be folded into a gain."""
    source = read(STABILIZER)
    assert "#define STABILIZER_VELOCITY_MEAS_Y_SIGN (-1.0f)" in source
    assert "seam2(svc_flow_nav)→seam3(drv_coax_ctrl)" in source


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


def test_rc_header_names_the_intent_frame() -> None:
    header = read(RC_HEADER)
    # norm[] must be identified as stick space, not a body-frame vector.
    assert "摇杆空间" in header
    # The single decision point must be named from here.
    assert "STABILIZER_RC_ATTITUDE_TARGET_PITCH_SIGN" in header
    # reversed must be fenced off from body-frame corrections.
    assert "禁止用它补偿机体坐标系" in header

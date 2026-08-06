"""Sign-convention self-check for the coaxial attitude controller.

Why this file exists: the sensor-to-servo chain carries several independent sign
switches. Their effects mask each other, because a negative gain is
mathematically the same as flipping a sign — so a polarity error can be absorbed
by "tuning the gain negative". The aircraft then self-levels correctly while the
stick response is reversed, which is exactly the failure that kept recurring.

Self-levelling only proves negative feedback. It does not prove absolute
direction. These tests run the real controller and pin both properties:

  * restoring direction  — a disturbance must produce an opposing moment
  * absolute direction   — a stick command must tilt the aircraft the way the
                           pilot expects, and opposite to the restoring case

Together they leave only one self-consistent combination of signs.
"""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


# ── static checks: the convention must stay centralised and gains positive ──

def test_gains_are_positive_so_polarity_errors_cannot_be_masked() -> None:
    source = read("Driver/Src/drv_coax_ctrl.c")

    # Defaults must be positive; a stored negative gain used to be how a sign
    # error was hidden behind an apparently stable aircraft.
    for line in ("params->roll_angle_kp = 0.0671f;",
                 "params->pitch_angle_kp = 0.0660f;",
                 "params->roll_rate_kd = 0.1104f;",
                 "params->pitch_rate_kd = 0.1138f;",
                 "params->yaw_angle_kp = 1.0f;",
                 "params->yaw_rate_kd = 0.15f;"):
        assert line in source, line

    # Magnitude is taken at use, so a negative value entered from the UI cannot
    # silently invert the feedback direction.
    for expr in ("fabsf(coax_ctrl_params.roll_angle_kp)",
                 "fabsf(coax_ctrl_params.pitch_angle_kp)",
                 "fabsf(coax_ctrl_params.roll_rate_kd)",
                 "fabsf(coax_ctrl_params.pitch_rate_kd)"):
        assert expr in source, expr


def test_control_law_is_negative_feedback_by_structure() -> None:
    source = read("Driver/Src/drv_coax_ctrl.c")

    # Both terms subtract. Previously kr was negated and kd was used raw, i.e.
    # the two gain types had opposite sign conventions.
    assert "(-kr_roll * solution->attitude_error[0]) -" in source
    assert "(-kr_pitch * solution->attitude_error[1]) -" in source

    # The rate term must be subtracted, not added. Check the operator that
    # precedes it rather than what follows, since the gyroscopic term is
    # legitimately added after it.
    assert "attitude_error[0]) -\n        (kd_roll" in source
    assert "attitude_error[1]) -\n        (kd_pitch" in source


def test_stick_polarity_lives_in_exactly_one_place() -> None:
    freertos = read("Core/Src/freertos.c")

    # The stick mapping is the only sign that changes pilot-facing direction;
    # every other sign applies to measured and target attitude alike and
    # therefore cancels in the attitude error.
    assert "#define STABILIZER_RC_ATTITUDE_TARGET_PITCH_SIGN (-1.0f)" in freertos
    assert "#define STABILIZER_RC_ATTITUDE_TARGET_ROLL_SIGN  (-1.0f)" in freertos
    assert freertos.count("STABILIZER_RC_ATTITUDE_TARGET_PITCH_SIGN") == 2
    assert freertos.count("STABILIZER_RC_ATTITUDE_TARGET_ROLL_SIGN") == 2


def test_sign_convention_is_documented_in_one_block() -> None:
    source = read("Driver/Src/drv_coax_ctrl.c")

    header_at = source.index("极性约定（唯一声明处）")
    for name in ("DRV_COAX_CTRL_FORCE_FRAME_ROLL_SIGN",
                 "DRV_COAX_CTRL_FORCE_FRAME_PITCH_SIGN",
                 "DRV_COAX_CTRL_RATE_FRAME_ROLL_SIGN",
                 "DRV_COAX_CTRL_RATE_FRAME_PITCH_SIGN",
                 "DRV_COAX_CTRL_SERVO_ALPHA_SIGN",
                 "DRV_COAX_CTRL_SERVO_BETA_SIGN"):
        first = source.index(f"#define {name}")
        assert first > header_at, f"{name} must be declared inside the block"


# ── runtime checks: physical direction, using the real controller ──

SIGN_HARNESS = r"""
#include "drv_coax_ctrl.h"
#include "drv_airframe_model.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define CHECK(cond, code) do { if (!(cond)) { \
    printf("FAIL %d\n", (code)); return (code); } } while (0)

static void base_state(DRV_COAX_CTRL_AttitudeInput *att,
                       DRV_COAX_CTRL_Reference *ref)
{
    memset(att, 0, sizeof(*att));
    memset(ref, 0, sizeof(*ref));
    /* Hover-ish: direct attitude mode with manual thrust holding weight. */
    ref->direct_attitude_target_valid = 1U;
    ref->manual_total_force_valid = 1U;
    ref->manual_total_force_n = DRV_AIRFRAME_MASS_KG * DRV_AIRFRAME_GRAVITY_M_S2;
}

int main(void)
{
    DRV_COAX_CTRL_AttitudeInput att;
    DRV_COAX_CTRL_Reference ref;
    DRV_COAX_CTRL_Output out;
    DRV_COAX_CTRL_Params params;
    float level_alpha, level_beta;
    float nose_up_alpha, right_down_beta;
    float stick_fwd_alpha, stick_right_beta;

    DRV_COAX_CTRL_Init();
    DRV_COAX_CTRL_GetDefaultParams(&params);

    /* Every gain must be positive in the shipped defaults. */
    CHECK(params.roll_angle_kp > 0.0f, 1);
    CHECK(params.pitch_angle_kp > 0.0f, 2);
    CHECK(params.roll_rate_kd > 0.0f, 3);
    CHECK(params.pitch_rate_kd > 0.0f, 4);

    DRV_COAX_CTRL_SetParams(&params);

    /* Reference: perfectly level, no command. */
    base_state(&att, &ref);
    DRV_COAX_CTRL_Run(&att, &ref, &out);
    level_alpha = out.alpha_rad;
    level_beta = out.beta_rad;
    CHECK(fabsf(level_alpha) < 1.0e-3f, 5);
    CHECK(fabsf(level_beta) < 1.0e-3f, 6);

    /*
     * A. RESTORING DIRECTION.
     * Disturb pitch nose-up with no stick input. The controller must tilt the
     * thrust vector so as to push the nose back down, i.e. away from level in a
     * specific direction. We only require a definite, repeatable sign here.
     */
    base_state(&att, &ref);
    att.pitch_rad = 0.15f;              /* nose up */
    DRV_COAX_CTRL_Run(&att, &ref, &out);
    nose_up_alpha = out.alpha_rad;
    CHECK(fabsf(nose_up_alpha) > 1.0e-3f, 7);

    /* Mirror disturbance must give the mirrored response: no even-order bug. */
    base_state(&att, &ref);
    att.pitch_rad = -0.15f;
    DRV_COAX_CTRL_Run(&att, &ref, &out);
    CHECK(nose_up_alpha * out.alpha_rad < 0.0f, 8);

    base_state(&att, &ref);
    att.roll_rad = 0.15f;               /* right side down */
    DRV_COAX_CTRL_Run(&att, &ref, &out);
    right_down_beta = out.beta_rad;
    CHECK(fabsf(right_down_beta) > 1.0e-3f, 9);

    base_state(&att, &ref);
    att.roll_rad = -0.15f;
    DRV_COAX_CTRL_Run(&att, &ref, &out);
    CHECK(right_down_beta * out.beta_rad < 0.0f, 10);

    /*
     * B. ABSOLUTE (STICK) DIRECTION.
     * Commanding nose-down from level must tilt the thrust the SAME way as
     * being disturbed nose-up does: both cases want the nose to go down. If
     * these two agree the loop is consistent; if they oppose, the aircraft
     * self-levels but flies backwards to the stick -- the original bug.
     */
    base_state(&att, &ref);
    ref.target_pitch_rad = -0.15f;      /* commanded nose down */
    DRV_COAX_CTRL_Run(&att, &ref, &out);
    stick_fwd_alpha = out.alpha_rad;
    CHECK(fabsf(stick_fwd_alpha) > 1.0e-3f, 11);
    CHECK(stick_fwd_alpha * nose_up_alpha > 0.0f, 12);

    base_state(&att, &ref);
    ref.target_roll_rad = 0.15f;        /* commanded right side down */
    DRV_COAX_CTRL_Run(&att, &ref, &out);
    stick_right_beta = out.beta_rad;
    CHECK(fabsf(stick_right_beta) > 1.0e-3f, 13);
    /* Commanding right-down is the opposite of correcting right-down. */
    CHECK(stick_right_beta * right_down_beta < 0.0f, 14);

    /*
     * C. RATE DAMPING must oppose the rate, not reinforce it. With the angle
     * gain zeroed, a positive body rate must produce a moment of the opposite
     * sign; a sign slip here shows up as growing oscillation in flight.
     */
    DRV_COAX_CTRL_GetDefaultParams(&params);
    params.roll_angle_kp = 0.0f;
    params.pitch_angle_kp = 0.0f;
    DRV_COAX_CTRL_SetParams(&params);

    base_state(&att, &ref);
    att.gyro_y_rad_s = 0.5f;
    DRV_COAX_CTRL_Run(&att, &ref, &out);
    {
        DRV_COAX_CTRL_Debug debug;
        DRV_COAX_CTRL_GetLastDebug(&debug);
        /* Positive pitch rate -> damping moment must be negative. */
        CHECK(debug.moment_cmd_n_m[1] < 0.0f, 15);
    }
    base_state(&att, &ref);
    att.gyro_x_rad_s = 0.5f;
    DRV_COAX_CTRL_Run(&att, &ref, &out);
    {
        DRV_COAX_CTRL_Debug debug;
        DRV_COAX_CTRL_GetLastDebug(&debug);
        CHECK(debug.moment_cmd_n_m[0] < 0.0f, 16);
    }

    /*
     * D. A NEGATIVE GAIN MUST NOT INVERT ANYTHING. This is the property that
     * makes the whole scheme trustworthy: entering -Kp can no longer be used to
     * paper over a polarity error. A negative value is rejected at the setter,
     * so the controller falls back to the (positive) defaults and the restoring
     * direction is unchanged.
     */
    DRV_COAX_CTRL_GetDefaultParams(&params);
    params.pitch_angle_kp = -params.pitch_angle_kp;
    DRV_COAX_CTRL_SetParams(&params);
    DRV_COAX_CTRL_GetParams(&params);
    CHECK(params.pitch_angle_kp > 0.0f, 17);

    base_state(&att, &ref);
    att.pitch_rad = 0.15f;
    DRV_COAX_CTRL_Run(&att, &ref, &out);
    CHECK(out.alpha_rad * nose_up_alpha > 0.0f, 18);

    /* Named-parameter entry must reject a negative gain too. */
    CHECK(DRV_COAX_CTRL_SetParam("coax.pitch_angle_kp", -0.5f) == 0U, 19);
    CHECK(DRV_COAX_CTRL_SetParam("coax.pitch_angle_kp", 0.5f) != 0U, 20);

    printf("ok\n");
    return 0;
}
"""


def test_controller_sign_convention_runtime(tmp_path: Path) -> None:
    gcc = shutil.which("gcc")
    if gcc is None:
        pytest.skip("host gcc is unavailable")

    stub_dir = tmp_path / "stub"
    stub_dir.mkdir()
    (stub_dir / "bsp_pwm.h").write_text(
        "#ifndef BSP_PWM_H\n#define BSP_PWM_H\n"
        "#define BSP_PWM_ESC_MIN_US 1000U\n"
        "#define BSP_PWM_ESC_MAX_US 2000U\n#endif\n",
        encoding="ascii",
    )

    harness = tmp_path / "sign_harness.c"
    harness.write_text(SIGN_HARNESS, encoding="ascii")
    executable = tmp_path / "sign_harness.exe"

    subprocess.run(
        [gcc, "-std=c11", "-Wall", "-Wextra", "-Werror",
         f"-I{stub_dir}", f"-I{ROOT / 'Driver' / 'Inc'}",
         str(ROOT / "Driver" / "Src" / "drv_coax_ctrl.c"),
         str(harness), "-lm", "-o", str(executable)],
        check=True, capture_output=True, text=True)

    result = subprocess.run([str(executable)], capture_output=True, text=True)
    assert result.returncode == 0, f"sign check failed: {result.stdout.strip()}"

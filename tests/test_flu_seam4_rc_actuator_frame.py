"""R-F4 seam 4 RC/actuator polarity contract."""

# Seam 4 is now a real migration, not just a naming exercise: stick intent is
# converted to canonical FLU in App/Src/app_rc_intent.c and nothing downstream
# carries a transmitter sign.
#
# What this module pins:
#   * stick direction is decided in exactly one place,
#   * every one of those five signs is *derivable* -- calibration wizard prompt
#     plus drv_frame_contract.h -- so none of them is a bench-tuned constant,
#   * no measurement sign survives downstream of the sensor Adapter,
#   * mechanical servo polarity is applied after allocation, from the runtime
#     ServoCalibration, never inside the control law.
#
# The one thing a host test still cannot settle is which way the airframe
# actually yaws: that depends on rotor handedness and differential thrust, which
# no calibration record covers. It stays a props-off observation.

from __future__ import annotations

from pathlib import Path
import shutil
import subprocess

import pytest


from _airframe_fixture import AIRFRAME_FIXTURE_C, AIRFRAME_SOURCE


ROOT = Path(__file__).resolve().parents[1]
STABILIZER = ROOT / "App" / "Src" / "app_stabilizer.c"
RC_HEADER = ROOT / "App" / "Inc" / "app_rc_config.h"
CTRL_SOURCE = ROOT / "Driver" / "Src" / "drv_coax_ctrl.c"
RC_INTENT_SOURCE = ROOT / "App" / "Src" / "app_rc_intent.c"


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_stick_direction_has_exactly_one_decision_point() -> None:
    """Pilot-space signs live only in the canonical intent Adapter."""
    source = read(STABILIZER)
    intent = read(RC_INTENT_SOURCE)
    assert "STABILIZER_RC_ATTITUDE_TARGET_PITCH_SIGN" not in source
    assert "STABILIZER_RC_ATTITUDE_TARGET_ROLL_SIGN" not in source
    assert "STABILIZER_RC_VELOCITY_Y_TO_FLU_SIGN" not in source
    assert "return -roll_norm * limit_m_s;" in intent
    assert "return pitch_norm * limit_rad;" in intent
    assert "return roll_norm * limit_rad;" in intent
    assert "return -yaw_norm * limit_rad_s;" in intent


def test_stick_positive_direction_still_comes_from_the_calibration_wizard() -> None:
    """摇杆 +1 的物理含义由标定向导定义，代码只能跟着它推。

    app_rc_intent 的五个符号全部建立在"norm>0 = 前 / 右 / 右转"之上。这个前提
    不在固件里，而在上位机向导的提示语里。谁改了向导的方向，就必须回来重推
    这五个符号——所以把它钉在这里，别让两边悄悄走散。
    """
    wizard = read(ROOT / "tools" / "panel_lib" / "pages" / "rc_wizard.py")
    steps = wizard.split("RC_WIZARD_STEPS", 1)[1].split(")\n\n", 1)[0]
    for function, direction in (("pitch", "向前推到底"),
                                ("roll", "向右推到底"),
                                ("yaw", "向右转到底")):
        marker = f'("{function}", +1,'
        assert marker in steps, marker
        prompt = steps.split(marker, 1)[1].split("\n", 1)[0]
        assert direction in prompt, f"{function} 的 +1 方向提示语已改：{prompt}"


def test_every_intent_sign_is_derivable_not_bench_tuned(tmp_path: Path) -> None:
    """五条摇杆→FLU 的符号，逐条按契约核对。

    这条测试替代了以前那种"某个 #define 等于 -1"的写法：那种断言只能证明有人
    写过这个值，证明不了值是对的。这里断言的是可推导的结论本身。
    """
    gcc = shutil.which("gcc")
    if gcc is None:
        pytest.skip("host gcc is unavailable")

    harness = tmp_path / "intent.c"
    harness.write_text(
        r'''#include "app_rc_intent.h"
#include <stdio.h>

int main(void) {
  const float s = 1.0f;   /* wizard: +1 = forward / right / right-turn */
  printf("%.6f %.6f %.6f %.6f %.6f\n",
         (double)APP_RcIntent_ForwardVelocity(s, 1.0f),
         (double)APP_RcIntent_LeftVelocity(s, 1.0f),
         (double)APP_RcIntent_TargetPitch(s, 1.0f),
         (double)APP_RcIntent_TargetRoll(s, 1.0f),
         (double)APP_RcIntent_YawRateLeft(s, 1.0f));
  return 0;
}
''',
        encoding="ascii",
    )
    executable = tmp_path / "intent.exe"
    subprocess.run(
        [gcc, "-std=c11", "-Wall", "-Wextra", "-Werror",
         f"-I{ROOT / 'App' / 'Inc'}",
         str(RC_INTENT_SOURCE), str(harness), "-lm", "-o", str(executable)],
        check=True, capture_output=True, text=True,
    )
    forward, left, pitch, roll, yaw = (
        float(v) for v in subprocess.run(
            [str(executable)], check=True, capture_output=True, text=True,
        ).stdout.split()
    )

    assert forward > 0.0, "前推 → +X 前；FLU +X 就是前，同号"
    assert left < 0.0, "右打 → 向右；FLU +Y 是左，向右为负"
    assert pitch > 0.0, "前推 → 机头下俯；FLU +pitch 的定义就是机头下俯"
    assert roll > 0.0, "右打 → 右翼下沉；FLU +roll 的定义就是右翼下沉"
    assert yaw < 0.0, "右转 → 机头向右；FLU +yaw 是机头向左，向右为负"


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
        AIRFRAME_FIXTURE_C + r'''#include "drv_coax_ctrl.h"
#include "app_rc_intent.h"
#include <stdio.h>
#include <string.h>

int main(void) {
  airframe_load_reference();
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
  ref.vy_m_s = APP_RcIntent_LeftVelocity(left_stick, 0.40f);
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
  ref.target_roll_rad = APP_RcIntent_TargetRoll(left_stick, 0.20f);
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
        [gcc, "-std=c11", "-O1", f"-I{stub}",
         f"-I{ROOT / 'App' / 'Inc'}",
         f"-I{ROOT / 'Driver' / 'Inc'}",
         str(RC_INTENT_SOURCE),
         str(AIRFRAME_SOURCE),
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


def test_velocity_measurement_is_already_flu_before_the_controller() -> None:
    """No measurement sign is allowed downstream of the sensor Adapter."""
    source = read(STABILIZER)
    assert "STABILIZER_VELOCITY_MEAS_Y_SIGN" not in source
    assert "velocity_control_y_m_s = nav_vy_m_s;" in source


def test_yaw_stick_maps_to_a_rate_reference_without_a_hidden_sign() -> None:
    """Yaw pilot convention is converted by the same named Adapter."""
    source = read(STABILIZER)
    assert "APP_RcIntent_YawRateLeft(yaw_norm," in source


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
    assert "app_rc_intent Module" in header
    # reversed must be fenced off from body-frame corrections.
    assert "禁止用它补偿机体坐标系" in header

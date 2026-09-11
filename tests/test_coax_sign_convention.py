"""Sign-convention self-check for the coaxial attitude controller.

This is a legacy runtime-adapter test, not the canonical body-frame definition.
The sole canonical contract is Driver/Inc/drv_frame_contract.h; this file remains
until the complete sensor-to-actuator runtime chain is migrated to FLU.

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

from _airframe_fixture import AIRFRAME_FIXTURE_C, AIRFRAME_SOURCE


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


# ── static checks: the convention must stay centralised and gains positive ──

def test_gains_are_positive_so_polarity_errors_cannot_be_masked() -> None:
    source = read("Driver/Src/drv_coax_ctrl.c")

    # Defaults must be positive; a stored negative gain used to be how a sign
    # error was hidden behind an apparently stable aircraft.
    for line in ("params->attitude.att_kp[0] = 0.0671f / 0.1104f;",
                 "params->attitude.att_kp[1] = 0.0660f / 0.1138f;",
                 "params->rate.kp[0] = 0.1104f;",
                 "params->rate.kp[1] = 0.1138f;",
                 # 2026-09-11：I_zz 从编译期常量改为机体模型运行时取值。系数 0.525
                 # 是内环带宽，仍然是正的——换的是数据来源，不是符号约定。
                 "params->rate.kp[2] = DRV_Airframe_Get()->izz_kgm2 * 0.525f;"):
        assert line in source, line

    # Magnitude is taken at use, so a negative value entered from the UI cannot
    # silently invert the feedback direction.
    assert "return (value >= 0.0f) ? 1U : 0U;" in source


def test_control_law_is_negative_feedback_by_structure() -> None:
    attitude = read("Driver/Src/drv_attitude_control.c")
    rate = read("Driver/Src/drv_rate_control.c")

    # Both terms subtract. Previously kr was negated and kd was used raw, i.e.
    # the two gain types had opposite sign conventions.
    assert "output->omega_ff[axis] -" in attitude
    assert "input->omega_sp[axis] - input->omega[axis]" in rate
    assert "p_term + output->i_term[axis] - d_term" in rate


def test_stick_polarity_lives_in_exactly_one_place() -> None:
    stabilizer = read("App/Src/app_stabilizer.c")
    intent = read("App/Src/app_rc_intent.c")
    assert '#include "app_rc_intent.h"' in stabilizer
    assert "STABILIZER_RC_ATTITUDE_TARGET_PITCH_SIGN" not in stabilizer
    assert "STABILIZER_RC_ATTITUDE_TARGET_ROLL_SIGN" not in stabilizer
    for function in ("APP_RcIntent_ForwardVelocity", "APP_RcIntent_LeftVelocity",
                     "APP_RcIntent_TargetPitch", "APP_RcIntent_TargetRoll",
                     "APP_RcIntent_YawRateLeft"):
        assert function in intent


def test_sign_convention_is_documented_in_one_block() -> None:
    source = read("Driver/Src/drv_coax_ctrl.c")

    # R-F6-2 (2026-09-06): once both position/velocity and attitude/rate are
    # genuinely FLU, the standard rotation-matrix formula needs no per-axis
    # sign compensation, so these four constants are deleted, not re-valued.
    assert "极性约定（唯一声明处）" in source
    for name in ("DRV_COAX_CTRL_FORCE_FRAME_ROLL_SIGN",
                 "DRV_COAX_CTRL_FORCE_FRAME_PITCH_SIGN",
                 "DRV_COAX_CTRL_RATE_FRAME_ROLL_SIGN",
                 "DRV_COAX_CTRL_RATE_FRAME_PITCH_SIGN"):
        assert f"#define {name}" not in source, f"{name} should be deleted, not re-valued"
    # Mechanical installation polarity is measured per aircraft and therefore
    # comes from the one runtime servo-calibration record, not compile-time
    # alpha/beta sign switches that can drift away from the saved evidence.
    assert "coax_ctrl_servo_calibration.pulse_sign[" in source
    assert "DRV_COAX_CTRL_SERVO_ALPHA_SIGN" not in source
    assert "DRV_COAX_CTRL_SERVO_BETA_SIGN" not in source
    assert "DRV_COAX_CTRL_ROLL_MOMENT_SIGN" not in source
    assert "DRV_COAX_CTRL_PITCH_MOMENT_SIGN" not in source


def test_tilt_moment_polarity_is_derived_from_measured_geometry() -> None:
    """极性必须由几何算出来，不能是一个可以随手翻的字面量。

    这两个符号历史上就是一对 `-1.0f` 常量。它们既没有推导，也没有任何东西
    挡住"照着现象翻一下试试"——而力矩极性翻错的表现恰好是正反馈，跟增益太
    大很像，很容易被误诊。现在它由重心到推力作用点的实测几何推出：重新量过
    飞机才可能改变它，改代码不行。

    2026-09-11：几何从编译期宏搬到了运行时机体模型（唯一来源是 Flash）。
    四条性质原样搬过来，一条没删——推导链的形状没变，只是 r_z 现在来自
    上位机写进去的实测值，代码里连一份副本都没有了。
    """
    source = read("Driver/Src/drv_coax_ctrl.c")
    model = read("Driver/Src/drv_airframe_params.c")

    assert "thrust_point_to_cg_z_m" in model
    assert (
        "out->thrust_point_to_cg_z_m = in->thrust_point_z_m - out->cg_z_m;" in model
    ), "r_z 必须由两个实测值相减得到"

    polarity = source.split("static float coax_ctrl_tilt_moment_polarity(void)", 1)[1]
    polarity = polarity.split("\n}", 1)[0]
    assert "thrust_point_to_cg_z_m" in polarity, (
        "极性必须引用实测几何，不能写成裸符号"
    )

    # 两轴共用同一个 -r_z 因子，所以只允许有一个极性来源。
    assert source.count("coax_ctrl_tilt_moment_polarity() *") == 2

    # r_z 为零 = 极性无定义。它必须挡住解锁，否则"漏填 thrust_point_z_m"会静默
    # 翻转俯仰与横滚两轴的极性——这正是本文件要防的那类失效。
    assert '"airframe.thrust_point_to_cg_z_m",' in model


def test_yaw_polarity_is_derived_from_rotor_handedness_and_marked_inferred() -> None:
    """偏航极性同样必须可推导；而且这次的上游是**反推值**，必须写明。

    分配式 `lower = (ku*F + Mz)/(ku+kl)` 里原本藏着一个没人写出来的假设：
    "加大下桨 = 正偏航"。它等价于断言下桨旋向，属于机械事实，不该以隐含形式
    存在。现在它由机体模型的 lower_rotor_spin_sense 字段推出。

    该值目前不是量出来的，是从"角速度环高增益抖振（=负反馈）"反推的，所以
    这里额外要求注释把这件事说清楚——一个未经实测的值伪装成实测值，比没有这个
    值更危险。

    2026-09-11：旋向从编译期宏变成机体模型里的一个字段。溯源注释必须跟着搬到
    字段声明处而不是就地删掉——删掉的话这个"反推值"下一任读代码的人就当成
    实测值了，而那正是本条测试存在的理由。
    """
    model = read("Driver/Inc/drv_airframe_params.h")
    source = read("Driver/Src/drv_coax_ctrl.c")

    assert "float lower_rotor_spin_sense;" in model
    # 溯源必须明说是反推、并留下证实/推翻的办法。
    provenance = model.split("float lower_rotor_spin_sense;", 1)[0]
    provenance = provenance[provenance.rindex("/*"):]
    assert "反推" in provenance, "反推值必须标注，不能冒充实测"
    assert "抖振" in provenance, "必须留下推理依据"
    assert "拆桨" in provenance, "必须留下证实/推翻的办法"

    polarity = source.split("static float coax_ctrl_yaw_torque_polarity(void)", 1)[1]
    polarity = polarity.split("\n}", 1)[0]
    assert "lower_rotor_spin_sense" in polarity, (
        "偏航极性必须引用旋向字段，不能写成裸符号"
    )
    # 分配与"已达成力矩"必须用同一套极性，只改一边等于自己骗自己。
    assert source.count("coax_ctrl_yaw_torque_polarity() *") == 2


# ── runtime checks: physical direction, using the real controller ──

SIGN_HARNESS = AIRFRAME_FIXTURE_C + r"""
#include "drv_coax_ctrl.h"

#include <math.h>
#include <stdio.h>

#define CHECK(cond, code) do { if (!(cond)) { \
    printf("FAIL %d\n", (code)); return (code); } } while (0)

static void base_state(DRV_COAX_CTRL_AttitudeInput *att,
                       DRV_COAX_CTRL_Reference *ref)
{
    const DRV_Airframe_Params *airframe = DRV_Airframe_Get();

    memset(att, 0, sizeof(*att));
    memset(ref, 0, sizeof(*ref));
    /* Hover-ish: direct attitude mode with manual thrust holding weight. */
    ref->direct_attitude_target_valid = 1U;
    ref->manual_total_force_valid = 1U;
    ref->manual_total_force_n = airframe->mass_kg * airframe->gravity_m_s2;
}

int main(void)
{
    /*
     * The firmware ships with no airframe data, so the control law needs one
     * installed before any of this means anything.  The fixture reproduces the
     * previous compile-time constants exactly -- every assertion below keeps
     * the meaning it had before the model moved to runtime.
     */
    airframe_load_reference();

    DRV_COAX_CTRL_AttitudeInput att;
    DRV_COAX_CTRL_Reference ref;
    DRV_COAX_CTRL_Output out;
    DRV_COAX_CTRL_Params params;
    DRV_COAX_CTRL_ServoCalibration cal;
    float level_alpha, level_beta;
    float nose_up_alpha, right_down_beta;
    float stick_fwd_alpha, stick_right_beta;

    DRV_COAX_CTRL_Init();
    DRV_COAX_CTRL_GetDefaultParams(&params);

    /* Every gain must be positive in the shipped defaults. */
    CHECK(params.attitude.att_kp[0] > 0.0f, 1);
    CHECK(params.attitude.att_kp[1] > 0.0f, 2);
    CHECK(params.rate.kp[0] > 0.0f, 3);
    CHECK(params.rate.kp[1] > 0.0f, 4);

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
    /* Sign only -- whether +pitch is nose-up or nose-down depends on the
     * estimator convention (NED vs NWU); this block pins relative signs. */
    att.pitch_rad = 0.15f;
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
    params.attitude.att_kp[0] = 0.0f;
    params.attitude.att_kp[1] = 0.0f;
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
    params.attitude.att_kp[1] = 0.5f;
    DRV_COAX_CTRL_SetParams(&params);
    params.attitude.att_kp[1] = -0.5f;
    DRV_COAX_CTRL_SetParams(&params);
    DRV_COAX_CTRL_GetParams(&params);
    CHECK(fabsf(params.attitude.att_kp[1] - 0.5f) < 1.0e-6f, 17);

    base_state(&att, &ref);
    att.pitch_rad = 0.15f;
    DRV_COAX_CTRL_Run(&att, &ref, &out);
    CHECK(out.alpha_rad * nose_up_alpha > 0.0f, 18);

    /* Named-parameter entry must reject a negative gain too. */
    CHECK(DRV_COAX_CTRL_SetParam("coax.att_pitch_kp", -0.5f) == 0U, 19);
    CHECK(DRV_COAX_CTRL_SetParam("coax.att_pitch_kp", 0.5f) != 0U, 20);

    /*
     * E. TILT -> MOMENT POLARITY FOLLOWS MEASURED GEOMETRY.
     * The thrust point sits below the CG, so tau = r x F makes a positive tilt
     * produce a positive FLU moment on both axes. Re-measure the airframe and
     * this expectation has to move with it -- which is the whole reason the
     * polarity is derived rather than written down as a sign.
     */
    CHECK(DRV_Airframe_Get()->thrust_point_to_cg_z_m < 0.0f, 21);

    DRV_COAX_CTRL_GetDefaultParams(&params);
    DRV_COAX_CTRL_SetParams(&params);
    DRV_COAX_CTRL_ResetState();
    base_state(&att, &ref);
    ref.target_roll_rad = 0.15f;
    DRV_COAX_CTRL_Run(&att, &ref, &out);
    CHECK(fabsf(out.beta_rad) > 1.0e-4f, 22);
    CHECK(out.beta_rad * out.moment_achieved_n_m[0] > 0.0f, 23);

    /*
     * F. THE CHAIN MUST REACH THE SERVO THE RIGHT WAY ROUND.
     *
     * Everything above compares moments against the same model that produced
     * them, so a whole-chain inversion stays self-consistent and passes. That
     * is exactly how the positive-feedback bug survived a green suite. This
     * block instead compares against a physical fact the firmware cannot
     * derive -- the author's bench observation of the persisted calibration
     * (data/calibration/servo_mechanical/2026-08-30, pulse_sign = -1/-1):
     *
     *     alpha pulse up -> thrust axis moves left (+Y)
     *     beta  pulse up -> thrust axis moves rear (-X)
     *
     * Right-wing-down is corrected by tilting the thrust right (-Y), so the
     * alpha pulse must go DOWN. Nose-down is corrected by tilting the thrust
     * forward (+X), so the beta pulse must go DOWN.
     */
    memset(&cal, 0, sizeof(cal));
    cal.center_us[DRV_COAX_CTRL_SERVO_ALPHA_INDEX] = 1530U;
    cal.center_us[DRV_COAX_CTRL_SERVO_BETA_INDEX] = 1500U;
    cal.min_us[DRV_COAX_CTRL_SERVO_ALPHA_INDEX] = 1000U;
    cal.min_us[DRV_COAX_CTRL_SERVO_BETA_INDEX] = 1000U;
    cal.max_us[DRV_COAX_CTRL_SERVO_ALPHA_INDEX] = 2000U;
    cal.max_us[DRV_COAX_CTRL_SERVO_BETA_INDEX] = 2000U;
    cal.pulse_sign[DRV_COAX_CTRL_SERVO_ALPHA_INDEX] = -1;
    cal.pulse_sign[DRV_COAX_CTRL_SERVO_BETA_INDEX] = -1;
    CHECK(DRV_COAX_CTRL_SetServoCalibration(&cal) != 0U, 24);

    DRV_COAX_CTRL_ResetState();
    base_state(&att, &ref);
    att.roll_rad = 0.15f;               /* right wing down */
    DRV_COAX_CTRL_Run(&att, &ref, &out);
    CHECK(out.servo_alpha_us <
          cal.center_us[DRV_COAX_CTRL_SERVO_ALPHA_INDEX], 25);

    DRV_COAX_CTRL_ResetState();
    base_state(&att, &ref);
    att.pitch_rad = 0.15f;              /* FLU: nose down */
    DRV_COAX_CTRL_Run(&att, &ref, &out);
    CHECK(out.servo_beta_us <
          cal.center_us[DRV_COAX_CTRL_SERVO_BETA_INDEX], 26);

    /*
     * G. THE MECHANICAL CALIBRATION IS THE ONLY VARIABLE.
     * Flip pulse_sign alone -- as a reversed linkage would -- and only the
     * pulse direction may flip. Nothing in the control law is allowed to
     * notice, which is what makes "calibrate once, reuse the firmware" true.
     */
    cal.pulse_sign[DRV_COAX_CTRL_SERVO_ALPHA_INDEX] = 1;
    cal.pulse_sign[DRV_COAX_CTRL_SERVO_BETA_INDEX] = 1;
    CHECK(DRV_COAX_CTRL_SetServoCalibration(&cal) != 0U, 27);

    DRV_COAX_CTRL_ResetState();
    base_state(&att, &ref);
    att.roll_rad = 0.15f;
    DRV_COAX_CTRL_Run(&att, &ref, &out);
    CHECK(out.servo_alpha_us >
          cal.center_us[DRV_COAX_CTRL_SERVO_ALPHA_INDEX], 28);

    DRV_COAX_CTRL_ResetState();
    base_state(&att, &ref);
    att.pitch_rad = 0.15f;
    DRV_COAX_CTRL_Run(&att, &ref, &out);
    CHECK(out.servo_beta_us >
          cal.center_us[DRV_COAX_CTRL_SERVO_BETA_INDEX], 29);

    /*
     * H. YAW POLARITY FOLLOWS THE ROTOR HANDEDNESS CONSTANT.
     * A rotor's reaction torque on the body opposes its own spin, so which
     * motor must speed up for +yaw is decided by the handedness, not by a
     * sign buried in the allocator.  With the shipped value (lower rotor
     * clockwise seen from above) a positive yaw moment must come from adding
     * thrust to the LOWER rotor.
     *
     * NOTE: that handedness is currently INFERRED, not measured -- see
     * airframe.lower_rotor_spin_sense.  This check pins the chain, not
     * the physical fact.
     */
    CHECK(DRV_Airframe_Get()->lower_rotor_spin_sense < 0.0f, 30);

    DRV_COAX_CTRL_GetDefaultParams(&params);
    DRV_COAX_CTRL_SetParams(&params);
    DRV_COAX_CTRL_ResetState();
    base_state(&att, &ref);
    att.gyro_z_rad_s = -0.5f;           /* yawing right -> +yaw moment wanted */
    DRV_COAX_CTRL_Run(&att, &ref, &out);
    {
        DRV_COAX_CTRL_Debug debug;
        DRV_COAX_CTRL_GetLastDebug(&debug);
        CHECK(debug.moment_cmd_n_m[2] > 0.0f, 31);
        CHECK(out.thrust_lower_n > out.thrust_upper_n, 32);
        /* Achieved torque must be reported with the same polarity it was
         * allocated with; reporting one and allocating the other is exactly
         * how a whole-chain inversion stays invisible. */
        CHECK(debug.yaw_torque_cmd > 0.0f, 33);
    }

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
         str(AIRFRAME_SOURCE),
         str(ROOT / "Driver" / "Src" / "drv_coax_ctrl.c"),
         str(ROOT / "Driver" / "Src" / "drv_position_control.c"),
         str(ROOT / "Driver" / "Src" / "drv_attitude_control.c"),
         str(ROOT / "Driver" / "Src" / "drv_rate_control.c"),
         str(harness), "-lm", "-o", str(executable)],
        check=True, capture_output=True, text=True)

    result = subprocess.run([str(executable)], capture_output=True, text=True)
    assert result.returncode == 0, f"sign check failed: {result.stdout.strip()}"

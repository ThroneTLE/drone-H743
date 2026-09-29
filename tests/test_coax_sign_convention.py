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

import re
import shutil
import subprocess
from pathlib import Path

import pytest

from _airframe_fixture import AIRFRAME_FIXTURE_C, AIRFRAME_SOURCE, PROP_MAP_SOURCE


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


# ── static checks: the convention must stay centralised and gains positive ──

def test_gains_are_positive_so_polarity_errors_cannot_be_masked() -> None:
    source = read("Driver/Src/drv_coax_ctrl.c")

    # Defaults must be positive; a stored negative gain used to be how a sign
    # error was hidden behind an apparently stable aircraft.
    for line in ("params->attitude.att_kp[0] = 1.8105f;",
                 "params->attitude.att_kp[1] = 1.7131f;",
                 # 2026-09-28 晚：按实测机体力臂 0.1082 m、舵机 333 Hz 的 A1 基线（出处见
                 # test_default_roll_pitch_gains_match_the_measured_airframe）。仍是正的。
                 "params->rate.kp[0] = 0.2748f;",
                 "params->rate.kp[1] = 0.2748f;",
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
    """极性（以及大小）必须由几何算出来，不能是一个可以随手翻的字面量。

    这两个符号历史上就是一对 `-1.0f` 常量。它们既没有推导，也没有任何东西
    挡住"照着现象翻一下试试"——而力矩极性翻错的表现恰好是正反馈，跟增益太
    大很像，很容易被误诊。

    2026-09-11：几何从编译期宏搬到了运行时机体模型（唯一来源是 Flash）。

    2026-09-27：几何的**那一点**换了。推力作用线穿过舵机转轴，τ = r × F 只取决于
    作用线的位置，所以 r_z 取"舵机转轴 − 重心"，不再取推力点；经验系数
    EFFECTIVENESS 一并删除，力矩的大小也只由几何给出。四条性质照原样重述在
    转轴 r_z 上：
      1. r_z 由两个实测值相减得到；
      2. 控制律的力臂引用这份几何，而不是裸符号或常数；
      3. 每个轴只有一处力臂乘法（正向模型唯一，反解/上限/达成力矩都走它）；
      4. 转轴为零（= 没填）挡住解锁——漏填会让 r_z 变成 +0.0946，两轴极性同时反掉。
    """
    source = read("Driver/Src/drv_coax_ctrl.c")
    model = read("Driver/Src/drv_airframe_params.c")

    # 1. 两个实测高度相减。
    assert "return (p != NULL) ? (p->servo1_axis_z_m - p->cg_z_m) : 0.0f;" in model
    assert "return (p != NULL) ? (p->servo2_axis_z_m - p->cg_z_m) : 0.0f;" in model

    # 2. 力臂 = −r_z，从机体模型取；经验系数与旧的推力点极性函数都已删除。
    assert "params->roll_tilt_lever_arm_m = -DRV_Airframe_RollTiltAxisToCgZ(airframe);" in source
    assert "params->pitch_tilt_lever_arm_m = -DRV_Airframe_PitchTiltAxisToCgZ(airframe);" in source
    for gone in ("#define DRV_COAX_CTRL_ROLL_EFFECTIVENESS",
                 "#define DRV_COAX_CTRL_PITCH_EFFECTIVENESS",
                 "coax_ctrl_tilt_moment_polarity(",
                 "thrust_lever_arm_m"):
        assert gone not in source, gone
    for axis, fn in (("roll", "coax_ctrl_roll_moment_from_tilt"),
                     ("pitch", "coax_ctrl_pitch_moment_from_tilt")):
        body = source.split(f"static float {fn}(", 1)[1].split("\n}", 1)[0]
        assert f"coax_ctrl_params.{axis}_tilt_lever_arm_m *" in body
        assert "thrust_point_to_cg_z_m" not in body, "符号不再看推力点"

    # 3. 每个轴只有一处力臂乘法：反解、力矩上限、实际达成力矩、调试分解、
    #    辨识出口都必须经由上面两个函数，不许另写一份。
    assert source.count("coax_ctrl_params.roll_tilt_lever_arm_m *") == 1
    assert source.count("coax_ctrl_params.pitch_tilt_lever_arm_m *") == 1

    # 4. 转轴没填挡住解锁；填了但离重心太近、或与推力点不在同侧也挡住。
    block = model.split("airframe_check_nonzero[] = {", 1)[1].split("};", 1)[0]
    assert '"airframe.servo1_axis_z_m",' in block
    assert '"airframe.servo2_axis_z_m",' in block
    assert '"airframe.thrust_point_to_cg_z_m",' in block
    for name in ("airframe.servo1_axis_z_m:near", "airframe.servo1_axis_z_m:sign",
                 "airframe.servo2_axis_z_m:near", "airframe.servo2_axis_z_m:sign"):
        assert f'"{name}"' in model, name


def test_default_roll_pitch_gains_match_the_measured_airframe() -> None:
    """默认增益必须与实测机体配套，数值由出处推得出来，而不是另一个魔数。

    力矩单位的增益只在某个倾转力臂下有意义（倾角 = asin(力矩 / (L·T))，舵机偏角 ∝ 1/L）。
    2026-09-27 实测：重心板下 0.0218 m、舵机转轴板下 0.13 m → L = 0.1082 m。
    俯仰对象：2026-09-28 回差补偿后光杆辨识（rod_004508 + rod_003912 联合；台架验证 rod_051131/051206/051758），
    作者指定的现行最优。横滚对象：−45° 斜杆 FF 联合减去俯仰推出（假设 Ixy = 0）。
    2026-09-28 舵机改为开机 333 Hz（−45° 带载纯延迟 30.2 → 22.4 ms）后，两轴按减去的 7.8 ms 重做同一套角点
    鲁棒整定（design_333hz.py）：俯仰 0.265/0.344、横滚 0.249/0.322，角度环都 1.619；−45° 台架 RATE rod_065433、
    ANGLE rod_065500 稳定——这是上一版默认（L0）。舵机改回 50 Hz 须换回 50 Hz 那组（俯仰 0.240/0.288/1.524、
    横滚 0.196/0.215/1.375）。
    现行默认 A1（作者 2026-09-28 晚存入 Flash）：内环压榨版 A 档增益 + 宽陷波 Q1.2，俯仰 0.2748/0.4607/1.7131、
    横滚 0.2748/0.4862/1.8105；−45° 台架 RATE rod_224114、ANGLE rod_224131 验证（summary.md 第 10 节）。
    前馈倍率：力矩单位已是真实 N·m，两轴取 1。积分限幅作者同意统一放到 0.05 N·m。
    """
    source = read("Driver/Src/drv_coax_ctrl.c")

    def default(lhs: str) -> float:
        match = re.search(re.escape(lhs) + r" = ([0-9.]+)f;", source)
        assert match is not None, lhs
        return float(match.group(1))

    lever = -0.0218 - (-0.13)

    assert default("params->rate.ff_gain[0]") == 1.0
    # 横滚：−45° 斜杆推算的对象（作者决定不测 +45°，假设 Ixy = 0），舵机 333 Hz 下的 A1 基线。
    assert default("params->rate.kp[0]") == pytest.approx(0.2748)
    assert default("params->rate.ki[0]") == pytest.approx(0.4862)
    assert default("params->attitude.att_kp[0]") == pytest.approx(1.8105)
    assert default("params->rate.ff_gain[1]") == 1.0
    assert default("params->rate.integrator_limit[0]") == 0.05
    assert default("params->rate.integrator_limit[1]") == 0.05
    # 俯仰：光杆辨识的对象，舵机 333 Hz 下的 A1 基线（台架 rod_224114/rod_224131）。
    assert default("params->rate.kp[1]") == pytest.approx(0.2748)
    assert default("params->rate.ki[1]") == pytest.approx(0.4607)
    assert default("params->attitude.att_kp[1]") == pytest.approx(1.7131)

    # 注释的锚点：说清配套的机体、俯仰出处、横滚推算、333 Hz 重整定与台架验证、上一版 L0 与 50 Hz 回退组、
    # A1 的来历与台架验证、改机体要重推。
    note = source.split("void DRV_COAX_CTRL_GetDefaultParams(", 1)[1]
    note = note.split("params->rate.kp[0] = 0.2748f;", 1)[0]
    assert "0.1082 m" in note and "1145.3 g" in note
    assert "rod_004508" in note and "rod_051758" in note
    assert "−45° 斜杆" in note and "Ixy = 0" in note
    assert "333 Hz" in note and "design_333hz" in note and "30.2 → 22.4 ms" in note
    assert "rod_065433" in note and "rod_065500" in note
    assert ".265/.344/1.619" in note and ".249/.322/1.619" in note, "上一版（L0）那组要写明"
    assert "0.240/0.288/1.524" in note and "0.196/0.215/1.375" in note, "舵机改回 50 Hz 时的那组要写明"
    assert "A1" in note and "Q1.2" in note and "rod_224114" in note and "rod_224131" in note
    assert "第 10 节" in note
    assert "∝ 1/L" in note and "重新推导" in note

    # 偏航与姿态环不含倾转力臂，不许跟着动。
    assert default("params->rate.ff_gain[2]") == 1.0
    assert default("params->rate.integrator_limit[2]") == pytest.approx(0.00020)
    assert "params->rate.kp[2] = DRV_Airframe_Get()->izz_kgm2 * 0.525f;" in source


def test_yaw_polarity_comes_only_from_the_measured_rotor_calibration() -> None:
    """偏航极性必须可推导，而且上游必须是**量出来的**，不是从调参现象反推的。

    分配式 `lower = (ku*F + Mz)/(ku+kl)` 里原本藏着一个没人写出来的假设：
    "加大下桨 = 正偏航"。它等价于断言下桨旋向，属于机械事实，不该以隐含形式
    存在。

    2026-09-11 它变成机体模型里的 `lower_rotor_spin_sense`；但那个值是从
    "角速度环高增益抖振（= 负反馈）"反推的。反推证明的是"整条链的符号彼此不
    矛盾"，不是"桨真的往那边转"——换一套增益、或者把某处符号和它一起翻过来，
    现象一模一样，而飞机的偏航方向已经反了。

    2026-09-13 起唯一来源是上位机通电标定的 `drv_prop_map`。本条钉三件事：
      1. 极性只由标定推出，代码里不许再出现裸符号；
      2. 那个旧字段**退役**且不再被任何人读——留着字段是为了 Flash ABI，
         名字加了 retired_ 前缀是为了让漏改的旧代码编译失败而不是安静地读；
      3. 新来源必须自带"为什么不能用反推值"的溯源，否则下一任会把它搬回去。
    """
    model = read("Driver/Inc/drv_airframe_params.h")
    source = read("Driver/Src/drv_coax_ctrl.c")
    table = read("Driver/Src/drv_airframe_params.c")
    prop = read("Driver/Inc/drv_prop_map.h")

    assert "float retired_lower_rotor_spin_sense;" in model
    assert "AIRFRAME_ENTRY(lower_rotor_spin_sense)" not in table, (
        "退役字段不许留在可写参数表里——留着就等于留了第二个来源"
    )
    assert '"airframe.lower_rotor_spin_sense",' not in table, (
        "解锁必填项已经搬到 DRV_PropMap_IsCalibrated()，不该两处都有"
    )

    assert "反推" in prop, "新来源必须说明它取代的是一个反推值"
    assert "俯视" in prop, "旋向必须写明是从哪个方向看的"

    polarity = source.split("static float coax_ctrl_yaw_torque_polarity(void)", 1)[1]
    polarity = polarity.split("\n}", 1)[0]
    assert "DRV_PropMap_YawTorquePolarity()" in polarity, (
        "偏航极性必须引用标定，不能写成裸符号"
    )
    assert "lower_rotor_spin_sense" not in polarity
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
     * The thrust line passes through the servo tilt axes, and both axes sit
     * below the CG (r_z = axis z - cg z < 0), so tau = r x F makes a positive
     * tilt produce a positive FLU moment on both axes. Re-measure the airframe
     * and this expectation has to move with it -- which is the whole reason
     * the polarity is derived rather than written down as a sign (block I
     * moves the axes and watches it happen).
     */
    CHECK(DRV_Airframe_RollTiltAxisToCgZ(DRV_Airframe_Get()) < 0.0f, 21);
    CHECK(DRV_Airframe_PitchTiltAxisToCgZ(DRV_Airframe_Get()) < 0.0f, 34);

    DRV_COAX_CTRL_GetDefaultParams(&params);
    DRV_COAX_CTRL_SetParams(&params);
    DRV_COAX_CTRL_ResetState();
    base_state(&att, &ref);
    ref.target_roll_rad = 0.15f;
    DRV_COAX_CTRL_Run(&att, &ref, &out);
    CHECK(fabsf(out.beta_rad) > 1.0e-4f, 22);
    CHECK(out.beta_rad * out.moment_achieved_n_m[0] > 0.0f, 23);

    DRV_COAX_CTRL_ResetState();
    base_state(&att, &ref);
    ref.target_pitch_rad = 0.15f;
    DRV_COAX_CTRL_Run(&att, &ref, &out);
    CHECK(fabsf(out.alpha_rad) > 1.0e-4f, 35);
    CHECK(out.alpha_rad * out.moment_achieved_n_m[1] > 0.0f, 36);

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
     * The handedness now comes from the ground-station rotor calibration
     * (drv_prop_map): the operator spins each motor, watches it, and records
     * what it does.  It is no longer back-solved from tuning behaviour.  The
     * fixture declares the same aircraft the old inferred value described, so
     * every number in this harness is unchanged by that move.
     */
    CHECK(DRV_PropMap_LowerSpinSense() < 0.0f, 30);
    CHECK(DRV_PropMap_YawTorquePolarity() > 0.0f, 31);

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

    /*
     * I. MOVE THE TILT AXES ACROSS THE CG AND THE MOMENT MUST FLIP -- AND
     *    NOTHING ELSE MAY FLIP IT.
     * Mirror both servo axes to the other side of the CG (same distance) and
     * the very same servo pulses must produce the opposite FLU moment of the
     * same size: the sign lives in the measured geometry and nowhere else.
     * The thrust point is still below the CG, so the arming gate must refuse
     * this airframe -- axis and props on opposite sides of the CG almost
     * always means a sign typo (e.g. +0.13 for "13 cm below the board"), and
     * a polarity flip is a takeoff flip.  While refused, the forward map
     * claims no moment at all (block K).  Move the thrust point across too
     * and the (hypothetical, but consistent) airframe arms again with the
     * flipped polarity and the same size.
     */
    {
        DRV_Airframe_Params mirrored;
        const float force_n =
            DRV_Airframe_Get()->mass_kg * DRV_Airframe_Get()->gravity_m_s2;
        const uint16_t alpha_us =
            (uint16_t)(cal.center_us[DRV_COAX_CTRL_SERVO_ALPHA_INDEX] + 80U);
        const uint16_t beta_us =
            (uint16_t)(cal.center_us[DRV_COAX_CTRL_SERVO_BETA_INDEX] - 60U);
        float before[3];
        float after[3];

        CHECK(DRV_Airframe_IsValid() == 1U, 37);
        CHECK(DRV_COAX_CTRL_MomentFromServoPulses(force_n, alpha_us, beta_us,
                                                  before) != 0U, 38);
        CHECK(fabsf(before[0]) > 1.0e-3f, 39);
        CHECK(fabsf(before[1]) > 1.0e-3f, 40);

        DRV_Airframe_GetParams(&mirrored);
        mirrored.servo1_axis_z_m =
            (2.0f * mirrored.cg_z_m) - mirrored.servo1_axis_z_m;
        mirrored.servo2_axis_z_m =
            (2.0f * mirrored.cg_z_m) - mirrored.servo2_axis_z_m;
        DRV_Airframe_SetParams(&mirrored);
        CHECK(DRV_Airframe_IsValid() == 0U, 46);
        CHECK(strcmp(DRV_Airframe_FirstInvalidName(),
                     "airframe.servo1_axis_z_m:sign") == 0, 47);
        CHECK(DRV_COAX_CTRL_MomentFromServoPulses(force_n, alpha_us, beta_us,
                                                  after) == 0U, 41);
        CHECK((after[0] == 0.0f) && (after[1] == 0.0f), 42);

        mirrored.thrust_point_to_cg_z_m = -mirrored.thrust_point_to_cg_z_m;
        DRV_Airframe_SetParams(&mirrored);
        CHECK(DRV_Airframe_IsValid() == 1U, 48);
        CHECK(DRV_COAX_CTRL_MomentFromServoPulses(force_n, alpha_us, beta_us,
                                                  after) != 0U, 49);
        CHECK(before[0] * after[0] < 0.0f, 50);
        CHECK(before[1] * after[1] < 0.0f, 51);
        CHECK(fabsf(before[0] + after[0]) < 1.0e-5f * fabsf(before[0]), 44);
        CHECK(fabsf(before[1] + after[1]) < 1.0e-5f * fabsf(before[1]), 45);
    }

    /*
     * J. THE GEOMETRY STORED ON THE BOARD ON 2026-09-27.
     * Parts table as weighed (754.6 g), both tilt axes measured at z = -0.13 m
     * (they intersect), CG derived from the parts table: -0.094558 m.  The
     * model must accept it and produce tau = 0.035442 * T * sin(tilt) --
     * nothing else in the product, no empirical effectiveness factor -- and
     * publish that signed lever as coax.*_tilt_lever_arm_m.  (0.035442 m is
     * the lever the default N*m gains were rescaled for, not a confirmed real
     * lever: the author later measured the CG at about -0.01 m, lever 0.12 m.)
     */
    float j_body_x = 0.0f;
    float j_body_y = 0.0f;
    {
        DRV_Airframe_Params real;
        float moment[3] = { 0.05f, -0.04f, 0.0f };
        float body_x = 0.0f;
        float body_y = 0.0f;
        const float thrust_n = 7.0f;

        memset(&real, 0, sizeof(real));
        real.board_mass_g = 75.0f;
        real.battery_mass_g = 232.0f;
        real.base_mass_g = 99.0f;
        real.servo_motor_mass_g = 348.6f;
        real.battery_cg_z_m = 0.109f;
        real.base_cg_z_m = -0.117f;
        real.servo_motor_cg_z_m = -0.244f;
        real.servo1_axis_z_m = -0.13f;
        real.servo2_axis_z_m = -0.13f;
        real.thrust_point_z_m = -0.2955f;
        real.ixx_kgm2 = 0.051f;
        real.iyy_kgm2 = 0.051f;
        real.izz_kgm2 = 0.005f;
        real.gravity_m_s2 = 9.81f;
        real.max_total_thrust_g = 1595.342f;
        real.servo_deg_per_us = 0.090f;
        real.derived_auto = 1.0f;
        DRV_Airframe_SetParams(&real);

        CHECK(DRV_Airframe_IsValid() == 1U, 52);
        CHECK(fabsf(DRV_Airframe_Get()->cg_z_m - (-0.094558f)) < 2.0e-6f, 53);
        DRV_COAX_CTRL_GetParams(&params);
        CHECK(fabsf(params.roll_tilt_lever_arm_m - 0.035442f) < 2.0e-6f, 54);
        CHECK(fabsf(params.pitch_tilt_lever_arm_m - 0.035442f) < 2.0e-6f, 55);

        CHECK(DRV_COAX_CTRL_SolveBodyTiltFromMoment(moment, thrust_n,
                                                    &body_x, &body_y) == 1U, 56);
        CHECK(fabsf(0.035442f * thrust_n * sinf(body_y) - moment[0]) < 2.0e-5f, 57);
        CHECK(fabsf(0.035442f * thrust_n * sinf(body_x) * cosf(body_y) -
                    moment[1]) < 2.0e-5f, 58);
        j_body_x = body_x;
        j_body_y = body_y;
    }

    /*
     * K. AN AIRFRAME THE GATE REFUSES MUST NOT STEER THE SERVOS.
     * Variant 0 is the board as it stood on 2026-09-27: both servo axes stored
     * as 0 (never filled), so the lever is cg - 0 = -0.0946 m -- the WRONG
     * SIGN.  The disarmed controller used to keep driving the servos through
     * it, so a hand-tilt servo-direction check looked reversed and invited
     * flipping pulse_sign.  Variant 1 puts the axes exactly on the CG (lever
     * 0), where the bisection used to pin to the negative tilt limit and still
     * report success.  Both must centre the servos, fail the public inverse
     * and claim zero moment from the forward map.  Restoring block J's
     * airframe must then give J's answer bit for bit: the guard changes
     * nothing while the model is valid.
     */
    {
        DRV_Airframe_Params device;
        DRV_Airframe_Params saved;
        float moment[3] = { 0.05f, -0.04f, 0.0f };
        float achieved[3];
        float body_x;
        float body_y;
        const float thrust_n = 7.0f;
        uint32_t variant;

        DRV_Airframe_GetParams(&saved);
        for (variant = 0U; variant < 2U; ++variant) {
            device = saved;
            if (variant == 0U) {
                device.servo1_axis_z_m = 0.0f;
                device.servo2_axis_z_m = 0.0f;
            } else {
                device.derived_auto = 0.0f;
                device.servo1_axis_z_m = device.cg_z_m;
                device.servo2_axis_z_m = device.cg_z_m;
            }
            DRV_Airframe_SetParams(&device);
            CHECK(DRV_Airframe_IsValid() == 0U, 59);
            DRV_COAX_CTRL_GetParams(&params);
            CHECK((variant == 0U) ? (params.roll_tilt_lever_arm_m < -0.09f)
                                  : (params.roll_tilt_lever_arm_m == 0.0f), 60);

            body_x = 1.0f;
            body_y = 1.0f;
            CHECK(DRV_COAX_CTRL_SolveBodyTiltFromMoment(moment, thrust_n,
                                                        &body_x, &body_y) == 0U, 61);
            CHECK((body_x == 0.0f) && (body_y == 0.0f), 62);
            achieved[0] = 1.0f;
            achieved[1] = 1.0f;
            achieved[2] = 1.0f;
            CHECK(DRV_COAX_CTRL_MomentFromServoPulses(thrust_n, 1600U, 1400U,
                                                      achieved) == 0U, 63);
            CHECK((achieved[0] == 0.0f) && (achieved[1] == 0.0f) &&
                  (achieved[2] == 0.0f), 64);

            /* Disarmed controller, disturbed on both axes: servos stay centred. */
            DRV_COAX_CTRL_ResetState();
            base_state(&att, &ref);
            att.roll_rad = 0.15f;
            att.pitch_rad = 0.15f;
            DRV_COAX_CTRL_Run(&att, &ref, &out);
            CHECK(out.servo_alpha_us ==
                  cal.center_us[DRV_COAX_CTRL_SERVO_ALPHA_INDEX], 65);
            CHECK(out.servo_beta_us ==
                  cal.center_us[DRV_COAX_CTRL_SERVO_BETA_INDEX], 66);
            CHECK((out.moment_achieved_n_m[0] == 0.0f) &&
                  (out.moment_achieved_n_m[1] == 0.0f), 67);
        }

        DRV_Airframe_SetParams(&saved);
        CHECK(DRV_Airframe_IsValid() == 1U, 68);
        CHECK(DRV_COAX_CTRL_SolveBodyTiltFromMoment(moment, thrust_n,
                                                    &body_x, &body_y) == 1U, 69);
        CHECK((body_x == j_body_x) && (body_y == j_body_y), 70);
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
         str(PROP_MAP_SOURCE),
         str(ROOT / "Driver" / "Src" / "drv_coax_ctrl.c"),
         str(ROOT / "Driver" / "Src" / "drv_position_control.c"),
         str(ROOT / "Driver" / "Src" / "drv_attitude_control.c"),
         str(ROOT / "Driver" / "Src" / "drv_rate_control.c"),
         str(harness), "-lm", "-o", str(executable)],
        check=True, capture_output=True, text=True)

    result = subprocess.run([str(executable)], capture_output=True, text=True)
    assert result.returncode == 0, f"sign check failed: {result.stdout.strip()}"

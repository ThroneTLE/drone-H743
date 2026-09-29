"""激励幅值 → 预计舵机摆幅，以及按当前力臂自动换默认幅值（2026-09-27 舵机回差问题）。

固件把 τ = I·α 按当前板子的倾转力臂反解成舵机倾角 δ = asin(τ/(L·T))。几何力矩模型下
L 随重心差好几倍：今晚 L = 0.035442 m 时默认 0.065 rad/s 摆约 10°，重心改到 −0.01
（L = 0.12 m）后只摆约 3°，落进舵机回差（2026-09-27 阶梯：摆幅 5～6° 以下拟合度 15～52%）。

钉的是：
* 纯函数的数与 2026-09-27 旧模型双脉冲实录对得上（amp 0.15、L 0.0825、T 7.4 N、斜坡 150 ms、
  I 0.051 → 记录力矩峰值 0.102 N·m、舵机约 9.6°），扫频按终止频率算并受舵机转速封顶；
* 页面读到板子参数后在幅值旁写预计摆幅；幅值还是预设值就自动换成舵机约 10° 的值并写明，
  手填的值永远不动；配置下发中、一轮在跑时不换；SERVO/ANGLE 不估算；
* 预计摆幅不到 5° 时顶部状态提醒，但照常能开始。
页面部分用 test_sysid_page 的假链路夹具。
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

from test_sysid_page import (  # noqa: E402,F401  页面夹具与报文工具
    drive_config, feed, page, thr_line,
)

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
try:
    from panel_lib.pages.sysid import excitation_scale as scale
    from panel_lib.pages.sysid.amplitude_hint import AMP_HINT_BY_MODE, AMP_HINT_UNKNOWN
finally:
    sys.path.pop(0)

RAMP_MS, T_N, I_KGM2 = 150, 7.4, 0.051
L_LEGACY, L_TONIGHT, L_CG_FIXED = 0.0825, 0.035442, 0.12


def doublet(amp, lever, **kw):
    return scale.estimate("doublet", amp, ramp_ms=kw.get("ramp_ms", RAMP_MS), chirp_f0_hz=0.5,
                          chirp_f1_hz=6.0, lever_m=lever, thrust_n=T_N, inertia_kg_m2=I_KGM2)


def chirp(amp, lever, f0=0.5, f1=6.0):
    return scale.estimate("chirp", amp, ramp_ms=RAMP_MS, chirp_f0_hz=f0, chirp_f1_hz=f1,
                          lever_m=lever, thrust_n=T_N, inertia_kg_m2=I_KGM2)


# ---------------------------------------------------------------- 纯函数


def test_legacy_doublet_matches_the_recorded_run():
    """实录：记录力矩峰值 0.102 N·m = I·2·amp/斜坡，舵机约 9.6°。"""
    assert I_KGM2 * 2 * 0.15 / (RAMP_MS * 1e-3) == pytest.approx(0.102)
    assert scale.swing_from_torque_deg(0.102, L_LEGACY, T_N) == pytest.approx(9.6, abs=0.1)
    est = doublet(0.15, L_LEGACY)
    assert est.swing_deg == pytest.approx(9.6, abs=0.15)
    assert not est.needs_new_amplitude and not est.warn_before_start


def test_tonight_default_amplitude_swings_about_ten_degrees():
    est = doublet(0.065, L_TONIGHT)
    assert est.swing_deg == pytest.approx(9.6, abs=0.15)
    assert est.suggested_amp_rad_s == pytest.approx(0.067)
    assert est.suggested_swing_deg == pytest.approx(10.0, abs=0.1)


def test_after_the_cg_fix_the_default_falls_into_backlash():
    est = doublet(0.065, L_CG_FIXED)
    assert est.swing_deg == pytest.approx(2.85, abs=0.1)
    assert est.warn_before_start and est.needs_new_amplitude
    assert est.suggested_amp_rad_s == pytest.approx(0.22, abs=0.01)
    assert est.suggested_swing_deg == pytest.approx(10.0, abs=0.1)
    assert "预计舵机摆幅约 2.9°（小于 6° 会落在舵机回差里）" == scale.swing_text(est)
    assert scale.suggestion_text(est) == "建议幅值 0.227（舵机约 10°）"


def test_step_moves_the_setpoint_half_as_fast_as_a_doublet():
    step = scale.estimate("step", 0.13, ramp_ms=RAMP_MS, chirp_f0_hz=0.5, chirp_f1_hz=6,
                          lever_m=L_TONIGHT, thrust_n=T_N, inertia_kg_m2=I_KGM2)
    assert step.swing_deg == pytest.approx(doublet(0.065, L_TONIGHT).swing_deg)
    assert doublet(0.065, L_TONIGHT, ramp_ms=75).swing_deg > 1.9 * doublet(0.065, L_TONIGHT).swing_deg


def test_chirp_swing_is_smallest_at_f0_and_the_suggestion_respects_servo_slew():
    """默认 0.022 在今晚的板子上：6 Hz 处约 9.3°，但舵机要转约 350°/s，太靠近约 400°/s 的上限。"""
    est = chirp(0.022, L_TONIGHT)
    assert est.swing_f0_deg == pytest.approx(0.77, abs=0.05)
    assert est.swing_deg == pytest.approx(9.28, abs=0.05)
    assert est.slew_dps == pytest.approx(2 * math.pi * 6 * est.swing_deg)
    assert est.slew_too_fast and est.needs_new_amplitude and est.slew_limited
    again = chirp(est.suggested_amp_rad_s, L_TONIGHT)
    assert again.slew_dps <= scale.SLEW_SAFE_DPS
    assert again.swing_deg >= scale.BACKLASH_SWING_DEG and not again.needs_new_amplitude
    assert "6 Hz 处舵机要转约 350°/s" in scale.swing_text(est)
    assert scale.swing_text(est).startswith(
        "预计舵机摆幅约 9.3°（扫到 6 Hz 时；0.5 Hz 起步时约 0.8°；小于 6° 会落在舵机回差里）")


def test_chirp_suggestion_keeps_ten_degrees_at_f1_when_the_servo_is_fast_enough():
    est = chirp(0.022, L_CG_FIXED, f1=3.0)
    assert not est.slew_limited
    assert est.suggested_swing_deg == pytest.approx(10.0, abs=0.3)
    assert chirp(est.suggested_amp_rad_s, L_CG_FIXED, f1=3.0).swing_deg == pytest.approx(10.0, abs=0.3)


def test_rod_lever_mixes_the_two_axes_by_azimuth():
    levers = (0.08, 0.12)                        # (roll, pitch)
    assert scale.rod_lever_m(levers, 90) == pytest.approx(0.12)
    assert scale.rod_lever_m(levers, 0) == pytest.approx(0.08)
    assert scale.rod_lever_m(levers, 45) == pytest.approx(0.10)
    assert scale.rod_lever_m((-0.05, -0.05), 45) == pytest.approx(0.05)
    assert scale.rod_lever_m(None, 45) is None
    assert scale.rod_lever_m((0.1, -0.1), 45) is None     # 两轴力矩抵消：算不出摆幅


@pytest.mark.parametrize("kwargs", [dict(lever_m=0.0), dict(thrust_n=-1.0), dict(inertia_kg_m2=math.nan)])
def test_unusable_inputs_give_no_estimate(kwargs):
    base = dict(ramp_ms=RAMP_MS, chirp_f0_hz=0.5, chirp_f1_hz=6.0, lever_m=L_TONIGHT,
                thrust_n=T_N, inertia_kg_m2=I_KGM2)
    base.update(kwargs)
    assert scale.estimate("doublet", 0.065, **base) is None


def test_unknown_profiles_and_bad_chirp_bands_give_no_estimate():
    assert scale.estimate("sine", 0.065, ramp_ms=RAMP_MS, chirp_f0_hz=0.5, chirp_f1_hz=6.0,
                          lever_m=L_TONIGHT, thrust_n=T_N, inertia_kg_m2=I_KGM2) is None
    assert chirp(0.022, L_TONIGHT, f0=6.0, f1=0.5) is None


def test_amplitudes_are_written_back_without_trailing_zeros():
    assert scale.format_amp(0.227) == "0.227"
    assert scale.format_amp(0.2) == "0.2"
    assert scale.format_amp(0.0670000001) == "0.067"


# ---------------------------------------------------------------- 页面

#: 今晚板上的几何力矩模型：部件表重心 −0.094558，两舵机转轴 −0.13 → L = 0.035442 m；机重 7.4 N。
BOARD_TONIGHT = (("airframe.weight_n", "7.400000"), ("airframe.ixx_kgm2", "0.051000"),
                 ("airframe.imu_z_m", "0.000000"), ("airframe.cg_z_m", "-0.094558"),
                 ("airframe.servo1_axis_z_m", "-0.130000"),
                 ("airframe.servo2_axis_z_m", "-0.130000"))
#: 机体模型页把重心改成实测 −0.01 后飞控的确认（App/Src/app_control.c 的 OK param 行）。
CG_FIX = ["OK param name=airframe.cg_z_m value=-0.010000"]


def param_lines(params=BOARD_TONIGHT):
    return [f"PARAM name={name} value={value}" for name, value in params]


def test_board_params_show_the_swing_and_replace_the_preset_amplitude(page):
    assert page.amp_var.get() == "0.065" and page.amp_hint_var.get() == AMP_HINT_UNKNOWN
    feed(page, param_lines())
    assert page.amp_var.get() == "0.067"
    hint = page.amp_hint_var.get()
    assert "预计舵机摆幅约 10.0°（小于 6° 会落在舵机回差里）" in hint
    assert "按当前力臂 L=0.0354 m 自动设为 0.067，保持舵机约 10°" in hint
    assert "建议幅值" not in hint
    assert "amp=0.067 " in page.command_preview_var.get()
    # 重心修正后力臂变成 0.12 m：没手填过的幅值跟着换。
    feed(page, CG_FIX)
    assert page.amp_var.get() == "0.227"
    assert "按当前力臂 L=0.1200 m 自动设为 0.227，保持舵机约 10°" in page.amp_hint_var.get()


def test_a_typed_amplitude_is_never_overridden_and_only_suggested(page):
    page.amp_var.set("0.03")
    feed(page, param_lines())
    assert page.amp_var.get() == "0.03"
    hint = page.amp_hint_var.get()
    assert "预计舵机摆幅约 4.5°（小于 6° 会落在舵机回差里）" in hint
    assert "建议幅值 0.067（舵机约 10°）" in hint and "自动设为" not in hint
    feed(page, CG_FIX)
    assert page.amp_var.get() == "0.03"
    assert "建议幅值 0.227" in page.amp_hint_var.get()


def test_small_swing_warns_in_the_banner_but_does_not_block_start(page):
    page.amp_var.set("0.03")
    feed(page, param_lines() + [thr_line(armed=1, thr_low=1)])
    assert page.banner_var.get() == "就绪，可以开始"
    detail = page.banner_detail_var.get()
    assert "提醒：按当前幅值 0.03 rad/s，舵机只摆约 4.5°" in detail
    assert "幅值改成 0.067。不改也能照常开始。" in detail
    drive_config(page, params=BOARD_TONIGHT)
    sent = page.panel.transport.lines
    assert sent[-1] == "SYSID START"
    assert any(line.startswith("SYSID EXC ") and " amp=0.03 " in line for line in sent)
    assert page.banner_var.get() == "正在开始…" and "提醒：" in page.banner_detail_var.get()


def test_a_comfortable_swing_adds_no_banner_warning(page):
    feed(page, param_lines() + [thr_line(armed=1, thr_low=1)])
    assert page.banner_var.get() == "就绪，可以开始"
    assert "提醒" not in page.banner_detail_var.get()


def test_the_amplitude_is_not_changed_while_a_run_is_configured_or_running(page):
    """PARAM? 是开始事务的一步：参数回来时正在下发，界面上的幅值必须是本轮发出去的那个。"""
    drive_config(page, params=BOARD_TONIGHT)
    exc = next(line for line in page.panel.transport.lines if line.startswith("SYSID EXC "))
    assert " amp=0.065 " in exc and page.amp_var.get() == "0.065"
    assert "预计舵机摆幅约 9.7°" in page.amp_hint_var.get()
    page.handle_line("SYSID start run=3 profile=1 amp_mrad_s=65 dur_ms=8000 rate_hz=250 "
                     "I=51000 ugm2 psi_mrad=785 auto=1 target_cn=740")
    feed(page, param_lines())
    assert page.amp_var.get() == "0.065"
    page.handle_line("SYSID end run=3 state=aborted reason=rc_arm dropped=0")
    assert page.amp_var.get() == "0.067"
    assert "自动设为 0.067" in page.amp_hint_var.get()


def test_chirp_preset_is_scaled_to_the_servo_speed(page):
    page.experiment_combo.current(1)
    page.experiment_combo.event_generate("<<ComboboxSelected>>")
    feed(page, param_lines())
    assert page.profile_var.get() == "chirp" and page.amp_var.get() == "0.018"
    hint = page.amp_hint_var.get()
    assert "扫到 6 Hz 时；0.5 Hz 起步时约" in hint
    assert "自动设为 0.018，保持舵机约 8°（扫到 6 Hz 时舵机转速不超过 300°/s）" in hint
    # 换回双脉冲：预设 0.065 → 按力臂再换。
    page.experiment_combo.current(0)
    page.experiment_combo.event_generate("<<ComboboxSelected>>")
    assert page.amp_var.get() == "0.067"


@pytest.mark.parametrize("mode", ["SERVO", "ANGLE"])
def test_servo_only_and_angle_modes_leave_the_amplitude_alone(page, mode):
    page.mode_var.set(mode)
    feed(page, param_lines() + [thr_line(armed=0, thr_low=1)])
    assert page.amp_var.get() == "0.065"
    assert page.amp_hint_var.get() == AMP_HINT_BY_MODE[mode]
    page.amp_var.set("0.01")
    assert page.amp_swing_warning() == ""
    page.amp_var.set("0.065")
    page.mode_var.set("FF")
    assert page.amp_var.get() == "0.067"


def test_legacy_firmware_levers_are_used_too(page):
    """旧力矩模型（力臂 × 冻结的有效系数）的板子照样能估：45° 杆取两轴平均。"""
    legacy = (("airframe.weight_n", "7.400000"), ("airframe.ixx_kgm2", "0.051000"),
              ("airframe.roll_thrust_lever_arm_m", "0.145000"),
              ("airframe.pitch_thrust_lever_arm_m", "0.145000"),
              ("airframe.thrust_point_to_cg_z_m", "-0.030000"))
    feed(page, param_lines(legacy))
    est = page.amp_swing_estimate()
    assert est.lever_m == pytest.approx(0.145 * (0.581 + 0.569) / 2)
    assert est.swing_deg == pytest.approx(10.0, abs=0.2)
    assert page.amp_var.get() == scale.format_amp(est.suggested_amp_rad_s)


def test_unknown_board_keeps_the_preset_and_says_what_is_missing(page):
    feed(page, param_lines(BOARD_TONIGHT[:2]))           # 只有机重与惯量，没有力臂
    assert page.amp_var.get() == "0.065"
    assert page.amp_hint_var.get() == AMP_HINT_UNKNOWN
    assert page.amp_swing_warning() == ""


def test_typed_target_thrust_and_inertia_feed_the_estimate(page):
    feed(page, param_lines())
    assert page.amp_var.get() == "0.067"
    page.target_thrust_var.set("14.8")                   # 推力翻倍 → 同样摆幅要约两倍幅值
    assert page.amp_var.get() == "0.134"
    page.inertia_var.set("0.102")                        # 惯量也翻倍 → 回到约 0.067
    assert page.amp_var.get() == "0.067"

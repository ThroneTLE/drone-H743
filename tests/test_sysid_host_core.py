"""主机分析核心：档案、推力源、拟合、整定、报告。

这些判据钉的都是"旧那套具体做错了什么"：

* 老面板的 `fit_ident_step` 是阈值穿越法，**没有延迟估计**。这里的拟合把延迟
  当拟合参数，用合成的已知 (I, c, T_d) 反过来验它能不能找回来。
* 老面板 `kp = 0.35/|K|` 拍脑袋。这里的带宽由**辨出来的延迟**决定，
  并且钉死"延迟越大带宽越低"这条单调性。
* 老推力台把转速当成 `kv*V*pct/100` 估算值却不作标注。这里每个推力源都自报
  `quality`，报告里必须印出来。
* 阻尼与偏心在短激励下强相关。这里要求**如实报不可分离**，而不是硬给一个数。
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from sysid import airframe_link, report, thrust_model, tune  # noqa: E402
from sysid.excitation import Excitation, PROFILE_CHIRP  # noqa: E402
from sysid.profile import FitResult, Gains, IdentProfile  # noqa: E402
from sysid.rig import AZIMUTH_45_RAD, Rig, imu_lever_arm_error  # noqa: E402
sys.path.pop(0)

np = pytest.importorskip("numpy")


# ---------------------------------------------------------------- 台架几何


def test_the_45_degree_rod_identifies_ixx_directly():
    """这是"XY 耦合、惯量当一致"这个前提在数学上的出口。

    n_z = 0 且 Jxx = Jyy 时，nᵀJn = Jxx·cos²ψ + Jyy·sin²ψ = Jxx——
    **与 ψ 无关**。所以绕 45° 杆辨出来的惯量直接就是 Ixx/Iyy，不必分两个轴做。
    """
    for azimuth in (0.0, AZIMUTH_45_RAD, 1.0, math.pi / 3):
        rig = Rig(azimuth_rad=azimuth)
        assert rig.effective_inertia((0.019, 0.019, 0.00035)) == pytest.approx(0.019)


def test_asymmetric_inertia_makes_the_azimuth_matter():
    """反过来也要成立：Jxx≠Jyy 时方位角就不能随便填了。

    没有这条，上一条测试只是在测一个恒等式，不是在测代码。
    """
    rig_x = Rig(azimuth_rad=0.0)
    rig_y = Rig(azimuth_rad=math.pi / 2)
    assert rig_x.effective_inertia((0.02, 0.05, 0.001)) == pytest.approx(0.02)
    assert rig_y.effective_inertia((0.02, 0.05, 0.001)) == pytest.approx(0.05)


def test_rotation_about_the_rod_leaves_no_residual():
    rig = Rig()
    component = 2.0 * math.sqrt(0.5)
    assert rig.rate_residual((component, component, 0.0)) == pytest.approx(0.0, abs=1e-6)


def test_rotation_across_the_rod_is_all_residual():
    """垂直分量全被光杆的约束力吃掉：不产生运动，只产生轴承载荷。"""
    rig = Rig()
    component = 2.0 * math.sqrt(0.5)
    assert rig.rate_residual((component, -component, 0.0)) == pytest.approx(2.0, abs=1e-5)
    assert rig.rate_residual((0.0, 0.0, 1.5)) == pytest.approx(1.5, abs=1e-6)


def test_a_rod_through_the_cg_has_no_restoring_moment():
    """d = 0 是名义值，但"准不准过质心"由数据说话，不在这里假定。"""
    assert Rig(axis_offset_above_cg_m=0.0).gravity_moment(0.75, 9.81, 0.2) == 0.0
    off = Rig(axis_offset_above_cg_m=0.01).gravity_moment(0.75, 9.81, 0.2)
    assert off < 0.0, "偏心在上方时重力力矩应当把机体往回拉"


def test_the_imu_offset_only_pollutes_the_accelerometer():
    """角速度是刚体不变量——这正是"内环辨识以陀螺为准"的理由。

    加速度计会多读到 ω̇×r + ω×(ω×r)；杆过质心时质心线加速度为零，
    所以这两项就是姿态角估计误差的**全部**来源。
    """
    rig = Rig(imu_above_cg_m=0.02)
    error = imu_lever_arm_error(rig, omega=(1.0, 1.0, 0.0), alpha=(5.0, 5.0, 0.0))
    assert any(abs(value) > 1e-3 for value in error)

    centred = Rig(imu_above_cg_m=0.0)
    assert imu_lever_arm_error(centred, (1.0, 1.0, 0.0), (5.0, 5.0, 0.0)) == (0.0, 0.0, 0.0)


# ---------------------------------------------------------------- 档案


def test_a_profile_round_trips_through_json(tmp_path):
    profile = IdentProfile(
        name="光杆架 带电池",
        rig=Rig(axis_offset_above_cg_m=0.002, imu_above_cg_m=0.015),
        excitation=Excitation(profile=PROFILE_CHIRP, chirp_f0_hz=0.4,
                              chirp_f1_hz=7.0, duration_ms=6000, ramp_ms=10),
        airframe={"ixx_kgm2": 0.019, "mass_kg": 0.7546},
        assumed_inertia_kg_m2=0.019,
        fit=FitResult(inertia_kg_m2=0.0212, delay_s=0.028, fit_percent=91.2),
        gains=Gains(rate_kp=0.52, rate_ki=0.31, att_kp=6.0),
        firmware_id="abc1234")
    path = profile.save(tmp_path)
    again = IdentProfile.load(path)

    assert again.name == profile.name
    assert again.rig.azimuth_deg == pytest.approx(45.0)
    assert again.rig.imu_above_cg_m == pytest.approx(0.015)
    assert again.excitation.chirp_f1_hz == pytest.approx(7.0)
    assert again.airframe["ixx_kgm2"] == pytest.approx(0.019)
    assert again.fit.delay_s == pytest.approx(0.028)
    assert again.gains.rate_kp == pytest.approx(0.52)
    assert again.created_utc and again.updated_utc


def test_a_newer_profile_schema_is_refused_rather_than_misread():
    """按旧结构硬解会得到一组看着正常的错值，那比读不出来危险。"""
    with pytest.raises(ValueError):
        IdentProfile.from_dict({"name": "x", "schema_version": 99})


def test_the_profile_carries_the_airframe_snapshot_not_just_the_result():
    """半年后要能回答"这组 PID 是在哪个机体上辨的"。"""
    profile = IdentProfile(name="t", airframe={"ixx_kgm2": 0.02, "mass_kg": 0.7})
    assert profile.to_dict()["airframe"]["mass_kg"] == pytest.approx(0.7)


def test_converged_inertia_feeds_the_next_run():
    """模型反演的收敛：把辨出来的惯量填回假定值，再跑一轮。"""
    profile = IdentProfile(name="t", assumed_inertia_kg_m2=0.019,
                           fit=FitResult(inertia_kg_m2=0.0231))
    assert profile.with_converged_inertia().assumed_inertia_kg_m2 == pytest.approx(0.0231)


def test_gain_commands_write_both_axes_and_never_save():
    """45° 杆辨的就是 XY 共用的那个惯量；分开写两个不同值等于假装辨了两次。

    而且**绝不出现 SAVE**，也不用 `PARAM SET`（它成功后固件 1.5 s 自动存 Flash）——
    候选增益只经 `SYSID PARAM` 写 RAM，落盘是作者事后的单独动作。
    """
    commands = Gains(rate_kp=0.5, rate_ki=0.2, att_kp=6.0).param_commands()
    assert "SYSID PARAM coax.rate_roll_kp 0.5" in commands
    assert "SYSID PARAM coax.rate_pitch_kp 0.5" in commands
    assert "SYSID PARAM coax.att_roll_kp 6" in commands
    assert "SYSID PARAM coax.att_pitch_kp 6" in commands
    assert all(c.startswith("SYSID PARAM ") for c in commands)
    assert not any("SAVE" in c or "PARAM SET" in c for c in commands)
    assert not any("rate_roll_kd" in c for c in commands), "没给的项不要写"


def test_gain_commands_only_write_the_identified_axis():
    commands = Gains(rate_kp=0.5, rate_ki=0.2, att_kp=6.0).param_commands(axes=("pitch",))
    assert commands and all("_pitch_" in c for c in commands)
    with pytest.raises(ValueError):
        Gains(rate_kp=0.5).param_commands(axes=("yaw",))


def test_gain_names_exist_in_the_real_controller():
    """写出去的参数名必须是飞控真认的，否则「设置成功」其实什么也没设。"""
    source = (ROOT / "Driver/Src/drv_coax_ctrl.c").read_text(encoding="utf-8")
    for command in Gains(rate_kp=1.0, rate_ki=1.0, rate_kd=1.0,
                         att_kp=1.0).param_commands():
        name = command.split()[2].removeprefix("coax.")
        assert f'"{name}"' in source, name


# ---------------------------------------------------------------- 推力源


def test_the_pwm_table_is_monotonic_and_bounded():
    source = thrust_model.PwmTable(max_total_force_n=15.6)
    assert source.thrust_n(1000) == pytest.approx(0.0, abs=1e-6)
    assert source.thrust_n(2000) == pytest.approx(15.6, rel=1e-6)
    previous = -1.0
    for pulse in range(1000, 2001, 50):
        value = source.thrust_n(pulse)
        assert value >= previous
        previous = value
    # 表外的脉宽必须夹住，不能外推出一个更大的推力。
    assert source.thrust_n(2500) == pytest.approx(15.6, rel=1e-6)
    assert source.thrust_n(500) == pytest.approx(0.0, abs=1e-6)


def test_every_thrust_source_declares_how_trustworthy_it_is():
    """老推力台把 `kv*V*pct/100` 的估算值和实测混在一起报，看报告的人无从分辨。

    推力的相对误差会**原样**变成惯量的相对误差（τ 里 F 和 I 是乘除关系）。
    """
    approx = thrust_model.PwmTable(max_total_force_n=15.6)
    assert approx.quality == "square-law-approximation"
    measured = thrust_model.PwmTable(table_n=tuple(range(21)))
    assert measured.quality == "firmware-table"
    rpm = thrust_model.RpmThrust(points=((0.0, 0.0), (10000.0, 15.0)))
    assert rpm.quality == "measured-rpm"
    scaled = thrust_model.VoltageScaled(base=measured, reference_voltage_v=11.1)
    assert "approximation" in scaled.quality


def test_voltage_scaling_follows_the_square_law():
    base = thrust_model.PwmTable(table_n=tuple([10.0] * 21))
    scaled = thrust_model.VoltageScaled(base=base, reference_voltage_v=12.0)
    assert scaled.thrust_n(1500, voltage_v=12.0) == pytest.approx(10.0)
    assert scaled.thrust_n(1500, voltage_v=6.0) == pytest.approx(2.5)


def test_rpm_thrust_refuses_to_guess_without_an_rpm():
    source = thrust_model.RpmThrust(points=((0.0, 0.0), (10000.0, 15.0)))
    with pytest.raises(ValueError):
        source.thrust_n(1500)


def test_rpm_thrust_interpolates_between_voltage_layers():
    source = thrust_model.RpmThrust(layers={
        11.1: ((0.0, 0.0), (10000.0, 12.0)),
        12.6: ((0.0, 0.0), (10000.0, 15.0)),
    })
    assert source.thrust_n(0, voltage_v=11.1, erpm=10000) == pytest.approx(12.0)
    assert source.thrust_n(0, voltage_v=12.6, erpm=10000) == pytest.approx(15.0)
    middle = source.thrust_n(0, voltage_v=11.85, erpm=10000)
    assert 12.0 < middle < 15.0
    # 标定范围之外夹住，不外推。
    assert source.thrust_n(0, voltage_v=9.0, erpm=10000) == pytest.approx(12.0)


# ---------------------------------------------------------------- 机体链接


def test_airframe_values_come_from_the_single_shared_field_table():
    """`drone_tcp_panel.py` 里曾经有第二副 airframe 词汇表，和真表对不上。"""
    lines = [
        "airframe.ixx_kgm2=0.019",
        "airframe.iyy_kgm2 0.019",
        "airframe.not_a_real_field=1.0",
        "coax.rate_roll_kp=0.5",
        "noise",
    ]
    values = airframe_link.parse_param_lines(lines)
    assert values["ixx_kgm2"] == pytest.approx(0.019)
    assert values["iyy_kgm2"] == pytest.approx(0.019)
    assert "not_a_real_field" not in values
    assert "rate_roll_kp" not in values


def test_an_incomplete_airframe_is_refused_with_the_missing_name():
    with pytest.raises(airframe_link.AirframeIncomplete):
        airframe_link.require_complete({"ixx_kgm2": 0.019})


def test_imu_offset_is_derived_from_the_same_公式_as_the_panel():
    values = {"imu_z_m": 0.01, "board_mass_g": 75.0, "battery_mass_g": 232.0,
              "base_mass_g": 99.0, "servo_motor_mass_g": 348.6,
              "battery_cg_z_m": 0.109, "base_cg_z_m": -0.117,
              "servo_motor_cg_z_m": -0.244}
    offset = airframe_link.imu_above_cg_m(values)
    complete = airframe_link.complete(values)
    assert offset == pytest.approx(0.01 - complete["cg_z_m"])


# ---------------------------------------------------------------- 拟合


def synthesise_response(inertia, damping, stiffness, delay_s, torque, dt):
    """已知参数下的真值响应，用显式积分。拟合器要能把这几个数找回来。"""
    n = len(torque)
    rate = np.zeros(n)
    angle = np.zeros(n)
    lag = int(round(delay_s / dt))
    for k in range(1, n):
        applied = torque[k - lag] if k - lag >= 0 else 0.0
        accel = (applied - damping * rate[k - 1] - stiffness * angle[k - 1]) / inertia
        rate[k] = rate[k - 1] + accel * dt
        angle[k] = angle[k - 1] + rate[k] * dt
    return rate


def test_the_fit_recovers_a_known_inertia_damping_and_delay():
    fit_module = pytest.importorskip("sysid.fit", reason="需要 scipy")
    dt = 0.002
    t = np.arange(0.0, 6.0, dt)
    rng = np.random.default_rng(7)
    torque = 0.05 * np.sign(np.sin(2 * math.pi * 0.8 * t)) + 0.002 * rng.standard_normal(t.size)
    truth = dict(inertia=0.019, damping=0.004, stiffness=0.0, delay_s=0.030)
    rate = synthesise_response(truth["inertia"], truth["damping"],
                               truth["stiffness"], truth["delay_s"], torque, dt)

    result = fit_module.fit_model(t, torque, rate, mass_kg=0.7546,
                                  inertia_guess=0.010)
    assert result.inertia_kg_m2 == pytest.approx(truth["inertia"], rel=0.15)
    assert result.delay_s == pytest.approx(truth["delay_s"], abs=0.008)
    assert result.fit_percent > 80.0


def test_the_delay_estimator_finds_a_known_shift():
    fit_module = pytest.importorskip("sysid.fit")
    dt = 0.002
    t = np.arange(0.0, 4.0, dt)
    command = np.sin(2 * math.pi * 1.3 * t)
    shift = 25  # 50 ms
    response = np.concatenate([np.zeros(shift), command[:-shift]])
    estimate = fit_module.estimate_delay(t, command, response)
    assert estimate.seconds == pytest.approx(shift * dt, abs=dt * 1.5)
    assert estimate.trustworthy
    assert estimate.resolution_s == pytest.approx(dt)


def test_an_uncorrelated_pair_is_reported_as_untrustworthy():
    """互相关峰值太低时那个延迟数字没有意义，必须自己说出来。"""
    fit_module = pytest.importorskip("sysid.fit")
    dt = 0.002
    t = np.arange(0.0, 4.0, dt)
    rng = np.random.default_rng(3)
    estimate = fit_module.estimate_delay(t, rng.standard_normal(t.size),
                                         rng.standard_normal(t.size))
    assert not estimate.trustworthy


def test_the_inertia_ratio_is_the_convergence_signal():
    """I_true/I_est = α_期望/α_实测。这是"多种惯量复用"的机制本身。"""
    fit_module = pytest.importorskip("sysid.fit")
    expected = np.array([1.0, -2.0, 3.0, -1.5, 0.5] * 20)
    measured = expected / 2.0  # 真惯量是假定的两倍 -> 角加速度只有一半
    assert fit_module.inertia_ratio(expected, measured) == pytest.approx(2.0, rel=1e-6)


def test_strongly_correlated_damping_and_eccentricity_are_flagged():
    """短激励下两者都只表现为"回中"。硬给一个数是在编。"""
    fit_module = pytest.importorskip("sysid.fit")
    dt = 0.002
    t = np.arange(0.0, 0.6, dt)          # 太短，分不开
    torque = 0.05 * np.sign(np.sin(2 * math.pi * 1.0 * t))
    rate = synthesise_response(0.019, 0.004, 0.05, 0.02, torque, dt)
    result = fit_module.fit_model(t, torque, rate, mass_kg=0.7546,
                                  inertia_guess=0.019)
    if not result.separable:
        assert any("不可分离" in w for w in result.warnings)
    else:
        # 可分离时也必须报出相关系数，不能只给一个数。
        assert result.damping_eccentricity_correlation is not None


# ---------------------------------------------------------------- 整定


def test_bandwidth_falls_as_the_identified_delay_grows():
    """带宽由**延迟**决定，不由惯量决定。

    老那套 `kp = 0.35/|K|` 里根本没有延迟这个量，所以它给的增益和执行器快慢无关。
    """
    fast = tune.rate_bandwidth_hz(0.010)
    slow = tune.rate_bandwidth_hz(0.040)
    assert fast > slow
    assert slow == pytest.approx(fast / 4.0, rel=1e-6)


def test_tuning_refuses_to_proceed_without_a_delay():
    with pytest.raises(ValueError):
        tune.rate_bandwidth_hz(0.0)
    with pytest.raises(ValueError):
        tune.rate_bandwidth_hz(float("nan"))


def test_a_near_zero_delay_cannot_push_the_bandwidth_past_the_cap():
    """延迟辨成 ≈0 时 π/(4·T_d) 发散，曾经给出 kp ≈ 7e10。带宽必须封顶。"""
    assert tune.rate_bandwidth_hz(1e-9) == pytest.approx(tune.RATE_BANDWIDTH_MAX_HZ)
    gains = tune.synthesise_rate(0.04, 0.0, 1e-9)
    omega_max = 2.0 * math.pi * tune.RATE_BANDWIDTH_MAX_HZ
    assert gains.kp == pytest.approx(0.04 * omega_max)
    assert "封顶" in gains.rationale


# 评审给出的飞行 TWD 对象量级（固件力矩单位）。
TWD_PLANT = dict(inertia_cg_kg_m2=0.046, torque_model_scale=1.09, dead_time_s=0.038,
                 servo_wn_rad_s=35.0, servo_zeta=0.47, reaction_couple_s2=0.0054)


def test_loop_shaping_takes_the_largest_kp_that_keeps_both_margins():
    plant = tune.FlightPlant(**TWD_PLANT)
    gains = tune.synthesise_rate_shaped(plant)
    assert gains.feasible
    assert gains.gain_margin_db >= tune.GAIN_MARGIN_MIN_DB - 1e-6
    assert gains.phase_margin_deg >= tune.PHASE_MARGIN_MIN_DEG
    assert gains.ki == pytest.approx(gains.kp * 2 * math.pi * gains.crossover_hz / 5, rel=0.02)
    assert gains.bandwidth_hz == gains.crossover_hz
    # "最大"：再加 5% 的 kp 就守不住其中一条裕度。
    bigger = gains.kp * 1.05
    margins = tune.loop_margins(bigger, bigger * 2 * math.pi * gains.crossover_hz / 5, plant)
    assert (margins.gain_margin_db < tune.GAIN_MARGIN_MIN_DB
            or margins.phase_margin_deg < tune.PHASE_MARGIN_MIN_DEG)


def test_the_reaction_couple_is_what_limits_the_gain():
    """同一对象去掉反作用力偶（ρ = 0），允许的 kp 明显更大——10 Hz 的舵机反作用才是瓶颈。"""
    with_couple = tune.synthesise_rate_shaped(tune.FlightPlant(**TWD_PLANT))
    without = tune.synthesise_rate_shaped(tune.FlightPlant(**{**TWD_PLANT,
                                                              "reaction_couple_s2": 0.0}))
    assert without.kp > 1.3 * with_couple.kp


def test_loop_shaping_says_so_when_the_margins_cannot_be_met():
    gains = tune.synthesise_rate_shaped(tune.FlightPlant(**TWD_PLANT), phase_margin_min_deg=85.0)
    assert not gains.feasible
    assert "无法同时满足" in gains.rationale


def test_loop_shaping_respects_the_bandwidth_cap():
    fast = tune.FlightPlant(inertia_cg_kg_m2=0.04, torque_model_scale=1.0, dead_time_s=0.0005,
                            servo_wn_rad_s=2000.0, servo_zeta=0.7, reaction_couple_s2=0.0)
    gains = tune.synthesise_rate_shaped(fast)
    assert gains.crossover_hz <= tune.RATE_BANDWIDTH_MAX_HZ
    with pytest.raises(ValueError):
        tune.synthesise_rate_shaped(tune.FlightPlant(**{**TWD_PLANT, "inertia_cg_kg_m2": -0.01}))


# ---------------------------------------------------------------- 反馈通路里的转速陷波


def test_without_a_notch_the_plant_is_bitwise_the_old_formula():
    freq = np.logspace(-2, 2, 500)
    plant = tune.FlightPlant(**TWD_PLANT)
    s = 2j * np.pi * freq
    wn, zeta = TWD_PLANT["servo_wn_rad_s"], TWD_PLANT["servo_zeta"]
    servo = wn * wn / (s * s + 2.0 * zeta * wn * s + wn * wn)
    old = (np.exp(-s * TWD_PLANT["dead_time_s"]) * servo
           * (TWD_PLANT["torque_model_scale"] + TWD_PLANT["reaction_couple_s2"] * s * s)
           / (TWD_PLANT["inertia_cg_kg_m2"] * s))
    assert plant.feedback_notch is None
    assert plant.response(freq).tobytes() == old.tobytes()


def test_the_deployed_notch_costs_about_five_and_a_half_degrees_at_10_hz():
    notch = tune.DEPLOYED_RPM_NOTCH
    assert math.degrees(float(np.angle(notch.response(10.0)))) == pytest.approx(-5.47, abs=0.05)
    assert abs(notch.response(0.0)) == pytest.approx(1.0, abs=1e-12)
    assert abs(notch.response(70.0)) < 1e-6
    with pytest.raises(ValueError):
        tune.FeedbackNotch(centers_hz=(600.0,), q=3.0, fs_hz=1000.0)
    with pytest.raises(ValueError):
        tune.FeedbackNotch(centers_hz=(70.0,), q=0.0, fs_hz=1000.0)


def test_the_tuner_backs_off_for_the_notch_and_keeps_the_margins():
    plant = tune.FlightPlant(**TWD_PLANT, feedback_notch=tune.DEPLOYED_RPM_NOTCH)
    gains = tune.synthesise_rate_shaped(plant)
    assert gains.feasible and gains.gain_margin_db >= tune.GAIN_MARGIN_MIN_DB - 1e-6
    assert gains.kp == pytest.approx(0.1829, rel=1e-3)
    assert gains.ki == pytest.approx(0.1468, rel=1e-3)
    assert "转速陷波" in gains.rationale and "70/70 Hz" in gains.rationale
    without = tune.synthesise_rate_shaped(tune.FlightPlant(**TWD_PLANT))
    assert gains.kp < without.kp and "转速陷波" not in without.rationale
    # 现行增益（kp 0.1388 / ki 0.0460）在带陷波的对象上的增益裕度：8.66 → 8.42 dB。
    assert tune.loop_margins(0.1388, 0.0460, plant).gain_margin_db == pytest.approx(8.42, abs=0.02)


def test_the_deployed_notch_mirrors_the_firmware_defaults():
    """最坏位置 = 每个谐波 h 的 (min_hz + fade_hz)·h，两个电机各一个；Q 同固件默认。"""
    import re
    header = (ROOT / "App/Inc/app_rpm_notch.h").read_text(encoding="utf-8")

    def macro(name):
        return re.search(rf"#define {name}\s+([0-9.x]+)[fU]?", header).group(1)

    mask = int(macro("APP_RPM_NOTCH_DEFAULT_HARMONICS").rstrip("U"), 0)
    worst = float(macro("APP_RPM_NOTCH_DEFAULT_MIN_HZ")) + float(macro("APP_RPM_NOTCH_DEFAULT_FADE_HZ"))
    centres = tuple(worst * h for h in (1, 2, 3) if mask & (1 << (h - 1)) for _motor in (0, 1))
    notch = tune.DEPLOYED_RPM_NOTCH
    assert tuple(sorted(notch.centers_hz)) == tuple(sorted(centres))
    assert notch.q == float(macro("APP_RPM_NOTCH_DEFAULT_Q"))
    assert notch.fs_hz == 1000.0          # BMI088 名义 ODR（BSP_IMU_GetGyroOdrHz）


def test_rate_gains_scale_with_inertia_and_cancel_known_damping():
    light = tune.synthesise_rate(0.010, 0.0, 0.030)
    heavy = tune.synthesise_rate(0.040, 0.0, 0.030)
    assert heavy.kp == pytest.approx(light.kp * 4.0, rel=1e-6)
    damped = tune.synthesise_rate(0.040, 0.05, 0.030)
    assert damped.kp < heavy.kp, "已有阻尼应当被比例项抵掉一部分"
    assert light.rationale and "45" in light.rationale


def test_a_very_damped_plant_still_gets_a_positive_gain():
    """kp = I·ωc − c 可能算成负数。负比例增益是正反馈，绝不能发出去。"""
    gains = tune.synthesise_rate(0.001, 10.0, 0.030)
    assert gains.kp > 0.0


def test_the_attitude_loop_stays_slower_than_the_rate_loop():
    """时标分离是串级能当串级用的前提。"""
    rate = tune.synthesise_rate(0.019, 0.004, 0.030)
    attitude = tune.synthesise_attitude(rate)
    assert attitude.bandwidth_hz < rate.bandwidth_hz / 2.0


def test_step_scoring_separates_overshoot_from_instability():
    """调参的人需要知道是超调大还是稳不下来，一个复合分数回答不了。"""
    t = [i * 0.01 for i in range(200)]
    settled = [0.15 * (1 - math.exp(-5 * x)) for x in t]
    good = tune.score_step_response(t, settled, 0.15)
    assert good.stable and good.overshoot_percent < 1.0

    diverging = [0.15 * math.exp(0.9 * x) for x in t]
    bad = tune.score_step_response(t, diverging, 0.15)
    assert not bad.stable
    assert "发散" in bad.note
    assert not bad.acceptable


# ---------------------------------------------------------------- 报告


def make_profile(**overrides) -> IdentProfile:
    base = dict(
        name="光杆架-默认",
        assumed_inertia_kg_m2=0.019,
        fit=FitResult(inertia_kg_m2=0.0195, damping_n_m_s=0.004,
                      eccentricity_m=0.0002, delay_s=0.031, fit_percent=93.0,
                      damping_eccentricity_correlation=0.42),
        gains=Gains(rate_kp=0.52, rate_ki=0.30, att_kp=6.1))
    base.update(overrides)
    return IdentProfile(**base)


def test_the_report_prints_where_the_thrust_number_came_from(tmp_path):
    path = report.write_report(make_profile(), tmp_path / "r.md",
                               thrust_quality="square-law-approximation",
                               samples=1200)
    text = path.read_text(encoding="utf-8")
    assert "square-law-approximation" in text
    assert "原样" in text, "必须说明推力误差会原样变成惯量误差"


def test_the_report_says_so_when_damping_and_eccentricity_cannot_be_separated(tmp_path):
    profile = make_profile(fit=FitResult(
        inertia_kg_m2=0.02, damping_n_m_s=0.004, eccentricity_m=0.01,
        delay_s=0.03, fit_percent=88.0,
        damping_eccentricity_correlation=0.991))
    text = report.write_report(profile, tmp_path / "r.md").read_text(encoding="utf-8")
    assert "不可分离" in text
    assert "0.991" in text


def test_the_report_never_tells_anyone_to_save_to_flash(tmp_path):
    text = report.write_report(make_profile(), tmp_path / "r.md").read_text(encoding="utf-8")
    assert "SYSID PARAM coax.rate_roll_kp" in text
    assert "未落 Flash" in text
    assert "SAVE" not in text.replace("不会自动 `SAVE`", "")


def test_the_report_shows_convergence_progress(tmp_path):
    text = report.write_report(make_profile(), tmp_path / "r.md").read_text(encoding="utf-8")
    assert "比值" in text
    assert "收敛进度" in text


def test_the_report_refuses_to_invent_numbers_it_does_not_have(tmp_path):
    profile = make_profile(fit=FitResult())
    text = report.write_report(profile, tmp_path / "r.md").read_text(encoding="utf-8")
    assert "未辨出" in text
    assert "0 kg·m²" not in text, "缺项不许填占位数"


def test_dropped_samples_are_visible_in_the_report(tmp_path):
    text = report.write_report(make_profile(), tmp_path / "r.md",
                               dropped_samples=17, gap_count=2,
                               samples=800).read_text(encoding="utf-8")
    assert "丢样 17 条" in text
    assert "断点 2 处" in text


def test_the_report_forbids_reusing_the_old_servo_delay_numbers(tmp_path):
    """`drv_servo_actuator_model.h` 那组是停电机、±200 µs、总线舵机条件下测的。"""
    text = report.write_report(make_profile(), tmp_path / "r.md").read_text(encoding="utf-8")
    assert "drv_servo_actuator_model.h" in text


def test_samples_csv_marks_where_the_timeline_breaks(tmp_path):
    sys.path.insert(0, str(ROOT / "tools"))
    try:
        from sysid.decode import FLAG_GAP, SysIdBatch
    finally:
        sys.path.pop(0)

    batches = [
        SysIdBatch(run_id=1, base_t_us=0, dt_us=2000, flags=0,
                   samples=({"gx": 0.1}, {"gx": 0.2})),
        SysIdBatch(run_id=1, base_t_us=900_000, dt_us=2000, flags=FLAG_GAP,
                   samples=({"gx": 0.3}, {"gx": 0.4})),
    ]
    path = report.write_samples_csv(batches, tmp_path / "s.csv")
    rows = path.read_text(encoding="utf-8").strip().splitlines()
    assert rows[0].startswith("t_us,run_id,gap,")
    assert rows[3].split(",")[2] == "1", "断点后的第一行必须打标"
    assert rows[4].split(",")[2] == "0"
